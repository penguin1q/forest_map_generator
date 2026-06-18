#!/usr/bin/env python3
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import bpy
import bmesh
from mathutils import Vector

LEAF_TOKENS = ("leaf", "leaves", "foliage", "crown")
WOOD_TOKENS = ("trunk", "branch", "stem", "wood", "bark", "limb", "tree")


def blender_script_argv():
    if "--" in sys.argv:
        return sys.argv[sys.argv.index("--") + 1:]
    return []


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze a Blender tree asset for Gazebo LOD generation."
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Directory where tree_blend_analysis.json is written. "
            "Default: directory of the opened .blend file, or current directory if unsaved."
        ),
    )
    parser.add_argument(
        "--output-name",
        default="tree_blend_analysis.json",
        help="Output JSON filename. Default: tree_blend_analysis.json",
    )
    parser.add_argument(
        "--leaf-tokens",
        nargs="*",
        default=list(LEAF_TOKENS),
        help="Object-name tokens used to classify leaf objects.",
    )
    parser.add_argument(
        "--wood-tokens",
        nargs="*",
        default=list(WOOD_TOKENS),
        help="Object-name tokens used to classify wood / trunk / branch objects.",
    )
    return parser.parse_args(blender_script_argv())


def classify_object(obj, leaf_tokens, wood_tokens):
    name = obj.name.lower()
    if any(token.lower() in name for token in leaf_tokens):
        return "leaf"
    if any(token.lower() in name for token in wood_tokens):
        return "wood"
    return "unknown"


def mesh_island_face_counts(obj):
    if obj.type != "MESH":
        return []

    mesh = obj.data
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.faces.ensure_lookup_table()

    visited = set()
    islands = []

    edge_to_faces = defaultdict(list)
    for face in bm.faces:
        for edge in face.edges:
            edge_to_faces[edge].append(face)

    for face in bm.faces:
        if face in visited:
            continue

        stack = [face]
        visited.add(face)
        face_count = 0

        while stack:
            current = stack.pop()
            face_count += 1
            for edge in current.edges:
                for next_face in edge_to_faces[edge]:
                    if next_face not in visited:
                        visited.add(next_face)
                        stack.append(next_face)

        islands.append(face_count)

    bm.free()
    return sorted(islands, reverse=True)


def object_bounds_world(obj):
    points = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    mn = Vector((min(p.x for p in points), min(p.y for p in points), min(p.z for p in points)))
    mx = Vector((max(p.x for p in points), max(p.y for p in points), max(p.z for p in points)))
    return mn, mx


def resolve_output_path(args):
    if args.output_dir:
        output_dir = Path(bpy.path.abspath(args.output_dir)).expanduser()
    elif bpy.data.filepath:
        output_dir = Path(bpy.data.filepath).resolve().parent
    else:
        output_dir = Path.cwd()

    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / args.output_name


def main():
    args = parse_args()

    report = {
        "blend_file": bpy.data.filepath,
        "objects": [],
        "summary": {
            "leaf_objects": 0,
            "wood_objects": 0,
            "unknown_objects": 0,
            "total_mesh_vertices": 0,
            "total_mesh_faces": 0,
            "total_leaf_vertices": 0,
            "total_leaf_faces": 0,
            "total_leaf_islands": 0,
        },
    }

    for obj in bpy.context.scene.objects:
        if obj.type not in {"MESH", "CURVE", "SURFACE"}:
            continue

        kind = classify_object(obj, args.leaf_tokens, args.wood_tokens)
        item = {
            "name": obj.name,
            "type": obj.type,
            "kind": kind,
            "modifiers": [modifier.type for modifier in obj.modifiers],
        }

        if hasattr(obj.data, "materials"):
            item["materials"] = [material.name if material else None for material in obj.data.materials]
        else:
            item["materials"] = []

        if obj.type == "MESH":
            vertices = len(obj.data.vertices)
            faces = len(obj.data.polygons)
            item["vertices"] = vertices
            item["faces"] = faces

            mn, mx = object_bounds_world(obj)
            item["bounds_min"] = [mn.x, mn.y, mn.z]
            item["bounds_max"] = [mx.x, mx.y, mx.z]
            item["bounds_size"] = [mx.x - mn.x, mx.y - mn.y, mx.z - mn.z]

            islands = mesh_island_face_counts(obj)
            item["mesh_island_count"] = len(islands)
            item["mesh_island_face_counts_top20"] = islands[:20]
            item["mesh_island_face_counts_min"] = min(islands) if islands else 0
            item["mesh_island_face_counts_max"] = max(islands) if islands else 0

            report["summary"]["total_mesh_vertices"] += vertices
            report["summary"]["total_mesh_faces"] += faces

            if kind == "leaf":
                report["summary"]["total_leaf_vertices"] += vertices
                report["summary"]["total_leaf_faces"] += faces
                report["summary"]["total_leaf_islands"] += len(islands)

        if kind == "leaf":
            report["summary"]["leaf_objects"] += 1
        elif kind == "wood":
            report["summary"]["wood_objects"] += 1
        else:
            report["summary"]["unknown_objects"] += 1

        report["objects"].append(item)

    output_path = resolve_output_path(args)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print("Wrote:", output_path)
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    print("\nTop mesh-like objects:")
    for item in sorted(report["objects"], key=lambda x: x.get("faces", 0), reverse=True)[:20]:
        print(
            f"{item['kind']:7s} {item['type']:7s} "
            f"{item['name'][:45]:45s} "
            f"v={item.get('vertices', '-')}, "
            f"f={item.get('faces', '-')}, "
            f"islands={item.get('mesh_island_count', '-')}"
        )


if __name__ == "__main__":
    main()
