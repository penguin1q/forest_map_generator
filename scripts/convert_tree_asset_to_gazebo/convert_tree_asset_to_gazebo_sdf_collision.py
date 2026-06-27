#!/usr/bin/env python3
import argparse
import json
import random
import shutil
import sys
from pathlib import Path

MODEL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="tree_link">
      <visual name="tree_visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/tree_mesh.obj</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
      </visual>

{collision_xml}
    </link>
  </model>
</sdf>
"""


MODEL_SDF_SPLIT_VISUAL_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="tree_link">
      <visual name="leaf_visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{leaf_mesh}</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
        <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
{leaf_segmentation_label_xml}
        </plugin>
      </visual>

      <visual name="wood_visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{wood_mesh}</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
        <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
{wood_segmentation_label_xml}
        </plugin>
      </visual>

{collision_xml}
    </link>
  </model>
</sdf>
"""


MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>forest_map_generator</name>
    <email>unknown@example.com</email>
  </author>
  <description>
    Converted low-poly tree asset for orchard simulation.
  </description>
</model>
"""


SUPPORTED_CONVERTIBLE_TYPES = {"MESH", "CURVE", "SURFACE"}
IGNORED_TYPES = {"CAMERA", "LIGHT"}


# Project-internal canonical semantic label IDs.
# These IDs are used for Gazebo SegmentationCamera labels and dataset export.
# Do not couple them directly to YOLO / COCO model IDs; use a separate mapping
# when importing detector outputs.
CANONICAL_SEMANTIC_LABELS = {
    "background": 0,
    # vegetation / tree
    "leaf": 1,
    "wood": 2,
    "fruit": 3,
    # terrain / static objects
    "ground": 20,
    "rock": 21,
    "pole": 22,
    "fence": 23,
    # dynamic / safety critical
    "person": 50,
    "vehicle": 51,
    "animal": 52,
    # robot / ignore
    "robot": 90,
    "ignore": 255,
}

# Tree collision is physical geometry, not camera-visible semantic geometry.
# Keep this class in semantic_parts.json, but do not add it to SDF <visual> labels.
CAMERA_SEGMENTATION_CLASS_LABELS = {
    "leaf": CANONICAL_SEMANTIC_LABELS["leaf"],
    "wood": CANONICAL_SEMANTIC_LABELS["wood"],
}


BRANCH_INCLUDE_TOKENS = (
    "branch",
    "trunk",
    "stem",
    "limb",
    "bough",
    "wood",
    "bark",
    "sapling",
    "tree",
)

BRANCH_EXCLUDE_TOKENS = (
    "leaf",
    "leaves",
    "foliage",
    "fruit",
    "blossom",
    "flower",
    "crown",
)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Convert a Blender tree asset into a Gazebo model directory with optional convex-hull / island-surface / inner-metaball leaf LOD."
    )

    parser.add_argument(
        "--input", required=True, help="Input model file，preferably .blend"
    )
    parser.add_argument("--model-name", required=True, help="Gazebo model name")
    parser.add_argument(
        "--output-dir", required=True, help="Output Gazebo model directory"
    )
    parser.add_argument("--visual-format", choices=("obj",), default="obj")

    parser.add_argument(
        "--collision-mode",
        choices=(
            "trunk_cylinder",
            "thick_branch_cylinders",
            "bounding_box",
            "bounding_cylinder",
            "visual_decimated_optional",
        ),
        default="trunk_cylinder",
    )

    parser.add_argument(
        "--collision-output-mode",
        choices=("mesh_stl", "sdf_primitive"),
        default="mesh_stl",
        help=(
            "How to write generated collision geometry. "
            "mesh_stl preserves the legacy behavior and writes meshes/tree_collision.stl. "
            "sdf_primitive writes SDF box/cylinder collision directly when supported."
        ),
    )

    parser.add_argument(
        "--origin-mode",
        choices=("bottom_center", "keep", "cursor", "named_empty"),
        default="bottom_center",
    )
    parser.add_argument("--origin-empty-name", default="TREE_ORIGIN")

    parser.add_argument("--visual-collection", default="visual")
    parser.add_argument("--collision-collection", default="collision")

    parser.add_argument(
        "--split-visuals",
        action="store_true",
        help="Export separate visual meshes for leaves and wood.",
    )
    parser.add_argument("--leaf-collection", default="leaf")
    parser.add_argument("--wood-collection", default="wood")
    parser.add_argument("--leaf-output-name", default="tree_leaf.obj")
    parser.add_argument("--wood-output-name", default="tree_wood.obj")

    parser.add_argument(
        "--split-unknown-as",
        choices=("wood", "leaf", "ignore"),
        default="wood",
    )

    parser.add_argument(
        "--write-semantic-parts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--write-segmentation-labels",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Write Gazebo SegmentationCamera <label> elements to split visual SDF. "
            "Only visual geometry is labeled; collision geometry is never labeled."
        ),
    )
    parser.add_argument(
        "--leaf-segmentation-label",
        type=int,
        default=CANONICAL_SEMANTIC_LABELS["leaf"],
        help="Gazebo SegmentationCamera label ID for leaf_visual. Default: 1.",
    )
    parser.add_argument(
        "--wood-segmentation-label",
        type=int,
        default=CANONICAL_SEMANTIC_LABELS["wood"],
        help="Gazebo SegmentationCamera label ID for wood_visual. Default: 2.",
    )

    parser.add_argument(
        "--leaf-lod-mode",
        choices=("original", "joined", "convex_hull", "island_surface", "none"),
        default="original",
        help="How to export leaf visuals. convex_hull replaces leaf meshes with clustered low-poly hulls; island_surface keeps only outer mesh islands for middle LOD.",
    )
    parser.add_argument("--leaf-cluster-count", type=int, default=12)
    parser.add_argument("--leaf-kmeans-iterations", type=int, default=16)
    parser.add_argument(
        "--leaf-hull-jitter",
        type=float,
        default=0.05,
        help="Artificial thickness added around leaf vertices before convex hull generation [m].",
    )
    parser.add_argument(
        "--leaf-hull-min-points",
        type=int,
        default=8,
        help="Minimum points required to build a hull for one cluster.",
    )
    parser.add_argument(
        "--leaf-hull-decimate-ratio",
        type=float,
        default=0.75,
        help="Decimate ratio applied to each generated hull. 1.0 means no reduction.",
    )
    parser.add_argument(
        "--leaf-sample-vertices-per-object",
        type=int,
        default=32,
        help="Maximum vertices sampled from each leaf object for hull construction.",
    )

    parser.add_argument(
        "--leaf-surface-keep-ratio",
        type=float,
        default=0.55,
        help="Ratio of outer leaf mesh islands to keep when --leaf-lod-mode island_surface is used.",
    )
    parser.add_argument(
        "--leaf-surface-score-mode",
        choices=("ellipsoid", "distance"),
        default="ellipsoid",
        help="How to score leaf mesh islands as outer surface islands.",
    )
    parser.add_argument(
        "--leaf-surface-min-islands",
        type=int,
        default=1,
        help="Minimum number of leaf mesh islands to keep in island_surface mode.",
    )
    parser.add_argument(
        "--leaf-inner-keep-ratio",
        type=float,
        default=0.0,
        help=(
            "Additional ratio of non-surface inner leaf mesh islands to keep. "
            "This fills the crown interior while preserving original leaf shapes. Can be combined with --leaf-inner-proxy-mode metaball."
        ),
    )
    parser.add_argument(
        "--leaf-inner-keep-mode",
        choices=("random", "even"),
        default="random",
        help="How to select additional inner islands when --leaf-inner-keep-ratio is greater than zero.",
    )
    parser.add_argument(
        "--leaf-inner-random-seed",
        type=int,
        default=0,
        help="Random seed used for deterministic inner island selection.",
    )
    parser.add_argument(
        "--leaf-inner-proxy-mode",
        choices=("remove", "metaball"),
        default="remove",
        help=(
            "How to represent inner islands that are not kept as original leaves. "
            "remove deletes them; metaball replaces them with a soft internal crown proxy."
        ),
    )
    parser.add_argument(
        "--leaf-inner-proxy-score-scale",
        type=float,
        default=0.85,
        help=(
            "Only islands deeper than surface_cutoff_score * this value are used for the inner metaball proxy. "
            "Smaller values move the metaball region further inside the crown so it is less visible near the surface."
        ),
    )
    parser.add_argument(
        "--leaf-inner-metaball-position-scale",
        type=float,
        default=0.90,
        help=(
            "Scale inner metaball source points toward the leaf crown center before generation. "
            "Values below 1.0 shrink the proxy inward."
        ),
    )
    parser.add_argument(
        "--leaf-inner-metaball-radius",
        type=float,
        default=0.28,
        help="Radius of each metaball element for inner crown proxy generation [m].",
    )
    parser.add_argument(
        "--leaf-inner-metaball-resolution",
        type=float,
        default=0.22,
        help="Metaball viewport resolution for the inner crown proxy. Larger is coarser/lighter.",
    )
    parser.add_argument(
        "--leaf-inner-metaball-render-resolution",
        type=float,
        default=None,
        help="Metaball render resolution for the inner crown proxy. Defaults to --leaf-inner-metaball-resolution.",
    )
    parser.add_argument(
        "--leaf-inner-metaball-threshold",
        type=float,
        default=0.45,
        help="Metaball threshold for the inner crown proxy. Smaller values merge blobs more strongly.",
    )
    parser.add_argument(
        "--leaf-inner-metaball-point-stride",
        type=int,
        default=3,
        help="Use one point every N source points from deleted inner islands for metaball generation.",
    )
    parser.add_argument(
        "--leaf-inner-metaball-max-points",
        type=int,
        default=260,
        help="Maximum number of source points used for the inner metaball proxy.",
    )
    parser.add_argument(
        "--leaf-inner-metaball-decimate-ratio",
        type=float,
        default=0.35,
        help="Decimate ratio applied after converting the inner metaball proxy to mesh.",
    )
    parser.add_argument(
        "--leaf-inner-metaball-shade-smooth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Apply smooth shading to the generated inner metaball proxy.",
    )

    parser.add_argument(
        "--wood-lod-mode",
        choices=("original", "decimate", "none"),
        default="original",
        help="How to export wood visuals. decimate duplicates wood mesh objects and applies a Decimate modifier for visual LOD.",
    )
    parser.add_argument(
        "--wood-decimate-ratio",
        type=float,
        default=0.50,
        help="Decimate ratio for wood visuals when --wood-lod-mode decimate is used. 1.0 means no reduction.",
    )
    parser.add_argument(
        "--wood-join-after-decimate",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Join decimated wood objects into one mesh before OBJ export. Usually unnecessary when wood is already one object.",
    )
    parser.add_argument(
        "--wood-shade-smooth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Apply smooth shading to decimated wood visual meshes.",
    )

    parser.add_argument("--scale", type=float, default=1.0)

    parser.add_argument("--trunk-radius", type=float, default=0.18)
    parser.add_argument("--trunk-height", type=float, default=1.2)
    parser.add_argument("--trunk-center-x", type=float, default=0.0)
    parser.add_argument("--trunk-center-y", type=float, default=0.0)

    parser.add_argument("--collision-min-radius", type=float, default=0.035)
    parser.add_argument("--collision-cylinder-sides", type=int, default=10)
    parser.add_argument("--collision-max-branch-height", type=float, default=2.2)
    parser.add_argument(
        "--collision-include-main-branches",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--collision-add-trunk-cylinder",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--collision-file",
        help="Existing STL file to copy as meshes/tree_collision.stl",
    )

    parser.add_argument("--copy-textures", action="store_true")
    parser.add_argument("--dry-run", action="store_true")

    return parser.parse_args(argv)


def blender_argv():
    if "--" in sys.argv:
        return sys.argv[sys.argv.index("--") + 1 :]
    return sys.argv[1:]


def warn(message):
    print(f"WARNING: {message}")


def info(message):
    print(message)


def load_bpy():
    try:
        import bpy
        import mathutils
    except ImportError as exc:
        raise RuntimeError(
            "This converter must be run with Blender，e.g. "
            "blender --background --python convert_tree_asset_to_gazebo_convex_lod.py -- <args>"
        ) from exc
    return bpy, mathutils


def reset_scene(bpy):
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def load_input_file(bpy, input_path):
    suffix = input_path.suffix.lower()

    if suffix == ".blend":
        bpy.ops.wm.open_mainfile(filepath=str(input_path))
        return

    reset_scene(bpy)

    if suffix == ".obj":
        if hasattr(bpy.ops.wm, "obj_import"):
            bpy.ops.wm.obj_import(filepath=str(input_path))
            return
        if hasattr(bpy.ops, "import_scene") and hasattr(bpy.ops.import_scene, "obj"):
            bpy.ops.import_scene.obj(filepath=str(input_path))
            return
        raise RuntimeError(
            "OBJ import operator is not available in this Blender version."
        )

    if suffix == ".fbx":
        if hasattr(bpy.ops.import_scene, "fbx"):
            bpy.ops.import_scene.fbx(filepath=str(input_path))
            return
        raise RuntimeError(
            "FBX import operator is not available in this Blender version."
        )

    if suffix in {".glb", ".gltf"}:
        if hasattr(bpy.ops.import_scene, "gltf"):
            bpy.ops.import_scene.gltf(filepath=str(input_path))
            return
        raise RuntimeError(
            "glTF import operator is not available in this Blender version."
        )

    raise RuntimeError(f"Unsupported input file extension: {suffix}")


def collection_objects(bpy, collection_name):
    collection = bpy.data.collections.get(collection_name)
    if collection is None:
        return None
    return list(collection.all_objects)


def object_set(objects):
    return {obj.name for obj in objects}


def is_export_candidate(obj):
    if obj.type in IGNORED_TYPES:
        return False
    if obj.type not in SUPPORTED_CONVERTIBLE_TYPES:
        return False
    if getattr(obj, "hide_viewport", False):
        return False
    return True


def unique_objects(objects):
    seen = set()
    result = []
    for obj in objects:
        if obj.name not in seen:
            seen.add(obj.name)
            result.append(obj)
    return result


def report_preparation_warnings(objects):
    for obj in objects:
        for modifier in getattr(obj, "modifiers", []):
            if modifier.type == "NODES":
                warn(
                    f"Object {obj.name} has Geometry Nodes. Realize instances if exported mesh is incomplete."
                )
            else:
                warn(
                    f"Object {obj.name} has modifier {modifier.name} ({modifier.type}). "
                    "Check the exported mesh if needed."
                )

        if getattr(obj, "particle_systems", None) and len(obj.particle_systems) > 0:
            warn(
                f"Object {obj.name} has particle systems. Convert particles to mesh if needed."
            )

        if getattr(obj, "instance_type", "NONE") != "NONE":
            warn(
                f"Object {obj.name} uses instancing ({obj.instance_type}). "
                "Realize instances if needed."
            )


def select_any(bpy, objects):
    bpy.ops.object.select_all(action="DESELECT")
    selected = []
    for obj in objects:
        obj.select_set(True)
        selected.append(obj)
    bpy.context.view_layer.objects.active = selected[0] if selected else None
    return selected


def select_only_meshes(bpy, objects):
    bpy.ops.object.select_all(action="DESELECT")
    selected = []
    for obj in objects:
        if obj.type == "MESH":
            obj.select_set(True)
            selected.append(obj)
    bpy.context.view_layer.objects.active = selected[0] if selected else None
    return selected


def clear_parents_keep_transform(bpy, objects):
    selected = select_any(bpy, objects)
    if selected:
        bpy.ops.object.parent_clear(type="CLEAR_KEEP_TRANSFORM")


def convert_object_to_mesh(bpy, obj):
    if obj.type == "MESH":
        return obj

    if obj.type not in {"CURVE", "SURFACE"}:
        warn(
            f"Object {obj.name} is type {obj.type} and cannot be converted to mesh safely."
        )
        return None

    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    try:
        bpy.ops.object.convert(target="MESH")
    except Exception as exc:
        warn(f"Object {obj.name} could not be converted to mesh: {exc}")
        return None

    converted = bpy.context.object
    if converted.type != "MESH":
        warn(f"Object {obj.name} is not a mesh after conversion.")
        return None

    return converted


def convert_objects_to_meshes(bpy, objects):
    meshes = []
    report_preparation_warnings(objects)
    for obj in objects:
        converted = convert_object_to_mesh(bpy, obj)
        if converted is not None and converted.type == "MESH":
            meshes.append(converted)
    return unique_objects(meshes)


def find_visual_objects(bpy, args, collision_collection_raw):
    visual_collection = collection_objects(bpy, args.visual_collection)
    collision_names = object_set(collision_collection_raw or [])

    if visual_collection is not None:
        info(f"Using visual collection: {args.visual_collection}")
        candidates = [
            obj
            for obj in visual_collection
            if is_export_candidate(obj) and obj.name not in collision_names
        ]
    else:
        info(
            f"Visual collection '{args.visual_collection}' not found; using visible mesh-like objects."
        )
        candidates = [
            obj
            for obj in bpy.context.scene.objects
            if is_export_candidate(obj) and obj.name not in collision_names
        ]

    ignored = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type not in SUPPORTED_CONVERTIBLE_TYPES | IGNORED_TYPES
    ]
    for obj in ignored:
        warn(f"Object {obj.name} is type {obj.type} and will not be exported.")

    return unique_objects(candidates)


def world_bounds(mathutils, objects):
    points = []
    for obj in objects:
        if obj.type != "MESH":
            continue
        for corner in obj.bound_box:
            points.append(obj.matrix_world @ mathutils.Vector(corner))

    if not points:
        raise RuntimeError("No mesh bounds could be computed.")

    min_x = min(p.x for p in points)
    max_x = max(p.x for p in points)
    min_y = min(p.y for p in points)
    max_y = max(p.y for p in points)
    min_z = min(p.z for p in points)
    max_z = max(p.z for p in points)

    return min_x, max_x, min_y, max_y, min_z, max_z


def bounds_center_xy(bounds):
    min_x, max_x, min_y, max_y, _, _ = bounds
    return (min_x + max_x) * 0.5, (min_y + max_y) * 0.5


def bounds_size(bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    return max_x - min_x, max_y - min_y, max_z - min_z


def log_visual_bounds(bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    size_x, size_y, size_z = bounds_size(bounds)

    info("Visual bounds after origin adjustment:")
    info(f"  min = ({min_x:.3f}, {min_y:.3f}, {min_z:.3f})")
    info(f"  max = ({max_x:.3f}, {max_y:.3f}, {max_z:.3f})")
    info(f"  size = ({size_x:.3f}, {size_y:.3f}, {size_z:.3f})")

    if size_z < max(size_x, size_y):
        warn("Visual model height may not be along Z.")


def branch_name_kind(obj):
    name = obj.name.lower()

    if any(token in name for token in BRANCH_EXCLUDE_TOKENS):
        return "excluded"

    if any(token in name for token in BRANCH_INCLUDE_TOKENS):
        return "included"

    return "unknown"


def semantic_visual_kind(obj):
    kind = branch_name_kind(obj)
    if kind == "excluded":
        return "leaf"
    if kind == "included":
        return "wood"
    return "unknown"


def collection_export_candidates(bpy, collection_name, visual_name_set):
    collection = collection_objects(bpy, collection_name)
    if collection is None:
        return None

    return [
        obj
        for obj in collection.all_objects
        if obj.name in visual_name_set and obj.type == "MESH"
    ]


def find_split_visual_groups(bpy, args, visual_objects):
    visual_name_set = object_set(visual_objects)

    leaf_from_collection = collection_export_candidates(
        bpy, args.leaf_collection, visual_name_set
    )
    wood_from_collection = collection_export_candidates(
        bpy, args.wood_collection, visual_name_set
    )

    if leaf_from_collection and wood_from_collection:
        return (
            unique_objects(leaf_from_collection),
            unique_objects(wood_from_collection),
            f"collections:{args.leaf_collection},{args.wood_collection}",
        )

    if leaf_from_collection is not None or wood_from_collection is not None:
        warn(
            "Semantic collections are partially available but incomplete. Falling back to name classification."
        )

    leaf_objects = []
    wood_objects = []
    ignored_unknown = []

    for obj in visual_objects:
        kind = semantic_visual_kind(obj)

        if kind == "leaf":
            leaf_objects.append(obj)
        elif kind == "wood":
            wood_objects.append(obj)
        elif args.split_unknown_as == "leaf":
            leaf_objects.append(obj)
        elif args.split_unknown_as == "wood":
            wood_objects.append(obj)
        else:
            ignored_unknown.append(obj)

    if ignored_unknown:
        warn(
            "Ignoring unclassified visual objects: "
            + ", ".join(obj.name for obj in ignored_unknown)
        )

    return (
        unique_objects(leaf_objects),
        unique_objects(wood_objects),
        f"name_tokens:unknown_as_{args.split_unknown_as}",
    )


def write_semantic_parts(
    output_dir,
    model_name,
    leaf_mesh,
    wood_mesh,
    split_source,
    leaf_lod_mode,
    segmentation_labels=None,
    collision_part=None,
):
    segmentation_labels = segmentation_labels or {}
    collision_part = collision_part or {
        "name": "tree_collision",
        "role": "collision",
        "class": "collision_wood",
        "contact_policy": "AVOID",
        "mesh": "meshes/tree_collision.stl",
        "gazebo_collision": "tree_link::tree_collision",
        "segmentation_label": None,
    }

    data = {
        "model": model_name,
        "visual_mode": "split_leaf_wood",
        "leaf_lod_mode": leaf_lod_mode,
        "split_source": split_source,
        "parts": [
            {
                "name": "leaf_visual",
                "class": "leaf",
                "contact_policy": "ALLOW_CONTACT",
                "mesh": f"meshes/{leaf_mesh}",
                "gazebo_visual": "tree_link::leaf_visual",
                "segmentation_label": segmentation_labels.get("leaf"),
            },
            {
                "name": "wood_visual",
                "class": "wood",
                "contact_policy": "AVOID",
                "mesh": f"meshes/{wood_mesh}",
                "gazebo_visual": "tree_link::wood_visual",
                "segmentation_label": segmentation_labels.get("wood"),
            },
            collision_part,
        ],
    }

    path = output_dir / "semantic_parts.json"
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return path


def curve_radius_scale(obj):
    scale = obj.matrix_world.to_scale()
    return (abs(scale.x) + abs(scale.y) + abs(scale.z)) / 3.0


def spline_points_and_radii(mathutils, obj, spline):
    matrix = obj.matrix_world
    bevel_depth = getattr(obj.data, "bevel_depth", 0.0) or 0.0
    radius_scale = curve_radius_scale(obj)

    points = []
    radii = []

    if spline.type == "BEZIER":
        for point in spline.bezier_points:
            points.append(matrix @ point.co)
            radii.append(bevel_depth * getattr(point, "radius", 1.0) * radius_scale)
    else:
        for point in spline.points:
            co = point.co
            weight = co.w if abs(co.w) > 1e-9 else 1.0
            points.append(
                matrix @ mathutils.Vector((co.x / weight, co.y / weight, co.z / weight))
            )
            radii.append(bevel_depth * getattr(point, "radius", 1.0) * radius_scale)

    return points, radii


def collect_thick_branch_segments(mathutils, objects, args):
    if not args.collision_include_main_branches:
        return []

    segments = []

    for obj in objects:
        kind = branch_name_kind(obj)

        if kind == "excluded":
            continue

        if obj.type != "CURVE":
            if kind == "included" and obj.type == "MESH":
                warn(
                    f"Branch-like object {obj.name} is a mesh，not a Curve. "
                    "thick_branch_cylinders cannot infer segments."
                )
            continue

        if kind == "unknown" and (getattr(obj.data, "bevel_depth", 0.0) or 0.0) <= 0.0:
            continue

        obj_segments = 0

        for spline in obj.data.splines:
            points, radii = spline_points_and_radii(mathutils, obj, spline)

            if len(points) < 2:
                continue

            if not any(radius > 0.0 for radius in radii):
                warn(f"Curve object {obj.name} has no usable bevel/radius information.")
                continue

            for index in range(len(points) - 1):
                p0 = points[index]
                p1 = points[index + 1]
                radius = min(radii[index], radii[index + 1])
                midpoint_z = (p0.z + p1.z) * 0.5

                if radius < args.collision_min_radius:
                    continue

                if midpoint_z > args.collision_max_branch_height:
                    continue

                if (p1 - p0).length <= 1e-6:
                    continue

                segments.append((p0.copy(), p1.copy(), radius, obj.name))
                obj_segments += 1

        if obj_segments:
            info(
                f"Collected {obj_segments} thick branch collision segments from {obj.name}"
            )

    if not segments:
        warn(
            "No usable thick branch curve segments found. Falling back may be required."
        )

    return segments


def transform_branch_segments(branch_segments, scale, origin_vector):
    transformed = []

    for p0, p1, radius, source_name in branch_segments:
        transformed.append(
            (
                p0 * scale - origin_vector,
                p1 * scale - origin_vector,
                radius * scale,
                source_name,
            )
        )

    return transformed


def apply_uniform_scale(objects, scale):
    if abs(scale - 1.0) < 1e-9:
        return

    for obj in objects:
        obj.location.x *= scale
        obj.location.y *= scale
        obj.location.z *= scale
        obj.scale.x *= scale
        obj.scale.y *= scale
        obj.scale.z *= scale


def compute_origin_vector(bpy, mathutils, args, visual_objects):
    if args.origin_mode == "keep":
        return mathutils.Vector((0.0, 0.0, 0.0))

    if args.origin_mode == "cursor":
        return mathutils.Vector(bpy.context.scene.cursor.location) * args.scale

    if args.origin_mode == "named_empty":
        empty = bpy.data.objects.get(args.origin_empty_name)
        if empty is None:
            raise RuntimeError(
                f"origin-mode=named_empty was requested，but Empty '{args.origin_empty_name}' was not found."
            )
        return mathutils.Vector(empty.matrix_world.translation) * args.scale

    bounds = world_bounds(mathutils, visual_objects)
    center_x, center_y = bounds_center_xy(bounds)
    min_z = bounds[4]
    return mathutils.Vector((center_x, center_y, min_z))


def shift_objects(objects, origin_vector):
    if origin_vector.length <= 1e-12:
        return

    for obj in objects:
        obj.location.x -= origin_vector.x
        obj.location.y -= origin_vector.y
        obj.location.z -= origin_vector.z


def copy_material_textures(bpy, visual_objects, textures_dir):
    copied = []
    textures_dir.mkdir(parents=True, exist_ok=True)
    used_names = set()

    for obj in visual_objects:
        for slot in getattr(obj, "material_slots", []):
            mat = slot.material

            if mat is None or not mat.use_nodes:
                continue

            for node in mat.node_tree.nodes:
                if node.type != "TEX_IMAGE" or node.image is None:
                    continue

                image = node.image

                if image.packed_file is not None:
                    warn(
                        f"Image texture '{image.name}' is packed. "
                        "Unpack it if Gazebo needs the texture."
                    )
                    continue

                source = Path(bpy.path.abspath(image.filepath))

                if not source.exists():
                    warn(f"Image texture path does not exist: {source}")
                    continue

                target_name = source.name
                stem = source.stem
                suffix = source.suffix
                index = 1

                while target_name in used_names:
                    target_name = f"{stem}_{index}{suffix}"
                    index += 1

                used_names.add(target_name)

                target = textures_dir / target_name
                shutil.copy2(source, target)
                image.filepath = str(target)
                copied.append(target)

    return copied


def operator_property_names(operator):
    try:
        return {prop.identifier for prop in operator.get_rna_type().properties}
    except Exception:
        return set()


def export_obj_compatible(bpy, objects, path):
    selected = select_only_meshes(bpy, objects)

    if not selected:
        raise RuntimeError("No mesh objects selected for OBJ export.")

    if not hasattr(bpy.ops.wm, "obj_export"):
        raise RuntimeError(
            "Blender OBJ export operator bpy.ops.wm.obj_export is not available."
        )

    operator = bpy.ops.wm.obj_export
    properties = operator_property_names(operator)

    kwargs = {"filepath": str(path)}

    if not properties or "export_selected_objects" in properties:
        kwargs["export_selected_objects"] = True

    # Keep OBJ / MTL material assignment when the Blender OBJ exporter supports it.
    if not properties or "export_materials" in properties:
        kwargs["export_materials"] = True
    if not properties or "path_mode" in properties:
        kwargs["path_mode"] = "RELATIVE"

    axis_settings = []

    if "forward_axis" in properties and "up_axis" in properties:
        kwargs["forward_axis"] = "Y"
        kwargs["up_axis"] = "Z"
        axis_settings.append("forward_axis=Y")
        axis_settings.append("up_axis=Z")
    elif "axis_forward" in properties and "axis_up" in properties:
        kwargs["axis_forward"] = "Y"
        kwargs["axis_up"] = "Z"
        axis_settings.append("axis_forward=Y")
        axis_settings.append("axis_up=Z")
    else:
        warn(
            "OBJ export axis options were unavailable. Exporting with operator defaults."
        )
        axis_settings.append("axis=operator_default")

    info("OBJ export axis settings used: " + ", ".join(axis_settings))

    try:
        operator(**kwargs)
    except TypeError as exc:
        raise RuntimeError(f"OBJ export failed: {exc}") from exc


def export_stl(bpy, objects, path):
    selected = select_only_meshes(bpy, objects)

    if not selected:
        raise RuntimeError("No collision mesh objects selected for STL export.")

    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=str(path), export_selected_objects=True)
    elif hasattr(bpy.ops, "export_mesh") and hasattr(bpy.ops.export_mesh, "stl"):
        bpy.ops.export_mesh.stl(filepath=str(path), use_selection=True)
    else:
        raise RuntimeError("STL export operator is not available.")


def create_trunk_collision(bpy, args):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=max(6, args.collision_cylinder_sides),
        radius=args.trunk_radius,
        depth=args.trunk_height,
        location=(args.trunk_center_x, args.trunk_center_y, args.trunk_height * 0.5),
    )

    obj = bpy.context.object
    obj.name = "tree_collision_trunk_cylinder"
    return [obj]


def create_cylinder_between_points(bpy, mathutils, name, p0, p1, radius, sides):
    p0_vec = mathutils.Vector(p0)
    p1_vec = mathutils.Vector(p1)
    direction = p1_vec - p0_vec
    length = direction.length

    if length <= 1e-6 or radius <= 0.0:
        return None

    midpoint = (p0_vec + p1_vec) * 0.5

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=max(6, sides),
        radius=radius,
        depth=length,
        location=midpoint,
    )

    obj = bpy.context.object
    obj.name = name
    obj.rotation_euler = direction.to_track_quat("Z", "Y").to_euler()
    return obj


def create_thick_branch_collision(bpy, mathutils, args, branch_segments):
    if not branch_segments:
        warn(
            "No curve-derived branch collision segments found. Falling back to trunk_cylinder."
        )
        return create_trunk_collision(bpy, args)

    collision_objects = []

    if args.collision_add_trunk_cylinder:
        info(
            "Adding fixed trunk cylinder in addition to curve-derived branch collision."
        )
        collision_objects.extend(create_trunk_collision(bpy, args))
    else:
        info("Using curve-derived thick branch collision only.")

    created = 0

    for index, (p0, p1, radius, source_name) in enumerate(branch_segments):
        obj = create_cylinder_between_points(
            bpy,
            mathutils,
            f"tree_collision_branch_{index:03d}",
            p0,
            p1,
            radius,
            args.collision_cylinder_sides,
        )

        if obj is not None:
            obj["source_object"] = source_name
            collision_objects.append(obj)
            created += 1

    if created:
        info(f"Generated {created} coarse thick-branch collision cylinders")
    else:
        warn(
            "No thick branch cylinders were generated. Falling back to trunk_cylinder."
        )
        return create_trunk_collision(bpy, args)

    return collision_objects


def create_bounding_box_collision(bpy, bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds

    bpy.ops.mesh.primitive_cube_add(
        size=1.0,
        location=(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        ),
    )

    obj = bpy.context.object
    obj.name = "tree_collision_bounding_box"
    obj.dimensions = (max_x - min_x, max_y - min_y, max_z - min_z)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    return [obj]


def create_bounding_cylinder_collision(bpy, bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds

    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    radius = max(max_x - min_x, max_y - min_y) * 0.5
    height = max_z - min_z

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24,
        radius=max(radius, 0.001),
        depth=max(height, 0.001),
        location=(center_x, center_y, min_z + height * 0.5),
    )

    obj = bpy.context.object
    obj.name = "tree_collision_bounding_cylinder"
    return [obj]


def duplicate_decimated_visual_collision(bpy, visual_objects):
    selected = select_only_meshes(bpy, visual_objects)

    if not selected:
        raise RuntimeError(
            "No visual mesh objects available for visual_decimated_optional collision."
        )

    bpy.ops.object.duplicate()
    duplicated = list(bpy.context.selected_objects)

    for obj in duplicated:
        obj.name = f"tree_collision_{obj.name}"

        try:
            modifier = obj.modifiers.new("collision_decimate", "DECIMATE")
            modifier.ratio = 0.25
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate collision object {obj.name}: {exc}")

    return duplicated


def make_generated_collision(
    bpy, mathutils, args, visual_bounds, visual_objects, branch_segments
):
    if args.collision_mode == "trunk_cylinder":
        return create_trunk_collision(bpy, args)

    if args.collision_mode == "thick_branch_cylinders":
        return create_thick_branch_collision(bpy, mathutils, args, branch_segments)

    if args.collision_mode == "bounding_box":
        return create_bounding_box_collision(bpy, visual_bounds)

    if args.collision_mode == "bounding_cylinder":
        return create_bounding_cylinder_collision(bpy, visual_bounds)

    if args.collision_mode == "visual_decimated_optional":
        warn("visual_decimated_optional can be expensive for Gazebo.")
        return duplicate_decimated_visual_collision(bpy, visual_objects)

    raise RuntimeError(f"Unsupported collision mode: {args.collision_mode}")


def mesh_object_center(mathutils, obj):
    points = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
    return sum(points, mathutils.Vector((0.0, 0.0, 0.0))) / len(points)


def quantized_key(p, eps=1e-5):
    return (
        round(p.x / eps),
        round(p.y / eps),
        round(p.z / eps),
    )


def unique_points(points, eps=1e-5):
    seen = set()
    out = []
    for p in points:
        key = quantized_key(p, eps)
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out


def sample_all_leaf_vertices_world(mathutils, leaf_objects, max_total_points=1200):
    points = []

    for obj in leaf_objects:
        if obj.type != "MESH":
            continue

        matrix = obj.matrix_world
        verts = obj.data.vertices

        for v in verts:
            points.append(matrix @ v.co)

    points = unique_points(points)

    if max_total_points > 0 and len(points) > max_total_points:
        step = max(1, len(points) // max_total_points)
        points = points[::step][:max_total_points]

    return points


def simple_kmeans(mathutils, points, k, iterations):
    if not points:
        return []

    k = max(1, min(k, len(points)))

    if len(points) <= k:
        return [[p] for p in points]

    # Z方向とXY方向にそこそこ散るように初期化
    sorted_points = sorted(points, key=lambda p: (p.z, p.x, p.y))
    centers = [
        sorted_points[int(i * (len(sorted_points) - 1) / max(1, k - 1))].copy()
        for i in range(k)
    ]

    for _ in range(iterations):
        clusters = [[] for _ in range(k)]

        for p in points:
            idx = min(
                range(k),
                key=lambda i: (p - centers[i]).length_squared,
            )
            clusters[idx].append(p)

        for i, cluster in enumerate(clusters):
            if cluster:
                centers[i] = sum(
                    cluster,
                    mathutils.Vector((0.0, 0.0, 0.0)),
                ) / len(cluster)

    return [cluster for cluster in clusters if cluster]


def sample_mesh_vertices_world(mathutils, obj, max_count):
    if obj.type != "MESH":
        return []

    verts = obj.data.vertices
    if not verts:
        return []

    if max_count <= 0 or len(verts) <= max_count:
        indices = range(len(verts))
    else:
        step = max(1, len(verts) // max_count)
        indices = range(0, len(verts), step)

    points = []
    matrix = obj.matrix_world

    for i in indices:
        if len(points) >= max_count and max_count > 0:
            break
        points.append(matrix @ verts[i].co)

    return points


def add_jitter_points(mathutils, points, radius):
    if radius <= 0.0:
        return unique_points(points)

    offsets = [
        mathutils.Vector(( radius, 0.0, 0.0)),
        mathutils.Vector((-radius, 0.0, 0.0)),
        mathutils.Vector((0.0,  radius, 0.0)),
        mathutils.Vector((0.0, -radius, 0.0)),
        mathutils.Vector((0.0, 0.0,  radius)),
        mathutils.Vector((0.0, 0.0, -radius)),
    ]

    out = []
    for p in points:
        out.append(p)
        for off in offsets:
            out.append(p + off)

    return unique_points(out)


def create_convex_hull_from_points(bpy, mathutils, points, name, min_points, decimate_ratio):
    import bmesh

    points = unique_points(points)

    if len(points) < min_points:
        warn(f"Cluster {name} has only {len(points)} unique points. Skipping.")
        return None

    mesh = bpy.data.meshes.new(name + "_mesh")
    bm = bmesh.new()

    bm_verts = []
    for p in points:
        bm_verts.append(bm.verts.new((p.x, p.y, p.z)))

    bm.verts.ensure_lookup_table()

    try:
        result = bmesh.ops.convex_hull(bm, input=bm_verts)

        # geom_unused / geom_interior に重複が混ざる場合があるのでsetで除去する
        delete_geom = []
        seen_ids = set()

        for key in ("geom_unused", "geom_interior"):
            for g in result.get(key, []):
                gid = id(g)
                if gid in seen_ids:
                    continue
                seen_ids.add(gid)
                delete_geom.append(g)

        if delete_geom:
            bmesh.ops.delete(bm, geom=delete_geom, context="VERTS")

        bm.to_mesh(mesh)

    except Exception as exc:
        bm.free()
        warn(f"Convex hull generation failed for {name}: {exc}")
        return None

    bm.free()

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)

    if len(obj.data.polygons) == 0:
        warn(f"Convex hull {name} has no faces. Skipping.")
        bpy.data.objects.remove(obj, do_unlink=True)
        return None

    obj.data.update()

    # flat shading
    for poly in obj.data.polygons:
        poly.use_smooth = False

    if decimate_ratio < 0.999:
        try:
            bpy.ops.object.select_all(action="DESELECT")
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj

            modifier = obj.modifiers.new("leaf_hull_decimate", "DECIMATE")
            modifier.ratio = max(0.05, min(1.0, decimate_ratio))
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate hull {name}: {exc}")

    return obj


def join_objects_to_single_mesh(bpy, objects, name):
    selected = select_only_meshes(bpy, objects)

    if not selected:
        return []

    bpy.context.view_layer.objects.active = selected[0]
    bpy.ops.object.join()

    joined = bpy.context.object
    joined.name = name
    joined.data.name = name + "_mesh"

    return [joined]


def create_leaf_convex_hull_lod(bpy, mathutils, leaf_objects, args):
    if not leaf_objects:
        return []

    # 重要：葉オブジェクト中心ではなく，葉メッシュ頂点をクラスタリングする
    leaf_points = sample_all_leaf_vertices_world(
        mathutils,
        leaf_objects,
        max_total_points=max(
            300,
            args.leaf_cluster_count * args.leaf_sample_vertices_per_object * 20,
        ),
    )

    if not leaf_points:
        warn("No leaf vertices were collected for convex hull LOD.")
        return []

    clusters = simple_kmeans(
        mathutils,
        leaf_points,
        args.leaf_cluster_count,
        args.leaf_kmeans_iterations,
    )

    hull_objects = []

    for idx, cluster_points in enumerate(clusters):
        if not cluster_points:
            continue

        points = add_jitter_points(
            mathutils,
            cluster_points,
            args.leaf_hull_jitter,
        )

        hull = create_convex_hull_from_points(
            bpy,
            mathutils,
            points,
            f"leaf_hull_{idx:02d}",
            args.leaf_hull_min_points,
            args.leaf_hull_decimate_ratio,
        )

        if hull is not None:
            mat = bpy.data.materials.get("leaf_hull_green")
            if mat is None:
                mat = bpy.data.materials.new("leaf_hull_green")
                mat.diffuse_color = (0.0, 0.32, 0.0, 1.0)
            hull.data.materials.append(mat)
            hull_objects.append(hull)

    info(
        f"Generated {len(hull_objects)} convex-hull leaf LOD objects "
        f"from {len(leaf_objects)} leaf objects and {len(leaf_points)} sampled leaf points."
    )

    return hull_objects



def collect_mesh_islands_for_object(mathutils, obj):
    """Return mesh islands with face indices and world-space center.

    This uses polygon adjacency through shared edges. It works well for leaf-card
    style meshes where each leaf remains an independent mesh island inside one
    joined leaf object.
    """
    if obj.type != "MESH":
        return []

    mesh = obj.data
    polygons = list(mesh.polygons)
    if not polygons:
        return []

    edge_to_faces = {}
    for poly in polygons:
        for edge_key in poly.edge_keys:
            edge_to_faces.setdefault(tuple(edge_key), []).append(poly.index)

    face_neighbors = {poly.index: set() for poly in polygons}
    for face_indices in edge_to_faces.values():
        if len(face_indices) < 2:
            continue
        for face_index in face_indices:
            face_neighbors[face_index].update(face_indices)
            face_neighbors[face_index].discard(face_index)

    visited = set()
    islands = []

    for poly in polygons:
        if poly.index in visited:
            continue

        stack = [poly.index]
        visited.add(poly.index)
        face_indices = []
        vertex_indices = set()

        while stack:
            face_index = stack.pop()
            face_indices.append(face_index)
            face = mesh.polygons[face_index]
            vertex_indices.update(face.vertices)

            for neighbor_index in face_neighbors[face_index]:
                if neighbor_index not in visited:
                    visited.add(neighbor_index)
                    stack.append(neighbor_index)

        if vertex_indices:
            center = mathutils.Vector((0.0, 0.0, 0.0))
            for vertex_index in vertex_indices:
                center += obj.matrix_world @ mesh.vertices[vertex_index].co
            center /= len(vertex_indices)
        else:
            center = obj.matrix_world @ poly.center

        islands.append(
            {
                "object": obj,
                "face_indices": set(face_indices),
                "center": center,
                "face_count": len(face_indices),
            }
        )

    return islands


def duplicate_object_with_face_subset(bpy, obj, face_indices_to_keep, name):
    if obj.type != "MESH" or not face_indices_to_keep:
        return None

    import bmesh

    mesh_copy = obj.data.copy()
    dup = obj.copy()
    dup.data = mesh_copy
    dup.name = name
    dup.data.name = name + "_mesh"
    bpy.context.collection.objects.link(dup)

    bm = bmesh.new()
    bm.from_mesh(mesh_copy)
    bm.faces.ensure_lookup_table()

    delete_faces = [face for face in bm.faces if face.index not in face_indices_to_keep]
    if delete_faces:
        bmesh.ops.delete(bm, geom=delete_faces, context="FACES")

    bm.verts.ensure_lookup_table()
    loose_verts = [vert for vert in bm.verts if not vert.link_faces]
    if loose_verts:
        bmesh.ops.delete(bm, geom=loose_verts, context="VERTS")

    bm.to_mesh(mesh_copy)
    bm.free()
    mesh_copy.update()

    if len(mesh_copy.polygons) == 0:
        bpy.data.objects.remove(dup, do_unlink=True)
        return None

    return dup



def first_material_from_objects(objects):
    for obj in objects:
        # Prefer material slots because they preserve the material actually assigned
        # to the object after conversion / duplication.
        for slot in getattr(obj, "material_slots", []):
            if slot.material is not None:
                return slot.material

        materials = getattr(getattr(obj, "data", None), "materials", [])
        for mat in materials:
            if mat is not None:
                return mat
    return None


def material_base_color_rgba(source_material, fallback=(0.0, 0.32, 0.0, 1.0)):
    if source_material is None:
        return fallback

    # Prefer Principled BSDF Base Color when a node material is used. This is
    # more reliable than diffuse_color for assets whose viewport color and node
    # material differ.
    if getattr(source_material, "use_nodes", False) and source_material.node_tree is not None:
        for node in source_material.node_tree.nodes:
            if node.type == "BSDF_PRINCIPLED":
                input_socket = node.inputs.get("Base Color")
                if input_socket is not None:
                    try:
                        color = input_socket.default_value
                        return (float(color[0]), float(color[1]), float(color[2]), float(color[3]))
                    except Exception:
                        pass

    if hasattr(source_material, "diffuse_color"):
        try:
            color = source_material.diffuse_color
            return (float(color[0]), float(color[1]), float(color[2]), float(color[3]))
        except Exception:
            pass

    return fallback


def create_plain_material_from_source(
    bpy,
    source_material,
    name,
    fallback=(0.0, 0.32, 0.0, 1.0),
):
    color = material_base_color_rgba(source_material, fallback)

    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)

    mat.diffuse_color = color
    mat.use_nodes = True

    bsdf = None
    if mat.node_tree is not None:
        for node in mat.node_tree.nodes:
            if node.type == "BSDF_PRINCIPLED":
                bsdf = node
                break

    if bsdf is not None:
        if "Base Color" in bsdf.inputs:
            bsdf.inputs["Base Color"].default_value = color
        if "Alpha" in bsdf.inputs:
            bsdf.inputs["Alpha"].default_value = color[3]
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.8
        if "Metallic" in bsdf.inputs:
            bsdf.inputs["Metallic"].default_value = 0.0

    return mat


def clone_leaf_proxy_material(bpy, leaf_objects, name="leaf_inner_metaball_material"):
    source = first_material_from_objects(leaf_objects)
    mat = create_plain_material_from_source(
        bpy,
        source,
        name,
        (0.0, 0.32, 0.0, 1.0),
    )
    color = material_base_color_rgba(source, (0.0, 0.32, 0.0, 1.0))
    info(
        "Leaf inner metaball material inherited base color: "
        f"rgba=({color[0]:.3f}, {color[1]:.3f}, {color[2]:.3f}, {color[3]:.3f})"
    )
    return mat


def points_from_islands(mathutils, islands):
    points = []
    for island in islands:
        obj = island["object"]
        mesh = obj.data
        vertex_indices = set()
        for face_index in island["face_indices"]:
            face = mesh.polygons[face_index]
            vertex_indices.update(face.vertices)
        for vertex_index in vertex_indices:
            points.append(obj.matrix_world @ mesh.vertices[vertex_index].co)
    return points


def downsample_points(points, stride, max_points):
    if not points:
        return []

    stride = max(1, int(stride))
    sampled = points[::stride]

    max_points = max(1, int(max_points))
    if len(sampled) > max_points:
        step = max(1, len(sampled) // max_points)
        sampled = sampled[::step][:max_points]

    return sampled


def create_inner_metaball_proxy(bpy, mathutils, islands, leaf_objects, args, crown_center):
    if not islands:
        return None

    points = points_from_islands(mathutils, islands)
    points = unique_points(points)

    position_scale = max(0.0, min(1.0, float(args.leaf_inner_metaball_position_scale)))
    if position_scale < 0.999:
        points = [crown_center + (point - crown_center) * position_scale for point in points]

    points = downsample_points(
        points,
        args.leaf_inner_metaball_point_stride,
        args.leaf_inner_metaball_max_points,
    )

    if not points:
        warn("No points were available for inner metaball proxy generation.")
        return None

    resolution = float(args.leaf_inner_metaball_resolution)
    render_resolution = (
        float(args.leaf_inner_metaball_render_resolution)
        if args.leaf_inner_metaball_render_resolution is not None
        else resolution
    )

    try:
        bpy.ops.object.metaball_add(type="BALL", location=points[0])
    except Exception as exc:
        warn(f"Inner metaball creation failed: {exc}")
        return None

    meta_obj = bpy.context.object
    meta_obj.name = "leaf_inner_metaball"
    meta_obj.data.name = "leaf_inner_metaball_meta"
    meta_obj.data.resolution = resolution
    meta_obj.data.render_resolution = render_resolution
    meta_obj.data.threshold = float(args.leaf_inner_metaball_threshold)
    meta_obj.data.elements[0].radius = float(args.leaf_inner_metaball_radius)

    for point in points[1:]:
        elem = meta_obj.data.elements.new(type="BALL")
        elem.co = point - meta_obj.location
        elem.radius = float(args.leaf_inner_metaball_radius)

    bpy.ops.object.select_all(action="DESELECT")
    meta_obj.select_set(True)
    bpy.context.view_layer.objects.active = meta_obj

    try:
        bpy.ops.object.convert(target="MESH")
    except Exception as exc:
        warn(f"Inner metaball conversion failed: {exc}")
        return None

    mesh_obj = bpy.context.object
    mesh_obj.name = "leaf_inner_metaball_mesh"
    mesh_obj.data.name = "leaf_inner_metaball_mesh_data"

    material = clone_leaf_proxy_material(bpy, leaf_objects)
    if material is not None:
        mesh_obj.data.materials.append(material)

    for poly in mesh_obj.data.polygons:
        poly.use_smooth = bool(args.leaf_inner_metaball_shade_smooth)

    if args.leaf_inner_metaball_decimate_ratio < 0.999:
        try:
            bpy.ops.object.select_all(action="DESELECT")
            mesh_obj.select_set(True)
            bpy.context.view_layer.objects.active = mesh_obj
            modifier = mesh_obj.modifiers.new("leaf_inner_metaball_decimate", "DECIMATE")
            modifier.ratio = max(0.05, min(1.0, args.leaf_inner_metaball_decimate_ratio))
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate inner metaball proxy: {exc}")

    info(
        "Generated inner metaball proxy: "
        f"source_islands={len(islands)}, source_points={len(points)}, "
        f"faces={len(mesh_obj.data.polygons)}, radius={args.leaf_inner_metaball_radius:.3f}, "
        f"position_scale={args.leaf_inner_metaball_position_scale:.3f}, "
        f"resolution={resolution:.3f}, threshold={args.leaf_inner_metaball_threshold:.3f}."
    )

    return mesh_obj


def create_leaf_island_surface_lod(bpy, mathutils, leaf_objects, args):
    if not leaf_objects:
        return []

    all_islands = []
    for obj in leaf_objects:
        islands = collect_mesh_islands_for_object(mathutils, obj)
        all_islands.extend(islands)
        info(
            f"Leaf island analysis: {obj.name}: "
            f"{len(islands)} islands, {sum(item['face_count'] for item in islands)} faces"
        )

    if not all_islands:
        warn("No leaf mesh islands were found for island_surface LOD.")
        return []

    center = mathutils.Vector((0.0, 0.0, 0.0))
    for island in all_islands:
        center += island["center"]
    center /= len(all_islands)

    min_x = min(island["center"].x for island in all_islands)
    max_x = max(island["center"].x for island in all_islands)
    min_y = min(island["center"].y for island in all_islands)
    max_y = max(island["center"].y for island in all_islands)
    min_z = min(island["center"].z for island in all_islands)
    max_z = max(island["center"].z for island in all_islands)

    scale = mathutils.Vector(
        (
            max((max_x - min_x) * 0.5, 1.0e-6),
            max((max_y - min_y) * 0.5, 1.0e-6),
            max((max_z - min_z) * 0.5, 1.0e-6),
        )
    )

    for island in all_islands:
        delta = island["center"] - center
        if args.leaf_surface_score_mode == "distance":
            score = delta.length
        else:
            score = mathutils.Vector(
                (delta.x / scale.x, delta.y / scale.y, delta.z / scale.z)
            ).length
        island["surface_score"] = score

    keep_ratio = max(0.0, min(1.0, args.leaf_surface_keep_ratio))
    keep_count = int(len(all_islands) * keep_ratio + 0.999999)
    keep_count = max(args.leaf_surface_min_islands, keep_count)
    keep_count = min(len(all_islands), keep_count)

    sorted_islands = sorted(
        all_islands,
        key=lambda item: item["surface_score"],
        reverse=True,
    )
    surface_islands = sorted_islands[:keep_count]
    inner_candidates = sorted_islands[keep_count:]

    inner_ratio = max(0.0, min(1.0, args.leaf_inner_keep_ratio))
    inner_keep_count = int(len(inner_candidates) * inner_ratio + 0.999999)
    inner_keep_count = min(len(inner_candidates), inner_keep_count)

    if inner_keep_count <= 0:
        inner_islands = []
    elif args.leaf_inner_keep_mode == "even":
        if inner_keep_count >= len(inner_candidates):
            inner_islands = list(inner_candidates)
        else:
            step = len(inner_candidates) / inner_keep_count
            inner_islands = [inner_candidates[int(i * step)] for i in range(inner_keep_count)]
    else:
        rng = random.Random(args.leaf_inner_random_seed)
        inner_islands = rng.sample(inner_candidates, inner_keep_count)

    kept_islands = surface_islands + inner_islands
    kept_island_ids = {id(item) for item in kept_islands}

    if surface_islands:
        surface_cutoff_score = min(item["surface_score"] for item in surface_islands)
    else:
        surface_cutoff_score = max(item["surface_score"] for item in all_islands)

    proxy_score_scale = max(0.0, min(1.0, float(args.leaf_inner_proxy_score_scale)))
    proxy_cutoff_score = surface_cutoff_score * proxy_score_scale

    proxy_islands = [
        item
        for item in inner_candidates
        if id(item) not in kept_island_ids and item["surface_score"] <= proxy_cutoff_score
    ]
    removed_transition_islands = [
        item
        for item in inner_candidates
        if id(item) not in kept_island_ids and item["surface_score"] > proxy_cutoff_score
    ]

    faces_by_object = {obj.name: set() for obj in leaf_objects if obj.type == "MESH"}
    object_by_name = {obj.name: obj for obj in leaf_objects if obj.type == "MESH"}

    for island in kept_islands:
        faces_by_object[island["object"].name].update(island["face_indices"])

    lod_objects = []
    kept_faces = 0
    original_faces = sum(island["face_count"] for island in all_islands)

    for obj_name, face_indices in faces_by_object.items():
        if not face_indices:
            continue
        dup = duplicate_object_with_face_subset(
            bpy,
            object_by_name[obj_name],
            face_indices,
            f"{obj_name}_island_surface_lod",
        )
        if dup is not None:
            lod_objects.append(dup)
            kept_faces += len(dup.data.polygons)

    proxy_faces = 0
    if args.leaf_inner_proxy_mode == "metaball" and proxy_islands:
        proxy_obj = create_inner_metaball_proxy(
            bpy,
            mathutils,
            proxy_islands,
            leaf_objects,
            args,
            center,
        )
        if proxy_obj is not None:
            lod_objects.append(proxy_obj)
            proxy_faces = len(proxy_obj.data.polygons)

    info(
        "Generated island-surface leaf LOD: "
        f"kept {len(kept_islands)} / {len(all_islands)} islands "
        f"(surface={len(surface_islands)}, inner_original={len(inner_islands)}, "
        f"inner_proxy={len(proxy_islands) if args.leaf_inner_proxy_mode == 'metaball' else 0}, "
        f"transition_removed={len(removed_transition_islands)}, "
        f"surface_ratio={keep_ratio:.3f}, inner_ratio={inner_ratio:.3f}, "
        f"proxy_score_scale={proxy_score_scale:.3f}, "
        f"surface_cutoff_score={surface_cutoff_score:.3f}, proxy_cutoff_score={proxy_cutoff_score:.3f}, "
        f"proxy_mode={args.leaf_inner_proxy_mode}), "
        f"original_leaf_faces {original_faces} -> kept_original_faces {kept_faces} "
        f"+ proxy_faces {proxy_faces} = {kept_faces + proxy_faces}."
    )
    info(
        "Leaf island surface center: "
        f"({center.x:.3f}, {center.y:.3f}, {center.z:.3f}), "
        f"score_mode={args.leaf_surface_score_mode}"
    )

    return lod_objects


def make_leaf_export_objects(bpy, mathutils, leaf_objects, args):
    if args.leaf_lod_mode == "original":
        return leaf_objects

    if args.leaf_lod_mode == "none":
        return []

    if args.leaf_lod_mode == "joined":
        return join_objects_to_single_mesh(bpy, leaf_objects, "joined_leaf_mesh")

    if args.leaf_lod_mode == "convex_hull":
        return create_leaf_convex_hull_lod(bpy, mathutils, leaf_objects, args)

    if args.leaf_lod_mode == "island_surface":
        return create_leaf_island_surface_lod(bpy, mathutils, leaf_objects, args)

    raise RuntimeError(f"Unsupported leaf_lod_mode: {args.leaf_lod_mode}")



def duplicate_mesh_objects_for_lod(bpy, objects, name_prefix):
    selected = select_only_meshes(bpy, objects)

    if not selected:
        return []

    bpy.ops.object.duplicate()
    duplicated = list(bpy.context.selected_objects)

    for index, obj in enumerate(duplicated):
        obj.name = f"{name_prefix}_{index:02d}_{obj.name}"
        obj.data.name = obj.name + "_mesh"

    return duplicated


def mesh_face_count(objects):
    total = 0
    for obj in objects:
        if obj.type == "MESH":
            total += len(obj.data.polygons)
    return total


def create_wood_decimate_lod(bpy, wood_objects, args):
    if not wood_objects:
        return []

    original_faces = mesh_face_count(wood_objects)
    lod_objects = duplicate_mesh_objects_for_lod(bpy, wood_objects, "wood_lod")

    if not lod_objects:
        warn("No wood mesh objects were available for decimation.")
        return []

    for obj in lod_objects:
        if obj.type != "MESH":
            continue

        try:
            bpy.ops.object.select_all(action="DESELECT")
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj

            modifier = obj.modifiers.new("wood_visual_decimate", "DECIMATE")
            modifier.ratio = max(0.05, min(1.0, args.wood_decimate_ratio))
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate wood object {obj.name}: {exc}")

        for poly in obj.data.polygons:
            poly.use_smooth = bool(args.wood_shade_smooth)

    if args.wood_join_after_decimate and len(lod_objects) > 1:
        joined = join_objects_to_single_mesh(bpy, lod_objects, "wood_decimated_joined")
        if joined:
            lod_objects = joined

    decimated_faces = mesh_face_count(lod_objects)
    ratio = (decimated_faces / original_faces) if original_faces > 0 else 0.0
    info(
        "Generated wood visual LOD: "
        f"objects={len(lod_objects)}, faces {original_faces} -> {decimated_faces} "
        f"(ratio={ratio:.3f}, requested={args.wood_decimate_ratio:.3f})."
    )

    return lod_objects


def make_wood_export_objects(bpy, wood_objects, args):
    if args.wood_lod_mode == "original":
        return wood_objects

    if args.wood_lod_mode == "none":
        return []

    if args.wood_lod_mode == "decimate":
        return create_wood_decimate_lod(bpy, wood_objects, args)

    raise RuntimeError(f"Unsupported wood_lod_mode: {args.wood_lod_mode}")


def validate_segmentation_label(label, class_name):
    if label is None:
        return None

    try:
        value = int(label)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"Segmentation label for {class_name} must be an integer: {label!r}"
        ) from exc

    if value < 0 or value > 255:
        raise RuntimeError(
            f"Segmentation label for {class_name} must be in [0, 255]: {value}"
        )

    return value


def make_segmentation_label_xml(label, indent="        "):
    if label is None:
        return ""
    return f"{indent}<label>{int(label)}</label>"


def build_tree_segmentation_labels(args):
    if not args.write_segmentation_labels:
        return {}

    return {
        "leaf": validate_segmentation_label(args.leaf_segmentation_label, "leaf"),
        "wood": validate_segmentation_label(args.wood_segmentation_label, "wood"),
    }



def sdf_float(value):
    return f"{float(value):.9g}"


def make_mesh_collision_xml(model_name):
    return f"""      <collision name="tree_collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/tree_collision.stl</uri>
          </mesh>
        </geometry>
      </collision>"""


def make_collision_primitive_description(args, visual_bounds):
    if args.collision_mode == "trunk_cylinder":
        pose = [
            float(args.trunk_center_x),
            float(args.trunk_center_y),
            float(args.trunk_height) * 0.5,
            0.0,
            0.0,
            0.0,
        ]
        return {
            "type": "cylinder",
            "radius": float(args.trunk_radius),
            "length": float(args.trunk_height),
            "pose": pose,
        }

    if args.collision_mode == "bounding_box":
        min_x, max_x, min_y, max_y, min_z, max_z = visual_bounds
        size = [max_x - min_x, max_y - min_y, max_z - min_z]
        pose = [
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
            0.0,
            0.0,
            0.0,
        ]
        return {"type": "box", "size": [float(v) for v in size], "pose": [float(v) for v in pose]}

    if args.collision_mode == "bounding_cylinder":
        min_x, max_x, min_y, max_y, min_z, max_z = visual_bounds
        center_x = (min_x + max_x) * 0.5
        center_y = (min_y + max_y) * 0.5
        radius = max(max_x - min_x, max_y - min_y) * 0.5
        height = max_z - min_z
        pose = [center_x, center_y, min_z + height * 0.5, 0.0, 0.0, 0.0]
        return {
            "type": "cylinder",
            "radius": float(max(radius, 0.001)),
            "length": float(max(height, 0.001)),
            "pose": [float(v) for v in pose],
        }

    return None


def make_collision_xml_from_description(desc, model_name):
    if desc is None:
        return make_mesh_collision_xml(model_name)

    pose_text = " ".join(sdf_float(v) for v in desc.get("pose", [0, 0, 0, 0, 0, 0]))

    if desc["type"] == "cylinder":
        return f"""      <collision name="tree_collision">
        <pose>{pose_text}</pose>
        <geometry>
          <cylinder>
            <radius>{sdf_float(desc["radius"])}</radius>
            <length>{sdf_float(desc["length"])}</length>
          </cylinder>
        </geometry>
      </collision>"""

    if desc["type"] == "box":
        size_text = " ".join(sdf_float(v) for v in desc["size"])
        return f"""      <collision name="tree_collision">
        <pose>{pose_text}</pose>
        <geometry>
          <box>
            <size>{size_text}</size>
          </box>
        </geometry>
      </collision>"""

    return make_mesh_collision_xml(model_name)


def make_collision_part_metadata(desc):
    base = {
        "name": "tree_collision",
        "role": "collision",
        "class": "collision_wood",
        "contact_policy": "AVOID",
        "gazebo_collision": "tree_link::tree_collision",
        "segmentation_label": None,
    }

    if desc is None:
        base["mesh"] = "meshes/tree_collision.stl"
    else:
        base["geometry"] = desc

    return base


def supports_sdf_primitive_collision(args, use_collision_file=False, use_collision_collection=False):
    if use_collision_file or use_collision_collection:
        return False
    return args.collision_mode in {"trunk_cylinder", "bounding_box", "bounding_cylinder"}

def write_gazebo_files(
    output_dir,
    model_name,
    split_visuals=False,
    leaf_mesh="tree_leaf.obj",
    wood_mesh="tree_wood.obj",
    segmentation_labels=None,
    collision_xml=None,
):
    segmentation_labels = segmentation_labels or {}
    collision_xml = collision_xml or make_mesh_collision_xml(model_name)

    if split_visuals:
        sdf_text = MODEL_SDF_SPLIT_VISUAL_TEMPLATE.format(
            model_name=model_name,
            leaf_mesh=leaf_mesh,
            wood_mesh=wood_mesh,
            leaf_segmentation_label_xml=make_segmentation_label_xml(
                segmentation_labels.get("leaf")
            ),
            wood_segmentation_label_xml=make_segmentation_label_xml(
                segmentation_labels.get("wood")
            ),
            collision_xml=collision_xml,
        )
    else:
        sdf_text = MODEL_SDF_TEMPLATE.format(
            model_name=model_name,
            collision_xml=collision_xml,
        )

    (output_dir / "model.sdf").write_text(sdf_text, encoding="utf-8")
    (output_dir / "model.config").write_text(
        MODEL_CONFIG_TEMPLATE.format(model_name=model_name),
        encoding="utf-8",
    )


def print_summary(
    args,
    input_path,
    output_dir,
    visual_objects,
    collision_source,
    bounds,
    copied,
    segmentation_labels=None,
):
    info("Conversion summary:")
    info(f"  input: {input_path}")
    info(f"  model_name: {args.model_name}")
    info(f"  output_dir: {output_dir}")
    info(f"  visual_format: {args.visual_format}")
    info(f"  origin_mode: {args.origin_mode}")
    info(f"  leaf_lod_mode: {args.leaf_lod_mode}")
    info(f"  wood_lod_mode: {args.wood_lod_mode}")
    info(f"  visual_objects: {', '.join(obj.name for obj in visual_objects)}")
    info(f"  collision_source: {collision_source}")
    info(f"  collision_output_mode: {args.collision_output_mode}")
    segmentation_labels = segmentation_labels or {}
    if segmentation_labels:
        info(
            "  segmentation_camera_labels: "
            + ", ".join(
                f"{name}={label}"
                for name, label in sorted(segmentation_labels.items())
            )
        )
    else:
        info("  segmentation_camera_labels: disabled")
    info(
        "  visual_bounds_after_origin: "
        f"x=[{bounds[0]:.3f}, {bounds[1]:.3f}], "
        f"y=[{bounds[2]:.3f}, {bounds[3]:.3f}], "
        f"z=[{bounds[4]:.3f}, {bounds[5]:.3f}]"
    )

    if copied:
        info("  copied_textures:")
        for path in copied:
            info(f"    {path}")


def validate_args(args):
    if args.scale <= 0.0:
        raise RuntimeError("--scale must be greater than zero")

    if args.write_segmentation_labels:
        validate_segmentation_label(args.leaf_segmentation_label, "leaf")
        validate_segmentation_label(args.wood_segmentation_label, "wood")

    if args.trunk_radius <= 0.0 or args.trunk_height <= 0.0:
        raise RuntimeError(
            "--trunk-radius and --trunk-height must be greater than zero"
        )

    if args.collision_min_radius <= 0.0:
        raise RuntimeError("--collision-min-radius must be greater than zero")

    if args.collision_cylinder_sides < 6:
        raise RuntimeError("--collision-cylinder-sides must be 6 or greater")

    if args.collision_max_branch_height <= 0.0:
        raise RuntimeError("--collision-max-branch-height must be greater than zero")

    if args.leaf_cluster_count <= 0:
        raise RuntimeError("--leaf-cluster-count must be greater than zero")

    if args.leaf_kmeans_iterations <= 0:
        raise RuntimeError("--leaf-kmeans-iterations must be greater than zero")

    if args.leaf_hull_jitter < 0.0:
        raise RuntimeError("--leaf-hull-jitter must be zero or greater")

    if args.leaf_hull_min_points < 4:
        raise RuntimeError("--leaf-hull-min-points must be 4 or greater")

    if not (0.0 < args.leaf_hull_decimate_ratio <= 1.0):
        raise RuntimeError("--leaf-hull-decimate-ratio must be in (0, 1].")

    if not (0.0 < args.leaf_surface_keep_ratio <= 1.0):
        raise RuntimeError("--leaf-surface-keep-ratio must be in (0, 1].")

    if args.leaf_surface_min_islands <= 0:
        raise RuntimeError("--leaf-surface-min-islands must be greater than zero.")

    if not (0.0 <= args.leaf_inner_keep_ratio <= 1.0):
        raise RuntimeError("--leaf-inner-keep-ratio must be in [0, 1].")

    if not (0.0 <= args.leaf_inner_proxy_score_scale <= 1.0):
        raise RuntimeError("--leaf-inner-proxy-score-scale must be in [0, 1].")

    if not (0.0 <= args.leaf_inner_metaball_position_scale <= 1.0):
        raise RuntimeError("--leaf-inner-metaball-position-scale must be in [0, 1].")

    if args.leaf_inner_metaball_radius <= 0.0:
        raise RuntimeError("--leaf-inner-metaball-radius must be greater than zero.")

    if args.leaf_inner_metaball_resolution <= 0.0:
        raise RuntimeError("--leaf-inner-metaball-resolution must be greater than zero.")

    if args.leaf_inner_metaball_render_resolution is not None and args.leaf_inner_metaball_render_resolution <= 0.0:
        raise RuntimeError("--leaf-inner-metaball-render-resolution must be greater than zero.")

    if args.leaf_inner_metaball_threshold <= 0.0:
        raise RuntimeError("--leaf-inner-metaball-threshold must be greater than zero.")

    if args.leaf_inner_metaball_point_stride <= 0:
        raise RuntimeError("--leaf-inner-metaball-point-stride must be greater than zero.")

    if args.leaf_inner_metaball_max_points <= 0:
        raise RuntimeError("--leaf-inner-metaball-max-points must be greater than zero.")

    if not (0.0 < args.leaf_inner_metaball_decimate_ratio <= 1.0):
        raise RuntimeError("--leaf-inner-metaball-decimate-ratio must be in (0, 1].")

    if not (0.0 < args.wood_decimate_ratio <= 1.0):
        raise RuntimeError("--wood-decimate-ratio must be in (0, 1].")



def convert(args):
    validate_args(args)

    bpy, mathutils = load_bpy()

    input_path = Path(args.input).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    meshes_dir = output_dir / "meshes"

    if not input_path.exists():
        raise RuntimeError(f"Input file does not exist: {input_path}")

    collision_file_path = (
        Path(args.collision_file).expanduser().resolve()
        if args.collision_file
        else None
    )

    if collision_file_path is not None and not collision_file_path.exists():
        raise RuntimeError(f"Collision file does not exist: {collision_file_path}")

    load_input_file(bpy, input_path)

    collision_collection_raw = collection_objects(bpy, args.collision_collection)
    visual_raw = find_visual_objects(bpy, args, collision_collection_raw)

    if not visual_raw:
        raise RuntimeError("No visual mesh-like objects were found for export.")

    collision_collection_available = bool(collision_collection_raw)
    use_collision_file = collision_file_path is not None
    use_collision_collection = collision_collection_available and not use_collision_file

    branch_segments_raw = []
    if not use_collision_file and args.collision_mode == "thick_branch_cylinders":
        branch_segments_raw = collect_thick_branch_segments(mathutils, visual_raw, args)

    visual_objects = convert_objects_to_meshes(bpy, visual_raw)

    if not visual_objects:
        raise RuntimeError("No visual mesh objects remain after conversion.")

    custom_collision_objects = []
    if use_collision_collection:
        info(f"Using collision collection: {args.collision_collection}")
        custom_collision_objects = convert_objects_to_meshes(
            bpy, collision_collection_raw
        )

        if not custom_collision_objects:
            warn(
                f"Collision collection '{args.collision_collection}' had no exportable meshes. "
                f"Falling back to collision-mode={args.collision_mode}."
            )
            use_collision_collection = False

    export_base_objects = visual_objects + custom_collision_objects

    clear_parents_keep_transform(bpy, export_base_objects)
    apply_uniform_scale(export_base_objects, args.scale)

    origin_vector = compute_origin_vector(bpy, mathutils, args, visual_objects)
    shift_objects(export_base_objects, origin_vector)

    visual_bounds = world_bounds(mathutils, visual_objects)
    branch_segments = transform_branch_segments(
        branch_segments_raw, args.scale, origin_vector
    )

    log_visual_bounds(visual_bounds)

    split_visual_groups = None

    if args.split_visuals:
        leaf_objects, wood_objects, split_source = find_split_visual_groups(
            bpy,
            args,
            visual_objects,
        )

        if not leaf_objects or not wood_objects:
            warn(
                "--split-visuals was requested，but leaf and wood groups could not both be resolved. "
                "Falling back to single tree_visual in model.sdf."
            )
        else:
            split_visual_groups = (leaf_objects, wood_objects, split_source)
            info(
                "Split visual groups resolved: "
                f"leaf={len(leaf_objects)} objects，wood={len(wood_objects)} objects "
                f"({split_source})"
            )

    if use_collision_file:
        collision_source = f"file:{collision_file_path}"
    elif use_collision_collection:
        collision_source = f"collection:{args.collision_collection}"
    else:
        collision_source = args.collision_mode

    copied_textures = []
    segmentation_labels = build_tree_segmentation_labels(args)

    if args.dry_run:
        print_summary(
            args,
            input_path,
            output_dir,
            visual_objects,
            collision_source,
            visual_bounds,
            copied_textures,
            segmentation_labels,
        )
        info("Dry run only. No files were written.")
        return

    meshes_dir.mkdir(parents=True, exist_ok=True)

    if args.copy_textures:
        copied_textures = copy_material_textures(
            bpy, visual_objects, meshes_dir / "textures"
        )

    semantic_parts_path = None

    use_sdf_primitive_collision = (
        args.collision_output_mode == "sdf_primitive"
        and supports_sdf_primitive_collision(args, use_collision_file, use_collision_collection)
    )

    if args.collision_output_mode == "sdf_primitive" and not use_sdf_primitive_collision:
        warn(
            "sdf_primitive collision output is only supported for generated "
            "trunk_cylinder, bounding_box, and bounding_cylinder collisions. "
            "Falling back to mesh_stl for this model."
        )

    collision_desc = (
        make_collision_primitive_description(args, visual_bounds)
        if use_sdf_primitive_collision
        else None
    )
    collision_xml = make_collision_xml_from_description(collision_desc, args.model_name)
    collision_part = make_collision_part_metadata(collision_desc)

    if split_visual_groups is not None:
        leaf_objects, wood_objects, split_source = split_visual_groups

        leaf_export_objects = make_leaf_export_objects(
            bpy, mathutils, leaf_objects, args
        )

        if not leaf_export_objects and args.leaf_lod_mode != "none":
            warn(
                "Leaf export objects are empty. Falling back to original leaf objects."
            )
            leaf_export_objects = leaf_objects

        if leaf_export_objects:
            export_obj_compatible(
                bpy, leaf_export_objects, meshes_dir / args.leaf_output_name
            )

        wood_export_objects = make_wood_export_objects(bpy, wood_objects, args)
        if not wood_export_objects and args.wood_lod_mode != "none":
            warn(
                "Wood export objects are empty. Falling back to original wood objects."
            )
            wood_export_objects = wood_objects

        if wood_export_objects:
            export_obj_compatible(bpy, wood_export_objects, meshes_dir / args.wood_output_name)

        # tree_mesh.objも互換用に出す．LOD leaf + LOD woodの合成にする．
        combined_export_objects = list(wood_export_objects) + list(leaf_export_objects)
        if combined_export_objects:
            export_obj_compatible(
                bpy, combined_export_objects, meshes_dir / "tree_mesh.obj"
            )
        else:
            export_obj_compatible(bpy, visual_objects, meshes_dir / "tree_mesh.obj")

        if args.write_semantic_parts:
            semantic_parts_path = write_semantic_parts(
                output_dir,
                args.model_name,
                args.leaf_output_name,
                args.wood_output_name,
                split_source,
                args.leaf_lod_mode,
                segmentation_labels,
                collision_part,
            )

    else:
        export_obj_compatible(bpy, visual_objects, meshes_dir / "tree_mesh.obj")

    collision_output_path = meshes_dir / "tree_collision.stl"

    if use_sdf_primitive_collision:
        info("Using SDF primitive collision. No tree_collision.stl will be written.")
    elif use_collision_file:
        shutil.copy2(collision_file_path, collision_output_path)
        info(f"Copied collision file: {collision_file_path} -> {collision_output_path}")
    elif use_collision_collection:
        export_stl(bpy, custom_collision_objects, collision_output_path)
    else:
        collision_objects = make_generated_collision(
            bpy,
            mathutils,
            args,
            visual_bounds,
            visual_objects,
            branch_segments,
        )
        export_stl(bpy, collision_objects, collision_output_path)

    write_gazebo_files(
        output_dir,
        args.model_name,
        split_visuals=split_visual_groups is not None,
        leaf_mesh=args.leaf_output_name,
        wood_mesh=args.wood_output_name,
        segmentation_labels=segmentation_labels,
        collision_xml=collision_xml,
    )

    print_summary(
        args,
        input_path,
        output_dir,
        visual_objects,
        collision_source,
        visual_bounds,
        copied_textures,
        segmentation_labels,
    )

    info(f"Wrote: {output_dir / 'model.config'}")
    info(f"Wrote: {output_dir / 'model.sdf'}")
    info(f"Wrote: {meshes_dir / 'tree_mesh.obj'}")

    if split_visual_groups is not None:
        info(f"Wrote: {meshes_dir / args.leaf_output_name}")
        info(f"Wrote: {meshes_dir / args.wood_output_name}")

    if not use_sdf_primitive_collision:
        info(f"Wrote: {meshes_dir / 'tree_collision.stl'}")
    else:
        info("Wrote: SDF primitive tree_collision in model.sdf")

    if semantic_parts_path is not None:
        info(f"Wrote: {semantic_parts_path}")


def main():
    args = parse_args(blender_argv())

    try:
        convert(args)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None


if __name__ == "__main__":
    main()
