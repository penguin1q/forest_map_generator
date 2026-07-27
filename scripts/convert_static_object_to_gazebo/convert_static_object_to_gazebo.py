#!/usr/bin/env python3
import argparse
import json
import shutil
import sys
from pathlib import Path

MODEL_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="object_link">
      <visual name="object_visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{visual_mesh}</uri>
          </mesh>
        </geometry>
        <cast_shadows>false</cast_shadows>
{visual_segmentation_plugin_xml}      </visual>
{collision_xml}    </link>
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
    Converted static object asset for Gazebo simulation.
  </description>
</model>
"""

SUPPORTED_CONVERTIBLE_TYPES = {"MESH", "CURVE", "SURFACE"}
IGNORED_TYPES = {"CAMERA", "LIGHT"}

CANONICAL_SEMANTIC_LABELS = {
    "background": 0,
    # vegetation / tree (reserved but handled by tree converter)
    "leaf": 1,
    "wood": 2,
    "fruit": 3,
    # terrain / static objects
    "ground": 20,
    "rock": 21,
    "pole": 22,
    "fence": 23,
    "basket": 24,
    "crate": 25,
    "wall": 26,
    "container": 27,
    "sign": 28,
    "obstacle": 29,
    # dynamic / safety critical
    "person": 50,
    "vehicle": 51,
    "animal": 52,
    # robot / ignore
    "robot": 90,
    "ignore": 255,
}


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Convert a static obstacle asset into a Gazebo model directory with "
            "semantic metadata and primitive or mesh collision."
        )
    )

    parser.add_argument("--input", required=True, help="Input model file，preferably .blend")
    parser.add_argument("--model-name", required=True, help="Gazebo model name")
    parser.add_argument("--output-dir", required=True, help="Output Gazebo model directory")
    parser.add_argument("--visual-format", choices=("obj",), default="obj")
    parser.add_argument("--visual-output-name", default="object_visual.obj")

    parser.add_argument(
        "--object-class",
        default="obstacle",
        help=(
            "Semantic class name for the object visual/collision. Examples: rock, basket, crate, pole, fence. "
            "If --segmentation-label auto is used, known classes receive canonical label IDs."
        ),
    )
    parser.add_argument(
        "--contact-policy",
        choices=("AVOID", "ALLOW_CONTACT", "IGNORE"),
        default="AVOID",
        help="Project-level contact policy metadata written to semantic_parts.json.",
    )

    parser.add_argument(
        "--collision-mode",
        choices=(
            "bounding_box",
            "bounding_cylinder",
            "bounding_sphere",
            "convex_hull",
            "visual_decimated",
            "none",
        ),
        default="bounding_box",
    )
    parser.add_argument(
        "--collision-output-mode",
        choices=("auto", "mesh_stl", "sdf_primitive"),
        default="auto",
        help=(
            "How to represent collision in model.sdf. auto uses sdf_primitive for primitive modes "
            "and mesh_stl for mesh-based modes."
        ),
    )

    parser.add_argument(
        "--origin-mode",
        choices=("bottom_center", "keep", "cursor", "named_empty"),
        default="bottom_center",
    )
    parser.add_argument("--origin-empty-name", default="OBJECT_ORIGIN")
    parser.add_argument("--scale", type=float, default=1.0)

    parser.add_argument(
        "--visual-decimate-ratio",
        type=float,
        default=1.0,
        help="Optional decimation ratio for the exported visual mesh. 1.0 keeps the original visual mesh.",
    )
    parser.add_argument(
        "--visual-join-before-export",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Join duplicated/decimated visual mesh objects before OBJ export.",
    )
    parser.add_argument(
        "--visual-shade-smooth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Apply smooth shading to duplicated/decimated visual mesh objects before export.",
    )

    parser.add_argument(
        "--collision-padding",
        type=float,
        default=0.0,
        help="Extra padding added uniformly to primitive collision sizes [m].",
    )
    parser.add_argument(
        "--collision-hull-max-points",
        type=int,
        default=120,
        help="Maximum sampled vertices used for convex hull collision generation.",
    )
    parser.add_argument(
        "--collision-hull-decimate-ratio",
        type=float,
        default=0.75,
        help="Decimate ratio applied to the convex hull collision mesh. 1.0 means no reduction.",
    )
    parser.add_argument(
        "--collision-visual-decimate-ratio",
        type=float,
        default=0.25,
        help="Decimate ratio used when collision-mode=visual_decimated.",
    )
    parser.add_argument(
        "--collision-collection",
        default="collision",
        help="Optional explicit collision collection name. If found, its meshes override generated collision.",
    )
    parser.add_argument(
        "--collision-file",
        help="Optional existing STL file to copy as meshes/object_collision.stl. Overrides generated collision.",
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
        help="Write Gazebo SegmentationCamera <label> plugin XML to the visual if a label is resolved.",
    )
    parser.add_argument(
        "--segmentation-label",
        default="auto",
        help=(
            "Segmentation label ID for object_visual. Use 'auto' to resolve from --object-class, 'none' to disable, "
            "or an integer in [0, 255]."
        ),
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
            "blender --background --python convert_static_object_to_gazebo.py -- <args>"
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
        raise RuntimeError("OBJ import operator is not available in this Blender version.")

    if suffix == ".fbx":
        if hasattr(bpy.ops.import_scene, "fbx"):
            bpy.ops.import_scene.fbx(filepath=str(input_path))
            return
        raise RuntimeError("FBX import operator is not available in this Blender version.")

    if suffix in {".glb", ".gltf"}:
        if hasattr(bpy.ops.import_scene, "gltf"):
            bpy.ops.import_scene.gltf(filepath=str(input_path))
            return
        raise RuntimeError("glTF import operator is not available in this Blender version.")

    raise RuntimeError(f"Unsupported input file extension: {suffix}")


def collection_objects(bpy, collection_name):
    collection = bpy.data.collections.get(collection_name)
    if collection is None:
        return None
    return list(collection.all_objects)


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
                    f"Object {obj.name} has modifier {modifier.name} ({modifier.type}). Check the exported mesh if needed."
                )
        if getattr(obj, "particle_systems", None) and len(obj.particle_systems) > 0:
            warn(f"Object {obj.name} has particle systems. Convert particles to mesh if needed.")
        if getattr(obj, "instance_type", "NONE") != "NONE":
            warn(
                f"Object {obj.name} uses instancing ({obj.instance_type}). Realize instances if needed."
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
        warn(f"Object {obj.name} is type {obj.type} and cannot be converted to mesh safely.")
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


def find_visual_objects(bpy, collision_collection_raw):
    collision_names = {obj.name for obj in (collision_collection_raw or [])}
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


def bounds_center(bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def bounds_size(bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    return max_x - min_x, max_y - min_y, max_z - min_z


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
    center_x = (bounds[0] + bounds[1]) * 0.5
    center_y = (bounds[2] + bounds[3]) * 0.5
    min_z = bounds[4]
    return mathutils.Vector((center_x, center_y, min_z))


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


def shift_objects(objects, origin_vector):
    if origin_vector.length <= 1e-12:
        return
    for obj in objects:
        obj.location.x -= origin_vector.x
        obj.location.y -= origin_vector.y
        obj.location.z -= origin_vector.z


def log_visual_bounds(bounds):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    size_x, size_y, size_z = bounds_size(bounds)
    info("Visual bounds after origin adjustment:")
    info(f"  min = ({min_x:.3f}, {min_y:.3f}, {min_z:.3f})")
    info(f"  max = ({max_x:.3f}, {max_y:.3f}, {max_z:.3f})")
    info(f"  size = ({size_x:.3f}, {size_y:.3f}, {size_z:.3f})")


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
        raise RuntimeError("Blender OBJ export operator bpy.ops.wm.obj_export is not available.")
    operator = bpy.ops.wm.obj_export
    properties = operator_property_names(operator)
    kwargs = {"filepath": str(path)}
    if not properties or "export_selected_objects" in properties:
        kwargs["export_selected_objects"] = True
    if not properties or "export_materials" in properties:
        kwargs["export_materials"] = True
    if not properties or "path_mode" in properties:
        kwargs["path_mode"] = "RELATIVE"
    if "forward_axis" in properties and "up_axis" in properties:
        kwargs["forward_axis"] = "Y"
        kwargs["up_axis"] = "Z"
    elif "axis_forward" in properties and "axis_up" in properties:
        kwargs["axis_forward"] = "Y"
        kwargs["axis_up"] = "Z"
    else:
        warn("OBJ export axis options were unavailable. Exporting with operator defaults.")
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


def duplicate_mesh_objects(bpy, objects, name_prefix):
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


def make_visual_export_objects(bpy, visual_objects, args):
    if args.visual_decimate_ratio >= 0.999:
        return visual_objects
    original_faces = mesh_face_count(visual_objects)
    duplicated = duplicate_mesh_objects(bpy, visual_objects, "visual_lod")
    if not duplicated:
        warn("No visual mesh objects were available for decimation. Falling back to original visual meshes.")
        return visual_objects
    for obj in duplicated:
        try:
            bpy.ops.object.select_all(action="DESELECT")
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            modifier = obj.modifiers.new("visual_decimate", "DECIMATE")
            modifier.ratio = max(0.05, min(1.0, args.visual_decimate_ratio))
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate visual object {obj.name}: {exc}")
        for poly in obj.data.polygons:
            poly.use_smooth = bool(args.visual_shade_smooth)
    if args.visual_join_before_export and len(duplicated) > 1:
        joined = join_objects_to_single_mesh(bpy, duplicated, "object_visual_joined")
        if joined:
            duplicated = joined
    info(
        "Generated visual LOD: faces "
        f"{original_faces} -> {mesh_face_count(duplicated)} "
        f"(ratio target={args.visual_decimate_ratio:.3f})."
    )
    return duplicated


def create_bounding_box_collision(bpy, bounds, padding):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    size_x = max(0.001, (max_x - min_x) + 2.0 * padding)
    size_y = max(0.001, (max_y - min_y) + 2.0 * padding)
    size_z = max(0.001, (max_z - min_z) + 2.0 * padding)
    center_x, center_y, center_z = bounds_center(bounds)
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(center_x, center_y, center_z))
    obj = bpy.context.object
    obj.name = "object_collision_bounding_box"
    obj.dimensions = (size_x, size_y, size_z)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    geometry = {
        "type": "box",
        "size": [size_x, size_y, size_z],
        "pose": [center_x, center_y, center_z, 0.0, 0.0, 0.0],
    }
    return [obj], geometry


def create_bounding_cylinder_collision(bpy, bounds, padding):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    center_x, center_y, center_z = bounds_center(bounds)
    radius = max(max_x - min_x, max_y - min_y) * 0.5 + padding
    height = max(0.001, (max_z - min_z) + 2.0 * padding)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=24,
        radius=max(radius, 0.001),
        depth=height,
        location=(center_x, center_y, center_z),
    )
    obj = bpy.context.object
    obj.name = "object_collision_bounding_cylinder"
    geometry = {
        "type": "cylinder",
        "radius": max(radius, 0.001),
        "length": height,
        "pose": [center_x, center_y, center_z, 0.0, 0.0, 0.0],
    }
    return [obj], geometry


def create_bounding_sphere_collision(bpy, bounds, padding):
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    center_x, center_y, center_z = bounds_center(bounds)
    size_x, size_y, size_z = bounds_size(bounds)
    radius = max(size_x, size_y, size_z) * 0.5 + padding
    bpy.ops.mesh.primitive_uv_sphere_add(radius=max(radius, 0.001), location=(center_x, center_y, center_z))
    obj = bpy.context.object
    obj.name = "object_collision_bounding_sphere"
    geometry = {
        "type": "sphere",
        "radius": max(radius, 0.001),
        "pose": [center_x, center_y, center_z, 0.0, 0.0, 0.0],
    }
    return [obj], geometry


def quantized_key(p, eps=1e-5):
    return (round(p.x / eps), round(p.y / eps), round(p.z / eps))


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


def sample_all_vertices_world(mathutils, objects, max_total_points=120):
    points = []
    for obj in objects:
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


def create_convex_hull_from_points(bpy, mathutils, points, name, decimate_ratio):
    import bmesh
    points = unique_points(points)
    if len(points) < 4:
        warn(f"Convex hull {name} has only {len(points)} unique points. Skipping.")
        return None
    mesh = bpy.data.meshes.new(name + "_mesh")
    bm = bmesh.new()
    bm_verts = [bm.verts.new((p.x, p.y, p.z)) for p in points]
    bm.verts.ensure_lookup_table()
    try:
        result = bmesh.ops.convex_hull(bm, input=bm_verts)
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
    for poly in obj.data.polygons:
        poly.use_smooth = False
    if decimate_ratio < 0.999:
        try:
            bpy.ops.object.select_all(action="DESELECT")
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            modifier = obj.modifiers.new("collision_hull_decimate", "DECIMATE")
            modifier.ratio = max(0.05, min(1.0, decimate_ratio))
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate convex hull {name}: {exc}")
    return obj


def create_convex_hull_collision(bpy, mathutils, visual_objects, args):
    points = sample_all_vertices_world(
        mathutils, visual_objects, max_total_points=args.collision_hull_max_points
    )
    if not points:
        raise RuntimeError("No vertices were available for convex hull collision generation.")
    hull = create_convex_hull_from_points(
        bpy, mathutils, points, "object_collision_convex_hull", args.collision_hull_decimate_ratio
    )
    if hull is None:
        raise RuntimeError("Convex hull collision generation failed.")
    geometry = {
        "type": "mesh",
        "mesh": "meshes/object_collision.stl",
        "mode": "convex_hull",
    }
    return [hull], geometry


def duplicate_decimated_visual_collision(bpy, visual_objects, ratio):
    selected = select_only_meshes(bpy, visual_objects)
    if not selected:
        raise RuntimeError("No visual mesh objects available for visual_decimated collision.")
    bpy.ops.object.duplicate()
    duplicated = list(bpy.context.selected_objects)
    for obj in duplicated:
        obj.name = f"object_collision_{obj.name}"
        try:
            modifier = obj.modifiers.new("collision_decimate", "DECIMATE")
            modifier.ratio = max(0.05, min(1.0, ratio))
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
            bpy.ops.object.modifier_apply(modifier=modifier.name)
        except Exception as exc:
            warn(f"Could not decimate collision object {obj.name}: {exc}")
    geometry = {
        "type": "mesh",
        "mesh": "meshes/object_collision.stl",
        "mode": "visual_decimated",
    }
    return duplicated, geometry


def resolve_collision_output_mode(args, using_explicit_mesh_source=False):
    if args.collision_mode == "none":
        return "none"
    if using_explicit_mesh_source:
        return "mesh_stl"
    if args.collision_output_mode != "auto":
        if args.collision_output_mode == "sdf_primitive" and args.collision_mode in {"convex_hull", "visual_decimated"}:
            warn(
                f"collision-output-mode=sdf_primitive is not supported for collision-mode={args.collision_mode}. "
                "Falling back to mesh_stl."
            )
            return "mesh_stl"
        return args.collision_output_mode
    if args.collision_mode in {"bounding_box", "bounding_cylinder", "bounding_sphere"}:
        return "sdf_primitive"
    return "mesh_stl"


def generate_collision(bpy, mathutils, visual_bounds, visual_objects, args, explicit_collision_objects=None):
    if args.collision_mode == "none":
        return [], None
    if explicit_collision_objects:
        geometry = {
            "type": "mesh",
            "mesh": "meshes/object_collision.stl",
            "mode": "explicit_collection",
        }
        return explicit_collision_objects, geometry
    if args.collision_mode == "bounding_box":
        return create_bounding_box_collision(bpy, visual_bounds, args.collision_padding)
    if args.collision_mode == "bounding_cylinder":
        return create_bounding_cylinder_collision(bpy, visual_bounds, args.collision_padding)
    if args.collision_mode == "bounding_sphere":
        return create_bounding_sphere_collision(bpy, visual_bounds, args.collision_padding)
    if args.collision_mode == "convex_hull":
        return create_convex_hull_collision(bpy, mathutils, visual_objects, args)
    if args.collision_mode == "visual_decimated":
        warn("visual_decimated collision can be expensive for Gazebo.")
        return duplicate_decimated_visual_collision(
            bpy, visual_objects, args.collision_visual_decimate_ratio
        )
    raise RuntimeError(f"Unsupported collision mode: {args.collision_mode}")


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
                    warn(f"Image texture '{image.name}' is packed. Unpack it if Gazebo needs the texture.")
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


def resolve_segmentation_label(value, object_class):
    if value is None:
        return None
    text = str(value).strip().lower()
    if text == "none":
        return None
    if text == "auto":
        if object_class in CANONICAL_SEMANTIC_LABELS:
            return CANONICAL_SEMANTIC_LABELS[object_class]
        warn(
            f"No canonical segmentation label is defined for object-class='{object_class}'. "
            "Segmentation labels will be omitted."
        )
        return None
    try:
        label = int(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"Segmentation label must be 'auto', 'none', or an integer: {value!r}") from exc
    if label < 0 or label > 255:
        raise RuntimeError(f"Segmentation label must be in [0, 255]: {label}")
    return label


def make_segmentation_plugin_xml(label):
    if label is None:
        return ""
    return (
        "        <plugin filename=\"gz-sim-label-system\" "
        "name=\"gz::sim::systems::Label\">\n"
        f"          <label>{int(label)}</label>\n"
        "        </plugin>\n"
    )


def primitive_geometry_to_collision_xml(geometry):
    pose = geometry.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pose_text = " ".join(f"{float(v):.9g}" for v in pose)
    gtype = geometry["type"]
    if gtype == "box":
        size = geometry["size"]
        size_text = " ".join(f"{float(v):.9g}" for v in size)
        body = f"""      <collision name=\"object_collision\">\n        <pose>{pose_text}</pose>\n        <geometry>\n          <box>\n            <size>{size_text}</size>\n          </box>\n        </geometry>\n      </collision>\n"""
        return body
    if gtype == "cylinder":
        body = f"""      <collision name=\"object_collision\">\n        <pose>{pose_text}</pose>\n        <geometry>\n          <cylinder>\n            <radius>{float(geometry['radius']):.9g}</radius>\n            <length>{float(geometry['length']):.9g}</length>\n          </cylinder>\n        </geometry>\n      </collision>\n"""
        return body
    if gtype == "sphere":
        body = f"""      <collision name=\"object_collision\">\n        <pose>{pose_text}</pose>\n        <geometry>\n          <sphere>\n            <radius>{float(geometry['radius']):.9g}</radius>\n          </sphere>\n        </geometry>\n      </collision>\n"""
        return body
    raise RuntimeError(f"Unsupported primitive geometry for SDF collision XML: {gtype}")


def mesh_collision_xml(model_name):
    return f"""      <collision name=\"object_collision\">\n        <geometry>\n          <mesh>\n            <uri>model://{model_name}/meshes/object_collision.stl</uri>\n          </mesh>\n        </geometry>\n      </collision>\n"""


def write_semantic_parts(output_dir, model_name, visual_mesh, object_class, contact_policy, segmentation_label, collision_representation):
    parts = [
        {
            "name": "object_visual",
            "role": "visual",
            "class": object_class,
            "contact_policy": contact_policy,
            "mesh": f"meshes/{visual_mesh}",
            "gazebo_visual": "object_link::object_visual",
            "segmentation_label": segmentation_label,
        }
    ]
    if collision_representation is not None:
        collision_part = {
            "name": "object_collision",
            "role": "collision",
            "class": object_class,
            "contact_policy": contact_policy,
            "gazebo_collision": "object_link::object_collision",
            "segmentation_label": None,
        }
        if collision_representation.get("type") == "mesh":
            collision_part["geometry"] = dict(collision_representation)
        else:
            collision_part["geometry"] = dict(collision_representation)
        parts.append(collision_part)
    data = {
        "model": model_name,
        "object_type": "static_object",
        "visual_mode": "single_mesh",
        "parts": parts,
    }
    path = output_dir / "semantic_parts.json"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def write_gazebo_files(output_dir, model_name, visual_mesh, collision_xml, visual_segmentation_plugin_xml):
    sdf_text = MODEL_SDF_TEMPLATE.format(
        model_name=model_name,
        visual_mesh=visual_mesh,
        collision_xml=collision_xml,
        visual_segmentation_plugin_xml=visual_segmentation_plugin_xml,
    )
    (output_dir / "model.sdf").write_text(sdf_text, encoding="utf-8")
    (output_dir / "model.config").write_text(
        MODEL_CONFIG_TEMPLATE.format(model_name=model_name),
        encoding="utf-8",
    )


def print_summary(args, input_path, output_dir, visual_objects, visual_bounds, collision_summary, copied_textures, segmentation_label, semantic_parts_path):
    info("Conversion summary:")
    info(f"  input: {input_path}")
    info(f"  model_name: {args.model_name}")
    info(f"  output_dir: {output_dir}")
    info(f"  object_class: {args.object_class}")
    info(f"  contact_policy: {args.contact_policy}")
    info(f"  collision_mode: {args.collision_mode}")
    info(f"  collision_output_mode: {args.collision_output_mode}")
    info(f"  visual_objects: {', '.join(obj.name for obj in visual_objects)}")
    info(f"  segmentation_label: {segmentation_label if segmentation_label is not None else 'disabled'}")
    info(f"  collision_summary: {collision_summary}")
    info(
        "  visual_bounds_after_origin: "
        f"x=[{visual_bounds[0]:.3f}, {visual_bounds[1]:.3f}], "
        f"y=[{visual_bounds[2]:.3f}, {visual_bounds[3]:.3f}], "
        f"z=[{visual_bounds[4]:.3f}, {visual_bounds[5]:.3f}]"
    )
    if semantic_parts_path is not None:
        info(f"  semantic_parts: {semantic_parts_path}")
    if copied_textures:
        info("  copied_textures:")
        for path in copied_textures:
            info(f"    {path}")


def validate_args(args):
    if args.scale <= 0.0:
        raise RuntimeError("--scale must be greater than zero")
    if not (0.0 < args.visual_decimate_ratio <= 1.0):
        raise RuntimeError("--visual-decimate-ratio must be in (0, 1].")
    if args.collision_padding < 0.0:
        raise RuntimeError("--collision-padding must be zero or greater.")
    if args.collision_hull_max_points <= 0:
        raise RuntimeError("--collision-hull-max-points must be greater than zero.")
    if not (0.0 < args.collision_hull_decimate_ratio <= 1.0):
        raise RuntimeError("--collision-hull-decimate-ratio must be in (0, 1].")
    if not (0.0 < args.collision_visual_decimate_ratio <= 1.0):
        raise RuntimeError("--collision-visual-decimate-ratio must be in (0, 1].")
    resolve_segmentation_label(args.segmentation_label, args.object_class)


def convert(args):
    validate_args(args)
    bpy, mathutils = load_bpy()
    input_path = Path(args.input).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    meshes_dir = output_dir / "meshes"
    if not input_path.exists():
        raise RuntimeError(f"Input file does not exist: {input_path}")
    collision_file_path = Path(args.collision_file).expanduser().resolve() if args.collision_file else None
    if collision_file_path is not None and not collision_file_path.exists():
        raise RuntimeError(f"Collision file does not exist: {collision_file_path}")

    load_input_file(bpy, input_path)

    collision_collection_raw = collection_objects(bpy, args.collision_collection)
    visual_raw = find_visual_objects(bpy, collision_collection_raw)
    if not visual_raw:
        raise RuntimeError("No visual mesh-like objects were found for export.")

    visual_objects = convert_objects_to_meshes(bpy, visual_raw)
    if not visual_objects:
        raise RuntimeError("No visual mesh objects remain after conversion.")

    explicit_collision_objects = []
    if collision_collection_raw:
        explicit_collision_objects = convert_objects_to_meshes(bpy, collision_collection_raw)
        if explicit_collision_objects:
            info(f"Using explicit collision collection: {args.collision_collection}")

    export_base_objects = list(visual_objects) + list(explicit_collision_objects)
    clear_parents_keep_transform(bpy, export_base_objects)
    apply_uniform_scale(export_base_objects, args.scale)
    origin_vector = compute_origin_vector(bpy, mathutils, args, visual_objects)
    shift_objects(export_base_objects, origin_vector)
    visual_bounds = world_bounds(mathutils, visual_objects)
    log_visual_bounds(visual_bounds)

    segmentation_label = resolve_segmentation_label(args.segmentation_label, args.object_class)
    if not args.write_segmentation_labels:
        segmentation_label = None
    visual_segmentation_plugin_xml = make_segmentation_plugin_xml(segmentation_label)

    collision_source_for_mode = bool(collision_file_path) or bool(explicit_collision_objects)
    effective_collision_output_mode = resolve_collision_output_mode(args, using_explicit_mesh_source=collision_source_for_mode)

    if args.dry_run:
        copied_textures = []
        semantic_parts_path = None
        collision_summary = f"mode={args.collision_mode}, output={effective_collision_output_mode}, dry-run"
        print_summary(
            args,
            input_path,
            output_dir,
            visual_objects,
            visual_bounds,
            collision_summary,
            copied_textures,
            segmentation_label,
            semantic_parts_path,
        )
        info("Dry run only. No files were written.")
        return

    meshes_dir.mkdir(parents=True, exist_ok=True)

    copied_textures = []
    if args.copy_textures:
        copied_textures = copy_material_textures(bpy, visual_objects, meshes_dir / "textures")

    visual_export_objects = make_visual_export_objects(bpy, visual_objects, args)
    if not visual_export_objects:
        visual_export_objects = visual_objects
    export_obj_compatible(bpy, visual_export_objects, meshes_dir / args.visual_output_name)

    collision_representation = None
    collision_xml = ""
    collision_summary = "none"

    if args.collision_mode != "none":
        if collision_file_path is not None:
            shutil.copy2(collision_file_path, meshes_dir / "object_collision.stl")
            collision_representation = {
                "type": "mesh",
                "mesh": "meshes/object_collision.stl",
                "mode": "explicit_file",
            }
            collision_xml = mesh_collision_xml(args.model_name)
            collision_summary = f"explicit_file -> meshes/object_collision.stl"
        else:
            collision_objects, generated_geometry = generate_collision(
                bpy,
                mathutils,
                visual_bounds,
                visual_objects,
                args,
                explicit_collision_objects=explicit_collision_objects,
            )
            if generated_geometry is not None:
                if effective_collision_output_mode == "sdf_primitive" and generated_geometry["type"] in {"box", "cylinder", "sphere"}:
                    collision_representation = generated_geometry
                    collision_xml = primitive_geometry_to_collision_xml(generated_geometry)
                    collision_summary = f"primitive {generated_geometry['type']}"
                else:
                    export_stl(bpy, collision_objects, meshes_dir / "object_collision.stl")
                    collision_representation = dict(generated_geometry)
                    collision_representation["mesh"] = "meshes/object_collision.stl"
                    collision_representation["type"] = "mesh"
                    collision_xml = mesh_collision_xml(args.model_name)
                    collision_summary = f"mesh_stl ({generated_geometry.get('mode', args.collision_mode)})"

    semantic_parts_path = None
    if args.write_semantic_parts:
        semantic_parts_path = write_semantic_parts(
            output_dir,
            args.model_name,
            args.visual_output_name,
            args.object_class,
            args.contact_policy,
            segmentation_label,
            collision_representation,
        )

    write_gazebo_files(
        output_dir,
        args.model_name,
        args.visual_output_name,
        collision_xml,
        visual_segmentation_plugin_xml,
    )

    print_summary(
        args,
        input_path,
        output_dir,
        visual_objects,
        visual_bounds,
        collision_summary,
        copied_textures,
        segmentation_label,
        semantic_parts_path,
    )
    info(f"Wrote: {output_dir / 'model.config'}")
    info(f"Wrote: {output_dir / 'model.sdf'}")
    info(f"Wrote: {meshes_dir / args.visual_output_name}")
    if collision_representation is not None and collision_representation.get("type") == "mesh":
        info(f"Wrote: {meshes_dir / 'object_collision.stl'}")
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
