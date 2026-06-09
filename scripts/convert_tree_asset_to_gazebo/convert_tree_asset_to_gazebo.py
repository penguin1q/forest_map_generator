#!/usr/bin/env python3
import argparse
import os
import shutil
import sys
from pathlib import Path


MODEL_SDF_TEMPLATE = '''<?xml version="1.0" ?>
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
      </visual>

      <collision name="tree_collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/tree_collision.stl</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
'''

MODEL_CONFIG_TEMPLATE = '''<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>forest_map_generator</name>
    <email>unknown@example.com</email>
  </author>
  <description>
    Private converted tree asset for orchard simulation.
  </description>
</model>
'''

SUPPORTED_CONVERTIBLE_TYPES = {"MESH", "CURVE", "SURFACE"}
IGNORED_TYPES = {"CAMERA", "LIGHT"}


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Convert a prepared Blender tree asset into a Gazebo model directory."
    )
    parser.add_argument("--input", required=True, help="Prepared input model file, preferably .blend")
    parser.add_argument("--model-name", required=True, help="Gazebo model name")
    parser.add_argument("--output-dir", required=True, help="Output Gazebo model directory")
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
        "--origin-mode",
        choices=("bottom_center", "keep", "cursor", "named_empty"),
        default="bottom_center",
    )
    parser.add_argument("--origin-empty-name", default="TREE_ORIGIN")
    parser.add_argument("--visual-collection", default="visual")
    parser.add_argument("--collision-collection", default="collision")
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
        help="Include low thick main-branch curve segments for thick_branch_cylinders.",
    )
    parser.add_argument(
        "--collision-add-trunk-cylinder",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also add the fixed upright trunk cylinder to thick_branch_cylinders collision.",
    )
    parser.add_argument("--collision-file", help="Existing STL file to copy as meshes/tree_collision.stl")
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
            "This converter must be run with Blender, e.g. "
            "blender --background --python convert_tree_asset_to_gazebo.py -- <args>"
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

    raise RuntimeError(
        f"Unsupported input file extension '{suffix}'. Use a prepared .blend file."
    )


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


def report_preparation_warnings(objects):
    for obj in objects:
        for modifier in getattr(obj, "modifiers", []):
            if modifier.type == "NODES":
                warn(
                    f"Object {obj.name} has Geometry Nodes. Realize instances manually "
                    "if the exported mesh is incomplete."
                )
            else:
                warn(
                    f"Object {obj.name} has modifier {modifier.name} ({modifier.type}). "
                    "Check the exported mesh if the modifier is not applied as expected."
                )
        if getattr(obj, "particle_systems", None) and len(obj.particle_systems) > 0:
            warn(
                f"Object {obj.name} has particle systems. Particle instances may need "
                "to be converted to real mesh objects before conversion."
            )
        if getattr(obj, "instance_type", "NONE") != "NONE":
            warn(
                f"Object {obj.name} uses instancing ({obj.instance_type}). Instances may "
                "need to be realized manually before conversion."
            )


def select_only(bpy, objects):
    bpy.ops.object.select_all(action="DESELECT")
    mesh_objects = [obj for obj in objects if obj.type == "MESH"]
    for obj in mesh_objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objects[0] if mesh_objects else None
    return mesh_objects


def clear_parents_keep_transform(bpy, objects):
    selected = select_any(bpy, objects)
    if selected:
        bpy.ops.object.parent_clear(type="CLEAR_KEEP_TRANSFORM")


def select_any(bpy, objects):
    bpy.ops.object.select_all(action="DESELECT")
    selected = []
    for obj in objects:
        obj.select_set(True)
        selected.append(obj)
    bpy.context.view_layer.objects.active = selected[0] if selected else None
    return selected


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
    except Exception as exc:  # Blender operators raise RuntimeError subclasses.
        warn(f"Object {obj.name} could not be converted to mesh: {exc}")
        return None

    converted = bpy.context.object
    if converted.type != "MESH":
        warn(f"Object {obj.name} is not a mesh after conversion. It may not be exported.")
        return None
    return converted


def convert_objects_to_meshes(bpy, objects):
    meshes = []
    report_preparation_warnings(objects)
    for obj in objects:
        converted = convert_object_to_mesh(bpy, obj)
        if converted is not None and converted.type == "MESH":
            meshes.append(converted)
        elif converted is not None:
            warn(f"Object {converted.name} is not a mesh after conversion. It may not be exported.")
    return unique_objects(meshes)


def unique_objects(objects):
    seen = set()
    result = []
    for obj in objects:
        if obj.name not in seen:
            seen.add(obj.name)
            result.append(obj)
    return result


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

    ignored = [obj for obj in bpy.context.scene.objects if obj.type not in SUPPORTED_CONVERTIBLE_TYPES | IGNORED_TYPES]
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
        raise RuntimeError("No mesh bounds could be computed from selected visual objects.")

    min_x = min(point.x for point in points)
    max_x = max(point.x for point in points)
    min_y = min(point.y for point in points)
    max_y = max(point.y for point in points)
    min_z = min(point.z for point in points)
    max_z = max(point.z for point in points)
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
    info("Visual bounds before export:")
    info(f"  min = ({min_x:.3f}, {min_y:.3f}, {min_z:.3f})")
    info(f"  max = ({max_x:.3f}, {max_y:.3f}, {max_z:.3f})")
    info(f"  size = ({size_x:.3f}, {size_y:.3f}, {size_z:.3f})")
    info(f"  height_z = {size_z:.3f}")
    if size_z < max(size_x, size_y):
        warn("visual model height is not along Z. The input Blender model may already be rotated.")


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


def branch_name_kind(obj):
    name = obj.name.lower()
    if any(token in name for token in BRANCH_EXCLUDE_TOKENS):
        return "excluded"
    if any(token in name for token in BRANCH_INCLUDE_TOKENS):
        return "included"
    return "unknown"


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
            points.append(matrix @ mathutils.Vector((co.x / weight, co.y / weight, co.z / weight)))
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
                    f"Branch-like object {obj.name} is a mesh, not a Curve. "
                    "thick_branch_cylinders cannot infer coarse branch segments from it."
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
                warn(
                    f"Curve object {obj.name} has no usable bevel/radius information; "
                    "skipping it for thick_branch_cylinders."
                )
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
            info(f"Collected {obj_segments} thick branch collision segments from {obj.name}")

    if not segments:
        warn("No usable thick branch curve segments found; falling back to trunk_cylinder collision.")
    return segments


def transform_branch_segments(branch_segments, scale, origin_vector):
    transformed = []
    for p0, p1, radius, source_name in branch_segments:
        transformed.append((p0 * scale - origin_vector, p1 * scale - origin_vector, radius * scale, source_name))
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
                f"origin-mode=named_empty was requested, but Empty '{args.origin_empty_name}' was not found."
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
                        f"Image texture '{image.name}' is packed in the .blend. "
                        "Save/unpack it to an external file if Gazebo needs the texture."
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
    selected = select_only(bpy, objects)
    if not selected:
        raise RuntimeError("No visual mesh objects selected for OBJ export.")
    if not hasattr(bpy.ops.wm, "obj_export"):
        raise RuntimeError(
            "Blender OBJ export operator bpy.ops.wm.obj_export is not available. "
            "Use Blender 4.x/5.x with OBJ export support enabled."
        )

    operator = bpy.ops.wm.obj_export
    properties = operator_property_names(operator)
    kwargs = {"filepath": str(path)}
    if not properties or "export_selected_objects" in properties:
        kwargs["export_selected_objects"] = True

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
            "OBJ export axis options were not available from bpy.ops.wm.obj_export. "
            "Exporting without explicit axis conversion; check the model orientation in Gazebo."
        )
        axis_settings.append("axis=operator_default")

    info("OBJ export axis settings used: " + ", ".join(axis_settings))
    try:
        operator(**kwargs)
    except TypeError as exc:
        raise RuntimeError(f"OBJ export failed with selected compatibility arguments: {exc}") from exc


def export_stl(bpy, objects, path):
    selected = select_only(bpy, objects)
    if not selected:
        raise RuntimeError("No collision mesh objects selected for STL export.")
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=str(path), export_selected_objects=True)
    elif hasattr(bpy.ops, "export_mesh") and hasattr(bpy.ops.export_mesh, "stl"):
        bpy.ops.export_mesh.stl(filepath=str(path), use_selection=True)
    else:
        raise RuntimeError("STL export operator is not available in this Blender version.")


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
        warn("No curve-derived branch collision segments found; falling back to trunk_cylinder.")
        return create_trunk_collision(bpy, args)

    collision_objects = []
    if args.collision_add_trunk_cylinder:
        info("Adding fixed trunk cylinder in addition to curve-derived branch collision.")
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
        warn("No thick branch cylinders were generated; falling back to trunk_cylinder.")
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
    selected = select_only(bpy, visual_objects)
    if not selected:
        raise RuntimeError("No visual mesh objects available for visual_decimated_optional collision.")
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


def make_generated_collision(bpy, mathutils, args, visual_bounds, visual_objects, branch_segments):
    if args.collision_mode == "trunk_cylinder":
        return create_trunk_collision(bpy, args)
    if args.collision_mode == "thick_branch_cylinders":
        return create_thick_branch_collision(bpy, mathutils, args, branch_segments)
    if args.collision_mode == "bounding_box":
        return create_bounding_box_collision(bpy, visual_bounds)
    if args.collision_mode == "bounding_cylinder":
        return create_bounding_cylinder_collision(bpy, visual_bounds)
    if args.collision_mode == "visual_decimated_optional":
        warn(
            "visual_decimated_optional can be expensive for Gazebo. Prefer a custom "
            "collision collection, collision-file, or trunk_cylinder for orchard navigation."
        )
        return duplicate_decimated_visual_collision(bpy, visual_objects)
    raise RuntimeError(f"Unsupported collision mode: {args.collision_mode}")


def write_gazebo_files(output_dir, model_name):
    (output_dir / "model.sdf").write_text(
        MODEL_SDF_TEMPLATE.format(model_name=model_name), encoding="utf-8"
    )
    (output_dir / "model.config").write_text(
        MODEL_CONFIG_TEMPLATE.format(model_name=model_name), encoding="utf-8"
    )


def print_summary(args, input_path, output_dir, visual_objects, collision_source, bounds, copied):
    info("Conversion summary:")
    info(f"  input: {input_path}")
    info(f"  model_name: {args.model_name}")
    info(f"  output_dir: {output_dir}")
    info(f"  visual_format: {args.visual_format}")
    info(f"  origin_mode: {args.origin_mode}")
    info(f"  visual_objects: {', '.join(obj.name for obj in visual_objects)}")
    info(f"  collision_source: {collision_source}")
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


def convert(args):
    if args.scale <= 0.0:
        raise RuntimeError("--scale must be greater than zero")
    if args.trunk_radius <= 0.0 or args.trunk_height <= 0.0:
        raise RuntimeError("--trunk-radius and --trunk-height must be greater than zero")
    if args.collision_min_radius <= 0.0:
        raise RuntimeError("--collision-min-radius must be greater than zero")
    if args.collision_cylinder_sides < 6:
        raise RuntimeError("--collision-cylinder-sides must be 6 or greater")
    if args.collision_max_branch_height <= 0.0:
        raise RuntimeError("--collision-max-branch-height must be greater than zero")

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
        custom_collision_objects = convert_objects_to_meshes(bpy, collision_collection_raw)
        if not custom_collision_objects:
            warn(
                f"Collision collection '{args.collision_collection}' had no exportable meshes; "
                f"falling back to collision-mode={args.collision_mode}."
            )
            use_collision_collection = False

    export_base_objects = visual_objects + custom_collision_objects
    clear_parents_keep_transform(bpy, export_base_objects)
    apply_uniform_scale(export_base_objects, args.scale)

    origin_vector = compute_origin_vector(bpy, mathutils, args, visual_objects)
    shift_objects(export_base_objects, origin_vector)
    visual_bounds = world_bounds(mathutils, visual_objects)
    branch_segments = transform_branch_segments(branch_segments_raw, args.scale, origin_vector)
    log_visual_bounds(visual_bounds)

    if use_collision_file:
        collision_source = f"file:{collision_file_path}"
    elif use_collision_collection:
        collision_source = f"collection:{args.collision_collection}"
    else:
        collision_source = args.collision_mode
    copied_textures = []

    if args.dry_run:
        print_summary(
            args,
            input_path,
            output_dir,
            visual_objects,
            collision_source,
            visual_bounds,
            copied_textures,
        )
        info("Dry run only; no files were written.")
        return

    meshes_dir.mkdir(parents=True, exist_ok=True)

    if args.copy_textures:
        copied_textures = copy_material_textures(bpy, visual_objects, meshes_dir / "textures")

    export_obj_compatible(bpy, visual_objects, meshes_dir / "tree_mesh.obj")

    collision_output_path = meshes_dir / "tree_collision.stl"
    if use_collision_file:
        shutil.copy2(collision_file_path, collision_output_path)
        info(f"Copied collision file: {collision_file_path} -> {collision_output_path}")
    elif use_collision_collection:
        collision_objects = custom_collision_objects
        export_stl(bpy, collision_objects, collision_output_path)
    else:
        collision_objects = make_generated_collision(
            bpy, mathutils, args, visual_bounds, visual_objects, branch_segments
        )
        export_stl(bpy, collision_objects, collision_output_path)
    write_gazebo_files(output_dir, args.model_name)

    print_summary(
        args,
        input_path,
        output_dir,
        visual_objects,
        collision_source,
        visual_bounds,
        copied_textures,
    )
    info(f"Wrote: {output_dir / 'model.config'}")
    info(f"Wrote: {output_dir / 'model.sdf'}")
    info(f"Wrote: {meshes_dir / 'tree_mesh.obj'}")
    info(f"Wrote: {meshes_dir / 'tree_collision.stl'}")


def main():
    args = parse_args(blender_argv())
    try:
        convert(args)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None


if __name__ == "__main__":
    main()
