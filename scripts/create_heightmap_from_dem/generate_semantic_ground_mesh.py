#!/usr/bin/env python3
import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import numpy as np
    import yaml
    from PIL import Image
except ImportError as e:
    print(
        "Missing dependency: %s\n"
        "Install dependencies with:\n"
        "  pip3 install PyYAML Pillow numpy" % e,
        file=sys.stderr,
    )
    sys.exit(2)


def resolve_path(path):
    return Path(path).expanduser().resolve()


def package_root():
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "package.xml").is_file() and (
            parent / "models" / "terrain"
        ).is_dir():
            return parent
    return Path.cwd().resolve()


def load_config(config_path):
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_heightmap_path(config_path, config, explicit_heightmap):
    if explicit_heightmap:
        return resolve_path(explicit_heightmap)

    heightmap_file = config["terrain"]["heightmap_file"]
    candidate = config_path.parent / heightmap_file
    if candidate.is_file():
        return candidate.resolve()

    pkg_root = package_root()
    model_candidate = pkg_root / "models" / "terrain" / "materials" / "textures" / heightmap_file
    return model_candidate.resolve()


def write_semantic_ground_obj(
    output_mesh,
    heightmap_png,
    width_m,
    height_m,
    height_range_m,
    terrain_pos_z,
    z_offset_m,
    stride,
):
    if stride < 1:
        raise RuntimeError("--stride must be at least 1")

    image = Image.open(heightmap_png).convert("L")
    png = np.asarray(image, dtype=np.float32)[::stride, ::stride]
    rows, cols = png.shape
    if rows < 2 or cols < 2:
        raise RuntimeError("semantic ground mesh needs at least 2x2 vertices")

    xs = np.linspace(-float(width_m) / 2.0, float(width_m) / 2.0, cols)
    ys = np.linspace(-float(height_m) / 2.0, float(height_m) / 2.0, rows)
    z = (png / 255.0) * float(height_range_m)
    z += float(terrain_pos_z) + float(z_offset_m)

    output_mesh.parent.mkdir(parents=True, exist_ok=True)
    with open(output_mesh, "w", encoding="utf-8") as f:
        f.write("# Gazebo semantic ground mesh generated from heightmap PNG\n")
        f.write("o semantic_ground\n")
        for j, y_m in enumerate(ys):
            for i, x_m in enumerate(xs):
                f.write(f"v {x_m:.6f} {y_m:.6f} {float(z[j, i]):.6f}\n")

        for j in range(rows - 1):
            for i in range(cols - 1):
                v00 = j * cols + i + 1
                v10 = j * cols + i + 2
                v01 = (j + 1) * cols + i + 1
                v11 = (j + 1) * cols + i + 2
                f.write(f"f {v00} {v10} {v11}\n")
                f.write(f"f {v00} {v11} {v01}\n")


def ensure_single(parent, tag, text):
    elems = parent.findall(tag)
    if elems:
        elems[0].text = text
        for elem in elems[1:]:
            parent.remove(elem)
    else:
        ET.SubElement(parent, tag).text = text


def ensure_visual_label(visual, label_id):
    label_plugin = None
    for plugin in visual.findall("plugin"):
        if (
            plugin.get("name") == "gz::sim::systems::Label"
            or plugin.get("filename") == "gz-sim-label-system"
        ):
            label_plugin = plugin
            break

    if label_plugin is None:
        label_plugin = ET.SubElement(
            visual,
            "plugin",
            {
                "filename": "gz-sim-label-system",
                "name": "gz::sim::systems::Label",
            },
        )

    ensure_single(label_plugin, "label", str(int(label_id)))


def find_link_for_semantic_ground(root):
    parent_map = {child: parent for parent in root.iter() for child in parent}
    for visual in root.findall(".//visual"):
        if visual.find(".//heightmap") is not None:
            parent = parent_map.get(visual)
            if parent is not None and parent.tag == "link":
                return parent

    link = root.find(".//link")
    if link is None:
        raise RuntimeError("no <link> found in terrain SDF")
    return link


def remove_named_children(parent, tag, name):
    for child in list(parent.findall(tag)):
        if child.get("name") == name:
            parent.remove(child)


def mesh_uri_for_sdf(mesh_path, pkg_root):
    terrain_meshes_dir = (pkg_root / "models" / "terrain" / "meshes").resolve()
    mesh_path = Path(mesh_path).resolve()
    try:
        rel_path = mesh_path.relative_to(terrain_meshes_dir)
        return "model://terrain/meshes/" + rel_path.as_posix()
    except ValueError:
        return mesh_path.as_uri()


def add_semantic_ground_visual(terrain_sdf, mesh_uri, label, visual_name):
    tree = ET.parse(terrain_sdf)
    root = tree.getroot()
    link = find_link_for_semantic_ground(root)
    remove_named_children(link, "visual", visual_name)

    visual = ET.SubElement(link, "visual", {"name": visual_name})
    geometry = ET.SubElement(visual, "geometry")
    mesh = ET.SubElement(geometry, "mesh")
    ET.SubElement(mesh, "uri").text = mesh_uri
    ensure_visual_label(visual, label)

    tree.write(terrain_sdf, encoding="utf-8", xml_declaration=True)


def build_parser():
    pkg_root = package_root()
    parser = argparse.ArgumentParser(
        description=(
            "Generate a Gazebo semantic ground mesh from an existing DEM heightmap "
            "PNG/config pair and optionally inject it into terrain model.sdf."
        )
    )
    parser.add_argument("--terrain-config", required=True)
    parser.add_argument("--heightmap", default=None)
    parser.add_argument("--output-mesh", required=True)
    parser.add_argument("--copy-to-terrain-model", action="store_true")
    parser.add_argument(
        "--terrain-sdf",
        default=str(pkg_root / "models" / "terrain" / "model.sdf"),
    )
    parser.add_argument("--update-terrain-sdf", action="store_true")
    parser.add_argument("--label", type=int, default=20)
    parser.add_argument("--z-offset", type=float, default=0.02)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--visual-name", default="semantic_ground_visual")
    return parser


def main():
    args = build_parser().parse_args()
    pkg_root = package_root()
    terrain_config = resolve_path(args.terrain_config)
    terrain_sdf = resolve_path(args.terrain_sdf)
    output_mesh = resolve_path(args.output_mesh)

    if not terrain_config.is_file():
        print(f"terrain config does not exist: {terrain_config}", file=sys.stderr)
        return 1

    try:
        config = load_config(terrain_config)
        terrain = config["terrain"]
        heightmap = resolve_heightmap_path(terrain_config, config, args.heightmap)
        if not heightmap.is_file():
            raise RuntimeError(f"heightmap PNG does not exist: {heightmap}")

        write_semantic_ground_obj(
            output_mesh,
            heightmap,
            terrain["width_m"],
            terrain["height_m"],
            terrain["height_range_m"],
            terrain.get("terrain_pos_z_m", 0.0),
            args.z_offset,
            args.stride,
        )

        mesh_for_sdf = output_mesh
        if args.copy_to_terrain_model:
            mesh_for_sdf = pkg_root / "models" / "terrain" / "meshes" / output_mesh.name
            mesh_for_sdf.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(output_mesh, mesh_for_sdf)

        mesh_uri = mesh_uri_for_sdf(mesh_for_sdf, pkg_root)
        if args.update_terrain_sdf:
            add_semantic_ground_visual(
                terrain_sdf,
                mesh_uri,
                args.label,
                args.visual_name,
            )

        print("heightmap:", heightmap)
        print("output_mesh:", output_mesh)
        print("sdf_mesh_uri:", mesh_uri)
        print("terrain_sdf_updated:", str(terrain_sdf) if args.update_terrain_sdf else "false")
        print("semantic_label:", int(args.label))
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
