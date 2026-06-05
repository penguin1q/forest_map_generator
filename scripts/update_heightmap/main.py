#!/usr/bin/env python3
import os
import sys
import shutil
import argparse
from datetime import datetime
from PIL import Image
import xml.etree.ElementTree as ET


def ensure_single(parent, tag, text):
    elems = parent.findall(tag)
    if elems:
        elems[0].text = text
        for e in elems[1:]:
            parent.remove(e)
    else:
        ET.SubElement(parent, tag).text = text


def ensure_blends(heightmap, b1_min, b1_fade, b2_min, b2_fade):
    blends = heightmap.findall("blend")

    while len(blends) < 2:
        heightmap.append(ET.Element("blend"))
        blends = heightmap.findall("blend")

    for e in blends[2:]:
        heightmap.remove(e)

    b1, b2 = blends[0], blends[1]

    ensure_single(b1, "min_height", str(b1_min))
    ensure_single(b1, "fade_dist", str(b1_fade))
    ensure_single(b2, "min_height", str(b2_min))
    ensure_single(b2, "fade_dist", str(b2_fade))


def backup(path):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bak = f"{path}.bak_{ts}"
    shutil.copy2(path, bak)
    return bak


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(os.path.dirname(script_dir))

    default_heightmap = os.path.join(script_dir, "heightmap.png")
    default_sdf = os.path.join(pkg_root, "models", "terrain", "model.sdf")
    default_target = os.path.join(
        pkg_root, "models", "terrain", "materials", "textures"
    )

    parser = argparse.ArgumentParser()

    parser.add_argument("--heightmap", default=default_heightmap)
    parser.add_argument("--height_range", type=float, default=50.0)
    parser.add_argument("--blend1_min", type=float, default=1.0)
    parser.add_argument("--blend1_fade", type=float, default=3.0)
    parser.add_argument("--blend2_min", type=float, default=10.0)
    parser.add_argument("--blend2_fade", type=float, default=3.0)
    parser.add_argument("--pos_x", type=float, default=0.0)
    parser.add_argument("--pos_y", type=float, default=0.0)
    parser.add_argument("--pos_z", type=float, default=0.0)
    parser.add_argument("--terrain_sdf", default=default_sdf)
    parser.add_argument("--target_dir", default=default_target)
    parser.add_argument("--dry_run", action="store_true")

    args = parser.parse_args()

    heightmap_path = os.path.abspath(args.heightmap)
    if not os.path.isfile(heightmap_path):
        print(f"heightmap not found: {heightmap_path}")
        sys.exit(1)

    with Image.open(heightmap_path) as img:
        w, h = img.size

    os.makedirs(args.target_dir, exist_ok=True)
    dst_heightmap = os.path.join(args.target_dir, os.path.basename(heightmap_path))

    terrain_sdf_path = os.path.abspath(args.terrain_sdf)
    terrain_model_name = os.path.basename(os.path.dirname(terrain_sdf_path))
    uri = (
        f"model://{terrain_model_name}/materials/textures/"
        f"{os.path.basename(heightmap_path)}"
    )
    size = f"{w} {h} {args.height_range}"
    pos = f"{args.pos_x} {args.pos_y} {args.pos_z}"

    print("heightmap:", heightmap_path)
    print("heightmap_size:", f"{w}x{h}")
    print("terrain_model:", terrain_model_name)
    print("terrain_sdf:", terrain_sdf_path)
    print("sdf_uri:", uri)
    print("sdf_size:", size)
    print("sdf_pos:", pos)
    print("copy_to:", dst_heightmap)

    tree = ET.parse(terrain_sdf_path)
    root = tree.getroot()

    heightmaps = root.findall(".//heightmap")
    if not heightmaps:
        print("no <heightmap> found in sdf")
        sys.exit(1)

    for hm in heightmaps:
        ensure_single(hm, "uri", uri)
        ensure_single(hm, "size", size)
        ensure_blends(
            hm, args.blend1_min, args.blend1_fade, args.blend2_min, args.blend2_fade
        )
        ensure_single(hm, "pos", pos)

    if args.dry_run:
        print("dry_run: no files modified")
        return

    bak_path = backup(terrain_sdf_path)
    shutil.copy2(heightmap_path, dst_heightmap)
    tree.write(terrain_sdf_path, encoding="utf-8", xml_declaration=True)

    print("backup:", bak_path)
    print("updated_sdf:", terrain_sdf_path)


if __name__ == "__main__":
    main()
