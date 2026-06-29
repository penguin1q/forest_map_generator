#!/usr/bin/env python3
"""Append static object <include> elements to a Gazebo world from GeoJSON.

This tool is intended to run after forest_map_generator has generated a base
world with terrain, trees, and optional roads. It keeps semantic IDs inside each
model and only places the models in the world.
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

try:
    import yaml
except ImportError:
    yaml = None

try:
    from pyproj import CRS, Transformer
except ImportError:
    CRS = None
    Transformer = None


SUPPORTED_LABELS = {
    "rock",
    "pole",
    "fence",
    "basket",
    "crate",
    "wall",
    "container",
    "sign",
    "obstacle",
}


def warn(message):
    print(f"warning: {message}", file=sys.stderr)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_yaml(path):
    if yaml is None:
        raise RuntimeError("PyYAML is required when --terrain-config is used")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def parse_bool(value, default=True):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() not in ("", "0", "false", "no", "off", "disabled")


def prop(properties, key, default=""):
    value = properties.get(key, default)
    return default if value is None else value


def prop_str(properties, key, default=""):
    value = prop(properties, key, default)
    return "" if value is None else str(value).strip()


def prop_float(properties, key, default, context):
    value = prop(properties, key, default)
    if value is None or str(value).strip() == "":
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        warn(f"{context}: invalid {key}={value!r}; using {default}")
        return float(default)


def utm_crs_from_lonlat(lon, lat):
    zone = int((float(lon) + 180.0) / 6.0) + 1
    epsg = 32600 + zone if float(lat) >= 0.0 else 32700 + zone
    return CRS.from_epsg(epsg)


class LonLatToWorld:
    def __init__(self, terrain_config_path):
        if CRS is None or Transformer is None:
            raise RuntimeError("pyproj is required for coordinate_mode=lonlat")
        config = load_yaml(terrain_config_path)
        georef = config.get("georeference", {})
        if not isinstance(georef, dict):
            raise RuntimeError("terrain_config georeference must be a mapping")
        origin_lat = georef.get("origin_lat_deg", georef.get("center_lat_deg"))
        origin_lon = georef.get("origin_lon_deg", georef.get("center_lon_deg"))
        if origin_lat is None or origin_lon is None:
            raise RuntimeError(
                "terrain_config needs georeference origin_lat_deg/origin_lon_deg "
                "or center_lat_deg/center_lon_deg"
            )
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.world_yaw_deg = float(georef.get("world_yaw_deg", 0.0))
        self.utm_crs = utm_crs_from_lonlat(self.origin_lon, self.origin_lat)
        self.to_utm = Transformer.from_crs(CRS.from_epsg(4326), self.utm_crs, always_xy=True)
        self.origin_easting, self.origin_northing = self.to_utm.transform(
            self.origin_lon, self.origin_lat
        )

    def convert(self, lon, lat):
        easting, northing = self.to_utm.transform(float(lon), float(lat))
        east = easting - self.origin_easting
        north = northing - self.origin_northing
        yaw = math.radians(self.world_yaw_deg)
        world_x = east * math.cos(yaw) + north * math.sin(yaw)
        world_y = -east * math.sin(yaw) + north * math.cos(yaw)
        return world_x, world_y


class HeightSampler:
    def __init__(self, heightmap_path, size_x, size_y, world_size_x, world_size_y, size_z):
        self.enabled = False
        self.data = None
        self.size_x = int(size_x)
        self.size_y = int(size_y)
        self.world_size_x = float(world_size_x)
        self.world_size_y = float(world_size_y)
        self.size_z = float(size_z)
        if heightmap_path is None:
            return
        if cv2 is None or np is None:
            warn("OpenCV/numpy are not available; z must be specified in GeoJSON")
            return
        data = cv2.imread(str(heightmap_path), cv2.IMREAD_GRAYSCALE)
        if data is None:
            warn(f"failed to read heightmap: {heightmap_path}; z must be specified")
            return
        self.data = np.array(data, dtype=np.float32)
        self.size_y, self.size_x = self.data.shape[:2]
        self.enabled = True

    def z_at(self, world_x, world_y):
        if not self.enabled:
            return None
        normalized_x = (float(world_x) / self.world_size_x) + 0.5
        normalized_y = -(float(world_y) / self.world_size_y) + 0.5
        px = int(round(normalized_x * (self.size_x - 1)))
        py = int(round(normalized_y * (self.size_y - 1)))
        if px < 0 or py < 0 or px >= self.size_x or py >= self.size_y:
            return None
        return float(self.data[py, px] / 255.0 * self.size_z)


def sanitize_name(value, fallback):
    raw = str(value or fallback).strip()
    allowed = []
    for ch in raw:
        if ch.isalnum() or ch in ("_", "-", "."):
            allowed.append(ch)
        else:
            allowed.append("_")
    name = "".join(allowed).strip("_")
    return name or fallback


def parse_candidates(value):
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def choose_model(properties, context):
    candidates = parse_candidates(prop(properties, "model", ""))
    if not candidates:
        candidates = parse_candidates(prop(properties, "models", ""))
    if not candidates:
        label = prop_str(properties, "label") or prop_str(properties, "semantic_label")
        if label:
            candidates = [label]
    if not candidates:
        warn(f"{context}: missing model/models/label; skipping")
        return None
    return candidates[0]


def parse_xy(coordinates, coordinate_mode, converter, context):
    if not isinstance(coordinates, (list, tuple)) or len(coordinates) < 2:
        warn(f"{context}: invalid coordinates; skipping")
        return None
    if coordinate_mode == "local_xy":
        try:
            return float(coordinates[0]), float(coordinates[1])
        except (TypeError, ValueError):
            warn(f"{context}: x/y must be numeric; skipping")
            return None
    if coordinate_mode == "lonlat":
        return converter.convert(coordinates[0], coordinates[1])
    raise RuntimeError(f"unsupported coordinate mode: {coordinate_mode}")


def object_xml(name, model, x, y, z, roll, pitch, yaw, scale_xyz):
    sx, sy, sz = scale_xyz
    scale_xml = ""
    if any(abs(v - 1.0) > 1e-9 for v in scale_xyz):
        scale_xml = f"            <scale>{sx:.6f} {sy:.6f} {sz:.6f}</scale>\n"
    return f'''
        <include>
            <name>{name}</name>
            <uri>model://{model}</uri>
            <pose>{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
{scale_xml}        </include>
'''


def create_static_object_xml(geojson_path, coordinate_mode, converter, height_sampler, default_z_offset):
    data = load_json(geojson_path)
    if data.get("type") != "FeatureCollection":
        raise RuntimeError("static object GeoJSON root type must be FeatureCollection")
    features = data.get("features", [])
    if not isinstance(features, list):
        raise RuntimeError("static object GeoJSON features must be a list")

    xml = "\n    <!-- Auto-generated static objects -->\n"
    accepted = 0
    skipped = 0
    for index, feature in enumerate(features):
        context = f"feature {index}"
        if not isinstance(feature, dict) or feature.get("type") != "Feature":
            skipped += 1
            warn(f"{context}: malformed feature; skipping")
            continue
        properties = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        if not parse_bool(properties.get("enabled"), True):
            continue
        if geometry.get("type") != "Point":
            skipped += 1
            warn(f"{context}: only Point geometry is supported for static objects; skipping")
            continue

        model = choose_model(properties, context)
        if model is None:
            skipped += 1
            continue
        label = prop_str(properties, "label") or prop_str(properties, "semantic_label")
        if label and label not in SUPPORTED_LABELS:
            warn(f"{context}: label {label!r} is not in static-object labels; keeping placement")

        xy = parse_xy(geometry.get("coordinates"), coordinate_mode, converter, context)
        if xy is None:
            skipped += 1
            continue
        world_x, world_y = xy

        z_text = prop_str(properties, "z")
        if z_text:
            try:
                terrain_z = float(z_text)
            except ValueError:
                skipped += 1
                warn(f"{context}: invalid z={z_text!r}; skipping")
                continue
        else:
            terrain_z = height_sampler.z_at(world_x, world_y)
            if terrain_z is None:
                terrain_z = 0.0
                warn(f"{context}: z omitted and terrain height unavailable; using z=0")
        z_offset = prop_float(properties, "z_offset", default_z_offset, context)
        world_z = terrain_z + z_offset

        roll = prop_float(properties, "roll", 0.0, context)
        pitch = prop_float(properties, "pitch", 0.0, context)
        yaw = prop_float(properties, "yaw", 0.0, context)
        scale = prop_float(properties, "scale", 1.0, context)
        scale_x = prop_float(properties, "scale_x", scale, context)
        scale_y = prop_float(properties, "scale_y", scale, context)
        scale_z = prop_float(properties, "scale_z", scale, context)

        raw_name = prop_str(properties, "name")
        fallback_name = f"{model}_{accepted:04d}"
        name = sanitize_name(raw_name, fallback_name)
        xml += object_xml(
            name=name,
            model=model,
            x=world_x,
            y=world_y,
            z=world_z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            scale_xyz=(scale_x, scale_y, scale_z),
        )
        accepted += 1

    xml += "    <!-- End auto-generated static objects -->\n"
    print(f"static object placement completed: accepted={accepted}, skipped={skipped}")
    return xml if accepted > 0 else ""


def inject_before_world_end(world_content, injected_xml):
    if not injected_xml:
        return world_content
    marker = "</world>"
    if marker not in world_content:
        raise RuntimeError("input world is missing </world>")
    return world_content.replace(marker, injected_xml + "  " + marker, 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-world", required=True, help="Generated base world file")
    parser.add_argument("--output-world", required=True, help="World file with static objects")
    parser.add_argument("--static-objects", required=True, help="Static object GeoJSON")
    parser.add_argument(
        "--coordinate-mode",
        choices=("local_xy", "lonlat"),
        default="local_xy",
        help="Coordinate interpretation for GeoJSON Point coordinates",
    )
    parser.add_argument("--terrain-config", default="", help="Required for lonlat mode")
    parser.add_argument("--heightmap", default="", help="Optional heightmap for z sampling")
    parser.add_argument("--terrain-world-size-x", type=float, default=257.0)
    parser.add_argument("--terrain-world-size-y", type=float, default=257.0)
    parser.add_argument("--terrain-size-z", type=float, default=50.0)
    parser.add_argument("--default-z-offset", type=float, default=0.0)
    args = parser.parse_args()

    converter = None
    if args.coordinate_mode == "lonlat":
        if not args.terrain_config:
            parser.error("--terrain-config is required for --coordinate-mode lonlat")
        converter = LonLatToWorld(args.terrain_config)

    height_sampler = HeightSampler(
        Path(args.heightmap) if args.heightmap else None,
        size_x=257,
        size_y=257,
        world_size_x=args.terrain_world_size_x,
        world_size_y=args.terrain_world_size_y,
        size_z=args.terrain_size_z,
    )
    injected_xml = create_static_object_xml(
        geojson_path=args.static_objects,
        coordinate_mode=args.coordinate_mode,
        converter=converter,
        height_sampler=height_sampler,
        default_z_offset=args.default_z_offset,
    )

    with open(args.input_world, "r", encoding="utf-8") as f:
        world_content = f.read()
    output_content = inject_before_world_end(world_content, injected_xml)
    output_path = Path(args.output_world)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(output_content)
    print(f"world with static objects saved to: {output_path}")


if __name__ == "__main__":
    main()
