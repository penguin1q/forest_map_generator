#!/usr/bin/env python3
import argparse
import json
import math
import sys
from pathlib import Path

try:
    import yaml
    from pyproj import CRS, Transformer
    from shapely.geometry import GeometryCollection, LineString, MultiLineString, Polygon
except ImportError as e:
    print(
        "Missing dependency: %s\n"
        "Install dependencies with:\n"
        "  pip3 install shapely pyproj PyYAML" % e,
        file=sys.stderr,
    )
    sys.exit(2)


CRS84 = {"type": "name", "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}}
EPS = 1e-8


def resolve_path(path):
    return Path(path).expanduser().resolve()


def utm_crs_from_lonlat(lon, lat):
    zone = int((float(lon) + 180.0) / 6.0) + 1
    epsg = 32600 + zone if float(lat) >= 0.0 else 32700 + zone
    return CRS.from_epsg(epsg)


class LocalLonLatProjector:
    def __init__(self, origin_lat, origin_lon, world_yaw_deg):
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.world_yaw_deg = float(world_yaw_deg)
        self.utm_crs = utm_crs_from_lonlat(self.origin_lon, self.origin_lat)
        self.to_utm = Transformer.from_crs(
            CRS.from_epsg(4326), self.utm_crs, always_xy=True
        )
        self.to_lonlat = Transformer.from_crs(
            self.utm_crs, CRS.from_epsg(4326), always_xy=True
        )
        self.origin_easting, self.origin_northing = self.to_utm.transform(
            self.origin_lon, self.origin_lat
        )

    def lonlat_to_local_xy(self, lon, lat):
        easting, northing = self.to_utm.transform(float(lon), float(lat))
        east = easting - self.origin_easting
        north = northing - self.origin_northing
        yaw = math.radians(self.world_yaw_deg)
        x = east * math.cos(yaw) + north * math.sin(yaw)
        y = -east * math.sin(yaw) + north * math.cos(yaw)
        return x, y

    def local_xy_to_lonlat(self, x, y):
        yaw = math.radians(self.world_yaw_deg)
        east = float(x) * math.cos(yaw) - float(y) * math.sin(yaw)
        north = float(x) * math.sin(yaw) + float(y) * math.cos(yaw)
        lon, lat = self.to_lonlat.transform(
            self.origin_easting + east, self.origin_northing + north
        )
        return [float(lon), float(lat)]


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def load_terrain_config(path):
    with open(path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    georef = config.get("georeference", {})
    if not isinstance(georef, dict):
        raise RuntimeError("terrain_config georeference section must be a mapping")

    origin_lat = georef.get("origin_lat_deg", georef.get("center_lat_deg"))
    origin_lon = georef.get("origin_lon_deg", georef.get("center_lon_deg"))
    if origin_lat is None or origin_lon is None:
        raise RuntimeError(
            "terrain_config needs georeference origin_lat_deg/origin_lon_deg "
            "or center_lat_deg/center_lon_deg"
        )

    yaw = georef.get("world_yaw_deg", 0.0)
    return LocalLonLatProjector(origin_lat, origin_lon, yaw)


def prop(properties, key, default=""):
    value = properties.get(key, default)
    return default if value is None else value


def prop_str(properties, key, default=""):
    value = prop(properties, key, default)
    return "" if value is None else str(value).strip()


def parse_required_float(properties, key, context):
    value = prop(properties, key, None)
    if value is None or str(value).strip() == "":
        print(f"warning: {context}: missing required property {key}; skipping")
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        print(f"warning: {context}: invalid {key}={value!r}; skipping")
        return None
    if parsed <= 0.0:
        print(f"warning: {context}: {key} must be > 0; skipping")
        return None
    return parsed


def parse_optional_float(properties, key, default, context):
    value = prop(properties, key, default)
    if value is None or str(value).strip() == "":
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        print(f"warning: {context}: invalid {key}={value!r}; using {default}")
        return float(default)


def is_enabled(properties):
    value = prop(properties, "enabled", 1)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() not in ("", "0", "false", "no", "off", "disabled")


def polygon_to_local(geometry, projector, context):
    if geometry.get("type") != "Polygon":
        print(
            "warning: %s: unsupported planting geometry type %r; skipping"
            % (context, geometry.get("type"))
        )
        return None

    rings = geometry.get("coordinates")
    if not isinstance(rings, list) or not rings:
        print(f"warning: {context}: invalid Polygon coordinates; skipping")
        return None

    try:
        exterior = [projector.lonlat_to_local_xy(lon, lat) for lon, lat, *_ in rings[0]]
        holes = [
            [projector.lonlat_to_local_xy(lon, lat) for lon, lat, *_ in ring]
            for ring in rings[1:]
        ]
        polygon = Polygon(exterior, holes)
    except (TypeError, ValueError) as e:
        print(f"warning: {context}: invalid Polygon lon/lat coordinates: {e}; skipping")
        return None

    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    if polygon.is_empty:
        print(f"warning: {context}: polygon is empty after validation; skipping")
        return None
    return polygon


def line_to_local(geometry, projector, context):
    if geometry.get("type") != "LineString":
        print(
            "warning: %s: unsupported row direction geometry type %r; skipping"
            % (context, geometry.get("type"))
        )
        return None

    coordinates = geometry.get("coordinates")
    if not isinstance(coordinates, list) or len(coordinates) < 2:
        print(f"warning: {context}: invalid LineString coordinates; skipping")
        return None

    try:
        points = [projector.lonlat_to_local_xy(lon, lat) for lon, lat, *_ in coordinates]
    except (TypeError, ValueError) as e:
        print(f"warning: {context}: invalid LineString lon/lat coordinates: {e}; skipping")
        return None

    line = LineString(points)
    if line.length <= EPS:
        print(f"warning: {context}: row direction length is zero; skipping")
        return None
    return line


def local_line_to_lonlat_coords(line, projector):
    return [projector.local_xy_to_lonlat(x, y) for x, y in line.coords]


def local_point_to_lonlat(point, projector):
    return projector.local_xy_to_lonlat(point.x, point.y)


def extract_line_parts(geometry):
    if geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry]
    if isinstance(geometry, MultiLineString):
        return list(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        parts = []
        for geom in geometry.geoms:
            parts.extend(extract_line_parts(geom))
        return parts
    return []


def offset_sequence(row_spacing, max_offset):
    offsets = [0.0]
    index = 1
    while index * row_spacing <= max_offset + EPS:
        value = index * row_spacing
        offsets.append(value)
        offsets.append(-value)
        index += 1
    return offsets


def generate_parallel_rows(effective_area, direction_line, row_spacing, min_row_length_m):
    x_min, y_min, x_max, y_max = effective_area.bounds
    width = x_max - x_min
    height = y_max - y_min
    diagonal = math.hypot(width, height)
    if diagonal <= EPS:
        return []

    coords = list(direction_line.coords)
    x1, y1 = coords[0]
    x2, y2 = coords[-1]
    direction_length = math.hypot(x2 - x1, y2 - y1)
    if direction_length <= EPS:
        return []

    ux = (x2 - x1) / direction_length
    uy = (y2 - y1) / direction_length
    nx = -uy
    ny = ux
    base_x, base_y = direction_line.interpolate(0.5, normalized=True).coords[0]
    long_len = diagonal * 2.0 + row_spacing * 4.0

    rows = []
    for offset in offset_sequence(row_spacing, diagonal + row_spacing):
        cx = base_x + offset * nx
        cy = base_y + offset * ny
        candidate = LineString(
            [
                (cx - ux * long_len, cy - uy * long_len),
                (cx + ux * long_len, cy + uy * long_len),
            ]
        )
        clipped = candidate.intersection(effective_area)
        for part in extract_line_parts(clipped):
            if part.length + EPS < min_row_length_m:
                continue
            rows.append(part)
    return rows


def generate_preview_points(row_line, tree_spacing):
    points = []
    distance = 0.0
    index = 0
    while distance <= row_line.length + EPS:
        points.append((distance, row_line.interpolate(min(distance, row_line.length))))
        index += 1
        distance = index * tree_spacing
    return points


def feature_collection(name, features):
    return {
        "type": "FeatureCollection",
        "name": name,
        "crs": CRS84,
        "features": features,
    }


def load_row_directions(path, projector):
    data = load_json(path)
    directions_by_area = {}
    for index, feature in enumerate(data.get("features", [])):
        context = f"row direction feature {index}"
        if not isinstance(feature, dict) or feature.get("type") != "Feature":
            print(f"warning: {context}: malformed feature; skipping")
            continue
        properties = feature.get("properties") or {}
        if not is_enabled(properties):
            continue
        area_name = prop_str(properties, "area_name")
        if not area_name:
            print(f"warning: {context}: missing required property area_name; skipping")
            continue
        line = line_to_local(feature.get("geometry") or {}, projector, context)
        if line is None:
            continue
        directions_by_area.setdefault(area_name, []).append(
            {
                "name": prop_str(properties, "name", f"{area_name}_direction"),
                "line": line,
            }
        )
    return directions_by_area


def process_planting_areas(args, projector, directions_by_area):
    planting_data = load_json(args.planting_areas)
    row_features = []
    preview_features = []
    total_skipped = 0

    for index, feature in enumerate(planting_data.get("features", [])):
        context = f"planting area feature {index}"
        if not isinstance(feature, dict) or feature.get("type") != "Feature":
            print(f"warning: {context}: malformed feature; skipping")
            total_skipped += 1
            continue

        properties = feature.get("properties") or {}
        if not is_enabled(properties):
            continue

        area_name = prop_str(properties, "name")
        if not area_name:
            print(f"warning: {context}: missing required property name; skipping")
            total_skipped += 1
            continue

        row_spacing = parse_required_float(properties, "row_spacing", area_name)
        tree_spacing = parse_required_float(properties, "tree_spacing", area_name)
        if row_spacing is None or tree_spacing is None:
            total_skipped += 1
            continue

        margin = parse_optional_float(properties, "margin", 0.0, area_name)
        yaw = parse_optional_float(properties, "yaw", 0.0, area_name)
        scale = parse_optional_float(properties, "scale", 1.0, area_name)
        tree_type = prop_str(properties, "tree_type", "tree1") or "tree1"

        polygon = polygon_to_local(feature.get("geometry") or {}, projector, area_name)
        if polygon is None:
            total_skipped += 1
            continue

        effective_area = polygon.buffer(-margin) if margin > 0.0 else polygon
        if effective_area.is_empty:
            print(f"warning: {area_name}: empty polygon after margin={margin}; skipping")
            total_skipped += 1
            continue

        directions = directions_by_area.get(area_name, [])
        if not directions:
            print(f"warning: {area_name}: no enabled row direction found; skipping")
            total_skipped += 1
            continue
        if len(directions) > 1:
            print(
                "warning: %s: multiple row directions found; using first (%s)"
                % (area_name, directions[0]["name"])
            )

        direction = directions[0]
        generated_rows = generate_parallel_rows(
            effective_area, direction["line"], row_spacing, args.min_row_length_m
        )

        area_row_count = 0
        area_point_count = 0
        for row_line in generated_rows:
            local_row_index = area_row_count
            name_prefix = f"{area_name}_row_{local_row_index:03d}"
            row_properties = {
                "mode": "trees_on_line",
                "name_prefix": name_prefix,
                "tree_type": tree_type,
                "spacing": float(tree_spacing),
                "yaw": float(yaw),
                "scale": float(scale),
                "source_area": area_name,
                "source_direction": direction["name"],
                "row_index": int(local_row_index),
            }
            row_features.append(
                {
                    "type": "Feature",
                    "properties": row_properties,
                    "geometry": {
                        "type": "LineString",
                        "coordinates": local_line_to_lonlat_coords(row_line, projector),
                    },
                }
            )

            for point_index, (distance_m, point) in enumerate(
                generate_preview_points(row_line, tree_spacing)
            ):
                preview_features.append(
                    {
                        "type": "Feature",
                        "properties": {
                            "name": f"{name_prefix}_tree_{point_index:03d}",
                            "source_area": area_name,
                            "source_row": name_prefix,
                            "tree_type": tree_type,
                            "spacing": float(tree_spacing),
                            "row_spacing": float(row_spacing),
                            "yaw": float(yaw),
                            "scale": float(scale),
                            "distance_m": float(distance_m),
                        },
                        "geometry": {
                            "type": "Point",
                            "coordinates": local_point_to_lonlat(point, projector),
                        },
                    }
                )
                area_point_count += 1
            area_row_count += 1

        print(f"area {area_name}:")
        print(f"  row_spacing: {row_spacing}")
        print(f"  tree_spacing: {tree_spacing}")
        print(f"  margin: {margin}")
        print(f"  generated rows: {area_row_count}")
        print(f"  generated preview points: {area_point_count}")

    return row_features, preview_features, total_skipped


def build_parser():
    parser = argparse.ArgumentParser(
        description="Generate GeoJSON tree rows from QGIS planting area polygons."
    )
    parser.add_argument("--terrain-config", required=True)
    parser.add_argument("--planting-areas", required=True)
    parser.add_argument("--row-directions", required=True)
    parser.add_argument("--output-tree-rows", required=True)
    parser.add_argument("--output-tree-points-preview", default="")
    parser.add_argument("--min-row-length-m", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--preview-only", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    args.terrain_config = resolve_path(args.terrain_config)
    args.planting_areas = resolve_path(args.planting_areas)
    args.row_directions = resolve_path(args.row_directions)
    args.output_tree_rows = resolve_path(args.output_tree_rows)
    args.output_tree_points_preview = (
        resolve_path(args.output_tree_points_preview)
        if args.output_tree_points_preview
        else None
    )

    if args.min_row_length_m <= 0.0:
        print("--min-row-length-m must be greater than zero", file=sys.stderr)
        return 1

    try:
        projector = load_terrain_config(args.terrain_config)
    except (OSError, yaml.YAMLError, RuntimeError, ValueError) as e:
        print(f"error: failed to read terrain config: {e}", file=sys.stderr)
        return 1

    print("terrain_config:", args.terrain_config)
    print("origin lat/lon:", f"{projector.origin_lat}, {projector.origin_lon}")
    print("world_yaw_deg:", projector.world_yaw_deg)
    print("utm_crs:", projector.utm_crs)
    print("input planting areas:", args.planting_areas)
    print("input row directions:", args.row_directions)

    try:
        directions_by_area = load_row_directions(args.row_directions, projector)
        row_features, preview_features, skipped = process_planting_areas(
            args, projector, directions_by_area
        )
    except (OSError, json.JSONDecodeError) as e:
        print(f"error: failed to read GeoJSON inputs: {e}", file=sys.stderr)
        return 1

    print("summary:")
    print("  generated rows:", len(row_features))
    print("  generated preview points:", len(preview_features))
    print("  skipped planting areas/features:", skipped)
    if not args.preview_only:
        print("output_tree_rows:", args.output_tree_rows)
    if args.output_tree_points_preview:
        print("output_tree_points_preview:", args.output_tree_points_preview)

    if args.dry_run:
        print("dry_run: no files written")
        return 0

    if not args.preview_only:
        write_json(
            args.output_tree_rows,
            feature_collection("generated_tree_rows", row_features),
        )
    if args.output_tree_points_preview:
        write_json(
            args.output_tree_points_preview,
            feature_collection("generated_tree_points_preview", preview_features),
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
