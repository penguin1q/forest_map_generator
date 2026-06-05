#!/usr/bin/env python3
import argparse
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import numpy as np
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import from_bounds
    from rasterio.warp import reproject
    from PIL import Image
    from pyproj import CRS, Transformer
    import yaml
except ImportError as e:
    print(
        "Missing dependency: %s\n"
        "Install DEM heightmap dependencies with:\n"
        "  pip3 install rasterio pyproj PyYAML Pillow numpy" % e,
        file=sys.stderr,
    )
    sys.exit(2)


def package_root():
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "package.xml").is_file() and (
            parent / "models" / "terrain"
        ).is_dir():
            return parent

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("forest_map_generator"))
    except Exception:
        return Path.cwd().resolve()


def resolve_path(path):
    return Path(path).expanduser().resolve()


def is_gazebo_heightmap_size(size):
    return size > 1 and (size - 1) & (size - 2) == 0


def utm_crs_for_lonlat(lon, lat):
    zone = int((lon + 180.0) // 6.0) + 1
    epsg = 32600 + zone if lat >= 0.0 else 32700 + zone
    return CRS.from_epsg(epsg)


class LocalLonLatProjector:
    def __init__(self, center_lat, center_lon):
        self.target_crs = utm_crs_for_lonlat(center_lon, center_lat)
        self.to_utm = Transformer.from_crs(
            "EPSG:4326", self.target_crs, always_xy=True
        )
        self.to_lonlat = Transformer.from_crs(
            self.target_crs, "EPSG:4326", always_xy=True
        )
        self.center_e, self.center_n = self.to_utm.transform(center_lon, center_lat)

    def local_xy_to_lonlat(self, x_m, y_m):
        lon, lat = self.to_lonlat.transform(
            self.center_e + float(x_m), self.center_n + float(y_m)
        )
        return [float(lon), float(lat)]


def format_meter(value):
    normalized = 0.0 if abs(value) < 1e-9 else float(value)
    return f"{normalized:.6f}".rstrip("0").rstrip(".")


def write_geojson(path, feature_collection):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(feature_collection, f, ensure_ascii=False, indent=2)
        f.write("\n")


def terrain_local_extent(width_m, height_m):
    return {
        "x_min": -float(width_m) / 2.0,
        "x_max": float(width_m) / 2.0,
        "y_min": -float(height_m) / 2.0,
        "y_max": float(height_m) / 2.0,
    }


def build_extent_geojson(center_lat, center_lon, width_m, height_m, size):
    extent = terrain_local_extent(width_m, height_m)
    projector = LocalLonLatProjector(center_lat, center_lon)
    corners = [
        (extent["x_min"], extent["y_min"]),
        (extent["x_max"], extent["y_min"]),
        (extent["x_max"], extent["y_max"]),
        (extent["x_min"], extent["y_max"]),
        (extent["x_min"], extent["y_min"]),
    ]
    return {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {
                    "name": "terrain_extent",
                    "width_m": float(width_m),
                    "height_m": float(height_m),
                    "size_px": int(size),
                    "center_lat_deg": float(center_lat),
                    "center_lon_deg": float(center_lon),
                    "world_frame": "ENU",
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [
                        [projector.local_xy_to_lonlat(x, y) for x, y in corners]
                    ],
                },
            }
        ],
    }


def build_corners_geojson(center_lat, center_lon, width_m, height_m):
    extent = terrain_local_extent(width_m, height_m)
    projector = LocalLonLatProjector(center_lat, center_lon)
    points = [
        ("center", 0.0, 0.0),
        ("southwest", extent["x_min"], extent["y_min"]),
        ("southeast", extent["x_max"], extent["y_min"]),
        ("northeast", extent["x_max"], extent["y_max"]),
        ("northwest", extent["x_min"], extent["y_max"]),
    ]
    features = []
    for name, x_m, y_m in points:
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "name": name,
                    "local_x_m": float(x_m),
                    "local_y_m": float(y_m),
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": projector.local_xy_to_lonlat(x_m, y_m),
                },
            }
        )
    return {"type": "FeatureCollection", "features": features}


def grid_values(min_value, max_value, step_m):
    start = math.ceil(min_value / step_m)
    end = math.floor(max_value / step_m)
    values = {round(i * step_m, 6) for i in range(start, end + 1)}
    if min_value <= 0.0 <= max_value:
        values.add(0.0)
    return sorted(values)


def build_grid_geojson(center_lat, center_lon, width_m, height_m, grid_step_m):
    extent = terrain_local_extent(width_m, height_m)
    projector = LocalLonLatProjector(center_lat, center_lon)
    xs = grid_values(extent["x_min"], extent["x_max"], grid_step_m)
    ys = grid_values(extent["y_min"], extent["y_max"], grid_step_m)
    features = []

    for x_m in xs:
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "name": f"x_{format_meter(x_m)}",
                    "axis": "x",
                    "value_m": float(x_m),
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        projector.local_xy_to_lonlat(x_m, extent["y_min"]),
                        projector.local_xy_to_lonlat(x_m, extent["y_max"]),
                    ],
                },
            }
        )

    for y_m in ys:
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "name": f"y_{format_meter(y_m)}",
                    "axis": "y",
                    "value_m": float(y_m),
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        projector.local_xy_to_lonlat(extent["x_min"], y_m),
                        projector.local_xy_to_lonlat(extent["x_max"], y_m),
                    ],
                },
            }
        )

    return {"type": "FeatureCollection", "features": features}


def path_for_config(path):
    try:
        return str(path.relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def resolve_qgis_marker_outputs(args):
    marker_dir = (
        resolve_path(args.output_qgis_markers_dir)
        if args.output_qgis_markers_dir
        else None
    )
    outputs = {}

    if marker_dir or args.output_extent_geojson:
        outputs["extent"] = (
            resolve_path(args.output_extent_geojson)
            if args.output_extent_geojson
            else marker_dir / "terrain_extent.geojson"
        )

    if marker_dir or args.output_corners_geojson:
        outputs["corners"] = (
            resolve_path(args.output_corners_geojson)
            if args.output_corners_geojson
            else marker_dir / "terrain_corners.geojson"
        )

    if not args.no_grid and (marker_dir or args.output_grid_geojson):
        outputs["grid"] = (
            resolve_path(args.output_grid_geojson)
            if args.output_grid_geojson
            else marker_dir / "terrain_grid.geojson"
        )

    return outputs


def filter_qgis_marker_outputs(marker_outputs, width_m, height_m, grid_step_m):
    outputs = dict(marker_outputs)
    if "grid" not in outputs:
        return outputs

    if grid_step_m <= 0.0:
        print("warning: --grid-step-m must be greater than zero; skipping grid")
        outputs.pop("grid")
        return outputs

    extent = terrain_local_extent(width_m, height_m)
    line_count = len(grid_values(extent["x_min"], extent["x_max"], grid_step_m))
    line_count += len(grid_values(extent["y_min"], extent["y_max"], grid_step_m))
    if line_count > 500:
        print("warning: QGIS grid would generate more than 500 lines; skipping grid")
        outputs.pop("grid")

    return outputs


def build_qgis_markers_config(marker_outputs, grid_step_m):
    if not marker_outputs:
        return None

    config = {
        "coordinate_format": "lonlat",
    }
    if "extent" in marker_outputs:
        config["extent_geojson"] = path_for_config(marker_outputs["extent"])
    if "corners" in marker_outputs:
        config["corners_geojson"] = path_for_config(marker_outputs["corners"])
    if "grid" in marker_outputs:
        config["grid_geojson"] = path_for_config(marker_outputs["grid"])
        config["grid_step_m"] = float(grid_step_m)
    return config


def print_qgis_marker_plan(marker_outputs, width_m, height_m, grid_step_m):
    if not marker_outputs:
        return

    extent = terrain_local_extent(width_m, height_m)
    print("local_extent:")
    print("  x:", f"{extent['x_min']} to {extent['x_max']} m")
    print("  y:", f"{extent['y_min']} to {extent['y_max']} m")
    if "extent" in marker_outputs:
        print("qgis_marker_extent:", marker_outputs["extent"])
    if "corners" in marker_outputs:
        print("qgis_marker_corners:", marker_outputs["corners"])
    if "grid" in marker_outputs:
        print("qgis_marker_grid:", marker_outputs["grid"])
        print("grid_step_m:", float(grid_step_m))


def write_qgis_marker_files(
    marker_outputs, center_lat, center_lon, width_m, height_m, size, grid_step_m
):
    if "extent" in marker_outputs:
        write_geojson(
            marker_outputs["extent"],
            build_extent_geojson(center_lat, center_lon, width_m, height_m, size),
        )

    if "corners" in marker_outputs:
        write_geojson(
            marker_outputs["corners"],
            build_corners_geojson(center_lat, center_lon, width_m, height_m),
        )

    if "grid" in marker_outputs:
        write_geojson(
            marker_outputs["grid"],
            build_grid_geojson(center_lat, center_lon, width_m, height_m, grid_step_m),
        )


def ensure_single(parent, tag, text):
    elems = parent.findall(tag)
    if elems:
        elems[0].text = text
        for elem in elems[1:]:
            parent.remove(elem)
    else:
        ET.SubElement(parent, tag).text = text


def update_terrain_sdf(
    terrain_sdf, heightmap_name, width_m, height_m, height_range_m, terrain_pos_z
):
    tree = ET.parse(terrain_sdf)
    root = tree.getroot()
    heightmaps = root.findall(".//heightmap")
    if not heightmaps:
        raise RuntimeError("no <heightmap> found in terrain SDF")

    uri = f"model://terrain/materials/textures/{heightmap_name}"
    size_text = f"{width_m} {height_m} {height_range_m}"
    pos_text = f"0.0 0.0 {terrain_pos_z}"

    for heightmap in heightmaps:
        ensure_single(heightmap, "uri", uri)
        ensure_single(heightmap, "size", size_text)
        ensure_single(heightmap, "pos", pos_text)

    tree.write(terrain_sdf, encoding="utf-8", xml_declaration=True)


def read_dem_to_local_grid(input_dem, center_lat, center_lon, width_m, height_m, size):
    # Practical Phase 4 georeference approach:
    # choose the UTM zone for the center lon/lat, build the requested ENU
    # rectangle as UTM easting/northing bounds, and reproject the source DEM
    # onto that local grid. Gazebo world X maps to Easting and Y maps to Northing.
    target_crs = utm_crs_for_lonlat(center_lon, center_lat)
    center_to_utm = Transformer.from_crs("EPSG:4326", target_crs, always_xy=True)
    center_e, center_n = center_to_utm.transform(center_lon, center_lat)

    west = center_e - width_m / 2.0
    east = center_e + width_m / 2.0
    south = center_n - height_m / 2.0
    north = center_n + height_m / 2.0
    dst_transform = from_bounds(west, south, east, north, size, size)

    with rasterio.open(input_dem) as src:
        if src.crs is None:
            raise RuntimeError("Input DEM has no CRS")

        dst = np.full((size, size), np.nan, dtype=np.float32)
        reproject(
            source=rasterio.band(src, 1),
            destination=dst,
            src_transform=src.transform,
            src_crs=src.crs,
            src_nodata=src.nodata,
            dst_transform=dst_transform,
            dst_crs=target_crs,
            dst_nodata=np.nan,
            resampling=Resampling.bilinear,
        )
        return dst, src.crs, src.nodata, target_crs, (west, south, east, north)


def normalize_to_png_array(elevation, height_range_m=None):
    valid = np.isfinite(elevation)
    if not np.any(valid):
        raise RuntimeError("DEM contains only nodata in the requested region")

    sampled_min = float(np.nanmin(elevation[valid]))
    sampled_max = float(np.nanmax(elevation[valid]))
    fill_value = float(np.nanmean(elevation[valid]))
    filled = elevation.copy()
    invalid_count = int(np.size(filled) - np.count_nonzero(valid))
    if invalid_count:
        print(
            "invalid_dem_cells:",
            invalid_count,
            "filled_with_mean_elevation_m:",
            f"{fill_value:.6f}",
        )
        filled[~valid] = fill_value

    if height_range_m is not None:
        if height_range_m <= 0.0:
            raise RuntimeError("--height-range-m must be greater than zero")
        height_min = sampled_min
        height_max = sampled_min + height_range_m
        clipped = np.clip(filled, height_min, height_max)
        if sampled_max > height_max:
            print(
                "warning: sampled elevation exceeds explicit height range; values will be clipped"
            )
    else:
        height_min = sampled_min
        height_max = sampled_max
        clipped = filled

    height_range = height_max - height_min
    if height_range <= 1e-9:
        print("warning: flat DEM region; writing mid-gray heightmap")
        png = np.full(elevation.shape, 128, dtype=np.uint8)
        return png, height_min, height_max, 0.0

    normalized = (clipped - height_min) / height_range
    png = np.rint(np.clip(normalized, 0.0, 1.0) * 255.0).astype(np.uint8)
    return png, height_min, height_max, height_range


def write_config(
    output_config,
    input_dem,
    input_crs,
    nodata,
    center_lat,
    center_lon,
    output_heightmap,
    size,
    width_m,
    height_m,
    height_min_m,
    height_max_m,
    height_range_m,
    terrain_pos_z,
    qgis_markers=None,
):
    config = {
        "georeference": {
            "coordinate_mode": "lonlat",
            "surface_model": "EARTH_WGS84",
            "world_frame_orientation": "ENU",
            "center_lat_deg": float(center_lat),
            "center_lon_deg": float(center_lon),
            "origin_lat_deg": float(center_lat),
            "origin_lon_deg": float(center_lon),
            # Phase 4 stores min elevation as the future world origin elevation.
            "origin_elevation_m": float(height_min_m),
            "world_yaw_deg": 0.0,
        },
        "terrain": {
            "heightmap_file": Path(output_heightmap).name,
            "size_px": int(size),
            "width_m": float(width_m),
            "height_m": float(height_m),
            "height_min_m": float(height_min_m),
            "height_max_m": float(height_max_m),
            "height_range_m": float(height_range_m),
            "terrain_pos_z_m": float(terrain_pos_z),
        },
        "source_dem": {
            "path": str(input_dem),
            "crs": str(input_crs),
            "nodata": None if nodata is None else float(nodata),
        },
    }
    if qgis_markers:
        config["qgis_markers"] = qgis_markers

    with open(output_config, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True)


def build_parser():
    pkg_root = package_root()
    parser = argparse.ArgumentParser(
        description="Create a Gazebo grayscale heightmap PNG from a DEM / GeoTIFF."
    )
    parser.add_argument("--input-dem", required=True)
    parser.add_argument("--output-heightmap", required=True)
    parser.add_argument("--output-config", required=True)
    parser.add_argument("--center-lat", required=True, type=float)
    parser.add_argument("--center-lon", required=True, type=float)
    parser.add_argument("--width-m", required=True, type=float)
    parser.add_argument("--height-m", required=True, type=float)
    parser.add_argument("--size", required=True, type=int)
    parser.add_argument("--height-range-m", type=float, default=None)
    parser.add_argument("--output-copy-to-textures", action="store_true")
    parser.add_argument("--update-terrain-sdf", action="store_true")
    parser.add_argument(
        "--terrain-sdf",
        default=str(pkg_root / "models" / "terrain" / "model.sdf"),
    )
    parser.add_argument("--terrain-pos-z", type=float, default=0.0)
    parser.add_argument("--output-qgis-markers-dir", default=None)
    parser.add_argument("--output-extent-geojson", default=None)
    parser.add_argument("--output-corners-geojson", default=None)
    parser.add_argument("--output-grid-geojson", default=None)
    parser.add_argument("--grid-step-m", type=float, default=10.0)
    parser.add_argument("--no-grid", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    pkg_root = package_root()

    input_dem = resolve_path(args.input_dem)
    output_heightmap = resolve_path(args.output_heightmap)
    output_config = resolve_path(args.output_config)
    terrain_sdf = resolve_path(args.terrain_sdf)
    textures_dir = pkg_root / "models" / "terrain" / "materials" / "textures"
    texture_copy_path = textures_dir / output_heightmap.name

    if not input_dem.is_file():
        print(f"input DEM does not exist: {input_dem}", file=sys.stderr)
        return 1
    if args.width_m <= 0.0 or args.height_m <= 0.0:
        print("--width-m and --height-m must be greater than zero", file=sys.stderr)
        return 1
    if args.size <= 1:
        print("--size must be greater than 1", file=sys.stderr)
        return 1

    marker_outputs = filter_qgis_marker_outputs(
        resolve_qgis_marker_outputs(args),
        args.width_m,
        args.height_m,
        args.grid_step_m,
    )

    if not is_gazebo_heightmap_size(args.size):
        print(
            "warning: --size is not 2^n + 1; "
            "Gazebo heightmaps typically use 257, 513, or 1025"
        )

    try:
        elevation, input_crs, nodata, target_crs, utm_bounds = read_dem_to_local_grid(
            input_dem,
            args.center_lat,
            args.center_lon,
            args.width_m,
            args.height_m,
            args.size,
        )
        png_array, height_min, height_max, height_range = normalize_to_png_array(
            elevation, args.height_range_m
        )
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    copied_to_textures = (
        str(texture_copy_path) if args.output_copy_to_textures else "false"
    )
    updated_terrain_sdf = str(terrain_sdf) if args.update_terrain_sdf else "false"

    print("input_dem:", input_dem)
    print("input_crs:", input_crs)
    print("target_utm_crs:", target_crs)
    print("center_lat/lon:", f"{args.center_lat}, {args.center_lon}")
    print("target_extent_m:", f"width={args.width_m}, height={args.height_m}")
    print("target_utm_bounds:", " ".join(f"{v:.3f}" for v in utm_bounds))
    print("output_size_px:", f"{args.size}x{args.size}")
    print("height_min_m:", f"{height_min:.6f}")
    print("height_max_m:", f"{height_max:.6f}")
    print("height_range_m:", f"{height_range:.6f}")
    print("output_heightmap:", output_heightmap)
    print("output_config:", output_config)
    print("copied_to_textures:", copied_to_textures)
    print("updated_terrain_sdf:", updated_terrain_sdf)
    print_qgis_marker_plan(
        marker_outputs, args.width_m, args.height_m, args.grid_step_m
    )

    if args.dry_run:
        print("dry_run: no files written")
        return 0

    try:
        output_heightmap.parent.mkdir(parents=True, exist_ok=True)
        output_config.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(png_array, mode="L").save(output_heightmap)
        write_config(
            output_config,
            input_dem,
            input_crs,
            nodata,
            args.center_lat,
            args.center_lon,
            output_heightmap,
            args.size,
            args.width_m,
            args.height_m,
            height_min,
            height_max,
            height_range,
            args.terrain_pos_z,
            build_qgis_markers_config(marker_outputs, args.grid_step_m),
        )
        write_qgis_marker_files(
            marker_outputs,
            args.center_lat,
            args.center_lon,
            args.width_m,
            args.height_m,
            args.size,
            args.grid_step_m,
        )

        if args.output_copy_to_textures:
            textures_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(output_heightmap, texture_copy_path)

        if args.update_terrain_sdf:
            update_terrain_sdf(
                terrain_sdf,
                output_heightmap.name,
                args.width_m,
                args.height_m,
                height_range,
                args.terrain_pos_z,
            )
    except Exception as e:
        print(f"error writing outputs: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
