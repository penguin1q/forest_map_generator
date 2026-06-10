#!/usr/bin/env python3
import csv
import json
import os
import cv2
import math
import rclpy
import random
from dataclasses import asdict, dataclass
from typing import Optional

import numpy as np
try:
    from pyproj import CRS, Transformer
except ImportError:
    CRS = None
    Transformer = None
try:
    import yaml
except ImportError:
    yaml = None
try:
    from stl import mesh
except ImportError:
    mesh = None
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


def utm_crs_from_lonlat(lon, lat):
    zone = int((float(lon) + 180.0) / 6.0) + 1
    epsg = 32600 + zone if float(lat) >= 0.0 else 32700 + zone
    return CRS.from_epsg(epsg)


class GeojsonLonLatConverter:
    def __init__(self, origin_lat, origin_lon, world_yaw_deg):
        if CRS is None or Transformer is None:
            raise RuntimeError(
                "pyproj is required for geojson_coordinate_mode=lonlat"
            )

        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.world_yaw_deg = float(world_yaw_deg)
        self.utm_crs = utm_crs_from_lonlat(self.origin_lon, self.origin_lat)
        self.to_utm = Transformer.from_crs(
            CRS.from_epsg(4326), self.utm_crs, always_xy=True
        )
        self.origin_easting, self.origin_northing = self.to_utm.transform(
            self.origin_lon, self.origin_lat
        )

    def lonlat_to_world_xy(self, lon, lat):
        easting, northing = self.to_utm.transform(float(lon), float(lat))
        east = easting - self.origin_easting
        north = northing - self.origin_northing

        # world_yaw_deg is the rotation from local ENU into the Gazebo world frame.
        # Positive yaw rotates the ENU vector clockwise in this inverse mapping.
        yaw = math.radians(self.world_yaw_deg)
        world_x = east * math.cos(yaw) + north * math.sin(yaw)
        world_y = -east * math.sin(yaw) + north * math.cos(yaw)
        return world_x, world_y


@dataclass
class TreeInstance:
    px: int
    py: int
    tree_type: str
    yaw: float = None
    scale: float = 1.0
    name: str = None
    world_x: Optional[float] = None
    world_y: Optional[float] = None
    world_z: Optional[float] = None


@dataclass
class Pose6D:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class Scale3D:
    x: float = 1.0
    y: float = 1.0
    z: float = 1.0


@dataclass
class SemanticInstance:
    id: str
    name: str
    model: str
    type: str
    pose: Pose6D
    scale: Scale3D
    semantic_parts: Optional[str]
    sdf_model_uri: str
    semantic_available: bool = True


def export_semantic_instances(instances, output_path, frame_id="orange_agv1/map"):
    data = {
        "schema_version": "0.1",
        "frame_id": frame_id,
        "generator": "forest_map_generator",
        "instances": [asdict(instance) for instance in instances],
    }
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")


def validate_semantic_instances_json(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    for key in ("schema_version", "frame_id", "instances"):
        if key not in data:
            raise ValueError("semantic_instances.json missing required key: %s" % key)
    if not isinstance(data["instances"], list):
        raise ValueError("semantic_instances.json 'instances' must be a list")

    ids = set()
    required_instance_keys = {
        "id",
        "name",
        "model",
        "type",
        "pose",
        "scale",
        "semantic_parts",
        "sdf_model_uri",
    }
    for index, instance in enumerate(data["instances"]):
        missing = required_instance_keys - set(instance.keys())
        if missing:
            raise ValueError(
                "semantic instance %d missing required keys: %s"
                % (index, ", ".join(sorted(missing)))
            )
        if instance["id"] in ids:
            raise ValueError("duplicate semantic instance id: %s" % instance["id"])
        ids.add(instance["id"])
        pose = instance["pose"]
        scale = instance["scale"]
        for key in ("x", "y", "z", "roll", "pitch", "yaw"):
            if not isinstance(pose.get(key), (int, float)):
                raise ValueError("semantic instance %s pose.%s must be numeric" % (instance["id"], key))
        for key in ("x", "y", "z"):
            if not isinstance(scale.get(key), (int, float)):
                raise ValueError("semantic instance %s scale.%s must be numeric" % (instance["id"], key))
        if instance["semantic_parts"] is not None and not isinstance(instance["semantic_parts"], str):
            raise ValueError("semantic instance %s semantic_parts must be a string or null" % instance["id"])
        if not isinstance(instance["semantic_available"], bool):
            raise ValueError("semantic instance %s semantic_available must be a boolean" % instance["id"])


# Terrain helper class
class TerrainHelper:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.package_path = node.package_path
        self.heightmap_file = node.heightmap_file
        self.terrain_size_x = node.terrain_size_x
        self.terrain_size_y = node.terrain_size_y
        self.terrain_world_size_x = node.terrain_world_size_x
        self.terrain_world_size_y = node.terrain_world_size_y
        self.terrain_size_z = node.terrain_size_z
        self.max_slope = node.max_slope
        self.terrain_dir = node.terrain_dir

        self.heightmap_data = None

    def load_heightmap(self):
        candidate_paths = [
            os.path.join(
                self.package_path,
                "models",
                self.terrain_dir,
                "heightmaps",
                self.heightmap_file,
            ),
            os.path.join(
                self.package_path,
                "models",
                self.terrain_dir,
                "materials",
                "textures",
                self.heightmap_file,
            ),
        ]
        heightmap_path = next(
            (path for path in candidate_paths if os.path.exists(path)), None
        )

        if heightmap_path is None:
            self.get_logger().error(
                "Heightmap file does not exist. Checked: "
                + ", ".join(candidate_paths)
            )
            return None

        try:
            heightmap_data = cv2.imread(heightmap_path, cv2.IMREAD_GRAYSCALE)
            if heightmap_data is None:
                self.get_logger().error(
                    f"Failed to load heightmap image from {heightmap_path}."
                )
                return None
            heightmap_data = np.array(heightmap_data, dtype=np.float32)
            self.heightmap_data = heightmap_data
            return heightmap_data
        except Exception as e:
            self.get_logger().error(f"Error loading heightmap: {e}")
            return None

    def calculate_scope(self, px, py, radius=1):
        heightmap_data = self.load_heightmap()
        if heightmap_data is None:
            return self.max_slope

        if (
            px < radius
            or px >= heightmap_data.shape[1] - radius
            or py < radius
            or py >= heightmap_data.shape[0] - radius
        ):
            return self.max_slope

        dz_dx = (heightmap_data[py, px + 1] - heightmap_data[py, px - 1]) / 2.0
        dz_dy = (heightmap_data[py + 1, px] - heightmap_data[py - 1, px]) / 2.0
        slope_angle = math.degrees(math.atan(math.sqrt(dz_dx**2 + dz_dy**2)))
        return slope_angle

    def pixel_to_world(self, px, py):
        heightmap_data = self.load_heightmap()
        if heightmap_data is None:
            self.get_logger().error("No heightmap data for coordinate conversion.")
            return 0.0, 0.0, 0.0

        terrain_world_size_x = float(self.terrain_world_size_x)
        terrain_world_size_y = float(self.terrain_world_size_y)
        terrain_world_size_z = float(self.terrain_size_z)
        terrain_world_pos_x = 0.0
        terrain_world_pos_y = 0.0
        terrain_world_pos_z = 0.0

        normalized_x = (px / (self.terrain_size_x - 1)) - 0.5
        normalized_y = (py / (self.terrain_size_y - 1)) - 0.5

        world_x = terrain_world_pos_x + normalized_x * terrain_world_size_x
        world_y = terrain_world_pos_y - normalized_y * terrain_world_size_y
        height_value = heightmap_data[py, px]
        world_z = terrain_world_pos_z + (height_value / 255.0) * terrain_world_size_z

        return world_x, world_y, world_z

    def world_to_pixel(self, world_x, world_y):
        terrain_world_size_x = float(self.terrain_world_size_x)
        terrain_world_size_y = float(self.terrain_world_size_y)

        normalized_x = (world_x / terrain_world_size_x) + 0.5
        normalized_y = -(world_y / terrain_world_size_y) + 0.5

        px = int(normalized_x * (self.terrain_size_x - 1))
        py = int(normalized_y * (self.terrain_size_y - 1))

        px = max(0, min(self.terrain_size_x - 1, px))
        py = max(0, min(self.terrain_size_y - 1, py))
        return px, py


# Tree generation class
class TreeGenerator(TerrainHelper):
    def __init__(self, node):
        super().__init__(node)

        self.num_trees = node.num_trees
        self.tree_types = node.tree_types
        self.min_tree_distance = node.min_tree_distance
        self.placement_file = node.placement_file
        self.geojson_coordinate_mode = node.geojson_coordinate_mode
        self.terrain_config_file = node.terrain_config_file
        self.tree_z_offset = node.tree_z_offset
        self.orchard_origin_x = node.orchard_origin_x
        self.orchard_origin_y = node.orchard_origin_y
        self.orchard_rows = node.orchard_rows
        self.orchard_cols = node.orchard_cols
        self.orchard_tree_spacing = node.orchard_tree_spacing
        self.orchard_row_spacing = node.orchard_row_spacing
        self.orchard_yaw_deg = node.orchard_yaw_deg
        self.orchard_jitter_xy = node.orchard_jitter_xy
        self.scale_min = node.scale_min
        self.scale_max = node.scale_max
        self.model_dirs = self._resolve_model_dirs(node.model_dirs)

    def _has_tree_types(self):
        if self.tree_types:
            return True

        self.get_logger().error("tree_types is empty. Aborting tree generation.")
        return False

    def _is_valid_pixel(self, px, py, heightmap_data, margin=10):
        return (
            margin <= px < heightmap_data.shape[1] - margin
            and margin <= py < heightmap_data.shape[0] - margin
        )

    def is_valid_tree_position(self, px, py, trees):
        if not self._is_valid_pixel(px, py, self.heightmap_data):
            return False

        slope = self.calculate_scope(px, py)
        if slope >= self.max_slope:
            return False

        for tree in trees:
            dist = math.sqrt((px - tree.px) ** 2 + (py - tree.py) ** 2)
            if dist < self.min_tree_distance:
                return False

        return True

    def _resolve_model_dirs(self, model_dirs):
        if isinstance(model_dirs, str):
            model_dirs = [model_dirs]
        if not model_dirs:
            model_dirs = ["models"]

        resolved = []
        for model_dir in model_dirs:
            if os.path.isabs(model_dir):
                path = os.path.abspath(model_dir)
            else:
                path = os.path.abspath(os.path.join(self.package_path, model_dir))
            if path not in resolved:
                resolved.append(path)
        return resolved

    def semantic_parts_uri_for_model(self, tree_type):
        for model_dir in self.model_dirs:
            semantic_path = os.path.join(model_dir, tree_type, "semantic_parts.json")
            if os.path.exists(semantic_path):
                return f"model://{tree_type}/semantic_parts.json", True
        return None, False

    def create_semantic_instance(
        self,
        instance_id,
        tree_name,
        tree_type,
        world_x,
        world_y,
        world_z,
        yaw,
        scale,
    ):
        semantic_parts, semantic_available = self.semantic_parts_uri_for_model(tree_type)
        scale_value = float(scale) if scale is not None else 1.0
        return SemanticInstance(
            id=instance_id,
            name=tree_name,
            model=tree_type,
            type="tree",
            pose=Pose6D(
                x=float(world_x),
                y=float(world_y),
                z=float(world_z),
                roll=0.0,
                pitch=0.0,
                yaw=float(yaw),
            ),
            scale=Scale3D(x=scale_value, y=scale_value, z=scale_value),
            semantic_parts=semantic_parts,
            sdf_model_uri=f"model://{tree_type}",
            semantic_available=semantic_available,
        )

    def create_tree_include_xml(
        self,
        tree_type,
        world_x,
        world_y,
        world_z,
        tree_id,
        yaw=None,
        scale=1.0,
        name=None,
        force_scale=False,
    ):
        if yaw is None:
            yaw = random.uniform(0, 2 * math.pi)

        tree_name = name if name else f"{tree_type}_{tree_id}"
        scale_xml = ""
        if scale is not None and (force_scale or abs(float(scale) - 1.0) > 1e-6):
            # Gazebo/SDF support for <scale> inside <include> can vary.
            # Model-level mesh scaling is more reliable if this is ignored.
            scale_xml = f"            <scale>{scale} {scale} {scale}</scale>\n"

        tree_xml = f'''
        <include>
            <name>{tree_name}</name>
            <uri>model://{tree_type}</uri>
            <pose>{world_x} {world_y} {world_z} 0 0 {yaw}</pose>
{scale_xml}        </include>
        '''
        return tree_xml

    def generate_trees(self):
        self.get_logger().info(
            f"Generating {self.num_trees} trees on heightmap {self.heightmap_file}..."
        )

        if not self._has_tree_types():
            return []

        heightmap_data = self.load_heightmap()
        if heightmap_data is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting tree generation."
            )
            return []

        self.get_logger().info(f"Heightmap dimensions: {heightmap_data.shape}")
        self.get_logger().info(
            f"Heightmap value range: {np.min(heightmap_data)} to {np.max(heightmap_data)}"
        )

        trees = []
        attempts = 0
        max_attempts = self.num_trees * 10

        while len(trees) < self.num_trees and attempts < max_attempts:
            attempts += 1
            px = random.randint(0, heightmap_data.shape[1] - 1)
            py = random.randint(0, heightmap_data.shape[0] - 1)

            if self.is_valid_tree_position(px, py, trees):
                tree_type = random.choice(self.tree_types)
                trees.append(TreeInstance(px=px, py=py, tree_type=tree_type))
                if len(trees) == 1:
                    world_x, world_y, world_z = self.pixel_to_world(px, py)
                    self.get_logger().info(
                        f"First tree - Pixel: ({px}, {py}), World: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
                    )
                self.get_logger().info(
                    f"Placed tree {len(trees)}/{self.num_trees} at ({px}, {py})"
                )

        self.get_logger().info(f"Tree placement completed. {len(trees)} trees placed.")
        return trees

    def generate_orchard_grid_trees(self):
        self.get_logger().info(
            "Generating orchard grid: rows=%d, cols=%d, tree_spacing=%.2fm, row_spacing=%.2fm"
            % (
                self.orchard_rows,
                self.orchard_cols,
                self.orchard_tree_spacing,
                self.orchard_row_spacing,
            )
        )

        if not self._has_tree_types():
            return []

        heightmap_data = self.load_heightmap()
        if heightmap_data is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting orchard generation."
            )
            return []

        yaw = math.radians(self.orchard_yaw_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        scale_low = min(self.scale_min, self.scale_max)
        scale_high = max(self.scale_min, self.scale_max)
        accepted = []
        skipped = 0

        for row in range(self.orchard_rows):
            for col in range(self.orchard_cols):
                local_x = col * self.orchard_tree_spacing
                local_y = row * self.orchard_row_spacing
                world_x = self.orchard_origin_x + local_x * cos_yaw - local_y * sin_yaw
                world_y = self.orchard_origin_y + local_x * sin_yaw + local_y * cos_yaw

                if self.orchard_jitter_xy > 0.0:
                    world_x += random.uniform(-self.orchard_jitter_xy, self.orchard_jitter_xy)
                    world_y += random.uniform(-self.orchard_jitter_xy, self.orchard_jitter_xy)

                px, py = self.world_to_pixel(world_x, world_y)
                if not self._is_valid_pixel(px, py, heightmap_data):
                    skipped += 1
                    continue

                slope = self.calculate_scope(px, py)
                if slope >= self.max_slope:
                    skipped += 1
                    continue

                tree_type = random.choice(self.tree_types)
                scale = random.uniform(scale_low, scale_high)
                accepted.append(
                    TreeInstance(
                        px=px,
                        py=py,
                        tree_type=tree_type,
                        yaw=yaw,
                        scale=scale,
                        name=f"{tree_type}_r{row}_c{col}",
                    )
                )

        self.get_logger().info(
            "Generated orchard grid: accepted %d trees, skipped %d invalid positions"
            % (len(accepted), skipped)
        )
        return accepted

    def _resolve_package_relative_file(self, file_value, empty_message):
        file_value = str(file_value).strip()
        if not file_value:
            self.get_logger().error(empty_message)
            return None

        if os.path.isabs(file_value):
            return file_value

        candidates = [
            os.path.join(self.package_path, file_value),
            os.path.abspath(file_value),
        ]

        install_marker = os.path.join(
            "install", "forest_map_generator", "share", "forest_map_generator"
        )
        if install_marker in self.package_path:
            workspace_root = self.package_path.split(install_marker)[0].rstrip(os.sep)
            candidates.append(
                os.path.join(workspace_root, "src", "forest_map_generator", file_value)
            )

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate

        return candidates[0]

    def _resolve_placement_file(self):
        return self._resolve_package_relative_file(
            self.placement_file,
            "placement_file is empty. Set it when placement_mode needs an input file.",
        )

    def _resolve_terrain_config_file(self):
        return self._resolve_package_relative_file(
            self.terrain_config_file,
            "terrain_config_file is empty. Set it for geojson_coordinate_mode=lonlat.",
        )

    def _csv_cell(self, row, key):
        value = row.get(key, "")
        return "" if value is None else str(value).strip()

    def _parse_optional_float(self, value, default, row_number, field_name):
        if value == "":
            return default

        try:
            return float(value)
        except ValueError:
            self.get_logger().warn(
                "CSV row %d: invalid %s '%s', using %.3f"
                % (row_number, field_name, value, default)
            )
            return default

    def _default_csv_tree_type(self, row_number):
        if self.tree_types:
            return self.tree_types[0]

        self.get_logger().error(
            "CSV row %d: tree_type is empty and tree_types parameter is empty."
            % row_number
        )
        return None

    def generate_csv_trees(self):
        csv_path = self._resolve_placement_file()
        if csv_path is None:
            return []

        self.get_logger().info(f"Reading tree placement CSV: {csv_path}")
        if not os.path.exists(csv_path):
            self.get_logger().error(f"CSV placement file does not exist: {csv_path}")
            return []

        heightmap_data = self.load_heightmap()
        if heightmap_data is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting CSV placement."
            )
            return []

        accepted = []
        skipped = 0

        try:
            with open(csv_path, newline="") as f:
                reader = csv.DictReader(f)
                fieldnames = set(reader.fieldnames or [])
                missing_headers = {"x", "y"} - fieldnames
                if missing_headers:
                    self.get_logger().error(
                        "CSV placement file is missing required headers: %s"
                        % ", ".join(sorted(missing_headers))
                    )
                    return []

                for row_number, row in enumerate(reader, start=2):
                    x_value = self._csv_cell(row, "x")
                    y_value = self._csv_cell(row, "y")
                    if x_value == "" or y_value == "":
                        skipped += 1
                        self.get_logger().warn(
                            "CSV row %d: x and y are required; skipping row."
                            % row_number
                        )
                        continue

                    try:
                        world_x = float(x_value)
                        world_y = float(y_value)
                    except ValueError:
                        skipped += 1
                        self.get_logger().warn(
                            "CSV row %d: invalid x/y values '%s', '%s'; skipping row."
                            % (row_number, x_value, y_value)
                        )
                        continue

                    px, py = self.world_to_pixel(world_x, world_y)
                    if not self._is_valid_pixel(px, py, heightmap_data):
                        skipped += 1
                        self.get_logger().warn(
                            "CSV row %d: point is outside or too close to heightmap edge; skipping row."
                            % row_number
                        )
                        continue

                    slope = self.calculate_scope(px, py)
                    if slope >= self.max_slope:
                        skipped += 1
                        self.get_logger().warn(
                            "CSV row %d: slope %.2f exceeds max_slope %.2f; skipping row."
                            % (row_number, slope, self.max_slope)
                        )
                        continue

                    tree_type = self._csv_cell(row, "tree_type")
                    if tree_type == "":
                        tree_type = self._default_csv_tree_type(row_number)
                        if tree_type is None:
                            skipped += 1
                            continue

                    _, _, heightmap_z = self.pixel_to_world(px, py)
                    z_value = self._csv_cell(row, "z")
                    if z_value == "":
                        world_z = heightmap_z
                    else:
                        try:
                            world_z = float(z_value)
                        except ValueError:
                            skipped += 1
                            self.get_logger().warn(
                                "CSV row %d: invalid z value '%s'; skipping row."
                                % (row_number, z_value)
                            )
                            continue

                    yaw = self._parse_optional_float(
                        self._csv_cell(row, "yaw"), 0.0, row_number, "yaw"
                    )
                    scale = self._parse_optional_float(
                        self._csv_cell(row, "scale"), 1.0, row_number, "scale"
                    )
                    name = self._csv_cell(row, "name")
                    if name == "":
                        name = f"{tree_type}_{len(accepted):04d}"

                    accepted.append(
                        TreeInstance(
                            px=px,
                            py=py,
                            tree_type=tree_type,
                            yaw=yaw,
                            scale=scale,
                            name=name,
                            world_x=world_x,
                            world_y=world_y,
                            world_z=world_z,
                        )
                    )
        except OSError as e:
            self.get_logger().error(f"Failed to read CSV placement file: {e}")
            return []

        self.get_logger().info(
            "CSV placement completed: accepted %d trees, skipped %d invalid rows"
            % (len(accepted), skipped)
        )
        return accepted

    def _geojson_property(self, properties, key):
        value = properties.get(key, "")
        return "" if value is None else str(value).strip()

    def _parse_geojson_optional_float(self, value, default, context, field_name):
        if value == "":
            return default, True

        try:
            return float(value), True
        except (TypeError, ValueError):
            self.get_logger().warn(
                "GeoJSON %s: invalid %s '%s'; using %.3f"
                % (context, field_name, value, default)
            )
            return default, True

    def _parse_geojson_required_float(self, value, context, field_name):
        if value == "":
            self.get_logger().warn(
                "GeoJSON %s: missing %s; skipping feature." % (context, field_name)
            )
            return None

        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                "GeoJSON %s: invalid %s '%s'; skipping feature."
                % (context, field_name, value)
            )
            return None

    def _geojson_default_tree_type(self, context):
        if self.tree_types:
            return self.tree_types[0]

        self.get_logger().warn(
            "GeoJSON %s: tree_type is empty and tree_types parameter is empty; skipping."
            % context
        )
        return None

    def _parse_geojson_xy(self, coordinates, context):
        if not isinstance(coordinates, (list, tuple)) or len(coordinates) < 2:
            self.get_logger().warn(
                "GeoJSON %s: invalid coordinates; expected [x, y]." % context
            )
            return None

        try:
            return float(coordinates[0]), float(coordinates[1])
        except (TypeError, ValueError):
            self.get_logger().warn(
                "GeoJSON %s: invalid coordinates; x and y must be numeric." % context
            )
            return None

    def _parse_geojson_lonlat(self, coordinates, context, converter):
        if not isinstance(coordinates, (list, tuple)) or len(coordinates) < 2:
            self.get_logger().warn(
                "GeoJSON %s: invalid coordinates; expected [lon, lat]." % context
            )
            return None

        try:
            lon = float(coordinates[0])
            lat = float(coordinates[1])
        except (TypeError, ValueError):
            self.get_logger().warn(
                "GeoJSON %s: invalid coordinates; lon and lat must be numeric."
                % context
            )
            return None

        return converter.lonlat_to_world_xy(lon, lat)

    def _parse_geojson_world_xy(self, coordinates, context, coordinate_mode, converter):
        if coordinate_mode == "local_xy":
            return self._parse_geojson_xy(coordinates, context)
        if coordinate_mode == "lonlat":
            return self._parse_geojson_lonlat(coordinates, context, converter)
        return None

    def _geojson_tree_type_candidates(self, properties, context):
        tree_type_value = self._geojson_property(properties, "tree_type")
        if tree_type_value == "":
            tree_type_value = self._geojson_property(properties, "tree_types")

        candidates = [
            candidate.strip()
            for candidate in tree_type_value.split(",")
            if candidate.strip()
        ]
        if candidates:
            return candidates

        fallback = self._geojson_default_tree_type(context)
        return [] if fallback is None else [fallback]

    def _choose_geojson_tree_type(self, candidates):
        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0]
        return random.choice(candidates)

    def _load_geojson_lonlat_converter(self):
        if yaml is None:
            self.get_logger().error(
                "PyYAML is required for geojson_coordinate_mode=lonlat."
            )
            return None

        terrain_config_path = self._resolve_terrain_config_file()
        if terrain_config_path is None:
            return None

        self.get_logger().info(
            "Reading terrain config: %s" % terrain_config_path
        )
        if not os.path.exists(terrain_config_path):
            self.get_logger().error(
                "terrain_config_file does not exist: %s" % terrain_config_path
            )
            return None

        try:
            with open(terrain_config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().error(
                "Failed to read terrain_config_file: %s" % e
            )
            return None

        georef = config.get("georeference", {})
        if not isinstance(georef, dict):
            self.get_logger().error(
                "terrain_config_file georeference section must be a mapping."
            )
            return None

        origin_lat = georef.get("origin_lat_deg", georef.get("center_lat_deg"))
        origin_lon = georef.get("origin_lon_deg", georef.get("center_lon_deg"))
        if origin_lat is None or origin_lon is None:
            self.get_logger().error(
                "terrain_config_file needs georeference origin_lat_deg/origin_lon_deg "
                "or center_lat_deg/center_lon_deg for lonlat mode."
            )
            return None

        world_yaw_deg = georef.get("world_yaw_deg", 0.0)
        try:
            converter = GeojsonLonLatConverter(origin_lat, origin_lon, world_yaw_deg)
        except (TypeError, ValueError, RuntimeError) as e:
            self.get_logger().error(
                "Failed to initialize GeoJSON lon/lat converter: %s" % e
            )
            return None

        self.get_logger().info(
            "GeoJSON origin: lat=%.9f, lon=%.9f"
            % (converter.origin_lat, converter.origin_lon)
        )
        self.get_logger().info("Using UTM CRS: %s" % converter.utm_crs)
        self.get_logger().info("world_yaw_deg: %.3f" % converter.world_yaw_deg)
        return converter

    def interpolate_points_along_polyline(
        self, coords, spacing, start_offset=0.0, end_offset=0.0
    ):
        segments = []
        total_length = 0.0
        for start, end in zip(coords[:-1], coords[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            if length <= 1e-9:
                continue
            segments.append((start, end, length, total_length))
            total_length += length

        usable_start = start_offset
        usable_end = total_length - end_offset
        if total_length <= 1e-9 or usable_end <= usable_start:
            return []

        def point_at(distance):
            if distance <= 0.0:
                return segments[0][0]
            if distance >= total_length:
                return segments[-1][1]

            for start, end, length, segment_start in segments:
                segment_end = segment_start + length
                if distance <= segment_end + 1e-9:
                    t = (distance - segment_start) / length
                    return (
                        start[0] + (end[0] - start[0]) * t,
                        start[1] + (end[1] - start[1]) * t,
                    )
            return segments[-1][1]

        points = []
        distance = usable_start
        while distance <= usable_end + 1e-8:
            point = point_at(min(distance, total_length))
            if not points or math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) > 1e-8:
                points.append(point)
            distance += spacing
        return points

    def _create_geojson_tree(
        self, world_x, world_y, z_value, tree_type, yaw, scale, name, context
    ):
        px, py = self.world_to_pixel(world_x, world_y)
        if not self._is_valid_pixel(px, py, self.heightmap_data):
            self.get_logger().warn(
                "GeoJSON %s: invalid tree position; outside or too close to heightmap edge."
                % context
            )
            return None

        slope = self.calculate_scope(px, py)
        if slope >= self.max_slope:
            self.get_logger().warn(
                "GeoJSON %s: invalid tree position; slope %.2f exceeds max_slope %.2f."
                % (context, slope, self.max_slope)
            )
            return None

        _, _, heightmap_z = self.pixel_to_world(px, py)
        if z_value == "":
            world_z = heightmap_z
        else:
            try:
                world_z = float(z_value)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    "GeoJSON %s: invalid z '%s'; skipping tree." % (context, z_value)
                )
                return None

        return TreeInstance(
            px=px,
            py=py,
            tree_type=tree_type,
            yaw=yaw,
            scale=scale,
            name=name,
            world_x=world_x,
            world_y=world_y,
            world_z=world_z,
        )

    def generate_geojson_trees(self):
        coordinate_mode = str(self.geojson_coordinate_mode).strip().lower()
        self.get_logger().info(f"GeoJSON coordinate mode: {coordinate_mode}")
        if coordinate_mode not in ("local_xy", "lonlat"):
            self.get_logger().error(
                "Unsupported geojson_coordinate_mode '%s'. Use 'local_xy' or 'lonlat'."
                % self.geojson_coordinate_mode
            )
            return []

        lonlat_converter = None
        if coordinate_mode == "lonlat":
            lonlat_converter = self._load_geojson_lonlat_converter()
            if lonlat_converter is None:
                return []

        geojson_path = self._resolve_placement_file()
        if geojson_path is None:
            return []

        self.get_logger().info(f"Reading GeoJSON placement file: {geojson_path}")
        if not os.path.exists(geojson_path):
            self.get_logger().error(
                f"GeoJSON placement file does not exist: {geojson_path}"
            )
            return []

        heightmap_data = self.load_heightmap()
        if heightmap_data is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting GeoJSON placement."
            )
            return []

        try:
            with open(geojson_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid GeoJSON file: {e}")
            return []
        except OSError as e:
            self.get_logger().error(f"Failed to read GeoJSON placement file: {e}")
            return []

        if data.get("type") != "FeatureCollection":
            self.get_logger().error("GeoJSON root type must be FeatureCollection.")
            return []

        crs = data.get("crs")
        if isinstance(crs, dict):
            crs_name = crs.get("properties", {}).get("name")
            if crs_name:
                self.get_logger().info(f"GeoJSON CRS field found: {crs_name}")

        accepted = []
        skipped_positions = 0
        skipped_features = 0
        features = data.get("features", [])
        if not isinstance(features, list):
            self.get_logger().error("GeoJSON features must be a list.")
            return []

        for feature_index, feature in enumerate(features):
            context = "feature %d" % feature_index
            if not isinstance(feature, dict) or feature.get("type") != "Feature":
                skipped_features += 1
                self.get_logger().warn("GeoJSON %s: malformed feature; skipping." % context)
                continue

            properties = feature.get("properties") or {}
            geometry = feature.get("geometry") or {}
            if not isinstance(properties, dict) or not isinstance(geometry, dict):
                skipped_features += 1
                self.get_logger().warn("GeoJSON %s: malformed feature; skipping." % context)
                continue

            geometry_type = geometry.get("type")
            coordinates = geometry.get("coordinates")

            tree_type_candidates = self._geojson_tree_type_candidates(
                properties, context
            )
            if not tree_type_candidates:
                skipped_features += 1
                continue
            feature_name = self._geojson_property(properties, "name") or context
            if len(tree_type_candidates) > 1:
                self.get_logger().info(
                    "Tree type candidates for %s: %s"
                    % (feature_name, ", ".join(tree_type_candidates))
                )
            yaw, _ = self._parse_geojson_optional_float(
                self._geojson_property(properties, "yaw"), 0.0, context, "yaw"
            )
            scale, _ = self._parse_geojson_optional_float(
                self._geojson_property(properties, "scale"), 1.0, context, "scale"
            )
            z_value = self._geojson_property(properties, "z")

            if geometry_type == "Point":
                xy = self._parse_geojson_world_xy(
                    coordinates, context, coordinate_mode, lonlat_converter
                )
                if xy is None:
                    skipped_positions += 1
                    continue

                if coordinate_mode == "lonlat":
                    self.get_logger().info(
                        "Point feature %s converted lon/lat to x/y: %.3f, %.3f"
                        % (feature_name, xy[0], xy[1])
                    )

                tree_type = self._choose_geojson_tree_type(tree_type_candidates)
                if tree_type is None:
                    skipped_features += 1
                    continue

                name = self._geojson_property(properties, "name")
                if name == "":
                    name = f"{tree_type}_{len(accepted):04d}"

                tree = self._create_geojson_tree(
                    xy[0], xy[1], z_value, tree_type, yaw, scale, name, context
                )
                if tree is None:
                    skipped_positions += 1
                    continue
                accepted.append(tree)
            elif geometry_type == "LineString":
                mode = self._geojson_property(properties, "mode")
                if mode != "trees_on_line":
                    skipped_features += 1
                    self.get_logger().warn(
                        "GeoJSON %s: LineString mode must be trees_on_line; skipping feature."
                        % context
                    )
                    continue

                spacing = self._parse_geojson_required_float(
                    self._geojson_property(properties, "spacing"), context, "spacing"
                )
                if spacing is None or spacing <= 0.0:
                    skipped_features += 1
                    self.get_logger().warn(
                        "GeoJSON %s: missing spacing or spacing <= 0; skipping feature."
                        % context
                    )
                    continue

                if not isinstance(coordinates, list) or len(coordinates) < 2:
                    skipped_features += 1
                    self.get_logger().warn(
                        "GeoJSON %s: invalid coordinates for LineString; skipping feature."
                        % context
                    )
                    continue

                line_coords = []
                malformed = False
                for coord_index, coord in enumerate(coordinates):
                    xy = self._parse_geojson_world_xy(
                        coord,
                        "%s coordinate %d" % (context, coord_index),
                        coordinate_mode,
                        lonlat_converter,
                    )
                    if xy is None:
                        malformed = True
                        break
                    line_coords.append(xy)
                if malformed:
                    skipped_features += 1
                    continue

                start_offset, _ = self._parse_geojson_optional_float(
                    self._geojson_property(properties, "start_offset"),
                    0.0,
                    context,
                    "start_offset",
                )
                end_offset, _ = self._parse_geojson_optional_float(
                    self._geojson_property(properties, "end_offset"),
                    0.0,
                    context,
                    "end_offset",
                )
                if start_offset < 0.0 or end_offset < 0.0:
                    skipped_features += 1
                    self.get_logger().warn(
                        "GeoJSON %s: start_offset/end_offset must be >= 0; skipping feature."
                        % context
                    )
                    continue

                candidate_points = self.interpolate_points_along_polyline(
                    line_coords, spacing, start_offset, end_offset
                )
                name_prefix = self._geojson_property(properties, "name_prefix") or "line"
                self.get_logger().info(
                    "LineString feature %s generated %d candidate points"
                    % (name_prefix, len(candidate_points))
                )
                if not candidate_points:
                    skipped_features += 1
                    self.get_logger().warn(
                        "GeoJSON %s: usable length is less than or equal to zero; skipping feature."
                        % context
                    )
                    continue

                for point_index, (world_x, world_y) in enumerate(candidate_points):
                    tree_type = self._choose_geojson_tree_type(tree_type_candidates)
                    if tree_type is None:
                        skipped_features += 1
                        break
                    name = f"{name_prefix}_{point_index:04d}"
                    tree = self._create_geojson_tree(
                        world_x,
                        world_y,
                        z_value,
                        tree_type,
                        yaw,
                        scale,
                        name,
                        "%s %s" % (context, name),
                    )
                    if tree is None:
                        skipped_positions += 1
                        continue
                    accepted.append(tree)
            else:
                skipped_features += 1
                self.get_logger().warn(
                    "GeoJSON %s: unsupported geometry type '%s'; skipping feature."
                    % (context, geometry_type)
                )

        self.get_logger().info(
            "GeoJSON placement completed: accepted %d trees, skipped %d invalid positions, skipped %d unsupported features"
            % (len(accepted), skipped_positions, skipped_features)
        )
        return accepted

    def generate_trees_xml(self, trees):
        trees_xml = "\n    <!-- Auto-generated trees -->\n"
        semantic_instances = []
        for i, tree in enumerate(trees):
            pixel_world_x, pixel_world_y, pixel_world_z = self.pixel_to_world(
                tree.px, tree.py
            )
            world_x = tree.world_x if tree.world_x is not None else pixel_world_x
            world_y = tree.world_y if tree.world_y is not None else pixel_world_y
            terrain_z = tree.world_z if tree.world_z is not None else pixel_world_z
            # tree_z_offset is a model-origin correction and is applied after
            # either terrain-derived or explicit z.
            world_z = terrain_z + float(self.tree_z_offset)
            if i == 0:
                self.get_logger().info(f"First tree terrain z: {terrain_z:.3f}")
                self.get_logger().info(
                    f"First tree final z after offset: {world_z:.3f}"
                )
            force_scale = any(
                value is not None
                for value in (tree.world_x, tree.world_y, tree.world_z)
            )
            instance_id = f"tree_{i + 1:06d}"
            tree_name = tree.name if tree.name else instance_id
            yaw = tree.yaw if tree.yaw is not None else random.uniform(0, 2 * math.pi)
            if tree.scale is not None:
                scale = tree.scale
            else:
                scale_low = min(self.scale_min, self.scale_max)
                scale_high = max(self.scale_min, self.scale_max)
                scale = random.uniform(scale_low, scale_high)
            trees_xml += self.create_tree_include_xml(
                tree.tree_type,
                world_x,
                world_y,
                world_z,
                i,
                yaw=yaw,
                scale=scale,
                name=tree_name,
                force_scale=force_scale,
            )
            semantic_instances.append(
                self.create_semantic_instance(
                    instance_id,
                    tree_name,
                    tree.tree_type,
                    world_x,
                    world_y,
                    world_z,
                    yaw,
                    scale,
                )
            )
        trees_xml += "    <!-- End auto-generated trees -->\n"
        return trees_xml, semantic_instances


# Road generation class
class RoadGenerator(TerrainHelper):
    def __init__(self, node, trees):
        super().__init__(node)

        self.node = node
        self.get_logger = node.get_logger
        self.package_path = node.package_path

        self.road_length = node.road_length
        self.road_width = node.road_width
        self.road_min_tree_dist = node.road_min_tree_dist
        self.tree_world_pos = []

        for tree in trees:
            pixel_wx, pixel_wy, _ = self.pixel_to_world(tree.px, tree.py)
            wx = tree.world_x if tree.world_x is not None else pixel_wx
            wy = tree.world_y if tree.world_y is not None else pixel_wy
            self.tree_world_pos.append((wx, wy))

        self.get_logger().info(
            "RoadGenerator initialized with road length: %.1fm, road width: %.1fm, min tree distance: %.1fm"
            % (self.road_length, self.road_width, self.road_min_tree_dist)
        )

    def find_start_end(self):
        height_map = self.load_heightmap()

        if height_map is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting road generation."
            )
            return None, None

        candidates = []
        for _ in range(100):
            wx = random.uniform(-128.0, 128.0)
            wy = random.uniform(-128.0, 128.0)
            px, py = self.world_to_pixel(wx, wy)

            slope = self.calculate_scope(px, py)
            if slope >= self.max_slope:
                continue

            valid = True
            for tx, ty in self.tree_world_pos:
                dist = math.sqrt((wx - tx) ** 2 + (wy - ty) ** 2)
                if dist < self.road_min_tree_dist:
                    valid = False
                    break
            if not valid:
                continue

            if (
                px < 10
                or px >= height_map.shape[1] - 10
                or py < 10
                or py >= height_map.shape[0] - 10
            ):
                continue

            candidates.append((wx, wy))

        if len(candidates) < 2:
            self.get_logger().error("Not enough valid candidates for road endpoints.")
            return None, None

        best_pair = None
        min_dist_diff = float("inf")
        for i in range(len(candidates)):
            for j in range(i + 1, len(candidates)):
                dist = math.hypot(
                    candidates[i][0] - candidates[j][0],
                    candidates[i][1] - candidates[j][1],
                )
                dist_diff = abs(dist - self.road_length)
                if dist_diff < min_dist_diff:
                    min_dist_diff = dist_diff
                    best_pair = (candidates[i], candidates[j])

        if best_pair is None:
            self.get_logger().error("No valid road endpoints found.")
            return None, None

        return best_pair[0], best_pair[1]

    def bresenham_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        line = []
        while True:
            line.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return zip(*line)

    def astar_path_planning(self, start, end):
        start_px, start_py = self.world_to_pixel(start[0], start[1])
        end_px, end_py = self.world_to_pixel(end[0], end[1])

        path_px = []
        line_px, line_py = self.bresenham_line(start_px, start_py, end_px, end_py)

        for px, py in zip(line_px, line_py):
            slope = self.calculate_scope(px, py)
            if slope >= self.max_slope:
                continue
            wx, wy, _ = self.pixel_to_world(px, py)
            valid = True
            for tx, ty in self.tree_world_pos:
                if math.hypot(wx - tx, wy - ty) < self.road_min_tree_dist:
                    valid = False
                    break
            if valid:
                path_px.append((px, py))

        path_world = [self.pixel_to_world(px, py) for px, py in path_px]
        return path_world

    def generate_road_xml(self, path_world):
        if len(path_world) < 2:
            return ""

        if mesh is None:
            self.get_logger().warn(
                "python module stl is not available; skipping road mesh generation. "
                "Install python3-stl or numpy-stl to enable generated roads."
            )
            return ""

        vertices = []
        road_width = self.road_width
        for i in range(len(path_world)):
            x, y, z = path_world[i]

            if i == 0:
                next_x, next_y, _ = path_world[i + 1]
                dx, dy = next_x - x, next_y - y
            elif i == len(path_world) - 1:
                prev_x, prev_y, _ = path_world[i - 1]
                dx, dy = x - prev_x, y - prev_y
            else:
                next_x, next_y, _ = path_world[i + 1]
                prev_x, prev_y, _ = path_world[i - 1]
                dx = (next_x - prev_x) / 2
                dy = (next_y - prev_y) / 2

            length = math.sqrt(dx * dx + dy * dy) + 1e-5
            dx, dy = dx / length, dy / length

            nx, ny = -dy, dx

            left_x = x + nx * (road_width / 2)
            left_y = y + ny * (road_width / 2)
            right_x = x - nx * (road_width / 2)
            right_y = y - ny * (road_width / 2)

            vertices.append((left_x, left_y, z + 0.05))
            vertices.append((right_x, right_y, z + 0.05))

        faces = []
        for i in range(len(vertices) // 2 - 1):
            idx = i * 2
            faces.append((idx, idx + 1, idx + 3))
            faces.append((idx, idx + 3, idx + 2))

        package_share_dir = get_package_share_directory("forest_map_generator")
        road_mesh_dir = os.path.join(package_share_dir, "models", "road", "meshes")
        road_stl_path = os.path.join(road_mesh_dir, "road.stl")

        road_mesh = mesh.Mesh(np.zeros(len(faces), dtype=mesh.Mesh.dtype))
        for i, face in enumerate(faces):
            for j in range(3):
                road_mesh.vectors[i][j] = vertices[face[j]]

        road_mesh.save(road_stl_path)
        self.get_logger().info(f"Road mesh saved to: {road_stl_path}")

        road_xml = """
        <!-- Auto-generated smooth road -->
        <include>
            <uri>model://road</uri>
            <name>generated_road</name>
            <pose>0 0 0 0 0 0</pose>
        </include>
        <!-- End auto-generated smooth road -->
        """

        return road_xml

    def generate_roads(self):
        self.get_logger().info("Road generation started")

        start_world, end_world = self.find_start_end()
        if start_world is None or end_world is None:
            self.get_logger().error(
                "Road generation failed: could not find valid start and end points."
            )
            return ""

        path_world = self.astar_path_planning(start_world, end_world)
        if not path_world or len(path_world) < 2:
            self.get_logger().error(
                "Road generation failed: could not find valid path."
            )
            return ""

        road_xml = self.generate_road_xml(path_world)
        self.get_logger().info(
            "Road generation completed (path points: %d)" % len(path_world)
        )

        return road_xml


# Main node
class ForestMapGenerator(Node):
    def __init__(self):
        super().__init__("forest_map_generator")
        self.get_logger().info("Forest Map Generator Node started.")

        self.declare_parameter("terrain_dir", "terrain")
        self.declare_parameter("heightmap_file", "heightmap.png")
        self.declare_parameter("num_trees", 50)
        self.declare_parameter("tree_types", ["oak_tree", "pine_tree"])
        self.declare_parameter("terrain_size_x", 257)
        self.declare_parameter("terrain_size_y", 257)
        self.declare_parameter("terrain_world_size_x", 257.0)
        self.declare_parameter("terrain_world_size_y", 257.0)
        self.declare_parameter("terrain_size_z", 50.0)
        self.declare_parameter("tree_z_offset", 0.0)
        self.declare_parameter("min_tree_distance", 5.0)
        self.declare_parameter("max_slope", 30.0)
        self.declare_parameter("output_world_file", "world_with_trees_roads.world")
        self.declare_parameter("road_length", 100)
        self.declare_parameter("road_width", 1.0)
        self.declare_parameter("road_min_tree_dist", 3.0)

        self.declare_parameter("placement_mode", "random")
        self.declare_parameter("placement_file", "")
        self.declare_parameter("geojson_coordinate_mode", "local_xy")
        self.declare_parameter("terrain_config_file", "")
        self.declare_parameter("enable_road_generation", True)
        self.declare_parameter("orchard_origin_x", -40.0)
        self.declare_parameter("orchard_origin_y", -30.0)
        self.declare_parameter("orchard_rows", 12)
        self.declare_parameter("orchard_cols", 20)
        self.declare_parameter("orchard_tree_spacing", 4.0)
        self.declare_parameter("orchard_row_spacing", 5.0)
        self.declare_parameter("orchard_yaw_deg", 0.0)
        self.declare_parameter("orchard_jitter_xy", 0.0)
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("scale_min", 1.0)
        self.declare_parameter("scale_max", 1.0)
        self.declare_parameter("model_dirs", ["models"])
        self.declare_parameter("export_semantic_instances", True)
        self.declare_parameter("semantic_instances_file", "semantic_instances.json")
        self.declare_parameter("semantic_frame_id", "orange_agv1/map")

        self.terrain_dir = self.get_parameter("terrain_dir").value
        self.heightmap_file = self.get_parameter("heightmap_file").value
        self.num_trees = self.get_parameter("num_trees").value
        self.tree_types = self.get_parameter("tree_types").value
        self.terrain_size_x = self.get_parameter("terrain_size_x").value
        self.terrain_size_y = self.get_parameter("terrain_size_y").value
        self.terrain_world_size_x = self.get_parameter("terrain_world_size_x").value
        self.terrain_world_size_y = self.get_parameter("terrain_world_size_y").value
        self.terrain_size_z = self.get_parameter("terrain_size_z").value
        self.tree_z_offset = self.get_parameter("tree_z_offset").value
        self.min_tree_distance = self.get_parameter("min_tree_distance").value
        self.max_slope = self.get_parameter("max_slope").value
        self.output_world_file = self.get_parameter("output_world_file").value
        self.road_length = self.get_parameter("road_length").value
        self.road_width = self.get_parameter("road_width").value
        self.road_min_tree_dist = self.get_parameter("road_min_tree_dist").value

        self.placement_mode = self.get_parameter("placement_mode").value
        self.placement_file = self.get_parameter("placement_file").value
        self.geojson_coordinate_mode = self.get_parameter("geojson_coordinate_mode").value
        self.terrain_config_file = self.get_parameter("terrain_config_file").value
        self.enable_road_generation = self.get_parameter("enable_road_generation").value
        self.orchard_origin_x = self.get_parameter("orchard_origin_x").value
        self.orchard_origin_y = self.get_parameter("orchard_origin_y").value
        self.orchard_rows = self.get_parameter("orchard_rows").value
        self.orchard_cols = self.get_parameter("orchard_cols").value
        self.orchard_tree_spacing = self.get_parameter("orchard_tree_spacing").value
        self.orchard_row_spacing = self.get_parameter("orchard_row_spacing").value
        self.orchard_yaw_deg = self.get_parameter("orchard_yaw_deg").value
        self.orchard_jitter_xy = self.get_parameter("orchard_jitter_xy").value
        self.random_seed = self.get_parameter("random_seed").value
        self.scale_min = self.get_parameter("scale_min").value
        self.scale_max = self.get_parameter("scale_max").value
        self.model_dirs = self.get_parameter("model_dirs").value
        self.export_semantic_instances = self.get_parameter("export_semantic_instances").value
        self.semantic_instances_file = self.get_parameter("semantic_instances_file").value
        self.semantic_frame_id = self.get_parameter("semantic_frame_id").value

        self.get_logger().info(f"terrain_dir: {self.terrain_dir}")
        self.get_logger().info(
            "heightmap image size: %d x %d px"
            % (self.terrain_size_x, self.terrain_size_y)
        )
        self.get_logger().info(
            "terrain world size: %.3f x %.3f m"
            % (self.terrain_world_size_x, self.terrain_world_size_y)
        )
        self.get_logger().info(f"terrain height range: {self.terrain_size_z} m")
        self.get_logger().info(f"tree_z_offset: {self.tree_z_offset}")

        if self.random_seed >= 0:
            random.seed(self.random_seed)
        # Use random_seed < 0 for non-deterministic placement while tuning.

        self.package_path = get_package_share_directory("forest_map_generator")

        self.tree_generator = TreeGenerator(self)
        self.road_generator = None

        self.run_generation()

    def generate_final_world_file(self, trees_xml, roads_xml):
        original_world_path = os.path.join(self.package_path, "worlds", "world.world")
        output_world_path = os.path.join(
            self.package_path, "worlds", self.output_world_file
        )

        try:
            with open(original_world_path, "r") as f:
                world_content = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read world file: {e}")
            return False

        terrain_uri = f"model://{self.terrain_dir}"
        if "<uri>model://terrain</uri>" in world_content:
            world_content = world_content.replace(
                "<uri>model://terrain</uri>", f"<uri>{terrain_uri}</uri>"
            )
            self.get_logger().info(f"World terrain URI: {terrain_uri}")
        elif terrain_uri not in world_content:
            self.get_logger().warn(
                "Base world does not contain model://terrain or %s; "
                "check that the world includes the same terrain model used by terrain_dir."
                % terrain_uri
            )

        combined_xml = roads_xml + trees_xml

        if "</world>" in world_content:
            new_world_content = world_content.replace(
                "</world>", combined_xml + "  </world>"
            )
        else:
            self.get_logger().error("Invalid world file: missing </world> tag.")
            return False

        try:
            with open(output_world_path, "w") as f:
                f.write(new_world_content)
            self.get_logger().info(f"World file saved to: {output_world_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to write world file: {e}")
            return False

    def semantic_instances_output_path(self):
        semantic_file = str(self.semantic_instances_file)
        if os.path.isabs(semantic_file):
            return semantic_file

        world_output_path = os.path.join(
            self.package_path,
            "worlds",
            str(self.output_world_file),
        )
        return os.path.join(os.path.dirname(world_output_path), semantic_file)

    def write_semantic_instances_file(self, semantic_instances):
        if not self.export_semantic_instances:
            self.get_logger().info("Semantic instance export disabled.")
            return True

        output_path = self.semantic_instances_output_path()
        try:
            export_semantic_instances(
                semantic_instances,
                output_path,
                frame_id=str(self.semantic_frame_id),
            )
            validate_semantic_instances_json(output_path)
            self.get_logger().info(
                "Semantic instances saved to: %s (%d instances)"
                % (output_path, len(semantic_instances))
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to write semantic instances file: {e}")
            return False

    def run_generation(self):
        placement_mode = str(self.placement_mode).strip().lower()
        self.get_logger().info(f"Placement mode: {placement_mode}")

        if int(self.terrain_size_x) <= 1 or int(self.terrain_size_y) <= 1:
            self.get_logger().error(
                "terrain_size_x/y must be heightmap image dimensions greater than 1."
            )
            return
        if float(self.terrain_world_size_x) <= 0.0 or float(self.terrain_world_size_y) <= 0.0:
            self.get_logger().error(
                "terrain_world_size_x/y must be terrain dimensions greater than 0 meters."
            )
            return

        if placement_mode == "random":
            trees = self.tree_generator.generate_trees()
        elif placement_mode == "orchard_grid":
            self.get_logger().info(
                "Orchard grid rows=%d cols=%d tree_spacing=%.2f row_spacing=%.2f yaw=%.2f deg"
                % (
                    self.orchard_rows,
                    self.orchard_cols,
                    self.orchard_tree_spacing,
                    self.orchard_row_spacing,
                    self.orchard_yaw_deg,
                )
            )
            trees = self.tree_generator.generate_orchard_grid_trees()
        elif placement_mode == "csv_points":
            trees = self.tree_generator.generate_csv_trees()
        elif placement_mode == "geojson":
            trees = self.tree_generator.generate_geojson_trees()
        else:
            self.get_logger().error(
                f"Unsupported placement_mode '{self.placement_mode}'. "
                "Use 'random', 'orchard_grid', 'csv_points', or 'geojson'."
            )
            return

        self.get_logger().info(f"Generated tree count: {len(trees)}")
        if trees:
            trees_xml, semantic_instances = self.tree_generator.generate_trees_xml(trees)
        else:
            trees_xml, semantic_instances = "", []

        if self.enable_road_generation:
            self.get_logger().info("Road generation enabled")
            self.road_generator = RoadGenerator(self, trees)
            roads_xml = self.road_generator.generate_roads()
        else:
            self.get_logger().info("Road generation disabled")
            roads_xml = ""

        if self.generate_final_world_file(trees_xml, roads_xml):
            semantic_ok = self.write_semantic_instances_file(semantic_instances)
            if semantic_ok:
                self.get_logger().info("Generation completed successfully!")
                self.get_logger().info(
                    f"Launch command: ros2 launch forest_map_generator gazebo.launch.py"
                )
            else:
                self.get_logger().error("Generation completed with semantic export failure.")
        else:
            self.get_logger().error("Generation failed!")


def main():
    rclpy.init()
    node = ForestMapGenerator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
