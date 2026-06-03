#!/usr/bin/env python3
import os
import cv2
import math
import rclpy
import random
from dataclasses import dataclass

import numpy as np
try:
    from stl import mesh
except ImportError:
    mesh = None
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


@dataclass
class TreeInstance:
    px: int
    py: int
    tree_type: str
    yaw: float = None
    scale: float = 1.0
    name: str = None


# Terrain helper class
class TerrainHelper:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.package_path = node.package_path
        self.heightmap_file = node.heightmap_file
        self.terrain_size_x = node.terrain_size_x
        self.terrain_size_y = node.terrain_size_y
        self.terrain_size_z = node.terrain_size_z
        self.max_slope = node.max_slope

        self.heightmap_data = None

    def load_heightmap(self):
        candidate_paths = [
            os.path.join(
                self.package_path,
                "models",
                "terrain",
                "heightmaps",
                self.heightmap_file,
            ),
            os.path.join(
                self.package_path,
                "models",
                "terrain",
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

        terrain_world_size_x = 257.0
        terrain_world_size_y = 257.0
        terrain_world_size_z = 50.0
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
        terrain_world_size_x = 257.0
        terrain_world_size_y = 257.0

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
    ):
        if yaw is None:
            yaw = random.uniform(0, 2 * math.pi)

        tree_name = name if name else f"{tree_type}_{tree_id}"
        scale_xml = ""
        if scale is not None and abs(float(scale) - 1.0) > 1e-6:
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

    def generate_trees_xml(self, trees):
        trees_xml = "\n    <!-- Auto-generated trees -->\n"
        for i, tree in enumerate(trees):
            world_x, world_y, world_z = self.pixel_to_world(tree.px, tree.py)
            trees_xml += self.create_tree_include_xml(
                tree.tree_type,
                world_x,
                world_y,
                world_z,
                i,
                yaw=tree.yaw,
                scale=tree.scale,
                name=tree.name,
            )
        trees_xml += "    <!-- End auto-generated trees -->\n"
        return trees_xml


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
            wx, wy, _ = self.pixel_to_world(tree.px, tree.py)
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

        self.declare_parameter("heightmap_file", "heightmap.png")
        self.declare_parameter("num_trees", 50)
        self.declare_parameter("tree_types", ["oak_tree", "pine_tree"])
        self.declare_parameter("terrain_size_x", 257)
        self.declare_parameter("terrain_size_y", 257)
        self.declare_parameter("terrain_size_z", 50)
        self.declare_parameter("min_tree_distance", 5.0)
        self.declare_parameter("max_slope", 30.0)
        self.declare_parameter("output_world_file", "world_with_trees_roads.world")
        self.declare_parameter("road_length", 100)
        self.declare_parameter("road_width", 1.0)
        self.declare_parameter("road_min_tree_dist", 3.0)

        self.declare_parameter("placement_mode", "random")
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

        self.heightmap_file = self.get_parameter("heightmap_file").value
        self.num_trees = self.get_parameter("num_trees").value
        self.tree_types = self.get_parameter("tree_types").value
        self.terrain_size_x = self.get_parameter("terrain_size_x").value
        self.terrain_size_y = self.get_parameter("terrain_size_y").value
        self.terrain_size_z = self.get_parameter("terrain_size_z").value
        self.min_tree_distance = self.get_parameter("min_tree_distance").value
        self.max_slope = self.get_parameter("max_slope").value
        self.output_world_file = self.get_parameter("output_world_file").value
        self.road_length = self.get_parameter("road_length").value
        self.road_width = self.get_parameter("road_width").value
        self.road_min_tree_dist = self.get_parameter("road_min_tree_dist").value

        self.placement_mode = self.get_parameter("placement_mode").value
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

    def run_generation(self):
        placement_mode = str(self.placement_mode).strip().lower()
        self.get_logger().info(f"Placement mode: {placement_mode}")

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
        else:
            self.get_logger().error(
                f"Unsupported placement_mode '{self.placement_mode}'. "
                "Use 'random' or 'orchard_grid'."
            )
            return

        self.get_logger().info(f"Generated tree count: {len(trees)}")
        trees_xml = self.tree_generator.generate_trees_xml(trees) if trees else ""

        if self.enable_road_generation:
            self.get_logger().info("Road generation enabled")
            self.road_generator = RoadGenerator(self, trees)
            roads_xml = self.road_generator.generate_roads()
        else:
            self.get_logger().info("Road generation disabled")
            roads_xml = ""

        if self.generate_final_world_file(trees_xml, roads_xml):
            self.get_logger().info("Generation completed successfully!")
            self.get_logger().info(
                f"Launch command: ros2 launch forest_map_generator gazebo.launch.py"
            )
        else:
            self.get_logger().error("Generation failed!")


def main():
    rclpy.init()
    node = ForestMapGenerator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
