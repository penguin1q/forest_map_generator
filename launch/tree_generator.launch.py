#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="forest_map_generator",
                executable="forest_map_generator",
                name="forest_map_generator",
                output="screen",
                parameters=[
                    {
                        "heightmap_file": "heightmap.png",
                        "num_trees": 200,
                        "tree_types": [
                            "tree1",
                            "tree2",
                            "tree3",
                            "tree4",
                            "tree5",
                            "tree6",
                            "tree7",
                            "tree8",
                            "tree9",
                            "tree10",
                            "tree11",
                            "tree12",
                            "tree13",
                            "tree14",
                        ],
                        "terrain_size_x": 257,
                        "terrain_size_y": 257,
                        "terrain_size_z": 50,
                        "min_tree_distance": 5.0,
                        "max_slope": 30.0,
                        "output_world_file": "world_with_trees.world",
                        "placement_mode": "orchard_grid",
                        "enable_road_generation": False,
                        "orchard_origin_x": -40.0,
                        "orchard_origin_y": -30.0,
                        "orchard_rows": 12,
                        "orchard_cols": 20,
                        "orchard_tree_spacing": 4.0,
                        "orchard_row_spacing": 5.0,
                        "orchard_yaw_deg": 10.0,
                        "orchard_jitter_xy": 0.3,
                        "random_seed": 0,
                        "scale_min": 0.9,
                        "scale_max": 1.1,
                    }
                ],
            ),
        ]
    )
