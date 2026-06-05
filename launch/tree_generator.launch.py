#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "forest_map_generator"


def _load_yaml(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config YAML does not exist: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError(f"Config YAML root must be a mapping: {path}")

    return data


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_yaml(config_file)

    common_config = config.get("common", {}) or {}
    tree_params = config.get("tree_generator", {}) or {}

    if not isinstance(tree_params, dict):
        raise ValueError("'tree_generator' section must be a mapping")

    world_name = common_config.get("world_name", "world_with_trees.world")
    tree_params = dict(tree_params)
    tree_params.setdefault("output_world_file", world_name)

    return [
        Node(
            package=PACKAGE_NAME,
            executable="forest_map_generator",
            name="forest_map_generator",
            output="screen",
            parameters=[tree_params],
        )
    ]


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "forest_map_generator.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the shared forest_map_generator YAML config file",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
