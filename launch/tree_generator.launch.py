#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "forest_map_generator"


def _default_config_file():
    config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME), "config")
    primary = os.path.join(config_dir, "forest_map_generator.yaml")
    sample = os.path.join(config_dir, "sample_forest_map_generator.yaml")
    return primary if os.path.exists(primary) else sample


def _load_yaml(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config YAML does not exist: {path}")

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError(f"Config YAML root must be a mapping: {path}")

    return data


def _as_list(value, field_name):
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, (list, tuple)) and all(isinstance(item, str) for item in value):
        return list(value)
    raise ValueError(f"'{field_name}' must be a string or a list of strings")


def _resolve_model_dirs(config_file, pkg_share, gazebo_config):
    if "model_dirs" in gazebo_config:
        model_dirs = _as_list(gazebo_config.get("model_dirs"), "gazebo.model_dirs")
    else:
        model_dirs = _as_list(gazebo_config.get("model_dir", "models"), "gazebo.model_dir")

    config_pkg_root = os.path.abspath(os.path.join(os.path.dirname(config_file), os.pardir))
    resolved = []
    for model_dir in model_dirs:
        if os.path.isabs(model_dir):
            path = os.path.abspath(model_dir)
        else:
            config_relative = os.path.abspath(os.path.join(config_pkg_root, model_dir))
            share_relative = os.path.abspath(os.path.join(pkg_share, model_dir))
            path = share_relative if os.path.exists(share_relative) else config_relative
        if path not in resolved:
            resolved.append(path)
    return resolved


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_yaml(config_file)

    common_config = config.get("common", {}) or {}
    gazebo_config = config.get("gazebo", {}) or {}
    tree_params = config.get("tree_generator", {}) or {}

    if not isinstance(gazebo_config, dict):
        raise ValueError("'gazebo' section must be a mapping")
    if not isinstance(tree_params, dict):
        raise ValueError("'tree_generator' section must be a mapping")

    pkg_share = get_package_share_directory(PACKAGE_NAME)
    world_name = common_config.get("world_name", "world_with_trees.world")
    tree_params = dict(tree_params)
    tree_params.setdefault("model_dirs", _resolve_model_dirs(config_file, pkg_share, gazebo_config))
    tree_params.setdefault("output_world_file", world_name)
    tree_params.setdefault("geojson_coordinate_mode", "local_xy")
    tree_params.setdefault("terrain_config_file", "")

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
    default_config_file = _default_config_file()

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
