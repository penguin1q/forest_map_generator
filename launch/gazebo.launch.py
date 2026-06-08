#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


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


def _append_env_paths(current_value, paths):
    existing = [path for path in current_value.split(os.pathsep) if path]
    for path in paths:
        if path not in existing:
            existing.append(path)
    return os.pathsep.join(existing)


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

    if not isinstance(gazebo_config, dict):
        raise ValueError("'gazebo' section must be a mapping")

    pkg_share = get_package_share_directory(PACKAGE_NAME)
    ros_prefix = get_package_prefix("rclcpp")

    world_name = common_config.get("world_name", "world_with_trees.world")
    world_dir = gazebo_config.get("world_dir", "worlds")
    model_paths = _resolve_model_dirs(config_file, pkg_share, gazebo_config)
    verbose = str(gazebo_config.get("verbose", 4))
    run = bool(gazebo_config.get("run", True))

    world_path = os.path.join(pkg_share, world_dir, world_name)
    ros_lib_path = os.path.join(ros_prefix, "lib")

    new_ign_path = _append_env_paths(
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""), model_paths
    )
    new_gz_path = _append_env_paths(
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""), model_paths
    )

    cmd = ["ign", "gazebo", "-v", verbose]
    if run:
        cmd.append("-r")
    cmd.append(world_path)

    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            additional_env={
                "IGN_GAZEBO_RESOURCE_PATH": new_ign_path,
                "GZ_SIM_RESOURCE_PATH": new_gz_path,
                "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ros_lib_path,
            },
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
