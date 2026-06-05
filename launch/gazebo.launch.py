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
    model_dir = gazebo_config.get("model_dir", "models")
    verbose = str(gazebo_config.get("verbose", 4))
    run = bool(gazebo_config.get("run", True))

    world_path = os.path.join(pkg_share, world_dir, world_name)
    model_path = os.path.join(pkg_share, model_dir)
    ros_lib_path = os.path.join(ros_prefix, "lib")

    current_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    new_ign_path = (
        f"{current_ign_path}:{model_path}" if current_ign_path else model_path
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
