#!/usr/bin/env python3
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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


def _as_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() not in ("", "0", "false", "no", "off", "disabled")


def _resolve_package_relative(package_share, value):
    value = str(value or "").strip()
    if not value:
        return ""
    if os.path.isabs(value):
        return value
    candidates = [
        os.path.join(package_share, value),
        os.path.abspath(value),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def _world_path(package_share, world_dir, world_name):
    world_name = str(world_name)
    if os.path.isabs(world_name):
        return world_name

    normalized_world = os.path.normpath(world_name)
    first_part = normalized_world.split(os.sep, 1)[0]
    if first_part == "worlds":
        return os.path.join(package_share, normalized_world)

    world_dir = str(world_dir or "worlds").strip() or "worlds"
    if os.path.isabs(world_dir):
        return os.path.join(world_dir, normalized_world)

    return os.path.join(package_share, world_dir, normalized_world)


def _heightmap_path(package_share, tree_params, static_params):
    explicit = static_params.get("heightmap_file", static_params.get("heightmap", ""))
    if explicit:
        return _resolve_package_relative(package_share, explicit)

    terrain_dir = tree_params.get("terrain_dir", "terrain")
    heightmap_file = tree_params.get("heightmap_file", "heightmap.png")
    candidates = [
        os.path.join(package_share, "models", terrain_dir, "heightmaps", heightmap_file),
        os.path.join(package_share, "models", terrain_dir, "materials", "textures", heightmap_file),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def _static_object_process(package_share, common_config, tree_params, static_params):
    enabled = _as_bool(static_params.get("enable", static_params.get("enabled", False)))
    if not enabled:
        return None

    static_file = static_params.get("file", static_params.get("static_objects_file", ""))
    if not static_file:
        raise ValueError("static_objects.enable is true, but static_objects.file is empty")

    world_name = common_config.get("world_name", "world_with_trees.world")
    world_dir = common_config.get("world_dir", "worlds")
    input_world_name = static_params.get("input_world_file", tree_params.get("output_world_file", world_name))
    output_world_name = static_params.get("output_world_file", input_world_name)

    script_path = os.path.join(
        package_share, "tools", "add_static_objects_to_world", "main.py"
    )
    static_geojson = _resolve_package_relative(package_share, static_file)
    input_world = _world_path(package_share, world_dir, input_world_name)
    output_world = _world_path(package_share, world_dir, output_world_name)

    coordinate_mode = static_params.get(
        "coordinate_mode",
        static_params.get(
            "geojson_coordinate_mode",
            tree_params.get("geojson_coordinate_mode", "local_xy"),
        ),
    )
    terrain_config = static_params.get(
        "terrain_config_file", tree_params.get("terrain_config_file", "")
    )
    terrain_config_path = _resolve_package_relative(package_share, terrain_config)
    heightmap = _heightmap_path(package_share, tree_params, static_params)

    cmd = [
        sys.executable,
        script_path,
        "--input-world",
        input_world,
        "--output-world",
        output_world,
        "--static-objects",
        static_geojson,
        "--coordinate-mode",
        str(coordinate_mode),
        "--heightmap",
        heightmap,
        "--terrain-world-size-x",
        str(tree_params.get("terrain_world_size_x", 257.0)),
        "--terrain-world-size-y",
        str(tree_params.get("terrain_world_size_y", 257.0)),
        "--terrain-size-z",
        str(tree_params.get("terrain_size_z", 50.0)),
        "--default-z-offset",
        str(static_params.get("default_z_offset", 0.0)),
    ]
    if str(coordinate_mode).strip().lower() == "lonlat":
        cmd.extend(["--terrain-config", terrain_config_path])

    return ExecuteProcess(
        cmd=cmd,
        name="add_static_objects_to_world",
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_yaml(config_file)
    package_share = get_package_share_directory(PACKAGE_NAME)

    common_config = config.get("common", {}) or {}
    gazebo_config = config.get("gazebo", {}) or {}
    tree_params = config.get("tree_generator", {}) or {}
    static_params = config.get("static_objects", {}) or {}

    if not isinstance(common_config, dict):
        raise ValueError("'common' section must be a mapping")
    if not isinstance(gazebo_config, dict):
        raise ValueError("'gazebo' section must be a mapping")
    if not isinstance(tree_params, dict):
        raise ValueError("'tree_generator' section must be a mapping")
    if not isinstance(static_params, dict):
        raise ValueError("'static_objects' section must be a mapping")

    pkg_share = get_package_share_directory(PACKAGE_NAME)
    world_name = common_config.get("world_name", "world_with_trees.world")
    world_dir = common_config.get("world_dir", "worlds")
    tree_params = dict(tree_params)
    tree_params.setdefault("model_dirs", _resolve_model_dirs(config_file, pkg_share, gazebo_config))
    tree_params.setdefault("output_world_file", _world_path(package_share, world_dir, world_name))
    tree_params.setdefault("geojson_coordinate_mode", "local_xy")
    tree_params.setdefault("terrain_config_file", "")

    tree_node = Node(
        package=PACKAGE_NAME,
        executable="forest_map_generator",
        name="forest_map_generator",
        output="screen",
        parameters=[tree_params],
    )

    actions = [tree_node]
    static_process = _static_object_process(
        package_share=package_share,
        common_config=common_config,
        tree_params=tree_params,
        static_params=static_params,
    )
    if static_process is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=tree_node,
                    on_exit=[static_process],
                )
            )
        )

    return actions


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
