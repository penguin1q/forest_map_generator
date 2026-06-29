import os
from glob import glob

from setuptools import find_packages, setup

package_name = "forest_map_generator"


def collect_model_data_files(models_root="models"):
    data_files = []

    if not os.path.isdir(models_root):
        return data_files

    for root, dirs, files in os.walk(models_root):
        # __pycache__ などを除外したい場合
        dirs[:] = [d for d in dirs if d != "__pycache__"]

        file_paths = []
        for filename in files:
            if filename.endswith(".pyc"):
                continue

            src_path = os.path.join(root, filename)
            file_paths.append(src_path)

        if file_paths:
            install_dir = os.path.join("share", package_name, root)
            data_files.append((install_dir, file_paths))

    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        *collect_model_data_files("models"),
        *collect_model_data_files("models_private"),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.csv")
            + glob("config/*.geojson")
            + glob("config/sample_*.yaml"),
        ),
        (
            "share/" + package_name + "/config/qgis_layers_mid",
            glob("config/qgis_layers_mid/*.geojson"),
        ),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
        (
            "share/" + package_name + "/scripts/update_heightmap",
            glob("scripts/update_heightmap/*.py"),
        ),
        (
            "share/" + package_name + "/scripts/create_heightmap_from_dem",
            glob("scripts/create_heightmap_from_dem/*.py"),
        ),
        (
            "share/" + package_name + "/scripts/convert_tree_asset_to_gazebo",
            glob("scripts/convert_tree_asset_to_gazebo/*.py")
            + glob("scripts/convert_tree_asset_to_gazebo/*.md"),
        ),
        (
            "share/" + package_name + "/scripts/ply_to_gazebo_textured",
            glob("scripts/ply_to_gazebo_textured/*.py"),
        ),
        (
            "share/" + package_name + "/tools/generate_tree_rows_from_areas",
            glob("tools/generate_tree_rows_from_areas/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "forest_map_generator = forest_map_generator.forest_map_generator:main",
        ],
    },
)
