from setuptools import setup, find_packages
from glob import glob
import os

package_name = "forest_map_generator"


def collect_terrain_data_files():
    data_files = []
    for terrain_dir in sorted(
        path for path in glob("models/terrain*") if os.path.isdir(path)
    ):
        share_dir = "share/" + package_name + "/" + terrain_dir
        model_files = [
            path
            for path in [
                os.path.join(terrain_dir, "model.config"),
                os.path.join(terrain_dir, "model.sdf"),
            ]
            if os.path.exists(path)
        ]
        if model_files:
            data_files.append((share_dir, model_files))

        for subdir in [
            "heightmaps",
            "materials/textures",
            "materials/scripts",
        ]:
            files = glob(os.path.join(terrain_dir, subdir, "*"))
            if files:
                data_files.append((share_dir + "/" + subdir, files))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", ["worlds/world.world"]),
        *collect_terrain_data_files(),
        (
            "share/" + package_name + "/models/road",
            ["models/road/model.config", "models/road/model.sdf"],
        ),
        ("share/" + package_name + "/models/road/meshes", glob("models/road/meshes/*")),
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
            "share/" + package_name + "/scripts/ply_to_gazebo_textured",
            glob("scripts/ply_to_gazebo_textured/*.py"),
        ),
        (
            "share/" + package_name + "/tools/generate_tree_rows_from_areas",
            glob("tools/generate_tree_rows_from_areas/*.py"),
        ),
        (
            "share/" + package_name + "/tools/add_static_objects_to_world",
            glob("tools/add_static_objects_to_world/*.py"),
        ),
        (
            "share/" + package_name + "/models/oak_tree/materials/textures",
            glob("models/oak_tree/materials/textures/*.png"),
        ),
        (
            "share/" + package_name + "/models/oak_tree/materials/scripts",
            glob("models/oak_tree/materials/scripts/*"),
        ),
        (
            "share/" + package_name + "/models/oak_tree/meshes",
            glob("models/oak_tree/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/oak_tree",
            ["models/oak_tree/model.config", "models/oak_tree/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/pine_tree/materials/textures",
            glob("models/pine_tree/materials/textures/*.png"),
        ),
        (
            "share/" + package_name + "/models/pine_tree/materials/scripts",
            glob("models/pine_tree/materials/scripts/*"),
        ),
        (
            "share/" + package_name + "/models/pine_tree/meshes",
            glob("models/pine_tree/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/pine_tree",
            ["models/pine_tree/model.config", "models/pine_tree/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree1/meshes",
            glob("models/tree1/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree1",
            ["models/tree1/model.config", "models/tree1/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree2/meshes",
            glob("models/tree2/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree2",
            ["models/tree2/model.config", "models/tree2/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree3/meshes",
            glob("models/tree3/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree3",
            ["models/tree3/model.config", "models/tree3/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree4/meshes",
            glob("models/tree4/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree4",
            ["models/tree4/model.config", "models/tree4/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree5/meshes",
            glob("models/tree5/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree5",
            ["models/tree5/model.config", "models/tree5/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree6/meshes",
            glob("models/tree6/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree6",
            ["models/tree6/model.config", "models/tree6/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree7/meshes",
            glob("models/tree7/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree7",
            ["models/tree7/model.config", "models/tree7/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree8/meshes",
            glob("models/tree8/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree8",
            ["models/tree8/model.config", "models/tree8/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree9/meshes",
            glob("models/tree9/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree9",
            ["models/tree9/model.config", "models/tree9/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree10/meshes",
            glob("models/tree10/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree10",
            ["models/tree10/model.config", "models/tree10/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree11/meshes",
            glob("models/tree11/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree11",
            ["models/tree11/model.config", "models/tree11/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree12/meshes",
            glob("models/tree12/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree12",
            ["models/tree12/model.config", "models/tree12/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree13/meshes",
            glob("models/tree13/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree13",
            ["models/tree13/model.config", "models/tree13/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/tree14/meshes",
            glob("models/tree14/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/tree14",
            ["models/tree14/model.config", "models/tree14/model.sdf"],
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
