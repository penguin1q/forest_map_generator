from setuptools import setup, find_packages
from glob import glob

package_name = "forest_map_generator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", ["worlds/world.world"]),
        (
            "share/" + package_name + "/models/terrain/heightmaps",
            glob("models/terrain/heightmaps/*.png"),
        ),
        (
            "share/" + package_name + "/models/terrain/materials/textures",
            glob("models/terrain/materials/textures/*.png"),
        ),
        (
            "share/" + package_name + "/models/terrain/materials/scripts",
            glob("models/terrain/materials/scripts/*"),
        ),
        (
            "share/" + package_name + "/models/terrain",
            ["models/terrain/model.config", "models/terrain/model.sdf"],
        ),
        (
            "share/" + package_name + "/models/road",
            ["models/road/model.config", "models/road/model.sdf"],
        ),
        ("share/" + package_name + "/models/road/meshes", glob("models/road/meshes/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.csv")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
        (
            "share/" + package_name + "/scripts/update_heightmap",
            glob("scripts/update_heightmap/*.py"),
        ),
        (
            "share/" + package_name + "/scripts/ply_to_gazebo_textured",
            glob("scripts/ply_to_gazebo_textured/*.py"),
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
