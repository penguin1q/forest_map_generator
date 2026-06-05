# forest_map_generator (ROS2 Humble + Gazebo Fortress)

A ROS 2 package for generating forest simulation environments in Gazebo Fortress, including
terrain heightmaps, procedural tree placement, and road mesh generation.

<p align="center">
  <img src="docs/images/gazebo_overview.png" width="850">
</p>

---


## Table of Contents

- [Overview](#overview)
- [Repository Layout](#repository-layout)
- [Key Components](#key-components)
  - [1. ForestMapGenerator (ROS 2 Node)](#1-forestmapgenerator-ros-2-node)
  - [2. TerrainHelper (Terrain Abstraction Layer)](#2-terrainhelper-terrain-abstraction-layer)
  - [3. TreeGenerator](#3-treegenerator)
  - [4. RoadGenerator (Road Mesh + Simple Path Planning)](#4-roadgenerator-road-mesh--simple-path-planning)
  - [5. update_heightmap script (Heightmap → Terrain SDF Update)](#5-update_heightmap-script-heightmap--terrain-sdf-update)
  - [6. ply_to_gazebo_textured pipeline (PLY → Gazebo Tree Model)](#6-ply_to_gazebo_textured-pipeline-ply--gazebo-tree-model)

---

## Overview

This project provides a complete pipeline to build a forest scene for Gazebo from:
1) a terrain heightmap (PNG) used by the `terrain` model, and  
2) tree assets (either built-in models such as `oak_tree` / `pine_tree`, or textured meshes generated from point clouds).

The core workflow is:

1) **Acquire a terrain heightmap**
   - Select a geographic region of interest and export a digital elevation model (DEM) as a heightmap.
   - `scripts/create_heightmap_from_dem/main.py` can crop DEM / GeoTIFF data into a Gazebo heightmap and optionally export QGIS marker GeoJSON files for the same real-world extent.
   - Public DEM-to-heightmap services can also be used to obtain real-world terrain heightmaps for almost any location worldwide (e.g., https://dx3377.com/dem/heightmap).
   - Export/download the terrain as a heightmap image and ensure it meets the Gazebo format requirements below.

2) **Prepare a Gazebo-compatible heightmap**
   - Convert or export the heightmap as a single-channel (grayscale) PNG.
   - Ensure the heightmap is square and follows Gazebo heightmap requirements (`N × N`, where `N = 2^k + 1`).
   - Use 8-bit grayscale (`0–255`) unless a higher bit-depth workflow is explicitly required.
   - Set `terrain_size_x` and `terrain_size_y` to match the actual heightmap resolution.  
     These values are automatically reported during heightmap loading and terrain/SDF update,
     and can be verified from the printed image dimensions in the console output.

3) **Update the terrain model**
   - `scripts/update_heightmap/main.py` updates `models/terrain/model.sdf`, including:
     - heightmap URI
     - heightmap size and vertical scale
     - texture blending parameters
     - terrain pose
   - The script also copies the heightmap PNG into:
     - `models/terrain/materials/textures/`
   - This ensures the heightmap is discoverable by Gazebo at runtime.

4) **Generate the forest world**
   - `tree_generator.launch.py` reads a shared YAML config file and starts `forest_map_generator/forest_map_generator.py`.
   - The node generates a new `.world` file by inserting:
     - randomly placed or file-driven tree `<include>` blocks
     - an optional generated road mesh (`models/road/meshes/road.stl`) and the corresponding road `<include>`
   - `gazebo.launch.py` reads the same YAML config so Gazebo opens the generated world named in `common.world_name`.
   - Tree and road placement are evaluated directly on the heightmap using shared terrain logic.

5) **Create Gazebo-ready tree models from point clouds (optional)**
   - `scripts/ply_to_gazebo_textured/main.py` converts colored `.ply` point clouds into Gazebo-ready tree models under `models/<tree_name>/`, including:
     - `meshes/tree_mesh.dae` (visual mesh)
     - `meshes/tree_collision.stl` (collision mesh)
     - `meshes/<tree_name>_albedo.png` (texture baked via Blender)

This pipeline enables reproducible construction of large-scale forest environments in Gazebo from either synthetic or real-world terrain data.


The package is structured as an `ament_python` ROS 2 package and is intended for research workflows where repeatable generation of natural outdoor scenes is required.

---

## Repository Layout


```text
forest_map_generator/
├── forest_map_generator/
│   └── forest_map_generator.py        # ROS 2 node (tree & road generation)
│
├── scripts/
│   ├── create_heightmap_from_dem/
│   │   └── main.py                    # DEM / GeoTIFF → heightmap + QGIS markers
│   │
│   ├── update_heightmap/
│   │   └── main.py                    # Heightmap → terrain SDF update
│   │
│   └── ply_to_gazebo_textured/
│       ├── main.py                    # PLY → Gazebo tree model pipeline
│       └── bake_vcol_to_texture.py    # Blender texture baking (called by main.py)
│
├── models/
│   ├── terrain/                       # Heightmap-based terrain model
│   ├── road/                          # Generated road model
│   ├── oak_tree/                      # Predefined tree model
│   ├── pine_tree/                     # Predefined tree model
│   └── tree*/                         # Auto-generated tree instances
│
├── worlds/
│   └── world.world                    # Base Gazebo world
│
├── launch/
│   ├── gazebo.launch.py
│   └── tree_generator.launch.py
│
├── config/
│   └── sample_forest_map_generator.yaml  # Shared YAML config example
│
└── docs/
    └── images/
        └── gazebo_overview.png

```

---

## Key Components

- 'ForestMapGenerator (ROS 2 node)': generates a new world file by injecting trees and roads into a base world.

- 'TerrainHelper': shared utility class for heightmap loading, pixel–world coordinate conversion, and terrain slope computation.

- 'TreeGenerator': slope-aware random tree placement on the heightmap, built on top of TerrainHelper.

- 'RoadGenerator': generates a smooth road mesh (road.stl) while respecting terrain slope and minimum clearance from trees.

- 'update_heightmap script': updates terrain SDF parameters and ensures the heightmap image is placed in the correct model path for Gazebo.

- 'ply_to_gazebo_textured pipeline': converts colored point clouds into textured Gazebo-ready meshes using Open3D and Blender texture baking.


### YAML Launch Configuration

Runtime settings are kept in a shared YAML file. The repository provides `config/sample_forest_map_generator.yaml`; copy it to a local config before editing:

```bash
cp config/sample_forest_map_generator.yaml config/forest_map_generator.yaml
```

Use the same file for generation and Gazebo:

```bash
export FOREST_CONFIG=/path/to/config/forest_map_generator.yaml
ros2 launch forest_map_generator tree_generator.launch.py config_file:=$FOREST_CONFIG
ros2 launch forest_map_generator gazebo.launch.py config_file:=$FOREST_CONFIG
```

Important sections:

- `common.world_name`: generated world filename shared by tree generation and Gazebo.
- `tree_generator`: ROS 2 node parameters for heightmap, terrain geometry, placement mode, and road generation.
- `tree_generator.terrain_dir`: terrain model directory under `models/`; the generated world uses `model://<terrain_dir>`.
- `gazebo`: world/model directories and Gazebo verbosity/run options. `gazebo.model_dir` is the parent models directory, normally `models`.

### 1. ForestMapGenerator (ROS 2 Node)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**

Primary ROS 2 node that procedurally generates a forest simulation world by injecting trees (and optionally roads) into a base Gazebo world.
The node samples valid placements directly on the terrain heightmap, converts heightmap pixels into world-frame poses, and writes a new .world file under worlds/.

**Execution Flow**
1. Load the terrain heightmap and compute local slope information
2. Sample valid tree positions subject to slope and distance constraints
3. Convert heightmap pixels to world-frame poses
4. Inject generated tree instances into a new Gazebo world file

**Launch Commands**

Prepare a local YAML config first:

```bash
cp config/sample_forest_map_generator.yaml config/forest_map_generator.yaml
```

Generate the world:

```bash
export FOREST_CONFIG=/path/to/forest_map_generator/config/forest_map_generator.yaml
ros2 launch forest_map_generator tree_generator.launch.py config_file:=$FOREST_CONFIG
```

Open the generated world in Gazebo using the same config:

```bash
ros2 launch forest_map_generator gazebo.launch.py config_file:=$FOREST_CONFIG
```

Both launch files read the same YAML. `tree_generator.launch.py` uses the `tree_generator` section, and `gazebo.launch.py` uses `common` and `gazebo`. The node writes a generated world file to the package `worlds/` directory; by default the filename comes from `common.world_name`.

**Parameters**

| Parameter | Type | Description |
|----------|------|-------------|
| `terrain_dir` | `string` | Terrain model directory under `models/`. Also used as the generated world terrain model URI, e.g. `model://terrain`. |
| `heightmap_file` | `string` | Heightmap image filename under `models/<terrain_dir>/heightmaps/`. Used for terrain elevation lookup and slope computation. |
| `num_trees` | `int` | Number of trees to generate and inject into the world. |
| `tree_types` | `list[string]` | List of Gazebo model names available under `models/` (e.g., `tree1`–`tree14`). A random type is selected per placement. |
| `terrain_size_x` | `int` | Heightmap resolution in X (pixels). Must match the heightmap image width. |
| `terrain_size_y` | `int` | Heightmap resolution in Y (pixels). Must match the heightmap image height. |
| `terrain_world_size_x` | `float` | Terrain width in Gazebo/world meters. |
| `terrain_world_size_y` | `float` | Terrain depth in Gazebo/world meters. |
| `terrain_size_z` | `float` | Terrain vertical height range in meters used to convert heightmap values to world Z. |
| `tree_z_offset` | `float` | Model-origin correction applied to every generated tree pose Z. |
| `min_tree_distance` | `float` | Minimum allowed distance (meters) between any two trees. |
| `max_slope` | `float` | Maximum allowed slope (degrees) for valid placements. Trees are rejected on steep terrain. |
| `output_world_file` | `string` | Output world filename written to `worlds/` (e.g., `world_with_trees.world`). |

Note: Several of the parameters above are automatically printed during heightmap loading and SDF update for verification and reproducibility.  
These outputs will be explained in detail in a later section.

**Output**
```text
worlds/<output_world_file>
```

What is written into the world

- A list of Gazebo `<include>` blocks, one per generated tree instance

- Each instance includes a randomized yaw for visual diversity

- Tree placement is slope-aware and respects minimum spacing constraints

**Assumptions**
- The terrain model and heightmap are pre-loaded in Gazebo
- All tree models listed in `tree_types` exist under `models/`

**Example YAML Configuration**

`tree_generator.launch.py` now reads parameters from YAML instead of keeping a large parameter dictionary in the launch file.

```yaml
common:
  world_name: world_with_trees.world

gazebo:
  run: true
  verbose: 4
  world_dir: worlds
  model_dir: models

tree_generator:
  terrain_dir: terrain
  heightmap_file: orchard_heightmap.png
  num_trees: 200
  tree_types: [tree1, tree2, tree3, tree4, tree5, tree6, tree7]

  # Image dimensions in pixels.
  terrain_size_x: 257
  terrain_size_y: 257

  # Terrain dimensions in Gazebo/world meters.
  terrain_world_size_x: 257.0
  terrain_world_size_y: 257.0
  terrain_size_z: 12.688472747802734
  tree_z_offset: 0.0

  min_tree_distance: 5.0
  max_slope: 30.0
  placement_mode: random
  placement_file: ""
  geojson_coordinate_mode: local_xy
  terrain_config_file: ""
  enable_road_generation: false
```

`output_world_file` can still be set under `tree_generator`, but if it is omitted the launch file injects `common.world_name` automatically. `terrain_dir` should match a terrain model directory under `models/`; the generated world replaces the default `model://terrain` include with `model://<terrain_dir>`.

**GeoJSON placement modes**

`placement_mode: geojson` supports `Point` and `LineString` features.

- `geojson_coordinate_mode: local_xy`: coordinates are interpreted as Gazebo world `x, y`.
- `geojson_coordinate_mode: lonlat`: coordinates are interpreted as `[longitude, latitude]` and converted to local ENU `x, y` using `terrain_config_file`.

Example lon/lat run command:

```bash
ros2 run forest_map_generator forest_map_generator --ros-args \
  -p placement_mode:=geojson \
  -p placement_file:=config/qgis_layers/tree_rows.geojson \
  -p geojson_coordinate_mode:=lonlat \
  -p terrain_config_file:=config/terrain_config.yaml \
  -p enable_road_generation:=false \
  -p output_world_file:=world_qgis_lonlat_test.world
```

`tree_type` and `tree_types` GeoJSON properties can contain comma-separated candidates, such as `"tree1,tree2,tree3"`. One candidate is randomly selected per generated tree.

**Reproducibility**  
For fixed parameters and heightmap input, the generation process is stochastic due to randomized tree placement, orientation, and type selection.  
A fixed random seed is planned to be introduced to enable reproducible map generation for benchmarking and evaluation.

### 2. TerrainHelper (Terrain Abstraction Layer)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**

`TerrainHelper` is a shared utility class that encapsulates all terrain-related operations, providing a consistent abstraction over the heightmap-based terrain model used in Gazebo.

It serves as the geometric and physical foundation for both tree and road generation by:

- loading and validating the terrain heightmap,

- computing local terrain slope,

- converting between heightmap pixel coordinates and world-frame coordinates.

By centralizing these operations, `TerrainHelper` ensures that terrain assumptions (scale, orientation, slope limits) remain consistent across different procedural components.

**Responsibilities**

- **Heightmap loading**
  - Loads grayscale PNG heightmaps from  
    `models/<terrain_dir>/heightmaps/<heightmap_file>`
  - Converts pixel values into floating-point elevation data
  - Reports image dimensions and value range for verification

- **Slope computation**
  - Estimates local terrain slope using finite differences on the heightmap
  - Computes slope angle in degrees from heightmap gradients
  - Enforces a maximum allowable slope (`max_slope`) for placement validity

- **Coordinate conversion**
  - Maps heightmap pixel coordinates `(px, py)` to Gazebo world coordinates `(x, y, z)`
  - Converts world-frame coordinates back to heightmap pixels
  - Maintains a consistent terrain reference frame shared by all generators

**Methods**

| Method | Description |
|-------|-------------|
| `load_heightmap()` | Loads the grayscale heightmap image from disk and caches it as a NumPy array for reuse. |
| `calculate_scope(px, py)` | Computes the local terrain slope angle (degrees) at the specified heightmap pixel using finite differences. |
| `pixel_to_world(px, py)` | Converts heightmap pixel coordinates to Gazebo world-frame coordinates `(x, y, z)` using terrain scale parameters. |
| `world_to_pixel(x, y)` | Converts Gazebo world-frame `(x, y)` coordinates back to heightmap pixel indices. |

**Design Notes**

- Terrain dimensions and scaling are explicitly parameterized using:
  - `terrain_size_x`, `terrain_size_y` — heightmap image resolution in pixels
  - `terrain_world_size_x`, `terrain_world_size_y` — terrain extents in Gazebo/world meters
  - `terrain_size_z` — vertical height range in meters
- All slope checks for trees and roads rely on the same slope computation logic.
- Boundary regions of the heightmap are conservatively rejected to avoid invalid gradient estimates.

**Consumers**

`TerrainHelper` is inherited by:

- `TreeGenerator` — for slope-aware tree placement and pixel-to-world coordinate conversion
- `RoadGenerator` — for slope-constrained path planning and road mesh generation

This design avoids duplicated terrain logic and ensures that all procedural elements are generated under identical terrain constraints.

**Assumptions**

- The heightmap is a single-channel (grayscale) PNG compatible with Gazebo heightmap terrain models.
- Heightmap resolution matches `terrain_size_x × terrain_size_y`.
- The terrain model is centered at the world origin with symmetric extents.


### 3. TreeGenerator

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**
TreeGenerator is responsible for procedural tree placement on the terrain heightmap and generating the corresponding Gazebo `<include>` blocks that will be injected into the output .world file. It inherits TerrainHelper so that placement validation and pixel-to-world coordinate conversion are consistent with the shared terrain model assumptions.

**Inputs (ROS 2 Parameters)**

| Parameter | Type | Description |
|----------|------|-------------|
| `num_trees` | `int` | Number of tree instances to place. |
| `tree_types` | `list[string]` | List of Gazebo model names under models/ (e.g., oak_tree, pine_tree). A random type is selected per placement. |
| `min_tree_distance` | `float` | Minimum spacing constraint between any two placed trees. |
| `max_slope` | `float` | Maximum allowable terrain slope in degrees. Candidate points with slope >= max_slope will be rejected. |
| `terrain_dir` | `string` | Terrain model directory under models/. |
| `heightmap_file` | `string` | Heightmap image filename under models/<terrain_dir>/heightmaps/. Used for elevation lookup and slope evaluation. |
| `terrain_size_x` | `int` | Heightmap resolution in X (pixels). |
| `terrain_size_y` | `int` | Heightmap resolution in Y (pixels). |
| `terrain_world_size_x` | `float` | Terrain width in Gazebo/world meters. |
| `terrain_world_size_y` | `float` | Terrain depth in Gazebo/world meters. |
| `terrain_size_z` | `float` | Terrain vertical scale used to map heightmap values into world-frame Z. |
| `tree_z_offset` | `float` | Model-origin correction applied to every generated tree pose Z. |

**Execution Flow**
1) Load the heightmap as a grayscale image and convert it to float32
2) Randomly sample candidate pixel coordinates (px, py)
3) Validate each candidate using boundary margin checks, local slope check, and minimum distance constraint to previously placed trees
4) Convert accepted pixel coordinates into world coordinates (x, y, z)
5) Generate one Gazebo `<include>` block per accepted placement
6) Return the full list of accepted trees and the concatenated XML snippet

**Key Methods**

| Method | Description |
|------|-------------|
| `generate_trees()` | Main placement loop. Randomly samples pixels and accepts valid tree positions until num_trees is reached or attempts exceed the limit. |
| `is_valid_tree_position(px, py, trees)` | Validity check for a candidate pixel: border constraints, slope check, and distance check against existing trees. |
| `create_tree_include_xml(tree_type, world_x, world_y, world_z, tree_id)` | Creates one `<include>` block for the selected tree type at a computed world pose, with randomized yaw. |
| `generate_trees_xml(trees)` | Converts all accepted tree placements into a single XML string to inject into the output world. |

**Output**
```text
worlds/<output_world_file>
```

What is written into the world:
- A list of Gazebo `<include>` blocks, one per generated tree instance
- Each instance includes a randomized yaw for visual diversity
- Tree placement is slope-aware and respects minimum spacing constraints

**Customization Guide (How to Define Your Own Tree Placement Logic)**
To customize the tree generation logic, the recommended approach is to modify or extend:

1) **Candidate sampling strategy**
- Edit: TreeGenerator.generate_trees()
- Example: restrict sampling to a region, or apply weighted sampling

2) **Validity constraints**
- Edit: TreeGenerator.is_valid_tree_position(px, py, trees)
- Example: add elevation limits, biome masks, or per-type slope limits

3) **Model selection / pose formatting**
- Edit: TreeGenerator.create_tree_include_xml(...)
- Example: weighted tree type sampling, fixed yaw, custom naming rules


### 4. RoadGenerator (Road Mesh + Simple Path Planning)

**Location**
```text
forest_map_generator/forest_map_generator.py
```

**Role**
RoadGenerator generates a smooth road mesh (`road.stl`) and injects a road `<include>` block into the output `.world` file. It evaluates road endpoints and the resulting path directly on the terrain heightmap, reusing `TerrainHelper` for slope checks and coordinate conversion.

**Main Parameters**
- **road_length** (float): target road length in meters used when selecting endpoints
- **road_width** (float): road mesh width in meters
- **road_min_tree_dist** (float): minimum clearance from any generated tree (world-frame distance)
- **max_slope** (float): maximum allowable slope (deg) for road samples

**Execution Flow**
1) Convert generated trees from pixel coordinates to world-frame `XY` for clearance checks
2) Sample candidate road endpoints in world coordinates and filter by slope and tree clearance
3) Select a best endpoint pair whose distance is closest to `road_length`
4) Generate a path between endpoints using a simple line-based sampling strategy
5) Expand the path into a ribbon mesh, triangulate it, and save as `STL`
6) Return a road `<include>` XML block for insertion into the generated world

**Key Methods**

| Method | Description |
|------|-------------|
| `generate_roads()` | Orchestrates endpoint selection, path generation, mesh export, and returns the road injection XML. |
| `find_start_end()` | Samples endpoint candidates in world coordinates, filters by slope and tree clearance, and selects the pair closest to `road_length`. |
| `bresenham_line(x0, y0, x1, y1)` | Produces integer pixel coordinates along a straight line between start and end pixels. |
| `astar_path_planning(start, end)` | Current implementation uses a straight-line sample set and filters points by slope and tree clearance, then converts accepted pixels into world-frame points. |
| `generate_road_xml(path_world)` | Builds a simple ribbon mesh from `path_world`, saves `models/road/meshes/road.stl`, and returns the Gazebo road `<include>` block. |

**Output**
- Road mesh:
```text
models/road/meshes/road.stl
```

- World injection:
```text
<include>
  <uri>model://road</uri>
  <name>generated_road</name>
  <pose>0 0 0 0 0 0</pose>
</include>
```

**Planned Improvements**
RoadGenerator currently uses a simplified straight-line sampling approach for path generation and a minimal ribbon mesh construction. Future work is planned in two areas:

1) **Road mesh / model fidelity**
- add thickness and collision-friendly geometry
- add UVs and textures for visual realism
- smoother curvature control and better handling of sharp turns
- optional elevation smoothing to reduce road waviness on rough terrain

2) **Path planning algorithm**
- replace the current line-filtering approach with a real grid-based `A*` (or `Dijkstra`) using:
  - slope cost
  - distance-to-tree cost
  - optional obstacle masks
- improve continuity by post-smoothing the path (e.g., spline fitting) while maintaining constraints


### 5. update_heightmap script (Heightmap → Terrain SDF Update)

**Location**
```text
scripts/update_heightmap/main.py
```

**Role**
This script updates the terrain model SDF (`models/<terrain_dir>/model.sdf`) to reference a new heightmap, and ensures the heightmap image is copied into the Gazebo-discoverable model path under `models/<terrain_dir>/materials/textures/`.

It edits the `<heightmap>` block(s) in the SDF, updating:
- `<uri>`: points to the heightmap under `model://<terrain_dir>/materials/textures/`
- `<size>`: sets `(width, height, height_range)`
- `<blend>`: ensures exactly two blend entries and updates their `min_height` and `fade_dist`
- `<pos>`: sets terrain heightmap pose offset

**CLI Arguments**
- `--heightmap`: input heightmap PNG path
- `--height_range`: vertical scale used in `<size>` (meters)
- `--blend1_min` / `--blend1_fade`: texture blend layer 1 parameters
- `--blend2_min` / `--blend2_fade`: texture blend layer 2 parameters
- `--pos_x` / `--pos_y` / `--pos_z`: heightmap pose offset written into `<pos>`
- `--terrain_sdf`: target terrain SDF path (default: `models/terrain/model.sdf`)
- `--target_dir`: copy destination for heightmap (default: `models/terrain/materials/textures/`)
- `--dry_run`: print computed values without modifying files

**Execution Flow**
1) Resolve package paths and defaults
2) Validate heightmap file exists and read image dimensions `(w, h)`
3) Construct SDF fields:
   - `uri = model://<terrain_dir>/materials/textures/<heightmap_name>`
   - `size = "w h height_range"`
   - `pos = "pos_x pos_y pos_z"`
4) Parse the SDF and locate all `<heightmap>` blocks
5) For each `<heightmap>`:
   - set `<uri>`, `<size>`, `<pos>`
   - enforce exactly two `<blend>` entries and update blend parameters
6) If not `--dry_run`:
   - create a timestamped backup of the original SDF
   - copy the heightmap PNG into `target_dir`
   - write the updated SDF back to disk

**Outputs**
- Heightmap copied to:
```text
models/terrain/materials/textures/<heightmap_name>.png
```

- Terrain SDF updated:
```text
models/terrain/model.sdf
```

- Backup created:
```text
models/terrain/model.sdf.bak_<YYYYMMDD_HHMMSS>
```

**Notes**
- The script edits every `<heightmap>` block found in the SDF (supports multiple occurrences).
- The script prints the final computed `uri`, `size`, `pos`, and copy destination for verification before writing.


### QGIS marker GeoJSON helper

`scripts/create_heightmap_from_dem/main.py` can write helper GeoJSON files for QGIS when DEM-derived terrain is generated.
Pass `--output-qgis-markers-dir config/qgis_markers` to create:

```text
config/qgis_markers/terrain_extent.geojson
config/qgis_markers/terrain_corners.geojson
config/qgis_markers/terrain_grid.geojson
```

The marker coordinates are standard `[longitude, latitude]`. They visualize the generated terrain extent, corner/center points, and local Gazebo coordinate grid in QGIS.
These files are for visualization and editing only; the ROS node does not consume the marker files directly.
Use `--no-grid` to skip grid generation, or `--grid-step-m <meters>` to change the grid spacing.
When markers are generated, `terrain_config.yaml` records their paths under `qgis_markers`.


### 6. ply_to_gazebo_textured pipeline (PLY → Gazebo Tree Model)

**Location**
```text
scripts/ply_to_gazebo_textured/
├── main.py
└── bake_vcol_to_texture.py
```

**Role**
This pipeline converts colored point clouds (`.ply`) into Gazebo-ready tree models under `models/<tree_name>/`. It produces a textured visual mesh (`.dae` + albedo `.png`) and a simplified collision mesh (`.stl`).

`main.py` handles point cloud processing, mesh reconstruction, simplification, and collision generation. Texture baking and Collada export are delegated to Blender via `bake_vcol_to_texture.py`.

**Inputs**
- Point clouds placed under:
```text
scripts/ply_to_gazebo_textured/trees_ply/*.ply
```

- Requires `open3d` and a working Blender CLI (`blender` in `PATH` or configured via `BLENDER_BIN`).

**Execution Flow**
1) Load `.ply` point cloud using `open3d`
2) Normalize geometry:
   - center `XY` at the cloud centroid
   - shift `Z` so the base is at `Z=0`
3) Downsample to `TARGET_POINTS` if needed
4) Estimate normals and run Poisson surface reconstruction
5) Remove low-density vertices and clean mesh topology
6) Transfer vertex colors from point cloud to mesh (kNN weighted average), or paint a uniform fallback color
7) Generate visual mesh:
   - simplify to `VISUAL_MAX_TRIANGLES`
   - export an intermediate colored visual `.ply`
8) Run Blender texture baking:
   - bake vertex colors into a texture image (`*_albedo.png`)
   - generate UVs if missing
   - export textured Collada (`tree_mesh.dae`)
9) Generate collision mesh:
   - simplify to `COLLISION_MAX_TRIANGLES`
   - export collision geometry as `tree_collision.stl`

**Outputs**
For each input `<tree_name>.ply`, the pipeline creates:
```text
models/<tree_name>/
└── meshes/
    ├── tree_mesh.dae
    ├── tree_collision.stl
    ├── <tree_name>_albedo.png
    └── <tree_name>_visual_colored.ply
```

**Key Scripts**
- `main.py`
  - reconstructs meshes from point clouds (`Poisson`)
  - applies decimation for visual and collision meshes
  - calls Blender for UV + texture baking
  - writes outputs into `models/<tree_name>/meshes/`

- `bake_vcol_to_texture.py`
  - imports the intermediate colored mesh (`.ply`)
  - creates UVs if missing (`uv.smart_project`)
  - builds a node graph that reads vertex color and bakes it to an image via `EMIT`
  - saves the baked texture (`.png`) and exports Collada (`.dae`)

**Notes**
- The visual mesh is intended for rendering (`tree_mesh.dae` + `*_albedo.png`).
- The collision mesh is intentionally simplified for simulation performance (`tree_collision.stl`).
- Blender is invoked in headless mode (`-b`) and called from `main.py` via `subprocess`.


---
