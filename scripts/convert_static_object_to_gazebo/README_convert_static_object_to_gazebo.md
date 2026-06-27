# convert_static_object_to_gazebo.py

`convert_static_object_to_gazebo.py` converts a general static obstacle asset into a Gazebo model directory.

This script is intended for static objects other than tree, ground, person, vehicle, animal, and robot assets. Tree assets should be converted with the dedicated tree converter.

## Purpose

The script generates a Gazebo-ready model with:

- `model.sdf`
- `model.config`
- `meshes/object_visual.obj`
- `meshes/object_collision.stl` when mesh collision is required
- `semantic_parts.json`

Typical targets are:

- harvest basket / orange basket
- crate / box
- rock
- pole
- fence
- wall
- container
- other static obstacles

## Requirements

Run this script with Blender, not with normal Python.

```bash
blender --background --python convert_static_object_to_gazebo.py -- <args>
```

Supported input formats:

- `.blend`
- `.obj`
- `.fbx`
- `.glb`
- `.gltf`

The visual output is currently OBJ only.

## Basic Usage

```bash
blender --background \
  --python convert_static_object_to_gazebo.py -- \
  --input path/to/object.obj \
  --model-name object_01 \
  --output-dir path/to/gazebo/models/object_01 \
  --object-class obstacle \
  --contact-policy AVOID \
  --collision-mode bounding_box \
  --collision-output-mode auto
```

## Recommended Examples

### Harvest basket / orange basket

For a harvest basket, a box collision is usually sufficient. This is the recommended lightweight setting.

```bash
blender --background \
  --python convert_static_object_to_gazebo.py -- \
  --input ~/blender_ws/object_models/harvest_basket.obj \
  --model-name harvest_basket_01 \
  --output-dir ~/humble_robotsim/src/forest_map_generator/models_private/harvest_basket_01 \
  --object-class basket \
  --contact-policy AVOID \
  --collision-mode bounding_box \
  --collision-output-mode sdf_primitive
```

This produces:

- visual: `meshes/object_visual.obj`
- collision: SDF `<box>` primitive
- semantic label: `basket = 24` when `--segmentation-label auto` is used

### Rock

For rocks, a bounding box is often too rough. A low-poly convex hull collision is usually a good compromise between shape accuracy and simulation cost.

```bash
blender --background \
  --python convert_static_object_to_gazebo.py -- \
  --input ~/blender_ws/object_models/rock_01.obj \
  --model-name rock_01 \
  --output-dir ~/humble_robotsim/src/forest_map_generator/models_private/rock_01 \
  --object-class rock \
  --contact-policy AVOID \
  --collision-mode convex_hull \
  --collision-output-mode auto \
  --collision-hull-max-points 100 \
  --collision-hull-decimate-ratio 0.6
```

This produces:

- visual: `meshes/object_visual.obj`
- collision: `meshes/object_collision.stl`
- semantic label: `rock = 21` when `--segmentation-label auto` is used

For a lighter rock collision, reduce the hull complexity:

```bash
--collision-hull-max-points 50 \
--collision-hull-decimate-ratio 0.4
```

### Pole

For a pole-like object, use a cylinder collision.

```bash
blender --background \
  --python convert_static_object_to_gazebo.py -- \
  --input ~/blender_ws/object_models/pole_01.obj \
  --model-name pole_01 \
  --output-dir ~/humble_robotsim/src/forest_map_generator/models_private/pole_01 \
  --object-class pole \
  --contact-policy AVOID \
  --collision-mode bounding_cylinder \
  --collision-output-mode sdf_primitive
```

## Collision Modes

### `bounding_box`

Creates a bounding box from the visual mesh bounds.

Recommended for:

- basket
- crate
- box
- container
- simple rectangular obstacles

Usually use with:

```bash
--collision-output-mode sdf_primitive
```

### `bounding_cylinder`

Creates a vertical cylinder from the visual mesh bounds.

Recommended for:

- pole
- post
- simple trunk-like static object

Usually use with:

```bash
--collision-output-mode sdf_primitive
```

### `bounding_sphere`

Creates a sphere from the largest visual mesh dimension.

Recommended for:

- small round rock
- ball-like obstacle
- very rough approximation

Usually use with:

```bash
--collision-output-mode sdf_primitive
```

### `convex_hull`

Samples visual mesh vertices and creates a convex hull collision mesh.

Recommended for:

- rock
- irregular static obstacle
- object where a simple box is too conservative

Usually use with:

```bash
--collision-output-mode auto
```

or explicitly:

```bash
--collision-output-mode mesh_stl
```

Useful parameters:

```bash
--collision-hull-max-points 100
--collision-hull-decimate-ratio 0.6
```

### `visual_decimated`

Duplicates the visual mesh and decimates it for collision.

This is less recommended for large worlds because it can still produce relatively expensive collision geometry.
Use it only when the convex hull is too coarse.

Useful parameter:

```bash
--collision-visual-decimate-ratio 0.25
```

### `none`

Writes no collision geometry.

Use this only for decorative static objects that should not affect navigation or physics.

## Collision Output Modes

### `auto`

Recommended default.

- primitive collision modes become SDF primitives
- mesh-based collision modes become STL mesh collision

Mapping:

| collision mode | auto output |
|---|---|
| `bounding_box` | SDF `<box>` |
| `bounding_cylinder` | SDF `<cylinder>` |
| `bounding_sphere` | SDF `<sphere>` |
| `convex_hull` | STL mesh |
| `visual_decimated` | STL mesh |
| `none` | no collision |

### `sdf_primitive`

Forces primitive collision output when possible.

Supported collision modes:

- `bounding_box`
- `bounding_cylinder`
- `bounding_sphere`

If used with `convex_hull` or `visual_decimated`, the script falls back to `mesh_stl`.

### `mesh_stl`

Writes `meshes/object_collision.stl` and references it as mesh collision in `model.sdf`.

This is useful when the collision shape cannot be represented as a simple primitive.

## Semantic Labels

The script writes a visual segmentation label when `--write-segmentation-labels` is enabled.

Default behavior:

```bash
--segmentation-label auto
```

Canonical labels currently defined in the script:

| class | label |
|---|---:|
| `rock` | 21 |
| `pole` | 22 |
| `fence` | 23 |
| `basket` | 24 |
| `crate` | 25 |
| `wall` | 26 |
| `container` | 27 |
| `sign` | 28 |
| `obstacle` | 29 |
| `ignore` | 255 |

Reserved but not intended for this script:

| class | label |
|---|---:|
| `leaf` | 1 |
| `wood` | 2 |
| `fruit` | 3 |
| `ground` | 20 |
| `person` | 50 |
| `vehicle` | 51 |
| `animal` | 52 |
| `robot` | 90 |

To disable labels:

```bash
--segmentation-label none
```

To set a custom label:

```bash
--segmentation-label 31
```

## semantic_parts.json

The script writes `semantic_parts.json` by default.

Example for a basket with SDF box collision:

```json
{
  "model": "harvest_basket_01",
  "object_type": "static_object",
  "visual_mode": "single_mesh",
  "parts": [
    {
      "name": "object_visual",
      "role": "visual",
      "class": "basket",
      "contact_policy": "AVOID",
      "mesh": "meshes/object_visual.obj",
      "gazebo_visual": "object_link::object_visual",
      "segmentation_label": 24
    },
    {
      "name": "object_collision",
      "role": "collision",
      "class": "basket",
      "contact_policy": "AVOID",
      "gazebo_collision": "object_link::object_collision",
      "segmentation_label": null,
      "geometry": {
        "type": "box",
        "size": [0.6, 0.4, 0.3],
        "pose": [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
      }
    }
  ]
}
```

For a rock with convex hull collision, the collision entry becomes mesh-based:

```json
{
  "name": "object_collision",
  "role": "collision",
  "class": "rock",
  "contact_policy": "AVOID",
  "gazebo_collision": "object_link::object_collision",
  "segmentation_label": null,
  "geometry": {
    "type": "mesh",
    "mesh": "meshes/object_collision.stl",
    "mode": "convex_hull"
  }
}
```

## Origin Handling

Default:

```bash
--origin-mode bottom_center
```

This shifts the model so that:

- XY origin is the visual bounding box center
- Z origin is the visual bounding box bottom

This is usually convenient for placing objects on the ground plane.

Available modes:

| mode | behavior |
|---|---|
| `bottom_center` | origin at visual bounds bottom center |
| `keep` | keep original Blender/world origin |
| `cursor` | use Blender cursor as origin |
| `named_empty` | use an Empty object as origin |

For `named_empty`:

```bash
--origin-mode named_empty \
--origin-empty-name OBJECT_ORIGIN
```

## Optional Visual Decimation

The visual mesh can be decimated before OBJ export:

```bash
--visual-decimate-ratio 0.5
```

By default, the visual mesh is not decimated:

```bash
--visual-decimate-ratio 1.0
```

For large worlds, visual mesh complexity often affects Gazebo rendering and GPU sensor cost more than collision complexity.
If RTF is low, consider preparing lower-detail visual assets in addition to simplifying collision geometry.

## Texture Copying

To copy texture image files used by materials:

```bash
--copy-textures
```

Textures are copied under:

```text
meshes/textures/
```

Packed Blender images are not automatically unpacked. Unpack them in Blender if Gazebo needs the texture files.

## Explicit Collision Sources

### Collision collection

If the input `.blend` contains a collection named `collision`, its mesh objects are used as explicit collision geometry.

Default collection name:

```bash
--collision-collection collision
```

The explicit collision collection overrides generated collision.

### Existing collision file

To use an existing STL collision file:

```bash
--collision-file path/to/object_collision.stl
```

This copies the file to:

```text
meshes/object_collision.stl
```

and writes mesh collision in `model.sdf`.

## Dry Run

To inspect detected settings without writing files:

```bash
blender --background \
  --python convert_static_object_to_gazebo.py -- \
  --input path/to/object.obj \
  --model-name test_object \
  --output-dir /tmp/test_object \
  --dry-run
```

## Output Directory Structure

Example:

```text
harvest_basket_01/
├── model.config
├── model.sdf
├── semantic_parts.json
└── meshes/
    └── object_visual.obj
```

For mesh collision objects:

```text
rock_01/
├── model.config
├── model.sdf
├── semantic_parts.json
└── meshes/
    ├── object_visual.obj
    └── object_collision.stl
```

## Notes for Large Gazebo Worlds

For large orchards or repeated static obstacles:

- prefer SDF primitive collision for boxes, poles, and simple objects
- use convex hull collision for rocks when a box is too coarse
- avoid `visual_decimated` collision unless necessary
- keep visual mesh complexity low when GPU LiDAR or camera sensors are active
- use separate model variants when needed, such as `*_mid_lod` and `*_data_lod`

## Common Option Summary

| option | purpose |
|---|---|
| `--input` | input asset path |
| `--model-name` | Gazebo model name |
| `--output-dir` | output model directory |
| `--object-class` | semantic class name |
| `--contact-policy` | project-level contact policy metadata |
| `--collision-mode` | collision approximation method |
| `--collision-output-mode` | SDF primitive or STL mesh output |
| `--origin-mode` | origin placement method |
| `--scale` | uniform scale |
| `--segmentation-label` | visual segmentation label |
| `--copy-textures` | copy material textures |
| `--dry-run` | inspect conversion without writing files |
