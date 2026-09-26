# G1 OAK-D semantic/color map (prototype)

This package **annotates** RTAB-Map's metric LiDAR `/cloud_map`; it does not
alter RTAB-Map geometry, publish `map -> odom`, or feed labels into navigation.
It projects each rectified OAK-D RGB image into the map at up to 1 Hz using the
image-time optical-frame TF and `CameraInfo.P`. The aligned depth image rejects
occluded or misregistered points. COCO Mask R-CNN supplies per-pixel *thing*
classes (e.g. `chair`, `dining table`, `person`); unlabeled geometry remains
class 0. Color and label evidence are accumulated per 4 cm map voxel and can be
revised by later observations. The output `/g1_semantic_map/cloud` has XYZ,
packed `rgb` (RViz), `label` (COCO ID), and `confidence` fields.
`/g1_semantic_map/class_names` publishes a latched JSON ID-to-name dictionary
when the COCO detector is enabled.

RTAB-Map already supports RGB-D attachment for map-node color. This separate
layer is useful when you need labels and controlled, revisable LiDAR color.
It should not be mistaken for an automatically recolored RTAB-Map database.

## Geometry and limits

- `T_camera_cloud` is looked up at the RGB image timestamp. Camera frame must
  be the RGB **optical** frame (+x right, +y down, +z forward); cloud points are
  in the `PointCloud2.header.frame_id`, normally `map`. Units are metres.
- RGB must be rectified; depth must be aligned to RGB and share its dimensions.
  `CameraInfo.P` is used, so do not feed a distorted raw image.
- The calibrated OAK-D-to-torso TF is required for real use. The current topic
  names, frame IDs, and calibration must be checked on the actual device; the
  launch file deliberately leaves OAK-D topic defaults empty.
- 1 Hz is a maximum processing cadence, not a promised inference speed.
  Mask R-CNN V2 is large (~177 MB weights) and may run below 1 FPS on CPU.
  Install compatible `torch` and `torchvision` in the runtime environment;
  weights download on first run. Use `device:=cuda` only if CUDA is available.
- Map-point sampling is capped at 200k points for responsiveness. Evidence is
  keyed to map-frame voxels; a major loop closure can move geometry to new
  voxels, which then needs fresh observations. This is not an RTAB graph edit.
- COCO has `dining table`, not an arbitrary universal `table` class. It is
  instance segmentation of known object categories, not wall/floor labeling.

## Build and run

In the existing ROS 2 Humble environment:

```bash
cd g1_ws
colcon build --packages-select g1_semantic_map
source install/setup.bash
ros2 launch g1_semantic_map semantic_map.launch.py \
  rgb_topic:=/VERIFIED/OAK/RGB_RECT \
  depth_topic:=/VERIFIED/OAK/ALIGNED_DEPTH \
  camera_info_topic:=/VERIFIED/OAK/CAMERA_INFO \
  detector:=coco_maskrcnn device:=cpu
```

Run `g1_sensors tf_chain` and `g1_mapping` separately, with only RTAB-Map
publishing `map -> odom`. Before real use, verify camera/depth/TF timestamps
and the calibrated optical-frame transform. View `/g1_semantic_map/cloud` in
RViz with its RGB transformer; inspect `label` and `confidence` fields in a
PointCloud2 subscriber or choose a field-based RViz transformer.

## Synthetic test (no robot/camera/model)

```bash
cd g1_ws
colcon build --packages-select g1_semantic_map
source install/setup.bash
python3 -m unittest discover -s src/g1_semantic_map/test -v
ros2 run g1_semantic_map synthetic_scene
# in another shell with the same environment:
ros2 launch g1_semantic_map semantic_map.launch.py \
  cloud_topic:=/synthetic/cloud_map rgb_topic:=/synthetic/oak/rgb \
  depth_topic:=/synthetic/oak/depth camera_info_topic:=/synthetic/oak/info \
  detector:=synthetic_oracle
ros2 run g1_semantic_map synthetic_check  # PASS verifies RGB and class fields
```

The synthetic scene checks end-to-end TF, projection, RGB, semantic labels and
PointCloud2 publishing. Its palette detector is a fixture-only oracle, **not**
a real COCO classifier. Unit tests also supply oracle class masks to check that semantic
labels update and depth occlusion is enforced. Neither test establishes COCO
accuracy, real OAK-D calibration, or live 1 Hz throughput.
