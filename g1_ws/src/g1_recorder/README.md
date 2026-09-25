# g1_recorder + keyframe_manager + scene_server (B's Day-1 packages)

Capture → keyframe → visualize for the Alien Bazaar G1 pipeline. Everything except the final
real capture works **offline** (synthetic data or a replayed bag), so the rest of the team can
develop against the canonical bag without the robot.

## Build
```bash
cd ~/ros2_ws
colcon build --packages-select g1_recorder keyframe_manager scene_server
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # whole graph uses CycloneDDS (see AGENTS.md)
```

## 1. Record (g1_recorder)
```bash
# Records the topic set in config/topics.yaml to an mcap bag with QoS overrides.
ros2 launch g1_recorder record.launch.py                 # -> g1_survey_<timestamp>/
ros2 launch g1_recorder record.launch.py output:=my_bag  # custom name
```
- **Topics:** edit `config/topics.yaml` (names are per AGENTS.md — **verify on the robot first**,
  see step 4).
- **Storage:** mcap, no compression.
- **QoS:** `config/qos_override.yaml` sets `/tf_static` to `transient_local` + `keep_all` so a
  consumer (RViz) that starts *after* playback still receives the static transforms.

## 2. Replay (robot off)
```bash
ros2 bag play g1_survey_<ts>            # add --loop to repeat, --clock for sim time
ros2 bag info g1_survey_<ts>            # confirm all topics + message counts
```
> **QoS gotcha:** sensor topics are often `best_effort`. On replay your subscribers/RViz must use
> `best_effort` reliability or they receive nothing. `keyframe_node` already subscribes with
> sensor-data QoS (best_effort), which matches both reliable and best_effort publishers.

## 3. Visualize (scene_server stub + RViz)
```bash
ros2 run scene_server ply_publisher                      # /vggt/scene_cloud in vggt_world + static map->vggt_world
rviz2 -d ~/ros2_ws/src/g1_recorder/rviz/scene.rviz       # Fixed Frame vggt_world
```
`scene_server` is a **placeholder owned by D**; the real reconstruction node must keep the
`/vggt/scene_cloud` topic and `vggt_world` frame contract.

## 4. On the robot: verify names/QoS before the canonical capture (R4)
```bash
ros2 topic list
ros2 topic info -v /camera/color/image_raw    # type + QoS (reliability/durability)
ros2 topic hz  /camera/color/image_raw        # rate sanity
```
Update `config/topics.yaml` (and `qos_override.yaml` if needed). **Do not hardcode unverified names.**

## 5. Extract keyframes (keyframe_manager)
```bash
# Against live streams or a replaying bag:
ros2 launch keyframe_manager keyframes.launch.py output_dir:=./keyframes
# Replaying a bag with sim time:
ros2 launch keyframe_manager keyframes.launch.py output_dir:=./keyframes use_sim_time:=true
#   (in another terminal) ros2 bag play <bag> --clock
```
Offline self-test with no robot/bag:
```bash
ros2 run keyframe_manager fake_rgbd_pub          # synthetic RGB-D; blurs every 3rd frame
ros2 run keyframe_manager keyframe_node --ros-args -p output_dir:=./keyframes
```
Selection (all must pass): sharpness (variance of Laplacian ≥ `blur_threshold`), temporal spacing
(≥ `min_time_gap_s`), translational baseline (≥ `min_translation_m`, needs odom). Tunables in
`keyframe_manager/config/keyframe_params.yaml`.

## FROZEN keyframe struct (contract for C / D / E)
```
<output_dir>/
  manifest.json              # {count, keyframes:[{id, dir, blur_var, frame_id}, ...]}
  keyframe_00/
    rgb.png                  # color, bgr8
    depth.npy                # uint16 MILLIMETRES, aligned to rgb  (H, W)
    camera_info.yaml         # width, height, distortion_model, k[9], d[]
    meta.yaml                # id, stamp{sec,nanosec}, frame_id, blur_var, depth_units=mm,
                             # depth_encoding, odom_pose{frame,position}?
```
Depth is stored as `uint16` mm regardless of source encoding (16UC1 mm passed through; 32FC1 m
converted ×1000). Consumers: read `depth_units` from `meta.yaml`.

## Milestones
R0 build ✅ · R1 record→replay + tf_static QoS ✅ · R2 viz stub ✅ · R3 keyframe extraction ✅ ·
**R4 verify names on robot** · **R5 canonical capture + shared bag** · R6 docs (this file).
