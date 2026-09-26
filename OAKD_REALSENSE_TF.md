# G1 OAK-D ↔ RealSense provisional TF and OAK-D bring-up

## Source and status

The **24-pose** ChArUco take `camera_calibration_triggered_20260926_181606_243087`
(7×5 squares, 30 mm squares, 22 mm markers, `DICT_4X4_50`) is the sole source of this
provisional extrinsic. The bag has 24 images and CameraInfo messages from **each** camera;
21 paired observations were usable, covering 20 varied board viewpoints. The fit used 16
training inliers and held out 5 pairs. The held-out median transform variation was
**5.37 mm / 1.16°**; median cross-projection errors were **1.90 px** in RealSense and
**7.38 px** in OAK-D. The stricter quality gate did **not** pass. Use as an initial estimate
and verify OAK depth against LiDAR before relying on centimetre-level POIs.

Source file on the calibration laptop:
`~/g1_camera_calibration/camera_calibration_triggered_20260926_181606_243087/charuco_calibration/calibration.yaml`.
The mount calculation and its provenance are also stored **on the G1 Orin** at
`/home/unitree/g1_calibration/oakd_realsense_provisional.yaml` (copied and hash-checked).
Neither file contains a password.

## Frame direction and numerical transform

TF notation here is `T_parent_child`: it maps a point expressed in `child` coordinates
into `parent` coordinates. Translation is in **metres**; quaternion is **x, y, z, w**.

Direct result from OpenCV, **RealSense RGB optical ← OAK-D RGB optical**:

| Parent | Child | XYZ (m) | Quaternion XYZW |
|---|---|---|---|
| `camera_color_optical_frame` | `oak_rgb_camera_optical_frame` | `0.033755545, 0.152859401, 0.302205256` | `0.365835554, 0.002419455, -0.014105887, 0.930569459` |

**Do not publish that optical-frame link directly.** Both camera drivers already publish
their internal camera frames; doing so would give OAK's optical frame two TF parents.
Composing the result with both drivers' recorded `/tf_static` yields the **one external
mount link**:

| Parent | Child | XYZ (m) | RPY (rad) | Quaternion XYZW |
|---|---|---|---|---|
| `camera_link` | `oak-d-base-frame` | `0.302104233, -0.016891008, -0.153149919` | `-0.030034576, -0.748448924, 0.010297047` | `-0.012095426, -0.365576675, -0.000697662, 0.930702374` |

The composition, checked numerically against the bag's two camera-internal TF chains, is:

```text
T_camera_link_oak_base
  = T_camera_link_camera_color_optical
    · T_camera_color_optical_oak_rgb_optical
    · inverse(T_oak_base_oak_rgb_optical)
```

This is the single `camera_link -> oak-d-base-frame` entry in the team's local
`g1_ws/src/g1_sensors/config/g1_sensors.yaml`; that config publishes the mount **only while
`g1_sensors tf_chain.launch.py` is running**. The Orin copy is a recovery artifact, not an
additional automatic TF publisher. **This Markdown handoff is the only file included in the
commit**; a colleague pulling it without the local `g1_sensors` change must enter the mount
numbers above in their TF configuration before using the frame chain.

## Start OAK-D ROS on the G1 Orin

The following commands run **on the Orin** (SSH target `unitree@192.168.123.164`). Keep
the launch terminal open. They do not start locomotion, SLAM or an extra mount-TF publisher.
No password belongs in a script or shell command; enter it interactively when SSH prompts.

```bash
# From the wired development laptop:
ssh unitree@192.168.123.164

# In the Orin terminal:
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'

# Verify that the OAK-D is attached and that the session sees the robot ROS graph:
lsusb -d 03e7:
ros2 topic list | grep '^/utlidar/cloud_livox_mid360$'

# The existing Foxy package; launch only one copy:
ros2 launch depthai_ros_driver camera.launch.py use_rviz:=false
```

The **installed** Foxy driver configuration requests `RGBD`, `stereo.i_align_depth: true`,
`stereo.i_output_disparity: false`, and an RGB optical target. It is configured for an OAK-D
connected at **USB HIGH speed (USB 2.0)**. In a second Orin terminal with the same ROS/DDS
environment, check actual publications instead of assuming the configuration succeeded:

```bash
ros2 topic list | grep '^/oak/'
ros2 topic echo --once /oak/rgb/camera_info
ros2 topic echo --once /oak/stereo/camera_info
ros2 topic info -v /oak/stereo/image_raw
ros2 topic echo --once /oak/stereo/image_raw --field encoding
ros2 topic echo --once /oak/stereo/image_raw --field header
```

Expected topics when this driver's RGBD pipeline works are `/oak/rgb/image_raw`,
`/oak/rgb/camera_info`, `/oak/stereo/image_raw` and `/oak/stereo/camera_info`.
**Check the depth Image `encoding`, `frame_id`, image size, depth scale, and CameraInfo on
live messages before 3D back-projection**; the older RGB-only calibration bags contain no
OAK depth. The installed driver source requests RGB-aligned depth and labels uncompressed
RAW16 depth as `16UC1`; DepthAI normally uses millimetres with zero meaning invalid.
Those last details are source/config expectations, **not yet measured on this live G1**.

To stop the OAK session, press **Ctrl-C in its launch terminal** and verify the OAK topics
disappear. Do not `kill -9` a ROS launch; it can orphan publishers. End other project live
nodes with the established `scripts/stop_ros.sh` (container) or `scripts/stop_humble.sh`
(host) procedures.

### Current bring-up blocker (2026-09-26)

A remote, short-lived Foxy launch probe on this G1 exited with **code 139
(segmentation fault) before `/oak/*` topics appeared**. It left no camera processes
running. The `NetworkInterfaceAddress` CycloneDDS URI above **passed a read-only
`ros2 topic list` check**, but that does *not* prove the driver will start. A newer
`<Interfaces><NetworkInterface .../></Interfaces>` URI was rejected by this Foxy
CycloneDDS build as an unknown element. If the launch fails in the colleague's
interactive terminal, keep the existing robot OS/DDS setup unchanged, inspect its
`~/.ros/log/<latest>/launch.log`, and resolve the launch environment before recording;
do not claim the OAK-D is running just because USB detects it.

## Publish and check the shared TF

With both camera drivers running, the **host** `g1_sensors tf_chain` is the intended
single owner of the mount link (alongside the G1 URDF/joint states). Build/source the
host workspace after its local config has the mount entry shown above:

```bash
cd ~/workspace/warsaw/g1_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select g1_sensors
source install/setup.bash
ros2 launch g1_sensors tf_chain.launch.py
```

In a second host terminal on the same wired ROS domain:

```bash
ros2 run tf2_ros tf2_echo camera_link oak-d-base-frame
ros2 run tf2_ros tf2_echo camera_color_optical_frame oak_rgb_camera_optical_frame
```

There must be **one** publisher of `camera_link -> oak-d-base-frame`: do not run a second
`static_transform_publisher` for it on the Orin when `tf_chain` is active. The MID-360
extrinsic still comes from the G1 URDF. For later depth/LiDAR refinement, keep this TF
as the initial guess, capture stationary overlapping 3D geometry, and replace only this
mount link after held-out alignment checks.
