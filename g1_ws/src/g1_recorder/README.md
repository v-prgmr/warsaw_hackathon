# G1 Rosbag Capture

`g1_recorder` records timestamped G1 sensor data to MCAP rosbag directories for RTAB-Map and
full-survey analysis. Recording is non-actuating and can run on the development laptop over the
wired robot network.

## Configuration Files

The package has three kinds of YAML configuration. They serve different purposes.

| File | Purpose |
|---|---|
| `config/realsense_rtab.yaml` | Configures RealSense resolution, rate, alignment, and sync |
| `config/rtab.yaml` | Selects the minimal topics recorded for RGB-D RTAB-Map |
| `config/full_survey.yaml` | Selects RGB-D, LiDAR, robot odometry, IMU, and TF topics |
| `config/qos_override.yaml` | Preserves transient-local `/tf_static` in the bag |

Camera settings cannot be placed in `rtab.yaml` or `full_survey.yaml`. Those files only contain
topic names for `ros2 bag record`.

## Build

From the repository:

```bash
cd g1_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select g1_recorder
source install/setup.bash
```

Rebuild and source the workspace after changing any YAML file. The launch file reads the installed
copy under `install/g1_recorder/share/g1_recorder/config/`.

## Connect To The Robot

Verified setup on 2026-09-25:

```text
Laptop address: 192.168.123.222/24
Robot/Orin:     192.168.123.164
Wired device:   enx3c33327bdb58
ROS domain:     0
RMW:            rmw_cyclonedds_cpp
```

Interface names may differ on another laptop. Check yours with:

```bash
ip -brief address
```

Configure the terminal used for discovery and recording:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enx3c33327bdb58"/></Interfaces></General></Domain></CycloneDDS>'
```

Replace `enx3c33327bdb58` if the wired device has another name. Confirm connectivity without
changing robot state:

```bash
ping -c 1 192.168.123.164
ros2 topic list --no-daemon --spin-time 10
```

## Configure RealSense

Start the RealSense ROS node on the Orin with aligned depth enabled. Then apply the package's
bandwidth-safe RTAB profile from the laptop:

```bash
ros2 param load /camera/camera \
  "$(ros2 pkg prefix g1_recorder)/share/g1_recorder/config/realsense_rtab.yaml"
```

The current profile uses:

```yaml
/camera/camera:
  ros__parameters:
    rgb_camera.profile: 640x480x15
    depth_module.profile: 640x480x15
    align_depth.enable: true
    enable_sync: true
    pointcloud.enable: false
    publish_tf: true
```

This profile is intentional:

- Matching RGB and depth dimensions avoid resizing aligned depth to 1280x720.
- Fifteen requested frames per second provide enough data for indoor RTAB-Map.
- The verified stream produces about 10-11 exact RGB-D pairs per second over the wired link.
- Disabling the RealSense point cloud avoids recording or transporting geometry derivable from
  depth.
- The profile reduces raw traffic from approximately 138 MB/s at the previous settings to roughly
  23 MB/s before DDS overhead.

The loaded parameters apply to the running camera node. Load the file again after restarting that
node. Confirm the important values:

```bash
ros2 param get /camera/camera rgb_camera.profile
ros2 param get /camera/camera depth_module.profile
ros2 param get /camera/camera align_depth.enable
ros2 param get /camera/camera enable_sync
ros2 param get /camera/camera pointcloud.enable
```

## Configure Recording Topics

A recording profile has one `topics` list:

```yaml
topics:
  - /some/topic
  - /another/topic
```

### RTAB Profile

`config/rtab.yaml` minimizes network and storage load:

```yaml
topics:
  - /camera/color/image_raw
  - /camera/aligned_depth_to_color/image_raw
  - /camera/color/camera_info
  - /tf
  - /tf_static
```

RTAB-Map can generate odometry with `rgbd_odometry`, so robot odometry, IMU, and LiDAR are omitted
from this profile.

### Full-Survey Profile

`config/full_survey.yaml` records supporting robot sensors:

```yaml
topics:
  - /camera/color/image_raw
  - /camera/aligned_depth_to_color/image_raw
  - /camera/color/camera_info
  - /utlidar/cloud_livox_mid360
  - /dog_imu_raw
  - /dog_odom
  - /tf
  - /tf_static
```

These robot topics were verified live:

| Topic | Type | Verified behavior |
|---|---|---|
| `/dog_odom` | `nav_msgs/msg/Odometry` | Approximately 1 kHz; `odom` to `robot_center` |
| `/dog_imu_raw` | `sensor_msgs/msg/Imu` | Approximately 1 kHz; frame `dog_imu_link` |
| `/utlidar/cloud_livox_mid360` | `sensor_msgs/msg/PointCloud2` | Approximately 10 Hz; frame `livox_frame` |

The robot currently does not publish dynamic `/tf`. The topic remains in both profiles so future
bridges are captured automatically. `/tf_static` is published by RealSense.

### Custom Profile

Create another YAML with the same `topics` structure, rebuild if it is stored in the package, and
pass its absolute path:

```bash
ros2 launch g1_recorder record.launch.py \
  topics_file:=/absolute/path/to/custom_topics.yaml \
  output:=custom_take
```

An explicit `topics_file` overrides `profile`.

## Record Both Takes

Use unique output directories. Rosbag refuses to overwrite an existing directory.

First record the minimal RTAB take:

```bash
ros2 launch g1_recorder record.launch.py \
  profile:=rtab \
  output:=rtab_take_01
```

Stop cleanly with `Ctrl-C`. Wait for `Recording stopped` before closing the terminal.

Then record the full survey take:

```bash
ros2 launch g1_recorder record.launch.py \
  profile:=full_survey \
  output:=full_survey_take_01
```

The default profile is `rtab`. If `output` is omitted, the launch file creates a name such as
`g1_survey_20260925_164500`.

Observed uncompressed storage with the safe camera profile was approximately 18 MiB/s for the RTAB
profile and 23 MiB/s for the full profile. Keep at least 2 GiB free per planned recording minute.

## Pause And Resume

The recorder starts immediately. From a second terminal with the same ROS/DDS environment:

```bash
ros2 service call /rosbag2_recorder/pause rosbag2_interfaces/srv/Pause "{}"
ros2 service call /rosbag2_recorder/is_paused rosbag2_interfaces/srv/IsPaused "{}"
ros2 service call /rosbag2_recorder/resume rosbag2_interfaces/srv/Resume "{}"
```

Pause/resume writes multiple active segments into the same bag. Use `Ctrl-C` to finalize that bag;
start another launch command for a separate take.

## Validate Every Bag

Inspect each take immediately:

```bash
ros2 bag info rtab_take_01
ros2 bag info full_survey_take_01
```

For an RTAB take, require non-zero counts for:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/tf_static
```

For a full-survey take, additionally require non-zero counts for:

```text
/utlidar/cloud_livox_mid360
/dog_imu_raw
/dog_odom
```

Depth count may exceed RGB count. The verified camera emitted extra depth frames, but every recorded
RGB frame had an exact timestamp match in both aligned depth and CameraInfo. This is suitable for
RTAB-Map. Missing or unmatched RGB frames are the failure condition, not unequal total counts.

## Replay

Replay with the robot off:

```bash
ros2 bag play rtab_take_01 --clock
```

Use `use_sim_time:=true` on consumers when playing with `--clock`. For consumers that require the
conventional `/odom` name, remap the recorded robot odometry:

```bash
ros2 bag play full_survey_take_01 --clock \
  --remap /dog_odom:=/odom
```

Sensor topics may use best-effort QoS during replay. RViz and custom subscribers should use a
sensor-data/best-effort subscription when they otherwise receive no messages.

## Troubleshooting

### Only local ROS topics appear

- Confirm wired connectivity to `192.168.123.164`.
- Confirm `ROS_DOMAIN_ID=0`.
- Bind CycloneDDS to the wired interface, not Wi-Fi.
- Use `--no-daemon` while testing discovery, or restart the local ROS daemon after changing DDS
  environment variables.

### Camera topics are absent

The RealSense ROS node is not running or is on another ROS domain. Expected topics are:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/tf_static
```

### Depth or RGB counts are very low

- Reapply `realsense_rtab.yaml`.
- Confirm both requested profiles are `640x480x15`.
- Do not enable the RealSense point cloud.
- Do not run several raw-image viewers or `ros2 topic hz` commands during the canonical capture.
- Record the RTAB profile separately from the full survey as documented above.

### Static transforms are missing after replay starts

Always use `config/qos_override.yaml`. It records `/tf_static` with transient-local durability and
keep-all history so late replay subscribers receive camera transforms.
