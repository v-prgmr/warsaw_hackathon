# G1 Rosbag Capture

`g1_recorder` records timestamped G1 sensor data to MCAP bags. Record raw inputs once, then
regenerate RTAB-Map, scans and semantic results offline. For live navigation, also record commands,
decisions and diagnostics. Recording is non-actuating and can run on the development laptop over
the wired robot network.

## Configuration Files

The package has three kinds of YAML configuration. They serve different purposes.

| File | Purpose |
|---|---|
| `config/realsense_rtab.yaml` | Configures RealSense resolution, rate, alignment, and sync |
| `config/rtab.yaml` | Camera-only topics for semantics / POI keyframes (not a mapping bag) |
| `config/survey.yaml` | Canonical raw sensor, robot state and TF inputs |
| `config/live_run.yaml` | Survey inputs plus observed control and common Nav2/audit outputs |
| `config/full_survey.yaml` | Legacy smaller survey profile |
| `config/qos_override.yaml` | Preserves transient-local `/tf_static` and `/map` |
| `config/cyclonedds_host_buffer.xml` | Optional host-only receive-buffer experiment |

Camera settings cannot be placed in `rtab.yaml` or `survey.yaml`. Those files only contain
topic names for `ros2 bag record`.

## Build

From the repository:

```bash
# Conda can make ament use ~/anaconda3/bin/python3, which lacks ROS modules.
conda deactivate 2>/dev/null || true

cd ~/workspace/warsaw/g1_ws
source /opt/ros/humble/setup.bash

which python3
python3 -c 'import catkin_pkg; print(catkin_pkg.__file__)'

colcon build --packages-select g1_recorder
source install/setup.bash
ros2 pkg prefix g1_recorder
```

`which python3` must resolve to `/usr/bin/python3`, not an Anaconda interpreter. Build from
`g1_ws`, not the repository root. `ros2 pkg prefix g1_recorder` should resolve under
`g1_ws/install/g1_recorder`.

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
unset ROS_LOCALHOST_ONLY  # offline playback may have set this to 1
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="enx3c33327bdb58" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>'
```

Replace `enx3c33327bdb58` if the wired device has another name. Confirm connectivity without
changing robot state:

```bash
ping -c 1 192.168.123.164
ros2 topic list --no-daemon --spin-time 10
```

In every new host terminal used for recording:

```bash
conda deactivate 2>/dev/null || true
cd ~/workspace/warsaw/g1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="enx3c33327bdb58" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>'

ros2 pkg prefix g1_recorder
ros2 topic list --no-daemon --spin-time 10 | grep '^/camera'
```

After that setup, change to the directory that should contain the bag and start the recorder:

```bash
cd ~/workspace/warsaw/datatset_rtab
ros2 launch g1_recorder record.launch.py profile:=survey output:=survey_take_01
```

## Configure RealSense

Before starting RealSense on the Orin, bind its Foxy/CycloneDDS process to `eth0`:

```bash
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>'
```

Do not reuse a stale Orin URI referencing `wlan0`; the RealSense node will fail before camera
initialization. Full Orin service stop/start and launch commands are in `start_realsense.md` at the
repository root.

After starting RealSense on the Orin, apply the package's bandwidth-safe RTAB profile from the
laptop if those parameters were not already supplied to `rs_launch.py`:

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

### RTAB Profile (camera only)

`config/rtab.yaml` records only the RealSense streams, for semantics / POI keyframes and camera
checks:

```yaml
topics:
  - /camera/color/image_raw
  - /camera/aligned_depth_to_color/image_raw
  - /camera/color/camera_info
  - /tf
  - /tf_static
```

It is **not a mapping bag**: RTAB-Map maps from LiDAR + IMU (AGENTS.md §8), and this profile omits
LiDAR, IMU, odometry and `/tf` from the robot. Use `profile:=survey` for canonical captures.

### Canonical Survey Profile

Use `profile:=survey` for reusable robot-off captures. `config/survey.yaml` contains RGB, aligned
depth, CameraInfo, LiDAR cloud and LiDAR IMU, `/dog_odom`, `/dog_imu_raw`, `/secondary_imu`,
`/lf/lowstate`, `/lf/bmsstate`, `/tf` and `/tf_static`. A 25 s live test capture (2026-09-25,
robot standing, no camera driver running) had data on every robot topic: LiDAR 10 Hz, LiDAR IMU
200 Hz, `/dog_odom` / `/dog_imu_raw` / `/secondary_imu` ~1 kHz, `/lf/bmsstate` 20 Hz. Check each
bag's counts. `/lf/lowstate` is the 20 Hz copy of `/lowstate`; the 1 kHz stream is ~2.4 MB/s and
20 Hz is enough to regenerate `/tf` (the sensors sit on the torso; only the waist joints move them).
`live_run` keeps the full-rate `/lowstate`. The RealSense `/camera/imu` is **not** included:
it was discovered but not verified to publish, and requires a supported IMU camera plus gyro/accel
configuration. Do not add it before checking the hardware and messages.

`/lowstate` and `/lf/lowstate` are Unitree-typed messages, not `/joint_states`. The recording host must have the
matching `unitree_hg` message package sourced. Check with `ros2 interface show
unitree_hg/msg/LowState` (and similarly for other Unitree types); if missing, use the project's
isolated bridge/container with those interfaces rather than assuming rosbag can deserialize them.
ROS-standard streams can still be recorded without those packages.
For `live_run`, also confirm `unitree_go/msg/WirelessController` and `unitree_api/msg/Request` and
`Response` are installed in the recording environment.

### Live-Run Profile

`profile:=live_run` adds `/cmd_vel`, the **observed** `/api/sport/request` and response,
`/wirelesscontroller`, `/scan`, `/map`, `/goal_pose`, `/initialpose`, `/plan`, hidden Nav2
`/navigate_to_pose/_action/feedback` and status, `/rosout`, and `/diagnostics`. The launch enables
`--include-hidden-topics` for this profile. Only publishing topics are saved: inspect `ros2 bag
info` after every run. The proposed `/api/loco/*` and `/semantic/*` and `/g1/scene_handoff` names
are not verified contracts; add the actual topics to `live_run.yaml` once implemented. Save the
RTAB-Map database and the independent low-level supervisor logs separately when applicable;
rosbag alone does not replace the required supervisor logs. No low-level command topic is added.

### Legacy Full-Survey Profile

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

The robot currently does not publish dynamic `/tf`. The topic remains in the profiles so future
bridges are captured automatically. `/tf_static` is published by RealSense.

**TF prerequisite:** the camera's internal TF does not connect `robot_center` to the RealSense or
`livox_frame`. Existing G1 URDF variants contain `d435_link` and `mid360_link`, but their
extrinsics and correspondence to the observed frames require validation on this robot. Supply a
calibrated base-to-sensor transform or correctly mapped joint states and
`robot_state_publisher` during capture; recording `/lf/lowstate` allows the latter to be regenerated
offline only after a verified converter exists. Neither profile creates the missing TF chain.

### Custom Profile

Create another YAML with the same `topics` structure, rebuild if it is stored in the package, and
pass its absolute path:

```bash
ros2 launch g1_recorder record.launch.py \
  topics_file:=/absolute/path/to/custom_topics.yaml \
  output:=custom_take
```

An explicit `topics_file` overrides `profile`.

## Record Takes

Use unique output directories. Rosbag refuses to overwrite an existing directory.

Record the canonical raw sensor take with the survey profile (the default), and write down 2–3
tape-measured dimensions of the scene next to the bag name (AGENTS.md §10.2):

```bash
ros2 launch g1_recorder record.launch.py profile:=survey output:=survey_take_01
```

Stop cleanly with `Ctrl-C`. Wait for `Recording stopped` before closing the terminal.

For a camera-only take (semantics / POI keyframes), use `profile:=rtab`.

For a live navigation run (only when the event control requirements are met):

```bash
ros2 launch g1_recorder record.launch.py profile:=live_run output:=live_run_01
```

The older, smaller `full_survey` profile remains available:

```bash
ros2 launch g1_recorder record.launch.py \
  profile:=full_survey \
  output:=full_survey_take_01
```

The default profile is `survey`. If `output` is omitted, the launch file creates a name such as
`g1_survey_20260925_164500`.

Observed short captures used approximately 18 MiB/s for `rtab` and 23 MiB/s for the legacy full
profile; new survey/live profiles may be larger. A 4.8 GiB, 287-second RTAB bag contained 2,699
RGB frames (~9.4 Hz), but that alone does **not** identify where missing frames originated. Check
free space and measure source and recorded rates before a long capture.

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
ros2 bag info survey_take_01
ros2 bag info live_run_01
```

For an RTAB take, require non-zero counts for:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/tf_static
```

For a survey take, additionally require non-zero counts for:

```text
/utlidar/cloud_livox_mid360
/dog_imu_raw
/dog_odom
/lf/lowstate
/lf/bmsstate
```

Depth count may exceed RGB count. A short verified capture had exact depth and CameraInfo matches
for every RGB frame; a later 153-second survey bag had 1,493 exact RGB-depth matches out of 1,494
RGB frames (and 1,494 CameraInfo matches). Check timestamp pairing on each bag: unequal total
counts alone do not indicate whether RTAB-Map has usable synchronized RGB-D input.

`/map` and `/tf_static` use transient-local QoS overrides. Check `ros2 topic info -v /map` before
live capture: a volatile publisher is incompatible with the configured transient-local recorder
subscription and needs a compatible override for that run. Hidden action topics must appear in
`ros2 bag info`; enabling hidden discovery does not synthesize absent Nav2 topics.

### Optional host-only receive-buffer experiment

If live source rates exceed bag counts, first measure `ros2 topic hz` with minimal competing
subscribers and inspect laptop NIC receive-drop counters. To test UDP buffer pressure **on the
laptop only**, record a before/after bag with the same scene and camera settings. Check the current
limit with `sysctl net.core.rmem_max`; with host administration approval raise it temporarily, for
example `sudo sysctl -w net.core.rmem_max=16777216`, and in the recording terminal set
`CYCLONEDDS_URI=file://$(ros2 pkg prefix g1_recorder)/share/g1_recorder/config/cyclonedds_host_buffer.xml`.
The 10 MB minimum makes CycloneDDS fail to start if the host's receive-buffer limit was not raised;
restore the previous `CYCLONEDDS_URI` if testing is interrupted. Replace the XML's laptop interface
name if it differs. This experiment is opt-in; it does not alter
the robot and does not establish that UDP buffers caused any observed frame-rate difference.

## Replay

Replay in an isolated DDS domain, **never on the robot's domain 0**: a replayed bag republishes
`/dog_odom`, `/lf/lowstate`, the LiDAR and (in `live_run` bags) `/api/sport/request` and `/cmd_vel`.
On domain 0 these reach the live robot's network. The Docker image does this with
`SIM=1 scripts/run_humble.sh` (domain 77, no robot-NIC binding). By hand:

```bash
source /opt/ros/humble/setup.bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=77

ros2 bag play rtab_take_01 --clock
```

The recording environment binds CycloneDDS to the robot Ethernet adapter. That adapter may not
exist after disconnecting the G1, so offline replay must unset `CYCLONEDDS_URI`. Apply the same
environment in every terminal that runs an offline consumer such as RTAB-Map or RViz. Avoid
`ROS_LOCALHOST_ONLY=1` with CycloneDDS for the mapping stack: it allows only ~9 participants per
host and the next node fails with "Failed to find a free participant index".

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

### Playback reports that the wired interface is unavailable

The terminal still has a robot-specific `CYCLONEDDS_URI`, but the Ethernet adapter is disconnected.
For robot-off replay, use:

```bash
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
```

Do not use localhost-only mode while recording from the robot.

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
