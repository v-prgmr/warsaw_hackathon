# g1_sensors — the G1's `/tf` chain (AGENTS.md §10.1)

The robot publishes no `/tf`. This package produces it on the host from `/lf/lowstate`:

```text
/lf/lowstate ─► lowstate_to_joint_states ─► /joint_states ─► robot_state_publisher (G1 29-DoF rev 1.0 URDF)
                (stamped on the robot clock)                          │
                                                                      ▼
robot_center ─► pelvis ─► waist yaw/roll/pitch ─► torso_link ─► mid360_link ─► livox_frame
     (glue)          └─► imu_in_pelvis ─► dog_imu_link      └─► d435_link ─► camera_link
                                          (glue)                              (glue)
```

`odom -> robot_center` comes from `g1_mapping` (launch it with `static_tf:=false`). Read-only
towards the robot: it subscribes to `/lf/lowstate` and the LiDAR IMU and publishes `/joint_states`,
`/tf`, `/tf_static`, `/robot_description`.

The same launch bridges `/lf/bmsstate` (`unitree_hg/BmsState.soc` = 0..100%)
to `/battery_state` (`sensor_msgs/BatteryState.percentage` = 0.0..1.0). Missing
BMS input produces no battery messages; an invalid SOC is published as NaN and
blocks the Loco gateway. To test only this read-only bridge without
`tf_chain.launch.py`, run:

```bash
ros2 run g1_sensors bms_to_battery_state --ros-args \
  -r bms:=/lf/bmsstate -r battery_state:=/battery_state
```

## Run

Live, in the robot-connected container:

```bash
ros2 launch g1_sensors tf_chain.launch.py                 # rviz:=true for RobotModel + TF + LiDAR
ros2 launch g1_mapping mapping.launch.py static_tf:=false
```

The `tf_chain` launch is the **single runtime owner** of the taped-mount provisional
`camera_link -> oak-d-base-frame` mount TF. Keep the RealSense and OAK-D drivers running for
their own camera-internal `/tf_static` links. After starting `tf_chain`, inspect the mount
and the connected optical frames with:

```bash
ros2 run tf2_ros tf2_echo camera_link oak-d-base-frame
ros2 run tf2_ros tf2_echo camera_color_optical_frame oak_rgb_camera_optical_frame
```

The current calibration artifact is stored on the G1 at
`/home/unitree/g1_calibration/oakd_livox_taped_20260927.yaml` for recovery; the prior
LiDAR and RGB results are retained separately. This artifact is **data**, not
a second publisher: do not run an additional `static_transform_publisher` on the Orin for this
child frame while `tf_chain` is running.

Run it during every capture so `/tf`, `/tf_static` and `/joint_states` land in the bag. On a bag
recorded without it (replay only in the `SIM=1` container, domain 77):

```bash
ros2 launch g1_sensors tf_chain.launch.py use_sim_time:=true
ros2 launch g1_mapping mapping.launch.py use_sim_time:=true static_tf:=false
ros2 bag play <bag> --clock 200
```

Topic names, the joint order, and the glue frames live in `config/g1_sensors.yaml`.

## Offline OAK-D depth ↔ MID-360 LiDAR extrinsic check

`register_oak_livox` reads **two finalized MCAP bags directly**; it does not replay data,
publish TF, or write to the robot. One bag must be under `<capture>/oak/` with
`/oak/stereo/image_raw`, `/oak/stereo/camera_info`, `/oak/rgb/camera_info` and
`/tf_static`. The other must be under `<capture>/livox/` with
`/utlidar/cloud_livox_mid360` and `/tf_static` from `g1_sensors tf_chain` (including
the existing provisional `camera_link -> oak-d-base-frame` mount). Record both on the
**same laptop clock** while the G1 stands still, even if OAK needs an isolated DDS
domain (78) to avoid the Orin's Foxy/CycloneDDS crash on the Unitree domain (0).
Keep raw LiDAR scans and depth messages rather than resampled map clouds.

To make a repeatable stationary capture from the **wired laptop** (with the OAK
driver already running on the Orin's `eth0` in domain 78, and LiDAR/TF in domain 0):

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/warsaw/g1_ws/install/setup.bash
ros2 run g1_sensors capture_oak_livox --duration 25 \
  --oak-mxid 14442C102106F1D000
```

It discovers the G1 Ethernet interface, checks both domains before recording,
then writes `<output-root>/oak_livox_geometry_<timestamp>/{oak,livox}/` and
`capture_info.yaml`. It stops and finalizes **only its two rosbag processes**;
the command neither starts/stops somebody else's camera driver nor actuates the
robot. Use different stationary G1 headings or locations in subsequent takes,
with a room corner and box/table overlapping both depth sensors. The first
25-second capture used roughly **1.3 GiB** of disk: check space before longer
takes. Pass the MXID printed in the driver's startup log for the actual camera
in each take; do not reuse this example value if the camera changes.

Observed on 2026-09-26: OAK depth is `1280×720`, `16UC1` **millimetres**, in
`oak_rgb_camera_optical_frame`, registered to RGB. Livox XYZ is in `livox_frame`;
its ~10 Hz clouds contain many (0,0,0) points and its per-point time is in
nanoseconds. For stationary scenes the tool accumulates scans within a window,
filters invalid ranges, back-projects OAK Z-depth with its RGB intrinsics,
voxel-downsamples both, crops LiDAR to the OAK frustum, and fits one shared
point-to-point ICP transform using the existing mount only as a *rough seed*.
The OAK MXID observed on this depth capture (`14442C102106F1D000`) differs from
the earlier RGB calibration camera (`14442C1001EAEFD000`), so verify the physical
camera/mount before trusting the seed.

```bash
cd ~/workspace/warsaw/g1_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select g1_sensors
source install/setup.bash
ros2 run g1_sensors register_oak_livox \
  ~/g1_camera_calibration/oak_livox_geometry_20260926_221118
```

The output is `<capture>/registration.yaml`: initial and fitted Livox↔OAK matrices,
nearest-cloud distances, direct OAK depth-ray residuals, and temporal holdout
results. No TF is published automatically. This recorded scene is only **one
physical viewpoint**: its held-out seconds share the same geometry and cannot
validate a mount update on their own. Capture several *independent* room/box views
with full stops. Pass three or more capture directories to the same command to fit one
shared extrinsic on all but the last: the **entire last capture is held out** as an
independent viewpoint. The tool rejects different OAK MXIDs, camera intrinsics or
recorded initial TFs across captures. Validate a shared transform before replacing
the one OAK mount link. Because OAK and Livox are on the torso, the relative
transform is constant across robot poses; never run competing publishers for
`oak-d-base-frame`.
If a multi-view ICP run exceeds the conservative correction bound, do not simply
relax the limit. Use `--candidate <first_capture>/registration.yaml` with all
three captures: this **does not refit**, but checks whether the first-view
estimate reduces errors in both other scenes. It also writes a proposed
`camera_link -> oak-d-base-frame` pose for **manual** review, not publication.
The tool also saves red-before and green-after LiDAR projections onto a held-out
OAK RGB frame next to its YAML report; check these against visible scene edges.

For a **taped or otherwise temporary mount** where exactly two new scenes must
produce a provisional TF, `--fit-all <scene1> <scene2>` uses both scenes in a
single trimmed ICP. The output explicitly has **no independent holdout**, keeps
`ready_for_tf_publication: false`, and saves before/after projections for both
scenes. Recheck the TF whenever the tape/mount shifts; do not mix bags from the
old physical mount.

The 2026-09-27 taped-mount take used
`oak_livox_geometry_20260927_044430` and `oak_livox_geometry_20260927_044703`.
The joint result and its no-holdout caveat are preserved in
`config/oakd_livox_taped_20260927.yaml` and in the first capture's
`registration_joint_taped_20260927.yaml`. `g1_sensors.yaml` publishes **only** that
mount pose when `tf_chain` next starts. The older `oakd_livox_provisional.yaml`
remains an archive for the previous, non-taped mount.

## Design notes

- **URDF:** `urdf/g1_29dof_rev_1_0.urdf`, vendored unchanged from `third_party/unitree_ros`. The
  robot is the 29-DoF model; every rev 1.0 variant (with or without hands) has the same sensor
  mounts. Rev 1.0 mounts `mid360_link` upside down (roll π), which matches the data, so
  `mid360_link -> livox_frame` is identity. The older `g1_29dof.urdf` has it upright: wrong for
  this robot.
- **Joint order:** `LowState.motor_state[0..28]` = the URDF's revolute joints in file order
  (unitree_sdk2 `G1JointIndex`, waist yaw = 12).
- **Stamps:** `LowState` has no header. Joint states are stamped on the **robot clock**: receive
  time + the median of (LiDAR IMU header stamp − receive time) over 2 s. `/tf` therefore
  lines up with LiDAR and IMU stamps even when the laptop clock is off (it was 73–76 s ahead before
  the laptops were synced to the robot, `g1_ws/README.md` §5). In replay the
  estimate is only as fine as `/clock`, so play with `--clock 200`.
- **Rate:** 20 Hz from `/lf/lowstate`, live and on survey bags. Measured live: it is as fresh as
  the 1 kHz `/lowstate` (tick lag 0–1 ms). Reading the 1 kHz streams in this Python node cost ~70 %
  of a CPU core. The sensors are on the torso, so only the three waist joints move them.
- **No Unitree SDK in the process** (AGENTS.md §5): `LowState` is read with the `unitree_hg`
  ROS 2 messages over the normal ROS 2 graph.

## Checks on `bags/probe_standing_live` (robot standing, 2026-09-25)

| Check | Result |
|---|---|
| TF `robot_center -> livox_frame` at every LiDAR stamp | 400 / 400 |
| `/joint_states` stamps vs the robot clock | 0.0 ms (laptop was 73.3 s ahead) |
| floor below `livox_frame` (LiDAR, 1.5–3 m ring) → pelvis height | 1.240 m → 0.768 m |
| pelvis height from the URDF feet | 0.790 m (`/dog_odom` z: 0.738 m) |
| LiDAR floor normal vs LiDAR IMU gravity (URDF-free) | 0.35° |
| LiDAR IMU vs torso IMU (`/secondary_imu`) through the URDF | ≤ 0.35° |
| torso-side sensors vs pelvis IMU (`/dog_imu_raw`) through the waist | ~1.5° (unresolved: waist encoder zero or IMU mounting) |
| `robot_center -> livox_frame` | roll 179.6°, pitch 3.7° (URDF 2.9° + waist 0.8°), z 0.472 m |
| `robot_center -> camera_link` | pitch 48.4° down, z 0.473 m |

`g1_mapping` with `static_tf:=false` on this chain: 0 lost scans with both `imu_source:=dog` and
`livox`.

## Open

- `d435_link -> camera_link` identity is unverified: check in RViz when the RealSense runs.
- OAK-D: `g1_sensors.yaml` now includes the **taped-mount provisional** OAK
  depth/Livox-derived `camera_link -> oak-d-base-frame` link. Its joint-fit source
  and no-holdout limitation are in `config/oakd_livox_taped_20260927.yaml`.
  Previous LiDAR and ChArUco calibrations are archived separately. The OAK driver publishes the
  internal `oak-d-base-frame -> oak -> oak_rgb_camera_optical_frame` chain; RealSense
  publishes `camera_link -> camera_color_optical_frame`. Publish the mount link **only
  through this `tf_chain` launch**, never also through another static TF node. Check
  LiDAR/OAK depth-cloud alignment before relying on its 3D POIs. If the OAK mount moves,
  remeasure and replace the calibration.
- The ~1.5° pelvis-IMU disagreement tilts the map by that much when `/dog_imu_raw` is the
  gravity reference; `imu_source:=livox` avoids it (its IMU is rigid with the LiDAR). Decide
  with `compare_imu_sources` on a walking bag.
- `robot_center -> pelvis` identity: `/dog_odom` height is 3–5 cm lower than the pelvis height
  from the LiDAR and the URDF feet. Irrelevant for mapping (odometry comes from ICP).
