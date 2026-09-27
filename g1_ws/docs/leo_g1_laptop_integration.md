# G1 ↔ laptop ↔ Leo Rover integration

This is the connection and frame contract for handing the G1 RTAB-Map scene to Leo without changing the Leo team's ROS publishers. Do not publish motion commands as part of this integration.

## Status (2026-09-27): one wall tag, three worlds

The same physical AprilTag (`tag36h11` ID 0, black square 0.160 m) now links **three** frames: the G1 map, the AR glasses and Leo.

```text
                        wall AprilTag 36h11 ID 0 (0.160 m)
            G1 chest OAK-D │        Spectacles │        Leo front OAK-D │
                           ▼                   ▼                        ▼
   tag_anchor: map -> ar_tag_0    ar_bridge: map -> ar_world    leo_in_map: map -> leo_odom -> leo_base
                        (all in the G1 RTAB-Map `map` frame; the glasses draw Leo as a blue box)
```

| Step of this doc | Status | Where |
| --- | --- | --- |
| §3 anchor the tag in the G1 map | **implemented, tested on the G1** (1–2 px, ~1 cm stable) | `g1_ar_bridge` `tag_anchor`: static TF `map -> ar_tag_0` (this doc's `g1_map -> shared_tag0`; one owner) |
| §4 place Leo in the G1 map | **implemented, tested in simulation only**; not yet run with the real Leo | `leo_relay` (on Leo, read-only) → UDP → `leo_in_map` (laptop): TF `map -> leo_odom -> leo_base`, `/leo_in_g1/status` |
| Leo marked in the AR glasses | implemented (simulation) | `leo_in_map` → `/ar_glasses/markers` ns `leo` (box of Leo's size, heading line, label) |
| §5 map transfer, Leo Nav2 | not started | — |

Run it: `bash scripts/start_ar_glasses.sh --tag-size 0.16 --orin --leo` (`ar_glasses/SETUP.md`), or by hand: `ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 camera:=oak leo:=true` on the laptop and the relay on Leo (below). Known limits, all visible in `/leo_in_g1/status`:

- **Leo's camera mount is not measured.** Default `leo_camera_xyz: [0, 0, 0]`, `leo_camera_rpy` = forward-looking, upside down (`config/ar_bridge.yaml`, `leo_in_map`). The position error is roughly the camera's offset from Leo's `base_link`; measure it with a tape and set `leo_camera_xyz` and `leo_camera_measured: true`.
- **Tag axis convention:** detectors differ (z out of vs into the tag). `leo_in_map` checks, per sighting, that the tag's normal faces the camera and converts; `last_sighting.flipped_tag_convention` shows which one Leo's detector uses.
- **Clocks:** the tag sighting and the odometry sample are matched on **Leo's** clock (both come from Leo), so the laptop↔Leo offset does not affect the pose; the published TF is stamped with laptop time.
- **Network:** the relay needs the laptop on Leo's hotspot. With the glasses as well, **laptop Wi-Fi and glasses both join Leo's hotspot** (the G1 stays on the Ethernet cable); check that the hotspot lets clients reach each other.

## What is running now

| System | Connection / ROS | Observed interfaces |
| --- | --- | --- |
| G1 | Wired `192.168.123.0/24` robot LAN; laptop ROS 2 Humble, CycloneDDS, domain 0 | Robot publishes raw `/utlidar/cloud_livox_mid360`, IMU, `/lf/lowstate`, etc. Host `g1_sensors` publishes its URDF TF chain; `g1_mapping` owns `map -> odom`, `/map`, `/odom` and `odom -> robot_center`. |
| Leo | Leo hotspot at `10.0.0.1`; ROS 2 Jazzy on the Pi | Existing team's `/leo/...` topics and Leo TF. OAK-D driver started separately on Leo: `/leo/leo_oak/rgb/image_raw`, `/leo/leo_oak/rgb/image_rect`, `/leo/leo_oak/rgb/camera_info`. AprilTag detector: `/leo/detections`, camera optical frame `leo_oak_rgb_camera_optical_frame`, observed tag frame `leo_tag0`. |
| Laptop | One interface to each robot network | Runs mapping and the future *selective* cross-robot handoff. It must have both connections up at once. |

The OAK-D detects **`tag36h11`, ID 0**, not ID 36: the printed `0000` is the ID; `36h11` is the family. The measured black-square edge is **0.160 m**. Leo's detector config is `/home/pi/leo_oak_tag36.yaml` (historical filename despite ID 0). The OAK-D is front-mounted upside down; its position and orientation relative to Leo's base have **not** been measured. A successful camera-to-tag detection is not yet a Leo-base or G1-map pose.

## 1. Connect the laptop to both networks

Keep the G1 wired connection and Leo hotspot connection active simultaneously. Do not put a G1 rosbag replay on live domain 0. Refer to [`../README.md`](../README.md#testing-from-the-computer-connected-to-the-robot-r4r5) for the G1 wired interface, CycloneDDS configuration and laptop clock sync to `192.168.123.161`. Interface names and the G1 host IP must be checked on this laptop rather than copied blindly.

```bash
ip -brief address
ip route get 10.0.0.1
ip route get 192.168.123.161
ping -c 2 10.0.0.1
ping -c 2 192.168.123.161
```

The Leo hotspot should route `10.0.0.1` over Wi-Fi; the G1 address should route over the wired NIC. Leo also has an internal Wi-Fi interface for internet access; do not mistake its `192.168.1.x` address for the robot-hotspot address. From the laptop, Leo can be inspected over SSH:

```bash
ssh pi@10.0.0.1
source /opt/ros/jazzy/setup.bash
ros2 topic list -t
ros2 topic info /leo/detections --verbose
```

The OAK-D view is available while connected to Leo's network at:

```text
http://10.0.0.1:8080/stream?topic=/leo/leo_oak/rgb/image_raw&type=ros_compressed
```

Run G1 discovery in its **own** Humble/CycloneDDS environment, bound to the wired NIC per `../README.md`; check `/map`, `/odom`, `/tf` and `/tf_static` when mapping is running. Check Leo's Jazzy graph from SSH on Leo. Do **not** assume that putting both NICs up makes one ROS 2 process see both graphs: Leo's current ROS graph uses Fast DDS, while the G1 laptop workflow uses CycloneDDS. Verify domain ID, discovery, topic types/QoS and timestamps before choosing a live transport. Keep any cross-graph communication explicit; two isolated processes, one per ROS environment, can exchange only approved data through a local IPC/network protocol on the laptop. A direct DDS bridge is an alternative only after its interoperability has been tested. Neither bridge exists in this repository yet.

## 2. Names and ownership: protect Leo's existing graph

The Leo team owns its existing `/leo/...` topics and Leo odometry/TF. Do not remap, restart or republish their nodes as part of this handoff. Namespace **our host-side G1 exports** under `/g1/...` in the bridge, for example `/g1/map`, `/g1/odom`, `/g1/tag_anchor` and `/g1/scene`; use `/leo_in_g1/...` for derived handoff diagnostics. These are *proposed export names*, not currently published topics. Preserve the `header.frame_id`, timestamps, OccupancyGrid origin/resolution and QoS when exporting.

Do **not** rename the robot's raw Unitree `/utlidar/...`, `/dog_imu_raw` or `/lf/lowstate` topics on the robot. `g1_mapping/config/g1_mapping.yaml` configures its inputs and `map`, `odom`, `robot_center` frame IDs. A ROS topic namespace does **not** rename TF frame IDs embedded in `/tf`, odometry or maps. The working G1 TF chain and Nav2 currently depend on `map -> odom -> robot_center`; do not globally rename it merely to avoid Leo's names. Instead, keep both TF graphs isolated at ingestion and introduce unique frame IDs at the *handoff boundary* (for example `g1_map`, `g1_odom`, `g1_robot_center`, `leo_odom`, `leo_base`). If a combined TF graph is required, translate every exported G1 and Leo frame ID consistently, including transform parent/child IDs and all message headers; do not relay both unmodified `/tf` streams or publish two parents for one child. Leo's actual odometry/base frame IDs must first be read from `/leo/merged_odom` and its TF, not guessed.

In the G1 graph itself, `g1_mapping` remains the **only** owner of `map -> odom`; its ICP odometry owns `odom -> robot_center`. On a combined/export graph, use one map-frame alias and one publisher per transform. Never let a Leo mapper or SLAM Toolbox claim the same `map -> odom` edge.

## 3. Anchor the stationary tag in the G1 map

1. Run `g1_sensors tf_chain` and `g1_mapping` on the live G1 streams (or in isolated replay). Confirm the complete G1 camera TF chain and RTAB-Map map-frame pose. G1 chest OAK-D (verified 2026-09-26/27): `/oak/rgb/image_raw`, `/oak/rgb/camera_info`, `/oak/stereo/image_raw`, frame `oak_rgb_camera_optical_frame`; mount `camera_link -> oak-d-base-frame` from `g1_sensors/config/oakd_livox_taped_20260927.yaml`. Do not reuse Leo camera names.
2. Detect the **same `tag36h11` ID 0, size 0.160 m** from G1 RGB and CameraInfo while G1 stands still and the tag is fixed. Give this G1 *observed-tag* frame a different name from Leo's `leo_tag0`.
3. At the detection timestamp compute `T_g1_map_tag = T_g1_map_g1_camera × T_g1_camera_tag`. Validate repeated observations and freeze/publish a single `g1_map -> shared_tag0` anchor for that stationary tag. A tag moved after anchoring invalidates the anchor.

The tag anchor should have one owner. Do not directly merge both detectors' tag TF frames: they represent observations from different cameras, not two independently owned map anchors. Log the timestamp, frame IDs, measured size and pose quality.

*Implemented:* `ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 camera:=oak` runs `tag_anchor`, the single owner of the static TF `map -> ar_tag_0` (ArUco axes: x right, y up, z out of the wall). It uses only views taken while the camera's `map` pose is still, fits the wall plane in the aligned depth, logs views, reprojection error, distance and `tag_up_vs_map_up_deg`, and publishes `/ar_glasses/anchor_status`. Stand the G1 still 1–1.5 m in front of the tag for a few seconds. After a new mapping session (new `map`), anchor again.

## 4. Place Leo in the G1 map

Leo's detector already reports `T_leo_camera_tag` under `leo_oak_rgb_camera_optical_frame -> leo_tag0`. A camera-to-base extrinsic `T_leo_base_leo_camera` is still needed. For an initial *diagnostic* estimate, a documented approximate front-mount transform including the upside-down roll may be used, but its position must not be silently treated as calibrated. Measure/calibrate it before trusting map-frame navigation or POIs.

For each timestamp-matched observation of the same fixed tag:

```text
T_g1_map_leo_base = T_g1_map_shared_tag0
                  × inverse(T_leo_camera_tag)
                  × inverse(T_leo_base_leo_camera)

T_g1_map_leo_odom = T_g1_map_leo_base
                  × inverse(T_leo_odom_leo_base)
```

*Implemented* as `leo_relay` + `leo_in_map` (`g1_ws/src/g1_ar_bridge`, math in `leo_localization.py`, tests `test_leo_localization.py`, `test_leo_ros.py`). The relay runs on Leo only while a session runs, sent inline over SSH (nothing installed), and only *reads* Leo's TF (`leo_oak_rgb_camera_optical_frame -> leo_tag0`) and `/leo/merged_odom`:

```bash
scp g1_ws/src/g1_ar_bridge/g1_ar_bridge/leo_relay.py pi@10.0.0.1:/tmp/g1_leo_relay.py
ssh -t pi@10.0.0.1 'source /opt/ros/jazzy/setup.bash && python3 /tmp/g1_leo_relay.py --laptop-ip <laptop IP on Leo net>'
# Leo TF on other topics: append  --ros-args -r /tf:=/leo/tf -r /tf_static:=/leo/tf_static
```
Use `ssh -t` (a terminal) so Ctrl-C or a dropped connection also stops the relay on Leo. `scripts/start_ar_glasses.sh --leo` does both steps in one window.

Here `T_A_B` maps coordinates expressed in frame B into frame A. The localization component (on the laptop or Leo) is the **sole** publisher of `g1_map -> leo_odom` in the exported tree (implemented: `leo_in_map` publishes `map -> leo_odom` and `leo_odom -> leo_base` in the G1 graph; Leo's own frames and topics are untouched). Leo's odometry remains the owner of `leo_odom -> leo_base` (its data, republished unchanged under our frame names by `leo_in_map`; nothing is sent back to Leo). Update the map/odom correction from accepted tag observations; do not overwrite Leo wheel odometry. Between sightings, Leo odometry propagates the pose and can drift. Check time synchronization between G1's `.161` clock, laptop and Leo before TF lookups; do not use latest-TF substitutions to conceal large stamp differences.

## 5. Transfer the map separately

Tag localization provides a **transform**, not the RTAB-Map database or occupancy grid. Selectively hand off the G1 `/map` (`nav_msgs/msg/OccupancyGrid`, with its frame ID mapped to the chosen export frame), any required 3D export/RTAB-Map database and map-frame POIs. Choose a map snapshot/file transfer for an initial robot-off test or a controlled `/g1/map` live bridge for continuing mapping. A Leo Nav2 instance, if used later, must consume this map with Leo-specific base/odom frames and have one localization/map-to-odom owner; it must not be brought up just to test AprilTag alignment.

Do not bridge `/cmd_vel`, G1 raw sensors, `/api/sport/request`, whole `/tf` graphs or actuator topics. First acceptance test is read-only: both networks reachable together, G1 map and tag anchor visible, Leo ID 0 detections visible, one coherent transform for Leo's pose in the G1 map, and a physically plausible map position. Only then validate map transfer and navigation in a separate gated step.
