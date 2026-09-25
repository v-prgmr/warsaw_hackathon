# AGENTS.md — Alien Bazaar Hackathon: Unitree G1 Side

## Purpose

This file is the shared engineering context for all coding agents working on the **Unitree G1 side** of the Alien Bazaar hackathon project.

It captures the architecture, decisions, constraints, module boundaries, milestone order, and current implementation assumptions agreed so far.

**Scope of this version:** G1 sensing, metric LiDAR-inertial mapping (RTAB-Map), ROS integration, semantic POI extraction, autonomous survey integration, and the G1-to-Leo handoff boundary.

**Out of scope for this version:** detailed Leo Rover implementation. Leo-side details will be added later.

Agents should treat the decisions in this file as authoritative unless a human explicitly changes them.

**Current architecture update (2026-09-25, evening): RTAB-Map is the mapping backbone and the only `map -> odom` owner. LiDAR (MID-360) + IMU provide geometry and odometry; a chest-mounted OAK-D RGB-D camera provides color, semantics, and POIs. Where older notes mention RGB-D as the geometry source, this architecture wins.**

---

# 1. Project Goal

Use a **Unitree G1 EDU** as a mobile survey / scene-understanding embodiment.

The G1 should:

1. Observe an indoor scene using its LiDAR (MID-360), IMU, and a chest-mounted OAK-D RGB-D camera.
2. Build a **metric map from LiDAR + IMU** using **RTAB-Map** as the mapping backbone.
3. Use the OAK-D RGB-D camera for color, semantic perception, and POI localization — not for odometry or map geometry.
4. Produce a robotics-ready scene representation: metric trajectory, 3D map (colored where RGB-D is attached), 2D occupancy map, and a defined ROS world frame.
5. Allow a natural-language object query such as `"red bottle"` or `"box on the table"`.
6. Convert the queried object into a **3D Point of Interest (POI)** in the shared map frame.
7. Eventually explore autonomously with frontier exploration (m-explore for ROS 2) on top of Nav2 plus a G1 locomotion executor.
8. Hand the metric scene + POI + supporting context to the Leo Rover side.

Primary concept:

```text
G1 surveys
    ↓
MID-360 LiDAR + IMU (geometry, odometry)   +   OAK-D chest RGB-D (color, semantics)
    ↓
RTAB-Map LiDAR-inertial mapping
    ↓
metric 3D map + trajectory + 2D occupancy grid
    ↓
natural-language query on RGB keyframes
    ↓
3D POI in map frame
    ↓
handoff to Leo Rover
```

The G1 is the **survey + spatial understanding embodiment**.

---

# 2. Primary Demo Philosophy

Keep the project modular and sequential.

Do not make the entire demo depend on every component working simultaneously.

Preferred development order:

```text
sensor acquisition + rosbag (LiDAR, IMU, TF mandatory)
    ↓
RTAB-Map LiDAR-inertial metric mapping (g1_mapping)
    ↓
TF sanity check + physical-measurement validation
    ↓
RViz + Nav2-compatible map outputs
    ↓
semantic POI (RGB-D keyframes captured while standing)
    ↓
autonomous G1 exploration (m-explore → Nav2)
    ↓
Leo handoff
```

For the early milestones, moving the G1 manually or via teleoperation is acceptable.

**Autonomous locomotion must not block mapping / POI development.**

---

# 3. Hardware Assumptions

Current target hardware:

- Unitree G1 EDU
- Onboard / attached NVIDIA Jetson Orin-class compute
- Head RealSense RGB-D camera (points ~48° down; not used for semantics)
- Chest-mounted OAK-D RGB-D camera (added by the team for semantics; exact model to be verified)
- Existing Unitree 3D LiDAR stream
- G1 IMU / robot state
- Ethernet available for development
- Wi-Fi may be used later, but Ethernet is preferred during bring-up

Important hardware details that must be verified on the real robot:

- exact RealSense and OAK-D models
- exact OAK-D mount position on the torso
- exact Jetson model / RAM
- exact RGB/depth ROS topics
- exact LiDAR topic and frame
- exact camera-to-base transform
- exact LiDAR-to-base transform

Do not hardcode unverified hardware details.

---

# 4. ROS / OS Decisions

## Robot side

The G1 onboard stack runs **ROS 2 Foxy** / Unitree DDS interfaces.

## Developer machine

Primary developer environment:

- Ubuntu 22.04 Jammy
- ROS 2 Humble
- x86_64

ROS 2 Humble is the preferred host-side distribution because:

- `rl_hnav` explicitly targets Ubuntu 22.04 + ROS 2 Humble
- Nav2 / SLAM Toolbox support is straightforward
- Unitree ROS 2 tooling supports host-side Humble workflows

Ubuntu 24.04 teammates may use a compatible container / environment later. Do not change the main project architecture to Jazzy unless required.

---

# 5. DDS / RMW Rule

The project already uses a **two-DDS-stack / bridge pattern** to avoid known CycloneDDS / XTypes conflicts between Unitree communication and normal ROS 2 components.

Do not collapse Unitree low-level DDS and normal ROS 2 communication into a single process unless the interoperability issue has been explicitly resolved.

General rule:

```text
Unitree DDS side
    ↓
bridge / isolated process boundary
    ↓
normal ROS 2 topics
```

Re-use the established `unitree_bridge` / isolated-DDS pattern where needed.

Avoid introducing unnecessary RMW changes once a working configuration is established.

---

---

# 6. G1 Software Architecture

The G1 side is divided into six gross modules.

```text
┌────────────────────────────┐
│ 1. Sensor Acquisition      │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 2. RTAB-Map LiDAR-Inertial │
│    Mapping                 │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 3. Frame / Physical        │
│    Validation              │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 4. Scene / Map Outputs     │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 5. Semantic Query / POI    │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 6. Leo Handoff Boundary    │
└────────────────────────────┘
```

One cross-cutting / later module sits beside this core path:

```text
A. Nav2 + m-explore (explore_lite) + G1 locomotion executor
   → autonomous exploration (predefined viewpoints as fallback)
```

## Primary mapping decision

**RTAB-Map is the mapping backbone and the only `map -> odom` owner. LiDAR (MID-360) + IMU are the geometry and odometry sensors; the chest-mounted OAK-D RGB-D camera provides color, semantics, and POIs.**

| Source | Responsible for |
|---|---|
| MID-360 LiDAR + IMU | odometry, 3D map geometry, 2D occupancy grid, loop closure (ICP proximity) |
| OAK-D RGB-D (chest) | map coloring, semantic detection, POI back-projection (aligned depth), visual loop closure when lighting allows |
| RealSense RGB-D (head) | nothing on the critical path (looks at the floor); optional near-field depth |
| G1 URDF + `joint_states` | all sensor extrinsics, via `/tf` (see §10.1) |

Why (observed on the robot and in bags, 2026-09-25):

- the head RealSense has a narrow FOV and points ~48° below the horizon (URDF `d435_joint`), so it mostly sees the floor. It is useless for semantics too, hence the chest-mounted OAK-D.
- frames blur while the G1 walks, and the latest captures were in low light. RGB-D visual odometry lost tracking on every survey bag tried (`full_survey_take_01`: 71 % of frames lost, 113 map fragments).
- the MID-360 covers 360°, is independent of lighting, and ICP odometry on the same bag tracked 1528/1529 scans and closed the loop (13 LiDAR loop closures with `g1_mapping` defaults, one map).
- RTAB-Map consumes LiDAR clouds directly, still produces the 2D occupancy grid, pose graph, and TF that Nav2 needs, and can attach RGB-D to map nodes for color.
- metric scale comes from LiDAR ranges.

## Nav stack ownership rule

Only **one** mapping/localization stack should own `map -> odom` at a time.

Primary plan:

```text
RTAB-Map
    ↓
map -> odom
/map
metric 3D scene
    ↓
Nav2
```

`rl_hnav`'s existing SLAM Toolbox setup remains a useful fallback/reference, but **do not run SLAM Toolbox and RTAB-Map simultaneously if both publish `map -> odom`**.

If RTAB-Map's navigation-map path proves unstable during the hackathon, a permitted fallback is:

```text
RTAB-Map = metric 3D / semantic scene
SLAM Toolbox = Nav2 2D map/localization
```

but that should be a deliberate fallback, not the default architecture.

## Runtime TF / topic ownership (live and replay)

The robot only publishes raw sensor streams. Everything else comes from nodes **we** run on the host. Each item below has exactly **one** publisher:

| TF / topic | Owner |
|---|---|
| `map -> odom`, `/map`, `/cloud_map` | `g1_mapping` (RTAB-Map) |
| `odom -> robot_center`, `/odom` | `g1_mapping` `icp_odometry` (default); `odom_to_tf` from `/dog_odom` only in `odom_source:=dog_odom` |
| `robot_center -> pelvis -> … URDF links` | `robot_state_publisher` (G1 URDF) + `/joint_states` from the `/lowstate` bridge (§10.1) |
| frames not in the URDF (`livox_frame`, `camera_link`, OAK-D mount, `robot_center <-> pelvis`) | one static-TF launch (§10.1); `g1_mapping static_tf:=true` only for legacy bags without `/tf` |
| camera-internal frames | the RealSense / OAK-D drivers (`/tf_static`) |
| `/scan` | `pointcloud_to_laserscan` (from the `rl_hnav` bridge) |
| `/cmd_vel` → legs | Nav2 → locomotion executor (Loco/Sport client, or `rl_hnav` under §25) |
| frontier goals | `explore_lite` (m-explore) |

When `rl_hnav`'s real-robot bridge (§14) runs together with `g1_mapping`, **disable** its `odom_tf_bridge` (`odom -> robot_center`), its static `robot_center -> lidar` transform, and SLAM Toolbox. Keep its `/scan` pipeline and `/cmd_vel` consumer. Two publishers of the same transform make TF jump; `/dog_odom` is also ~2× short (§8).

---

# 7. Module 1 — Sensor Acquisition

## Goal

Provide synchronized scene observations from the G1 for RTAB-Map LiDAR-inertial mapping, semantic perception, and offline replay.

## Sensors

Primary (geometry / odometry):

- LiDAR PointCloud2 with per-point time
- IMU
- `/lowstate` (joint positions → `/joint_states` → `/tf` via the URDF)
- `/tf_static`
- timestamps

Semantic (color / POI), from the chest OAK-D:

- RGB
- depth aligned to RGB
- camera intrinsics

Useful:

- robot odometry (`/dog_odom`)
- robot state (battery)

## What the robot publishes vs what we run

The robot/Orin publishes only raw streams, all from Unitree's bare-DDS services (no ROS nodes). Verified live on 2026-09-25 (listen-only, robot standing): `/utlidar/cloud_livox_mid360` 10 Hz, `/utlidar/imu_livox_mid360` 200 Hz, `/dog_odom` / `/dog_imu_raw` / `/lowstate` / `/secondary_imu` ~1 kHz, `/lf/lowstate` / `/lf/bmsstate` 20 Hz (the `/lf/*` topics are 20 Hz copies). `/tf_static` appears only while the RealSense driver runs; no camera driver was running during the check. The robot is the **29-DoF** model (confirmed by the team; `/lowstate` shows 29 active joints, `mode_machine` 5). Unitree's own SLAM topics (`/unitree/slam_*`, `/global_map`) exist but are idle; do not use them as a second map owner.

**The robot does not publish `/tf`.** Nothing needs "enabling" on the robot: `/tf`, `/joint_states`, `/odom`, `/map`, `/scan`, and POIs only exist while our host nodes run (§6 ownership table, §10.1). A bag contains `/tf` only if those nodes ran during the capture.

## OAK-D (chest camera, semantics)

The head RealSense points at the floor, so a **chest-mounted OAK-D** is the semantic / POI camera.

- Driver: `depthai-ros`. Topic names, frame names, model, and the host it is plugged into (Orin or laptop) must be verified on the hardware and then added to `g1_recorder/config/survey.yaml` and `g1_mapping/config/g1_mapping.yaml`. Do not guess them.
- Publish RGB + depth **aligned to RGB at the same resolution** + CameraInfo, like the RealSense profile.
- If the OAK-D is plugged into the laptop rather than the Orin, its stamps come from a different clock than the robot's streams (see the clock note in the addendum).
- Mounting follows §25.2: ≤ 1 kg of extra hardware in total, on the torso or head, velcro / zip ties / manufacturer holes, no traces on removal, and it must not cover the LiDAR, cameras, vents, or indicators.
- Its extrinsic is a **measured** static transform `torso_link -> <oak frame>`, checked in RViz (§10.1). No calibration.
- RGB keyframes for POI queries are captured **while the robot is standing still** (settled); walking shakes the chest camera too.

## RealSense (head)

The head RealSense stays in the stack only as an optional near-field depth source; it is off the critical path. If it is used, run `realsense-ros` on the Orin (or on the laptop if the USB cable reaches it) with RGB and aligned depth at the **same resolution** (e.g. the `640x480x15` profile in `g1_recorder/config/realsense_rtab.yaml`). Topics: `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`.

## LiDAR

The existing G1 / Unitree LiDAR stream is the **primary geometry and odometry sensor**.

Do not create a new LiDAR driver if the robot already publishes the required cloud.

Observed in bags (2026-09-25):

```text
topic       /utlidar/cloud_livox_mid360   sensor_msgs/PointCloud2, ~10 Hz, ~20k points
frame_id    livox_frame                   mounted upside down: the floor lies along −z, ~1.23 m away
fields      x y z intensity (float32), ring (uint16, 0–3), time (float32)
time        NANOSECONDS relative to header.stamp (0 … ~1.0e8 within a 100 ms scan)
zeros       ~38 % of points are (0,0,0) → filter before use
```

RTAB-Map (`rtabmap_conversions`) reads a float32 `time` field as **seconds**, so the cloud must be converted before deskewing. `g1_mapping` does this.

IMU sources: `/dog_imu_raw` (~1 kHz, frame `dog_imu_link`, orientation populated) and `/utlidar/imu_livox_mid360` (the MID-360's internal IMU, 200 Hz, rigidly attached to the LiDAR). The LiDAR IMU reports acceleration in **g** and **no orientation**; Its raw gyro has ~0.6–0.9 °/s bias (all G1 gyros do; Unitree compensates it onboard in `/dog_imu_raw`'s orientation). `g1_mapping imu_source:=livox` converts it and runs `imu_complementary_filter` with gyro-bias estimation (standing: yaw drift 0.005 °/s, vs 0.94 °/s without bias estimation). `/dog_imu_raw` stays the default until a walking bag compares both.

LiDAR also feeds obstacle perception / `/scan` for Nav2.

## Recording

Record raw sensor streams to `rosbag2`.

The entire mapping pipeline must support replay with the G1 powered off.

**Mandatory** (a bag without these is not a canonical bag):

```text
/utlidar/cloud_livox_mid360
/dog_imu_raw
/dog_odom
/lf/lowstate     # 20 Hz copy of /lowstate (joint states): /tf can be regenerated offline from it
/tf_static
```

`/tf` is recorded only if our TF nodes (`/lowstate` bridge + `robot_state_publisher` + static glue, §10.1) run during the capture. Run them during capture when possible; `/lf/lowstate` is the fallback for regenerating `/tf` at replay. The full-rate `/lowstate` (~2.4 MB/s) is only needed in `live_run` bags.

Semantic streams: OAK-D RGB + aligned depth + CameraInfo (names to be verified, see above). The RealSense streams are optional.

Also record: `/utlidar/imu_livox_mid360`, `/secondary_imu`, `/lf/bmsstate`. Check every bag's message counts: a topic in the profile that nobody published is silently missing from the bag.

For every canonical capture, also write down 2–3 tape-measured dimensions next to the bag name (M2).

The canonical rosbag is the shared development fixture for mapping and semantic perception.

---

# 8. Module 2 — RTAB-Map LiDAR-Inertial Mapping

## Goal

Build the primary **metric robotics map from the MID-360 LiDAR + IMU**, with RTAB-Map as the backbone. Package: `g1_ws/src/g1_mapping`.

`g1_mapping` is a **host-side** stack (laptop / Docker, or the Orin): it consumes `/utlidar/cloud_livox_mid360` + `/dog_imu_raw`, which the robot already publishes, and produces `/map`, `/odom`, `/cloud_map`, and `map -> odom -> robot_center`. Run it live next to Nav2 / m-explore, or on a replayed bag (`use_sim_time:=true`, `ros2 bag play --clock`).

2D grid notes: `Grid/MaxGroundHeight` must be set (relative to `robot_center`, ~0.72 m above the floor). With normals-only ground segmentation, horizontal tabletops were classified as ground, i.e. free space. The IMU is also fed to the `rtabmap` node, so gravity constraints keep the map level.

RTAB-Map is the core mapper for V1.

Conceptually:

```text
MID-360 PointCloud2 (per-point time)        IMU
                │                            │
   time ns→s + zero-point filter             │
                │                            │
                ▼                            ▼
        deskewing (IMU-stabilized frame) ◄───┘
                │
                ▼
      LiDAR odometry  (odom -> robot_center)
                │
                ▼
            RTAB-Map  ◄── optional RGB-D per node (color, visual loop closure)
                │
      ┌─────────┼──────────┐
      ▼         ▼          ▼
 metric poses  3D map    2D map
      │                    │
      └──────── TF ────────┘
```

## Odometry source

Selectable in `g1_mapping` (`odom_source`):

1. **`icp` (default):** `rtabmap_odom icp_odometry` on the MID-360 cloud with IMU, following `rtabmap_examples/lidar3d.launch.py`. No extra dependencies. On `full_survey_take_01` it tracked 1528/1529 scans with zero resets.
2. **`dog_odom` (fallback):** `/dog_odom` bridged to TF. It runs at the IMU rate (~1 kHz) and appears to be leg/IMU odometry, not Unitree LIO. On `full_survey_take_01` it **underestimated distance by ~2×**: a similarity fit to the ICP trajectory needs scale 1.97 (RMSE 0.42 m after scaling, 1.32 m without). Use it only as a fallback or motion guess.
3. **Upgrade path, only if ICP drifts:** FAST-LIO2 (`hku-mars/FAST_LIO`, `ROS2` branch). It accepts our PointCloud2 through its Velodyne-type handler (`lidar_type: 2`, `scan_line: 4`, `timestamp_unit: 3` = ns); its MID360 handler expects Livox fields and ignores per-point time. It needs `livox_ros_driver2` at build time. Its `camera_init -> body` TF must not compete with RTAB-Map's `map -> odom`. Point-LIO is ROS 1 only.

Expected useful outputs include:

- metric robot / sensor trajectory
- metric 3D point-cloud map (LiDAR; RGB-colored where RGB-D is attached)
- 2D occupancy grid for navigation
- pose graph / loop-closure constraints
- map database
- `map -> odom` transform while mapping/localizing
- ROS map state usable by Nav2

Exact topic and frame names live in `g1_mapping/config/*.yaml`, not in code.

## Input expectations

Primary:

```text
LiDAR PointCloud2 (per-point time)
IMU
TF (G1 URDF + joint_states, /tf_static)
```

Optional / supporting:

```text
RGB + aligned depth + CameraInfo   (color, visual loop closure)
/dog_odom                          (fallback odometry)
```

Recommended progression:

```text
LiDAR + IMU (ICP odometry)
    ↓
verify one-session map, loop closures, level floor
    ↓
attach RGB-D for color
    ↓
tune / consider FAST-LIO2 only if ICP odometry drifts
```

## Keyframe handling

RTAB-Map manages its own mapping nodes / keyframes internally.

RGB keyframes for semantic POI queries are captured **while the robot stands still** (after settling), not while walking.

The existing `keyframe_manager` package is still useful for:

- offline inspection
- Grounding DINO + SAM2 semantic querying
- creating reproducible image/depth fixtures

It is **not a dependency for primary mapping**.

---

# 10. Module 3 — Frame Chain and Physical Validation

LiDAR ranges give metric scale, so **global metric-scale recovery is not a core problem**.

Because the LiDAR now builds the map, **it is no longer an independent check of the map**. Independent validation comes from physical measurements (§10.2).

The remaining geometric problems are:

1. sensor extrinsics (from the G1 URDF, §10.1)
2. consistent ROS frame ownership
3. map / odom / base / camera / LiDAR TF correctness
4. independent metric validation by physical measurement
5. OAK-D depth ↔ LiDAR consistency (the camera extrinsic matters for POIs)

## 10.1 Frame chain and extrinsics

Maintain an explicit TF chain:

```text
map
  ↓                      (RTAB-Map)
odom
  ↓                      (LiDAR odometry, or /dog_odom fallback)
robot_center
  ↓                      (G1 URDF via robot_state_publisher + joint_states)
pelvis → waist joints → torso_link
  ├── mid360_link → livox_frame
  ├── d435_link   → camera_link → camera_color_optical_frame
  └── <OAK-D mount> → OAK-D driver frames          (measured, static)
```

**Extrinsics come from the G1 URDF** (`unitree_ros/robots/g1_description`: `d435_joint`, `mid360_joint` on `torso_link`), published on `/tf` by `robot_state_publisher` from `joint_states`. The waist joints move the head relative to the pelvis, so joint states are required.

**The robot does not publish `/tf`.** We produce it on the host:

1. `/lowstate` → `/joint_states` bridge (the isolated-DDS `unitree_bridge` / `lowstate_to_jointstate` pattern, §5)
2. `robot_state_publisher` with the URDF that matches the robot variant (29-DoF, confirmed; which `g1_29dof*` file, e.g. with or without hands, still needs verifying) from `third_party/unitree_ros/robots/g1_description`
3. one static-TF launch for the glue frames below
4. `odom -> robot_center` from `g1_mapping` (§6 ownership table)

Run 1–3 during captures so `/tf` lands in the bag. Once they run, launch `g1_mapping` with `static_tf:=false`.

**No calibration.** Verify only with an RViz sanity check:

- the floor is horizontal and at the same height in the LiDAR cloud and in the OAK-D depth cloud
- LiDAR and depth clouds overlap on the floor and walls
- the camera frustum points where the RGB image shows

Frame-convention glue that is not in the URDF must be an explicit, documented static transform:

- `mid360_link -> livox_frame`: observed upside down (floor along −z of `livox_frame`)
- `d435_link -> camera_link`: the realsense-ros root frame
- `robot_center <-> pelvis`: unverified
- `dog_imu_link <->` the URDF IMU link: unverified
- `torso_link -> <OAK-D mount>`: measured by hand at mounting time; write the numbers down

Never pass XYZ coordinates without a `frame_id`.

Verify `T_base_camera`, `T_base_lidar`, `map -> odom`, and `odom -> base` before using the map for G1-to-Leo handoff.

## 10.2 Physical-measurement validation (independent check)

For every canonical capture, tape-measure **2–3 dimensions** the LiDAR can see (e.g. wall length, room width, table height or edge length). Write them down **at recording time**, next to the bag name.

Compare them against the RTAB-Map map. The milestone requires numerical agreement within an agreed tolerance (proposed: ±5 cm or ±2 %, whichever is larger), not only visual overlap.

## 10.3 OAK-D depth ↔ LiDAR consistency

The OAK-D depth is an independent range sensor. Overlay the depth cloud on the LiDAR map and check the point-to-plane distance on the floor and walls. This validates the camera extrinsic that POI back-projection depends on.

---

# 11. Module 4 — Scene / Map Outputs

The primary scene representation now comes from RTAB-Map.

Core outputs required by the project:

```text
metric colored 3D scene / cloud
metric camera / robot trajectory
2D occupancy map
map -> odom TF
map database / graph state
```

For Nav2, the important robotics outputs are:

```text
/map
map -> odom
/odom
```

For semantic perception / Leo handoff, the important outputs are:

```text
metric 3D scene
camera poses
map frame
supporting RGB-D observations
```

Use RViz for visualization.

Save reproducible offline artifacts where practical, for example:

```text
RTAB-Map database
exported metric scene cloud / mesh
trajectory
map metadata
semantic POI records
```

The existing `scene_server` package may be adapted to expose a stable project-level scene API independent of the mapping backend.

---

# 12. Module 5 — Natural-Language Query / POI

Do not query the raw 3D point cloud directly in V1.

Use RGB observations for semantic detection, then lift the result into the RTAB-Map metric world.

Preferred pipeline:

```text
natural-language query
    ↓
Grounding DINO
    ↓
2D text-conditioned detection
    ↓
SAM 2
    ↓
object mask
    ↓
aligned OAK-D metric depth
    +
camera pose in RTAB-Map
    ↓
back-project mask into 3D
    ↓
transform into map frame
    ↓
multi-view fusion
    ↓
3D POI
```

Output contract:

```text
POI {
    id
    label
    xyz
    frame_id   # expected: map for handoff
    confidence
    supporting_keyframes
}
```

For V1, use the OAK-D depth as the geometric source for the object whenever possible. POI keyframes come from the chest OAK-D while the robot stands still.

---

# 13. `rl_hnav` Role

`rl_hnav` is a G1 locomotion / navigation execution option. Under the event rules, the preferred fast path is the Unitree high-level Loco/Sport client; `rl_hnav` remains the low-level option when the Section 25 safety requirements are satisfied.

Do not treat it as the mapping / reconstruction system. RTAB-Map is the primary metric mapping backbone.

The useful interface is:

```text
Nav2
    ↓
/cmd_vel
[vx, vy, wz]
    ↓
rl_hnav / rl_sar
    ↓
RL locomotion policy
    ↓
Unitree LowCmd
    ↓
G1
```

The existing `rl_hnav` repository already includes:

- ROS 2 `/cmd_vel` consumption
- Unitree LowState / LowCmd integration
- real G1 / G1 EDU23 path
- EDU23 joint remapping
- command freshness timeout
- real-robot navigation bridge
- `/dog_odom -> /odom`
- LiDAR PointCloud2 -> LaserScan
- SLAM Toolbox integration
- Nav2 integration

Do not rebuild these components unless necessary.

---

# 14. `rl_hnav` Real-Robot Navigation Bridge

Existing real-robot inputs assumed by `rl_hnav` include:

```text
/dog_odom
/utlidar/cloud_livox_mid360
```

Its bridge produces:

```text
/odom
/scan
TF: odom -> robot_center
TF: robot_center -> lidar frame
```

Flow:

```text
/dog_odom
    ↓
odom_tf_bridge
    ↓
/odom + TF

LiDAR PointCloud2
    ↓
pointcloud_to_laserscan
    ↓
/scan_raw
    ↓
scan_restamper
    ↓
/scan
```

Then:

```text
/map
/odom
/scan
NavigateToPose
    ↓
Nav2
    ↓
/cmd_vel
    ↓
rl_hnav
```

The static LiDAR extrinsic in `rl_hnav` currently needs to be verified / calibrated on the actual G1.

**With `g1_mapping` running (default), use only the `/scan` part and the `/cmd_vel` consumer of this bridge.** Disable `odom_tf_bridge`, the static `robot_center -> lidar` transform, and SLAM Toolbox; `g1_mapping` owns `/odom`, `odom -> robot_center`, `/map`, and `map -> odom` (§6 ownership table).

---

# 15. Autonomous Exploration / Survey Behavior

Autonomous G1 exploration is a later milestone (M5).

**Exploration uses m-explore for ROS 2** (`explore_lite`, `robo-friends/m-explore-ros2`, Humble). It is frontier-based: it reads the RTAB-Map `/map` OccupancyGrid, picks the next frontier, and sends Nav2 `NavigateToPose` goals. It is not in the Humble apt repositories, so build it from source with colcon. Predefined survey viewpoints remain the fallback.

Desired behavior:

```text
RTAB-Map /map (LiDAR 2D grid)
    ↓
explore_lite: next frontier
    ↓
Nav2 NavigateToPose
    ↓
/cmd_vel
    ↓
G1 locomotion executor (high-level Loco/Sport client preferred; rl_hnav only under §25)
    ↓
G1 walks
    ↓
goal reached
    ↓
zero velocity
    ↓
settle
    ↓
capture RGB-D keyframe (standing)
    ↓
next frontier  (stop when no frontiers are left; optional return_to_init)
```

`explore_lite` settings for the G1: `robot_base_frame: robot_center`, `costmap_topic: /map`. Pause and resume exploration with `explore/resume` (`std_msgs/Bool`), e.g. to capture keyframes or when the operator needs to stop.

Capture after the robot has stopped / settled.

V1 environment assumption:

```text
indoor
flat floor
controlled area
```

Do not make stairs or rough-terrain locomotion a demo dependency.

---

---

# 16. Milestones

## M0 — ROS / Sensor Connectivity

Goal: reliable access to all required G1 sensor streams and rosbag recording.

Acceptance criteria:

- RGB visible
- aligned metric depth visible
- camera intrinsics available
- LiDAR PointCloud2 visible
- timestamps sensible
- TF / static transforms understood
- rosbag records and replays successfully

Deliverable: one reproducible canonical rosbag.

## M1 — RTAB-Map Metric LiDAR-Inertial Map

Goal: a recorded or live MID-360 + IMU stream produces a coherent metric map and trajectory.

Acceptance criteria:

- `g1_mapping` runs RTAB-Map on a replayed canonical bag (`ros2 bag play --clock`) with LiDAR + IMU odometry
- one mapping session: no odometry resets over the survey
- loop closures where the path revisits a place
- the floor stays level (small z drift over the survey)
- 2D occupancy grid is produced
- map database can be saved / reopened
- optional: RGB-D attached for a colored 3D map

Deliverables:

```text
RTAB-Map database
metric trajectory
metric 3D map (LiDAR; colored where RGB-D is attached)
2D occupancy grid
```

No RGB-D odometry dependency.

## M2 — Frame / Physical-Measurement Validation

Goal: confirm the map is correctly framed and metrically correct against the real world.

The LiDAR builds the map, so it is **no longer an independent check**. M2 therefore requires **physical measurements**.

Acceptance criteria:

- extrinsics come from the G1 URDF via `/tf` + `joint_states` (waist joints); no calibration
- RViz sanity check passes (§10.1): level floor in both LiDAR and depth, overlapping clouds, camera frustum matches the image
- **2–3 tape-measured dimensions**, written down at recording time, agree with the map within the agreed tolerance (§10.2)
- the OAK-D depth cloud overlays the LiDAR map (§10.3)
- TF chain is explicit and inspectable

## M3 — ROS / Nav2-Ready Map Outputs

Goal: the mapping stack provides the world representation required by RViz and Nav2.

Acceptance criteria:

- metric scene visible in RViz
- `/map` available
- `map -> odom` available from the selected map/localization owner
- `/odom` available
- no competing nodes publish conflicting `map -> odom`
- Nav2 can consume the resulting map/TF chain

## M4 — Semantic POI

Goal: natural-language query produces a metric 3D object location in the shared map frame.

Preferred implementation:

```text
Grounding DINO
+ SAM2
+ aligned OAK-D depth (chest camera)
+ RTAB-Map camera pose
```

Expected output:

```text
label
xyz
frame_id = map
confidence
```

## M5 — Autonomous G1 Exploration

Goal: the G1 autonomously explores a bounded area with m-explore (`explore_lite`) frontier exploration on the RTAB-Map `/map`, while mapping continues and RGB-D keyframes are captured when it stands. Predefined survey viewpoints are the fallback.

Primary navigation stack:

```text
RTAB-Map map/localization (/map, map -> odom)
    ↓
explore_lite (frontier goals)
    ↓
Nav2 NavigateToPose
    ↓
/cmd_vel
    ↓
G1 locomotion executor
    ↓
G1
```

Preferred fast / lower-risk executor under event rules:

```text
high-level Unitree Loco/Sport client
Move(vx, vy, wz)
```

`rl_hnav` remains a valid low-level option only if the organizer low-level-control requirements are satisfied.

Acceptance criteria:

- NavigateToPose goal accepted
- G1 reaches a simple indoor goal
- G1 stops safely
- mapping remains coherent
- explore_lite runs several frontier goals in sequence and stops when no frontiers are left
- exploration stays inside the prepared safety zone (physical boundary and/or Nav2 keepout mask)
- `explore/resume` pauses exploration; keyframes are captured while standing

See Section 25 before any actuation.

## M6 — Shared Frame + Leo Handoff

Goal: produce the G1-side handoff package.

Required outputs:

```text
metric scene
POI in map frame
supporting RGB-D observations / keyframes
shared map frame
confidence
```

Expected contract:

```text
SceneHandoff {
    scene
    scene_frame        # expected: map
    POI
    supporting_keyframes
    confidence
}
```

---

# 17. Software Packages / Nodes

Preferred project split:

```text
g1_sensors
g1_recorder
g1_mapping / rtabmap_bringup
scene_server
semantic_query
```

Existing / optional helpers:

```text
keyframe_manager
```

External packages to reuse rather than reimplement:

```text
realsense-ros
rtabmap_ros
Nav2
m-explore-ros2 (explore_lite)   # exploration; build from source, not in Humble apt
rl_hnav / rl_sar where permitted
pointcloud_to_laserscan where needed
```

Recommended distinction:

```text
real-time acquisition / mapping / robot control
    -> ROS 2

offline analysis / semantic-query experiments
    -> Python first

stable scene / POI interfaces
    -> ROS 2 project packages
```

---

# 18. First Development Workflow

Recommended execution sequence:

```text
1. Get ROS 2 Humble working on development machine (Docker image: scripts/run_humble.sh)
2. Connect to G1 over Ethernet
3. Discover existing G1 topics
4. Verify LiDAR / IMU / /dog_odom / TF (/tf from G1 URDF + joint_states) / RGB + aligned depth (same resolution) / CameraInfo
5. Record canonical rosbag (survey profile) + write down 2–3 tape-measured dimensions
6. Replay rosbag with robot off
7. Build g1_mapping (RTAB-Map is already in the Docker image)
8. Run g1_mapping on the replayed bag (ICP odometry + IMU)
9. Verify one-session map, loop closures, level floor, 2D occupancy grid
10. RViz TF sanity check + compare the tape measurements with the map
11. Expose /map + map->odom + scene outputs in RViz
12. Add Grounding DINO + SAM2 -> metric POI (keyframes captured while standing)
13. Integrate Nav2 on the RTAB-Map /map
14. Integrate safe G1 locomotion executor
15. Add m-explore (explore_lite) frontier exploration on top of Nav2
```

Do not start with autonomous locomotion.

The first mapping milestone is now **MID-360 + IMU -> RTAB-Map (g1_mapping) -> metric map**.

---

# 19. Safety Rules for Agents

When interacting with the physical G1:

- prefer Ethernet during bring-up
- default to read-only sensor discovery
- do not publish `LowCmd` unless explicitly instructed
- do not enable real locomotion while debugging perception
- use `publish_lowcmd:=false` / dry-run modes where available
- maintain a `/cmd_vel` freshness timeout
- do not bypass existing G1 safety / motion-switcher logic
- do not assume a simulator-tested command is safe on hardware

The organizer's (x-kom) rules for using the G1 are binding and take precedence over everything in this file. See **Section 25**.

Autonomous motion should only be enabled after:

```text
sensor streams verified
TF verified
odom verified
/cmd_vel visible
rl_hnav dry-run verified
physical safety zone prepared
```

---

---

# 20. Engineering Risks

## Risk 1 — LiDAR-inertial odometry / RTAB-Map tracking quality

Failure modes: wrong deskew timing, geometrically degenerate scenes (long featureless corridors, open space), near-range self-hits, IMU extrinsic or orientation errors.

Mitigations:

- convert the MID-360 `time` field (float32 ns) to seconds before deskewing
- deskew in an IMU-stabilized frame; prefer the LiDAR's internal IMU when it is recorded
- filter zero and near-range points; tune the voxel size for indoor use
- survey in geometrically rich areas; walk slowly
- keep `/dog_odom` available as a fallback / motion guess
- FAST-LIO2 as the upgrade path if ICP odometry drifts
- rosbag every run for repeatable tuning

## Risk 2 — Coordinate-frame mismatch

Mitigations:

- explicit `map`, `odom`, base, camera, and LiDAR frames
- extrinsics from the G1 URDF via `/tf` + `joint_states`; RViz sanity check (§10.1)
- driver frame conventions not in the URDF (e.g. the upside-down `livox_frame`) bridged by explicit, documented static transforms
- ensure only one mapping/localization stack owns `map -> odom`
- never pass unlabelled XYZ coordinates between robots

## Risk 2b — RGB-D semantic observation quality

The head RealSense was useless for semantics (narrow FOV, ~48° down); the chest OAK-D replaces it. Any body camera still shakes while walking and suffers in low light.

Mitigations:

- capture POI keyframes only while the robot stands still
- adequate lighting
- mount the OAK-D so target objects (tables, ~0.5–3 m away) are in view; check the view before recording
- verify the OAK-D stamps share the robot's clock (plug it into the Orin), or align clocks

## Risk 3 — Sensor timing / DDS issues

Mitigations:

- rosbag everything
- preserve timestamps
- use existing isolated DDS / bridge pattern
- avoid unnecessary RMW changes
- verify topics / QoS before adding custom bridges

## Risk 4 — RTAB-Map + Nav2 integration conflicts

Potential failure mode: RTAB-Map and SLAM Toolbox both publish map state / `map -> odom`.

Mitigations:

- choose one map/localization owner
- primary = RTAB-Map
- SLAM Toolbox only as deliberate fallback
- inspect TF publishers before enabling Nav2

## Risk 5 — Autonomous G1 navigation

Mitigation:

- manual survey is valid for M0–M4
- start on flat indoor terrain
- frontier exploration will head for any unexplored gap: bound the area physically and/or with a Nav2 keepout mask, keep crowds, stairs and doors to the outside out of reach (§25.2), and keep an operator with the e-stop
- pause with `explore/resume`; use predefined viewpoints if exploration misbehaves
- prefer high-level Unitree locomotion under event rules
- use `rl_hnav` only after the required harness/supervisor validation

---

# 21. Explicit Non-Goals for V1

Do not spend hackathon time on these unless all core milestones are already stable:

- full multi-robot SLAM
- arbitrary terrain / stairs
- training a new G1 locomotion policy
- end-to-end VLA locomotion
- next-best-view planning beyond m-explore frontier exploration
- direct natural-language querying of raw 3D embeddings
- sophisticated dynamic-object tracking
- general-purpose manipulation on G1
- replacing sensor metric depth (OAK-D / RealSense) with learned depth
- perfect dense LiDAR/RGB fusion
- running RTAB-Map and SLAM Toolbox simultaneously as competing map owners

---

# 22. Core Interfaces

## Sensor side

Geometry / odometry:

```text
LiDAR PointCloud2 (per-point time)
IMU
TF (G1 URDF + joint_states) + TF static
odom (/dog_odom, fallback)
timestamps
```

Semantic (chest OAK-D):

```text
RGB image
aligned metric depth (same resolution as RGB)
CameraInfo
```

## Primary mapping side

```text
LiDAR-inertial odometry (odom -> robot_center)
RTAB-Map metric trajectory
metric 3D map (colored where RGB-D is attached)
2D occupancy map
map database / pose graph
map -> odom
```

## Scene side

```text
shared scene in map frame
/map
camera / robot trajectory
supporting RGB-D observations
```

## POI side

```text
POI {
    id
    label
    xyz
    frame_id
    confidence
    supporting_keyframes
}
```

Expected handoff frame: `map`.

## Navigation side

```text
NavigateToPose
/map
/odom
/scan or obstacle input
/cmd_vel
explore_lite: /map in, NavigateToPose out, explore/resume (std_msgs/Bool), base frame robot_center
```

## G1 locomotion side

Preferred high-level path:

```text
/cmd_vel or equivalent velocity command
    ↓
high-level Unitree Loco/Sport client
    ↓
Move(vx, vy, wz)
```

Optional low-level path, only when Section 25 requirements are satisfied:

```text
/cmd_vel
    ↓
rl_hnav / rl_sar
    ↓
Unitree LowCmd
```

---

# 23. Definition of Success for the G1 Side

Minimum successful G1-side demo:

1. G1 provides LiDAR + IMU and synchronized OAK-D RGB-D observations.
2. RTAB-Map builds a coherent **metric** map and trajectory.
3. LiDAR / physical measurements validate the map geometry and frame setup.
4. The metric colored scene and 2D map are visible in RViz.
5. A natural-language object query returns a plausible metric 3D POI in `map`.
6. The G1-side system outputs a clear map + POI handoff package for Leo.

Stretch:

7. G1 autonomously explores a bounded area with m-explore frontier exploration on Nav2 plus an event-compliant locomotion executor.

---

# 24. Agent Operating Rules

All agents working on this project should:

- read this file before changing architecture
- preserve module boundaries unless there is a concrete reason to change them
- prefer small independently testable milestones
- keep offline replay possible
- log frame IDs and timestamps
- never drop coordinate-frame metadata
- avoid hardcoding robot-specific topics until verified on hardware
- use standard ROS 2 message types where practical
- reuse `rl_hnav` components rather than recreating them
- make hardware actuation opt-in, not default
- keep RTAB-Map as the primary mapping backend
- make outputs inspectable in RViz / saved files
- report assumptions explicitly
- distinguish observed hardware facts from guesses

If an implementation choice conflicts with this file, stop and surface the conflict rather than silently changing the architecture.

---

---

## Day-1 Execution Addendum (2026-09-25)

> Decisions made during Day-1 G1 sensing/reconstruction planning. Supplements — does not replace —
> the sections above. Where this conflicts with an assumption above, this addendum wins for V1.

### Confirmed environment
- **Dev machine:** Ubuntu 22.04 + ROS 2 Humble, x86_64, Intel Iris Xe (**no CUDA**) → laptop is
  orchestration / RViz / native-camera only.
- **DDS:** whole ROS 2 graph on **`rmw_cyclonedds_cpp`** (set `RMW_IMPLEMENTATION`), **not** Fast
  DDS; NIC `enp3s0`. Keep the two-CycloneDDS isolation: SDK's bundled **CycloneDDS 0.10.2** stays
  in a separate process from the system **CycloneDDS 11.x** graph to avoid the XTypes crash.
- **Reuse (already on the dev machine):** `~/unitree_sdk2` (built), `~/unitree_ros2/setup.sh` (DDS
  env), and `~/ros2_ws/src/unitree_bridge/src/lowstate_to_jointstate.cpp` — the working
  isolation-bridge reference (LowState → `/joint_states`). Clone this pattern for any Unitree→ROS
  relay; do not rebuild it.
- **RealSense placement:** run `realsense-ros` natively on the **laptop (Humble)** if the head-cam
  USB reaches it, else on the **Orin**; `align_depth:=true`, `pointcloud.enable:=true`. Verify
  actual topic names on the robot — do not hardcode.

### Mapping architecture update — LiDAR-inertial (2026-09-25, evening)

**Supersedes earlier notes that used RGB-D as the geometry source.** RTAB-Map stays the backbone and the only `map -> odom` owner. **LiDAR (MID-360) + IMU now provide odometry, the 3D map, and the 2D map. RGB-D provides color, semantics, and POIs.**

Evidence from `bags/full_survey_take_01` (robot walked ~25 m in a loop, 153 s; see §6 and §8):

| RTAB-Map setup | Odometry | Map |
|---|---|---|
| RGB-D visual odometry | 71 % of frames lost, 317 resets | 113 fragments |
| `/dog_odom` + RGB-D | continuous, but ~2× distance underestimate | 1 map, 0 loop closures |
| **MID-360 ICP odometry (`g1_mapping`)** | **1528/1529 scans OK, 0 resets** | **1 map, 13 loop closures, z drift ±9 cm, 2D grid** |

Consequences:

- `g1_mapping` (`g1_ws/src/g1_mapping`) is the mapping package. Default odometry is `icp_odometry` + IMU; `/dog_odom` is a switchable fallback (§8).
- POI RGB keyframes are captured while the robot stands still.
- M2 needs tape-measured dimensions written down at recording time, because the LiDAR is no longer an independent check (§10.2).
- Extrinsics come from the G1 URDF via `/tf` + `joint_states`. Verification is an RViz sanity check only, with no calibration (§10.1). Until bags contain `/tf`, `g1_mapping` ships **estimated** static transforms (ground-plane fits on depth / LiDAR, cross-checked against the URDF) for replaying legacy bags; they are not a calibration.
- Exploration (M5) uses m-explore for ROS 2 (`explore_lite`) on top of Nav2 and the RTAB-Map `/map` (§15).
- 2D grid: `Grid/MaxGroundHeight` set and the IMU fed to the `rtabmap` node. On `full_survey_take_01`, table-height cells went from 69 occupied / 357 free to 245 / 10.
- The recording laptop's clock was ~72 s ahead of the robot's clock (`full_survey_take_01`). Replay is unaffected (header stamps), but live Nav2 / TF timeouts need aligned clocks: fix it on the laptop side only (§25.2 forbids robot network/OS changes) or run the stack on the Orin.

### Update — OAK-D chest camera, `/tf`, ownership (2026-09-25, night)

- **Semantics move to a chest-mounted OAK-D** (the head RealSense looks at the floor). Topic / frame names, model, and host are to be verified, then added to `survey.yaml` and `g1_mapping.yaml` (§7).
- **The robot publishes no `/tf`.** It comes from our `/lowstate` bridge + `robot_state_publisher` (G1 URDF) + static glue frames (§10.1). No bag has had `/tf` so far because none of these ran during capture.
- **Ownership contract** for TF and topics, including which parts of `rl_hnav`'s bridge to disable next to `g1_mapping`: §6.
- Current people: Vishal — locomotion (`rl_hnav`) + exploration (m-explore); Inko — OAK-D chest mount; stanislawix — `g1_mapping` (C). The `/tf` chain (A) is **unassigned**.

### Day-1 task assignment (updated critical path A→B→C→D; semantic work in parallel)
| Owner | Package(s) | Milestone | Offline-capable |
|-------|-----------|-----------|-----------------|
| A | `g1_sensors`: `/lowstate` → `/joint_states` bridge, `robot_state_publisher` (G1 URDF), static glue frames incl. the OAK-D mount | M0 | needs robot |
| B | `g1_recorder` + existing `keyframe_manager` | M0 | yes after canonical bag |
| C | `g1_mapping` (RTAB-Map LiDAR-inertial) bringup + tuning | M1→M3 | yes (from bag) |
| D | URDF TF chain + physical-measurement validation + `scene_server` canonical outputs | M2→M3 | yes (from bag/map DB) |
| E | `semantic_query` (Grounding DINO + SAM2 + OAK-D RGB-D backprojection) | M4 | yes |
Cross-cutting (assign to lead): freeze the **sensor/topic/frame contract** + **canonical scene/POI output contract** before coding; own the **canonical shared rosbag**; own the **safety checklist**. The keyframe struct remains frozen for semantic/offline work.

### Recording conventions (`g1_recorder`)
- **Mandatory:** `/utlidar/cloud_livox_mid360`, `/dog_imu_raw`, `/dog_odom`, `/lf/lowstate`, `/tf_static`,
  plus `/tf` whenever our TF nodes run during capture (the robot does not publish it). Semantic: OAK-D
  RGB + aligned depth + CameraInfo (**same resolution**; names to be verified). RealSense streams optional.
  Also: `/utlidar/imu_livox_mid360`, `/secondary_imu`, `/lf/bmsstate`.
  Profiles: `g1_recorder/config/survey.yaml` (default), `live_run.yaml`. Check message counts per bag.
- Existing bags `full_survey_take_01`, `rtab_take_*`, `rosbag2_2026_09_25-15_05_44` have **no `/tf`**; the
  `rtab_take_*` ones have no LiDAR/IMU either. Only `full_survey_take_01` works with `g1_mapping` (using its
  static fallback).
- Write 2–3 tape-measured dimensions next to every canonical bag (M2).
- **QoS gotchas:** `/tf_static` needs `durability: transient_local` + `history: keep_all` via
  `--qos-profile-overrides-path`, else RViz opened after playback starts gets no TF. Sensor topics
  are often `best_effort` → replay subscribers/RViz must match QoS.
- Produce **one canonical rosbag** as the shared fixture so B/C/D/E develop offline in parallel;
  the whole pipeline must re-run **robot-off** from it (demo insurance).

### Implementation status — B's packages (updated 2026-09-25)
Built and **offline-verified** in `g1_ws/src/` (dev distro; portable to Humble):
`g1_recorder`, `keyframe_manager`, `scene_server` (stub).
- **R0 build ✅ · R1 record→replay + tf_static QoS ✅ · R2 viz stub ✅ · R3 keyframe extraction ✅.**
  Remaining: **R4** verify real topic names/QoS on robot, **R5** canonical capture + publish shared bag.
- **Bags:** mcap, no compression. This rosbag2 build accepts topics as positional arguments
  (`--topics` is rejected). `/tf_static` transient_local override confirmed required and working.
- **`scene_server` is a placeholder owned by D** — adapt it to expose canonical RTAB-Map-backed metric scene/map outputs. The stub publishes `/scene_cloud` in `map`.

#### FROZEN keyframe struct — offline / semantic contract
`keyframe_manager` writes this stable offline fixture. Semantic-query code may read it; RTAB-Map primary mapping does not depend on it:
```
<output_dir>/
  manifest.json            # {count, keyframes:[{id, dir, blur_var, frame_id}, ...]}
  keyframe_NN/
    rgb.png                # color, bgr8
    depth.npy              # uint16 MILLIMETRES, aligned to rgb, shape (H, W)
    camera_info.yaml       # width, height, distortion_model, k[9], d[]
    meta.yaml              # id, stamp{sec,nanosec}, frame_id, blur_var,
                           # depth_units="mm", depth_encoding, odom_pose{frame,position}?
```
Depth is always stored uint16 mm (16UC1 passed through; 32FC1 m ×1000). Read `depth_units` from
`meta.yaml`. Selection = sharpness (var-of-Laplacian) ∧ temporal spacing ∧ translational baseline.

Code: **`g1_ws/`** (`g1_recorder`, `keyframe_manager`, `scene_server`). Build with `colcon build`.

### Testing from the computer connected to the robot (R4/R5)
RealSense (and the OAK-D, if plugged into the Orin) + LiDAR + odom run on the **Orin**. The dev laptop only sees them if it joins the Orin's
DDS graph over the **wired** link (Wi-Fi alone will not — verified: with only Wi-Fi up the laptop
sees zero robot topics). Procedure:
1. **Join the robot LAN:** plug Ethernet, `sudo ip addr add 192.168.123.222/24 dev enp3s0 && sudo ip link set enp3s0 up`, `ping <ORIN_IP>` (confirm subnet/IP with the robot owner; Unitree default `192.168.123.0/24`).
2. **DDS env:** `source ~/unitree_ros2/setup.sh` (sets `rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=enp3s0`), `export ROS_DOMAIN_ID=<Orin's>` (confirm; default 0).
3. **Discover (read-only):** `ros2 run g1_recorder discover_sensors.sh` → report of nodes/topics/QoS/rates/TF. Reconcile `topics.yaml` + `keyframe_params.yaml` with verified names; confirm **aligned depth** exists (`align_depth:=true`). Alternatively run the script **on the Orin** (zero network variables) and `scp` the report back.
4. **Record on the laptop** (keeps recording off the robot command path, per §25). Full commands in `g1_ws/README.md`.

Sensors driven by the Orin (RealSense, LiDAR, odom; the OAK-D only if plugged into the Orin) share its
clock → cross-sensor time sync is a non-issue for them; the recorder uses message header stamps. The
*recording laptop's* clock is separate (~72 s offset observed).

After R5 canonical capture, the next core test is:

```text
canonical bag -> g1_mapping (RTAB-Map, LiDAR + IMU) -> metric map/trajectory -> TF sanity check + tape measurements -> RViz
```

---

# 25. Organizer (x-kom) Rules for the G1 — Binding

Source: the hackathon's G1 usage regulations from x-kom (paraphrased from the Polish original). These override any conflicting statement above. If a task would violate them, stop and surface it.

## 25.1 Control tiers

| Tier | What | Conditions |
|---|---|---|
| Built-in | Remote, vendor app, built-in locomotion and motions | No restrictions on the ground floor |
| High-level SDK | Loco/Sport client, Arm SDK, hand control, sensors; own software onboard or offboard. Vendor controller keeps balance | Free-standing on the ground floor OK; demos of finished behaviors outside mats OK, with an operator next to the robot |
| Low-level (`lowcmd`) | Own policies on any joints, incl. legs / dynamic behaviors | Robot on harness or stand, **and** a working own safety supervisor (25.3) |
| Free-standing whole-body / dynamic | Own whole-body control, dynamic behaviors | Only if it already passed on the harness with no supervisor trips, supervisor is running, on mats, two people present. Damage from trials outside these conditions is the team's liability |

Implications for this project:
- Perception work (M0–M4) needs only sensors + built-in/high-level modes. No `lowcmd` is needed. Keep it that way.
- `rl_hnav` publishes `LowCmd` -> it is the **low-level tier**. See M5.
- Preferred `/cmd_vel` executor for a fast, low-risk path: high-level SDK Loco/Sport client `Move(vx, vy, wz)`; `rl_hnav` only if the low-level requirements are met.
- Default for any agent-written code: no actuation. Opt-in only (consistent with Section 19).

## 25.2 Prohibitions relevant to code and setup

- No stairs, ramps, platforms, balconies, or going outside. Flat indoor ground floor only.
- No driving the robot through crowds; on mats only operators in the zone, spectators behind a clear line.
- No enabling actuators with fewer than two team members present, or if nobody holds the remote with the e-stop.
- No free-standing custom leg control without a prior harness trial and a running supervisor.
- Nothing carried in hands during low-level trials. During built-in locomotion only closed, unbreakable objects within the hand payload limit. No open liquids, glass, hot or sharp items.
- No hard power cut. Shutdown = OS shutdown, then the switch (exception: danger to people).
- Do not walk the robot below **20 %** battery; do not run actuators below **10 %**. Original charger only, supervised. Code should read battery state and refuse/abort motion under those thresholds.
- Do **not** change firmware, OS, system account passwords, or network configuration (adding the event network is the only exception). Do not remove x-kom SSH keys. Our own SSH keys/accounts may be added onboard and must be listed at return.
  - Consequence for agents: do not `apt upgrade`, change kernel/OS settings, netplan/NetworkManager config, DDS/network interface config on the robot, etc. Keep installs (e.g. `realsense-ros`) scoped, minimal, documented, and reversible; prefer running heavy stacks on the dev machine / Docker. Existing `unitree_bridge` / DDS setup must be used as-is.
- No disassembly (except hands), drilling, gluing. Never cover cameras, LiDAR, vents or indicators. Do not attach devices to robot connectors that could damage it.
- Extra hardware (cameras, mics, 3D-printed mounts) <= **1 kg** total, on torso or head, hands via velcro/zip ties or manufacturer mounting holes, no traces on removal.
- Do not leave the robot standing unattended; in breaks: stand, harness, or rest pose.
- Hands: Revo 2 <-> dummy hands swap only with the robot fully off. Use dummy hands for collision/impact/fast-contact tests.

## 25.3 Required safety supervisor (for any low-level control)

Before the first run of own low-level control, run a separate process/thread that:

- logs full robot state at loop rate, **>= 50 Hz**: joint pos/vel, torque, temperature, per-actuator error flags, IMU, battery, and the commands sent to actuators. Keep logs until the end of the hackathon; hand to x-kom on request.
- automatically switches the robot to **damping/limp mode** when: no new state frame or loop stall > **100 ms**; an actuator reports an error; temperature or torque exceed team thresholds (with margin vs. manufacturer limits); commanded vs. measured position deviates above a per-behavior threshold; body tilt exceeds a per-behavior threshold.
- after a loop stall does **not** catch up on missed ticks; resumes from the current state with rate limiting.
- does no disk writes, network communication or heavy inference in the command-generating thread (log via a queue/other thread; keep GroundingDINO / SAM2 etc. off the control path).
- never disables, takes over or delays the remote's emergency stop.
- Thresholds are chosen per behavior and documented by the team (store next to the code, e.g. `docs/safety_thresholds.md`). Each new code version must pass **>= 3 minutes of dry-run** (no actuation) without a supervisor trip before it drives actuators.
- On an incident, thresholds doc, logs, and the code version at that moment go to x-kom. Tag/commit the exact version that runs on actuators.

## 25.4 Incident procedure

An incident = fall, collision, unnatural sound, overheating, error message in the vendor app, spontaneous motion, or a supervisor trip.

1. Emergency stop from the remote.
2. Leave the robot as found.
3. Photos from several sides + a note with the time.
4. Preserve logs and the code version at the time of the incident.
5. Phone x-kom immediately. Delay in reporting counts as concealment.

Agents must never "fix and retry" after an incident, and must not delete or rotate logs.

## 25.5 Effect on the plan

- M0–M4 (sensing, RTAB-Map metric mapping, validation, POI): unaffected; built-in mode + sensors + manual/remote survey. Read-only sensor discovery stays the default.
- M5 (autonomous survey): gated by 25.1 / 25.3. Decide early whether to use high-level Loco client (fewer requirements) or `rl_hnav` (harness trial + supervisor first). Budget time for the harness trial and supervisor dry-run.
- Mounting extra sensors: <= 1 kg, non-destructive, no occluding of existing sensors.
- Recording (M0): `rosbag2` and supervisor logs run on the dev machine or a non-critical thread; do not add load to the command path.
