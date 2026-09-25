# AGENTS.md — Alien Bazaar Hackathon: Unitree G1 Side

## Purpose

This file is the shared engineering context for all coding agents working on the **Unitree G1 side** of the Alien Bazaar hackathon project.

It captures the architecture, decisions, constraints, module boundaries, milestone order, and current implementation assumptions agreed so far.

**Scope of this version:** G1 sensing, metric RGB-D mapping, ROS integration, semantic POI extraction, autonomous survey integration, and the G1-to-Leo handoff boundary.

**Out of scope for this version:** detailed Leo Rover implementation. Leo-side details will be added later.

Agents should treat the decisions in this file as authoritative unless a human explicitly changes them.

**Current architecture update (2026-09-25): RTAB-Map RGB-D is the primary metric mapping backbone. VGGT is optional / stretch only. Where older historical notes mention VGGT as the primary map, this newer architecture wins.**

---

# 1. Project Goal

Use a **Unitree G1 EDU** as a mobile survey / scene-understanding embodiment.

The G1 should:

1. Observe an indoor scene using its RGB-D camera and LiDAR.
2. Build a **metric map directly from RGB-D** using **RTAB-Map** as the primary mapping backbone.
3. Use LiDAR as an independent geometric validation source and, if useful, an additional registration / ICP source.
4. Produce a robotics-ready scene representation: metric trajectory, colored 3D map, 2D occupancy map, and a defined ROS world frame.
5. Allow a natural-language object query such as `"red bottle"` or `"box on the table"`.
6. Convert the queried object into a **3D Point of Interest (POI)** in the shared map frame.
7. Eventually navigate autonomously between survey viewpoints using ROS 2 Nav2 plus a G1 locomotion executor.
8. Hand the metric scene + POI + supporting context to the Leo Rover side.

Primary concept:

```text
G1 surveys
    ↓
RealSense RGB-D + LiDAR
    ↓
RTAB-Map metric RGB-D mapping
    ↓
metric map + trajectory + occupancy grid
    ↓
natural-language query
    ↓
3D POI in map frame
    ↓
handoff to Leo Rover
```

**VGGT is now an optional parallel / research branch, not the primary mapper and not a dependency for the core demo.**

Optional branch:

```text
RGB keyframes
    ↓
VGGT
    ↓
learned dense reconstruction
    ↓
compare / align against RTAB-Map metric world
```

The G1 is the **survey + spatial understanding embodiment**.

---

# 2. Primary Demo Philosophy

Keep the project modular and sequential.

Do not make the entire demo depend on every component working simultaneously.

Preferred development order:

```text
sensor acquisition + rosbag
    ↓
RTAB-Map RGB-D metric mapping
    ↓
LiDAR / TF validation
    ↓
RViz + Nav2-compatible map outputs
    ↓
semantic POI
    ↓
autonomous G1 survey
    ↓
Leo handoff
```

For the early milestones, moving the G1 manually or via teleoperation is acceptable.

**Autonomous locomotion must not block mapping / POI development.**

**VGGT must not block the core path.** If pursued, it should run in parallel after the RTAB-Map baseline is working.

---

# 3. Hardware Assumptions

Current target hardware:

- Unitree G1 EDU
- Onboard / attached NVIDIA Jetson Orin-class compute
- Head RealSense RGB-D camera
- Existing Unitree 3D LiDAR stream
- G1 IMU / robot state
- Ethernet available for development
- Wi-Fi may be used later, but Ethernet is preferred during bring-up

Important hardware details that must be verified on the real robot:

- exact RealSense model
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
│ 2. RTAB-Map RGB-D Mapping  │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 3. LiDAR / Frame Validation│
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

Two cross-cutting / later modules sit beside this core path:

```text
A. Nav2 + G1 locomotion executor
   → autonomous survey viewpoints

B. Optional VGGT branch
   → learned RGB-only reconstruction for comparison / enrichment
```

## Primary mapping decision

**RTAB-Map is the primary metric mapping backbone.**

Why:

- RealSense already provides metric aligned depth.
- RTAB-Map consumes RGB-D directly.
- metric scale is available from the start.
- it produces camera/robot poses, a metric 3D map, a 2D occupancy map, pose-graph state, and ROS TF needed by robotics.
- it integrates naturally with ROS 2 and Nav2.
- it removes VGGT scale recovery from the critical path.

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

---

# 7. Module 1 — Sensor Acquisition

## Goal

Provide synchronized, calibrated scene observations from the G1 for RTAB-Map, semantic perception, offline replay, and optional VGGT experiments.

## Sensors

Primary:

- RGB
- aligned metric depth
- camera intrinsics
- timestamps
- LiDAR PointCloud2

Useful:

- IMU
- robot state
- odometry
- TF

## RealSense

Run `realsense-ros` on the G1 onboard Orin if possible; running it on the Humble laptop is also acceptable if the camera is physically connected there.

Desired outputs are standard ROS 2 topics equivalent to:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

Actual topic names must be discovered on the hardware rather than assumed.

**Aligned metric depth is now a primary mapping input, not merely a scale anchor for VGGT.**

RTAB-Map should consume RGB + aligned depth + camera calibration.

## LiDAR

The existing G1 / Unitree LiDAR stream is expected to already be available as a PointCloud2 topic.

`rl_hnav` currently assumes a real-robot stream such as:

```text
/utlidar/cloud_livox_mid360
```

Do not create a new LiDAR driver if the robot already publishes the required cloud.

For the core mapping path, LiDAR is initially used for:

- geometric validation
- obstacle perception / `/scan`
- optional ICP / registration refinement if time permits

## Recording

Record raw sensor streams to `rosbag2`.

The entire mapping pipeline must support replay with the G1 powered off.

Minimum useful recording:

```text
RGB
aligned depth
camera_info
LiDAR cloud
IMU
TF
TF static
odometry if available
```

The canonical rosbag is the shared development fixture for mapping, semantic perception, and optional VGGT work.

---

# 8. Module 2 — RTAB-Map RGB-D Mapping

## Goal

Build the primary **metric robotics map directly from RealSense RGB-D**.

RTAB-Map is the core mapper for V1.

Conceptually:

```text
RGB + aligned metric depth + CameraInfo
                │
                ▼
           RGB-D odometry
                │
                ▼
            RTAB-Map
                │
      ┌─────────┼──────────┐
      ▼         ▼          ▼
 metric poses  3D map    2D map
      │                    │
      └──────── TF ────────┘
```

Expected useful outputs include:

- metric camera / robot trajectory
- metric colored 3D point-cloud map
- 2D occupancy grid for navigation
- pose graph / loop-closure constraints
- map database
- `map -> odom` transform while mapping/localizing
- ROS map state usable by Nav2

Exact ROS topic names should be taken from the installed `rtabmap_ros` configuration rather than hardcoded here.

## Input expectations

Primary:

```text
RGB image
aligned metric depth
CameraInfo
```

Optional / supporting:

```text
/odom
IMU
LiDAR-derived scan or cloud
TF
```

Start with the simplest reliable RGB-D configuration.

Do not add every sensor into RTAB-Map on the first run.

Recommended progression:

```text
RGB-D only
    ↓
verify metric map + trajectory
    ↓
add odometry / IMU if useful
    ↓
add LiDAR / ICP only if it materially improves robustness
```

## Keyframe handling

RTAB-Map manages its own mapping nodes / keyframes internally.

The existing `keyframe_manager` package is still useful for:

- offline inspection
- Grounding DINO + SAM2 semantic querying
- optional VGGT reconstruction
- creating reproducible image/depth fixtures

It is **no longer a dependency for primary mapping**.

---

# 9. Optional Module — VGGT Learned Reconstruction

VGGT is no longer on the critical path.

Use it only if the RTAB-Map metric baseline is already working or if a parallel team member can pursue it independently.

## Purpose

VGGT can provide:

- learned RGB-only camera poses
- dense predicted geometry / point maps
- confidence
- a visually rich feed-forward reconstruction

This can be useful for:

- comparing learned reconstruction vs classical RGB-D SLAM
- producing an additional visual scene representation
- research/demo value
- testing whether learned geometry adds useful structure

Conceptually:

```text
selected RGB keyframes
        ↓
       VGGT
        ↓
camera poses + dense geometry + confidence
        ↓
optional alignment against RTAB-Map world
```

We do **not currently have access to VGGT-Ω**. If VGGT is used, use the public VGGT release.

The backend must remain swappable between Orin, laptop GPU, and cloud GPU.

VGGT output must not replace the metric RTAB-Map world unless it has been explicitly registered and validated.

---

# 10. Module 3 — LiDAR / Frame Validation and Optional Fusion

With RTAB-Map as the primary RGB-D mapper, **global metric-scale recovery is no longer a core problem**.

The RealSense depth already provides metric geometry.

The remaining geometric problems are:

1. correct sensor extrinsics
2. consistent ROS frame ownership
3. map / odom / base / camera / LiDAR TF correctness
4. validating RTAB-Map geometry against LiDAR
5. optional LiDAR-assisted registration / ICP if useful

## 10.1 Frame chain

Maintain an explicit TF chain such as:

```text
map
  ↓
odom
  ↓
robot_center / base
  ├── camera frame
  └── lidar frame
```

Never pass XYZ coordinates without a `frame_id`.

Verify `T_base_camera`, `T_base_lidar`, `map -> odom`, and `odom -> base` before using the map for G1-to-Leo handoff.

## 10.2 LiDAR cross-check

Use LiDAR as an independent geometric validation source.

Check walls, table surfaces, room dimensions, floor orientation, large object geometry, and consistency between LiDAR and RGB-D map.

The milestone should include numerical checks, not only visual overlap.

## 10.3 Optional fusion / ICP

If RGB-D odometry or map alignment benefits from LiDAR, optional later work may use:

```text
LiDAR cloud + RTAB-Map RGB-D geometry
                ↓
        ICP / scan registration
                ↓
      refined constraint / validation
```

Do not add ICP merely because it is available; add it only if the baseline needs it.

## 10.4 VGGT-only note

If the optional VGGT branch is used, VGGT still needs explicit registration:

```text
p_map = s R p_vggt + t
```

For VGGT only:

- RealSense depth may estimate `s`
- RTAB-Map / TF provides the shared world reference
- LiDAR can cross-check the result

This is not part of the core RTAB-Map path.

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

Optional VGGT outputs should use a clearly separate namespace such as `/vggt/scene_cloud` and `/vggt/camera_path`.

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
aligned RealSense metric depth
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

For V1, use the RealSense depth as the geometric source for the object whenever possible.

Do not make semantic localization depend on VGGT.

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

---

# 15. Autonomous Survey Behavior

Autonomous G1 survey is a later milestone.

Desired behavior:

```text
survey viewpoint
    ↓
Nav2 NavigateToPose
    ↓
/cmd_vel
    ↓
rl_hnav
    ↓
G1 walks
    ↓
goal reached
    ↓
zero velocity
    ↓
settle
    ↓
capture RGB-D keyframe
    ↓
next viewpoint
```

Do not run VGGT continuously while the robot walks in V1.

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

## M1 — RTAB-Map Metric RGB-D Map

Goal: recorded or live RealSense RGB-D produces a coherent metric map and trajectory.

Acceptance criteria:

- RTAB-Map runs on the RGB-D stream
- metric trajectory is produced
- colored 3D geometry resembles the real scene
- scale agrees with simple physical measurements
- map database can be saved / reopened

Deliverables:

```text
RTAB-Map database
metric trajectory
metric colored scene
```

No VGGT dependency.

## M2 — LiDAR / TF Validation

Goal: confirm the RGB-D map is correctly framed and metrically consistent with the G1 LiDAR.

Acceptance criteria:

- camera and LiDAR extrinsics are known / validated
- RTAB-Map scene broadly overlays LiDAR geometry
- known wall / table / room measurements agree within an agreed tolerance
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
+ aligned RealSense depth
+ RTAB-Map camera pose
```

Expected output:

```text
label
xyz
frame_id = map
confidence
```

## M5 — Autonomous G1 Survey

Goal: G1 autonomously reaches predefined survey viewpoints and continues mapping / capturing observations.

Primary navigation stack:

```text
RTAB-Map map/localization
    ↓
Nav2
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
- multiple viewpoints can execute sequentially

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

## Stretch M7 — VGGT Comparison Branch

Goal: produce an optional learned RGB-only reconstruction and compare it against the RTAB-Map metric world.

This milestone must never block M0–M6.

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
vggt_reconstruction        # optional branch
metric_registration_vggt   # optional branch only
```

External packages to reuse rather than reimplement:

```text
realsense-ros
rtabmap_ros
Nav2
rl_hnav / rl_sar where permitted
pointcloud_to_laserscan where needed
```

Recommended distinction:

```text
real-time acquisition / mapping / robot control
    -> ROS 2

offline analysis / optional VGGT experiments
    -> Python first

stable scene / POI interfaces
    -> ROS 2 project packages
```

---

# 18. First Development Workflow

Recommended execution sequence:

```text
1. Get ROS 2 Humble working on development machine
2. Connect to G1 over Ethernet
3. Discover existing G1 topics
4. Verify RGB / aligned depth / CameraInfo / LiDAR / IMU / odom / TF
5. Record canonical rosbag
6. Replay rosbag with robot off
7. Install / configure RTAB-Map ROS 2
8. Run RTAB-Map on replayed RGB-D
9. Verify metric trajectory + colored 3D map
10. Validate TF / dimensions against LiDAR
11. Expose /map + map->odom + scene outputs in RViz
12. Add Grounding DINO + SAM2 -> metric POI
13. Integrate Nav2
14. Integrate safe G1 locomotion executor
15. Only then pursue optional VGGT comparison if useful
```

Do not start with autonomous locomotion.

Do not start by solving VGGT scale.

The first mapping milestone is now **RGB-D -> RTAB-Map -> metric map**.

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

## Risk 1 — RGB-D odometry / RTAB-Map tracking quality

Mitigations:

- aligned RealSense depth
- adequate lighting / visual texture
- slow / stop survey motion if needed
- preserve camera calibration
- use odometry / IMU support if helpful
- use LiDAR / ICP only if the RGB-D baseline needs it
- rosbag every run for repeatable tuning

## Risk 2 — Coordinate-frame mismatch

Mitigations:

- explicit `map`, `odom`, base, camera, and LiDAR frames
- validate static extrinsics
- visualize TF in RViz
- ensure only one mapping/localization stack owns `map -> odom`
- never pass unlabelled XYZ coordinates between robots

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

## Risk 5 — Optional VGGT compute

VGGT remains GPU-heavy.

Mitigation:

- keep the backend swappable
- do not block the core demo on learned reconstruction
- use offboard/cloud GPU if needed

## Risk 6 — Autonomous G1 navigation

Mitigation:

- manual survey is valid for M0–M4
- start on flat indoor terrain
- use predefined viewpoints before exploration planning
- prefer high-level Unitree locomotion under event rules
- use `rl_hnav` only after the required harness/supervisor validation

---

# 21. Explicit Non-Goals for V1

Do not spend hackathon time on these unless all core milestones are already stable:

- continuous VGGT reconstruction while walking
- making VGGT the primary mapping system
- full multi-robot SLAM
- arbitrary terrain / stairs
- training a new G1 locomotion policy
- end-to-end VLA locomotion
- autonomous next-best-view exploration
- direct natural-language querying of raw 3D embeddings
- sophisticated dynamic-object tracking
- general-purpose manipulation on G1
- replacing RealSense metric depth with learned depth
- perfect dense LiDAR/RGB fusion
- running RTAB-Map and SLAM Toolbox simultaneously as competing map owners

---

# 22. Core Interfaces

## Sensor side

```text
RGB image
aligned metric depth
CameraInfo
LiDAR PointCloud2
IMU
TF
odom
timestamps
```

## Primary mapping side

```text
RTAB-Map metric trajectory
metric colored 3D map
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

## Optional VGGT side

```text
VGGT camera poses
VGGT dense geometry
VGGT confidence
T_map_vggt   # only if aligned
```

---

# 23. Definition of Success for the G1 Side

Minimum successful G1-side demo:

1. G1 provides synchronized RGB-D observations and LiDAR.
2. RTAB-Map builds a coherent **metric** map and trajectory.
3. LiDAR / physical measurements validate the map geometry and frame setup.
4. The metric colored scene and 2D map are visible in RViz.
5. A natural-language object query returns a plausible metric 3D POI in `map`.
6. The G1-side system outputs a clear map + POI handoff package for Leo.

Stretch:

7. G1 autonomously navigates between survey viewpoints using Nav2 plus an event-compliant locomotion executor.
8. VGGT produces a parallel learned reconstruction aligned / compared against the RTAB-Map world.

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
- keep RTAB-Map as the primary mapping backend; keep the optional VGGT backend swappable
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
  orchestration / RViz / native-camera only; **never runs VGGT**.
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
- **VGGT compute is swappable and NOT pinned:** candidates = G1 built-in **Jetson Orin**, a
  separate **8 GB Jetson**, or **cloud GPU**. BF16 where supported. Get it running reliably first,
  optimize placement later.
- **VGGT submodules:** model code in `third_party/vggt` (facebookresearch/vggt), weights in
  `VGGT-1B` (facebook/VGGT-1B, Git LFS, ~10 GB, CC-BY-NC-4.0). On machines that do not run VGGT
  (incl. the Iris Xe dev laptop) **never fetch the LFS weights**: init submodules with
  `GIT_LFS_SKIP_SMUDGE=1` and set `lfs.fetchexclude '*'` in `VGGT-1B` (see README). On a VGGT
  machine fetch only `model.safetensors`.

### Mapping architecture update (2026-09-25)

This update supersedes the earlier assumption that VGGT is the primary reconstruction path.

**Primary V1 map: RTAB-Map RGB-D.**

```text
RealSense RGB + aligned metric depth
            ↓
         RTAB-Map
            ↓
metric trajectory + metric 3D scene + /map + map->odom
            ↓
Grounding DINO + SAM2 + depth + camera pose
            ↓
3D POI in map frame
```

LiDAR remains an independent metric / geometry check and a navigation obstacle source. It may later contribute ICP constraints if needed.

VGGT becomes a **parallel stretch branch**:

```text
keyframe_manager -> VGGT -> optional alignment / comparison to RTAB-Map
```

The previous `metric_registration` package is no longer required for the primary map. If retained, scope it to **VGGT-only registration** or general map-validation utilities.

The existing `keyframe_manager` remains useful for semantic queries, offline fixtures, and VGGT experiments, but is not required by RTAB-Map.

**Map ownership rule:** RTAB-Map should be the only `map -> odom` owner when it is used for navigation. Do not launch SLAM Toolbox concurrently as another map owner. SLAM Toolbox remains a deliberate fallback only.

### Day-1 task assignment (updated critical path A→B→C→D; semantic work in parallel)
| Owner | Package(s) | Milestone | Offline-capable |
|-------|-----------|-----------|-----------------|
| A | `g1_sensors` + relay/`odom_tf_bridge`/static TF | M0 | needs robot |
| B | `g1_recorder` + existing `keyframe_manager` | M0 | yes after canonical bag |
| C | `g1_mapping` / `rtabmap_ros` bringup + tuning | M1→M3 | yes (from bag) |
| D | TF/LiDAR validation + `scene_server` canonical outputs | M2→M3 | yes (from bag/map DB) |
| E | `semantic_query` (Grounding DINO + SAM2 + RGB-D backprojection) | M4 | yes |
| Stretch | `vggt_reconstruction` + VGGT-only registration/comparison | M7 | yes |
Cross-cutting (assign to lead): freeze the **sensor/topic/frame contract** + **canonical scene/POI output contract** before coding; own the **canonical shared rosbag**; own the **safety checklist**. The keyframe struct remains frozen for semantic/VGGT/offline work.

### Recording conventions (`g1_recorder`)
- Record set: `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`,
  `/camera/color/camera_info`, `/utlidar/cloud_livox_mid360`, IMU, `/odom`, `/tf`, `/tf_static`.
- **QoS gotchas:** `/tf_static` needs `durability: transient_local` + `history: keep_all` via
  `--qos-profile-overrides-path`, else RViz opened after playback starts gets no TF. Sensor topics
  are often `best_effort` → replay subscribers/RViz must match QoS.
- Produce **one canonical rosbag** as the shared fixture so B/C/D/E develop offline in parallel;
  the whole pipeline must re-run **robot-off** from it (demo insurance).

### Implementation status — B's packages (updated 2026-09-25)
Built and **offline-verified** in `ros2_ws/src/` (dev distro; portable to Humble):
`g1_recorder`, `keyframe_manager`, `scene_server` (stub).
- **R0 build ✅ · R1 record→replay + tf_static QoS ✅ · R2 viz stub ✅ · R3 keyframe extraction ✅.**
  Remaining: **R4** verify real topic names/QoS on robot, **R5** canonical capture + publish shared bag.
- **Bags:** mcap, no compression. This rosbag2 build needs the **`--topics`** flag (positional
  topics are rejected). `/tf_static` transient_local override confirmed required and working.
- **`scene_server` is a placeholder owned by D** — adapt it to expose canonical RTAB-Map-backed metric scene/map outputs. Keep `/vggt/scene_cloud` + `vggt_world` only for the optional VGGT branch.

#### FROZEN keyframe struct — offline / semantic / optional-VGGT contract
`keyframe_manager` writes this stable offline fixture. Semantic-query and optional VGGT code may read it; RTAB-Map primary mapping does not depend on it:
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
RealSense + LiDAR + odom run on the **Orin**. The dev laptop only sees them if it joins the Orin's
DDS graph over the **wired** link (Wi-Fi alone will not — verified: with only Wi-Fi up the laptop
sees zero robot topics). Procedure:
1. **Join the robot LAN:** plug Ethernet, `sudo ip addr add 192.168.123.222/24 dev enp3s0 && sudo ip link set enp3s0 up`, `ping <ORIN_IP>` (confirm subnet/IP with the robot owner; Unitree default `192.168.123.0/24`).
2. **DDS env:** `source ~/unitree_ros2/setup.sh` (sets `rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=enp3s0`), `export ROS_DOMAIN_ID=<Orin's>` (confirm; default 0).
3. **Discover (read-only):** `ros2 run g1_recorder discover_sensors.sh` → report of nodes/topics/QoS/rates/TF. Reconcile `topics.yaml` + `keyframe_params.yaml` with verified names; confirm **aligned depth** exists (`align_depth:=true`). Alternatively run the script **on the Orin** (zero network variables) and `scp` the report back.
4. **Record on the laptop** (keeps recording off the robot command path, per §25). Full commands in `g1_ws/README.md`.

Since RealSense runs on the Orin, all sensors share the Orin clock → cross-sensor time sync is a
non-issue for the recorded robot-side streams; the recorder uses message header stamps.

After R5 canonical capture, the next core test is:

```text
canonical bag -> RTAB-Map RGB-D -> metric map/trajectory -> LiDAR/TF validation -> RViz
```

Do not spend critical-path time on VGGT scale recovery unless the RTAB-Map path is already working.

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
- does no disk writes, network communication or heavy inference in the command-generating thread (log via a queue/other thread; keep VGGT/GroundingDINO etc. off the control path).
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