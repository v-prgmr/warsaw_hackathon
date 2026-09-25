# AGENTS.md — Alien Bazaar Hackathon: Unitree G1 Side

## Purpose

This file is the shared engineering context for all coding agents working on the **Unitree G1 side** of the Alien Bazaar hackathon project.

It captures the architecture, decisions, constraints, module boundaries, milestone order, and current implementation assumptions agreed so far.

**Scope of this version:** G1 sensing, reconstruction, metric registration, ROS integration, semantic POI extraction, and later autonomous survey locomotion.

**Out of scope for this version:** detailed Leo Rover implementation. The G1-to-Leo handoff contract is included only at the boundary. Leo-side details will be added later.

Agents should treat the decisions in this file as authoritative unless a human explicitly changes them.

---

# 1. Project Goal

Use a **Unitree G1 EDU** as a mobile survey / scene-understanding embodiment.

The G1 should:

1. Observe an indoor scene using its RGB-D camera and LiDAR.
2. Reconstruct the scene using VGGT.
3. Convert the reconstruction into a **metric 3D scene**.
4. Allow a natural-language object query such as `"red bottle"` or `"box on the table"`.
5. Convert the queried object into a **3D Point of Interest (POI)** with a known coordinate frame.
6. Eventually navigate autonomously between survey viewpoints using ROS 2 Nav2 + `rl_hnav`.
7. Hand the metric scene + POI + supporting context to the Leo Rover side.

Core concept:

```text
G1 surveys
    ↓
RGB-D + LiDAR
    ↓
VGGT reconstruction
    ↓
metric registration
    ↓
metric scene
    ↓
natural-language query
    ↓
3D POI
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
sensor acquisition
    ↓
offline reconstruction
    ↓
metric registration
    ↓
RViz visualization
    ↓
semantic POI
    ↓
autonomous G1 survey
    ↓
Leo handoff
```

For the early milestones, moving the G1 manually or via teleoperation is acceptable.

**Autonomous locomotion must not block reconstruction development.**

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

# 6. G1 Software Architecture

The G1 side is divided into six gross modules.

```text
┌────────────────────────────┐
│ 1. Sensor Acquisition      │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 2. Keyframe Manager        │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 3. VGGT Reconstruction     │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 4. Metric Registration     │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 5. Scene Server / Output   │
└─────────────┬──────────────┘
              ↓
┌────────────────────────────┐
│ 6. Semantic Query / POI    │
└────────────────────────────┘
```

Autonomous locomotion using `rl_hnav` is integrated after the reconstruction path works.

---

# 7. Module 1 — Sensor Acquisition

## Goal

Provide synchronized, calibrated scene observations from the G1.

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

Run `realsense-ros` on the G1 onboard Orin if possible.

Desired outputs are standard ROS 2 topics equivalent to:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

Actual topic names must be discovered on the hardware rather than assumed.

Depth used for VGGT metric anchoring must be **aligned to the RGB/color frame**.

## LiDAR

The existing G1 / Unitree LiDAR stream is expected to already be available as a PointCloud2 topic.

`rl_hnav` currently assumes a real-robot stream such as:

```text
/utlidar/cloud_livox_mid360
```

Do not create a new LiDAR driver if the robot already publishes the required cloud.

## Recording

Record the raw sensor streams to `rosbag2`.

The system must support replaying reconstruction with the G1 powered off.

This is both a development workflow and demo insurance.

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

---

# 8. Module 2 — Keyframe Manager

VGGT should **not** consume the live RGB stream frame-by-frame at camera rate.

VGGT is treated as a batch / semi-batch reconstruction system.

Initial target:

```text
~8–20 keyframes
```

Later this can be increased if compute allows.

Keyframe selection should prefer:

- sharp frames
- spatially diverse viewpoints
- sufficient translational baseline
- useful overlap between adjacent views
- views covering the POI workspace
- views compatible with available GPU memory

Reject:

- motion-blurred frames
- near-duplicate frames
- frames during strong body motion where possible

Each keyframe record should retain:

```text
RGB image
aligned metric depth
timestamp
camera intrinsics
optional camera/base pose metadata
```

---

# 9. Module 3 — VGGT Reconstruction

## Model decision

We do **not currently have access to VGGT-Ω**.

Use the public / older VGGT release first.

Primary goal is not model novelty. It is to obtain:

- relative camera poses
- dense predicted depth / point maps
- confidence
- visual 3D reconstruction

The VGGT module is feed-forward.

Do not add SLAM or loop-closure logic inside this module.

Conceptually:

```text
RGB keyframes
    ↓
VGGT
    ↓
camera poses
dense geometry
confidence
```

## Compute

Preferred:

- run on the G1 Orin if memory/runtime is acceptable

Allowed:

- laptop GPU
- local workstation
- cloud/offboard GPU

The architecture must not depend on VGGT running specifically onboard.

Use one interface so the backend can move between:

```text
Orin
laptop
cloud
```

without changing the rest of the pipeline.

Initial precision target:

```text
BF16 where supported
```

---

# 10. Module 4 — Metric Registration

This is currently the most important reconstruction risk.

VGGT geometry must not be assumed to be metrically correct.

There are two distinct problems:

1. **metric scale**
2. **coordinate-frame registration**

Do not confuse them.

## 10.1 Metric Scale

Use the RealSense aligned metric depth as the primary scale anchor.

For each selected frame, compare VGGT depth and RealSense depth at corresponding RGB pixels.

Only use valid pixels.

Recommended mask:

```text
RealSense depth valid
VGGT confidence above threshold
not near invalid-depth boundaries
not obviously dynamic
within useful RealSense depth range
```

Per-frame scale:

```text
s_i = median(
    D_realsense(u,v) /
    D_vggt(u,v)
)
```

Then robustly combine the frame-level estimates:

```text
s = median(s_1, s_2, ... s_N)
```

Track the per-frame values.

Example diagnostic:

```text
frame 01  1.73
frame 02  1.71
frame 03  1.72
frame 04  2.34  <- investigate
```

Do not silently hide large disagreement.

## 10.2 Coordinate Frame Registration

Multiplying the VGGT scene by `s` makes distances metric.

It does **not** automatically place the reconstruction into the ROS `map` frame.

Initially it is acceptable to define:

```text
vggt_world
```

as the reconstruction frame.

Later establish:

```text
T_map_vggt
```

to connect VGGT space to the shared robot world frame.

General transform:

```text
p_map = s R p_vggt + t
```

where:

- `s` = scale
- `R` = orientation
- `t` = translation

For the first reconstruction milestone, publishing a correct metric cloud in `vggt_world` is sufficient.

For cross-robot handoff, `T_map_vggt` becomes mandatory.

## 10.3 LiDAR Cross-Check

LiDAR is the **independent geometric validation source**.

Do not use LiDAR as the primary scale source unless RealSense depth is unavailable or unreliable.

Use LiDAR to verify:

- wall distances
- table surfaces
- room dimensions
- floor orientation
- major object geometry

Compare the scaled VGGT scene against LiDAR.

The metric milestone should include numerical checks, not only visual overlap.

---

# 11. Module 5 — Scene Server / ROS Output

After metric registration, publish the scene back into ROS 2.

Primary outputs:

```text
/vggt/scene_cloud
/vggt/camera_path
```

Optional:

```text
/vggt/status
/vggt/keyframes
/vggt/confidence
```

Publish the necessary TF frames.

Initial scene frame:

```text
vggt_world
```

Later:

```text
map
  ↓
vggt_world
```

Use RViz for visualization.

Also save offline artifacts such as:

```text
scene.ply
camera_poses_metric.json
registration_metrics.json
```

Primary G1 reconstruction deliverable:

> a metrically correct colored point cloud with a defined coordinate frame.

---

# 12. Module 6 — Natural-Language Query / POI

Do not query the 3D point cloud directly in V1.

Use the RGB keyframes for semantic detection and then lift the result into 3D.

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
aligned depth / VGGT geometry
    ↓
back-project mask into 3D
    ↓
multi-view fusion
    ↓
3D POI
```

Example query:

```text
"red bottle"
```

Output contract:

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

Optional later:

```text
3D bounding box
object mask
object snapshot
```

---

# 13. `rl_hnav` Role

`rl_hnav` is the G1 locomotion / navigation execution layer.

Do not treat it as the reconstruction system.

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

# 16. Milestones

## M0 — ROS / Sensor Connectivity

Goal:

> Reliable access to all required G1 sensor streams and rosbag recording.

Acceptance criteria:

- RGB visible
- aligned metric depth visible
- camera intrinsics available
- LiDAR PointCloud2 visible
- timestamps sensible
- TF / static transforms understood
- rosbag records and replays successfully

Deliverable:

```text
one reproducible rosbag
```

Do not proceed by assuming topic names. Inspect the physical robot.

## M1 — Visual Reconstruction

Goal:

> Recorded RGB keyframes produce a coherent VGGT reconstruction.

Acceptance criteria:

- ~8–20 selected frames
- VGGT inference succeeds
- camera trajectory qualitatively matches survey motion
- reconstructed geometry visually resembles the scene

Deliverables:

```text
raw VGGT point cloud
camera poses
confidence outputs
```

No metric accuracy requirement yet.

## M2 — Metric Reconstruction

Goal:

> VGGT reconstruction becomes metrically correct.

Acceptance criteria:

- robust RealSense-to-VGGT scale estimate
- stable per-frame scale estimates
- known dimensions reconstructed within an agreed tolerance
- LiDAR overlay broadly agrees with reconstructed surfaces

Deliverables:

```text
metric_scene.ply
scale estimate
per-frame scale diagnostics
LiDAR validation result
```

This is currently the highest-risk reconstruction milestone.

## M3 — ROS Visualization

Goal:

> The metric VGGT scene is available as ROS data.

Acceptance criteria:

- `/vggt/scene_cloud` publishes
- camera path publishes
- correct `frame_id`
- scene visible in RViz
- saved `.ply` matches published cloud

Deliverables:

```text
/vggt/scene_cloud
/vggt/camera_path
scene.ply
```

## M4 — Semantic POI

Goal:

> Natural-language query produces a metric 3D object location.

Acceptance criteria:

Input:

```text
"red bottle"
```

Output:

```text
label
xyz
frame_id
confidence
```

The POI must visually / physically correspond to the intended object.

Preferred implementation:

```text
Grounding DINO + SAM2 + RGB-D backprojection
```

## M5 — Autonomous G1 Survey

Goal:

> G1 autonomously reaches predefined survey viewpoints.

Stack:

```text
SLAM / localization
    ↓
Nav2
    ↓
/cmd_vel
    ↓
rl_hnav
    ↓
G1
```

Acceptance criteria:

- NavigateToPose goal accepted
- G1 reaches a simple indoor goal
- G1 stops safely
- keyframe capture triggered after settling
- multiple viewpoints can be executed sequentially

Do not start this milestone before M0–M3 are independently working.

**Event-rule constraint (see Section 25):** `rl_hnav` drives the legs via `LowCmd` = custom low-level control. It must first run on the harness/stand with a working safety supervisor, and only then free-standing, on mats, with two people present. If that path is not cleared, fall back to the built-in locomotion via high-level SDK (Loco/Sport client) as the `/cmd_vel` executor.

## M6 — Shared Frame + Leo Handoff

Goal:

> Produce the G1-side handoff package.

Required outputs:

```text
metric scene
POI
supporting keyframes
T_map_vggt
confidence
```

Expected handoff contract:

```text
SceneHandoff {
    scene
    scene_frame
    T_map_scene
    POI
    supporting_keyframes
    confidence
}
```

Leo-side behavior is intentionally not specified in this version.

---

# 17. Software Packages / Nodes

Preferred initial package split:

```text
g1_sensors
g1_recorder
keyframe_manager
vggt_reconstruction
metric_registration
scene_server
semantic_query
```

Do not over-fragment the system.

Some modules may initially be plain Python scripts instead of long-running ROS nodes.

Recommended distinction:

```text
real-time acquisition / robot control
    -> ROS 2

batch reconstruction / analysis
    -> Python first

publish results / integration
    -> ROS 2
```

The VGGT reconstruction and metric registration may initially run offline from rosbag-derived data.

Once stable, they can be wrapped into ROS nodes.

---

# 18. First Development Workflow

Recommended first-day sequence:

```text
1. Get ROS 2 Humble working on development machine
2. Connect to G1 over Ethernet
3. Discover existing G1 topics
4. Verify RGB / depth / LiDAR / IMU / odom
5. Record rosbag
6. Replay rosbag with robot off
7. Extract ~8–20 RGB-D keyframes
8. Run VGGT offline
9. Export raw reconstruction
10. Solve RealSense-based metric scale
11. Validate against LiDAR
12. Publish final cloud into RViz
```

Do not start with autonomous locomotion.

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

# 20. Engineering Risks

## Risk 1 — VGGT metric scale

Highest current reconstruction risk.

Mitigations:

- RealSense aligned metric depth
- confidence masking
- per-frame robust median ratio
- reject outlier frames
- LiDAR cross-check
- retain RealSense metric point cloud as fallback

## Risk 2 — Coordinate-frame mismatch

A metric cloud is not automatically in the ROS map frame.

Mitigations:

- explicit `vggt_world`
- preserve all frame IDs
- later solve / publish `T_map_vggt`
- visualize TF in RViz
- never pass unlabelled XYZ coordinates between robots

## Risk 3 — Sensor timing / DDS issues

Mitigations:

- rosbag everything
- preserve timestamps
- use existing isolated DDS / bridge pattern
- avoid unnecessary RMW changes
- verify topics before adding custom bridges

## Risk 4 — Onboard compute

VGGT-1B may not fit / run fast enough on the available Jetson.

Mitigation:

- keep backend swappable
- use fewer keyframes
- use BF16 if supported
- offload to laptop/cloud if necessary

Do not block the project on onboard VGGT inference.

## Risk 5 — Autonomous G1 navigation

Mitigation:

- manual survey is valid for M0–M4
- use `rl_hnav` instead of training a new locomotion controller
- start on flat indoor terrain
- use predefined viewpoints before exploration planning

---

# 21. Explicit Non-Goals for V1

Do not spend hackathon time on these unless all core milestones are already stable:

- continuous real-time VGGT reconstruction while walking
- full multi-robot SLAM
- arbitrary terrain / stairs
- training a new G1 locomotion policy
- end-to-end VLA locomotion
- autonomous next-best-view exploration
- direct natural-language querying of raw 3D embeddings
- sophisticated dynamic-object tracking
- general-purpose manipulation on G1
- replacing RealSense depth with learned metric depth
- perfect dense LiDAR/RGB fusion

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

## Reconstruction side

```text
VGGT camera poses
VGGT point map / depth
VGGT confidence
metric scale
T_map_vggt
```

## Scene side

```text
/vggt/scene_cloud
/vggt/camera_path
scene.ply
camera_poses_metric.json
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

## Navigation side

```text
NavigateToPose
/map
/odom
/scan
/cmd_vel
```

## G1 locomotion side

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

1. G1 collects a small set of RGB-D observations.
2. VGGT reconstructs the scene.
3. RealSense depth makes the reconstruction metric.
4. LiDAR verifies the metric result.
5. The metric colored scene is visible in RViz.
6. A natural-language object query returns a plausible metric 3D POI.
7. The G1-side system outputs a clear handoff package for Leo.

Stretch:

8. G1 autonomously navigates between survey viewpoints using Nav2 + `rl_hnav`.

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
- keep the VGGT backend swappable
- make outputs inspectable in RViz / saved files
- report assumptions explicitly
- distinguish observed hardware facts from guesses

If an implementation choice conflicts with this file, stop and surface the conflict rather than silently changing the architecture.

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

### Day-1 task assignment (critical path A→B→C→D; M4/M5 run in parallel)
| Owner | Package(s) | Milestone | Offline-capable |
|-------|-----------|-----------|-----------------|
| A | `g1_sensors` + relay/`odom_tf_bridge`/static TF | M0 | needs robot |
| B | `g1_recorder` + `keyframe_manager` | M0→M1 | yes (vs bag) |
| C | `vggt_reconstruction` + `/vggt` service API | M1 | yes (fully) |
| D | `metric_registration` + `scene_server` | M2→M3 | yes (mock VGGT out) |
| E | `semantic_query` (Grounding DINO + SAM2) | M4 | yes (fully) |

Cross-cutting (assign to lead): freeze the **keyframe struct** + **VGGT-output struct** + topic/
frame names before coding; own the **canonical shared rosbag**; own the **safety checklist**.

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
- **`scene_server` is a placeholder owned by D** — the real reconstruction node MUST keep the
  `/vggt/scene_cloud` topic + `vggt_world` frame contract (a static `map→vggt_world` TF is provided).

#### FROZEN keyframe struct — Module 2 output, contract for C/D/E
`keyframe_manager` writes, and VGGT/registration/POI must read:
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
non-issue (no chrony/NTP needed); the recorder uses message header stamps.

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

- M0–M4 (sensing, reconstruction, POI): unaffected; built-in mode + sensors + manual/remote survey. Read-only sensor discovery stays the default.
- M5 (autonomous survey): gated by 25.1 / 25.3. Decide early whether to use high-level Loco client (fewer requirements) or `rl_hnav` (harness trial + supervisor first). Budget time for the harness trial and supervisor dry-run.
- Mounting extra sensors: <= 1 kg, non-destructive, no occluding of existing sensors.
- Recording (M0): `rosbag2` and supervisor logs run on the dev machine or a non-critical thread; do not add load to the command path.
