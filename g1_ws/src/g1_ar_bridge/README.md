# g1_ar_bridge: Snap Spectacles ↔ G1 (AGENTS.md §27)

Puts what the G1 knows into the room of a person wearing Snap Spectacles (2024): the robot, its
LiDAR map, the Nav2 path, semantic POIs and 3D boxes. The glasses run the unchanged
**Dimensional OS** Lens (see `ar_glasses/README.md`); this package is the laptop side, the
"bridge" the Lens connects to (`ws://<laptop IP>:8787`, protocol v19). It also puts the glasses
into our TF tree.

```text
   Spectacles (Lens)                      robot laptop (Ubuntu, ROS 2 Humble, Ethernet to G1)
  ─────────────────────                   ───────────────────────────────────────────────────
  camera frames + glasses pose ──Wi-Fi──► ar_bridge ◄── TF map -> ar_tag_0 ◄── tag_anchor ◄── robot camera
  robot box, LiDAR, POIs, path ◄────────   │   ◄── TF map -> robot_center, /cloud_map, /plan (g1_mapping, Nav2)
                                           │   ◄── /ar_glasses/markers (POIs / boxes)
                                           └──► TF map -> ar_world -> spectacles, /ar_glasses/hmd_pose
```

## How the glasses are located: one AprilTag, two cameras

One **AprilTag 36h11 (ID 0) on a wall** is the shared reference. Both sides measure it:

| Side | Measures | How |
|---|---|---|
| robot (`tag_anchor`) | `T_map_tag` | robot camera image + CameraInfo (+ aligned depth) → tag PnP over many frames while the robot **stands still**, TF `map ← camera` from our stack. Depth fits the wall plane and corrects the tag's orientation. Published as the static TF `map -> ar_tag_0`. |
| glasses (`ar_bridge`) | `T_ar_tag` | the Lens's **AprilTag registration** streams camera JPEGs with the glasses' pose in their own world; the bridge finds the tag and refines its pose over all views (multi-view reprojection, so walking sideways triangulates it). |

`T_ar_map = T_ar_tag · T_map_tag⁻¹`, levelled to yaw + translation (both worlds know "up": the
Spectacles from their IMU, RTAB-Map's `map` from the robot IMU). The tag centre is kept exact;
a large "up" disagreement (> `max_tilt_deg`) refuses the registration, since it means a wrong
camera TF. The two measurements do not have to be simultaneous: the tag does not move and the
anchor is in `map`. If the glasses finish first, the bridge waits (status *"Waiting for the robot
camera…"*) and commits as soon as `map -> ar_tag_0` appears.

Simulated accuracy (`test/test_alignment.py`, 17 cm tag, perfect cameras and tracking): yaw
≤ 0.4°, points ≤ 1-2 cm. In the real room expect a few cm (camera intrinsics, the robot
camera's mount TF, Spectacles tracking).

**Manual Placement** (drag the marker onto the robot, *Complete*) still works as a fallback:
the bridge uses the robot's `map` pose at that moment.

## Run it

### A. At home, no robot, no ROS (Windows / Ubuntu / macOS)

Tests the glasses side with a real tag on your wall and a simulated G1:

```bash
cd g1_ws/src/g1_ar_bridge
python -m pip install -r requirements-sim.txt        # websockets, numpy, opencv-python
python -m g1_ar_bridge.sim_main --tag-size 0.16      # black square of your print, in m
```

Glasses: Dimensional OS → the printed IP → Registration → **AprilTag** → look at the tag from
1-2 m and step sideways. The virtual G1 appears `--tag-distance` (1.5) m in front of the tag,
facing it; the synthetic room's front wall should lie on your wall; two POIs and a green box sit
on a virtual table to the robot's right.

### B. With the robot (Ubuntu laptop on the robot's Ethernet)

Inside the robot-connected container (`scripts/run_humble.sh`, `--net=host`, so port 8787 is
reachable on the laptop's Wi-Fi):

```bash
# our stack: /tf and the map (AGENTS.md §10.1, §8)
ros2 launch g1_sensors tf_chain.launch.py
ros2 launch g1_mapping mapping.launch.py static_tf:=false
# a robot camera that sees the wall tag: head RealSense (topics below), or the chest OAK-D
# bridge + anchor
ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16
```

1. Stand the robot **still, 1-2 m in front of the tag**, facing it. Within a few seconds
   `tag_anchor` logs `anchored map -> ar_tag_0` (`ros2 topic echo /ar_glasses/anchor_status`).
2. Glasses: Dimensional OS → the laptop's **Wi-Fi** IP → Registration → **AprilTag** → look at
   the wall tag and step sideways. On success the Lens shows the robot box on the real robot.
3. Check in RViz: fixed frame `map`, TF display shows `ar_tag_0`, `ar_world`, `spectacles`.
   `ros2 run tf2_ros tf2_echo robot_center spectacles` is the robot→glasses relation.

Which robot camera:

* **Head RealSense** (default topics `/camera/color/image_raw`, `/camera/color/camera_info`,
  `/camera/aligned_depth_to_color/image_raw`): its extrinsic is in the URDF, but it looks 48° down
  (AGENTS.md §7), so put the tag **low on the wall or on the floor** ~1 m in front of the robot.
* **Chest OAK-D**: best for a tag at chest height, but its topics are not verified yet and its
  mount TF (`torso_link -> <oak frame>`) must exist (measured, or from the calibration package).
  Pass `image_topic:=… camera_info_topic:=… depth_topic:=…` (depth aligned to the RGB image,
  same size).

The image's `frame_id` must be the camera's **optical** frame (OpenCV axes).

## Frames and topics

| Name | Type | From → to | Notes |
|---|---|---|---|
| `map -> ar_tag_<id>` | static TF | `tag_anchor` | tag centre, ArUco axes (x right, y up, z out of the wall) |
| `map -> ar_world` | static TF | `ar_bridge` | after each registration; the Spectacles' world (Y up) |
| `ar_world -> spectacles` | TF, ~2 Hz | `ar_bridge` | glasses, x forward / y left / z up |
| `/ar_glasses/hmd_pose` | PoseStamped (`map`) | `ar_bridge` | same pose as `spectacles` |
| `/ar_glasses/markers` | MarkerArray in | any node | POIs / boxes for the glasses: TEXT, SPHERE → labelled marker; CUBE → 3D box (+ `text`); LINE_STRIP / LINE_LIST → lines; DELETE / DELETEALL |
| `/ar_glasses/user_command` | String out | `ar_bridge` | voice / typed commands from the glasses |
| `/ar_glasses/status`, `/ar_glasses/anchor_status` | String (JSON) | both nodes | registration and anchor state |
| `/cloud_map`, `/plan` | in | g1_mapping, Nav2 | map cloud (voxelised, ≤ 1500 pts per frame), path |

`ros2 run g1_ar_bridge publish_demo_pois` publishes a labelled marker and a box in front of the
robot, to check the glasses before `semantic_query` exists.

Parameters: `config/ar_bridge.yaml` (documented inline). Launch arguments override the tag, the
camera topics and the port.

## Safety

Read-only towards the robot. The Lens's navigation marker, joystick and e-stop button are
disabled in the handshake; goals and e-stop requests are refused (AGENTS.md §19, §25). The only
real e-stop is the robot's remote. Voice commands are only republished as text.

## Tests

```bash
python3 -m pytest g1_ws/src/g1_ar_bridge/test        # no ROS needed (ROS test skips itself)
```

`test_ros_integration.py` runs `ros2 launch g1_ar_bridge ar_bridge.launch.py` with a fake robot
camera (rendered tag + depth + TF) and a scripted Lens; it needs a sourced Humble workspace with
this package built and uses `ROS_DOMAIN_ID=77`.
