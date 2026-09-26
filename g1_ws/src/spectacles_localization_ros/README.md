# Spectacles localization ROS prototype

Scope: localize a Spectacles wearer in the existing G1 RTAB-Map `map` frame.
See [architecture and frame/packet contract](../../../docs/spectacles_localization_architecture.md).
No physical glasses integration has been tested yet.

## Build and offline test

Run in the project's ROS 2 Humble environment (`SIM=1 scripts/run_humble.sh`
from the repo root for an isolated DDS domain), then:

```bash
cd /ws/g1_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select spectacles_localization_ros
source install/setup.bash
ros2 launch spectacles_localization_ros spectacles_localization.launch.py mock:=true
```

The mock publishes `map -> april_tag_0`, then registers and publishes
`map -> spectacles`; it logs `MOCK PASS` when the wearer pose agrees with
synthetic ground truth. Press Ctrl-C to stop. Check in a second shell in the
same ROS domain:

```bash
ros2 topic echo /spectacles/localization_status
ros2 run tf2_ros tf2_echo map spectacles
rviz2
```

In RViz, use Fixed Frame `map` and add TF, Pose (`/spectacles/pose`), Pose
(`/spectacles/mock_truth`), and any available RTAB cloud/map displays.

Run the geometry regression tests outside ROS (also works on the Ubuntu host):

```bash
cd /ws
PYTHONPATH=g1_ws/src/spectacles_localization_ros \
  python3 -m unittest discover -s g1_ws/src/spectacles_localization_ros/test -v
```

## Physical prototype

1. Confirm the Spectacles model and Lens Studio version. Make a real Lens that
   produces the [documented JSON packet](../../../docs/spectacles_localization_architecture.md)
   with synchronized World tracking, tag observation or paired camera JPEG,
   intrinsics, verified head-to-optical transform, and a new session ID at
   each local-tracking reset. No such Lens is currently in this repo.
2. Measure the AprilTag 36h11 black-square edge and its position **and
   orientation** in the RTAB map. Set `tag.size_m`, `tag_pose_map`, and
   `publish_tag_tf: true` in `config/spectacles_localization.yaml`. If another
   node already owns `map -> april_tag_0`, leave the YAML tag pose null and
   `publish_tag_tf: false` instead.
3. Start the existing `g1_sensors`/`g1_mapping` stack normally. Do not start
   SLAM Toolbox or any extra `map -> odom` publisher.
4. Build/source this package, then start the bridge on a dedicated lab LAN:

```bash
ros2 launch spectacles_localization_ros spectacles_localization.launch.py \
  bind_host:=0.0.0.0 token:=YOUR_TEST_TOKEN
```

The Lens connects to `ws://<host-lan-ip>:8765` and includes the matching
`token` in each packet. This is plaintext, prototype-only, and requires Snap
Experimental APIs; do not send private camera images over an untrusted LAN.
By default the bridge binds `127.0.0.1` and does not require a token.

5. Stand still and observe the tag for at least 10 detections; watch the status
   become `LOCALIZED`. Walk away: `/spectacles/pose` and `map -> spectacles`
   should continue moving without the tag. Revisit the tag and record its
   residual. Check the wearer location against tape-measured landmarks.

## ROS interface

| Topic / TF | Type | Meaning |
|---|---|---|
| `/spectacles/observation` | `std_msgs/String` | validated JSON ingress (no JPEG) |
| `/spectacles/device_pose_local` | `geometry_msgs/PoseStamped` | `T_spectaclesWorld_device` |
| `/spectacles/tag_detection` | `geometry_msgs/PoseStamped` | `T_device_tag` |
| `/spectacles/pose` | `geometry_msgs/PoseStamped` | `T_map_device` |
| `/spectacles/localization_status` | `std_msgs/String` | JSON state, scatter, age, tag-revisit error |
| `map -> spectacles` | TF dynamic | same head/device pose in ROS tree |
| `map -> april_tag_0` | TF static, optional | only when YAML owns measured tag pose |

`/spectacles/mock_truth` is published only in mock mode. No custom ROS
messages, G1 commands, map imports, ICP, or Spectacles-side ROS runtime.
