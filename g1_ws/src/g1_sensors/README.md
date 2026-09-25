# g1_sensors — the G1's `/tf` chain (AGENTS.md §10.1)

The robot publishes no `/tf`. This package produces it on the host from `/lowstate`:

```text
/lowstate ──► lowstate_to_joint_states ──► /joint_states ──► robot_state_publisher (G1 29-DoF rev 1.0 URDF)
                (stamped on the robot clock)                          │
                                                                      ▼
robot_center ─► pelvis ─► waist yaw/roll/pitch ─► torso_link ─► mid360_link ─► livox_frame
     (glue)          └─► imu_in_pelvis ─► dog_imu_link      └─► d435_link ─► camera_link
                                          (glue)                              (glue)
```

`odom -> robot_center` comes from `g1_mapping` (launch it with `static_tf:=false`). Read-only
towards the robot: it subscribes to `/lowstate` and `/dog_imu_raw` and publishes `/joint_states`,
`/tf`, `/tf_static`, `/robot_description`.

## Run

Live, in the robot-connected container:

```bash
ros2 launch g1_sensors tf_chain.launch.py                 # rviz:=true for RobotModel + TF + LiDAR
ros2 launch g1_mapping mapping.launch.py static_tf:=false
```

Run it during every capture so `/tf`, `/tf_static` and `/joint_states` land in the bag. On a bag
recorded without it (replay only in the `SIM=1` container, domain 77):

```bash
ros2 launch g1_sensors tf_chain.launch.py use_sim_time:=true lowstate_topic:=/lf/lowstate
ros2 launch g1_mapping mapping.launch.py use_sim_time:=true static_tf:=false
ros2 bag play <bag> --clock 200
```

Topic names, the joint order, and the glue frames live in `config/g1_sensors.yaml`.

## Design notes

- **URDF:** `urdf/g1_29dof_rev_1_0.urdf`, vendored unchanged from `third_party/unitree_ros`. The
  robot is the 29-DoF model; every rev 1.0 variant (with or without hands) has the same sensor
  mounts. Rev 1.0 mounts `mid360_link` upside down (roll π), which matches the data, so
  `mid360_link -> livox_frame` is identity. The older `g1_29dof.urdf` has it upright: wrong for
  this robot.
- **Joint order:** `LowState.motor_state[0..28]` = the URDF's revolute joints in file order
  (unitree_sdk2 `G1JointIndex`, waist yaw = 12).
- **Stamps:** `LowState` has no header. Joint states are stamped on the **robot clock**: receive
  time + the median of (`/dog_imu_raw` header stamp − receive time) over 2 s. `/tf` therefore
  lines up with LiDAR and IMU stamps even though the laptop clock is ~73 s ahead. In replay the
  estimate is only as fine as `/clock`, so play with `--clock 200`.
- **Rate:** `/joint_states` and the moving TFs at 50 Hz live; 20 Hz on survey bags
  (`/lf/lowstate`). The sensors are on the torso, so only the three waist joints move them.
- **No Unitree SDK in the process** (AGENTS.md §5): `/lowstate` is read with the `unitree_hg`
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
- OAK-D: measure its mount relative to `torso_link` and add it to `static_transforms`.
- The ~1.5° pelvis-IMU disagreement tilts the map by that much when `/dog_imu_raw` is the
  gravity reference; `imu_source:=livox` avoids it (its IMU is rigid with the LiDAR). Decide
  with `compare_imu_sources` on a walking bag.
- `robot_center -> pelvis` identity: `/dog_odom` height is 3–5 cm lower than the pelvis height
  from the LiDAR and the URDF feet. Irrelevant for mapping (odometry comes from ICP).
