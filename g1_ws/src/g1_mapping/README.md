# g1_mapping — RTAB-Map LiDAR-inertial mapping (AGENTS.md §8)

MID-360 + IMU give odometry, the 3D map and the 2D occupancy grid; RealSense RGB-D optionally colors
map nodes. RTAB-Map is the only `map -> odom` owner. Topic and frame names live in
`config/g1_mapping.yaml`.

```text
/utlidar/cloud_livox_mid360 -> livox_cloud_fix -> lidar_deskewing -> icp_odometry -> rtabmap
   (time ns -> s, drop zeros)   (IMU-stabilized frame via imu_to_tf)   (odom->robot_center)   (map->odom, /map, /cloud_map)
```

## Run on a bag

In the sim container (`SIM=1 scripts/run_humble.sh`, DDS domain 77). Never replay on the robot's
domain 0: the bag's robot topics would reach the live G1 (see `g1_recorder/README.md`).

```bash
ros2 launch g1_mapping mapping.launch.py use_sim_time:=true
ros2 bag play bags/full_survey_take_01 --clock
```

| Argument | Default | Meaning |
|---|---|---|
| `odom_source` | `icp` | `icp` = `rtabmap_odom icp_odometry` + IMU; `dog_odom` = `/dog_odom` → TF (fallback) |
| `use_sim_time` | `false` | `true` when replaying with `--clock` |
| `use_imu` | `true` | IMU for the ICP motion guess and deskewing |
| `imu_source` | `dog` | `dog` = `/dog_imu_raw` (pelvis); `livox` = MID-360 internal IMU via `livox_imu_fix` + `imu_complementary_filter` (bag needs `/utlidar/imu_livox_mid360`) |
| `deskewing` | `true` | deskew with the per-point `time` field |
| `use_rgbd` | `false` | attach RGB + aligned depth to map nodes (color; grid stays LiDAR-only) |
| `static_tf` | `true` | publish the **estimated** fallback extrinsics from the YAML (legacy bags without `/tf` or `/lf/lowstate`). Set `false` when `g1_sensors tf_chain` runs (live, or replaying `/lf/lowstate`) |
| `database_path` | `~/.ros/g1_rtabmap.db` | RTAB-Map database |
| `delete_db` | `true` | start a new map (deletes the database); ignored with `localization:=true` |
| `localization` | `false` | localize in an existing database instead of mapping |
| `rtabmap_viz`, `rviz` | `false` | GUIs (`rviz/mapping.rviz`: TF, `/map`, `/cloud_map`, deskewed scan, `/odom`) |

## Compare the IMU sources on a walking bag

```bash
ros2 run g1_mapping compare_imu_sources bags/<walking_bag>            # in the SIM=1 container
```

It measures the waist-joint motion while walking (from `/lf/lowstate`: the joints between the pelvis
IMU and the torso LiDAR) and the raw gyro bias while standing. It then replays the bag through
`g1_mapping` with `imu_source:=dog` and `livox`, and writes `<bag>_imu_compare/report.md`: lost scans,
ICP inlier ratio, loop closures, final `map -> odom` correction, trajectory z range on the flat
floor, and wall/floor thickness in `/cloud_map`. It refuses to replay on `ROS_DOMAIN_ID` 0;
`--analyze-only` skips the replays. Check the tape measurements against both maps as well.

Save the 2D map while it is being published: `ros2 run nav2_map_server map_saver_cli -f <name>`.

## Why the extra nodes

- `livox_cloud_fix`: the G1 publishes the per-point `time` as float32 **nanoseconds**, while
  `rtabmap_conversions` reads a float32 `time` as **seconds**. It also drops the ~38 % (0,0,0) points.
- `livox_imu_fix` (only for `imu_source:=livox`): `/utlidar/imu_livox_mid360` reports acceleration
  in **g** and no orientation. It scales to m/s², and `imu_complementary_filter` adds the
  orientation and estimates the gyro bias while the robot is still (raw bias ~0.6–0.9 °/s; all G1
  gyros have similar bias, which Unitree compensates onboard for `/dog_imu_raw`). On a 41 s standing
  capture (2026-09-25): roll 179.6° / pitch 2.7° (`livox_frame` upside down + the URDF's 2.3° mount
  pitch); yaw drift −0.005 °/s, vs 0.94 °/s with Madgwick (no bias estimation) and −0.016 °/s for
  `/dog_imu_raw`. Start with the robot standing for a few seconds. `dog` stays the default until a
  walking bag compares both.
- `odom_to_tf` (only for `odom_source:=dog_odom`): `/dog_odom` → TF + `/odom`, like rl_hnav's
  `odom_tf_bridge`. Prefer the team's bridge on the live robot.

## Results on `bags/full_survey_take_01` (153 s, ~25 m loop, no `/tf` in the bag → `static_tf:=true`)

| Mode | Odometry length | Nodes | LiDAR loop closures |
|---|---|---|---|
| `icp` (default) | 25.1 m, 0 resets | 129 | 13 |
| `icp` + `use_rgbd` | 24.9 m | 126 | 10 |
| `dog_odom` | 14.9 m (≈2× short, see AGENTS.md §8) | 113 | 0 |

ICP odometry: median 40 ms per scan (p95 50 ms) at 10 Hz on the dev laptop.

## Known limitations

- The static fallback extrinsics are ground-plane estimates for legacy bags, not a calibration.
  With `g1_sensors` (G1 URDF + joint states) use `static_tf:=false`.
- The recording laptop's clock was ~72 s ahead of the robot's clock. Replay is fine (header stamps),
  but live nodes comparing stamps to `now()` (Nav2, TF timeouts) need the clocks aligned.
- The first scan logs one `guess`/`deskew` error before the IMU TF is available; harmless.
