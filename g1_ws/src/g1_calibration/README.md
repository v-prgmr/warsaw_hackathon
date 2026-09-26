# g1_calibration — chessboard calibration of the chest OAK-D (AGENTS.md §10.4)

The OAK-D on the chest gives colour, semantics and POIs; the LiDAR and the RealSense sit on the
head. POI back-projection needs the OAK-D's pose relative to the LiDAR map, so this package
measures it with a printed chessboard:

| Step | Result | Method |
|---|---|---|
| intrinsics | OAK `camera_info` check / replacement | OpenCV `calibrateCamera`, compared with the factory (EEPROM) values |
| OAK ↔ RealSense | `T_realsense_oak` | both cameras see the same board: `cv2.stereoCalibrate`, intrinsics fixed |
| OAK ↔ LiDAR, path A | `T_lidar_oak` via the RealSense | `T_lidar_realsense` (G1 URDF /tf) · `T_realsense_oak` |
| OAK ↔ LiDAR, path B | `T_lidar_oak` direct | board plane from the OAK (PnP) = board points in the accumulated MID-360 cloud, over ≥ 3 tilted poses (Zhang & Pless 2004; Unnikrishnan & Hebert 2005): closed form + Gauss-Newton on point-to-plane residuals |
| output | `torso_link -> <OAK root frame>` | one line for `g1_sensors/config/g1_sensors.yaml` `static_transforms` |

Paths A and B are independent (A trusts the URDF's RealSense mount, B only the LiDAR), so their
agreement is the check. `final: auto` takes B (it relates the OAK to the sensor that builds the
map) when its plane RMS passes, else A.

The head RealSense is the reference camera: its extrinsic comes from the URDF + joint states
(`g1_sensors tf_chain`) and its intrinsics from its factory calibration. Both sensors and the
chest camera are rigid on `torso_link`, so the waist joints do not change the result.

## Board

- `ros2 run g1_calibration make_board --out board.png` writes the board of `config/calibration.yaml`
  (default 8 x 5 inner corners, 80 mm squares = 72 x 48 cm of squares + 3 cm margin). Print at
  **100 % scale**, glue **flat** onto a rigid board (foam board, plywood).
- **Measure the squares** (over several squares, with calipers) and put the value in `square`.
- The inner-corner counts must be one odd and one even (8 x 5, 9 x 6): then the corner order is
  unambiguous and both cameras agree on the board frame. The tools refuse symmetric boards.
- For the LiDAR the board must stand **free**: nothing directly behind it within ~10 cm, no
  wall around it, and its lower edge ≥ 15 cm above the floor. A tripod / stand is best; if a person holds it, hold it from the bottom edge.
- Big is better for the LiDAR (Livox recommends ~1 x 1.5 m at 3 m for their tool); at our 1–1.5 m
  the default size gives several hundred LiDAR points per pose in 3 s.

## Procedure (robot standing, built-in mode, no actuation — AGENTS.md §25)

1. On the robot: LiDAR (always on), RealSense driver (`start_realsense.md`), OAK-D driver
   (depthai-ros). **Verify** the OAK topics and TF root frame and set them in
   `config/calibration.yaml` (`cameras.oak`, `frames.oak_root`).
2. On the laptop (robot-connected container):
   ```bash
   ros2 launch g1_sensors tf_chain.launch.py                            # URDF /tf (LiDAR <- RealSense)
   ros2 launch g1_calibration capture.launch.py output_dir:=calib_$(date +%F)
   ros2 run g1_calibration calib_trigger                                 # 2nd terminal: Enter = 1 sample
   ```
   `rqt_image_view` shows `/calib_capture/debug/oak` (also `.../realsense`): the board with its
   origin circled, distance and PnP error.
3. Take **15–25 samples**. For each: board **still**, robot **still**, press Enter, wait
   `lidar_seconds` (3 s). The board must be fully visible in the OAK image; for path A also in
   the RealSense image (it looks ~48° down: hold the board low, knee height, 0.8–1.5 m in
   front, tilted up towards the head).
   Vary: left / centre / right, near / far, tilted **up/down AND left/right** by 20–45°. Poses
   that all face the same way make the LiDAR solve ill-conditioned (the tool refuses them).
4. Calibrate (offline, any container):
   ```bash
   ros2 run g1_calibration calibrate_intrinsics calib_2026-09-26 --camera oak   # check factory K
   ros2 run g1_calibration calibrate_extrinsics calib_2026-09-26
   ```
5. Read `calib_…/results/report.md` and look at `overlay_*.png` (LiDAR points coloured by
   distance on the OAK image; board points green: they must lie on the board and edges must
   line up). Checks: reprojection < 0.5 px, LiDAR plane RMS < 2.5 cm, paths A/B within 2 cm / 1°.
6. Paste the printed `static_transforms` line into `g1_sensors/config/g1_sensors.yaml`, note
   the date + dataset in AGENTS.md (§10.4), and check in RViz that the OAK depth cloud overlays
   the LiDAR map (AGENTS.md §10.3).

Capturing from a bag works too (`use_sim_time:=true`, replay in the SIM=1 container): record the
cameras, the LiDAR, `/tf`, `/tf_static` while holding each pose for ≥ 3 s.

## Intrinsics

The OAK-D is factory-calibrated and its **on-device depth-to-RGB alignment uses the EEPROM
calibration**, so keep the factory intrinsics unless `calibrate_intrinsics` shows they are clearly
worse. The tool prints both RMS errors and a recommendation. A calibrated
`results/oak_intrinsics.yaml` (ROS camera_info format) can be used for the RGB stream (depthai-ros
`i_calibration_file` / `set_camera_info`, or `calibrate_extrinsics --oak-intrinsics`); changing
the device itself is Luxonis' calibration tool (ChArUco, rewrites the EEPROM). For intrinsics,
also take close-up views that fill the image corners (`--images <dir>` adds extra images).
`--rational` fits the 8-coefficient model for wide-angle lenses.

## Launch arguments (`capture.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `config` | installed `config/calibration.yaml` | board, topics, frames, thresholds |
| `output_dir` | `calib_<timestamp>` | dataset directory; samples are appended |
| `lidar_seconds` | `3.0` | LiDAR accumulation per sample |
| `use_sim_time` | `false` | `true` when capturing from a replayed bag |
| `viewer` | `true` | `rqt_image_view` on `/calib_capture/debug/oak` |

## Dataset and outputs

```text
calib_…/sample_NNN/{meta.yaml, oak.png, oak_camera_info.yaml, realsense.png,
                    realsense_camera_info.yaml, lidar.npy}
calib_…/results/{extrinsics.yaml, report.md, overlay_sample_NNN.png,
                 oak_intrinsics.yaml, oak_intrinsics_report.yaml}
```

`meta.yaml` holds the TF looked up at capture (`torso_link <- livox_frame`,
`livox_frame <- camera_color_optical_frame`, `oak_root <- oak optical`). Without them the tools
fall back to `fallback_transforms` (URDF LiDAR mount) and, for path B's starting point, the
tape-measured `initial_guess`; path A then needs `body_to_realsense_optical`.

Conventions: `T_a_b` maps points from frame b to frame a (TF parent a, child b); Euler angles as
`static_transform_publisher` (R = Rz·Ry·Rx); quaternions (x, y, z, w).

## Design notes

- No timestamp sync: robot and board stand still during a sample, so the OAK (possibly on the
  laptop clock) and the robot sensors (robot clock) need no common time base.
- The board search in the LiDAR cloud is seeded by path A (or the tape-measured guess) and
  refined coarse-to-fine (30 → 10 → 5 cm); the predicted board normal rejects walls and floor.
  The closed-form solve does not depend on the seed.
- The capture node only subscribes: read-only towards the robot.

## Tests

`test/` runs the whole pipeline on a synthetic G1 rig (URDF LiDAR / RealSense mounts, rendered
chessboard images, noisy LiDAR board + floor + wall): both paths recover the true extrinsic within
1 cm / 0.3°, the corner order is canonical under any in-plane rotation, degenerate pose sets are
refused. Run with `scripts/run_tests.sh` (AGENTS.md §26).
