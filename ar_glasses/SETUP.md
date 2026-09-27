# AR glasses + G1: setup and running a session

Snap Spectacles (2024) show what the G1 knows in the real room: the robot, the LiDAR map, the
Nav2 path and POIs. The glasses and the robot are aligned with one AprilTag on a wall that both
see. Background: [`README.md`](README.md) (glasses side, Lens deployment),
[`g1_ar_bridge`](../g1_ws/src/g1_ar_bridge/README.md) (the bridge), AGENTS.md §27.
Nothing here moves the robot.

```text
  G1 ──Ethernet cable── laptop (192.168.123.222)             robot data: LiDAR, joints, OAK-D
  laptop Wi-Fi ──── phone hotspot / router ──── Spectacles (Wi-Fi, no cable)
                    the glasses connect to  ws://<laptop Wi-Fi IP>:8787
```

## 1. One-time setup

### Glasses (Windows or macOS, once per pair of glasses)
Install the **Dimensional OS** Lens with Lens Studio 5.15.4 over USB-C: [`README.md`](README.md)
Part 1. It stays in the glasses' **Drafts**; after that no computer cable is needed.

### Robot laptop (Ubuntu with Docker)
```bash
git clone git@github.com:v-prgmr/warsaw_hackathon.git && cd warsaw_hackathon
SIM=1 scripts/run_humble.sh          # builds the Docker image g1-humble (needs internet), opens a shell
# inside that shell:
cd /ws/g1_ws && colcon build --packages-select g1_sensors g1_mapping g1_ar_bridge && exit
```

Clock sync to the robot, so live TF times match (AGENTS.md §7, approved by x-kom):
```bash
sudo mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=192.168.123.161\nFallbackNTP=\n' | sudo tee /etc/systemd/timesyncd.conf.d/g1-robot.conf
sudo systemctl restart systemd-timesyncd
```

### The tag
Print AprilTag **36h11, ID 0**, with its white margin, flat on card. Measure the **black
square** with a ruler (e.g. 16.0 cm → `0.16`). Stick it on a wall at **chest height (about
1.1–1.2 m)** for the chest OAK-D (for the head RealSense, which looks 48° down: low on the wall
or flat on the floor).

## 2. Each session

### Connections
1. Ethernet cable laptop ↔ robot, then on the laptop:
   ```bash
   sudo ip addr add 192.168.123.222/24 dev enp2s0     # your wired interface; "File exists" is fine
   sudo ip link set enp2s0 up
   ```
2. Laptop Wi-Fi and glasses Wi-Fi on the **same** hotspot or router (event Wi-Fi often blocks
   devices from seeing each other: use a phone hotspot). Wi-Fi to the robot is not enough for
   ROS: its data only travels over the cable.

### Start everything: one script
```bash
bash scripts/start_ar_glasses.sh --tag-size 0.16 --orin
```
It checks the cable, the Orin and the clock sync, **prints the IP to type into the glasses**,
starts a Docker container and opens one window each for:

| Window | What |
|---|---|
| AR 0 (with `--orin`) | SSH to the Orin, starts the OAK-D driver; **type the Orin password** there |
| AR 1 | robot TF (`g1_sensors tf_chain`: URDF, joints, OAK-D mount calibration) |
| AR 2 | map (`g1_mapping`, RTAB-Map) |
| AR 3 | the glasses bridge + the robot's tag measurement (`g1_ar_bridge`) + the demo scene |
| AR 4 | checks (`/ar_glasses/anchor_status`) |

Options: `--no-demo` (no virtual table / green box), `--camera realsense`, `--nic <iface>`, `--build` (rebuild first), `--print-only` (show
the commands only), and **`--bridge-only`** when another laptop already runs TF + map (e.g.
`scripts/start_g1_navigation.sh`): the script refuses to start a second TF/map owner, which
would make TF jump (AGENTS.md §6). Stop with `bash scripts/stop_ar_glasses.sh`.

### Then
1. **Robot measures the tag.** Stand it **still**, 1–1.5 m in front of the tag, facing it
   (vendor remote). Window AR 3: `anchored map -> ar_tag_0 ... 'tag_up_vs_map_up_deg': X` —
   X should be a few degrees; a large X means the camera calibration is off.
2. **Glasses.** Menu → **Drafts → Dimensional OS** → *Start Robot & Bridge*: **Next** →
   *Connect*: the IP printed by the script (AR 3: `Lens connected`) → *Registration*:
   **AprilTag**. Look at the tag from 1–1.5 m, step sideways in small steps, **pause about 1 s
   at each**. **Do not press Skip.** After about 30 s, AR 3: `registered (april_tag)`; the
   robot box stands on the real G1.
3. **Look.** Wrist menu (left palm up) → LiDAR **full**: the map points on walls and furniture.
   The **demo scene** from the home test (a virtual table, a green box on it, a "red bottle"
   label) stands to the front right of where the robot was at start; it is virtual, and
   `--no-demo` turns it off.
   In AR 4 (Ctrl-C the echo first):
   ```bash
   ros2 run tf2_ros tf2_echo robot_center spectacles     # the glasses' position from the robot
   ros2 run g1_ar_bridge publish_demo_pois               # the demo scene (already on with the script)
   ```
   Any node can draw in the glasses: `visualization_msgs/MarkerArray` on `/ar_glasses/markers`
   (frame `map`).

**Each time** the Lens starts or reconnects, the glasses scan the tag again (their world is new
every session). **Each time mapping restarts**, the robot measures the tag again (new map).

### Without the script (four terminals)
```bash
# host:
ROBOT_IFACE=enp2s0 scripts/run_humble.sh                     # T1; T2-T4 join it:
docker exec -it $(docker ps -q --filter ancestor=g1-humble | head -1) bash
# in each:  source /ws/g1_ws/install/setup.bash
ros2 launch g1_sensors tf_chain.launch.py                                          # T1
ros2 launch g1_mapping mapping.launch.py static_tf:=false                          # T2
ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 camera:=oak    # T3
ros2 topic echo /ar_glasses/anchor_status                                          # T4
```
OAK-D driver on the Orin (domain **0** for the glasses; the OAK/LiDAR calibration capture uses 78):
```bash
ssh unitree@192.168.123.164
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
ros2 launch depthai_ros_driver camera.launch.py use_rviz:=false
```

### End
```bash
bash scripts/stop_ar_glasses.sh      # or, for any of our containers: scripts/stop_humble.sh
```
Ctrl-C the OAK-D driver on the Orin. Never `kill -9` a `ros2 launch` (AGENTS.md §19).

## 3. When something does not work

| Symptom | Fix |
|---|---|
| Glasses: "Bridge not connected" | same hotspot for laptop and glasses? The IP printed by the script (not 192.168.123.x)? AR 3 running? On the laptop `ip neigh` should list the glasses |
| Script: "no 192.168.123.x address" / Orin does not answer | cable, robot on, the `ip addr add` commands above |
| Script: "someone already publishes /tf or /map" | another laptop runs TF + map: use `--bridge-only`, or stop the other one |
| AR 1: `waiting for clock_reference` | normal for ~2 s at start |
| No `/oak/` topics | OAK-D driver on the Orin not running, or in the wrong domain (must be 0) |
| AR 3 stays "searching" | the robot camera does not see the tag: closer, facing it, light |
| Registration never finishes | read the `waiting for:` line in AR 3. The glasses hide their text above 80 %; the bridge now keeps it below until it commits. Keep `bags/ar_registration/` and replay it: `python3 -m g1_ar_bridge.replay_registration bags/ar_registration` (in the container) |
| "disagree on 'up'" | the robot camera's TF is wrong: check the OAK-D mount calibration |
| Robot box offset from the robot | wrong `--tag-size`, the tag moved, or the (taped) OAK-D mount shifted: re-measure the calibration (`g1_sensors` README, OAK-D ↔ LiDAR) |
| Dimensional OS missing from Drafts | redeploy from Lens Studio ([`README.md`](README.md) Part 1) |

## 4. With the Leo Rover too (shared world)

The same wall tag also places the **Leo Rover** in the G1 map, and the glasses mark it as a blue
box with a heading line and a "Leo Rover" label (details and status:
[`g1_ws/docs/leo_g1_laptop_integration.md`](../g1_ws/docs/leo_g1_laptop_integration.md)).

```text
  G1 ──Ethernet── laptop ── Wi-Fi ── Leo's hotspot (10.0.0.1) ── glasses (Wi-Fi)
```
1. Laptop Wi-Fi **and** glasses on **Leo's hotspot** (the G1 stays on the cable).
2. `bash scripts/start_ar_glasses.sh --tag-size 0.16 --orin --leo`: an extra window SSHes to
   Leo (`pi@10.0.0.1`, type Leo's password) and runs a **read-only relay** there for the session
   (nothing installed; ask the Leo team first). Leo's AprilTag detector must be running.
3. Point Leo's front camera at the wall tag (0.2–4 m). The launcher's AR 3 window logs
   `Leo placed in map: (x, y) m, yaw …`; `ros2 topic echo /leo_in_g1/status` shows sightings,
   rejections and the pose. Between sightings Leo's odometry carries the pose.
4. **Provisional:** Leo's camera mount is not measured yet (`leo_camera_xyz` in
   `config/ar_bridge.yaml`): expect an offset of about the camera's distance from Leo's centre
   until it is measured.

## 5. Useful extras
- **Home test without the robot** (real glasses, tag on your wall, fake robot):
  `SIM=1 scripts/run_humble.sh`, then
  `ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 fake_robot:=true`.
- **Every glasses registration is recorded** in `bags/ar_registration/`, for
  `replay_registration` (per-frame diagnosis, other thresholds, annotated frames).
- **Reuse a map and the robot's tag measurement:** `g1_mapping ... localization:=true
  database_path:=/ws/bags/maps/room.db` + the bridge's `anchor_file` / `load_saved`
  (`config/ar_bridge.yaml`); only valid while the tag and the camera mount do not move.
- The OAK-D mount is calibrated against the LiDAR (`g1_sensors/config/oakd_livox_taped_20260927.yaml`,
  taped mount): if the camera is bumped, recalibrate before trusting the overlay.
