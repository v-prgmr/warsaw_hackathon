# First autonomous Nav2 step — NO-MOTION prep (G0–G3)

This runbook takes the stack from cold to **fully ready for the first Loco motion**, and **stops at
the motor gate**. Nothing here moves the robot: G0–G2 never touch `/cmd_vel` to the legs, G3 keeps
the Loco client disabled. Stage 4 (the first few-cm nudge) and the live Nav2 goal are **out of scope
here** — do them only after every gate below is green, under supervision (`g1_loco_cmdvel/README.md`,
AGENTS.md §25).

## Gates (all must be green before Stage 4)
- [ ] G0 laptop clock synced to `192.168.123.161`
- [ ] G1 `check_rtabmap_plan` → **READY** on a surveyed (driven-around) map
- [ ] G2 Nav2 command-only: plan computes, velocities sane, real `/cmd_vel` has **0 publishers**
- [ ] G3 Stage-3 dry run: gateway clamps + watchdog zero, SDK client stays disabled

Wired NIC is `enx3c33327bdb58` (change if `ip -brief address` differs); robot domain is
`ROS_DOMAIN_ID=0`; Loco computer `.161`, Orin `.164`.

---

## Common shell (every live terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/workspace/warsaw/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="enx3c33327bdb58" priority="default" multicast="default"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
ping -c1 192.168.123.164   # reach the robot; confirm the wired link is up
```

## G0 — Clock sync (no motion)
```bash
sudo mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=192.168.123.161\nFallbackNTP=\n' | sudo tee /etc/systemd/timesyncd.conf.d/g1-robot.conf
sudo systemctl restart systemd-timesyncd
timedatectl timesync-status          # Server: 192.168.123.161; first sync steps the clock
```
**Gate G0:** `timedatectl` shows server `.161` and the offset has stepped to ~0.
Undo after the event: `sudo rm /etc/systemd/timesyncd.conf.d/g1-robot.conf && sudo systemctl restart systemd-timesyncd`.

## G1 — Sensing + survey a map (manual driving with the vendor remote only)
Terminal A — TF + joint states + `/battery_state`:
```bash
ros2 launch g1_sensors tf_chain.launch.py
```
Terminal B — RTAB-Map (owns `map -> odom -> robot_center`, `/map`, `/cloud_map`):
```bash
ros2 launch g1_mapping mapping.launch.py static_tf:=false     # live: g1_sensors owns the URDF TF
```
Terminal C — `/scan` for Nav2 (rl_hnav pipeline, its TF/SLAM **off** so it doesn't fight RTAB-Map, §14):
```bash
# either the standalone scan pipeline (rl_hnav/README.md) or:
ros2 launch rl_hnav real_robot_bridge.launch.py \
  publish_odom_tf:=false publish_lidar_tf:=false override_scan_stamp:=false use_slam:=false
```
Now **drive the G1 slowly with the vendor remote** to cover the area (a small loop helps closures).
Watch `/map` fill and ICP loop closures in Terminal B.

Save the 2D map (optional, for reuse / localization mode):
```bash
ros2 run nav2_map_server map_saver_cli -f g1_first_map
```
**Gate G1:**
```bash
ros2 run g1_nav2 check_rtabmap_plan       # must print READY (standing-only map => NOT READY)
```

## G2 — Nav2 command-only dry run (no motion — velocity is namespaced away from the legs)
```bash
ros2 launch g1_nav2 rtabmap_nav_dry_run.launch.py   # controller/smoother -> /g1_nav2_dry_run/cmd_vel
ros2 topic info /cmd_vel -v --no-daemon             # MUST show 0 publishers, 0 subscribers
```
Send a short goal and watch it plan (velocity only appears on the dry-run topic):
```bash
# read current robot_center x,y from TF, then goal ~0.5 m ahead in map:
ros2 topic echo /g1_nav2_dry_run/cmd_vel &          # observe: small, sane vx/wz
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: <cur_x+0.5>, y: <cur_y>}, orientation: {w: 1.0}}}}"
```
**Gate G2:** a plan is produced, `/g1_nav2_dry_run/cmd_vel` carries small velocities, and real
`/cmd_vel` still has **0 publishers**.

## G3 — Stage-3 Loco dry run (no motion — SDK client disabled, isolated domain 77)
Run **off** the robot graph: new terminals with `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and **no** `CYCLONEDDS_URI`. Full procedure in
`g1_ws/src/g1_loco_cmdvel/README.md` → "Stage-3 dry run". In short:
```bash
# T1 gateway (disabled), T2 SDK client on lo (dry-run), T3 echo /cmd_vel
ros2 run g1_loco_cmdvel cmd_vel_gateway --ros-args -p enabled:=false -p require_battery:=true \
  -p battery_topic:=/battery_state -p min_battery_percent:=0.20 \
  -p battery_timeout_sec:=1.0 -p command_timeout_sec:=0.30 -p max_vx:=0.10 -p max_vy:=0.05 -p max_wz:=0.20
ros2 run g1_loco_cmdvel g1_loco_client --network-interface=lo
```
**Gate G3:** gateway reports blocked/clamped commands, watchdog sends zero after 0.30 s, SDK client
only logs (never `--enabled=true` here). Optionally verify clamping with a synthetic
`BatteryState percentage: 0.5` + test `/cmd_vel` (gateway `enabled:=true`, **client still disabled**).

---

## STOP — motor gate
All four gates green? The next action is **Stage 4** (first enabled motion, a few cm) in
`g1_loco_cmdvel/README.md`, then the live Nav2 goal (G5 in the chat runbook). Those move motors:
2 people, harness, remote operator ready with the damping combo (Ctrl-C is **not** an e-stop). Do
not proceed here.

## End every session cleanly (AGENTS.md §19)
```bash
bash ~/workspace/warsaw/scripts/stop_ros.sh     # never kill -9 a ros2 launch
```
Nothing of ours may keep publishing on the robot's domain 0 after the session.
