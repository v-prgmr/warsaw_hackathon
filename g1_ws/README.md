# g1_ws — G1 sensing / capture workspace

ROS 2 packages for the G1 capture → keyframe → visualize path (Module 1/2 + a Module 5 stub).

| Package | Type | Role |
|---------|------|------|
| `g1_recorder` | ament_cmake | MCAP `rtab`, canonical `survey`, and audited `live_run` recording profiles + discovery |
| `g1_sensors` | ament_python | the robot's `/tf`: `/lowstate` → `/joint_states` bridge, G1 29-DoF rev 1.0 URDF, static glue frames; see its README |
| `g1_mapping` | ament_python | RTAB-Map LiDAR-inertial mapping (MID-360 + IMU, optional RGB-D color); see its README |
| `g1_loco_cmdvel` | ament_cmake | Safety-gated `/cmd_vel` to high-level Unitree G1 Loco `SetVelocity`; see its README |
| `g1_ar_bridge` | ament_python | Snap Spectacles bridge (AGENTS.md §27): wall-AprilTag alignment of the glasses with `map` (`tag_anchor` + `ar_bridge`), robot / LiDAR / POIs to the glasses; read-only; see its README |
| `keyframe_manager` | ament_python | select ~8–20 RGB-D keyframes → frozen keyframe struct + manifest |
| `scene_server` | ament_python | **STUB (owned by D)** — synthetic `/scene_cloud` in `map` for the RViz surface |

## Build
```bash
conda deactivate 2>/dev/null || true
cd ~/workspace/warsaw/g1_ws
source /opt/ros/humble/setup.bash          # or your distro
which python3                              # must be /usr/bin/python3
colcon build
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Offline test (no robot)
```bash
# record→replay + tf_static QoS
ros2 run demo_nodes_cpp talker &
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom &
ros2 launch g1_recorder record.launch.py profile:=survey  # Ctrl-C to stop -> g1_survey_<ts>/
ros2 bag info g1_survey_* && ros2 bag play g1_survey_*

# viz stub
ros2 run scene_server ply_publisher                 # rviz2 -d src/g1_recorder/rviz/scene.rviz

# keyframe extraction against a synthetic RGB-D source
ros2 run keyframe_manager fake_rgbd_pub &
ros2 run keyframe_manager keyframe_node --ros-args -p output_dir:=./keyframes
```

## Testing from the computer connected to the robot (R4/R5)

The RealSense + LiDAR + odom run on the **Orin**. To see/record them from the dev laptop, the
laptop must join the robot's DDS graph over the **wired** link (Wi-Fi will not carry it reliably).

### 1. Put the laptop on the robot LAN
```bash
# Plug Ethernet laptop <-> G1. Replace the interface if `ip -brief address` differs.
sudo ip addr add 192.168.123.222/24 dev enx3c33327bdb58
sudo ip link set enx3c33327bdb58 up
ping -c1 <ORIN_IP>                      # e.g. 192.168.123.164 — must succeed
```

### 2. Use the CycloneDDS config bound to the wired NIC
```bash
source /opt/ros/humble/setup.bash
source ~/workspace/warsaw/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="enx3c33327bdb58" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>'
```

### 3. Discover (read-only) and reconcile config
```bash
ros2 run g1_recorder discover_sensors.sh            # -> sensor_discovery_<ts>.md
```
The report lists nodes, all topics+types, per-topic QoS/rate, and static TF frames. Then:
- Reconcile the **verified** names in `g1_recorder/config/survey.yaml` and
  `g1_recorder/config/live_run.yaml` and
  `keyframe_manager/config/keyframe_params.yaml`. **Do not keep unverified names.**
- Confirm **aligned depth** exists (RealSense launched with `align_depth.enable:=true`) — required
  for POI back-projection and map coloring (mapping itself uses LiDAR + IMU).
- If a sensor topic is `best_effort` and capture drops messages, add a QoS override in
  `g1_recorder/config/qos_override.yaml`.

Verified on the live domain-0 graph on 2026-09-25: `/dog_odom` is `nav_msgs/msg/Odometry` with
frames `odom` -> `robot_center`, and `/dog_imu_raw` is `sensor_msgs/msg/Imu` in `dog_imu_link`.
The RealSense topics only appear after its ROS node is started.

> Alternative: run `discover_sensors.sh` **directly on the Orin** (zero network variables) and
> `scp` the report back — surest way to confirm the RealSense namespace and `align_depth`.

### 4. Record the canonical bag (recorder on the laptop, off the robot command path)
```bash
ros2 param load /camera/camera \
  "$(ros2 pkg prefix g1_recorder)/share/g1_recorder/config/realsense_rtab.yaml"
ros2 launch g1_recorder record.launch.py profile:=survey output:=survey_take
# write down 2-3 tape-measured dimensions of the scene next to the bag name (AGENTS.md §10.2)
ros2 bag info survey_take                            # verify all required topics have messages
# replay only in an isolated DDS domain (SIM=1 scripts/run_humble.sh -> domain 77), never on the
# robot's domain 0: the bag's /dog_odom, /lf/lowstate, LiDAR (and /api/sport/request in live_run
# bags) would reach the live robot's network
ros2 bag play survey_take --clock             # in another terminal, run keyframe_manager with use_sim_time:=true
```
Publish that bag as the team's shared fixture.

See `g1_recorder/README.md` for the full command reference and the frozen keyframe struct.

### 5. Sync the laptop clock to the robot (before live Nav2 / m-explore runs)

All sensor topics come from Unitree's locomotion computer `192.168.123.161` and carry its clock,
which ran 76.5 s behind the laptop (2026-09-26). RTAB-Map and replay cope; Nav2 and m-explore
compare TF ages with the laptop clock and stop working. `.161` already runs an NTP server, so the
laptop can follow it; nothing changes on the robot (AGENTS.md §7, Clock; approved by x-kom on
2026-09-26):

```bash
sudo mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=192.168.123.161\nFallbackNTP=\n' | sudo tee /etc/systemd/timesyncd.conf.d/g1-robot.conf
sudo systemctl restart systemd-timesyncd
timedatectl timesync-status          # Server: 192.168.123.161; the first sync steps the clock
```

Undo after the event: `sudo rm /etc/systemd/timesyncd.conf.d/g1-robot.conf && sudo systemctl
restart systemd-timesyncd`. Docker containers follow the host clock.

### 6. End every live session cleanly

Nothing of ours may keep publishing onto the robot's network after a session (AGENTS.md §19):

```bash
bash /ws/scripts/stop_ros.sh      # inside the container: Ctrl-C to every ROS process, then escalate
scripts/stop_humble.sh            # on the host: the same in every g1-humble container, then docker stop
```

Never `kill -9` a `ros2 launch`: its nodes are orphaned and keep running. `kill -INT` on a launch
started with `&` from a script does nothing (bash starts background jobs with SIGINT ignored).
Our Python nodes also exit on their own when their launch process dies.
