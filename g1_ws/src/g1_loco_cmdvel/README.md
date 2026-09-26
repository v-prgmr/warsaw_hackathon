# g1_loco_cmdvel

Safety-gated bridge from ROS 2 `/cmd_vel` to the Unitree G1 high-level Loco
API. The SDK client and ROS gateway are separate processes so the Unitree SDK's
bundled DDS libraries do not enter the normal ROS 2 process.

The Unitree SDK method is `LocoClient::SetVelocity(vx, vy, omega, duration)`.
This package intentionally defaults to no actuation:

- `cmd_vel_gateway` defaults to `enabled:=false`; battery is invalid until a
  fresh `sensor_msgs/msg/BatteryState` message arrives.
- `g1_loco_client` defaults to dry-run and rejects `--enabled=true` without
  its separate explicit acknowledgment.
- The SDK client logs received commands and only calls `SetVelocity` after the
  explicit `--enabled=true --i-accept-high-level-actuation=true` opt-in.
- Even when enabled, the SDK client independently subscribes to the raw Unitree
  `rt/lf/bmsstate` stream, requires fresh SOC >= 20%, clamps every packet,
  calls `StopMove` after 0.30 s without a command or on BMS loss, and stops on
  SIGINT/SIGTERM. The client never accepts a ROS battery parameter as a bypass.

## Build

```bash
cd ~/workspace/warsaw/g1_ws
source /opt/ros/humble/setup.bash
export UNITREE_SDK_ROOT=~/workspace/warsaw/rl_hnav/src/rl_sar/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2
colcon build --packages-select g1_loco_cmdvel
source install/setup.bash
```

To check the Unitree SDK's own battery channel **read-only**, with Ethernet
connected and no locomotion process running:

```bash
ros2 run g1_loco_cmdvel g1_bms_probe enx3c33327bdb58
```

This SDK-only executable must load Unitree's bundled CycloneDDS libraries.
The installed SDK binaries use RPATH to prevent the sourced ROS environment
from loading ROS's incompatible `libddsc.so.0` in the same process. The probe
does not create a Loco client or publish motion commands.

## Stage-3 dry run

Run this test **off the robot DDS graph**: in each terminal, source the Humble
and `g1_ws` setups, then set `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`, and
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Unset any Ethernet `CYCLONEDDS_URI`.
The SDK client must remain disabled; in that mode it never initializes SDK DDS.
No Nav2 or mapping stack is needed for synthetic input.

Terminal 1, normal ROS process:

```bash
ros2 run g1_loco_cmdvel cmd_vel_gateway --ros-args \
  -p enabled:=false \
  -p require_battery:=true \
  -p battery_topic:=/battery_state \
  -p min_battery_percent:=0.20 \
  -p battery_timeout_sec:=1.0 -p command_timeout_sec:=0.30 \
  -p max_vx:=0.10 -p max_vy:=0.05 -p max_wz:=0.20
```

Terminal 2, SDK-isolated process:

```bash
ros2 run g1_loco_cmdvel g1_loco_client \
  --network-interface=lo
```

Terminal 3, inspect commands without actuating:

```bash
ros2 topic echo /cmd_vel
```

The gateway must report blocked commands and the SDK process must only log
packets. To test clipping, the gateway may run with `enabled:=true` **only while
the SDK client stays disabled**. With the battery gate enabled, publish a
synthetic `BatteryState` with `percentage: 0.5`, then a test `/cmd_vel` to
observe the clamped packet; a missing, low, or stale battery must yield zero.
When commands stop, the gateway sends zero after 0.30 s and the dry SDK client
logs its own watchdog action. Do not pass `--enabled=true` to the SDK client
during Stage 3.

The supervised high-level test requires both processes to be explicitly
enabled:

```bash
ros2 run g1_loco_cmdvel cmd_vel_gateway --ros-args \
  -p enabled:=true -p require_battery:=true \
  -p battery_topic:=/battery_state

ros2 run g1_loco_cmdvel g1_loco_client \
  --network-interface=<WIRED_INTERFACE> \
  --enabled=true \
  --i-accept-high-level-actuation=true
```

Only use that mode after the operator, e-stop, battery, flat-area, speed-limit,
and controller readiness checks are complete. Start `g1_sensors tf_chain`
to publish the live `/battery_state`. The gateway battery gate requires a
standard `sensor_msgs/msg/BatteryState` message with **fractional** percentage
at least `0.20` (20%), refreshed within 1 s. The verified G1 bridge converts
`/lf/bmsstate.soc` 0..100 to that fraction (live check: SOC 97 -> 0.97 at
20 Hz, 2026-09-26). The SDK client independently checks the raw BMS at 20 Hz;
without either stream, motion commands remain blocked. A manual vendor-remote
survey and successful `check_rtabmap_plan` dry run are prerequisites for a
Nav2 goal on the walking robot.

## Stage 4 — one tiny supervised movement (not yet run on hardware)

The first time the enabled Loco path moves the robot: **one** 0.05 m/s forward command for about
0.25 s, then zero (a few centimetres). It is not an explore_lite or Nav2 test. Do not start rl_sar
and do not publish `LowCmd`.

**Before starting**

- Robot: feet on flat ground, supported by the harness. Two people present; one holds the paired
  Unitree remote, ready for its emergency damping combination. It depends on the firmware (L2+B on
  V1.0.4; V1.0.2 differs): check the robot's firmware version first. Damping lets the robot
  settle into the harness; keep people clear of the joints.
- Stage 3 passed (gateway + SDK client dry run on the isolated domain 77). Check the battery again
  right before the test: it must be above 20 %.
- Terminals 1, 3, 4 share the robot DDS environment. Terminal 2 needs none (Unitree SDK process):

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/warsaw/g1_ws/install/setup.bash      # unitree_hg messages must be sourced too
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="<WIRED_INTERFACE>" priority="default" multicast="default"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
```

**Order:** Terminal 1 → Terminal 4 preflight → Terminal 2 → Terminal 3 → Terminal 4 graph check →
the one command.

Terminal 1, live battery bridge (leave running). Do not also launch `tf_chain.launch.py`: it starts
a second `/battery_state` publisher (or launch it with `publish_battery_state:=false`).

```bash
ros2 run g1_sensors bms_to_battery_state --ros-args \
  -r bms:=/lf/bmsstate -r battery_state:=/battery_state
```

Terminal 4, preflight. Proceed only if the SOC is above 20, the ROS fraction agrees (58 → 0.58),
the SDK probe succeeds, and `/cmd_vel` has no publisher or subscriber (`Unknown topic '/cmd_vel'`
is expected here):

```bash
ros2 topic echo /lf/bmsstate --once --no-daemon --field soc
ros2 topic echo /battery_state --once --no-daemon --field percentage
ros2 run g1_loco_cmdvel g1_bms_probe <WIRED_INTERFACE>
ros2 topic info /cmd_vel -v --no-daemon
```

Terminal 2, high-level Unitree client (leave running; start only after the preflight and when the
remote operator is ready). It must print `actuation=ENABLED`; do not send a command if it exits or
reports a BMS / SDK error:

```bash
env -u LD_LIBRARY_PATH -u CYCLONEDDS_URI \
  ~/workspace/warsaw/g1_ws/install/g1_loco_cmdvel/lib/g1_loco_cmdvel/g1_loco_client \
  --network-interface=<WIRED_INTERFACE> --enabled=true --i-accept-high-level-actuation=true
```

Terminal 3, ROS velocity gateway (leave running), with tighter limits than the defaults:

```bash
ros2 run g1_loco_cmdvel cmd_vel_gateway --ros-args \
  -p enabled:=true -p require_battery:=true -p battery_topic:=/battery_state \
  -p min_battery_percent:=0.20 -p battery_timeout_sec:=1.0 -p command_timeout_sec:=0.30 \
  -p max_vx:=0.05 -p max_vy:=0.05 -p max_wz:=0.10
```

Terminal 4, final graph check: zero publishers and exactly ONE subscriber
(`g1_loco_cmdvel_gateway`); stop if it differs. Then, only when the remote operator says "ready",
run the command **once**. It refuses a stale or low battery and unexpected `/cmd_vel` endpoints,
and always ends with zeros:

```bash
ros2 topic info /cmd_vel -v --no-daemon
ros2 run g1_loco_cmdvel stage4_single_command.py
```

Offline dry run of that script (domain 77, SDK client in dry-run, 2026-09-26): 5 × `vx=0.05` then
6 zeros reached the client; a stopped battery bridge, a battery at 15 %, or an extra `/cmd_vel`
subscriber sent no motion packet.

**Stop and clean up.** Watch the robot and Terminal 2: it should log the small command, then zero.
Normal stop: Ctrl-C in Terminal 3, then 2, then 1, then in Terminal 4:

```bash
bash ~/workspace/warsaw/scripts/stop_ros.sh 3
ros2 topic info /cmd_vel -v --no-daemon      # no /cmd_vel publisher or subscriber may remain
```

If the behavior is unexpected, the remote operator uses the emergency damping combination
**immediately**. Ctrl-C is not an e-stop. Do not retry after a fall, collision, abnormal sound,
error, or failed stop: preserve the logs and follow the incident procedure (AGENTS.md §25.4).

Notes:

- The battery and command checks use receive times on the laptop, so the test does not depend on
  the laptop clock sync.
- `g1_loco_client` writes with `std::cout` without flushing. On a terminal that is line-buffered;
  when redirecting to a log file, start it with `stdbuf -oL -eL`, or the log stays empty until exit.
