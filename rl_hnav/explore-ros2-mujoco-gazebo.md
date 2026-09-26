# ROS 2 frontier exploration with MuJoCo and Gazebo

This is the **simulation-only** G1 workflow. MuJoCo runs the walking policy;
Gazebo supplies the room and sensors; SLAM Toolbox builds `/map`; Nav2 drives
`/cmd_vel`; `explore_lite` chooses frontier goals. Do not use this launch on the
physical G1.

## One-time setup

Run from the `rl_hnav` repository root. The exploration source is pinned by
`exploration.repos`. If it is not already present, import it and apply the local
fix once:

```bash
cd /home/vrazer/workspace/warsaw/rl_hnav
vcs import src < exploration.repos
git -C src/m-explore-ros2 apply ../../patches/m-explore-ros2-success.patch
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to explore_lite g1_nav2
```

On a prepared checkout, skip the import and patch; just build if sources have
changed. The patch must not be applied twice. Source `install/setup.bash` in
**each** terminal after building.

## Start the three processes

Open three separate terminals. Each command uses `sim_ros2`, which selects the
same private localhost ROS domain (76 by default). If changing
`G1_SIM_DOMAIN_ID`, set the same value in all terminals before launching.

**Terminal 1 — MuJoCo locomotion:**

```bash
cd /home/vrazer/workspace/warsaw/rl_hnav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_nav2 sim_ros2 run rl_sar rl_mujoco g1 scene_29dof \
  --ros-args -p navigation_mode:=true \
  -p cmd_vel_timeout_sec:=0.6 -p cmd_vel_topic:=/cmd_vel
```

In the MuJoCo viewer, pause if necessary, press `0` to load the policy, press
`1` for GetUp, then start/play the simulation. Confirm the robot is standing
before enabling exploration.

**Terminal 2 — Gazebo room, SLAM, Nav2 and RViz:**

```bash
cd /home/vrazer/workspace/warsaw/rl_hnav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_nav2 sim_ros2 launch g1_nav2 nav_amcl.launch.py \
  world_name:=square_loop.world use_rviz:=true
```

Launch this stack **once**; a second copy creates competing `/map`, TF and
Nav2 publishers. Wait until the map and Nav2 are ready.

**Terminal 3 — autonomous frontier exploration:**

```bash
cd /home/vrazer/workspace/warsaw/rl_hnav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run g1_nav2 sim_ros2 launch g1_nav2 g1_exploration_sim.launch.py
```

The explorer first runs a read-only preflight of Nav2, `/map`, `/odom`, TF and
the controller footprint. Only after it prints `Nav2 preflight passed` does
it start sending `NavigateToPose` goals. RViz can display `/explore/frontiers`
as a MarkerArray.

## Inspect and stop

In a fourth terminal, source the same ROS/workspace setup and use the wrapper
for each diagnostic command:

```bash
ros2 run g1_nav2 sim_ros2 topic echo /explore/status
ros2 run g1_nav2 sim_ros2 topic echo /navigate_to_pose/_action/status
ros2 run g1_nav2 sim_ros2 topic echo /map --once --no-daemon --field info
ros2 run g1_nav2 sim_ros2 topic hz /scan
```

If the explorer reports `No frontiers found, stopping`, startup and Nav2
connection succeeded, but it found no qualifying boundary between known free
space and unknown space. Inspect `/map` in RViz and verify `/scan` is updating.
The configured minimum frontier length is 0.75 m in
`src/g1_nav2/params/explore_g1_sim.yaml`. After addressing the map or sensor
issue, restart Terminal 3; a completed explorer does not resume itself.

The local `m-explore-ros2-success.patch` keeps the current goal until Nav2
returns a result, rather than preempting it every time a SLAM update moves a
frontier centroid. It cancels and blacklists a goal only after 60 seconds
without 5 cm of progress toward it. Nav2's own progress checker permits
30 seconds without 20 cm of movement before starting recovery. After rebuilding
these changes, stop the old explorer and Nav2 launches and start new processes;
running nodes do not pick up a rebuilt binary or updated YAML automatically.

To pause exploration and cancel its goals before stopping Terminal 3:

```bash
ros2 run g1_nav2 sim_ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
```

Use Ctrl-C in the three launch terminals to shut down. If an old launch left
orphaned ROS processes, inspect running processes before restarting; do not
launch a second Nav2 stack alongside the old one.
