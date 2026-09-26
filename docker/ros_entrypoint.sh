#!/bin/bash
# Sourced from /etc/bash.bashrc for every shell, and used as the container entrypoint.
source /opt/ros/humble/setup.bash
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
[ -f /ws/g1_ws/install/setup.bash ] && source /ws/g1_ws/install/setup.bash
mkdir -p "$HOME"

# Simulation mode: keep the sim graph off the robot's DDS graph. A sim Nav2 publishes /cmd_vel,
# which must never reach a real G1 on the same network (AGENTS.md §19).
if [ "${G1_SIM:-0}" = "1" ]; then
  unset CYCLONEDDS_URI
  export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
fi

# Only exec when run as the entrypoint, not when sourced.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  exec "$@"
fi
