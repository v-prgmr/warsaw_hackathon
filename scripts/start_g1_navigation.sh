#!/usr/bin/env bash
# Direct RViz goals on RTAB-Map. No m-explore or RL locomotion process.
set -euo pipefail

ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
NIC=enx3c33327bdb58
LIVE=false
PRINT_ONLY=false

usage() {
  cat <<'HELP'
Usage: bash scripts/start_g1_navigation.sh [--live] [--nic INTERFACE] [--print-only]

Opens GNOME Terminal windows for the real G1's sensing and direct RViz navigation.
  --live        Enable the high-level Loco client and gateway; Nav2 outputs /cmd_vel.
                Requires supervised motion prerequisites from AGENTS.md sections 19/25.
  --nic NAME    Wired robot interface (default: enx3c33327bdb58).
  --print-only  Print commands without opening terminals or starting any process.

Without --live, commands stay on /g1_nav2_dry_run/cmd_vel and no executor starts.
Start once with existing navigation/TF/gateway launches stopped to avoid duplicates.
Each run uses a new timestamped RTAB-Map database under ~/.ros/.
Nav2/scan packages are installed in rl_hnav; no RL policy or simulator is launched.
Use RViz's Nav2 Goal tool only once the map, TF and costmaps are ready.
Stop the session with scripts/stop_ros.sh in the same host/container as the nodes.
HELP
}

while (($#)); do
  case "$1" in
    --live) LIVE=true; shift ;;
    --print-only) PRINT_ONLY=true; shift ;;
    --nic) [[ $# -ge 2 ]] || { usage; exit 2; }; NIC=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
  esac
done
[[ $NIC =~ ^[a-zA-Z0-9_.:-]+$ && $NIC != lo ]] || {
  printf 'Supply a wired robot interface, not loopback.\n' >&2; exit 2;
}

for setup in /opt/ros/humble/setup.bash "$ROOT/g1_ws/install/setup.bash" \
             "$ROOT/rl_hnav/install/setup.bash"; do
  [[ -r $setup ]] || { printf 'Missing setup: %s\n' "$setup" >&2; exit 1; }
done
if ! $PRINT_ONLY; then
  command -v gnome-terminal >/dev/null || { printf 'gnome-terminal is required.\n' >&2; exit 1; }
  ip link show "$NIC" >/dev/null
fi

export G1_BRINGUP_ROOT="$ROOT" G1_BRINGUP_NIC="$NIC"
export G1_BRINGUP_DB="$HOME/.ros/g1_rviz_$(date +%Y%m%d_%H%M%S)_$$.db"
export G1_BRINGUP_URDF="$ROOT/g1_ws/install/g1_sensors/share/g1_sensors/urdf/g1_29dof_rev_1_0.urdf"
export G1_BRINGUP_MESH_DIR="$ROOT/third_party/unitree_ros/robots/g1_description/meshes"
[[ -r $G1_BRINGUP_URDF && -d $G1_BRINGUP_MESH_DIR ]] || {
  printf 'Missing G1 URDF or mesh directory:\n%s\n%s\n' "$G1_BRINGUP_URDF" "$G1_BRINGUP_MESH_DIR" >&2
  exit 1
}

open_terminal() {
  local title=$1 command=$2
  if $PRINT_ONLY; then
    printf '\n[%s]\n%s\n' "$title" "$command"
    return
  fi
  gnome-terminal --window --title="$title" -- bash -c '
    # Source ROS overlays without nounset: setup scripts reference optional variables.
    source /opt/ros/humble/setup.bash || exit
    source "$G1_BRINGUP_ROOT/g1_ws/install/setup.bash" || exit
    source "$G1_BRINGUP_ROOT/rl_hnav/install/setup.bash" || exit
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
    export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${G1_BRINGUP_NIC}\" priority=\"default\" multicast=\"default\"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>"
    cd "$G1_BRINGUP_ROOT" || exit
    printf "%s\n\n" "$1"
    # Run in the foreground so Ctrl-C reaches the command. Keep output visible afterward.
    trap ":" INT
    eval "$1"
    code=$?
    printf "\nProcess finished (status %s). No automatic restart.\n" "$code"
    exec bash -i
  ' bash "$command"
}

open_terminal 'G1 1 - TF and battery' 'ros2 launch g1_sensors tf_chain.launch.py urdf:="$G1_BRINGUP_URDF" mesh_dir:="$G1_BRINGUP_MESH_DIR"'
open_terminal 'G1 2 - RTAB-Map' 'ros2 launch g1_mapping mapping.launch.py static_tf:=false database_path:="$G1_BRINGUP_DB"'
open_terminal 'G1 3 - Scan bridge' 'ros2 launch humanoid_nav_bridge real_robot_bridge.launch.py publish_odom_tf:=false publish_lidar_tf:=false override_scan_stamp:=false'

if $LIVE; then
  open_terminal 'G1 4 - LIVE Nav2' 'ros2 launch g1_nav2 rtabmap_nav_live.launch.py max_vx:=0.50 max_wz:=1.0'
  open_terminal 'G1 5 - LIVE Loco SDK' 'env -u LD_LIBRARY_PATH -u CYCLONEDDS_URI stdbuf -oL -eL "$G1_BRINGUP_ROOT/g1_ws/install/g1_loco_cmdvel/lib/g1_loco_cmdvel/g1_loco_client" --network-interface="$G1_BRINGUP_NIC" --enabled=true --i-accept-high-level-actuation=true'
  open_terminal 'G1 6 - LIVE gateway' 'ros2 run g1_loco_cmdvel cmd_vel_gateway --ros-args -p enabled:=true -p require_battery:=true -p battery_topic:=/battery_state -p min_battery_percent:=0.20 -p battery_timeout_sec:=1.0 -p command_timeout_sec:=0.30 -p max_vx:=0.50 -p max_vy:=0.50 -p max_wz:=1.0'
  open_terminal 'G1 - Velocity monitor' 'ros2 topic echo /cmd_vel'
else
  open_terminal 'G1 4 - Command-only Nav2' 'ros2 launch g1_nav2 rtabmap_nav_dry_run.launch.py max_vx:=0.50 max_wz:=1.0'
  open_terminal 'G1 - Velocity monitor' 'ros2 topic echo /g1_nav2_dry_run/cmd_vel'
fi
open_terminal 'G1 - RViz goals' 'rviz2 -d "$(ros2 pkg prefix g1_nav2)/share/g1_nav2/rviz/g1_nav_minimal.rviz"'
printf '\nLive actuation: %s\nMap database: %s\n' "$LIVE" "$G1_BRINGUP_DB"
