#!/usr/bin/env bash
# Companion to start_g1_navigation.sh and start_g1_explore_navigation.sh. Order (AGENTS.md §19):
#   1. the Loco executor first: SIGINT to g1_loco_client (it calls StopMove) and cmd_vel_gateway
#   2. then the frontier explorer, Nav2, the /scan bridge, the velocity monitor and the navigation
#      RViz (SIGINT, TERM, KILL)
#   3. its own container g1-nav: everything else there (TF, map: RTAB-Map saves its database), then
#      the container. Joined to start_g1_session (g1-robot): only the navigation, the session stays.
# Never kill -9 a launch first: its nodes are orphaned and keep publishing onto the robot's network.
set -uo pipefail
ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
CONTAINER=g1-nav
SESSION_CONTAINER=g1-robot

# Anchored on the process's own argv[0], so the windows' wrapper shells (whose arguments contain
# the same command text) and this script are never matched.
EXECUTOR='^[^ ]*/g1_loco_client( |$)|^[^ ]*/cmd_vel_gateway( |$)|^[^ ]*python3 [^ ]*/ros2 run g1_loco_cmdvel '
NAV="$EXECUTOR"'|^[^ ]*python3 [^ ]*/ros2 (launch (g1_nav2|humanoid_nav_bridge) |run (g1_nav2|explore_lite) |topic echo [^ ]*cmd_vel)|^[^ ]*/explore_lite/explore( |$)'
# The navigation RViz (unified session + navigation, or the old minimal one). Joined to a
# session it stays open: it is the session's only RViz (start_g1_navigation.sh closed the other).
RVIZ='^[^ ]*rviz2 -d [^ ]*(g1_nav_minimal|g1_session_nav)\.rviz'

pids_with_children() {  # pattern -> matching PIDs and all their descendants
  local all frontier kids
  all=$(pgrep -f -- "$1")
  frontier=$all
  while [[ -n $frontier ]]; do
    kids=$(for p in $frontier; do pgrep -P "$p"; done)
    all="$all $kids"
    frontier=$kids
  done
  echo $all | tr ' ' '\n' | grep -v "^$$\$" | sort -un
}

stop_matching() {  # pattern, label, seconds to wait after SIGINT
  local pids
  pids=$(pids_with_children "$1")
  [[ -n $pids ]] || { echo "$2: none running"; return 0; }
  echo "$2: SIGINT"; ps -o pid=,args= -p "$(echo $pids | tr ' ' ',')" | cut -c1-150
  kill -INT $pids 2>/dev/null
  for sig in TERM KILL; do
    for _ in $(seq $(($3 * 10))); do
      pids=$(for p in $pids; do kill -0 "$p" 2>/dev/null && echo "$p"; done)
      [[ -z $pids ]] && { echo "$2: stopped"; return 0; }
      sleep 0.1
    done
    echo "$2: still running, SIG$sig"; kill -"$sig" $pids 2>/dev/null
    set -- "$1" "$2" 3
  done
  sleep 1
  for p in $pids; do kill -0 "$p" 2>/dev/null && { echo "$2: could not stop $p"; return 1; }; done
  return 0
}

inside() {  # runs where the nodes are (container or host)
  local status=0
  stop_matching "$EXECUTOR" "Loco executor (StopMove)" 5 || status=1
  stop_matching "$NAV" "explorer, Nav2, /scan, monitor" 20 || status=1
  if [[ ${1:-} == all ]]; then
    stop_matching "$RVIZ" "navigation RViz" 5 || status=1
    bash "$(dirname "$0")/stop_ros.sh" || status=1      # TF, map and anything else left
  elif pgrep -f -- "$RVIZ" >/dev/null; then
    echo "navigation RViz: left open, it is the session's RViz now (closes with stop_g1_session.sh)"
  fi
  return $status
}

case "${1:-}" in
  -h|--help)
    echo "Usage: bash scripts/stop_g1_navigation.sh   (stops what start_g1_navigation.sh started)"
    echo "Windows stay open for their logs. The remote's e-stop is the emergency stop, not this."
    exit 0 ;;
  --inside) inside "${2:-}"; exit $? ;;
  "") ;;
  *) echo "Unexpected argument. Use --help." >&2; exit 2 ;;
esac

running() { docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"; }
status=0
did=false
if running "$CONTAINER"; then
  echo "== container $CONTAINER"
  docker exec "$CONTAINER" bash /ws/scripts/stop_g1_navigation.sh --inside all || status=1
  docker stop "$CONTAINER" >/dev/null && echo "stopped container $CONTAINER"
  did=true
fi
if running "$SESSION_CONTAINER"; then
  echo "== navigation inside $SESSION_CONTAINER (the session keeps running)"
  docker exec "$SESSION_CONTAINER" bash /ws/scripts/stop_g1_navigation.sh --inside || status=1
  did=true
fi
# Host ROS only (e.g. a laptop with ROS Humble installed): the host also sees container processes.
if [[ -r /opt/ros/humble/setup.bash ]] && { ! $did || pgrep -f -- "$NAV" >/dev/null; }; then
  echo "== host"
  if pgrep -f -- "$NAV" >/dev/null || pgrep -f -- '--ros-args' >/dev/null; then
    inside all || status=1
  else
    echo "nothing running on the host"
  fi
fi
((status == 0)) && echo "G1 navigation stopped. The windows stay open for their logs." \
  || { echo "Something did not stop: see above." >&2; exit 1; }
