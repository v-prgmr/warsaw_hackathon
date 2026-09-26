#!/bin/bash
# Stop every ROS 2 process in this container cleanly (AGENTS.md §19). Usage: bash scripts/stop_ros.sh [timeout_s]
#   1. SIGINT to every ROS process (`ros2 launch|run|bag` and all nodes), like Ctrl-C in a terminal,
#      then wait: nodes shut down cleanly (RTAB-Map saves its database) and launch exits after them.
#      Signalling only the launch is not enough: a launch started in the background (`&`) from a
#      script inherits SIGINT as *ignored* and never shuts down (seen live, 2026-09-26).
#   2. Only then SIGTERM, then SIGKILL, to whatever is left (orphaned nodes included).
# Never SIGKILL a `ros2 launch` first: its nodes are orphaned and keep publishing onto the robot's
# network. Run this as a script: a `pkill -f <pattern>` typed into `bash -c` also matches that shell.
set -u
TIMEOUT="${1:-20}"
ROS='--ros-args|/ros2 (launch|run|bag) '   # launched nodes always carry --ros-args

wait_gone() {  # pattern, seconds
  for _ in $(seq $(($2 * 10))); do pgrep -f -- "$1" >/dev/null || return 0; sleep 0.1; done
  return 1
}

if ! pgrep -f -- "$ROS" >/dev/null; then echo "no ROS processes running"; exit 0; fi
pkill -INT -f -- "$ROS"
if wait_gone "$ROS" "$TIMEOUT"; then echo "all ROS processes exited cleanly"; exit 0; fi
echo "still running ${TIMEOUT} s after SIGINT, escalating:"; pgrep -af -- "$ROS"
pkill -TERM -f -- "$ROS"
wait_gone "$ROS" 5 || { pkill -KILL -f -- "$ROS"; sleep 1; }
if pgrep -f -- "$ROS" >/dev/null; then echo "could not stop:"; pgrep -af -- "$ROS"; exit 1; fi
echo "stopped (escalated)"
