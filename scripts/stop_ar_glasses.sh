#!/usr/bin/env bash
# Companion to start_ar_glasses.sh: stop ROS cleanly in the AR container, then the container
# (AGENTS.md §19: SIGINT first, so RTAB-Map saves its database; never kill -9 a launch).
set -uo pipefail
CONTAINER=g1-ar
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  echo "Usage: bash scripts/stop_ar_glasses.sh   (the Orin's OAK-D driver: Ctrl-C in its window)"
  exit 0
fi
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "container $CONTAINER is not running"
else
  docker exec "$CONTAINER" bash /ws/scripts/stop_ros.sh
  docker stop "$CONTAINER" >/dev/null && echo "stopped container $CONTAINER"
fi
echo "The windows stay open for their logs; close them. On the Orin: Ctrl-C in the OAK-D window."
