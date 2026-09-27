#!/usr/bin/env bash
# Companion to start_ar_glasses.sh: stop ROS cleanly in the AR containers (g1-search, g1-ar), then them
# (AGENTS.md §19: SIGINT first, so RTAB-Map saves its database; never kill -9 a launch).
set -uo pipefail
CONTAINER=g1-ar
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  echo "Usage: bash scripts/stop_ar_glasses.sh   (the Orin's OAK-D driver: Ctrl-C in its window)"
  exit 0
fi
for c in g1-search "$CONTAINER"; do       # object search first, then the robot stack
  if docker ps --format '{{.Names}}' | grep -qx "$c"; then
    docker exec "$c" bash /ws/scripts/stop_ros.sh
    docker stop "$c" >/dev/null && echo "stopped container $c"
  elif [[ $c == "$CONTAINER" ]]; then
    echo "container $c is not running"
  fi
done
echo "The windows stay open for their logs; close them. On the Orin: Ctrl-C in the OAK-D window."
