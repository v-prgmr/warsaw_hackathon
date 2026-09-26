#!/bin/bash
# Host side: end every g1-humble container (AGENTS.md §19). Stops ROS cleanly inside each one
# first (stop_ros.sh, so RTAB-Map saves its database), then `docker stop`: when a container ends,
# every process in it ends, so nothing of ours keeps publishing onto the robot's network.
ids=$(docker ps -q --filter ancestor=g1-humble)
if [ -z "$ids" ]; then echo "no g1-humble containers running"; exit 0; fi
for id in $ids; do docker exec "$id" bash /ws/scripts/stop_ros.sh || true; done
docker stop $ids >/dev/null && echo "stopped containers: $(echo $ids)"
