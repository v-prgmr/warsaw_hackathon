#!/bin/bash
# Build (if needed) and start the host-side ROS 2 Humble container.
# Usage:
#   ROBOT_IFACE=enp2s0 scripts/run_humble.sh [command...]   # robot mode: DDS bound to the G1 NIC
#   SIM=1 scripts/run_humble.sh [command...]                # sim mode: isolated DDS domain, no robot
# The container is removed on exit (--rm): one live session = one container. End a session with
# `bash /ws/scripts/stop_ros.sh` inside, or scripts/stop_humble.sh on the host (AGENTS.md §19).
set -e
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=g1-humble
ROBOT_IFACE="${ROBOT_IFACE:-enp2s0}"
SIM="${SIM:-0}"

docker build -t "$IMAGE" "$REPO_DIR"

if [ "$SIM" != "1" ] && ! ip -br addr show "$ROBOT_IFACE" 2>/dev/null | grep -q "192.168.123."; then
  echo "WARNING: $ROBOT_IFACE has no 192.168.123.x address; DDS will not see the robot." >&2
fi

xhost +local: >/dev/null 2>&1 || true

# GPU for Gazebo / RViz rendering (Mesa via /dev/dri).
GPU_ARGS=()
if [ -d /dev/dri ]; then
  GPU_ARGS+=(--device /dev/dri)
  for g in video render; do
    gid="$(getent group "$g" | cut -d: -f3)"
    [ -n "$gid" ] && GPU_ARGS+=(--group-add "$gid")
  done
fi

docker run -it --rm \
  --net=host --ipc=host \
  --user "$(id -u):$(id -g)" \
  "${GPU_ARGS[@]}" \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -e HOME=/tmp/home \
  -e ROBOT_IFACE="$ROBOT_IFACE" \
  -e G1_SIM="$SIM" \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$REPO_DIR":/ws \
  "$IMAGE" "${@:-bash}"
