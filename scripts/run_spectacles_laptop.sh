#!/usr/bin/env bash
# ROS Humble on this Ubuntu laptop, isolated from the robot DDS graph.
set -euo pipefail

repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
image_name="g1-spectacles-laptop"
docker_config="${TMPDIR:-/tmp}/g1-spectacles-docker-config"
mkdir -p "$docker_config"

if ! docker --config "$docker_config" image inspect g1-humble:latest >/dev/null 2>&1; then
  echo "Missing local g1-humble:latest image; build the repository's base Dockerfile first." >&2
  exit 1
fi

docker --config "$docker_config" build --network=host -q \
  -f "$repo_dir/docker/Dockerfile.spectacles_laptop" \
  -t "$image_name" "$repo_dir" >/dev/null

terminal_args=()
if [ -t 0 ]; then
  terminal_args=(-it)
fi

docker --config "$docker_config" run --rm "${terminal_args[@]}" \
  --net=host --ipc=host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp/home \
  -e G1_SIM=1 \
  -e ROS_DOMAIN_ID=77 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e SPECTACLES_BRIDGE_TOKEN="${SPECTACLES_BRIDGE_TOKEN:-}" \
  -v "$repo_dir":/ws \
  -w /ws \
  "$image_name" "${@:-bash}"
