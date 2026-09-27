#!/usr/bin/env bash
# Fetch, patch and build m-explore-ros2 (explore_lite) for the G1 (AGENTS.md §15). Idempotent.
#   1. clone robo-friends/m-explore-ros2 into rl_hnav/src/m-explore-ros2 (git-ignored) at the
#      commit pinned in rl_hnav/exploration.repos (needs internet the first time)
#   2. apply rl_hnav/patches/m-explore-ros2-success.patch (latched RTAB-Map /map, keep the
#      active goal, cancel + blacklist stalled goals), unless it is already applied
#   3. build explore_lite_msgs + explore_lite (and g1_nav2) in a throwaway g1-humble container
#      (isolated DDS domain; the robot is not touched), or with the host's ROS in --host mode
# Then: bash scripts/start_g1_explore_navigation.sh
set -euo pipefail

ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
HNAV=$ROOT/rl_hnav
SRC=$HNAV/src/m-explore-ros2
PATCH=$HNAV/patches/m-explore-ros2-success.patch
HOST=false
case "${1:-}" in
  --host) HOST=true ;;
  -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; echo "Usage: bash scripts/setup_m_explore.sh [--host]"; exit 0 ;;
  "") ;;
  *) echo "Unexpected argument. Use --help." >&2; exit 2 ;;
esac

URL=$(awk '/url:/{print $2; exit}' "$HNAV/exploration.repos")
REV=$(awk '/version:/{print $2; exit}' "$HNAV/exploration.repos")
[[ -n $URL && -n $REV ]] || { echo "cannot read url/version from rl_hnav/exploration.repos" >&2; exit 1; }

echo "1. m-explore-ros2 @ ${REV:0:7}"
if [[ ! -d $SRC/.git ]]; then
  git clone -q "$URL" "$SRC"
fi
if [[ $(git -C "$SRC" rev-parse HEAD) != "$REV"* ]]; then
  if ! git -C "$SRC" diff --quiet; then
    echo "$SRC has local changes on another commit; move them away first" >&2; exit 1
  fi
  git -C "$SRC" fetch -q origin
  git -C "$SRC" checkout -q "$REV"
fi
echo "   OK: $(git -C "$SRC" log -1 --format='%h %s')"

echo "2. patch $(basename "$PATCH")"
if git -C "$SRC" apply --reverse --check "$PATCH" 2>/dev/null; then
  echo "   already applied"
elif git -C "$SRC" apply --check "$PATCH" 2>/dev/null; then
  git -C "$SRC" apply "$PATCH" && echo "   applied"
else
  echo "   the patch neither applies nor is applied: $SRC has other changes" >&2
  echo "   reset it with: git -C $SRC checkout -- . && bash $0" >&2
  exit 1
fi

echo "3. build explore_lite_msgs explore_lite g1_nav2"
BUILD='cd rl_hnav && colcon build --packages-up-to explore_lite g1_nav2'
if $HOST; then
  bash -c "source /opt/ros/humble/setup.bash && cd '$ROOT' && $BUILD"
else
  docker image inspect g1-humble >/dev/null 2>&1 \
    || { echo "Docker image g1-humble missing: SIM=1 scripts/run_humble.sh (once, then exit)" >&2; exit 1; }
  docker run --rm --user "$(id -u):$(id -g)" -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
    -e HOME=/tmp/home -e G1_SIM=1 -v "$ROOT":/ws g1-humble bash -c "cd /ws && $BUILD"
fi
echo
echo "Done. Start it with: bash scripts/start_g1_explore_navigation.sh   (dry run first)"
