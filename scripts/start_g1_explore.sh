#!/usr/bin/env bash
# Plug-in: add the m-explore frontier explorer (explore_lite, AGENTS.md §15) to an already running
# scripts/start_g1_navigation.sh. It opens one window, "G1 8", next to the running Nav2 (container
# g1-nav, the session container g1-robot, or host ROS), which waits for Enter:
#   dry-run Nav2: preflight check_rtabmap_plan, then frontier goals that Nav2 only plans
#   live Nav2:    no preflight (it refuses a live /cmd_vel); asks to type EXPLORE first, then the
#                 G1 WALKS to every frontier. Safety zone ready, remote e-stop in hand.
# Setup once: bash scripts/setup_m_explore.sh. All in one instead: start_g1_explore_navigation.sh.
set -euo pipefail

ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
ORIN=192.168.123.164
HERE=false
STOP=false
MIN_FRONTIER=

usage() {
  cat <<'HELP'
Usage: bash scripts/start_g1_explore.sh [options]      (start_g1_navigation.sh must be running)

Adds the frontier explorer to the running navigation, in a new window "G1 8".
  --here               run the explorer in this terminal instead of a new window
  --min-frontier M     smallest frontier to explore, metres (default 0.3, explore_g1_rtabmap.yaml)
  --stop               stop only the explorer (navigation keeps running)
Pause / resume: ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}' / true
Stop everything: bash scripts/stop_g1_navigation.sh
HELP
}

while (($#)); do
  case "$1" in
    --here) HERE=true; shift ;;
    --stop) STOP=true; shift ;;
    --min-frontier) [[ $# -ge 2 ]] || { usage; exit 2; }; MIN_FRONTIER=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
  esac
done
[[ -z $MIN_FRONTIER || $MIN_FRONTIER =~ ^[0-9]*\.?[0-9]+$ ]] || { echo "--min-frontier: metres, e.g. 0.3" >&2; exit 2; }

die() { printf '\033[31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

# ---- where does the navigation run? -----------------------------------------------------------
NAV_LAUNCH='^[^ ]*python3 [^ ]*/ros2 launch g1_nav2 rtabmap_nav_(dry_run|live)'   # the launch itself
running() { docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$1"; }
TARGET=
for c in g1-nav g1-robot; do
  if running "$c" && docker exec "$c" pgrep -f "$NAV_LAUNCH" >/dev/null; then TARGET=$c; break; fi
done
if [[ -n $TARGET ]]; then
  where="container $TARGET"
  ros_sh() { docker exec -i "$TARGET" bash -c "source /ros_entrypoint.sh; source /ws/rl_hnav/install/setup.bash; $1"; }
  in_target() { docker exec "$TARGET" bash -c "$1"; }
elif [[ -r /opt/ros/humble/setup.bash ]] && pgrep -f "$NAV_LAUNCH" >/dev/null; then
  where="host ROS"
  NIC=$(ip -o route get "$ORIN" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "dev") {print $(i + 1); exit}}')
  export G1_NAV_ROOT=$ROOT G1_NAV_NIC=${NIC:-enp2s0}
  HOST_ENV='source /opt/ros/humble/setup.bash; source "$G1_NAV_ROOT/g1_ws/install/setup.bash";
    source "$G1_NAV_ROOT/rl_hnav/install/setup.bash";
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0;
    export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${G1_NAV_NIC}\" priority=\"default\" multicast=\"default\"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>"'
  ros_sh() { bash -c "$HOST_ENV; $1"; }
  in_target() { bash -c "$1"; }
else
  die "no running navigation found (Nav2 from start_g1_navigation.sh). Start it first:
    bash scripts/start_g1_navigation.sh          (or everything at once: start_g1_explore_navigation.sh)"
fi

# The explorer's own processes, anchored on argv[0] so wrapper shells are never matched.
EXPLORER='^[^ ]*python3 [^ ]*/ros2 (launch g1_nav2 rtabmap_explore|run explore_lite )|^[^ ]*/explore_lite/explore( |$)'

if $STOP; then
  pids=$(in_target "pgrep -f '$EXPLORER'" || true)
  [[ -n $pids ]] || { echo "no explorer running ($where)"; exit 0; }
  echo "stopping the explorer ($where): $(echo $pids)"
  in_target "pkill -INT -f '$EXPLORER'; for _ in \$(seq 100); do pgrep -f '$EXPLORER' >/dev/null || exit 0; sleep 0.1; done; pkill -TERM -f '$EXPLORER'" \
    && echo "explorer stopped; navigation keeps running (the current Nav2 goal: RViz or the remote)"
  exit 0
fi

in_target "pgrep -f '$EXPLORER'" >/dev/null \
  && die "an explorer already runs ($where). Stop it: bash scripts/start_g1_explore.sh --stop"
ros_sh 'ros2 pkg prefix explore_lite >/dev/null' \
  || die "explore_lite (m-explore) is not built. Once, with internet: bash scripts/setup_m_explore.sh$([[ $where == 'host ROS' ]] && echo ' --host')"

LIVE=false
in_target "pgrep -f '^[^ ]*python3 [^ ]*/ros2 launch g1_nav2 rtabmap_nav_live'" >/dev/null && LIVE=true
echo "Navigation: $where, Nav2 $($LIVE && echo LIVE || echo 'dry run')"

ARGS=""
[[ -n $MIN_FRONTIER ]] && ARGS=" min_frontier_size:=$MIN_FRONTIER"
if $LIVE; then
  cat <<EOF
LIVE: the explorer sends the G1 to every frontier it finds, on its own (AGENTS.md §15 / §25):
  - the dry run of the explorer on this map looked sane (frontiers inside the zone)
  - the area is bounded: doors, stairs, corridors and people out of reach
  - one person holds the remote, ready for emergency damping; battery above 20 %
EOF
  [[ -t 0 ]] || die "live exploration needs a terminal to confirm"
  read -r -p 'Type EXPLORE to add the explorer: ' answer
  [[ $answer == EXPLORE ]] || die "not confirmed; nothing started"
  ARGS="preflight:=false$ARGS"
  MODE='LIVE: the G1 WALKS to every frontier it finds. Safety zone ready, remote e-stop in hand.'
else
  ARGS="preflight:=true$ARGS"
  MODE='Dry run: Nav2 plans and shows the velocities it would send (G1 5); nothing moves.'
fi
CMD="cat <<EOF
Frontier exploration (explore_lite on the RTAB-Map /map; goals -> Nav2 NavigateToPose).
  $MODE
  Survey with the remote first: on a standing-only map there are no frontiers.
  Pause:  ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'   (resume: true)
  Stop:   Ctrl-C here, or bash scripts/start_g1_explore.sh --stop
  RViz:   add a MarkerArray display on /explore/frontiers
EOF
read -r -p 'Press Enter to start exploring (Ctrl-C to skip): ' _ && ros2 launch g1_nav2 rtabmap_explore.launch.py $ARGS"

WRAPPER='printf "%s\n\n" "$1"; trap : INT; eval "$1"; code=$?
  printf "\nExplorer finished (status %s). No automatic restart. This shell stays open.\n" "$code"; exec bash -i'
TITLE="G1 8 - frontier explorer"; $LIVE && TITLE+=" (LIVE: the G1 walks)"
if [[ -n $TARGET ]]; then
  RUN=(docker exec -it "$TARGET" bash -c "source /ros_entrypoint.sh; source /ws/rl_hnav/install/setup.bash; $WRAPPER" bash "$CMD")
else
  RUN=(bash -c "$HOST_ENV; cd \"\$G1_NAV_ROOT\"; $WRAPPER" bash "$CMD")
fi
if $HERE; then
  "${RUN[@]}"
else
  command -v gnome-terminal >/dev/null || die "gnome-terminal is required (or use --here)"
  gnome-terminal --window --title="$TITLE" -- "${RUN[@]}"
  echo "Opened \"$TITLE\": press Enter there to start. Stop only the explorer: bash scripts/start_g1_explore.sh --stop"
fi
