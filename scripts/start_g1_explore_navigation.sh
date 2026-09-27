#!/usr/bin/env bash
# start_g1_navigation.sh + m-explore: Nav2 on the RTAB-Map map with the frontier explorer
# (explore_lite, AGENTS.md §15) in its own window; RViz goals work too. No RL locomotion.
# Setup once: bash scripts/setup_m_explore.sh. The explorer waits for Enter in its window.
# Dry run by default: Nav2's velocities go to /g1_nav2_dry_run/cmd_vel and nothing moves.
# --live adds the high-level Loco executor (cmd_vel_gateway + g1_loco_client): the robot WALKS
# to every frontier. Safety zone (doors, stairs, people out of reach), remote e-stop in hand.
# Gates before --live: g1_ws/NAV2_FIRST_STEP.md (G0-G3), g1_loco_cmdvel Stage 4, AGENTS.md §19/§25.
set -euo pipefail

ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
NIC=
LIVE=false
PRINT_ONLY=false
RVIZ=true
RUNNER=auto                   # auto | docker | host
MAX_VX=0.30                   # m/s   (team-approved, 2026-09-26; g1_loco_client refuses > 0.50)
MAX_WZ=1.0                    # rad/s (team-approved, 2026-09-26; g1_loco_client refuses > 1.0)
CONTAINER=g1-nav              # our own robot container (docker mode)
SESSION_CONTAINER=g1-robot    # start_g1_session.sh's container: already runs TF + map
ORIN=192.168.123.164

usage() {
  cat <<'HELP'
Usage: bash scripts/start_g1_explore_navigation.sh [options]

Opens one window each for the G1's frontier exploration on the RTAB-Map map:
  G1 1  robot TF (g1_sensors tf_chain)        G1 4  Nav2 (dry run, or --live)
  G1 2  map (g1_mapping, RTAB-Map)            G1 5  velocity monitor
  G1 3  /scan (humanoid_nav_bridge)           G1 6  RViz with the "Nav2 Goal" tool
  G1 7  checks shell (check_rtabmap_plan)
  G1 8  frontier explorer (explore_lite; waits for Enter, then sends goals to Nav2)
  --live: also  G1 L1  Loco SDK client   G1 L2  cmd_vel_gateway   -> the robot WALKS

If start_g1_session.sh is running (container g1-robot), TF and map come from it: only
windows 3-8 open, inside that container. Otherwise its own container g1-nav runs everything.

  --live         enable the Loco executor; Nav2 publishes /cmd_vel. Asks you to type LIVE.
  --max-vx V     forward (and sideways) speed limit for Nav2 and the gateway
                 (default 0.30 m/s, max 0.50)
  --max-wz W     turn rate limit for Nav2 and the gateway (default 1.0 rad/s, max 1.0)
                 first live goal: --max-vx 0.05 --max-wz 0.10 (the Stage 4 limits)
  --no-rviz      do not open RViz
  --nic NAME     wired interface to the G1 (default: the one that routes to the Orin)
  --docker       run in Docker (default when there is no ROS on the host, or with g1-robot)
  --host         run with the host's ROS Humble and host-built g1_ws / rl_hnav
  --print-only   print the commands, start nothing
Stop: bash scripts/stop_g1_navigation.sh   (the remote's e-stop stays the emergency stop)
HELP
}

while (($#)); do
  case "$1" in
    --live) LIVE=true; shift ;;
    --max-vx) [[ $# -ge 2 ]] || { usage; exit 2; }; MAX_VX=$2; shift 2 ;;
    --max-wz) [[ $# -ge 2 ]] || { usage; exit 2; }; MAX_WZ=$2; shift 2 ;;
    --no-rviz) RVIZ=false; shift ;;
    --nic) [[ $# -ge 2 ]] || { usage; exit 2; }; NIC=$2; shift 2 ;;
    --docker) RUNNER=docker; shift ;;
    --host) RUNNER=host; shift ;;
    --print-only) PRINT_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
  esac
done

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33mWARNING:\033[0m %s\n' "$*" >&2; }
die() { printf '\033[31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

in_range() {  # value, max: a positive number not above max
  [[ $1 =~ ^[0-9]*\.?[0-9]+$ ]] && awk -v v="$1" -v m="$2" 'BEGIN{exit !(v > 0 && v <= m)}'
}
in_range "$MAX_VX" 0.50 || die "--max-vx must be in (0, 0.50] m/s"
in_range "$MAX_WZ" 1.0 || die "--max-wz must be in (0, 1.0] rad/s"

if [[ -z $NIC ]]; then
  NIC=$(ip -o route get "$ORIN" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "dev") {print $(i + 1); exit}}')
  [[ -n $NIC ]] || NIC=enp2s0
fi
[[ $NIC =~ ^[a-zA-Z0-9_.:-]+$ && $NIC != lo ]] || die "--nic: a wired interface, not loopback"

session_running() { docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$SESSION_CONTAINER"; }
if [[ $RUNNER == auto ]]; then
  if session_running || [[ ! -r /opt/ros/humble/setup.bash ]]; then RUNNER=docker; else RUNNER=host; fi
fi
ATTACH=false
[[ $RUNNER == docker ]] && session_running && ATTACH=true
[[ $RUNNER == host ]] && session_running && die "start_g1_session.sh runs (container $SESSION_CONTAINER):
    use --docker (it joins that session), or stop it first: bash scripts/stop_g1_session.sh"

# ---- what each window runs -------------------------------------------------------------------
STAMP=$(date +%Y%m%d_%H%M%S)
if [[ $RUNNER == docker ]]; then
  WS=/ws
  DB=/ws/bags/maps/g1_nav_${STAMP}.db          # survives the container (bags/ is git-ignored)
  CMD_TF='ros2 launch g1_sensors tf_chain.launch.py'
else
  WS=$ROOT
  DB=$HOME/.ros/g1_nav_${STAMP}.db
  URDF=$ROOT/g1_ws/install/g1_sensors/share/g1_sensors/urdf/g1_29dof_rev_1_0.urdf
  MESH_DIR=$ROOT/third_party/unitree_ros/robots/g1_description/meshes
  CMD_TF="ros2 launch g1_sensors tf_chain.launch.py urdf:=$URDF mesh_dir:=$MESH_DIR"
fi
CLIENT=$WS/g1_ws/install/g1_loco_cmdvel/lib/g1_loco_cmdvel/g1_loco_client
CMD_MAP="ros2 launch g1_mapping mapping.launch.py static_tf:=false database_path:=$DB"
CMD_SCAN='ros2 launch humanoid_nav_bridge real_robot_bridge.launch.py publish_odom_tf:=false publish_lidar_tf:=false override_scan_stamp:=false'
if $LIVE; then
  CMD_NAV="ros2 launch g1_nav2 rtabmap_nav_live.launch.py max_vx:=$MAX_VX max_wz:=$MAX_WZ"
  CMD_MON='ros2 topic echo /cmd_vel geometry_msgs/msg/Twist'
else
  CMD_NAV="ros2 launch g1_nav2 rtabmap_nav_dry_run.launch.py max_vx:=$MAX_VX max_wz:=$MAX_WZ"
  CMD_MON='ros2 topic echo /g1_nav2_dry_run/cmd_vel geometry_msgs/msg/Twist'
fi
# The Unitree SDK brings its own CycloneDDS 0.10.2: no ROS library path, no ROS DDS config (AGENTS.md §5).
CMD_CLIENT="env -u LD_LIBRARY_PATH -u CYCLONEDDS_URI stdbuf -oL -eL $CLIENT --network-interface=$NIC --enabled=true --i-accept-high-level-actuation=true"
CMD_GATEWAY="ros2 run g1_loco_cmdvel cmd_vel_gateway --ros-args -p enabled:=true -p require_battery:=true -p battery_topic:=/battery_state -p min_battery_percent:=0.20 -p battery_timeout_sec:=1.0 -p command_timeout_sec:=0.30 -p max_vx:=$MAX_VX -p max_vy:=$MAX_VX -p max_wz:=$MAX_WZ"
CMD_RVIZ='rviz2 -d "$(ros2 pkg prefix g1_nav2)/share/g1_nav2/rviz/g1_nav_minimal.rviz"'
CMD_CHECKS='cat <<EOF
Checks (shell with ROS + rl_hnav sourced):
  ros2 run g1_nav2 check_rtabmap_plan        TF, /scan, costmaps, battery, a test path -> READY?
  ros2 topic info /cmd_vel -v                dry run: 0 publishers; live: 1 subscriber (the gateway)
  ros2 topic echo /battery_state --field percentage
  ros2 topic hz /map                         the map grows while the robot surveys
EOF'
# The explorer sends goals as soon as it runs: it waits for Enter. The dry run checks the plan first;
# check_rtabmap_plan refuses while /cmd_vel exists, so --live relies on the dry run's check.
PREFLIGHT=true; $LIVE && PREFLIGHT=false
if $LIVE; then
  EXPLORE_MODE='LIVE: the G1 WALKS to every frontier it finds. Safety zone ready, remote e-stop in hand.'
else
  EXPLORE_MODE='Dry run: Nav2 plans and shows the velocities it would send (G1 5); nothing moves.'
fi
CMD_EXPLORE="cat <<EOF
Frontier exploration (explore_lite on the RTAB-Map /map; goals -> Nav2 NavigateToPose).
  $EXPLORE_MODE
  Survey with the remote first: on a standing-only map there are no frontiers.
  Pause:  ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'   (resume: true)
  Stop:   Ctrl-C here (or bash scripts/stop_g1_navigation.sh)
  RViz:   add a MarkerArray display on /explore/frontiers
EOF
read -r -p 'Press Enter to start exploring (Ctrl-C to skip): ' _ && ros2 launch g1_nav2 rtabmap_explore.launch.py preflight:=$PREFLIGHT"

if $PRINT_ONLY; then
  if $ATTACH; then say "Runs inside $SESSION_CONTAINER (start_g1_session.sh already runs TF + map)"
  elif [[ $RUNNER == docker ]]; then say "Container $CONTAINER (g1-humble, robot DDS on $NIC)"
  else say "Host ROS Humble, robot DDS on $NIC"; fi
  $ATTACH || { say "[G1 1 - robot TF]"; echo "$CMD_TF"; say "[G1 2 - map]"; echo "$CMD_MAP"; }
  say "[G1 3 - /scan]"; echo "$CMD_SCAN"
  if $LIVE; then
    say "[G1 L1 - LIVE Loco SDK client]"; echo "$CMD_CLIENT"
    say "[G1 L2 - LIVE cmd_vel_gateway]"; echo "$CMD_GATEWAY"
  fi
  say "[G1 4 - Nav2 $($LIVE && echo LIVE || echo 'dry run')]"; echo "$CMD_NAV"
  say "[G1 5 - velocity monitor]"; echo "$CMD_MON"
  $RVIZ && { say "[G1 6 - RViz goals]"; echo "$CMD_RVIZ"; }
  say "[G1 7 - checks]"; echo "ros2 run g1_nav2 check_rtabmap_plan"
  say "[G1 8 - frontier explorer, after Enter]"; echo "ros2 launch g1_nav2 rtabmap_explore.launch.py preflight:=$PREFLIGHT"
  exit 0
fi

# ---- checks ----------------------------------------------------------------------------------
command -v gnome-terminal >/dev/null || die "gnome-terminal is required"

say "1. Robot network ($NIC)"
ip -br addr show "$NIC" 2>/dev/null | grep -q '192\.168\.123\.' || die "$NIC has no 192.168.123.x address. Plug the robot's Ethernet cable, then:
    sudo ip addr add 192.168.123.222/24 dev $NIC
    sudo ip link set $NIC up"
ping -c1 -W2 "$ORIN" >/dev/null 2>&1 || die "the Orin ($ORIN) does not answer on $NIC"
echo "OK: $(ip -br addr show "$NIC" | awk '{print $3}'), Orin answers"

say "2. Clock sync to the robot (Nav2 compares TF ages with the laptop clock)"
server=$(timedatectl timesync-status 2>/dev/null | awk '/Server:/{print $2}')
if [[ $server == 192.168.123.161 ]]; then echo "OK: NTP server 192.168.123.161"
elif $LIVE; then die "laptop clock not synced to the robot (server: ${server:-none}): g1_ws/README.md §5"
else warn "laptop clock not synced to the robot (server: ${server:-none}); Nav2 may drop TF (g1_ws/README.md §5)"
fi

say "3. ROS environment"
if [[ $RUNNER == docker ]]; then
  command -v docker >/dev/null || die "docker is not installed"
  docker image inspect g1-humble >/dev/null 2>&1 \
    || die "Docker image g1-humble missing: build it once with internet (SIM=1 scripts/run_humble.sh, then exit)"
  if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    die "navigation already runs (container $CONTAINER). Stop it first: bash scripts/stop_g1_navigation.sh"
  fi
  if $ATTACH; then
    TARGET=$SESSION_CONTAINER
    docker exec "$TARGET" pgrep -f 'g1_nav2|humanoid_nav_bridge|cmd_vel_gateway|g1_loco_client' >/dev/null \
      && die "navigation already runs inside $TARGET. Stop it first: bash scripts/stop_g1_navigation.sh"
    for what in tf_chain.launch.py mapping.launch.py; do
      docker exec "$TARGET" pgrep -f "ros2 launch .*$what" >/dev/null \
        || die "$SESSION_CONTAINER runs, but not its $what. Restart the session (stop_g1_session.sh, start_g1_session.sh) or stop it and run this script alone."
    done
    echo "OK: joining the session in $TARGET (its TF + map)"
  else
    others=$(docker ps --filter ancestor=g1-humble --format '{{.Names}}')
    [[ -z $others ]] || die "another g1-humble container runs ($others): a second stack would duplicate TF/map.
    Stop it first (scripts/stop_humble.sh), or start start_g1_session.sh first and then this script."
    TARGET=$CONTAINER
    xhost +local: >/dev/null 2>&1 || true
    ARGS=(--net=host --ipc=host --user "$(id -u):$(id -g)"
          -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro
          -e HOME=/tmp/home -e ROBOT_IFACE="$NIC" -e G1_SIM=0
          -e DISPLAY="${DISPLAY:-}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw
          -v "$ROOT":/ws)
    if [ -d /dev/dri ]; then                    # GPU rendering for RViz (like run_humble.sh)
      ARGS+=(--device /dev/dri)
      for g in video render; do
        gid="$(getent group "$g" | cut -d: -f3)"
        [ -n "$gid" ] && ARGS+=(--group-add "$gid")
      done
    fi
    mkdir -p "$ROOT/bags/maps"
    docker run -d --rm --name "$TARGET" "${ARGS[@]}" g1-humble sleep infinity >/dev/null
    echo "started container $TARGET"
  fi
  ros_sh() { docker exec -i "$TARGET" bash -c "source /ros_entrypoint.sh; $1"; }
  abort() { $ATTACH || docker stop "$TARGET" >/dev/null; die "$@"; }
else
  for setup in /opt/ros/humble/setup.bash "$ROOT/g1_ws/install/setup.bash" "$ROOT/rl_hnav/install/setup.bash"; do
    [[ -r $setup ]] || die "missing $setup (host mode needs host-built workspaces; or use --docker)"
  done
  [[ -r $URDF && -d $MESH_DIR ]] || die "missing the G1 URDF or meshes: $URDF $MESH_DIR"
  export G1_NAV_ROOT=$ROOT G1_NAV_NIC=$NIC
  HOST_ENV='source /opt/ros/humble/setup.bash; source "$G1_NAV_ROOT/g1_ws/install/setup.bash";
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0;
    export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${G1_NAV_NIC}\" priority=\"default\" multicast=\"default\"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>"'
  ros_sh() { bash -c "$HOST_ENV; $1"; }
  abort() { die "$@"; }
fi

# Rebuild in the container when a package's sources are newer than its install (after git pull).
# Joining a session: only the navigation packages, never the ones its running nodes use.
stale() {  # workspace dir, package. colcon rewrites build/<pkg>/colcon_build.rc (the exit code)
  local stamp="$ROOT/$1/build/$2/colcon_build.rc"   # on every build; install/ keeps source mtimes
  [[ ! -e $stamp || $(<"$stamp") != 0 ]] \
    || [[ -n $(find "$ROOT/$1/src/$2" -type f -not -name '*.pyc' -newer "$stamp" -print -quit) ]]
}
if [[ $RUNNER == docker ]]; then
  G1_PKGS=(g1_loco_cmdvel); $ATTACH || G1_PKGS=(g1_sensors g1_mapping g1_loco_cmdvel)
  NAV_PKGS=(g1_nav2 humanoid_nav_bridge)
  build_g1=(); build_nav=()
  for p in "${G1_PKGS[@]}"; do stale g1_ws "$p" && build_g1+=("$p"); done
  for p in "${NAV_PKGS[@]}"; do stale rl_hnav "$p" && build_nav+=("$p"); done
  if ((${#build_g1[@]})); then
    # g1_loco_client needs the Unitree SDK submodule; without it only cmd_vel_gateway is built.
    # Always pass the SDK path: CMake caches it, and a stale path fails the whole build.
    sdk=
    [[ -e $ROOT/third_party/unitree_sdk2/include/unitree/robot/g1/loco/g1_loco_client.hpp ]] \
      && sdk=/ws/third_party/unitree_sdk2
    say "   building ${build_g1[*]} (g1_ws; Unitree SDK: ${sdk:-none, no g1_loco_client})"
    ros_sh "cd /ws/g1_ws && colcon build --packages-select ${build_g1[*]} --cmake-args -DUNITREE_SDK_ROOT=$sdk" \
      || abort "g1_ws build failed"
  fi
  if ((${#build_nav[@]})); then
    say "   building ${build_nav[*]} (rl_hnav)"
    ros_sh "cd /ws/rl_hnav && colcon build --packages-select ${build_nav[*]}" || abort "rl_hnav build failed"
  fi
  ros_sh 'source /ws/rl_hnav/install/setup.bash && ros2 pkg prefix g1_nav2 >/dev/null' \
    || abort "g1_nav2 is not built in /ws/rl_hnav/install"
fi
ros_sh "source $WS/rl_hnav/install/setup.bash && ros2 pkg prefix explore_lite >/dev/null" \
  || abort "explore_lite (m-explore) is not built. Once, with internet: bash scripts/setup_m_explore.sh$([[ $RUNNER == host ]] && echo ' --host')"

say "4. Who already publishes on the robot network"
probe=$(ros_sh 'python3 - <<EOF
import time, rclpy
rclpy.init(); n = rclpy.create_node("g1_nav_probe")
end = time.time() + 4.0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
c = n.count_publishers
# the OAK-D driver (Orin) publishes its camera frames on /tf: not a robot TF/map owner
tf = [i for i in n.get_publishers_info_by_topic("/tf") if not i.node_name.startswith("oak")]
print(len(tf) + c("/map"), c("/scan"), c("/cmd_vel") + n.count_subscribers("/cmd_vel"))
EOF' 2>/dev/null | tail -1) || probe="0 0 0"
read -r tf_map scan cmd_vel <<<"${probe:-0 0 0}"
if ! $ATTACH && ((tf_map > 0)); then
  abort "someone already publishes /tf or /map (another laptop's stack, or a session run by hand).
    Only one TF/map owner may run (AGENTS.md §6)."
fi
((scan == 0)) || abort "someone already publishes /scan (a navigation stack already runs?)"
((cmd_vel == 0)) || abort "/cmd_vel already has publishers or subscribers (another Nav2 or gateway?).
    Stop it first: bash scripts/stop_g1_navigation.sh"
echo "OK: $($ATTACH && echo "TF + map from the session; ")no /scan, no /cmd_vel yet"

if $LIVE; then
  say "5. LIVE: the robot will walk"
  ros_sh "test -x $CLIENT" || abort "the Loco SDK client is not built ($CLIENT).
    It needs the Unitree SDK (once, with internet), then Stage 3 + Stage 4 (g1_ws/src/g1_loco_cmdvel/README.md):
      git submodule update --init third_party/unitree_sdk2
      docker exec -it <container> bash -c 'cd /ws/g1_ws && colcon build --packages-select g1_loco_cmdvel --cmake-args -DUNITREE_SDK_ROOT=/ws/third_party/unitree_sdk2'"
  cat <<EOF
Before typing LIVE, all of these are true (AGENTS.md §19 / §25, g1_ws/NAV2_FIRST_STEP.md):
  - Stage 4 (one 0.05 m/s command under harness) passed with this laptop and this build
  - the dry run of this script on the current map passed: check_rtabmap_plan READY, sane velocities
  - two team members present; one holds the paired remote, ready for emergency damping
  - flat floor, safety zone cleared, spectators behind the line; battery above 20 %
  - speed limits: max_vx $MAX_VX m/s, max_wz $MAX_WZ rad/s (first goal: --max-vx 0.05 --max-wz 0.10)
EOF
  [[ -t 0 ]] || abort "--live needs a terminal to confirm"
  read -r -p 'Type LIVE to start the executor: ' answer
  [[ $answer == LIVE ]] || abort "not confirmed; nothing started"
fi

# ---- windows ---------------------------------------------------------------------------------
open_window() {  # title, command
  local wrapper='printf "%s\n\n" "$1"; trap : INT; eval "$1"; code=$?
    printf "\nProcess finished (status %s). No automatic restart. This shell stays open.\n" "$code"; exec bash -i'
  if [[ $RUNNER == docker ]]; then
    gnome-terminal --window --title="$1" -- docker exec -it "$TARGET" bash -c \
      "source /ros_entrypoint.sh; source /ws/rl_hnav/install/setup.bash; $wrapper" bash "$2"
  else
    gnome-terminal --window --title="$1" -- bash -c \
      "$HOST_ENV; source \"\$G1_NAV_ROOT/rl_hnav/install/setup.bash\"; cd \"\$G1_NAV_ROOT\"; $wrapper" bash "$2"
  fi
}

say "Opening windows"
if ! $ATTACH; then
  open_window 'G1 1 - robot TF' "$CMD_TF"; sleep 3
  open_window 'G1 2 - map (RTAB-Map)' "$CMD_MAP"; sleep 3
fi
open_window 'G1 3 - /scan bridge' "$CMD_SCAN"
if $LIVE; then                        # executor first, like Stage 4: client, then gateway
  open_window 'G1 L1 - LIVE Loco SDK client' "$CMD_CLIENT"; sleep 2
  open_window 'G1 L2 - LIVE cmd_vel_gateway' "$CMD_GATEWAY"; sleep 1
  open_window 'G1 4 - LIVE Nav2 (publishes /cmd_vel)' "$CMD_NAV"
else
  open_window 'G1 4 - Nav2 dry run (no motion)' "$CMD_NAV"
fi
open_window 'G1 5 - velocity monitor' "$CMD_MON"
$RVIZ && open_window 'G1 6 - RViz goals' "$CMD_RVIZ"
open_window 'G1 7 - checks' "$CMD_CHECKS"
open_window "G1 8 - frontier explorer$($LIVE && echo ' (LIVE: the G1 walks)')" "$CMD_EXPLORE"

$ATTACH || printf '\nMap database: %s\n' "$DB"
if $LIVE; then
  cat <<EOF

LIVE (max_vx $MAX_VX m/s, max_wz $MAX_WZ rad/s):
  1. "G1 L1" must print actuation=ENABLED; "G1 L2" must be up with the battery OK.
  2. Checks window: ros2 topic info /cmd_vel -v  -> 1 publisher (velocity_smoother), 1 subscriber (gateway).
  3. Remote operator ready -> RViz "Nav2 Goal", short (0.5-1 m), on free floor.
Emergency: the remote. Stop: bash scripts/stop_g1_navigation.sh (StopMove first, then the rest).
EOF
  echo '  Explorer: press Enter in "G1 8" only with the safety zone ready; it walks to each frontier.'
else
  cat <<EOF

Dry run (nothing moves; velocities go to /g1_nav2_dry_run/cmd_vel):
  1. RViz: wait for the map. Standing only? Survey the room with the vendor remote first.
  2. Checks window: ros2 run g1_nav2 check_rtabmap_plan  -> READY
  3. RViz "Nav2 Goal" 0.5-1 m ahead; "G1 5" shows the velocities it WOULD send.
Stop: bash scripts/stop_g1_navigation.sh     Live, only after the gates: --live (see --help)
EOF
  echo '  Explorer: press Enter in "G1 8": preflight, then frontier goals Nav2 only plans.'
fi
