#!/usr/bin/env bash
# One command for an AR-glasses session on the real G1 (ar_glasses/SETUP.md, AGENTS.md §27).
# Read-only towards the robot: nothing here moves it.
set -euo pipefail

ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
NIC=enp2s0
TAG_SIZE=0.16
CAMERA=oak
BRIDGE_ONLY=false
ORIN=false
BUILD=false
PRINT_ONLY=false
CONTAINER=g1-ar
ORIN_HOST=unitree@192.168.123.164

usage() {
  cat <<'HELP'
Usage: bash scripts/start_ar_glasses.sh [options]

Starts, in one Docker container (image g1-humble, robot DDS on the wired NIC), one terminal
window each for: robot TF (g1_sensors tf_chain), map (g1_mapping), the glasses bridge +
robot tag anchor (g1_ar_bridge), and a checks window. Prints the IP to type into the glasses.

  --tag-size M     edge of the AprilTag's BLACK square in metres (default 0.16)
  --camera NAME    robot camera that sees the wall tag: oak (default) | realsense
  --nic NAME       wired interface to the G1 (default enp2s0)
  --bridge-only    only the bridge: another laptop already runs TF + map
                   (e.g. scripts/start_g1_navigation.sh). Two TF/map owners make TF jump.
  --orin           also open a window that SSHes to the Orin and starts the OAK-D driver
                   (you type the Orin password there)
  --build          colcon build g1_sensors g1_mapping g1_ar_bridge first
  --print-only     print the commands, start nothing
Stop everything with: bash scripts/stop_ar_glasses.sh
HELP
}

while (($#)); do
  case "$1" in
    --tag-size) [[ $# -ge 2 ]] || { usage; exit 2; }; TAG_SIZE=$2; shift 2 ;;
    --camera) [[ $# -ge 2 ]] || { usage; exit 2; }; CAMERA=$2; shift 2 ;;
    --nic) [[ $# -ge 2 ]] || { usage; exit 2; }; NIC=$2; shift 2 ;;
    --bridge-only) BRIDGE_ONLY=true; shift ;;
    --orin) ORIN=true; shift ;;
    --build) BUILD=true; shift ;;
    --print-only) PRINT_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
  esac
done
[[ $TAG_SIZE =~ ^0?\.[0-9]+$|^[0-9]+\.[0-9]+$ ]] || { echo "--tag-size must be metres, e.g. 0.16" >&2; exit 2; }
[[ $CAMERA == oak || $CAMERA == realsense ]] || { echo "--camera must be oak or realsense" >&2; exit 2; }
[[ $NIC =~ ^[a-zA-Z0-9_.:-]+$ && $NIC != lo ]] || { echo "--nic: a wired interface" >&2; exit 2; }

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33mWARNING:\033[0m %s\n' "$*" >&2; }
die() { printf '\033[31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

# ---- commands run inside the container (one window each) ----------------------------------
CMD_TF='ros2 launch g1_sensors tf_chain.launch.py'
CMD_MAP='ros2 launch g1_mapping mapping.launch.py static_tf:=false'
CMD_BRIDGE="ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=${TAG_SIZE} camera:=${CAMERA}"
CMD_CHECKS='echo "Checks: anchor status below. Ctrl-C, then e.g.:"; echo "  ros2 run tf2_ros tf2_echo robot_center spectacles"; echo "  ros2 run g1_ar_bridge publish_demo_pois"; ros2 topic echo /ar_glasses/anchor_status'
ORIN_REMOTE='source /opt/ros/foxy/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0; export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>"; ros2 launch depthai_ros_driver camera.launch.py use_rviz:=false'
# quoted twice: once for the local shell of the window, once for the Orin's login shell
CMD_ORIN="ssh -t ${ORIN_HOST} $(printf %q "bash -lc $(printf %q "$ORIN_REMOTE")")"

if $PRINT_ONLY; then
  say "Container (robot mode, DDS on $NIC):"
  echo "docker run -d --rm --name $CONTAINER --net=host --ipc=host ... -e ROBOT_IFACE=$NIC g1-humble sleep infinity"
  $ORIN && { say "[AR 0 - Orin OAK-D driver]"; echo "ssh -t $ORIN_HOST   # then on the Orin:"
             echo "$ORIN_REMOTE" | tr ';' '\n' | sed 's/^ *//'; }
  $BRIDGE_ONLY || { say "[AR 1 - robot TF]"; echo "$CMD_TF"; say "[AR 2 - map]"; echo "$CMD_MAP"; }
  say "[AR 3 - glasses bridge]"; echo "$CMD_BRIDGE"
  say "[AR 4 - checks]"; echo "$CMD_CHECKS"
  exit 0
fi

# ---- host checks ------------------------------------------------------------------------------
command -v docker >/dev/null || die "docker is not installed"
command -v gnome-terminal >/dev/null || die "gnome-terminal is required (or run the commands by hand: ar_glasses/SETUP.md)"
docker image inspect g1-humble >/dev/null 2>&1 \
  || die "Docker image g1-humble missing: build it once with internet (SIM=1 scripts/run_humble.sh, then exit)"
[[ -r $ROOT/g1_ws/install/setup.bash ]] || BUILD=true

say "1. Robot network ($NIC)"
if ! ip -br addr show "$NIC" 2>/dev/null | grep -q '192\.168\.123\.'; then
  die "$NIC has no 192.168.123.x address. Plug the robot's Ethernet cable, then:
    sudo ip addr add 192.168.123.222/24 dev $NIC
    sudo ip link set $NIC up"
fi
ping -c1 -W2 192.168.123.164 >/dev/null 2>&1 || die "the Orin (192.168.123.164) does not answer on $NIC"
echo "OK: $(ip -br addr show "$NIC" | awk '{print $3}'), Orin answers"

say "2. Clock sync to the robot (AGENTS.md §7)"
server=$(timedatectl timesync-status 2>/dev/null | awk '/Server:/{print $2}')
if [[ $server == 192.168.123.161 ]]; then
  echo "OK: NTP server 192.168.123.161"
else
  warn "laptop clock not synced to the robot (server: ${server:-none}). Live TF times will be off. Fix once:
    sudo mkdir -p /etc/systemd/timesyncd.conf.d
    printf '[Time]\nNTP=192.168.123.161\nFallbackNTP=\n' | sudo tee /etc/systemd/timesyncd.conf.d/g1-robot.conf
    sudo systemctl restart systemd-timesyncd"
fi

say "3. IP to type into the glasses (Dimensional OS -> Connect)"
glasses_ips=$(ip -4 -o addr show | awk -v nic="$NIC" '$2!="lo" && $2!=nic && $2!~/^(docker|br-|veth)/ {split($4,a,"/"); print "  " a[1] "   (" $2 ")"}')
if [[ -n $glasses_ips ]]; then
  echo "$glasses_ips"
  echo "  Use the one of the Wi-Fi the glasses are on (same hotspot / router), port 8787."
else
  warn "no Wi-Fi address: connect the laptop's Wi-Fi to the same hotspot as the glasses"
fi

# ---- container ----------------------------------------------------------------------------------
say "4. Container"
if docker ps -q --filter ancestor=g1-humble | grep -q .; then
  die "a g1-humble container is already running (a second stack would duplicate TF/map).
    Stop it first: bash scripts/stop_ar_glasses.sh   (or scripts/stop_humble.sh)"
fi
docker run -d --rm --name "$CONTAINER" --net=host --ipc=host \
  --user "$(id -u):$(id -g)" \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -e HOME=/tmp/home -e ROBOT_IFACE="$NIC" -e G1_SIM=0 \
  -v "$ROOT":/ws g1-humble sleep infinity >/dev/null
in_container() { docker exec "$CONTAINER" bash -c "source /ros_entrypoint.sh; $1"; }
echo "started container $CONTAINER"

if $BUILD; then
  say "   building g1_sensors g1_mapping g1_ar_bridge"
  in_container 'cd /ws/g1_ws && colcon build --packages-select g1_sensors g1_mapping g1_ar_bridge' \
    || { docker stop "$CONTAINER" >/dev/null; die "build failed"; }
fi

say "5. What is on the robot network"
graph=$(in_container 'python3 - <<EOF
import time, rclpy
rclpy.init()
n = rclpy.create_node("ar_setup_probe")
end = time.time() + 4.0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
topics = {t for t, _ in n.get_topic_names_and_types()}
print("tf_publishers", n.count_publishers("/tf"))
print("lidar", "/utlidar/cloud_livox_mid360" in topics)
print("oak", "/oak/rgb/image_raw" in topics)
print("realsense", "/camera/color/image_raw" in topics)
print("map", n.count_publishers("/map"))
EOF' 2>/dev/null) || true
get() { awk -v k="$1" '$1==k{print $2}' <<<"$graph"; }
[[ $(get lidar) == True ]] || warn "robot LiDAR not visible: robot on? cable in? (continuing)"
tf_pubs=$(get tf_publishers); map_pubs=$(get map)
if ! $BRIDGE_ONLY && [[ ${tf_pubs:-0} -gt 0 || ${map_pubs:-0} -gt 0 ]]; then
  docker stop "$CONTAINER" >/dev/null
  die "someone already publishes /tf ($tf_pubs) or /map ($map_pubs) on the robot network, e.g. the
    navigation laptop. Only one TF/map owner may run (AGENTS.md §6). Use: --bridge-only"
fi
if $BRIDGE_ONLY && [[ ${tf_pubs:-0} -eq 0 ]]; then
  warn "--bridge-only but nobody publishes /tf yet: start the other laptop's TF + map"
fi
cam_key=$([[ $CAMERA == oak ]] && echo oak || echo realsense)
if [[ $(get "$cam_key") != True ]] && ! $ORIN; then
  warn "no $CAMERA image topic yet: start the camera driver on the Orin (--orin, or ar_glasses/SETUP.md)"
fi
echo "LiDAR: $(get lidar)   $CAMERA: $(get "$cam_key")   /tf publishers: ${tf_pubs:-?}"

# ---- windows ------------------------------------------------------------------------------------
open_window() {  # title, command (inside the container unless the title starts with "AR 0")
  local title=$1 command=$2
  if [[ $title == "AR 0"* ]]; then
    gnome-terminal --window --title="$title" -- bash -c "$command; echo; echo 'finished'; exec bash -i"
  else
    gnome-terminal --window --title="$title" -- docker exec -it "$CONTAINER" bash -c \
      "source /ros_entrypoint.sh; printf '%s\n\n' \"\$1\"; eval \"\$1\"; echo; echo 'Process finished (no restart). This shell stays open.'; exec bash -i" \
      bash "$command"
  fi
}

say "6. Opening windows"
if $ORIN; then open_window 'AR 0 - Orin OAK-D driver (type the Orin password)' "$CMD_ORIN"; sleep 1; fi
if ! $BRIDGE_ONLY; then
  open_window 'AR 1 - robot TF' "$CMD_TF"; sleep 3
  open_window 'AR 2 - map (RTAB-Map)' "$CMD_MAP"; sleep 2
fi
open_window 'AR 3 - glasses bridge + tag anchor' "$CMD_BRIDGE"; sleep 1
open_window 'AR 4 - checks' "$CMD_CHECKS"

cat <<EOF

Next:
  1. Robot: stand it STILL 1-1.5 m in front of the wall tag, facing it (vendor remote).
     Window "AR 3" prints: anchored map -> ar_tag_0
  2. Glasses: Drafts -> Dimensional OS -> Start Robot & Bridge: Next -> Connect: the IP above
     -> Registration: AprilTag. Step sideways slowly, pause ~1 s per step, do NOT press Skip.
     "AR 3" prints: registered (april_tag)   (~30 s)
  3. Wrist menu (left palm up) -> LiDAR full.
Stop everything: bash scripts/stop_ar_glasses.sh
EOF
