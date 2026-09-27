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
OAK_DOMAIN=78
SEARCH=false
BUILD=false
DEMO=true
LEO=false
LEO_HOST=pi@10.0.0.1
PRINT_ONLY=false
CONTAINER=g1-ar
SEARCH_CONTAINER=g1-search
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
  --orin           also open a window that SSHes to the Orin and starts the OAK-D driver in DDS
                   domain 78 (it segfaults on the robot's domain 0), plus a relay window that
                   copies its topics into domain 0 (g1_sensors oak_domain_relay)
  --oak-domain N   the OAK-D driver's DDS domain (default 78; 0 = directly, no relay)
  --search         object search: type "red cup" in the glasses -> Grounding DINO + SAM2 on the
                   chest OAK-D (GPU container g1-semantic, docker/Dockerfile.semantic) -> 3D box in
                   the glasses. First run downloads the models (~1 GB) into bags/hf_cache
                   (you type the Orin password there)
  --no-demo        do not draw the demo scene (virtual table + green box + red bottle in front
                   of the robot, as in the home test); use it once real POIs are published
  --leo            also place the Leo Rover in the G1 map from its sightings of the same wall tag
                   and mark it in the glasses: opens a window that runs the read-only relay on
                   Leo over SSH (type Leo's password there). The laptop Wi-Fi AND the glasses
                   must be on Leo's hotspot. See g1_ws/docs/leo_g1_laptop_integration.md
  --leo-host U@IP  Leo's SSH login (default pi@10.0.0.1)
  --build          colcon build g1_sensors g1_mapping g1_ar_bridge semantic_query first
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
    --oak-domain) [[ $# -ge 2 ]] || { usage; exit 2; }; OAK_DOMAIN=$2; shift 2 ;;
    --search) SEARCH=true; shift ;;
    --build) BUILD=true; shift ;;
    --no-demo) DEMO=false; shift ;;
    --leo) LEO=true; shift ;;
    --leo-host) [[ $# -ge 2 ]] || { usage; exit 2; }; LEO_HOST=$2; shift 2 ;;
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
CMD_BRIDGE="ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=${TAG_SIZE} camera:=${CAMERA} demo_pois:=${DEMO} leo:=${LEO}"
CMD_RELAY="ros2 run g1_sensors oak_domain_relay --from-domain ${OAK_DOMAIN} --to-domain 0"
CMD_SEARCH='ros2 launch semantic_query semantic_query.launch.py'
CMD_CHECKS='echo "Checks: anchor status below. Ctrl-C, then e.g.:"; echo "  ros2 run tf2_ros tf2_echo robot_center spectacles"; echo "  ros2 run g1_ar_bridge publish_demo_pois"; ros2 topic echo /ar_glasses/anchor_status'
[[ $OAK_DOMAIN =~ ^[0-9]+$ ]] || { echo "--oak-domain must be a number" >&2; exit 2; }
# On the Orin: check the camera is on USB, stop a stuck driver cleanly (SIGINT to the real launch
# process only; the pattern is anchored so this shell never matches itself), start the driver.
ORIN_REMOTE='lsusb | grep -qi 03e7 || echo "!! no OAK-D (USB id 03e7) on the Orin: check its cable";
old=$(pgrep -f "^/usr/bin/python3 .*ros2 launch depthai_ros_driver");
if [ -n "$old" ]; then echo "stopping the old OAK-D driver ($old)"; kill -INT $old; sleep 5; fi;
source /opt/ros/foxy/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID='"$OAK_DOMAIN"';
export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>";
echo "starting the OAK-D driver in DDS domain '"$OAK_DOMAIN"'"; ros2 launch depthai_ros_driver camera.launch.py use_rviz:=false'
# quoted twice: once for the local shell of the window, once for the Orin's login shell
CMD_ORIN="ssh -t ${ORIN_HOST} $(printf %q "bash -lc $(printf %q "$ORIN_REMOTE")")"

# Leo relay: the script is sent inline (no install on Leo), run in a real terminal so Ctrl-C
# (or closing the window) also stops it on Leo. It only reads Leo's TF and odometry.
LEO_IP=${LEO_HOST#*@}
LEO_LAPTOP_IP=$(ip route get "$LEO_IP" 2>/dev/null | awk '{for (i = 1; i < NF; i++) if ($i == "src") print $(i + 1)}' | head -1)
leo_relay_cmd() {
  local b64 remote
  b64=$(base64 -w0 "$ROOT/g1_ws/src/g1_ar_bridge/g1_ar_bridge/leo_relay.py")
  remote="source /opt/ros/jazzy/setup.bash && echo $b64 | base64 -d > /tmp/g1_leo_relay.py && exec python3 /tmp/g1_leo_relay.py --laptop-ip ${LEO_LAPTOP_IP:-<laptop IP on Leo net>}"
  echo "ssh -t $LEO_HOST $(printf %q "bash -lc $(printf %q "$remote")")"
}

if $PRINT_ONLY; then
  say "Container (robot mode, DDS on $NIC):"
  echo "docker run -d --rm --name $CONTAINER --net=host --ipc=host ... -e ROBOT_IFACE=$NIC g1-humble sleep infinity"
  $ORIN && { say "[AR 0 - Orin OAK-D driver]"; echo "ssh -t $ORIN_HOST   # then on the Orin:"
             echo "$ORIN_REMOTE" | tr ';' '\n' | sed 's/^ *//'; }
  $BRIDGE_ONLY || { say "[AR 1 - robot TF]"; echo "$CMD_TF"; say "[AR 2 - map]"; echo "$CMD_MAP"; }
  $ORIN && [[ $OAK_DOMAIN != 0 ]] && { say "[AR 0 - OAK relay, domain $OAK_DOMAIN -> 0]"; echo "$CMD_RELAY"; }
  say "[AR 3 - glasses bridge]"; echo "$CMD_BRIDGE"
  $SEARCH && { say "[AR 7 - object search, container $SEARCH_CONTAINER (image g1-semantic, GPU)]"; echo "$CMD_SEARCH"; }
  $LEO && { say "[AR 5 - Leo relay on $LEO_HOST]"
            echo "ssh -t $LEO_HOST, then: source /opt/ros/jazzy/setup.bash && python3 leo_relay.py --laptop-ip ${LEO_LAPTOP_IP:-<laptop IP on Leo net>}"
            echo "(the launcher sends g1_ws/src/g1_ar_bridge/g1_ar_bridge/leo_relay.py inline)"; }
  say "[AR 4 - checks]"; echo "$CMD_CHECKS"
  exit 0
fi

# ---- host checks ------------------------------------------------------------------------------
command -v docker >/dev/null || die "docker is not installed"
command -v gnome-terminal >/dev/null || die "gnome-terminal is required (or run the commands by hand: ar_glasses/SETUP.md)"
docker image inspect g1-humble >/dev/null 2>&1 \
  || die "Docker image g1-humble missing: build it once with internet (SIM=1 scripts/run_humble.sh, then exit)"
if $SEARCH; then
  docker image inspect g1-semantic >/dev/null 2>&1 \
    || die "Docker image g1-semantic missing: build it once with internet:
    docker build -t g1-semantic -f docker/Dockerfile.semantic ."
  [[ -r $ROOT/g1_ws/install/semantic_query/share/semantic_query/package.xml ]] || BUILD=true
fi
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
if $LEO; then
  say "3b. Leo Rover ($LEO_HOST)"
  if ping -c1 -W2 "$LEO_IP" >/dev/null 2>&1 && [[ -n $LEO_LAPTOP_IP ]]; then
    echo "OK: Leo answers; this laptop is $LEO_LAPTOP_IP on Leo's network"
    [[ $LEO_LAPTOP_IP == 10.0.0.* ]] || warn "the laptop reaches Leo from $LEO_LAPTOP_IP: the glasses must be on that same network"
  else
    warn "Leo ($LEO_IP) does not answer: connect the laptop Wi-Fi to Leo's hotspot. Leo will not appear."
    LEO_LAPTOP_IP=
  fi
fi

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
  say "   building g1_sensors g1_mapping g1_ar_bridge semantic_query"
  in_container 'cd /ws/g1_ws && colcon build --packages-select g1_sensors g1_mapping g1_ar_bridge semantic_query' \
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
open_window() {  # title, command[, container] (in a container unless the title starts with "AR 0")
  local title=$1 command=$2 container=${3:-$CONTAINER}
  if [[ $title == "AR 0"* ]]; then
    gnome-terminal --window --title="$title" -- bash -c "$command; echo; echo 'finished'; exec bash -i"
  else
    gnome-terminal --window --title="$title" -- docker exec -it "$container" bash -c \
      "source /ros_entrypoint.sh; printf '%s\n\n' \"\$1\"; eval \"\$1\"; echo; echo 'Process finished (no restart). This shell stays open.'; exec bash -i" \
      bash "$command"
  fi
}

if $SEARCH; then
  say "5b. Object search container ($SEARCH_CONTAINER, image g1-semantic)"
  GPU_ARGS=()
  if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' && command -v nvidia-smi >/dev/null \
      && nvidia-smi >/dev/null 2>&1; then
    GPU_ARGS=(--gpus all); echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
  else
    warn "no NVIDIA GPU for Docker: the search runs on the CPU (several seconds per frame)"
  fi
  mkdir -p "$ROOT/bags/hf_cache"
  docker run -d --rm --name "$SEARCH_CONTAINER" --net=host --ipc=host "${GPU_ARGS[@]}" \
    --user "$(id -u):$(id -g)" \
    -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
    -e HOME=/tmp/home -e ROBOT_IFACE="$NIC" -e G1_SIM=0 \
    -v "$ROOT":/ws g1-semantic sleep infinity >/dev/null
  echo "started container $SEARCH_CONTAINER"
fi

say "6. Opening windows"
if $ORIN; then
  open_window "AR 0 - Orin OAK-D driver, domain $OAK_DOMAIN (type the Orin password)" "$CMD_ORIN"
  [[ $OAK_DOMAIN != 0 ]] && open_window "OAK relay, domain $OAK_DOMAIN -> 0" "$CMD_RELAY"
  sleep 1
fi
if ! $BRIDGE_ONLY; then
  open_window 'AR 1 - robot TF' "$CMD_TF"; sleep 3
  open_window 'AR 2 - map (RTAB-Map)' "$CMD_MAP"; sleep 2
fi
open_window 'AR 3 - glasses bridge + tag anchor' "$CMD_BRIDGE"; sleep 1
if $LEO && [[ -n $LEO_LAPTOP_IP ]]; then
  open_window "AR 0 - Leo relay on $LEO_HOST (type Leo's password; read-only)" "$(leo_relay_cmd)"
fi
$SEARCH && open_window 'AR 7 - object search (Grounding DINO + SAM2)' "$CMD_SEARCH" "$SEARCH_CONTAINER"
open_window 'AR 4 - checks' "$CMD_CHECKS"

cat <<EOF

Next:
  1. Robot: stand it STILL 1-1.5 m in front of the wall tag, facing it (vendor remote).
     Window "AR 3" prints: anchored map -> ar_tag_0
  2. Glasses: Drafts -> Dimensional OS -> Start Robot & Bridge: Next -> Connect: the IP above
     -> Registration: AprilTag. Step sideways slowly, pause ~1 s per step, do NOT press Skip.
     "AR 3" prints: registered (april_tag)   (~30 s)
  3. Wrist menu (left palm up) -> LiDAR full. The demo scene (virtual table, green box,
     red bottle) stands in front of where the robot was at start (off: --no-demo).
  4. With --search: type "red cup" (or "search for a red cup") in the glasses' text box. The
     robot looks for up to 20 s; found objects get a green 3D box + label. "stop", "clear".
     The first search after start waits for the models to load (window AR 7).
  5. With --leo: Leo appears as a blue box once it sees the same wall tag
     (ros2 topic echo /leo_in_g1/status). Its camera mount is not measured yet: provisional.
Stop everything: bash scripts/stop_ar_glasses.sh
EOF
