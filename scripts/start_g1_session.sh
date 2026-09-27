#!/usr/bin/env bash
# Start a full G1 session in separate windows: OAK-D camera on the Orin, robot TF, map, the AR
# glasses bridge + tag anchor, RViz, and the object search (Grounding DINO + SAM2 on the GPU).
# Step by step and by hand: ROBOT_SESSION.md. Read-only towards the robot: nothing moves it.
set -euo pipefail

ROOT=$(realpath "$(dirname "${BASH_SOURCE[0]}")/..")
NIC=enp2s0
TAG_SIZE=0.16
OAK_DOMAIN=0
ORIN=true
SEARCH=true
SKELETON=true
RVIZ=true
GLASSES=true
DEMO=false
BUILD=false
PRINT_ONLY=false
CONTAINER=g1-robot
SEARCH_CONTAINER=g1-search
ORIN_HOST=unitree@192.168.123.164
RVIZ_CONFIG=/ws/g1_ws/src/g1_ar_bridge/rviz/g1_session.rviz

usage() {
  cat <<'HELP'
Usage: bash scripts/start_g1_session.sh [options]          (details: ROBOT_SESSION.md)

Opens one window each for:
  G1 0  Orin: OAK-D driver (type the Orin password)     G1 3  AR glasses bridge + robot tag anchor
  G1 1  robot TF (g1_sensors tf_chain)                   G1 4  RViz (camera, 3D map, boxes, TF)
  G1 2  map (g1_mapping, RTAB-Map)                       G1 5  checks shell
  G1 6  object search (semantic_query, GPU container)
Then ask for objects from any terminal:  bash scripts/g1_search.sh red cup

  --tag-size M      black square of the wall AprilTag in metres (default 0.16)
  --oak-domain N    DDS domain of the OAK-D driver (default 0, directly; 78 adds a relay into
                    domain 0 - use it if the driver crashes with "Segmentation fault" on 0)
  --no-orin         do not open the Orin window (the OAK-D driver already runs)
  --no-search       no object search       --no-rviz     no RViz
  --no-glasses      no AR glasses bridge / tag anchor
  --no-skeleton     do not draw the robot's skeleton (links from URDF + TF) in the glasses
  --demo            draw the demo scene (virtual table + green box) for the glasses
  --build           rebuild the packages first (done automatically when sources are newer)
  --nic NAME        wired interface to the G1 (default enp2s0)
  --print-only      print the commands, start nothing
Stop everything: bash scripts/stop_g1_session.sh
HELP
}

while (($#)); do
  case "$1" in
    --tag-size) [[ $# -ge 2 ]] || { usage; exit 2; }; TAG_SIZE=$2; shift 2 ;;
    --oak-domain) [[ $# -ge 2 ]] || { usage; exit 2; }; OAK_DOMAIN=$2; shift 2 ;;
    --no-orin) ORIN=false; shift ;;
    --no-search) SEARCH=false; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --no-glasses) GLASSES=false; shift ;;
    --no-skeleton) SKELETON=false; shift ;;
    --demo) DEMO=true; shift ;;
    --build) BUILD=true; shift ;;
    --nic) [[ $# -ge 2 ]] || { usage; exit 2; }; NIC=$2; shift 2 ;;
    --print-only) PRINT_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
  esac
done
[[ $TAG_SIZE =~ ^[0-9]*\.[0-9]+$ ]] || { echo "--tag-size must be metres, e.g. 0.16" >&2; exit 2; }
[[ $OAK_DOMAIN =~ ^[0-9]+$ ]] || { echo "--oak-domain must be a number" >&2; exit 2; }
[[ $NIC =~ ^[a-zA-Z0-9_.:-]+$ && $NIC != lo ]] || { echo "--nic: a wired interface" >&2; exit 2; }

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33mWARNING:\033[0m %s\n' "$*" >&2; }
die() { printf '\033[31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }

# ---- what each window runs -------------------------------------------------------------------
ORIN_REMOTE='lsusb | grep -qi 03e7 || echo "!! no OAK-D (USB id 03e7) on the Orin: check its cable";
old=$(pgrep -f "^/usr/bin/python3 .*ros2 launch depthai_ros_driver");
if [ -n "$old" ]; then echo "stopping the old OAK-D driver ($old)"; kill -INT $old; sleep 5; fi;
source /opt/ros/foxy/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID='"$OAK_DOMAIN"';
export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>";
echo "starting the OAK-D driver in DDS domain '"$OAK_DOMAIN"' (look for USB SPEED and Camera ready!)";
ros2 launch depthai_ros_driver camera.launch.py use_rviz:=false'
CMD_ORIN="ssh -t ${ORIN_HOST} $(printf %q "bash -lc $(printf %q "$ORIN_REMOTE")")"
CMD_RELAY="ros2 run g1_sensors oak_domain_relay --from-domain ${OAK_DOMAIN} --to-domain 0 --in-reliable"
CMD_TF='ros2 launch g1_sensors tf_chain.launch.py'
CMD_MAP='ros2 launch g1_mapping mapping.launch.py static_tf:=false'
CMD_BRIDGE="ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=${TAG_SIZE} camera:=oak demo_pois:=${DEMO} skeleton:=${SKELETON}"
CMD_RVIZ="rviz2 -d ${RVIZ_CONFIG}"
CMD_SEARCH='ros2 launch semantic_query semantic_query.launch.py'
CMD_CHECKS='cat <<EOF
Checks (this is a shell in the robot container):
  ros2 topic hz /oak/rgb/image_raw                 camera rate
  ros2 topic echo /ar_glasses/anchor_status        the robot measured the wall tag? ("anchored")
  ros2 topic echo /ar_glasses/reply                answers of the object search
  ros2 topic echo /semantic_query/poi              the found object (map frame)
Ask for an object from any host terminal:  bash scripts/g1_search.sh red cup
EOF'

if $PRINT_ONLY; then
  say "Robot container $CONTAINER (g1-humble, robot DDS on $NIC, screen access for RViz)"
  $ORIN && { say "[G1 0 - Orin OAK-D driver, domain $OAK_DOMAIN]"; echo "ssh -t $ORIN_HOST   # then on the Orin:"
             echo "$ORIN_REMOTE" | sed 's/^ *//'; }
  [[ $OAK_DOMAIN != 0 ]] && { say "[G1 0b - OAK relay, domain $OAK_DOMAIN -> 0]"; echo "$CMD_RELAY"; }
  say "[G1 1 - robot TF]"; echo "$CMD_TF"
  say "[G1 2 - map]"; echo "$CMD_MAP"
  $GLASSES && { say "[G1 3 - AR glasses bridge + tag anchor]"; echo "$CMD_BRIDGE"; }
  $RVIZ && { say "[G1 4 - RViz]"; echo "$CMD_RVIZ"; }
  $SEARCH && { say "[G1 6 - object search, container $SEARCH_CONTAINER (g1-semantic, GPU)]"; echo "$CMD_SEARCH"; }
  exit 0
fi

# ---- checks ----------------------------------------------------------------------------------
command -v docker >/dev/null || die "docker is not installed"
command -v gnome-terminal >/dev/null || die "gnome-terminal is required (or follow ROBOT_SESSION.md by hand)"
docker image inspect g1-humble >/dev/null 2>&1 \
  || die "Docker image g1-humble missing: build it once with internet (SIM=1 scripts/run_humble.sh, then exit)"
if $SEARCH; then
  docker image inspect g1-semantic >/dev/null 2>&1 \
    || die "Docker image g1-semantic missing: build it once with internet:
    docker build -t g1-semantic -f docker/Dockerfile.semantic .    (or use --no-search)"
fi
if docker ps -q --filter ancestor=g1-humble | grep -q .; then
  die "a g1-humble container is already running (a second stack would duplicate TF/map).
    Stop it first: bash scripts/stop_g1_session.sh   (or scripts/stop_humble.sh)"
fi

say "1. Robot network ($NIC)"
if ! ip -br addr show "$NIC" 2>/dev/null | grep -q '192\.168\.123\.'; then
  die "$NIC has no 192.168.123.x address. Plug the robot's Ethernet cable, then:
    sudo ip addr add 192.168.123.222/24 dev $NIC
    sudo ip link set $NIC up"
fi
ping -c1 -W2 192.168.123.164 >/dev/null 2>&1 || die "the Orin (192.168.123.164) does not answer on $NIC"
echo "OK: $(ip -br addr show "$NIC" | awk '{print $3}'), Orin answers"

say "2. Clock sync to the robot"
server=$(timedatectl timesync-status 2>/dev/null | awk '/Server:/{print $2}')
[[ $server == 192.168.123.161 ]] && echo "OK: NTP server 192.168.123.161" \
  || warn "laptop clock not synced to the robot (server: ${server:-none}); see ROBOT_SESSION.md"

if $GLASSES; then
  say "3. IP to type into the glasses (Dimensional OS -> Connect)"
  ip -4 -o addr show | awk -v nic="$NIC" '$2!="lo" && $2!=nic && $2!~/^(docker|br-|veth)/ {split($4,a,"/"); print "  " a[1] "   (" $2 ")"}'
fi

# ---- containers ------------------------------------------------------------------------------
say "4. Containers"
xhost +local: >/dev/null 2>&1 || true
COMMON=(--net=host --ipc=host --user "$(id -u):$(id -g)"
        -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro
        -e HOME=/tmp/home -e ROBOT_IFACE="$NIC" -e G1_SIM=0
        -e DISPLAY="${DISPLAY:-}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "$ROOT":/ws)
if [ -d /dev/dri ]; then                      # GPU rendering for RViz (like run_humble.sh)
  COMMON+=(--device /dev/dri)
  for g in video render; do
    gid="$(getent group "$g" | cut -d: -f3)"
    [ -n "$gid" ] && COMMON+=(--group-add "$gid")
  done
fi
docker run -d --rm --name "$CONTAINER" "${COMMON[@]}" g1-humble sleep infinity >/dev/null
echo "started $CONTAINER (robot stack, RViz)"
in_container() { docker exec "$CONTAINER" bash -c "source /ros_entrypoint.sh; $1"; }

# rebuild when asked, when never built, or when a package's sources are newer than its install
PKGS=(g1_sensors g1_mapping g1_ar_bridge semantic_query)
for p in "${PKGS[@]}"; do
  stamp="$ROOT/g1_ws/install/$p/share/$p/package.xml"
  if [[ ! -e $stamp ]] || [[ -n $(find "$ROOT/g1_ws/src/$p" -type f -newer "$stamp" -print -quit) ]]; then
    BUILD=true
  fi
done
if $BUILD; then
  say "   building ${PKGS[*]}"
  in_container "cd /ws/g1_ws && colcon build --packages-select ${PKGS[*]}" \
    || { docker stop "$CONTAINER" >/dev/null; die "build failed"; }
fi

if $SEARCH; then
  GPU=()
  if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' && nvidia-smi >/dev/null 2>&1; then
    GPU=(--gpus all); echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
  else
    warn "no NVIDIA GPU for Docker: the object search runs on the CPU (several seconds per frame)"
  fi
  mkdir -p "$ROOT/bags/hf_cache"
  docker run -d --rm --name "$SEARCH_CONTAINER" "${COMMON[@]}" "${GPU[@]}" g1-semantic sleep infinity >/dev/null
  echo "started $SEARCH_CONTAINER (object search)"
fi

say "5. Who already publishes on the robot network"
tf_pubs=$(in_container 'python3 - <<EOF
import time, rclpy
rclpy.init(); n = rclpy.create_node("g1_session_probe")
end = time.time() + 4.0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
# the OAK-D driver (Orin) publishes its camera frames on /tf: not a robot TF/map owner
tf = [i for i in n.get_publishers_info_by_topic("/tf") if not i.node_name.startswith("oak")]
print(len(tf) + n.count_publishers("/map"))
EOF' 2>/dev/null | tail -1) || tf_pubs=0
if [[ ${tf_pubs:-0} -gt 0 ]]; then
  docker stop "$CONTAINER" >/dev/null; $SEARCH && docker stop "$SEARCH_CONTAINER" >/dev/null
  die "someone already publishes /tf or /map on the robot network (another laptop's stack?).
    Only one TF/map owner may run (AGENTS.md §6)."
fi
echo "OK: no other TF/map owner"

# ---- windows ---------------------------------------------------------------------------------
open_window() {  # title, command[, container | "host"]
  local title=$1 command=$2 where=${3:-$CONTAINER}
  if [[ $where == host ]]; then
    gnome-terminal --window --title="$title" -- bash -c "$command; echo; echo 'finished'; exec bash -i"
  else
    gnome-terminal --window --title="$title" -- docker exec -it "$where" bash -c \
      "source /ros_entrypoint.sh; printf '%s\n\n' \"\$1\"; eval \"\$1\"; echo; echo 'Process finished (no restart). This shell stays open.'; exec bash -i" \
      bash "$command"
  fi
}

say "6. Opening windows"
if $ORIN; then open_window "G1 0 - Orin OAK-D driver, domain $OAK_DOMAIN (type the Orin password)" "$CMD_ORIN" host; fi
[[ $OAK_DOMAIN != 0 ]] && open_window "G1 0b - OAK relay, domain $OAK_DOMAIN -> 0" "$CMD_RELAY"
open_window 'G1 1 - robot TF' "$CMD_TF"; sleep 3
open_window 'G1 2 - map (RTAB-Map)' "$CMD_MAP"; sleep 2
$GLASSES && open_window 'G1 3 - AR glasses bridge + tag anchor' "$CMD_BRIDGE"
$RVIZ && { sleep 2; open_window 'G1 4 - RViz' "$CMD_RVIZ"; }
open_window 'G1 5 - checks' "$CMD_CHECKS"
$SEARCH && open_window 'G1 6 - object search (Grounding DINO + SAM2)' "$CMD_SEARCH" "$SEARCH_CONTAINER"

cat <<EOF

Next:
  1. Window "G1 0": type the Orin password; wait for "Camera ready!" (USB SPEED: SUPER is best).
     If it ends with "Segmentation fault", restart with --oak-domain 78.
  2. Robot: stand it STILL 1-1.5 m in front of the wall tag -> "G1 3" prints: anchored map -> ar_tag_0
  3. Object search: wait for "poi_node up (... device=cuda)" in "G1 6", then from any terminal:
       bash scripts/g1_search.sh red cup
     RViz shows the green 3D box (/ar_glasses/markers) and the 2D box + mask (/semantic_query/image).
  4. Glasses (optional): Dimensional OS -> the IP above -> Registration: AprilTag; type "red cup" there too.
Stop everything: bash scripts/stop_g1_session.sh   (and Ctrl-C in the Orin window)
EOF
