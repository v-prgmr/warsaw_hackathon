# Running a G1 session: camera, TF, map, AR glasses, RViz, object search

This is the setup that works on the robot (2026-09-27): the chest OAK-D camera, robot TF, the
RTAB-Map map, the AR glasses bridge with the wall-tag anchor, RViz, and the object search
("red cup" → Grounding DINO + SAM2 → a 3D box). Read-only towards the robot: nothing here moves it.

**One command:** `bash scripts/start_g1_session.sh` (§2). **By hand, terminal by terminal:** §3.
**Ask for an object:** `bash scripts/g1_search.sh red cup` (§4).

```text
  G1 ──Ethernet── laptop enp2s0 (192.168.123.222)    robot data, OAK-D camera (DDS domain 0)
  laptop Wi-Fi ──── same Wi-Fi ──── Spectacles        only for the AR glasses

  Orin:    OAK-D driver ──────────────┐
  laptop:  container g1-robot         ├── domain 0 ──► tf_chain, mapping, glasses bridge, RViz
           container g1-search (GPU)  ┘                object search (Grounding DINO + SAM2)
```

## 1. Once per laptop

```bash
git pull
SIM=1 scripts/run_humble.sh                                     # builds the image g1-humble, then: exit
docker build -t g1-semantic -f docker/Dockerfile.semantic .     # GPU image for the object search
```
Both need internet. The first search start downloads the models (~1 GB) into `bags/hf_cache`.
Clock sync to the robot (AGENTS.md §7): `g1_ws/README.md` §5 or `ar_glasses/SETUP.md` §1.

Before each session, Ethernet to the robot (host terminal):
```bash
sudo ip addr add 192.168.123.222/24 dev enp2s0     # "File exists" is fine
sudo ip link set enp2s0 up
ping -c2 192.168.123.164                           # the Orin answers
```

## 2. One command

```bash
bash scripts/start_g1_session.sh                 # --help for the options
```
It checks the cable, the Orin and the clock, rebuilds the packages when their sources changed (after
a `git pull`), starts the two containers and opens these windows:

| Window | Runs | You do |
|---|---|---|
| G1 0 | on the Orin: OAK-D driver, DDS domain 0 | **type the Orin password**; wait for `Camera ready!` |
| G1 1 | `ros2 launch g1_sensors tf_chain.launch.py` | – |
| G1 2 | `ros2 launch g1_mapping mapping.launch.py static_tf:=false` | – |
| G1 3 | `ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 camera:=oak` | watch for `anchored map -> ar_tag_0` |
| G1 4 | `rviz2 -d …/g1_session.rviz` | look (§6) |
| G1 5 | a shell in the robot container, for checks | – |
| G1 6 | `ros2 launch semantic_query semantic_query.launch.py` (GPU container) | wait for `poi_node up (… device=cuda)` |

Options: `--no-search`, `--no-rviz`, `--no-glasses`, `--no-skeleton`, `--no-orin` (driver already running),
`--demo` (virtual table + green box for the glasses), `--tag-size 0.16`, `--oak-domain 78` (see §6).
Stop: `bash scripts/stop_g1_session.sh`, then Ctrl-C in the Orin window.

## 3. By hand, terminal by terminal

**Orin terminal:** the OAK-D camera driver.
```bash
ssh unitree@192.168.123.164
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress><AllowMulticast>spdp</AllowMulticast></General></Domain></CycloneDDS>'
ros2 launch depthai_ros_driver camera.launch.py use_rviz:=false
```
Look for `USB SPEED: SUPER` (USB 3; `HIGH` = USB 2, much slower) and `Camera ready!`.

**T1 (host):** creates the robot container, with screen access so RViz works in every terminal.
```bash
cd ~/alien_hack/warsaw_hackathon
scripts/stop_humble.sh                    # old containers, if any
ROBOT_IFACE=enp2s0 scripts/run_humble.sh
```
Once, after a `git pull`, inside T1:
```bash
cd /ws/g1_ws && colcon build --packages-select g1_sensors g1_mapping g1_ar_bridge semantic_query && source install/setup.bash
```

**T2, T3, T4, T5 (host):** each joins the same container.
```bash
docker exec -it $(docker ps -q --filter ancestor=g1-humble | head -1) bash
```

| Terminal | Command |
|---|---|
| T1 | `ros2 launch g1_sensors tf_chain.launch.py` |
| T2 | `ros2 launch g1_mapping mapping.launch.py static_tf:=false` |
| T3 | `ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 camera:=oak` |
| T4 | `rviz2 -d /ws/g1_ws/src/g1_ar_bridge/rviz/g1_session.rviz` |
| T5 | free: checks and questions (§4) |

**T6 (host):** the object search, in its GPU container (the robot container has no PyTorch).
```bash
cd ~/alien_hack/warsaw_hackathon
docker run -it --rm --name g1-search --net=host --ipc=host --gpus all \
  --user "$(id -u):$(id -g)" -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -e HOME=/tmp/home -e ROBOT_IFACE=enp2s0 -e G1_SIM=0 -v "$PWD":/ws g1-semantic \
  bash -c 'source /ros_entrypoint.sh; ros2 launch semantic_query semantic_query.launch.py'
```

## 4. Asking for objects

From **any host terminal** (it answers when found or after 20 s):
```bash
bash scripts/g1_search.sh red cup
bash scripts/g1_search.sh where is my bottle
bash scripts/g1_search.sh stop        # stop searching
bash scripts/g1_search.sh clear       # remove the boxes
```
```text
> red cup
Searching for red cup…
Found red cup (0.71), 1.8 m from the robot. Box 8 x 8 x 11 cm.
  in map: xyz [1.52, -0.31, 0.02], box [8, 8, 11] cm, confidence 0.71  (/semantic_query/poi)
```
From a terminal **inside the robot container** (T5):
```bash
ros2 topic pub --once /semantic_query/query std_msgs/msg/String "{data: 'red cup'}"
ros2 topic echo /ar_glasses/reply          # answers
ros2 topic echo /semantic_query/poi        # the found object: label, xyz in map, box, confidence
```
From the **AR glasses, by voice** (the glasses have no keyboard):
1. Open the menu (look at your left palm) and switch the mode from **Manual** to **Agent**.
2. Say the wake word **"robot"** and then the request, in one go: **"Robot, find the red cup."**
   Everything after "robot" goes to the search. For the next 30 s you can talk without the wake
   word: "where is my bottle", "clear".
3. The answer appears in the glasses' panel ("Searching for red cup…", "Found red cup (0.71)…"), and
   the object gets a **green 3D box with a label pin** ("red cup (0.71)") on top.

| Say | Does |
|---|---|
| "Robot, find the red cup" / "Robot, where is my bottle" | search (20 s) |
| "cancel" / "never mind" | stop the search |
| "clear" | remove the boxes |
| "help" | the glasses list what you can say |

Do **not** say "stop": any phrase with "stop" is the Lens's **emergency stop**, which is switched off
for the glasses (the robot remote is the e-stop) and never reaches the search. Use "cancel".

The search tries new camera frames for up to 20 s (~1 s per frame on the GPU): put the object
0.5–3 m in front of the G1's chest camera, or turn the robot towards it while it searches.

## 5. What the glasses show

Once registered on the tag (Dimensional OS → Registration → AprilTag):

| Thing | What |
|---|---|
| **the robot** | its marker follows the G1; a pin above it: **"Unitree G1 · 70%"** (battery) |
| **the robot's skeleton** | its links from the URDF + TF as a stick figure that moves with the joints (`ar_skeleton`; `--no-skeleton` to hide) |
| **the wall tag** | a pin **"AprilTag 0"** |
| **lines on the floor** | the room's walls from the 2D map, traced on the floor (updated as the map grows) |
| **found objects** | a green 3D box + a label pin ("red cup (0.71)"); "searching: red cup" above the robot while it looks |
| **3D LiDAR map** | wrist menu → LiDAR **full**: up to 1500 points, only where you look (a Lens limit) |
| robot bounding box | wrist menu → **Debug Mode** (the Lens has no G1 3D model) |

Everything comes from `/ar_glasses/markers` (labels, pins, boxes, lines; `g1_ar_bridge` README), so
RViz shows the same.

## 6. RViz

`g1_session.rviz` (Fixed Frame `map`) already shows:

| Display | Topic |
|---|---|
| G1 OAK-D camera | `/oak/rgb/image_raw` |
| Object search (2D box + mask) | `/semantic_query/image` (only during a search) |
| Glasses markers: search boxes, demo table, Leo | `/ar_glasses/markers` |
| 3D map | `/g1_mapping/cloud_map_3d` (`/cloud_map` is flat) |
| 2D map, odometry, robot model, TF, LiDAR, frontiers | `/map`, `/odom`, `/robot_description`, … |

With navigation (`scripts/start_g1_navigation.sh`, `start_g1_explore_navigation.sh`) there is one RViz
for both: `g1_nav2/rviz/g1_session_nav.rviz` = this config + Nav2 costmaps, `/plan`, footprint, `/scan`,
`/explore/frontiers`, the **Nav2 Goal** tool and the Navigation 2 panel. Joined to this session, the
navigation launcher closes G1 4's RViz and opens the unified one (G1 6); `stop_g1_navigation.sh`
leaves it open for the session.

Adding displays by hand: a display shows nothing when its QoS does not match the publisher. Camera
images from the driver are Reliable (RViz's default works); the map clouds are latched (set
**Durability Policy: Transient Local** if they stay empty).

## 7. When something does not work

| Symptom | Fix |
|---|---|
| G1 0 ends with `Segmentation fault` | the Orin's Foxy driver sometimes crashes on domain 0 (Unitree DDS traffic). Restart with `--oak-domain 78`: the driver runs in domain 78 and `g1_sensors oak_domain_relay --in-reliable` copies it into domain 0 |
| Camera slow / laggy | the driver says `USB SPEED: HIGH`: use a USB 3 port (blue) and a USB 3 cable (`SUPER`). With the relay: `--in-reliable` (Best Effort drops whole frames) |
| Camera topics listed but no images | the driver stalled (e.g. after moving the robot or the cable): Ctrl-C in G1 0 and start it again |
| G1 3 stays `searching` | the tag is not readable in the robot camera: glare/sun on it, cut off at the image edge, or > 3 m away. Tag at chest height, printed side to the room, G1 1–1.5 m in front |
| Voice does nothing | Agent mode on (menu)? Start with "robot, …"; the Lens shows what it heard. "stop" never reaches the search: say "cancel" |
| `g1_search.sh`: "The object search is not running" | wait for `poi_node up` in G1 6 (the first start downloads the models) |
| Search finds wrong things | raise `min_confidence` in `g1_ws/src/semantic_query/config/semantic_query.yaml` |
| "someone already publishes /tf or /map" | another laptop runs the robot stack: only one may (AGENTS.md §6) |

More: `ar_glasses/SETUP.md` (glasses), `g1_ws/src/semantic_query/README.md` (search),
`g1_ws/src/g1_ar_bridge/README.md` (bridge), `g1_ws/src/g1_sensors/README.md` (OAK-D relay).
