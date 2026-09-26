# AR glasses (Snap Spectacles 2024): see the robot's world in AR (AGENTS.md §27)

Goal: someone wearing Spectacles sees what the G1 knows, placed in the real room: the robot, its
path, the LiDAR map, and **semantic POIs / 3D boxes** ("red bottle", "box on the table").

The glasses run the open-source **Dimensional OS** Lens of
[spectacles-dimensional-os](https://github.com/V4C38/spectacles-dimensional-os) (MIT), unchanged.
The Lens is only a viewer: it connects over Wi-Fi to a **bridge** (`ws://<laptop IP>:8787`,
protocol v19) that tells it what to draw. Our bridge is the ROS 2 package
**`g1_ws/src/g1_ar_bridge`**, running on the Ubuntu laptop that is connected to the G1.

```text
Windows + Lens Studio 5.15.4 ──USB-C, once──► Spectacles (the Lens stays in Drafts)

Spectacles ──Wi-Fi, ws://<laptop IP>:8787──► a bridge on a laptop:
   g1_ar_bridge ar_bridge   robot laptop, ROS 2: real robot, map, POIs      (Part 3)
   g1_ar_bridge sim_main    any laptop, no ROS: simulated G1 + your real wall tag (Part 2)
   mock_bridge.py           any laptop, only `websockets`: protocol smoke test  (Part 1)
```

Pinned upstream commit: **`ebf1d38`** (2026-09-26). Lens Studio version of that project:
**5.15.4** (5.15.x is the last Lens Studio line for Spectacles 2024).

## How the glasses find the robot: one AprilTag on a wall, two cameras

The glasses and the robot each have their own world. They are linked by **one printed AprilTag
on a wall** that both see:

```text
robot camera (head RealSense or chest OAK-D) sees the tag  ->  tag pose in the robot's map
Spectacles camera sees the same tag                         ->  tag pose in the glasses' world
                                                            =>  glasses world <-> robot map
```

The robot measures the tag once, while it stands still in front of it (`tag_anchor`, a few
seconds). The glasses measure it when you choose **AprilTag** in the Lens's registration. The
order does not matter. Afterwards, everything in the robot's map (the robot, the LiDAR, POIs,
boxes) is drawn in the right place, and the glasses' position is published on the robot's TF
tree (`map -> ar_world -> spectacles`). Nothing needs to be mounted on the robot. The Lens needs
no change: its AprilTag mode sends camera frames and the glasses' pose to the bridge, and the
bridge does the math. Details: `g1_ws/src/g1_ar_bridge/README.md`.

**The tag:** AprilTag family **36h11, ID 0**. Print it at least 15 cm wide (A4 is ideal).
Keep the white paper around it (do not trim to the black square). Glue it flat on card, tape
it to the wall, and **measure the black square's edge** with a ruler; that number
(`tag_black_size_m`, e.g. `0.16`) goes into the commands below. With the head RealSense (it
looks 48° down), put the tag low on the wall or on the floor ~1 m in front of the robot. With
the chest OAK-D, put it at chest height.

## What is where

| Path | What |
|---|---|
| `../g1_ws/src/g1_ar_bridge/` | **the bridge** (ROS 2 package + `sim_main` without ROS) and its tests |
| `UBUNTU_BRIDGE_SETUP.txt` | hand-out for the robot laptop: network, container, launch, checks |
| `lens_patches/g1-wall-tag-texts.patch` | optional: Lens texts for our setup ("robot laptop" instead of "Mac", wall tag) |
| `mock_bridge/mock_bridge.py` | protocol stand-in: handshake, manual + mock-AprilTag registration, simulated walking, synthetic LiDAR, demo POIs (`--demo-pois`). Only `websockets` |
| `mock_bridge/test_mock_bridge.py` | protocol tests (a WebSocket client playing the Lens) |
| `upstream/` | your clone of spectacles-dimensional-os (git-ignored; see step 3) |

---

## Part 1 — Windows: install, deploy the Lens, test with the mock bridge

Time: ~1 h the first time (downloads). Nothing here touches the robot.

### 1. Install the tools

1. **Git for Windows** (includes Git LFS): <https://git-scm.com/download/win>, default options.
   Then open **Git Bash** once and run:
   ```bash
   git lfs install
   ```
   Git LFS is required: the Lens Studio packages of the upstream project are LFS files. Without
   it you get 130-byte placeholder files and Lens Studio opens a broken project.
2. **Python 3.12** from <https://www.python.org/downloads/windows/>. In the installer tick
   **"Add python.exe to PATH"**.
3. **Lens Studio 5.15.4** (not the newest 5.2x release): <https://ar.snap.com/download/v5-15-4>.
   Needs Windows 10/11 64-bit and a CPU with AVX2. Log in with your Snapchat account when asked.

### 2. Prepare the glasses (once)

1. In the **Spectacles phone app**: update the glasses to the latest Snap OS, then
   **Developer Settings → Lens Development → Enable Wired Connectivity** (persists).
2. Put the glasses on the **same Wi-Fi as the laptop** (Spectacles app → Wi-Fi). The USB cable
   is only for Lens Studio; the Lens talks to the bridge over Wi-Fi.
3. USB: a **USB-C to USB-C** cable into a **native USB-C port** of the laptop, not a hub or a
   dock. Windows shows a **"USB device malfunctioned"** notice every time: known and harmless.

### 3. Get the code

In Git Bash:

```bash
cd ~/Documents
git clone https://github.com/v-prgmr/warsaw_hackathon.git
cd warsaw_hackathon
git checkout claude/ar-glasses

# the upstream Lens project, pinned, inside our (git-ignored) ar_glasses/upstream
git clone https://github.com/V4C38/spectacles-dimensional-os.git ar_glasses/upstream
git -C ar_glasses/upstream checkout ebf1d38
git -C ar_glasses/upstream lfs pull

# check: several MB, not ~130 bytes
ls -la ar_glasses/upstream/lens-studio/Packages/*.lspkg
```

### 4. Open the project and push it to the glasses

1. Plug in the glasses (USB-C) and switch them on.
2. Start **Lens Studio 5.15.4** → **Open Project** →
   `Documents\warsaw_hackathon\ar_glasses\upstream\lens-studio\spectacles-dimensional-os.esproj`.
   If Lens Studio offers to **upgrade** the project, say **no**.
3. Wait until the packages finish loading; the **Logger** panel should show no red errors.
4. With wired connectivity on, Lens Studio connects to the plugged-in glasses by itself (the
   Spectacles device shows as connected in the Preview panel / toolbar).
5. Click **Preview Lens** (the Spectacles button, top right). The Lens is sent to the glasses and
   saved there in **Drafts** as *Dimensional OS*. It stays there when you unplug, reboot, or
   switch the laptop to Ubuntu.
6. The Lens uses **Experimental APIs** (plain `ws://` WebSocket + camera frames). That is why it
   can only be sent from Lens Studio, not published. Accept the camera / internet permission
   prompts on the glasses.

Optional, any time later: our texts in the Lens (the wizard says "Mac" and "tag on the robot"
otherwise). In Git Bash, then repeat step 4.5:

```bash
git -C ar_glasses/upstream apply ../lens_patches/g1-wall-tag-texts.patch
```

### 5. Run the mock bridge

In **PowerShell** (or Git Bash):

```powershell
cd $HOME\Documents\warsaw_hackathon\ar_glasses\mock_bridge
py -m pip install -r requirements.txt       # once; "No module named 'websockets'" = skipped
py mock_bridge.py --demo-pois
```

It prints `Type one of these into the Lens as the Bridge IP: 192.168.x.y`.

**Windows firewall:** the first run shows a Defender prompt → allow on **Private networks**.
Check the Wi-Fi is a *Private* network (Settings → Network & Internet → Wi-Fi → your network →
Private). If the glasses still cannot connect, open the port once in an **admin** PowerShell:

```powershell
netsh advfirewall firewall add rule name="AR bridge 8787" dir=in action=allow protocol=TCP localport=8787
```

Optional self-check (same laptop, second PowerShell):
`py -m pytest` in `ar_glasses\mock_bridge` → `10 passed`.

### 6. Use it on the glasses

**Drafts → Dimensional OS** opens a 3-step wizard:

1. **"Start Robot & Bridge"** (the upstream text says "Run start.sh on your Mac": ignore it):
   the bridge just has to be running on some laptop. Press **Next**.
2. **"Connect"**: type the IP the bridge printed (no port; the Lens always uses `:8787`). The
   bridge prints `Lens connected from ...`. The Lens remembers the IP; enter the new one when
   the laptop changes network or OS.
3. **"Registration"**: the footer button switches between **AprilTag** and **Manual Placement**.
   - **Manual Placement**: drag the robot marker onto a spot on the floor, then **Complete**.
   - **AprilTag** (the real method, Parts 2-3): look at the wall tag from 1-2 m and step
     sideways slowly; the bar fills and the wizard finishes by itself. With the mock bridge
     there is no real detection: it places the robot 1.5 m in front of you after ~12 frames.

Then: **palm up (left hand)** opens the wrist menu. The **LiDAR** button cycles off → obstacles →
full. Re-registration is there too. With the mock and `--demo-pois`: two labelled markers and a
green 3D box appear next to the robot. With the mock only, dragging the navigation marker walks
the simulated robot; our real bridge disables it (safety).

---

## Part 2 — Home test of the wall tag, no robot (Windows / Ubuntu / macOS)

This uses the real bridge code with a **simulated G1** and **your printed tag**, so the tag,
the glasses' camera frames and the math are tested before robot time. Tape the tag to a wall,
then in PowerShell:

```powershell
cd $HOME\Documents\warsaw_hackathon\g1_ws\src\g1_ar_bridge
py -m pip install -r requirements-sim.txt         # once: websockets, numpy, opencv-python
py -m g1_ar_bridge.sim_main --tag-size 0.16       # YOUR black-square size in metres
```

(Ubuntu: `python3 -m pip install --user -r requirements-sim.txt`, then
`python3 -m g1_ar_bridge.sim_main --tag-size 0.16`.)

On the glasses: Dimensional OS → the printed IP → Registration → **AprilTag** → look at the tag
from 1-2 m and step sideways. Expected: *"Tag 0 seen N/6"* while collecting, then the wizard
finishes. The virtual G1 stands **1.5 m in front of the tag, facing it** (`--tag-distance`). The
synthetic room's front wall (LiDAR *full* in the wrist menu) lies on your real wall, and two POIs
and a box sit on a virtual table to the robot's right. If the box floats off the wall, check
`--tag-size` first. The terminal also prints where the glasses are relative to the robot.

## Part 3 — With the robot (Ubuntu laptop on the robot's Ethernet)

The Lens stays on the glasses; only the bridge changes. Summary (full hand-out:
`UBUNTU_BRIDGE_SETUP.txt`):

```bash
scripts/run_humble.sh                                   # robot-connected container (--net=host)
colcon build --packages-select g1_ar_bridge && source install/setup.bash   # in /ws/g1_ws
ros2 launch g1_sensors tf_chain.launch.py               # /tf (AGENTS.md §10.1)
ros2 launch g1_mapping mapping.launch.py static_tf:=false
# + the robot camera driver (head RealSense: realsense-ros with aligned depth)
ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16
```

1. Stand the robot **still, 1-2 m in front of the tag**, facing it, until the log says
   `anchored map -> ar_tag_0` (`ros2 topic echo /ar_glasses/anchor_status`).
2. Glasses: Dimensional OS → the laptop's **Wi-Fi** IP → Registration → **AprilTag** → look at
   the tag, step sideways. The robot box appears on the real robot.
3. POIs: anything published as `visualization_msgs/MarkerArray` on `/ar_glasses/markers` (in
   `map`) shows up; `ros2 run g1_ar_bridge publish_demo_pois` for a first check.

**Never** run the upstream Dimensional OS stack (`launcher/scripts/start.sh`) against the real
G1 next to ours: it is a second robot stack with its own map and it can walk the robot
(AGENTS.md §6, §25).

### Troubleshooting

| Symptom | Fix |
|---|---|
| Lens Studio does not see the glasses | native USB-C port, C-to-C cable, glasses on; "Enable Wired Connectivity" on in the app; restart Lens Studio; wireless fallback: same Wi-Fi + logged in with the same Snapchat account as the app |
| "Lens Studio version not compatible" / push fails | use exactly 5.15.4 and update the glasses' Snap OS in the app |
| Project opens with missing packages / errors | Git LFS was not active: `git -C ar_glasses/upstream lfs pull`, reopen |
| `No module named 'websockets'` | `py -m pip install -r requirements.txt` (use `py -m pip`, not `pip`: same Python as `py`) |
| Lens cannot connect ("WebSocket connection timeout") | bridge running; glasses and laptop on the same Wi-Fi; the laptop's Wi-Fi IP (not 127.0.0.1); firewall rule above / `sudo ufw allow 8787/tcp`; event Wi-Fi often isolates devices → phone hotspot or your own router for both |
| Bridge says `connected` but nothing appears | finish the registration; the robot, LiDAR and POIs appear only after it |
| AprilTag: "Tag not visible" | 1-2 m away, tag well lit, not trimmed, ID 0 of family 36h11; the glasses' camera must see it |
| AprilTag: stuck on "Waiting for the robot camera to see tag 0" | the robot has not anchored the tag yet: stand it still facing the tag; check `/ar_glasses/anchor_status` and TF `map <- <camera optical frame>` |
| AprilTag: "disagree on 'up' by N deg" | the robot camera's TF is wrong (mount / frame): check it in RViz |
| Robot box / POIs offset from the real ones | wrong `tag_black_size_m`, or the tag moved after anchoring; re-anchor and re-register |
| Glasses get hot | LiDAR on *full* is heavy; use *obstacles* or off |

## Conventions (protocol v19)

- AR world frame: metres, **Y up**; the Lens converts to its centimetres. The robot marker's
  local **+X is forward**. Quaternions `[x, y, z, w]`.
- Outbound JSON text frames end with `\n`; LiDAR is binary (`0x01`, float32 ts, float16 xyz);
  camera frames from the Lens are binary `ARF1` envelopes (JSON header + JPEG).
- Port **8787** is fixed in the Lens (`WS_PORT` in `WebSocketTransport.ts`).
