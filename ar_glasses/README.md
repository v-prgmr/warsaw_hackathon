# AR glasses (Snap Spectacles 2024): see the robot's world in AR (AGENTS.md §27)

Goal: someone wearing Spectacles sees what the G1 knows, placed in the real room: the robot, its
path, the LiDAR map, and **semantic POIs / 3D boxes** ("red bottle", "box on the table").

We start from the open-source Lens of
[spectacles-dimensional-os](https://github.com/V4C38/spectacles-dimensional-os) (MIT; Go2 tested,
G1 "supported, not tested"). Its Lens talks to a bridge over a WebSocket (protocol v19,
`dimos-ar/PROTOCOL.md` upstream) and aligns the glasses with the robot using AprilTags on the robot.

```text
Windows + Lens Studio 5.15.4 ──USB-C (only to install the Lens)──► Spectacles (Lens stays in Drafts)

Spectacles ──Wi-Fi, ws://<laptop IP>:8787──► bridge on the laptop
                                             1. mock_bridge.py   (this folder; Windows or Ubuntu, no robot)
                                             2. upstream dimos-ar bridge (Ubuntu/macOS, ROBOT_IP=fake)
                                             3. g1_ar_bridge (ROS 2, our stack: RTAB-Map map, TF, POIs) — next
```

Pinned upstream commit: **`ebf1d38`** (2026-09-26). Lens Studio version of that project:
**5.15.4** (5.15.x is the last Lens Studio line for Spectacles 2024).

## What is where

| Path | What |
|---|---|
| `mock_bridge/mock_bridge.py` | stand-in bridge: handshake, registration (manual, or a mock AprilTag flow), a simulated G1 that walks to goals, synthetic LiDAR room, demo POIs + a 3D box (`--demo-pois`). Python 3.9+ and `websockets` only |
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

### 5. Run the mock bridge

In **PowerShell** (or Git Bash):

```powershell
cd $HOME\Documents\warsaw_hackathon\ar_glasses\mock_bridge
py -m pip install -r requirements.txt
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

1. On the glasses open **Drafts → Dimensional OS**.
2. The **Registration Wizard** asks for the **Bridge IP**: type the IP the mock printed (no port;
   the Lens always uses `:8787`). The mock prints `Lens connected from ...`. The Lens remembers
   this IP; when the laptop's IP changes (other network, other OS), enter the new one.
3. Register the robot position:
   - **Manual Placement** (recommended with the mock): drag the robot marker onto a spot on the
     floor, then commit. A G1-sized box appears there.
   - **AprilTag** also works with the mock: there is no real tag detection; after ~12 camera
     frames it places the robot 1.5 m in front of you, facing you.
4. **Palm up (left hand)** opens the wrist menu: the **LiDAR** button cycles off → obstacles →
   full (a synthetic 6 × 5 m room with a table). Emergency stop and re-registration are there too.
5. **Navigation:** drag the navigation marker somewhere: the simulated robot walks there
   (0.4 m/s) along a yellow path, then reports success. Only the simulation moves.
6. With `--demo-pois`: two labelled markers (*red bottle (0.87)*, *box on the table (0.74)*) and a
   green 3D box appear next to the robot. This is exactly how our semantic POIs will be shown:
   the Lens's `draw_world_annotation` skill, no Lens change needed.

### Troubleshooting

| Symptom | Fix |
|---|---|
| Lens Studio does not see the glasses | native USB-C port, C-to-C cable, glasses on; "Enable Wired Connectivity" on in the app; restart Lens Studio; wireless fallback: same Wi-Fi + logged in with the same Snapchat account as the app |
| "Lens Studio version not compatible" / push fails | use exactly 5.15.4 and update the glasses' Snap OS in the app |
| Project opens with missing packages / errors | Git LFS was not active: `git -C ar_glasses/upstream lfs pull`, reopen |
| Lens cannot connect to the bridge | glasses and laptop on the same Wi-Fi; the laptop's IP (not 127.0.0.1); firewall rule above; event Wi-Fi often isolates devices → use a phone hotspot or your own router for both |
| Mock says `connected` but nothing appears | finish the registration (commit); the robot, LiDAR and POIs appear only after it |
| Glasses get hot | LiDAR on *full* is heavy; use *obstacles* or off |

---

## Part 2 — Ubuntu: same glasses, same Lens

The Lens stays on the glasses. On Ubuntu only the bridge changes:

```bash
cd ~/warsaw_hackathon && git checkout claude/ar-glasses
python3 -m pip install --user websockets
python3 ar_glasses/mock_bridge/mock_bridge.py --demo-pois   # type this laptop's IP into the Lens
```

Then, in order:

1. **Upstream bridge without a robot** (Dimensional OS, Ubuntu is its main platform): in
   `ar_glasses/upstream`, `./launcher/scripts/setup.sh --stack g1`, then
   `ROBOT_IP=fake ./launcher/scripts/start.sh` (Python 3.12; the author tested on macOS only).
   **Never** point it at the real G1 while our stack runs: it is a second robot stack with its
   own map and it can walk the robot (AGENTS.md §6, §25).
2. **`g1_ar_bridge`** (next step, ROS 2): the same protocol, fed by our stack. Robot pose from
   `map -> torso_link` (RTAB-Map + `g1_sensors`), LiDAR from `/cloud_map`, path from Nav2, POIs
   and 3D boxes from `semantic_query` as `draw_world_annotation`. It reuses upstream's AprilTag
   alignment code (MIT). Navigation goals from the glasses stay **off** by default (§19, §25).
3. Lens changes (new message types, our own UI) are done in Lens Studio on Windows and pushed
   again; the bridge side stays on Ubuntu.

## Conventions (protocol v19)

- AR world frame: metres, **Y up**; the Lens converts to its centimetres. The robot marker's
  local **+X is forward**. Quaternions `[x, y, z, w]`.
- Outbound JSON text frames end with `\n`; LiDAR is binary (`0x01`, float32 ts, float16 xyz);
  camera frames from the Lens are binary `ARF1` envelopes (JSON header + JPEG).
- Port **8787** is fixed in the Lens (`WS_PORT` in `WebSocketTransport.ts`).
