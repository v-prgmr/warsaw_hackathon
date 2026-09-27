# AR glasses + G1: what to build next (ideas and design notes)

Status 2026-09-27. For teammates **and coding agents**: each idea lists what already exists, what
to build, the exact interfaces, the safety rules that apply, and when it counts as done. Read
AGENTS.md (§6 ownership, §19 safety, §25 x-kom rules, §27 AR glasses) before implementing. Setup
and running a session: [`SETUP.md`](SETUP.md). The bridge: [`g1_ar_bridge`](../g1_ws/src/g1_ar_bridge/README.md).

**What already works** (merged, `main`): the glasses and the robot share one frame through a wall
AprilTag; the glasses show the robot (live pose), the RTAB-Map cloud (`/cloud_map`), the Nav2
path (`/plan`) and anything published as `visualization_msgs/MarkerArray` on
`/ar_glasses/markers` (frame `map`: TEXT / SPHERE → labelled marker, CUBE → 3D box, LINE_STRIP /
LINE_LIST → lines, DELETE / DELETEALL). Typed / voice commands from the glasses arrive on
`/ar_glasses/user_command` (`std_msgs/String`). The glasses' pose is in TF
(`map -> ar_world -> spectacles`) and on `/ar_glasses/hmd_pose`. A demo scene (virtual table,
green box, "red bottle") shows the POI look (`demo_pois:=true`).

**Hard rules for every idea** (from AGENTS.md; they override this file):
- The glasses are a viewer. They are **never** the e-stop; the robot remote is (§19, §25).
- Anything that makes the G1 walk is **M5 actuation**: only through the high-level Loco path
  (`g1_loco_cmdvel`, §13), with two people present, someone on the remote's e-stop, battery
  above 20 %, a marked safe area, and after the Nav2 command-only test (§15). It is a **team
  decision**, never an agent's default. Code ships with actuation **off**.
- One owner per TF / topic (§6). Do not add a second `map -> odom` or `/tf` publisher.
- Heavy inference (Grounding DINO, SAM2) never runs in a robot-command process (§25.3).

| # | Idea | Moves the robot? | Needs | Effort |
|---|---|---|---|---|
| 1 | "Find the red bottle" → box on the real object | no | `semantic_query` (M4) | medium–large |
| 2 | Point-to-go: goal from the glasses → Nav2 | plan-only: no; execute: **yes** | bridge change; live = team decision | small (plan) / medium (execute) |
| 3 | See what the robot is thinking (path, frontiers, safe zone, status) | no | small nodes | small |
| 4 | Calibration / validation views (TF axes, camera frustum, measuring) | no | small node; measuring needs a point picker | small–medium |
| 5 | Leo in the shared world (built, provisional); Leo handoff view; "follow me" | Leo view: no; follow: **yes** | Leo camera mount measured; same rules as 2 | started |

Suggested order for the demo: **3 → 2 (plan-only) → 1**, then 2 (execute) if the team approves.

---

## 1. "Find the red bottle" → a labelled box on the real object (M4 showcase)

**Goal.** The wearer says or types "red bottle". The robot finds it in its chest camera and a
labelled 3D box appears **on the real object** in the room. This is the M4 output (AGENTS.md §12,
§16) and exactly what Leo gets in the handoff (M6), so it shows "the robot understands the room".

**Already there**
- Input: the Lens's voice / text box → `/ar_glasses/user_command` (`std_msgs/String`). The bridge
  answers the glasses "Sent to the robot: …".
- Output: `/ar_glasses/markers` draws boxes and labels where the robot's map says.
- Camera: chest OAK-D on the Orin: `/oak/rgb/image_raw`, `/oak/rgb/camera_info`,
  `/oak/stereo/image_raw` (RGB-aligned depth, 1280×720, `16UC1` mm), frame
  `oak_rgb_camera_optical_frame`; TF `map <- oak_rgb_camera_optical_frame` from `g1_sensors` +
  `g1_mapping` (mount: `g1_sensors/config/oakd_livox_taped_20260927.yaml`, ~3 cm).
- The look: the demo scene (`g1_ar_bridge/demo_pois_node.py`) is the template.

**To build: `semantic_query`** (AGENTS.md §12 pipeline)
1. Subscribe to `/ar_glasses/user_command` (and a CLI / service for testing). Parse the query text.
2. Take the latest RGB + aligned depth + CameraInfo + TF `map <- camera` **with the robot
   standing still** (keyframes while walking are blurred, §7). Use the frame's header stamp for
   the TF lookup.
3. Grounding DINO (text → 2D boxes) → SAM2 (mask) → back-project the masked depth with the
   intrinsics → points in the camera frame → transform to `map` → robust centre (median) and an
   oriented or axis-aligned box (percentiles, not min/max).
4. Optional multi-view fusion: merge detections of the same label within ~0.2 m over several
   keyframes; confidence = detector score × views.
5. Publish (a) the POI contract (AGENTS.md §12: id, label, xyz, `frame_id: map`, confidence,
   supporting keyframes), e.g. as a small custom msg or JSON on `/semantic/pois`, and (b) a
   `MarkerArray` on `/ar_glasses/markers` with **`ns: "semantic"`**: one CUBE (box, `text` =
   "red bottle (0.87)") per POI. Update by re-publishing the same `ns` + `id`; clear with DELETE.
6. Reply to the wearer: publish a short status the bridge can show (today the reply is fixed;
   add e.g. `/ar_glasses/reply` → the bridge's `user_command` reply text: small bridge change).

**Where it runs.** The robot laptop has **no CUDA** (Intel Iris Xe). Run Grounding DINO + SAM2 on
a GPU machine on the same network (it only needs the three OAK topics and TF), or offline on
`keyframe_manager` keyframes first (frozen struct, AGENTS.md Day-1 addendum). Never inside a
process that commands the robot.

**Done when**: typing "red bottle" in the glasses makes a box appear on a real bottle, within
~5 cm, with the robot standing 1–3 m away; the same POI is available in `map` for the Leo handoff;
`--no-demo` hides the virtual scene so it is not confused with detections.

---

## 2. Point-to-go: place a goal in the glasses → Nav2

**Goal.** The wearer places the Lens's navigation marker on the floor; the robot plans there (and,
only if approved, walks there).

**Already there**
- The Lens has a navigation marker. It sends `nav_goal` `{position: [x, y, z] m, orientation:
  [x, y, z, w]}` in **its AR world** (Y up), and shows `nav_status` messages: `state` in `idle |
  navIntent | navigating | resolved`, with `outcome: succeeded | failed` when resolved (optional
  `error_code`, `stall_reason: no_path | planner_idle`, `goal`).
- Today the bridge **disables** the marker in the handshake (`capabilities.nav.available:
  false`) and answers every `nav_goal` with `resolved / failed` (`server.py on_nav_goal`).
- The bridge already knows `T_ar_map` after registration and streams `/plan` to the glasses.
- Nav2 on the RTAB-Map map: `g1_nav2 rtabmap_nav_dry_run.launch.py` (velocity only to
  `/g1_nav2_dry_run/cmd_vel`) and `rtabmap_nav_live.launch.py` (→ `/cmd_vel` → `g1_loco_cmdvel`);
  `scripts/start_g1_navigation.sh [--live]` starts either. **Both expose the same action
  `/navigate_to_pose`**, so the bridge cannot tell dry run from live by the action name.

**Design: two stages, a parameter `nav_goals: off | plan | execute` (default `off`)**
- Convert the goal: `T_map_goal = inv(T_ar_map) · T_ar_goal`, keep only x, y and yaw (the goal
  lies on the floor), frame `map`.
- **`plan` (safe, build first):** call the planner-only action `/compute_path_to_pose`
  (`nav2_msgs/action/ComputePathToPose`), never `/navigate_to_pose`. The path goes to the glasses
  (existing `path` stream) and `nav_status` goes `navIntent → resolved / succeeded` (or `failed`
  with `stall_reason: no_path`). The robot **cannot** move in this mode, even if a live Nav2 is
  running. Enable the marker in the handshake only in this mode or above.
- **`execute` (team decision, M5):** `/navigate_to_pose`, `nav_status: navigating` with feedback,
  `resolved` on the result. Required before sending, checked every time: `nav_goals: execute`
  set explicitly by a human; `/battery_state` fresh and ≥ 20 %; the goal inside the prepared safe
  area (a polygon parameter or Nav2 keepout mask); a single goal at a time. A second marker
  placement replaces the goal; "cancel" from the glasses may cancel the goal as a convenience,
  but it is **not** a stop: the remote's e-stop stays the only emergency stop. Also allow the
  glasses' `emergency_stop` message only to cancel the Nav2 goal, and keep saying so in the UI.
- Bridge files: `server.py` (`on_nav_goal`, handshake capabilities, `nav_status`),
  `bridge_node.py` (`RosWorld`: action clients), `config/ar_bridge.yaml`, tests with a fake
  action server.

**Done when (plan):** placing the marker shows Nav2's path on the floor within ~1 s, a goal in a
wall shows "no path", and `ros2 topic info /cmd_vel` has no new publishers.
**Done when (execute, after approval):** the G1 walks to a marker 2 m away inside the safe area,
stops, and `nav_status` resolves `succeeded`; the remote's e-stop interrupts at any time.

---

## 3. See what the robot is thinking (safety and debugging)

**Goal.** Everyone wearing the glasses sees where the robot is going and where it may go, before it
walks. Most useful during M5 exploration.

**Already there:** the Nav2 global path (`/plan`) is drawn; any `MarkerArray` on
`/ar_glasses/markers` is drawn.

**To build**
- **Frontiers:** m-explore (`explore_lite`) publishes its frontiers as a `MarkerArray` on
  `/explore/frontiers` (`visualize: true`; see `rl_hnav/README.md`). Let the bridge draw extra
  MarkerArray topics: make `markers_topic` a list, or relay `/explore/frontiers` → `/ar_glasses/markers` with
  `ns: "frontiers"` (limit to the ~10 largest; the Lens has limited drawing budget).
- **Safe zone / keepout:** draw the prepared safe-area polygon (or the Nav2 keepout mask's
  boundary) as a LINE_STRIP on the floor, `ns: "safety"`, in a distinct colour.
- **Status above the robot:** a TEXT marker following `robot_center` (~0.4 m above the head):
  battery (`/battery_state`), the current goal / Nav2 action status
  (`/navigate_to_pose/_action/status`), explore_lite's state (`/explore/status`; paused with
  `explore/resume`), "stopped". Update at 1–2 Hz; only publish when the text changes.
- **Live LiDAR option:** the bridge shows `/cloud_map`; a parameter to show the current
  `/utlidar/cloud_livox_mid360` scan instead (it transforms any frame to `map`; filter zeros).

**Done when:** during a Nav2 or explore_lite run the glasses show the path, the frontiers, the safe
zone and the robot's status text, all in the right place, without the Lens slowing down.

---

## 4. Calibration and validation views

**Goal.** Check the robot's calibration by eye and measure the map against the room (M2).

**To build**
- **TF axes:** a node that draws chosen frames (e.g. `robot_center`, `livox_frame`,
  `oak_rgb_camera_optical_frame`, `ar_tag_0`) as red/green/blue LINE_LIST triads (0.2 m),
  `ns: "tf_axes"`. If the OAK-D axes do not sit on the real camera, the taped mount has shifted
  (recalibrate: `g1_sensors` README, OAK-D ↔ LiDAR).
- **Camera frustum:** from `/oak/rgb/camera_info` + TF, draw the view pyramid to 2 m; objects the
  robot "sees" should be inside it.
- **Measuring:** pick two points and show the distance, to compare with a tape measure (AGENTS.md
  §10.2). The Lens has no point picker for us today: reuse the navigation marker as a picker in a
  "measure" mode (bridge-side, no Lens change), or a small Lens patch.

**Done when:** the axes of `oak_rgb_camera_optical_frame` appear on the real chest camera, and a
measured wall length agrees with the tape within ±5 cm or ±2 %.

---

## 5. Leo Rover in the shared world, and later ideas

- **Leo in the shared world — built (2026-09-27), provisional.** The same wall tag places Leo in
  the G1 `map` from Leo's own tag detections (`leo_relay` on Leo → UDP → `leo_in_map`: TF
  `map -> leo_odom -> leo_base`; Leo's odometry carries it between sightings) and marks it in
  the glasses (`ns: "leo"`: box, heading, label). Tested in simulation only. To finish: run it
  with the real Leo, **measure Leo's camera mount** (`leo_camera_xyz`, `leo_camera_measured`),
  check the pose against a tape measurement, then show Leo's own path / goal the same way.
  Contract and status: `g1_ws/docs/leo_g1_laptop_integration.md`.
- **Leo handoff view (M6):** show the handoff POI and Leo's planned route in the same `map`, so a
  person confirms the target in the glasses before Leo goes. Needs Leo's path in our frame
  (shared map frame, AGENTS.md M6).
- **"Follow me" / "come here":** the glasses' position is already in TF (`spectacles`,
  `/ar_glasses/hmd_pose`); Nav2 could take the person's floor position (with a stand-off
  distance) as the goal. This makes the robot walk: same rules as idea 2 *execute*, plus a
  maximum speed and never towards a person closer than ~1 m.
- **Persisting the alignment:** the glasses must scan the tag each Lens session (their world is new
  every session). Snap persistent anchors would need a Lens change; the robot side can already
  reuse a saved map + anchor (`SETUP.md` extras).

---

## Pointers for agents

| What | Where |
|---|---|
| Bridge protocol handlers, handshake, registration | `g1_ws/src/g1_ar_bridge/g1_ar_bridge/server.py` |
| ROS side (TF, topics, markers → annotations) | `bridge_node.py`, `ros_util.py` |
| Marker look / demo scene | `demo_pois_node.py`, `annotations.py` |
| Lens protocol reference (upstream, v19) | `ar_glasses/upstream/lens-studio/Assets/Scripts/ARBridge/Network/Protocol.ts` (clone per `ar_glasses/README.md`) |
| Nav2 on RTAB-Map (dry run / live) | `rl_hnav/src/g1_nav2`, `scripts/start_g1_navigation.sh` |
| High-level locomotion (actuation) | `g1_ws/src/g1_loco_cmdvel` (AGENTS.md §13) |
| Tests to extend | `g1_ws/src/g1_ar_bridge/test/` (scripted Lens over a real WebSocket, ROS launch test) |
