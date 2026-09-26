"""The AR bridge: WebSocket server for the Spectacles Lens (protocol v19), fed by a ``World``.

Registration (the AR world <-> ROS ``map`` transform, ``T_ar_map``):

* ``april_tag`` (Lens "AprilTag" button): the Lens streams camera frames with the glasses'
  pose; the bridge finds the wall tag in them (``T_ar_tag``) and combines it with the tag
  pose the robot camera measured in ``map`` (``World.anchor``, i.e. TF ``map -> ar_tag_<id>``).
  Both can happen at the same time: the bridge keeps collecting glasses views until the robot
  anchor exists, then commits.
* ``manual_pose`` (Lens "Manual Placement"): the marker dragged onto the robot + the robot's
  map pose at commit.

After registration it streams the robot pose, the map cloud, the Nav2 path and POIs/boxes
(``World.annotations``), and polls the glasses' pose (``get_user_hmd_transform``) for
``World.on_hmd_pose``. It never moves the robot: navigation goals and e-stop requests from the
glasses are refused (AGENTS.md §19, §25).
"""
import asyncio
import collections
import concurrent.futures
import itertools
import json
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import websockets

from . import protocol as P
from .alignment import TagPoseEstimator, registration_progress, solve_ar_map
from .annotations import signature, to_wire_args
from .apriltag import TagDetector
from .geometry import (FLIP_YZ, ar_marker_pose, ar_marker_to_T, inv_T, level_ros_pose,
                       pose_to_T, transform_points)

NAV_DISABLED = "Glasses navigation off (safety)"
ESTOP_DISABLED = "Use the robot remote's e-stop"


@dataclass
class BridgeConfig:
    robot_id: str = "unitree_g1"
    display_name: str = "Unitree G1"
    body_bounds_m: tuple = (0.35, 0.50, 1.32)     # [length x, width z, height y]
    footprint_m: tuple = (0.30, 0.25)
    base_height_m: float = 0.78                   # robot_center (= pelvis) above the floor
    tag_id: int = 0
    tag_black_size_m: float = 0.16                # edge of the tag's black square
    min_views: int = 6                            # glasses views of the tag before commit
    min_baseline_m: float = 0.3                   # glasses movement while collecting
    max_view_reproj_px: float = 4.0               # per glasses frame (upstream gates at 3-6 px)
    max_rms_px: float = 3.0                       # multi-view fit, to commit
    max_tilt_deg: float = 10.0                    # "up" disagreement robot tag vs glasses tag
    registration_timeout_s: float = 180.0
    pose_hz: float = 10.0
    lidar_hz: float = 2.0
    hmd_hz: float = 2.0
    lidar_full_cap: int = P.LIDAR_FULL_CAP
    lidar_obstacle_cap: int = P.LIDAR_OBSTACLE_CAP
    lidar_min_height_m: float = 0.05              # above the floor: hides the floor itself
    lidar_max_height_m: float = 2.2
    status_period_s: float = 2.0                  # registration heartbeat (Lens times out 10 s)


class ArBridgeServer:
    def __init__(self, world, config=None, log=print):
        self.world = world
        self.cfg = config or BridgeConfig()
        self.log = log
        self.detector = TagDetector()
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.clients = set()
        self.camera = {}                  # id(ws) -> (K, width, height)
        self.policy_sent = set()
        self.request_ids = itertools.count(1)
        self._warned = set()
        self._last_error = (None, 0.0)
        self.reset()

    # --- state -----------------------------------------------------------------------------

    def reset(self):
        self.committed = False
        self.method = None
        self.approximate = False
        self.T_ar_map = None
        self.reg_info = {}
        self.session = None
        self.lidar_mode = "off"
        self.obstacle_band = (0.1, 0.5, 0.8)
        self.sent_annotations = {}        # id -> signature
        self.pending_skills = {}          # request_id -> ("hmd" | "annotation", id, sent at)
        self.annotation_attempts = {}
        self.pose_history = collections.deque(maxlen=20)   # (t, position, yaw)
        self.last_path = None
        self.last_path_sent = 0.0
        self.hmd_pending_since = None
        self.last_hmd = None              # (T_map_hmd, T_ar_hmd, monotonic time)
        self.frame_busy = False

    def status(self):
        """Summary for logs / ROS status topic."""
        out = {"clients": len(self.clients), "registered": self.committed,
               "method": self.method, "lidar_mode": self.lidar_mode}
        if self.session is not None:
            out["session"] = self.session["mode"]
            out["tag_views"] = self.session.get("views", 0)
        out.update(self.reg_info)
        return out

    # --- outbound --------------------------------------------------------------------------

    def hello(self):
        cfg = self.cfg
        robot = {"robot_id": cfg.robot_id, "display_name": cfg.display_name,
                 "visual_origin_frame": "robot_center",
                 "body_bounds_m": list(cfg.body_bounds_m), "footprint_m": list(cfg.footprint_m),
                 "base_height_m": cfg.base_height_m, "default_render_offset_m": [0.0, 0.0, 0.0],
                 "tag_tracking_profile": {"tag_ids": [cfg.tag_id],
                                          "tag_total_size_m": round(cfg.tag_black_size_m / 0.8,
                                                                    4)}}
        yes = {"available": True, "reason": None}
        caps = {"lidar": yes, "odom": yes, "path": yes,
                "nav": {"available": False, "reason": NAV_DISABLED},
                "navigation": {"available": False, "reason": NAV_DISABLED},
                "emergency_stop": {"available": False, "reason": ESTOP_DISABLED}}
        return P.hello(robot, caps)

    def snapshot(self):
        return P.runtime_snapshot(self.cfg.robot_id, self.committed, self.method,
                                  self.approximate)

    async def send(self, ws, text):
        try:
            await ws.send(text + "\n")
        except websockets.ConnectionClosed:
            pass

    async def broadcast(self, text):
        for ws in list(self.clients):
            await self.send(ws, text)

    async def broadcast_binary(self, data):
        for ws in list(self.clients):
            try:
                await ws.send(data)
            except websockets.ConnectionClosed:
                pass

    def warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.log(text)

    # --- connection ------------------------------------------------------------------------

    async def handle(self, ws, *_):
        self.clients.add(ws)
        peer = getattr(ws, "remote_address", None)
        self.log(f"[ar_bridge] Lens connected from {peer}")
        await self.send(ws, self.hello())
        await self.send(ws, self.snapshot())
        try:
            async for raw in ws:
                if isinstance(raw, (bytes, bytearray)):
                    await self.on_binary(ws, bytes(raw))
                    continue
                for line in raw.splitlines():
                    if line.strip():
                        await self.on_text(ws, line)
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            self.camera.pop(id(ws), None)
            self.policy_sent.discard(id(ws))
            self.log(f"[ar_bridge] Lens disconnected {peer}")
            if not self.clients:
                # a new Lens session has a new AR world: the old registration is meaningless
                if self.committed:
                    self.log("[ar_bridge] last Lens gone: registration cleared")
                self.reset()
                self.world.on_unregistered()

    async def on_text(self, ws, line):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self.warn_once("nonjson", f"[ar_bridge] ignored non-JSON text: {line[:80]!r}")
            return
        if not isinstance(msg, dict):
            return
        kind = msg.get("type")
        handler = getattr(self, f"on_{kind}", None)
        if handler is None:
            self.warn_once(f"type:{kind}", f"[ar_bridge] no handler for {kind!r}")
            return
        try:
            await handler(ws, msg)
        except (KeyError, TypeError, ValueError) as exc:
            self.log(f"[ar_bridge] bad {kind} message: {exc}")

    # --- simple messages -------------------------------------------------------------------

    async def on_ping(self, ws, msg):
        await self.send(ws, P.pong(self.cfg.robot_id, msg.get("client_ts", 0.0)))

    async def on_get_status(self, ws, _msg):
        await self.send(ws, self.snapshot())

    async def on_camera_info(self, ws, msg):
        K = np.array([[float(msg["fx"]), 0.0, float(msg["cx"])],
                      [0.0, float(msg["fy"]), float(msg["cy"])], [0.0, 0.0, 1.0]])
        self.camera[id(ws)] = (K, int(msg["width"]), int(msg["height"]))
        self.log(f"[ar_bridge] camera_info {msg['width']}x{msg['height']} fx={K[0, 0]:.0f}")
        if id(ws) not in self.policy_sent:
            self.policy_sent.add(id(ws))
            # detection range for a tag of >= 20 px, +25 % like upstream
            max_d = K[0, 0] * self.cfg.tag_black_size_m / 20.0 * 1.25
            await self.send(ws, P.capture_policy(max_distance_m=min(max(max_d, 1.0), 8.0),
                                                 min_observations=3))

    async def on_set_lidar_mode(self, _ws, msg):
        self.lidar_mode = msg.get("mode", "off")
        if self.lidar_mode == "obstacles":
            self.obstacle_band = (float(msg.get("obstacle_min_distance_m", 0.1)),
                                  float(msg.get("obstacle_opaque_distance_m", 0.5)),
                                  float(msg.get("obstacle_max_distance_m", 0.8)))
        self.log(f"[ar_bridge] lidar mode {self.lidar_mode}")

    async def on_nav_goal(self, ws, _msg):
        await self.send(ws, P.nav_status("resolved", "failed"))
        await self.send(ws, P.agent_response(
            "Navigation from the glasses is disabled on the G1 (AGENTS.md §19, §25)."))

    async def on_emergency_stop(self, ws, _msg):
        self.log("[ar_bridge] e-stop pressed on the glasses: this bridge cannot stop the robot")
        await self.send(ws, P.agent_response(
            "This bridge cannot stop the robot. Use the e-stop on the robot's remote."))

    async def on_joystick_command(self, _ws, _msg):
        self.warn_once("joystick", "[ar_bridge] joystick commands are ignored (no actuation)")

    async def on_user_command(self, ws, msg):
        text = str(msg.get("text", ""))
        self.log(f"[ar_bridge] user_command {text!r}")
        await self.send(ws, P.agent_status("busy", "thinking"))
        reply = self.world.on_user_command(text) or f"Sent to the robot: {text}"
        await self.send(ws, P.agent_response(reply))
        await self.send(ws, P.agent_status("idle"))

    async def on_ar_skill_result(self, _ws, msg):
        rid = msg.get("request_id")
        entry = self.pending_skills.pop(rid, None)
        if entry is None:
            return
        kind, ann_id = entry[0], entry[1]
        if kind == "hmd":
            self.hmd_pending_since = None
            data = msg.get("data") or {}
            if msg.get("ok") and self.committed and "position" in data:
                T_ar_hmd = pose_to_T(data["position"], data["orientation"])
                T_map_hmd = inv_T(self.T_ar_map) @ T_ar_hmd
                self.last_hmd = (T_map_hmd, T_ar_hmd, time.monotonic())
                self.world.on_hmd_pose(T_map_hmd, T_ar_hmd)
            return
        if msg.get("ok"):
            return
        error = str(msg.get("error"))
        # the Lens creates its annotation presenter when it leaves the wizard: send again
        tries = self.annotation_attempts.get(ann_id, 0)
        self.sent_annotations.pop(ann_id, None)
        if "unavailable" in error and tries < 5:
            self.annotation_attempts[ann_id] = tries + 1
        else:
            self.annotation_attempts[ann_id] = 99      # give up; logged once
            self.warn_once(f"ann:{ann_id}", f"[ar_bridge] annotation {ann_id} failed: {error}")

    # --- registration ----------------------------------------------------------------------

    async def on_registration_command(self, _ws, msg):
        command = msg.get("command")
        if command == "start":
            mode = msg.get("mode")
            if mode == "manual_pose":
                self.session = {"mode": mode, "candidate": None}
                await self.broadcast(P.registration_status(
                    "manual_placement", "Drag the marker onto the robot, then Complete", mode))
            elif mode == "april_tag":
                self.session = {"mode": mode, "started": time.monotonic(), "views": 0,
                                "frames": 0, "last_status": 0.0, "visible": False,
                                "est": TagPoseEstimator(
                                    self.cfg.tag_black_size_m,
                                    max_reproj_px=self.cfg.max_view_reproj_px),
                                "estimate": None, "solution": None}
                await self.broadcast(P.registration_status(
                    "april_tag", self.tag_prompt(), mode, tag_visible=False, progress=0))
            else:
                await self.broadcast(P.registration_status(
                    "failed", f"Unknown registration mode {mode!r}"))
        elif command == "stop":
            self.session = None
            await self.broadcast(P.registration_status("idle", "Registration stopped"))
        elif command == "commit":
            await self.commit_manual()

    def tag_prompt(self):
        return (f"Look at AprilTag {self.cfg.tag_id} on the wall from 1-2 m and move "
                f"sideways slowly")

    async def on_registration_pose(self, _ws, msg):
        if self.session is None or self.session["mode"] != "manual_pose":
            return
        self.session["candidate"] = (list(msg["position"]), list(msg["orientation"]))
        await self.broadcast(P.registration_status(
            "awaiting_commit", "Robot position ready - Complete to confirm", "manual_pose"))

    async def commit_manual(self):
        s = self.session
        if s is None or s["mode"] != "manual_pose" or s.get("candidate") is None:
            await self.broadcast(P.registration_status(
                "manual_placement", "No robot position to commit yet", "manual_pose"))
            return
        T_map_robot = self.world.robot_pose()
        if T_map_robot is None:
            await self.broadcast(P.registration_status(
                "failed", "No robot pose in map (TF map -> robot). Is g1_mapping running?",
                "manual_pose"))
            self.session = None
            return
        T_ar_base, _ = ar_marker_to_T(*s["candidate"])
        T_ar_map = T_ar_base @ inv_T(level_ros_pose(T_map_robot))
        await self.commit(T_ar_map, "manual_pose", approximate=True,
                          info={"method": "manual_pose"})

    async def commit(self, T_ar_map, method, approximate, info, progress=None, confidence=1.0):
        self.T_ar_map = T_ar_map
        self.committed, self.method, self.approximate = True, method, approximate
        self.reg_info = info
        self.session = None
        self.sent_annotations = {}
        self.annotation_attempts = {}
        self.pose_history.clear()
        self.last_path = None
        self.log(f"[ar_bridge] registered ({method}): {info}")
        await self.broadcast(P.bridge_status(True, method, approximate))
        extra = {}
        if method == "april_tag":
            extra = {"progress": 100 if progress is None else progress,
                     "registration_confidence": round(max(0.7, min(1.0, confidence)), 3),
                     "scale_locked": True, "tag_visible": True}
        text = ("Aligned with the robot map" if method == "april_tag"
                else "Manual registration committed")
        await self.broadcast(P.registration_status("succeeded", text, method, **extra))
        self.world.on_registered(T_ar_map, method, info)

    # --- camera frames (AprilTag registration) --------------------------------------------

    async def on_binary(self, ws, data):
        try:
            header, jpeg = P.parse_camera_frame(data)
        except ValueError as exc:
            self.warn_once("badframe", f"[ar_bridge] ignored binary frame: {exc}")
            return
        # the Lens keeps one frame in flight until it is acked: ack every frame, even dropped
        await self.send(ws, P.camera_frame_ack(header.get("seq", 0)))
        s = self.session
        if s is None or s["mode"] != "april_tag" or self.frame_busy:
            return
        cam = self.camera.get(id(ws))
        if cam is None:
            self.warn_once("nocaminfo", "[ar_bridge] camera frame before camera_info: dropped")
            return
        self.frame_busy = True
        try:
            loop = asyncio.get_running_loop()
            result = await loop.run_in_executor(self.executor, self.process_frame, s, header,
                                                jpeg, cam)
        finally:
            self.frame_busy = False
        if self.session is s:
            await self.update_tag_registration(s, result)

    def process_frame(self, s, header, jpeg, cam):
        """Worker thread: decode, detect, single-view PnP, multi-view estimate."""
        K, width, height = cam
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            return {"visible": False, "reason": "JPEG decode failed"}
        if img.shape[1] != width or img.shape[0] != height:
            sx, sy = img.shape[1] / float(width), img.shape[0] / float(height)
            K = np.array([[K[0, 0] * sx, 0, K[0, 2] * sx], [0, K[1, 1] * sy, K[1, 2] * sy],
                          [0, 0, 1.0]])
        s["frames"] += 1
        found = [c for i, c in self.detector.detect(img) if i == self.cfg.tag_id]
        if not found:
            return {"visible": False, "reason": "tag not visible"}
        T_ar_cv = pose_to_T(header["cam_pos"], header["cam_rot"]) @ FLIP_YZ
        view, why = s["est"].make_view(found[0], K, T_ar_cv, stamp=float(header.get("ts", 0)))
        if view is None:
            return {"visible": True, "reason": why}
        s["est"].add(view)
        s["views"] = len(s["est"].views)
        s["estimate"] = s["est"].estimate()
        return {"visible": True, "reason": "ok"}

    async def update_tag_registration(self, s, result):
        if self.session is not s:          # stopped, restarted or already committed
            return
        cfg = self.cfg
        s["visible"] = result["visible"]
        est = s.get("estimate")
        anchor = self.world.anchor(cfg.tag_id)
        progress = registration_progress(est, cfg.min_views, cfg.min_baseline_m)
        sol = None
        if est is not None and anchor is not None:
            sol = solve_ar_map(est.T_world_tag, anchor)
        s["solution"] = sol
        ready = (est is not None and est.n_views >= cfg.min_views and est.rms_px <= cfg.max_rms_px
                 and (est.baseline_m >= cfg.min_baseline_m or est.n_views >= 2 * cfg.min_views))
        confidence = 0.0 if est is None else max(0.0, min(1.0, 1.0 - est.rms_px / (
            2 * cfg.max_rms_px))) * min(1.0, est.n_views / float(cfg.min_views))
        if ready and sol is not None:
            if sol.tilt_deg > cfg.max_tilt_deg:
                self.session = None
                await self.broadcast(P.registration_status(
                    "failed", f"Robot and glasses disagree on 'up' by {sol.tilt_deg:.0f} deg: "
                    "check the robot camera's TF", "april_tag"))
                return
            info = {"method": "april_tag", "tag_id": cfg.tag_id, "views": est.n_views,
                    "rms_px": round(est.rms_px, 2), "baseline_m": round(est.baseline_m, 2),
                    "yaw_deg": round(sol.yaw_deg, 2), "tilt_deg": round(sol.tilt_deg, 2)}
            await self.commit(sol.T_ar_map, "april_tag", approximate=False, info=info,
                              confidence=confidence)
            return
        if ready and anchor is None:
            progress = min(progress, 95)
            message = (f"Glasses ready. Waiting for the robot camera to see tag {cfg.tag_id} "
                       f"(TF map -> ar_tag_{cfg.tag_id})")
        elif not result["visible"]:
            message = self.tag_prompt()
        elif result["reason"] != "ok":
            message = f"Tag seen but not used: {result['reason']}"
        else:
            n = 0 if est is None else est.n_views
            message = f"Tag {cfg.tag_id} seen {n}/{cfg.min_views}: keep moving sideways"
        await self.send_tag_status(s, message, progress, confidence)

    async def send_tag_status(self, s, message, progress, confidence):
        extra = {"tag_visible": bool(s.get("visible")), "progress": int(progress),
                 "registration_confidence": round(confidence, 3)}
        sol = s.get("solution")
        T_map_robot = self.world.robot_pose()
        if sol is not None and T_map_robot is not None:
            pos, quat, _ = ar_marker_pose(sol.T_ar_map @ T_map_robot)
            extra["preview_pose"] = {"position": P.r4(pos), "orientation": P.r4(quat)}
        s["last_status"] = time.monotonic()
        s["last_message"] = message
        s["confidence"] = confidence
        await self.broadcast(P.registration_status("april_tag", message, "april_tag", **extra))

    async def tag_session_tick(self):
        s = self.session
        if s is None or s["mode"] != "april_tag":
            return
        now = time.monotonic()
        if now - s["started"] > self.cfg.registration_timeout_s:
            self.session = None
            if s.get("estimate") is None:
                why = f"Tag {self.cfg.tag_id} not found by the glasses"
            elif self.world.anchor(self.cfg.tag_id) is None:
                why = f"The robot camera has not measured tag {self.cfg.tag_id} (no TF)"
            else:
                why = "Not enough good views of the tag"
            await self.broadcast(P.registration_status("failed", why, "april_tag"))
            return
        if now - s["last_status"] >= self.cfg.status_period_s:
            est = s.get("estimate")
            if est is not None and s.get("solution") is None and \
                    self.world.anchor(self.cfg.tag_id) is not None:
                # the robot anchor appeared after the glasses views: finish now
                await self.update_tag_registration(s, {"visible": s.get("visible"),
                                                       "reason": "ok"})
                return
            progress = registration_progress(est, self.cfg.min_views, self.cfg.min_baseline_m)
            await self.send_tag_status(s, s.get("last_message") or self.tag_prompt(), progress,
                                       s.get("confidence", 0.0))

    # --- runtime streams -------------------------------------------------------------------

    async def send_pose(self):
        T_map_robot = self.world.robot_pose()
        if T_map_robot is None:
            return
        pos, quat, yaw = ar_marker_pose(self.T_ar_map @ T_map_robot)
        now = time.monotonic()
        hist = self.pose_history
        hist.append((now, pos, yaw))
        while len(hist) > 2 and now - hist[1][0] >= 0.5:
            hist.popleft()                                  # keep ~0.5 s for the velocity
        velocity, yaw_rate = [0.0, 0.0, 0.0], 0.0
        t0, p0, y0 = hist[0]
        if now - t0 > 0.05:
            dt = now - t0
            velocity = [(a - b) / dt for a, b in zip(pos, p0)]
            yaw_rate = math.atan2(math.sin(yaw - y0), math.cos(yaw - y0)) / dt
            if math.hypot(velocity[0], velocity[2]) < 0.03:
                velocity = [0.0, 0.0, 0.0]
            if abs(yaw_rate) < 0.05:
                yaw_rate = 0.0
        await self.broadcast(P.pose(pos, quat, velocity, yaw_rate))

    def lidar_ar(self):
        pts = np.asarray(self.world.lidar_points(), dtype=np.float64).reshape(-1, 3)
        if len(pts) == 0:
            return pts
        floor = self.world.floor_z()
        if floor is not None:
            h = pts[:, 2] - floor
            pts = pts[(h >= self.cfg.lidar_min_height_m) & (h <= self.cfg.lidar_max_height_m)]
        if self.lidar_mode == "obstacles":
            T_map_robot = self.world.robot_pose()
            if T_map_robot is not None:
                lo, _, hi = self.obstacle_band
                d = np.hypot(pts[:, 0] - T_map_robot[0, 3], pts[:, 1] - T_map_robot[1, 3])
                pts = pts[(d >= lo) & (d <= hi + 1.0)]
            cap = self.cfg.lidar_obstacle_cap
        else:
            cap = self.cfg.lidar_full_cap
        if len(pts) > cap:
            idx = np.random.default_rng(len(pts)).choice(len(pts), cap, replace=False)
            pts = pts[np.sort(idx)]
        return transform_points(self.T_ar_map, pts)

    async def send_path(self):
        waypoints = self.world.path() or []
        now = time.monotonic()
        key = tuple(np.round(np.asarray(waypoints, dtype=np.float64).reshape(-1), 2).tolist())
        if key == self.last_path and now - self.last_path_sent < 5.0:
            return
        self.last_path, self.last_path_sent = key, now
        pts = transform_points(self.T_ar_map, waypoints) if waypoints else []
        await self.broadcast(P.path([list(p) for p in pts]))

    async def send_annotations(self):
        current = self.world.annotations() or {}
        for ann_id, ann in current.items():
            sig = signature(ann)
            if self.sent_annotations.get(ann_id) == sig:
                continue
            if self.annotation_attempts.get(ann_id, 0) >= 99:
                continue
            rid = f"ann-{next(self.request_ids)}"
            self.pending_skills[rid] = ("annotation", ann_id, time.monotonic())
            self.sent_annotations[ann_id] = sig
            await self.broadcast(P.ar_skill(rid, "draw_world_annotation",
                                            to_wire_args(ann, self.T_ar_map)))
        for ann_id in [a for a in self.sent_annotations if a not in current]:
            self.sent_annotations.pop(ann_id)
            rid = f"ann-{next(self.request_ids)}"
            self.pending_skills[rid] = ("annotation", ann_id, time.monotonic())
            await self.broadcast(P.ar_skill(rid, "draw_world_annotation",
                                            {"id": ann_id, "active": False}))

    async def poll_hmd(self):
        now = time.monotonic()
        if self.hmd_pending_since is not None and now - self.hmd_pending_since < 2.0:
            return
        # forget requests the Lens never answered (e.g. it was busy in a menu)
        for old in [r for r, e in self.pending_skills.items() if now - e[2] > 30.0]:
            self.pending_skills.pop(old)
        rid = f"hmd-{next(self.request_ids)}"
        self.pending_skills[rid] = ("hmd", None, now)
        self.hmd_pending_since = now
        await self.broadcast(P.ar_skill(rid, "get_user_hmd_transform"))

    async def run(self):
        period = 1.0 / self.cfg.pose_hz
        lidar_every = max(1, int(round(self.cfg.pose_hz / max(self.cfg.lidar_hz, 1e-3))))
        hmd_every = max(1, int(round(self.cfg.pose_hz / max(self.cfg.hmd_hz, 1e-3))))
        for tick in itertools.count():
            await asyncio.sleep(period)
            try:
                await self.tick(tick, lidar_every, hmd_every)
            except Exception as exc:          # keep the bridge alive; log the problem
                text, now = repr(exc), time.monotonic()
                if text != self._last_error[0] or now - self._last_error[1] > 5.0:
                    self._last_error = (text, now)
                    self.log(f"[ar_bridge] tick error: {text}")

    async def tick(self, tick, lidar_every, hmd_every):
        if not self.clients:
            return
        await self.tag_session_tick()
        if not self.committed:
            return
        await self.send_pose()
        if self.lidar_mode != "off" and tick % lidar_every == 0:
            await self.broadcast_binary(P.encode_lidar(self.lidar_ar()))
        if tick % 10 == 0:
            await self.send_path()
            await self.send_annotations()
        if self.cfg.hmd_hz > 0 and tick % hmd_every == 0:
            await self.poll_hmd()


async def serve(bridge, host="0.0.0.0", port=P.PORT):
    # max_size=None: camera stills from the Lens can exceed websockets' 1 MiB default
    return await websockets.serve(bridge.handle, host, port, max_size=None)
