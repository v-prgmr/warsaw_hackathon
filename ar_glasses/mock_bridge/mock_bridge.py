#!/usr/bin/env python3
"""Mock AR bridge for the Spectacles Lens of spectacles-dimensional-os (protocol v19).

Runs on Windows, macOS and Ubuntu with only Python 3.9+ and `websockets`, no robot, no ROS,
no Dimensional OS. It stands in for the real bridge so the Lens can be deployed and tested
end to end: handshake, clock ping, registration (manual placement, or a mock AprilTag flow that
completes after a few camera frames), a simulated robot that walks to navigation goals you
place, a synthetic LiDAR room, and demo POI markers / a 3D box via the Lens's
`draw_world_annotation` skill (how our semantic POIs will be shown later).

    pip install websockets
    python mock_bridge.py                 # listens on 0.0.0.0:8787, prints the IP to type in
    python mock_bridge.py --demo-pois     # + labelled POI markers and a box after registration

Coordinates are the AR world frame of the protocol: metres, Y up (the Lens converts to cm).
The robot marker's local +X is its forward direction. Nothing here can move a real robot.
Protocol reference: upstream dimos-ar/PROTOCOL.md (pinned commit in ../README.md).
"""
import argparse
import asyncio
import json
import math
import random
import socket
import struct
import time

import websockets

PROTOCOL_VERSION = 19
PORT = 8787
ROBOT_ID = "unitree_g1"
POSE_HZ = 10.0
LIDAR_HZ = 4.0
WALK_SPEED_MPS = 0.4
GOAL_TOLERANCE_M = 0.05
APRILTAG_MOCK_FRAMES = 12
DEMO_POI_DELAY_S = 2.0   # let the Lens leave the registration wizard before annotating
DEMO_POI_RETRIES = 3
LIDAR_FULL_CAP = 1500
LIDAR_OBSTACLE_CAP = 200

# Unitree G1 geometry as in upstream dimos-ar robot_profile/g1.py
G1_ROBOT = {
    "robot_id": ROBOT_ID,
    "display_name": "Unitree G1 (mock bridge)",
    "visual_origin_frame": "base_link",
    "body_bounds_m": [0.65, 0.45, 1.35],
    "footprint_m": [0.32, 0.24],
    "base_height_m": 0.95,
    "default_render_offset_m": [0.0, 0.0, 0.0],
    "tag_tracking_profile": {"tag_ids": [0, 1], "tag_total_size_m": 0.0875},
}


def dumps(payload):
    return json.dumps(payload, separators=(",", ":"), allow_nan=False)


def yaw_quat(yaw):
    """Quaternion (x, y, z, w) of a rotation by `yaw` about the world-up Y axis."""
    return [0.0, math.sin(yaw / 2.0), 0.0, math.cos(yaw / 2.0)]


def yaw_of(q):
    """Heading of the marker's +X axis about Y (inverse of yaw_quat for yaw-only quaternions)."""
    x, y, z, w = q
    # rotated +X axis: first column of the rotation matrix
    ax = 1 - 2 * (y * y + z * z)
    az = 2 * (x * z - w * y)
    return math.atan2(-az, ax)


def forward(yaw):
    """World direction of the marker's +X axis for a yaw about Y."""
    return math.cos(yaw), -math.sin(yaw)  # (x, z)


def rotate_by_quat(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w): v + 2w(u x v) + 2u x (u x v)."""
    x, y, z, w = q
    ux, uy, uz = x, y, z
    vx, vy, vz = v
    cx, cy, cz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    ccx, ccy, ccz = uy * cz - uz * cy, uz * cx - ux * cz, ux * cy - uy * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def left_of(fx, fz):
    """Left of a horizontal forward (fx, fz) in the Y-up world: up x forward = (fz, -fx)."""
    return fz, -fx


def encode_lidar(ts, points):
    out = bytearray(struct.pack("<Bf", 0x01, float(ts)))
    for p in points:
        out += struct.pack("<eee", *p)
    return bytes(out)


def parse_camera_frame(data):
    """ARF1 envelope: magic, uint32 header length, JSON header, JPEG. Returns the header."""
    if len(data) < 8 or data[:4] != b"ARF1":
        return None
    (hlen,) = struct.unpack("<I", data[4:8])
    try:
        return json.loads(data[8:8 + hlen].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


class MockBridge:
    def __init__(self, demo_pois=False, log=print):
        self.log = log
        self.demo_pois = demo_pois
        self.clients = set()
        self.reset()

    def reset(self):
        self.registered = False
        self.method = None           # "manual_pose" | "april_tag"
        self.approximate = False
        self.reg_mode = None         # active registration session mode
        self.candidate = None        # (position, yaw) awaiting commit
        self.tag_frames = 0
        self.last_cam = None         # (cam_pos, cam_rot) from the latest camera frame
        self.pos = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.room_center = None
        self.goal = None             # dict: position, orientation, source
        self.nav_state = "idle"
        self.nav_outcome = None
        self.speed = 0.0
        self.velocity = [0.0, 0.0, 0.0]
        self.lidar_mode = "off"
        self.obstacle_band = (0.1, 0.5, 0.8)
        self.sent_policy = set()
        self.annotations = []
        self.poi_retries = 0

    # --- outbound -------------------------------------------------------------------------

    def bridge_wire(self):
        return {"robot_connected": True, "reconnecting": False,
                "world_frame_committed": self.registered,
                "world_frame_method": self.method if self.registered else None,
                "world_frame_approximate": self.approximate if self.registered else False}

    def nav_wire(self):
        nav = {"state": self.nav_state}
        if self.nav_outcome:
            nav["outcome"] = self.nav_outcome
        if self.goal is not None and self.nav_state in ("navIntent", "navigating"):
            nav["goal"] = self.goal
        return nav

    def hello(self):
        caps = {name: {"available": True} for name in
                ("lidar", "odom", "nav", "path", "navigation")}
        caps["emergency_stop"] = {"available": True}
        return dumps({"type": "hello", "protocol_version": PROTOCOL_VERSION, "robot": G1_ROBOT,
                      "capabilities": caps})

    def snapshot(self):
        payload = {"type": "runtime_snapshot", "ts": time.time(), "robot_id": ROBOT_ID,
                   "bridge": self.bridge_wire(), "nav": self.nav_wire(),
                   "agent": {"state": "idle"}}
        if self.goal is not None and self.nav_state == "navigating":
            payload["path"] = {"waypoints": self.path_points()}
        return dumps(payload)

    def registration_status(self, state, message, mode=None, **extra):
        payload = {"type": "registration_status", "ts": time.time(), "state": state,
                   "message": message}
        if mode:
            payload["mode"] = mode
        payload.update(extra)
        return dumps(payload)

    def pose(self):
        return dumps({"type": "pose", "ts": round(time.time(), 3),
                      "position": [round(v, 4) for v in self.pos],
                      "orientation": [round(v, 4) for v in yaw_quat(self.yaw)],
                      "speed_mps": round(self.speed, 4),
                      "velocity_mps": [round(v, 4) for v in self.velocity],
                      "yaw_rate_rad_s": 0.0})

    def path_points(self):
        if self.goal is None:
            return []
        g = self.goal["position"]
        return [[round(self.pos[i] + (g[i] - self.pos[i]) * k / 4.0, 3) for i in range(3)]
                for k in range(5)]

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

    # --- inbound --------------------------------------------------------------------------

    async def handle(self, ws, *_):
        self.clients.add(ws)
        peer = getattr(ws, "remote_address", None)
        self.log(f"[mock] Lens connected from {peer}")
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
            self.log(f"[mock] Lens disconnected {peer}")

    async def on_text(self, ws, line):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self.log(f"[mock] ignored non-JSON text: {line[:80]!r}")
            return
        kind = msg.get("type")
        handler = getattr(self, f"on_{kind}", None)
        if handler is None:
            self.log(f"[mock] no handler for {kind!r}")
            return
        await handler(ws, msg)

    async def on_ping(self, ws, msg):
        now = time.time()
        await self.send(ws, dumps({"type": "pong", "ts": now, "robot_id": ROBOT_ID,
                                   "client_ts": msg.get("client_ts", 0.0), "bridge_ts": now}))

    async def on_get_status(self, ws, _msg):
        await self.send(ws, self.snapshot())

    async def on_camera_info(self, ws, msg):
        self.log(f"[mock] camera_info {msg.get('width')}x{msg.get('height')}")
        if id(ws) not in self.sent_policy:
            self.sent_policy.add(id(ws))
            await self.send(ws, dumps({"type": "capture_policy", "ts": time.time(),
                                       "max_capture_distance_m": 2.5,
                                       "min_capture_distance_m": 0.35,
                                       "max_capture_speed_mps": 0.45,
                                       "static_speed_mps": 0.05, "min_observations": 3}))

    async def on_binary(self, ws, data):
        header = parse_camera_frame(data)
        if header is None:
            self.log(f"[mock] ignored binary frame of {len(data)} bytes")
            return
        await self.send(ws, dumps({"type": "camera_frame_ack", "ts": time.time(),
                                   "seq": header.get("seq", 0),
                                   "capturing_budgeted_complete": False}))
        if "cam_pos" in header and "cam_rot" in header:
            self.last_cam = (header["cam_pos"], header["cam_rot"])
        if self.reg_mode == "april_tag":
            await self.mock_tag_progress()

    async def mock_tag_progress(self):
        """No tag detection here: each camera frame advances a fake registration that places
        the robot 1.5 m in front of the glasses, 1.2 m below them (roughly on the floor)."""
        self.tag_frames += 1
        progress = min(100, int(100 * self.tag_frames / APRILTAG_MOCK_FRAMES))
        preview = None
        if self.last_cam is not None:
            (cx, cy, cz), rot = self.last_cam
            fx, _, fz = rotate_by_quat(rot, (0.0, 0.0, -1.0))  # camera looks along -Z
            norm = math.hypot(fx, fz) or 1.0
            fx, fz = fx / norm, fz / norm
            position = [cx + 1.5 * fx, cy - 1.2, cz + 1.5 * fz]
            yaw = math.atan2(fz, -fx)  # robot faces the user: +X points back at the camera
            preview = (position, yaw)
        if self.tag_frames < APRILTAG_MOCK_FRAMES or preview is None:
            extra = {"tag_visible": True, "progress": progress}
            if preview is not None:
                extra["preview_pose"] = {"position": preview[0],
                                         "orientation": yaw_quat(preview[1])}
            await self.broadcast(self.registration_status(
                "april_tag", "Mock bridge: keep looking around (no real tag detection)",
                "april_tag", **extra))
            return
        await self.commit(preview[0], preview[1], method="april_tag", approximate=True)

    async def on_registration_command(self, _ws, msg):
        command = msg.get("command")
        if command == "start":
            self.reg_mode = msg.get("mode")
            self.candidate = None
            self.tag_frames = 0
            if self.reg_mode == "manual_pose":
                await self.broadcast(self.registration_status(
                    "manual_placement", "Place the robot marker, then commit", "manual_pose"))
            else:
                await self.broadcast(self.registration_status(
                    "april_tag", "Mock bridge: look around; completes after a few frames",
                    "april_tag", tag_visible=False, progress=0))
        elif command == "stop":
            self.reg_mode = None
            self.candidate = None
            await self.broadcast(self.registration_status("idle", "Registration stopped"))
        elif command == "commit":
            if self.candidate is None:
                await self.broadcast(self.registration_status(
                    "manual_placement", "No robot pose to commit yet", "manual_pose"))
                return
            position, yaw = self.candidate
            await self.commit(position, yaw, method="manual_pose", approximate=True)

    async def on_registration_pose(self, _ws, msg):
        if self.reg_mode != "manual_pose":
            return
        self.candidate = (list(msg["position"]), yaw_of(msg["orientation"]))
        await self.broadcast(self.registration_status(
            "awaiting_commit", "Manual robot pose ready — review and commit", "manual_pose"))

    async def commit(self, position, yaw, method, approximate):
        self.pos, self.yaw = [float(v) for v in position], yaw
        self.room_center = (self.pos[0], self.pos[1], self.pos[2], yaw)
        self.registered, self.method, self.approximate = True, method, approximate
        self.reg_mode, self.candidate = None, None
        self.log(f"[mock] registered ({method}) at {[round(v, 2) for v in self.pos]}")
        await self.broadcast(dumps({"type": "bridge_status", "ts": time.time(),
                                    **self.bridge_wire()}))
        await self.broadcast(self.registration_status(
            "succeeded", "Manual registration committed" if method == "manual_pose"
            else "Registration successful (mock)", method))
        if self.demo_pois:
            self.poi_retries = 0
            asyncio.ensure_future(self.send_demo_pois(delay=DEMO_POI_DELAY_S))

    async def on_nav_goal(self, _ws, msg):
        if not self.registered:
            await self.broadcast(dumps({"type": "nav_status", "ts": time.time(),
                                        "state": "resolved", "outcome": "failed"}))
            return
        self.goal = {"source": "user", "position": [float(v) for v in msg["position"]],
                     "orientation": msg.get("orientation", yaw_quat(self.yaw))}
        self.nav_state, self.nav_outcome = "navigating", None
        self.log(f"[mock] nav goal {[round(v, 2) for v in self.goal['position']]}")
        await self.broadcast(dumps({"type": "nav_status", "ts": time.time(), **self.nav_wire()}))
        await self.broadcast(dumps({"type": "path", "ts": round(time.time(), 3),
                                    "waypoints": self.path_points()}))

    async def on_emergency_stop(self, _ws, _msg):
        self.log("[mock] emergency stop")
        await self.finish_nav("failed")

    async def finish_nav(self, outcome):
        self.goal, self.speed, self.velocity = None, 0.0, [0.0, 0.0, 0.0]
        self.nav_state, self.nav_outcome = "resolved", outcome
        await self.broadcast(dumps({"type": "nav_status", "ts": time.time(), **self.nav_wire()}))
        self.nav_state, self.nav_outcome = "idle", None

    async def on_set_lidar_mode(self, _ws, msg):
        self.lidar_mode = msg.get("mode", "off")
        if self.lidar_mode == "obstacles":
            self.obstacle_band = (msg.get("obstacle_min_distance_m", 0.1),
                                  msg.get("obstacle_opaque_distance_m", 0.5),
                                  msg.get("obstacle_max_distance_m", 0.8))
        self.log(f"[mock] lidar mode {self.lidar_mode}")

    async def on_joystick_command(self, _ws, _msg):
        pass

    async def on_user_command(self, ws, msg):
        text = msg.get("text", "")
        self.log(f"[mock] user_command {text!r}")
        await self.send(ws, dumps({"type": "agent_status", "ts": time.time(), "state": "busy",
                                   "detail": "thinking"}))
        await self.send(ws, dumps({"type": "agent_response", "ts": time.time(),
                                   "text": "Mock bridge: no agent here. I heard: " + text}))
        await self.send(ws, dumps({"type": "agent_status", "ts": time.time(), "state": "idle"}))

    async def on_ar_skill_result(self, _ws, msg):
        if msg.get("ok", False):
            return
        error = str(msg.get("error"))
        self.log(f"[mock] ar_skill {msg.get('skill')} failed: {error}")
        # The Lens builds its annotation presenter when it enters runtime mode: retry later.
        if ("unavailable" in error and self.demo_pois and self.registered
                and self.poi_retries < DEMO_POI_RETRIES):
            self.poi_retries += 1
            asyncio.ensure_future(self.send_demo_pois(delay=DEMO_POI_DELAY_S))

    # --- demo POIs (what the semantic query will send) ------------------------------------

    def robot_frame(self, ahead, left, up):
        """World point `ahead` m along the robot's +X, `left` m to its left, `up` m above."""
        fx, fz = forward(self.yaw)
        lx, lz = left_of(fx, fz)
        return [round(self.pos[0] + ahead * fx + left * lx, 3), round(self.pos[1] + up, 3),
                round(self.pos[2] + ahead * fz + left * lz, 3)]

    async def send_demo_pois(self, delay=0.0):
        if delay:
            await asyncio.sleep(delay)
        self.annotations = []
        self.poi_retries = 0
        pois = [("poi-bottle", "red bottle (0.87)", self.robot_frame(1.5, 0.4, 0.8)),
                ("poi-box", "box on the table (0.74)", self.robot_frame(2.0, -0.6, 0.9))]
        for aid, label, point in pois:
            await self.annotate({"id": aid, "kind": "marker", "points": [point], "label": label})
        # 3D bounding box of the "box" as three polylines (bottom loop, top loop, a vertical)
        corners = []
        for up in (0.75, 1.05):
            loop = [self.robot_frame(2.0 + a, -0.6 + b, up)
                    for a, b in ((-0.15, -0.2), (0.15, -0.2), (0.15, 0.2), (-0.15, 0.2),
                                 (-0.15, -0.2))]
            corners.append(loop)
        await self.annotate({"id": "bbox-box-bottom", "kind": "line", "points": corners[0],
                             "color": [0.1, 0.9, 0.3]})
        await self.annotate({"id": "bbox-box-top", "kind": "line", "points": corners[1],
                             "color": [0.1, 0.9, 0.3]})
        for i in range(4):
            a, b = corners[0][i], corners[1][i]
            mid = [round((a[k] + b[k]) / 2, 3) for k in range(3)]
            await self.annotate({"id": f"bbox-box-edge{i}", "kind": "line",
                                 "points": [a, mid, b], "color": [0.1, 0.9, 0.3]})

    async def annotate(self, args):
        self.annotations.append(args["id"])
        await self.broadcast(dumps({"type": "ar_skill", "ts": time.time(),
                                    "request_id": f"mock-{args['id']}-{random.randint(0, 1 << 30)}",
                                    "skill": "draw_world_annotation", "args": args}))

    # --- simulation -----------------------------------------------------------------------

    def step(self, dt):
        if self.goal is None or not self.registered:
            return False
        g = self.goal["position"]
        dx, dz = g[0] - self.pos[0], g[2] - self.pos[2]
        dist = math.hypot(dx, dz)
        if dist <= GOAL_TOLERANCE_M:
            self.pos[0], self.pos[2] = g[0], g[2]
            return True
        step = min(dist, WALK_SPEED_MPS * dt)
        self.pos[0] += dx / dist * step
        self.pos[2] += dz / dist * step
        self.yaw = math.atan2(-dz, dx)  # face the direction of travel (marker +X)
        self.speed = step / dt
        self.velocity = [dx / dist * self.speed, 0.0, dz / dist * self.speed]
        return False

    def lidar_points(self):
        """A synthetic 6 x 5 m room with a table, around the registration point."""
        if self.room_center is None:
            return []
        cx, cy, cz, yaw = self.room_center
        fx, fz = forward(yaw)
        lx, lz = left_of(fx, fz)
        rng = random.Random(42)
        pts = []
        for _ in range(3000):
            a, b = rng.uniform(-3.0, 3.0), rng.uniform(-2.5, 2.5)
            wall = rng.random()
            if wall < 0.25:
                a = -3.0
            elif wall < 0.5:
                a = 3.0
            elif wall < 0.75:
                b = -2.5
            else:
                b = 2.5
            pts.append((a, b, rng.uniform(0.0, 2.0)))
        for _ in range(400):  # table top at 0.75 m
            pts.append((rng.uniform(1.8, 2.3), rng.uniform(-0.9, -0.3), 0.75))
        world = [(cx + a * fx + b * lx, cy + h, cz + a * fz + b * lz) for a, b, h in pts]
        if self.lidar_mode == "obstacles":
            lo, _, hi = self.obstacle_band
            world = [p for p in world
                     if lo <= math.hypot(p[0] - self.pos[0], p[2] - self.pos[2]) <= hi + 1.5]
            return world[:LIDAR_OBSTACLE_CAP]
        return world[:LIDAR_FULL_CAP]

    async def run(self):
        period = 1.0 / POSE_HZ
        lidar_every = max(1, int(round(POSE_HZ / LIDAR_HZ)))
        tick = 0
        while True:
            await asyncio.sleep(period)
            tick += 1
            if not self.clients or not self.registered:
                continue
            arrived = self.step(period)
            await self.broadcast(self.pose())
            if self.goal is not None and tick % 5 == 0:
                await self.broadcast(dumps({"type": "path", "ts": round(time.time(), 3),
                                            "waypoints": self.path_points()}))
            if arrived:
                await self.finish_nav("succeeded")
            if self.lidar_mode != "off" and tick % lidar_every == 0:
                await self.broadcast_binary(encode_lidar(time.time(), self.lidar_points()))


def local_ips():
    ips = set()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))  # no packet is sent; picks the default interface
            ips.add(s.getsockname()[0])
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


async def serve(host, port, bridge):
    # max_size=None: camera stills from the Lens can exceed websockets' 1 MiB default
    server = await websockets.serve(bridge.handle, host, port, max_size=None)
    return server


async def main_async(args):
    bridge = MockBridge(demo_pois=args.demo_pois)
    server = await serve(args.host, args.port, bridge)
    ips = local_ips() or ["<this computer's IP>"]
    print(f"Mock AR bridge (protocol v{PROTOCOL_VERSION}) listening on {args.host}:{args.port}")
    print("Type one of these into the Lens as the Bridge IP: " + ", ".join(ips))
    print("Ctrl-C to stop.")
    async with server:
        await bridge.run()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=PORT,
                    help="the Lens connects to port 8787 (WS_PORT in WebSocketTransport.ts)")
    ap.add_argument("--demo-pois", action="store_true",
                    help="after registration, show two labelled POIs and a 3D box")
    args = ap.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
