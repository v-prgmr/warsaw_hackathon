"""Lens protocol v19 (spectacles-dimensional-os ``dimos-ar/PROTOCOL.md``, pinned ``ebf1d38``).

JSON text frames end with ``\\n``; LiDAR is binary (``0x01``, float32 ts, float16 xyz); camera
frames from the Lens are binary ``ARF1`` envelopes. All coordinates: AR world, metres, Y up.
Message shapes follow ``ar_glasses/mock_bridge/mock_bridge.py``, which was checked against the
Lens parser (``Protocol.ts``) and runs with the real glasses.
"""
import json
import math
import struct
import time

import numpy as np

PROTOCOL_VERSION = 19
PORT = 8787                      # fixed in the Lens (WS_PORT in WebSocketTransport.ts)
LIDAR_FULL_CAP = 1500
LIDAR_OBSTACLE_CAP = 200
CAMERA_FRAME_MAGIC = b"ARF1"
MAX_HEADER_BYTES = 4096


def dumps(payload):
    return json.dumps(payload, separators=(",", ":"), allow_nan=False)


def r4(v):
    return [round(float(x), 4) for x in v]


def r3(v):
    return [round(float(x), 3) for x in v]


def hello(robot, capabilities):
    return dumps({"type": "hello", "protocol_version": PROTOCOL_VERSION, "robot": robot,
                  "capabilities": capabilities})


def bridge_wire(committed, method, approximate):
    return {"robot_connected": True, "reconnecting": False,
            "world_frame_committed": bool(committed),
            "world_frame_method": method if committed else None,
            "world_frame_approximate": bool(approximate) if committed else False}


def bridge_status(committed, method, approximate):
    return dumps({"type": "bridge_status", "ts": time.time(),
                  **bridge_wire(committed, method, approximate)})


def runtime_snapshot(robot_id, committed, method, approximate, path=None):
    payload = {"type": "runtime_snapshot", "ts": time.time(), "robot_id": robot_id,
               "bridge": bridge_wire(committed, method, approximate), "nav": {"state": "idle"},
               "agent": {"state": "idle"}}
    if path:
        payload["path"] = {"waypoints": [r3(p) for p in path]}
    return dumps(payload)


def registration_status(state, message, mode=None, **extra):
    payload = {"type": "registration_status", "ts": time.time(), "state": state,
               "message": message}
    if mode:
        payload["mode"] = mode
    payload.update(extra)
    return dumps(payload)


def pose(position, orientation, velocity=None, yaw_rate=None):
    # speed_mps is left out on purpose: the Lens re-arms camera capture on robot speed edges,
    # which only matters for robot-mounted tags (not our wall tag).
    payload = {"type": "pose", "ts": round(time.time(), 3), "position": r4(position),
               "orientation": r4(orientation)}
    if velocity is not None:
        payload["velocity_mps"] = r4(velocity)
    if yaw_rate is not None:
        payload["yaw_rate_rad_s"] = round(float(yaw_rate), 4)
    return dumps(payload)


def path(waypoints):
    return dumps({"type": "path", "ts": round(time.time(), 3),
                  "waypoints": [r3(p) for p in waypoints]})


def nav_status(state, outcome=None):
    payload = {"type": "nav_status", "ts": time.time(), "state": state}
    if outcome:
        payload["outcome"] = outcome
    return dumps(payload)


def pong(robot_id, client_ts):
    now = time.time()
    return dumps({"type": "pong", "ts": now, "robot_id": robot_id, "client_ts": client_ts,
                  "bridge_ts": now})


def capture_policy(max_distance_m, min_distance_m=0.3, max_speed_mps=0.45,
                   static_speed_mps=0.05, min_observations=3):
    return dumps({"type": "capture_policy", "ts": time.time(),
                  "max_capture_distance_m": round(float(max_distance_m), 4),
                  "min_capture_distance_m": round(float(min_distance_m), 4),
                  "max_capture_speed_mps": round(float(max_speed_mps), 4),
                  "static_speed_mps": round(float(static_speed_mps), 4),
                  "min_observations": int(min_observations)})


def camera_frame_ack(seq):
    return dumps({"type": "camera_frame_ack", "ts": time.time(), "seq": int(seq),
                  "capturing_budgeted_complete": False})


def agent_status(state, detail=None):
    payload = {"type": "agent_status", "ts": time.time(), "state": state}
    if detail:
        payload["detail"] = detail
    return dumps(payload)


def agent_response(text):
    return dumps({"type": "agent_response", "ts": time.time(), "text": text})


def ar_skill(request_id, skill, args=None):
    payload = {"type": "ar_skill", "ts": time.time(), "request_id": request_id, "skill": skill}
    if args is not None:
        payload["args"] = args
    return dumps(payload)


def encode_lidar(points, ts=None):
    """Binary ``lidar_f16``: 0x01, float32 LE timestamp, float16 LE x, y, z per point."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.all(np.isfinite(pts), axis=1) & np.all(np.abs(pts) < 6.0e4, axis=1)]
    head = struct.pack("<Bf", 0x01, float(time.time() if ts is None else ts))
    return head + pts.astype("<f2").tobytes()


def decode_lidar(data):
    n = (len(data) - 5) // 6
    return np.frombuffer(data[5:5 + 6 * n], dtype="<f2").astype(np.float64).reshape(-1, 3)


def parse_camera_frame(data):
    """``ARF1`` envelope -> ``(header dict, jpeg bytes)``; raises ValueError when malformed."""
    if len(data) < 8 or data[:4] != CAMERA_FRAME_MAGIC:
        raise ValueError("not an ARF1 camera frame")
    (hlen,) = struct.unpack_from("<I", data, 4)
    if hlen < 2 or hlen > MAX_HEADER_BYTES or len(data) < 8 + hlen:
        raise ValueError("bad camera_frame header length")
    try:
        header = json.loads(data[8:8 + hlen].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"bad camera_frame header: {exc}") from exc
    if not isinstance(header, dict) or header.get("type") != "camera_frame":
        raise ValueError("camera_frame header has the wrong type")
    for key, n in (("cam_pos", 3), ("cam_rot", 4)):
        v = header.get(key)
        if (not isinstance(v, list) or len(v) != n
                or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in v)):
            raise ValueError(f"camera_frame invalid {key}")
    return header, data[8 + hlen:]


def build_camera_frame(seq, cam_pos, cam_rot, jpeg, robot_id="unitree_g1", ts=None):
    """What the Lens sends (used by tests and tools)."""
    now = time.time() if ts is None else ts
    header = dumps({"type": "camera_frame", "robot_id": robot_id, "seq": int(seq), "ts": now,
                    "send_ts": now, "cam_pos": [float(v) for v in cam_pos],
                    "cam_rot": [float(v) for v in cam_rot]}).encode("utf-8")
    return CAMERA_FRAME_MAGIC + struct.pack("<I", len(header)) + header + bytes(jpeg)
