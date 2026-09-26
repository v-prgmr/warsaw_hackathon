"""The mock bridge speaks the Lens protocol (v19) the way the real dimos-ar bridge does.

    pip install websockets pytest && python -m pytest ar_glasses/mock_bridge
"""
import asyncio
import json
import math
import struct
import time

import pytest

websockets = pytest.importorskip("websockets")
import mock_bridge as mb  # noqa: E402


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, timeout=30))


class Client:
    """Minimal Lens stand-in: newline-framed JSON in, JSON / binary out."""

    def __init__(self, ws):
        self.ws = ws
        self.pending = []

    async def send(self, **msg):
        msg.setdefault("ts", time.time())
        msg.setdefault("robot_id", mb.ROBOT_ID)
        await self.ws.send(json.dumps(msg))

    async def recv(self, want=None, timeout=5.0):
        """Next JSON message (of type `want`), skipping others; binary frames are returned."""
        end = time.time() + timeout
        while time.time() < end:
            if not self.pending:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=end - time.time())
                if isinstance(raw, bytes):
                    if want == "binary":
                        return raw
                    continue
                assert raw.endswith("\n"), "every JSON text frame ends with a newline"
                self.pending += [json.loads(x) for x in raw.splitlines() if x]
            msg = self.pending.pop(0)
            if want is None or msg["type"] == want:
                return msg
        raise AssertionError(f"no {want!r} message within {timeout} s")


async def session(test, demo_pois=False):
    bridge = mb.MockBridge(demo_pois=demo_pois, log=lambda *_: None)
    server = await mb.serve("127.0.0.1", 0, bridge)
    port = server.sockets[0].getsockname()[1]
    sim = asyncio.ensure_future(bridge.run())
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}", max_size=None) as ws:
            return await test(Client(ws), bridge)
    finally:
        sim.cancel()
        server.close()
        await server.wait_closed()


async def register_manually(c, position=(1.0, 0.0, -2.0), yaw=0.0):
    await c.send(type="registration_command", command="start", mode="manual_pose")
    assert (await c.recv("registration_status"))["state"] == "manual_placement"
    await c.send(type="registration_pose", position=list(position),
                 orientation=mb.yaw_quat(yaw))
    assert (await c.recv("registration_status"))["state"] == "awaiting_commit"
    await c.send(type="registration_command", command="commit")
    status = await c.recv("bridge_status")
    assert status["world_frame_committed"] and status["world_frame_method"] == "manual_pose"
    done = await c.recv("registration_status")
    assert done["state"] == "succeeded" and done["mode"] == "manual_pose"


def test_handshake_and_clock_ping():
    async def t(c, _):
        hello = await c.recv()
        assert hello["type"] == "hello" and hello["protocol_version"] == 19
        assert hello["robot"]["robot_id"] == "unitree_g1"
        assert set(hello["robot"]["tag_tracking_profile"]) == {"tag_ids", "tag_total_size_m"}
        assert all(set(v) >= {"available"} for v in hello["capabilities"].values())
        snap = await c.recv()
        assert snap["type"] == "runtime_snapshot" and snap["robot_id"] == "unitree_g1"
        assert snap["bridge"]["world_frame_committed"] is False
        assert snap["nav"] == {"state": "idle"} and snap["agent"] == {"state": "idle"}
        await c.send(type="ping", client_ts=123.5)
        pong = await c.recv("pong")
        assert pong["client_ts"] == 123.5 and abs(pong["bridge_ts"] - time.time()) < 2
        await c.send(type="get_status")
        assert (await c.recv("runtime_snapshot"))["type"] == "runtime_snapshot"
    run(session(t))


def test_manual_registration_then_pose_stream():
    async def t(c, _):
        await register_manually(c, position=(1.0, 0.1, -2.0), yaw=0.5)
        pose = await c.recv("pose")
        assert pose["position"] == [1.0, 0.1, -2.0]
        assert pose["orientation"] == [round(v, 4) for v in mb.yaw_quat(0.5)]
        await c.send(type="get_status")
        snap = await c.recv("runtime_snapshot")
        assert snap["bridge"]["world_frame_committed"] is True
    run(session(t))


def test_commit_without_pose_is_refused():
    async def t(c, bridge):
        await c.send(type="registration_command", command="start", mode="manual_pose")
        await c.recv("registration_status")
        await c.send(type="registration_command", command="commit")
        assert (await c.recv("registration_status"))["state"] == "manual_placement"
        assert not bridge.registered
    run(session(t))


def test_nav_goal_walks_the_simulated_robot_and_succeeds():
    async def t(c, _):
        await register_manually(c, position=(0.0, 0.0, 0.0))
        await c.send(type="nav_goal", position=[0.0, 0.0, -0.6], orientation=[0, 0, 0, 1])
        nav = await c.recv("nav_status")
        assert nav["state"] == "navigating" and nav["goal"]["source"] == "user"
        path = await c.recv("path")
        assert path["waypoints"][-1] == [0.0, 0.0, -0.6]
        moving = await c.recv("pose")
        while moving["speed_mps"] == 0:
            moving = await c.recv("pose")
        assert moving["position"][2] < 0 and moving["velocity_mps"][2] < 0
        # facing the direction of travel: the marker's +X points along -Z
        fx, fz = mb.forward(mb.yaw_of(moving["orientation"]))
        assert fz == pytest.approx(-1.0, abs=1e-3)
        done = await c.recv("nav_status", timeout=10)
        assert done == {"type": "nav_status", "ts": done["ts"], "state": "resolved",
                        "outcome": "succeeded"}
        final = await c.recv("pose")
        assert final["position"][2] == pytest.approx(-0.6, abs=0.06)
    run(session(t))


def test_nav_goal_before_registration_fails_and_estop_stops():
    async def t(c, _):
        await c.send(type="nav_goal", position=[1, 0, 1])
        assert (await c.recv("nav_status"))["outcome"] == "failed"
        await register_manually(c)
        await c.send(type="nav_goal", position=[5, 0, 5])
        await c.recv("nav_status")
        await c.send(type="emergency_stop")
        stop = await c.recv("nav_status")
        assert stop["state"] == "resolved" and stop["outcome"] == "failed"
    run(session(t))


def test_lidar_binary_frames():
    async def t(c, _):
        await register_manually(c)
        await c.send(type="set_lidar_mode", mode="full")
        frame = await c.recv("binary")
        assert frame[0] == 0x01
        n = (len(frame) - 5) / 6
        assert n == int(n) and 0 < n <= 1500
        pts = [struct.unpack_from("<eee", frame, 5 + 6 * i) for i in range(int(n))]
        assert all(0.0 - 0.01 <= p[1] <= 2.01 for p in pts)  # floor to 2 m above the marker
        await c.send(type="set_lidar_mode", mode="obstacles", obstacle_min_distance_m=0.1,
                     obstacle_opaque_distance_m=0.5, obstacle_max_distance_m=0.8)
        await asyncio.sleep(0.6)
        frame = await c.recv("binary")
        assert (len(frame) - 5) / 6 <= 200
    run(session(t))


def camera_frame(seq, cam_pos=(0.0, 1.6, 0.0), cam_rot=(0.0, 0.0, 0.0, 1.0)):
    header = json.dumps({"type": "camera_frame", "robot_id": mb.ROBOT_ID, "seq": seq,
                         "ts": time.time(), "send_ts": time.time(), "cam_pos": list(cam_pos),
                         "cam_rot": list(cam_rot)}).encode()
    return b"ARF1" + struct.pack("<I", len(header)) + header + b"\xff\xd8fakejpeg\xff\xd9"


def test_camera_frames_are_acked_and_mock_apriltag_registers_in_front_of_the_user():
    async def t(c, bridge):
        await c.send(type="camera_info", width=1008, height=756, fx=700.0, fy=700.0, cx=504.0,
                     cy=378.0, distortion=[], camera_model="pinhole", device_model="spectacles")
        assert (await c.recv("capture_policy"))["min_observations"] == 3
        await c.send(type="registration_command", command="start", mode="april_tag")
        assert (await c.recv("registration_status"))["state"] == "april_tag"
        for seq in range(mb.APRILTAG_MOCK_FRAMES):
            await c.ws.send(camera_frame(seq))
            assert (await c.recv("camera_frame_ack"))["seq"] == seq
        status = await c.recv("bridge_status")
        assert status["world_frame_method"] == "april_tag"
        # glasses at 1.6 m looking along -Z: robot 1.5 m ahead, 1.2 m lower, facing the user
        assert bridge.pos == pytest.approx([0.0, 0.4, -1.5])
        fx, fz = mb.forward(bridge.yaw)
        assert (fx, fz) == pytest.approx((0.0, 1.0), abs=1e-6)
    run(session(t))


def test_demo_pois_use_the_lens_annotation_skill():
    async def t(c, _):
        await register_manually(c, position=(0.0, 0.0, 0.0), yaw=0.0)
        skills = []
        while len(skills) < 8:
            skills.append(await c.recv("ar_skill"))
        assert {s["skill"] for s in skills} == {"draw_world_annotation"}
        markers = [s["args"] for s in skills if s["args"]["kind"] == "marker"]
        assert [m["label"] for m in markers] == ["red bottle (0.87)", "box on the table (0.74)"]
        # 1.5 m ahead (+X), 0.4 m to the left (-Z for yaw 0), 0.8 m up
        assert markers[0]["points"] == [[1.5, 0.8, -0.4]]
        lines = [s["args"] for s in skills if s["args"]["kind"] == "line"]
        assert all(len(line["points"]) >= 3 for line in lines)  # >= 3 points: straight polyline
        assert len({s["request_id"] for s in skills}) == 8
        # the Lens was not in runtime mode yet: the mock sends them again
        await c.send(type="ar_skill_result", request_id=skills[0]["request_id"], ok=False,
                     skill="draw_world_annotation", error="annotation presenter unavailable")
        again = await c.recv("ar_skill", timeout=5)
        assert again["args"]["id"] == "poi-bottle"
    run(session(t, demo_pois=True))


def test_agent_reply_and_unknown_messages_are_harmless():
    async def t(c, _):
        await c.send(type="user_command", text="come here")
        assert (await c.recv("agent_status"))["state"] == "busy"
        assert "come here" in (await c.recv("agent_response"))["text"]
        await c.send(type="something_new")
        await c.ws.send("not json")
        await c.send(type="ping", client_ts=1.0)
        assert (await c.recv("pong"))["client_ts"] == 1.0
    run(session(t))


def test_geometry_helpers():
    for yaw in (-2.5, -0.3, 0.0, 1.2, 3.0):
        assert mb.yaw_of(mb.yaw_quat(yaw)) == pytest.approx(yaw)
        fx, fz = mb.forward(yaw)
        rx, _, rz = mb.rotate_by_quat(mb.yaw_quat(yaw), (1.0, 0.0, 0.0))
        assert (rx, rz) == pytest.approx((fx, fz))
    lx, lz = mb.left_of(*mb.forward(0.0))
    assert (lx, lz) == pytest.approx((0.0, -1.0))  # left of +X is -Z (Y up, right-handed)
    assert mb.parse_camera_frame(b"nope") is None
    assert math.isclose(struct.unpack("<e", struct.pack("<e", 1.5))[0], 1.5)
