"""The bridge end to end over a real WebSocket, with a scripted Lens and rendered camera frames.

The "Lens" reports camera poses in its own AR world, which is related to ``map`` by a ground
truth ``T_AR_MAP`` the bridge has never seen; the bridge must recover it from the wall tag.
"""
import asyncio
import json
import math
import time

import cv2
import numpy as np
import pytest
from synthetic import K_from_hfov, cam_tag, glcam_from_cv, look_at_cv, render_tag

from g1_ar_bridge import protocol as P
from g1_ar_bridge.geometry import (ar_from_map_yaw_t, ar_marker_pose, ar_yaw_quat, inv_T,
                                   make_T, pose_to_T, rot_z, rotation_angle_deg, T_to_pose,
                                   transform_points)
from g1_ar_bridge.server import ArBridgeServer, BridgeConfig, serve
from g1_ar_bridge.world import SimWorld

websockets = pytest.importorskip("websockets")

TAG = 0.16
T_AR_MAP = ar_from_map_yaw_t(0.8, [0.3, 1.1, -0.7])      # unknown to the bridge
GW, GH = 1008, 756
GK = K_from_hfov(GW, GH, 83.0)


def run(coro, timeout=60):
    return asyncio.run(asyncio.wait_for(coro, timeout=timeout))


class Lens:
    """Minimal Lens stand-in: newline-framed JSON in, JSON / binary out."""

    def __init__(self, ws):
        self.ws = ws
        self.pending = []
        self.binary = []

    async def send(self, **msg):
        msg.setdefault("ts", time.time())
        msg.setdefault("robot_id", "unitree_g1")
        await self.ws.send(json.dumps(msg))

    async def recv(self, want=None, timeout=10.0, where=None):
        end = time.time() + timeout
        while time.time() < end:
            if not self.pending:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=max(0.01, end - time.time()))
                if isinstance(raw, bytes):
                    if want == "binary":
                        return raw
                    self.binary.append(raw)
                    continue
                assert raw.endswith("\n"), "every JSON text frame ends with a newline"
                self.pending += [json.loads(x) for x in raw.splitlines() if x]
            msg = self.pending.pop(0)
            if want is None or (msg["type"] == want and (where is None or where(msg))):
                return msg
        raise AssertionError(f"no {want!r} message within {timeout} s")


async def session(test, world, cfg=None):
    cfg = cfg or BridgeConfig(tag_black_size_m=TAG)
    bridge = ArBridgeServer(world, cfg, log=lambda *_: None)
    server = await serve(bridge, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]
    loop_task = asyncio.ensure_future(bridge.run())
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}", max_size=None) as ws:
            return await test(Lens(ws), bridge)
    finally:
        loop_task.cancel()
        server.close()
        await server.wait_closed()


def frame_for(world, seq, eye_map, T_ar_map=T_AR_MAP):
    """What the Lens sends from an eye position (map coordinates): JPEG + GL camera pose in AR."""
    T_map_cv = look_at_cv(eye_map, world.T_map_tag[:3, 3], (0, 0, 1))
    img = render_tag(GK, GW, GH, cam_tag(T_map_cv, world.T_map_tag), TAG, noise_sigma=2.0,
                     seed=seq)
    ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    assert ok
    pos, quat = T_to_pose(glcam_from_cv(T_ar_map @ T_map_cv))
    return P.build_camera_frame(seq, pos, quat, jpeg.tobytes())


EYES = [[-0.3, -0.9, 0.82], [-0.1, 0.8, 0.8], [0.2, -0.4, 0.85], [-0.5, 0.3, 0.82],
        [0.1, 1.0, 0.8], [-0.2, -1.1, 0.84], [0.3, 0.1, 0.83], [-0.4, 0.6, 0.8]]


async def camera_info(lens):
    await lens.send(type="camera_info", width=GW, height=GH, fx=GK[0, 0], fy=GK[1, 1],
                    cx=GK[0, 2], cy=GK[1, 2], distortion=[], camera_model="pinhole",
                    device_model="spectacles")
    return await lens.recv("capture_policy")


async def tag_registration(lens, world, eyes=EYES):
    await lens.send(type="registration_command", command="start", mode="april_tag")
    first = await lens.recv("registration_status")
    assert first["state"] == "april_tag" and first["tag_visible"] is False
    statuses = []
    for seq, eye in enumerate(eyes):
        await lens.ws.send(frame_for(world, seq, eye))
        assert (await lens.recv("camera_frame_ack"))["seq"] == seq
        st = await lens.recv("registration_status")
        statuses.append(st)
        if st["state"] == "succeeded":
            break
    return statuses


def test_hello_announces_the_wall_tag_and_no_actuation():
    async def t(lens, _):
        hello = await lens.recv("hello")
        assert hello["protocol_version"] == 19
        prof = hello["robot"]["tag_tracking_profile"]
        assert prof == {"tag_ids": [0], "tag_total_size_m": pytest.approx(TAG / 0.8)}
        caps = hello["capabilities"]
        assert caps["nav"]["available"] is False and caps["nav"]["reason"]
        assert caps["emergency_stop"]["available"] is False
        assert caps["lidar"]["available"] is True
        snap = await lens.recv("runtime_snapshot")
        assert snap["bridge"]["world_frame_committed"] is False
        await lens.send(type="ping", client_ts=12.5)
        assert (await lens.recv("pong"))["client_ts"] == 12.5
        policy = await camera_info(lens)
        assert policy["min_observations"] == 3 and policy["max_capture_distance_m"] >= 1.0
    run(session(t, SimWorld()))


def test_wall_tag_registration_recovers_the_map_and_streams_everything():
    world = SimWorld(demo_pois=True)

    async def t(lens, bridge):
        await camera_info(lens)
        statuses = await tag_registration(lens, world)
        done = statuses[-1]
        assert done["state"] == "succeeded" and done["mode"] == "april_tag"
        assert done["scale_locked"] is True and done["registration_confidence"] >= 0.7
        visible = [s for s in statuses if s["state"] == "april_tag"]
        assert all(s["tag_visible"] for s in visible)
        assert [s["progress"] for s in visible] == sorted(s["progress"] for s in visible)
        assert "preview_pose" in visible[-1]
        assert bridge.committed and bridge.method == "april_tag"
        # recovered transform
        assert rotation_angle_deg(bridge.T_ar_map[:3, :3], T_AR_MAP[:3, :3]) < 1.0
        pts = np.array([[0, 0, 0], [3, 1, 0], [1.5, 0, 0.22]])
        err = np.linalg.norm(transform_points(bridge.T_ar_map, pts)
                             - transform_points(T_AR_MAP, pts), axis=1)
        assert np.all(err < 0.04), err
        # robot pose (map origin) as the Lens receives it
        pose = await lens.recv("pose")
        exp_pos, exp_quat, _ = ar_marker_pose(T_AR_MAP @ world.robot_pose())
        assert np.linalg.norm(np.array(pose["position"]) - exp_pos) < 0.04
        assert abs(np.dot(pose["orientation"], exp_quat)) > math.cos(math.radians(1.0) / 2)
        # LiDAR (full): the room, floor removed, back in map through the true transform
        await lens.send(type="set_lidar_mode", mode="full")
        frame = await lens.recv("binary", timeout=5)
        pts_ar = P.decode_lidar(frame)
        assert 100 < len(pts_ar) <= P.LIDAR_FULL_CAP
        pts_map = transform_points(inv_T(T_AR_MAP), pts_ar)
        assert np.all(pts_map[:, 0] < 1.5 + 0.08) and np.all(pts_map[:, 2] > -0.78)
        assert np.mean(np.abs(pts_map[:, 0] - 1.5) < 0.08) > 0.2       # the tag wall
        # POIs and the box
        skills = []
        while len([s for s in skills if s["skill"] == "draw_world_annotation"]) < 8:
            skills.append(await lens.recv("ar_skill"))
        anns = {s["args"]["id"]: s["args"] for s in skills
                if s["skill"] == "draw_world_annotation"}
        bottle = anns["poi-bottle"]
        assert bottle["kind"] == "marker" and bottle["label"] == "red bottle (0.87)"
        exp = transform_points(T_AR_MAP, [[0.9, -0.85, -0.78 + 0.87]])[0]
        assert np.linalg.norm(np.array(bottle["points"][0]) - exp) < 0.05
        assert all(len(a["points"]) >= 3 for a in anns.values() if a["kind"] == "line")
        # the glasses' pose: answer the bridge's get_user_hmd_transform
        hmd = next((s for s in skills if s["skill"] == "get_user_hmd_transform"), None)
        if hmd is None:
            hmd = await lens.recv("ar_skill", where=lambda m: m["skill"] ==
                                  "get_user_hmd_transform")
        T_map_eye = make_T(rot_z(0.3), [-0.4, 0.2, 0.85])
        pos, quat = T_to_pose(T_AR_MAP @ T_map_eye)
        got = []
        world.on_hmd_pose = lambda T_map_hmd, T_ar_hmd: got.append(T_map_hmd)
        await lens.send(type="ar_skill_result", request_id=hmd["request_id"], ok=True,
                        skill="get_user_hmd_transform",
                        data={"position": pos, "orientation": quat})
        for _ in range(50):
            if got:
                break
            await asyncio.sleep(0.02)
        assert got and np.linalg.norm(got[0][:3, 3] - [-0.4, 0.2, 0.85]) < 0.04
    run(session(t, world))


def test_robot_anchor_can_arrive_after_the_glasses_views():
    world = SimWorld(demo_pois=False)
    anchor = world.T_map_tag
    world.T_map_tag = None                          # robot camera has not seen the tag yet
    world.anchor = lambda tag_id: world.T_map_tag
    frames_world = SimWorld()                       # same geometry, for rendering

    async def t(lens, bridge):
        await camera_info(lens)
        statuses = await tag_registration(lens, frames_world)
        assert statuses[-1]["state"] == "april_tag"
        assert "Waiting for the robot camera" in statuses[-1]["message"]
        assert statuses[-1]["progress"] <= 95
        world.T_map_tag = anchor                    # tag_anchor publishes map -> ar_tag_0
        done = await lens.recv("registration_status", timeout=8,
                               where=lambda m: m["state"] == "succeeded")
        assert done["mode"] == "april_tag" and bridge.committed
        assert rotation_angle_deg(bridge.T_ar_map[:3, :3], T_AR_MAP[:3, :3]) < 1.0
    run(session(t, world))


def test_manual_registration_uses_the_robot_pose_in_map():
    world = SimWorld(demo_pois=False)
    T_map_robot = make_T(rot_z(0.5), [0.4, -0.2, 0.0])
    world.robot_pose = lambda: T_map_robot

    async def t(lens, bridge):
        await lens.send(type="registration_command", command="start", mode="manual_pose")
        assert (await lens.recv("registration_status"))["state"] == "manual_placement"
        # the user drops the marker exactly on the robot (as seen in the AR world)
        pos, quat, _ = ar_marker_pose(T_AR_MAP @ T_map_robot)
        await lens.send(type="registration_pose", position=pos, orientation=quat)
        assert (await lens.recv("registration_status"))["state"] == "awaiting_commit"
        await lens.send(type="registration_command", command="commit")
        status = await lens.recv("bridge_status")
        assert status["world_frame_method"] == "manual_pose" and status["world_frame_approximate"]
        assert (await lens.recv("registration_status"))["state"] == "succeeded"
        assert np.allclose(bridge.T_ar_map, T_AR_MAP, atol=1e-3)
    run(session(t, world))


def test_goals_and_estop_from_the_glasses_are_refused():
    async def t(lens, bridge):
        await lens.send(type="nav_goal", position=[1, 0, 1], orientation=[0, 0, 0, 1])
        nav = await lens.recv("nav_status")
        assert nav["state"] == "resolved" and nav["outcome"] == "failed"
        assert "disabled" in (await lens.recv("agent_response"))["text"]
        await lens.send(type="emergency_stop")
        assert "remote" in (await lens.recv("agent_response"))["text"]
        await lens.send(type="joystick_command", vx=0.5, vy=0.0, wz=0.0)
        await lens.send(type="user_command", text="find the red bottle")
        assert (await lens.recv("agent_status"))["state"] == "busy"
        assert "red bottle" in (await lens.recv("agent_response"))["text"]
        await lens.ws.send("not json")
        await lens.send(type="something_new")
        await lens.send(type="ping", client_ts=1.0)
        assert (await lens.recv("pong"))["client_ts"] == 1.0
    run(session(t, SimWorld()))


def test_timeout_and_stop():
    cfg = BridgeConfig(tag_black_size_m=TAG, registration_timeout_s=0.5)

    async def t(lens, bridge):
        await lens.send(type="registration_command", command="start", mode="april_tag")
        await lens.recv("registration_status")
        failed = await lens.recv("registration_status", timeout=5,
                                 where=lambda m: m["state"] == "failed")
        assert "not found" in failed["message"]
        await lens.send(type="registration_command", command="start", mode="april_tag")
        await lens.recv("registration_status")
        await lens.send(type="registration_command", command="stop")
        assert (await lens.recv("registration_status", where=lambda m: m["state"] == "idle"))
        assert bridge.session is None
    run(session(t, SimWorld(), cfg))


def test_last_lens_leaving_clears_the_registration():
    world = SimWorld(demo_pois=False)
    T_map_robot = make_T(rot_z(0.0), [0.0, 0.0, 0.0])
    world.robot_pose = lambda: T_map_robot

    async def t(lens, bridge):
        await lens.send(type="registration_command", command="start", mode="manual_pose")
        await lens.send(type="registration_pose", position=[0, 0, 0],
                        orientation=ar_yaw_quat(0.0))
        await lens.send(type="registration_command", command="commit")
        await lens.recv("registration_status", where=lambda m: m["state"] == "succeeded")
        await lens.ws.close()
        for _ in range(100):
            if not bridge.clients:
                break
            await asyncio.sleep(0.02)
        assert not bridge.committed and bridge.T_ar_map is None
        return True
    assert run(session(t, world))


def test_camera_frame_parsing_rejects_garbage():
    with pytest.raises(ValueError):
        P.parse_camera_frame(b"nope")
    frame = P.build_camera_frame(3, [0, 1, 2], [0, 0, 0, 1], b"\xff\xd8x\xff\xd9")
    header, jpeg = P.parse_camera_frame(frame)
    assert header["seq"] == 3 and jpeg == b"\xff\xd8x\xff\xd9"
    bad = P.build_camera_frame(3, [0, 1], [0, 0, 0, 1], b"")
    with pytest.raises(ValueError):
        P.parse_camera_frame(bad)
    pts = np.array([[0.5, 1.0, -2.0], [np.nan, 0, 0], [1e6, 0, 0]])
    assert np.allclose(P.decode_lidar(P.encode_lidar(pts)), [[0.5, 1.0, -2.0]], atol=1e-3)
    assert pose_to_T([0, 0, 0], [0, 0, 0, 1]).shape == (4, 4)
