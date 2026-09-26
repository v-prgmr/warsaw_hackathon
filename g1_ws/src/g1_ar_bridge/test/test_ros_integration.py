"""ROS 2 end to end: ``ros2 launch g1_ar_bridge ar_bridge.launch.py`` with a fake robot camera
(rendered wall tag + aligned depth + TF) and a scripted Lens over the WebSocket.

Checks TF map -> ar_tag_0 (tag_anchor), the registration, TF map -> ar_world and
ar_world -> spectacles (ar_bridge), /ar_glasses/hmd_pose and /ar_glasses/user_command.
Needs a sourced ROS 2 Humble workspace with g1_ar_bridge built, and ``websockets``; skipped
otherwise. Uses ROS_DOMAIN_ID 77 unless one is set (never the robot's domain 0).
"""
import asyncio
import json
import math
import os
import shutil
import signal
import subprocess
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
websockets = pytest.importorskip("websockets")
if shutil.which("ros2") is None:
    pytest.skip("ros2 CLI not found", allow_module_level=True)

import cv2  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from synthetic import (K_from_hfov, cam_tag, glcam_from_cv, look_at_cv, render_depth,  # noqa: E402
                       render_tag)
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from g1_ar_bridge import protocol as P  # noqa: E402
from g1_ar_bridge.geometry import (T_to_pose, ar_from_map_yaw_t, inv_T, make_T,  # noqa: E402
                                   rotation_angle_deg, transform_points)
from g1_ar_bridge.ros_util import transform_msg_to_T  # noqa: E402
from g1_ar_bridge.world import wall_tag_pose  # noqa: E402

if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
    os.environ["ROS_DOMAIN_ID"] = "77"

TAG = 0.17
FLOOR = -0.78
T_MAP_TAG = wall_tag_pose(1.6, 1.0, FLOOR, lateral=0.1)
T_MAP_CAM = look_at_cv([0.12, 0.0, 0.3], T_MAP_TAG[:3, 3] + [0.0, 0.05, -0.1], (0, 0, 1))
T_AR_MAP = ar_from_map_yaw_t(-0.6, [1.2, 0.95, 0.4])
RW, RH = 848, 480
RK = K_from_hfov(RW, RH, 69.0)
GW, GH = 1008, 756
GK = K_from_hfov(GW, GH, 83.0)
PORT = 18787


def tf_msg(T, parent, child, stamp):
    m = TransformStamped()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, parent, child
    (x, y, z), (qx, qy, qz, qw) = T_to_pose(T)
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = x, y, z
    r = m.transform.rotation
    r.x, r.y, r.z, r.w = qx, qy, qz, qw
    return m


class FakeRobot:
    """Publishes what the robot stack would: TF, camera, depth, map cloud, POI markers."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("fake_robot")
        n = self.node
        self.static = StaticTransformBroadcaster(n)
        stamp = n.get_clock().now().to_msg()
        self.static.sendTransform([
            tf_msg(np.eye(4), "map", "robot_center", stamp),
            tf_msg(T_MAP_CAM, "map", "camera_color_optical_frame", stamp)])
        self.img = n.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.info = n.create_publisher(CameraInfo, "/camera/color/camera_info",
                                       qos_profile_sensor_data)
        self.depth = n.create_publisher(Image, "/camera/aligned_depth_to_color/image_raw",
                                        qos_profile_sensor_data)
        self.cloud = n.create_publisher(PointCloud2, "/cloud_map", 1)
        self.markers = n.create_publisher(MarkerArray, "/ar_glasses/markers", 10)
        self.cmds, self.hmd, self.status = [], [], []
        n.create_subscription(String, "/ar_glasses/user_command",
                              lambda m: self.cmds.append(m.data), 10)
        n.create_subscription(PoseStamped, "/ar_glasses/hmd_pose", self.hmd.append, 10)
        n.create_subscription(String, "/ar_glasses/anchor_status",
                              lambda m: self.status.append(json.loads(m.data)), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, n)
        gray = render_tag(RK, RW, RH, cam_tag(T_MAP_CAM, T_MAP_TAG), TAG, noise_sigma=2.0)
        self.rgb = np.repeat(gray[..., None], 3, axis=2)
        self.depth_m = render_depth(RK, RW, RH, cam_tag(T_MAP_CAM, T_MAP_TAG), noise_m=0.003)
        n.create_timer(0.1, self.publish_camera)
        n.create_timer(1.0, self.publish_world)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def publish_camera(self):
        stamp = self.node.get_clock().now().to_msg()
        img = Image(height=RH, width=RW, encoding="rgb8", step=RW * 3,
                    data=self.rgb.tobytes())
        img.header.stamp, img.header.frame_id = stamp, "camera_color_optical_frame"
        depth = Image(height=RH, width=RW, encoding="32FC1", step=RW * 4,
                      data=np.nan_to_num(self.depth_m, nan=0.0).astype(np.float32).tobytes())
        depth.header = img.header
        info = CameraInfo(height=RH, width=RW, distortion_model="plumb_bob",
                          d=[0.0] * 5, k=RK.reshape(-1).tolist())
        info.header = img.header
        self.info.publish(info)
        self.depth.publish(depth)
        self.img.publish(img)

    def publish_world(self):
        rng = np.random.default_rng(0)
        pts = np.column_stack([np.full(500, 1.62), rng.uniform(-2, 2, 500),
                               rng.uniform(FLOOR, FLOOR + 2, 500)]).astype(np.float32)
        msg = PointCloud2(height=1, width=len(pts), is_bigendian=False, point_step=12,
                          row_step=12 * len(pts), is_dense=True, data=pts.tobytes(),
                          fields=[PointField(name=c, offset=4 * i, datatype=7, count=1)
                                  for i, c in enumerate("xyz")])
        msg.header.frame_id = "map"
        self.cloud.publish(msg)
        m = Marker()
        m.header.frame_id = "map"
        m.ns, m.id, m.type, m.action = "poi", 7, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = 1.0, -0.5, 0.1
        m.pose.orientation.w = 1.0
        m.text = "red bottle (0.87)"
        self.markers.publish(MarkerArray(markers=[m]))

    def lookup(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None
        return transform_msg_to_T(tf.transform)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()


def wait_for(fn, timeout, period=0.1):
    end = time.time() + timeout
    while time.time() < end:
        value = fn()
        if value is not None and value is not False:
            return value
        time.sleep(period)
    return None


def glasses_frame(seq, eye):
    T_map_cv = look_at_cv(eye, T_MAP_TAG[:3, 3], (0, 0, 1))
    img = render_tag(GK, GW, GH, cam_tag(T_map_cv, T_MAP_TAG), TAG, noise_sigma=2.0, seed=seq)
    ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    pos, quat = T_to_pose(glcam_from_cv(T_AR_MAP @ T_map_cv))
    return P.build_camera_frame(seq, pos, quat, jpeg.tobytes())


async def lens_session():
    """Register with the wall tag, answer one HMD request, send a user command."""
    eyes = [[-0.3, -0.9, 0.82], [-0.1, 0.8, 0.8], [0.2, -0.4, 0.85], [-0.5, 0.3, 0.82],
            [0.1, 1.0, 0.8], [-0.2, -1.1, 0.84], [0.3, 0.1, 0.83], [-0.4, 0.6, 0.8]]
    out = {"skills": []}
    async with websockets.connect(f"ws://127.0.0.1:{PORT}", max_size=None) as ws:
        pending = []

        async def recv(want, timeout=15.0, where=None):
            end = time.time() + timeout
            while time.time() < end:
                if not pending:
                    raw = await asyncio.wait_for(ws.recv(), max(0.01, end - time.time()))
                    if isinstance(raw, bytes):
                        if want == "binary":
                            return raw
                        continue
                    pending.extend(json.loads(x) for x in raw.splitlines() if x)
                msg = pending.pop(0)
                if msg["type"] == "ar_skill":
                    out["skills"].append(msg)
                if msg["type"] == want and (where is None or where(msg)):
                    return msg
            raise AssertionError(f"no {want}")

        async def send(**msg):
            msg.setdefault("ts", time.time())
            msg.setdefault("robot_id", "unitree_g1")
            await ws.send(json.dumps(msg))

        out["hello"] = await recv("hello")
        await send(type="camera_info", width=GW, height=GH, fx=GK[0, 0], fy=GK[1, 1],
                   cx=GK[0, 2], cy=GK[1, 2], distortion=[], camera_model="pinhole")
        await recv("capture_policy")
        await send(type="registration_command", command="start", mode="april_tag")
        await recv("registration_status")
        for seq, eye in enumerate(eyes):
            await ws.send(glasses_frame(seq, eye))
            await recv("camera_frame_ack")
            st = await recv("registration_status")
            if st["state"] == "succeeded":
                break
        out["done"] = st
        await send(type="set_lidar_mode", mode="full")
        out["lidar"] = P.decode_lidar(await recv("binary"))
        hmd = await recv("ar_skill", where=lambda m: m["skill"] == "get_user_hmd_transform")
        T_map_eye = make_T(np.eye(3), [-0.4, 0.2, 0.85])
        pos, quat = T_to_pose(T_AR_MAP @ T_map_eye)
        await send(type="ar_skill_result", request_id=hmd["request_id"], ok=True,
                   skill="get_user_hmd_transform", data={"position": pos, "orientation": quat})
        await send(type="user_command", text="find the red bottle")
        await recv("agent_response")
        await recv("ar_skill", where=lambda m: m["skill"] == "draw_world_annotation")
        await asyncio.sleep(1.0)
    return out


@pytest.fixture
def launched():
    robot = FakeRobot()
    proc = subprocess.Popen(
        ["ros2", "launch", "g1_ar_bridge", "ar_bridge.launch.py", f"tag_black_size_m:={TAG}",
         f"port:={PORT}"], start_new_session=True)
    try:
        yield robot
    finally:
        proc.send_signal(signal.SIGINT)          # launch forwards it to the nodes
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        robot.close()


def test_wall_tag_alignment_through_ros(launched):
    robot = launched
    # 1. the robot camera anchors the tag in map
    T_anchor = wait_for(lambda: robot.lookup("map", "ar_tag_0"), 40)
    assert T_anchor is not None, f"no map -> ar_tag_0; status: {robot.status[-1:]}"
    assert np.linalg.norm(T_anchor[:3, 3] - T_MAP_TAG[:3, 3]) < 0.01
    assert rotation_angle_deg(T_anchor[:3, :3], T_MAP_TAG[:3, :3]) < 1.0
    assert wait_for(lambda: robot.status and robot.status[-1].get("anchor", {}).get("depth")
                    == "used", 10), robot.status[-1:]
    # 2. the Lens registers through the bridge
    time.sleep(1.0)
    out = asyncio.run(asyncio.wait_for(lens_session(), 90))
    assert out["hello"]["robot"]["tag_tracking_profile"]["tag_ids"] == [0]
    assert out["done"]["state"] == "succeeded" and out["done"]["mode"] == "april_tag"
    T_map_ar = wait_for(lambda: robot.lookup("map", "ar_world"), 10)
    assert T_map_ar is not None
    T_ar_map = inv_T(T_map_ar)
    assert rotation_angle_deg(T_ar_map[:3, :3], T_AR_MAP[:3, :3]) < 1.0
    pts = np.array([[0.0, 0.0, 0.0], [3.0, -1.0, 0.0]])
    err = np.linalg.norm(transform_points(T_ar_map, pts) - transform_points(T_AR_MAP, pts), 1)
    assert np.all(err < 0.05), err
    # the map cloud reached the glasses, in their world
    assert len(out["lidar"]) > 50
    back = transform_points(inv_T(T_AR_MAP), out["lidar"])
    assert np.median(np.abs(back[:, 0] - 1.62)) < 0.05
    # 3. the glasses in our TF tree and on the hmd topic
    T_map_hmd = wait_for(lambda: robot.lookup("map", "spectacles"), 10)
    assert T_map_hmd is not None and np.linalg.norm(T_map_hmd[:3, 3] - [-0.4, 0.2, 0.85]) < 0.05
    # the Lens camera looked along AR -Z = map ... the body frame's x axis is its forward
    assert wait_for(lambda: robot.hmd, 5)
    assert robot.hmd[-1].header.frame_id == "map"
    assert wait_for(lambda: robot.cmds, 5) == ["find the red bottle"]
    assert math.isfinite(robot.hmd[-1].pose.position.x)
