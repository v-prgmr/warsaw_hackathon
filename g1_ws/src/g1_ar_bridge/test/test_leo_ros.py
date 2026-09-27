"""Leo Rover in the G1 map, end to end through ROS 2 (no Leo, no G1 needed).

1. ``leo_relay`` (what runs on Leo) reads a fake Leo TF ``leo_oak_rgb_camera_optical_frame ->
   leo_tag0`` and ``/leo/merged_odom`` and sends them over UDP.
2. ``ar_bridge.launch.py fake_robot:=true leo:=true`` (fake G1 anchor ``map -> ar_tag_0``) gets
   a scripted relay over UDP and must publish TF ``map -> leo_odom -> leo_base`` at Leo's true
   pose and the Leo marker on ``/ar_glasses/markers``.
Needs a sourced Humble workspace with g1_ar_bridge built; uses ROS_DOMAIN_ID 77 (never 0).
"""
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
if shutil.which("ros2") is None:
    pytest.skip("ros2 CLI not found", allow_module_level=True)

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer, TransformBroadcaster, TransformListener  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

from g1_ar_bridge.geometry import inv_T, make_T, rot_to_quat, rot_z  # noqa: E402
from g1_ar_bridge.leo_localization import DEFAULT_CAMERA_RPY, camera_mount  # noqa: E402
from g1_ar_bridge.ros_util import transform_msg_to_T  # noqa: E402
from g1_ar_bridge.world import wall_tag_pose  # noqa: E402

if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
    os.environ["ROS_DOMAIN_ID"] = "77"

RELAY_PORT, BRIDGE_PORT, LEO_PORT = 18791, 18788, 18792
T_MAP_TAG = wall_tag_pose(1.5, 1.0, -0.78)          # = ar_bridge.launch.py fake_robot defaults
T_BASE_CAM = camera_mount([0.0, 0.0, 0.0], DEFAULT_CAMERA_RPY)   # = config default


def udp_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", port))
    s.settimeout(0.2)
    return s


def tf_msg(T, parent, child, stamp):
    m = TransformStamped()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = T[:3, 3]
    (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
     m.transform.rotation.w) = rot_to_quat(T[:3, :3])
    return m


@pytest.fixture
def ros():
    rclpy.init()
    node = rclpy.create_node("leo_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_relay_reads_leo_tf_and_odometry_and_sends_udp(ros):
    rx = udp_socket(RELAY_PORT)
    relay = subprocess.Popen([sys.executable, "-m", "g1_ar_bridge.leo_relay", "--laptop-ip",
                              "127.0.0.1", "--port", str(RELAY_PORT)], start_new_session=True)
    tfb = TransformBroadcaster(ros)
    odom_pub = ros.create_publisher(Odometry, "/leo/merged_odom", 10)
    T_cam_tag = make_T(rot_z(0.2), [0.1, -0.05, 1.3])
    got = {}
    try:
        end = time.time() + 20
        while time.time() < end and not ({"tag", "odom"} <= got.keys()):
            stamp = ros.get_clock().now().to_msg()
            tfb.sendTransform(tf_msg(T_cam_tag, "leo_oak_rgb_camera_optical_frame", "leo_tag0",
                                     stamp))
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_link"
            o.pose.pose.position.x, o.pose.pose.orientation.w = 0.7, 1.0
            odom_pub.publish(o)
            rclpy.spin_once(ros, timeout_sec=0.05)
            try:
                data, _ = rx.recvfrom(65536)
            except socket.timeout:
                continue
            for line in data.decode().splitlines():
                msg = json.loads(line)
                got[msg["type"]] = msg
    finally:
        relay.send_signal(signal.SIGINT)
        relay.wait(timeout=10)
        rx.close()
    assert np.allclose(np.asarray(got["tag"]["T"]), T_cam_tag, atol=1e-6)
    assert got["tag"]["frame_id"] == "leo_oak_rgb_camera_optical_frame"
    assert got["odom"]["position"][0] == pytest.approx(0.7)
    assert (got["odom"]["frame_id"], got["odom"]["child_frame_id"]) == ("odom", "base_link")


def test_leo_is_placed_in_the_g1_map_and_marked_for_the_glasses(ros, tmp_path):
    launch = subprocess.Popen(
        ["ros2", "launch", "g1_ar_bridge", "ar_bridge.launch.py", "fake_robot:=true", "leo:=true",
         f"port:={BRIDGE_PORT}", f"leo_udp_port:={LEO_PORT}", f"record_dir:={tmp_path}"],
        start_new_session=True)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    buf = Buffer()
    TransformListener(buf, ros)
    markers = {}
    ros.create_subscription(MarkerArray, "/ar_glasses/markers",
                            lambda msg: markers.update({(m.ns, m.id): m for m in msg.markers}), 10)
    # Leo: odometry frame unrelated to the G1 map; Leo 1 m in front of the wall tag, facing it
    T_map_leoodom = make_T(rot_z(math.radians(40)), [3.0, -2.0, -0.70])
    T_map_base = make_T(rot_z(math.radians(10)), [0.5, 0.1, -0.70])
    T_odom_base = inv_T(T_map_leoodom) @ T_map_base
    T_cam_tag = inv_T(T_map_base @ T_BASE_CAM) @ T_MAP_TAG
    leo_t = 1000.0

    def send(msg):
        tx.sendto(json.dumps(msg).encode(), ("127.0.0.1", LEO_PORT))

    found = None
    try:
        end = time.time() + 40
        while time.time() < end:
            leo_t += 0.05
            send({"type": "odom", "stamp": leo_t, "frame_id": "odom",
                  "child_frame_id": "base_link", "position": T_odom_base[:3, 3].tolist(),
                  "orientation": list(rot_to_quat(T_odom_base[:3, :3]))})
            send({"type": "tag", "stamp": leo_t - 0.02, "T": T_cam_tag.tolist(),
                  "frame_id": "leo_oak_rgb_camera_optical_frame", "child_frame_id": "leo_tag0"})
            rclpy.spin_once(ros, timeout_sec=0.05)
            try:
                found = transform_msg_to_T(buf.lookup_transform("map", "leo_base",
                                                                Time()).transform)
            except Exception:  # noqa: BLE001  (not yet)
                found = None
            if found is not None and ("leo", 0) in markers:
                break
    finally:
        launch.send_signal(signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        tx.close()
    assert found is not None, "no TF map -> leo_base"
    assert np.linalg.norm(found[:3, 3] - T_map_base[:3, 3]) < 1e-3
    assert math.atan2(found[1, 0], found[0, 0]) == pytest.approx(math.radians(10), abs=1e-3)
    box, label = markers[("leo", 0)], markers[("leo", 2)]
    assert (box.pose.position.x, box.pose.position.y) == pytest.approx((0.5, 0.1), abs=1e-3)
    assert box.pose.position.z == pytest.approx(-0.78 + 0.25 / 2, abs=1e-3)   # on the floor
    assert label.text.startswith("Leo Rover")
