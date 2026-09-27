"""semantic_query end to end in ROS with the mock detector (no GPU): a query from the glasses'
topic starts a search, the node publishes a 3D box on /ar_glasses/markers, a reply and a POI.
Uses ROS_DOMAIN_ID 77 unless set (never the robot's 0)."""
import json
import os
import shutil
import signal
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
if shutil.which("ros2") is None:
    pytest.skip("ros2 CLI not found", allow_module_level=True)
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
    os.environ["ROS_DOMAIN_ID"] = "77"


def test_search_from_the_glasses_marks_a_box():
    procs = [subprocess.Popen(cmd, start_new_session=True) for cmd in (
        ["ros2", "run", "semantic_query", "fake_oak_pub"],
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "map",
         "--child-frame-id", "oak_rgb_camera_optical_frame"],
        ["ros2", "launch", "semantic_query", "semantic_query.launch.py", "backend:=mock"])]
    rclpy.init()
    node = rclpy.create_node("search_test")
    got = {"markers": {}, "replies": [], "poi": []}
    node.create_subscription(MarkerArray, "/ar_glasses/markers", lambda m: got["markers"].update(
        {(x.ns, x.id): x for x in m.markers}), 10)
    node.create_subscription(String, "/ar_glasses/reply", lambda m: got["replies"].append(m.data), 10)
    node.create_subscription(String, "/semantic_query/poi", lambda m: got["poi"].append(m.data), 10)
    cmd = node.create_publisher(String, "/ar_glasses/user_command", 10)
    try:
        end, sent = time.time() + 40, 0.0
        while time.time() < end and not got["poi"]:
            if time.time() - sent > 3.0:                     # until the node is up
                cmd.publish(String(data="search for a red cup"))
                sent = time.time()
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in procs:
            p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    assert got["poi"], f"no POI; replies: {got['replies']}"
    poi = json.loads(got["poi"][-1])
    assert poi["label"] == "red cup" and poi["frame_id"] == "map"
    # fake OAK: flat depth 1.5 m; identity TF map == optical frame -> box ~1.5 m along z
    assert poi["box"]["center"][2] == pytest.approx(1.5, abs=0.05)
    box = got["markers"][("semantic_query", 0)]
    assert box.type == box.CUBE and box.text.startswith("red cup")
    assert any(r.startswith("Searching for red cup") for r in got["replies"])
    assert any(r.startswith("Found red cup") for r in got["replies"])
