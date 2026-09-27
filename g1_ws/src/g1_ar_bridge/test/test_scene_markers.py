"""Room outline on the floor, robot label and tag pin (floor_outline, scene_markers_node)."""
import math
import os
import shutil
import signal
import subprocess
import time

import numpy as np
import pytest

from g1_ar_bridge.floor_outline import outline_polylines, signature

RES = 0.05


def room_grid(w_m=5.0, h_m=4.0, margin_m=0.5, gap=True):
    """Occupancy grid (origin at -margin) of a rectangular room with a door gap and a table."""
    W, H = int((w_m + 2 * margin_m) / RES), int((h_m + 2 * margin_m) / RES)
    g = np.full((H, W), 0, dtype=np.int8)
    m = int(margin_m / RES)
    x0, y0, x1, y1 = m, m, m + int(w_m / RES), m + int(h_m / RES)
    g[y0, x0:x1 + 1] = 100
    g[y1, x0:x1 + 1] = 100
    g[y0:y1 + 1, x0] = 100
    g[y0:y1 + 1, x1] = 100
    if gap:
        g[y0, x0 + 20:x0 + 36] = 0                     # 0.8 m door
    g[y0 + 30:y0 + 44, x0 + 40:x0 + 60] = 100          # a table (0.7 x 1.0 m)
    g[5, 5] = 100                                      # a speck: dropped (too short)
    return g, (-margin_m, -margin_m)


def test_room_outline_is_a_few_straight_lines_of_the_right_size():
    g, origin = room_grid()
    lines = outline_polylines(g.ravel(), g.shape[1], g.shape[0], RES, origin)
    assert 2 <= len(lines) <= 6
    pts = np.vstack(lines)
    assert sum(len(p) for p in lines) < 60                          # simplified
    assert pts[:, 0].min() == pytest.approx(0.0, abs=0.1)           # walls at 0..5 x 0..4 m
    assert pts[:, 0].max() == pytest.approx(5.0, abs=0.1)
    assert pts[:, 1].min() == pytest.approx(0.0, abs=0.1)
    assert pts[:, 1].max() == pytest.approx(4.0, abs=0.1)
    table = [p for p in lines if p[:, 0].max() < 3.5 and p[:, 1].min() > 1.0]
    assert table, "the table outline is kept"


def test_outline_respects_the_caps_and_rotation():
    g, origin = room_grid()
    few = outline_polylines(g.ravel(), g.shape[1], g.shape[0], RES, origin, max_lines=1)
    assert len(few) == 1
    rot = outline_polylines(g.ravel(), g.shape[1], g.shape[0], RES, (0.0, 0.0),
                            origin_yaw=math.pi / 2)
    assert np.vstack(rot)[:, 0].max() <= 0.05                       # grid x -> map y
    assert outline_polylines(np.zeros(100), 10, 10, RES, (0, 0)) == []
    assert signature(g.ravel()) != signature(room_grid(gap=False)[0].ravel())


rclpy = pytest.importorskip("rclpy")


@pytest.mark.skipif(shutil.which("ros2") is None, reason="ros2 CLI not found")
def test_scene_markers_node_publishes_outline_labels_and_pins():
    if os.environ.get("ROS_DOMAIN_ID", "0") in ("", "0"):
        os.environ["ROS_DOMAIN_ID"] = "77"
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import BatteryState
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import MarkerArray

    proc = subprocess.Popen(["ros2", "run", "g1_ar_bridge", "scene_markers"],
                            start_new_session=True)
    rclpy.init()
    n = rclpy.create_node("scene_markers_test")
    got = {}
    n.create_subscription(MarkerArray, "/ar_glasses/markers",
                          lambda msg: got.update({(m.ns, m.id): m for m in msg.markers}), 10)
    map_pub = n.create_publisher(OccupancyGrid, "/map", 1)
    bat_pub = n.create_publisher(BatteryState, "/battery_state", 10)
    tfs = StaticTransformBroadcaster(n)
    robot = TransformStamped()
    robot.header.frame_id, robot.child_frame_id = "map", "robot_center"
    robot.transform.translation.x, robot.transform.translation.z = 2.0, 0.0
    robot.transform.rotation.w = 1.0
    tag = TransformStamped()
    tag.header.frame_id, tag.child_frame_id = "map", "ar_tag_0"
    tag.transform.translation.x, tag.transform.translation.z = 5.0, 0.3
    tag.transform.rotation.w = 1.0
    tfs.sendTransform([robot, tag])
    g, origin = room_grid()
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = RES
    grid.info.width, grid.info.height = g.shape[1], g.shape[0]
    grid.info.origin.position.x, grid.info.origin.position.y = origin
    grid.info.origin.orientation.w = 1.0
    grid.data = g.ravel().tolist()
    try:
        end = time.time() + 30
        while time.time() < end and not ({("robot_label", 0), ("tag_label", 0)} <= got.keys()
                                         and any(k[0] == "floor_outline" for k in got)
                                         and "%" in got[("robot_label", 0)].text):
            map_pub.publish(grid)
            bat_pub.publish(BatteryState(percentage=0.7))
            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.destroy_node()
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)          # like Ctrl-C: the node too, not only ros2 run
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
    lines = [m for k, m in got.items() if k[0] == "floor_outline"]
    assert lines and all(m.type == m.LINE_STRIP for m in lines)
    assert all(abs(p.z - (0.0 - 0.78 + 0.02)) < 1e-6 for m in lines for p in m.points)  # floor
    assert got[("robot_label", 0)].text == "Unitree G1 · 70%"
    assert got[("robot_label", 0)].pose.position.z == pytest.approx(0.65)
    assert got[("tag_label", 0)].text == "AprilTag 0"
