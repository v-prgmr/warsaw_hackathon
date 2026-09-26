"""scene_server stub: latched /scene_cloud in `map` with x y z rgb (M3 contract)."""
import numpy as np
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from sensor_msgs_py import point_cloud2

from scene_server.ply_publisher import PlyPublisher, _make_synthetic_cloud, _pack_rgb


class Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_pack_rgb_roundtrip():
    v = np.array([_pack_rgb(230, 120, 30)], dtype=np.float32).view(np.uint32)[0]
    assert ((v >> 16) & 255, (v >> 8) & 255, v & 255) == (230, 120, 30)


def test_synthetic_scene_contents():
    pts = np.array([p[:3] for p in _make_synthetic_cloud()])
    assert len(pts) > 10000
    assert pts[:, 2].min() == 0.0 and pts[:, 2].max() == pytest.approx(0.58)


def test_publishes_latched_cloud_in_map(ros):
    node = PlyPublisher()
    assert node.pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert node.pub.topic_name == "/scene_cloud"
    node.pub = Capture()
    node._publish_cloud()
    (msg,) = node.pub.msgs
    assert msg.header.frame_id == "map"
    assert [f.name for f in msg.fields] == ["x", "y", "z", "rgb"]
    assert msg.width == len(node.points)
    first = next(iter(point_cloud2.read_points(msg, field_names=("x", "y", "z"))))
    assert tuple(first) == pytest.approx(node.points[0][:3])
    node.destroy_node()
