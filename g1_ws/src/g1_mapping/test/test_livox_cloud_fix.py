"""livox_cloud_fix: MID-360 time ns -> s, zero / out-of-range points dropped, output layout."""
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2, PointField

from g1_mapping.livox_cloud_fix import LivoxCloudFix

F32, U16 = PointField.FLOAT32, PointField.UINT16


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


@pytest.fixture
def node(ros):
    n = LivoxCloudFix()
    n.pub = Capture()
    yield n
    n.destroy_node()


def g1_cloud(rows, packed=True, height=1):
    """rows: (x, y, z, intensity, ring, time_ns). Unitree field order, packed or 4-byte aligned."""
    if packed:
        offsets, step = [0, 4, 8, 12, 16, 18], 22
    else:
        offsets, step = [0, 4, 8, 12, 16, 20], 24
    names = ["x", "y", "z", "intensity", "ring", "time"]
    types = [F32, F32, F32, F32, U16, F32]
    np_types = ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"]
    dt = np.dtype({"names": names, "formats": np_types, "offsets": offsets, "itemsize": step})
    arr = np.zeros(len(rows), dtype=dt)
    for i, r in enumerate(rows):
        arr[i] = r
    msg = PointCloud2()
    msg.header.frame_id = "livox_frame"
    msg.header.stamp.sec = 42
    msg.height, msg.width = height, len(rows) // height
    msg.fields = [PointField(name=n, offset=o, datatype=t, count=1)
                  for n, o, t in zip(names, offsets, types)]
    msg.point_step, msg.row_step = step, step * msg.width
    msg.data = arr.tobytes()
    return msg


def out_points(msg):
    dt = np.dtype({"names": [f.name for f in msg.fields],
                   "formats": ["<f4" if f.datatype == F32 else "<u2" for f in msg.fields],
                   "offsets": [f.offset for f in msg.fields], "itemsize": msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)


@pytest.mark.parametrize("packed", [True, False])
def test_converts_time_and_drops_zero_points(node, packed):
    rows = [(1.0, 2.0, 0.5, 10.0, 1, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0, 5.0e7),        # (0,0,0) placeholder
            (-3.0, 0.5, -1.2, 20.0, 3, 9.9e7)]
    node.cb(g1_cloud(rows, packed))
    assert len(node.pub.msgs) == 1
    out = node.pub.msgs[0]
    assert out.header.frame_id == "livox_frame" and out.header.stamp.sec == 42
    assert [f.name for f in out.fields] == ["x", "y", "z", "intensity", "time", "ring"]
    assert out.point_step == 24 and out.row_step == 24 * out.width and out.height == 1
    pts = out_points(out)
    assert len(pts) == 2
    np.testing.assert_allclose(pts["time"], [0.0, 0.099], atol=1e-7)
    np.testing.assert_allclose(pts["x"], [1.0, -3.0])
    np.testing.assert_allclose(pts["intensity"], [10.0, 20.0])
    assert list(pts["ring"]) == [1, 3]


def test_range_filter_and_non_finite(node):
    rows = [(0.1, 0.0, 0.0, 1, 0, 0),          # closer than range_min (0.3 m): self-hit
            (40.0, 0.0, 0.0, 1, 0, 0),         # beyond range_max
            (np.nan, 1.0, 1.0, 1, 0, 0),
            (2.0, 0.0, 0.0, 1, 0, 0)]
    node.cb(g1_cloud(rows))
    pts = out_points(node.pub.msgs[0])
    assert len(pts) == 1 and pts["x"][0] == 2.0


def test_organized_cloud(node):
    rows = [(1.0 + i, 0.0, 0.0, 1, i % 4, i * 1e6) for i in range(6)]
    node.cb(g1_cloud(rows, height=2))
    out = node.pub.msgs[0]
    assert out.height == 1 and out.width == 6


def test_rejects_missing_time_field(node):
    msg = g1_cloud([(1.0, 0.0, 0.0, 1, 0, 0)])
    msg.fields = [f for f in msg.fields if f.name != "time"]
    node.cb(msg)
    assert node.pub.msgs == []


def test_rejects_padded_rows_and_ignores_empty(node):
    msg = g1_cloud([(1.0, 0.0, 0.0, 1, 0, 0)])
    msg.row_step += 8
    node.cb(msg)
    empty = g1_cloud([(1.0, 0.0, 0.0, 1, 0, 0)])
    empty.width, empty.data = 0, b""
    node.cb(empty)
    assert node.pub.msgs == []


def test_all_points_filtered_publishes_empty_cloud(node):
    node.cb(g1_cloud([(0.0, 0.0, 0.0, 0, 0, 0)] * 3))
    assert node.pub.msgs[0].width == 0 and len(node.pub.msgs[0].data) == 0


def test_output_readable_by_sensor_msgs_py(node):
    from sensor_msgs_py import point_cloud2
    node.cb(g1_cloud([(1.0, 2.0, 3.0, 4.0, 2, 5.0e7)]))
    (p,) = list(point_cloud2.read_points(node.pub.msgs[0],
                                         field_names=("x", "y", "z", "time", "ring")))
    assert tuple(p)[:3] == (1.0, 2.0, 3.0)
    assert abs(p[3] - 0.05) < 1e-7 and p[4] == 2
