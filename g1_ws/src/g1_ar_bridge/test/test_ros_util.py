"""ROS message helpers, with duck-typed stand-ins for the messages (no ROS needed)."""
import math
import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest

from g1_ar_bridge import ros_util as ru
from g1_ar_bridge.geometry import make_T, rot_z


def pc2(points, extra_field=True):
    """PointCloud2-like: x y z float32 (+ intensity), little endian, 1 row."""
    fields = [NS(name="x", offset=0, datatype=7, count=1), NS(name="y", offset=4, datatype=7,
                                                              count=1),
              NS(name="z", offset=8, datatype=7, count=1)]
    step = 12
    if extra_field:
        fields.append(NS(name="intensity", offset=12, datatype=7, count=1))
        step = 16
    data = b"".join(struct.pack("<fff", *p) + (b"\0" * (step - 12)) for p in points)
    return NS(fields=fields, is_bigendian=False, point_step=step, row_step=step * len(points),
              width=len(points), height=1, data=data)


def test_pointcloud2_parsing_drops_nans_and_honours_the_point_step():
    pts = [(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0), (-1.5, 0.25, 0.0)]
    got = ru.pointcloud2_xyz(pc2(pts))
    assert np.allclose(got, [[1, 2, 3], [-1.5, 0.25, 0]])
    assert ru.pointcloud2_xyz(pc2([(0.0, 0.0, 1.0)], extra_field=False)).shape == (1, 3)
    no_xyz = pc2([(0.0, 0.0, 0.0)])
    no_xyz.fields = no_xyz.fields[:2]
    assert ru.pointcloud2_xyz(no_xyz).shape == (0, 3)


def test_voxel_downsample():
    pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [0.5, 0.5, 0.5]])
    assert len(ru.voxel_downsample(pts, 0.1)) == 2
    assert len(ru.voxel_downsample(pts, 0.0)) == 3


def image(encoding, arr):
    arr = np.ascontiguousarray(arr)
    h, w = arr.shape[:2]
    return NS(encoding=encoding, height=h, width=w, step=arr.strides[0], data=arr.tobytes())


def test_image_encodings():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[..., 0] = 255                               # pure red
    g_rgb = ru.image_to_gray(image("rgb8", rgb))
    g_bgr = ru.image_to_gray(image("bgr8", rgb[..., ::-1]))
    assert g_rgb.shape == (2, 3) and np.all(g_rgb == 76) and np.all(g_bgr == 76)
    mono = np.arange(6, dtype=np.uint8).reshape(2, 3)
    assert np.array_equal(ru.image_to_gray(image("mono8", mono)), mono)
    assert ru.image_to_gray(image("yuv422", mono)) is None
    depth = np.array([[0, 1234]], np.uint16)
    d = ru.depth_image_to_m(image("16UC1", depth))
    assert np.isnan(d[0, 0]) and d[0, 1] == pytest.approx(1.234)
    f = ru.depth_image_to_m(image("32FC1", np.array([[2.5, -1.0]], np.float32)))
    assert f[0, 0] == pytest.approx(2.5) and np.isnan(f[0, 1])


def marker(mtype, mid, x=0.0, y=0.0, z=0.0, yaw=0.0, text="", ns="demo", points=(),
           scale=(0.3, 0.4, 0.2), action=0, frame="map"):
    return NS(header=NS(frame_id=frame), ns=ns, id=mid, type=mtype, action=action,
              pose=NS(position=NS(x=x, y=y, z=z),
                      orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
              scale=NS(x=scale[0], y=scale[1], z=scale[2]), color=NS(r=1.0, g=0.5, b=0.0),
              lifetime=NS(sec=0, nanosec=0), text=text,
              points=[NS(x=p[0], y=p[1], z=p[2]) for p in points])


def test_markers_become_annotations_in_map():
    cur = {}
    T_map_odom = make_T(rot_z(math.pi / 2), [1.0, 0.0, 0.0])

    def lookup(frame):
        return {"map": np.eye(4), "odom": T_map_odom}.get(frame)

    ru.apply_marker_array(cur, [
        marker(ru.TEXT_VIEW_FACING, 0, 1.0, 2.0, 0.5, text="red bottle"),
        marker(ru.CUBE, 1, 2.0, 0.0, 0.4, yaw=0.3, text="box"),
        marker(ru.LINE_STRIP, 2, points=[(0, 0, 0), (1, 0, 0), (1, 1, 0)], frame="odom"),
        marker(ru.SPHERE, 3, frame="unknown_frame"),
    ], lookup)
    assert set(cur) == {"demo/0", "demo/1", "demo/2"}
    text = cur["demo/0"][0]
    assert text.kind == "marker" and text.label == "red bottle"
    assert np.allclose(text.points, [[1.0, 2.0, 0.5]])
    box = cur["demo/1"]
    assert sum(a.kind == "line" for a in box) == 6 and box[-1].label == "box"
    assert all(len(a.points) >= 3 for a in box if a.kind == "line")
    # the box's corners are 0.25 m (half diagonal of 0.3 x 0.4) from its centre, horizontally
    bottom = box[0].points
    assert np.allclose(np.hypot(bottom[:, 0] - 2.0, bottom[:, 1]), 0.25)
    line = cur["demo/2"][0]
    assert np.allclose(line.points, [[1, 0, 0], [1, 1, 0], [0, 1, 0]])   # odom -> map
    ru.apply_marker_array(cur, [marker(ru.CUBE, 1, action=ru.DELETE)], lookup)
    assert "demo/1" not in cur
    ru.apply_marker_array(cur, [marker(0, 0, action=ru.DELETEALL)], lookup)
    assert cur == {}
