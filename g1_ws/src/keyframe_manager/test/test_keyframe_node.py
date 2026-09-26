"""keyframe_node: selection rules and the FROZEN keyframe struct (AGENTS.md, B's packages)."""
import json
import os

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo

from keyframe_manager.keyframe_node import KeyframeNode

BRIDGE = CvBridge()


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def make_node(tmp_path, **params):
    values = {"output_dir": str(tmp_path / "kf"), **params}
    import keyframe_manager.keyframe_node as mod
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        kw["parameter_overrides"] = [Parameter(k, value=v) for k, v in values.items()]
        orig(self, name, **kw)
    mod.Node.__init__ = init
    try:
        return KeyframeNode()
    finally:
        mod.Node.__init__ = orig


def frame(t, sharp=True, depth_float=False, seed=0):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, (60, 80, 3)).astype(np.uint8)
    img = np.ascontiguousarray(np.repeat(np.repeat(img, 8, axis=0), 8, axis=1))  # 480x640
    if not sharp:
        img = cv2.GaussianBlur(img, (31, 31), 12)
    color = BRIDGE.cv2_to_imgmsg(img, encoding="bgr8")
    color.header.stamp.sec, color.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    color.header.frame_id = "oak_rgb_camera_optical_frame"
    if depth_float:
        d = np.full((480, 640), 1.5, np.float32)
        d[0, 0], d[0, 1], d[0, 2] = np.nan, 70.0, -1.0
        depth = BRIDGE.cv2_to_imgmsg(d, encoding="32FC1")
    else:
        depth = BRIDGE.cv2_to_imgmsg(np.full((480, 640), 1500, np.uint16), encoding="16UC1")
    depth.header = color.header
    info = CameraInfo()
    info.header = color.header
    info.width, info.height, info.distortion_model = 640, 480, "plumb_bob"
    info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    info.d = [0.1, -0.2, 0.0, 0.0, 0.0]
    return color, depth, info


def odom(x, frame_id="odom"):
    m = Odometry()
    m.header.frame_id = frame_id
    m.pose.pose.position.x = x
    return m


def test_frozen_struct(ros, tmp_path):
    n = make_node(tmp_path)
    n.on_odom(odom(0.0, "map"))
    n.on_rgbd(*frame(10.0))
    out = tmp_path / "kf"
    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["count"] == 1
    (kf,) = manifest["keyframes"]
    assert set(kf) == {"id", "dir", "blur_var", "frame_id"}
    assert kf["dir"] == "keyframe_00" and kf["frame_id"] == "oak_rgb_camera_optical_frame"
    d = out / "keyframe_00"
    assert sorted(os.listdir(d)) == ["camera_info.yaml", "depth.npy", "meta.yaml", "rgb.png"]
    depth = np.load(d / "depth.npy")
    assert depth.dtype == np.uint16 and depth.shape == (480, 640) and depth[5, 5] == 1500
    assert cv2.imread(str(d / "rgb.png")).shape == (480, 640, 3)
    info = yaml.safe_load((d / "camera_info.yaml").read_text())
    assert set(info) == {"width", "height", "distortion_model", "k", "d"}
    assert len(info["k"]) == 9
    meta = yaml.safe_load((d / "meta.yaml").read_text())
    assert {"id", "stamp", "frame_id", "blur_var", "depth_units", "depth_encoding"} <= set(meta)
    assert meta["depth_units"] == "mm" and meta["stamp"] == {"sec": 10, "nanosec": 0}
    assert meta["odom_pose"] == {"frame": "map", "position": [0.0, 0.0, 0.0]}
    n.destroy_node()


def test_float_depth_metres_to_mm_with_invalid_values_zeroed(ros, tmp_path):
    n = make_node(tmp_path)
    n.on_rgbd(*frame(1.0, depth_float=True))
    depth = np.load(tmp_path / "kf" / "keyframe_00" / "depth.npy")
    assert depth[5, 5] == 1500
    assert list(depth[0, :3]) == [0, 0, 0]  # NaN, 70 m (> uint16 mm), negative
    n.destroy_node()


def test_selection_rules(ros, tmp_path):
    n = make_node(tmp_path, min_time_gap_s=0.5, min_translation_m=0.15, max_keyframes=3)
    n.on_odom(odom(0.0))
    n.on_rgbd(*frame(1.0))                      # accept #0
    n.on_rgbd(*frame(1.2))                      # too soon
    n.on_odom(odom(0.1))
    n.on_rgbd(*frame(2.0))                      # baseline 0.1 m < 0.15 m
    n.on_odom(odom(0.3))
    n.on_rgbd(*frame(3.0, sharp=False))         # blurry
    n.on_rgbd(*frame(4.0, seed=1))              # accept #1
    n.on_odom(odom(1.0))
    n.on_rgbd(*frame(0.5, seed=2))              # bag restarted (time went back): accept #2
    n.on_odom(odom(2.0))
    n.on_rgbd(*frame(9.0, seed=3))              # max_keyframes reached
    stamps = [yaml.safe_load((tmp_path / "kf" / k["dir"] / "meta.yaml").read_text())["stamp"]
              for k in n.manifest]
    assert [s["sec"] for s in stamps] == [1, 4, 0]
    n.destroy_node()


def test_no_odom_skips_baseline(ros, tmp_path):
    n = make_node(tmp_path, min_time_gap_s=0.5)
    n.on_rgbd(*frame(1.0))
    n.on_rgbd(*frame(2.0, seed=1))
    assert len(n.manifest) == 2
    meta = yaml.safe_load((tmp_path / "kf" / "keyframe_01" / "meta.yaml").read_text())
    assert "odom_pose" not in meta
    n.destroy_node()
