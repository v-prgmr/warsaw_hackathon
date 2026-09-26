"""calib_capture on live topics: synthetic cameras + LiDAR + TF -> one complete sample on disk."""
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from std_srvs.srv import Trigger
import tf2_ros
from geometry_msgs.msg import TransformStamped
import yaml

from g1_calibration import capture_node
from g1_calibration.dataset import load_dataset
from g1_calibration.geometry import invert, quaternion_from_matrix, T_BODY_OPTICAL

from synthetic import (BODY_LIDAR, BODY_RS_OPT, FRAMES, OAK, RS, TRUE_BODY_OAK_OPT, board_poses,
                       config, lidar_points, render)

cv_bridge = pytest.importorskip("cv_bridge")


def info_msg(intr, frame):
    m = CameraInfo()
    m.header.frame_id = frame
    m.width, m.height = intr.width, intr.height
    m.k = [float(v) for v in intr.k.ravel()]
    m.d = [float(v) for v in intr.d]
    m.distortion_model = "plumb_bob"
    return m


def cloud_msg(points):
    # Unitree-like layout with extra fields: x y z intensity (f32), ring (u16), time (f32)
    dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                   "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                   "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 22})
    arr = np.zeros(len(points), dt)
    arr["x"], arr["y"], arr["z"] = points.T
    m = PointCloud2()
    m.header.frame_id = "livox_frame"
    m.height, m.width = 1, len(points)
    m.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in
                [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4),
                 ("time", 18, 7)]]
    m.point_step, m.row_step, m.data = 22, 22 * len(points), arr.tobytes()
    return m


def tf_msg(parent, child, t):
    m = TransformStamped()
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = t[:3, 3]
    q = quaternion_from_matrix(t[:3, :3])
    (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
     m.transform.rotation.w) = q
    return m


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_capture_writes_a_complete_sample(ros, tmp_path):
    cfg_path = tmp_path / "calibration.yaml"
    cfg_path.write_text(yaml.safe_dump(config()))
    out = tmp_path / "ds"
    orig = rclpy.node.Node.__init__

    def init(self, name, **kw):
        if name == "calib_capture":
            kw["parameter_overrides"] = [Parameter("config", value=str(cfg_path)),
                                         Parameter("output_dir", value=str(out)),
                                         Parameter("lidar_seconds", value=1.0)]
        orig(self, name, **kw)
    capture_node.Node.__init__ = init
    try:
        node = capture_node.CalibCapture()
    finally:
        capture_node.Node.__init__ = orig

    rng = np.random.default_rng(0)
    (t_body_board,) = board_poses(1, rng)
    bridge = cv_bridge.CvBridge()
    imgs = {}
    for cam, intr, t in (("oak", OAK, TRUE_BODY_OAK_OPT), ("realsense", RS, BODY_RS_OPT)):
        m = bridge.cv2_to_imgmsg(render(intr, invert(t) @ t_body_board, rng), "bgr8")
        m.header.frame_id = FRAMES[cam]
        imgs[cam] = (m, info_msg(intr, FRAMES[cam]))
    cloud = cloud_msg(lidar_points(t_body_board, rng).astype(np.float32))

    pub_node = rclpy.create_node("fake_sensors")
    qos = rclpy.qos.qos_profile_sensor_data
    pubs = {cam: (pub_node.create_publisher(type(m), config()["cameras"][cam]["image"], qos),
                  pub_node.create_publisher(CameraInfo, config()["cameras"][cam]["camera_info"],
                                            qos))
            for cam, (m, _) in imgs.items()}
    lidar_pub = pub_node.create_publisher(PointCloud2, "/utlidar/cloud_livox_mid360", qos)
    static = tf2_ros.StaticTransformBroadcaster(pub_node)
    static.sendTransform([
        tf_msg("torso_link", "livox_frame", BODY_LIDAR),
        tf_msg("livox_frame", FRAMES["realsense"], invert(BODY_LIDAR) @ BODY_RS_OPT),
        tf_msg("oak-d-base-frame", FRAMES["oak"], T_BODY_OPTICAL)])

    def tick():
        for cam, (img, info) in imgs.items():
            pubs[cam][1].publish(info)
            pubs[cam][0].publish(img)
        lidar_pub.publish(cloud)
    pub_node.create_timer(0.1, tick)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(pub_node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        time.sleep(1.0)
        res = node.on_capture(Trigger.Request(), Trigger.Response())
    finally:
        executor.shutdown()
        node.destroy_node()
        pub_node.destroy_node()
    assert res.success, res.message
    assert "oak: board OK" in res.message and "realsense: board OK" in res.message
    assert "TF 3/3" in res.message
    (s,) = load_dataset(str(out))
    assert set(s.images) == {"oak", "realsense"}
    assert s.frames == FRAMES
    assert len(s.lidar) > 5 * 1000  # several scans accumulated, ranges filtered
    np.testing.assert_allclose(s.transform("torso_link", "livox_frame"), BODY_LIDAR, atol=1e-6)
    np.testing.assert_allclose(s.intrinsics["oak"].k, OAK.k)
