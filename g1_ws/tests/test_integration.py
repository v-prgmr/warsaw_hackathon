"""End-to-end: the real launch files against the synthetic G1 (tests/sim_g1.py).

Needs a built + sourced workspace with rtabmap_ros, imu_complementary_filter and unitree_hg
(the project Docker image). Each test runs ~40 s. Run only in an isolated DDS domain:
    SIM=1 scripts/run_humble.sh scripts/run_tests.sh --integration
"""
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("rtabmap_msgs.msg")
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rtabmap_msgs.msg import OdomInfo  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
import tf2_ros  # noqa: E402

SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_g1.py")
DURATION = 30.0


@pytest.fixture(scope="module", autouse=True)
def isolated_domain():
    if (os.environ.get("ROS_DOMAIN_ID", "0") or "0") == "0":
        pytest.skip("refusing to publish robot topics on ROS_DOMAIN_ID 0 (AGENTS.md §19)")
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Proc:
    def __init__(self, cmd, log):
        self.log = open(log, "w")
        self.p = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT,
                                  start_new_session=True)

    def stop(self):
        if self.p.poll() is None:
            os.killpg(self.p.pid, signal.SIGINT)
            try:
                self.p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)
        self.log.close()


class Monitor:
    def __init__(self):
        self.node = rclpy.create_node("integration_monitor")
        self.odom, self.truth, self.info, self.joint_states = [], [], [], []
        self.lowstate_count = 0
        self.map = None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self.node)
        n = self.node
        n.create_subscription(Odometry, "/odom", lambda m: self.odom.append(m), 100)
        n.create_subscription(Odometry, "/dog_odom", lambda m: self.truth.append(m), 1000)
        n.create_subscription(OdomInfo, "/odom_info", lambda m: self.info.append(m), 100)
        n.create_subscription(JointState, "/joint_states",
                              lambda m: self.joint_states.append(
                                  (m, n.get_clock().now().nanoseconds)), 100)
        try:
            from unitree_hg.msg import LowState
            from rclpy.qos import qos_profile_sensor_data
            n.create_subscription(LowState, "/lf/lowstate",  # g1_sensors' default input
                                  lambda m: setattr(self, "lowstate_count",
                                                    self.lowstate_count + 1),
                                  qos_profile_sensor_data)
        except ImportError:
            pass
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), latched)

    def wait_for_publishers(self, topics, timeout=60.0):
        """Block until every topic has a publisher (the launched nodes are up)."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(self.node.count_publishers(t) for t in topics):
                return
        missing = [t for t in topics if not self.node.count_publishers(t)]
        raise AssertionError(f"launch not ready after {timeout:.0f} s: no publisher on {missing}")

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def close(self):
        self.node.destroy_node()


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def dump(mon, path):
    """odom vs truth per /odom message, for debugging a failed run."""
    with open(path, "w") as f:
        f.write("t odom_x odom_y odom_yaw truth_x truth_y truth_yaw\n")
        for m in mon.odom:
            t = stamp_s(m)
            g = min(mon.truth, key=lambda x: abs(stamp_s(x) - t))
            p, q = m.pose.pose.position, g.pose.pose.position
            f.write(f"{t:.3f} {p.x:.3f} {p.y:.3f} {yaw(m.pose.pose.orientation):.3f} "
                    f"{q.x:.3f} {q.y:.3f} {yaw(g.pose.pose.orientation):.3f}\n")


def tracking_error(mon):
    """Motion since the first /odom pose vs the simulated ground truth over the same stamps.

    icp_odometry starts at identity once the IMU is available, so the robot may already have
    walked a bit: compare relative motion, not absolute positions.
    """
    assert len(mon.odom) > 100, f"only {len(mon.odom)} /odom messages"

    def truth_at(t):
        g = min(mon.truth, key=lambda m: abs(stamp_s(m) - t))
        assert abs(stamp_s(g) - t) < 0.01
        return g

    def pose(m):
        p = m.pose.pose.position
        return np.array([p.x, p.y]), yaw(m.pose.pose.orientation)

    (o0, oy0), (o1, oy1) = pose(mon.odom[0]), pose(mon.odom[-1])
    (g0, gy0) = pose(truth_at(stamp_s(mon.odom[0])))
    (g1, gy1) = pose(truth_at(stamp_s(mon.odom[-1])))
    c, s_ = math.cos(gy0 - oy0), math.sin(gy0 - oy0)
    moved_odom = np.array([[c, -s_], [s_, c]]) @ (o1 - o0)  # odom motion in the truth frame
    dist = float(np.linalg.norm(moved_odom - (g1 - g0)))
    dyaw = math.degrees(abs(math.remainder((oy1 - oy0) - (gy1 - gy0), 2 * math.pi)))
    t0, t1 = stamp_s(mon.odom[0]), stamp_s(mon.odom[-1])
    path = [pose(m)[0] for m in mon.truth if t0 <= stamp_s(m) <= t1]
    travelled = float(sum(np.linalg.norm(b - a) for a, b in zip(path[:-1], path[1:])))
    return dist, dyaw, travelled


def run(tmp_path, launches, sim_args=(), ready_topics=("/odom", "/map"), settle=3.0):
    procs = [Proc(cmd, tmp_path / f"launch_{i}.log") for i, cmd in enumerate(launches)]
    mon = Monitor()
    try:
        # wait for the nodes (a cold container can take > 5 s), then let them finish setting up
        mon.wait_for_publishers(ready_topics)
        mon.spin(settle)
        sim = Proc([sys.executable, SIM, "--duration", str(DURATION), *sim_args],
                   tmp_path / "sim.log")
        while sim.p.poll() is None:
            mon.spin(0.5)
        sim.stop()
        mon.spin(3.0)
        dump(mon, tmp_path / "trajectory.txt")
        return mon
    finally:
        for p in procs:
            p.stop()


def mapping(tmp_path, *args):
    return ["ros2", "launch", "g1_mapping", "mapping.launch.py",
            f"database_path:={tmp_path / 'map.db'}", *args]


@pytest.mark.parametrize("imu_source", ["dog", "livox"])
def test_icp_mapping_tracks_the_walk(tmp_path, imu_source):
    mon = run(tmp_path, [mapping(tmp_path, f"imu_source:={imu_source}")])
    try:
        dist, dyaw, travelled = tracking_error(mon)
        assert travelled > 7.0
        assert dist < 0.03 * travelled, f"drift {dist:.2f} m over {travelled:.1f} m"
        assert dyaw < 3.0
        lost = sum(m.lost for m in mon.info[5:])  # first scans: before the IMU TF exists
        assert lost == 0, f"{lost} lost ICP scans"
        assert mon.map is not None and mon.map.info.width > 0, "no /map"
        grid = np.array(mon.map.data)
        assert (grid == 100).sum() > 100 and (grid == 0).sum() > 1000
        mon.tf.lookup_transform("map", "robot_center", Time())
    finally:
        mon.close()


def test_dog_odom_fallback(tmp_path):
    mon = run(tmp_path, [mapping(tmp_path, "odom_source:=dog_odom")])
    try:
        dist, _, _ = tracking_error(mon)  # /odom is /dog_odom republished
        assert dist < 0.01
        mon.tf.lookup_transform("odom", "robot_center", Time())
        assert mon.map is not None
    finally:
        mon.close()


def test_tf_chain_on_robot_clock_with_mapping(tmp_path):
    """g1_sensors tf_chain + g1_mapping static_tf:=false, robot clock 73 s behind the host."""
    offset = -73.0
    mon = run(tmp_path, [["ros2", "launch", "g1_sensors", "tf_chain.launch.py"],
                         mapping(tmp_path, "static_tf:=false")],
              sim_args=("--lowstate", "--clock-offset", str(offset)),
              ready_topics=("/odom", "/map", "/joint_states"))
    try:
        assert len(mon.joint_states) > 100
        rate, rate_in = len(mon.joint_states) / DURATION, mon.lowstate_count / DURATION
        expected = min(50.0, rate_in)  # publish_rate 50 Hz, or every message if slower
        assert 0.8 * expected < rate < 55, \
            f"/joint_states at {rate:.0f} Hz from /lowstate at {rate_in:.0f} Hz"
        js, received_ns = mon.joint_states[-1]
        lag = received_ns * 1e-9 + offset - stamp_s(js)
        assert abs(lag) < 0.2, f"joint states not on the robot clock (off by {lag:.2f} s)"
        tr = mon.tf.lookup_transform("robot_center", "livox_frame", Time())
        assert tr.transform.translation.z == pytest.approx(0.472, abs=0.01)
        q = tr.transform.rotation
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)))
        assert abs(abs(roll) - 180.0) < 1.0  # upside down
        dist, _, travelled = tracking_error(mon)
        # The sim mounts the LiDAR at g1_mapping's legacy extrinsic, the URDF differs by ~4 cm/3 deg
        assert dist < 0.05 * travelled, f"drift {dist:.2f} m over {travelled:.1f} m"
        assert mon.map is not None
    finally:
        mon.close()
