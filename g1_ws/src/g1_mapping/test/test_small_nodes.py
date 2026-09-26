"""livox_imu_fix (g -> m/s^2, orientation unknown) and odom_to_tf (/dog_odom -> TF)."""
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from g1_mapping.livox_imu_fix import LivoxImuFix
from g1_mapping.odom_to_tf import OdomToTf


class Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    sendTransform = publish  # noqa: N815 (TransformBroadcaster API)


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_livox_imu_fix(ros):
    node = LivoxImuFix()
    node.pub = Capture()
    imu = Imu()
    imu.header.frame_id = "livox_frame"
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = \
        0.01, -0.02, -1.0
    imu.angular_velocity.z = 0.3
    node.cb(imu)
    out = node.pub.msgs[0]
    assert out.header.frame_id == "livox_frame"
    assert out.linear_acceleration.z == pytest.approx(-9.80665)
    assert out.linear_acceleration.x == pytest.approx(0.0980665)
    assert out.angular_velocity.z == pytest.approx(0.3)
    assert out.orientation_covariance[0] == -1.0  # REP-145: orientation unknown
    node.destroy_node()


@pytest.mark.parametrize("republish", [True, False])
def test_odom_to_tf(ros, republish):
    node = OdomToTf()
    node.republish = republish
    node.br = Capture()
    node.pub = Capture() if republish else None
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.header.stamp.sec = 7
    odom.child_frame_id = "robot_center"
    odom.pose.pose.position.x, odom.pose.pose.position.z = 1.5, 0.74
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = 0.6, 0.8
    node.cb(odom)
    (tf,) = node.br.msgs
    assert (tf.header.frame_id, tf.child_frame_id, tf.header.stamp.sec) == \
        ("odom", "robot_center", 7)
    assert tf.transform.translation.x == 1.5 and tf.transform.translation.z == 0.74
    assert tf.transform.rotation.z == 0.6 and tf.transform.rotation.w == 0.8
    if republish:
        assert node.pub.msgs == [odom]
    node.destroy_node()
