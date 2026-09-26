import math

from geometry_msgs.msg import Pose, Transform
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time

from g1_nav2.map_odom_tf import MapOdomTF, map_odom_pose


def test_map_odom_correction_rotates_odom_position():
    odom_base = Transform()
    odom_base.translation.x = 2.0
    odom_base.rotation.z = math.sin(math.pi / 4)
    odom_base.rotation.w = math.cos(math.pi / 4)

    map_base = Pose()
    map_base.position.x = 5.0
    map_base.position.y = 6.0
    map_base.orientation.z = 1.0  # base faces pi in the map
    map_base.orientation.w = 0.0

    x, y, heading = map_odom_pose(map_base, odom_base)

    assert math.isclose(x, 5.0, abs_tol=1e-6)
    assert math.isclose(y, 4.0, abs_tol=1e-6)
    assert math.isclose(heading, math.pi / 2, abs_tol=1e-6)


def test_navigation_map_restamps_without_changing_slam_data():
    source = OccupancyGrid()
    source.header.frame_id = "map"
    source.header.stamp.sec = 10
    source.data = [-1, 0, 100]

    class Publisher:
        messages = []

        def publish(self, message):
            self.messages.append(message)

    class Clock:
        def now(self):
            return Time(seconds=25)

    class Relay:
        map = source
        correction = (0.0, 0.0, 0.0)
        map_pub = Publisher()

        def get_clock(self):
            return Clock()

    relay = Relay()
    MapOdomTF.publish_map(relay)
    result = relay.map_pub.messages[0]
    assert result.header.stamp.sec == 25
    assert list(result.data) == [-1, 0, 100]
    assert source.header.stamp.sec == 10
