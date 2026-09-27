"""Draw the G1 as an articulated stick figure ("x-ray") on /ar_glasses/markers.

The glasses show the robot as a single marker at ``robot_center``. This node overlays the whole
body: it reads the URDF link tree and looks up every link in ``map`` through /tf, then publishes
one LINE_LIST marker whose segments are the bones (parent link origin -> child link origin). As
the robot walks, robot_state_publisher updates /tf from /joint_states (which the g1_sensors
tf_chain regenerates from /lf/lowstate), so the skeleton articulates with it. Works live and from
a bag (with tf_chain + the mapper running, so ``map -> ... -> each link`` resolves).

No new protocol or Lens change: the AR bridge already forwards LINE_LIST markers as line
annotations to the glasses (ros_util.marker_to_annotations). The forward kinematics is
robot_state_publisher's; this node only reads the resulting transforms.

The URDF comes from the latched /robot_description (what robot_state_publisher publishes), or from
``urdf_path`` if that is set. ``root_frame`` (default ``map``) is what the bones are expressed in;
``exclude`` drops links whose name contains any of the given substrings (e.g. camera glue frames).
"""
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .lifetime import exit_with_parent
from .skeleton import chain_point_lists, chains, parse_bones


class SkeletonNode(Node):
    def __init__(self):
        super().__init__("ar_skeleton")
        self.root_frame = self.declare_parameter("root_frame", "map").value
        rate = float(self.declare_parameter("rate_hz", 10.0).value)
        self.line_width = float(self.declare_parameter("line_width_m", 0.02).value)
        self.ns = self.declare_parameter("ns", "skeleton").value
        self.color = [float(c) for c in self.declare_parameter("color", [0.2, 0.9, 1.0]).value]
        self.lifetime_s = float(self.declare_parameter("lifetime_s", 0.5).value)
        self.exclude = [s for s in self.declare_parameter("exclude", [""]).value if s]
        urdf_path = self.declare_parameter("urdf_path", "").value

        self.bones = []
        self.links = set()
        self.last_marker_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, "/ar_glasses/markers", 10)
        self.warned_empty = False

        if urdf_path:
            with open(urdf_path) as f:
                self.set_urdf(f.read())
        else:
            # robot_state_publisher latches /robot_description (transient_local, depth 1).
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(String, "robot_description", self.on_description, qos)

        self.create_timer(1.0 / rate if rate > 0 else 0.1, self.tick)

    def on_description(self, msg):
        self.set_urdf(msg.data)

    def set_urdf(self, urdf_xml):
        bones = [(p, c) for p, c in parse_bones(urdf_xml)
                 if not self._excluded(p) and not self._excluded(c)]
        self.bones = bones
        self.links = {name for bone in bones for name in bone}
        self.get_logger().info(f"URDF loaded: {len(self.links)} links, {len(bones)} bones, "
                               f"{len(chains(bones))} limb strokes")

    def _excluded(self, link):
        return any(s in link for s in self.exclude)

    def tick(self):
        if not self.bones:
            self.get_logger().info("waiting for /robot_description", throttle_duration_sec=5.0)
            return
        positions = {}
        for link in self.links:
            try:
                tf = self.tf_buffer.lookup_transform(self.root_frame, link, Time())
            except TransformException:
                continue
            t = tf.transform.translation
            positions[link] = (t.x, t.y, t.z)
        strokes = chain_point_lists(self.bones, positions)
        if not strokes:
            if not self.warned_empty:
                self.get_logger().warn(f"no link resolves to '{self.root_frame}' yet "
                                       "(is tf_chain + the mapper running?)")
                self.warned_empty = True
            return
        self.warned_empty = False
        markers = [self.build_marker(i, pts) for i, pts in enumerate(strokes)]
        # a stroke can vanish when a frame drops out: delete the ids we no longer publish
        markers += [self.delete_marker(i) for i in range(len(strokes), self.last_marker_count)]
        self.last_marker_count = len(strokes)
        self.pub.publish(MarkerArray(markers=markers))

    def build_marker(self, mid, pts):
        m = Marker()
        m.header.frame_id = self.root_frame
        m.ns, m.id, m.type, m.action = self.ns, mid, Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.line_width
        m.color.r, m.color.g, m.color.b = self.color
        m.color.a = 1.0
        if self.lifetime_s > 0:
            m.lifetime = Duration(seconds=self.lifetime_s).to_msg()
        m.points = [Point(x=x, y=y, z=z) for x, y, z in pts]
        return m

    def delete_marker(self, mid):
        m = Marker()
        m.header.frame_id = self.root_frame
        m.ns, m.id, m.action = self.ns, mid, Marker.DELETE
        return m


def main():
    exit_with_parent()
    rclpy.init()
    try:
        rclpy.spin(SkeletonNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
