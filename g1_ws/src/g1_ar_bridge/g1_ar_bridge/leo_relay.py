"""Read-only relay ON THE LEO ROVER: its tag detection + odometry -> UDP JSON to the laptop.

Leo runs ROS 2 Jazzy + Fast DDS on its own network; the G1 laptop runs Humble + CycloneDDS
(g1_ws/docs/leo_g1_laptop_integration.md §1). This script is the explicit, minimal link: it
only READS Leo's graph (TF ``leo_oak_rgb_camera_optical_frame -> leo_tag0`` from Leo's
AprilTag detector, and ``/leo/merged_odom``) and sends them to the laptop's ``leo_in_map``.
It creates no publishers and changes nothing on Leo. Standalone: no install needed, e.g.

    scp g1_ws/src/g1_ar_bridge/g1_ar_bridge/leo_relay.py pi@10.0.0.1:/tmp/g1_leo_relay.py
    ssh -t pi@10.0.0.1 'source /opt/ros/jazzy/setup.bash && python3 /tmp/g1_leo_relay.py --laptop-ip 10.0.0.23'

(``ssh -t``: Ctrl-C or a dropped connection also stops it on Leo; ``scripts/start_ar_glasses.sh
--leo`` does both steps in one window). Ask the Leo team before running it.
Leo's TF on other topics: append ``--ros-args -r /tf:=/leo/tf -r /tf_static:=/leo/tf_static``.
"""
import argparse
import json
import socket
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def matrix(tf):
    t, q = tf.translation, tf.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
            [0.0, 0.0, 0.0, 1.0]]


class LeoRelay(Node):
    def __init__(self, args):
        super().__init__("g1_leo_relay")
        self.args = args
        self.dest = (args.laptop_ip, args.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(1.0 / args.tag_rate, self.poll_tag)
        self.create_timer(2.0, self.heartbeat)
        self.last_tag_stamp = None
        self.last_odom_sent = 0.0
        self.counts = {"tag": 0, "odom": 0}
        self.get_logger().info(
            f"relaying {args.camera_frame} -> {args.tag_frame} and {args.odom_topic} to "
            f"{args.laptop_ip}:{args.port} (UDP JSON; read-only)")

    def send(self, msg):
        try:
            self.sock.sendto((json.dumps(msg) + "\n").encode(), self.dest)
        except OSError as exc:
            self.get_logger().warning(f"send failed: {exc}", throttle_duration_sec=5.0)

    def poll_tag(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.args.camera_frame, self.args.tag_frame,
                                                 Time())
        except TransformException:
            return
        stamp = stamp_s(tf.header.stamp)
        if stamp == self.last_tag_stamp:          # the detector's TF only changes when it sees it
            return
        self.last_tag_stamp = stamp
        self.counts["tag"] += 1
        self.send({"type": "tag", "stamp": stamp, "frame_id": tf.header.frame_id,
                   "child_frame_id": tf.child_frame_id, "T": matrix(tf.transform),
                   "tag_id": self.args.tag_id, "tag_size_m": self.args.tag_size})

    def on_odom(self, msg):
        now = time.monotonic()
        if now - self.last_odom_sent < 1.0 / self.args.odom_rate:
            return
        self.last_odom_sent = now
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.counts["odom"] += 1
        self.send({"type": "odom", "stamp": stamp_s(msg.header.stamp),
                   "frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id,
                   "position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w]})

    def heartbeat(self):
        self.send({"type": "hello", "relay": "leo", "counts": dict(self.counts),
                   "clock": self.get_clock().now().nanoseconds * 1e-9})


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ros_args = []
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        argv, ros_args = argv[:i], argv[i:]
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--laptop-ip", required=True, help="the laptop's IP on Leo's network")
    ap.add_argument("--port", type=int, default=8791)
    ap.add_argument("--camera-frame", default="leo_oak_rgb_camera_optical_frame")
    ap.add_argument("--tag-frame", default="leo_tag0")
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--tag-size", type=float, default=0.160)
    ap.add_argument("--odom-topic", default="/leo/merged_odom")
    ap.add_argument("--tag-rate", type=float, default=15.0)
    ap.add_argument("--odom-rate", type=float, default=20.0)
    args = ap.parse_args(argv)
    rclpy.init(args=["leo_relay"] + ros_args)
    node = LeoRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
