"""Relay the OAK-D topics from an isolated DDS domain into the robot domain (laptop side).

The OAK-D driver on the Orin (ROS 2 Foxy, CycloneDDS) segfaults on the robot's DDS domain 0,
where Unitree's bare-DDS services publish (g1_sensors README; AGENTS.md §5). So the driver runs
in its own domain (default 78) and this node, on the laptop, copies its topics into domain 0
for tf_chain / tag_anchor / semantic_query. Messages are copied serialized (no decode). Images
are decimated by their header stamp (the same stamp keeps RGB and aligned depth together).

    ros2 run g1_sensors oak_domain_relay                  # 78 -> 0, images at <= 10 Hz
    ros2 run g1_sensors oak_domain_relay --max-rate 15

Orin side (then): ROS_DOMAIN_ID=78 ros2 launch depthai_ros_driver camera.launch.py
Read-only towards the robot: it only republishes camera data.
"""
import argparse
import struct
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

IMAGES = ("/oak/rgb/image_raw", "/oak/stereo/image_raw")
INFOS = ("/oak/rgb/camera_info", "/oak/stereo/camera_info")
SENSOR = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def cdr_stamp(data):
    """header.stamp (seconds) of a serialized message that starts with std_msgs/Header."""
    if len(data) < 12:
        return None
    little = data[1] == 1                    # CDR encapsulation: 0x0001 = little endian
    sec, nsec = struct.unpack("<iI" if little else ">iI", data[4:12])
    return sec + nsec * 1e-9


class Relay:
    def __init__(self, args):
        self.ctx_in, self.ctx_out = Context(), Context()
        rclpy.init(context=self.ctx_in, domain_id=args.from_domain)
        rclpy.init(context=self.ctx_out, domain_id=args.to_domain)
        self.src = rclpy.create_node("oak_relay_in", context=self.ctx_in)
        self.dst = rclpy.create_node("oak_relay_out", context=self.ctx_out)
        self.max_rate = args.max_rate
        self.last_bucket = {}
        self.counts = {}
        for topic in IMAGES:
            self.add(topic, Image, SENSOR, SENSOR, decimate=True)
        for topic in INFOS:
            self.add(topic, CameraInfo, SENSOR, SENSOR, decimate=True)
        self.add("/tf_static", TFMessage, LATCHED, LATCHED, decimate=False)
        self.src.create_timer(10.0, self.report)
        self.src.get_logger().info(
            f"relaying OAK-D topics domain {args.from_domain} -> {args.to_domain} "
            f"(images <= {self.max_rate} Hz)")

    def add(self, topic, mtype, qos_in, qos_out, decimate):
        pub = self.dst.create_publisher(mtype, topic, qos_out)
        self.counts[topic] = 0

        def forward(data):
            if decimate and self.max_rate > 0:
                stamp = cdr_stamp(data)
                if stamp is not None:
                    bucket = int(stamp * self.max_rate)
                    if self.last_bucket.get(topic) == bucket:
                        return
                    self.last_bucket[topic] = bucket
            pub.publish(data)
            self.counts[topic] += 1

        self.src.create_subscription(mtype, topic, forward, qos_in, raw=True)

    def report(self):
        self.src.get_logger().info("relayed (10 s): " + ", ".join(
            f"{t.rsplit('/', 2)[-2] if t != '/tf_static' else 'tf_static'}"
            f"{'/info' if t.endswith('camera_info') else ''} {n}" for t, n in self.counts.items()))
        for t in self.counts:
            self.counts[t] = 0

    def spin(self):
        ex_out = SingleThreadedExecutor(context=self.ctx_out)
        ex_out.add_node(self.dst)
        threading.Thread(target=ex_out.spin, daemon=True).start()
        ex_in = SingleThreadedExecutor(context=self.ctx_in)
        ex_in.add_node(self.src)
        ex_in.spin()

    def shutdown(self):
        for node in (self.src, self.dst):
            node.destroy_node()
        for ctx in (self.ctx_in, self.ctx_out):
            rclpy.try_shutdown(context=ctx)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from-domain", type=int, default=78)
    ap.add_argument("--to-domain", type=int, default=0)
    ap.add_argument("--max-rate", type=float, default=10.0, help="images per second, 0 = all")
    args, _ = ap.parse_known_args(argv)
    relay = Relay(args)
    try:
        relay.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        relay.shutdown()


if __name__ == "__main__":
    main()
