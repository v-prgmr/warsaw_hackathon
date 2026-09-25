"""Normalize the G1 MID-360 PointCloud2 for RTAB-Map.

The Unitree stream (/utlidar/cloud_livox_mid360) stores the per-point `time` field as float32
NANOSECONDS relative to header.stamp, but rtabmap_conversions reads a float32 `time` field as
SECONDS. Deskewing on the raw cloud would therefore treat a 100 ms scan as ~3 years long.
About 38 % of the points are also (0,0,0) placeholders.

This node republishes the cloud with:
  * `time` converted to float32 seconds (time_scale, default 1e-9)
  * zero / non-finite / out-of-range points removed
  * fields x y z intensity time ring (24-byte points), same header

Topics (remap): input -> raw cloud, output -> fixed cloud.
"""
import array

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

_NP_TYPE = {
    PointField.INT8: "i1", PointField.UINT8: "u1", PointField.INT16: "<i2",
    PointField.UINT16: "<u2", PointField.INT32: "<i4", PointField.UINT32: "<u4",
    PointField.FLOAT32: "<f4", PointField.FLOAT64: "<f8",
}
_OUT_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
    ("time", "<f4"), ("ring", "<u2"), ("_pad", "<u2"),
])
_OUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
]


class LivoxCloudFix(Node):
    def __init__(self):
        super().__init__("livox_cloud_fix")
        self.time_field = self.declare_parameter("time_field", "time").value
        self.time_scale = float(self.declare_parameter("time_scale", 1.0e-9).value)
        self.range_min = float(self.declare_parameter("range_min", 0.3).value)
        self.range_max = float(self.declare_parameter("range_max", 30.0).value)
        self.pub = self.create_publisher(
            PointCloud2, "output", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(PointCloud2, "input", self.cb, qos_profile_sensor_data)
        self._warned = False

    def cb(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return
        if msg.is_bigendian or msg.row_step != msg.width * msg.point_step:
            if not self._warned:
                self.get_logger().error("Unsupported cloud layout (big-endian or padded rows).")
                self._warned = True
            return
        fields = {f.name: f for f in msg.fields}
        if self.time_field not in fields:
            if not self._warned:
                self.get_logger().error(f"Input cloud has no '{self.time_field}' field.")
                self._warned = True
            return
        dt = np.dtype({
            "names": [f.name for f in msg.fields],
            "formats": [_NP_TYPE[f.datatype] for f in msg.fields],
            "offsets": [f.offset for f in msg.fields],
            "itemsize": msg.point_step,
        })
        pts = np.frombuffer(msg.data, dtype=dt, count=n)
        x, y, z = pts["x"], pts["y"], pts["z"]
        r = np.sqrt(x * x + y * y + z * z)
        keep = np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)

        out = np.zeros(int(keep.sum()), dtype=_OUT_DTYPE)
        out["x"], out["y"], out["z"] = x[keep], y[keep], z[keep]
        if "intensity" in fields:
            out["intensity"] = pts["intensity"][keep]
        out["time"] = pts[self.time_field][keep].astype(np.float64) * self.time_scale
        if "ring" in fields:
            out["ring"] = pts["ring"][keep]

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = out.size
        cloud.fields = _OUT_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = _OUT_DTYPE.itemsize
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = array.array("B", out.tobytes())
        self.pub.publish(cloud)


def main():
    rclpy.init()
    try:
        rclpy.spin(LivoxCloudFix())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
