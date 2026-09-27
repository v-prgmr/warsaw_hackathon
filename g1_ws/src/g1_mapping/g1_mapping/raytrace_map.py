"""A LiDAR occupancy voxel map that clears dynamic obstacles by ray tracing.

RTAB-Map's /cloud_map ray-traces only *per node*: a cell is cleared when a NEW map node
re-observes it as free, and nodes are created only on motion (RGBD/LinearUpdate). So a person
who walks in front of the G1 while it stands still gets frozen into the current node and stays in
/cloud_map "even after they are long gone". This node solves that independently of the pose graph.

Each incoming scan is transformed into a drift-but-jump-free fixed frame (odom). Every voxel a ray
*ends in* gets a hit (log-odds up); every voxel a ray *passes through* on the way there gets a
miss (log-odds down). Static structure is re-hit every scan and clamps to occupied; a surface that
stops being seen (a person who left) is ray-traced through and decays below the publish threshold
within ~0.6 s, then is dropped from the map. Occluded structure (nothing passes through it) keeps
its last value, so walls behind a person are not erased.

Unlike 3D OctoMap ray tracing (tried in g1_mapping.yaml: floor rays under tabletops erased
tables), a surface here survives as long as it is re-hit, so continuously-visible furniture stays.

Publishes /g1_mapping/cloud_raytraced (PointCloud2 xyz + intensity=occupancy prob) in the fixed
frame. Point RViz / the AR bridge (g1_ar_bridge cloud_topic) at it instead of /cloud_map to get
the self-cleaning cloud. Independent of RTAB-Map: the pose graph, database and 2D grid are
untouched.

Topics (remap): input -> a fixed/deskewed LiDAR cloud (e.g. /g1_mapping/cloud), output -> map.
"""
import array

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from g1_mapping.lifetime import exit_with_parent

# Voxel coordinates are packed into one int64 dict key: (i + OFFSET) into 21 bits per axis.
# 21 bits -> +/-1e6 voxels -> +/-100 km at 0.1 m, far beyond any indoor/outdoor run.
_BITS = 21
_OFFSET = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1

_NP_TYPE = {
    PointField.INT8: "i1", PointField.UINT8: "u1", PointField.INT16: "<i2",
    PointField.UINT16: "<u2", PointField.INT32: "<i4", PointField.UINT32: "<u4",
    PointField.FLOAT32: "<f4", PointField.FLOAT64: "<f8",
}
_OUT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
_OUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def _pack(ijk):
    """(N, 3) int voxel coords -> (N,) int64 keys."""
    i = (ijk[:, 0].astype(np.int64) + _OFFSET) & _MASK
    j = (ijk[:, 1].astype(np.int64) + _OFFSET) & _MASK
    k = (ijk[:, 2].astype(np.int64) + _OFFSET) & _MASK
    return (i << (2 * _BITS)) | (j << _BITS) | k


def _unpack_centers(keys, voxel):
    """(N,) int64 keys -> (N, 3) float32 voxel-centre coordinates."""
    i = ((keys >> (2 * _BITS)) & _MASK) - _OFFSET
    j = ((keys >> _BITS) & _MASK) - _OFFSET
    k = (keys & _MASK) - _OFFSET
    ijk = np.stack([i, j, k], axis=1).astype(np.float64)
    return ((ijk + 0.5) * voxel).astype(np.float32)


def _quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class RaytraceMap(Node):
    def __init__(self):
        super().__init__("raytrace_map")
        self.fixed_frame = self.declare_parameter("fixed_frame", "odom").value
        self.voxel = float(self.declare_parameter("voxel_size", 0.1).value)
        self.max_range = float(self.declare_parameter("max_range", 5.0).value)
        # Log-odds: a hit outweighs a miss (0.85 vs 0.4) so intermittently-seen surfaces survive,
        # while a surface that stops being seen decays below l_publish in ~(l_max-l_publish)/miss
        # scans (~6 scans, 0.6 s at 10 Hz) and below 0 (dropped) a few scans later.
        self.l_hit = float(self.declare_parameter("log_odds_hit", 0.85).value)
        self.l_miss = float(self.declare_parameter("log_odds_miss", 0.4).value)
        self.l_max = float(self.declare_parameter("log_odds_max", 3.5).value)
        self.l_publish = float(self.declare_parameter("publish_threshold", 1.0).value)
        # Process every Nth scan (raise on a weak CPU); publish the map at publish_rate Hz.
        self.decimation = max(1, int(self.declare_parameter("decimation", 1).value))
        publish_rate = float(self.declare_parameter("publish_rate", 4.0).value)

        self.map = {}  # int64 voxel key -> float log-odds, kept only while > 0
        self._seq = 0
        self._last_stamp = None
        self._warned = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, "output", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))  # latch: late viewers get the last map
        self.create_subscription(PointCloud2, "input", self.on_cloud, qos_profile_sensor_data)
        self.create_timer(1.0 / publish_rate, self.publish_map)

    def on_cloud(self, msg):
        self._seq += 1
        if self._seq % self.decimation:
            return
        pts = self._read_xyz(msg)
        if pts is None or len(pts) == 0:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05))
        except Exception:
            try:  # fall back to the latest transform if the exact stamp is unavailable
                tf = self.tf_buffer.lookup_transform(
                    self.fixed_frame, msg.header.frame_id, Time())
            except Exception as exc:
                if not self._warned:
                    self.get_logger().warn(
                        f"No TF {self.fixed_frame} <- {msg.header.frame_id}: {exc}")
                    self._warned = True
                return
        self._last_stamp = msg.header.stamp

        R = _quat_to_rot(tf.transform.rotation)
        t = tf.transform.translation
        origin = np.array([t.x, t.y, t.z])
        pts = pts @ R.T + origin  # sensor frame -> fixed frame

        # Clearing works within max_range of the sensor; drop farther returns (can't clear them).
        rng = np.linalg.norm(pts - origin, axis=1)
        pts = pts[(rng > self.voxel) & (rng <= self.max_range)]
        if len(pts) == 0:
            return

        hit_keys = np.unique(_pack(np.floor(pts / self.voxel).astype(np.int64)))
        miss_keys = self._ray_misses(origin, _unpack_centers(hit_keys, self.voxel))
        self._update(hit_keys, miss_keys)

    def _ray_misses(self, origin, endpoints):
        """Voxels the rays origin->endpoints pass through, excluding the endpoint voxels."""
        d = endpoints - origin
        length = np.linalg.norm(d, axis=1)
        unit = d / length[:, None]
        step = self.voxel * 0.5  # half a voxel so no cell is skipped along the ray
        dists = np.arange(1, int(np.ceil(self.max_range / step)) + 1) * step  # (K,)
        # sample points (K, M, 3), keep only those before the endpoint's own voxel
        samples = origin[None, None, :] + unit[None, :, :] * dists[:, None, None]
        keep = dists[:, None] < (length[None, :] - self.voxel)  # (K, M)
        pts = samples[keep]
        if len(pts) == 0:
            return np.empty(0, dtype=np.int64)
        return np.unique(_pack(np.floor(pts / self.voxel).astype(np.int64)))

    def _update(self, hit_keys, miss_keys):
        # Aggregate this scan's deltas per voxel (a cell can be both hit and missed by other rays).
        keys = np.concatenate([miss_keys, hit_keys])
        deltas = np.concatenate([np.full(miss_keys.shape, -self.l_miss),
                                 np.full(hit_keys.shape, self.l_hit)])
        uniq, inv = np.unique(keys, return_inverse=True)
        agg = np.zeros(uniq.shape[0])
        np.add.at(agg, inv, deltas)
        m = self.map
        for key, delta in zip(uniq.tolist(), agg.tolist()):
            value = m.get(key, 0.0) + delta
            if value <= 0.0:
                m.pop(key, None)  # free / cleared: forget it, keeping the map bounded
            else:
                m[key] = value if value < self.l_max else self.l_max

    def publish_map(self):
        if not self.map or self.pub.get_subscription_count() == 0:
            return
        keys = np.fromiter(self.map.keys(), dtype=np.int64, count=len(self.map))
        vals = np.fromiter(self.map.values(), dtype=np.float64, count=len(self.map))
        keep = vals >= self.l_publish
        keys, vals = keys[keep], vals[keep]
        centers = _unpack_centers(keys, self.voxel)

        out = np.zeros(len(keys), dtype=_OUT_DTYPE)
        out["x"], out["y"], out["z"] = centers[:, 0], centers[:, 1], centers[:, 2]
        # occupancy probability from log-odds, for colouring in RViz
        out["intensity"] = (1.0 / (1.0 + np.exp(-vals))).astype(np.float32)

        cloud = PointCloud2()
        cloud.header.frame_id = self.fixed_frame
        cloud.header.stamp = self._last_stamp or self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = out.size
        cloud.fields = _OUT_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = _OUT_DTYPE.itemsize
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = array.array("B", out.tobytes())
        self.pub.publish(cloud)

    def _read_xyz(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return None
        if msg.is_bigendian or msg.row_step != msg.width * msg.point_step:
            if not self._warned:
                self.get_logger().error("Unsupported cloud layout (big-endian or padded rows).")
                self._warned = True
            return None
        fields = {f.name: f for f in msg.fields}
        if not all(k in fields for k in ("x", "y", "z")):
            return None
        dt = np.dtype({
            "names": [f.name for f in msg.fields],
            "formats": [_NP_TYPE[f.datatype] for f in msg.fields],
            "offsets": [f.offset for f in msg.fields],
            "itemsize": msg.point_step,
        })
        pts = np.frombuffer(msg.data, dtype=dt, count=n)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
        return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    exit_with_parent()
    rclpy.init()
    try:
        rclpy.spin(RaytraceMap())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
