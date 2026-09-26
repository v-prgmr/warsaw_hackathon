"""Synthetic G1 sensor streams for the integration tests (no robot, no bag).

Publishes what the robot publishes (AGENTS.md §7), in the robot's formats:
  /utlidar/cloud_livox_mid360  PointCloud2 in livox_frame (mounted upside down), x y z intensity
                               ring time, `time` in float32 NANOSECONDS, ~38 % (0,0,0) points
  /utlidar/imu_livox_mid360    Imu in livox_frame, acceleration in g, no orientation
  /dog_imu_raw                 Imu in dog_imu_link (= robot_center), orientation populated
  /dog_odom                    Odometry odom -> robot_center (ground truth here)
  /lowstate, /lf/lowstate      unitree_hg/LowState, all joints at 0, 200 Hz / 20 Hz (--lowstate)

The robot walks through a ray-cast room (walls, floor, ceiling, pillars). Mount geometry is the
legacy fallback extrinsic of g1_mapping (robot_center -> livox_frame), so g1_mapping must run with
static_tf:=true. Only for tests: run it in an isolated DDS domain.
"""
import argparse
import math

import numpy as np
import rclpy
import rclpy.time
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

G = 9.80665
FLOOR_BELOW_BASE = 0.72  # robot_center height above the floor


def rot_rpy(roll, pitch, yaw):
    """tf2 static_transform_publisher convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


# robot_center -> livox_frame (g1_mapping/config/g1_mapping.yaml static_transforms)
LIVOX_T = np.array([0.0, 0.0, 0.51])
LIVOX_R = rot_rpy(math.pi, 0.101, 0.0)

# Room: interior of a box, plus solid pillars / furniture (min xyz, max xyz), world frame with
# the floor at z = 0.
ROOM = (np.array([-4.0, -3.0, 0.0]), np.array([9.0, 4.0, 2.8]))
OBSTACLES = [
    (np.array([1.0, 1.5, 0.0]), np.array([1.6, 2.1, 2.8])),     # pillar
    (np.array([4.0, -2.2, 0.0]), np.array([5.2, -1.4, 0.75])),  # table
    (np.array([6.5, 1.0, 0.0]), np.array([7.0, 3.0, 1.8])),     # shelf
    (np.array([-2.5, -2.5, 0.0]), np.array([-1.8, -1.2, 1.2])),  # cabinet
    (np.array([2.5, -3.0, 0.0]), np.array([2.7, -2.0, 2.8])),   # wall stub
]


def trajectory(t):
    """Robot pose (x, y, yaw) and body rates at time t: straight walk, then a turn (r = 1.2 m)
    that stays > 1 m from every wall."""
    v, w = 0.3, 0.0
    if t < 12.0:
        x, y, yaw = 0.3 * t, 0.0, 0.0
    else:
        dt = t - 12.0
        w = 0.25
        yaw = w * dt
        x = 3.6 + (v / w) * math.sin(yaw)
        y = (v / w) * (1.0 - math.cos(yaw))
    return x, y, yaw, v, w


def ray_hits(origins, dirs):
    """Distance along each ray to the first surface (room interior or obstacle)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / dirs
        # Inside the room: exit distance of the room box.
        t1 = (ROOM[0] - origins) * inv
        t2 = (ROOM[1] - origins) * inv
        best = np.nanmin(np.where(np.maximum(t1, t2) > 0, np.maximum(t1, t2), np.inf), axis=1)
        for lo, hi in OBSTACLES:
            a = (lo - origins) * inv
            b = (hi - origins) * inv
            tmin = np.nanmax(np.minimum(a, b), axis=1)
            tmax = np.nanmin(np.maximum(a, b), axis=1)
            hit = (tmax >= tmin) & (tmin > 0)
            best = np.where(hit & (tmin < best), tmin, best)
    return best


class SimG1(Node):
    def __init__(self, duration, lowstate, points, clock_offset=0.0, seed=0):
        super().__init__("sim_g1")
        self.duration, self.points = duration, points
        self.offset_ns = int(clock_offset * 1e9)
        self.rng = np.random.default_rng(seed)
        self.t0 = self.get_clock().now().nanoseconds
        self.pub_cloud = self.create_publisher(PointCloud2, "/utlidar/cloud_livox_mid360",
                                               qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, "/dog_imu_raw", qos_profile_sensor_data)
        self.pub_limu = self.create_publisher(Imu, "/utlidar/imu_livox_mid360",
                                              qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, "/dog_odom", 50)
        self.pub_low = None
        if lowstate:
            from unitree_hg.msg import LowState
            self.LowState = LowState
            self.pub_low = self.create_publisher(LowState, "/lowstate", qos_profile_sensor_data)
            self.pub_low_lf = self.create_publisher(LowState, "/lf/lowstate",
                                                    qos_profile_sensor_data)
            self.low_count = 0
        # Ray casting takes tens of ms: keep it off the IMU / LowState timer so those stream
        # smoothly like on the robot (numpy releases the GIL).
        self.create_timer(0.1, self.on_scan, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.005, self.on_imu, callback_group=MutuallyExclusiveCallbackGroup())
        self.done = False

    def elapsed(self):
        return (self.get_clock().now().nanoseconds - self.t0) * 1e-9

    def stamp(self):
        """Header stamps on the ROBOT clock (host clock + clock_offset, e.g. -73 s)."""
        return rclpy.time.Time(
            nanoseconds=self.get_clock().now().nanoseconds + self.offset_ns).to_msg()

    def on_imu(self):
        t = self.elapsed()
        if t > self.duration:
            self.done = True
            return
        x, y, yaw, v, w = trajectory(t)
        stamp = self.stamp()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "dog_imu_link"
        # A tiny roll: RTAB-Map ignores IMU orientations with x = y = z = 0 exactly.
        imu.orientation.x = 1e-4
        imu.orientation.z, imu.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        imu.angular_velocity.z = w
        imu.linear_acceleration.z = G
        self.pub_imu.publish(imu)

        limu = Imu()
        limu.header.stamp = stamp
        limu.header.frame_id = "livox_frame"
        acc = LIVOX_R.T @ np.array([0.0, 0.0, 1.0])  # specific force in g
        gyr = LIVOX_R.T @ np.array([0.0, 0.0, w])
        limu.linear_acceleration.x, limu.linear_acceleration.y, limu.linear_acceleration.z = acc
        limu.angular_velocity.x, limu.angular_velocity.y, limu.angular_velocity.z = gyr
        self.pub_limu.publish(limu)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "robot_center"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.position.z = FLOOR_BELOW_BASE
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pub_odom.publish(odom)

        if self.pub_low is not None:
            self.pub_low.publish(self.LowState())
            self.low_count += 1
            if self.low_count % 10 == 0:  # the robot's 20 Hz copy
                self.pub_low_lf.publish(self.LowState())

    def on_scan(self):
        # Like the real sensor: published when the 100 ms sweep ENDS, stamped at its start.
        t_start = self.elapsed() - 0.1
        if t_start < 0.0 or t_start > self.duration:
            return
        stamp = rclpy.time.Time(nanoseconds=Time.from_msg(self.stamp()).nanoseconds
                                - 100_000_000).to_msg()
        n = self.points
        # MID-360 in its own (upright) frame: 360 deg azimuth, -7..52 deg elevation.
        az = self.rng.uniform(-math.pi, math.pi, n)
        el = np.radians(self.rng.uniform(-7.0, 52.0, n))
        d_sensor = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], 1)
        dt = np.sort(self.rng.uniform(0.0, 0.1, n))  # s since header stamp
        origins = np.empty((n, 3))
        dirs = np.empty((n, 3))
        for k, chunk in enumerate(np.array_split(np.arange(n), 10)):
            x, y, yaw, _, _ = trajectory(t_start + dt[chunk].mean())
            r_wb = rot_rpy(0.0, 0.0, yaw)
            r_ws = r_wb @ LIVOX_R
            origins[chunk] = np.array([x, y, FLOOR_BELOW_BASE]) + r_wb @ LIVOX_T
            dirs[chunk] = d_sensor[chunk] @ r_ws.T
        rng_m = ray_hits(origins, dirs)
        rng_m = rng_m + self.rng.normal(0.0, 0.01, n)
        pts = d_sensor * rng_m[:, None]
        pts[~np.isfinite(rng_m) | (rng_m > 40.0)] = 0.0
        pts[self.rng.random(n) < 0.38] = 0.0  # the G1 stream's (0,0,0) placeholders

        # Unitree layout: x y z intensity (f32), ring (u16), time (f32, ns), 22-byte packed.
        dtype = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                          "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                          "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 22})
        arr = np.zeros(n, dtype=dtype)
        arr["x"], arr["y"], arr["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        arr["intensity"] = 50.0
        arr["ring"] = (np.arange(n) % 4).astype(np.uint16)
        arr["time"] = (dt * 1e9).astype(np.float32)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "livox_frame"
        msg.height, msg.width = 1, n
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step, msg.row_step = 22, 22 * n
        msg.is_dense = False
        msg.data = arr.tobytes()
        self.pub_cloud.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--points", type=int, default=20000)
    ap.add_argument("--lowstate", action="store_true")
    ap.add_argument("--clock-offset", type=float, default=0.0,
                    help="robot clock minus host clock in s (the laptop was ~73 s ahead)")
    args = ap.parse_args()
    rclpy.init()
    node = SimG1(args.duration, args.lowstate, args.points, args.clock_offset)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
