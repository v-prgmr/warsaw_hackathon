"""Where the bridge gets the robot's world from: ROS (``ros_world.RosWorld``) or a simulation.

Everything a ``World`` returns is in the ROS ``map`` frame (x forward, y left, z up, metres);
the bridge converts to the AR world after registration.
"""
import math
import time

import numpy as np

from .annotations import poi
from .geometry import make_T, rot_z


class World:
    """Interface used by ``server.ArBridgeServer`` (all methods may be called from the
    bridge's asyncio thread; implementations must be thread-safe)."""

    def robot_pose(self):
        """4x4 ``T_map_robot`` (latest), or None."""
        return None

    def anchor(self, tag_id):
        """4x4 ``T_map_tag`` of the wall tag measured by the robot camera, or None."""
        return None

    def floor_z(self):
        """Map z of the floor under the robot, or None."""
        return None

    def lidar_points(self):
        """(N, 3) map points (already downsampled)."""
        return np.zeros((0, 3))

    def path(self):
        """Current navigation path as a list of map points."""
        return []

    def annotations(self):
        """dict id -> ``annotations.Annotation`` in map."""
        return {}

    def on_registered(self, T_ar_map, method, info):
        pass

    def on_unregistered(self):
        pass

    def on_hmd_pose(self, T_map_hmd, T_ar_hmd):
        """The Spectacles' pose (Lens camera: x right, y up, looks along -Z) in map and AR."""

    def on_user_command(self, text):
        """Voice / typed command from the glasses; returns the reply text or None."""
        return None


def wall_tag_pose(distance, height_above_floor, floor_z, lateral=0.0):
    """Tag on a wall ``distance`` m in front of the map origin, facing back towards it
    (ArUco axes: x right as seen by the viewer = -y map, y up = +z map, z = -x map)."""
    R = np.column_stack([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])
    return make_T(R, [distance, lateral, floor_z + height_above_floor])


class SimWorld(World):
    """No robot: the G1 stands at the map origin facing a wall with the AprilTag.

    Used by ``sim_main`` for testing the Spectacles side at home: stick the printed tag on a
    wall, register with "AprilTag", and the virtual G1 appears ``tag_distance`` m in front of
    the tag, facing it, inside a synthetic room whose front wall is your wall.
    """

    def __init__(self, tag_id=0, tag_distance=1.5, tag_height=1.0, tag_lateral=0.0,
                 base_height=0.78, demo_pois=True, log=print):
        self.tag_id = tag_id
        self.base_height = base_height
        self.T_map_tag = wall_tag_pose(tag_distance, tag_height, -base_height, tag_lateral)
        self.tag_distance = tag_distance
        self.demo_pois = demo_pois
        self.log = log
        self.registered = None
        self._last_hmd_log = 0.0
        self._points = self._room(tag_distance)

    def robot_pose(self):
        return make_T(rot_z(0.0), [0.0, 0.0, 0.0])

    def anchor(self, tag_id):
        return self.T_map_tag if tag_id == self.tag_id else None

    def floor_z(self):
        return -self.base_height

    def lidar_points(self):
        return self._points

    def _room(self, front):
        """6 x 5 m room: the tag wall at x = ``front``, a table at 0.75 m on the right."""
        rng = np.random.default_rng(42)
        floor = -self.base_height
        back = front - 6.0
        pts = []
        for _ in range(2500):
            a = rng.uniform(back, front)
            b = rng.uniform(-2.5, 2.5)
            h = rng.uniform(0.0, 2.2)
            wall = rng.random()
            if wall < 0.35:
                a = front
            elif wall < 0.55:
                a = back
            elif wall < 0.775:
                b = -2.5
            else:
                b = 2.5
            pts.append((a, b, floor + h))
        for _ in range(400):     # floor near the robot
            pts.append((rng.uniform(back, front), rng.uniform(-2.5, 2.5), floor))
        for _ in range(300):     # table top
            pts.append((rng.uniform(0.6, 1.2), rng.uniform(-1.3, -0.7), floor + 0.75))
        return np.asarray(pts)

    def annotations(self):
        if not self.demo_pois:
            return {}
        floor = -self.base_height
        anns = poi("poi-bottle", "red bottle", [0.9, -0.85, floor + 0.87], 0.87)
        anns += poi("poi-box", "box on the table", [1.0, -1.1, floor + 0.95], 0.74,
                    box=([1.0, -1.1, floor + 0.87], [0.3, 0.4, 0.24], rot_z(0.3)))
        return {a.id: a for a in anns}

    def on_registered(self, T_ar_map, method, info):
        self.registered = T_ar_map
        self.log(f"[sim] registered ({method}): {info}")

    def on_unregistered(self):
        self.registered = None

    def on_hmd_pose(self, T_map_hmd, T_ar_hmd):
        now = time.monotonic()
        if now - self._last_hmd_log < 5.0:
            return
        self._last_hmd_log = now
        p = T_map_hmd[:3, 3]
        self.log(f"[sim] Spectacles in map: x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f} m "
                 f"({math.hypot(p[0], p[1]):.2f} m from the robot)")

    def on_user_command(self, text):
        return f"(simulation) I heard: {text}"
