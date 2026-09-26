"""AprilTag-to-map registration, independent of ROS and Spectacles SDK."""

from dataclasses import dataclass
from math import degrees, sqrt
from statistics import median

from .transforms import (average, compose, inverse, rotation_distance,
                         translation_distance)


@dataclass(frozen=True)
class Result:
    map_world: object
    translation_stddev_m: float
    angular_stddev_deg: float
    used: int


class Registrar:
    def __init__(self, minimum_detections=10, timeout_sec=10.0,
                 max_translation_residual_m=0.25, max_angular_residual_deg=15.0):
        self.minimum = int(minimum_detections)
        self.timeout = float(timeout_sec)
        self.max_translation = float(max_translation_residual_m)
        self.max_angle = float(max_angular_residual_deg)
        if self.minimum < 2 or self.timeout <= 0:
            raise ValueError("Invalid registration window")
        self.samples = []
        self.result = None

    def reset(self):
        self.samples.clear()
        self.result = None

    def add(self, stamp_sec, map_tag, world_device, device_tag):
        """T_M_W = T_M_Tag * inverse(T_D_Tag) * inverse(T_W_D)."""
        candidate = compose(compose(map_tag, inverse(device_tag)),
                            inverse(world_device))
        self.samples.append((float(stamp_sec), candidate))
        self.samples = [(t, pose) for t, pose in self.samples
                        if float(stamp_sec) - t <= self.timeout]
        if len(self.samples) < self.minimum:
            return None
        candidates = [pose for _, pose in self.samples]
        center = tuple(median(t.position[i] for t in candidates)
                       for i in range(3))
        # A medoid avoids letting one bad quaternion set the initial direction.
        medoid = min(candidates, key=lambda a: sum(
            rotation_distance(a.orientation, b.orientation) for b in candidates))
        kept = [t for t in candidates
                if translation_distance(t.position, center) <= self.max_translation
                and degrees(rotation_distance(t.orientation, medoid.orientation))
                <= self.max_angle]
        if len(kept) < self.minimum:
            return None
        estimate = average(kept)
        pos_std = sqrt(sum(translation_distance(t.position, estimate.position)**2
                           for t in kept) / len(kept))
        ang_std = degrees(sqrt(sum(rotation_distance(t.orientation,
                                                     estimate.orientation)**2
                                   for t in kept) / len(kept)))
        self.result = Result(estimate, pos_std, ang_std, len(kept))
        return self.result

    def map_device(self, world_device):
        if self.result is None:
            return None
        return compose(self.result.map_world, world_device)
