"""Small, dependency-free SE(3) operations. Quaternions are ROS xyzw."""

from dataclasses import dataclass
from math import acos, isfinite, sqrt


@dataclass(frozen=True)
class Transform:
    """T_parent_child: child coordinates mapped into parent coordinates."""

    position: tuple
    orientation: tuple

    def __post_init__(self):
        if len(self.position) != 3 or len(self.orientation) != 4:
            raise ValueError("Expected xyz and xyzw")
        if not all(isfinite(float(v)) for v in list(self.position) + list(self.orientation)):
            raise ValueError("Non-finite transform")
        object.__setattr__(self, "position", tuple(float(v) for v in self.position))
        object.__setattr__(self, "orientation", normalize(self.orientation))


def normalize(q):
    norm = sqrt(sum(float(x) * float(x) for x in q))
    if norm < 1e-12:
        raise ValueError("Zero quaternion")
    return tuple(float(x) / norm for x in q)


def multiply(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return normalize((
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ))


def rotate(q, v):
    # Rodrigues' quaternion formula; q is unit length.
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y*vz - z*vy)
    ty = 2 * (z*vx - x*vz)
    tz = 2 * (x*vy - y*vx)
    return (vx + w*tx + (y*tz - z*ty),
            vy + w*ty + (z*tx - x*tz),
            vz + w*tz + (x*ty - y*tx))


def compose(a, b):
    moved = rotate(a.orientation, b.position)
    return Transform(tuple(a.position[i] + moved[i] for i in range(3)),
                     multiply(a.orientation, b.orientation))


def inverse(t):
    x, y, z, w = t.orientation
    q = (-x, -y, -z, w)
    return Transform(rotate(q, tuple(-v for v in t.position)), q)


def rotation_distance(a, b):
    dot = abs(sum(x*y for x, y in zip(a, b)))
    return 2 * acos(min(1.0, max(-1.0, dot)))


def translation_distance(a, b):
    return sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def average_quaternions(quaternions):
    """Markley quaternion mean (dominant eigenvector of sum(q q^T))."""
    if not quaternions:
        raise ValueError("No quaternions")
    matrix = [[0.0] * 4 for _ in range(4)]
    for raw in quaternions:
        q = normalize(raw)
        for i in range(4):
            for j in range(4):
                matrix[i][j] += q[i] * q[j]
    result = list(normalize(quaternions[0]))
    for _ in range(100):
        next_q = [sum(matrix[i][j] * result[j] for j in range(4))
                  for i in range(4)]
        next_q = list(normalize(next_q))
        if abs(sum(a*b for a, b in zip(result, next_q))) > 1 - 1e-12:
            result = next_q
            break
        result = next_q
    return tuple(result)


def average(transforms):
    if not transforms:
        raise ValueError("No transforms")
    count = len(transforms)
    return Transform(tuple(sum(t.position[i] for t in transforms) / count
                           for i in range(3)),
                     average_quaternions([t.orientation for t in transforms]))
