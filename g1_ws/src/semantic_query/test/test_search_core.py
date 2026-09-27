"""semantic_query core (no ROS): command parsing and the 3D box from mask + depth."""
import math

import numpy as np
import pytest

from semantic_query.backproject import CameraIntrinsics
from semantic_query.box3d import gravity_box, region_points, transform_points
from semantic_query.commands import parse_command


@pytest.mark.parametrize("text, expected", [
    ("red cup", ("search", "red cup")),
    ("Search for a red cup", ("search", "red cup")),
    ("find the  Red Cup.", ("search", "red cup")),
    ("where is my bottle?", ("search", "bottle")),
    ("look for some scissors", ("search", "scissors")),
    ("stop", ("stop", None)),
    ("Cancel", ("stop", None)),
    ("clear", ("clear", None)),
    ("help", ("help", None)),
    ("What can you do?", ("help", None)),
    ("never mind", ("stop", None)),
    ("", (None, None)),
    ("find", ("search", "find")),
])
def test_parse_command(text, expected):
    assert parse_command(text) == expected


K = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
# optical frame (x right, y down, z forward) -> map (x forward, y left, z up), camera 1 m high
T_MAP_CAM = np.eye(4)
T_MAP_CAM[:3, :3] = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
T_MAP_CAM[:3, 3] = [0.0, 0.0, 1.0]


def render_cup(radius=0.04, height=0.10, at=(1.0, 0.1, 0.75), w=640, h=480):
    """Depth (mm) of a vertical cylinder seen by the camera, on a far background wall."""
    depth = np.full((h, w), 3000, dtype=np.uint16)          # wall 3 m away
    mask = np.zeros((h, w), dtype=bool)
    cam = T_MAP_CAM[:3, 3]
    R = T_MAP_CAM[:3, :3]
    for v in range(h):
        for u in range(w):
            ray_c = np.array([(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0])
            d = R @ ray_c                                   # ray in map
            # intersect with the cylinder x^2 + y^2 = r^2 around `at` (vertical axis)
            ox, oy = cam[0] - at[0], cam[1] - at[1]
            a = d[0] ** 2 + d[1] ** 2
            b = 2 * (ox * d[0] + oy * d[1])
            c = ox * ox + oy * oy - radius ** 2
            disc = b * b - 4 * a * c
            if a < 1e-12 or disc < 0:
                continue
            s = (-b - math.sqrt(disc)) / (2 * a)
            z = cam[2] + s * d[2]
            if s > 0 and at[2] - height / 2 <= z <= at[2] + height / 2:
                depth[v, u] = int(round(s * 1000))          # optical z == s (unit z in ray_c)
                mask[v, u] = True
    return depth, mask


def test_cup_box_has_the_cup_size_and_position():
    depth, mask = render_cup()
    ys, xs = np.nonzero(mask)
    box_xyxy = (xs.min(), ys.min(), xs.max(), ys.max())
    # SAM2 masks bleed a little: grow the mask by 3 px so background pixels get in too
    grown = mask.copy()
    for dy in range(-3, 4):
        for dx in range(-3, 4):
            grown |= np.roll(np.roll(mask, dy, 0), dx, 1)
    pts = region_points(depth, K, box_xyxy, grown)
    box = gravity_box(transform_points(T_MAP_CAM, pts), camera_xyz=T_MAP_CAM[:3, 3])
    cx, cy, cz = box["center"]
    sx, sy, sz = box["size"]
    assert (cx, cy) == pytest.approx((1.0, 0.1), abs=0.02)   # centre of the cup
    assert cz == pytest.approx(0.75, abs=0.01)
    assert sz == pytest.approx(0.10, abs=0.015)               # height
    assert sorted([sx, sy]) == pytest.approx([0.08, 0.08], abs=0.02)   # ~ diameter both ways


def test_no_points_without_valid_depth():
    depth = np.zeros((480, 640), dtype=np.uint16)
    assert region_points(depth, K, (100, 100, 200, 200)) is None
