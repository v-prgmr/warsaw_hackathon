"""Chessboard detection: canonical order under any in-plane rotation, accurate PnP."""
import math

import numpy as np
import pytest

from g1_calibration.board import Board, board_plane, board_pose, detect_corners
from g1_calibration.geometry import (make_transform, rpy_to_matrix, transform_difference,
                                     transform_points)

from synthetic import BOARD, OAK, render


def test_rejects_symmetric_board():
    with pytest.raises(ValueError, match="symmetric"):
        Board(cols=8, rows=6, square=0.05)
    with pytest.raises(ValueError):
        Board(cols=2, rows=5, square=0.05)
    Board(cols=9, rows=6, square=0.05)


def test_object_points_and_extent():
    obj = BOARD.object_points()
    assert obj.shape == (40, 3)
    np.testing.assert_allclose(obj[1], [0.08, 0, 0])      # row by row, x first
    np.testing.assert_allclose(obj[8], [0, 0.08, 0])
    x0, x1, y0, y1 = BOARD.extent()
    assert (x0, x1) == pytest.approx((-0.11, 0.67)) and (y0, y1) == pytest.approx((-0.11, 0.43))


def test_render_has_dark_origin_square():
    img, h = BOARD.render(1000.0)
    px = np.linalg.inv(h) @ [0.5 * BOARD.square, 0.5 * BOARD.square, 1.0]  # centre of (0, 0)
    assert img[int(px[1]), int(px[0])] == 0
    px = np.linalg.inv(h) @ [1.5 * BOARD.square, 0.5 * BOARD.square, 1.0]
    assert img[int(px[1]), int(px[0])] == 255


@pytest.mark.parametrize("spin_deg", [0, 37, 90, 180, 210, 270])
@pytest.mark.parametrize("tilt", [0.0, 0.5])
def test_canonical_order_and_pose(spin_deg, tilt):
    rng = np.random.default_rng(spin_deg)
    x0, x1, y0, y1 = BOARD.extent()
    center = np.array([(x0 + x1) / 2, (y0 + y1) / 2, 0.0])
    r = rpy_to_matrix(tilt, 0.2, 0.0) @ rpy_to_matrix(0.0, 0.0, math.radians(spin_deg))
    t_cam_board = make_transform(r, np.array([0.05, 0.02, 1.3]) - r @ center)
    corners = detect_corners(render(OAK, t_cam_board, rng), BOARD)
    assert corners is not None
    # canonical: the same order as the true board frame, whatever the in-plane rotation
    truth = transform_points(t_cam_board, BOARD.object_points())
    truth_px = truth[:, :2] / truth[:, 2:3] * 560.0 + [400.0, 300.0]
    assert np.abs(corners - truth_px).max() < 1.0
    tf, rms = board_pose(corners, BOARD, OAK.k, OAK.d)
    dt, dr = transform_difference(tf, t_cam_board)
    # one planar view: rotation to a few tenths of a degree (the calibration uses many views)
    assert rms < 0.5 and dt < 0.006 and dr < 0.6, (rms, dt, dr)
    n, d = board_plane(tf)
    assert d > 0 and n[2] > 0  # normal points away from the camera


def test_no_board():
    assert detect_corners(np.full((480, 640, 3), 128, np.uint8), BOARD) is None
