"""Calibration solvers: camera intrinsics, camera-to-camera, and camera-to-LiDAR (board planes).

Camera to LiDAR follows the plane-correspondence method of Zhang & Pless (2004) and
Unnikrishnan & Hebert (2005): for every board pose i the camera gives the board plane
(n_ci, d_ci) from PnP, the LiDAR gives the points p_ij on the board. The extrinsic T_cam_lidar
= (R, t) makes every LiDAR board point lie on the camera's plane:

    n_ci · (R p_ij + t) = d_ci

Closed form first (rotation from the normals by SVD, translation by linear least squares), then
Gauss-Newton on all point-to-plane residuals with a Huber loss. At least 3 board poses with
non-parallel normals are required; 10-20 well-tilted poses are recommended.
"""
import cv2
import numpy as np

from .geometry import invert, make_transform, so3_exp, transform_difference
from .lidar_board import fit_plane


class CalibrationError(RuntimeError):
    pass


# --- intrinsics ---------------------------------------------------------------------------

def calibrate_intrinsics(object_points, image_points, image_size, fix_k3=True, rational=False):
    """OpenCV calibrateCamera. Returns dict(k, d, rms, per_view_rms)."""
    if len(object_points) < 8:
        raise CalibrationError(f"{len(object_points)} views: take at least 8 (15-25 recommended)")
    flags = 0
    if rational:
        flags |= cv2.CALIB_RATIONAL_MODEL
    elif fix_k3:
        flags |= cv2.CALIB_FIX_K3
    obj = [np.asarray(o, np.float32) for o in object_points]
    img = [np.asarray(i, np.float32).reshape(-1, 1, 2) for i in image_points]
    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(obj, img, tuple(image_size), None, None,
                                                  flags=flags)
    per_view = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, k, d)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj - i) ** 2, axis=2)))))
    return {"k": k, "d": d.ravel(), "rms": float(rms), "per_view_rms": per_view}


def reprojection_rms(object_points, image_points, k, d):
    """RMS reprojection error of fixed intrinsics (e.g. the factory ones), one PnP per view."""
    errs = []
    for o, i in zip(object_points, image_points):
        o = np.asarray(o, np.float64)
        i = np.asarray(i, np.float64).reshape(-1, 1, 2)
        ok, r, t = cv2.solvePnP(o, i, k, d, flags=cv2.SOLVEPNP_IPPE)
        r, t = cv2.solvePnPRefineLM(o, i, k, d, r, t)
        proj, _ = cv2.projectPoints(o, r, t, k, d)
        errs.append(np.sum((proj - i) ** 2, axis=2).ravel())
    return float(np.sqrt(np.mean(np.concatenate(errs))))


# --- camera to camera ---------------------------------------------------------------------

def calibrate_camera_pair(object_points, points_a, points_b, k_a, d_a, k_b, d_b, size_a,
                          poses_a=None, poses_b=None):
    """T_b_a (camera a -> camera b) from simultaneous board views, intrinsics fixed.

    Returns dict(T_b_a, rms_px, spread) where spread compares the per-view estimates
    T_b_board @ inv(T_a_board) to the joint solution (m, deg), if the poses are given.
    """
    if len(object_points) < 3:
        raise CalibrationError(f"{len(object_points)} views seen by both cameras: take >= 3 "
                               "(10+ recommended)")
    obj = [np.asarray(o, np.float32) for o in object_points]
    pa = [np.asarray(p, np.float32).reshape(-1, 1, 2) for p in points_a]
    pb = [np.asarray(p, np.float32).reshape(-1, 1, 2) for p in points_b]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-9)
    rms, *_, r, t, _, _ = cv2.stereoCalibrate(
        obj, pa, pb, np.asarray(k_a, np.float64), np.asarray(d_a, np.float64),
        np.asarray(k_b, np.float64), np.asarray(d_b, np.float64), tuple(size_a),
        flags=cv2.CALIB_FIX_INTRINSIC, criteria=criteria)
    t_b_a = make_transform(r, t.ravel())
    out = {"T_b_a": t_b_a, "rms_px": float(rms), "views": len(obj)}
    if poses_a is not None and poses_b is not None:
        diffs = [transform_difference(t_b_a, tb @ invert(ta)) for ta, tb in zip(poses_a, poses_b)]
        out["spread"] = {"translation_m_max": max(d[0] for d in diffs),
                         "rotation_deg_max": max(d[1] for d in diffs)}
    return out


# --- camera to LiDAR ----------------------------------------------------------------------

def normals_condition(normals):
    """Singular values of the stacked unit normals, largest first. The smallest must be clearly
    > 0: the board poses must tilt in different directions."""
    return np.linalg.svd(np.asarray(normals), compute_uv=False)


def camera_lidar_closed_form(observations):
    """Initial T_cam_lidar from the plane normals and board centroids."""
    n_c = np.array([o["n_c"] for o in observations])
    n_l, cent = [], []
    for o in observations:
        n, _, _ = fit_plane(o["points"])
        if n @ o["points"].mean(axis=0) < 0:
            n = -n
        n_l.append(n)
        cent.append(o["points"].mean(axis=0))
    n_l, cent = np.array(n_l), np.array(cent)
    h = n_l.T @ n_c  # sum n_l n_c^T
    u, _, vt = np.linalg.svd(h)
    s = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))])
    r = vt.T @ s @ u.T  # n_c ≈ R n_l
    b = np.array([o["d_c"] for o in observations]) - np.einsum("ij,ij->i", n_c, cent @ r.T)
    t = np.linalg.lstsq(n_c, b, rcond=None)[0]
    return make_transform(r, t)


def calibrate_camera_lidar(observations, t_init=None, huber=0.03, iterations=30,
                           min_condition=0.08):
    """T_cam_lidar from board observations.

    observations: list of dict(n_c (3,), d_c, points (M, 3) board points in the LiDAR frame).
    Returns dict(T_cam_lidar, rms_m, per_view_rms_m, condition).
    """
    if len(observations) < 3:
        raise CalibrationError(f"{len(observations)} board poses with LiDAR points: need >= 3 "
                               "(10-20 recommended)")
    cond = normals_condition([o["n_c"] for o in observations])
    if cond[-1] / cond[0] < min_condition:
        raise CalibrationError(
            f"board poses too similar (normal singular values {np.round(cond, 3).tolist()}): "
            "tilt the board left/right AND up/down between poses")
    tf = camera_lidar_closed_form(observations) if t_init is None else t_init.copy()
    r, t = tf[:3, :3], tf[:3, 3]
    for _ in range(iterations):
        jtj, jtr = np.zeros((6, 6)), np.zeros(6)
        for o in observations:
            p = o["points"] @ r.T  # R p
            res = (p + t) @ o["n_c"] - o["d_c"]
            j = np.hstack([np.cross(p, o["n_c"]), np.tile(o["n_c"], (len(p), 1))])
            a = np.abs(res)
            w = np.where(a <= huber, 1.0, huber / np.maximum(a, 1e-12)) / len(p)  # per-view
            jtj += (j * w[:, None]).T @ j
            jtr += (j * w[:, None]).T @ res
        dx = -np.linalg.solve(jtj + 1e-9 * np.eye(6), jtr)
        r = so3_exp(dx[:3]) @ r
        t = t + dx[3:]
        if np.linalg.norm(dx) < 1e-9:
            break
    tf = make_transform(r, t)
    per_view = [float(np.sqrt(np.mean(((o["points"] @ r.T + t) @ o["n_c"] - o["d_c"]) ** 2)))
                for o in observations]
    all_res = np.concatenate([(o["points"] @ r.T + t) @ o["n_c"] - o["d_c"]
                              for o in observations])
    return {"T_cam_lidar": tf, "rms_m": float(np.sqrt(np.mean(all_res ** 2))),
            "per_view_rms_m": per_view, "condition": cond.tolist()}
