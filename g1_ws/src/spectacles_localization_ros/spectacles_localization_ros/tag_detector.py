"""Optional AprilTag 36h11 detection from a paired Spectacles JPEG frame.

Requires OpenCV with the aruco contrib module. Input camera frame is ROS optical:
X right, Y down, Z forward. T_device_camera must describe that optical frame.
"""

import base64
from math import sqrt

from .transforms import Transform, compose


def rotation_to_quaternion(matrix):
    """Convert a proper 3x3 rotation matrix to ROS xyzw."""
    trace = float(matrix[0][0] + matrix[1][1] + matrix[2][2])
    if trace > 0:
        scale = sqrt(trace + 1.0) * 2
        q = ((matrix[2][1] - matrix[1][2]) / scale,
             (matrix[0][2] - matrix[2][0]) / scale,
             (matrix[1][0] - matrix[0][1]) / scale,
             0.25 * scale)
    else:
        axis = max(range(3), key=lambda i: matrix[i][i])
        if axis == 0:
            scale = sqrt(1 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2
            q = (0.25 * scale, (matrix[0][1] + matrix[1][0]) / scale,
                 (matrix[0][2] + matrix[2][0]) / scale,
                 (matrix[2][1] - matrix[1][2]) / scale)
        elif axis == 1:
            scale = sqrt(1 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2
            q = ((matrix[0][1] + matrix[1][0]) / scale, 0.25 * scale,
                 (matrix[1][2] + matrix[2][1]) / scale,
                 (matrix[0][2] - matrix[2][0]) / scale)
        else:
            scale = sqrt(1 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2
            q = ((matrix[0][2] + matrix[2][0]) / scale,
                 (matrix[1][2] + matrix[2][1]) / scale,
                 0.25 * scale, (matrix[1][0] - matrix[0][1]) / scale)
    return q


def detect(payload, tag_id, tag_size_m):
    """Return T_device_tag or None. Reject malformed/unsupported camera packets."""
    import cv2
    import numpy as np

    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
        raise RuntimeError("OpenCV aruco with AprilTag 36h11 support is required")
    encoded = payload["camera_jpeg_base64"]
    if len(encoded) > 2_000_000:
        raise ValueError("JPEG is too large")
    binary = base64.b64decode(encoded, validate=True)
    image = cv2.imdecode(np.frombuffer(binary, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("Could not decode JPEG")
    intrinsics = payload["camera_intrinsics"]
    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
    if fx <= 0 or fy <= 0:
        raise ValueError("Invalid camera intrinsics")
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
    distortion = np.array(intrinsics.get("distortion", [0, 0, 0, 0, 0]), dtype=float)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    if ids is None:
        return None
    half = tag_size_m / 2.0
    # IPPE_SQUARE order: top-left, top-right, bottom-right, bottom-left.
    tag_corners = np.array([[-half, half, 0], [half, half, 0],
                            [half, -half, 0], [-half, -half, 0]], dtype=float)
    for points, seen_id in zip(corners, ids.flatten()):
        if int(seen_id) != tag_id:
            continue
        ok, rotation_vec, translation_vec = cv2.solvePnP(
            tag_corners, points.reshape(4, 2), camera_matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or float(translation_vec[2][0]) <= 0:
            continue
        rotation_matrix, _ = cv2.Rodrigues(rotation_vec)
        camera_tag = Transform(tuple(float(x) for x in translation_vec.flat),
                               rotation_to_quaternion(rotation_matrix))
        extrinsic = payload["camera_pose_device"]
        device_camera = Transform(tuple(extrinsic["position"]),
                                  tuple(extrinsic["orientation"]))
        return compose(device_camera, camera_tag)
    return None
