"""Synthetic AprilTag image test; skipped when OpenCV contrib is unavailable."""

import base64
import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from spectacles_localization_ros.tag_detector import detect


@unittest.skipUnless(cv2 is not None and hasattr(cv2, "aruco"),
                     "OpenCV contrib not installed")
class TagDetectorTests(unittest.TestCase):
    def test_front_facing_tag_pose(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 0, 120)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 0, 120)
        image = np.full((480, 640), 255, dtype=np.uint8)
        image[180:300, 260:380] = marker
        ok, jpeg = cv2.imencode(".jpg", image)
        self.assertTrue(ok)
        payload = {
            "camera_jpeg_base64": base64.b64encode(jpeg.tobytes()).decode("ascii"),
            "camera_intrinsics": {"fx": 750, "fy": 750, "cx": 320, "cy": 240},
            "camera_pose_device": {
                "position": [0, 0, 0], "orientation": [0, 0, 0, 1],
            },
        }
        tag = detect(payload, 0, 0.16)
        self.assertIsNotNone(tag)
        self.assertLess(abs(tag.position[0]), 0.03)
        self.assertLess(abs(tag.position[1]), 0.03)
        self.assertGreater(tag.position[2], 0.8)
        self.assertLess(tag.position[2], 1.2)
        self.assertIsNone(detect(payload, 1, 0.16))
