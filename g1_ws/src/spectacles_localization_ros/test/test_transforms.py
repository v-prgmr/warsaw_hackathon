"""Offline tests: fail on wrong transform direction/composition."""

import math
import unittest

from spectacles_localization_ros.protocol import parse, pose_dict
from spectacles_localization_ros.registration import Registrar
from spectacles_localization_ros.transforms import (
    Transform, average_quaternions, compose, inverse, rotation_distance,
    translation_distance,
)


def yaw(degrees):
    radians = math.radians(degrees)
    return (0, 0, math.sin(radians / 2), math.cos(radians / 2))


def assert_pose_close(test, actual, expected, metres=1e-8, degrees=1e-6):
    test.assertLess(translation_distance(actual.position, expected.position), metres)
    test.assertLess(math.degrees(rotation_distance(actual.orientation,
                                                   expected.orientation)), degrees)


class TransformTests(unittest.TestCase):
    def test_inverse_and_noncommutative_composition(self):
        a = Transform((1, 2, 3), yaw(67))
        b = Transform((-2, 0.4, 1), yaw(-21))
        identity = Transform((0, 0, 0), (0, 0, 0, 1))
        assert_pose_close(self, compose(a, inverse(a)), identity)
        assert_pose_close(self, compose(inverse(a), a), identity)
        self.assertGreater(translation_distance(compose(a, b).position,
                                                compose(b, a).position), 0.1)

    def test_registration_exact_with_rotated_and_offset_frames(self):
        map_world = Transform((1.1, -0.7, 0.15), yaw(38))
        map_tag = Transform((2.3, 0.9, 1.2), yaw(-19))
        world_device = Transform((0.2, 0.4, 1.6), yaw(12))
        device_tag = compose(inverse(world_device),
                             compose(inverse(map_world), map_tag))
        registrar = Registrar(minimum_detections=10)
        for i in range(12):
            result = registrar.add(i * 0.05, map_tag, world_device, device_tag)
        self.assertIsNotNone(result)
        assert_pose_close(self, result.map_world, map_world)
        assert_pose_close(self, registrar.map_device(world_device),
                          compose(map_world, world_device))
        # This deliberately wrong order must be visibly different.
        wrong = compose(compose(map_tag, inverse(world_device)),
                        inverse(device_tag))
        self.assertGreater(translation_distance(wrong.position,
                                                map_world.position), 0.1)

    def test_outlier_rejection(self):
        map_world = Transform((0.6, -0.3, 0.2), yaw(20))
        map_tag = Transform((2, 1, 1.2), yaw(-10))
        registrar = Registrar(minimum_detections=10)
        for i in range(9):
            local = Transform((0.1 * i, 0, 1.6), yaw(i))
            detected = compose(inverse(local), compose(inverse(map_world), map_tag))
            self.assertIsNone(registrar.add(i * 0.05, map_tag, local, detected))
        local = Transform((1, 0, 1.6), yaw(10))
        bad = Transform((20, 0, 0), yaw(170))
        self.assertIsNone(registrar.add(0.46, map_tag, local, bad))
        detected = compose(inverse(local), compose(inverse(map_world), map_tag))
        result = registrar.add(0.5, map_tag, local, detected)
        self.assertIsNotNone(result)
        self.assertEqual(result.used, 10)
        assert_pose_close(self, result.map_world, map_world)

    def test_quaternion_mean_handles_sign(self):
        q = yaw(80)
        mean = average_quaternions([q, tuple(-v for v in q), q])
        self.assertLess(rotation_distance(mean, q), 1e-8)

    def test_contract_rejects_unconverted_snap_coordinates(self):
        message = {
            "schema_version": 1, "coordinate_system": "snap_cm",
            "session_id": "test", "timestamp_sec": 1.0,
            "tracking_valid": True,
            "device_pose": pose_dict(Transform((0, 0, 0), (0, 0, 0, 1))),
            "tag_id": None, "tag_pose_device": None,
        }
        with self.assertRaises(ValueError):
            parse(message)
        message["coordinate_system"] = "ros_rh_m"
        self.assertEqual(parse(message).session_id, "test")


if __name__ == "__main__":
    unittest.main()
