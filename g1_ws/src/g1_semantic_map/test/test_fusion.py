import unittest

import numpy as np

from g1_semantic_map.fusion import VoxelFusion, project_visible, transform_matrix


class ProjectionTest(unittest.TestCase):
    def test_tf_and_optical_axes(self):
        t = transform_matrix((0, 0, 0), (0, 0, 0, 1))
        points = np.array([[0, 0, 2], [1, 0, 2], [0, 1, 2], [0, 0, -1]],
                          dtype=np.float32)
        depth = np.full((9, 9), 2, np.float32)
        indices, u, v = project_visible(points, t, (4, 4, 4, 4), depth)
        np.testing.assert_array_equal(indices, [0, 1, 2])
        np.testing.assert_array_equal(u, [4, 6, 4])
        np.testing.assert_array_equal(v, [4, 4, 6])

    def test_occlusion_and_missing_depth_rejected(self):
        points = np.array([[0, 0, 2], [0, 0, 3], [1, 0, 2]], np.float32)
        depth = np.zeros((9, 9), np.float32)
        depth[4, 4] = 2.0
        indices, _, _ = project_visible(points, np.eye(4), (4, 4, 4, 4), depth)
        np.testing.assert_array_equal(indices, [0])

    def test_translation_and_rotation(self):
        # camera at map x=1, looking along map +x; map point (3,0,0)
        # must lie at camera optical z=2.
        s = np.sqrt(0.5)
        t_map_camera = transform_matrix((1, 0, 0), (0, s, 0, s))
        indices, u, v = project_visible(
            np.array([[3, 0, 0]], np.float32), np.linalg.inv(t_map_camera),
            (4, 4, 4, 4), np.full((9, 9), 2, np.float32))
        np.testing.assert_array_equal(indices, [0])
        np.testing.assert_array_equal([u[0], v[0]], [4, 4])


class FusionTest(unittest.TestCase):
    def test_color_and_labels_revise_over_time(self):
        points = np.array([[0, 0, 2], [1, 0, 2]], np.float32)
        fusion = VoxelFusion(voxel_m=0.05)
        fusion.observe(points, np.array([0]), np.array([[255, 0, 0]]),
                       np.array([62]), np.array([0.9]), 1.0)
        fusion.observe(points, np.array([0]), np.array([[0, 0, 255]]),
                       np.array([67]), np.array([0.9]), 2.0)
        rgb, labels, confidence = fusion.render(points)
        self.assertTrue(120 <= rgb[0, 0] <= 135)
        self.assertTrue(120 <= rgb[0, 2] <= 135)
        self.assertEqual(labels[0], 67)
        self.assertEqual(labels[1], 0)
        self.assertGreater(confidence[0], 0)
        # Repeated later chair observations must correct a mistaken table.
        for stamp in range(3, 10):
            fusion.observe(points, np.array([0]), np.array([[0, 255, 0]]),
                           np.array([62]), np.array([0.9]), float(stamp))
        _, labels, _ = fusion.render(points)
        self.assertEqual(labels[0], 62)

    def test_cloud_replacement_uses_map_voxels(self):
        fusion = VoxelFusion()
        p = np.array([[1.001, 2.001, 3.001]], np.float32)
        fusion.observe(p, np.array([0]), np.array([[20, 30, 40]]),
                       np.array([0]), np.array([0.0]), 1.0)
        rgb, _, _ = fusion.render(np.array([[1.002, 2.002, 3.002]], np.float32))
        np.testing.assert_array_equal(rgb[0], [20, 30, 40])


if __name__ == "__main__":
    unittest.main()
