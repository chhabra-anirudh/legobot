import unittest
import numpy as np
from calibrate_gripper import GripperGeometry, slab_vertices


class ClippingTests(unittest.TestCase):
    def test_triangle_crossing_slab_without_vertices_inside(self):
        triangles = np.array([[[-2, 3, -2], [2, 3, -2], [0, 3, 2]]], dtype=float)
        points = slab_vertices(triangles, [0, 0, 0], 1)
        self.assertGreater(len(points), 0)
        self.assertTrue(np.all(np.abs(points[:, [0, 2]]) <= .5))
        np.testing.assert_allclose(points[:, 1], 3)

    def test_disjoint_slab(self):
        points = slab_vertices(np.array([[[3, 0, 3], [4, 0, 3], [3, 0, 4]]]), [0, 0, 0], 1)
        self.assertEqual(points.shape, (0, 3))


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = GripperGeometry()

    def test_candidate_matches_width_and_recenters_cube(self):
        candidate = self.geometry.contact_candidate([0, 0, .004], .0254, .35)
        measured = self.geometry.measure(candidate['angle_rad'], candidate['cube_center_tool_m'], .0254)
        self.assertAlmostEqual(measured['gap_m'], .0254, places=7)
        self.assertAlmostEqual(measured['left_clearance_m'], 0, places=7)
        self.assertAlmostEqual(measured['right_clearance_m'], 0, places=7)

    def test_current_animation_does_not_contact_cube(self):
        measured = self.geometry.measure(.19, [0, 0, .004], .0254)
        self.assertGreater(measured['left_clearance_m'], 0)
        self.assertGreater(measured['right_clearance_m'], 0)

    def test_missing_geometry_is_not_reported_as_clear(self):
        self.assertFalse(self.geometry.measure(.2, [0, 0, 10], .0254)['slab_intersects_both_fingers'])

    def test_invalid_joint_angle(self):
        for angle in [-.1, 1.1, float('nan')]:
            with self.assertRaises(ValueError):
                self.geometry.triangles(angle)


if __name__ == '__main__':
    unittest.main()
