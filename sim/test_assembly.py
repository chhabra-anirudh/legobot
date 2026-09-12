"""Numerical checks of the motion contract; run with unittest."""
import unittest
import numpy as np
from simulate_assembly import Arm, HERE, load_config, plan


class AssemblyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = load_config(HERE/'assembly_config.json')
        cls.arm = Arm()
        cls.frames = plan(cls.c, cls.arm)

    def test_joint_limits_and_tool_orientation(self):
        for q, _, _, _, _ in self.frames:
            self.assertTrue(np.all(q >= self.arm.lo) and np.all(q <= self.arm.hi))
            _, R = self.arm.fk(q)
            self.assertLess(np.linalg.norm(R-np.diag([1, -1, -1])), .01)

    def test_final_stack_accuracy(self):
        h = self.c['block_size_m'][2]
        for i, actual in enumerate(self.frames[-1][1]):
            target = np.r_[self.c['stack_xy_m'], self.c['table_height_m']+h*(i+.5)]
            self.assertLess(np.linalg.norm(actual-target), .001)

    def test_attachment_and_stationary_released_blocks(self):
        previous = self.frames[0][1]
        attachment = None
        previous_held = None
        for q, blocks, _, held, grip in self.frames:
            if held is not None:
                p, R = self.arm.fk(q)
                if previous_held is None:
                    np.testing.assert_allclose(blocks, previous, atol=1e-12)
                    attachment = R.T @ (blocks[held]-p)
                np.testing.assert_allclose(blocks[held], p+R@attachment, atol=1e-12)
                self.assertEqual(grip, self.c['gripper_grasp_rad'])
            for i in range(len(blocks)):
                if i != held:
                    np.testing.assert_array_equal(blocks[i], previous[i])
            previous, previous_held = blocks, held

    def test_finger_sequence(self):
        angles = [frame[4] for frame in self.frames]
        self.assertEqual(angles[0], self.c['gripper_open_rad'])
        self.assertEqual(angles[-1], self.c['gripper_open_rad'])
        self.assertTrue(all(self.c['gripper_grasp_rad'] <= a <= self.c['gripper_open_rad'] for a in angles))
        for q, blocks, label, held, grip in self.frames:
            if 'descend' in label or 'retreat' in label:
                self.assertEqual(grip, self.c['gripper_open_rad'])
        self.assertEqual(sum('attach (' in f[2] for f in self.frames), 3)

    def test_unreachable_target_rejected(self):
        with self.assertRaises(ValueError):
            self.arm.solve(np.array([10., 10., 10.]), np.diag([1., -1., -1.]), np.zeros(7))


if __name__ == '__main__':
    unittest.main()
