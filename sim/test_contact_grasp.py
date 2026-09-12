"""Physics contract tests; optional dependencies live in requirements-physics.txt."""
import importlib.util
import unittest

import numpy as np

HAS_MUJOCO = importlib.util.find_spec('mujoco') is not None
if HAS_MUJOCO:
    import mujoco
    from contact_grasp import make_model, run_episode


@unittest.skipUnless(HAS_MUJOCO, 'install requirements-physics.txt for contact checks')
class ContactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result, cls.frames = run_episode()

    def test_default_grasp_holds_with_contact_for_entire_interval(self):
        self.assertTrue(self.result['numerically_stable'])
        self.assertTrue(self.result['hold_success'])
        self.assertTrue(self.result['success'])
        self.assertFalse(self.result['finger_table_collision'])
        self.assertEqual(self.result['hold_contact_fraction'], 1)

    def test_open_gripper_cannot_lift(self):
        result, _ = run_episode(close=False)
        self.assertFalse(result['hold_success'])
        self.assertFalse(result['success'])

    def test_zero_friction_cannot_pass_hold(self):
        result, _ = run_episode(friction=0)
        self.assertFalse(result['hold_success'])

    def test_cube_is_free_and_has_no_weld(self):
        model = make_model()
        cube = model.body('cube').id
        self.assertEqual(model.joint('cube_free').type[0], mujoco.mjtJoint.mjJNT_FREE)
        for i in range(model.neq):
            if model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
                self.assertNotIn(cube, [model.eq_obj1id[i], model.eq_obj2id[i]])

    def test_vertical_commands_and_mimic_tracking(self):
        target_xy = self.frames[0]['target_position_m'][:2]
        for frame in self.frames:
            np.testing.assert_allclose(frame['target_position_m'][:2], target_xy)
            self.assertLess(abs(np.diff(frame['finger_angles_rad'])[0]), .01)

    def test_full_success_requires_placement_threshold(self):
        self.assertEqual(self.result['success'], self.result['hold_success'] and
                         not self.result['finger_table_collision'] and
                         self.result['placement_error_m'] < .003 and
                         self.result['final_tilt_rad'] < np.deg2rad(5) and
                         self.result['final_speed_m_s'] < .01)

    def test_without_spin_friction_release_drift_is_detected(self):
        result, _ = run_episode(torsional_friction=0)
        self.assertTrue(result['hold_success'])
        self.assertFalse(result['success'])
        self.assertGreater(result['placement_error_m'], .003)

    def test_warning_callback_restored_after_failure(self):
        previous = mujoco.get_mju_user_warning()
        with self.assertRaises(ValueError):
            run_episode(depth=1)
        self.assertIs(mujoco.get_mju_user_warning(), previous)

    def test_invalid_physics_parameters(self):
        for kwargs in [{'friction': -1}, {'mass': 0}, {'geometry': 'unknown'}]:
            with self.assertRaises(ValueError):
                make_model(**kwargs)
        with self.assertRaises(ValueError):
            run_episode(depth=1)


if __name__ == '__main__':
    unittest.main()
