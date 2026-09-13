"""Checks on the planner that produces hardware trajectories.

This module exports the joint path that `robot/build_bridge.py` streams to the
real arm, so the properties asserted here are the ones a bad plan would violate
before anyone notices on hardware: joint limits, tool orientation, reach, and a
reproducible export.
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent/'compiler'))

import build_structure as B  # noqa: E402
import reach_map  # noqa: E402
from simulate_assembly import Arm, load_config  # noqa: E402
from schema import load as load_structure  # noqa: E402
from tool_frame import GRIP_YAW, rotation_for  # noqa: E402

DOG = HERE.parent/'compiler'/'examples'/'dog.json'


def calibration_for(c, origin, pitch=None):
    pitch = pitch if pitch else float(c['block_size_m'][0])
    return {'origin_xy_m': np.array(origin, dtype=float),
            'pitch_m': np.array([pitch, pitch]),
            'layer_height_m': float(c['block_size_m'][2])}


class ReachTests(unittest.TestCase):
    """The reach map must gate the build area, not only the staging slots."""

    @classmethod
    def setUpClass(cls):
        cls.c = load_config(HERE/'assembly_config.json')
        cls.structure = load_structure(DOG)
        cls.table = float(cls.c['table_height_m'])
        cls.reach = reach_map.load(cls.table)
        cls.pitch = float(cls.c['block_size_m'][0])

    def test_reach_map_exposes_its_sample_spacing(self):
        data = reach_map.load_full(self.table)
        self.assertEqual(data['cells'], self.reach)
        self.assertAlmostEqual(float(data['pitch_m']), self.pitch)

    def test_far_origin_is_reported_as_unreachable(self):
        """The old hardcoded default put part of the dog outside the workspace."""
        cal = calibration_for(self.c, [.22, .30])
        missing = B.unreachable_columns(self.structure, self.c, cal, self.reach, self.pitch)
        self.assertTrue(missing)
        for column, _ in missing:
            self.assertIn(column, {(v.x, v.y) for v in self.structure.voxels})

    def test_candidate_origins_are_all_actually_in_reach(self):
        cal = calibration_for(self.c, [0, 0])
        fits = B.candidate_origins(self.structure, self.c, cal, self.reach, self.pitch)
        self.assertTrue(fits)
        for origin in fits:
            cal = calibration_for(self.c, origin)
            self.assertEqual(
                B.unreachable_columns(self.structure, self.c, cal, self.reach, self.pitch), [],
                f'origin {origin} was offered but does not fit')

    def test_candidate_origins_are_ranked_by_margin_and_deterministic(self):
        cal = calibration_for(self.c, [0, 0])
        first = B.candidate_origins(self.structure, self.c, cal, self.reach, self.pitch)
        self.assertEqual(first, B.candidate_origins(self.structure, self.c, cal,
                                                    self.reach, self.pitch))
        margins = []
        for origin in first:
            cells = [(round(origin[0]+v.x*self.pitch, 4), round(origin[1]+v.y*self.pitch, 4))
                     for v in self.structure.voxels]
            margins.append(min(B._margin(cell, self.reach, self.pitch) for cell in cells))
        self.assertEqual(margins, sorted(margins, reverse=True))

    def test_plan_refuses_an_out_of_reach_origin_and_names_the_cells(self):
        arm = Arm(HERE/'robot_limits.json')
        cal = calibration_for(self.c, [.22, .30])
        with self.assertRaises(ValueError) as caught:
            B.plan_build(self.structure, self.c, arm, cal, self.reach)
        message = str(caught.exception)
        self.assertIn('outside the arm', message)
        self.assertIn('--origin', message)   # must suggest a way forward


class PlanTests(unittest.TestCase):
    """Properties of a planned build that hardware would punish us for."""

    @classmethod
    def setUpClass(cls):
        cls.c = load_config(HERE/'assembly_config.json')
        cls.arm = Arm(HERE/'robot_limits.json')
        cls.structure = load_structure(DOG)
        cls.reach = reach_map.load(float(cls.c['table_height_m']))
        cal = calibration_for(cls.c, [0, 0])
        pitch = float(cls.c['block_size_m'][0])
        cls.origin = B.candidate_origins(cls.structure, cls.c, cal, cls.reach, pitch)[0]
        cls.cal = calibration_for(cls.c, cls.origin)
        cls.frames, cls.steps, cls.staged, cls.cold, cls.plan = B.plan_build(
            cls.structure, cls.c, cls.arm, cls.cal, cls.reach)

    def test_every_pose_respects_the_robots_calibrated_joint_limits(self):
        for q, *_ in self.frames:
            self.assertTrue(np.all(q >= self.arm.lo - 1e-9), 'joint below the arm limit')
            self.assertTrue(np.all(q <= self.arm.hi + 1e-9), 'joint above the arm limit')

    GRASP_PHASES = ('descend', 'close fingers', 'attach', 'place', 'release', 'open fingers')

    def tilt(self, q):
        """Angle between the tool z-axis and straight down, in degrees."""
        _, R = self.arm.fk(q)
        return float(np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1))))

    def test_tool_points_straight_down_whenever_it_touches_a_cube(self):
        """Approach, grip, and release must be vertical: a tilted pad pair does
        not meet opposing cube faces."""
        checked = 0
        for frame in self.frames:
            label = frame[4]
            if not any(phase in label for phase in self.GRASP_PHASES):
                continue
            checked += 1
            self.assertLess(self.tilt(frame[0]), .5, f'tool tilted during {label!r}')
        self.assertGreater(checked, 100, 'expected many grasp-phase frames')

    def test_joint_space_transfers_tilt_the_tool_while_carrying(self):
        """A known limitation, asserted so it cannot quietly get worse.

        Transfers interpolate in joint space, which keeps every intermediate pose
        reachable but does not hold the tool vertical. The cube is carried through
        that tilt. Attachment is idealized here so the simulation does not care;
        two foam pads holding a cube by friction might. If this bound ever needs
        raising, check the slip case first.
        """
        carrying = [self.tilt(f[0]) for f in self.frames if f[5] is not None]
        self.assertTrue(carrying)
        self.assertLess(max(carrying), 30., 'carried tilt grew beyond the documented 25.5 deg')
        self.assertGreater(max(carrying), 5., 'tilt vanished: update the docs if this is fixed')

    def test_one_step_per_cube_and_every_cube_placed_once(self):
        self.assertEqual(len(self.steps), len(self.structure.voxels))
        cells = [voxel.cell for voxel, _ in self.steps]
        self.assertEqual(sorted(cells), sorted(v.cell for v in self.structure.voxels))

    def test_support_comes_before_the_cube_it_holds(self):
        placed = set()
        for voxel, _ in self.steps:
            x, y, z = voxel.cell
            if z:
                self.assertIn((x, y, z-1), placed, f'{voxel.cell} placed before its support')
            placed.add(voxel.cell)

    def test_gripper_angle_never_leaves_the_configured_range(self):
        lo, hi = sorted((self.c['gripper_grasp_rad'], self.c['gripper_open_rad']))
        for frame in self.frames:
            self.assertGreaterEqual(frame[6], lo - 1e-9)
            self.assertLessEqual(frame[6], hi + 1e-9)

    def test_placement_plan_targets_match_the_grasp_offset(self):
        """`tool_target_m` must be the cube centre moved by the rotated grasp point."""
        for row in self.plan['placements']:
            rotation = rotation_for(GRIP_YAW[row['grip_axis']])
            expected = np.array(row['cube_centre_m']) - rotation @ self.c['tool_grasp_point_m']
            np.testing.assert_allclose(row['tool_target_m'], expected, atol=1e-5)

    def test_plan_records_the_origin_it_was_built_from(self):
        np.testing.assert_allclose(self.plan['calibration']['build_origin_m'],
                                   np.round(self.origin, 5), atol=1e-9)


class ExportTests(unittest.TestCase):
    """The robot bridge consumes these files; their shape is a contract."""

    def test_trajectory_export_is_reproducible_and_well_formed(self):
        with tempfile.TemporaryDirectory() as tmp:
            first, second = Path(tmp)/'a.json', Path(tmp)/'b.json'
            for out in (first, second):
                code = B.main([str(DOG), '--check', '--export-trajectory', str(out)])
                self.assertEqual(code, 0)
            self.assertEqual(first.read_bytes(), second.read_bytes(),
                             'same inputs must give a byte-identical trajectory')
            traj = json.loads(first.read_text())

        c = load_config(HERE/'assembly_config.json')
        self.assertEqual(len(traj['joint_names']), 8)
        self.assertEqual(traj['joint_names'][-1], 'left_left_gripper')
        self.assertAlmostEqual(traj['frame_seconds'], float(c['frame_seconds']))
        self.assertTrue(traj['points'])
        for point in traj['points']:
            self.assertEqual(len(point), len(traj['joint_names']),
                             'every point must carry one value per joint')
            self.assertTrue(all(np.isfinite(point)))

    def test_unreachable_origin_exits_nonzero_without_writing_a_trajectory(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'traj.json'
            code = B.main([str(DOG), '--check', '--origin', '0.22', '0.30',
                           '--export-trajectory', str(out)])
            self.assertEqual(code, 2)
            self.assertFalse(out.exists(), 'a rejected plan must not leave a trajectory behind')


if __name__ == '__main__':
    unittest.main()
