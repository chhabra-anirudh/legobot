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
from schema import Structure, Voxel, load as load_structure  # noqa: E402
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
        # Exactly what main() uses: the measurement minus the robot's own chassis.
        # Testing against the raw map put cubes inside the machine.
        cls.reach = reach_map.clear_of_base(reach_map.load(cls.table), cls.c)
        cls.pitch = float(cls.c['block_size_m'][0])

    def test_reach_map_exposes_its_sample_spacing(self):
        """The map's spacing is its own property, not the block size: it was probed
        on a 25.4 mm lattice and the blocks are now 50.8 mm across. Callers match
        against the nearest sample, so the two need not agree."""
        data = reach_map.load_full(self.table)
        self.assertEqual(data['cells'], reach_map.load(self.table))
        xs = sorted({x for x, _ in data['cells']})
        spacing = min(b-a for a, b in zip(xs, xs[1:]) if b > a)
        self.assertAlmostEqual(float(data['pitch_m']), spacing, places=6)

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

    def centre_of(self, origin):
        cal = calibration_for(self.c, origin)
        return [B.cell_centre(v.cell, self.c, cal) for v in self.structure.voxels]

    def test_facing_the_robot_is_opt_in_and_costs_carried_tilt(self):
        """`--face` puts the build beyond the chassis nose. It is not the default
        because the arm works folded there and the carried cube tips much further."""
        cal = calibration_for(self.c, [0, 0])
        nose = float(np.asarray(self.c['base_footprint_m'], dtype=float)[0][1])
        # A three-block line, small enough for the shallow area in front of the
        # chassis; the dog does not fit there at 50.8 mm cells.
        small = Structure('line', 'line', [Voxel(x, 0, 0, 'red') for x in range(3)])
        facing = B.candidate_origins(small, self.c, cal, self.reach, self.pitch,
                                     face=True)[0]
        placed = [B.cell_centre(v.cell, self.c, calibration_for(self.c, facing))
                  for v in small.voxels]
        self.assertGreater(min(p[0] for p in placed), nose)
        free = B.candidate_origins(small, self.c, cal, self.reach, self.pitch)[0]
        self.assertNotEqual(list(free), list(facing),
                            'facing should change the chosen origin')

    def test_candidate_origins_are_ranked_front_then_near_and_are_deterministic(self):
        """Front half first, then closest to the base. Distance is radial: once the
        whole disc is probed the reachable set is a ring, so "smallest y" is the
        robot's right-hand side, not near it."""
        cal = calibration_for(self.c, [0, 0])
        fits = B.candidate_origins(self.structure, self.c, cal, self.reach, self.pitch)
        self.assertEqual(fits, B.candidate_origins(self.structure, self.c, cal,
                                                   self.reach, self.pitch))
        behind = [sum(p[0] for p in self.centre_of(o))/len(self.structure.voxels) < 0
                  for o in fits]
        self.assertEqual(behind, sorted(behind), 'front-half origins must come first')
        front = [o for o, is_behind in zip(fits, behind) if not is_behind]
        self.assertTrue(front)
        distances = [round(max(np.hypot(p[0], p[1]) for p in self.centre_of(o)), 3)
                     for o in front]
        self.assertEqual(distances, sorted(distances), 'front origins run near to far')

    def test_the_chosen_build_sits_in_front_of_the_robot(self):
        cal = calibration_for(self.c, [0, 0])
        best = B.candidate_origins(self.structure, self.c, cal, self.reach, self.pitch)[0]
        centres = self.centre_of(best)
        self.assertGreater(sum(p[0] for p in centres)/len(centres), 0,
                           'the build ended up behind the robot')

    def test_no_cube_is_placed_inside_the_robots_own_base(self):
        """IK converges at points inside the machine, so the map alone is not enough.
        Before this filter existed, 14 of the dog's 16 cubes were inside the base."""
        cal = calibration_for(self.c, B.candidate_origins(
            self.structure, self.c, calibration_for(self.c, [0, 0]), self.reach, self.pitch)[0])
        positions, _, _ = B.staging_layout(self.structure, self.c, cal, self.reach)
        (x_min, x_max), (y_min, y_max) = np.asarray(self.c['base_footprint_m'], dtype=float)
        half = self.pitch/2
        for point in list(positions) + self.centre_of(cal['origin_xy_m']):
            self.assertFalse(x_min-half < point[0] < x_max+half
                             and y_min-half < point[1] < y_max+half,
                             f'cube at {np.round(point, 4)} is inside the robot base')

    def test_the_raw_reach_map_does_include_cells_inside_the_base(self):
        """Guards the reason `clear_of_base` exists: the measurement really does
        report cells the chassis occupies, because IK does not know about it."""
        raw = reach_map.load(self.table)
        self.assertLess(len(reach_map.clear_of_base(raw, self.c)), len(raw))

    def test_staging_fills_from_the_outside_of_the_workspace_inwards(self):
        cal = calibration_for(self.c, B.candidate_origins(
            self.structure, self.c, calibration_for(self.c, [0, 0]), self.reach, self.pitch)[0])
        positions, _, _ = B.staging_layout(self.structure, self.c, cal, self.reach)
        distances = [float(np.hypot(*p[:2])) for p in positions]
        # The first slot handed out is the outermost eligible one, to within the
        # millimetre that the sort rounds to so that near-equal radii group by row
        # and each colour stays in a contiguous run. Not necessarily the outermost
        # reachable cell either: slots sit on a two-cell lattice and keep a moat
        # around the build, so some outer cells are not eligible at all.
        self.assertGreaterEqual(distances[0], max(distances) - .001)
        # No claim that every slot lies outside the build: at 50.8 mm cells this dog
        # spans 0.36 m, so the supply has to share the workspace with it.
        build = [np.hypot(p[0], p[1]) for p in self.centre_of(cal['origin_xy_m'])]
        self.assertGreater(distances[0], max(build),
                           'the outermost slot should still be beyond the build')

    def test_the_drawn_table_covers_the_workspace_and_every_cube(self):
        """The old box was hardcoded at x 0.08..0.52 and predated the reach map, so
        builds at negative x were animated hovering over nothing."""
        cal = calibration_for(self.c, B.candidate_origins(
            self.structure, self.c, calibration_for(self.c, [0, 0]), self.reach, self.pitch)[0])
        positions, _, _ = B.staging_layout(self.structure, self.c, cal, self.reach)
        build = [B.cell_centre(v.cell, self.c, cal) for v in self.structure.voxels]
        centre, half = B.table_box(self.c, list(self.reach)+build+list(positions))
        cube = float(self.c['block_size_m'][0])/2
        for point in list(positions) + build + [(x, y) for x, y in self.reach]:
            for axis in (0, 1):
                self.assertLessEqual(abs(float(point[axis])-centre[axis]) + cube, half[axis],
                                     f'{point} is not on the table')
        self.assertAlmostEqual(centre[2], self.table-.015, places=6)

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
        cls.reach = reach_map.clear_of_base(
            reach_map.load(float(cls.c['table_height_m'])), cls.c)
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
        two foam pads holding a cube by friction might.

        The bound has moved twice and the history is the point. It was 30 deg, based
        on 25.5 deg measured on a layout that put 14 of this dog's 16 cubes **inside
        the robot's chassis** — the reach map did not exclude the base. Placing
        honestly pushed it to 33-47, and forcing the build in front of the robot to
        76, because the arm works folded there. Facing is now opt-in (`--face`) and
        the free placement measures 22.8 deg, better than the original figure.

        Do not raise this bound to make a placement policy fit. Check slip in
        `sim/contact_grasp.py` first: at 76 deg two foam pads holding by friction is
        a different proposition from 23.
        """
        carrying = [self.tilt(f[0]) for f in self.frames if f[5] is not None]
        self.assertTrue(carrying)
        self.assertLess(max(carrying), 30., 'carried tilt grew beyond the measured 22.8 deg')
        self.assertGreater(max(carrying), 5., 'tilt vanished: update the docs if this is fixed')

    def test_the_planner_reports_the_carried_tilt_it_measured(self):
        """The number has to reach the operator, not just the test suite."""
        tilt, tipped, carrying = B.carried_tilt_deg(self.arm, self.frames)
        self.assertEqual(carrying, sum(1 for f in self.frames if f[5] is not None))
        self.assertAlmostEqual(tilt, max(self.tilt(f[0]) for f in self.frames
                                         if f[5] is not None), places=6)
        self.assertGreater(tipped, 0)

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
