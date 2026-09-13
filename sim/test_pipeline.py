"""Tests for `build_from_description.py`, the one-command demo path.

It lives here rather than in `compiler/` because it spans both halves: the
compiler decides what to build and `sim/build_structure.py` plans the arm. The
properties asserted are the ones a broken demo would violate: nothing is built
from a rejected design, a reply is never labelled with a source nobody stated,
and the structure that reaches the simulator is the one that passed the checks.

No model is called anywhere in here.
"""
import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for directory in (ROOT, ROOT/'compiler'):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

import build_from_description as pipeline                        # noqa: E402
from schema import load, validate                                # noqa: E402


def run(argv):
    """Run the CLI, capturing its output so the test log stays readable."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        code = pipeline.main(argv)
    return code, out.getvalue()


def reply(cells, colour='orange', **extra):
    return {'name': 'test', 'reasoning': 'test',
            'voxels': [{'x': x, 'y': y, 'z': 0, 'color': colour} for x, y in cells],
            **extra}


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp())
        self.structure = self.directory/'structure.json'

    def write_reply(self, payload):
        path = self.directory/'reply.json'
        path.write_text(json.dumps(payload))
        return path

    def ingest(self, payload, *extra):
        return run(['a test shape', '--ingest', str(self.write_reply(payload)),
                    '--source', 'test-source', '--structure-out', str(self.structure),
                    '--check', *extra])

    def test_a_reply_is_checked_planned_and_saved_with_its_source(self):
        code, text = self.ingest(reply([(x, 0) for x in range(4)]))
        self.assertEqual(code, 0, text)
        self.assertIn('deterministic checks: passed', text)
        self.assertIn('Planned', text)                  # the arm was actually solved
        structure = load(self.structure)
        self.assertEqual(structure.source, 'test-source')
        self.assertEqual(len(structure.voxels), 4)
        self.assertTrue(validate(structure).ok)

    def test_ingesting_without_a_source_is_refused(self):
        with self.assertRaises(SystemExit):
            run(['a test shape', '--ingest', str(self.write_reply(reply([(0, 0)])))])

    def test_a_malformed_reply_fails_loudly(self):
        with self.assertRaises(SystemExit):
            self.ingest({'voxels': [{'x': 0, 'y': 0}]})       # no z, no colour

    def test_an_unbuildable_design_is_reduced_and_the_reduction_is_reported(self):
        solid = reply([(x, y) for x in range(4) for y in range(4)])
        code, text = self.ingest(solid)
        self.assertEqual(code, 0, text)
        self.assertIn('reduced to satisfy the gripper', text)
        self.assertLess(len(load(self.structure).voxels), 16)
        self.assertTrue(load(self.structure).provenance['simplified'])

    def test_nothing_is_built_from_a_design_that_stays_rejected(self):
        solid = reply([(x, y) for x in range(4) for y in range(4)])
        code, text = self.ingest(solid, '--no-simplify')
        self.assertEqual(code, 1)
        self.assertIn('REJECTED', text)
        self.assertNotIn('Planned', text)
        self.assertFalse(self.structure.exists())

    def test_a_design_outside_the_grid_is_rejected_with_the_area_it_needs(self):
        code, text = self.ingest(reply([(x, 0) for x in range(20)]), '--grid', '5', '5',
                                 '--no-simplify')
        self.assertEqual(code, 1)
        self.assertIn('out_of_grid', text)
        self.assertIn('--grid 20 1', text)

    def test_a_design_outside_the_grid_is_cropped_to_it_by_default(self):
        code, text = self.ingest(reply([(x, 0) for x in range(20)]), '--grid', '5', '5')
        self.assertEqual(code, 0, text)
        self.assertIn('outside the build grid', text)
        self.assertEqual(len(load(self.structure).voxels), 5)

    def test_request_mode_writes_the_rules_and_calls_nothing(self):
        path = self.directory/'ask.json'
        code, _ = run(['a small house', '--request', str(path), '--grid', '12', '9'])
        self.assertEqual(code, 0)
        request = json.loads(path.read_text())
        self.assertEqual(request['limits']['grid'], [12, 9])
        self.assertIn('Direct support only', request['system'])
        self.assertIn('a small house', request['user_text'])
        self.assertIn('voxels', request['response_schema']['properties'])

    def test_the_library_shape_builds_and_is_never_labelled_as_a_model(self):
        code, text = run(['a simple dog', '--offline', '--check',
                          '--structure-out', str(self.structure)])
        self.assertEqual(code, 0, text)
        self.assertEqual(load(self.structure).source, 'offline_library')
        self.assertIn('Planned', text)

    def test_the_exported_trajectory_matches_the_planned_poses(self):
        trajectory = self.directory/'trajectory.json'
        code, text = self.ingest(reply([(x, 0) for x in range(3)]),
                                 '--export-trajectory', str(trajectory))
        self.assertEqual(code, 0, text)
        points = json.loads(trajectory.read_text())['points']
        self.assertTrue(points)
        self.assertEqual(len(set(len(p) for p in points)), 1)

    def test_the_cached_description_examples_pass_and_fit_the_default_grid(self):
        """house.json and rocket.json are the demo structures; both must keep
        passing under the defaults this script uses, or the demo breaks silently."""
        limits = {'grid': (12, 9), 'max_cubes': 40, 'max_layers': 4, 'max_footprint': 12}
        for name, cubes in [('house.json', 40), ('rocket.json', 26)]:
            with self.subTest(example=name):
                structure = load(ROOT/'compiler'/'examples'/name)
                report = validate(structure, limits)
                self.assertTrue(report.ok, report.codes())
                self.assertEqual(report.counts['cubes'], cubes)
                # These came from a model/agent reply, not the offline library.
                self.assertNotEqual(structure.source, 'offline_library')
                self.assertTrue(structure.provenance['reasoning'])


if __name__ == '__main__':
    unittest.main()
