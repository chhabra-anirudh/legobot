import contextlib
import io
import unittest
from pathlib import Path

from schema import Structure, Voxel, build_order, validate
from generate import from_ascii, offline
from preview import layer_view, render

EXAMPLES = Path(__file__).resolve().parent/'examples'


def structure(voxels, **kwargs):
    kwargs.setdefault('structure_id', 'test-1')
    kwargs.setdefault('name', 'test')
    return Structure(voxels=voxels, **kwargs)


def tower(height, color='red'):
    return [Voxel(0, 0, z, color) for z in range(height)]


class ValidationTests(unittest.TestCase):
    def test_supported_tower_passes(self):
        report = validate(structure(tower(3)))
        self.assertTrue(report.ok, report.codes())
        self.assertEqual(report.counts['cubes'], 3)
        self.assertEqual(report.counts['layers'], 3)

    def test_floating_voxel_is_rejected(self):
        report = validate(structure([Voxel(0, 0, 0, 'red'), Voxel(0, 0, 2, 'red')]))
        self.assertIn('unsupported', report.codes())

    def test_overhang_on_a_side_neighbour_is_rejected(self):
        # (1,0,1) touches (0,0,1) but has nothing directly below it.
        voxels = [Voxel(0, 0, 0, 'red'), Voxel(0, 0, 1, 'red'), Voxel(1, 0, 1, 'red')]
        report = validate(structure(voxels))
        self.assertIn('unsupported', report.codes())
        self.assertIn((1, 0, 1), [p.cell for p in report.problems])

    def test_disconnected_islands_are_rejected(self):
        report = validate(structure([Voxel(0, 0, 0, 'red'), Voxel(5, 5, 0, 'red')]))
        self.assertIn('disconnected', report.codes())

    def test_duplicate_cell_is_rejected(self):
        report = validate(structure([Voxel(0, 0, 0, 'red'), Voxel(0, 0, 0, 'blue')]))
        self.assertIn('duplicate', report.codes())

    def test_negative_layer_is_rejected(self):
        report = validate(structure([Voxel(0, 0, -1, 'red')]))
        self.assertIn('negative_layer', report.codes())

    def test_unknown_color_is_rejected(self):
        report = validate(structure([Voxel(0, 0, 0, 'chartreuse')]))
        self.assertIn('unknown_color', report.codes())

    def test_boolean_coordinate_is_not_an_integer(self):
        report = validate(structure([Voxel(True, 0, 0, 'red')]))
        self.assertIn('non_integer', report.codes())

    def test_budgets_and_inventory(self):
        self.assertIn('cube_budget', validate(structure(tower(3)),
                                              {'max_cubes': 2}).codes())
        self.assertIn('layer_budget', validate(structure(tower(3)),
                                               {'max_layers': 2}).codes())
        self.assertIn('inventory', validate(structure(tower(3)),
                                            inventory={'red': 1}).codes())

    def test_wrong_cube_size_is_rejected(self):
        self.assertIn('cube_size', validate(structure(tower(1), cube_size_m=.05)).codes())

    def test_empty_structure_is_rejected(self):
        self.assertIn('empty', validate(structure([])).codes())

    def test_round_trip_preserves_voxels(self):
        original = structure(tower(2), source='offline_library')
        restored = Structure.from_dict(original.to_dict())
        self.assertEqual(restored.to_dict(), original.to_dict())
        self.assertEqual(restored.source, 'offline_library')

    def test_build_order_places_support_before_the_cube_above(self):
        placed = set()
        for v in build_order(structure(tower(4))):
            if v.z:
                self.assertIn((v.x, v.y, v.z-1), placed)
            placed.add(v.cell)


class OfflineDogTests(unittest.TestCase):
    def test_ascii_rows_map_first_row_to_highest_y(self):
        voxels = from_ascii('R.\n.R')
        self.assertIn((0, 1, 0), [v.cell for v in voxels])
        self.assertIn((1, 0, 0), [v.cell for v in voxels])

    def test_offline_dog_passes_every_check(self):
        """Eleven blocks, not the twenty-odd it used to be: blocks are 2 inches across
        now, so a cell covers four times the table area and the reachable workspace
        takes a much smaller drawing."""
        dog = offline('a simple dog')
        report = validate(dog)
        self.assertTrue(report.ok, report.codes())
        self.assertEqual(report.counts['layers'], 1)
        self.assertGreater(report.counts['cubes'], 8)
        self.assertLessEqual(max(report.counts['footprint']), 10)

    def test_offline_structures_are_never_labelled_as_model_output(self):
        self.assertEqual(offline('dog').source, 'offline_library')

    def test_unknown_subject_has_no_offline_shape(self):
        with self.assertRaises(KeyError):
            offline('a spiral staircase')


class PreviewTests(unittest.TestCase):
    def test_layer_view_marks_occupied_cells(self):
        view = layer_view(structure([Voxel(0, 0, 0, 'red')]), 0)
        self.assertEqual(view, 'R')

    def test_render_reports_rejection(self):
        bad = structure([Voxel(0, 0, 0, 'red'), Voxel(0, 0, 2, 'red')])
        self.assertIn('REJECTED', render(bad))

    def test_render_reports_acceptance_and_source(self):
        text = render(offline('dog'))
        self.assertIn('checks: passed', text)
        self.assertIn('offline_library', text)


def quiet(argv):
    """Run the check CLI without printing its preview into the test output."""
    import check
    with contextlib.redirect_stdout(io.StringIO()):
        return check.main(argv)


class ExampleFileTests(unittest.TestCase):
    """The cached demo structures must keep behaving as documented."""

    def test_accepted_example_passes_and_records_its_model(self):
        from schema import load
        dog = load(EXAMPLES/'dog.json')
        self.assertEqual(dog.source, 'claude-opus-5')
        self.assertTrue(validate(dog).ok)
        self.assertEqual(quiet([str(EXAMPLES/'dog.json')]), 0)

    def test_rejection_example_fails_for_the_documented_reasons(self):
        """Floating cube and island, plus no finger clearance.

        This example is the original 45-cube dog silhouette with two faults added.
        A solid silhouette is also unbuildable in its own right: with two opposed
        pads, an interior cube whose in-layer neighbours are already placed cannot
        be reached, so `no_clearance` is reported alongside the two planted faults.
        The accepted `dog.json` is a one-cube-wide outline for exactly this reason.
        """
        from schema import load
        report = validate(load(EXAMPLES/'dog-floating-rejected.json'))
        self.assertFalse(report.ok)
        self.assertEqual(report.codes(), ['disconnected', 'no_clearance', 'unsupported'])
        self.assertEqual(quiet([str(EXAMPLES/'dog-floating-rejected.json')]), 1)


if __name__ == '__main__':
    unittest.main()
