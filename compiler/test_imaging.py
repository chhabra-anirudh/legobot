"""Tests for the picture path: image to grid, reduction to buildable, and the CLI.

The properties asserted here are the ones a broken reduction would violate: the
grid keeps the picture's aspect and colours, the reduction always returns
something `schema.validate` accepts, and it never silently keeps a cube the
gripper cannot place or drops one it could.
"""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

import image_to_structure
import imaging
import simplify
from placement import _greedy, _ranking, sequence
from schema import Structure, Voxel, load, validate

try:
    from PIL import Image, ImageDraw
    import numpy  # noqa: F401  imaging needs both
    PILLOW = True
except ImportError:                        # pragma: no cover - environment dependent
    PILLOW = False

HERE = Path(__file__).resolve().parent
CAT_IMAGE = HERE/'examples'/'images'/'cat.png'


def voxels(cells, colour='orange'):
    return [Voxel(x, y, z, colour) for x, y, z in cells]


def cellmap(cells, colour='orange'):
    return {cell: colour for cell in cells}


def quiet(function, *args, **kwargs):
    """Run a CLI without its preview landing in the test output."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        code = function(*args, **kwargs)
    return code, out.getvalue()


@unittest.skipUnless(PILLOW, 'Pillow is not installed')
class ImagingTests(unittest.TestCase):
    def image(self, size=(40, 40), draw=None, background=(255, 255, 255, 255)):
        path = Path(tempfile.mkdtemp())/'test.png'
        canvas = Image.new('RGBA', size, background)
        if draw:
            draw(ImageDraw.Draw(canvas))
        canvas.save(path)
        return path

    def test_a_solid_square_fills_its_cells_with_the_palette_colour(self):
        path = self.image(draw=lambda d: d.rectangle([10, 10, 29, 29], fill=(230, 60, 60, 255)))
        sheet = imaging.to_sheet(path, grid=(4, 4))
        self.assertEqual(sorted(sheet.cells), [(1, 1), (1, 2), (2, 1), (2, 2)])
        self.assertEqual(set(sheet.cells.values()), {'red'})

    def test_an_image_with_no_visible_background_finds_no_subject(self):
        """The background is estimated from the border, so a frame-filling flat
        colour leaves nothing to separate. The CLI says so rather than guessing."""
        path = self.image(draw=lambda d: d.rectangle([0, 0, 39, 39], fill=(230, 60, 60, 255)))
        self.assertEqual(imaging.to_sheet(path, grid=(4, 4)).cells, {})

    def test_colours_snap_to_the_cube_palette(self):
        self.assertEqual(imaging.nearest_color((250, 205, 60)), 'yellow')
        self.assertEqual(imaging.nearest_color((250, 250, 250)), 'white')

    def test_aspect_ratio_is_preserved_so_a_wide_picture_is_not_squashed(self):
        path = self.image(size=(80, 40),
                          draw=lambda d: d.rectangle([0, 10, 79, 29], fill=(60, 130, 235, 255)))
        sheet = imaging.to_sheet(path, grid=(8, 8))
        rows = {y for _, y in sheet.cells}
        # A bar half as tall as a 2:1 image stays 2 cells tall in an 8x8 grid; a
        # squashed downsample would stretch it over more rows.
        self.assertEqual(len(rows), 2, sheet.ascii())
        self.assertEqual({x for x, _ in sheet.cells}, set(range(8)))

    def test_transparency_defines_the_subject_when_present(self):
        path = self.image(background=(0, 0, 0, 0),
                          draw=lambda d: d.rectangle([10, 10, 29, 29], fill=(60, 180, 105, 255)))
        sheet = imaging.to_sheet(path, grid=(4, 4))
        self.assertEqual(sorted(sheet.cells), [(1, 1), (1, 2), (2, 1), (2, 2)])
        self.assertEqual(set(sheet.cells.values()), {'green'})

    def test_mono_forces_one_colour_and_rejects_an_unknown_one(self):
        path = self.image(draw=lambda d: d.rectangle([8, 8, 31, 31], fill=(230, 60, 60, 255)))
        self.assertEqual(set(imaging.to_sheet(path, grid=(4, 4), mono='white').cells.values()),
                         {'white'})
        with self.assertRaises(ValueError):
            imaging.to_sheet(path, grid=(4, 4), mono='chartreuse')

    def test_invert_reads_the_background_as_the_subject(self):
        path = self.image(draw=lambda d: d.rectangle([10, 10, 29, 29], fill=(230, 60, 60, 255)))
        plain = imaging.to_sheet(path, grid=(4, 4))
        inverted = imaging.to_sheet(path, grid=(4, 4), invert=True)
        self.assertTrue(set(plain.cells).isdisjoint(inverted.cells))

    def test_the_sheet_records_which_file_it_came_from(self):
        path = self.image(draw=lambda d: d.rectangle([0, 0, 39, 39], fill=(230, 60, 60, 255)))
        sheet = imaging.to_sheet(path, grid=(2, 2))
        self.assertEqual(sheet.image_sha256, imaging.digest(path))
        self.assertEqual(sheet.provenance()['grid'], [2, 2])

    def test_an_empty_grid_is_rejected(self):
        path = self.image()
        with self.assertRaises(ValueError):
            imaging.to_sheet(path, grid=(0, 4))


class SimplifyTests(unittest.TestCase):
    def test_the_contour_of_a_solid_block_is_its_perimeter(self):
        solid = cellmap([(x, y, 0) for x in range(4) for y in range(4)])
        traced = simplify.contour(solid)
        self.assertEqual(len(traced), 12)                       # 16 cells, 4 interior
        self.assertNotIn((1, 1, 0), traced)
        self.assertNotIn((2, 2, 0), traced)

    def test_a_diagonal_pinch_in_the_walk_is_filled_in_rather_than_left_split(self):
        # (1,1) and (2,2) touch only at a corner, with both shared neighbours empty,
        # so the boundary walk changes cell without crossing a face. Left alone that
        # is a `disconnected` rejection that throws away half the outline.
        pinched = cellmap([(1, 1, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0),
                           (3, 1, 0), (3, 2, 0), (2, 2, 0)])
        traced = simplify.contour(pinched)
        self.assertEqual(sorted(set(traced) - set(pinched)), [(1, 2, 0)])
        self.assertEqual(len(simplify.connected(traced)), len(traced), traced)

    def test_an_inserted_cube_is_reported_as_an_addition(self):
        pinched = cellmap([(1, 1, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0),
                           (3, 1, 0), (3, 2, 0), (2, 2, 0)])
        _, actions = simplify.make_buildable(pinched)
        self.assertEqual([a['action'] for a in actions], ['added'])
        self.assertIn('added 1 cube', simplify.report(actions))

    def test_blobs_touching_only_at_a_corner_keep_the_larger_one(self):
        separate = cellmap([(0, 0, 0), (1, 0, 0), (0, 1, 0), (2, 2, 0)])
        cells, _ = simplify.make_buildable(separate)
        self.assertNotIn((2, 2, 0), cells)
        self.assertTrue(validate(Structure('t', 't', voxels(cells))).ok)

    def test_a_solid_silhouette_becomes_something_the_checker_accepts(self):
        solid = cellmap([(x, y, 0) for x in range(6) for y in range(5)])
        cells, actions = simplify.make_buildable(solid)
        report = validate(Structure('t', 't', voxels(cells)))
        self.assertTrue(report.ok, report.codes())
        self.assertTrue(actions)
        self.assertIn('dropped', simplify.report(actions))

    def test_cells_outside_the_grid_are_dropped(self):
        cells, actions = simplify.make_buildable(cellmap([(0, 0, 0), (1, 0, 0), (9, 0, 0)]),
                                                 grid=(4, 4))
        self.assertNotIn((9, 0, 0), cells)
        self.assertEqual(actions[0]['reason'], 'outside the build grid')

    def test_floating_cubes_are_dropped_rather_than_left_unsupported(self):
        cells, _ = simplify.make_buildable(cellmap([(0, 0, 0), (0, 0, 2)]),
                                           keep_outline=False)
        self.assertEqual(sorted(cells), [(0, 0, 0)])

    def test_only_the_largest_piece_survives(self):
        cells = simplify.connected(cellmap([(0, 0, 0), (1, 0, 0), (2, 0, 0), (9, 9, 0)]))
        self.assertNotIn((9, 9, 0), cells)
        self.assertEqual(len(cells), 3)

    def test_reduction_is_deterministic(self):
        solid = cellmap([(x, y, 0) for x in range(7) for y in range(6)])
        first, _ = simplify.make_buildable(dict(solid))
        second, _ = simplify.make_buildable(dict(solid))
        self.assertEqual(first, second)

    def test_an_empty_input_reduces_to_nothing_without_raising(self):
        self.assertEqual(simplify.make_buildable({}), ({}, []))


class PlacementRetryTests(unittest.TestCase):
    """A cube walled in by higher-ranked junctions must not be reported blocked."""

    # (8,6) has one neighbour on each axis, and both are junctions that the plain
    # ranking places first. This is the pattern that cost the cat two cubes.
    WALLED_IN = [(7, 5, 0), (8, 4, 0), (8, 5, 0), (8, 6, 0), (9, 6, 0), (9, 7, 0), (10, 6, 0)]

    def test_plain_greedy_strands_the_corner(self):
        cells = {v.cell: v for v in voxels(self.WALLED_IN)}
        _, blocked = _greedy(cells, _ranking(cells))
        self.assertIn((8, 6, 0), blocked)

    def test_the_retry_places_every_cube(self):
        steps, blocked = sequence(voxels(self.WALLED_IN))
        self.assertEqual(blocked, [])
        self.assertEqual(len(steps), len(self.WALLED_IN))

    def test_a_genuinely_solid_region_is_still_reported_as_blocked(self):
        _, blocked = sequence(voxels([(x, y, 0) for x in range(3) for y in range(3)]))
        self.assertTrue(blocked)

    def test_support_still_precedes_the_cube_above_it(self):
        steps, blocked = sequence(voxels([(0, 0, z) for z in range(4)]))
        self.assertEqual(blocked, [])
        placed = set()
        for voxel, _ in steps:
            if voxel.z:
                self.assertIn((voxel.x, voxel.y, voxel.z-1), placed)
            placed.add(voxel.cell)


@unittest.skipUnless(PILLOW, 'Pillow is not installed')
class ImagePipelineTests(unittest.TestCase):
    """The CLI paths, end to end, without any model access."""

    def setUp(self):
        self.directory = Path(tempfile.mkdtemp())
        self.output = self.directory/'structure.json'

    def test_trace_produces_a_checked_structure_labelled_as_a_trace(self):
        code, text = quiet(image_to_structure.main,
                           [str(CAT_IMAGE), '--trace', '--grid', '12', '8',
                            '--output', str(self.output)])
        self.assertEqual(code, 0, text)
        structure = load(self.output)
        self.assertEqual(structure.source, 'image_trace')
        self.assertTrue(validate(structure).ok)
        self.assertEqual(structure.provenance['image_sha256'], imaging.digest(CAT_IMAGE))
        self.assertTrue(structure.provenance['simplified'])

    def test_a_reply_is_ingested_under_the_source_the_caller_states(self):
        reply = {'name': 'square', 'reasoning': 'ring',
                 'voxels': [{'x': x, 'y': y, 'z': 0, 'color': 'blue'}
                            for x, y in [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2),
                                         (1, 2), (0, 2), (0, 1)]]}
        path = self.directory/'reply.json'
        path.write_text(json.dumps(reply))
        code, text = quiet(image_to_structure.main,
                           [str(CAT_IMAGE), '--ingest', str(path), '--source', 'a-model-id',
                            '--grid', '12', '8', '--output', str(self.output)])
        self.assertEqual(code, 0, text)
        structure = load(self.output)
        self.assertEqual(structure.source, 'a-model-id')
        self.assertEqual(len(structure.voxels), 8)
        self.assertEqual(structure.provenance['mode'], 'vision_reply')

    def test_ingesting_without_a_source_is_refused(self):
        path = self.directory/'reply.json'
        path.write_text(json.dumps({'voxels': []}))
        with self.assertRaises(SystemExit):
            quiet(image_to_structure.main, [str(CAT_IMAGE), '--ingest', str(path)])

    def test_request_mode_writes_the_rules_and_the_grid_and_calls_nothing(self):
        path = self.directory/'request.json'
        code, _ = quiet(image_to_structure.main,
                        [str(CAT_IMAGE), '--request', str(path), '--grid', '12', '8'])
        self.assertEqual(code, 0)
        request = json.loads(path.read_text())
        self.assertEqual(request['limits']['grid'], [12, 8])
        self.assertIn('Direct support only', request['system'])
        self.assertIn('O', request['downsampled_grid'])
        self.assertIn('voxels', request['response_schema']['properties'])

    def test_provenance_survives_a_json_round_trip(self):
        quiet(image_to_structure.main, [str(CAT_IMAGE), '--trace', '--grid', '12', '8',
                                        '--output', str(self.output)])
        structure = load(self.output)
        self.assertEqual(Structure.from_dict(structure.to_dict()).provenance,
                         structure.provenance)


class CatExampleTests(unittest.TestCase):
    """The cached picture example must keep passing, and keep its provenance."""

    def test_the_cat_is_accepted_and_records_the_image_it_came_from(self):
        cat = load(HERE/'examples'/'cat.json')
        report = validate(cat, {'grid': (12, 8), 'max_layers': 1})
        self.assertTrue(report.ok, report.codes())
        self.assertEqual(report.counts['cubes'], 36)
        self.assertIn('cat.png', cat.provenance['image_path'])
        self.assertNotEqual(cat.source, 'offline_library')

    @unittest.skipUnless(PILLOW, 'Pillow is not installed')
    def test_the_example_image_still_hashes_to_what_the_structure_claims(self):
        cat = load(HERE/'examples'/'cat.json')
        self.assertEqual(cat.provenance['image_sha256'], imaging.digest(CAT_IMAGE))


if __name__ == '__main__':
    unittest.main()
