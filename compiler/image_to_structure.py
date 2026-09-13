#!/usr/bin/env python3
"""Picture to a checked voxel structure.

    image -> coarse colour grid -> voxel design -> deterministic checker -> build plan

The grid comes from `imaging.py` and is always deterministic. Turning that grid
into a design has three paths, and each one records honestly which was used:

- `--trace`: no model at all. Outline the silhouette and reduce it with
  `simplify.py` until the buildability rules hold. Source `image_trace`.
- default: send the image **and** the deterministic grid to a vision model, which
  proposes a design; every proposal is checked, and a rejected one is sent back
  with its problems exactly as in `generate.py`. Source is the model id.
- `--request` / `--ingest`: write the request out, let something else answer it,
  and read the answer back in under a `--source` label the caller supplies. This
  exists so a session agent can act as the vision model without pretending the
  pipeline called an API.

A picture is not buildable as a picture: it is a solid silhouette, and the
gripper cannot finish a solid region. Whatever comes out is a simplification, and
the simplification is printed and stored in `provenance`, never hidden.
"""
import argparse
import base64
import mimetypes
import sys
from pathlib import Path

import imaging
import request as vision_request
import simplify
from generate import MODEL, propose
from preview import render
from schema import DEFAULT_LIMITS, Structure, Voxel, build_order, save, validate

GUIDANCE = """You are also given a photograph or drawing, and a deterministic
downsample of that image onto the build grid. In the downsample '.' is background
and each letter is the nearest cube colour to that cell: R red, G green, B blue,
Y yellow, O orange, W white.

The downsample is a solid silhouette, so it is NOT buildable as it stands. Your job
is to read the subject from the image and redraw it as a one-cube-wide outline that
obeys every rule above, keeping the pose, proportions, and colours of the picture
as far as the grid allows. Simplify aggressively: a recognisable outline of the
main subject beats a faithful but unbuildable copy. Ignore the background."""


def cells_to_voxels(cells):
    return [Voxel(x, y, z, colour) for (x, y, z), colour in sorted(cells.items())]


def sheet_cells(sheet, z=0):
    """Sheet cells, keyed by 3D cell so the simplifier and schema can share them."""
    return {(x, y, z): colour for (x, y), colour in sheet.cells.items()}


def image_block(path):
    """Base64 image content block. Anthropic accepts png, jpeg, gif, and webp."""
    media_type = mimetypes.guess_type(str(path))[0]
    if media_type not in {'image/png', 'image/jpeg', 'image/gif', 'image/webp'}:
        raise SystemExit(f'unsupported image type {media_type!r} for {path}; '
                         'use png, jpeg, gif, or webp')
    data = base64.standard_b64encode(Path(path).read_bytes()).decode()
    return {'type': 'image', 'source': {'type': 'base64', 'media_type': media_type,
                                        'data': data}}


def grid_block(sheet):
    return {'type': 'text',
            'text': f'Deterministic downsample of the image onto the '
                    f'{sheet.width}x{sheet.height} build grid '
                    f'(first row is the highest y, +x to the right):\n{sheet.ascii()}'}


def trace(sheet, limits, structure_id, subject=''):
    """Deterministic design: outline the silhouette, then reduce until it passes."""
    cells, actions = simplify.make_buildable(sheet_cells(sheet), grid=limits.get('grid'))
    structure = Structure(structure_id=structure_id,
                          name=subject or f'traced outline of {Path(sheet.image_path).name}',
                          voxels=cells_to_voxels(cells), source='image_trace',
                          prompt=subject,
                          provenance={**sheet.provenance(), 'mode': 'image_trace',
                                      'simplified': simplify.summary(actions)})
    return structure, actions


def from_reply(reply, sheet, limits, structure_id, source, subject='', reduce=True):
    """Turn a vision reply (`name`, `reasoning`, `voxels`) into a checked structure."""
    return vision_request.structure(
        reply, structure_id, source, prompt=subject,
        provenance={**sheet.provenance(), 'mode': 'vision_reply'},
        grid=limits.get('grid'), reduce=reduce)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('image', help='png, jpeg, gif, or webp file')
    parser.add_argument('--subject', default='',
                        help='what the picture shows, if you want to tell the model')
    parser.add_argument('--grid', type=int, nargs=2, default=[10, 8], metavar=('W', 'H'),
                        help='one-inch cube grid (default: 10 8); planner checks actual reach and staging')
    parser.add_argument('--coverage', type=float, default=imaging.DEFAULT_COVERAGE,
                        help='fraction of a cell that must be subject for a cube')
    parser.add_argument('--tolerance', type=float, default=imaging.DEFAULT_TOLERANCE,
                        help='RGB distance from the estimated background colour')
    parser.add_argument('--mono', help='force every cube to one colour')
    parser.add_argument('--invert', action='store_true',
                        help='treat the background as the subject')
    parser.add_argument('--trace', action='store_true',
                        help='deterministic outline only; never calls a model')
    parser.add_argument('--request', metavar='FILE',
                        help='write the vision request here and exit, for an external '
                             'model or agent to answer')
    parser.add_argument('--ingest', metavar='FILE',
                        help='read a reply produced from --request')
    parser.add_argument('--source', default='',
                        help='required with --ingest: what produced the reply, e.g. '
                             'a model id. Never label a hand-drawn design as a model')
    parser.add_argument('--no-simplify', action='store_true',
                        help='do not reduce a design that fails the checks; report it')
    parser.add_argument('--model', default=None, help='vision model id')
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--max-cubes', type=int, default=DEFAULT_LIMITS['max_cubes'])
    parser.add_argument('--max-layers', type=int, default=1,
                        help='pictures are flat silhouettes at z=0 (default: 1)')
    parser.add_argument('--output', help='write the accepted structure JSON here')
    parser.add_argument('--order', action='store_true', help='print the placement order')
    args = parser.parse_args(argv)

    limits = {**DEFAULT_LIMITS, 'max_cubes': args.max_cubes, 'max_layers': args.max_layers,
              'max_footprint': max(args.grid), 'grid': tuple(args.grid)}
    sheet = imaging.to_sheet(args.image, grid=args.grid, coverage=args.coverage,
                             mono=args.mono, tolerance=args.tolerance, invert=args.invert)
    counts = sheet.counts()
    print(f'{args.image}\nsha256 {sheet.image_sha256[:16]}…  '
          f'grid {sheet.width}x{sheet.height}  {counts["cells"]} filled cells')
    print(f'\ndeterministic downsample (solid, not buildable as is):\n{sheet.ascii()}')
    if not sheet.cells:
        print('\nNo subject found. The background estimate may have swallowed the image: '
              'try --tolerance, --coverage, or --invert.', file=sys.stderr)
        return 2

    stem = Path(args.image).stem.lower().replace(' ', '-')[:24]
    if args.request:
        vision_request.write(
            args.request,
            'Design a voxel structure from this picture'
            + (f': {args.subject}' if args.subject else '.'), limits, GUIDANCE,
            extra={'image_path': str(Path(args.image).resolve()),
                   'downsampled_grid': sheet.ascii()})
        print(f'\nwrote {args.request}: answer it, then re-run with '
              f'--ingest <reply.json> --source <what answered it>')
        return 0

    if args.trace:
        structure, actions = trace(sheet, limits, f'{stem}-trace', args.subject)
    elif args.ingest:
        if not args.source.strip():
            parser.error('--ingest needs --source: record what produced the reply')
        structure, actions = from_reply(vision_request.read(args.ingest), sheet,
                                        limits, f'{stem}-vision', args.source.strip(),
                                        args.subject, reduce=not args.no_simplify)
    else:
        structure, _, transcript = propose(
            args.subject or f'the subject of the attached picture ({Path(args.image).name})',
            limits, args.attempts, args.model or MODEL,
            structure_id=f'{stem}-vision', verbose=True,
            attachments=[image_block(args.image), grid_block(sheet)], guidance=GUIDANCE)
        structure.provenance = {**sheet.provenance(), 'mode': 'vision_api',
                                'attempts': transcript}
        actions = []
        if not validate(structure, limits).ok and not args.no_simplify:
            cells = {v.cell: v.color for v in structure.voxels}
            cells, actions = simplify.make_buildable(cells, grid=limits.get('grid'),
                                                     keep_outline=False)
            structure.voxels = cells_to_voxels(cells)
            structure.provenance['simplified'] = simplify.summary(actions)

    if actions:
        print(f'\nsimplified to fit the gripper and support rules '
              f'(source: {structure.source}):\n{simplify.report(actions)}')
    report = validate(structure, limits)
    print()
    print(render(structure, report))
    if args.order and report.ok:
        print('\nplacement order (order only; sim/build_structure.py solves the arm):')
        for i, v in enumerate(build_order(structure), 1):
            print(f'  {i:3d}. ({v.x}, {v.y}, {v.z}) {v.color}')
    if args.output and report.ok:
        save(structure, args.output)
        print(f'\nwrote {args.output}\nnext: .venv/bin/python sim/build_structure.py '
              f'{args.output} --check')
    elif args.output:
        print('\nnot written: the structure was rejected', file=sys.stderr)
    return 0 if report.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
