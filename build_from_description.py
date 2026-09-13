#!/usr/bin/env python3
"""Describe something; watch the arm build it with coloured cubes in simulation.

    description -> LLM -> voxel structure -> deterministic checks -> simulated build

One command for the whole demo. Each stage is printed, including which model or
agent produced the design and anything that had to change to make it buildable.

Three ways to get the design, and the structure file always records which was used:

    # a vision/text model, called by this script (needs ANTHROPIC_API_KEY)
    python build_from_description.py "a cat"

    # no model access: hand-authored library shapes, labelled offline_library
    python build_from_description.py "a simple dog" --offline

    # something else answers the request: another model, or an agent in a session
    python build_from_description.py "a cat" --request outputs/ask.json
    python build_from_description.py "a cat" --ingest outputs/reply.json --source "<what answered>"

What this is: a compiler check plus a kinematic simulation. Grasp attachment in the
simulated build is idealized, so a build that finishes here is a build that
satisfies the stated rules — not evidence that the real arm can place 40 cubes.
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for directory in (HERE/'compiler', HERE/'sim'):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

import build_structure                                          # noqa: E402
import request as design_request                                # noqa: E402
import simplify                                                 # noqa: E402
from generate import MODEL, offline, propose                     # noqa: E402
from preview import render                                      # noqa: E402
from schema import DEFAULT_LIMITS, Voxel, save, validate       # noqa: E402

# One-inch cube defaults. Actual build/staging feasibility is checked against
# the current reach map and chassis exclusion for each requested structure.
DEFAULT_GRID = (12, 9)
DEFAULT_MAX_CUBES = 40


def slug(text):
    keep = [c if c.isalnum() else '-' for c in text.strip().lower()]
    return ''.join(keep).strip('-')[:32] or 'structure'


def design(args, limits, structure_id):
    """Get a structure and the reduction it needed. Never invents a source."""
    if args.ingest:
        reply = design_request.read(args.ingest)
        return design_request.structure(reply, structure_id, args.source,
                                        prompt=args.description, grid=limits.get('grid'),
                                        provenance={'mode': 'reply', 'grid': list(limits['grid'])},
                                        reduce=not args.no_simplify)
    if args.offline:
        return offline(args.description, structure_id), []
    structure, _, transcript = propose(args.description, limits, args.attempts,
                                       args.model, structure_id=structure_id, verbose=True)
    structure.provenance = {'mode': 'api', 'grid': list(limits['grid']),
                            'attempts': transcript}
    if validate(structure, limits).ok or args.no_simplify:
        return structure, []
    # The model already saw the checker's problems and still failed; reduce
    # deterministically rather than throwing the attempt away.
    cells, actions = simplify.make_buildable({v.cell: v.color for v in structure.voxels},
                                             grid=limits.get('grid'))
    structure.voxels = [Voxel(*cell, colour) for cell, colour in sorted(cells.items())]
    structure.provenance['simplified'] = simplify.summary(actions)
    return structure, actions


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog='Stages: describe -> design -> check -> build. See docs/HANDOFF.md.')
    parser.add_argument('description', help='what to build, e.g. "a cat"')
    parser.add_argument('--grid', type=int, nargs=2, default=list(DEFAULT_GRID),
                        metavar=('W', 'H'), help='build area in cells (default: 12 9, '
                                                 'the measured fit at a 0.5 m table)')
    parser.add_argument('--max-cubes', type=int, default=DEFAULT_MAX_CUBES,
                        help='cube budget (default: 40; 48 is the measured staging limit)')
    parser.add_argument('--max-layers', type=int, default=DEFAULT_LIMITS['max_layers'])
    parser.add_argument('--offline', action='store_true',
                        help='use the hand-authored library instead of a model')
    parser.add_argument('--model', default=MODEL)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--request', metavar='FILE',
                        help='write the design request here and stop, for another model '
                             'or an agent to answer')
    parser.add_argument('--ingest', metavar='FILE', help='read a reply to --request')
    parser.add_argument('--source', default='',
                        help='required with --ingest: what produced the reply')
    parser.add_argument('--no-simplify', action='store_true',
                        help='report a design that fails the checks instead of reducing it')
    parser.add_argument('--structure-out', help='where to write the structure JSON '
                                                '(default: outputs/<slug>.json)')
    parser.add_argument('--origin', type=float, nargs=2,
                        help='root-frame XY of cell (0,0); omit to use the reach map')
    parser.add_argument('--check', action='store_true',
                        help='plan the build but open no viewer and write no recording')
    parser.add_argument('--save', help='write a Rerun recording instead of opening the viewer')
    parser.add_argument('--export-trajectory', help='write the joint trajectory for the robot')
    parser.add_argument('--order', action='store_true', help='print the placement order')
    args = parser.parse_args(argv)

    limits = {**DEFAULT_LIMITS, 'max_cubes': args.max_cubes, 'max_layers': args.max_layers,
              'max_footprint': max(args.grid), 'grid': tuple(args.grid)}
    identifier = slug(args.description)

    print(f'[1/4] description: {args.description!r}')
    print(f'      build area {args.grid[0]}x{args.grid[1]} cells, '
          f'up to {args.max_cubes} cubes, {args.max_layers} layers')

    if args.request:
        design_request.write(args.request, f'Design a voxel structure: {args.description}',
                             limits)
        print(f'[2/4] wrote {args.request}. Answer it, then re-run with\n'
              f'      --ingest <reply.json> --source "<what answered it>"')
        return 0
    if args.ingest and not args.source.strip():
        parser.error('--ingest needs --source: record what produced the reply')

    structure, actions = design(args, limits, identifier)
    print(f'[2/4] design from {structure.source}: {len(structure.voxels)} cubes')
    if actions:
        print('      reduced to satisfy the gripper and support rules:')
        print(simplify.report(actions))

    report = validate(structure, limits)
    print(f'[3/4] deterministic checks: {"passed" if report.ok else "REJECTED"}')
    print()
    print(render(structure, report))
    if not report.ok:
        if 'out_of_grid' in report.codes():
            wide, deep = report.counts['footprint']
            print(f'\nThe design needs a {wide}x{deep} area. Try --grid {wide} {deep}, '
                  f'then check it fits the arm:\n'
                  f'  python sim/build_structure.py <structure> --list-origins',
                  file=sys.stderr)
        print('\nNothing is built from a rejected structure. Rules are in '
              'compiler/README.md.', file=sys.stderr)
        return 1

    path = Path(args.structure_out or HERE/'outputs'/f'{identifier}.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    save(structure, path)
    print(f'\n[4/4] simulated build (structure: {path})')

    build_argv = [str(path)]
    for flag, value in [('--origin', args.origin), ('--save', args.save),
                        ('--export-trajectory', args.export_trajectory)]:
        if value:
            build_argv += [flag] + ([str(v) for v in value] if isinstance(value, list)
                                    else [str(value)])
    if args.check:
        build_argv.append('--check')
    if args.order:
        build_argv.append('--coords')
    return build_structure.main(build_argv)


if __name__ == '__main__':
    raise SystemExit(main())
