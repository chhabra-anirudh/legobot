"""Check a saved structure file and print its preview.

Exit code 0 means the structure satisfies the implemented rules; 1 means it was
rejected and the reasons are printed. Nothing here contacts a model.
"""
import argparse

from schema import DEFAULT_LIMITS, build_order, load, validate
from preview import render


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('path', help='structure JSON file')
    parser.add_argument('--max-cubes', type=int, default=DEFAULT_LIMITS['max_cubes'])
    parser.add_argument('--max-layers', type=int, default=DEFAULT_LIMITS['max_layers'])
    parser.add_argument('--order', action='store_true', help='print the bottom-up build order')
    args = parser.parse_args(argv)

    structure = load(args.path)
    report = validate(structure, {**DEFAULT_LIMITS, 'max_cubes': args.max_cubes,
                                  'max_layers': args.max_layers})
    print(render(structure, report))
    if args.order and report.ok:
        print('\nbuild order (placement sequence only; no tool poses):')
        for i, v in enumerate(build_order(structure), 1):
            print(f'  {i:3d}. ({v.x}, {v.y}, {v.z}) {v.color}')
    return 0 if report.ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
