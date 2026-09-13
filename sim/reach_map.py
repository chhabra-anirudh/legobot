#!/usr/bin/env python3
"""Measure which build cells the arm can actually serve, and cache the result.

The arm is on a fixed base, so reach is a hard constraint on where a structure
and its staged cubes can go. At a high table the reachable set is a narrow arc,
not a rectangle, which is why this is measured rather than assumed.

A cell counts as reachable only if IK converges for both grip orientations at
both the grasp height and the travel height. Probing is slow (an unreachable
target burns the full solver budget), so results are cached per table height.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from simulate_assembly import Arm, load_config  # noqa: E402
from tool_frame import GRIP_YAW, rotation_for  # noqa: E402


def cache_path(table_height):
    return HERE/f'reach_{table_height:.3f}.json'


def probe(arm, c, table_height, x_range, y_range, pitch):
    seed = np.zeros(len(arm.names))
    cube = float(c['block_size_m'][2])
    heights = (table_height + cube/2, table_height + cube*2 + c['clearance_m'])
    cells = []
    xs = np.round(np.arange(x_range[0], x_range[1]+1e-9, pitch), 4)
    ys = np.round(np.arange(y_range[0], y_range[1]+1e-9, pitch), 4)
    for x in xs:
        for y in ys:
            if all(_solves(arm, c, x, y, z, axis, seed)
                   for z in heights for axis in GRIP_YAW):
                cells.append([float(x), float(y)])
    return {'table_height_m': table_height, 'pitch_m': pitch,
            'note': 'IK converged for both grip orientations at grasp and travel height',
            'cells': cells}


def _solves(arm, c, x, y, z, axis, seed):
    rotation = rotation_for(GRIP_YAW[axis])
    target = np.array([x, y, z]) - rotation @ c['tool_grasp_point_m']
    try:
        arm.solve(target, rotation, seed)
        return True
    except ValueError:
        return False


def load(table_height):
    path = cache_path(table_height)
    if not path.exists():
        raise FileNotFoundError(
            f'no reach map for a {table_height} m table; run '
            f'`python sim/reach_map.py --table {table_height}` first')
    data = json.loads(path.read_text())
    return {(round(x, 4), round(y, 4)) for x, y in data['cells']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=HERE/'assembly_config.json')
    parser.add_argument('--table', type=float, default=None,
                        help='table height in metres (default: from the config)')
    parser.add_argument('--x', type=float, nargs=2, default=[-.25, .45])
    parser.add_argument('--y', type=float, nargs=2, default=[.15, .50])
    args = parser.parse_args(argv)

    c, arm = load_config(args.config), Arm(HERE/'robot_limits.json')
    table = args.table if args.table is not None else float(c['table_height_m'])
    pitch = float(c['block_size_m'][0])
    data = probe(arm, c, table, args.x, args.y, pitch)
    path = cache_path(table)
    path.write_text(json.dumps(data, indent=1)+'\n')
    cells = [tuple(v) for v in data['cells']]
    print(f'{len(cells)} reachable cells at a {table} m table -> {path.name}')
    if cells:
        xs = [x for x, _ in cells]
        ys = [y for _, y in cells]
        print(f'x {min(xs):.4f}..{max(xs):.4f}   y {min(ys):.4f}..{max(ys):.4f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
