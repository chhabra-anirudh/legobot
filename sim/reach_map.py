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


def load_full(table_height):
    """The cached map as {'cells': set of (x, y), 'pitch_m': float, ...}.

    The sample spacing matters to callers: the map is a lattice of probed points,
    so deciding whether an arbitrary coordinate is reachable needs a tolerance
    relative to that spacing, not an exact lookup.
    """
    path = cache_path(table_height)
    if not path.exists():
        raise FileNotFoundError(
            f'no reach map for a {table_height} m table; run '
            f'`python sim/reach_map.py --table {table_height}` first')
    data = json.loads(path.read_text())
    data['cells'] = {(round(x, 4), round(y, 4)) for x, y in data['cells']}
    return data


def load(table_height):
    """Just the reachable (x, y) set. See `load_full` for the sample spacing.

    Raw measurement: cells where IK converged. It does **not** exclude the robot's
    own base — see `clear_of_base`, which callers apply before placing anything.
    """
    return load_full(table_height)['cells']


def clear_of_base(cells, c):
    """Drop cells where a cube would intersect the robot's own base.

    IK converging says nothing about whether a point is inside the machine, and it
    is: the arm is mounted on a mast above the chassis, so part of its reachable
    disc lands on the robot itself. At the 0.5 m table this removes 27 of 246
    mapped cells — the three nearest rows — which is exactly where a build placed
    "as near the robot as possible" would otherwise go.

    Kept separate from the cached map on purpose: the map is a measurement of the
    arm, this is a geometric fact about the chassis, and the two are re-measured by
    different means.
    """
    (x_min, x_max), (y_min, y_max) = np.asarray(c['base_footprint_m'], dtype=float)
    pad = float(c['block_size_m'][0])/2 + float(c['base_clearance_m'])
    return {(x, y) for x, y in cells
            if not (x_min-pad < x < x_max+pad and y_min-pad < y < y_max+pad)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', type=Path, default=HERE/'assembly_config.json')
    parser.add_argument('--table', type=float, default=None,
                        help='table height in metres (default: from the config)')
    # Wide enough to contain the whole reachable disc rather than a slice of it.
    # The first 0.5 m map was probed over x [-.25, .45], y [.15, .50] and found 246
    # cells; it was clipped on three sides. The same arm, probed over the window
    # below, has 542. A window that cuts the workspace silently biases every
    # placement decision downstream, so keep this wider than the answer.
    parser.add_argument('--x', type=float, nargs=2, default=[-.35, .40])
    parser.add_argument('--y', type=float, nargs=2, default=[-.20, .45])
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
