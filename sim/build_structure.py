#!/usr/bin/env python3
"""Execute a checked voxel structure on the simulated arm.

Turns voxel cells into calibrated root-frame poses, chooses a placement order and
a tool yaw per placement, solves the arm for every motion, and animates the build.

Two things this does not do: it does not simulate contact, and attachment remains
idealized exactly as in `simulate_assembly.py`. What it does add is a conservative
finger-clearance rule, so a structure that cannot be assembled cube by cube is
reported as such instead of being animated as if it worked.
"""
import argparse
import json
from pathlib import Path
import os
import sys

import numpy as np
import rerun as rr

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent/'compiler'))

from cube_colors import rgb  # noqa: E402
from simulate_assembly import URDF, Arm, load_config  # noqa: E402
from visualize_urdf import joint_transform, log_robot  # noqa: E402
from schema import load as load_structure, validate  # noqa: E402
from placement import sequence  # noqa: E402
import reach_map  # noqa: E402

from tool_frame import DOWN, GRIP_YAW, ORIENTATIONS, rotation_for  # noqa: E402,F401


def cell_centre(cell, c, calibration):
    """Voxel cell to root-frame cube centre, in metres."""
    x, y, z = cell
    pitch = calibration['pitch_m']
    return np.array([calibration['origin_xy_m'][0] + x*pitch[0],
                     calibration['origin_xy_m'][1] + y*pitch[1],
                     c['table_height_m'] + (z+.5)*calibration['layer_height_m']])


def footprint_xy(structure, c, calibration):
    """{(x, y) voxel column: root-frame XY of its cube centre}, rounded to the map."""
    return {(v.x, v.y): tuple(np.round(cell_centre((v.x, v.y, 0), c, calibration)[:2], 4))
            for v in structure.voxels}


def unreachable_columns(structure, c, calibration, reach, pitch):
    """Footprint columns whose cube centre is outside the measured reach.

    The reach map is a lattice of probed points, so this compares against the
    nearest sample rather than demanding an exact hit: an origin need not sit on
    the lattice. That makes this a **pre-filter with a useful error message**,
    not the authority — the IK solve in `plan_build` is still what decides.

    The map is probed at cube-centre positions with the tool grasp offset already
    applied, so comparing cube centres here is the right comparison.
    """
    tol = pitch/2
    out = []
    for column, (x, y) in sorted(footprint_xy(structure, c, calibration).items()):
        if not any(abs(rx-x) <= tol and abs(ry-y) <= tol for rx, ry in reach):
            out.append((column, (x, y)))
    return out


def candidate_origins(structure, c, calibration, reach, pitch, limit=None):
    """Lattice origins where the whole footprint lands in reach, best first.

    Ranked by margin: how far the tightest footprint column sits from the edge of
    the reachable set, in cells. A build with room around it survives calibration
    error; one hugging the boundary does not. Ties break on lower y then lower x
    so the result is deterministic.
    """
    columns = [(v.x, v.y) for v in structure.voxels]
    xs = sorted({x for x, _ in reach})
    ys = sorted({y for _, y in reach})
    scored = []
    for ox in xs:
        for oy in ys:
            cells = [(round(ox + cx*pitch, 4), round(oy + cy*pitch, 4))
                     for cx, cy in columns]
            if not all(cell in reach for cell in cells):
                continue
            margin = min(_margin(cell, reach, pitch) for cell in cells)
            scored.append((-margin, oy, ox, [ox, oy]))
    scored.sort()
    return [origin for _, _, _, origin in scored][:limit]


def _margin(cell, reach, pitch, cap=4):
    """Cells of reachable padding around `cell`, capped so ranking stays cheap."""
    x, y = cell
    for ring in range(1, cap+1):
        for dx in range(-ring, ring+1):
            for dy in range(-ring, ring+1):
                if max(abs(dx), abs(dy)) != ring:
                    continue
                if (round(x+dx*pitch, 4), round(y+dy*pitch, 4)) not in reach:
                    return ring-1
    return cap


def staging_layout(structure, c, calibration, reach):
    """Place every cube on the table before the build starts, sorted by color.

    Slots come from the measured reach map, not from a guessed rectangle: on a
    fixed base at a high table the reachable set is an arc. Slots sit on a
    two-cell lattice so each staged cube keeps all four neighbouring cells free
    and can be gripped along either axis, and so a pick never has to break a
    magnetic pair. Each color gets a contiguous run of slots, so the colors end
    up in separate places on the table.

    Returns (positions, colors, picks); picks maps a color to its slot indices.
    """
    pitch = float(calibration['pitch_m'][0])
    z = c['table_height_m'] + c['block_size_m'][2]/2
    footprint = {tuple(np.round(cell_centre((v.x, v.y, 0), c, calibration)[:2], 4))
                 for v in structure.voxels}
    # Keep a one-cell moat around the build so the fingers never meet a staged cube.
    blocked = {(round(x+dx*pitch, 4), round(y+dy*pitch, 4))
               for x, y in footprint for dx in (-1, 0, 1) for dy in (-1, 0, 1)}

    xs = sorted({x for x, _ in reach})
    ys = sorted({y for _, y in reach})
    free = [(x, y) for (x, y) in sorted(reach, key=lambda p: (p[1], p[0]))
            if (x, y) not in blocked
            and xs.index(x) % 2 == 0 and ys.index(y) % 2 == 0]

    needed = {}
    for v in structure.voxels:
        needed[v.color] = needed.get(v.color, 0) + 1
    if sum(needed.values()) > len(free):
        raise ValueError(
            f'{sum(needed.values())} cubes need staging but only {len(free)} isolated '
            f'slots are reachable beside the build. Use a smaller structure, or lower '
            f'the table: reach shrinks sharply near the top of the vertical travel.')

    positions, colors, picks, cursor = [], [], {}, 0
    for color, count in sorted(needed.items(), key=lambda kv: -kv[1]):
        picks[color] = []
        for _ in range(count):
            x, y = free[cursor]
            cursor += 1
            picks[color].append(len(positions))
            positions.append(np.array([x, y, z]))
            colors.append(color)
    return positions, colors, picks


def plan_build(structure, c, arm, calibration, reach):
    """Solve every motion for the whole build. Raises on an unreachable pose."""
    steps, blocked = sequence(structure.voxels)
    if blocked:
        raise ValueError(
            f'{len(blocked)} cubes have no clearance for the fingers at any point in '
            f'this order, starting at {blocked[0]}. With two opposed pads and cubes '
            f'that touch, a cube whose in-layer neighbours are already placed cannot '
            f'be reached. Use a structure that is one cube wide, or change the '
            f'placement strategy.')

    pitch = float(calibration['pitch_m'][0])
    missing = unreachable_columns(structure, c, calibration, reach, pitch)
    if missing:
        cells = ', '.join(f'{col} at ({xy[0]:.4g}, {xy[1]:.4g})' for col, xy in missing[:4])
        more = f' and {len(missing)-4} more' if len(missing) > 4 else ''
        suggestions = candidate_origins(structure, c, calibration, reach, pitch, limit=3)
        advice = ('  Try: ' + '  '.join(f'--origin {o[0]} {o[1]}' for o in suggestions)
                  if suggestions else
                  '  No origin fits this structure in the measured reach. Use a smaller '
                  'structure, or lower the table: reach shrinks sharply near the top of '
                  'the vertical travel.')
        raise ValueError(
            f'{len(missing)} of {len(footprint_xy(structure, c, calibration))} build '
            f'columns fall outside the arm\'s measured reach at origin '
            f'({calibration["origin_xy_m"][0]:.4g}, {calibration["origin_xy_m"][1]:.4g}): '
            f'{cells}{more}.\n{advice}')

    travel_z = (c['table_height_m'] + calibration['layer_height_m']*(structure_height(structure)+1)
                + c['clearance_m'])
    grip = c['gripper_open_rad']
    # Every cube is on the table from the first frame, sorted into per-color rows.
    positions, colors, picks = staging_layout(structure, c, calibration, reach)
    centres = [p.copy() for p in positions]
    names = [f'{color} {i}' for i, color in enumerate(colors)]
    taken = {color: 0 for color in picks}
    frames = []
    cold_starts = []
    attachment = None
    q = None
    current_yaw = GRIP_YAW['y']

    def record(label, held=None):
        p, R = arm.fk(q)
        if held is not None:
            centres[held] = p + R @ attachment
        frames.append((q.copy(), np.array(centres, dtype=float), list(colors), list(names),
                       label, held, grip))

    def move(target, yaw, label, held=None):
        """Interpolate position and wrist yaw together.

        Yaw is interpolated rather than switched, because the solver follows the
        path from the previous pose: a sudden quarter turn leaves it with no
        usable seed and it fails to converge.
        """
        nonlocal q, current_yaw
        start, _ = arm.fk(q)
        start_yaw = current_yaw
        turn = (yaw - start_yaw + np.pi) % (2*np.pi) - np.pi
        n = max(1, int(np.ceil(np.linalg.norm(target-start)/c['cartesian_step_m'])))
        n = max(n, int(np.ceil(abs(turn)/np.radians(5))))
        for f in np.linspace(0, 1, n+1)[1:]:
            current_yaw = start_yaw + turn*f
            waypoint = start+(target-start)*f
            rotation = rotation_for(current_yaw)
            try:
                q = arm.solve(waypoint, rotation, q)
            except ValueError:
                # Warm starting from the previous pose can strand the solver in a
                # branch with no solution even where a cold start converges. Retry
                # cold, and count it: a cold solution may sit in a different branch,
                # so the joint path can jump. Fine to watch in simulation, not fine
                # to stream to hardware without smoothing.
                q = arm.solve(waypoint, rotation, np.zeros(len(arm.names)))
                cold_starts.append(np.round(waypoint, 4).tolist())
            record(label, held)
        current_yaw = yaw


    def move_joint(target, yaw, label, held=None):
        """Travel between two reachable poses by interpolating joint angles.

        At a high table the reachable set is a thin arc, so a straight Cartesian
        line between two reachable points leaves the workspace partway across.
        Interpolating in joint space keeps every intermediate pose feasible by
        construction. The tool traces a curve rather than a line; that is fine
        above the table, but it is not a collision-checked path.
        """
        nonlocal q, current_yaw
        rotation = rotation_for(yaw)
        try:
            goal = arm.solve(target, rotation, q)
        except ValueError:
            goal = arm.solve(target, rotation, np.zeros(len(arm.names)))
            cold_starts.append(np.round(target, 4).tolist())
        steps_n = max(2, int(np.ceil(np.max(np.abs(goal-q))/np.radians(3))))
        start_q = q.copy()
        for f in np.linspace(0, 1, steps_n+1)[1:]:
            q = start_q + (goal-start_q)*f
            record(label, held)
        current_yaw = yaw

    def fingers(target, label, held=None):
        nonlocal grip
        start = grip
        n = max(1, int(np.ceil(c['gripper_motion_seconds']/c['frame_seconds'])))
        for value in np.linspace(start, target, n+1)[1:]:
            grip = float(value)
            record(label, held)

    # Staged rows run along x, so a staged cube always has its y neighbours free.
    pick_yaw = GRIP_YAW['y']
    pick_offset = -rotation_for(pick_yaw) @ c['tool_grasp_point_m']
    q = arm.solve(positions[0] + pick_offset + [0, 0, c['clearance_m']], rotation_for(pick_yaw),
                  np.zeros(len(arm.names)))
    record('Ready: every cube staged on the table, sorted by color')

    for index, (voxel, axis) in enumerate(steps):
        place_yaw = GRIP_YAW[axis]
        place_offset = -rotation_for(place_yaw) @ c['tool_grasp_point_m']
        held = picks[voxel.color][taken[voxel.color]]
        taken[voxel.color] += 1
        source = centres[held].copy()
        destination = cell_centre(voxel.cell, c, calibration)
        step = f'{index+1}/{len(steps)} {voxel.color} to {voxel.cell}'

        move_joint(np.r_[(source+pick_offset)[:2], travel_z], pick_yaw,
                   f'{step}: approach the {voxel.color} row')
        move(source+pick_offset, pick_yaw, f'{step}: descend')
        fingers(c['gripper_grasp_rad'], f'{step}: close fingers')
        p, R = arm.fk(q)
        attachment = R.T @ (centres[held]-p)
        record(f'{step}: attach (idealized, no contact sensing)', held)
        move_joint(np.r_[(source+pick_offset)[:2], travel_z], pick_yaw, f'{step}: lift', held)
        # The placement axis can differ from the pick axis; the wrist turns in transit.
        move_joint(np.r_[(destination+place_offset)[:2], travel_z], place_yaw,
                   f'{step}: transfer (grip along {axis})', held)
        move(destination+place_offset, place_yaw, f'{step}: place', held)
        record(f'{step}: release')
        attachment = None
        fingers(c['gripper_open_rad'], f'{step}: open fingers')
        move_joint(np.r_[(destination+place_offset)[:2], travel_z], place_yaw,
                   f'{step}: retreat')
    return (frames, steps, {color: len(idx) for color, idx in picks.items()}, cold_starts,
            placement_plan(structure, c, calibration, steps, positions, colors, picks))


def placement_plan(structure, c, calibration, steps, staged_positions, staged_colors, picks):
    """Root-frame coordinates for every cube, derived from the build origin.

    Two positions per step and they are not the same: `cube_centre_m` is where the
    cube ends up, `tool_target_m` is what the arm is commanded to. They differ by
    the tool grasp point rotated into the root frame, which is 18 mm for this
    gripper — enough to move a target outside the reachable set near the boundary.
    """
    pitch = [float(v) for v in calibration['pitch_m']]
    travel_z = float(c['table_height_m'] + calibration['layer_height_m']
                     * (structure_height(structure)+1) + c['clearance_m'])
    taken = {color: 0 for color in picks}
    plan = []
    for index, (voxel, axis) in enumerate(steps):
        rotation = rotation_for(GRIP_YAW[axis])
        offset = -rotation @ c['tool_grasp_point_m']
        slot = picks[voxel.color][taken[voxel.color]]
        taken[voxel.color] += 1
        centre = cell_centre(voxel.cell, c, calibration)
        plan.append({
            'step': index+1,
            'voxel': list(voxel.cell),
            'color': voxel.color,
            'grip_axis': axis,
            'tool_yaw_rad': round(float(GRIP_YAW[axis]), 6),
            'pick_from_m': [round(float(v), 5) for v in staged_positions[slot]],
            'cube_centre_m': [round(float(v), 5) for v in centre],
            'tool_target_m': [round(float(v), 5) for v in centre+offset],
        })
    return {
        'frame': 'root',
        'units': 'metres',
        'structure_id': structure.structure_id,
        'calibration': {
            'build_origin_m': [round(float(v), 5) for v in calibration['origin_xy_m']],
            'pitch_m': [round(v, 5) for v in pitch],
            'layer_height_m': round(float(calibration['layer_height_m']), 5),
            'table_height_m': round(float(c['table_height_m']), 5),
            'travel_height_m': round(travel_z, 5),
            'tool_grasp_point_m': [round(float(v), 5) for v in c['tool_grasp_point_m']],
        },
        'staging': {color: [[round(float(v), 5) for v in staged_positions[i]] for i in idx]
                    for color, idx in picks.items()},
        'placements': plan,
        'caveat': 'Derived from the simulation. The root frame is assumed to match the '
                  'robot; table height and origin are not yet measured on hardware.',
    }


def structure_height(structure):
    return max(v.z for v in structure.voxels) + 1


def render(structure, c, arm, frames, calibration, save, labels=False):
    os.environ['PATH'] = str(Path(sys.executable).parent)+os.pathsep+os.environ.get('PATH', '')
    rr.init('voxel-structure-build', spawn=not bool(save))
    if save:
        rr.save(save)
    commandable = log_robot(str(URDF), dict(zip(arm.names, frames[0][0])))
    rr.log('workspace/table', rr.Boxes3D(centers=[[.3, .35, c['table_height_m']-.015]],
           half_sizes=[[.22, .20, .015]], colors=[[85, 100, 115]]), static=True)
    # Footprint of the finished structure, so the target is visible from frame one.
    targets = [cell_centre(v.cell, c, calibration) for v in structure.voxels if v.z == 0]
    rr.log('workspace/footprint', rr.Boxes3D(
        centers=[[t[0], t[1], c['table_height_m']] for t in targets],
        half_sizes=[[c['block_size_m'][0]/2, c['block_size_m'][1]/2, .001]]*len(targets),
        colors=[[80, 220, 150]]*len(targets)), static=True)
    for index, (q, centres, colors, names, label, held, grip) in enumerate(frames):
        rr.set_time('simulation', duration=index*c['frame_seconds'])
        for name, value in zip(arm.names, q):
            joint, path, _ = commandable[name]
            t, R = joint_transform(joint, value)
            rr.log(path, rr.Transform3D(translation=t, mat3x3=R))
        finger, path, followers = commandable['left_left_gripper']
        for joint, entity, value in [(finger, path, grip)] + [
                (j, p, mult*grip+off) for j, p, mult, off in followers]:
            t, R = joint_transform(joint, value)
            rr.log(entity, rr.Transform3D(translation=t, mat3x3=R))
        rr.log('gripper/angle_rad', rr.Scalars(grip))
        box = rr.Boxes3D(centers=centres,
                         half_sizes=np.tile(c['block_size_m']/2, (len(centres), 1)),
                         colors=[rgb(name) for name in colors],
                         labels=([f'{t}{" (held)" if i == held else ""}'
                                  for i, t in enumerate(names)] if labels else None))
        rr.log('workspace/blocks', box)
        rr.log('status', rr.TextLog(label))
    rr.get_global_data_recording().flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('structure', help='checked structure JSON from the compiler')
    parser.add_argument('--config', type=Path, default=HERE/'assembly_config.json')
    parser.add_argument('--origin', type=float, nargs=2, default=None,
                        help='root-frame XY of voxel cell (0,0), metres. Omit to pick '
                             'the best-fitting origin from the measured reach map.')
    parser.add_argument('--list-origins', action='store_true',
                        help='print origins that fit this structure, best first, and exit')
    parser.add_argument('--pitch', type=float, default=None,
                        help='centre-to-centre cube pitch in metres (default: nominal cube size)')
    parser.add_argument('--check', action='store_true', help='plan only, no viewer or recording')
    parser.add_argument('--save', help='write a Rerun recording instead of opening the viewer')
    parser.add_argument('--export', help='write the root-frame placement plan as JSON')
    parser.add_argument('--export-trajectory',
                        help='write the joint trajectory as JSON for the robot bridge')
    parser.add_argument('--coords', action='store_true',
                        help='print the root-frame coordinates derived from the build origin')
    parser.add_argument('--labels', action='store_true',
                        help='draw a color/cell label on every cube')
    args = parser.parse_args(argv)

    c, arm = load_config(args.config), Arm(HERE/'robot_limits.json')
    structure = load_structure(args.structure)
    report = validate(structure)
    if not report.ok:
        print('Structure rejected by the compiler; not executing:', file=sys.stderr)
        for problem in report.problems:
            print(f'  {problem}', file=sys.stderr)
        return 1
    pitch = args.pitch if args.pitch else float(c['block_size_m'][0])
    calibration = {'origin_xy_m': np.zeros(2),
                   'pitch_m': np.array([pitch, pitch]),
                   'layer_height_m': float(c['block_size_m'][2])}

    try:
        reach = reach_map.load(float(c['table_height_m']))
        if args.list_origins:
            fits = candidate_origins(structure, c, calibration, reach, pitch)
            print(f'{len(fits)} origins fit this structure at a '
                  f'{c["table_height_m"]} m table, best margin first:')
            for origin in fits[:20]:
                print(f'  --origin {origin[0]} {origin[1]}')
            return 0 if fits else 2
        if args.origin is None:
            # A hardcoded default silently rots whenever the table moves, so
            # derive one from the measured reach instead and say which was used.
            fits = candidate_origins(structure, c, calibration, reach, pitch)
            if not fits:
                raise ValueError(
                    'no origin places this structure inside the measured reach at a '
                    f'{c["table_height_m"]} m table. Use a smaller structure, or lower '
                    'the table: reach shrinks sharply near the top of the vertical travel.')
            chosen = None
            for origin in fits:
                calibration['origin_xy_m'] = np.array(origin, dtype=float)
                try:
                    staging_layout(structure, c, calibration, reach)
                except ValueError:
                    continue      # fits the reach but leaves no room to stage cubes
                chosen = origin
                break
            if chosen is None:
                raise ValueError(
                    f'{len(fits)} origins place this structure inside the measured reach, '
                    f'but none leaves enough isolated slots beside it to stage '
                    f'{len(structure.voxels)} cubes. Use a smaller structure or a lower table.')
            print(f'origin not given; chose --origin {chosen[0]} {chosen[1]} '
                  f'from the reach map ({len(fits)} fit; --list-origins shows them).')
        else:
            calibration['origin_xy_m'] = np.array(args.origin, dtype=float)
        frames, steps, staged, cold, plan = plan_build(structure, c, arm, calibration, reach)
    except (ValueError, FileNotFoundError) as exc:
        print(f'Cannot execute this structure: {exc}', file=sys.stderr)
        return 2

    print(f'Planned {len(frames)} joint-limited poses for {len(steps)} cubes '
          f'({structure_height(structure)} layers).')
    print('Staged on the table before the build starts: '
          + ', '.join(f'{n} {color}' for color, n in sorted(staged.items())))
    if cold:
        print(f'{len(cold)} waypoints needed a cold IK restart, first at {cold[0]}. '
              f'The joint path may jump there; smooth before sending this to hardware.')
    print('Kinematics only: attachment is idealized and contact, magnetic force, '
          'arm/table collisions, and actuator rates are not simulated.')
    if args.export:
        Path(args.export).write_text(json.dumps(plan, indent=2)+'\n')
        print(f'wrote {args.export}')
    if args.export_trajectory:
        traj = {
            'frame_seconds': float(c['frame_seconds']),
            'joint_names': list(arm.names) + ['left_left_gripper'],
            'units': 'lj0 metres, lj1..lj6 and gripper radians (URDF convention)',
            'structure_id': structure.structure_id,
            'source': 'sim/build_structure.py',
            'caveat': 'Kinematics only. No contact, no collision checking, and the joint '
                      'path may jump wherever a cold IK restart was needed.',
            'cold_restart_waypoints': cold,
            'points': [[round(float(v), 6) for v in f[0]] + [round(float(f[6]), 6)]
                       for f in frames],
        }
        Path(args.export_trajectory).write_text(json.dumps(traj)+'\n')
        print(f'wrote {args.export_trajectory}: {len(traj["points"])} points at '
              f'{traj["frame_seconds"]}s = {len(traj["points"])*traj["frame_seconds"]:.1f}s')
    if args.coords:
        cal = plan['calibration']
        print(f"\nbuild origin {cal['build_origin_m']} m, pitch {cal['pitch_m'][0]} m, "
              f"table {cal['table_height_m']} m, travel {cal['travel_height_m']} m")
        print(f"\n{'#':>3} {'voxel':>10} {'color':<7} {'axis':<5}"
              f"{'cube centre (x,y,z) m':>28}{'tool target (x,y,z) m':>28}")
        for row in plan['placements']:
            print(f"{row['step']:>3} {str(tuple(row['voxel'])):>10} {row['color']:<7} "
                  f"{row['grip_axis']:<5}{str(row['cube_centre_m']):>28}"
                  f"{str(row['tool_target_m']):>28}")
        print('\nstaging (first slot per color):')
        for color, slots in sorted(plan['staging'].items()):
            print(f"  {color:<7} {len(slots):>2} cubes, from {slots[0]} to {slots[-1]}")
    if not args.check:
        render(structure, c, arm, frames, calibration, args.save, args.labels)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
