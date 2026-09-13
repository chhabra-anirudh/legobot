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


def rotate_cell(x, y, quarter_turns):
    """Rotate a voxel column by whole quarter turns about the build origin.

    Whole turns only, so cells stay on the lattice and the checker's clearance
    reasoning still holds. This is a placement choice, not a change to the design:
    a quarter turn swaps which way the picture's own x axis runs on the table, which
    is what lets a wide design stand across the robot's line of sight rather than
    stretching away from it.
    """
    for _ in range(quarter_turns % 4):
        x, y = -y, x
    return x, y


def grip_axis_in_root(axis, quarter_turns):
    """The placement axis named in root coordinates.

    `placement.sequence` decides the finger axis in **voxel** space. An odd quarter
    turn maps voxel x onto root y, so the wrist yaw has to swap with it or the
    fingers close across the faces they were meant to grip.
    """
    if quarter_turns % 2 == 0:
        return axis
    return 'y' if axis == 'x' else 'x'


def cell_centre(cell, c, calibration):
    """Voxel cell to root-frame cube centre, in metres."""
    x, y, z = cell
    pitch = calibration['pitch_m']
    x, y = rotate_cell(x, y, calibration.get('quarter_turns', 0))
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


def candidate_origins(structure, c, calibration, reach, pitch, limit=None, face=False):
    """Lattice origins where the whole footprint lands in reach, best first.

    Ranked, in order:

    0. With `face=True` only: **clear of the chassis nose first**, so the robot is
       looking at what it builds. Off by default, because it is expensive — the arm
       works folded at those placements and the carried cube tips much further
       (76 degrees against 33 for the same structure placed freely).
    1. **In front of the base before behind it.** The robot drives and faces +x,
       so a build at negative x sits behind the machine with the mast between the
       two. Nothing here checks the arm against its own body, which is a second
       reason to prefer the front — a heuristic, not a collision check.
    2. **Closest to the robot**, measured from the base centre to the furthest cube
       of the build, so the whole structure sits as near the machine as the
       workspace allows and the supply gets the outside.
    3. **More margin**: how many cells of reachable padding surround the tightest
       footprint column, so a build survives calibration error.
    4. **Furthest forward**, then lowest x, so the result is deterministic.

    A deep structure may have no front-half origin at all — the front of the
    workspace is shallow — and then it is placed behind, which `--list-origins`
    makes visible.

    Distance is radial on purpose. The reachable set is a **ring around the base**
    once the whole disc is probed rather than one slice of it, so "smallest y" is
    not "nearest the robot" — it is the robot's right-hand side. Callers must pass
    reach already filtered by `reach_map.clear_of_base`, or the closest origins
    will be inside the chassis.
    """
    columns = [rotate_cell(v.x, v.y, calibration.get('quarter_turns', 0))
               for v in structure.voxels]
    xs = sorted({x for x, _ in reach})
    ys = sorted({y for _, y in reach})
    nose_x = float(np.asarray(c['base_footprint_m'], dtype=float)[0][1]
                   + float(c['base_clearance_m']))
    scored = []
    for ox in xs:
        for oy in ys:
            cells = [(round(ox + cx*pitch, 4), round(oy + cy*pitch, 4))
                     for cx, cy in columns]
            if not all(cell in reach for cell in cells):
                continue
            margin = min(_margin(cell, reach, pitch) for cell in cells)
            reach_out = max(np.hypot(x, y) for x, y in cells)
            forward = sum(x for x, _ in cells)/len(cells)
            facing = face and min(x for x, _ in cells) - pitch/2 > nose_x
            scored.append((face and not facing, forward < 0, round(reach_out, 3), -margin,
                           -round(forward, 4), ox, [ox, oy]))
    scored.sort()
    return [origin for *_, origin in scored][:limit]


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

    Slots fill **from the outside of the workspace inwards**, farthest from the
    base first, which is the opposite end from where `candidate_origins` puts the
    build. The supply and the structure then occupy different parts of the table
    instead of interleaving. A large build leaves few outer slots and the supply
    wraps closer in; that is a consequence of measured reach, not a preference.

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
    # On the build's side of the robot first, then outermost. The side test matters
    # because the reachable set is a ring: without it the supply wraps around the
    # machine, and then no rectangular table holds every cube without covering the
    # robot itself. Within a side, outermost first keeps the space next to the robot
    # for the build. Remaining ties break on y then x, so each colour still ends up
    # in a contiguous run rather than scattered.
    bearing = np.arctan2(*reversed(np.mean([cell_centre((v.x, v.y, 0), c, calibration)[:2]
                                            for v in structure.voxels], axis=0)))

    forward = np.array([np.cos(bearing), np.sin(bearing)])

    def order(slot):
        angle = abs((np.arctan2(slot[1], slot[0]) - bearing + np.pi) % (2*np.pi) - np.pi)
        # Furthest along the build's bearing, not merely furthest from the base: that
        # keeps the supply past the chassis instead of level with it, which is what
        # lets one rectangle hold the build, the supply and no robot.
        return (angle > np.pi/2, -round(float(np.dot(slot, forward)), 3),
                -round(np.hypot(*slot), 3), -slot[1], slot[0])

    # Slots must be a whole empty cell apart. Convert physical cube pitch to
    # reach-map samples so spacing remains valid if either lattice changes.
    spacing = min((b-a for a, b in zip(xs, xs[1:]) if b > a), default=pitch)
    step = max(2, int(round(2*pitch/spacing)))
    free = [(x, y) for (x, y) in sorted(reach, key=order)
            if (x, y) not in blocked
            and xs.index(x) % step == 0 and (len(ys)-1-ys.index(y)) % step == 0]

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


def choose_placement(structure, c, calibration, reach, pitch, *, turns=(0, 1, 2, 3), face=False):
    """Choose a reachable rotation/origin with room for all supply cubes.

    Callers must pass the measured reach after chassis exclusion. This shared
    selection keeps learned construction and the command-line planner aligned.
    """
    nose_x = float(np.asarray(c['base_footprint_m'], dtype=float)[0][1])
    candidates = []
    for quarter in turns:
        calibration['quarter_turns'] = quarter
        fits = candidate_origins(structure, c, calibration, reach, pitch,
                                 face=face)
        for origin in fits:
            calibration['origin_xy_m'] = np.array(origin, dtype=float)
            try:
                staging_layout(structure, c, calibration, reach)
            except ValueError:
                continue  # fits the reach but leaves no room to stage cubes
            cubes = [cell_centre(v.cell, c, calibration) for v in structure.voxels]
            facing = min(p[0] for p in cubes) > nose_x
            candidates.append((not facing, quarter, origin, len(fits)))
            break         # best origin at this rotation; try the next rotation
    # Every rotation that can be built, best facing first. A wide design often
    # only stands in front of the robot one way round.
    candidates.sort(key=lambda item: (item[0], item[1]))
    chosen = ((candidates[0][2], candidates[0][1], candidates[0][3])
              if candidates else None)
    if chosen is None:
        raise ValueError(
            f'no placement fits this structure inside the measured reach at a '
            f'{c["table_height_m"]} m table with room to stage '
            f'{len(structure.voxels)} cubes, at any quarter turn. Use a smaller '
            f'structure, or lower the table: reach shrinks sharply near the top '
            f'of the vertical travel.')
    return chosen


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
        axis = grip_axis_in_root(axis, calibration.get('quarter_turns', 0))
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
        axis = grip_axis_in_root(axis, calibration.get('quarter_turns', 0))
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
            'build_quarter_turns': int(calibration.get('quarter_turns', 0)),
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


def carried_tilt_deg(arm, frames):
    """Tool tilt from vertical, in degrees, over the frames that carry a cube.

    Returns (max, count over 10 deg, total carrying frames). Transfers interpolate
    in joint space, which keeps every intermediate pose reachable but does not hold
    the tool vertical, so a carried cube is tipped. Attachment here is idealized and
    cannot drop anything; two foam pads holding by friction are a different question,
    open in `sim/contact_grasp.py`. Reported rather than buried for that reason.
    """
    tilts = []
    for q, _, _, _, _, held, _ in frames:
        if held is None:
            continue
        _, R = arm.fk(q)
        tilts.append(float(np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1)))))
    if not tilts:
        return 0., 0, 0
    return max(tilts), sum(1 for t in tilts if t > 10), len(tilts)


def table_rectangle(c, points, margin_m=.02):
    """One rectangular table, turned to face the work and clear of the robot.

    Returns `(centre, half_extent, yaw)`, all in the root frame, with `yaw` the
    rotation about z that the rectangle is drawn at.

    A table is a rectangle, so this stays a rectangle. What it does not do is stay
    axis-aligned: the arm serves a region off to one side of the chassis, so an
    axis-aligned box over that region also covers the robot, and the machine ends up
    standing in its own table. Turning the rectangle to the bearing of the work and
    pushing its near edge past the chassis corners gives the arrangement a person
    would actually set up — robot at the near edge, table extending away from it.

    The yaw is a property of where the arm reaches, not a viewing choice. This mount
    reaches to the robot's left-front, so the table sits at an angle to the nose.
    """
    pad = float(c['block_size_m'][0])/2 + margin_m
    xy = np.array([[float(p[0]), float(p[1])] for p in points])
    (bx0, bx1), (by0, by1) = np.asarray(c['base_footprint_m'], dtype=float)
    keep = float(c['base_clearance_m'])
    chassis = np.array([[x, y] for x in (bx0-keep, bx1+keep) for y in (by0-keep, by1+keep)])

    def rectangle(yaw):
        """Smallest rectangle at this yaw holding every cube, and its chassis overlap."""
        rot = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        local, base = xy @ rot.T, chassis @ rot.T
        lo, hi = local.min(axis=0)-pad, local.max(axis=0)+pad
        overlap = np.all((base > lo) & (base < hi), axis=1).sum()
        return lo, hi, overlap, np.prod(hi-lo)

    # Turn the rectangle until it holds every cube without covering the robot. The
    # arm serves a region off to one side, so the axis-aligned rectangle over that
    # region also covers the chassis and the machine stands in its own table; some
    # angle usually clears it. Ties go to the smaller table.
    best = None
    for degrees in range(0, 180):
        lo, hi, overlap, area = rectangle(np.radians(degrees))
        key = (overlap, round(area, 4), degrees)
        if best is None or key < best[0]:
            best = (key, np.radians(degrees), lo, hi)
    (overlap, _, _), yaw, lo, hi = best
    mid = (lo+hi)/2
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    centre_xy = rot @ mid
    return ([centre_xy[0], centre_xy[1], float(c['table_height_m'])-.015],
            [(hi[0]-lo[0])/2, (hi[1]-lo[1])/2, .015], yaw, int(overlap))


def table_box(c, points, margin_m=.02):
    """Centre and half-extent of the table surface to draw, in the root frame.

    `points` is anything with x and y first: reach-map cells, cube centres, staging
    slots. The box covers them all, padded by a cube half-width plus `margin_m`.
    See `table_boxes` for the version that keeps the robot out of its own table.

    Derived rather than fixed on purpose. The previous hardcoded box spanned
    x 0.08..0.52, which predated the reach measurement (x -0.25..0.36), so builds
    at negative x were animated hovering over nothing.

    This is a visual proxy for a surface whose position in `root` has never been
    surveyed — only its height is set, and by hand. It says "the arm's workspace
    has to be on the table", not "the table is here".
    """
    pad = float(c['block_size_m'][0])/2 + margin_m
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    centre = [(min(xs)+max(xs))/2, (min(ys)+max(ys))/2, float(c['table_height_m'])-.015]
    half = [(max(xs)-min(xs))/2 + pad, (max(ys)-min(ys))/2 + pad, .015]
    return centre, half


def render(structure, c, arm, frames, calibration, save, labels=False, reach=()):
    os.environ['PATH'] = str(Path(sys.executable).parent)+os.pathsep+os.environ.get('PATH', '')
    rr.init('voxel-structure-build', spawn=not bool(save))
    if save:
        rr.save(save)
    commandable = log_robot(str(URDF), dict(zip(arm.names, frames[0][0])))
    # Only what is actually on the table defines it: the structure and the staged
    # cubes. Sizing it to the whole reachable ring drew a table the robot stood in.
    on_table = ([cell_centre(v.cell, c, calibration) for v in structure.voxels]
                + list(frames[0][1]))
    centre, half, yaw, overlap = table_rectangle(c, on_table)
    if overlap:
        print(f'Note: no rectangle holds every cube while clearing the chassis; the '
              f'table drawn covers {overlap} of its 4 corners.', file=sys.stderr)
    rr.log('workspace/table', rr.Transform3D(
        translation=centre, mat3x3=np.array([[np.cos(yaw), -np.sin(yaw), 0],
                                             [np.sin(yaw), np.cos(yaw), 0],
                                             [0, 0, 1]])), static=True)
    rr.log('workspace/table/surface', rr.Boxes3D(
        half_sizes=[half], colors=[[85, 100, 115]],
        labels=[f'table, {2*half[0]:.2f} x {2*half[1]:.2f} m at '
                f'{np.degrees(yaw):.0f} deg to the robot']), static=True)
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
    parser.add_argument('--face', action='store_true',
                        help='place the build clear of the chassis nose so the robot looks '
                             'at it. Costs carried tilt: the arm works folded there, and '
                             'the same structure tips 76 deg against 33 placed freely.')
    parser.add_argument('--rotate', type=int, choices=[0, 90, 180, 270], default=None,
                        help='turn the build on the table by whole quarter turns. The '
                             'design is unchanged; this decides which way its own x axis '
                             'runs, so a wide structure can stand across the robot\'s line '
                             'of sight instead of stretching away from it. Omit to try '
                             'every quarter turn and keep the one the robot faces best.')
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
        measured = reach_map.load(float(c['table_height_m']))
        # IK converges at points inside the robot's own chassis; nothing can be
        # built or staged there. Drop them before any placement decision.
        reach = reach_map.clear_of_base(measured, c)
        blocked_by_base = len(measured)-len(reach)
        if blocked_by_base:
            print(f'{len(reach)} of {len(measured)} mapped cells are usable; '
                  f'{blocked_by_base} sit inside the robot base and were dropped.')
        turns = [args.rotate//90] if args.rotate is not None else [0, 1, 2, 3]
        if args.list_origins:
            for quarter in turns:
                calibration['quarter_turns'] = quarter
                fits = candidate_origins(structure, c, calibration, reach, pitch,
                                         face=args.face)
                print(f'{len(fits)} origins fit at {quarter*90} degrees on a '
                      f'{c["table_height_m"]} m table, best facing first:')
                for origin in fits[:10]:
                    print(f'  --rotate {quarter*90} --origin {origin[0]} {origin[1]}')
            return 0
        if args.origin is None:
            # A hardcoded default silently rots whenever the table moves, so derive
            # one from the measured reach instead and say which was used. Each
            # quarter turn is a different set of origins: a wide design may only
            # stand in front of the robot one way round.
            chosen = choose_placement(structure, c, calibration, reach, pitch,
                                      turns=turns, face=args.face)
            origin, quarter, count = chosen
            calibration['origin_xy_m'] = np.array(origin, dtype=float)
            calibration['quarter_turns'] = quarter
            nose = float(np.asarray(c['base_footprint_m'], dtype=float)[0][1])
            cubes = [cell_centre(v.cell, c, calibration) for v in structure.voxels]
            facing = min(p[0] for p in cubes) > nose
            print(f'placement not given; chose --rotate {quarter*90} --origin '
                  f'{origin[0]} {origin[1]} from the reach map ({count} fit at this '
                  f'rotation; --list-origins shows them).')
            print(f'The build sits {"in front of" if facing else "beside or behind"} the '
                  f'robot: x {min(p[0] for p in cubes):+.3f}..{max(p[0] for p in cubes):+.3f}, '
                  f'y {min(p[1] for p in cubes):+.3f}..{max(p[1] for p in cubes):+.3f} m '
                  f'(chassis front at x {nose:+.3f}).')
        else:
            calibration['origin_xy_m'] = np.array(args.origin, dtype=float)
            calibration['quarter_turns'] = turns[0]
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
    tilt, tipped, carrying = carried_tilt_deg(arm, frames)
    print(f'Carried cubes are tipped up to {tilt:.1f} deg from vertical '
          f'({tipped} of {carrying} carrying frames past 10 deg); grasp and release '
          f'stay vertical. Idealized attachment cannot drop a cube — friction might.')
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
        render(structure, c, arm, frames, calibration, args.save, args.labels, reach)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
