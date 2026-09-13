# /// script
# dependencies = [
#   "bbos",
#   "numpy<2",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Stream a simulated build trajectory to the left arm. Dry run unless told otherwise.

Converts the simulation's URDF joint vector to motor turns, resamples to the
daemon's control period, and checks every point against the arm's own calibrated
range before anything is sent. Without --execute it opens no writer and moves
nothing; it just reports what it would do and what would be rejected.

    uv run build_bridge.py dog-trajectory.json                 # dry run
    uv run build_bridge.py dog-trajectory.json --execute       # moves the arm

The simulation's URDF limits are wider than the robot's calibrated ranges on
several joints, so a trajectory that is valid in simulation can still be out of
range here. That is what the dry run is for.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import bbos
from bbos import Config, Reader, Type

ARM = 'arm_left'
CONTROL_PERIOD_S = .015


def read_state(arm, attempts=4, per_attempt=3.):
    """One arm_state sample. The first Reader in a process never reports ready."""
    for _ in range(attempts):
        with Reader(f'{arm}.state', Type('arm_state')) as r:
            deadline = time.time() + per_attempt
            while not r.ready() and time.time() < deadline:
                time.sleep(.05)
            if r.ready():
                return np.asarray(r.data['pos'], dtype=float).copy()
    return None


def joint_range(dof):
    """(lo, hi) per joint in motor turns. cal_min/cal_max are named extremes, not
    ordered, so take min/max per joint. The gripper is deliberately unlimited."""
    path = Path(bbos.__file__).parent/'daemons'/ARM/'ranges.calibration.json'
    if not path.exists():
        return None, None
    cal = json.loads(path.read_text())
    lo = np.minimum(cal['cal_min'], cal['cal_max']).astype(float)
    hi = np.maximum(cal['cal_min'], cal['cal_max']).astype(float)
    lo[dof-1], hi[dof-1] = -np.inf, np.inf
    return lo, hi


def resample(points, dt_in, dt_out):
    """Linear resample of a joint trajectory onto the control period."""
    points = np.asarray(points, dtype=float)
    t_in = np.arange(len(points))*dt_in
    t_out = np.arange(0, t_in[-1]+1e-9, dt_out)
    return np.stack([np.interp(t_out, t_in, points[:, j])
                     for j in range(points.shape[1])], axis=1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('trajectory')
    parser.add_argument('--execute', action='store_true',
                        help='actually drive the arm (otherwise dry run)')
    parser.add_argument('--speed', type=float, default=.25,
                        help='time scale; 0.25 runs the build at quarter speed')
    parser.add_argument('--gripper', action='store_true',
                        help='drive the gripper too (default: hold it where it is)')
    args = parser.parse_args(argv)

    traj = json.loads(Path(args.trajectory).read_text())
    cfg = Config(ARM)
    dof = int(cfg.dof)
    urdf = np.asarray(traj['points'], dtype=float)
    if urdf.shape[1] != dof:
        raise SystemExit(f'trajectory has {urdf.shape[1]} joints, arm has {dof}')

    dt_out = CONTROL_PERIOD_S*max(args.speed, 1e-3)
    resampled = resample(urdf, float(traj['frame_seconds']), dt_out)
    turns = np.stack([cfg.urdf2q(p.copy()) for p in resampled])

    state = read_state(ARM)
    if state is None:
        raise SystemExit('no arm state; is the daemon running?')

    lo, hi = joint_range(dof)
    print(f'{args.trajectory}: {len(urdf)} sim points -> {len(turns)} control points '
          f'at {CONTROL_PERIOD_S*1000:.0f} ms, {len(turns)*CONTROL_PERIOD_S:.1f}s of motion '
          f'(speed {args.speed})')
    if traj.get('cold_restart_waypoints'):
        print(f'NOTE: {len(traj["cold_restart_waypoints"])} cold IK restarts in this path; '
              f'the joint path may jump there.')

    names = list(cfg.joint_names)
    print(f"\n{'joint':<20}{'min':>10}{'max':>10}{'range lo':>10}{'range hi':>10}"
          f"{'over':>8}{'start gap':>11}")
    violations = 0
    for j, name in enumerate(names):
        col = turns[:, j]
        over = 0 if lo is None else int(np.sum((col < lo[j]) | (col > hi[j])))
        violations += over
        rlo = '-inf' if lo is None or not np.isfinite(lo[j]) else f'{lo[j]:.4f}'
        rhi = 'inf' if hi is None or not np.isfinite(hi[j]) else f'{hi[j]:.4f}'
        print(f'{name:<20}{col.min():>10.4f}{col.max():>10.4f}{rlo:>10}{rhi:>10}'
              f'{over:>8}{col[0]-state[j]:>11.4f}')

    step = np.max(np.abs(np.diff(turns, axis=0)), axis=0)
    print(f'\nlargest per-tick step (turns): {np.round(step, 4).tolist()}')
    gap = float(np.max(np.abs(turns[0]-state)))
    print(f'largest gap from the arm\'s current pose to the trajectory start: {gap:.4f} turns')

    if violations:
        print(f'\n{violations} control points fall outside the arm\'s calibrated range. '
              f'The daemon would clamp these; the build would not follow the plan. '
              f'Refusing to execute.')
        return 1
    if not args.execute:
        print('\nDry run only. Nothing was sent and no writer was opened.')
        print('Re-run with --execute to drive the arm, with the workspace clear '
              'and a hand on the e-stop.')
        return 0

    # --- execute -----------------------------------------------------------
    from bbos import Writer

    if not args.gripper:
        # Our open/grasp angles are not validated against the real jaw, so hold
        # the gripper wherever it already is rather than driving it blind.
        turns[:, dof-1] = state[dof-1]

    ramp_s = max(2., gap/.05)          # <= 0.05 turns per second on the worst joint
    ramp_n = int(ramp_s/CONTROL_PERIOD_S)
    print(f'\napproach ramp: {ramp_s:.1f}s to close a {gap:.4f} turn gap, '
          f'then {len(turns)*CONTROL_PERIOD_S:.1f}s of build')
    print('enabling torque in 3s - hand on the e-stop')
    time.sleep(3)

    # A Writer is paced by bbos itself: `buf()` ends in Loop.keeptime(), which
    # sleeps out the type's 15 ms period. That clock is per process and shared,
    # and it only sleeps once per registered keeptime writer, so a second paced
    # writer we do not touch every tick would halve the rate. Torque is written
    # once, so open it unpaced and let ctrl alone drive the loop. For the same
    # reason there is no sleep in the send loop; adding one would slow the build
    # rather than speed-limit it.
    with Writer(f'{ARM}.torque', Type('arm_torque'), keeptime=False) as w_torque, \
            Writer(f'{ARM}.ctrl', Type('arm_ctrl')) as w_ctrl:
        enable = np.ones(dof, dtype=np.bool_)
        if not args.gripper:
            enable[dof-1] = False
        with w_torque.buf() as b:
            b['enable'][:] = enable
            b['tau_mode'][:] = np.zeros(dof, dtype=np.bool_)
            b['compliance_mode'] = False

        def send(q):
            with w_ctrl.buf() as b:
                b['pos'][:] = q.astype(np.float32)
                b['vel'][:] = np.zeros(dof, np.float32)
                b['tau'][:] = np.zeros(dof, np.float32)
                b['alpha'] = 1.

        try:
            for i in range(ramp_n+1):
                send(state + (turns[0]-state)*(i/ramp_n))
            print('at trajectory start; running the build')
            for i, q in enumerate(turns):
                send(q)
                if i % 500 == 0:
                    print(f'  {i}/{len(turns)} ({i*CONTROL_PERIOD_S:.0f}s)', flush=True)
            print('build complete. The daemon cuts torque as soon as this writer '
                  'closes, so the arm goes limp on exit - it is not holding anything.')
        except KeyboardInterrupt:
            print('\ninterrupted - releasing the writer, so the arm goes limp')
            return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
