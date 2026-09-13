# /// script
# dependencies = [
#   "bbos",
#   "numpy<2",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Read-only probe of the left arm. Opens no writers and moves nothing.

Reports where the arm currently is, in both motor turns and URDF radians, so the
simulation's joint convention can be checked against the real robot before any
command is sent.

Run on the robot:  uv run probe_arm.py
"""
import sys
import time

import numpy as np
from bbos import Reader, Config, Type

ARM = sys.argv[1] if len(sys.argv) > 1 else 'arm_left'


def read_state(arm, attempts=4, per_attempt=3.):
    """Read one arm_state sample.

    The first Reader opened in a process never reports ready, however long you
    wait; a freshly constructed one does. So retry with a new Reader rather than
    polling the first forever — polling looks like a dead daemon when the daemon
    is healthy.
    """
    for _ in range(attempts):
        with Reader(f'{arm}.state', Type('arm_state')) as r:
            deadline = time.time() + per_attempt
            while not r.ready() and time.time() < deadline:
                time.sleep(.05)
            if r.ready():
                return {k: np.asarray(r.data[k], dtype=float).copy()
                        for k in ('pos', 'vel', 'torque', 'temp', 'current')}
    return None


def main():
    cfg = Config(ARM)
    names = list(cfg.joint_names)
    print(f'{ARM}: dof={cfg.dof}')
    print(f'joints: {names}')
    print(f'ik_sign: {np.asarray(cfg.ik_sign).tolist()}')
    print(f'gripper_sign: {cfg.gripper_sign}  wheel_radius: {cfg.wheel_radius}')

    state = read_state(ARM)
    if state is None:
        print('no state published; is the daemon running?')
        return 1
    if True:
        turns = np.asarray(state['pos'], dtype=float)
        urdf = cfg.q2urdf(turns.copy())
        print('\n{:<22}{:>12}{:>14}{:>12}'.format('joint', 'turns', 'urdf', 'temp C'))
        temps = np.asarray(state['temp'], dtype=float)
        for i, name in enumerate(names):
            unit = 'm' if i == 0 else 'rad'
            print('{:<22}{:>12.5f}{:>12.4f} {:<3}{:>8.1f}'.format(
                name, turns[i], urdf[i], unit, temps[i]))
        print('\ncurrent (A):', np.round(np.asarray(state['current'], dtype=float), 3).tolist())
        print('torque     :', np.round(np.asarray(state['torque'], dtype=float), 3).tolist())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
