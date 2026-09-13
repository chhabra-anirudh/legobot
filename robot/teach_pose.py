# /// script
# dependencies = [
#   "bbos",
#   "numpy<2",
# ]
# [tool.uv.sources]
# bbos = { path = "/home/bracketbot/bbos", editable = true }
# ///
"""Record where the arm actually is, so calibration replaces guessed numbers.

Read-only: opens no writer and commands nothing. Hand-guide the arm with torque
disabled (`uv run examples/view_arms.py --control`, leave "Enable torque" OFF),
put the tool where you want it, then run this with a label.

    uv run teach_pose.py table_surface
    uv run teach_pose.py build_origin
    uv run teach_pose.py supply_orange

Appends URDF-unit joint vectors to taught_poses.json. Forward kinematics is left
to the workstation, which has the URDF the simulation uses.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
from bbos import Reader, Config, Type

OUT = Path(__file__).resolve().parent/'taught_poses.json'


def read_state(arm, attempts=4, per_attempt=3.):
    """Read one arm_state sample.

    The first Reader opened in a process never reports ready, however long you
    wait; a freshly constructed one does. Retry with a new Reader rather than
    polling the first forever — polling looks like a dead daemon when the daemon
    is healthy.
    """
    for _ in range(attempts):
        with Reader(f'{arm}.state', Type('arm_state')) as r:
            deadline = time.time() + per_attempt
            while not r.ready() and time.time() < deadline:
                time.sleep(.05)
            if r.ready():
                return np.asarray(r.data['pos'], dtype=float).copy()
    return None


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    label = sys.argv[1]
    arm = sys.argv[2] if len(sys.argv) > 2 else 'arm_left'
    cfg = Config(arm)

    turns = read_state(arm)
    if turns is None:
        print('no state published; is the daemon running?')
        return 1
    urdf = cfg.q2urdf(turns.copy())

    record = {'label': label, 'arm': arm, 'recorded_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
              'joint_names': list(cfg.joint_names),
              'turns': [round(float(v), 6) for v in turns],
              'urdf': [round(float(v), 6) for v in urdf],
              'note': 'lj0 is metres, lj1..lj6 and the gripper are radians'}
    poses = json.loads(OUT.read_text()) if OUT.exists() else []
    poses.append(record)
    OUT.write_text(json.dumps(poses, indent=2)+'\n')

    print(f'recorded {label!r} ({len(poses)} poses in {OUT.name})')
    for name, t, u in zip(cfg.joint_names, turns, urdf):
        unit = 'm' if name.endswith('j0') else 'rad'
        print(f'  {name:<20}{t:>10.5f} turns{u:>10.4f} {unit}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
