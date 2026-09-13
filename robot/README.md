# Robot-side scripts

These run **on the BracketBot**, not on the workstation. Each has a PEP 723 header
pointing `bbos` at `/home/bracketbot/bbos`, so run them with `uv run <script>.py`
from `~/bbapps` — there is no venv to create.

Full setup, hardware facts, and the bbos interface reference are in
[docs/ROBOT_SETUP.md](../docs/ROBOT_SETUP.md). Read that first.

| File | Moves the arm? | What |
| --- | --- | --- |
| `probe_arm.py` | no | Joint state in motor turns and URDF radians |
| `teach_pose.py` | no | Append the current pose to `taught_poses.json` |
| `build_bridge.py` | **yes, with `--execute`** | Stream a planned trajectory to the left arm |
| `diagnostics/*.py` | no | IPC channel/log probes |
| `deploy.sh` | no | Sync scripts to the robot, pull measurements back |

`~/bbapps` on the robot is **not version controlled and is shared with other
teams**. This repo is the source of truth. After any teaching session run
`./robot/deploy.sh pull`; to restore the robot after someone else has been on it,
run `./robot/deploy.sh push`.

`build_bridge.py` is a dry run unless given `--execute`: without it no writer is
opened and nothing moves. It refuses to execute if any control point falls outside
the arm's calibrated range. Pass `--gripper` to drive the jaw; otherwise the
gripper is pinned to its current position with its torque disabled.
