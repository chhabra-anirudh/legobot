# Live handoff — start here

Last updated: 2026-09-12. Maintainer for this handoff: current robotics/simulation
workstream. Individual teammate assignments are not yet confirmed.

## Current milestone

Repository bootstrap and finger-gripper kinematic baseline. The shared source of
truth is this repository, not the earlier `battle_of_schools_hackathon` files.
See [BUILD_PLAN.md](BUILD_PLAN.md) for the combined hackathon and ML proposal.

### Implemented

- [x] Supplied BracketBot URDF and meshes; Rerun viewer.
- [x] Left-arm FK and damped-least-squares IK with joint limits.
- [x] Three 25.4 mm cubes and configurable supply/table/stack locations.
- [x] Approach, descend, close, attach, lift, transfer, place, release, open, retreat.
- [x] Both simulated fingers move using the URDF mimic relation.
- [x] Attachment preserves the cube centre at pickup and follows the actual tool.
- [x] Numerical regression tests and headless recording path.
- [x] Merged plan and contributor/agent handoff instructions.

### Known limits

This is kinematics only. Attachment/release is idealized; cubes do not experience
gravity or contact and remain axis-aligned. There are no collision checks,
magnetic dynamics, hardware commands, learned policy, camera integration, or voxel
compiler. Playback time does not enforce physical actuator speed/acceleration.

Cube size (25.4 mm) and finger-gripper hardware were confirmed by the user.
Table positions, grasp point, and 0.35/0.19 rad open/grasp angles are provisional.
The teammate plan's exact jaw-width figures have not been independently validated.
The solver starts at an approach pose, not a physical startup/home configuration.

## Next tasks — claim before editing shared files

| Priority | Task and completion criterion | Owner / branch |
| --- | --- | --- |
| 1 | Verify finger contact geometry; document jaw width vs angle and tool grasp point | Unassigned — Robotics |
| 2 | Minimal contact simulator: cube can be lifted and held under gravity without idealized attachment | Unassigned — Robotics/ML |
| 3 | Freeze structure/placement interfaces; validate supported two-layer examples against fake executor | Unassigned — Compiler |
| 4 | Identify robot API, feedback, camera, fixtures, actual access time; measure grasp repeatability | Unassigned — Hardware/integration |
| 5 | Record successful expert episodes with observation/action contract and held-out split | Unassigned — ML; depends on task 2 |
| 6 | Behavior-cloning baseline and held-out evaluation against scripted expert | Unassigned — ML; depends on task 5 |

**Next robotics action:** inspect the actual finger contact zone and establish a
calibration procedure. Then add a one-cube contact scene. Do not train on the
current artificial attachment and label it a learned physical grasp.

## Reproduction and verification

Use the repository-root setup commands in [README](../README.md).

```sh
.venv/bin/python -m unittest discover -s sim -p 'test_*.py'
.venv/bin/python sim/simulate_assembly.py --check
.venv/bin/python sim/simulate_assembly.py --save /tmp/legobot.rrd
```

Baseline checks: five tests cover joint limits/orientation, final position within
1 mm, attachment/stationary released cubes, finger sequencing, and unreachable
IK rejection. Default sequence generates 396 frames. Headless replay contains
54 links and 50 meshes. These are numerical results, not physical success rates.

The imported `sim/` was checked using the existing Python 3.13 environment with
Rerun 0.37.2: all five tests, `--check`, and headless recording passed after
relocation. A fresh-environment install has not yet been exercised. The viewer
default URDF path was corrected to resolve the included model.

Bootstrap commits are unsigned: the configured GPG database was locked and timed
out. Signing was disabled per command only; user Git settings were not changed.

## Handoff protocol

Before stopping, update this file with the current milestone, changed behavior,
checks actually run, outstanding failures, and the next concrete action. Commit
and push working milestones. For incomplete experiments, identify the branch and
how to reproduce the failure. Do not mark a task complete just because scaffolding
exists. See [CONTRIBUTING](../CONTRIBUTING.md).

## Open questions

- Teammate owners and branches for compiler, robotics/ML, and workstation work.
- Robot control API, control frequency, feedback and grasp verification signals.
- Real jaw calibration, magnetic behavior, table pose, and layered cube pitch.
- Available work hours, robot access, and organizer rules on pre-event work.

## Repository sync blocker

The local bootstrap is committed on `main`, but the push to
`https://github.com/chhabra-anirudh/legobot.git` failed: Git's configured
`credential-manager-core` helper is unavailable and GitHub rejected HTTPS
credentials. GitHub CLI is not installed in this environment. Authenticate this
checkout with an approved GitHub credential helper or SSH setup, then run
`git push -u origin main`. Do not put tokens in this file or in chat. Fetch and
inspect remote history first if teammates have initialized the repository since
this clone. Remove this blocker after confirming a successful push.
