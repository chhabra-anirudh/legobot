# Live handoff — start here

Last updated: 2026-09-12. Maintainer for this handoff: current robotics/simulation
workstream. Individual teammate assignments are not yet confirmed.

## Current milestone

Isolated contact-grasp experiment on `feat/contact-grasp` (Codex robotics workstream). The shared source of
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

The original three-cube replay is kinematics only. Attachment/release is idealized; cubes do not experience
gravity or contact and remain axis-aligned. There are no collision checks,
magnetic dynamics, hardware commands, learned policy, camera integration, or voxel
compiler. Playback time does not enforce physical actuator speed/acceleration.

Cube size (25.4 mm) and finger-gripper hardware were confirmed by the user.
Table positions, grasp point, and 0.35/0.19 rad open/grasp angles are provisional.
The mesh probe now shows a 31.42 mm gap at the provisional 0.19 rad angle;
this does not grip a 25.4 mm cube at the configured depth. See
[GRIPPER_CALIBRATION.md](GRIPPER_CALIBRATION.md). Physical calibration is pending.
The solver starts at an approach pose, not a physical startup/home configuration.

## Next tasks — claim before editing shared files

| Priority | Task and completion criterion | Owner / branch |
| --- | --- | --- |
| 1 | Verify finger contact geometry; document jaw width vs angle and tool grasp point | Mesh probe merged; physical measurements pending / owner unassigned |
| 2 | Isolated contact lift/hold implemented; refine collision proxies and fix release placement | Codex — `feat/contact-grasp`; full-arm integration pending |
| 3 | Freeze structure/placement interfaces; validate supported two-layer examples against fake executor | Unassigned — Compiler |
| 4 | Identify robot API, feedback, camera, fixtures, actual access time; measure grasp repeatability | Unassigned — Hardware/integration |
| 5 | Diagnostic top-down expert implemented; add failure-aware transitions before demonstration collection | Codex — `feat/contact-grasp` |
| 6 | Record successful expert episodes with observation/action contract and held-out split | Unassigned — ML; depends on task 5 |
| 7 | Behavior-cloning baseline and held-out evaluation against scripted expert | Unassigned — ML; depends on task 6 |

**Next robotics action:** add a one-cube contact scene using the documented
geometric candidate and validate finger collision proxies, table clearance, and
a held-lift success signal. Carry out the physical calibration procedure when
robot access is available. Assume no robot access until confirmed: the scripted
simulation expert is the default demonstration source. Do not train on the
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

## Latest verification and sync

The user manually pushed bootstrap commits; `git fetch origin` succeeded before
this work and local `main` matched `origin/main`. The old sync blocker is resolved.

This milestone adds triangle-clipped finger-envelope measurements, a JSON report,
a width-matching geometric candidate, and a physical calibration procedure.
All 11 tests pass (5 existing + 6 geometry tests), and the unchanged assembly
still validates 396 frames. Commands were run with the existing parent Python
3.13/Rerun environment. No physical grasp is claimed and default grasp settings
were deliberately left provisional pending contact validation.

The candidate is 0.153774 rad with a +1.101 mm local Y centre correction. At the
configured 0.19 rad angle the cube has clearance on both sides. Do not generate
successful physical-grasp labels from the existing idealized attachment.

## Branch integration review

Reviewed and integrated `feat/gripper-calibration` (3667218) and
`docs/offline-grasp-primitive` (ef55393). Their overlapping handoff edits were
combined. At merge time the offline primitive was a plan; the current contact branch implements
an isolated diagnostic version (see latest checkpoint below).
No changes were made to the assembly configuration during this merge.

The proposed `grasp_depth_from_top = 1/3` targets a contact reference 4.23 mm
above cube centre. A reference height is not a measured pad extent. Raising the
tool may increase nominal table clearance but requires a new depth-dependent
mesh/contact check; do not claim it guarantees clearance or a stable grasp.
For a downward tool and fixed local contact reference, raising the tool by `h/6`
moves the cube centre by **+h/6 in tool-local Z**, not minus. The existing
`tool_grasp_point_m` means cube centre, so do not silently reinterpret that field
as a pad/contact reference when implementing this feature.

Next integration milestone: a physically evaluated one-cube contact scene plus
the configurable top-down expert. Keep hardware calibration and learned-policy
training explicitly pending. Source branches are retained for team traceability.

Merge verification: all 11 tests passed; the default assembly validated 396 frames;
headless recording completed (54 links, 50 meshes); a freshly generated geometry
report matched the checked-in JSON exactly. No hardware or ML validation ran.

## Latest checkpoint: contact physics

See [CONTACT_SIM.md](CONTACT_SIM.md), [review findings](REVIEW_2026-09-12.md), and
[development results](contact_baseline_results.json). The isolated URDF gripper
now lifts a free cube and maintains two force-bearing contacts throughout a
one-second hold. No attachment acts on the cube. Mesh-hull placement still fails
the 3 mm final error gate; the simpler box-pad control passes. Open-finger and
zero-friction controls fail hold as expected. Do not claim full pick/place readiness.

All 18 tests pass in the new repository-local environment, installed with
`requirements-physics.txt`; both physics geometry modes can save Rerun recordings.
The original kinematic plan still validates 396 frames. Physics traces go to
ignored `outputs/`, not source control. Branch: `feat/contact-grasp`.

**Next action overrides older sequencing above:** validate/refine finger contact
proxies and fix lowering/release drift, then add failure-aware expert transitions
and a frozen demonstration contract. No learned model has been trained.
