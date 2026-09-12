# Live handoff — start here

Updated 2026-09-12. Active work: Codex, robotics/simulation,
branch **`fix/centered-grasp-colors`**, based on main at 8394727. Other individual owners are unassigned.

## Current state

The shared source of truth is this repository. `main` includes the gripper geometry
and offline-plan merges (2646ccf), plus `feat/contact-grasp` through 9ec4dea:
contact physics and its nominal release fix are now integrated into local `main`.

- **Kinematic assembly:** actual URDF, left-arm FK/IK, three 25.4 mm cubes, animated
  mimic fingers, 396 frames. Cube attachment remains idealized in this old replay.
- **Geometry tools:** triangle-clipped jaw-envelope report and physical calibration
  procedure. The old 0.19 rad animation angle does not actually close on the cube
  at its configured depth; keep it labeled an animation setting.
- **Contact experiment:** free cube under gravity, dynamic finger joints with mimic
  coupling, gripper following a mocap target. No constraint attaches the cube.
- **Refined collision assets:** CoACD components with source/version/asset hashes.
  Contact includes sliding and spin friction. Default is `decomposed`; `mesh_hulls`
  and invented `pads` modes remain explicit comparison controls.
- **Offline expert:** diagnostic top-down descend/close/lift/hold/lower/open/retreat,
  with configurable depth. Not yet failure-aware or a frozen training interface.
- **Outputs:** per-episode result, 50 Hz JSONL trace, optional Rerun recording.

## Latest verification and remaining limits

All **26 tests pass** with physics dependencies installed. The original kinematic
plan still validates 396 frames. Refined physics recording generated successfully.

[Refined development checks](contact_refined_results.json): five translated nominal
cases pass full hold/return checks with ~**0.752 mm** final error, below the unchanged
3 mm limit. The 40 g case also passes. Zero spin friction and coarse hull controls
fail placement; friction=0.5 gives ~3.46 mm error and fails. Friction=0 produces
solver warnings and is marked numerically invalid. Open fingers fail lift cleanly.
Full success also checks final tilt/speed and finger-table contact.

These are small development checks with **perfect cube XY**, not reliability,
perception robustness, hardware, or ML evaluation. Friction, inertia, gains, and
contact proxies remain uncalibrated. Finger-to-finger collisions are excluded.
There is no full-arm contact/dynamics, magnetic model, robot bridge, vision,
voxel compiler, trained policy, or deployment-quality expert.

Physics uses `experiment_table` (table Z=0), **not** calibrated robot `root`.
Traces explicitly use internal WXYZ quaternions; future public contracts use XYZW.
Warning handling is process-global: run episodes sequentially or in separate
processes. Warnings invalidate success and are preserved in result JSON.

## Reproduce

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-physics.txt
.venv/bin/python -m unittest discover -s sim -p 'test_*.py'
.venv/bin/python sim/simulate_assembly.py --check
.venv/bin/python sim/contact_grasp.py --output outputs/contact --save /tmp/contact.rrd
.venv/bin/python sim/evaluate_contact.py --output outputs/evaluation.json
```

CoACD is only needed for asset regeneration (`requirements-collision.txt`), not
normal execution. See [CONTACT_SIM.md](CONTACT_SIM.md) and
[GRIPPER_CALIBRATION.md](GRIPPER_CALIBRATION.md). Generated runs belong in ignored
`outputs/`; only small evaluation summaries and required collision assets are tracked.

## Next tasks — claim before editing shared files

| Priority | Next concrete outcome | Owner |
| --- | --- | --- |
| 1 | Validate collision surfaces and low-friction behavior; calibrate when robot access exists | Robotics, unassigned |
| 2 | Add failure-aware stage transitions and freeze observation/action timing, units, and frames | Robotics/ML, unassigned |
| 3 | Generate successful contact demonstrations with pose/noise variation; split by episode | ML, depends on 1–2 |
| 4 | Train behavior cloning and evaluate against the scripted expert on held-out scenes | ML, depends on 3 |
| Parallel | Freeze voxel/placement schemas; supported two-layer examples and fake executor | Compiler, unassigned |
| Parallel | Confirm robot API, feedback, access, fixtures, and calibration procedure | Hardware, unassigned |

**Next robotics action:** validate real foam geometry/compliance and heavier-cube
slip, then add failure-aware transitions. Do not train on perfect-state
diagnostic traces unchanged or call the old idealized replay a physical grasp.
Assume no robot access until confirmed; hardware work need not block simulation.

## Collaboration

Follow [CONTRIBUTING.md](../CONTRIBUTING.md) and [BUILD_PLAN.md](BUILD_PLAN.md).
Fetch before integration, use focused feature commits, push working milestones,
and update this handoff before stopping. Source branches are retained. Configured
GPG signing was previously locked, so agent commits have used per-command unsigned
commits without changing user settings. GitHub pushes now work.

Historical baseline failures and review findings are retained in
[REVIEW_2026-09-12.md](REVIEW_2026-09-12.md) and `contact_baseline_results.json`;
this handoff describes the current state.

## Fresh WSL setup and local main integration (2026-09-12)

Merged `origin/feat/contact-grasp` through 9ec4dea without conflicts; source
branches retained. Created checkout-local `.venv` with `uv venv --python 3.13 .venv`
and `uv pip install --python .venv/bin/python -r requirements-physics.txt`.
Python 3.13.14, MuJoCo 3.13.0, Rerun 0.37.2, NumPy 2.5.3.

Initial Windows clone converted URDF line endings to CRLF, triggering the asset
hash guard. Restored exact committed LF bytes; no model, calibration, collision
assets, or acceptance thresholds changed.

Fresh-environment verification (all commands use `.venv/bin/python`):
- `-m unittest discover -s sim -p 'test_*.py'`: all 20 tests pass.
- `sim/simulate_assembly.py --check`: 396 poses validated.
- `sim/contact_grasp.py --output outputs/contact --save outputs/contact.rrd`:
  full success, 0.752 mm placement error, full hold contact, no solver warnings.
- `sim/simulate_assembly.py --save outputs/assembly.rrd`: completed, 54 links/50 meshes.
- `sim/evaluate_contact.py --output outputs/contact-evaluation.json`: five nominal
  translations, pads, and heavier cube pass. Open fingers, zero sliding friction,
  zero spin friction, coarse hulls, and lower friction remain failures. Lower
  friction gives 3.262 mm error and heavier cube 0.834 mm here; numeric values
  differ slightly from the checked-in report but all pass/fail outcomes agree.

Generated recordings/traces/results remain ignored under `outputs/`. Open with
`.venv/bin/rerun outputs/contact.rrd outputs/assembly.rrd`; use `physics` or
`simulation` timeline respectively and press Play. These are scripted simulation
results under provisional physics assumptions, not hardware or ML validation.
Next: failure-aware expert stages and low-friction/contact-proxy validation on a
new feature branch, before collecting training demonstrations.

Remote publication completed after explicit user approval. Fetched and confirmed
no divergent remote changes, then pushed the verified merge and viewer handoff
to `origin/main` successfully on 2026-09-12.
Viewer setup: native WSL Rerun could not create an R32Float render target with
this graphics driver. Browser serving starts successfully with:
`.venv/bin/rerun --serve-web --bind 127.0.0.1 --web-viewer-port 9090 outputs/contact.rrd outputs/assembly.rrd`.
Open `http://127.0.0.1:9090?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy`.
Both recordings loaded into the local server; browser rendering is not verified.

## Latest override: user-confirmed soft fingers and centered colored replay

See [CENTERED_GRASP.md](CENTERED_GRASP.md). The user confirmed foam-and-mesh pads.
Default geometry is now `foam_pads` over the unchanged decomposed finger geometry;
pad dimensions/compliance remain provisional in `sim/foam_pad_config.json`.
The corrected EEF targets half cube height and X=-18 mm relative to cube centre.
Actual force centroids are near opposing face centres; torque limits are unchanged.

Schema v2 adds per-finger normal force, contact centroids, stable cube colors,
and a stricter grip gate (>=0.5 N each and <2 mm maximum relative motion through
lift/hold). Nominal foam runs give ~1.54 N, 1.89 mm motion, ~0.043 mm final error.
Lower friction 0.5 now passes. The 40 g foam case fails slip and stays labeled a
failure; do not substitute the old bare-finger result. All 26 tests pass. Both
updated colored/physics recordings generated. Historical reports above are not
current-default results; use `contact_foam_results.json`.

Run: `.venv/bin/python sim/contact_grasp.py --color blue --show-contacts --save
outputs/foam-centered.rrd`. Three-cube animation uses red/green/blue from config.
New recordings are required to see the changes. `.gitattributes` now preserves
URDF LF bytes for Windows checkouts, addressing the teammate's WSL hash issue.
This branch has not been merged into main. Next: calibrate foam and heavy-load
slip, then failure-aware control and demonstration contracts.
