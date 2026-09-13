# Live handoff — start here

Updated 2026-09-12. Active work: robotics/simulation and first hardware
execution, branch **`fix/assembly-centered-grasp`**. Other individual owners are unassigned.

**Newest section is [First hardware execution](#first-hardware-execution-the-build-trajectory-on-the-real-arm-2026-09-12)** — the arm has now
been driven from the simulated plan. Earlier sections describe older states and
are kept for history; where they disagree, the hardware section is current.

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
- **Prompt to voxel structure:** `compiler/` turns a prompt into a checked voxel
  model using `claude-opus-5` with structured output, then applies deterministic
  schema/support/connectivity/budget checks and a re-checked repair round. Two
  cached examples: an accepted 45-cube dog and a negative control. The executor
  half (build coordinates, tool poses, clearance) is not implemented.

## Latest verification and remaining limits

All **26 sim tests and 22 compiler tests pass** with physics dependencies
installed. The kinematic plan now validates **409** frames: `tool_grasp_point_m`
gained the 18 mm X offset so the assembly grasps where the foam pads actually
meet the cube, instead of putting the nominal tool origin over the cube centre
and contacting near the edge. Frame count changed because the IK targets moved. Refined physics recording generated successfully.

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

# Structure generation (needs the anthropic SDK and API credentials):
.venv/bin/python -m pip install -r requirements-llm.txt
.venv/bin/python compiler/generate.py "a simple dog" --output outputs/dog.json
.venv/bin/python compiler/generate.py "a simple dog" --offline   # no model access
.venv/bin/python compiler/check.py compiler/examples/dog.json --order
.venv/bin/python -m unittest discover -s compiler -p 'test_*.py'
```

CoACD is only needed for asset regeneration (`requirements-collision.txt`), not
normal execution. See [CONTACT_SIM.md](CONTACT_SIM.md) and
[GRIPPER_CALIBRATION.md](GRIPPER_CALIBRATION.md). Generated runs belong in ignored
`outputs/`; only small evaluation summaries and required collision assets are tracked.

## Next tasks — claim before editing shared files

| Priority | Next concrete outcome | Owner |
| --- | --- | --- |
| 0 | **Stage the 16 cubes and run a real build.** Coordinates are in `outputs/dog-plan.json` under `staging`. Runs so far used an empty table, so nothing has verified that a commanded close actually picks a cube up | Hardware/robotics, unassigned |
| 0 | Add a held-cube check. Gripper current is already published in `arm_state`, and the J7 relief loop's holding current is a natural signal: a close that reaches the commanded angle with no current rise means an empty jaw | Robotics, unassigned |
| 0 | Check build cells against the reach map in `build_structure.py`, and make a working `--origin` the default, so an out-of-reach build area reports itself instead of failing as `IK failed at ...` | Robotics, unassigned |
| 1 | Teach a real `table_surface` pose to replace the set-by-hand 0.5 m, then re-run `sim/reach_map.py` and re-export | Hardware, unassigned |
| 1 | Validate collision surfaces and low-friction behavior; calibrate when robot access exists | Robotics, unassigned |
| 2 | Add failure-aware stage transitions and freeze observation/action timing, units, and frames | Robotics/ML, unassigned |
| 3 | Generate successful contact demonstrations with pose/noise variation; split by episode | ML, depends on 1–2 |
| 4 | Train behavior cloning and evaluate against the scripted expert on held-out scenes | ML, depends on 3 |
| Parallel | Voxel schema and checker are implemented in `compiler/`; **next** is the placement side: calibrated build coordinates, tool poses, clearance filtering, and a fake executor for supported two-layer examples | Compiler, unassigned |
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

## Prompt to voxel structure, and the centered assembly grasp (2026-09-12)

Branch `fix/assembly-centered-grasp`, based on `fix/centered-grasp-colors`.

`sim/assembly_config.json` now sets `tool_grasp_point_m = [0.018, 0, 0.004]`, so
the assembly's EEF-to-cube-centre offset is `[-0.018, 0, 0.004]` and matches
`sim/contact_grasp.py`. Before this the assembly placed the nominal tool origin
over the cube centre while the foam pads sit 18 mm away in X, so the fingers met
the cube near its edge. Depth was already correct at half height.

`compiler/` is new: `schema.py` (structure schema and deterministic checks),
`generate.py` (prompt to structure via `claude-opus-5` structured output, plus a
hand-authored offline library), `preview.py` (text preview), `check.py` (check a
saved file). `requirements-llm.txt` adds the `anthropic` SDK; it is not needed for
simulation. See [compiler/README.md](../compiler/README.md).

Support is direct support only, so overhangs are rejected. A standing figure with
legs is therefore not buildable, and recognisable subjects come out as flat
silhouettes at `z=0`. The dog example was generated live and accepted on the first
attempt (45 cubes, one layer). Structures record the `source` that produced them;
hand-authored shapes are labelled `offline_library` and are never presented as
model output.

Verified: 22 compiler tests, 26 sim tests, `simulate_assembly.py --check` at 409
poses, and both Rerun recordings regenerated. Structure checking is a rule check
under provisional limits, not evidence that a build will physically succeed. The
default cube/layer/footprint budgets and the per-color inventory are unconfirmed.
Next on this track: calibrated build coordinates and tool poses, finger clearance
filtering, and a fake executor.

## First hardware execution: the build trajectory on the real arm (2026-09-12)

Branch `fix/assembly-centered-grasp`. The simulated build was streamed to the
robot's left arm for the first time. **The arm moved.** Attachment is still
idealized and the arm cannot sense whether it holds a cube, so this is a motion
and gripper-actuation run, not a validated build.

### The pipeline that now runs end to end

```sh
# workstation: plan and export the joint trajectory
.venv/bin/python sim/build_structure.py compiler/examples/dog.json --check \
    --origin 0.15 0.2008 \
    --export-trajectory outputs/dog-trajectory.json --export outputs/dog-plan.json

# copy outputs/dog-trajectory.json to the robot, then in ~/bbapps:
uv run probe_arm.py                                        # read-only
uv run build_bridge.py dog-trajectory.json --gripper       # dry run, no writer
uv run build_bridge.py dog-trajectory.json --gripper --execute
```

`--origin` is **required** and was previously recorded nowhere.
`build_structure.py`'s default `[.22, .30]` does not fit the reachable area and
fails with a bare `IK failed at ...`. The reach map is consulted for staging slots
but **not** for build cells, so an out-of-reach origin surfaces as a confusing
solver error instead of "this origin does not fit". Worth fixing.

Prefer an origin that plans with **no cold IK restarts** — the planner reports
them, and they mean the joint path may jump. `0.15 0.2008` is clean at a 0.5 m table.

### Table lowered to 0.5 m — this is the single biggest win

`table_height_m` was briefly set to 0.75 m for the real table. At that height the
reachable set is a **thin arc: 95 cells, and only 6 valid 8x3 build origins**.
The table was physically lowered to 0.5 m, giving **246 reachable cells and 133
valid origins** — a broad region instead of a sliver. Reach shrinks sharply near
the top of the vertical travel, exactly as `staging_layout` warns.

Re-run `sim/reach_map.py --table <h>` after any table change; results are cached
per height as `sim/reach_<h>.json`. The 0.5 m map is clipped at the probe boundary
(x=-0.25 is fully reachable), so the true reachable set is wider than measured.

### Why the gripper was not picking anything up

Two independent causes, both now fixed:

1. `build_bridge.py` does not drive the gripper unless `--gripper` is passed — it
   pins that column to the current position and disables its torque. That was the
   deliberate safe default for an unvalidated jaw. **Pass `--gripper`.**
2. `gripper_grasp_rad` was **0.19 rad, which is a 31.4 mm inner gap around a
   25.4 mm cube — 6 mm of air.** It could never have gripped, driven or not. This
   was the documented "animation setting, not a grasp" finally biting.

**Hardware measurement (2026-09-12):** with a cube held in the jaw,
`left_left_gripper` reads **0.2301 rad**. That is *wider* than the mesh report's
bare-finger width-matching angle of 0.1538 rad, consistent with the foam pads
taking up roughly 6 mm per side. `gripper_grasp_rad` is now **0.21 rad**,
deliberately a little past the measured hold so the foam compresses.
`gripper_open_rad` stays 0.35 rad (~10 mm clearance per side; ample).

Closing past contact is safe **on this hardware specifically**: the arm daemon runs
a J7 current-relief loop (`j7_relief_enable = True`, `i_hold = 1.5 A`, backoff up
to 0.1 turns) and caps the gripper at `torque_limit[7] = 300/1000`. It settles to
a gentle hold rather than stalling. This supersedes the older
[GRIPPER_CALIBRATION.md](GRIPPER_CALIBRATION.md) warning against closing past
contact, which was written when the hardware controller was unknown.
The gripper's calibrated range is `[-0.379, 0]` turns; URDF rad = -2*pi*turns.
Note the URDF caps this joint at 1.0 rad while the real jaw opens to ~2.38 rad.

### What the robot side actually needed

`robot/build_bridge.py`'s `--execute` path had never been run. Two real bugs, both
caught before torque was enabled, so nothing moved either time:

1. `Writer` has no `.data`/`.write()`. The API is `with w.buf() as b: b[field] = ...`.
2. `Writer(..., keeptime=True)` is the default and **paces the loop itself**:
   `buf()` ends in `Loop.keeptime()`, which sleeps out the type's 15 ms period.
   That clock is per process and shared, and only sleeps once per registered
   keeptime writer. Two paced writers plus our own `time.sleep(15 ms)` would have
   run the build at roughly a third of the intended rate. Torque is now opened
   `keeptime=False` (written once) and `ctrl` alone paces the loop, with no sleep
   in the send loop. Measured rate then matched design: ~7.5 s per 500 ticks.

One writer per topic is allowed. `examples/view_arms.py --control` had been left
running for 1h50m and owned `arm_left.torque`; it was stopped with SIGINT (not
SIGTERM) so its `finally` block ran and left the arm limp. **If the bridge reports
`Writer for arm_left.torque already exists`, find and stop that process — the
daemon is not broken.**

**The daemon has a deadman.** When the ctrl writer closes, `if not r_ctrl.readable`
disables torque immediately and re-sends at 1 Hz. So the arm goes **limp** as soon
as the bridge exits; its "build complete; holding position" message is misleading.
Do not rely on the arm holding anything after the script returns.

### Verified on hardware

- `arm_left` daemon live; `probe_arm.py` reads state in turns and URDF radians.
- First run (0.75 m table, no gripper): 20587 control points, 308.8 s, completed,
  exit 0, **0 points outside the arm's calibrated range**, no cold IK restarts.
- Second run (0.5 m table, `--gripper`): 1699 sim points -> 22641 control points,
  339.6 s, 0 range violations, gripper column sweeping -0.0557..-0.0334 turns.
- A trajectory regenerated on the workstation was byte-identical to the robot's
  copy (sha256 `df11e981...`), so both sides agree on the plan.

### Limits that still stand

- The 0.5 m table height is a **set value, not a surveyed one**. `taught_poses.json`
  still contains no `table_surface` pose; the two early entries FK to z=0.177 m and
  are near-identical to each other, so the arm was not moved between them. Teaching
  a real table pose remains worthwhile.
- Attachment is idealized; there is no held-cube signal. A commanded close is not
  evidence of a grasp.
- The staging layout needs 16 cubes placed at specific coordinates
  (`outputs/dog-plan.json`, `staging`). Runs so far used an empty table.
- Still no contact, collision checking, magnetic model, vision, or learned policy
  in the hardware path.

### Test debt cleared in this session

Two failures predating this work, both stale tests rather than broken code:

- `sim/test_assembly.py` could not plan at all while the table was at 0.75 m: the
  legacy three-cube demo's `supply_xy_m`/`stack_xy_m` fell outside the reachable
  arc. Those two fields are used **only** by `simulate_assembly.py`
  (`build_structure.py` derives staging from the reach map). With the table back at
  0.5 m the original values are valid again and were restored.
- `compiler/test_schema.py` expected the rejection example to fail for exactly
  `['disconnected', 'unsupported']`. The new `no_clearance` check correctly adds a
  third reason: the original 45-cube dog silhouette is a solid blob, and with two
  opposed pads an interior cube whose in-layer neighbours are placed cannot be
  reached. That is why the accepted `dog.json` is now a 16-cube, one-cube-wide
  outline rather than the 45-cube silhouette older notes below describe.

All **26 sim tests and 22 compiler tests pass**.

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
