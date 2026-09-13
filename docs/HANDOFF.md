# Live handoff — start here

Updated 2026-09-12. Active work: robotics/simulation and first hardware
execution. **Everything below is merged and pushed to `main` (8f083e3)** —
start new work from `main`, not from a feature branch. Individual owners are unassigned.

**New here? Read [ROBOT_SETUP.md](ROBOT_SETUP.md) first** — setup, the full run
recipe, hardware calibration numbers, the bbos interface reference, and the traps.
This file is the running log.

**Newest section is [2 inch blocks, the chassis in the workspace, and the 0.78 m table](#2-inch-blocks-the-chassis-in-the-workspace-and-the-078-m-table-2026-09-13)**. For hardware, the current
section is [First hardware execution](#first-hardware-execution-the-build-trajectory-on-the-real-arm-2026-09-12) — the arm has been driven from the simulated
plan. Earlier sections describe older states and are kept for history; where they
disagree, the newer section is current.

Robot-side state is backed up in `robot/` (scripts, `calibration/`, `diagnostics/`).
The robot's `~/bbapps` is shared with other teams and is not version controlled:
run `./robot/deploy.sh pull` after teaching poses, and `./robot/deploy.sh push` to
restore our scripts after someone else has used the machine.

## Current state

The shared source of truth is this repository. `main` includes the gripper geometry
and offline-plan merges (2646ccf), plus `feat/contact-grasp` through 9ec4dea:
contact physics and its nominal release fix are now integrated into local `main`.

- **Blocks are 2 inches across and 1 inch tall** (`block_size_m` `[0.0508, 0.0508,
  0.0254]`), jaw grasp 0.36 rad and open 0.50 rad. The grasp angle is derived, not
  measured on hardware. Older sections below describe the 1 inch cube.
- **Kinematic assembly:** actual URDF, left-arm FK/IK, three cubes, animated
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
  schema/support/connectivity/clearance/budget checks and a re-checked repair
  round. Cached examples: an accepted 16-cube dog, a 36-cube cat from a picture,
  and a negative control.
- **Picture to voxel structure:** `compiler/imaging.py` downsamples an image onto
  the build grid and `compiler/simplify.py` reduces it to buildable line art;
  `compiler/image_to_structure.py` runs the whole path deterministically or via a
  vision model. See [the picture section](#a-picture-to-a-simulated-build-2026-09-13).
- **Executor:** `sim/build_structure.py` turns a checked structure into staged
  cubes, placement order, tool yaws, calibrated coordinates, a joint trajectory,
  and an animation, gated on the measured reach map.
- **One-command demo:** `build_from_description.py` runs description → design →
  checks → simulated build, with `--offline` and `--request`/`--ingest` paths for
  when there is no API key. See [the section below](#one-command-description-to-simulated-build-2026-09-13).

## Latest verification and remaining limits

All **59 sim tests and 52 compiler tests pass** with physics dependencies
installed (Pillow is additionally needed for the image tests; they skip without it). The kinematic plan now validates **409** frames: `tool_grasp_point_m`
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

# From a picture (--trace needs no model access; Pillow only):
.venv/bin/python compiler/image_to_structure.py compiler/examples/images/cat.png \
    --trace --grid 12 8 --output outputs/cat.json
.venv/bin/python sim/build_structure.py compiler/examples/cat.json --check

# The whole demo in one command (add --check to plan without a viewer):
.venv/bin/python build_from_description.py "a simple dog" --offline
.venv/bin/python sim/build_structure.py compiler/examples/smiley.json --save outputs/smiley.rrd
```

CoACD is only needed for asset regeneration (`requirements-collision.txt`), not
normal execution. See [CONTACT_SIM.md](CONTACT_SIM.md) and
[GRIPPER_CALIBRATION.md](GRIPPER_CALIBRATION.md). Generated runs belong in ignored
`outputs/`; only small evaluation summaries and required collision assets are tracked.

## Next tasks — claim before editing shared files

| Priority | Next concrete outcome | Owner |
| --- | --- | --- |
| 0 | **Bring the table down from 0.78 m.** At 0.78 m only a single row of six blocks is buildable: 146 mapped cells against 542 at 0.5 m, measured over a window wide enough to rule out clipping. Nothing else on this list matters at that height | Hardware, unassigned |
| 0 | **Measure the 2 inch grasp angle on hardware.** `gripper_grasp_rad = 0.36` is derived from the mesh report plus the foam allowance measured on the 1 inch cube. Repeat the 0.2301 measurement with a real 2 inch block | Hardware, unassigned |
| 0 | **Stage the blocks and run a real build.** Coordinates are in `outputs/dog-plan.json` under `staging`. Runs so far used an empty table, so nothing has verified that a commanded close actually picks a cube up | Hardware/robotics, unassigned |
| 0 | Add a held-cube check. Gripper current is already published in `arm_state`, and the J7 relief loop's holding current is a natural signal: a close that reaches the commanded angle with no current rise means an empty jaw | Robotics, unassigned |
| 0 | **Run the model paths once with a real key.** `ANTHROPIC_API_KEY` is not set in this checkout, so every design so far is a library shape or an in-session reply. One key, then `build_from_description.py "a cat"` and the picture path, and record how often a first proposal passes the checks | Compiler, unassigned |
| 1 | Teach a real `table_surface` pose to replace the set-by-hand 0.5 m, then re-run `sim/reach_map.py` and re-export | Hardware, unassigned |
| 1 | ~~First two-layer example~~ — **done.** `outputs/face-078.json` is two layers (eyes resting on the mouth) and plans at 0.78 m; `wall-078.json` is a two-layer wall. Neither has run on hardware | Compiler |
| 1 | **Survey the base footprint.** `base_footprint_m` comes from the URDF meshes, not a measurement of the real chassis, and it now decides which cells are usable | Hardware, unassigned |
| 1 | Validate collision surfaces and low-friction behavior; calibrate when robot access exists | Robotics, unassigned |
| 2 | Count the real cubes per colour and pass them as `inventory` — `validate` supports it and nothing uses it, so a 40-cube design can currently call for 26 white cubes nobody owns | Compiler/hardware, unassigned |
| 2 | Add failure-aware stage transitions and freeze observation/action timing, units, and frames | Robotics/ML, unassigned |
| 3 | Generate successful contact demonstrations with pose/noise variation; split by episode | ML, depends on 1–2 |
| 4 | Train behavior cloning and evaluate against the scripted expert on held-out scenes | ML, depends on 3 |
| Parallel | Confirm robot API, feedback, access, fixtures, and calibration procedure | Hardware, unassigned |

**Done since the last handoff** (do not redo): reach-gated build origins with
auto-selection; picture input with deterministic reduction to buildable line art;
the placement retry that stopped dropping placeable cubes; and
`build_from_description.py`, the one-command description-to-simulated-build path.

**Next robotics action:** validate real foam geometry/compliance and heavier-cube
slip, then add failure-aware transitions. Do not train on perfect-state
diagnostic traces unchanged or call the old idealized replay a physical grasp.
Assume no robot access until confirmed; hardware work need not block simulation.

## 2 inch blocks, the chassis in the workspace, and the 0.78 m table (2026-09-13)

Offline work, on `main`. Three things here matter more than the rest: **the reach map
was placing cubes inside the robot**, the **0.78 m table cannot be built on**, and the
blocks are now **2 inches across and 1 inch tall**.

### The reach map included cells inside the robot's own chassis

IK converges happily at points inside the machine, and nothing filtered them out. At
the 0.5 m table **147 of 542 mapped cells sit inside the base**. With the build ranked
"as near the robot as possible" that came out as far as **14 of the 16-cube dog's cubes
placed inside the chassis**. `reach_map.clear_of_base` now drops them, using
`base_footprint_m` in the config — `x +/-0.094, y +/-0.196`, measured off the URDF
meshes below z=1.2 m, plus a cube half-width and `base_clearance_m`.

Kept separate from the cached map on purpose: the map is a measurement of the arm, the
footprint is a fact about the chassis, and they are re-measured by different means.

This also invalidated the old **25.5 degree** carried-tilt figure, which was measured
on one of those invalid layouts. Valid placements measure **22.8 degrees** for the same
dog, and the planner now prints the number every run.

### The probe window was cutting the workspace in three places

The 0.5 m map had 246 cells over `x [-0.25, 0.45], y [0.15, 0.50]`. Re-probed over
`x [-0.35, 0.40], y [-0.20, 0.45]` it has **542**. The workspace is a **ring around the
base**, not a patch to one side, so "smallest y" was never "nearest the robot" — it is
the robot's right-hand side. `candidate_origins` now ranks by **radial distance** from
the base, then margin, then how far forward the build sits. Defaults in
`sim/reach_map.py` are the wide window, so a re-probe reproduces this.

Table height barely affects the footprint: `lj0` is a 1.03 m lift, so 0.4 m and 0.5 m
give the same 542 cells.

### The 0.78 m table cannot be built on

Probed over a window reaching out to 0.60 m in both axes, so this is not clipping:
**146 cells, 140 usable**, and the reachable set spans only `x -0.37..0.36,
y -0.22..0.41` — a thin annulus, because the arm runs out of vertical travel and has to
stretch flat. On the 2 inch lattice that leaves **a single row of six blocks**. The
recorded demo at that height (`outputs/face-078.rrd`) is 8 blocks: a mouth of six with
two eyes resting on it, because two rows in *plan* is impossible there — rows must be
50.8 mm apart and the usable rows that far apart share one or two columns.

**The table has to come down.** At 0.5 m the same probe gives 542 cells and 395 usable.

### 2 inch blocks: what changed

`block_size_m` is `[0.0508, 0.0508, 0.0254]`. Height is unchanged, so no vertical
number moved. Jaw angles are **grasp 0.36 rad, open 0.50 rad**, derived from the mesh
report — the derivation and its caveat are in
[ROBOT_SETUP.md](ROBOT_SETUP.md#jaw-angles-for-the-2-inch-block-and-how-they-were-derived).
The 2 inch grasp angle is derived, **not measured on hardware**.

A cell now covers four times the table area, so:

- the largest solid build area in front of the robot is **10x3 cells** (4x8 the
  alternative), against 18x7 at 25.4 mm. Defaults in `build_from_description.py` are a
  10x3 grid and 20 blocks (24 still stages);
- every example was re-fitted. Library dog **11 blocks**, `rocket.json` **11**,
  `smiley.json` **10**. The previous versions were up to 0.6 m long and fitted nowhere;
- `staging_layout` steps by **cells, not reach-map samples**. It had been staging
  blocks touching each other, since a cell is now two samples wide.

### Other changes in this batch

- **Quarter-turn placement.** `--rotate 0|90|180|270` turns a build on the table; the
  design is untouched, only which way its own x axis runs. Omitted, the planner tries
  all four and keeps the best. The finger axis is swapped with the lattice, or the
  wrist would close across the wrong faces.
- **`--face`** places the build clear of the chassis nose so the robot looks at it.
  **Opt-in**, because the arm works folded there: the same structure tips 76 degrees
  against 33 placed freely. Off by default.
- **The table is drawn from measurement.** It was a hardcoded box at `x 0.08..0.52`,
  predating the reach measurement, so builds hovered over nothing. `table_rectangle`
  now fits one rectangle to the blocks actually on the table and turns it to clear the
  chassis, reporting when no angle can do both.
- **`Structure.pieces`.** Connectivity was a hard rule, which rejected a face with dots
  for eyes. It is a design rule, not a physical one — the arm places one block at a
  time and never needs two to touch — so a design declares how many parts it has.
  Default 1, so a stray floating cube is still an error.
- **Supply placement** prefers slots on the build's side of the robot and furthest
  along its bearing, so the supply does not wrap around the machine and a single
  rectangular table can hold everything.

Verified: **59 sim tests** and **52 compiler tests** pass. Recordings regenerated:
`smiley-050.rrd` (0.5 m, 10 blocks, 944 poses, tilt 30.1 deg) and `face-078.rrd`
(0.78 m, 8 blocks, 1024 poses, tilt 25.7 deg). Still kinematics with idealized
attachment, and nothing here has run on hardware.

## The table, and where the build and the supply sit (2026-09-13)

Found by watching a recording: **the cubes were not on the table.** The drawn
surface was hardcoded at `x 0.08..0.52, y 0.15..0.55`, which predated the reach
measurement. Measured reach is `x -0.25..0.36, y 0.15..0.40`, and origins are now
chosen from that map, so a typical build sat at negative x — animated hovering over
nothing. The box was a leftover, not a survey.

Three changes in `sim/build_structure.py`:

- **`table_box()`** derives the surface from the measured reach plus every cube in
  the scene, padded by a cube half-width. It is still only a **visual proxy**: the
  table's position in `root` has never been surveyed, and only its height is set,
  by hand. It now asserts "the workspace has to be on the table" instead of
  claiming to know where the table is.
- **`candidate_origins()` ranks nearest the robot first**, then by margin, then by
  how well the build centres across the reachable width. Previously margin ranked
  first, which put builds mid-table. The nearest row turns out to be all-or-nothing
  on margin — it is the edge of the *probed window* — so without the centring tie
  break the structure jammed into the corner at the lowest probed x.
- **`staging_layout()` fills from the far side inwards**, so the supply sits at the
  opposite end of the table from the build.

For the 16-cube dog the separation is clean: build at `y 0.150..0.201`, supply at
`y 0.353..0.404`. The reachable band is only ~11 cells deep, so a 9-deep structure
(house, cat) leaves too few far rows for 40 cubes and the supply wraps around the
sides. That is measured reach, not a preference, and lowering the table would buy
depth back.

Recordings regenerated. Pose counts moved slightly because the origins moved:
house 4109, rocket 2674, cat 3740, dog 1688. Verified: **55 sim tests** (52 before)
and 52 compiler tests pass. The four new tests assert the properties that were
silently violated: origins run near to far, the chosen origin is not on the probed
x edge, staging starts at the far row, and every cube plus the whole reachable band
lies inside the drawn table.

## One command: description to simulated build (2026-09-13)

Offline work, on `main`. **This is the demo path**, agreed with the user: the user
describes what they want, a model turns that into voxels, the checks decide whether
it is buildable, and the arm places coloured cubes in simulation.

```sh
.venv/bin/python build_from_description.py "a small house"            # needs a key
.venv/bin/python build_from_description.py "a simple dog" --offline   # no model
.venv/bin/python build_from_description.py "a small house" --request outputs/ask.json
.venv/bin/python build_from_description.py "a small house" \
    --ingest outputs/reply.json --source "<what answered it>" --save outputs/house.rrd
```

`build_from_description.py` (repository root) runs the four stages and prints each
one: description and limits, the design and its source, the deterministic checks
and preview, then `sim/build_structure.py` planning and animating the build. It
exits non-zero without building anything if the structure is rejected, and passes
`--origin`, `--save`, `--export-trajectory`, and `--check` through to the planner.

`compiler/request.py` is new and shared with the picture path: it writes the design
request (rules, limits, response schema) and reads a reply back under a
caller-supplied `--source`. Three ways to get a design, and the structure file
always records which was used — `offline_library`, a model id, or whatever the
caller stated. **No source is ever invented**, and `--ingest` refuses to run
without one.

### Defaults are measured, not chosen

At the 0.5 m table: a fully occupied **12x9** build area still has 19 valid
origins in the reach map (12x8 has 30; 12x12 has none), and up to **48** cubes find
isolated staging slots beside the build. Defaults are a 12x9 grid and a 40-cube
budget, leaving margin for calibration error. Re-run `sim/reach_map.py --table <h>`
after the table moves; `sim/build_structure.py <structure> --list-origins` checks a
specific structure.

A design that overflows the grid is cropped to it by default, reported as
`dropped N: outside the build grid`; with `--no-simplify` it is rejected instead,
and the error names the area the design actually needs.

### Verified

New examples `compiler/examples/house.json` and `rocket.json` (40 and 26 cubes,
3991 and 2687 poses). Taking the house as the detailed case: from the description
"a small house",
answered in-session by `claude-opus-5-medium` (recorded as such in `source` — this
was **not** an API call by the pipeline, and no API has yet been called with
credentials). 40 cubes, one layer, **no simplification needed**, planned as
**3991** joint-limited poses at an origin chosen from the reach map, and the Rerun
recording was generated. The hand-authored dog also builds: 37 cubes, 3799 poses.

**52 sim tests** (41 before) and **52 compiler tests** pass. `sim/test_pipeline.py`
is new and covers the one-command path: a reply is checked, planned, and saved with
its stated source; a malformed reply fails loudly; an unbuildable design is reduced
with the reduction reported; a design that stays rejected builds nothing and leaves
no structure file behind; the exported trajectory matches the planned poses.

Still compiler and kinematic simulation only. Attachment is idealized, so a
finished build here means the design satisfies the stated rules — not that the arm
can place 40 cubes in a row.

## A picture to a simulated build (2026-09-13)

Offline work, on `main`. The pipeline now accepts an **image** as well as a prompt:

```sh
.venv/bin/python -m pip install -r requirements-llm.txt   # anthropic SDK + Pillow

# deterministic, no model in the path at all
.venv/bin/python compiler/image_to_structure.py compiler/examples/images/cat.png \
    --trace --grid 12 8 --output outputs/cat.json
# vision model path (untested against a live API: no credentials in this checkout)
.venv/bin/python compiler/image_to_structure.py photo.png --subject "a cat" --grid 12 8

# build it with coloured cubes in simulation
.venv/bin/python sim/build_structure.py compiler/examples/cat.json --save outputs/cat.rrd
```

### What is new

- `compiler/imaging.py`: image to a coarse colour grid. Aspect ratio preserved,
  image centred, colours snapped to `sim/cube_colors.py`. Subject/background comes
  from alpha when present, otherwise from distance to a background colour estimated
  from the border pixels. Good for drawings and plain backgrounds; a busy
  photograph gives a busy mask, which is why the downsample is always printed and
  why `--tolerance`, `--coverage`, and `--invert` exist.
- `compiler/simplify.py`: the reduction to something placeable. It traces the
  **outer contour** of the silhouette by following the crack between filled and
  empty cells, fills in diagonal pinch points so the loop stays face-connected,
  thins any surviving 2x2 block, drops unsupported and stranded cubes, and repeats
  until the checker accepts it. Every cube dropped or added is printed and stored
  in the structure's new `provenance` field.
- `compiler/image_to_structure.py`: three explicit paths, with honest labels —
  `--trace` (`image_trace`, no model), the API path (the model id), and
  `--request`/`--ingest` with a caller-supplied `--source`, which is how an
  interactive agent can act as the vision model without the pipeline claiming it
  called an API.
- `Structure.provenance` (optional, omitted when empty, so existing files are
  byte-identical): image path, image SHA-256, grid, mode, and the simplifications.

### Finding: the placement search was dropping cubes it could have placed

The first cat lost two cubes to `no_clearance`. Both were placeable. `placement.py`
ranked cubes and never reconsidered, so a corner with one neighbour on each axis
was walled in by two junctions that outranked it. A failed order is now retried
with the stranded cubes forced to the front (`RETRIES = 8`). A bounded DFS was
tried first and rejected: it exhausted 20 000 nodes on this 36-cube case without
finding an order, while the retry finds one immediately. Genuinely solid regions
are still reported as blocked, and that is still evidence of impracticality, not a
proof that no order exists.

### Cached example

`compiler/examples/cat.json`, from `compiler/examples/images/cat.png` — a tracked
1.8 kB flat-colour drawing, made with Pillow for this test rather than a
photograph, so the example does not depend on anyone's camera roll. 36 cubes, one
layer, no simplification needed. Its `source` states
it came from a vision reply produced by an interactive `claude-opus-5-medium` agent
session — **not** from an API call by this pipeline, and not hand-drawn from
nothing: the agent was shown the image. `--trace` on the same picture gives a
17-cube outline, which is what a deterministic pass without any model produces.

Verified: **52 compiler tests** (22 before) and 41 sim tests pass.
`sim/build_structure.py compiler/examples/cat.json` plans **3653** joint-limited
poses for 36 cubes, choosing `--origin -0.1738 0.1754` from the reach map (34 of
the lattice origins fit), and the Rerun recording was regenerated. Staging fits:
36 isolated slots are reachable beside the build at the 0.5 m table.

This is compiler and kinematic simulation work only. Attachment is still
idealized, nothing here has run on hardware, and a picture that passes the checks
is a picture that satisfies the stated rules — not one the robot has built.

## Reach-checked build origins, and the transfer tilt (2026-09-13)

Offline work; the robot was in use by another team. On `main`.

### The planner now gates the build area on the reach map

Previously the reach map was consulted for **staging slots only**. Build cells
went straight to the IK solver, so an origin that did not fit the workspace
surfaced as a bare `IK failed at [0.3724, 0.3328, 0.8408]` with no hint that the
origin was the problem. `sim/build_structure.py` now:

- pre-filters every footprint column against the reach map before any IK, and on
  failure names the offending columns **and suggests origins that fit**;
- makes `--origin` optional. Omitted, it derives an origin from the measured map,
  ranked by margin — how much reachable padding surrounds the tightest column, so
  the build survives calibration error — and prints what it chose. A hardcoded
  default rots every time the table moves, which is exactly what happened at 0.75 m;
- adds `--list-origins`. The 16-cube dog has **137** valid origins at 0.5 m.

The check is a pre-filter matched against the nearest lattice sample, so an origin
need not lie on the lattice; the IK solve remains the authority. `reach_map.load_full`
exposes the map's sample spacing, which that tolerance needs. `load` is unchanged.

### `build_structure.py` had no tests; now it has 15

This is the module that generates the joint path streamed to the real arm, and it
was the only major module with no test file. `sim/test_build_structure.py` covers
reach gating and origin ranking, and asserts the properties a bad plan would
violate: every pose inside the arm's **calibrated** limits, one step per cube,
support placed before what rests on it, gripper angle in range, `tool_target_m`
consistent with the rotated grasp point, byte-identical re-export, and a rejected
plan leaving no trajectory file behind.

### Finding: transfers tilt the carried cube up to 25.5 degrees

Writing those tests turned up something the existing checks missed. Measured over
the 16-cube dog:

| Phase | Max tool tilt from vertical |
| --- | --- |
| descend, close, attach, place, release, open | **< 0.03 deg** |
| lift, retreat | < 0.03 deg |
| approach | 19.5 deg |
| **transfer (carrying a cube)** | **25.5 deg** |

104 of the 632 carrying frames exceed 10 degrees. This follows from `move_joint`
interpolating in joint space: that keeps every intermediate pose reachable, which
is why it is used, but it does not hold the tool vertical. Grasp-critical phases
use `move`, which IK-solves each waypoint, and they are vertical.

**Idealized attachment is why nobody noticed.** The simulation carries the cube
rigidly, so it cannot drop one. Two foam pads holding a cube by friction at 25
degrees is a different question, and an open one. The bound is asserted in the
test suite so it cannot quietly grow.

Next on this track: decide whether the tilt matters — either hold the tool vertical
through transfers, or show the grip survives it in the contact simulation, which
does model friction. Do not "fix" it by reverting transfers to Cartesian
interpolation without re-checking reachability; that is what joint-space
interpolation was introduced to solve.

Verified: **41 sim tests** (26 before) and 22 compiler tests pass.

## Merged to main (2026-09-12)

`fix/assembly-centered-grasp` fast-forwarded into `main` at **8f083e3** and pushed.
Six commits, previously unmerged, from two authors: the centered soft-pad grasp and
stable cube colors, the compiler that turns a prompt into a checked voxel structure,
the assembly grasp centred on the measured contact patch, the robot-side scripts,
the first hardware executions, and the robot backup plus
[ROBOT_SETUP.md](ROBOT_SETUP.md). Source branches are retained.

Verified on `main` before pushing: 26 sim tests, 22 compiler tests,
`simulate_assembly.py --check` at 409 poses, and `build_structure.py` planning the
16-cube dog at 1699 poses.

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

`--origin` was required and recorded nowhere; the old default `[.22, .30]` did not
fit the reachable area and failed with a bare `IK failed at ...`. **Fixed 2026-09-13**
— see "Reach-checked build origins" below. Omitting `--origin` now derives one from
the measured reach map.

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
(This branch was merged into main on 2026-09-12.) Next: calibrate foam and heavy-load
slip, then failure-aware control and demonstration contracts.
