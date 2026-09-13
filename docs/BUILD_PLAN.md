# Prompt-to-structure robot: merged hackathon plan

This merges the supplied **PixelForge 3D: Full Build Plan** with the existing
BracketBot simulation and our imitation-learning discussion. The source document
is a proposal; its hardware measurements, event details, deadlines, and shopping
claims are not independently verified here. Planning assumption: three teammates
and 24 working hours. Adjust the schedule to actual robot access and event rules.

## What we are building

A user describes a small structure. An LLM proposes a voxel model. A deterministic
compiler checks the model against inventory, support, workspace, and gripper
clearance, then produces a placement sequence. The robot builds it with 25.4 mm
magnetic cubes using one finger gripper.

The learning component is a **pickup policy trained by imitation**. It learns
small tool movements and gripper actions from demonstrations. It does not decide
structural validity or send raw motor commands. The existing IK/controller remains
between the policy and the robot.

Demo promise: prompt → checked preview → physical construction, with a separately
measured learned-pickup result. If learned pickup is not ready for hardware, show
its simulation evaluation clearly labeled and use scripted pickup for the live
build. Do not present scripted actions as learned behavior.

## Scope and success criteria

Status marks below are as of 2026-09-12; see [ROBOT_SETUP.md](ROBOT_SETUP.md).

| Level | Target | Status |
| --- | --- | --- |
| First milestone | One cube picked, lifted, held, and placed on the real bot | **Partial.** The arm executes the full build motion on hardware with the gripper driven, but on an empty table and with no held-cube signal. The pick itself is unverified |
| Minimum live demo | A compiler-approved 4–8 cube structure, two layers, fixed supply poses | Not started. A checked 16-cube single-layer structure plans and executes as motion |
| ML milestone | Behavior-cloned pickup evaluated on held-out randomized poses in contact simulation | Not started |
| Stretch live demo | Learned pickup on hardware; 15–20 cubes and three layers only after repeatable smaller builds | Not started |
| Deferred | RL fine-tuning, dual arms, image-to-action learning, arbitrary piles, unsupported bridges | Deferred |

Use towers, stepped structures, and supported letters as initial shapes. An arch
with an empty space directly below its lintel fails our support rule. A dense
footprint may also fail finger clearance even when each cube has support.
Do not promise arbitrary 3D models.

## Current implementation: what is actually working

- The existing URDF is loaded in Rerun, with a left-arm damped-least-squares IK solver.
- Cubes are exactly 0.0254 m per side according to the user.
- Three cubes follow a scripted stacking sequence; both fingers animate using the
  URDF mimic relationship. Five numerical tests pass for the current configuration.
- Grasp angles, tool grasp point, table, and supply coordinates are provisional.
- Attachment is idealized. There is no contact, friction, magnetic force,
  collision checking, motor-rate enforcement, learned policy, CV, or hardware bridge.
- The 396-frame replay demonstrates kinematics, not physical grasp success.

## Corrections to carry into the merged design

1. **Check rather than “prove physically buildable.”** Discrete rules can establish
   validity under stated assumptions. They cannot establish real friction,
   calibration accuracy, magnetic behavior, dynamic stability, or reliable execution.
   Pitch: “We compile generated structures into robot-specific, checked build plans.”
2. **Verify the jaw geometry.** The source quotes a 25.4 mm gap at 0.185 rad and
   roughly 57 mm outer width. Treat these as hypotheses until the measurement
   cross-section and real finger contacts are checked. Our current 0.19 rad grasp
   command is an animation parameter, not a force-controlled grasp.
3. **Specify tool orientation, not just wrist angle.** Search candidate tool yaw
   values while maintaining the approach direction, then solve the full arm IK.
   Joint `lj6` is not generally interchangeable with yaw in the root frame.
   Stay with the left arm already implemented; use `root` for external poses.
4. **Check approach and withdrawal.** Empty neighboring cells at the destination
   alone do not establish a clear path for fingers, hand, arm, or carried cube.
   Check the open-finger retreat too, and check intermediate joint configurations.
5. **Calibrate pose accuracy against clearance.** A generic “under 10 mm” camera
   target is not sufficient evidence for a 25.4 mm grasp. Derive tolerated error
   from measured jaw clearance, cube uncertainty, and approach geometry.
6. **Treat magnets as an experiment.** Test attraction to the proposed base plate,
   face compatibility, double-picks, sliding, and release by hand. Do not assume
   magnetic self-registration or that fixed polarity makes assembly impossible.
7. **Use simple support first.** Direct support excludes overhangs. A global centre
   of mass inside a footprint alone is not a stability certificate; check partial
   builds as well and avoid claiming dynamic stability.

## Architecture and shared interfaces

```text
prompt → LLM → voxel schema → compiler → placement plan → executor → IK/robot bridge
                               ↑                          ↓
                    inventory + calibration       scripted or learned PICK
                                                          ↑
                                              calibrated cube observation
```

One codebase with separate modules is enough. Do not spend the build window
operating five independently deployed services.

Freeze versioned interfaces first:

- **Structure:** integer voxel coordinates, requested color, nominal cube size,
  unique structure ID. Reject duplicates, negative layers, nonfinite values,
  unsupported colors, and size/budget violations.
- **Calibration:** `T_root_build`, taught supply poses, measured XY pitch and layer
  height, tool grasp point, arm selection, units, and calibration version.
- **Placement step:** voxel, assigned color, target tool pose in `root` (position
  in metres and quaternion in XYZW order), approach/retreat poses, primitive name.
  Record the yaw candidate as metadata; the executor solves actual joint values.
- **Cube observation:** `frame=root`, position, orientation, timestamp with a shared
  clock definition, uncertainty, and confidence. Missing/stale observations are
  explicit failures, not a pose of zero.
- **Execution result:** step ID, state, success/failure reason, attempt count,
  controller mode (`scripted` or `learned`), and elapsed time.

The compiler must use calibrated build coordinates instead of directly treating
voxel indices as robot coordinates. Keep nominal cube size separate from measured
stack pitch. Recompile after a geometry or calibration change.

## Work packages and their order

### 1. Physical grasp and control baseline

Identify the robot API, available encoder/gripper/contact feedback, camera setup,
and emergency stop/reset procedure. Fix the mobile base for this task. Teach a
small workspace and one separated pickup location; add magazines only if they
reduce reset effort. Test a single cube before designing a multi-cube feeder.

Measure jaw opening versus angle, finger contact region, tool-to-cube transform,
and repeatability. Determine what signal verifies a held cube: camera evidence,
gripper feedback if available, or supervised confirmation during initial tests.
Do not assume that a commanded close means a successful grasp.

**Status (2026-09-12): hardware access confirmed and the control path is built.**
The robot API, feedback channels, calibrated joint ranges, and gripper force
behaviour are identified and written up in [ROBOT_SETUP.md](ROBOT_SETUP.md).
`robot/build_bridge.py` streams a planned trajectory to the left arm, range-checked
against the arm's own calibration, and has executed a full 16-cube build path
twice. Jaw angle was measured against a real cube (0.2301 rad) and replaced the
guessed animation value. **Not done:** the 20-attempt gate below. Both runs used an
empty table, and there is still no signal that distinguishes a held cube from an
empty jaw, so pick attempts cannot yet be scored at all. Building that signal comes
before running the gate.

Gate: record 20 scripted pick–lift–hold–place attempts and failure causes. An initial
18/20 target is a development gate, not sufficient evidence for a reliable long
build. At independent 90% per-cube success, 20 placements succeed together only
about 12% of the time. Evaluate complete builds and recovery separately.

### 2. Compiler and executor baseline

Implement schema validation, direct support, connectivity, height/cube budgets,
and loaded inventory. Search bottom-up placement orders and candidate tool yaws.
Use conservative finger/hand geometry for the initial clearance filter, then
validate paths against the robot model where supported. Place time/node limits
on search. A timeout means “not resolved,” not “physically impossible.”

Reject unsupported geometry with a visible explanation. If offering a repair,
show the changed preview and rerun every check; deleting one voxel can invalidate
support or connectivity elsewhere. Cache several fully checked demo structures.

Gate: a valid small structure produces executable steps; a floating voxel and an
intentionally blocked placement are rejected for the correct reason.

**Status (2026-09-13): implemented, for prompts and for pictures.** `compiler/`
turns a prompt or an image into a voxel model — via `claude-opus-5` with structured
output, or deterministically with `--trace` — then runs schema, direct support,
6-connectivity, finger clearance, colour, and cube/layer/footprint/inventory
checks, with a re-checked repair round when a proposal fails. An image is
downsampled onto the build grid and reduced to buildable line art, with every
dropped or added cube reported. `sim/build_structure.py` is the executor:
calibrated build coordinates, tool yaw per placement, reach-map gating, placement
order, staging layout, joint trajectory export. Examples: `dog.json` (16 cubes,
prompt), `cat.json` (36 cubes, picture), `dog-floating-rejected.json` (negative
control). **Not implemented:** full-path collision validation against the robot
model, and a confirmed per-colour inventory.

### 3. Contact simulation and demonstration collection

Build a contact simulation from the existing model. MuJoCo is the proposed engine;
first validate conversion, inertias, actuators, and collision proxies in a minimal
one-cube scene. Preserve the robot geometry needed to check nearby collisions;
keep Rerun for inspection rather than treating it as a physics engine.

Use the scripted approach–close–lift controller as the expert — by default the
preconfigured top-down primitive specified in work package 4, so demonstration
collection does not block on robot access. Remove idealized attachment when
judging grasp success. Log observations, commands, episode
parameters, contact information available in simulation, and actual outcomes.
Vary cube position/yaw within a bounded pickup region. Split by episode and scene
parameters before training; neighboring frames from one trajectory must not leak
across training and evaluation.

Gate: the expert holds a cube above the table for a defined interval under gravity,
then places it. Failed expert episodes are identified rather than mislabeled as
successful demonstrations. Hardware-only sensors must not appear accidentally in
the deployed policy's training observations, and simulator-only contacts may be
used for labeling but not as unavailable policy inputs.

### 4. First learned policy: behavior cloning

#### Default when there is no robot access: preconfigured top-down grasp

Assume no robot access unless hardware time is confirmed. In that default case, do
not wait for the robot to start the imitation-learning workstream: generate the
demonstrations from a preconfigured scripted grasp primitive in simulation, and
label every resulting artifact as simulated. Switch the expert to teleoperated or
hand-guided demonstrations only once hardware is actually available.

**Outcome (2026-09-12): the one-third proposal below was tested and not adopted.**
The contact experiment settled on a grasp at **half cube height** (`--depth .5`)
with the EEF shifted **-18 mm in X** from the cube centre, because the foam pad
centre sits 18 mm from the nominal tool origin. Moving the bare fingers lower
alone was rejected for causing table contact. `sim/assembly_config.json` now uses
`tool_grasp_point_m = [0.018, 0, 0.004]`, matching `sim/contact_grasp.py`. See
[CENTERED_GRASP.md](CENTERED_GRASP.md). The proposal is kept below because the
depth fraction remains configurable and the reasoning still applies if pad
geometry changes after real calibration.

The proposed primitive approaches from directly above and grips opposing side
faces near the top of the cube. Defined in `root`, with cube height `h` (25.4 mm) and the tool z-axis
pointing down:

1. **Hover over the block.** Tool contact reference above the cube centre in XY, at
   `z_top + clearance` (0.04 m default). Choose tool yaw so the finger closing
   axis is perpendicular to the cube faces it will contact; keep the approach
   direction fixed while searching yaw.
2. **Descend straight down.** Move along −z only, until the tool contact reference
   reaches the grasp band one third of the cube height below the top face:
   `z_grasp = z_top − h/3 = z_centre + h/6`. For 25.4 mm cubes that is 8.47 mm
   below the top face and 4.23 mm above the cube centre. Store this as a config
   fraction (`grasp_depth_from_top = 1/3`), not a hardcoded offset; the current
   simulation defines `tool_grasp_point_m` as the cube centre in tool coordinates.
   Introduce an explicit contact-reference convention; do not silently change
   that field's meaning. With a fixed contact reference and downward tool, raising
   the tool by `h/6` moves the cube centre by **+h/6 in tool-local Z**.
3. **Close fingers** to the grasp angle, then check for a held cube instead of
   treating the commanded close as success.
4. **Lift straight up** along +z to travel height before any lateral motion.
   Place and retreat mirror this: descend vertically, release, open, lift
   vertically.

This higher contact target is intended to improve table/lower-layer clearance.
It does not guarantee it: the pad extent, jaw angle, approach/retract sweep, and
calibration error still need checking. The merged [mesh report](GRIPPER_CALIBRATION.md)
finds about 1.55 mm nominal fingertip clearance near the current width-matching
candidate. Raising the tool by `h/6` would add about 4.23 mm at an unchanged angle,
but the width-matching angle and contact region change with depth. Re-run the
probe and validate contact before adopting the new pose.

A contact reference one third of the height below the top is not the same as a
contact patch contained in the upper third. Measure the patch's actual vertical
extent and ensure that both fingers overlap the cube sides without reaching the
table or lower layer. This is what ruled the fraction out in simulation: the
soft-pad model holds at half height and the higher band was not validated. The
question returns whenever pad geometry or the measured contact patch changes.

Vertical approach, grasp depth, hover clearance, and yaw remain part of the
recorded episode parameters so a later hardware expert can be compared against
this scripted one on the same contract.

#### Policy

Start with a small supervised policy using robot state, relative cube pose,
observation age/uncertainty, and optionally short history. Output bounded Cartesian
increments and a defined gripper target. Specify frame, control period, units,
normalization, limits, and episode termination. The controller enforces limits
and rejects invalid commands independently of the network.

Compare against the scripted expert on identical held-out episode seeds. Report
pick/hold success, placement error, drops, collisions, timeouts, and completion
time; do not use training loss as the demo's success metric.

If cloning drifts, add a small correction dataset from the states the policy visits
(DAgger-style iteration). RL is a stretch only after this works. There is no need
to complete BC, DAgger, and RL to demonstrate a real learned component.

Gate: choose an explicit evaluation threshold before running the final evaluation;
record all trials, including failures. Keep the scripted physical fallback.

### 5. Integration and guarded transfer

Calibrate the fixed camera transform if variable-position pickup is needed. For the
first physical demo, taught fixed supply locations can avoid making CV a blocker.
Compare real observations with the training contract. Introduce measured ranges
of pose error, friction, mass, latency, and gains into simulation experiments.

Test the learned primitive with a single separated cube at low speed. Keep the
compiler and placement executor deterministic. If perception or policy checks
fail, stop/retry or request reset. Use a taught pickup fallback only when a cube
is actually present at that taught location; do not continue from an arbitrary
failed-grasp state as though a cube were held.

Gate: two-layer construction repeated end to end, with logged controller mode.

### 6. Demo freeze and presentation

Show prompt input, the checked preview, one clear rejection example, and physical
placements. Display whether pickup is learned or scripted. Show held-out ML results
and disclose whether they are simulated or physical. Record complete-build success
and runtime, not just successful individual grasps. Keep offline structures and
a clearly identified backup video. Reduce structure size to fit measured cycle time.

## Team ownership and 24-hour schedule

| Owner | Primary responsibility |
| --- | --- |
| A — Robotics/ML | IK/controller, physical grasp baseline, simulator, demonstrations, BC |
| B — Compiler/UI | Schemas, validation, placement search, executor integration, preview |
| C — Workstation/integration | Fixtures, measurements, calibration with A, reset, logs, video |

Do not assign camera calibration, LLM, UI, and the entire compiler simultaneously
to B. C coordinates calibration; A supplies robot-frame measurements. CV is optional
for the fixed-pickup demo. Confirm each member's skills before final assignment.

| Working hours | Integrated target | Scope decision |
| --- | --- | --- |
| 0–2 | Confirm access; freeze interfaces; hand-test cubes/gripper; fake executor | Inventory and hardware facts become measured inputs |
| 2–6 | One scripted physical pick; basic compiler; minimal contact scene | Fix physical grasp before adding autonomy |
| 6–10 | Two-cube/two-layer loop; expert data; first BC training if scene works | If contact sim is late, retain physical demo and narrow ML claim |
| 10–14 | Held-out BC evaluation; clearance planning; small full build | Freeze live scope at two layers if larger builds are unreliable |
| 14–18 | Optional learned hardware pick; recovery and repeated complete builds | No RL unless all core gates already pass |
| 18–21 | Feature freeze; measured demo runs; offline fallback | Fix failures only; choose demonstrated build size |
| 21–24 | Record video, results, architecture slide, and rehearse | Label learned/scripted and sim/real honestly |

These are proposed cutoffs, not instructions from organizers. Confirm event rules
and permitted pre-event work directly; the source document's assertions are not
an authorization or a verified rule set.

## Immediate next implementation session

Updated 2026-09-12, after the first hardware executions. Ordered.

1. **Add a held-cube signal.** `arm_state.current` is already published and the
   daemon's J7 current-relief loop holds at a target current; a close that reaches
   the commanded angle with no current rise means an empty jaw. Without this the
   system cannot tell a successful pick from a failed one, which blocks the
   work-package-1 gate, demonstration labelling, and any honest success metric.
2. **Stage cubes and verify one real pick.** Coordinates are in the exported
   placement plan under `staging`. Confirm the jaw acquires a cube during the arm's
   own motion, not just when hand-closed.
3. ~~Check build cells against the reach map~~ — **done 2026-09-13.** The planner
   pre-filters the footprint, auto-selects an origin from the measured map when
   none is given, and suggests fitting origins when one is rejected.
   `sim/build_structure.py` also gained its first test suite.
4. **Teach a real `table_surface` pose** to replace the set-by-hand 0.5 m table
   height. Reach is extremely sensitive to it: 0.75 m gives 95 reachable cells and
   6 valid build origins, 0.5 m gives 246 and 133.
5. Extend the top-down grasp primitive into a failure-aware expert with
   observation/action recording. The primitive exists in `sim/contact_grasp.py`;
   the frozen training contract does not. Depends on 1.
6. Train the first behavior-cloning model only once demonstrations reflect a
   physically successful grasp.
7. **Decide whether the 25.5 degree transfer tilt matters.** Joint-space transfers
   do not hold the tool vertical while a cube is carried. Idealized attachment
   hides it; friction between two foam pads may not. Check it in the contact
   simulation before assuming it is fine, and do not revert to Cartesian
   interpolation without re-checking reachability.

The source document's visibility-aware color allocation is a useful later polish
item, not a first-day dependency. Keep three or fewer supply colors, fixed pickup
poses, one arm, and supported structures until the full loop is reliable.
