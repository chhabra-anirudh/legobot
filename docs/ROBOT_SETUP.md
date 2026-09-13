# Robot setup and status report

Everything needed to pick this up cold: what the project is, what actually works,
how to run it on the real robot, and the traps that cost us time. Written
2026-09-12 after the first hardware executions.

Read this, then [HANDOFF.md](HANDOFF.md) for the running log and
[BUILD_PLAN.md](BUILD_PLAN.md) for the agreed scope.

---

## 1. What this is

A user describes a structure — in words, or by handing over a picture. An LLM
proposes a voxel model. A deterministic compiler checks it against support,
connectivity, colour, budget, and gripper clearance rules. A planner turns accepted
cells into calibrated root-frame poses, solves the arm for every motion, and exports
a joint trajectory. A bridge streams that trajectory to a real BracketBot arm.

```
description ─┐
             ├─> LLM -> voxel schema -> checks -> placement plan -> trajectory -> robot bridge
picture ─────┘                            |                              |
                                    inventory/budgets              arm calibration
```

One command runs everything up to the simulated build:

```sh
.venv/bin/python build_from_description.py "a small house"
```

## 2. Status: what works, what does not

**Works end to end.** A prompt-derived, checked 16-cube structure has been
planned, exported, and **executed on the real arm twice**. Both runs completed
with zero joint-range violations.

**Works in simulation (2026-09-13).** Description or picture to a checked structure
to an animated build with coloured cubes, in one command. The largest examples that
plan cleanly are 40 cubes (`compiler/examples/house.json`, from a description) and
36 cubes (`compiler/examples/cat.json`, from a picture). Nothing above 16 cubes has
been run on hardware.

**Does not work / not implemented:**

- **No held-cube sensing.** Attachment is idealized in the planner: it *assumes*
  a commanded close picks the cube up. The robot has no idea whether it is
  holding anything. A run with an empty table and a run with cubes look identical.
- **No verified pick.** Both hardware runs used an **empty table**. Nothing has
  confirmed that the jaw actually acquires a cube in the arm's own motion.
- **No collision checking** of the arm against the table, itself, or the structure.
  Transfers interpolate in joint space, which keeps poses reachable but traces an
  unchecked curve.
- **Transfers tilt the carried cube up to 25.5 degrees.** Joint-space interpolation
  holds reachability, not tool orientation. Every grasp-critical phase (descend,
  close, place, release) is vertical to within 0.03 degrees, but 104 of the 632
  carrying frames exceed 10 degrees of tilt. Idealized attachment means the
  simulation never drops anything; two foam pads holding a cube by friction might.
  Locked in by `sim/test_build_structure.py` so it cannot quietly get worse.
- **No vision of the workspace**, no magnetic model, no learned policy in the
  hardware path. The compiler can now read a *picture of what to build*, which is
  not perception: the robot still cannot see where its cubes are.
- **No live model call yet.** The API paths for both description and picture are
  implemented but have never run with credentials. Designs so far came from
  `--offline` library shapes or from `--request`/`--ingest` answered in-session.
- **Table height is a set value, not a survey.** See §6.

## 3. Workstation setup

Python 3.10+.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-physics.txt   # sim + mujoco
.venv/bin/python -m pip install -r requirements-llm.txt       # anthropic SDK + Pillow

.venv/bin/python -m unittest discover -s sim -p 'test_*.py'         # 52 tests
.venv/bin/python -m unittest discover -s compiler -p 'test_*.py'    # 52 tests
.venv/bin/python sim/simulate_assembly.py --check
```

All 104 tests should pass. Nothing here talks to the robot. The image tests skip
without Pillow; no test calls a model.

## 4. Robot setup

### Connect

Open a shared SSH control master once; every later command reuses it without
re-authenticating.

```sh
ssh -M -S ~/.ssh/claude-%C -o ControlPersist=1h -o ServerAliveInterval=30 \
    bracketbot@bracketbot-0152.local
```

Then run commands over it with `ssh -S ~/.ssh/claude-%C bracketbot@bracketbot-0152.local '...'`.

### Restore our scripts

`~/bbapps` on the robot is **not version controlled and is shared with other
teams**. This repo is the source of truth; the robot copy is disposable.

```sh
./robot/deploy.sh push     # repo -> robot
./robot/deploy.sh pull     # robot -> repo (save new taught poses)
```

Scripts run on the robot with `uv run <script>.py` — each has a PEP 723 header
pointing `bbos` at `/home/bracketbot/bbos`, so there is no venv to manage.

### Check it is alive

```sh
uv run probe_arm.py      # read-only: joint state in turns and URDF radians
uv run chan_check.py     # which IPC channels have a live writer
uv run log_read.py arm_left.log
```

If `probe_arm.py` reports no state, check the daemon is running:
`ps -ef | grep "daemon.py arm_left"`.

## 5. Running a build, end to end

```sh
# 1. Generate a structure (or use the cached example)
.venv/bin/python compiler/generate.py "a simple dog" --output outputs/dog.json
.venv/bin/python compiler/check.py compiler/examples/dog.json --order

#    From a description or a picture, with the simulated build included:
.venv/bin/python build_from_description.py "a small house" --check \
    --structure-out outputs/house.json
.venv/bin/python compiler/image_to_structure.py compiler/examples/images/cat.png \
    --trace --grid 12 8 --output outputs/cat.json

# 2. Plan and export. Omit --origin and the planner picks one from the reach map.
.venv/bin/python sim/build_structure.py compiler/examples/dog.json --check \
    --export-trajectory outputs/dog-trajectory.json \
    --export outputs/dog-plan.json --coords

# Pin the origin instead, or see what else fits:
.venv/bin/python sim/build_structure.py compiler/examples/dog.json --check --origin 0.15 0.2008
.venv/bin/python sim/build_structure.py compiler/examples/dog.json --list-origins

# 3. Ship it
scp -o ControlPath=~/.ssh/claude-%C outputs/dog-trajectory.json \
    bracketbot@bracketbot-0152.local:~/bbapps/

# 4. On the robot: dry run first. It opens no writer and moves nothing.
uv run build_bridge.py dog-trajectory.json --gripper
uv run build_bridge.py dog-trajectory.json --gripper --execute
```

The trajectory is deterministic: regenerating it from the same structure, config,
and origin gives a **byte-identical** file. Verified by sha256 against the robot's copy.

### Choosing the build origin

`--origin` is the root-frame XY of voxel cell (0,0). **Omit it and the planner
derives one from the measured reach map**, ranked so the build sits as far from
the edge of the reachable set as possible, then prints what it chose. There is no
hardcoded default any more: one silently rots every time the table moves, which is
exactly what happened when the table went to 0.75 m.

Give an origin that does not fit and the planner names the offending columns and
suggests origins that do, instead of failing with a bare `IK failed at [...]`:

```
Cannot execute this structure: 7 of 16 build columns fall outside the arm's
measured reach at origin (0.22, 0.3): (4, 2) at (0.3216, 0.3508), ... and 3 more.
  Try: --origin -0.1484 0.2262  --origin -0.123 0.2262  --origin -0.0976 0.2262
```

The reach check is a **pre-filter**, matched against the nearest lattice sample so
an origin need not sit on the lattice. The IK solve is still the authority.

### Reading the dry run

It must report **0 in the `over` column** for every joint — those are points
outside the arm's own calibrated range, which the daemon would clamp, silently
making the robot not follow the plan. `build_bridge.py` refuses to execute if any
exist.

Also check the planner's **cold IK restart** count. A cold restart means the
solver reseeded from zero and may have landed in a different branch, so the joint
path can jump. `--origin 0.15 0.2008` is clean; prefer an origin with none.

### Useful flags

| Flag | Effect |
| --- | --- |
| `--gripper` | **Drive the gripper.** Without it the jaw is pinned to its current position *and its torque is disabled*. |
| `--speed 0.25` | Time scale. Default is quarter speed. Lower is slower. |
| `--execute` | Actually move. Without it, pure dry run. |

## 6. Hardware facts and calibration

| Quantity | Value | Provenance |
| --- | --- | --- |
| Control period | 15 ms | `@realtime(ms=15)` on `arm_ctrl` |
| DOF | 8 (`lj0`..`lj6`, `left_left_gripper`) | `Config('arm_left')` |
| `lj0` units | metres; `lj1`..`lj6` and gripper radians | URDF convention |
| Gripper sign | URDF rad = `-2*pi*turns` | `gripper_sign = -1` |
| Gripper range | `[-0.379, 0]` turns (~0 to 2.38 rad) | `ranges.calibration.json` |
| **Grasp angle (1 inch cube)** | **0.2301 rad with a 25.4 mm cube held** | **Measured on hardware** |
| `gripper_grasp_rad` | **0.21 rad** for the one-inch cube | restored one-inch configuration |
| `gripper_open_rad` | **0.35 rad** | restored one-inch configuration |
| Gripper torque cap | 300 of 1000 | `torque_limit[7]` |
| J7 current relief | enabled, `i_hold = 1.5 A`, backoff <= 0.1 turns | `constants.py` |
| Table height | **0.5 m (set, not surveyed)**; 0.78 m measured and rejected | `sim/assembly_config.json` |
| Block size | **25.4 x 25.4 x 25.4 mm** (one-inch cube) | confirmed by the user |
| Tool grasp point | `[0.018, 0, 0.004]` m in `left_eef` | foam pad centre offset |

### Historical two-inch proposal (superseded by explicit user instruction)

**Current blocks remain one inch on every axis.** The following derivation is
historical only; its 0.36/0.50 rad commands are not active. Learning uses a
separate provisional contact-controller opening, not hardware calibration.

#### Original derivation

The blocks changed from 1 inch cubes to 2 inches across and 1 inch tall. Height is
unchanged, so **nothing about how far down the arm reaches changed** — only the jaw
opening. From the mesh report (`docs/gripper_geometry.json`, gap against angle):

| Quantity | 1 inch | 2 inch |
| --- | --- | --- |
| Bare width-matching angle | 0.1538 rad | **0.3042 rad** |
| Hardware hold, with foam | 0.2301 rad (measured) | **0.3806 rad** (bare + the same +0.0763 foam allowance) |
| `gripper_grasp_rad` | 0.21 | **0.36** (0.02 past the hold, as before) |
| `gripper_open_rad` | 0.35 | **0.50** (~9 mm clearance per side) |

The 2 inch grasp angle is **derived, not measured**: the foam allowance is carried
over from the 1 inch hardware measurement. Re-measure with a real block in the jaw
the way 0.2301 was measured. The joint limit is 1.0 rad and the mesh report covers
0.6 rad with both fingers on the slab, so there is headroom either way. Fingertip
clearance above the table grows to about 8 mm at 0.50 rad, which is why the vertical
numbers did not need touching.

### The gripper actually grips now — here is why it did not before

Two independent causes:

1. `build_bridge.py` does not drive the gripper without `--gripper`.
2. `gripper_grasp_rad` was **0.19 rad, a 31.4 mm inner gap around a 25.4 mm
   cube** — six millimetres of air. It could never have gripped, flag or no flag.
   That value was always documented as an animation setting, not a grasp.

The measured 0.2301 rad is **wider** than the mesh report's bare-finger
width-matching angle of 0.1538 rad, because the foam pads take up roughly 6 mm
per side. **Do not use bare-finger mesh geometry as a grasp command on padded
fingers.**

**Closing past contact is safe on this hardware**, which reverses the older
warning in [GRIPPER_CALIBRATION.md](GRIPPER_CALIBRATION.md) (written when the
controller was unknown). The daemon's J7 current-relief loop settles to a gentle
current-limited hold rather than stalling. Its `i_hold` is marked "TUNE on
hardware" upstream and has not been tuned here.

### Table height drives everything

`table_height_m` is the most consequential number in the config, because reach
collapses near the top of the arm's vertical travel:

| Table | Reachable cells | Valid 8x3 build origins |
| --- | --- | --- |
| 0.75 m | 95 (a thin arc) | 6 |
| 0.50 m | 246 (a broad region) | 133 |

After any table change, **re-run the reach map** — results cache per height as
`sim/reach_<h>.json`:

```sh
.venv/bin/python sim/reach_map.py --table 0.5
```

The 0.5 m map is clipped at the probe boundary (x = -0.25 is fully reachable), so
the true reachable set is wider than measured. Widen `--x`/`--y` if you need more.

## 7. bbos interface reference

What we had to reverse-engineer. Source: `~/bbos/bbos/ipc.py`,
`~/bbos/bbos/daemons/arm_left/`.

### Channels

| Channel | Fields |
| --- | --- |
| `arm_left.state` | `pos`, `vel`, `torque`, `temp`, `current` — all `float32[dof]` |
| `arm_left.ctrl` | `pos`, `vel`, `tau` `float32[dof]`; `alpha` `float32` scalar |
| `arm_left.torque` | `enable`, `tau_mode` `bool[dof]`; `compliance_mode`, `axis_aligned`, `force_only`, `j0_homing`, `calibrating` bool scalars |

The daemon reads `ctrl.pos` in **motor turns**, rate-limits it (`clip_target`),
clamps it to the calibrated range (`clip_range`), then low-pass filters it.

### Writer API — three traps

1. **`Writer` has no `.data` / `.write()`.** Use the context manager:
   ```python
   with w.buf() as b:
       b['pos'][:] = q.astype(np.float32)
   ```
   `Reader` *does* have `.data`. They are not symmetric.

2. **`Writer` paces your loop for you.** `keeptime=True` is the default, and
   `buf()` ends in `Loop.keeptime()`, which sleeps out the type's 15 ms period.
   So **do not add your own `time.sleep`** — it stacks. Worse, the clock is
   *per process and shared*: `keeptime()` only sleeps once per registered
   keeptime writer, so a second paced writer you do not touch every tick halves
   your rate. Open write-once channels with `keeptime=False` and let exactly one
   writer pace the loop. `build_bridge.py` opens `torque` unpaced and lets `ctrl`
   drive.

3. **One writer per channel, process-wide.** Opening a second raises
   `RuntimeError: Writer for arm_left.torque already exists (pid=N)`. That is not
   a broken daemon — find the process and stop it. `examples/view_arms.py
   --control` is the usual culprit; stop it with **SIGINT, not SIGTERM**, so its
   `finally` block runs and leaves the arm limp.

### The deadman

When the ctrl writer closes, the daemon sees `not r_ctrl.readable` and **disables
torque immediately**, re-sending at 1 Hz. So the arm goes **limp the moment your
script exits**. It is not holding anything afterwards. This is good safety
behaviour, but do not design around the arm holding a pose after the bridge returns.

### Reader warm-up quirk

The **first** `Reader` opened in a process never reports ready, however long you
poll. A freshly constructed one does. So retry with a *new* `Reader` rather than
polling the first forever — otherwise a healthy daemon looks dead.
`robot/diagnostics/two_readers.py` demonstrates it; `read_state()` in our scripts
works around it.

## 8. Safety

- Run the dry run first, every time. It opens no writer and moves nothing.
- **Joint limits are not workspace limits.** Every point can be inside the arm's
  calibrated range while the tool drives straight into the table. The dry run
  cannot catch a wrong `table_height_m`. Check the table before executing.
- Keep a hand on the e-stop. The bridge prints a 3 s warning before enabling torque.
- The workspace must be clear, or staged exactly as `outputs/dog-plan.json` says.
- `teach_pose.py`, `probe_arm.py`, and everything in `robot/diagnostics/` are
  read-only: they open no writer and cannot move the arm.

## 9. What to do next

In priority order — see the table in [HANDOFF.md](HANDOFF.md) for ownership.

1. **Stage cubes and verify a real pick.** Nothing has yet confirmed the jaw
   acquires a cube during the arm's own motion. Coordinates are in
   `outputs/dog-plan.json` under `staging`.
2. **Add a held-cube signal.** `arm_state.current` is already published, and the
   J7 relief loop's holding current is a natural detector: a close that reaches
   the commanded angle with *no* current rise means an empty jaw. This is the
   single highest-value missing piece — without it the system cannot tell success
   from failure.
3. ~~Check build cells against the reach map~~ — **done 2026-09-13.** The planner
   pre-filters the footprint against the reach map, auto-selects an origin when
   none is given, and suggests fitting origins when one is rejected.
4. **Teach a real `table_surface` pose** to replace the set-by-hand 0.5 m.
5. **Decide whether the 25.5-degree transfer tilt matters.** Either hold the tool
   vertical through transfers, or show that the grip survives it. The cheapest
   check is the existing contact simulation, which does model friction.
5. Contact simulation, demonstrations, and behaviour cloning — the ML track,
   unchanged and described in [BUILD_PLAN.md](BUILD_PLAN.md) §3-4.

## 10. Repository map

| Path | What |
| --- | --- |
| `compiler/` | Prompt to voxel structure, deterministic checks, examples |
| `sim/` | URDF, IK, planner, reach maps, contact experiment, config, tests |
| `sim/assembly_config.json` | Table height, grasp angles, tool grasp point — **the main knobs** |
| `sim/robot_limits.json` | Arm calibrated ranges in URDF units; derived from the robot |
| `sim/reach_<h>.json` | Cached reach map per table height |
| `robot/` | Scripts that run **on the robot** with `uv run` |
| `robot/deploy.sh` | Push scripts to the robot / pull measurements back |
| `robot/calibration/` | Backups of robot-side state, with provenance |
| `robot/diagnostics/` | Read-only IPC probes |
| `docs/` | This report, the handoff log, the build plan, calibration studies |
