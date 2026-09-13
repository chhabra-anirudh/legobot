# LegoBot

Prompt- or picture-to-structure assembly with BracketBot and magnetic blocks
**2 inches across and 1 inch tall** (50.8 x 50.8 x 25.4 mm). One arm and a finger
gripper; imitation learning for pickup is planned.

**Start here:** [Robot setup and status report](docs/ROBOT_SETUP.md) — full context for picking this up cold ·
[Current status and next tasks](docs/HANDOFF.md) · [Build plan](docs/BUILD_PLAN.md) ·
[Contributing](CONTRIBUTING.md) · [Gripper calibration](docs/GRIPPER_CALIBRATION.md)

## Describe it, watch it built

```sh
.venv/bin/python build_from_description.py "a small house"
```

`description -> LLM -> voxel structure -> deterministic checks -> simulated build`.
One command: it asks the model for a design, checks it (support, connectivity,
finger clearance, colours, budgets, reach), reduces it if it cannot be built and
says what it changed, then stages coloured cubes on the table and animates the arm
placing every one of them. Select the `simulation` timeline in Rerun and press Play.

Without `ANTHROPIC_API_KEY`, the same command runs two other ways — hand-authored
shapes with `--offline`, or `--request`/`--ingest` so another model or an agent
answers the design request. The structure file always records which one produced it.

```sh
.venv/bin/python build_from_description.py "a simple dog" --offline --save outputs/dog.rrd
.venv/bin/python build_from_description.py "a rocket" --request outputs/ask.json
.venv/bin/python build_from_description.py "a rocket" --ingest outputs/reply.json \
    --source "<what answered it>"

# cached examples, planned straight into the simulator
.venv/bin/python sim/build_structure.py compiler/examples/smiley.json   # 10 blocks
.venv/bin/python sim/build_structure.py compiler/examples/rocket.json   # 11 blocks
```

The reachable build area is **10x3 blocks** at a 0.5 m table. At 0.78 m only a single
row of six fits — see [the handoff](docs/HANDOFF.md) before moving the table.

The simulated build is kinematics with idealized attachment: finishing here means
the design satisfies the stated rules, not that the real arm can place 40 cubes.

## What works today

A kinematic simulation stacks three cubes using the supplied robot URDF, with
joint-limited inverse kinematics and animated finger joints. Grasp attachment is
idealized. An isolated [contact experiment](docs/CONTACT_SIM.md) now tests a free cube under
gravity using the finger joints. The [centered foam-pad model](docs/CENTERED_GRASP.md) passes nominal lift/hold/return
checks with measured force/slip gates and configurable colors. Foam parameters
remain provisional; heavier-load slip and real calibration are unresolved. A [structure compiler](compiler/README.md) turns a prompt **or a picture** into a
voxel model with Claude and checks it deterministically for support,
connectivity, finger clearance, colors, and budgets; a picture is downsampled onto
the build grid and reduced to buildable line art, with every cube it drops
reported. `sim/build_structure.py` then stages coloured cubes on the table and
animates the whole build with joint-limited IK, gated on the measured reach map. A [robot bridge](robot/build_bridge.py) now streams a planned trajectory
to the real arm over bbos, range-checked against the arm's own calibration, and
**the arm has been driven from a simulated plan**. Attachment is still idealized
and there is no held-cube signal, so that is a motion run, not a verified build.
Full-arm collision checking, learned pickup, and vision are not implemented yet.

## Run the baseline

Python 3.10+ is required. From the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python sim/simulate_assembly.py --check
.venv/bin/python -m unittest discover -s sim -p 'test_*.py'
.venv/bin/python sim/simulate_assembly.py
```

In Rerun select the `simulation` timeline and press Play. To save without a GUI:

```sh
.venv/bin/python sim/simulate_assembly.py --save /tmp/legobot.rrd
```

Edit `sim/assembly_config.json` to change the workspace and provisional grasp
settings. See [simulation documentation](sim/README.md) for the coordinate
conventions and learning walkthrough. These scripts do not command the real robot.

## From a picture instead of a description

```sh
.venv/bin/python -m pip install -r requirements-llm.txt
.venv/bin/python compiler/image_to_structure.py compiler/examples/images/cat.png \
    --trace --grid 10 3 --output outputs/cat.json      # deterministic, no model
.venv/bin/python sim/build_structure.py outputs/cat.json --save outputs/cat.rrd
```

Drop `--trace` to send the picture to a vision model instead. A picture is
downsampled onto the build grid and reduced to buildable line art, with every block
dropped or added reported. `compiler/examples/images/cat.png` is the tracked test
image.

## Layout

- `build_from_description.py`: the one-command demo — description to simulated build.
- `docs/BUILD_PLAN.md`: agreed working proposal, milestones, and technical caveats.
- `docs/HANDOFF.md`: current implementation, task ownership, verification, blockers.
- `sim/`: simulation scripts, tests, configuration, and supplied URDF/meshes.
- `compiler/`: prompt or picture to voxel structure, deterministic checks, examples.
- `robot/`: read-only arm probe, pose teaching, and the trajectory bridge. These
  run on the robot with `uv run`, not on the workstation.
- `AGENTS.md`: handoff and collaboration instructions for coding agents.

Update the handoff at each completed milestone so the next teammate can continue
without needing the original chat.
