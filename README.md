# LegoBot

Prompt-to-structure assembly with BracketBot and 1-inch (25.4 mm) magnetic cubes.
One arm and a finger gripper; imitation learning for pickup is planned.

**Start here:** [Current status and next tasks](docs/HANDOFF.md) ·
[Build plan](docs/BUILD_PLAN.md) · [Contributing](CONTRIBUTING.md) · [Gripper calibration](docs/GRIPPER_CALIBRATION.md)

## What works today

A kinematic simulation stacks three cubes using the supplied robot URDF, with
joint-limited inverse kinematics and animated finger joints. Grasp attachment is
idealized. An isolated [contact experiment](docs/CONTACT_SIM.md) now tests a free cube under
gravity using the finger joints. The [centered foam-pad model](docs/CENTERED_GRASP.md) passes nominal lift/hold/return
checks with measured force/slip gates and configurable colors. Foam parameters
remain provisional; heavier-load slip and real calibration are unresolved. A [structure compiler](compiler/README.md) turns a prompt into a
voxel model with Claude and checks it deterministically for support,
connectivity, colors, and budgets. Full-arm collision checking, learned pickup,
vision, placement/tool-pose compilation, and hardware execution are not
implemented yet.

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

## Layout

- `docs/BUILD_PLAN.md`: agreed working proposal, milestones, and technical caveats.
- `docs/HANDOFF.md`: current implementation, task ownership, verification, blockers.
- `sim/`: simulation scripts, tests, configuration, and supplied URDF/meshes.
- `compiler/`: prompt to voxel structure, deterministic checks, and examples.
- `AGENTS.md`: handoff and collaboration instructions for coding agents.

Update the handoff at each completed milestone so the next teammate can continue
without needing the original chat.
