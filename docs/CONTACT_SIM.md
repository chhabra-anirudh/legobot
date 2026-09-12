# Isolated gripper contact simulation

## Run

From the repository root (Python 3.10+):

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-physics.txt
.venv/bin/python sim/contact_grasp.py --output outputs/contact-baseline --save /tmp/contact.rrd
.venv/bin/rerun /tmp/contact.rrd
.venv/bin/python -m unittest discover -s sim -p 'test_*.py'
```

Select the `physics` timeline in Rerun. It shows recorded dynamic finger and cube
poses, not an attached cube. The supplied mesh surfaces are displayed; the convex
collision hulls can differ from these surfaces. Use `--geometry pads` to display
and test a simpler box-pad control. This mode is explicitly not the real pads.

Negative controls:

```sh
.venv/bin/python sim/contact_grasp.py --no-close
.venv/bin/python sim/contact_grasp.py --friction 0
```

`--depth` specifies the proposed grasp-depth fraction (default 1/3). `--seed`
changes initial XY by up to 3 mm; the expert knows that exact XY. This exercises
translated scenes, not perception robustness or policy generalization.

## What is modeled

- A free 25.4 mm cube, gravity, a table plane, friction, and soft contact.
- Actual URDF finger hinge origins/axes and a joint equality for the mimic relation.
- Convex-decomposed finger collision shapes by default; `mesh_hulls` retains
  a separate coarse hull per finger as a control. Finger-to-finger collision is excluded
  because the coarse whole-finger hulls overlap near the mechanism. This exclusion
  does not disable finger/cube or finger/table contacts.
- A dynamic free EEF constrained to a commanded mocap target. Only the **EEF** is
  welded to that target. The cube is never welded, manually repositioned after
  reset, or subjected to an external lifting force.
- Two synthetic position actuators, each kp=4, kv=.05 and torque limited to
  ±0.15 N m. Together they drive the coupled jaws. These are experimental gains,
  not measured BracketBot motor parameters. Finger masses are assumed 40 g each,
  EEF mass 0.5 kg, cube mass 30 g, sliding friction 0.8, spin friction radius 1 mm,
  timestep 1 ms.
- Optional invented box pads (20 × 8 × 8 mm) for a simpler control experiment.
  They are not a validated decomposition of the supplied finger surfaces.

The cube and table use the local **`experiment_table`** frame, with table Z=0.
This has no calibrated transform to robot `root`; do not feed these poses directly
to the real bot. This is not a full-arm dynamics, collision, or magnetic model.

## Expert and outcomes

The scripted sequence settles, descends, closes, lifts 60 mm, holds one second,
lowers, opens, retreats, and waits for the cube to settle. The finger command closes
toward 0.12 rad with limited torque; actual fingers stop at contact. This differs
from assigning a geometric width-matching angle and assuming a grip.

The tool-local cube-centre depth starts from 4 mm and adds `h*(1/2-depth)`.
This preserves the earlier centre-reference convention. The one-third proposal
raises the EEF by 4.23 mm relative to the centre-depth baseline. It does not
establish the exact contact patch in the real fingers.

A hold passes only if **every 1 ms sample** of the one-second interval has cube
centre Z > 50 mm and compressive contact >0.01 N on both fingers. Full episode
success additionally requires no finger/table penetration beyond 0.1 mm and final
cube-centre error under 3 mm. Simulator warnings/nonfinite state invalidate success.
Final orientation and general trajectory collisions are not certified by this metric.

Historical baseline: see [initial development checks](contact_baseline_results.json). The initial
mesh-hull runs pass the hold but miss the final placement threshold. The box-pad
control passes the full sequence. Open-gripper and zero-friction controls fail
hold. Do not replace the mesh-hull result with the pad result when reporting robot
readiness. These small development checks are not a statistical reliability study.

## Traces and next work

`result.json` contains parameters and pass/fail metrics. `trace.jsonl` samples
physics at 50 Hz with stage, cube pose, actual EEF pose, target EEF pose, actual
finger angles, commanded angles, and contact count. Internal quaternions in these
traces are explicitly **WXYZ** (MuJoCo convention); any future public robot contract
must convert to XYZW. Rerun conversion already does so.

These traces contain perfect simulator state and contact labels. They are debugging
records, not a frozen observation/action contract or successful demonstration set.
Before imitation learning:

1. Improve finger collision geometry, especially concave areas; validate against
   the mesh envelope and real contact patches when hardware is available.
2. Improve lower-friction behavior and verify the nominal release fix without
   relaxing the placement threshold.
3. Add success-aware transitions, failure abort/retry behavior, and an explicit
   observation/action timing contract. Current diagnostic episodes continue after
   a failed hold so we can inspect what happened; this is not deployable control.
4. Add perception noise, relative XY/yaw errors and physically justified parameter
   variation; split complete episodes before training.
5. Integrate checked arm trajectories and robot-frame calibration.

The experiment uses MuJoCo's [simulation API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html)
for dynamics and contact-force queries. Physics setup and results remain subject
to the explicit proxy and parameter assumptions above.

## Refined collision model and release fix (current default)

The historical results above are retained in `contact_baseline_results.json`.
The new default is `--geometry decomposed`. It loads the committed CoACD parts
in `sim/collision/` (separate convex components, not a single hull per finger).
`--geometry mesh_hulls` retains the earlier coarse model as a failure control.
The refined recording displays these collision components directly.

Contact dimension is now 4: normal, sliding, and spin friction. The previous
3-dimensional contacts ignored the configured spin coefficient, letting the cube
rotate between the fingers despite two force-bearing contacts. Spin resistance
is controlled by `--torsional-friction` (default **0.001 m**, an assumed effective
contact radius; not a dimensionless coefficient or measured hardware value).
The geometry change and contact law change are both required in current checks.
Full success additionally checks final tilt below 5 degrees relative to a cube
face and final linear speed below 10 mm/s. The 3 mm position threshold is unchanged.

```sh
.venv/bin/python sim/evaluate_contact.py --output outputs/contact-evaluation.json
.venv/bin/python sim/contact_grasp.py --torsional-friction 0
```

[Refined development results](contact_refined_results.json): all five translated
nominal cases return within about **0.752 mm** and pass hold, placement, tilt,
settling, and table-contact checks. A 40 g cube also passes. Zero-spin and coarse
hull controls still drift; lower sliding friction (0.5) gives about 3.46 mm error
and remains a failure. The zero-sliding-friction case triggers solver warnings;
it is **numerically invalid**, not clean physical evidence about frictionless
behavior. The open-finger control fails cleanly. Warnings are captured in results,
and invalidate success; the warning callback is process-global, so run episodes
sequentially or in separate processes, not parallel threads.

Five translated scenes with exact pose knowledge are not a generalization or
reliability estimate. Calibration, collision-proxy validation, observation noise,
and success-aware control remain necessary before learning/deployment.

### Rebuild collision assets

```sh
.venv/bin/python -m pip install -r requirements-collision.txt
.venv/bin/python sim/decompose_fingers.py
```

[CoACD](https://github.com/SarahWeiii/CoACD) is pinned to 1.0.14. Generation uses
seed 0 and a 0.5 mm real-metric concavity target with automatic preprocessing.
This parameter is not a certified surface error bound. The manifest records the
parameters, part counts, source URDF/mesh hashes, and generated-asset hashes. The
simulator rejects stale/mismatched sources or assets. Regeneration may vary by
platform; commit reviewed assets so all teammates use the same geometry. Finger
mass is divided equally among parts (still 40 g total per finger), another
provisional inertia approximation. Finger-to-finger collision remains excluded.

Next: improve performance at lower friction, validate contact geometry, and add
failure-aware stages before collecting a training set. The nominal release issue
is resolved in this experimental configuration; real-robot grip fidelity is not.
