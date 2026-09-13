# Pickup training: completed simulation baseline

Verified 2026-09-13. One-inch (25.4 mm) cubes retained throughout.
The current checkpoint passes the defined simulated pickup gate. This does not
complete image-to-robot construction or prove hardware reliability.

## What was trained

A NumPy neural policy with two 64-unit hidden layers learns bounded tool XYZ,
yaw and jaw adjustments from contact-simulation demonstrations. The deterministic
controller still selects approach, close, lift and hold phases. Policy inputs are
relative target/tool state, jaw state and phase; contact forces and success labels
are excluded. Simulated cube pose supplies the target; real perception remains a
separate integration task. The policy does not learn image interpretation or
placement, and this is behavior cloning with corrective rollouts, not RL.

Training uses 128 episodes (64 expert demonstrations and 64 successful corrective
rollouts), totaling 18,724 state/action pairs. There are 16 separate validation
episodes (2,329 pairs). The final fit ran 250 epochs and selected the lowest
validation imitation error. Closed-loop validation passed 16/16.

Randomization covers cube XY within +/-30 mm, yaw within +/-0.15 rad, friction
0.5–1.0, mass 22–30 g, pose bias +/-0.4 mm per axis, yaw bias +/-0.015 rad and
initial tool offset +/-5 mm per axis. The gripper and cubes have contact dynamics;
cubes are free bodies, with no attachment constraint. Full-arm dynamics and
magnetic forces are absent. Provisional learning jaw commands are distinct from
hardware/planner calibration; do not transfer them directly to the robot.

## Frozen evaluation

Original test seeds had already been examined during an earlier checkpoint's
development. After freezing this final checkpoint, a fresh 32-episode test set
(seeds 7000–7031) was reserved before evaluation. Training, validation and original
test seeds are excluded. The fresh protocol binds checkpoint and physics hashes;
the reservation command refuses to overwrite an existing reservation.

Learned policy: **32/32 successful**. Scripted expert: **32/32 successful** on the
same scenarios. The original gate (at least 90% success, no more than 10 percentage
points below the expert) passes. This finite test does not establish 100% general
reliability. Detailed per-episode evidence: `pickup_evaluation_results.json`.
Success requires a lift above 40 mm, a full one-second hold, at least 0.5 N at each
finger throughout the hold, less than 2 mm lift/hold slip, no solver warnings and
no excessive finger-table penetration. Open-finger and no-motion negative controls
fail, as expected. Ten learning/integration regression tests pass.

Checkpoint: `outputs/pickup-v2/policy.npz` (ignored local artifact).
SHA256: `1631c70a7472f8a5b918a1e23a718abcf03f7859fe13fade357f5a4d7d01b047`.
Dataset, protocol and model metadata are stored beside it. No model weights,
datasets or recordings are committed.

## What works beyond pickup

The picture compiler feeds checked one-inch structures into the learned-build
runner, sharing the main planner's chassis exclusion and quarter-turn selection.
A four-cube two-layer fixture completes in contact simulation with learned pickup
and scripted placement; maximum final error is 0.405 mm. Larger image-derived
builds fail during descent beside existing cubes: 9/17 and 15/36 completed. Their
pickups passed. These are development runs, not held-out construction evaluation.

## Next work, in order

1. Develop contact-aware placement: identify insertion collisions, correct cube
   pose during descent, detect a wedged/slipping cube before release, and implement
   recovery. Keep the original 3 mm position and 5 degree tilt success limits.
2. Collect successful placement/recovery demonstrations and train the remaining
   local control needed for reliable construction. Test whole image-derived builds
   on unseen images, layouts and physical variations, not just isolated pickups.
3. Execute the actual arm trajectory in contact simulation, including relevant
   arm/table/chassis collisions and actuator limits. Resolve the existing carried
   tilt, facing-selection and staging-order planner test failures.
4. Connect calibrated real observations, held-cube detection and the robot bridge.
   Verify the supplied hardware IK model/frames before use; the native IK library
   is ARM64 and its wrapper references a different robot model.
5. Run supervised hardware pickup/placement trials, then repeated complete builds.
   The plan calls for at least 18/20 successful single-cube cycles before advancing,
   followed by repeated multi-cube construction. Hardware access and calibration
   remain prerequisites; simulation results do not fulfill these gates.

Evaluation command for this frozen checkpoint:
```
OPENBLAS_NUM_THREADS=1 .venv/bin/python sim/train_pickup.py evaluate --directory outputs/pickup-v2 --fresh --workers 4
```
