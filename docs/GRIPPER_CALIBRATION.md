# Gripper geometry and physical calibration

## Reproduce the model measurement

```sh
.venv/bin/python sim/calibrate_gripper.py --output docs/gripper_geometry.json
.venv/bin/python -m unittest discover -s sim -p 'test_*.py'
```

The report uses the supplied STL visuals and exact URDF transforms, including
the follower joint's mimic multiplier and offset. Both fingers are transformed
into `left_eef`. Every mesh triangle is clipped against the cube's X and Z extent;
the extrema of the resulting Y coordinates define a conservative inner gap.
This captures triangles crossing the slab even when none of their vertices are
inside it. It is an envelope measurement, not a collision/contact solver.

Measurements depend on cube depth. At the current cube centre `[0, 0, 0.004]` m
in `left_eef`, for a 25.4 mm cube:

| Joint angle | Inner envelope gap | Outer width in cube slab |
| --- | --- | --- |
| 0.15 rad | 24.77 mm | 50.16 mm |
| 0.185 rad | 30.58 mm | 55.97 mm |
| 0.19 rad (current animation) | 31.42 mm | 56.80 mm |
| 0.35 rad (open) | 58.67 mm | 87.14 mm |

A width-matching geometric candidate is **0.153774 rad**, with the cube shifted
to local Y **+1.101 mm**. It is not yet adopted by the animation or hardware:
matching two envelope extrema does not establish opposed contact patches or
frictional holding capacity. The nominal fingertip clearance above a supporting
table is only about 1.55 mm near this candidate, before calibration error or
compliance. Test grasp depth and table clearance together.

The earlier teammate claim of 25.4 mm at 0.185 rad does not match this particular
probe depth. It may use another contact zone. Preserve the probe definition when
comparing numbers; do not silently select the measurement that looks best.

## Physical calibration procedure (pending robot access)

1. Fix the robot base and identify actual encoder units, gripper command semantics,
   available force/current/contact feedback, and the stop/reset procedure.
2. With no cube, measure jaw gap at a marked contact depth for several small angle
   increments. Record commanded and measured angle, measured gap, contact depth,
   repeat direction (opening/closing), and repeatability. Compare against the mesh.
3. Using a separated cube, determine where both pads contact its sides. Record the
   cube centre in `left_eef`, observed jaw angle, and available feedback. Verify
   fingertip-to-table clearance; do not infer it from cube size alone.
4. Lift slightly, hold, place, and inspect for slip. Start with supervised tests;
   record failures as well as successes. Decide a measurable held-cube signal.
5. Record calibration date, robot identity, tool configuration, units, uncertainty,
   and valid operating range. Only then update the working grasp configuration.

Do not close past contact using a geometric angle estimate as a force command.
The proper command depends on the real gripper controller and feedback.

## Next simulation task

Build a minimal contact scene with gravity, a free cube, a supporting table, and
validated finger collision geometry. Test the candidate depth/angle, the open and
close sweep, and lift/hold without artificial attachment. Use the mesh report to
check collision proxy dimensions. Do not replace fingers with a single convex
hull spanning both jaws; that would fill the grasp opening. This report is not a
full-arm collision or magnetic-force model.
