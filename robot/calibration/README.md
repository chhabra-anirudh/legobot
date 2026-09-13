# Robot-specific calibration, copied off the hardware

Robot `bracketbot-0152`, left arm. These files are **backups of state that lives on
the robot**, where `~/bbapps` is not version controlled and other teams share the
machine. Re-pull them with `./robot/deploy.sh pull` after any teaching session.

## `ranges.calibration.json`

Copied verbatim from `~/bbos/bbos/daemons/arm_left/ranges.calibration.json`.
Per-joint motor-turn extremes. `cal_min`/`cal_max` are *named* extremes, not
ordered — take `min`/`max` per joint, as `build_bridge.joint_range` does.

`sim/robot_limits.json` is derived from this file (converted to URDF units and
intersected with the URDF's own limits). **If this file changes, regenerate
`sim/robot_limits.json`**, or the simulation will plan trajectories the robot
silently clamps.

Gripper (index 7) range is `[-0.379, 0]` turns, about 0 to 2.38 rad. URDF rad =
`-2*pi*turns` (`gripper_sign = -1`). Note the URDF caps that joint at 1.0 rad,
narrower than the real jaw.

## `taught_poses.json`

Appended by `teach_pose.py`, which is read-only with respect to the arm. Recorded
with torque off and the arm hand-guided.

| Entry | Recorded | What it actually is |
| --- | --- | --- |
| `smoke_test_current_pose` | 18:00 | Wherever the arm happened to be. Not a calibration. |
| `supply_orange` | 18:12 | **Not a supply pose.** Near-identical to the entry above, so the arm was not moved between them. Ignore. |
| `supply_orange` | 19:42 | **The useful one, and mislabelled.** Jaw hand-closed on a 25.4 mm cube: `left_left_gripper` = **0.2301 rad**. This is the source of `gripper_grasp_rad` in `sim/assembly_config.json`. |

The labels are wrong but the numbers are real, so they are kept verbatim rather
than rewritten. Label new poses accurately.

**Still missing: a `table_surface` pose.** Table height is currently a set value
(0.5 m), not a surveyed one. Hand-guide the tool until it touches the table, run
`uv run teach_pose.py table_surface`, then FK it on the workstation to get the
true root-frame z.
