# chopped_urdf_v2

Self-contained copy of the BracketBot `chopped_urdf_v2` model: the URDF, every
mesh it references, and a standalone Rerun viewer.

    chopped_urdf_v2/
      urdf/chopped_urdf_v2.urdf       the model (54 links, 53 joints)
      meshes/*.stl                    all 50 referenced meshes
      package.xml                     so package:// and $(find ...) resolve
      launch/chopped_urdf_v2.launch   RViz + robot_state_publisher (ROS 1)
      manifest.json, draco/*.glb      Draco-compressed meshes for web viewers
    visualize_urdf.py, view_urdf.sh   Rerun viewer (no ROS needed)
    pyproject.toml                    deps for the viewer

The URDF refers to its meshes as `package://chopped_urdf_v2/meshes/<name>.stl`,
so keep the `chopped_urdf_v2/` directory intact — loaders resolve those paths
relative to the package root.

## Load it in Rerun (no ROS)

    ./view_urdf.sh --urdf chopped_urdf_v2/urdf/chopped_urdf_v2.urdf

First run provisions `rerun-sdk` and `numpy` into `./.venv` (via `uv` if
installed, otherwise the system `python3`). This opens the Rerun viewer plus a
browser slider panel for posing the joints. Other modes:

    ./view_urdf.sh --urdf ... --no-sliders          # viewer only, zero pose
    ./view_urdf.sh --urdf ... --list-joints         # print the movable joints
    ./view_urdf.sh --urdf ... --joint lj2=0.8       # one-shot pose (rad / m)
    ./view_urdf.sh --urdf ... --save bot.rrd        # headless recording

## Load it in ROS 1

Drop `chopped_urdf_v2/` into a catkin workspace's `src/`, build, then:

    roslaunch chopped_urdf_v2 chopped_urdf_v2.launch

## The model

Root link is `root`, at the wheel-axle midpoint with the tyres on z=0.

18 movable joints:

| Joint | Type | Range |
| --- | --- | --- |
| `rj0` / `lj0` | prismatic | -1.03044 .. 0 m (mast carriage) |
| `rj1`-`rj6` / `lj1`-`lj6` | revolute | ±2.094395 rad |
| `right_left_gripper`, `right_right_gripper` | revolute | 0 .. 1 rad |
| `left_left_gripper`, `left_right_gripper` | revolute | 0 .. 1 rad |

Each arm's second gripper joint `mimic`s the first, so a gripper is one DOF.
Every joint axis is exactly `(0,0,1)` — this was prepared for QP IK, where a raw
Onshape export snapped to the nearest principal axis puts FK ~27 cm out with no
warning. For IK the arm base link is `arm_base` and the end effectors are
`right_eef` / `left_eef`.

## Magnetic block assembly: first simulation

This prototype uses the actual left-arm URDF to stack three 1 inch (25.4 mm)
cubes. It is a **kinematic simulation**, not a physics simulation: blocks attach
ideally to a virtual tool point, and remain fixed after release. Both simulated fingers open and close through the URDF mimic relationship.
Finger angles are provisional, not a contact-derived grip. There is no collision checking, gravity, magnetic force,
or hardware connection. Successful IK does not establish hardware safety.

From this directory, using the existing viewer environment:

```sh
.venv/bin/python simulate_assembly.py --check
.venv/bin/python simulate_assembly.py
# Or make a recording for later playback:
.venv/bin/python simulate_assembly.py --save /tmp/assembly.rrd
.venv/bin/rerun /tmp/assembly.rrd
# Numerical regression checks:
.venv/bin/python -m unittest test_assembly.py
```

If the environment is missing, run `./view_urdf.sh --list-joints --urdf
chopped_urdf_v2/urdf/chopped_urdf_v2.urdf` once to provision it. In Rerun, select
the `simulation` timeline and press Play. Cubes retain their configured colors; labels identify the held block. The status log names each operation. The demo
starts at an already solved approach pose, not the robot's real startup pose.
Playback timing is illustrative and does not enforce motor speed/acceleration limits.

Edit `assembly_config.json` for dimensions and workspace placement. All values
use metres in the URDF root frame; Z is up. `supply_xy_m` describes block centres
on the table. `stack_xy_m` describes the common centre of the final stack.
`tool_grasp_point_m` locates the desired block centre in the `left_eef`
coordinate frame. The desired tool position is `block_position - R @ grasp_point`,
where `R` rotates tool coordinates into the root frame. The old 40 mm offset above
the cube has been removed because the URDF end-effector is already near the tips.
The cube dimensions are user-confirmed; the table location, tool point and finger
angles still need calibration. `gripper_open_rad` and `gripper_grasp_rad` are
provisional joint angles (0.35 and 0.19 radians), not measured jaw widths.
`gripper_motion_seconds` controls the opening/closing animation duration. Keep supply positions separated by more than the block width.
Changed geometry can make the sequence unreachable; `--check` reports IK failure.

### How to learn from the code

1. Read `Arm.fk`: forward kinematics maps joint values to the tool pose by
   composing the URDF transforms from the base to `left_eef`.
2. Read `Arm.solve`: inverse kinematics makes small joint adjustments to reduce
   tool position and orientation error. A numerical Jacobian describes how each
   joint changes the tool pose. Damping stabilizes the solve and joint limits
   bound its results. This local solver can fail even when another solution exists.
3. Read `plan`: approach → descend → close fingers → attach → lift → transfer → place
   → release → open fingers → retreat. Cartesian waypoints are solved sequentially using the previous
   solution as the next seed. The tool is held approximately downward.
4. Read `render`: time-indexed joint transforms animate the existing robot meshes.

Tests check joint limits, orientation, attachment behavior, stationary released
blocks, finger sequencing, final placements within 1 mm, and rejection of an unreachable target.
That tolerance is numerical, not a claim about physical robot precision.

On attachment, the actual tool-to-block offset is captured using `R.T @
(block_position - tool_position)`. The carried centre then follows `tool_position
+ R @ offset`, avoiding a position jump at pickup. Cubes remain axis-aligned in
the display; block rotational dynamics are not modeled. Individual IK poses have
a 0.5 mm position tolerance; pickup and placement errors can accumulate, hence
the 1 mm final-stack test tolerance. Release still fixes a block in space.

Next milestones: calibrate the tool transform and jaw width versus angle; identify the real robot
control API; add collision/contact physics and a calibrated grasp model; model
magnetic polarity and connection behavior; then validate slow single-block moves
with feedback on the physical robot. The assembly sequence can inform a future
hardware backend, but these simulated trajectories are not hardware commands.

## Shared repository environment

The root README provisions `../.venv` when working inside `sim/`. The commands
above use a `sim/.venv` environment provisioned by `view_urdf.sh`; either works.
Prefer the root README commands for team development.
