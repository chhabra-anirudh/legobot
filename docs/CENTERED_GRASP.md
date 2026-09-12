# Centered soft-pad grasp and cube colors

## Latest defaults

The user confirmed the real fingers have a soft **foam-and-mesh** contact surface.
The previous rigid-only mesh pinched upper corners. The new `foam_pads` mode
adds compliant contact proxies over the unchanged decomposed fingers. This is a
soft-contact approximation, not a deformable foam/material simulation.

The target is now at half cube height (`--depth .5`), with EEF X shifted -18 mm to
align the actual contact patch rather than the nominal tool origin. Symmetric foam
pads use zero Y correction; bare fingers retain the measured +1.1 mm correction.
Moving the bare fingers lower alone was rejected because it caused table contact.

`sim/foam_pad_config.json` contains **unmeasured, provisional** pad dimensions:
20 mm width, 12 mm height, 4 mm thickness, and contact time constant 15 ms. The
reference pad orientation is defined at 0.18 rad. The gripper still commands
0.12 rad under the existing ±0.15 N m actuator limits; we did not increase squeeze
torque to hide a position problem. Contacts stop actual closure.

Results embed the full pad configuration for reproducibility. Calibrate actual
pad dimensions, attachment position, compression under load, friction, and motor
feedback before hardware use. A softer 20 ms exploratory setting exceeded the
new 2 mm slip gate; the chosen 15 ms setting is an experimental assumption, not a
measurement of this particular foam. Keep this sensitivity visible when training.

## Grip quality is measured

Result/trace schema is now **version 2**. Alongside the existing placement gate:

- Each finger must sustain at least **0.5 N** throughout hold.
- Maximum cube-centre motion relative to the actual EEF during lift/hold must stay
  below **2 mm**. This includes initial settling/compression, not only steady slip.
- Traces report normal forces and force-weighted contact centroids in the cube's
  coordinate frame. Optional cyan dots mark those centroids in replay.
- Tests check both centroids lie within 2 mm horizontally and 3 mm vertically of
  the side-face centres at the end of close.

[Current foam results](contact_foam_results.json): nominal runs maintain about
1.54 N per finger, 1.89 mm maximum relative motion, and ~0.043 mm final placement
error. Five translated cases and sliding friction 0.5 pass. A 40 g cube exceeds the
slip limit and **fails** despite returning near its starting position. Open fingers
fail. No-spin friction now passes; the corrected contact location no longer relies
on spin resistance for the nominal case. Perfect cube pose is still supplied;
these are development checks, not generalization or hardware reliability evidence.

The [bare centered comparison](contact_centered_results.json) improves width
alignment and stability but still contacts near the upper side edges. Use
`--geometry decomposed` to inspect that model. `mesh_hulls` and the old invented
`pads` mode remain diagnostic controls, not defaults.

## View the update

```sh
.venv/bin/python sim/contact_grasp.py --color blue --show-contacts \
  --output outputs/foam-centered --save outputs/foam-centered.rrd
.venv/bin/rerun outputs/foam-centered.rrd
```

Select **physics → Play**. `--show-contacts` adds cyan force-centroid markers;
`grip/normal_force_n` shows the two finger forces. Omit markers for a clean replay.
Select a stable cube color with `--color red|green|blue|yellow|orange|white`.
The cube no longer changes color to indicate a grasp. This is still an isolated
gripper scene; the full arm remains in the separate kinematic replay.

For the three-cube replay, `sim/assembly_config.json` now contains
`block_colors: ["red", "green", "blue"]`; held state appears in labels rather than
replacing the color. Run `.venv/bin/python sim/simulate_assembly.py`.

WSL browser fallback from the teammate's verified setup:

```sh
.venv/bin/rerun --serve-web --bind 127.0.0.1 --web-viewer-port 9090 outputs/foam-centered.rrd
```

Open the URL printed by Rerun. Existing recorded files do not change; regenerate
a recording with the updated script to see this grasp and the new colors.

## Repository updates included

Started from `origin/main` at 8394727: teammates had merged contact physics and
verified a fresh WSL environment. Added `.gitattributes` to preserve URDF LF bytes
on Windows, preventing the recorded collision-source hash mismatch without
weakening the guard. The original URDF and collision assets are unchanged.
