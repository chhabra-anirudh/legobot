"""Reproduce development checks. Not held-out ML or hardware evaluation."""
import argparse
import json
from pathlib import Path
from contact_grasp import run_episode

CASES = [(f'translation_{seed}', {'seed': seed}) for seed in range(5)] + [
    ('old_off_center_pose', {'depth':1/3, 'grasp_x':0, 'geometry':'decomposed'}),
    ('open_control', {'close': False}),
    ('zero_sliding_friction', {'friction': 0}),
    ('zero_spin_friction', {'torsional_friction': 0}),
    ('bare_decomposed_control', {'geometry':'decomposed'}),
    ('coarse_hull_control', {'geometry': 'mesh_hulls'}),
    ('pad_control', {'geometry': 'pads'}),
    ('lower_friction', {'friction': .5}),
    ('heavier_cube', {'mass': .04}),
]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, default=Path('outputs/contact-evaluation.json'))
    args = p.parse_args()
    runs = []
    for name, params in CASES:
        result, _ = run_episode(**params)
        result['case'] = name
        runs.append(result)
        print(f"{name}: hold={result['hold_success']} full={result['success']} error={1000*result['placement_error_m']:.3f} mm")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({'note': 'Development checks; exact cube XY known to expert; parameters are uncalibrated. Includes expected-failure controls. Not a reliability estimate.', 'runs': runs}, indent=2)+'\n')


if __name__ == '__main__':
    main()
