"""Offline collision asset generation; requires coacd==1.0.14."""
import hashlib
import json
from pathlib import Path
from importlib.metadata import version

import coacd
import numpy as np
from calibrate_gripper import URDF
from visualize_urdf import parse_urdf, load_stl, rpy_to_matrix

OUTPUT = Path(__file__).resolve().parent/'collision'
PARAMETERS = dict(threshold=.0005, real_metric=True, seed=0, preprocess_mode='auto',
                  resolution=2000, mcts_iterations=100, mcts_nodes=20, mcts_max_depth=3)


def main():
    links, joints, _ = parse_urdf(str(URDF))
    joints = {j['name']: j for j in joints}
    OUTPUT.mkdir(exist_ok=True)
    metadata = {'coacd_version': version('coacd'), 'parameters': PARAMETERS, 'urdf_sha256': hashlib.sha256(URDF.read_bytes()).hexdigest(), 'fingers': []}
    for i, name in enumerate(['left_left_gripper', 'left_right_gripper']):
        visual = links[joints[name]['child']][0]
        vertices, faces, _ = load_stl(visual['mesh'])
        vertices = vertices*visual['scale'] @ rpy_to_matrix(visual['rpy']).T+visual['xyz']
        unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
        parts = coacd.run_coacd(coacd.Mesh(unique, inverse[faces]), **PARAMETERS)
        payload = {f'{kind}{j}': array for j, part in enumerate(parts) for kind, array in zip(['vertices', 'faces'], part)}
        np.savez_compressed(OUTPUT/f'finger{i}.npz', **payload)
        metadata['fingers'].append({'joint':name, 'source_mesh':Path(visual['mesh']).name,
                                  'source_sha256':hashlib.sha256(Path(visual['mesh']).read_bytes()).hexdigest(),
                                  'asset_sha256':hashlib.sha256((OUTPUT/f'finger{i}.npz').read_bytes()).hexdigest(), 'parts': len(parts)})
    (OUTPUT/'manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    main()
