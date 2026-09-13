"""Collect physically verified expert episodes, train BC, and evaluate closed loop.

Usage: python sim/train_pickup.py collect|train|evaluate --directory outputs/pickup
Test seeds are reserved before collection. Failures stay in the manifest, and only
successful expert episodes supply supervised training targets. No API calls.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from learning_env import CONTRACT, episode, scenario
from pickup_policy import Policy, fit

PROTOCOL = {
    'version': 1, 'train_seeds': list(range(64)), 'validation_seeds': list(range(1000,1016)),
    'test_seeds': list(range(2000,2032)),
    'selection': 'lowest validation imitation MSE; no test-data model selection',
    'pickup_acceptance': {'minimum_test_success_rate': .9, 'maximum_gap_below_expert': .1},
    'full_goal': 'image-derived multi-cube physical simulation and hardware verification remain separate gates',
    'distribution': {'xy_m': [-.03,.03], 'cube_yaw_rad': [-.15,.15], 'friction': [.5,1.],
        'mass_kg': [.022,.03], 'pose_bias_m_each_axis': [-.0004,.0004],
        'yaw_bias_rad': [-.015,.015], 'initial_tool_offset_m_each_axis': [-.005,.005]},
    'grasp_success': {'hold_seconds': 1., 'minimum_finger_force_n': .5,
        'minimum_lift_m': .04, 'maximum_lift_hold_slip_m': .002,
        'solver_warnings_allowed': 0, 'finger_table_penetration_limit_m': .0001},
}


def fingerprint():
    here = Path(__file__).resolve().parent
    names = ['learning_env.py', 'contact_grasp.py', 'foam_pad_config.json',
             'collision/manifest.json', 'chopped_urdf_v2/urdf/chopped_urdf_v2.urdf']
    return {name: hashlib.sha256((here/name).read_bytes()).hexdigest() for name in names}


def expert_worker(seed):
    return episode(scenario(seed))


def collect(directory, workers):
    directory.mkdir(parents=True, exist_ok=True)
    protocol_path = directory/'protocol.json'
    if protocol_path.exists() and json.loads(protocol_path.read_text()) != PROTOCOL:
        raise ValueError('existing run has a different frozen protocol; use a new directory')
    protocol_path.write_text(json.dumps(PROTOCOL, indent=2)+'\n')
    manifest = {'contract': CONTRACT, 'protocol': PROTOCOL, 'source_hashes': fingerprint(), 'episodes': []}
    for split in ('train','validation'):
        obs, actions, ids = [], [], []
        with ProcessPoolExecutor(max_workers=workers) as pool:
            seeds = PROTOCOL[split+'_seeds']
            for seed, (result, samples) in zip(seeds, pool.map(expert_worker, seeds)):
                manifest['episodes'].append({'split': split, **result})
                if result['success']:
                    obs.extend(s[0] for s in samples)
                    actions.extend(s[1] for s in samples)
                    ids.extend([seed]*len(samples))
                print(f'{split} {seed}: success={result["success"]} {result["reason"]}', flush=True)
        if not obs:
            raise RuntimeError(f'no physically successful {split} demonstrations')
        np.savez_compressed(directory/f'{split}.npz', observations=np.asarray(obs),
                            actions=np.asarray(actions), episode_ids=np.asarray(ids))
    manifest['dataset_hashes'] = {s: hashlib.sha256((directory/f'{s}.npz').read_bytes()).hexdigest()
                                  for s in ('train','validation')}
    (directory/'dataset.json').write_text(json.dumps(manifest, indent=2)+'\n')


def train(directory, epochs):
    manifest = json.loads((directory/'dataset.json').read_text())
    if manifest['contract'] != CONTRACT or manifest['source_hashes'] != fingerprint():
        raise ValueError('dataset physics/contract changed; regenerate demonstrations')
    for split in ('train','validation'):
        if hashlib.sha256((directory/f'{split}.npz').read_bytes()).hexdigest() != manifest['dataset_hashes'][split]:
            raise ValueError('dataset checksum mismatch')
    with np.load(directory/'train.npz', allow_pickle=False) as tr, np.load(directory/'validation.npz', allow_pickle=False) as va:
        if set(tr['episode_ids']) & set(va['episode_ids']):
            raise ValueError('episode split leakage')
        policy, history = fit(tr['observations'], tr['actions'], va['observations'], va['actions'], epochs=epochs)
        policy.save(directory/'policy.npz', {'algorithm': 'behavior_cloning_numpy_mlp_64_64',
            'seed': 7, 'epochs': epochs, 'source_hashes': manifest['source_hashes'],
            'dataset_hashes': manifest['dataset_hashes'], 'protocol': manifest['protocol'],
            'train_episode_ids': sorted(set(tr['episode_ids'].tolist())),
            'validation_episode_ids': sorted(set(va['episode_ids'].tolist())),
            'best_validation_mse': min(h['validation_mse'] for h in history)})
    (directory/'training.json').write_text(json.dumps(history, indent=2)+'\n')


def evaluate_worker(payload):
    seed, checkpoint = payload
    p = scenario(seed)
    expert, _ = episode(p)
    policy = Policy.load(checkpoint)
    learned, _ = episode(p, policy)
    return {'seed': seed, 'expert': expert, 'learned': learned}


def reserve_test(directory, start, count):
    """Freeze a fresh test set against the selected checkpoint before evaluation."""
    policy = Policy.load(directory/'policy.npz')
    if count < 32:
        raise ValueError('at least 32 fresh test episodes are required')
    seeds = list(range(start, start+count))
    excluded = set(policy.metadata['train_episode_ids']+policy.metadata['validation_episode_ids']
                   +policy.metadata['protocol']['test_seeds'])
    if set(seeds) & excluded:
        raise ValueError('fresh test overlaps training, validation, or prior test seeds')
    frozen = {'checkpoint_sha256': policy.metadata['checkpoint_sha256'],
              'source_hashes': fingerprint(), 'test_seeds': seeds,
              'pickup_acceptance': policy.metadata['protocol']['pickup_acceptance'],
              'reason': 'Original test seeds were inspected during prior checkpoint development; reserve fresh seeds before evaluating the frozen final checkpoint.'}
    with (directory/'fresh-test-protocol.json').open('x') as f:
        f.write(json.dumps(frozen, indent=2)+'\n')
    print(f'Reserved {count} fresh episodes against checkpoint {frozen["checkpoint_sha256"]}')


def evaluate(directory, workers, split, fresh=False):
    policy = Policy.load(directory/'policy.npz')
    if policy.metadata['source_hashes'] != fingerprint():
        raise ValueError('physics changed since training')
    protocol = policy.metadata['protocol']
    if fresh:
        if split != 'test':
            raise ValueError('fresh reservation is for test evaluation only')
        protocol = json.loads((directory/'fresh-test-protocol.json').read_text())
        if protocol['checkpoint_sha256'] != policy.metadata['checkpoint_sha256'] or protocol['source_hashes'] != fingerprint():
            raise ValueError('checkpoint or physics changed after fresh-test reservation')
    seeds = protocol[split+'_seeds']
    if split == 'test' and set(seeds) & set(policy.metadata['train_episode_ids']+policy.metadata['validation_episode_ids']):
        raise ValueError('test split leakage')
    runs = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for result in pool.map(evaluate_worker, [(s, str(directory/'policy.npz')) for s in seeds]):
            runs.append(result)
            print(f'{split} {result["seed"]}: expert={result["expert"]["success"]} learned={result["learned"]["success"]} {result["learned"]["reason"]}', flush=True)
    rates = {actor: sum(r[actor]['success'] for r in runs)/len(runs) for actor in ('expert','learned')}
    gate = protocol['pickup_acceptance']
    report = {'scope': 'simulated pickup only; no hardware or full-build claim', 'split': split,
              'checkpoint_sha256': policy.metadata['checkpoint_sha256'], 'success_rates': rates,
              'fresh_test': fresh, 'evaluation_protocol': protocol,
              'pickup_gate_passed': rates['learned'] >= gate['minimum_test_success_rate'] and
                                    rates['expert']-rates['learned'] <= gate['maximum_gap_below_expert'],
              'runs': runs}
    label = 'fresh-test' if fresh else split
    (directory/f'{label}-evaluation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('runs', 'evaluation_protocol')}, indent=2), flush=True)
    return report['pickup_gate_passed']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['collect','train','evaluate','reserve-test'])
    parser.add_argument('--directory', type=Path, default=Path('outputs/pickup-v1'))
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--epochs', type=int, default=150)
    parser.add_argument('--split', choices=['validation','test'], default='test')
    parser.add_argument('--fresh', action='store_true', help='evaluate the separately frozen fresh test protocol')
    parser.add_argument('--test-seed-start', type=int, default=7000)
    parser.add_argument('--test-count', type=int, default=32)
    args = parser.parse_args()
    if args.command == 'collect':
        collect(args.directory, args.workers)
    elif args.command == 'train':
        train(args.directory, args.epochs)
    elif args.command == 'reserve-test':
        reserve_test(args.directory, args.test_seed_start, args.test_count)
    else:
        if not evaluate(args.directory, args.workers, args.split, fresh=args.fresh):
            raise SystemExit(1)


if __name__ == '__main__':
    main()