"""DAgger-style corrective demonstrations on training scenes only.

A mixture of the learned actor and expert visits off-demonstration states. The
expert labels those states; only episodes whose physical hold succeeds enter the
training set. Every attempted correction, including failures, remains recorded.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import numpy as np
from learning_env import episode, expert_action, scenario
from pickup_policy import Policy


def worker(payload):
    seed, checkpoint, beta = payload
    policy = Policy.load(checkpoint)
    rng = np.random.default_rng(seed+100000)
    def actor(obs):
        return expert_action(obs) if rng.random() < beta else policy(obs)
    result, samples = episode(scenario(seed), actor)
    result['controller'] = 'expert_policy_mixture'
    result['expert_probability'] = beta
    result['label_policy_sha256'] = policy.metadata['checkpoint_sha256']
    return result, samples


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, default=Path('outputs/pickup-v1'))
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--beta', type=float, default=.5)
    p.add_argument('--round', type=int, default=1)
    a = p.parse_args()
    if not 0 < a.beta <= 1 or a.round < 1:
        p.error('beta in (0,1], positive round required')
    directory = a.directory
    manifest = json.loads((directory/'dataset.json').read_text())
    checkpoint = directory/'policy.npz'
    seeds = list(range(3000+a.round*100, 3000+a.round*100+64))
    forbidden = set(manifest['protocol']['validation_seeds']+manifest['protocol']['test_seeds'])
    if forbidden & set(seeds):
        raise ValueError('correction episodes overlap held-out split')
    with np.load(directory/'train.npz', allow_pickle=False) as f:
        obs, actions, ids = f['observations'].tolist(), f['actions'].tolist(), f['episode_ids'].tolist()
    if set(ids) & set(seeds):
        raise ValueError('correction round already collected')
    successes = 0
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for seed, (result, samples) in zip(seeds, pool.map(worker, [(s,str(checkpoint),a.beta) for s in seeds])):
            manifest['episodes'].append({'split':'train', 'correction_round':a.round, **result})
            print(f'correction {seed}: {result["success"]} {result["reason"]}', flush=True)
            if result['success']:
                successes += 1
                obs.extend(s[0].tolist() for s in samples)
                actions.extend(s[1].tolist() for s in samples)
                ids.extend([seed]*len(samples))
    np.savez_compressed(directory/'train.npz', observations=np.asarray(obs), actions=np.asarray(actions), episode_ids=np.asarray(ids))
    manifest['dataset_hashes']['train'] = hashlib.sha256((directory/'train.npz').read_bytes()).hexdigest()
    (directory/'dataset.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'{successes}/{len(seeds)} successful correction episodes', flush=True)


if __name__ == '__main__':
    main()