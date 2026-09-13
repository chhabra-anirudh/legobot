"""Small NumPy MLP trained by supervised behavior cloning; no expert at inference."""
import hashlib
import json
from pathlib import Path
import numpy as np
from learning_env import CONTRACT


class Policy:
    def __init__(self, mean, scale, weights):
        self.mean, self.scale = np.asarray(mean), np.asarray(scale)
        self.weights = [np.asarray(w) for w in weights]
        self.metadata = {}

    def __call__(self, observation):
        x = np.asarray(observation, dtype=np.float64)
        if x.shape[-1] != len(self.mean) or not np.all(np.isfinite(x)):
            raise ValueError('policy observation violates contract')
        h = (x-self.mean)/self.scale
        for i in range(0, len(self.weights), 2):
            h = h @ self.weights[i] + self.weights[i+1]
            if i < len(self.weights)-2:
                h = np.tanh(h)
        return np.clip(h, -1., 1.)

    def save(self, path, metadata):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, mean=self.mean, scale=self.scale,
                            **{f'w{i}': w for i, w in enumerate(self.weights)})
        metadata = {**metadata, 'contract': CONTRACT, 'checkpoint_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        path.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')

    @classmethod
    def load(cls, path):
        path = Path(path)
        meta = json.loads(path.with_suffix('.json').read_text())
        if meta['contract'] != CONTRACT:
            raise ValueError('checkpoint observation/action contract mismatch')
        if meta['checkpoint_sha256'] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('checkpoint checksum mismatch')
        with np.load(path, allow_pickle=False) as f:
            result = cls(f['mean'], f['scale'], [f[f'w{i}'] for i in range(6)])
        result.metadata = meta
        return result


def fit(train_x, train_y, valid_x, valid_y, *, epochs=150, seed=7, batch_size=256):
    """Two tanh hidden layers; Adam, with checkpoint selection on validation MSE."""
    rng = np.random.default_rng(seed)
    mean = train_x.mean(axis=0)
    scale = np.maximum(train_x.std(axis=0), .002)
    x, vx = (train_x-mean)/scale, (valid_x-mean)/scale
    sizes = [x.shape[1], 64, 64, train_y.shape[1]]
    weights = []
    for a, b in zip(sizes[:-1], sizes[1:]):
        weights += [rng.normal(0, np.sqrt(2/(a+b)), (a,b)), np.zeros(b)]
    moments = [np.zeros_like(w) for w in weights]
    variances = [np.zeros_like(w) for w in weights]
    best, best_loss, history, step = None, np.inf, [], 0
    for epoch in range(epochs):
        permutation = rng.permutation(len(x))
        for start in range(0, len(x), batch_size):
            ix = permutation[start:start+batch_size]
            layers = [x[ix]]
            for i in range(0, 6, 2):
                h = layers[-1] @ weights[i] + weights[i+1]
                layers.append(np.tanh(h) if i < 4 else h)
            grad = 2*(layers[-1]-train_y[ix])/train_y[ix].size
            grads = [None]*6
            for i in (4, 2, 0):
                grads[i] = layers[i//2].T @ grad
                grads[i+1] = grad.sum(axis=0)
                grad = grad @ weights[i].T
                if i:
                    grad *= 1-layers[i//2]**2
            step += 1
            for i in range(6):
                moments[i] = .9*moments[i]+.1*grads[i]
                variances[i] = .999*variances[i]+.001*grads[i]**2
                weights[i] -= .001*(moments[i]/(1-.9**step))/(np.sqrt(variances[i]/(1-.999**step))+1e-8)
        pred = vx
        for i in range(0, 6, 2):
            pred = pred @ weights[i]+weights[i+1]
            if i < 4:
                pred = np.tanh(pred)
        loss = float(np.mean((pred-valid_y)**2))
        history.append({'epoch': epoch+1, 'validation_mse': loss})
        if loss < best_loss:
            best_loss, best = loss, [w.copy() for w in weights]
        if (epoch+1) % 25 == 0:
            print(f'epoch {epoch+1}: validation MSE {loss:.6g}', flush=True)
    return Policy(mean, scale, best), history