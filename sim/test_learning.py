"""Behavioral checks for physically labeled learning and checkpoint integrity."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

HAS_PHYSICS = importlib.util.find_spec('mujoco') is not None
if HAS_PHYSICS:
    import mujoco
    from learning_env import ContactScene, CONTRACT, DT, episode, scenario
    from pickup_policy import Policy, fit
    from train_pickup import PROTOCOL


@unittest.skipUnless(HAS_PHYSICS, 'install requirements-physics.txt')
class LearningEnvironmentTests(unittest.TestCase):
    def test_learned_build_uses_chassis_filtered_shared_placement(self):
        from build_learned import checked_plan
        import build_structure as planner
        path = Path(__file__).resolve().parent.parent/'compiler/examples/two-layer-bar.json'
        structure, plan = checked_plan(path)
        c = planner.load_config(planner.HERE/'assembly_config.json')
        self.assertGreater(plan['chassis_excluded_map_cells'], 0)
        self.assertEqual(len(plan['placements']), len(structure.voxels))
        (xmin, xmax), (ymin, ymax) = c['base_footprint_m']
        half = .0254/2
        for placement in plan['placements']:
            for field in ('pick_from_m', 'cube_centre_m'):
                x, y, _ = placement[field]
                self.assertFalse(xmin-half < x < xmax+half and ymin-half < y < ymax+half,
                                 f'{field} lies inside chassis: {placement[field]}')
        self.assertGreater(plan['kinematic_pose_count'], 0)

    def test_direct_image_compilation_preserves_image_provenance(self):
        import hashlib
        from build_learned import compile_image
        from schema import load, validate
        source = Path(__file__).resolve().parent.parent/'compiler/examples/images/cat.png'
        with tempfile.TemporaryDirectory() as folder:
            path = compile_image(source, Path(folder), [10, 8])
            structure = load(path)
            self.assertTrue(validate(structure).ok)
            self.assertEqual(structure.source, 'image_trace')
            self.assertIn(hashlib.sha256(source.read_bytes()).hexdigest(),
                          json.dumps(structure.provenance))
            self.assertAlmostEqual(structure.cube_size_m, .0254)

    def test_nominal_feedback_expert_physically_holds(self):
        result, samples = episode(scenario(8000, randomize=False))
        self.assertTrue(result['success'], result)
        self.assertGreaterEqual(result['hold_samples']*DT, 1.)
        self.assertLess(result['max_lift_hold_slip_m'], .002)
        self.assertTrue(samples)
        self.assertEqual(samples[0][0].shape, (len(CONTRACT['observations']),))

    def test_open_fingers_never_get_success_label(self):
        from learning_env import expert_action
        def open_actor(obs):
            action = expert_action(obs)
            action[4] = 1.
            return action
        result, _ = episode(scenario(8000, randomize=False), open_actor)
        self.assertFalse(result['success'])
        self.assertFalse(result['hold_contact_valid'])
        self.assertTrue(result['reason'])

    def test_no_motion_actor_times_out(self):
        result, _ = episode(scenario(8000, randomize=False), lambda obs: np.zeros(5))
        self.assertFalse(result['success'])
        self.assertEqual(result['reason'], 'approach_timeout')

    def test_every_construction_cube_is_free_under_gravity(self):
        with ContactScene([[0,0,.0127],[.1,0,.06]]) as scene:
            cube = scene.model.body('cube_1').id
            self.assertEqual(scene.model.joint('cube_1_free').type[0], mujoco.mjtJoint.mjJNT_FREE)
            for i in range(scene.model.neq):
                if scene.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
                    self.assertNotIn(cube, [scene.model.eq_obj1id[i], scene.model.eq_obj2id[i]])
            for _ in range(20):
                scene.step(np.zeros(5),record=False)
            self.assertLess(scene.data.body('cube_1').xpos[2], .02)

    def test_invalid_action_does_not_advance_physics_and_callback_restored(self):
        callback = mujoco.get_mju_user_warning()
        with ContactScene([[0,0,.0127]]) as scene:
            before = scene.data.qpos.copy()
            with self.assertRaises(ValueError):
                scene.step([0,0,np.nan,0,0])
            np.testing.assert_array_equal(before,scene.data.qpos)
            self.assertEqual(scene.data.time,0.)
        self.assertIs(mujoco.get_mju_user_warning(), callback)

    def test_contact_labels_are_excluded_and_splits_are_disjoint(self):
        for feature in CONTRACT['observations']:
            self.assertFalse(any(word in feature for word in ['force','contact','success','friction','mass']))
        tr,va,te=(set(PROTOCOL[s+'_seeds']) for s in ('train','validation','test'))
        self.assertFalse(tr & va or tr & te or va & te)

    def test_checkpoint_integrity_and_contract_are_enforced(self):
        dims=[len(CONTRACT['observations']),64,64,5]
        weights=[]
        for a,b in zip(dims[:-1],dims[1:]):
            weights += [np.zeros((a,b)),np.zeros(b)]
        policy=Policy(np.zeros(dims[0]),np.ones(dims[0]),weights)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'policy.npz'
            policy.save(path,{'algorithm':'test'})
            np.testing.assert_array_equal(Policy.load(path)(np.zeros(dims[0])),np.zeros(5))
            meta=json.loads(path.with_suffix('.json').read_text())
            meta['contract']['control_period_s']=.1
            path.with_suffix('.json').write_text(json.dumps(meta))
            with self.assertRaises(ValueError):
                Policy.load(path)
            policy.save(path,{'algorithm':'test'})
            with path.open('ab') as out:
                out.write(b'tampered')
            with self.assertRaises(ValueError):
                Policy.load(path)

    def test_neural_training_reduces_held_out_regression_error(self):
        rng=np.random.default_rng(42)
        x=rng.normal(size=(1024,14))
        y=np.tanh(x[:,:5]*.3)
        model,history=fit(x[:800],y[:800],x[800:],y[800:],epochs=20)
        prediction=model(x[800:])
        self.assertLess(np.mean((prediction-y[800:])**2), .015)
        self.assertLess(history[-1]['validation_mse'],history[0]['validation_mse'])


if __name__ == '__main__':
    unittest.main()