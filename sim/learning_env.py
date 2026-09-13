"""Step-based gripper contact environment for behavior cloning and construction.

Only the gripper is dynamic here; full-arm kinematics/collision integration is
separate. Cubes are free under gravity throughout pickup, transport and release.
Policy observations exclude contacts, forces, success flags, and physics parameters.
"""
from contextlib import AbstractContextManager
import json
from pathlib import Path

import mujoco
import numpy as np
from contact_grasp import make_model
from tool_frame import rotation_for

DT = .02
ACTION_SCALE = np.array([.0024, .0024, .0024, .012, .012])
STAGES = ('approach', 'close', 'lift', 'hold')
OBSERVATIONS = (
    'goal_minus_command_x_over_60mm', 'goal_minus_command_y_over_60mm',
    'goal_minus_command_z_over_60mm', 'goal_minus_command_yaw_over_pi',
    'eef_minus_command_x_over_10mm', 'eef_minus_command_y_over_10mm',
    'eef_minus_command_z_over_10mm', 'jaw0_over_0.35rad', 'jaw1_over_0.35rad',
    'jaw_command_over_0.35rad', 'stage_approach', 'stage_close', 'stage_lift', 'stage_hold')
CONTRACT = {
    'version': 1, 'frame': 'experiment_table', 'control_period_s': DT,
    'observations': list(OBSERVATIONS),
    'actions': ['delta_x_m', 'delta_y_m', 'delta_z_m', 'delta_yaw_rad', 'delta_jaw_command_rad'],
    'action_scale': ACTION_SCALE.tolist(), 'action_bounds_normalized': [-1., 1.],
    'joint_open_rad': .26, 'joint_close_rad': .12,
    'goal_source': 'observed cube pose plus explicit tool grasp offset and requested lift',
    'policy_scope': 'goal-conditioned pickup feedback; phase supervisor and placement remain scripted',
    'contacts': 'simulation-only outcome labels, never policy input',
    'hardware_ready': False,
}


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def expert_action(observation):
    """Analytic feedback expert to imitate, not called by a learned actor."""
    obs = np.asarray(observation)
    a = np.empty(5)
    a[:3] = np.tanh(obs[:3]*.06/.004)
    a[3] = np.tanh(obs[3]*np.pi/.10)
    target = .26 if obs[10] > .5 else .12
    a[4] = np.clip((target-obs[9]*.35)/ACTION_SCALE[4], -1, 1)
    return a


class ContactScene(AbstractContextManager):
    def __init__(self, positions, *, friction=.8, mass=.03, yaw=0.):
        positions = np.asarray(positions, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 3 or not len(positions):
            raise ValueError('nonempty Nx3 cube positions required')
        if not np.all(np.isfinite(positions)):
            raise ValueError('finite cube positions required')
        self.model = make_model(friction=friction, mass=mass, extra_cubes=positions[1:])
        self.data = mujoco.MjData(self.model)
        self.names = ['cube']+[f'cube_{i}' for i in range(1, len(positions))]
        self.finger_ids = {self.model.body(f'finger{i}').id: i for i in range(2)}
        for name, pos in zip(self.names, positions):
            adr = self.model.joint(name+'_free').qposadr[0]
            self.data.qpos[adr:adr+3] = pos
            self.data.qpos[adr+3:adr+7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
        self.selected = 0
        self.yaw = float(yaw)
        self.command = positions[0] - rotation_for(yaw) @ np.array([.018, 0, .004]) + [0, 0, .06]
        self.jaw = .26
        self.data.ctrl[:] = self.jaw
        for name in ('jaw0', 'jaw1'):
            self.data.qpos[self.model.joint(name).qposadr[0]] = self.jaw
        adr = self.model.joint('eef_free').qposadr[0]
        self.data.qpos[adr:adr+3] = self.command
        self._set_mocap()
        self.data.qpos[adr+3:adr+7] = self.data.mocap_quat[0]
        mujoco.mj_forward(self.model, self.data)
        self.warnings = {}
        self.previous_warning = mujoco.get_mju_user_warning()
        mujoco.set_mju_user_warning(self._warning)
        self.bad_table = False
        self.frames = []
        self.active_stage = 'settle'
        self.forces = np.zeros(2)
        self.min_forces = np.zeros(2)
        self.min_height = 0.
        self.max_step_slip = 0.
        self.slip_reference = None
        self.max_slip = 0.
        self.last_delta = np.zeros(5)
        self.closed = False

    def _warning(self, message):
        self.warnings[message] = self.warnings.get(message, 0)+1

    def __exit__(self, *args):
        if not self.closed:
            mujoco.set_mju_user_warning(self.previous_warning)
            self.closed = True

    @property
    def cube(self):
        return self.data.body(self.names[self.selected])

    def _set_mocap(self):
        self.data.mocap_pos[0] = self.command
        mujoco.mju_mat2Quat(self.data.mocap_quat[0], rotation_for(self.yaw).ravel())

    def observe(self, goal, yaw, stage):
        if stage not in STAGES:
            raise ValueError('unknown pickup stage')
        fingers = [self.data.qpos[self.model.joint(f'jaw{i}').qposadr[0]] for i in range(2)]
        return np.r_[(np.asarray(goal)-self.command)/.06, wrap(yaw-self.yaw)/np.pi,
                     (self.data.body('eef').xpos-self.command)/.01,
                     np.asarray(fingers)/.35, self.jaw/.35,
                     [float(stage == s) for s in STAGES]].astype(np.float32)

    def relative_cube(self):
        return self.data.body('eef').xmat.reshape(3, 3).T @ (self.cube.xpos-self.data.body('eef').xpos)

    def stable(self):
        return not self.warnings and np.all(np.isfinite(self.data.qpos)) and not any(w.number for w in self.data.warning)

    def step(self, action, stage='motion', record=True):
        action = np.asarray(action, dtype=float)
        if action.shape != (5,) or not np.all(np.isfinite(action)):
            raise ValueError('action must be five finite normalized values')
        delta = np.clip(action, -1, 1)*ACTION_SCALE
        # Bounded acceleration avoids impulsive tool motion at phase boundaries.
        slew = np.array([.00072, .00072, .00072, .0006, .012])
        delta = self.last_delta + np.clip(delta-self.last_delta, -slew, slew)
        self.last_delta = delta.copy()
        # Bounds enforced independently of the policy; no action teleports a cube.
        start, start_yaw, start_jaw = self.command.copy(), self.yaw, self.jaw
        target = start + delta[:3]
        if target[2] < .010 or target[2] > .5 or np.any(np.abs(target[:2]) > 1.):
            raise ValueError('tool action leaves experiment workspace')
        target_jaw = float(np.clip(start_jaw+delta[4], .12, .26))
        self.min_forces = np.full(2, np.inf)
        self.min_height = np.inf
        self.max_step_slip = 0.
        count = round(DT/self.model.opt.timestep)
        for k in range(count):
            fraction = (k+1)/count
            self.command = start + (target-start)*fraction
            self.yaw = start_yaw + delta[3]*fraction
            self.jaw = start_jaw+(target_jaw-start_jaw)*fraction
            self._set_mocap()
            self.data.ctrl[:] = self.jaw
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.forces[:] = 0
            cube_id = self.cube.id
            table = self.model.geom('table').id
            for j, contact in enumerate(self.data.contact):
                b1, b2 = (int(self.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2))
                if table in (contact.geom1, contact.geom2) and (b1 in self.finger_ids or b2 in self.finger_ids) and contact.dist < -.0001:
                    self.bad_table = True
                if cube_id not in (b1, b2) or contact.dist > 0:
                    continue
                finger = b2 if b1 == cube_id else b1
                if finger in self.finger_ids:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, j, force)
                    self.forces[self.finger_ids[finger]] += max(0., force[0])
            self.min_forces = np.minimum(self.min_forces, self.forces)
            self.min_height = min(self.min_height, float(self.cube.xpos[2]))
            if self.slip_reference is not None:
                slip = float(np.linalg.norm(self.relative_cube()-self.slip_reference))
                self.max_step_slip = max(self.max_step_slip, slip)
                self.max_slip = max(self.max_slip, slip)
        if record:
            self.frames.append({'time_s': float(self.data.time), 'stage': stage,
                'selected_cube': self.selected, 'command_position_m': self.command.tolist(),
                'command_yaw_rad': self.yaw, 'command_jaw_rad': self.jaw,
                'cube_positions_m': [self.data.body(n).xpos.tolist() for n in self.names],
                'cube_quaternions_xyzw': [np.roll(self.data.body(n).xquat, -1).tolist() for n in self.names],
                'finger_poses': [{'position_m': self.data.body(f'finger{i}').xpos.tolist(),
                                 'quaternion_wxyz': self.data.body(f'finger{i}').xquat.tolist()} for i in range(2)],
                'normal_forces_n': self.forces.tolist()})

    def move(self, goal, yaw, jaw, *, stage='motion', timeout=12.):
        """Bounded scripted transport; learned pickup uses run_pick instead."""
        for _ in range(round(timeout/DT)):
            delta = np.r_[np.tanh((np.asarray(goal)-self.command)/.008),
                          np.tanh(wrap(yaw-self.yaw)/.1), np.clip((jaw-self.jaw)/.012, -1, 1)]
            self.step(delta, stage)
            if not self.stable():
                return False
            if np.linalg.norm(np.asarray(goal)-self.command) < .0002 and abs(wrap(yaw-self.yaw)) < .001 and abs(jaw-self.jaw) < .001:
                return True
        return False


def run_pick(scene, actor=expert_action, *, observed_position=None, observed_yaw=0.):
    """Learned low-level actions; deterministic phase sequencing and physical gates.

    The observation is frozen when close begins. Cube pose comes from a simulated
    pose sensor (caller adds noise); real deployment needs a calibrated source.
    No contact forces or privileged success labels reach the actor.
    """
    position = scene.cube.xpos.copy() if observed_position is None else np.asarray(observed_position)
    grasp = position - rotation_for(observed_yaw) @ np.array([.018, 0., .004])
    hover = grasp + [0, 0, .06]
    samples = []
    hold_valid = True
    hold_count = 0
    reason = ''
    scene.slip_reference = None
    scene.max_slip = 0.
    scene.bad_table = False
    min_force = np.full(2, np.inf)
    for stage, goal, min_time, timeout in [('approach', grasp, .1, 4.),
            ('close', grasp, .8, 1.4), ('lift', hover, .3, 4.), ('hold', hover, 1., 1.2)]:
        if stage == 'lift':
            scene.slip_reference = scene.relative_cube().copy()
        reached = False
        for i in range(round(timeout/DT)):
            obs = scene.observe(goal, observed_yaw, stage)
            action = np.asarray(actor(obs), dtype=float)
            samples.append((obs, expert_action(obs)))
            scene.step(action, stage)
            if not scene.stable():
                reason = 'solver_warning_or_nonfinite_state'
                break
            if scene.bad_table:
                reason = 'finger_table_collision'
                break
            if stage == 'hold':
                hold_count += 1
                min_force = np.minimum(min_force, scene.min_forces)
                hold_valid &= bool(scene.min_height > position[2]+.04 and np.min(scene.min_forces) >= .5)
            elapsed = (i+1)*DT
            if stage == 'lift' and elapsed > .8 and np.linalg.norm(scene.relative_cube()-scene.slip_reference) > .01:
                reason = 'dropped_or_missed_cube'
                break
            at_goal = np.linalg.norm(np.asarray(goal)-scene.command) < .0004 and abs(wrap(observed_yaw-scene.yaw)) < .005
            if elapsed >= min_time and at_goal:
                reached = True
                break
        if reason:
            break
        if not reached:
            reason = stage+'_timeout'
            break
    success = bool(not reason and hold_count >= round(1./DT) and hold_valid and scene.max_slip < .002)
    if not success and not reason:
        reason = 'insufficient_hold_contact_or_excessive_slip'
    return {'success': success, 'reason': reason, 'hold_samples': hold_count,
            'hold_contact_valid': bool(hold_valid and hold_count),
            'min_hold_normal_forces_n': np.where(np.isfinite(min_force), min_force, 0).tolist(),
            'max_lift_hold_slip_m': scene.max_slip, 'solver_warnings': dict(scene.warnings)}, samples


def scenario(seed, *, randomize=True):
    rng = np.random.default_rng(seed)
    return {'seed': seed, 'position_m': [*rng.uniform(-.03, .03, 2), .0127],
            'cube_yaw_rad': float(rng.uniform(-.15, .15)) if randomize else 0.,
            'friction': float(rng.uniform(.5, 1.)) if randomize else .8,
            'mass_kg': float(rng.uniform(.022, .03)) if randomize else .03,
            'observation_bias_m': rng.uniform(-.0004, .0004, 3).tolist() if randomize else [0., 0., 0.],
            'observation_yaw_bias_rad': float(rng.uniform(-.015, .015)) if randomize else 0.,
            'initial_tool_offset_m': rng.uniform(-.005, .005, 3).tolist() if randomize else [0., 0., 0.]}


def episode(parameters, actor=expert_action, *, return_frames=False):
    p = parameters
    with ContactScene([p['position_m']], friction=p['friction'], mass=p['mass_kg'], yaw=p['cube_yaw_rad']) as scene:
        # Initial tool perturbation is applied only at reset, before simulation.
        offset = np.asarray(p['initial_tool_offset_m'])
        scene.command += offset
        adr = scene.model.joint('eef_free').qposadr[0]
        scene.data.qpos[adr:adr+3] += offset
        scene._set_mocap()
        for _ in range(round(.3/DT)):
            scene.step(np.zeros(5), 'settle')
        observed = scene.cube.xpos.copy()+p['observation_bias_m']
        result, samples = run_pick(scene, actor, observed_position=observed,
                                  observed_yaw=p['cube_yaw_rad']+p['observation_yaw_bias_rad'])
        result.update(parameters=p, controller='scripted_expert' if actor is expert_action else 'learned',
                      simulation_scope='isolated_dynamic_gripper', duration_s=float(scene.data.time))
        if return_frames:
            return result, samples, scene.frames
        return result, samples


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--nominal', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result, samples = episode(scenario(args.seed, randomize=not args.nominal))
    print(json.dumps(result, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())