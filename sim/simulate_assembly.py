#!/usr/bin/env python3
"""Kinematic assembly prototype. All positions are root-frame metres.

No hardware commands, collision checking, or magnetic/contact physics.
"""
import argparse
import json
from pathlib import Path
import os
import sys

import numpy as np
import rerun as rr
from visualize_urdf import parse_urdf, joint_transform, log_robot

HERE = Path(__file__).resolve().parent
URDF = HERE / 'chopped_urdf_v2/urdf/chopped_urdf_v2.urdf'


class Arm:
    """Forward kinematics plus joint-limited damped least-squares IK."""
    def __init__(self):
        _, joints, _ = parse_urdf(str(URDF))
        by_child = {j['child']: j for j in joints}
        chain, link = [], 'left_eef'
        while link in by_child:
            joint = by_child[link]
            chain.append(joint)
            link = joint['parent']
        self.chain = chain[::-1]
        self.active = [j for j in self.chain if j['type'] != 'fixed']
        self.names = [j['name'] for j in self.active]
        self.lo = np.array([j['lower'] for j in self.active])
        self.hi = np.array([j['upper'] for j in self.active])

    def fk(self, q):
        values = dict(zip(self.names, q))
        p, R = np.zeros(3), np.eye(3)
        for j in self.chain:
            t, r = joint_transform(j, values.get(j['name'], 0))
            p, R = p + R @ t, R @ r
        return p, R

    def solve(self, target, rotation, seed):
        q = seed.copy()
        for _ in range(350):
            p, R = self.fk(q)
            # Local small-angle rotation error expressed in the root frame.
            w = sum((np.cross(R[:, i], rotation[:, i]) for i in range(3))) / 2
            error = np.r_[target - p, 0.15 * w]
            if np.linalg.norm(target - p) < 0.0005 and np.linalg.norm(R - rotation) < 0.01:
                return q
            J = np.zeros((6, len(q)))
            for i in range(len(q)):
                dq = q.copy()
                dq[i] += 1e-5
                pp, Rp = self.fk(dq)
                dw = sum((np.cross(R[:, k], Rp[:, k]) for k in range(3))) / 2
                J[:, i] = np.r_[(pp-p)/1e-5, 0.15*dw/1e-5]
            delta = J.T @ np.linalg.solve(J @ J.T + 0.00001*np.eye(6), error)
            q = np.clip(q + np.clip(delta, -0.08, 0.08), self.lo, self.hi)
        raise ValueError(f'IK failed at {np.round(target, 4).tolist()}; change workspace or seed')


def load_config(path):
    c = json.loads(Path(path).read_text())
    for key in ['block_size_m', 'supply_xy_m', 'stack_xy_m', 'tool_grasp_point_m']:
        c[key] = np.asarray(c[key], dtype=float)
        if not np.all(np.isfinite(c[key])):
            raise ValueError(f'{key} must be finite')
    if c['block_size_m'].shape != (3,) or np.any(c['block_size_m'] <= 0):
        raise ValueError('block_size_m must contain three positive dimensions')
    if c['supply_xy_m'].ndim != 2 or c['supply_xy_m'].shape[1] != 2 or len(c['supply_xy_m']) == 0:
        raise ValueError('supply_xy_m must contain at least one [x,y] pair')
    if c['stack_xy_m'].shape != (2,):
        raise ValueError('stack_xy_m must be [x,y]')
    for key in ['table_height_m', 'clearance_m', 'cartesian_step_m', 'frame_seconds', 'gripper_motion_seconds']:
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if c['tool_grasp_point_m'].shape != (3,):
        raise ValueError('tool_grasp_point_m must be [x,y,z] in the left_eef frame')
    _, joints, _ = parse_urdf(str(URDF))
    finger = next(j for j in joints if j['name'] == 'left_left_gripper')
    for key in ['gripper_open_rad', 'gripper_grasp_rad']:
        if not np.isfinite(c[key]) or not finger['lower'] <= c[key] <= finger['upper']:
            raise ValueError(f'{key} must be within URDF gripper limits')
    if c['gripper_open_rad'] <= c['gripper_grasp_rad']:
        raise ValueError('open angle must exceed grasp angle for this gripper')
    return c


def plan(c, arm):
    """Generate and validate the entire motion before visualization."""
    h = c['block_size_m'][2]
    blocks = np.c_[c['supply_xy_m'], np.full(len(c['supply_xy_m']), c['table_height_m']+h/2)]
    rotation = np.diag([1., -1., -1.])
    # Desired block centre = tool position + tool rotation @ grasp point.
    offset = -rotation @ c['tool_grasp_point_m']
    grip = c['gripper_open_rad']
    attachment = None
    q = arm.solve(blocks[0] + offset + [0, 0, c['clearance_m']], rotation, np.zeros(len(arm.names)))
    frames = []
    def record(label, held=None):
        p, R = arm.fk(q)
        if held is not None:
            blocks[held] = p + R @ attachment
        frames.append((q.copy(), blocks.copy(), label, held, grip))
    def move(target, label, held=None):
        nonlocal q
        start, _ = arm.fk(q)
        n = max(1, int(np.ceil(np.linalg.norm(target-start)/c['cartesian_step_m'])))
        for f in np.linspace(0, 1, n+1)[1:]:
            q = arm.solve(start+(target-start)*f, rotation, q)
            record(label, held)
    def fingers(target, label, held=None):
        nonlocal grip
        start = grip
        n = max(1, int(np.ceil(c['gripper_motion_seconds']/c['frame_seconds'])))
        for value in np.linspace(start, target, n+1)[1:]:
            grip = float(value)
            record(label, held)
    record('Ready (simulation starts at approach pose)')
    for i in range(len(blocks)):
        source = blocks[i].copy()
        destination = np.r_[c['stack_xy_m'], c['table_height_m']+h*(i+0.5)]
        # Transfer above the finished stack, including the carried block.
        travel_z = c['table_height_m'] + h*len(blocks) + offset[2] + c['clearance_m']
        move(np.r_[source[:2], travel_z], f'{i+1}: approach')
        move(source+offset, f'{i+1}: descend')
        fingers(c['gripper_grasp_rad'], f'{i+1}: close fingers')
        p, R = arm.fk(q)
        attachment = R.T @ (blocks[i]-p)
        record(f'{i+1}: attach (idealized, no contact sensing)', i)
        move(np.r_[source[:2], travel_z], f'{i+1}: lift', i)
        move(np.r_[destination[:2], travel_z], f'{i+1}: transfer', i)
        move(destination+offset, f'{i+1}: place', i)
        # Keep the actual achieved position after release; no target snapping.
        record(f'{i+1}: release')
        attachment = None
        fingers(c['gripper_open_rad'], f'{i+1}: open fingers')
        move(np.r_[destination[:2], travel_z], f'{i+1}: retreat')
    return frames


def render(c, arm, frames, save):
    os.environ['PATH'] = str(Path(sys.executable).parent)+os.pathsep+os.environ.get('PATH', '')
    rr.init('magnetic-block-assembly', spawn=not bool(save))
    if save:
        rr.save(save)
    commandable = log_robot(str(URDF), dict(zip(arm.names, frames[0][0])))
    rr.log('workspace/table', rr.Boxes3D(centers=[[0.3, 0.35, c['table_height_m']-0.015]],
           half_sizes=[[0.22, 0.20, 0.015]], colors=[[85, 100, 115]]), static=True)
    rr.log('workspace/target', rr.Boxes3D(centers=[[*c['stack_xy_m'], c['table_height_m']]],
           half_sizes=[[c['block_size_m'][0]/2, c['block_size_m'][1]/2, 0.001]],
           colors=[[80, 220, 150]]), static=True)
    for index, (q, blocks, label, held, grip) in enumerate(frames):
        rr.set_time('simulation', duration=index*c['frame_seconds'])
        for name, value in zip(arm.names, q):
            joint, path, _ = commandable[name]
            t, R = joint_transform(joint, value)
            rr.log(path, rr.Transform3D(translation=t, mat3x3=R))
        finger, path, followers = commandable['left_left_gripper']
        for joint, entity, value in [(finger, path, grip)] + [
                (j, p, mult*grip+off) for j, p, mult, off in followers]:
            t, R = joint_transform(joint, value)
            rr.log(entity, rr.Transform3D(translation=t, mat3x3=R))
        rr.log('gripper/angle_rad', rr.Scalars(grip))
        rr.log('workspace/blocks', rr.Boxes3D(centers=blocks,
               half_sizes=np.tile(c['block_size_m']/2, (len(blocks), 1)),
               colors=[[240, 110, 70] if i == held else [75, 155, 230] for i in range(len(blocks))],
               labels=[f'Block {i+1}' for i in range(len(blocks))]))
        rr.log('status', rr.TextLog(label))
    rr.get_global_data_recording().flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE/'assembly_config.json')
    parser.add_argument('--check', action='store_true', help='solve and validate without opening viewer')
    parser.add_argument('--save', help='save Rerun recording without opening viewer')
    args = parser.parse_args()
    c, arm = load_config(args.config), Arm()
    frames = plan(c, arm)
    print(f'Validated {len(frames)} joint-limited poses for {len(c["supply_xy_m"])} blocks.')
    print('Kinematics only: collisions, grasp mechanics, and magnetic forces are not validated.')
    if not args.check:
        render(c, arm, frames, args.save)


if __name__ == '__main__':
    main()
