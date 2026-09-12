"""Isolated URDF gripper contact experiment; not a full-arm digital twin."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from visualize_urdf import parse_urdf, load_stl, joint_transform, rpy_to_matrix
from calibrate_gripper import URDF


def numbers(values):
    return ' '.join(str(float(v)) for v in np.asarray(values).ravel())


def quaternion(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).ravel())
    return numbers(q)


def make_model(friction=.8, mass=.03, geometry="mesh_hulls"):
    """Separate convex hull per finger: intentionally conservative proxies.

    The dynamic EEF follows a mocap target through a weld; fingers use URDF hinges.
    No equality or force attaches the cube to the gripper.
    """
    if geometry not in ['mesh_hulls', 'pads']:
        raise ValueError('unknown collision geometry')
    if not np.isfinite(friction) or friction < 0 or not np.isfinite(mass) or mass <= 0:
        raise ValueError('finite nonnegative friction and positive mass required')
    links, joints, _ = parse_urdf(str(URDF))
    joints = {j['name']: j for j in joints}
    et, er = joint_transform(joints['left_eef_frame'], 0)
    root = ET.Element('mujoco', model='isolated_bracketbot_gripper')
    ET.SubElement(root, 'compiler', angle='radian')
    ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.81', integrator='implicitfast', cone='elliptic')
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'geom', name='table', type='plane', size='.2 .2 .01', friction=f'{friction} 0.001 0.0001')
    ET.SubElement(world, 'body', name='target', mocap='true', pos='0 0 .08', quat='0 1 0 0')
    hand = ET.SubElement(world, 'body', name='eef', pos='0 0 .08', quat='0 1 0 0', gravcomp='1')
    ET.SubElement(hand, 'freejoint', name='eef_free')
    ET.SubElement(hand, 'inertial', pos='0 0 0', mass='.5', diaginertia='.001 .001 .001')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'weld', body1='target', body2='eef', solref='.005 1')
    actuator = ET.SubElement(root, 'actuator')
    for i, name in enumerate(['left_left_gripper', 'left_right_gripper']):
        joint = joints[name]
        t, R = joint_transform(joint, 0)
        body = ET.SubElement(hand, 'body', name=f'finger{i}', pos=numbers(er.T@(t-et)), quat=quaternion(er.T@R))
        ET.SubElement(body, 'joint', name=f'jaw{i}', type='hinge', axis=numbers(joint['axis']),
                      range=numbers([joint['lower'], joint['upper']]), damping='.02', armature='.001')
        for k, vis in enumerate(links[joint['child']]):
            vertices, faces, _ = load_stl(vis['mesh'])
            vertices = (vertices*vis['scale']) @ rpy_to_matrix(vis['rpy']).T + vis['xyz']
            unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
            mesh_name = f'finger_mesh{i}_{k}'
            ET.SubElement(asset, 'mesh', name=mesh_name, vertex=numbers(unique),
                          face=' '.join(str(int(v)) for v in inverse[faces].ravel()))
            ET.SubElement(body, 'geom', name=f'visual{i}', type='mesh', mesh=mesh_name,
                          contype='0', conaffinity='0', group='2', density='0')
            if geometry == 'pads':
                ref_t, ref_R = joint_transform(joint, .15)
                desired = np.array([0, (.0127+.004)*(1 if i == 0 else -1), .004])
                local = ref_R.T @ (et+er@desired-ref_t)
                ET.SubElement(body, 'geom', name=f'finger_geom{i}', type='box', size='.01 .004 .004',
                              pos=numbers(local), quat=quaternion(ref_R.T@er), mass='.04',
                              friction=f'{friction} 0.001 0.0001', condim='3', solref='.005 1', rgba='.2 .8 .4 1')
                continue
            ET.SubElement(body, 'geom', name=f'finger_geom{i}', type='mesh', mesh=mesh_name,
                          mass='.04', friction=f'{friction} 0.001 0.0001', condim='3',
                          solref='.005 1', rgba='.4 .6 .8 1')
        ET.SubElement(actuator, 'position', name=f'jaw_drive{i}', joint=f'jaw{i}', kp='4', kv='.05',
                      ctrlrange='0 1', forcerange='-.15 .15')
    mimic = joints['left_right_gripper']['mimic']
    ET.SubElement(equality, 'joint', joint1='jaw1', joint2='jaw0',
                  polycoef=numbers([mimic['offset'], mimic['multiplier'], 0, 0, 0]), solref='.005 1')
    contact = ET.SubElement(root, 'contact')
    ET.SubElement(contact, 'exclude', body1='finger0', body2='finger1')
    cube = ET.SubElement(world, 'body', name='cube', pos='0 0 .0127')
    ET.SubElement(cube, 'freejoint', name='cube_free')
    ET.SubElement(cube, 'geom', name='cube_geom', type='box', size='.0127 .0127 .0127', mass=str(mass),
                  friction=f'{friction} 0.001 0.0001', condim='3', solref='.005 1', rgba='.95 .4 .2 1')
    xml = ET.tostring(root, encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml)


def run_episode(*, close=True, friction=.8, seed=0, depth=1/3, mass=.03, geometry="mesh_hulls"):
    if not np.isfinite(depth) or not 0 < depth < 1:
        raise ValueError('depth must be a fraction strictly between zero and one')
    model = make_model(friction, mass, geometry)
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-.003, .003, 2)
    address = model.joint('cube_free').qposadr[0]
    data.qpos[address:address+2] = xy
    for name in ['jaw0', 'jaw1']:
        data.qpos[model.joint(name).qposadr[0]] = .35
    data.ctrl[:] = .35
    # Existing cube-centre local Z=.004, raised by h*(1/2-depth).
    grasp_z = .0127 + .004 + .0254*(.5-depth)
    # Y centering remains a configurable candidate, not measured real calibration.
    grasp = np.r_[xy+[0, .0011], grasp_z]
    hover = grasp + [0, 0, .06]
    data.mocap_pos[0] = hover
    eef_address = model.joint('eef_free').qposadr[0]
    data.qpos[eef_address:eef_address+3] = hover
    mujoco.mj_forward(model, data)
    cube_id = model.geom('cube_geom').id
    table_id = model.geom('table').id
    finger_ids = {model.geom(f'finger_geom{i}').id for i in range(2)}
    frames = []
    hold_good, hold_total, bad_table = 0, 0, False
    stages = [('settle', .3, hover, .35), ('descend', 1., grasp, .35),
              ('close', .8, grasp, .12 if close else .35),
              ('lift', 1., hover, .12 if close else .35),
              ('hold', 1., hover, .12 if close else .35),
              ('lower', 1., grasp, .12 if close else .35),
              ('release', .5, grasp, .35), ('retreat', 1., hover, .35),
              ('settle_final', .5, hover, .35)]
    for stage, duration, target, angle in stages:
        start = data.mocap_pos[0].copy()
        initial = data.ctrl.copy()
        count = round(duration/model.opt.timestep)
        for step in range(count):
            f = (step+1)/count
            smooth = f*f*(3-2*f)
            data.mocap_pos[0] = start+(target-start)*smooth
            data.ctrl[:] = initial+(angle-initial)*smooth
            mujoco.mj_step(model, data)
            # Refresh derived poses/contact data at the newly integrated state.
            mujoco.mj_forward(model, data)
            contacts = set()
            for contact_index, contact in enumerate(data.contact):
                pair = {contact.geom1, contact.geom2}
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, contact_index, force)
                if contact.dist <= 0 and cube_id in pair and force[0] > .01:
                    contacts.update(pair & finger_ids)
                if contact.dist < -.0001 and table_id in pair and pair & finger_ids:
                    bad_table = True
            pos = data.body('cube').xpos.copy()
            if stage == 'hold':
                hold_total += 1
                hold_good += int(pos[2] > .05 and len(contacts) == 2)
            if step % 20 == 0:
                frames.append({'time_s': float(data.time), 'stage': stage,
                               'cube_position_m': pos.tolist(),
                               'cube_quaternion_wxyz': data.body('cube').xquat.tolist(),
                               'eef_position_m': data.body('eef').xpos.tolist(),
                               'target_position_m': data.mocap_pos[0].tolist(),
                               'finger_poses': [{'position_m': data.body(f'finger{i}').xpos.tolist(),
                                                 'quaternion_wxyz': data.body(f'finger{i}').xquat.tolist()} for i in range(2)],
                               'finger_angles_rad': [float(data.qpos[model.joint(f'jaw{i}').qposadr[0]]) for i in range(2)],
                               'command_rad': data.ctrl.tolist(), 'finger_contacts': len(contacts)})
    final = data.body('cube').xpos.copy()
    stable = bool(np.all(np.isfinite(data.qpos)) and not any(w.number for w in data.warning))
    result = {'schema_version': 1, 'frame': 'experiment_table', 'mujoco_version': mujoco.__version__,
              'numerically_stable': stable, 'seed': seed, 'mode': 'isolated_gripper_contact_sim', 'close_enabled': close, 'geometry': geometry,
              'friction': friction, 'mass_kg': mass, 'grasp_depth_from_top': depth,
              'hold_success': bool(stable and hold_good == hold_total and hold_total > 0),
              'hold_contact_fraction': hold_good/hold_total,
              'finger_table_collision': bad_table,
              'placement_error_m': float(np.linalg.norm(final-np.r_[xy, .0127])),
              'final_cube_position_m': final.tolist(),
              'success': bool(stable and hold_good == hold_total and not bad_table and np.linalg.norm(final-np.r_[xy, .0127]) < .003)}
    return result, frames


def save_replay(frames, path, geometry):
    """Rerun poses from physics. Mesh visuals are not the convex collision hulls."""
    import rerun as rr
    rr.init('isolated-gripper-contact')
    rr.save(str(path))
    rr.log('/', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log('table', rr.Boxes3D(centers=[[0, 0, -.005]], half_sizes=[[.15, .15, .005]]), static=True)
    rr.log('cube/shape', rr.Boxes3D(half_sizes=[[.0127]*3], colors=[[240, 110, 70]]), static=True)
    model = make_model(geometry=geometry)
    for i in range(2):
        if geometry == 'pads':
            geom = model.geom(f'finger_geom{i}')
            rr.log(f'finger{i}/shape', rr.Transform3D(translation=geom.pos, quaternion=rr.Quaternion(xyzw=np.roll(geom.quat, -1))), static=True)
            rr.log(f'finger{i}/shape', rr.Boxes3D(half_sizes=[geom.size], colors=[[60, 190, 110]]), static=True)
        else:
            # Compiled mesh vertices + mesh geom transform display the supplied surface.
            geom = model.geom(f'finger_geom{i}')
            mesh_id = geom.dataid[0]
            va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
            fa, fn = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
            rr.log(f'finger{i}/shape', rr.Transform3D(translation=geom.pos, quaternion=rr.Quaternion(xyzw=np.roll(geom.quat, -1))), static=True)
            rr.log(f'finger{i}/shape', rr.Mesh3D(vertex_positions=model.mesh_vert[va:va+vn],
                   triangle_indices=model.mesh_face[fa:fa+fn], albedo_factor=[.4,.6,.8,1]), static=True)
    for f in frames:
        rr.set_time('physics', duration=f['time_s'])
        rr.log('cube', rr.Transform3D(translation=f['cube_position_m'], quaternion=rr.Quaternion(xyzw=np.roll(f['cube_quaternion_wxyz'], -1))))
        for i, pose in enumerate(f['finger_poses']):
            rr.log(f'finger{i}', rr.Transform3D(translation=pose['position_m'], quaternion=rr.Quaternion(xyzw=np.roll(pose['quaternion_wxyz'], -1))))
        rr.log('status', rr.TextLog(f['stage']+' — isolated contact experiment, '+geometry))
        rr.log('contacts', rr.Scalars(f['finger_contacts']))
    rr.get_global_data_recording().flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', choices=['mesh_hulls', 'pads'], default='mesh_hulls')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-close', action='store_true')
    parser.add_argument('--friction', type=float, default=.8)
    parser.add_argument('--depth', type=float, default=1/3)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--save', type=Path, help='Rerun physics recording')
    args = parser.parse_args()
    result, frames = run_episode(seed=args.seed, close=not args.no_close, friction=args.friction, depth=args.depth, geometry=args.geometry)
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        (args.output/'trace.jsonl').write_text(''.join(json.dumps(f)+'\n' for f in frames))
    if args.save:
        save_replay(frames, args.save, args.geometry)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
