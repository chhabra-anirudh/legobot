"""Isolated URDF gripper contact experiment; not a full-arm digital twin."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from visualize_urdf import parse_urdf, load_stl, joint_transform, rpy_to_matrix
from calibrate_gripper import URDF
from cube_colors import COLORS, rgb


def numbers(values):
    return ' '.join(str(float(v)) for v in np.asarray(values).ravel())


def quaternion(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).ravel())
    return numbers(q)


def make_model(friction=.8, mass=.03, geometry="foam_pads", torsional_friction=.001):
    """Separate convex hull per finger: intentionally conservative proxies.

    The dynamic EEF follows a mocap target through a weld; fingers use URDF hinges.
    No equality or force attaches the cube to the gripper.
    """
    if not np.isfinite(torsional_friction) or torsional_friction < 0:
        raise ValueError('nonnegative finite torsional friction required')
    if geometry not in ['mesh_hulls', 'decomposed', 'foam_pads', 'pads']:
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
    ET.SubElement(world, 'geom', name='table', type='plane', size='.2 .2 .01', friction=f'{friction} {torsional_friction} 0.0001')
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
            if geometry in ['decomposed', 'foam_pads']:
                archive = Path(__file__).resolve().parent/'collision'/f'finger{i}.npz'
                manifest = json.loads((archive.parent/'manifest.json').read_text())
                source = manifest['fingers'][i]
                if (hashlib.sha256(URDF.read_bytes()).hexdigest() != manifest['urdf_sha256'] or
                    hashlib.sha256(Path(vis['mesh']).read_bytes()).hexdigest() != source['source_sha256'] or
                    hashlib.sha256(archive.read_bytes()).hexdigest() != source['asset_sha256']):
                    raise ValueError('collision assets stale or modified; regenerate and review')
                with np.load(archive, allow_pickle=False) as parts:
                    count = len(parts.files)//2
                    for part in range(count):
                        part_name = f'part{i}_{part}'
                        ET.SubElement(asset, 'mesh', name=part_name, vertex=numbers(parts[f'vertices{part}']),
                                      face=' '.join(str(int(v)) for v in parts[f'faces{part}'].ravel()))
                        ET.SubElement(body, 'geom', name=f'finger_geom{i}_{part}', type='mesh', mesh=part_name,
                                      mass=str(.04/count), friction=f'{friction} {torsional_friction} 0.0001', condim='4',
                                      solref='.005 1', rgba='.4 .6 .8 1')
                continue
            if geometry == 'pads':
                ref_t, ref_R = joint_transform(joint, .15)
                desired = np.array([0, (.0127+.004)*(1 if i == 0 else -1), .004])
                local = ref_R.T @ (et+er@desired-ref_t)
                ET.SubElement(body, 'geom', name=f'finger_geom{i}', type='box', size='.01 .004 .004',
                              pos=numbers(local), quat=quaternion(ref_R.T@er), mass='.04',
                              friction=f'{friction} {torsional_friction} 0.0001', condim='4', solref='.005 1', rgba='.2 .8 .4 1')
                continue
            ET.SubElement(body, 'geom', name=f'finger_geom{i}', type='mesh', mesh=mesh_name,
                          mass='.04', friction=f'{friction} {torsional_friction} 0.0001', condim='4',
                          solref='.005 1', rgba='.4 .6 .8 1')
        if geometry == 'foam_pads':
            foam = json.loads((Path(__file__).resolve().parent/'foam_pad_config.json').read_text())
            for key in ['width_m','height_m','thickness_m','contact_time_constant_s','pad_mass_kg']:
                if not np.isfinite(foam[key]) or foam[key] <= 0:
                    raise ValueError(f'invalid foam parameter {key}')
            if not 0 <= foam['reference_angle_rad'] <= 1 or not all(np.isfinite(foam[k]) for k in ['reference_angle_rad','center_x_eef_m','center_z_eef_m','inner_half_gap_m']) or foam['inner_half_gap_m'] <= 0:
                raise ValueError('invalid foam reference pose or gap')
            ref_t, ref_R = joint_transform(joint, foam['reference_angle_rad'])
            desired = np.array([foam['center_x_eef_m'],
                                (foam['inner_half_gap_m']+foam['thickness_m']/2)*(1 if i == 0 else -1),
                                foam['center_z_eef_m']])
            local = ref_R.T @ (et+er@desired-ref_t)
            ET.SubElement(body, 'geom', name=f'foam_geom{i}', type='box',
                          size=numbers(np.array([foam['width_m'],foam['thickness_m'],foam['height_m']])/2),
                          pos=numbers(local), quat=quaternion(ref_R.T@er), mass=str(foam['pad_mass_kg']),
                          friction=f'{friction} {torsional_friction} 0.0001', condim='4', priority='1',
                          solref=f"{foam['contact_time_constant_s']} 1", solimp='.9 .95 .001', rgba='.2 .25 .3 1')
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
                  friction=f'{friction} {torsional_friction} 0.0001', condim='4', solref='.005 1', rgba='.95 .4 .2 1')
    xml = ET.tostring(root, encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml)


def _run_episode(*, close=True, friction=.8, seed=0, depth=.5, mass=.03, geometry="foam_pads", torsional_friction=.001, grasp_x=-.018, color="blue"):
    if not np.isfinite(depth) or not 0 < depth < 1:
        raise ValueError('depth must be a fraction strictly between zero and one')
    if not np.isfinite(grasp_x) or abs(grasp_x) > .05:
        raise ValueError('grasp_x must be finite and within 50 mm of cube centre')
    cube_rgb = rgb(color)
    model = make_model(friction, mass, geometry, torsional_friction)
    model.geom('cube_geom').rgba[:] = [v/255 for v in cube_rgb]+[1.]
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
    grasp_y = 0. if geometry == 'foam_pads' else .0011
    grasp = np.r_[xy+[grasp_x, grasp_y], grasp_z]
    hover = grasp + [0, 0, .06]
    data.mocap_pos[0] = hover
    eef_address = model.joint('eef_free').qposadr[0]
    data.qpos[eef_address:eef_address+3] = hover
    mujoco.mj_forward(model, data)
    cube_id = model.geom('cube_geom').id
    table_id = model.geom('table').id
    finger_bodies = {model.body(f'finger{i}').id for i in range(2)}
    finger_ids = {g for g in range(model.ngeom) if model.geom_bodyid[g] in finger_bodies
                  and model.geom_contype[g] != 0}
    frames = []
    hold_good, hold_total, bad_table = 0, 0, False
    min_hold_forces = np.full(2, np.inf)
    lift_reference = None
    max_slip = 0.
    stages = [('settle', .3, hover, .35), ('descend', 1., grasp, .35),
              ('close', .8, grasp, .12 if close else .35),
              ('lift', 1., hover, .12 if close else .35),
              ('hold', 1., hover, .12 if close else .35),
              ('lower', 1., grasp, .12 if close else .35),
              ('release', .5, grasp, .35), ('retreat', 1., hover, .35),
              ('settle_final', .5, hover, .35)]
    for stage, duration, target, angle in stages:
        if stage == 'lift':
            lift_reference = (data.body('cube').xpos-data.body('eef').xpos) @ data.body('eef').xmat.reshape(3,3)
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
            normal_forces = np.zeros(2)
            weighted_positions = np.zeros((2,3))
            body_index = {model.body(f'finger{i}').id:i for i in range(2)}
            for contact_index, contact in enumerate(data.contact):
                pair = {contact.geom1, contact.geom2}
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, contact_index, force)
                if contact.dist <= 0 and cube_id in pair and force[0] > .01:
                    contacts.update(int(model.geom_bodyid[g]) for g in pair & finger_ids)
                    for g in pair & finger_ids:
                        index = body_index[int(model.geom_bodyid[g])]
                        normal_forces[index] += force[0]
                        weighted_positions[index] += force[0]*contact.pos
                if contact.dist < -.0001 and table_id in pair and pair & finger_ids:
                    bad_table = True
            pos = data.body('cube').xpos.copy()
            contact_centers = [((weighted_positions[i]/normal_forces[i]-pos) @ data.body('cube').xmat.reshape(3,3)).tolist()
                               if normal_forces[i] > 0 else None for i in range(2)]
            if stage in ['lift', 'hold']:
                relative = (pos-data.body('eef').xpos) @ data.body('eef').xmat.reshape(3,3)
                max_slip = max(max_slip, float(np.linalg.norm(relative-lift_reference)))
            if stage == 'hold':
                min_hold_forces = np.minimum(min_hold_forces, normal_forces)
                hold_total += 1
                hold_good += int(pos[2] > .05 and len(contacts) == 2 and np.min(normal_forces) >= .5)
            if step % 20 == 0:
                frames.append({'time_s': float(data.time), 'stage': stage,
                               'cube_position_m': pos.tolist(), 'cube_color':color,
                               'cube_quaternion_wxyz': data.body('cube').xquat.tolist(),
                               'eef_position_m': data.body('eef').xpos.tolist(),
                               'target_position_m': data.mocap_pos[0].tolist(),
                               'finger_poses': [{'position_m': data.body(f'finger{i}').xpos.tolist(),
                                                 'quaternion_wxyz': data.body(f'finger{i}').xquat.tolist()} for i in range(2)],
                               'finger_angles_rad': [float(data.qpos[model.joint(f'jaw{i}').qposadr[0]]) for i in range(2)],
                               'command_rad': data.ctrl.tolist(), 'finger_contacts': len(contacts),
                               'normal_forces_n':normal_forces.tolist(), 'contact_centers_cube_m':contact_centers})
    final = data.body('cube').xpos.copy()
    final_tilt = float(np.arccos(np.clip(np.max(np.abs(data.body('cube').xmat.reshape(3,3)[2])), 0, 1)))
    final_speed = float(np.linalg.norm(data.qvel[model.joint('cube_free').dofadr[0]:model.joint('cube_free').dofadr[0]+3]))
    stable = bool(np.all(np.isfinite(data.qpos)) and not any(w.number for w in data.warning))
    result = {'schema_version': 2, 'frame': 'experiment_table', 'mujoco_version': mujoco.__version__,
              'numerically_stable': stable, 'seed': seed, 'mode': 'isolated_gripper_contact_sim', 'close_enabled': close, 'geometry': geometry,
              'friction': friction, 'torsional_friction_m': torsional_friction, 'mass_kg': mass, 'grasp_depth_from_top': depth, 'grasp_x_m':grasp_x, 'grasp_y_m':grasp_y, 'cube_color':color,
              'foam_pad_parameters':json.loads((Path(__file__).resolve().parent/'foam_pad_config.json').read_text()) if geometry == 'foam_pads' else None,
              'hold_success': bool(stable and hold_good == hold_total and hold_total > 0 and max_slip < .002),
              'hold_contact_fraction': hold_good/hold_total,
              'min_hold_normal_forces_n':min_hold_forces.tolist(), 'max_lift_hold_slip_m':max_slip,
              'finger_table_collision': bad_table,
              'placement_error_m': float(np.linalg.norm(final-np.r_[xy, .0127])),
              'final_cube_position_m': final.tolist(),
              'final_tilt_rad': final_tilt, 'final_speed_m_s': final_speed,
              'success': bool(stable and max_slip < .002 and hold_good == hold_total and not bad_table and np.linalg.norm(final-np.r_[xy, .0127]) < .003
                              and final_tilt < np.deg2rad(5) and final_speed < .01)}
    return result, frames


def run_episode(**kwargs):
    """Run sequentially: MuJoCo's warning callback is process-global."""
    messages = {}
    previous = mujoco.get_mju_user_warning()
    def record_warning(message):
        messages[message] = messages.get(message, 0)+1
    mujoco.set_mju_user_warning(record_warning)
    try:
        result, frames = _run_episode(**kwargs)
    finally:
        mujoco.set_mju_user_warning(previous)
    result['solver_warnings'] = messages
    if messages:
        result['numerically_stable'] = False
        result['hold_success'] = False
        result['success'] = False
    return result, frames


def save_replay(frames, path, geometry, show_contacts=False):
    """Rerun poses from physics. Mesh visuals are not the convex collision hulls."""
    import rerun as rr
    rr.init('isolated-gripper-contact')
    rr.save(str(path))
    rr.log('/', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log('table', rr.Boxes3D(centers=[[0, 0, -.005]], half_sizes=[[.15, .15, .005]]), static=True)
    rr.log('cube/shape', rr.Boxes3D(half_sizes=[[.0127]*3], colors=[rgb(frames[0]['cube_color'])], labels=[frames[0]['cube_color']+' cube']), static=True)
    model = make_model(geometry=geometry)
    for i in range(2):
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] != model.body(f'finger{i}').id or model.geom_contype[geom_id] == 0:
                continue
            geom = model.geom(geom_id)
            entity = f'finger{i}/shape{geom_id}'
            rr.log(entity, rr.Transform3D(translation=geom.pos, quaternion=rr.Quaternion(xyzw=np.roll(geom.quat, -1))), static=True)
            if geom.type[0] == mujoco.mjtGeom.mjGEOM_BOX:
                rr.log(entity, rr.Boxes3D(half_sizes=[geom.size], colors=[(geom.rgba[:3]*255).astype(int)]), static=True)
            else:
                mesh_id = geom.dataid[0]
                va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
                fa, fn = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
                rr.log(entity, rr.Mesh3D(vertex_positions=model.mesh_vert[va:va+vn],
                       triangle_indices=model.mesh_face[fa:fa+fn], albedo_factor=[.4,.6,.8,1]), static=True)
    for f in frames:
        rr.set_time('physics', duration=f['time_s'])
        rr.log('cube', rr.Transform3D(translation=f['cube_position_m'], quaternion=rr.Quaternion(xyzw=np.roll(f['cube_quaternion_wxyz'], -1))))
        for i, pose in enumerate(f['finger_poses']):
            rr.log(f'finger{i}', rr.Transform3D(translation=pose['position_m'], quaternion=rr.Quaternion(xyzw=np.roll(pose['quaternion_wxyz'], -1))))
        rr.log('status', rr.TextLog(f['stage']+' — isolated contact experiment, '+geometry))
        rr.log('contacts', rr.Scalars(f['finger_contacts']))
        rr.log('grip/normal_force_n', rr.Scalars(f['normal_forces_n']))
        if show_contacts:
            points = [p for p in f['contact_centers_cube_m'] if p is not None]
            rr.log('cube/contact_centers', rr.Points3D(points if points else np.empty((0,3)), radii=.0008, colors=[[0,255,255]]))
    rr.get_global_data_recording().flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', choices=['mesh_hulls', 'decomposed', 'foam_pads', 'pads'], default='foam_pads')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-close', action='store_true')
    parser.add_argument('--friction', type=float, default=.8)
    parser.add_argument('--torsional-friction', type=float, default=.001)
    parser.add_argument('--color', choices=list(COLORS), default='blue')
    parser.add_argument('--show-contacts', action='store_true')
    parser.add_argument('--grasp-x', type=float, default=-.018, help='EEF X offset from cube centre (metres)')
    parser.add_argument('--depth', type=float, default=.5)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--save', type=Path, help='Rerun physics recording')
    args = parser.parse_args()
    result, frames = run_episode(seed=args.seed, close=not args.no_close, friction=args.friction, depth=args.depth, geometry=args.geometry, torsional_friction=args.torsional_friction, grasp_x=args.grasp_x, color=args.color)
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        (args.output/'trace.jsonl').write_text(''.join(json.dumps(f)+'\n' for f in frames))
    if args.save:
        save_replay(frames, args.save, args.geometry, args.show_contacts)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
