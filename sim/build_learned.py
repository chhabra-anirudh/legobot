"""Execute image-compiler structures with learned pickup and free contact cubes.

This is a gripper/cube construction experiment, not full-arm dynamics or a hardware
controller. The existing planner checks arm IK, but these Cartesian transport
segments are not yet the planner's actual arm trajectory. Report that distinction.
"""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from learning_env import ContactScene, DT, expert_action, run_pick
from pickup_policy import Policy
from tool_frame import rotation_for
import build_structure as planner
from cube_colors import rgb


def checked_plan(structure_path):
    structure = planner.load_structure(structure_path)
    report = planner.validate(structure)
    if not report.ok:
        raise ValueError('compiler rejected structure: '+str(report.problems))
    c = planner.load_config(planner.HERE/'assembly_config.json')
    if not np.allclose(c['block_size_m'], [.0254, .0254, .0254], rtol=0, atol=1e-9):
        raise ValueError('Learning scene/checkpoint currently models 25.4 mm cubes; current build uses different block dimensions. Recalibrate contact and retrain before construction.')
    arm = planner.Arm(planner.HERE/'robot_limits.json')
    pitch = float(c['block_size_m'][0])
    calibration = {'origin_xy_m': np.zeros(2), 'pitch_m': np.array([pitch,pitch]), 'layer_height_m': pitch}
    measured = planner.reach_map.load(float(c['table_height_m']))
    reach = planner.reach_map.clear_of_base(measured, c)
    origin, quarter, count = planner.choose_placement(structure, c, calibration, reach, pitch)
    calibration['origin_xy_m'] = np.asarray(origin)
    calibration['quarter_turns'] = quarter
    frames, _, _, cold, plan = planner.plan_build(structure, c, arm, calibration, reach)
    plan['kinematic_pose_count'] = len(frames)
    plan['cold_ik_restarts'] = cold
    plan['chassis_excluded_map_cells'] = len(measured)-len(reach)
    plan['candidate_origin_count'] = count
    plan['carried_tilt_deg'] = planner.carried_tilt_deg(arm, frames)[0]
    return structure, plan


def run_build(plan, actor, *, seed=5000, friction=.8, mass=.03):
    table = float(plan['calibration']['table_height_m'])
    shift = np.array([0.,0.,table])
    steps = plan['placements']
    starts = np.array([s['pick_from_m'] for s in steps])-shift
    targets = np.array([s['cube_centre_m'] for s in steps])-shift
    if len(set(map(tuple,starts))) != len(starts):
        raise ValueError('duplicate staging slot')
    rng = np.random.default_rng(seed)
    records = []
    full_success = True
    with ContactScene(starts,friction=friction,mass=mass) as scene:
        for _ in range(round(.3/DT)):
            scene.step(np.zeros(5),'settle')
        for index, placement in enumerate(steps):
            scene.selected = index
            scene.slip_reference = None
            source = scene.cube.xpos.copy()
            if np.linalg.norm(source-starts[index]) > .003:
                records.append({'step':index+1,'success':False,'reason':'staged_cube_displaced'})
                full_success = False
                break
            # The reset is only at episode start. Moving to another cube is physical.
            source_hover = source-rotation_for(0.)@np.array([.018,0.,.004])+[0,0,.06]
            if not scene.move(source_hover,0.,.26,stage='approach_next_cube'):
                records.append({'step':index+1,'success':False,'reason':'approach_timeout'})
                full_success = False
                break
            observed = source+rng.uniform(-.0004,.0004,3)
            if scene.bad_table:
                records.append({'step':index+1,'success':False,'reason':'approach_finger_table_collision'})
                full_success = False
                break
            pickup, _ = run_pick(scene,actor,observed_position=observed,observed_yaw=0.)
            record = {'step':index+1,'voxel':placement['voxel'],'color':placement['color'],'pickup':pickup}
            if not pickup['success']:
                record.update(success=False,reason='pickup:'+pickup['reason'])
                records.append(record)
                full_success = False
                break
            yaw = float(placement['tool_yaw_rad'])
            destination = targets[index]
            # Tool/cube offset is observed after lift, capturing actual compression.
            # This is a simulated pose observation, not a force input to the policy.
            grasp_offset = scene.relative_cube().copy()
            target_tool = destination-rotation_for(yaw)@grasp_offset
            travel = target_tool.copy()
            travel[2] = max(float(np.max(targets[:,2]))+.07, scene.command[2])
            raised = scene.command.copy(); raised[2] = travel[2]
            ok = scene.move(raised,scene.yaw,.12,stage='raise_for_transfer')
            if ok:
                ok = scene.move(travel,yaw,.12,stage='transfer')
            if np.linalg.norm(scene.relative_cube()-grasp_offset) > .01:
                ok = False
            if ok:
                ok = scene.move(target_tool,yaw,.12,stage='lower')
            if ok:
                ok = scene.move(target_tool,yaw,.26,stage='release')
                scene.slip_reference = None
                if ok:
                    ok = scene.move(travel,yaw,.26,stage='retreat')
                for _ in range(round(.5/DT)):
                    scene.step(np.zeros(5),'settle_placement')
            error = float(np.linalg.norm(scene.cube.xpos-destination))
            dof = scene.model.joint(scene.names[index]+'_free').dofadr[0]
            speed = float(np.linalg.norm(scene.data.qvel[dof:dof+3]))
            tilt = float(np.arccos(np.clip(np.max(np.abs(scene.cube.xmat.reshape(3,3)[2])),0,1)))
            success = bool(ok and scene.stable() and not scene.bad_table and error < .003 and speed < .01 and tilt < np.deg2rad(5))
            record.update(success=success,reason='' if success else 'transport_or_placement_failed',
                          placement_error_m=error,final_speed_m_s=speed,final_tilt_rad=tilt,
                          max_pick_transport_slip_m=scene.max_slip, finger_table_collision=scene.bad_table)
            records.append(record)
            print(f'cube {index+1}/{len(steps)}: success={success} placement={error*1000:.3f} mm',flush=True)
            if not success:
                full_success = False
                break
        errors = [float(np.linalg.norm(scene.data.body(n).xpos-t)) for n,t in zip(scene.names,targets)]
        final_speeds, final_tilts = [], []
        for name in scene.names:
            dof = scene.model.joint(name+'_free').dofadr[0]
            final_speeds.append(float(np.linalg.norm(scene.data.qvel[dof:dof+3])))
            final_tilts.append(float(np.arccos(np.clip(np.max(np.abs(scene.data.body(name).xmat.reshape(3,3)[2])),0,1))))
        final_valid = [e < .003 and v < .01 and t < np.deg2rad(5)
                       for e,v,t in zip(errors,final_speeds,final_tilts)]
        full_success &= len(records)==len(steps) and all(r['success'] for r in records) and all(final_valid) and scene.stable()
        report = {'success':bool(full_success),'controller':'scripted_expert' if actor is expert_action else 'learned_pickup_scripted_placement',
                  'scope':'dynamic gripper and all free cubes; no full-arm collision/dynamics or magnetic model',
                  'frame':'experiment_table','root_to_experiment_translation_m':(-shift).tolist(),
                  'seed':seed,'friction':friction,'mass_kg':mass,'structure_id':plan['structure_id'],
                  'requested_cubes':len(steps),'completed_cubes':int(sum(r['success'] and final_valid[i] for i,r in enumerate(records))),
                  'duration_s':float(scene.data.time),'final_errors_m':errors,'placements':records,
                  'final_speeds_m_s':final_speeds,'final_tilts_rad':final_tilts,
                  'solver_warnings':dict(scene.warnings)}
        return report, scene.frames


def save_replay(frames, plan, path):
    import rerun as rr
    from contact_grasp import make_model
    rr.init('learned-contact-construction')
    rr.save(str(path))
    rr.log('/',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rr.log('table',rr.Boxes3D(centers=[[0,.25,-.005]],half_sizes=[[.6,.6,.005]],colors=[[70,75,80]]),static=True)
    for i,s in enumerate(plan['placements']):
        rr.log(f'cubes/{i}/shape',rr.Boxes3D(half_sizes=[[.0127]*3],colors=[rgb(s['color'])]),static=True)
    model = make_model()
    for i in range(2):
        for g in range(model.ngeom):
            geom=model.geom(g)
            if model.geom_bodyid[g] != model.body(f'finger{i}').id or not model.geom_contype[g]:
                continue
            entity=f'finger{i}/geometry{g}'
            rr.log(entity,rr.Transform3D(translation=geom.pos,quaternion=rr.Quaternion(xyzw=np.roll(geom.quat,-1))),static=True)
            if geom.type[0] == 6: # mjGEOM_BOX
                rr.log(entity,rr.Boxes3D(half_sizes=[geom.size],colors=[(geom.rgba[:3]*255).astype(int)]),static=True)
            else:
                mid=geom.dataid[0]; va,vn=model.mesh_vertadr[mid],model.mesh_vertnum[mid]; fa,fn=model.mesh_faceadr[mid],model.mesh_facenum[mid]
                rr.log(entity,rr.Mesh3D(vertex_positions=model.mesh_vert[va:va+vn],triangle_indices=model.mesh_face[fa:fa+fn]),static=True)
    for f in frames:
        rr.set_time('contact_build',duration=f['time_s'])
        for i,(pos,quat) in enumerate(zip(f['cube_positions_m'],f['cube_quaternions_xyzw'])):
            rr.log(f'cubes/{i}',rr.Transform3D(translation=pos,quaternion=rr.Quaternion(xyzw=quat)))
        for i,pose in enumerate(f['finger_poses']):
            rr.log(f'finger{i}',rr.Transform3D(translation=pose['position_m'],quaternion=rr.Quaternion(xyzw=np.roll(pose['quaternion_wxyz'],-1))))
        rr.log('status',rr.TextLog(f'cube {f["selected_cube"]+1}: {f["stage"]}'))
    rr.get_global_data_recording().flush()


def compile_image(image_path, output, grid):
    """Run the existing deterministic image compiler and preserve its provenance."""
    output.mkdir(parents=True, exist_ok=True)
    structure_path = output/'image-structure.json'
    subprocess.run([sys.executable, str(planner.HERE.parent/'compiler/image_to_structure.py'),
                    str(image_path), '--trace', '--grid', *map(str, grid),
                    '--output', str(structure_path)], check=True)
    return structure_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('structure',type=Path, help='checked structure JSON, or image when --image is set')
    parser.add_argument('--image', action='store_true', help='compile the input image with deterministic tracing before construction')
    parser.add_argument('--grid', type=int, nargs=2, default=[10, 8], metavar=('W', 'H'), help='image sampling grid; no automatic reduction to fit the robot')
    control=parser.add_mutually_exclusive_group(required=True)
    control.add_argument('--policy',type=Path)
    control.add_argument('--expert',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('outputs/learned-build'))
    parser.add_argument('--save',type=Path)
    parser.add_argument('--seed',type=int,default=5000)
    args=parser.parse_args()
    structure_path = compile_image(args.structure, args.output, args.grid) if args.image else args.structure
    structure,plan=checked_plan(structure_path)
    actor=expert_action if args.expert else Policy.load(args.policy)
    if args.policy:
        from train_pickup import fingerprint
        if actor.metadata['source_hashes'] != fingerprint():
            raise ValueError('policy was trained against different physics; retrain before building')
    report,frames=run_build(plan,actor,seed=args.seed)
    report['structure_sha256']=hashlib.sha256(structure_path.read_bytes()).hexdigest()
    report['source']=structure.source
    report['provenance']=structure.provenance
    if args.policy:
        report['policy_sha256']=actor.metadata['checkpoint_sha256']
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    (args.output/'trace.jsonl').write_text(''.join(json.dumps(f)+'\n' for f in frames))
    if args.save:
        save_replay(frames,plan,args.save)
    print(json.dumps({k:v for k,v in report.items() if k not in ('placements','provenance')},indent=2))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())