"""Fresh known-probe-tip observations in the current native layout; static only."""
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT,URDF,load_json,save_json,robot_for,transform
from unloading_sim.eco65.native_io import verify_snapshot,file_hash
from unloading_sim.moveit2_backend import ResidentMoveItClient,digest
from unloading_sim.touch_calibration import points
from eco65_touch_sources import bind_scene_sources,derived_scene_identity
from eco65_touch_geometry import StaticGeometry
from eco65_j1_visualization import face_angle


def board_capture_projection(T_D_camera,K,resolution,T_D_board,size_m,margin_px=12.):
    """Acquisition-only FOV gate; nominal camera pose is not solver evidence."""
    C=np.asarray(T_D_camera);P=np.asarray(T_D_board);w,h=size_m
    local=np.array([[0.,0.,0.],[w,0.,0.],[w,h,0.],[0.,h,0.]])
    optical=(local@P[:3,:3].T+P[:3,3]-C[:3,3])@C[:3,:3]
    if np.min(optical[:,2])<=0:return dict(accepted=False,reason='BOARD_BEHIND_CAMERA')
    projected=optical@np.asarray(K).T;uv=projected[:,:2]/projected[:,2,None]
    lower=uv.min(axis=0);upper=uv.max(axis=0)
    return dict(accepted=bool(np.all(lower>=margin_px) and np.all(upper<=np.asarray(resolution)-1-margin_px)),
        bounds_px=[lower.tolist(),upper.tolist()],minimum_depth_m=float(optical[:,2].min()),
        source='NOMINAL_CAMERA_FOR_ACQUISITION_ONLY')


def prepare(a):
    import cv2
    started=perf_counter();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    s=verify_snapshot(load_json(a.snapshot));prior=load_json(a.probe_source);robot=robot_for(s['tool'],s['scene'])
    probe=deepcopy(prior['probe']);board=deepcopy(prior['board']);camera=deepcopy(s['scene']['permanent_camera'])
    if not np.array_equal(probe['original_T_flange_tcp'],s['tool']['T_flange_tcp']):raise ValueError('ORIGINAL_SUCTION_TCP_CHANGED')
    # Only fixture definitions are retained. Every q, placement and image is new.
    # This round uses a fixed virtual jig without a claim about its hardware stand.
    board.pop('support',None)
    board['fixture_assumption']='STATIC_LOCKED_SIMULATION_JIG_NO_PHYSICAL_SUPPORT_QUALIFICATION'
    sources=bind_scene_sources(a.snapshot,a.composed_usd,a.geometry_manifest)
    kin=dict(active_joint_names=robot.active_joint_names,base_link=robot.base_link,T_D_base=robot.base_transform.tolist(),
        limits_rad=robot.joint_limits.tolist(),urdf_sha256=file_hash(URDF),
        joints=[dict(name=j.name,type=j.joint_type,parent=j.parent,child=j.child,origin=j.origin.tolist(),axis=j.axis.tolist()) for j in robot.joints])
    q0=load_json(a.capture_home)['q_rad']
    c=dict(snapshot=a.snapshot.resolve().relative_to(ROOT).as_posix(),snapshot_sha256=file_hash(a.snapshot),source_fingerprint=s['fingerprint'],
        scene_sources=sources,current_native_layout=True,camera=camera,probe=probe,board=board,
        thresholds=deepcopy(prior['thresholds']),kinematics=kin,display_q=q0,placements=[],photos=[],touches=[],
        probe_definition_source=dict(path=a.probe_source.resolve().relative_to(ROOT).as_posix(),sha256=file_hash(a.probe_source)),
        statuses=['FRESH_TCP_FK_GEOMETRIC_TOUCH','PHYSICAL_CONTACT_NOT_EVALUATED','NO_CALIBRATION_MOTION_CLAIM'])
    c['derived_scene_identity']=derived_scene_identity(c);save_json(out/'configuration.json',c)
    aruco=cv2.aruco.CharucoBoard(tuple(board['squares']),board['square_m'],board['marker_m'],cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,board['dictionary'])))
    cv2.imwrite(str(out/'board_pattern.png'),aruco.generateImage((1200,1000),marginSize=0,borderBits=1))
    geometry=StaticGeometry(s,c,out);client=ResidentMoveItClient(a.worker_command,log_path=out/'worker.log')
    identity=s['native_model']['init']['identity'];attempts=[];calls=[];deadline=started+480
    def call(req):
        if perf_counter()>deadline:raise ValueError('TCP_CALIBRATION_PREPARATION_DEADLINE')
        t=perf_counter();r=client.request(req,timeout=40)
        calls.append(dict(request=req,result=r,wall_s=perf_counter()-t));return r
    try:
        if call(s['native_model']['init'])['status']!='READY':raise ValueError('NATIVE_INIT_FAILED')
        rng=np.random.default_rng(652101)
        # Board to the robot's left/front-free side; both independent placements
        # are selected from this bounded jig set, without moving approved layout.
        centers=[[-.72,.26,.52],[-.72,.32,.55],[-.66,.35,.54],[-.73,-.28,.53],[-.65,-.34,.54],
                 [-.78,.42,.56],[-.72,.48,.56],[-.75,.45,.63],[-.83,.46,.63],[-.72,.45,.68]]
        for pid,split,ids in [('train_01','train',range(6)),('heldout_01','validation',[0,3,5])]:
            accepted_group=False
            candidates=centers if split=='train' else [(np.array(c['placements'][0]['center_D_m'])+[.015,.025,0]).tolist()]
            for center in candidates:
                direction=np.array(center)-np.array(s['scene']['base_position_m']);heading=direction[:2]/np.linalg.norm(direction[:2])
                # Printed +Z points away from the robot, and downward 30 deg.
                normal=np.r_[heading*np.cos(np.pi/6),-.5]
                right=np.r_[[-heading[1],heading[0]],0.];down=np.cross(normal,right)
                R=np.column_stack((right,down,normal));P=transform(np.array(center)-R@np.array([.075,.0625,0]),R)
                face,_,_=face_angle(robot,np.array(q0),np.array(camera['T_link1_optical']),center,robot.joint_limits[0],require_away=False)
                views=[]
                for offset in np.radians([-8,0,8]):
                    q=np.array(q0);q[0]=face+offset
                    projection=board_capture_projection(robot.named_link_frames(q)['link_1']@camera['T_link1_optical'],
                        camera['K'],camera['resolution'],P,board['pattern_size_m'])
                    if not robot.joint_limits[0,0]<=q[0]<=robot.joint_limits[0,1] or not projection['accepted']:break
                    views.append((q,projection))
                if len(views)!=3:
                    attempts.append(dict(placement=pid,center=center,accepted=False,
                        failures=['CALIBRATION_BOARD_OUTSIDE_CAPTURE_FOV'],projection=projection))
                    continue
                proposed=[];qseed=np.array(q0)
                for index in ids:
                    point=board['touch_points'][index];goal=points(P,point['p_P_m']);accepted=None
                    seeds=[qseed,*[np.array(t['q_rad']) for t in c['touches']],*rng.uniform(robot.joint_limits[:,0],robot.joint_limits[:,1],(12,6))]
                    for roll in (0.,np.pi):
                        orientation=R@Rotation.from_euler('z',roll).as_matrix()
                        tcp=transform(goal-orientation@probe['p_F_tip_m'],orientation)@s['tool']['T_flange_tcp']
                        for seed in seeds[:16]:
                            r=call(dict(op='ik',identity=identity,q_start=np.asarray(seed).tolist(),goal_pose=tcp.tolist(),seed=652101+len(calls),
                                planning_only=True,planning_mode='cold_from_scratch',position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4))
                            if r.get('status')!='SUCCESS':continue
                            q=np.asarray(r['q']);error=float(np.linalg.norm(points(robot.named_link_frames(q)['link_6'],probe['p_F_tip_m'])-goal))
                            g=geometry.check(q,P,point['point_id'])
                            attempts.append(dict(placement=pid,center=center,point=point['point_id'],q_rad=q.tolist(),tip_error_m=error,
                                accepted=g['accepted'],failures=g['failures']))
                            if error<c['thresholds']['tip_target_m'] and g['accepted']:
                                fk=call(dict(op='fk',identity=identity,q_start=q.tolist(),links=['link_1','link_6']))
                                if fk['status']!='SUCCESS' or any(not np.allclose(fk['frames'][n],robot.named_link_frames(q)[n],atol=1e-8,rtol=0) for n in ['link_1','link_6']):raise ValueError('NATIVE_FK_MISMATCH')
                                accepted=dict(sample_id=pid+'_touch_'+point['point_id'],board_placement_id=pid,split=split,point_id=point['point_id'],
                                    q_rad=q.tolist(),probe_id=digest(probe),tip_error_m=error,geometry=g,
                                    status='SYNTHETIC_GEOMETRIC_TOUCH',physical_contact='NOT_EVALUATED');qseed=q;break
                        if accepted:break
                    if accepted is None:break
                    proposed.append(accepted);print('TCP_DATUM',pid,index,flush=True)
                if len(proposed)!=len(list(ids)):continue
                photos=[]
                for i,(q,projection) in enumerate(views):
                    g=geometry.check(q,P,None)
                    if not g['accepted']:break
                    photos.append(dict(sample_id=pid+'_photo_'+str(i),board_placement_id=pid,split=split,q_rad=q.tolist(),static_check=g,
                        acquisition_projection=projection))
                if len(photos)!=3:continue
                c['placements'].append(dict(board_placement_id=pid,split=split,T_D_P=P.tolist(),center_D_m=center))
                c['photos'].extend(photos);c['touches'].extend(proposed);accepted_group=True;save_json(out/'configuration.json',c);break
            if not accepted_group:raise ValueError('BOUNDED_TCP_TOUCH_OR_CAPTURE_GEOMETRY_EXHAUSTED:'+pid)
        save_json(out/'report.json',dict(status='PASS',touches=len(c['touches']),photos=len(c['photos']),wall_s=perf_counter()-started))
    except Exception as error:
        save_json(out/'report.json',dict(status='FAIL',error=str(error),wall_s=perf_counter()-started));raise
    finally:
        client.close();save_json(out/'native_calls.json',calls);save_json(out/'attempts.json',attempts)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('snapshot','probe-source','capture-home','composed-usd','geometry-manifest','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--worker-command',required=True);prepare(p.parse_args())
