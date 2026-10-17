"""Prepare private camera/probe fixtures; finite native IK, never trajectory planning."""
import argparse,copy,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT,URDF,load_json,save_json,robot_for,transform
from unloading_sim.eco65.native_io import file_hash
from unloading_sim.moveit2_backend import ResidentMoveItClient,digest
from unloading_sim.touch_calibration import points,intrinsics
from build_eco65_full_face_layout import verify_static
from eco65_touch_sources import bind_scene_sources,derived_scene_identity


def prepare(snapshot_path,pose_path,camera_manifest,out,worker,width=640,height=480,optical_offset_m=(0,0,0),*,composed_usd,geometry_manifest,installation=None):
    import cv2
    snapshot_path=snapshot_path.resolve();pose_path=pose_path.resolve();camera_manifest=camera_manifest.resolve()
    start=time.perf_counter();out=out.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    s=verify_static(load_json(snapshot_path));pose=load_json(pose_path);cad=load_json(camera_manifest)
    sources=bind_scene_sources(snapshot_path,composed_usd,geometry_manifest)
    if pose['source_snapshot_fingerprint']!=s['fingerprint']:raise ValueError('STATIC_POSE_IDENTITY')
    if not s['scene']['mast'].get('mount_interface',{}).get('seat'):raise ValueError('LATEST_CONNECTED_MAST_REQUIRED')
    for p,sha in {cad['source']:cad['source_sha256'],**{p[k]:p[k+'_sha256'] for p in cad['parts'] for k in ['visual','collision']}}.items():
        if file_hash(ROOT/p)!=sha:raise ValueError('CAMERA_ASSET_CHANGED:'+p)
    robot=robot_for(s['tool'],s['scene']);mast=s['scene']['mast']
    # Register known equal-size rear-cover bounds and RGB front window to the
    # existing mast camera pocket. This is a nominal mechanical installation.
    rear=next(p for p in cad['parts'] if p['id']=='camera_part_001')
    old=next(p for p in mast['parts'] if p['id']=='mast_part_004')
    R=np.array([[-1,0,0],[0,0,-1],[0,-1,0.]])
    C=transform(np.mean(old['bounds_m'],axis=0)-R@np.mean(rear['bounds_m'],axis=0),R)
    rgb=next(p for p in cad['parts'] if p['id']=='camera_part_005')
    nominal=np.mean(rgb['bounds_m'],axis=0);nominal[2]=cad['bounds_m'][0][2]-.00005
    optical=transform(points(C,nominal),np.array(mast['T_mast_camera'])[:3,:3])
    optical[:3,3]+=optical[:3,:3]@np.asarray(optical_offset_m,float)
    keep=[p for p in cad['parts'] if p['id'] not in ['camera_part_000','camera_part_002']]
    camera=dict(source=cad['source'],source_sha256=cad['source_sha256'],cad_manifest=camera_manifest.relative_to(ROOT).as_posix(),
        cad_manifest_sha256=file_hash(camera_manifest),parts=keep,T_mast_CAD=C.tolist(),T_mast_optical=optical.tolist(),
        T_link1_optical=(np.array(mast['T_link1_mast'])@optical).tolist(),
        replaced_mast_parts=[p['id'] for p in mast['parts'] if p['id'] not in ['mast_part_001','mast_part_010','mast_part_019']],
        omitted_variants=['camera_part_000 (front shell V0)','camera_part_002 (internal bracket V0)'],
        retained_mast='Original silver base, upright/U-shaped camera pocket and J1 conformal seat',
        mounting_assumption='Rear cover dimensions matched to existing CAD pocket; fasteners not qualified',
        optical_assumption='RGB window XY centre; virtual entrance pupil 0.05 mm in front of housing front plane; not factory optical centre',
        configurable_optical_offset_m=list(optical_offset_m),unknown=['device resolution','factory K/distortion','RGB/depth extrinsics','optical centre','depth range/noise'],
        resolution=[width,height],fov_deg=[64,50],K=intrinsics(width,height).tolist(),distortion=[0.]*5,
        pixel_convention='Integer pixel centres; edges -0.5,N-0.5',source_resolution=[1024,1024],
        projection_adapter='Square-pixel 64x64 degree native render; explicit ray-preserving resampling to 64x50 degrees',
        depth='SYNTHETIC_IDEAL_REGISTERED_OPTICAL_Z; not device depth performance')
    # Derive probe clearance and bracket position from original rigid plate and
    # full suction-tool mesh bounds, without changing the existing suction TCP.
    F=np.array(s['tool']['T_flange_tool_cad']);vertices={}
    for p in s['tool']['visual_parts']:
        vertices[p['id']]=points(F,trimesh.load(ROOT/p['visual'],process=False).vertices)
    allv=np.concatenate(list(vertices.values()));plate=vertices['part_000'];plate_lo,plate_hi=plate.min(0),plate.max(0)
    x=float(allv[:,0].max()+.012);z0=float((plate_lo[2]+plate_hi[2])/2);ztip=float(allv[:,2].max()+.085)
    bracket=F@transform([.098,.005,0])
    probe=dict(p_F_tip_m=[x,0,ztip],known_tip='SIMULATION_INPUT_NOT_CALIBRATED',radius_m=.0025,
        rod_start_F_m=[x,0,z0],cone_length_m=.012,
        bracket_T_F=bracket.tolist(),bracket_size_m=[.016,.008,.012],
        plate_source='part_000',tool_bounds_F_m=[allv.min(0).tolist(),allv.max(0).tolist()],
        design_basis='2.5 mm rod outside measured tool X envelope by 12 mm; ideal tip 85 mm beyond tool Z envelope',
        original_T_flange_tcp=copy.deepcopy(s['tool']['T_flange_tcp']))
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board=cv2.aruco.CharucoBoard((6,5),.025,.018,dictionary)
    pattern=board.generateImage((1200,1000),marginSize=0,borderBits=1);cv2.imwrite(str(out/'board_pattern.png'),pattern)
    board_config=dict(dictionary='DICT_4X4_50',squares=[6,5],square_m=.025,marker_m=.018,
        pattern_size_m=[.15,.125],outer_size_m=[.20,.175],thickness_m=.008,stud_radius_m=.004,stud_projection_m=.006,
        charuco_points_P_m=board.getChessboardCorners().tolist(),opencv_version=cv2.__version__,
        touch_points=[dict(point_id=f'p{i}',p_P_m=[x,y,-.006]) for i,(x,y) in enumerate([(-.015,0),(.165,0),(-.015,.0625),(.165,.0625),(-.015,.125),(.165,.125)])])
    thresholds=dict(touch_residual_m=.0001,tip_target_m=.0001,minimum_charuco_corners=8,reprojection_rms_px=.8,
        pnp_ambiguity_gap_px=.03,extrinsic_consistency_m=.005,extrinsic_consistency_deg=1.,
        truth_translation_m=.005,truth_rotation_deg=1.,validation_point_m=.005,projection_error_px=1.,fov_error_deg=.15,
        scope='Predeclared simulation working targets, not device specifications')
    kin=dict(active_joint_names=robot.active_joint_names,base_link=robot.base_link,T_D_base=robot.base_transform.tolist(),
        limits_rad=robot.joint_limits.tolist(),urdf_sha256=file_hash(URDF),
        joints=[dict(name=j.name,type=j.joint_type,parent=j.parent,child=j.child,origin=j.origin.tolist(),axis=j.axis.tolist()) for j in robot.joints])
    fixture_preparation=None
    if installation is not None:
        from eco65_touch_sources import verify_scene_sources
        new=load_json(installation)
        if verify_scene_sources(new)!=sources:raise ValueError('INSTALLATION_SOURCE_MISMATCH')
        if (new['camera']['source_sha256']!=camera['source_sha256'] or new['camera']['K']!=camera['K'] or
            new['camera']['parts']!=camera['parts'] or new['probe']['original_T_flange_tcp']!=probe['original_T_flange_tcp'] or
            new['board']['touch_points']!=board_config['touch_points']):raise ValueError('INSTALLATION_CHANGES_PINNED_DEFINITION')
        camera=copy.deepcopy(new['camera']);probe=copy.deepcopy(new['probe']);board_config=copy.deepcopy(new['board'])
        fixture_preparation=copy.deepcopy(new['fixture_preparation'])
    configuration=dict(snapshot=snapshot_path.relative_to(ROOT).as_posix(),snapshot_sha256=file_hash(snapshot_path),source_fingerprint=s['fingerprint'],
        scene_sources=sources,
        camera=camera,probe=probe,board=board_config,thresholds=thresholds,kinematics=kin,display_q=pose['home_q'],
        statuses=['SYNTHETIC_GEOMETRIC_TOUCH','PHYSICAL_CONTACT_NOT_EVALUATED','NO_MOTION_EXECUTED'])
    if fixture_preparation:configuration['fixture_preparation']=fixture_preparation
    configuration['derived_scene_identity']=derived_scene_identity(configuration)
    save_json(out/'configuration.json',configuration) # thresholds written BEFORE search/render/solve
    calls=[];attempts=[];placements=[];touches=[];observations=[]
    client=ResidentMoveItClient(worker,log_path=out/'worker.log');init=s['native_model']['init'];identity=init['identity']
    def call(req):
        t=time.perf_counter();row=dict(request=req);calls.append(row)
        try:row['result']=client.request(req,timeout=35);return row['result']
        finally:row['seconds']=time.perf_counter()-t
    try:
        ready=call(init)
        if ready.get('status')!='READY':raise ValueError('NATIVE_INIT_REJECTED')
        # Actual worker FK is cross-checked; these calls do not plan motions.
        fkr=call(dict(op='fk',identity=identity,q_start=pose['home_q'],links=['link_1','link_6']))
        save_json(out/'native_fk.json',fkr)
        expected_frames=robot.named_link_frames(pose['home_q'])
        if fkr.get('status')!='SUCCESS' or any(not np.allclose(fkr['frames'][name],expected_frames[name],atol=1e-8,rtol=0) for name in ['link_1','link_6']):
            raise ValueError('NATIVE_FK_FRAME_MISMATCH')
        from eco65_touch_geometry import StaticGeometry
        geometry=StaticGeometry(s,configuration,out)
        rng=np.random.default_rng(6516)
        boardR=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
        if fixture_preparation:boardR=np.array(fixture_preparation['board_rotation_D'])
        heldout_offset=np.array(fixture_preparation['heldout_translation_D_m'] if fixture_preparation else [.015,.025,-.02])
        for placement_id,split,offset,indices in [('train_01','train',np.zeros(3),range(6)),('heldout_01','validation',heldout_offset,[0,3,5])]:
            found=False
            candidates=[[-.73,.20,.70],[-.76,.20,.72],[-.79,.20,.70]] if split=='train' else [placements[0]['center_D_m']]
            if fixture_preparation and split=='train':candidates=fixture_preparation['candidate_centers_D_m']
            for center in candidates:
                center=np.array(center)+offset;P=transform(center-boardR@np.array([.075,.0625,0]),boardR)
                proposed=[];qseed=np.array(pose['home_q'])
                for index in indices:
                    point=board_config['touch_points'][index];goal=points(P,point['p_P_m']);flange=transform(goal-boardR@np.array(probe['p_F_tip_m']),boardR)
                    tcp=flange@np.array(s['tool']['T_flange_tcp']);accepted=None
                    seeds=[*[np.array(t['q_rad']) for t in touches if t['point_id']==point['point_id']],qseed,
                        *[np.array(t['q_rad']) for t in proposed],*rng.uniform(robot.joint_limits[:,0],robot.joint_limits[:,1],(11,6))]
                    goals=[tcp]
                    rolled=boardR@Rotation.from_euler('z',180,degrees=True).as_matrix()
                    goals.append(transform(goal-rolled@np.array(probe['p_F_tip_m']),rolled)@s['tool']['T_flange_tcp'])
                    for tcp_goal,seed in [(g,seed) for g in goals for seed in seeds]:
                        reply=call(dict(op='ik',identity=identity,q_start=seed.tolist(),goal_pose=tcp_goal.tolist(),seed=6516+len(calls),
                            planning_only=True,planning_mode='cold_from_scratch',position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4))
                        if reply.get('status')!='SUCCESS':continue
                        if reply.get('native_ik_calls')!=1:raise ValueError('NATIVE_IK_PROVENANCE_MISSING')
                        q=np.array(reply['q']);tip=points(robot.named_link_frames(q)['link_6'],probe['p_F_tip_m']);error=float(np.linalg.norm(tip-goal))
                        check=geometry.check(q,P,point['point_id'])
                        attempts.append(dict(placement=placement_id,center=center.tolist(),point=point['point_id'],q_rad=q.tolist(),tip_error_m=error,geometry=check))
                        if error<thresholds['tip_target_m'] and check['accepted']:
                            accepted=dict(sample_id=placement_id+'_touch_'+point['point_id'],board_placement_id=placement_id,split=split,point_id=point['point_id'],
                                q_rad=q.tolist(),probe_id=digest(probe),tip_error_m=error,geometry=check,
                                status='SYNTHETIC_GEOMETRIC_TOUCH',physical_contact='PHYSICAL_CONTACT_NOT_EVALUATED')
                            qseed=q;break
                    if accepted is None:break
                    proposed.append(accepted)
                if len(proposed)==len(list(indices)):
                    placements.append(dict(board_placement_id=placement_id,split=split,T_D_P=P.tolist(),center_D_m=center.tolist()))
                    touches.extend(proposed);found=True;break
            if not found:raise ValueError('BOUNDED_FIXTURE_IK_OR_GEOMETRY_SEARCH_EXHAUSTED:'+placement_id)
            angles=np.radians([82,90,98])
            if fixture_preparation:
                from eco65_j1_visualization import face_angle
                centre_angle,_,_=face_angle(robot,pose['home_q'],np.array(camera['T_link1_optical']),center,robot.joint_limits[0],require_away=False)
                angles=centre_angle+np.radians(fixture_preparation['view_offsets_deg'])
            for i,angle in enumerate(angles):
                q=np.array(pose['home_q']);q[0]=angle
                check=geometry.check(q,P,None)
                observations.append(dict(sample_id=placement_id+'_photo_'+str(i),board_placement_id=placement_id,split=split,q_rad=q.tolist(),static_check=check))
                if not check['accepted']:raise ValueError('CAPTURE_POSE_STATIC_GEOMETRY_REJECTED')
            configuration.update(placements=placements,photos=observations,touches=touches)
            save_json(out/'configuration.json',configuration)
        configuration.update(placements=placements,photos=observations,touches=touches)
        save_json(out/'configuration.json',configuration)
    finally:
        client.close();save_json(out/'native_calls.json',calls);save_json(out/'search_report.json',dict(attempts=attempts,seconds=time.perf_counter()-start,scope='FINITE_STATIC_IK_NO_PATHS'))
    print('CALIBRATION_PREPARED',len(touches),len(observations),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ['snapshot','pose','camera-manifest','output','composed-usd','geometry-manifest']:p.add_argument('--'+arg,type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480)
    p.add_argument('--optical-offset-m',nargs=3,type=float,default=[0.,0.,0.]);p.add_argument('--installation',type=Path);a=p.parse_args()
    if min(a.width,a.height)<64:p.error('Simulation image dimensions must be >=64')
    prepare(a.snapshot,a.pose,a.camera_manifest,a.output,a.worker_command,a.width,a.height,a.optical_offset_m,
        composed_usd=a.composed_usd,geometry_manifest=a.geometry_manifest,installation=a.installation)
