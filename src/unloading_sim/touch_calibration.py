"""Absolute-board fixed extrinsics from images and known-tip geometric touches.

This module never accepts a scene or camera/board truth transform. It has no
Isaac/ROS dependency. T_A_B maps B coordinates into A; all distances are metres.
"""
from pathlib import Path
import json
import hashlib
import re
import numpy as np
from scipy.spatial.transform import Rotation


def rigid(T):
    T=np.asarray(T,float)
    if (T.shape!=(4,4) or not np.isfinite(T).all() or
        not np.allclose(T[3],[0,0,0,1],atol=1e-10,rtol=0) or
        not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-8,rtol=0) or
        abs(np.linalg.det(T[:3,:3])-1)>1e-8):raise ValueError('INVALID_RIGID_TRANSFORM')
    return T


def inv(T):
    T=rigid(T);out=np.eye(4);out[:3,:3]=T[:3,:3].T;out[:3,3]=-T[:3,:3].T@T[:3,3]
    return out


def points(T,p):
    T=rigid(T);return np.asarray(p)@T[:3,:3].T+T[:3,3]


def register_points(local,measured,min_span_m=.01):
    a=np.asarray(local,float);b=np.asarray(measured,float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=3 or len(a)<3 or not np.isfinite([a,b]).all():
        raise ValueError('INVALID_POINT_CORRESPONDENCES')
    ac=a-a.mean(0);bc=b-b.mean(0)
    singular=np.linalg.svd(ac,compute_uv=False)/np.sqrt(len(a))
    if singular[1]<min_span_m:raise ValueError('NEAR_COLLINEAR_OR_INSUFFICIENT_SPAN')
    if np.linalg.svd(bc,compute_uv=False)[1]/np.sqrt(len(b))<min_span_m:raise ValueError('MEASURED_POINTS_DEGENERATE')
    u,s,vt=np.linalg.svd(ac.T@bc);fix=np.eye(3);fix[2,2]=np.linalg.det(vt.T@u.T)
    R=vt.T@fix@u.T;T=np.eye(4);T[:3,:3]=R;T[:3,3]=b.mean(0)-R@a.mean(0)
    residual=points(T,a)-b
    return rigid(T),dict(singular_spans_m=singular.tolist(),residual_xyz_m=residual.tolist(),
        rms_m=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),max_m=float(np.linalg.norm(residual,axis=1).max()))


def fk(kinematics,q):
    q=np.asarray(q,float);names=kinematics['active_joint_names']
    limits=np.array(kinematics['limits_rad'])
    if q.shape!=(len(names),) or not np.isfinite(q).all() or np.any(q<limits[:,0]) or np.any(q>limits[:,1]):
        raise ValueError('INVALID_JOINT_STATE')
    T=rigid(kinematics['T_D_base']).copy();result={kinematics['base_link']:T.copy()}
    for j in kinematics['joints']:
        T=T@rigid(j['origin'])
        if j['name'] in names:
            if j['type']!='revolute':raise ValueError('UNSUPPORTED_CALIBRATION_JOINT')
            axis=np.array(j['axis']);motion=np.eye(4)
            motion[:3,:3]=Rotation.from_rotvec(axis*q[names.index(j['name'])]).as_matrix();T=T@motion
        result[j['child']]=T.copy()
    return result


def intrinsics(width=640,height=480,hfov_deg=64.,vfov_deg=50.):
    # Integer indices locate pixel centres; image edges are -0.5 and N-0.5.
    return np.array([[width/(2*np.tan(np.radians(hfov_deg)/2)),0,(width-1)/2],
                     [0,height/(2*np.tan(np.radians(vfov_deg)/2)),(height-1)/2],[0,0,1.]])


OPTICAL_FROM_USD=np.diag([1.,-1.,-1.,1.])


def fuse(transforms):
    transforms=np.array([rigid(T) for T in transforms]);out=np.eye(4)
    out[:3,:3]=Rotation.from_matrix(transforms[:,:3,:3]).mean().as_matrix()
    out[:3,3]=transforms[:,:3,3].mean(0)
    errors=[dict(translation_m=float(np.linalg.norm(T[:3,3]-out[:3,3])),
        rotation_deg=float(np.degrees(Rotation.from_matrix(out[:3,:3].T@T[:3,:3]).magnitude()))) for T in transforms]
    return out,errors


def camera_from_board(image,board_config,K,thresholds,overlay_path=None):
    import cv2
    dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,board_config['dictionary']))
    board=cv2.aruco.CharucoBoard(tuple(board_config['squares']),board_config['square_m'],board_config['marker_m'],dictionary)
    if not np.allclose(board.getChessboardCorners(),board_config['charuco_points_P_m'],atol=1e-8):
        raise ValueError('OPENCV_BOARD_COORDINATE_MISMATCH')
    params=cv2.aruco.CharucoParameters();params.cameraMatrix=np.array(K);params.distCoeffs=np.zeros(5)
    corners,ids,mc,mi=cv2.aruco.CharucoDetector(board,params).detectBoard(image)
    if ids is None or len(ids)<thresholds['minimum_charuco_corners']:raise ValueError('INSUFFICIENT_VISUAL_FEATURES')
    obj,img=board.matchImagePoints(corners,ids);obj=np.asarray(obj).reshape(-1,3);img=np.asarray(img).reshape(-1,2)
    if np.linalg.svd(obj-obj.mean(0),compute_uv=False)[1]<.01:raise ValueError('VISUAL_CORNERS_COLLINEAR')
    result=cv2.solvePnPGeneric(obj,img,np.array(K),np.zeros(5),flags=cv2.SOLVEPNP_IPPE)
    candidates=[]
    for rv,tv in zip(result[1],result[2]):
        R=cv2.Rodrigues(rv)[0];T=np.eye(4);T[:3,:3]=R;T[:3,3]=tv.ravel()
        projected=cv2.projectPoints(obj,rv,tv,K,np.zeros(5))[0].reshape(-1,2)
        rms=float(np.sqrt(np.mean(np.sum((projected-img)**2,axis=1))))
        candidates.append(dict(T_C_P=T.tolist(),rms_px=rms,positive_depth=bool(np.all(points(T,obj)[:,2]>0)),
                               printed_face_forward=bool(R[:,2]@tv.ravel()>0)))
    valid=sorted([c for c in candidates if c['positive_depth'] and c['printed_face_forward']],key=lambda c:c['rms_px'])
    if not valid:raise ValueError('PNP_CHEIRALITY_OR_BOARD_FACE')
    if len(valid)>1 and valid[1]['rms_px']-valid[0]['rms_px']<thresholds['pnp_ambiguity_gap_px']:
        raise ValueError('AMBIGUOUS_PLANAR_PNP')
    best=valid[0];T=np.array(best['T_C_P']);rv=cv2.Rodrigues(T[:3,:3])[0];tv=T[:3,3].copy()
    rv,tv=cv2.solvePnPRefineLM(obj,img,K,np.zeros(5),rv,tv)
    T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv.ravel()
    if not np.all(points(T,obj)[:,2]>0) or T[:3,2]@T[:3,3]<=0:
        raise ValueError('REFINED_PNP_CHEIRALITY_OR_BOARD_FACE')
    projected=cv2.projectPoints(obj,rv,tv,K,np.zeros(5))[0].reshape(-1,2)
    rms=float(np.sqrt(np.mean(np.sum((projected-img)**2,axis=1))))
    if rms>thresholds['reprojection_rms_px']:raise ValueError('REPROJECTION_ERROR')
    if overlay_path:
        overlay=image.copy();cv2.aruco.drawDetectedCornersCharuco(overlay,corners,ids)
        cv2.drawFrameAxes(overlay,K,np.zeros(5),rv,tv,.05);cv2.imwrite(str(overlay_path),overlay)
    return rigid(T),dict(corner_count=len(ids),ids=ids.ravel().tolist(),coverage_m=np.ptp(obj,axis=0).tolist(),
        reprojection_rms_px=rms,planar_candidates=candidates,opencv_version=cv2.__version__,
        coordinate_convention='OpenCV runtime board.getChessboardCorners; P +X image right,+Y down,+Z into board')


def read_observation_image(root,photo,camera):
    import cv2
    root=Path(root).resolve();path=(root/photo['image']).resolve();path.relative_to(root)
    if hashlib.sha256(path.read_bytes()).hexdigest()!=photo['sha256']:raise ValueError('OBSERVATION_IMAGE_CHANGED')
    image=cv2.imread(str(path))
    if image is None:raise ValueError('IMAGE_NOT_READABLE')
    if list(image.shape[1::-1])!=camera['resolution']:raise ValueError('IMAGE_RESOLUTION_MISMATCH')
    return image


RESULT_CONTRACT = dict(schema='known_tip_fixed_extrinsics_v2',parent_frame='link_1',child_frame='camera_optical',
    direction='T_parent_child maps camera_optical coordinates into link_1',translation_unit='metre',
    rotation='3x3 proper rotation matrix',optical_axes='+X right,+Y down,+Z forward')
# An acceptance profile for this experiment, not a theoretical calibration minimum.
STAGE1_PROFILE = dict(name='known_tip_stage1_acceptance_v1',training_touch_points=6,
    validation_touch_points=3,training_images=3,validation_images=3,distinct_J1_views=3,
    J1_separation_rad=1e-6,point_span_m=.01)
TRAINING_CHECKS = ('training_touch_registration','training_image_reprojection','multi_observation_consistency')


def fingerprint(value):
    """SHA256 of complete JSON value: sorted keys, compact separators, UTF-8, no NaN."""
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf8')).hexdigest()


def observation_binding(obs):
    return dict(schema='complete_observations_binding_v1',algorithm='sha256',
        canonicalization='JSON sorted keys, compact separators, ensure_ascii=True, UTF-8, finite numbers',
        scope='Entire parsed observations.json, including all images/hashes, q, K/distortion, definitions, groups, thresholds and metadata',
        sha256=fingerprint(obs))


def definition_fingerprints(obs):
    # These are NOT compared with the legacy full-camera configuration identifier.
    return {key:fingerprint(obs[key]) for key in ('camera','probe','board','kinematics')}


def sample_groups(obs):
    return dict(training_samples=[p['sample_id'] for p in obs['photos'] if p['split']=='train'],
        held_out_samples=[p['sample_id'] for p in obs['photos'] if p['split']=='validation'],
        training_touch_samples=[t['sample_id'] for t in obs['touches'] if t['split']=='train'],
        held_out_touch_samples=[t['sample_id'] for t in obs['touches'] if t['split']=='validation'])


def validate_observations(obs):
    allowed={'schema','kinematics','camera','board','probe','photos','touches','thresholds','source','identifiers'}
    if set(obs)!=allowed or obs['schema']!='known_tip_touch_observations_v1':raise ValueError('OBSERVATIONS_ONLY_SCHEMA_REQUIRED')
    fingerprint(obs) # Reject nonfinite JSON anywhere, including unused diagnostic fields.
    for photo in obs['photos']:
        if not any(t['board_placement_id']==photo['board_placement_id'] and t['split']==photo['split'] for t in obs['touches']):
            raise ValueError('BOARD_PLACEMENT_OR_SPLIT_MISMATCH')
    if obs['source']!='SIMULATION':raise ValueError('SIMULATION_OBSERVATIONS_REQUIRED')
    ids=obs['identifiers']
    if not all(re.fullmatch('[0-9a-f]{64}',ids.get(k,'')) for k in ('camera','probe','board')):
        raise ValueError('INVALID_CONFIGURATION_IDENTIFIERS')
    for key in ('probe','board'):
        if ids[key]!=fingerprint(obs[key]):raise ValueError('CONFIGURATION_IDENTITY_MISMATCH:'+key)
    camera=obs['camera'];K=np.asarray(camera['K'],float)
    if (K.shape!=(3,3) or not np.isfinite(K).all() or min(K[0,0],K[1,1])<=0 or
        not np.array_equal(K[2],[0,0,1]) or np.any(np.asarray(camera['distortion'])!=0) or
        camera['pixel_convention']!='Integer pixel centres; edges -0.5,N-0.5'):
        raise ValueError('UNSUPPORTED_CAMERA_CONVENTION')
    if len(camera['resolution'])!=2 or any(type(n)!=int or n<1 for n in camera['resolution']):raise ValueError('INVALID_RESOLUTION')
    required=('touch_residual_m','minimum_charuco_corners','reprojection_rms_px','pnp_ambiguity_gap_px',
        'extrinsic_consistency_m','extrinsic_consistency_deg','truth_translation_m','truth_rotation_deg','validation_point_m')
    if any(not np.isfinite(obs['thresholds'][k]) or obs['thresholds'][k]<=0 for k in required):raise ValueError('INVALID_THRESHOLDS')
    point_ids=[p['point_id'] for p in obs['board']['touch_points']]
    if len(point_ids)!=len(set(point_ids)):raise ValueError('DUPLICATE_BOARD_POINT_ID')
    samples=obs['photos']+obs['touches'];names=[p['sample_id'] for p in samples]
    if len(names)!=len(set(names)):raise ValueError('DUPLICATE_SAMPLE_ID')
    if any(not re.fullmatch('[A-Za-z0-9_-]+',n) for n in names):raise ValueError('INVALID_SAMPLE_ID')
    touch_keys=[(t['board_placement_id'],t['point_id']) for t in obs['touches']]
    if len(touch_keys)!=len(set(touch_keys)):raise ValueError('DUPLICATE_TOUCH_POINT_RECORD')
    groups={}
    for row in samples:
        if row['split'] not in ('train','validation'):raise ValueError('INVALID_SAMPLE_SPLIT')
        previous=groups.setdefault(row['board_placement_id'],row['split'])
        if previous!=row['split']:raise ValueError('BOARD_PLACEMENT_OR_SPLIT_MISMATCH')
        frames=fk(obs['kinematics'],row['q_rad'])
        if not {'link_1','link_6'}<=frames.keys():raise ValueError('REQUIRED_KINEMATIC_FRAMES_MISSING')
    for photo in obs['photos']:
        if photo['camera_configuration_id']!=ids['camera']:raise ValueError('CAMERA_IDENTITY_MISMATCH')
        if not np.array_equal(photo['K'],K):raise ValueError('PHOTO_INTRINSICS_MISMATCH')
        if not re.fullmatch('[0-9a-f]{64}',photo['sha256']):raise ValueError('INVALID_IMAGE_HASH')
        if not any(t['board_placement_id']==photo['board_placement_id'] for t in obs['touches']):
            raise ValueError('BOARD_PLACEMENT_OR_SPLIT_MISMATCH')
    for touch in obs['touches']:
        if touch['point_id'] not in point_ids:raise ValueError('UNKNOWN_TOUCH_POINT')
        if touch['probe_id']!=ids['probe']:raise ValueError('PROBE_IDENTITY_MISMATCH')
    return obs


def coverage(obs,split):
    profile=STAGE1_PROFILE;records=[]
    placements=sorted({p['board_placement_id'] for p in obs['photos']+obs['touches'] if p['split']==split})
    if not placements:raise ValueError('COVERAGE_NO_'+split.upper()+'_PLACEMENT')
    local={p['point_id']:p['p_P_m'] for p in obs['board']['touch_points']}
    for placement in placements:
        photos=[p for p in obs['photos'] if p['board_placement_id']==placement]
        touches=[t for t in obs['touches'] if t['board_placement_id']==placement]
        group='training' if split=='train' else 'validation'
        if len(photos)<profile[group+'_images'] or len(touches)<profile[group+'_touch_points']:
            raise ValueError('COVERAGE_INSUFFICIENT_'+group.upper()+':'+placement)
        angles=sorted(p['q_rad'][obs['kinematics']['active_joint_names'].index('joint_1')] for p in photos)
        distinct=[]
        for angle in angles:
            if not any(abs(np.arctan2(np.sin(angle-a),np.cos(angle-a)))<=profile['J1_separation_rad'] for a in distinct):distinct.append(angle)
        if len(distinct)<profile['distinct_J1_views']:raise ValueError('COVERAGE_DISTINCT_J1_VIEWS:'+placement)
        measured=[points(fk(obs['kinematics'],t['q_rad'])['link_6'],obs['probe']['p_F_tip_m']) for t in touches]
        _,span=register_points([local[t['point_id']] for t in touches],measured,profile['point_span_m'])
        records.append(dict(board_placement_id=placement,images=len(photos),distinct_J1_rad=distinct,
            distinct_touch_points=len(touches),singular_spans_m=span['singular_spans_m']))
    return dict(status='PASS',profile=profile,placements=records)


def training_evidence(obs,root,output):
    """Shared light recomputation; never trust stored PASS flags as evidence."""
    thresholds=obs['thresholds'];kin=obs['kinematics'];tip=obs['probe']['p_F_tip_m']
    local={r['point_id']:r['p_P_m'] for r in obs['board']['touch_points']}
    groups={};registrations={};detections=[];estimates=[]
    for t in obs['touches']:
        groups.setdefault(t['board_placement_id'],[]).append(t)
    for photo in obs['photos']:
        group=groups.get(photo['board_placement_id'],[])
        if not group or any(t['split']!=photo['split'] for t in group):raise ValueError('BOARD_PLACEMENT_OR_SPLIT_MISMATCH')
        if photo['split']!='train':continue
        pid=photo['board_placement_id']
        if pid not in registrations:
            measured=[points(fk(kin,t['q_rad'])['link_6'],tip) for t in group]
            T,report=register_points([local[t['point_id']] for t in group],measured)
            registrations[pid]=dict(T_D_P=T.tolist(),**report)
        image=read_observation_image(root,photo,obs['camera'])
        C,report=camera_from_board(image,obs['board'],np.array(obs['camera']['K']),thresholds,output/(photo['sample_id']+'_detected.png'))
        X=inv(fk(kin,photo['q_rad'])['link_1'])@np.array(registrations[pid]['T_D_P'])@inv(C)
        detections.append(dict(sample_id=photo['sample_id'],T_C_P=C.tolist(),**report));estimates.append(X)
    X,consistency=fuse(estimates)
    checks=dict(training_touch_registration=dict(status='PASS' if all(r['max_m']<=thresholds['touch_residual_m'] for r in registrations.values()) else 'FAIL',
        maximum_residual_m=max(r['max_m'] for r in registrations.values()),threshold_m=thresholds['touch_residual_m']),
        training_image_reprojection=dict(status='PASS' if all(d['reprojection_rms_px']<=thresholds['reprojection_rms_px'] for d in detections) else 'FAIL',
            maximum_rms_px=max(d['reprojection_rms_px'] for d in detections),threshold_px=thresholds['reprojection_rms_px']),
        multi_observation_consistency=dict(status='PASS' if all(e['translation_m']<=thresholds['extrinsic_consistency_m'] and e['rotation_deg']<=thresholds['extrinsic_consistency_deg'] for e in consistency) else 'FAIL',
            maximum_translation_m=max(e['translation_m'] for e in consistency),maximum_rotation_deg=max(e['rotation_deg'] for e in consistency),
            threshold_m=thresholds['extrinsic_consistency_m'],threshold_deg=thresholds['extrinsic_consistency_deg']))
    return dict(T_parent_child=X.tolist(),registrations=registrations,detections=detections,
        individual_X=[T.tolist() for T in estimates],consistency=consistency,training_checks=checks)


def validate_result(result,obs):
    for key,value in RESULT_CONTRACT.items():
        if result.get(key)!=value:raise ValueError('RESULT_CONTRACT_MISMATCH:'+key)
    rigid(result['T_parent_child'])
    if result.get('status')!='SOLVED_INDEPENDENT_VALIDATION_REQUIRED':raise ValueError('RESULT_NOT_ELIGIBLE_FOR_VALIDATION')
    if result.get('identifiers')!=obs['identifiers']:raise ValueError('RESULT_CONFIGURATION_IDENTITY_MISMATCH')
    if result.get('definition_fingerprints')!=definition_fingerprints(obs):raise ValueError('RESULT_DEFINITION_IDENTITY_MISMATCH')
    if result.get('observation_binding')!=observation_binding(obs):raise ValueError('OBSERVATION_BINDING_MISMATCH_RESOLVE_REQUIRED')
    if result.get('acceptance_profile')!=STAGE1_PROFILE:raise ValueError('ACCEPTANCE_PROFILE_MISMATCH')
    for key,ids in sample_groups(obs).items():
        if result.get(key)!=ids:raise ValueError('RESULT_SAMPLE_GROUP_MISMATCH:'+key)
    for key in TRAINING_CHECKS:
        if result.get('training_checks',{}).get(key,{}).get('status')!='PASS':raise ValueError('TRAINING_EVIDENCE_MISSING_OR_FAILED:'+key)
    return result


def solve(observation_directory,output):
    """Only observations and hash-verified images; no scene/truth access."""
    root=Path(observation_directory);obs=validate_observations(json.loads((root/'observations.json').read_text(encoding='utf8')))
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    covered=coverage(obs,'train');evidence=training_evidence(obs,root,output)
    passed=all(c['status']=='PASS' for c in evidence['training_checks'].values())
    result=dict(**RESULT_CONTRACT,**evidence,identifiers=obs['identifiers'],definition_fingerprints=definition_fingerprints(obs),
        observation_binding=observation_binding(obs),acceptance_profile=STAGE1_PROFILE,training_coverage=covered,
        source='SYNTHETIC_GEOMETRIC_TOUCH_AND_RENDERED_RGB',probe_tip='KNOWN_SIMULATION_INPUT_NOT_SELF_CALIBRATED',
        **sample_groups(obs),status='SOLVED_INDEPENDENT_VALIDATION_REQUIRED' if passed else 'SOLVE_FAILED',
        motion='NO_MOTION_EXECUTED',physical_contact='NOT_EVALUATED')
    (output/'extrinsics.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf8')
    return result


def save_failure(output,error,filename):
    """CLI callers only use a fresh output; never replace an earlier result."""
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    report=dict(status='FAIL',error=type(error).__name__+': '+str(error),
        checks={name:dict(status='NOT_EVALUATED') for name in (*TRAINING_CHECKS,'held_out_point_error','held_out_multi_J1_consistency','simulation_truth_error')})
    with (output/filename).open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report
