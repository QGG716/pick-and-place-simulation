"""Absolute-board fixed extrinsics from images and known-tip geometric touches.

This module never accepts a scene or camera/board truth transform. It has no
Isaac/ROS dependency. T_A_B maps B coordinates into A; all distances are metres.
"""
from pathlib import Path
import json
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation


def rigid(T):
    T=np.asarray(T,float)
    if (T.shape!=(4,4) or not np.isfinite(T).all() or
        not np.allclose(T[3],[0,0,0,1],atol=1e-10) or
        not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-8) or
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


def solve(observation_directory,output):
    """Only the exported observation manifest and its images are accessible here."""
    import cv2
    root=Path(observation_directory);obs=json.loads((root/'observations.json').read_text())
    allowed={'schema','kinematics','camera','board','probe','photos','touches','thresholds','source','identifiers'}
    if set(obs)!=allowed or obs['schema']!='known_tip_touch_observations_v1':raise ValueError('OBSERVATIONS_ONLY_SCHEMA_REQUIRED')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    thresholds=obs['thresholds'];kin=obs['kinematics'];tip=obs['probe']['p_F_tip_m']
    local={r['point_id']:r['p_P_m'] for r in obs['board']['touch_points']}
    groups={};registrations={};detections=[];estimates=[]
    for t in obs['touches']:
        if t['point_id'] not in local:raise ValueError('UNKNOWN_TOUCH_POINT')
        groups.setdefault(t['board_placement_id'],[]).append(t)
    for photo in obs['photos']:
        group=groups.get(photo['board_placement_id'],[])
        if not group or any(t['split']!=photo['split'] for t in group):raise ValueError('BOARD_PLACEMENT_OR_SPLIT_MISMATCH')
        if photo['split']!='train':continue
        pid=photo['board_placement_id']
        if pid not in registrations:
            if len({t['point_id'] for t in group})<6:raise ValueError('SIX_TRAINING_TOUCHES_REQUIRED')
            measured=[points(fk(kin,t['q_rad'])['link_6'],tip) for t in group]
            T,report=register_points([local[t['point_id']] for t in group],measured)
            if report['max_m']>thresholds['touch_residual_m']:raise ValueError('TOUCH_REGISTRATION_ERROR')
            registrations[pid]=dict(T_D_P=T.tolist(),**report)
        image=read_observation_image(root,photo,obs['camera'])
        C,report=camera_from_board(image,obs['board'],np.array(obs['camera']['K']),thresholds,output/(photo['sample_id']+'_detected.png'))
        X=inv(fk(kin,photo['q_rad'])['link_1'])@np.array(registrations[pid]['T_D_P'])@inv(C)
        detections.append(dict(sample_id=photo['sample_id'],T_C_P=C.tolist(),**report));estimates.append(X)
    if len(estimates)<3:raise ValueError('THREE_TRAINING_IMAGES_REQUIRED')
    X,consistency=fuse(estimates)
    if any(e['translation_m']>thresholds['extrinsic_consistency_m'] or e['rotation_deg']>thresholds['extrinsic_consistency_deg'] for e in consistency):
        raise ValueError('CAPTURE_JOINT_OR_OBSERVATION_INCONSISTENCY')
    result=dict(schema='known_tip_fixed_extrinsics_v1',parent_frame='link_1',child_frame='camera_optical',
        direction='T_parent_child maps camera_optical coordinates into link_1',translation_unit='metre',rotation='3x3 proper rotation matrix',
        optical_axes='+X right,+Y down,+Z forward',T_parent_child=X.tolist(),identifiers=obs['identifiers'],
        source='SYNTHETIC_GEOMETRIC_TOUCH_AND_RENDERED_RGB',probe_tip='KNOWN_SIMULATION_INPUT_NOT_SELF_CALIBRATED',
        training_samples=[p['sample_id'] for p in obs['photos'] if p['split']=='train'],
        held_out_samples=[p['sample_id'] for p in obs['photos'] if p['split']=='validation'],
        training_touch_samples=[t['sample_id'] for t in obs['touches'] if t['split']=='train'],
        held_out_touch_samples=[t['sample_id'] for t in obs['touches'] if t['split']=='validation'],
        registrations=registrations,detections=detections,individual_X=[T.tolist() for T in estimates],consistency=consistency,
        status='SOLVED_INDEPENDENT_VALIDATION_REQUIRED',motion='NO_MOTION_EXECUTED',physical_contact='NOT_EVALUATED')
    (output/'extrinsics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    return result
