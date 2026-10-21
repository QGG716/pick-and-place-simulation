"""Fresh pixel-based simulation calibration and constrained RGB-D box detection.

TCP FK and detected board pixels determine extrinsics in touch_calibration.
This module binds those validated observations to the current planning world.
Hidden box faces use the explicitly supplied size/alignment prior.
"""
from pathlib import Path
from time import perf_counter
import hashlib
import numpy as np
from ..moveit2_backend import digest
from .model import load_json, save_json


def signed(data):
    return dict(data, fingerprint=digest(data))


def verify_record(data):
    if digest({k:v for k,v in data.items() if k!='fingerprint'})!=data['fingerprint']:
        raise ValueError('PERCEPTION_RECORD_CHANGED')
    return data


def read_image(root, row):
    import cv2
    path = Path(root)/row['image']
    if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
        raise ValueError('PERCEPTION_IMAGE_CHANGED')
    image = cv2.imread(str(path))
    if image is None: raise ValueError('PERCEPTION_IMAGE_UNREADABLE')
    return image


def require_tcp_calibration(cal):
    """A low reprojection error alone cannot qualify a TCP calibration."""
    verify_record(cal)
    if (cal.get('calibration_method')!='TCP_FK_REFERENCED_CAMERA_EXTRINSICS' or
            cal.get('status')!='PASS' or not cal.get('tcp_definition_fingerprint') or
            not cal.get('tcp_observation_binding') or
            cal.get('tcp_heldout_validation',{}).get('status')!='PASS'):
        raise ValueError('TCP_REFERENCED_CALIBRATION_REQUIRED')
    from ..touch_calibration import validate_observations, validate_result, observation_binding, fingerprint
    obs=validate_observations(cal['tcp_observations']); result=cal['tcp_solver_result']
    validate_result(result,obs)
    validation=cal['tcp_heldout_validation']
    if (cal['tcp_definition_fingerprint']!=fingerprint(obs['probe']) or
            cal['tcp_observation_binding']!=observation_binding(obs) or
            validation['observation_binding']!=observation_binding(obs) or
            any(c['status']!='PASS' for c in validation['checks'].values()) or
            not np.array_equal(cal['T_link1_optical'],result['T_parent_child'])):
        raise ValueError('TCP_CALIBRATION_EVIDENCE_BINDING_FAILED')
    return cal


def detect(observations_path, calibration_path, output, *, carton_size=(.15,.1,.1)):
    """Observe three top cartons from rendered RGB and Z depth, without GT masks."""
    import cv2
    started = perf_counter(); path = Path(observations_path); root = path.parent
    obs = verify_record(load_json(path)); cal = verify_record(load_json(calibration_path))
    require_tcp_calibration(cal)
    if (cal['status']!='PASS' or cal['observation_fingerprint']!=obs['fingerprint'] or
            cal['installation_fingerprint']!=obs['installation_fingerprint']):
        raise ValueError('FRESH_VALIDATED_CALIBRATION_REQUIRED')
    out = Path(output); out.mkdir(parents=True,exist_ok=False)
    row = obs['carton_image']; image = read_image(root,row); depth_path = root/row['depth']
    if hashlib.sha256(depth_path.read_bytes()).hexdigest()!=row['depth_sha256']:
        raise ValueError('PERCEPTION_DEPTH_CHANGED')
    depth = np.load(depth_path); K = np.asarray(cal['K']); T = np.asarray(row['T_D_link1'])@cal['T_link1_optical']
    v,u = np.indices(depth.shape); valid = np.isfinite(depth)&(depth>.05)&(depth<2.)
    z = np.where(valid,depth,0.); p = np.stack(((u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z),axis=-1)
    world = p@T[:3,:3].T+T[:3,3]
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # Broad working envelope, not expected face/target coordinates.
    mask = valid&(world[...,0]>.04)&(world[...,0]<.45)&(abs(world[...,1])<.25)&(world[...,2]>.2)&(world[...,2]<.7)
    mask &= (hsv[...,0]>3)&(hsv[...,0]<40)&(hsv[...,1]>28)&(hsv[...,2]>35)
    pts = world[mask]
    if len(pts)<2000: raise ValueError('INSUFFICIENT_CARTON_RGBD_PIXELS:'+str(len(pts)))
    # Robust horizontal plane mode; textured seams/edges do not define height.
    bins = np.arange(.2,.702,.002); hist,edges = np.histogram(pts[:,2],bins)
    peaks = [i for i in range(1,len(hist)-1) if hist[i]>=hist[i-1] and hist[i]>=hist[i+1] and hist[i]>150]
    if not peaks: raise ValueError('CARTON_TOP_PLANE_NOT_OBSERVED')
    top_bin = max(peaks); top_seed = (edges[top_bin]+edges[top_bin+1])/2
    top_points = pts[abs(pts[:,2]-top_seed)<.003]
    top = float(np.median(top_points[:,2]))
    front_points = pts[(pts[:,2]>top-carton_size[2]+.008)&(pts[:,2]<top-.008)]
    binsx = np.arange(.04,.451,.002); hx,ex = np.histogram(front_points[:,0],binsx)
    front_seed = (ex[np.argmax(hx)]+ex[np.argmax(hx)+1])/2
    fp = front_points[abs(front_points[:,0]-front_seed)<.004]
    if len(fp)<500: raise ValueError('CARTON_FRONT_PLANE_NOT_OBSERVED')
    front = float(np.median(fp[:,0])); alltop = pts[(abs(pts[:,2]-top)<.003)|(abs(pts[:,0]-front)<.003)&(pts[:,2]>top-.09)]
    # Separate columns at observed gaps; no expected column centers are inputs.
    hy,ey = np.histogram(alltop[:,1],np.arange(-.25,.251,.002))
    active = hy>max(8,hy.max()*.08)
    active = cv2.morphologyEx(active.astype(np.uint8)[None,:],cv2.MORPH_CLOSE,np.ones((1,2),np.uint8))[0].astype(bool)
    starts = np.flatnonzero(active&~np.r_[False,active[:-1]]); ends = np.flatnonzero(active&~np.r_[active[1:],False])+1
    intervals = [(float(ey[a]),float(ey[b])) for a,b in zip(starts,ends) if .065<ey[b]-ey[a]<.12]
    if len(intervals)!=3: raise ValueError('THREE_VISIBLE_COLUMNS_REQUIRED:'+str(intervals))
    detections = []
    for i,(lo,hi) in enumerate(intervals):
        selected = alltop[(alltop[:,1]>=lo)&(alltop[:,1]<hi)]
        ylo,yhi = np.quantile(selected[:,1],[.005,.995]); center = [front+carton_size[0]/2,float((ylo+yhi)/2),top-carton_size[2]/2]
        detections.append(dict(column_rank=i,pose_D_m=center,size_m=list(carton_size),
            observed_front_x_m=front,observed_top_z_m=top,observed_y_bounds_m=[float(ylo),float(yhi)],pixels=len(selected)))
    overlay = image.copy(); overlay[mask] = (.55*overlay[mask]+.45*np.array([0,210,0])).astype(np.uint8)
    invT = np.linalg.inv(T)
    for i,d in enumerate(detections):
        p = invT@np.r_[d['pose_D_m'],1.]; pix = K@p[:3]; uv = tuple(np.round(pix[:2]/pix[2]).astype(int))
        cv2.circle(overlay,uv,8,(0,0,255),2); cv2.putText(overlay,'TOP '+str(i)+(' TARGET' if i==1 else ''),uv,cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    cv2.imwrite(str(out/'detections.png'),overlay)
    result = signed(dict(schema='eco65_calibrated_rgbd_detection_v1',status='PASS',source=obs['source'],
        calibration_fingerprint=cal['fingerprint'],observation_fingerprint=obs['fingerprint'],
        image_sha256=row['sha256'],depth_sha256=hashlib.sha256(depth_path.read_bytes()).hexdigest(),
        detections=detections,selected_column_rank=1,selection='HIGHEST_VISIBLE_LAYER_MIDDLE_OF_THREE_COLUMNS',
        assumptions=['KNOWN_150x100x100_MM_CARTON_SIZE','AXIS_ALIGNED_STACK','HIDDEN_FACES_CONSTRAINT_COMPLETED'],
        instance_ground_truth_used=False,nominal_camera_transform_used=False,
        observed_plane_spread_m=dict(top=float(np.std(top_points[:,2])),front=float(np.std(fp[:,0]))),
        wall_s=perf_counter()-started,physical_execution='NOT_RUN',execution_ready=False))
    save_json(out/'detection.json',result); return result
