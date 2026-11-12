"""Task-local RGB-D collision scene in confirmed base coordinates.

Unknown voxels are occupied. A measured clear volume can supply blind workspace
coverage, but never erase observed obstacles. No simulation world is imported.
"""
from itertools import product
import json
from pathlib import Path
import cv2
import numpy as np
from ..coordinates import rigid
from ..detection import metric_points
from .geometry import transform


def load_site(path):
    c=json.loads(Path(path).read_text())
    if c.get('schema')!='eco65_real_site_v1' or c.get('frame')!='base_link':
        raise ValueError('REAL_SITE_BASE_LINK_CONFIG_REQUIRED')
    lo,hi=np.asarray(c['workspace_bounds_m'],float)
    cell=float(c['voxel_m'])
    if (lo.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo)
            or not .005<=cell<=.08 or np.prod(np.ceil((hi-lo)/cell))>300000):
        raise ValueError('INVALID_BOUNDED_SITE_WORKSPACE')
    if c.get('unknown_policy')!='OCCUPIED':raise ValueError('UNKNOWN_SPACE_MUST_REMAIN_OCCUPIED')
    ids=set()
    for b in c.get('fixed_obstacles',[]):
        if b['id'] in ids or not b.get('measurement_basis'):raise ValueError('SITE_OBSTACLE_ID_OR_MEASUREMENT_REQUIRED')
        ids.add(b['id']);rigid(b['pose'])
        if np.asarray(b['size_m']).shape!=(3,) or not np.isfinite(b['size_m']).all() or min(b['size_m'])<=0:
            raise ValueError('INVALID_SITE_BOX_SIZE')
    for r in c.get('measured_clear_regions',[]):
        a,b=np.asarray(r['bounds_m'],float)
        if not r.get('measurement_basis') or not np.isfinite([a,b]).all() or np.any(a>=b):
            raise ValueError('BLIND_CLEAR_VOLUME_REQUIRES_FIELD_MEASUREMENT')
    receiver=c.get('receiver')
    if receiver:
        if not receiver.get('support_ids') or not set(receiver['support_ids'])<=ids or not receiver.get('measurement_basis'):
            raise ValueError('RECEIVER_MUST_BIND_MEASURED_SITE_SUPPORT')
        a,b=np.asarray(receiver['bounds_m'],float)
        if not np.isfinite([a,b]).all() or np.any(a>=b) or not 0<=receiver.get('margin_m',.003)<.1:
            raise ValueError('INVALID_RECEIVER_BOUNDS')
    c['config_path']=str(Path(path).resolve())
    return c


def _box(ident,lo,hi,category):
    T=np.eye(4);T[:3,3]=(lo+hi)/2
    return dict(id=ident,pose=T.tolist(),size=(hi-lo).tolist(),category=category)


def _project(points,k):
    from ..reference.camera_geometry import ProjectionInfo,opencv_brown_coefficients
    info=ProjectionInfo('realsense',k['width'],k['height'],k['fx'],k['fy'],k['ppx'],k['ppy'],
                        distortion=tuple(k['coeffs']),distortion_model=k['model'])
    coeff=(0.,)*5 if k['model']=='SYNTHETIC_PINHOLE' else opencv_brown_coefficients(info)
    K=np.array([[k['fx'],0,k['ppx']],[0,k['fy'],k['ppy']],[0,0,1.]])
    uv,_=cv2.projectPoints(points,np.zeros(3),np.zeros(3),K,np.asarray(coeff))
    return uv.reshape(-1,2)


def build_scene(observation,result,boxes,config,self_filter=None):
    frame=observation.frame
    if result.observation_id!=frame.observation_id:raise ValueError('SCENE_FRAME_MISMATCH')
    lo,hi=np.asarray(config['workspace_bounds_m'],float);cell=config['voxel_m']
    shape=np.ceil((hi-lo)/cell).astype(int);axes=[lo[i]+(np.arange(shape[i])+.5)*cell for i in range(3)]
    centers=np.stack(np.meshgrid(*axes,indexing='ij'),-1).reshape(-1,3)
    W=rigid(observation.world_camera);C=np.linalg.inv(W)
    offsets=np.asarray(list(product((-.5,.5),repeat=3)))*cell
    vertices=centers[:,None,:]+offsets
    camera=transform(C,vertices.reshape(-1,3)).reshape(-1,8,3)
    uv=_project(camera.reshape(-1,3),frame.intrinsics).reshape(-1,8,2)
    # Full projected voxel footprint must have valid, continuous farther depth.
    # Check every pixel in its bounding rectangle via integral min queries;
    # using only 8 corners would miss an intervening obstacle or depth hole.
    h,w=frame.depth.shape;depth=frame.depth.astype(float)*frame.depth_scale_m
    bounds=np.c_[np.floor(uv.min(1)),np.ceil(uv.max(1))].astype(int)
    free=np.zeros(len(centers),bool)
    in_view=(camera[:,:,2].min(1)>.1)&(bounds[:,0]>=0)&(bounds[:,1]>=0)&(bounds[:,2]<w)&(bounds[:,3]<h)
    for i in np.flatnonzero(in_view):
        x0,y0,x1,y1=bounds[i];patch=depth[y0:y1+1,x0:x1+1]
        free[i]=bool(patch.size and patch.min()>camera[i,:,2].max()+.006)
    for region in config.get('measured_clear_regions',[]):
        a,b=np.asarray(region['bounds_m'])
        free |= np.all(centers-cell/2>=a,axis=1)&np.all(centers+cell/2<=b,axis=1)
    # Every valid depth outside reconstructed target masks remains an obstacle,
    # including unreconstructed detections and unlabelled tables/floors/objects.
    points=metric_points(frame);valid=np.isfinite(points).all(-1)&(depth>.1)&(depth<3.)
    claimed=np.zeros(valid.shape,bool)
    world_points=transform(W,points.reshape(-1,3)).reshape(points.shape)
    for box in boxes:
        local=transform(np.linalg.inv(box['pose']),world_points.reshape(-1,3)).reshape(points.shape)
        # A failed/unexplained depth outside the reconstructed full envelope
        # remains an obstacle even if SAM labelled it as the target.
        contained=np.all(abs(local)<=np.asarray(box['size_m'])/2+.002,axis=-1)
        claimed |= result.masks[box['detection_index']] & contained
    observed=transform(W,points[valid & ~claimed])
    removed_self=0
    if self_filter is not None:
        owned=self_filter.contains(observed,observation.q_capture)
        removed_self=int(owned.sum());observed=observed[~owned]
        # A voxel is owned only when ALL corners are inside actual robot solids.
        # Partial/uncertain cells remain occupied unless field evidence covers them.
        owned_cells=self_filter.contains(vertices.reshape(-1,3),observation.q_capture,group_size=8)
        free |= owned_cells
    for fixed in config.get('fixed_obstacles',[]):
        local=transform(np.linalg.inv(fixed['pose']),observed)
        observed=observed[~np.all(abs(local)<=np.asarray(fixed['size_m'])/2+1e-9,axis=1)]
    indices=np.floor((observed-lo)/cell).astype(int)
    inside=np.all(indices>=0,axis=1)&np.all(indices<shape,axis=1)
    indices=indices[inside]
    occupancy=(~free).reshape(tuple(shape));measured=np.zeros_like(occupancy)
    if len(indices):measured[tuple(indices.T)]=True
    occupancy |= measured
    # Unknown target interiors are not declared free. Only explicit measured
    # clear workspace or observed free rays can remove that unknown occupancy.
    world=[dict(id=b['id'],pose=b['pose'],size=b['size_m'],category='measured_fixed') for b in config.get('fixed_obstacles',[])]
    for ix in range(shape[0]):
        for iy in range(shape[1]):
            # Merge consecutive occupied cells along Z with identical ownership.
            for kind,grid in [('observed',measured),('unknown',occupancy & ~measured)]:
                row=grid[ix,iy];edges=np.diff(np.r_[False,row,False].astype(int))
                for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
                    lower=lo+np.array([ix,iy,a])*cell;upper=lo+np.array([ix+1,iy+1,b])*cell
                    world.append(_box(f'rgbd_{kind}_{ix}_{iy}_{a}',lower,upper,kind))
    # Finite boundary walls keep paths inside the region actually classified.
    for axis in range(3):
        for sign in (0,1):
            a=lo.copy();b=hi.copy();a[axis]=(lo if sign==0 else hi)[axis]-.04*(sign==0);b[axis]=a[axis]+.04
            world.append(_box(f'site_boundary_{axis}_{sign}',a,b,'workspace_boundary'))
    return dict(schema='eco65_real_rgbd_scene_v1',frame='base_link',world=world,
        observation_id=frame.observation_id,q_capture=list(observation.q_capture),T_base_camera=W.tolist(),
        unknown_policy='OCCUPIED',unknown_voxels=int((occupancy & ~measured).sum()),
        observed_voxels=int(measured.sum()),measured_clear_regions=config.get('measured_clear_regions',[]),
        self_returns_removed=removed_self,self_filter_pose='q_capture',
        imported_simulation_objects=0,hardware_execution='NOT_RUN')
