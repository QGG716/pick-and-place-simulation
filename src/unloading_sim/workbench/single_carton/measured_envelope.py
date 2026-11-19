"""Evidence-bounded collision geometry, separate from the observed contact box.

Constants are calibrated by the deterministic synthetic sweep, not sensor
qualification. Missing silhouette depth never certifies a carton boundary.
"""
from copy import deepcopy
import cv2
import numpy as np
from .geometry import transform, corners

BOUNDARY_PIXEL_FOOTPRINTS = 1.0
DEPTH_NOISE_ALLOWANCE_M = .003


def max_carton_extent(geometry=None):
    g=geometry or {};value=g.get('max_carton_extent_m',.40)
    if type(value) not in (int,float) or not np.isfinite(value) or not 0<value<=.40:
        raise ValueError('MAX_CARTON_EXTENT_INVALID')
    if 'max_carton_extent_m' in g and not str(g.get('max_carton_extent_basis','')).strip():
        raise ValueError('MAX_CARTON_EXTENT_BASIS_REQUIRED')
    return float(value)


def measured_geometry(box):
    """Ownership/display/mass estimate; never use this to reduce collision size."""
    return box.get('measured_box',box)


def payload_mass_properties(box,mass):
    measured=measured_geometry(box);size=np.asarray(measured['size_m'])
    relative=np.linalg.inv(box['pose'])@np.asarray(measured['pose'])
    inertia=mass/12*np.diag([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])
    return dict(mass_kg=mass,com_box_m=relative[:3,3].tolist(),
        inertia_box_kg_m2=(relative[:3,:3]@inertia@relative[:3,:3].T).tolist(),
        inertia_reference='ABOUT_ESTIMATED_COM_IN_COLLISION_ENVELOPE_AXES',
        geometry_source='MEASURED_BOX' if box.get('measured_box') else 'CONSERVATIVE_ENVELOPE_NO_MEASURED_BOX',
        basis='USER_CONFIGURED_MASS_UNIFORM_CUBOID_INERTIA_ESTIMATE_NOT_MEASURED_MASS_DISTRIBUTION')


def conservative_envelope(box,points,mask,world_camera,maximum=.40):
    b=deepcopy(box);p=np.asarray(points,float);mask=np.asarray(mask,bool)
    W=np.asarray(world_camera,float);T=np.asarray(box['pose'],float)
    camera_pose=np.linalg.inv(W)@T;R=camera_pose[:3,:3]
    valid=np.isfinite(p).all(-1)&(p[...,2]>.1)&(p[...,2]<3.)
    use=valid&mask
    local=(p-camera_pose[:3,3])@R;observed=local[use]
    if not len(observed):raise ValueError('NO_MEASURED_ENVELOPE_SUPPORT')
    measured_lo=-np.asarray(box['size_m'])/2;measured_hi=-measured_lo
    if np.max(box['size_m'])>maximum:raise ValueError('MEASURED_BOX_EXCEEDS_CONFIGURED_MAXIMUM_EXTENT')
    low=np.minimum(observed.min(0),measured_lo);high=np.maximum(observed.max(0),measured_hi)
    # Full 3-D footprint on each visible surface, including foreshortening.
    differences=[]
    for axis in (0,1):
        delta=np.diff(local,axis=axis);a=np.take(use,range(use.shape[axis]-1),axis=axis)
        z=np.take(use,range(1,use.shape[axis]),axis=axis)
        near=a&z&(np.linalg.norm(delta,axis=-1)<.02)
        if near.any():differences.append(abs(delta[near]))
    if not differences:raise ValueError('PIXEL_FOOTPRINT_UNRESOLVED')
    footprint=np.maximum(np.quantile(np.concatenate(differences),.95,axis=0),.0001)
    pad=BOUNDARY_PIXEL_FOOTPRINTS*footprint+DEPTH_NOISE_ALLOWANCE_M
    # A direct boundary needs a valid target return immediately adjacent to
    # valid, farther background. A hole between it and the mask edge is unknown.
    boundary=mask&~cv2.erode(mask.astype('uint8'),np.ones((3,3),'uint8')).astype(bool)
    background=np.where(valid&~mask,p[...,2],-np.inf).astype('float32')
    farther=cv2.dilate(background,np.ones((3,3),'uint8'))>p[...,2]+.012
    witnesses=use&boundary&farther
    lo=low.copy();hi=high.copy();intervals=[]
    for axis in range(3):
        for sign in (-1,1):
            edge=low[axis] if sign<0 else high[axis]
            near=abs(local[...,axis]-edge)<=pad[axis]
            direct=bool(np.any(witnesses&near))
            # The dominant observed plane is direct position evidence for +Z,
            # even though its outer contour can be occluded.
            if axis==2 and sign==1:direct=True
            if direct:outer=edge+sign*pad[axis];method='DIRECT_DEPTH_BOUNDARY'
            else:
                outer=(high[axis]-maximum-pad[axis] if sign<0 else low[axis]+maximum+pad[axis])
                method='MISSING_BOUNDARY_MAXIMUM_EXTENT'
            if sign<0:lo[axis]=min(outer,low[axis])
            else:hi[axis]=max(outer,high[axis])
            intervals.append(dict(axis=axis,side=sign,measured_m=float(edge),outer_m=float(outer),
                interval_m=sorted([float(edge),float(outer)]),method=method,
                direct_boundary_pixels=int(np.count_nonzero(witnesses&near)),
                pixel_footprint_m=float(footprint[axis]),noise_allowance_m=DEPTH_NOISE_ALLOWANCE_M))
    measured={k:deepcopy(box[k]) for k in ('pose','size_m','corners_m','faces')}
    b.update(measured_box=measured,measured_size_m=deepcopy(box['size_m']))
    from .partial_geometry import observed_support_plane
    front=camera_pose[:3,3]+R[:,2]*measured_hi[2]
    support=p[use&(abs(local[...,2]-measured_hi[2])<.003)]
    plane=observed_support_plane(p,mask,W,front,R,measured_lo[:2],measured_hi[:2],support) if len(support) else None
    if plane:
        g=T[2,:3];axis=int(np.argmax(abs(g)))
        # Only an axis-aligned measured support can bound an OBB face exactly.
        # Otherwise a clipped polyhedron is needed: refuse instead of cutting
        # off possible payload corners or extending the OBB through support.
        if np.linalg.norm(np.delete(g,axis))>1e-6:
            raise ValueError('MEASURED_SUPPORT_REQUIRES_CLIPPED_ENVELOPE')
        cut=(plane['top_world_m']-T[2,3])/g[axis]
        if g[axis]>0:lo[axis]=cut
        else:hi[axis]=cut
        if np.any(hi<=lo):raise ValueError('MEASURED_SUPPORT_ENVELOPE_CONFLICT')
        b['quality']['observed_support_plane']=plane
        for interval in intervals:
            if interval['axis']==axis and interval['side']==(-1 if g[axis]>0 else 1):
                interval.update(outer_m=float(cut),interval_m=sorted([interval['measured_m'],float(cut)]),
                    method='MEASURED_SUPPORT_PLANE',support=plane)
    center=(lo+hi)/2;size=hi-lo;pose=T.copy();pose[:3,3]+=T[:3,:3]@center
    b.update(pose=pose.tolist(),size_m=size.tolist(),corners_m=corners(pose,size).tolist())
    b['collision_envelope']={k:deepcopy(b[k]) for k in ('pose','size_m','corners_m')}
    b['quality']['hidden_extent_measured']=False
    b['quality']['full_extent_bounded']=True
    b['uncertainty']=dict(kind='MEASURED_BOUNDARY_INTERVALS',bound_method='PIXEL_FOOTPRINT_AND_MAXIMUM_EXTENT',
        frame='MEASURED_BOX',intervals=intervals,lower_m=lo.tolist(),upper_m=hi.tolist(),
        max_carton_extent_m=maximum,pixel_footprints=BOUNDARY_PIXEL_FOOTPRINTS,
        depth_noise_allowance_m=DEPTH_NOISE_ALLOWANCE_M,
        basis='SYNTHETIC_SWEEP_CALIBRATION_NOT_FIELD_DIMENSION_VALIDATION')
    return b
