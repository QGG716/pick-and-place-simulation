"""Depth-supported contact patches without an assumed carton size or thickness.

A patch is sufficient for seal geometry, not for a complete payload envelope.
No image/occlusion boundary or observed extent becomes a measured carton edge.
"""
import cv2
import numpy as np
from .geometry import _visible_planes,transform,rigid


def continuous_depth(points,mask):
    """Keep actual returns with no measured neighbour depth jump.

    Invalid stereo pixels have no depth; zero is not a measured surface at
    the camera origin. Keep holes absent without erasing their valid neighbours.
    Real returns outside the instance still participate in the jump test.
    """
    p=np.asarray(points,float)
    valid=np.isfinite(p).all(-1)&(p[...,2]>.1)&(p[...,2]<3.)
    kernel=np.ones((3,3),'uint8')
    high=cv2.dilate(np.where(valid,p[...,2],-np.inf).astype('float32'),kernel)
    low=cv2.erode(np.where(valid,p[...,2],np.inf).astype('float32'),kernel)
    gap=np.full(valid.shape,np.inf)
    np.subtract(high.astype(float),low.astype(float),out=gap,where=valid)
    return np.asarray(mask,bool)&valid&(gap<.012)


def observed_surface(points,mask,world_camera,*,seed=6518,min_points=120,residual_m=.002):
    p=np.asarray(points,float);mask=np.asarray(mask,bool);W=rigid(world_camera)
    if p.shape!=(*mask.shape,3):raise ValueError('MASK_DEPTH_SHAPE_MISMATCH')
    continuous=continuous_depth(p,mask)
    _,planes=_visible_planes(np.where(continuous[...,None],p,np.nan),mask,seed,min_points,residual_m)
    main=planes[0];normal=main['normal'];support=main['support'];center=main['center']
    _,sv,vh=np.linalg.svd(support-center,full_matrices=False)
    if main['rms']>residual_m or sv[1]/np.sqrt(len(support))<.004:
        raise ValueError('LOCAL_PLANE_NORMAL_UNRELIABLE')
    u=vh[0]-normal*(vh[0]@normal);u/=np.linalg.norm(u);v=np.cross(normal,u)
    xy=np.c_[(support-center)@u,(support-center)@v]
    rect=cv2.boxPoints(cv2.minAreaRect(xy.astype('float32'))).astype(float);edge=rect[1]-rect[0]
    x=(u*edge[0]+v*edge[1])/np.linalg.norm(edge);y=np.cross(normal,x)
    uv=np.c_[(support-center)@x,(support-center)@y];lo,hi=uv.min(0),uv.max(0)
    if min(hi-lo)<.01:raise ValueError('OBSERVED_CONTACT_PATCH_TOO_SMALL')
    shift=(lo+hi)/2;face_center=center+x*shift[0]+y*shift[1];uv-=shift
    _,idx=np.unique(np.floor(uv/.0015).astype(int),axis=0,return_index=True)
    face=dict(center_m=transform(W,[face_center])[0].tolist(),normal=(W[:3,:3]@normal).tolist(),
        u=(W[:3,:3]@x).tolist(),v=(W[:3,:3]@y).tolist(),size_m=(hi-lo).tolist(),
        observed=True,face='observed_local_plane',support_points_uv_m=uv[idx].tolist(),
        support_radius_m=.0025,check_observed_margin=True)
    return dict(observed_face=face,observed_patch_size_m=(hi-lo).tolist(),
        plane_rms_m=main['rms'],plane_inliers=main['count'],valid_depth_points=int(continuous.sum()),
        observed_planes=[dict(center_m=transform(W,[q['center']])[0].tolist(),
            normal=(W[:3,:3]@q['normal']).tolist(),rms_m=q['rms'],support_count=q['count']) for q in planes],
        geometry_state='OBSERVED_SURFACE_ONLY',full_extent_bounded=False,hidden_extent_measured=False,
        prior_applied=False,full_dimensions_m=None,
        observed_extent_is_carton_size=False,image_or_occlusion_boundary_is_box_edge=False)
