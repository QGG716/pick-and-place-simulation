"""Observed contact patches and a bounded union of prior-constrained cuboids.

Image/occlusion boundaries impose containment only, never a measured box edge.
Yaw cells cover continuous rotations; padding covers the intervals between cell
centres. Collision geometry is the union's enclosing box, not the visible sheet.
"""
from itertools import permutations, product
import cv2
import numpy as np
from .geometry import _visible_planes, corners, rigid, transform, GeometryRejected


def credible_edges(points,mask,plane,R,origin,seed):
    """Only a visible plane-to-farther-background break can constrain an edge.

    Occluders are closer than the extrapolated plane; image borders have no
    outside depth. Neither is treated as a physical carton edge.
    """
    n=plane['normal'];denom=points@n;offset=plane['center']@n
    ratio=np.divide(offset,denom,out=np.zeros_like(denom),where=abs(denom)>1e-9)
    predicted=points[...,2]*ratio
    outside=(~mask)&np.isfinite(points).all(-1)&(points[...,2]>.1)&(predicted>.1)
    farther=outside & (points[...,2]-predicted>.015)
    near=cv2.dilate(farther.astype('uint8'),np.ones((5,5),'uint8')).astype(bool)
    boundary=mask & near & np.isfinite(points).all(-1) & (abs(denom-offset)<.002)
    xy=(points[boundary]-origin)@R[:,:2]
    rng=np.random.default_rng(seed);edges=[]
    for _ in range(4):
        if len(xy)<25:break
        best=None;direction=None
        for _ in range(80):
            a,b=xy[rng.choice(len(xy),2,replace=False)];d=b-a;length=np.linalg.norm(d)
            if length<.025:continue
            d/=length;normal=np.array([-d[1],d[0]])
            inliers=abs((xy-a)@normal)<.002
            if best is None or inliers.sum()>best.sum():best=inliers;direction=d
        if best is None or best.sum()<25:break
        sample=xy[best];span=np.ptp(sample@direction)
        if span<.025:break
        edges.append(dict(center_uv_m=sample.mean(0).tolist(),direction_uv=direction.tolist(),
                          span_m=float(span),support_count=int(best.sum()),
                          basis='OBSERVED_PLANE_TO_FARTHER_DEPTH_BREAK'))
        xy=xy[~best]
    return edges


def recover_partial(points, mask, world_camera, *, prior, dimension_tolerance_m=.008,
                    seed=6518, min_points=120, residual_m=.002, **_):
    p=np.asarray(points,float);mask=np.asarray(mask,bool);W=rigid(world_camera)
    size_prior=np.asarray(prior,float)
    if size_prior.shape!=(3,) or not np.isfinite(size_prior).all() or np.any(size_prior<=.005) or np.any(size_prior>2.):
        raise ValueError('INVALID_CONFIGURABLE_CARTON_PRIOR')
    if p.shape!=(*mask.shape,3):raise ValueError('MASK_DEPTH_SHAPE_MISMATCH')
    valid=mask & np.isfinite(p).all(-1) & (p[...,2]>.1) & (p[...,2]<3.)
    # Keep holes and depth jumps out of the seal support; do not interpolate.
    z=np.where(valid,p[...,2],0).astype('float32')
    lo=cv2.erode(z,np.ones((3,3),'uint8'));hi=cv2.dilate(z,np.ones((3,3),'uint8'))
    continuous=valid & (hi-lo<.012)
    cleaned=np.where(continuous[...,None],p,np.nan)
    _,planes=_visible_planes(cleaned,mask,seed,min_points,residual_m)
    main=planes[0];n=main['normal'];center=main['center'];support=main['support']
    if main['rms']>residual_m:raise ValueError('LOCAL_PLANE_NORMAL_UNRELIABLE')
    _,sv,vh=np.linalg.svd(support-center,full_matrices=False)
    if sv[1]/np.sqrt(len(support))<.004:raise ValueError('LOCAL_PLANE_NORMAL_UNRELIABLE')
    ref=vh[0]-n*(vh[0]@n);ref/=np.linalg.norm(ref);other=np.cross(n,ref)
    xy=np.c_[(support-center)@ref,(support-center)@other]
    # OpenCV returns float32 corners. Normalize their direction in float64;
    # float32 lengths inject scale into an otherwise rigid measured frame and
    # violate native SE(3) validation. Never repair calibration/input transforms.
    rect=cv2.boxPoints(cv2.minAreaRect(xy.astype('float32'))).astype(float);edge=rect[1]-rect[0]
    x=(ref*edge[0]+other*edge[1])/np.linalg.norm(edge);y=np.cross(n,x)
    R=np.column_stack((x,y,n));local=(support-center)@R
    low,high=local[:,:2].min(0),local[:,:2].max(0)
    face_local=(low+high)/2;face_center=center+R[:,:2]@face_local
    patch_size=high-low
    if min(patch_size)<.01:raise ValueError('OBSERVED_CONTACT_PATCH_TOO_SMALL')
    uv=(support-face_center)@R[:,:2]
    edges=credible_edges(p,mask,main,R,face_center,seed)
    _,idx=np.unique(np.floor(uv/.0015).astype(int),axis=0,return_index=True)
    face=dict(center_m=transform(W,[face_center])[0].tolist(),normal=(W[:3,:3]@n).tolist(),
              u=(W[:3,:3]@x).tolist(),v=(W[:3,:3]@y).tolist(),size_m=patch_size.tolist(),
              observed=True,face='observed_local_plane',support_points_uv_m=uv[idx].tolist(),
              support_radius_m=.0025,check_observed_margin=True)
    # Side planes constrain yaw, but silhouette fragments do not. Within each
    # 5-degree cell expand projected extents by the maximum angular displacement.
    side=next((s for s in planes[1:] if abs(s['normal']@n)<np.sin(np.deg2rad(10))),None)
    steps=36;half_step=np.pi/(2*steps);hypotheses=[];vertices=[]
    dimensions=sorted(set(permutations(map(float,size_prior))))
    all_local=(p[continuous]-face_center)@R
    for angle in np.arange(steps)*np.pi/steps:
        rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        if side is not None:
            side_xy=side['normal']@R[:,:2]
            if np.max(abs(side_xy@rot))<np.cos(np.deg2rad(10)+half_step):continue
        projected=uv@rot
        for dims in dimensions:
            dims=np.asarray(dims);angular=np.linalg.norm(dims[:2])*np.sin(half_step)
            tolerance=dimension_tolerance_m+angular
            minimum=projected.max(0)-dims[:2]/2-tolerance
            maximum=projected.min(0)+dims[:2]/2+tolerance
            for edge in edges:
                direction=np.asarray(edge['direction_uv'])@rot
                tangent=int(np.argmax(abs(direction)));axis=1-tangent
                angular_error=np.arctan2(.004,edge['span_m'])+half_step
                if abs(direction[tangent])<np.cos(angular_error):
                    minimum[:]=1.;maximum[:]=-1.;break
                edge_at=(np.asarray(edge['center_uv_m'])@rot)[axis]
                inward=1 if projected[:,axis].mean()>edge_at else -1
                center_at=edge_at+inward*dims[axis]/2
                edge_error=dimension_tolerance_m+angular+.003
                minimum[axis]=max(minimum[axis],center_at-edge_error)
                maximum[axis]=min(maximum[axis],center_at+edge_error)
            if np.any(minimum>maximum):continue
            # All valid observed depths must fit behind the plane and in the
            # prior depth; side evidence can exclude a thin-face interpretation.
            if np.mean((all_local[:,2]>=-dims[2]-dimension_tolerance_m)&(all_local[:,2]<=3*residual_m))<.95:continue
            hypotheses.append(dict(yaw_center_rad=float(angle),yaw_half_width_rad=half_step,
                dimensions_m=dims.tolist(),center_interval_uv_m=[minimum.tolist(),maximum.tolist()]))
            for c in product(*zip(minimum,maximum)):
                for sign in product((-1,1),repeat=2):
                    point=(np.asarray(c)+np.asarray(sign)*dims[:2]/2)@rot.T
                    # Cover the yaw cell and dimension tolerance in every direction.
                    for delta in product((-tolerance,tolerance),repeat=2):
                        vertices.append([*(point+delta),-dims[2]-dimension_tolerance_m])
    if not hypotheses:
        raise GeometryRejected('DIMENSION_PRIOR_CONFLICT_WITH_OBSERVED_SURFACE',
            dict(prior_m=size_prior.tolist(),observed_patch_m=patch_size.tolist()))
    vertices=np.asarray(vertices);lower=vertices.min(0);upper=vertices.max(0);upper[2]=0.
    envelope_size=upper-lower;pose=np.eye(4);pose[:3,:3]=R
    pose[:3,3]=face_center+R@((lower+upper)/2);pose=W@pose
    reference=np.eye(4);reference[:3,:3]=W[:3,:3]@R;reference[:3,3]=transform(W,[face_center])[0]
    return dict(pose=pose.tolist(),size_m=envelope_size.tolist(),corners_m=corners(pose,envelope_size).tolist(),
        faces=[face],observed_faces=['local_depth_continuous_plane'],
        inferred_faces=['prior_constrained_unobserved_cuboid_surfaces'],
        dimension_source='CONFIGURED_PRIOR_WITH_CONTINUOUS_CONSERVATIVE_ENVELOPE',
        nominal_size_m=size_prior.tolist(),
        uncertainty=dict(kind='BOUNDED_CUBOID_UNION',hypotheses=hypotheses,
            reference_pose=reference.tolist(),dimension_tolerance_m=dimension_tolerance_m,
            collision_envelope='ALL_FEASIBLE_CENTERS_DIMENSIONS_AND_YAW_CELLS',
            image_or_occlusion_boundary_is_box_edge=False,unique_pose=False),
        quality=dict(plane_rms_m=main['rms'],plane_inliers=main['count'],valid_depth_points=int(continuous.sum()),
            hidden_extent_measured=False,unseen_surfaces_observed=False,
            observed_patch_size_m=patch_size.tolist(),side_normal_observed=side is not None,
            credible_visible_edges=edges,
            ambiguity='PRIOR_SIZE_PERMUTATION_YAW_AND_CENTER_INTERVALS'))


def possible_bottom_interval(box,returned_pose):
    """Bounds on each admissible cuboid's bottom at the proposed release pose."""
    uncertainty=box['uncertainty'];ref=rigid(returned_pose)@np.linalg.inv(box['pose'])@rigid(uncertainty['reference_pose'])
    bottom=[]
    for h in uncertainty['hypotheses']:
        dims=np.asarray(h['dimensions_m']);a=h['yaw_center_rad'];rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
        for center in product(*zip(*h['center_interval_uv_m'])):
            points=[]
            for sign in product((-1,1),repeat=3):
                xy=(np.asarray(center)+np.asarray(sign[:2])*dims[:2]/2)@rot.T
                points.append([*xy,(-dims[2] if sign[2]<0 else 0.)])
            z=transform(ref,points)[:,2].min()
            pad=(np.linalg.norm(dims[:2])*np.sin(h['yaw_half_width_rad'])*np.linalg.norm(ref[2,:2])+
                 uncertainty['dimension_tolerance_m']*np.abs(ref[2,:3]).sum())
            bottom.extend([z-pad,z+pad])
    return min(bottom),max(bottom)
