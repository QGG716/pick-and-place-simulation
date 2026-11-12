"""Observed contact patches and a bounded union of prior-constrained cuboids.

Image/occlusion boundaries impose containment only, never a measured box edge.
Yaw cells cover continuous rotations; padding covers the intervals between cell
centres. Collision geometry is the union's enclosing box, not the visible sheet.
"""
from itertools import permutations, product
import cv2
import numpy as np
from .geometry import _visible_planes, corners, rigid, transform, GeometryRejected

# Plane-support boundary noise allowed beyond the configured carton tolerance.
PATCH_EDGE_NOISE_M=.003


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


def observed_support_plane(points,mask,W,face_center,R,patch_low,patch_high,support):
    """Horizontal surface the visible face stands on, from real depth only.

    Requires a face in-plane axis within 5 degrees of world vertical, at least
    50 non-target depth points at the patch's lowest edge (in front of, beside
    or behind the face), a flat height distribution and a visible patch that
    actually reaches it. Returns None otherwise; nothing is inferred.
    """
    g=(W[:3,:3]@R)[2];k=int(np.argmax(abs(g[:2])))
    if abs(g[k])<np.cos(np.deg2rad(5)):return None
    lateral=1-k
    patch_z=transform(W,support)[:,2];bottom=float(np.percentile(patch_z,1))
    finite=np.isfinite(points).all(-1)&(points[...,2]>.1)&(points[...,2]<3.)&~mask
    local=(points-face_center)@R;world_z=(points@W[:3,:3].T+W[:3,3])[...,2]
    near=(finite&(local[...,lateral]>=patch_low[lateral]-.03)&(local[...,lateral]<=patch_high[lateral]+.03)
          &(local[...,2]>=-.3)&(local[...,2]<=.1)&(world_z>=bottom-.02)&(world_z<=bottom+.01))
    if near.sum()<50:return None
    # Dominant height mode; a horizontal top extends along the face normal
    # (depth), while a vertical front face below the target does not.
    z=world_z[near];depth=local[...,2][near]
    edges=np.arange(bottom-.02,bottom+.0101,.002);hist=np.histogram(z,edges)[0]
    peak=edges[int(np.argmax(hist))]+.001;flat=abs(z-peak)<=.003
    if flat.sum()<50 or np.subtract(*np.percentile(depth[flat],[95,5]))<.01:return None
    height=float(np.median(z[flat]));noise=float(1.4826*np.median(abs(z[flat]-height)))
    if noise>.003 or abs(bottom-height)>.008:return None
    return dict(top_world_m=height,noise_m=noise,points=int(flat.sum()),patch_bottom_world_m=bottom,
                vertical_face_axis=k,basis='OBSERVED_HORIZONTAL_RGBD_SURFACE_AT_VISIBLE_FACE_BOTTOM_EDGE')


def resting_check(plane,edges,W,face_center,R,prior,tolerance):
    """Bind the support only if the box measurably stands on it.

    A credible top edge (plane to farther background) and the support must be
    separated by one prior dimension within tolerance. A carton sunk behind a
    closer lip deeper than that tolerance shows a shorter visible height and is
    rejected, leaving the full envelope (and its collision) unchanged.
    """
    k=plane['vertical_face_axis'];g=(W[:3,:3]@R)[2];c0=float((W[:3,:3]@face_center+W[:3,3])[2])
    heights=[]
    for edge in edges:
        direction=np.asarray(edge['direction_uv'])
        if abs(direction[k])>np.sin(np.deg2rad(5)):continue
        at=np.zeros(3);at[:2]=edge['center_uv_m'];z=c0+g@at
        if z>plane['top_world_m']:heights.append(z-plane['top_world_m'])
    if not heights:return dict(applied=False,reason='NO_CREDIBLE_TOP_EDGE_FOR_RESTING_HEIGHT')
    height=max(heights);allowed=tolerance+PATCH_EDGE_NOISE_M+3*plane['noise_m']
    error=float(np.min(abs(np.asarray(prior)-height)))
    if error>allowed:
        return dict(applied=False,resting_height_m=height,prior_height_error_m=error,
                    reason='VISIBLE_HEIGHT_DISAGREES_WITH_PRIOR_POSSIBLY_SUNK_OR_OCCLUDED')
    return dict(applied=True,resting_height_m=height,prior_height_error_m=error,allowed_error_m=allowed,
                assumption='CARTON_RESTS_ON_OBSERVED_SUPPORT_HEIGHT_CONSISTENT_WITHIN_TOLERANCE')


def bound_by_support(lower,upper,W,face_center,R,low,high,plane):
    """Remove only envelope volume that lies entirely below the support top.

    For every other coordinate in its envelope range, points beyond the cut are
    below the plane, so every admissible cuboid resting on it stays inside.
    """
    g=(W[:3,:3]@R)[2];c0=float((W[:3,:3]@face_center+W[:3,3])[2]);k=plane['vertical_face_axis']
    others=sum(max(g[j]*lower[j],g[j]*upper[j]) for j in range(3) if j!=k)
    cut=(plane['top_world_m']-c0-others)/g[k]
    lower=lower.copy();upper=upper.copy()
    if g[k]>0:lower[k]=max(lower[k],min(cut,low[k]))
    else:upper[k]=min(upper[k],max(cut,high[k]))
    return lower,upper


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
            dims=np.asarray(dims)
            # In this yaw cell's frame, every admissible cuboid (each size within
            # +/- one tolerance, rotation within the cell) has an axis-aligned
            # bound of half-size <= H. The envelope is exactly the union of such
            # bounds containing the patch: centre in [max-H, min+H], extent
            # [max-2H, min+2H]. Tolerance and yaw growth are counted once.
            growth=(dims[1::-1]+dimension_tolerance_m)*np.sin(half_step)
            H=(dims[:2]+dimension_tolerance_m+growth+PATCH_EDGE_NOISE_M)/2
            # A visible patch cannot exceed the face it belongs to (174 mm is not
            # a 150 mm face even with an 8 mm tolerance).
            if np.any(np.ptp(projected,axis=0)>2*H):continue
            minimum=projected.max(0)-H
            maximum=projected.min(0)+H
            for edge in edges:
                direction=np.asarray(edge['direction_uv'])@rot
                tangent=int(np.argmax(abs(direction)));axis=1-tangent
                angular_error=np.arctan2(.004,edge['span_m'])+half_step
                if abs(direction[tangent])<np.cos(angular_error):
                    minimum[:]=1.;maximum[:]=-1.;break
                edge_at=(np.asarray(edge['center_uv_m'])@rot)[axis]
                inward=1 if projected[:,axis].mean()>edge_at else -1
                center_at=edge_at+inward*dims[axis]/2
                edge_error=dimension_tolerance_m+growth[axis]+.003
                minimum[axis]=max(minimum[axis],center_at-edge_error)
                maximum[axis]=min(maximum[axis],center_at+edge_error)
            if np.any(minimum>maximum):continue
            # All valid observed depths must fit behind the plane and in the
            # prior depth; side evidence can exclude a thin-face interpretation.
            if np.mean((all_local[:,2]>=-dims[2]-dimension_tolerance_m)&(all_local[:,2]<=3*residual_m))<.95:continue
            hypotheses.append(dict(yaw_center_rad=float(angle),yaw_half_width_rad=half_step,
                dimensions_m=dims.tolist(),center_interval_uv_m=[minimum.tolist(),maximum.tolist()],
                bound_half_size_m=H.tolist()))
            for c in product(*zip(minimum,maximum)):
                for sign in product((-1,1),repeat=2):
                    point=(np.asarray(c)+np.asarray(sign)*H)@rot.T
                    vertices.append([*point,-dims[2]-dimension_tolerance_m])
    if not hypotheses:
        raise GeometryRejected('DIMENSION_PRIOR_CONFLICT_WITH_OBSERVED_SURFACE',
            dict(prior_m=size_prior.tolist(),observed_patch_m=patch_size.tolist()))
    vertices=np.asarray(vertices);lower=vertices.min(0);upper=vertices.max(0);upper[2]=0.
    plane=observed_support_plane(p,mask,W,face_center,R,low,high,support)
    if plane is not None:
        plane.update(resting_check(plane,edges,W,face_center,R,size_prior,dimension_tolerance_m))
    if plane is not None and plane['applied']:
        before=(lower.copy(),upper.copy())
        lower,upper=bound_by_support(lower,upper,W,face_center,R,low,high,plane)
        plane['envelope_removed_below_support_m']=float(np.max(np.r_[lower-before[0],before[1]-upper]))
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
            credible_visible_edges=edges,observed_support_plane=plane,
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
