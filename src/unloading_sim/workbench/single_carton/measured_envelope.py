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


def single_face_policy(geometry=None):
    """User decision 2026-11-23: a carton whose only accepted plane is its
    front face is not rejected; its unobserved thickness is bounded by the
    configured maximum carton extent (free-space witnesses only tighten it)."""
    value=(geometry or {}).get('single_face_depth_policy')
    if value is None or (isinstance(value,dict) and value.get('mode')=='DISABLED'):return None
    if (not isinstance(value,dict) or value.get('mode')!='BOUND_BY_MAX_CARTON_EXTENT' or
        not isinstance(value.get('basis'),str) or not value['basis'].strip()):
        raise ValueError('SINGLE_FACE_DEPTH_POLICY_INVALID')
    return dict(value)


def continuation_policy(geometry=None):
    """User decision 2026-11-23: a front face continuing coplanar past its
    mask (too little RGB edge) is not rejected; those edges stay missing."""
    value=(geometry or {}).get('coplanar_continuation_policy')
    if value is None or (isinstance(value,dict) and value.get('mode')=='DISABLED'):return None
    if (not isinstance(value,dict) or value.get('mode')!='EXTEND_ENVELOPE_TO_NEXT_BOUNDARY' or
        not isinstance(value.get('basis'),str) or not value['basis'].strip()):
        raise ValueError('COPLANAR_CONTINUATION_POLICY_INVALID')
    return dict(value)


# Same 3x3 depth-jump test as site_scene.FLYING_DEPTH_JUMP_M.
OWN_SURFACE_DEPTH_JUMP_M = .012
OWN_SURFACE_BASIS = ('USER_CONFIRMATION_2026_11_20_OWN_MASK_RETURNS_DEPTH_CONNECTED_TO_THE_MEASURED_FACES_'
                     'ARE_THE_CARTON_ITSELF')


def depth_jump(z,valid):
    zz=np.where(valid,z,0.).astype(np.float32);square=np.ones((3,3),np.uint8)
    return (cv2.dilate(zz,square)-cv2.erode(zz,square))>OWN_SURFACE_DEPTH_JUMP_M


def own_connected_surface(region,seed):
    """Pixels of ``region`` (own mask, valid, no depth jump) 4-connected to ``seed``.

    A mask that bleeds onto the background or an occluder is separated from the
    carton by a depth jump; returns depth-continuous with the measured faces
    (a bulging or skewed side, the user confirmed 2026-11-20) are the carton.
    """
    region=np.asarray(region,bool)
    _,labels=cv2.connectedComponents(region.astype(np.uint8),connectivity=4)
    ids=np.unique(labels[region&np.asarray(seed,bool)]);ids=ids[ids!=0]
    return region&np.isin(labels,ids)


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


# Height uncertainty of a fitted face plane for free-ray witnesses: the 2 mm
# plane residual plus the 3 mm depth-noise allowance (both existing values).
# Checked 2026-11-24 against the user's ruler measurement (carton 174 x 133 x
# 110 mm): without it 4 of 6 field frames left the true back 5-23 mm outside
# the envelope; with 5 mm every frame contains it (closest margin 14 mm).
FREE_PLANE_INWARD_M=.005


def plane_free_witnesses(points,camera_pose,face_axis,face_position,measured_lo,measured_hi,pad):
    """Bound a measured cuboid face's in-plane extents by sensor-free rays.

    ``face_axis``/``face_position`` name a visible face of the carton in the
    measured box frame (front, side or top). A cuboid containing the observed
    rectangle spans each observed coordinate interval on that face. If a ray
    crosses the face plane at a point inside the span of one in-plane axis and
    the sensor returned clearly farther depth, the carton is not at that point
    (a solid there would have returned it), so it cannot extend that far along
    the other in-plane axis. Foreground, absent and off-image returns give no
    witness. The existing RGB-D free-space margin and pixel guard apply.
    """
    from .visibility_geometry import FREE_MARGIN_M,PIXEL_GUARD
    p=np.asarray(points,float);R=camera_pose[:3,:3];normal=R[:,face_axis]
    # The fitted face plane is not the carton's surface everywhere: a top or
    # side seen at a grazing angle (camera ~6 cm above a carton top) turns a
    # few millimetres of sag, bulge or tape into tens of millimetres along the
    # face (field 90c7aaed: back bound 23 mm short of the true 133 mm depth).
    # A ray is a free witness only where it is free at the plane moved inward
    # by FREE_PLANE_INWARD_M, which lengthens grazing bounds; never shortens.
    inward=FREE_PLANE_INWARD_M if face_position>0 else -FREE_PLANE_INWARD_M
    face_position=face_position-inward
    on_face=camera_pose[:3,3]+normal*face_position
    valid=np.isfinite(p).all(-1)&(p[...,2]>.1)&(p[...,2]<3.)
    with np.errstate(divide='ignore',invalid='ignore'):
        denominator=(p@normal)/p[...,2]
        z=(on_face@normal)/denominator
        query=p*(z/p[...,2])[...,None]
        free=valid&(z>.1)&(abs(denominator)>1e-8)&(
            p[...,2]>z+FREE_MARGIN_M/abs(denominator)+FREE_MARGIN_M)
    free=cv2.erode(free.astype('uint8'),np.ones((2*PIXEL_GUARD+1,)*2,'uint8'),
        borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
    query=np.where(np.isfinite(query).all(-1)[...,None],query,0.)
    local=(query-camera_pose[:3,3])@R;witnesses={}
    in_plane=[a for a in range(3) if a!=face_axis]
    for axis in in_plane:
        other=in_plane[0] if axis==in_plane[1] else in_plane[1]
        strip=(local[...,other]>measured_lo[other]+pad[other])&(
            local[...,other]<measured_hi[other]-pad[other])
        for sign in (-1,1):
            edge=measured_lo[axis] if sign<0 else measured_hi[axis]
            use=free&strip&((local[...,axis]<edge-pad[axis]) if sign<0 else
                           (local[...,axis]>edge+pad[axis]))
            if not use.any():continue
            values=local[use,axis];k=int(np.argmax(values) if sign<0 else np.argmin(values))
            y,x=map(int,np.argwhere(use)[k]);outer=float(values[k]+sign*pad[axis])
            witnesses[(axis,sign)]=dict(outer_m=outer,pixel_xy=[x,y],
                observed_depth_m=float(p[y,x,2]),plane_depth_m=float(z[y,x]),
                query_face_local_m=local[y,x].tolist(),face_axis=int(face_axis),
                face_position_m=float(face_position),witness_pixels=int(use.sum()),
                interior_other_axis_m=[float(measured_lo[other]+pad[other]),float(measured_hi[other]-pad[other])],
                free_margin_m=FREE_MARGIN_M,pixel_guard=PIXEL_GUARD,
                method='SENSOR_FREE_POINT_IN_OBSERVED_RECTANGLE_SPAN',
                mask_boundary_used_as_carton_edge=False)
    return witnesses


def front_free_witnesses(points,camera_pose,measured_lo,measured_hi,pad):
    """The measured front face (+Z) case of plane_free_witnesses."""
    return plane_free_witnesses(points,camera_pose,2,measured_hi[2],measured_lo,measured_hi,pad)


def visible_face_witnesses(points,camera_pose,models,W,measured_lo,measured_hi,pad):
    """Tightest witness per boundary over every accepted visible plane."""
    out={}
    C=np.linalg.inv(np.asarray(W,float))
    for model in models or [None]:
        if model is None:axis,position=2,measured_hi[2]
        else:
            n=C[:3,:3]@np.asarray(model['normal']);c=transform(C,[model['center_m']])[0]
            axis=int(np.argmax(abs(camera_pose[:3,:3].T@n)))
            position=float((c-camera_pose[:3,3])@camera_pose[:3,axis])
            if axis==2:position=measured_hi[2]
        for key,w in plane_free_witnesses(points,camera_pose,axis,position,measured_lo,measured_hi,pad).items():
            if key not in out or (w['outer_m']-out[key]['outer_m'])*key[1]<0:out[key]=w
    return out


SUPPORT_TILT_LIMIT_RAD = float(np.deg2rad(5))  # same limit as observed_support_plane
# Direct boundary coverage. Calibrated by the synthetic mask-defect sweep in
# docs/evidence/<date>_geometry_robustness (values recorded there).
DIRECT_MIN_PIXELS = 10
DIRECT_MIN_SPAN_M = .015


def direct_coverage(local,candidates,axis,low,high,pad):
    """Strip rule for a direct depth boundary, as in front_free_witnesses.

    A witness pixel must lie inside the measured box on the coordinate running
    ALONG the edge (other coordinate minus pad); a pixel at the end of both
    other coordinates is a corner, which cannot certify an edge. The accepted
    pixels need a minimum count and a minimum span along the edge.
    """
    others=[o for o in range(3) if o!=axis]
    interior={o:(local[...,o]>low[o]+pad[o])&(local[...,o]<high[o]-pad[o]) for o in others}
    accepted=candidates&(interior[others[0]]|interior[others[1]])
    spans={}
    for o in others:
        values=local[accepted&interior[o],o]
        spans[o]=float(np.ptp(values)) if len(values) else 0.
    count=int(accepted.sum());span=max(spans.values())
    return dict(accepted=bool(count>=DIRECT_MIN_PIXELS and span>=DIRECT_MIN_SPAN_M),
        candidate_pixels=int(candidates.sum()),strip_pixels=count,span_m=span,
        corner_pixels_rejected=int((candidates&~accepted).sum()),min_pixels=DIRECT_MIN_PIXELS,
        min_span_m=DIRECT_MIN_SPAN_M,rule='EDGE_PIXELS_INSIDE_OTHER_AXIS_SPAN_MINUS_PAD')


def support_clip(T,lo,hi,plane):
    """Conservatively clip an OBB bottom at a horizontal support up to 5 deg.

    The new bottom face is placed so its HIGHEST point is at the support top
    minus one robust sigma of the support height estimate: every removed point
    lies below the support plane, no volume above it is removed. Its lowest point is ``drop_m`` below the plane; the observed
    support solid is then bound to that lowest point (existing 0.4 mm detach
    rule). The drop may not exceed the existing support-surface noise window,
    otherwise the support is not applied and the envelope keeps its interval.
    """
    from .site_scene import SUPPORT_ABOVE_M
    T=np.asarray(T,float);g=T[2,:3];axis=int(np.argmax(abs(g)))
    tilt=float(np.arccos(min(1.,abs(g[axis]))))
    others=[j for j in range(3) if j!=axis]
    window=float(max(SUPPORT_ABOVE_M,4*plane.get('noise_m',0.)))
    record=dict(axis=axis,side=-1 if g[axis]>0 else 1,tilt_rad=tilt,tilt_limit_rad=SUPPORT_TILT_LIMIT_RAD,
        support_noise_window_m=window,rule='HIGHEST_POINT_OF_NEW_BOTTOM_AT_SUPPORT_TOP_NO_VOLUME_ABOVE_REMOVED')
    if tilt>SUPPORT_TILT_LIMIT_RAD:
        return dict(record,applied=False,reason='MEASURED_SUPPORT_TILT_EXCEEDS_5_DEG')
    # The support top is itself a noisy estimate (median of its points). The
    # highest bottom point goes one robust height sigma below it, so noise in
    # that estimate cannot cut the true carton; the drop budget includes it.
    margin=float(plane.get('noise_m',0.))
    highest=sum(max(g[j]*lo[j],g[j]*hi[j]) for j in others)
    cut=float((plane['top_world_m']-margin-T[2,3]-highest)/g[axis])
    drop=float(sum(abs(g[j])*(hi[j]-lo[j]) for j in others))
    record.update(cut_m=cut,drop_m=drop,support_estimate_margin_m=margin,
        highest_bottom_world_m=float(plane['top_world_m']-margin),
        lowest_bottom_world_m=float(plane['top_world_m']-margin-drop))
    if drop+margin>window+1e-12:
        return dict(record,applied=False,reason='MEASURED_SUPPORT_TILT_DROP_EXCEEDS_SUPPORT_NOISE_WINDOW')
    return dict(record,applied=True)


def conservative_envelope(box,points,mask,world_camera,maximum=.40,*,rgb=None,seam_policy=None,bbox=None,
                          shadow_policy=None,neighbors=None,colour_policy=None,own_mask_first=False):
    b=deepcopy(box);p=np.asarray(points,float);mask=np.asarray(mask,bool)
    W=np.asarray(world_camera,float);T=np.asarray(box['pose'],float)
    depth_unmeasured=(box.get('quality') or {}).get('depth_measured') is False
    camera_pose=np.linalg.inv(W)@T;R=camera_pose[:3,:3]
    valid=np.isfinite(p).all(-1)&(p[...,2]>.1)&(p[...,2]<3.)
    use=valid&mask
    raw_count=int(use.sum())
    # A segmentation mask is not a guarantee of valid stereo correspondence.
    # The original reconstruction already fitted trustworthy visible planes;
    # use their accepted residual, not arbitrary far/background returns inside
    # the mask, for measured extrema. Excluded returns remain in the scene
    # pipeline; no obstacle or unknown region is made free by this selection.
    models=box.get('observed_plane_models',[])
    if models:
        world=transform(W,p.reshape(-1,3)).reshape(p.shape);supported=np.zeros(mask.shape,bool)
        from .geometry import ADJACENT_EDGE_BAND_M
        main=models[0];main_distance=abs((world-np.asarray(main['center_m']))@np.asarray(main['normal']))
        disconnected=0;adjacent_only=np.zeros(mask.shape,bool)
        for k,plane in enumerate(models):
            on=use&(abs((world-np.asarray(plane['center_m']))@np.asarray(plane['normal']))<=plane['residual_m']+1e-9)
            if k:
                # An adjacent face only counts where its patch connects to the
                # common edge with the front face (same rule as thickness):
                # mask bleed across the plane's extension is not carton surface.
                _,labels=cv2.connectedComponents(on.astype('uint8'),connectivity=8)
                ids=np.unique(labels[on&(main_distance<=ADJACENT_EDGE_BAND_M)]);ids=ids[ids!=0]
                connected=on&np.isin(labels,ids);disconnected+=int((on&~connected).sum());on=connected
            supported|=on
            if k:adjacent_only|=on
        use&=supported
        b['quality']['envelope_side_points_not_connected_to_common_edge']=disconnected
    b['quality']['envelope_surface_depth_points']=int(use.sum())
    b['quality']['envelope_rejected_off_plane_depth_points']=raw_count-int(use.sum())
    b['quality']['envelope_depth_source']='ACCEPTED_VISIBLE_PLANES' if models else 'LEGACY_ALL_VALID_MASK_DEPTH'
    local=(p-camera_pose[:3,3])@R
    if models:
        # Adjacent-face samples beyond the robust connected far end (plus the
        # depth-noise allowance) are another surface crossing that plane's
        # extension, not this carton. They stay scene obstacles; the far
        # boundary itself is still decided by direct or free-ray evidence.
        limit=np.asarray(box['size_m'])/2+DEPTH_NOISE_ALLOWANCE_M
        beyond=use&adjacent_only&~(abs((world-np.asarray(models[0]['center_m']))@np.asarray(models[0]['normal']))
            <=models[0]['residual_m']+1e-9)&np.any(abs(local)>limit,axis=-1)
        use&=~beyond;b['quality']['envelope_side_points_beyond_robust_far_end']=int(beyond.sum())
    observed=local[use]
    if not len(observed):raise ValueError('NO_MEASURED_ENVELOPE_SUPPORT')
    measured_lo=-np.asarray(box['size_m'])/2;measured_hi=-measured_lo
    if np.max(box['size_m'])>maximum:raise ValueError('MEASURED_BOX_EXCEEDS_CONFIGURED_MAXIMUM_EXTENT')
    if depth_unmeasured and np.max(observed.max(0)-observed.min(0))>maximum:
        raise ValueError('MEASURED_BOX_EXCEEDS_CONFIGURED_MAXIMUM_EXTENT')
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
    free_witnesses=visible_face_witnesses(p,camera_pose,models,W,measured_lo,measured_hi,pad)
    # A direct boundary needs a valid target return immediately adjacent to
    # valid, farther background. A hole between it and the mask edge is unknown.
    boundary=mask&~cv2.erode(mask.astype('uint8'),np.ones((3,3),'uint8')).astype(bool)
    background=np.where(valid&~mask,p[...,2],-np.inf).astype('float32')
    farther=cv2.dilate(background,np.ones((3,3),'uint8'))>p[...,2]+.012
    witnesses=use&boundary&farther
    lo=low.copy();hi=high.copy();intervals=[]
    from . import coplanar_seam,stereo_shadow,rgb_color_boundary
    try:front_plane=coplanar_seam.FrontPlane(p,camera_pose,measured_hi[2])
    except ValueError as error:front_plane=error
    rms=float((box.get('quality') or {}).get('plane_rms_m',0.))
    for axis in range(3):
        for sign in (-1,1):
            edge=low[axis] if sign<0 else high[axis]
            near=abs(local[...,axis]-edge)<=pad[axis]
            coverage=direct_coverage(local,witnesses&near,axis,low,high,pad)
            direct=coverage['accepted']
            # The dominant observed plane is direct position evidence for +Z,
            # even though its outer contour can be occluded.
            if axis==2 and sign==1:direct=True
            # A lone visible face has no measured back: front-face silhouette
            # witnesses near the placeholder thickness are not a back boundary.
            if axis==2 and sign<0 and depth_unmeasured:direct=False
            shadow=None
            if axis==2 and sign==1:
                # The front face is a fitted plane, not a boundary between
                # pixels: no pixel-footprint pad and no extreme-sample plus
                # noise double count. Bound = fitted plane + depth-noise
                # allowance, never inside an accepted sample. (Field 6e13d059:
                # 6.03 mm in front of the plane left 0.67 mm to the rigid cup
                # core against the 2.4 mm pair clearance.)
                outer=max(float(measured_hi[2])+DEPTH_NOISE_ALLOWANCE_M,float(observed[:,2].max()))
                method='DIRECT_DEPTH_BOUNDARY'
            elif direct:outer=edge+sign*pad[axis];method='DIRECT_DEPTH_BOUNDARY'
            else:
                outer=(high[axis]-maximum-pad[axis] if sign<0 else low[axis]+maximum+pad[axis])
                method='MISSING_BOUNDARY_MAXIMUM_EXTENT'
                witness=free_witnesses.get((axis,sign))
                if witness and (witness['outer_m']-outer)*sign<0:
                    outer=(min(witness['outer_m'],low[axis]-pad[axis]) if sign<0 else
                           max(witness['outer_m'],high[axis]+pad[axis]))
                    method='SENSOR_FREE_RECTANGLE_SPAN_BOUND'
                if shadow_policy is not None and axis!=2:
                    # User policy (2026-11-20): a no-return band on the stereo
                    # shadow side, ended by a farther return, is background.
                    # Same strip/coverage rule as the direct boundary; only a
                    # tighter bound than the sensor evidence above is used.
                    shadow=stereo_shadow.shadow_edge(p,mask,use,local,front_plane,axis,sign,low,high,pad,rms)
                    image=shadow.pop('witnesses',None)
                    if image is not None:
                        shadow['coverage']=direct_coverage(local,(witnesses|image)&near,axis,low,high,pad)
                        shadow['accepted']=bool(shadow['coverage']['accepted'] and 'outer_m' in shadow)
                    if shadow.get('accepted') and (shadow['outer_m']-outer)*sign<0:
                        outer=shadow['outer_m'];method='STEREO_SHADOW_DEPTH_STEP_POLICY'
            missing=None
            if method=='MISSING_BOUNDARY_MAXIMUM_EXTENT':
                # Seam policy only after direct and free-ray evidence are both
                # absent; every remaining missing edge is classified.
                missing=coplanar_seam.missing_edge(p,mask,rgb,front_plane,axis,sign,low,high,pad,rms,seam_policy,bbox,neighbors)
                if missing.get('seam_policy')=='APPLIED':
                    outer=missing['outer_m'];method='COPLANAR_SEAM_MASK_RGB_EDGE_POLICY'
                elif colour_policy is not None:
                    # User policy (2026-11-20): carton colour differs from its
                    # surroundings; only after every depth rule and the seam.
                    colour=rgb_color_boundary.colour_edge(p,mask,rgb,front_plane,axis,sign,low,high,pad,rms,
                                                          colour_policy,footprint[axis])
                    missing['rgb_color_boundary']=colour
                    if colour.get('applied'):outer=colour['outer_m'];method='RGB_COLOR_BOUNDARY_POLICY'
            if sign<0:lo[axis]=min(outer,low[axis])
            else:hi[axis]=max(outer,high[axis])
            intervals.append(dict(axis=axis,side=sign,measured_m=float(edge),outer_m=float(outer),
                interval_m=sorted([float(edge),float(outer)]),method=method,
                direct_boundary_pixels=int(np.count_nonzero(witnesses&near)),direct_coverage=coverage,
                pixel_footprint_m=float(footprint[axis]),noise_allowance_m=DEPTH_NOISE_ALLOWANCE_M))
            if method=='SENSOR_FREE_RECTANGLE_SPAN_BOUND':
                intervals[-1]['free_space_witness']=free_witnesses[(axis,sign)]
            if axis==2 and sign==1:
                intervals[-1].update(rule='FITTED_FRONT_PLANE_PLUS_DEPTH_NOISE_ALLOWANCE_NOT_EXTREME_SAMPLE_PLUS_PIXEL_PAD',
                    fitted_plane_m=float(measured_hi[2]),extreme_accepted_sample_m=float(observed[:,2].max()))
            if shadow is not None:intervals[-1]['stereo_shadow']=shadow
            if method=='STEREO_SHADOW_DEPTH_STEP_POLICY':
                intervals[-1].update(basis=shadow_policy['basis'],
                    evidence_kind='USER_POLICY_INVALID_BAND_IS_BACKGROUND_BEYOND_DEPTH_STEP')
            if method=='RGB_COLOR_BOUNDARY_POLICY':
                intervals[-1].update(rgb_color_boundary=missing['rgb_color_boundary'],missing_boundary=missing,
                    basis=colour_policy['basis'],uncertainty_m=colour_policy['uncertainty_m'],
                    evidence_kind='USER_POLICY_RGB_COLOUR_DIFFERS_FROM_SURROUNDINGS_NOT_DEPTH_BOUNDARY')
            elif method=='COPLANAR_SEAM_MASK_RGB_EDGE_POLICY':
                intervals[-1].update(seam=missing,basis=seam_policy['basis'],uncertainty_m=seam_policy['uncertainty_m'],
                    evidence_kind='USER_POLICY_RGB_SEAM_NOT_DEPTH_BOUNDARY')
            elif missing is not None:
                intervals[-1].update(missing_boundary_class=missing['classification'],missing_boundary=missing)
    # The carton's own returns off the fitted planes (e.g. a bulging or skewed
    # side) must lie inside the envelope: grow (never shrink) each bound to
    # contain them plus the same pad. Other instances' pixels never count.
    # Unlabelled returns just outside the mask (stereo fattening past the
    # silhouette) count when depth-continuous, within the scene's pixel band.
    from .site_scene import TARGET_PIXEL_BAND_PX
    band=cv2.dilate(mask.astype(np.uint8),np.ones((2*TARGET_PIXEL_BAND_PX+1,)*2,np.uint8)).astype(bool)
    region=band&valid&~depth_jump(p[...,2],valid)
    if neighbors is not None:
        # own_mask_claim_policy (2026-11-23): an overlapping instance does not
        # take pixels of this carton's own mask away from it.
        region&=~(np.asarray(neighbors,bool)&~mask if own_mask_first else np.asarray(neighbors,bool))
    if own_mask_first:
        # Own-mask no-return pixels (tape, grazing top face) connect own
        # returns; depth jumps still separate background bleed. Grows only.
        own=own_connected_surface(region|(mask&~valid),use)&region
        # Silhouette fattening (7-14 px, user 2026-11-20): returns on the
        # front-face plane band within SILHOUETTE_ARTIFACT_PX of the mask,
        # whatever their instance label, are contained (grow only); another
        # measured carton's pixels are cut back by non_interpenetration.
        from .site_scene import SILHOUETTE_ARTIFACT_PX,TARGET_PLANE_BAND_M
        near_mask=cv2.distanceTransform((~mask).astype(np.uint8),cv2.DIST_L2,5)<=SILHOUETTE_ARTIFACT_PX
        with np.errstate(invalid='ignore'):on_front=abs(local[...,2]-measured_hi[2])<=TARGET_PLANE_BAND_M
        face_fat=valid&near_mask&on_front&~depth_jump(p[...,2],valid)&~mask
        b['quality']['face_band_fattening_points_contained']=int((face_fat&~own).sum())
        own|=face_fat
    else:own=own_connected_surface(region,use)
    extra=own&~use
    b['quality']['own_surface_points_off_fitted_planes']=int(extra.sum())
    if extra.any():
        e=local[extra]
        # Front face (+Z): cardboard may bulge up to TARGET_PLANE_BAND_M past
        # the fitted plane (existing site_scene rule); own returns within that
        # band are contained (no extra pad: the 3 mm allowance is in the plane
        # bound), returns farther in front are not this face and are only
        # recorded (field 67dbcaac: 95 px of corner noise 4-12 mm in front).
        from .site_scene import TARGET_PLANE_BAND_M
        face_band=float(measured_hi[2])+TARGET_PLANE_BAND_M
        b['quality']['own_surface_points_in_front_beyond_face_band']=int((e[:,2]>face_band).sum())
        for interval in intervals:
            a,sign=interval['axis'],interval['side']
            if a==2 and sign>0:
                within=e[:,2][e[:,2]<=face_band]
                reach=float(within.max()) if len(within) else -np.inf
            else:reach=float(e[:,a].min()-pad[a] if sign<0 else e[:,a].max()+pad[a])
            if (reach-interval['outer_m'])*sign>0:
                interval['own_surface_extension']=dict(previous_outer_m=interval['outer_m'],outer_m=reach,
                    points=int(extra.sum()),basis=OWN_SURFACE_BASIS,rule='OWN_MASK_DEPTH_CONNECTED_RETURNS_PLUS_PAD')
                interval['outer_m']=reach;interval['interval_m']=sorted([interval['measured_m'],reach])
                if sign<0:lo[a]=reach
                else:hi[a]=reach
    measured={k:deepcopy(box[k]) for k in ('pose','size_m','corners_m','faces')}
    b.update(measured_box=measured,measured_size_m=deepcopy(box['size_m']))
    from .partial_geometry import observed_support_plane
    front=camera_pose[:3,3]+R[:,2]*measured_hi[2]
    support=p[use&(abs(local[...,2]-measured_hi[2])<.003)]
    plane=observed_support_plane(p,mask,W,front,R,measured_lo[:2],measured_hi[:2],support) if len(support) else None
    if plane:
        clip=support_clip(T,lo,hi,plane)
        if clip['applied']:
            axis=clip['axis'];cut=clip['cut_m']
            if clip['side']<0:lo[axis]=cut
            else:hi[axis]=cut
            if np.any(hi<=lo):raise ValueError('MEASURED_SUPPORT_ENVELOPE_CONFLICT')
            plane=dict(plane,**{k:v for k,v in clip.items() if k!='applied'})
            b['quality']['observed_support_plane']=plane
            for interval in intervals:
                if interval['axis']==axis and interval['side']==clip['side']:
                    interval.update(outer_m=float(cut),interval_m=sorted([interval['measured_m'],float(cut)]),
                        method='MEASURED_SUPPORT_PLANE',support=plane)
        else:b['quality']['observed_support_plane_not_applied']=dict(plane,**clip)
    center=(lo+hi)/2;size=hi-lo;pose=T.copy();pose[:3,3]+=T[:3,:3]@center
    b.update(pose=pose.tolist(),size_m=size.tolist(),corners_m=corners(pose,size).tolist())
    b['collision_envelope']={k:deepcopy(b[k]) for k in ('pose','size_m','corners_m')}
    b['quality']['hidden_extent_measured']=False
    b['quality']['full_extent_bounded']=True
    b['quality']['coplanar_seam_policy']=dict(seam_policy) if seam_policy else dict(mode='DISABLED')
    b['quality']['stereo_shadow_boundary_policy']=dict(shadow_policy) if shadow_policy else dict(mode='DISABLED')
    b['quality']['rgb_color_boundary_policy']=dict(colour_policy) if colour_policy else dict(mode='DISABLED')
    b['uncertainty']=dict(kind='MEASURED_BOUNDARY_INTERVALS',bound_method='PIXEL_FOOTPRINT_AND_MAXIMUM_EXTENT',
        frame='MEASURED_BOX',intervals=intervals,lower_m=lo.tolist(),upper_m=hi.tolist(),
        max_carton_extent_m=maximum,pixel_footprints=BOUNDARY_PIXEL_FOOTPRINTS,
        depth_noise_allowance_m=DEPTH_NOISE_ALLOWANCE_M,
        basis='SYNTHETIC_SWEEP_CALIBRATION_NOT_FIELD_DIMENSION_VALIDATION')
    return b


EDGE_METHOD_LABELS={'DIRECT_DEPTH_BOUNDARY':'深度边界','SENSOR_FREE_RECTANGLE_SPAN_BOUND':'自由射线',
    'MEASURED_SUPPORT_PLANE':'实测支撑面','CONFIRMED_STACK_SUPPORT_PLANE':'叠箱支撑面',
    'COPLANAR_SEAM_MASK_RGB_EDGE_POLICY':'接缝策略','STEREO_SHADOW_DEPTH_STEP_POLICY':'阴影带深度差','RGB_COLOR_BOUNDARY_POLICY':'颜色边界',
    'MISSING_BOUNDARY_MAXIMUM_EXTENT':'缺边','NON_INTERPENETRATION_OTHER_INSTANCE_SURFACE':'邻物实测面截止'}
MISSING_CLASS_LABELS={'COPLANAR_CONTINUATION':'共面延续','INVALID_DEPTH_BAND':'无效深度带',
    'FOREGROUND_OCCLUDER':'前景遮挡','IMAGE_BORDER':'画面边缘','DETECTION_BOX_EDGE':'贴检测框边',
    'NO_RGB_EDGE':'无RGB边缘','OTHER':'其他'}


def edge_summary(box):
    """One display line: envelope size and the method of each of the six edges."""
    u=box.get('uncertainty') or {}
    if u.get('kind')!='MEASURED_BOUNDARY_INTERVALS':return None
    parts=[]
    for i in u['intervals']:
        name='宽高厚'[i['axis']]+('-' if i['side']<0 else '+')
        text=EDGE_METHOD_LABELS.get(i['method'],i['method'])
        if i['method']=='MISSING_BOUNDARY_MAXIMUM_EXTENT':
            text+='（'+MISSING_CLASS_LABELS.get(i.get('missing_boundary_class'),'未分类')+'）'
        parts.append(name+' '+text)
    size=(np.asarray(box['size_m'])*1000).round(1).tolist()
    return '包络 '+str(size)+' mm · '+'，'.join(parts)
