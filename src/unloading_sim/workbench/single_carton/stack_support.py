"""A confirmed stack relation bounds a static support, not a carton size.

The field statement supplies stacked support topology. Frozen RGB-D must
independently associate the two instances at the upper carton's bottom. The
lower carton may extend arbitrarily far inward/laterally: keep its entire
possible body within the enforced workspace, below that support plane.
"""
from itertools import combinations
import cv2
import numpy as np
from .geometry import rigid,transform


def policy(geometry):
    value=(geometry or {}).get('confirmed_stack_support',{'mode':'DISABLED'})
    if not isinstance(value,dict):raise ValueError('STACK_SUPPORT_POLICY_INVALID')
    if value.get('mode')=='DISABLED':return None
    if (value.get('mode')!='USER_CONFIRMED_TWO_CARTON_STACK' or
        not isinstance(value.get('basis'),str) or not value['basis'].strip()):
        raise ValueError('STACK_SUPPORT_FIELD_CONFIRMATION_REQUIRED')
    spread=value.get('max_height_spread_m')
    if spread is not None:
        # User decision 2026-11-23 ("放宽判据"): the height-estimate spread
        # limit may be enlarged with a basis; the default stays SUPPORT_BELOW_M.
        from .site_scene import SUPPORT_BELOW_M
        if (isinstance(spread,bool) or not isinstance(spread,(int,float)) or
            not SUPPORT_BELOW_M<=float(spread)<=MAX_HEIGHT_SPREAD_CAP_M):
            raise ValueError(f'STACK_SUPPORT_MAX_HEIGHT_SPREAD_OUT_OF_RANGE_{SUPPORT_BELOW_M}_{MAX_HEIGHT_SPREAD_CAP_M}')
        if not isinstance(value.get('max_height_spread_basis'),str) or not value['max_height_spread_basis'].strip():
            raise ValueError('STACK_SUPPORT_MAX_HEIGHT_SPREAD_BASIS_REQUIRED')
    return dict(value)


def intersect_vertices(normals,offsets):
    """Vertices of a bounded half-space intersection n.x <= d."""
    A=np.asarray(normals,float);b=np.asarray(offsets,float);vertices=[]
    for index in combinations(range(len(A)),3):
        matrix=A[list(index)]
        if abs(np.linalg.det(matrix))<1e-10:continue
        p=np.linalg.solve(matrix,b[list(index)])
        if np.all(A@p<=b+1e-9):vertices.append(p)
    if len(vertices)<4:raise ValueError('STACK_SUPPORT_EMPTY_WORKSPACE_INTERSECTION')
    return np.unique(np.round(vertices,12),axis=0)


SEAM_BAND_OWNERSHIP_BASIS=('USER_DECISION_2026_11_20_SEAM_BAND_POINTS_OWNED_BY_UPPER_VISIBLE_FACE_'
                           'LOWER_VISIBLE_TOP_IS_OCCLUSION_LIMITED_NOT_THE_SUPPORT_HEIGHT')
STACK_INSTANCE_DUPLICATE_IOU=.95   # same rule as recover_boxes deduplication
STACK_INSTANCE_NESTED=.95          # same rule as nested-instance fitting
LOWER_TOP_QUANTILE=99.5
# The upper's bottom is an in-plane edge of its measured front face; a lone
# face (depth bounded by the maximum extent, 2026-11-23) measures it the same way.
UPPER_MEASURED_SOURCES=('RGBD_VISIBLE_BOUNDARIES','RGBD_VISIBLE_FACE_BOUNDARIES_DEPTH_MAX_EXTENT')
# Upper bound of the optional height-spread limit (user decision 2026-11-23):
# beyond it the lower interface returns (<= SUPPORT_BELOW_M under the plane)
# could no longer be the lower's top seen under an overhang.
MAX_HEIGHT_SPREAD_CAP_M=.03


def effective_instances(masks,outside_ignored=()):
    """Physical instances after duplicate removal, nested/group handling and
    OUTSIDE_WORKSPACE ignoring. A mask containing two or more other instances
    (each >= 95 %) is a whole-stack mask, not another carton."""
    masks=[np.asarray(m,bool) for m in masks];kept=[]
    for i,m in enumerate(masks):
        if any(np.count_nonzero(m&masks[j])/max(1,np.count_nonzero(m|masks[j]))>=STACK_INSTANCE_DUPLICATE_IOU for j in kept):continue
        kept.append(i)
    kept=[i for i in kept if i not in set(outside_ignored)]
    groups=[]
    for i in kept:
        area=np.count_nonzero(masks[i])
        inside=[j for j in kept if j!=i and 0<np.count_nonzero(masks[j])<area and
                np.count_nonzero(masks[i]&masks[j])/np.count_nonzero(masks[j])>=STACK_INSTANCE_NESTED]
        if len(inside)>=2:groups.append(i)
    return [i for i in kept if i not in groups],dict(after_duplicates_and_outside=kept,group_masks=groups,
        total_masks=len(masks),outside_ignored=sorted(set(outside_ignored)))


def _measured_frame(upper):
    m=upper.get('measured_box') or upper
    T=rigid(m['pose']);R=T[:3,:3];half=np.asarray(m['size_m'],float)/2
    axis=int(np.argmax(abs(R[2])));down=-np.sign(R[2,axis])  # side of the bottom face
    normal=R[:,axis]*np.sign(R[2,axis])
    return T,R,half,axis,int(down),normal


def clip_upper_bottom(upper,offset,record):
    """Set the upper envelope bottom to the support plane (bigger or cut below it)."""
    from .geometry import corners
    T,R,half,axis,side,normal=_measured_frame(upper)
    u=upper['uncertainty'];lo=np.asarray(u['lower_m'],float);hi=np.asarray(u['upper_m'],float)
    c=float((offset-normal@T[:3,3])/(normal@R[:,axis]))
    if side<0:
        if c>-half[axis]+1e-9:raise ValueError('STACK_SUPPORT_PLANE_ABOVE_MEASURED_BOTTOM')
        previous=float(lo[axis]);lo[axis]=c
    else:
        if c<half[axis]-1e-9:raise ValueError('STACK_SUPPORT_PLANE_ABOVE_MEASURED_BOTTOM')
        previous=float(hi[axis]);hi[axis]=c
    if np.any(hi<=lo):raise ValueError('STACK_SUPPORT_ENVELOPE_CONFLICT')
    pose=T.copy();pose[:3,3]+=R@((lo+hi)/2);size=hi-lo
    upper.update(pose=pose.tolist(),size_m=size.tolist(),corners_m=corners(pose,size).tolist())
    upper['collision_envelope']={k:upper[k] for k in ('pose','size_m','corners_m')}
    u.update(lower_m=lo.tolist(),upper_m=hi.tolist())
    for interval in u['intervals']:
        if interval['axis']==axis and interval['side']==side:
            interval.update(previous_method=interval['method'],previous_outer_m=interval['outer_m'],
                method='CONFIRMED_STACK_SUPPORT_PLANE',outer_m=c,interval_m=sorted([interval['measured_m'],c]),
                support=record)
    upper.setdefault('quality',{})['confirmed_stack_support_clip']=dict(record,axis=axis,side=side,
        previous_outer_m=previous,outer_m=c,rule='ENVELOPE_BOTTOM_AT_UPPER_VISIBLE_FACE_BOTTOM_BOUND')


def contain_seam_band_front(upper,world_points,owned,record):
    """Grow the upper envelope's front (+Z of the measured frame) to contain
    the seam-band returns the upper owns, within the existing face band."""
    from .geometry import corners
    from .site_scene import TARGET_PLANE_BAND_M
    T,R,half,axis,side,normal=_measured_frame(upper)
    if axis==2 or not owned.any():return None
    u=upper['uncertainty'];lo=np.asarray(u['lower_m'],float);hi=np.asarray(u['upper_m'],float)
    t=((world_points[owned]-T[:3,3])@R)[:,2];t=t[t<=half[2]+TARGET_PLANE_BAND_M]
    if not len(t) or t.max()<=hi[2]:return dict(grown=False,points=int(owned.sum()))
    previous=float(hi[2]);hi[2]=float(t.max())
    pose=T.copy();pose[:3,3]+=R@((lo+hi)/2);size=hi-lo
    upper.update(pose=pose.tolist(),size_m=size.tolist(),corners_m=corners(pose,size).tolist())
    upper['collision_envelope']={k:upper[k] for k in ('pose','size_m','corners_m')}
    u.update(lower_m=lo.tolist(),upper_m=hi.tolist())
    extension=dict(grown=True,points=int(owned.sum()),previous_outer_m=previous,outer_m=hi[2],
        cap_m=float(half[2]+TARGET_PLANE_BAND_M),basis=SEAM_BAND_OWNERSHIP_BASIS,record=record)
    for interval in u['intervals']:
        if interval['axis']==2 and interval['side']==1:
            interval['seam_band_front_extension']=extension;interval['outer_m']=hi[2]
            interval['interval_m']=sorted([interval['measured_m'],hi[2]])
    return extension


def infer_supports(observation,result,boxes,unresolved,site,geometry,world_points,valid,outside_ignored=()):
    """Return source-bound support bodies and explicit rejected associations.

    The upper envelope may be a MEASURED_BOUNDARY_INTERVALS envelope; its
    bottom is then set (in place) to the support plane. User decision
    2026-11-20: the upper carton's visible face decides the seam band. The
    plane is the upper measured bottom moved down by that edge's direct
    boundary pad (one pixel footprint plus the depth-noise allowance), or the
    seam line minus its uncertainty where the seam policy bounded that edge,
    whichever is lower. The lower visible-face top is limited by occlusion
    under the upper's front edge (field frames: 9.6-11.3 mm below the upper
    bottom); it only checks consistency and no longer lowers the plane.
    Returns in the seam band above the plane inside the upper envelope are
    the upper's (site_scene.stack_seam_band_claims); below it, the lower's.
    """
    from .site_scene import TARGET_PIXEL_BAND_PX,TARGET_PLANE_BAND_M,SUPPORT_MIN_POINTS,SUPPORT_BELOW_M,SUPPORT_ABOVE_M,NEAR_TARGET_M
    resting=resting_policy(geometry)
    if resting is not None:
        return general_supports(observation,result,boxes,unresolved,site,world_points,valid,outside_ignored,resting)
    setting=policy(geometry)
    if setting is None:return {},[]
    effective,counting=effective_instances(result.masks,outside_ignored)
    boxes=[b for b in boxes if b['detection_index'] in effective]
    unresolved=[u for u in unresolved if u['detection_index'] in effective]
    if len(boxes)!=1 or not unresolved:
        return {},[dict(status='NOT_APPLIED',reason='CONFIRMATION_ONLY_COVERS_TWO_CARTON_STACK',instances=counting,
                        effective_instances=effective)]
    upper=boxes[0]
    def reject(reason,lower=None,**detail):
        return {},[dict(status='NOT_APPLIED',reason=reason,upper_id=upper['id'],
                        lower_index=None if lower is None else lower['detection_index'],**detail)]
    kind=(upper.get('uncertainty') or {}).get('kind')
    if ((upper.get('uncertainty') and kind!='MEASURED_BOUNDARY_INTERVALS') or
        (upper.get('quality') or {}).get('all_dimensions_source') not in UPPER_MEASURED_SOURCES):
        return reject('MEASURED_UPPER_BOTTOM_AND_RELIABLE_LOWER_FACE_REQUIRED')
    T,R,half,axis,side,normal=_measured_frame(upper)
    if normal[2]<np.cos(np.deg2rad(10)):
        return reject('UPPER_BOTTOM_NOT_WITHIN_EXISTING_SUPPORT_ORIENTATION_RANGE')
    # The upper's own pixels: a nested difference fit excludes its children.
    upper_mask=np.asarray(result.masks[upper['detection_index']],bool).copy()
    fit=upper.get('instance_fit') or {}
    if fit.get('used')=='DIFFERENCE_MASK':
        for child in fit.get('excluded',[]):upper_mask&=~np.asarray(result.masks[child['detection_index']],bool)
    band=cv2.dilate(upper_mask.astype('uint8'),np.ones((2*TARGET_PIXEL_BAND_PX+1,)*2,'uint8')).astype(bool)
    local=(world_points-T[:3,3])@R;planar=[i for i in range(3) if i!=axis]
    measured_bottom=float(normal@T[:3,3]-half[axis]);distance=world_points@normal-measured_bottom
    # Same column: the lower is the one unresolved instance with a same-frame
    # interface at the upper's bottom; other instances (elsewhere in view)
    # neither block nor join the confirmed two-carton relation.
    # A support is under the upper: its interface returns lie within the upper
    # measured footprint (plus the measured extent pad), not merely within the
    # 60 mm neighbourhood (field 9b51a37e: a black object behind the upper's
    # corner gave 90 such points and made the lower "ambiguous").
    from .site_scene import MEASURED_EXTENT_PAD_M
    under=np.all(abs(local[...,planar])<=half[planar]+MEASURED_EXTENT_PAD_M,axis=-1)
    interfaces={}
    for u in unresolved:
        m=np.asarray(result.masks[u['detection_index']],bool)
        interfaces[u['detection_index']]=int((m&band&valid&under&(distance>=-SUPPORT_BELOW_M)&
                                              (distance<=SUPPORT_ABOVE_M+SUPPORT_BELOW_M)).sum())
    lowers=[u for u in unresolved if interfaces[u['detection_index']]>=SUPPORT_MIN_POINTS]
    if not lowers:return reject('NO_SAME_FRAME_SUPPORT_INTERFACE',interface_points_by_instance=interfaces)
    if len(lowers)>1:return reject('AMBIGUOUS_STACK_LOWER_MORE_THAN_ONE_INTERFACE',interface_points_by_instance=interfaces)
    lower=lowers[0];e=lower.get('evidence') or {};face=e.get('observed_face')
    def reject(reason,_lower=lower,**detail):
        return {},[dict(status='NOT_APPLIED',reason=reason,upper_id=upper['id'],lower_index=_lower['detection_index'],**detail)]
    if not face:return reject('MEASURED_UPPER_BOTTOM_AND_RELIABLE_LOWER_FACE_REQUIRED')
    if abs(normal@np.asarray(face['normal']))>np.sin(np.deg2rad(10)):
        return reject('LOWER_FACE_NOT_COMPATIBLE_WITH_STACK_SUPPORT_PLANE')
    lower_mask=np.asarray(result.masks[lower['detection_index']],bool)
    column=dict(lower_index=lower['detection_index'],interface_points_by_instance=interfaces,
                other_instances_not_in_column=[u['detection_index'] for u in unresolved if u is not lower])
    # Height estimates along the support normal.
    measured_bottom=float(normal@T[:3,3]-half[axis])
    estimates=dict(upper_measured_bottom=measured_bottom)
    from .measured_envelope import BOUNDARY_PIXEL_FOOTPRINTS,DEPTH_NOISE_ALLOWANCE_M
    bottom_interval=next((i for i in (upper.get('uncertainty') or {}).get('intervals',[])
                          if i['axis']==axis and i['side']==side),None)
    pixel_footprint=float(bottom_interval.get('pixel_footprint_m',0.)) if bottom_interval else 0.
    face_pad=BOUNDARY_PIXEL_FOOTPRINTS*pixel_footprint+DEPTH_NOISE_ALLOWANCE_M
    # Same bound as a direct depth boundary of that edge: lowest observed
    # face sample (the interval's measured position) plus the pad.
    lowest=(float(bottom_interval['measured_m']) if bottom_interval else side*half[axis])
    selected=dict(upper_visible_face_bottom_bound=float(normal@(T[:3,3]+R[:,axis]*(lowest+side*face_pad))))
    front=np.asarray(face['normal']);on_face=abs((world_points-np.asarray(face['center_m']))@front)<=TARGET_PLANE_BAND_M
    lower_face=lower_mask&~upper_mask&valid&on_face
    if int(lower_face.sum())>=SUPPORT_MIN_POINTS:
        estimates['lower_visible_top']=float(np.percentile(world_points[lower_face]@normal,LOWER_TOP_QUANTILE))
    if bottom_interval and bottom_interval['method']=='COPLANAR_SEAM_MASK_RGB_EDGE_POLICY':
        selected['seam_line_minus_uncertainty']=float(normal@(T[:3,3]+R[:,axis]*bottom_interval['outer_m']))
    estimates.update(selected)
    offset=min(selected.values())
    record_estimates=dict(estimates=estimates,selected_m=offset,selected_from=sorted(selected),
        upper_visible_face_pad_m=face_pad,spread_m=float(max(estimates.values())-min(estimates.values())))
    if estimates.get('lower_visible_top',-np.inf)-measured_bottom>SUPPORT_ABOVE_M:
        return reject('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',**record_estimates)
    spread_limit=float(setting.get('max_height_spread_m',SUPPORT_BELOW_M))
    record_estimates['spread_limit_m']=spread_limit
    if setting.get('max_height_spread_m') is not None:record_estimates['spread_limit_basis']=setting['max_height_spread_basis']
    if record_estimates['spread_m']>spread_limit:
        return reject('STACK_SUPPORT_HEIGHT_ESTIMATES_DISAGREE',**record_estimates)
    support=np.asarray(face['center_m'])+np.asarray(face['support_points_uv_m'])@np.asarray([face['u'],face['v']])
    signed=support@normal-offset
    if signed.min()>=0:
        return reject('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',min_below_plane_m=float(signed.min()),**record_estimates)
    distance=world_points@normal-offset
    joint=lower_mask&band&valid&under&(distance>=-SUPPORT_BELOW_M)&(distance<=SUPPORT_ABOVE_M+measured_bottom-offset)
    if int(joint.sum())<SUPPORT_MIN_POINTS:
        return reject('NO_SAME_FRAME_SUPPORT_INTERFACE',interface_points=int(joint.sum()),**record_estimates)
    # Full possible lower body: workspace AND inward side of its observed
    # front plane AND below the support plane. No depth/width guess.
    lo,hi=np.asarray(site['workspace_bounds_m'],float)
    A=list(np.eye(3))+list(-np.eye(3));b=list(hi)+list(-lo)
    A += [front,normal]
    b += [float(front@np.asarray(face['center_m'])+TARGET_PLANE_BAND_M),offset]
    vertices=intersect_vertices(A,b)
    x=front-normal*(normal@front);x/=np.linalg.norm(x)
    frame=np.column_stack((x,np.cross(normal,x),normal));v=vertices@frame
    low,high=v.min(0),v.max(0);high[2]=offset
    pose=np.eye(4);pose[:3,:3]=frame;pose[:3,3]=frame@((low+high)/2)
    body=dict(id=f"rgbd_confirmed_stack_support_{lower['detection_index']}_for_{upper['id']}",
        category='confirmed_stack_support',pose=pose.tolist(),size=(high-low).tolist(),
        supports_target_id=upper['id'],detection_index=lower['detection_index'],geometry_uncertain=True,
        support_plane=dict(normal_world=normal.tolist(),offset_m=offset),
        support_basis=setting['basis'],observation_id=observation.frame.observation_id)
    record=dict(status='APPLIED',upper_id=upper['id'],lower_index=lower['detection_index'],
        object_id=body['id'],basis=setting['basis'],plane=body['support_plane'],
        plane_source='UPPER_VISIBLE_FACE_BOTTOM_DIRECT_BOUND_OR_SEAM_PLUS_USER_CONFIRMED_STACK_TOPOLOGY',
        seam_band_ownership=dict(rule='RETURNS_INSIDE_UPPER_ENVELOPE_BETWEEN_PLANE_AND_UPPER_MEASURED_BOTTOM_ARE_UPPER',
            basis=SEAM_BAND_OWNERSHIP_BASIS,upper_index=upper['detection_index'],lower_index=lower['detection_index'],
            plane_m=offset,upper_measured_bottom_m=measured_bottom,band_top_m=measured_bottom+SUPPORT_ABOVE_M,
            normal_world=normal.tolist(),image_band_px=TARGET_PIXEL_BAND_PX),
        **record_estimates,instances=counting,column=column,
        interface_points=int(joint.sum()),interface_gap_m=[float(-distance[joint].max()),float(-distance[joint].min())],
        lower_front_max_above_plane_m=float(signed.max()),
        hidden_depth_measured=False,full_dimensions_m=None,carton_end_at_workspace_boundary_assumed=False,
        complete_static_domain_vertices=vertices.tolist(),support_penetration_limit_m=.0004,
        contact_permission_scope='THIS_TARGET_PAYLOAD_DURING_DETACH_ONLY_NO_ROBOT_OR_TOOL_EXCEPTION')
    if kind=='MEASURED_BOUNDARY_INTERVALS':
        try:clip_upper_bottom(upper,offset,dict(object_id=body['id'],**record_estimates))
        except ValueError as error:return reject(str(error),**record_estimates)
        record['upper_envelope_bottom_set_to_plane']=True
        # Seam-band returns the upper owns (user decision 2026-11-20) must be
        # inside its envelope: those just in front of the fitted face (bulged
        # bottom edge) grow the front, capped by the existing face band.
        labelled=np.zeros(valid.shape,bool)
        for m in result.masks:labelled|=np.asarray(m,bool)
        local=(world_points-T[:3,3])@R;lateral=[i for i in (0,1) if i!=axis]
        u=upper['uncertainty'];lo=np.asarray(u['lower_m']);hi=np.asarray(u['upper_m'])
        height=world_points@normal
        owned=(valid&band&(upper_mask|lower_mask|~labelled)&(height>=offset)&(height<=measured_bottom+SUPPORT_ABOVE_M)&
               np.all((local[...,lateral]>=lo[lateral])&(local[...,lateral]<=hi[lateral]),axis=-1)&(local[...,2]>=lo[2]))
        record['seam_band_front_extension']=contain_seam_band_front(upper,world_points,owned,dict(object_id=body['id']))
    return {lower['detection_index']:(body,record)},[record]


# ---------------------------------------------------------------------------
# General resting contact (user decision 2026-11-23, item 4). Stacking is the
# normal case, not a field confirmation of one two-carton column: every
# measured carton rests on whatever is directly under its measured bottom.
#
# * The support plane comes from the upper alone (its visible face bottom
#   bound, or the seam line minus its uncertainty), as in the confirmed stack;
#   the lower visible top is occlusion-limited and only checks conflict.
# * Every unresolved instance with same-frame interface returns under the
#   upper footprint (the upper's own pixels excluded) and a reliable face is a
#   lower: its complete possible body below the plane (workspace, inward of
#   its front plane) replaces its neighbour shadow and is a support of that
#   upper (payload contact during detach only, 0.4 mm penetration, as before).
#   Several lowers may support one upper; a lower supports one upper.
# * A measured carton lying under another measured carton is demoted to such
#   a lower before the scene is built (demote_resting_lowers): it cannot be
#   picked first, and its possible body is the support body.
# * A lower without a reliable face stays an ordinary neighbour.
# No spread limit, no single-column or single-lower requirement.
# ---------------------------------------------------------------------------

def resting_policy(geometry):
    value=(geometry or {}).get('resting_contact_policy')
    if value is None or (isinstance(value,dict) and value.get('mode')=='DISABLED'):return None
    if (not isinstance(value,dict) or value.get('mode')!='GENERAL_RESTING_CONTACT' or
        not isinstance(value.get('basis'),str) or not value['basis'].strip()):
        raise ValueError('RESTING_CONTACT_POLICY_INVALID')
    beside=value.get('lower_top_beside')
    if beside is not None and (beside!='SUPPORT_SURFACE' or not isinstance(value.get('lower_top_beside_basis'),str)
                               or not value['lower_top_beside_basis'].strip()):
        raise ValueError('RESTING_CONTACT_LOWER_TOP_BESIDE_INVALID')
    return dict(value)


def _measured_corners(box):
    from .geometry import corners
    m=box.get('measured_box') or box
    return corners(m['pose'],m['size_m'])


def demote_resting_lowers(boxes,rejected,geometry):
    """Move measured cartons that lie under another measured carton to the
    unresolved list (reason RESTING_UNDER_RESOLVED_UPPER), keeping their face."""
    from .site_scene import SUPPORT_ABOVE_M,SUPPORT_BELOW_M,MEASURED_EXTENT_PAD_M
    if resting_policy(geometry) is None or len(boxes)<2:return boxes,rejected
    demoted={}
    for lower in boxes:
        lc=_measured_corners(lower);envelope_top=float(np.asarray(lower['corners_m'])[:,2].max())
        for upper in boxes:
            if upper is lower or upper['detection_index'] in demoted:continue
            uc=_measured_corners(upper);bottom=float(uc[:,2].min())
            overlap=np.all(np.minimum(lc[:,:2].max(0),uc[:,:2].max(0))-np.maximum(lc[:,:2].min(0),uc[:,:2].min(0))>MEASURED_EXTENT_PAD_M)
            if (overlap and float(lc[:,2].max())<=bottom+SUPPORT_ABOVE_M and envelope_top>=bottom-SUPPORT_BELOW_M):
                face=dict(lower['faces'][0]);quality=lower.get('quality') or {}
                demoted[lower['detection_index']]=dict(detection_index=lower['detection_index'],reason='RESTING_UNDER_RESOLVED_UPPER',
                    evidence=dict(observed_face=face,resting_under=upper['id'],demoted_box_id=lower['id'],
                        demoted_measured_box=lower.get('measured_box'),demoted_envelope={k:lower[k] for k in ('pose','size_m')},
                        measurement_failure='RESTING_UNDER_RESOLVED_UPPER',measured_face_size_m=quality.get('measured_face_size_m'),
                        rule='LOWER_CANNOT_BE_PICKED_BEFORE_ITS_UPPER_ITS_POSSIBLE_BODY_IS_THE_SUPPORT_BODY'))
                break
    if not demoted:return boxes,rejected
    return ([b for b in boxes if b['detection_index'] not in demoted],
            sorted(list(rejected)+list(demoted.values()),key=lambda r:r['detection_index']))


def general_supports(observation,result,boxes,unresolved,site,world_points,valid,outside_ignored,setting):
    from .site_scene import TARGET_PIXEL_BAND_PX,TARGET_PLANE_BAND_M,SUPPORT_MIN_POINTS,SUPPORT_BELOW_M,SUPPORT_ABOVE_M,MEASURED_EXTENT_PAD_M
    from .measured_envelope import BOUNDARY_PIXEL_FOOTPRINTS,DEPTH_NOISE_ALLOWANCE_M
    effective,counting=effective_instances(result.masks,outside_ignored)
    uppers=[b for b in boxes if b['detection_index'] in effective]
    candidates=[u for u in unresolved if u['detection_index'] in effective]
    labelled=np.zeros(valid.shape,bool)
    for m in result.masks:labelled|=np.asarray(m,bool)
    bodies={};records=[]
    def bottom_of(b):
        return float(_measured_corners(b)[:,2].min())
    for upper in sorted(uppers,key=lambda b:(-bottom_of(b),b['detection_index'])):
        def note(reason,lower=None,**detail):
            records.append(dict(status='NOT_APPLIED',reason=reason,rule='GENERAL_RESTING_CONTACT',upper_id=upper['id'],
                                lower_index=None if lower is None else lower['detection_index'],**detail))
        kind=(upper.get('uncertainty') or {}).get('kind')
        # Same eligibility as the confirmed stack: a measured box, with or
        # without measured boundary intervals (only those are clipped).
        if ((upper.get('uncertainty') and kind!='MEASURED_BOUNDARY_INTERVALS') or
            (upper.get('quality') or {}).get('all_dimensions_source') not in UPPER_MEASURED_SOURCES):
            note('MEASURED_UPPER_BOTTOM_REQUIRED');continue
        T,R,half,axis,side,normal=_measured_frame(upper)
        if normal[2]<np.cos(np.deg2rad(10)):note('UPPER_BOTTOM_NOT_WITHIN_EXISTING_SUPPORT_ORIENTATION_RANGE');continue
        upper_mask=np.asarray(result.masks[upper['detection_index']],bool).copy()
        fit=upper.get('instance_fit') or {}
        if fit.get('used')=='DIFFERENCE_MASK':
            for child in fit.get('excluded',[]):upper_mask&=~np.asarray(result.masks[child['detection_index']],bool)
        band=cv2.dilate(upper_mask.astype('uint8'),np.ones((2*TARGET_PIXEL_BAND_PX+1,)*2,'uint8')).astype(bool)
        local=(world_points-T[:3,3])@R;planar=[i for i in range(3) if i!=axis]
        measured_bottom=float(normal@T[:3,3]-half[axis]);distance=world_points@normal-measured_bottom
        under=np.all(abs(local[...,planar])<=half[planar]+MEASURED_EXTENT_PAD_M,axis=-1)
        bottom_interval=next((i for i in (upper.get('uncertainty') or {}).get('intervals',[])
                              if i['axis']==axis and i['side']==side),None)
        pixel_footprint=float(bottom_interval.get('pixel_footprint_m',0.)) if bottom_interval else 0.
        face_pad=BOUNDARY_PIXEL_FOOTPRINTS*pixel_footprint+DEPTH_NOISE_ALLOWANCE_M
        lowest=(float(bottom_interval['measured_m']) if bottom_interval else side*half[axis])
        selected=dict(upper_visible_face_bottom_bound=float(normal@(T[:3,3]+R[:,axis]*(lowest+side*face_pad))))
        if bottom_interval and bottom_interval['method']=='COPLANAR_SEAM_MASK_RGB_EDGE_POLICY':
            selected['seam_line_minus_uncertainty']=float(normal@(T[:3,3]+R[:,axis]*bottom_interval['outer_m']))
        offset=min(selected.values())
        estimates=dict(upper_measured_bottom=measured_bottom,**selected)
        chosen=[]
        for lower in candidates:
            # User decision 2026-11-26 ("按此实现"): a lower carrying
            # several measured uppers gets one support body per upper (keyed by
            # lower index and upper id); the planner keeps, per target, only the
            # target's own body under it (shared_lower_world).
            if (lower['detection_index'],upper['id']) in bodies:continue
            lower_mask=np.asarray(result.masks[lower['detection_index']],bool)&~upper_mask
            interface=int((lower_mask&band&valid&under&(distance>=-SUPPORT_BELOW_M)&
                           (distance<=SUPPORT_ABOVE_M+SUPPORT_BELOW_M)).sum())
            if interface<SUPPORT_MIN_POINTS:continue
            face=(lower.get('evidence') or {}).get('observed_face')
            if not face:note('LOWER_WITHOUT_OBSERVED_FACE_REMAINS_NEIGHBOR',lower,interface_points=interface);continue
            front=np.asarray(face['normal'],float)
            # The complete possible body extends inward of the lower's front
            # face to the workspace limits. A face that is not upright (e.g. the
            # lower's top seen from above) cannot bound it away from the robot
            # (replay 8af1e4ef: a top face gave a body reaching base_link): the
            # front bound is then the vertical plane, normal to the upper's face
            # direction, through the lower's frontmost observed return (a carton
            # does not reach toward the camera past its visible front edge).
            upright=abs(normal@front)<=np.sin(np.deg2rad(10))
            front_bound=dict(kind='LOWER_OBSERVED_FRONT_FACE_PLANE')
            on_face=abs((world_points-np.asarray(face['center_m']))@front)<=TARGET_PLANE_BAND_M
            lower_face=lower_mask&valid&on_face;lower_estimates=dict(estimates)
            if int(lower_face.sum())>=SUPPORT_MIN_POINTS:
                lower_estimates['lower_visible_top']=float(np.percentile(world_points[lower_face]@normal,LOWER_TOP_QUANTILE))
            if lower_estimates.get('lower_visible_top',-np.inf)-measured_bottom>SUPPORT_ABOVE_M:
                note('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',lower,estimates=lower_estimates);continue
            support=np.asarray(face['center_m'])+np.asarray(face['support_points_uv_m'])@np.asarray([face['u'],face['v']])
            signed=support@normal-offset
            # An upright lower face must reach below the plane. A lower seen
            # only from above is its top, i.e. the support surface itself (field
            # 9707b9cc, 2 mm above the conservative plane); its conflict test is
            # the lower-visible-top check above.
            if upright and signed.min()>=0:
                note('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',lower,min_below_plane_m=float(signed.min()));continue
            d2=world_points@normal-offset
            joint=lower_mask&band&valid&under&(d2>=-SUPPORT_BELOW_M)&(d2<=SUPPORT_ABOVE_M+measured_bottom-offset)
            if int(joint.sum())<SUPPORT_MIN_POINTS:
                note('NO_SAME_FRAME_SUPPORT_INTERFACE',lower,interface_points=int(joint.sum()));continue
            lo,hi=np.asarray(site['workspace_bounds_m'],float)
            A=list(np.eye(3))+list(-np.eye(3));b=list(hi)+list(-lo)
            if upright:
                A+=[front,normal];b+=[float(front@np.asarray(face['center_m'])+TARGET_PLANE_BAND_M),offset]
            else:
                toward=R[:,2]-normal*(normal@R[:,2])
                if np.linalg.norm(toward)<.5:
                    note('LOWER_FACE_NOT_COMPATIBLE_WITH_STACK_SUPPORT_PLANE',lower,interface_points=interface);continue
                toward/=np.linalg.norm(toward)
                returns=world_points[np.asarray(result.masks[lower['detection_index']],bool)&valid]
                limit=float((returns@toward).max())+TARGET_PLANE_BAND_M
                A+=[toward,normal,front];b+=[limit,offset,float(front@np.asarray(face['center_m'])+TARGET_PLANE_BAND_M)]
                front_bound=dict(kind='FRONTMOST_OBSERVED_LOWER_RETURN_ALONG_UPPER_FACE_DIRECTION',normal_world=toward.tolist(),
                                 offset_m=limit,lower_face_plane_also_bounds=True)
            try:vertices=intersect_vertices(A,b)
            except ValueError as error:note(str(error),lower);continue
            x=(front if upright else toward);x=x-normal*(normal@x);x/=np.linalg.norm(x)
            frame=np.column_stack((x,np.cross(normal,x),normal));v=vertices@frame
            vlow,vhigh=v.min(0),v.max(0);vhigh[2]=offset
            pose=np.eye(4);pose[:3,:3]=frame;pose[:3,3]=frame@((vlow+vhigh)/2)
            body=dict(id=f"rgbd_resting_support_{lower['detection_index']}_for_{upper['id']}",
                category='confirmed_stack_support',pose=pose.tolist(),size=(vhigh-vlow).tolist(),
                supports_target_id=upper['id'],detection_index=lower['detection_index'],geometry_uncertain=True,
                support_plane=dict(normal_world=normal.tolist(),offset_m=offset),
                support_basis=setting['basis'],observation_id=observation.frame.observation_id)
            record=dict(status='APPLIED',rule='GENERAL_RESTING_CONTACT',upper_id=upper['id'],lower_index=lower['detection_index'],
                object_id=body['id'],basis=setting['basis'],plane=body['support_plane'],
                plane_source='UPPER_VISIBLE_FACE_BOTTOM_DIRECT_BOUND_OR_SEAM_GENERAL_RESTING_CONTACT',
                seam_band_ownership=dict(rule='RETURNS_INSIDE_UPPER_ENVELOPE_BETWEEN_PLANE_AND_UPPER_MEASURED_BOTTOM_ARE_UPPER',
                    basis=SEAM_BAND_OWNERSHIP_BASIS,upper_index=upper['detection_index'],lower_index=lower['detection_index'],
                    plane_m=offset,upper_measured_bottom_m=measured_bottom,band_top_m=measured_bottom+SUPPORT_ABOVE_M,
                    normal_world=normal.tolist(),image_band_px=TARGET_PIXEL_BAND_PX),
                estimates=lower_estimates,selected_m=offset,selected_from=sorted(selected),upper_visible_face_pad_m=face_pad,
                spread_m=float(max(lower_estimates.values())-min(lower_estimates.values())),spread_limit_m=None,
                interface_points=int(joint.sum()),interface_gap_m=[float(-d2[joint].max()),float(-d2[joint].min())],
                lower_front_max_above_plane_m=float(signed.max()),instances=counting,
                hidden_depth_measured=False,full_dimensions_m=None,carton_end_at_workspace_boundary_assumed=False,
                complete_static_domain_vertices=vertices.tolist(),support_penetration_limit_m=.0004,front_bound=front_bound,
                contact_permission_scope='THIS_TARGET_PAYLOAD_DURING_DETACH_ONLY_NO_ROBOT_OR_TOOL_EXCEPTION')
            chosen.append((lower,body,record,lower_mask))
        if not chosen:
            note('NO_UNRESOLVED_INSTANCE_UNDER_UPPER_OBSERVED_SUPPORT_RULE_APPLIES');continue
        if kind!='MEASURED_BOUNDARY_INTERVALS':
            for lower,body,record,_ in chosen:bodies[(lower['detection_index'],upper['id'])]=(body,record);records.append(record)
            continue
        try:clip_upper_bottom(upper,offset,dict(object_ids=[c[1]['id'] for c in chosen],estimates=estimates,
                                                rule='GENERAL_RESTING_CONTACT'))
        except ValueError as error:
            for lower,_,_,_ in chosen:note(str(error),lower)
            continue
        lowers_mask=np.zeros(valid.shape,bool)
        for _,_,_,m in chosen:lowers_mask|=m
        local=(world_points-T[:3,3])@R;lateral=[i for i in (0,1) if i!=axis]
        u=upper['uncertainty'];lo=np.asarray(u['lower_m']);hi=np.asarray(u['upper_m'])
        height=world_points@normal
        owned=(valid&band&(upper_mask|lowers_mask|~labelled)&(height>=offset)&(height<=measured_bottom+SUPPORT_ABOVE_M)&
               np.all((local[...,lateral]>=lo[lateral])&(local[...,lateral]<=hi[lateral]),axis=-1)&(local[...,2]>=lo[2]))
        extension=contain_seam_band_front(upper,world_points,owned,dict(object_ids=[c[1]['id'] for c in chosen]))
        for lower,body,record,_ in chosen:
            record['upper_envelope_bottom_set_to_plane']=True;record['seam_band_front_extension']=extension
            bodies[(lower['detection_index'],upper['id'])]=(body,record);records.append(record)
    return bodies,records


def lower_indices(bodies):
    """Detection indices of the lowers represented by support bodies (any key form)."""
    return {body['detection_index'] for body,_ in bodies.values()}


def shared_lower_world(world,target,boxes,planes,setting=None):
    """The planning world of ``target`` where a lower carries several measured uppers.

    User decision 2026-11-26 ("按此实现"): every upper resting on the same
    lower has its own support body (and interface / beside pieces). While
    ``target`` is searched, its own body is its support plane (unchanged
    rule); a body or piece of the same lower bound to another measured upper
    B is kept only on B's side - of the target's adjacent contact plane facing
    B when they touch, otherwise of the target envelope's lateral side facing
    B - and never reaches under the target. Where it reaches that boundary it
    is the target's adjacent contact there, exactly like a touching
    neighbour (2026-11-24 rule, adjacent_contact._clip_to_plane): only the
    attached payload, only at detach, may touch it, no corner more than
    0.4 mm past the plane; robot and tool keep the full clearance. Farther
    pieces stay ordinary obstacles. Returns (world, records); the input world
    itself when nothing applies.
    """
    from .adjacent_contact import _vertices,_sides,_clip_to_plane
    own={o['detection_index'] for o in world if o.get('category')=='confirmed_stack_support'
         and o.get('supports_target_id')==target['id'] and 'detection_index' in o}
    others={b['id']:b for b in boxes if b['id']!=target['id']}
    if not own or not others:return world,[]
    T=rigid(target['pose']);half=np.asarray(target['size_m'],float)/2;centre=T[:3,3]
    vertical,sides=_sides(T);axis=sides[0][0]
    mine=[p for p in planes or () if p['target_id']==target['id']]
    basis='User decision 2026-11-26 "按此实现" (shared lower carton)'
    contact=dict(setting or {},basis=(setting or {}).get('basis') or basis)
    def boundary(upper):
        c=np.asarray(upper['corners_m'],float).mean(0)
        for plane in mine:
            if float(np.asarray(plane['normal_world'],float)@c)<plane['offset_m']:return plane,'ADJACENT_CONTACT_PLANE'
        sign=1. if float(T[:3,axis]@(c-centre))>=0 else -1.
        n=-sign*T[:3,axis];point=centre+T[:3,axis]*sign*half[axis]
        return dict(target_id=target['id'],axis=axis,side=int(sign),normal_world=n.tolist(),offset_m=float(n@point)),'TARGET_ENVELOPE_LATERAL_SIDE'
    out=[];records=[]
    for o in world:
        upper=others.get(o.get('supports_target_id'))
        if (o.get('category')!='confirmed_stack_support' or upper is None or o.get('detection_index') not in own):
            out.append(o);continue
        plane,source=boundary(upper);n=np.asarray(plane['normal_world'],float);d=float(plane['offset_m'])
        v=_vertices(o)
        record=dict(object_id=o['id'],lower_index=o['detection_index'],bound_to=upper['id'],target=target['id'],
                    boundary=source,normal_world=n.tolist(),offset_m=d,beyond_boundary_m=float((v@n).max())-d)
        if float((v@n).min())>=d:
            records.append(dict(record,result='REMOVED_ENTIRELY_ON_TARGET_SIDE'));continue
        clipped=_clip_to_plane(o,plane,target,o['id']+'_adj_for_'+target['id'],contact)
        if clipped is None:
            # Farther than the band on the other upper's side, or not beside
            # the target envelope on the other two axes: an ordinary obstacle.
            out.append(o);continue
        obj=dict(clipped[0],detection_index=o['detection_index'],shared_lower_clip=dict(record,basis=basis))
        out.append(obj);records.append(dict(record,result='ADJACENT_CONTACT_ON_OTHER_UPPER_SIDE',object=obj['id'],
                                            clipped_m=clipped[1]['clipped_m'],extended_m=clipped[1]['extended_m']))
    if not records:return world,[]
    return out,records
