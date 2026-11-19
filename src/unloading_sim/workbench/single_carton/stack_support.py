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


STACK_INSTANCE_DUPLICATE_IOU=.95   # same rule as recover_boxes deduplication
STACK_INSTANCE_NESTED=.95          # same rule as nested-instance fitting
LOWER_TOP_QUANTILE=99.5


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
        previous_outer_m=previous,outer_m=c,rule='ENVELOPE_BOTTOM_AT_LOWEST_SUPPORT_ESTIMATE')


def infer_supports(observation,result,boxes,unresolved,site,geometry,world_points,valid,outside_ignored=()):
    """Return source-bound support bodies and explicit rejected associations.

    The upper envelope may be a MEASURED_BOUNDARY_INTERVALS envelope; its
    bottom is then set (in place) to the support plane, which is the LOWEST of
    the upper measured bottom, the lower visible-face robust top (pixels of
    the upper mask excluded) and, where the seam policy bounded the bottom
    edge, the seam line minus its uncertainty. Lowering can only enlarge the
    payload; the volume removed is below that plane.
    """
    from .site_scene import TARGET_PIXEL_BAND_PX,TARGET_PLANE_BAND_M,SUPPORT_MIN_POINTS,SUPPORT_BELOW_M,SUPPORT_ABOVE_M,NEAR_TARGET_M
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
        (upper.get('quality') or {}).get('all_dimensions_source')!='RGBD_VISIBLE_BOUNDARIES'):
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
    footprint=np.all(abs(local[...,planar])<=half[planar]+NEAR_TARGET_M,axis=-1)
    measured_bottom=float(normal@T[:3,3]-half[axis]);distance=world_points@normal-measured_bottom
    # Same column: the lower is the one unresolved instance with a same-frame
    # interface at the upper's bottom; other instances (elsewhere in view)
    # neither block nor join the confirmed two-carton relation.
    interfaces={}
    for u in unresolved:
        m=np.asarray(result.masks[u['detection_index']],bool)
        interfaces[u['detection_index']]=int((m&band&valid&footprint&(distance>=-SUPPORT_BELOW_M)&
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
    estimates=dict(upper_measured_bottom=float(normal@T[:3,3]-half[axis]))
    front=np.asarray(face['normal']);on_face=abs((world_points-np.asarray(face['center_m']))@front)<=TARGET_PLANE_BAND_M
    lower_face=lower_mask&~upper_mask&valid&on_face
    if int(lower_face.sum())>=SUPPORT_MIN_POINTS:
        estimates['lower_visible_top']=float(np.percentile(world_points[lower_face]@normal,LOWER_TOP_QUANTILE))
    for interval in (upper.get('uncertainty') or {}).get('intervals',[]):
        if (interval['axis']==axis and interval['side']==side and
                interval['method']=='COPLANAR_SEAM_MASK_RGB_EDGE_POLICY'):
            estimates['seam_line_minus_uncertainty']=float(normal@(T[:3,3]+R[:,axis]*interval['outer_m']))
    offset=min(estimates.values())
    record_estimates=dict(estimates=estimates,selected_m=offset,spread_m=float(max(estimates.values())-offset))
    if estimates.get('lower_visible_top',-np.inf)-estimates['upper_measured_bottom']>SUPPORT_ABOVE_M:
        return reject('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',**record_estimates)
    if record_estimates['spread_m']>SUPPORT_BELOW_M:
        return reject('STACK_SUPPORT_HEIGHT_ESTIMATES_DISAGREE',**record_estimates)
    support=np.asarray(face['center_m'])+np.asarray(face['support_points_uv_m'])@np.asarray([face['u'],face['v']])
    signed=support@normal-offset
    if signed.min()>=0:
        return reject('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',min_below_plane_m=float(signed.min()),**record_estimates)
    distance=world_points@normal-offset
    joint=lower_mask&band&valid&footprint&(distance>=-SUPPORT_BELOW_M)&(distance<=SUPPORT_ABOVE_M+record_estimates['spread_m'])
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
        plane_source='LOWEST_OF_UPPER_MEASURED_BOTTOM_LOWER_VISIBLE_TOP_AND_SEAM_PLUS_USER_CONFIRMED_STACK_TOPOLOGY',
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
    return {lower['detection_index']:(body,record)},[record]
