"""Geometry-first ECO65 process templates; no IK or synthetic robot paths.

Historical v2 retains its frozen-bottom gate. Prototype v3 follows the latest
user requirement: top suction has no height gate, but still requires exposure,
actual full seals and a native complete cycle. All equipment/cup geometry and
native q remain ECO65 data. Process heights are distinct from hardware travel.
"""
from collections import deque
from copy import deepcopy
import numpy as np
from .model import transform, check_transform
from ..moveit2_backend import digest

POLICY = dict(schema='eco65_transverse_templates_v2',
    reference_sha='eb30f3e0a4bd0d957b02aec3f66c39fb9a8f28d1',
    receiver='conveyor_transverse', top_bottom_threshold_m=.100,
    desired_drop_m=.050, joint_policy='RAW_FINITE_JOINTS_BIDIRECTIONAL_NO_J1_MONOTONICITY_V4',
    orientation='FROZEN_CARTON_AND_TOOL_SPATIAL_ORIENTATION',
    synchronization='BEFORE_APPROACH_OR_PREGRASP_OR_CONTACT_BEFORE_ATTACH',
    cup_selection='MAXIMIZE_ACTUAL_FULL_SEALS_WITHIN_TEMPLATE_INACTIVE_CUPS_MAY_MISS',
    release='SAME_POSE_NOMINAL_DROP_CHECKED_RETREAT_IDEAL_DRAIN')

PROTOTYPE_POLICY = dict(POLICY, schema='eco65_quarter_transverse_templates_v3',
    reference_sha='4d191ecab065420e4b70c9b266c87654996bd91f',
    top_bottom_threshold_m=None, top_permission='EXPOSURE_SEAL_NATIVE_COLLISION_COMPLETE_CYCLE',
    desired_drop_m=.0125, release='SAME_POSE_NOMINAL_DROP_CHECKED_RETREAT_CHECKED_IDEAL_OUTFEED')


def policy_for(scene):
    policy=scene.get('transverse_process_policy')
    if policy not in (POLICY,PROTOTYPE_POLICY):raise ValueError('TRANSVERSE_PROCESS_POLICY_UNKNOWN')
    return policy


def pose_of(box):
    value=np.asarray(box['pose'],float)
    return transform(value) if value.shape==(3,) else check_transform(value)


def corners(box, pose=None):
    p=pose_of(box) if pose is None else check_transform(pose)
    xyz=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])
    return (xyz*np.asarray(box['size_m']))@p[:3,:3].T+p[:3,3]


def freeze_box(box, original_boxes, floor_z=0., policy=POLICY):
    vertices=corners(box);lo,hi=vertices.min(0),vertices.max(0)
    # Geometry identifies the original layer, never current exposed endpoints.
    peers=[b for b in original_boxes if abs(corners(b)[:,2].min()-lo[2])<min(hi[2]-lo[2],b['size_m'][2])*.25]
    ys=[pose_of(b)[1,3] for b in peers]
    y=pose_of(box)[1,3]
    role=('SINGLE' if len(ys)==1 else 'LEFT_END' if abs(y-max(ys))<1e-8
          else 'RIGHT_END' if abs(y-min(ys))<1e-8 else 'MIDDLE')
    return dict(target_id=box['id'],original_pose=pose_of(box).tolist(),
        original_role=role,original_layer_ids=[b['id'] for b in peers],
        original_box_digest=digest(box),floor_z_m=float(floor_z),
        z_b_m=float(lo[2]-floor_z),bottom_world_m=float(lo[2]),vertical_height_m=float(hi[2]-lo[2]),
        top_allowed=bool(policy['top_bottom_threshold_m'] is None or lo[2]-floor_z < policy['top_bottom_threshold_m']),
        top_permission=policy.get('top_permission','HISTORICAL_FROZEN_BOTTOM_GATE'))


def projection_limits(box, receiver):
    p=pose_of(box);extent=np.abs(p[:3,:3])@np.asarray(box['size_m'])/2
    lo,hi=np.asarray(receiver['bounds_D_m']);m=receiver['margin_m']
    return lo[:2]+extent[:2]+m,hi[:2]-extent[:2]-m


def junction_projection(box, pose, spec, receiver):
    """Full projected OBB versus the seam inlet corridor (reference SAT)."""
    p=check_transform(pose);projected=corners(box,p)[:,:2]
    y=spec['longitudinal_center_y_m'];half=spec.get('longitudinal_width_m',spec['transverse_width_m'])/2
    x0=spec['transverse_xmin_m'];x1=receiver['bounds_D_m'][1][0]
    rectangle=np.array([[x0,y-half],[x1,y-half],[x1,y+half],[x0,y+half]])
    axes=[np.array([1.,0.]),np.array([0.,1.])]
    for v in p[:2,:3].T:
        if np.linalg.norm(v)>1e-12:axes.append(np.array([-v[1],v[0]])/np.linalg.norm(v))
    hit=all(min(max(projected@a),max(rectangle@a))>=max(min(projected@a),min(rectangle@a))-1e-9 for a in axes)
    return dict(junction=bool(hit),region_class='LONGITUDINAL_INLET_JUNCTION' if hit else 'TRANSVERSE',
        influence_polygon_world_xy_m=rectangle.tolist(),box_projection_world_xy_m=projected.tolist())


def channel_blockers(box, end_y, remaining):
    a=corners(box);lo,hi=a.min(0),a.max(0);dy=end_y-pose_of(box)[1,3]
    lo[1]+=min(0.,dy);hi[1]+=max(0.,dy)
    blocked=[]
    for other in remaining:
        if other['id']==box['id']:continue
        v=corners(other)
        if np.all(np.minimum(hi,v.max(0))-np.maximum(lo,v.min(0))>1e-8):blocked.append(other['id'])
    return sorted(blocked)


def face_blockers(box, face, remaining):
    axis,sign={'front':(0,-1),'left':(1,1),'right':(1,-1),'top':(2,1)}[face]
    a=corners(box);lo,hi=a.min(0),a.max(0);others=[i for i in range(3) if i!=axis]
    blocked=[]
    for other in remaining:
        if other['id']==box['id']:continue
        v=corners(other);ol,oh=v.min(0),v.max(0)
        gap=ol[axis]-hi[axis] if sign>0 else lo[axis]-oh[axis]
        # Face exposure is local, not an infinite ray: a distant carton across
        # an opened column does not cover this face. Native tool clearance is
        # still required for the actual approach and every inactive cup.
        ahead=-1e-6<=gap<=.02
        if ahead and np.all(np.minimum(hi[others],oh[others])-np.maximum(lo[others],ol[others])>1e-8):
            blocked.append(other['id'])
    return sorted(blocked)


def process_options(box, original_boxes, remaining, spec, receiver):
    policy=PROTOTYPE_POLICY if spec['config'].get('process_policy_schema')==PROTOTYPE_POLICY['schema'] else POLICY
    frozen=freeze_box(box,original_boxes,spec['config']['floor_z_m'],policy)
    p=pose_of(box);left,right=projection_limits(box,receiver)
    rejection=[];options=[];d=policy['desired_drop_m'];h=frozen['vertical_height_m']
    heights={'HIGH':frozen['bottom_world_m']-d,'LOW':frozen['bottom_world_m']-h-d}
    if np.any(left>right):
        return dict(options=[],rejected=[dict(reason='TRANSVERSE_FULL_PROJECTION_UNAVAILABLE')],frozen=frozen)
    projected=p.copy();projected[0,3]=(left[0]+right[0])/2
    reception=junction_projection(box,projected,spec,receiver)
    def add(name,face,level,legs,*,bottom=None,receiver_height=None):
        height=receiver_height if receiver_height is not None else spec['low_height_m'] if level=='MINIMUM' else heights[level]
        value=dict(template=name,target_id=box['id'],face=face,level=level,height_m=float(height),
            frozen=deepcopy(frozen),segment_directions_world=legs,
            release_bottom_world_m=frozen['bottom_world_m'] if bottom is None else float(bottom),
            center_limits_xy_m=[left.tolist(),right.tolist()],reception=deepcopy(reception),
            requires_open_channel=legs[0][1]!=0,spatial_orientation='HELD',policy=policy['schema'])
        value['expected_drop_m']=value['release_bottom_world_m']-height
        blockers=face_blockers(box,face,remaining)
        reason=('FACE_NOT_EXPOSED' if blockers else
                'PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE' if not spec['low_height_m']-1e-12<=height<=spec['high_height_m']+1e-12 else None)
        if value['requires_open_channel']:
            sign=legs[0][1];turn=np.clip(p[1,3],left[1],right[1])
            if abs(turn-p[1,3])<1e-8:turn=min(right[1],p[1,3]+h) if sign>0 else max(left[1],p[1,3]-h)
            channels=channel_blockers(box,float(turn),remaining)
            value['channel_blockers']=channels
            if channels:reason='LATERAL_CHANNEL_BLOCKED';blockers=channels
            if sign*(turn-p[1,3])<=1e-8:reason='NO_INWARD_LATERAL_SPACE'
        if reason:rejection.append(dict(value,reason=reason,blocker_ids=blockers))
        else:options.append(value)
    if frozen['top_allowed']:
        if left[1]<=p[1,3]<=right[1]:
            # The +Z separation must clear the actual receiver/frame and the
            # frozen box; no after-extraction receiver motion is permitted.
            if policy==PROTOTYPE_POLICY:
                # A separately named lifting process. HIGH/LOW are still
                # rejected outside travel and are never clamped into this.
                receive=max(spec['low_height_m'],min(spec['high_height_m'],frozen['bottom_world_m']))
                bottom=max(frozen['bottom_world_m']+d,receive+d)
                add('TOP_LIFTED_TRANSVERSE_RECEIVER','top','LIFTED',[[0.,0.,1.],[-1.,0.,0.]],
                    bottom=bottom,receiver_height=receive)
            else:
                bottom=max(frozen['bottom_world_m']+d,spec['low_height_m']+d)
                add('TOP_FIXED_MINIMUM_RECEIVER','top','MINIMUM',[[0.,0.,1.],[-1.,0.,0.]],bottom=bottom)
        else:rejection.append(dict(reason='LOW_TOP_FIXED_Y_REGION_UNAVAILABLE',frozen=frozen))
    if left[1]<=p[1,3]<=right[1]:
        if not reception['junction']:add('FRONT_DIRECT_HIGH','front','HIGH',[[-1.,0.,0.]])
        add('FRONT_JUNCTION_LOW' if reception['junction'] else 'FRONT_DIRECT_LOW','front','LOW',[[-1.,0.,0.]])
    else:
        sign=1. if p[1,3]<left[1] else -1.
        add('FRONT_LATERAL_LOW','front','LOW',[[0.,sign,0.],[-1.,0.,0.]])
    if frozen['original_role']=='LEFT_END':add('LEFT_INNER_LOW','right','LOW',[[0.,-1.,0.],[-1.,0.,0.]])
    if frozen['original_role']=='RIGHT_END':add('RIGHT_INNER_HIGH','left','HIGH',[[0.,1.,0.],[-1.,0.,0.]])
    for option in options:option['option_id']=digest(option)
    return dict(options=options,rejected=rejection,frozen=frozen)


def interval_samples(low,high,resolution=.002):
    if low>high:return
    yield float(high)
    if high-low<1e-10:return
    yield float(low)
    queue=deque([(low,high)])
    while queue:
        a,b=queue.popleft()
        if b-a<=resolution:continue
        mid=(a+b)/2;yield float(mid);queue.extend(((a,mid),(mid,b)))


def fair_process_candidates(ranked, limit):
    """Preserve ranking within each legal option, then interleave options.

    A finite cup/IK pool must not consume its entire budget on high reception
    while an already legal low-reception template has never been attempted.
    This changes search order only; it neither invents nor removes options.
    """
    groups={}
    for candidate in ranked:
        groups.setdefault(candidate['process_option']['option_id'],deque()).append(candidate)
    result=[]
    while len(result)<limit and any(groups.values()):
        for group in groups.values():
            if group and len(result)<limit:result.append(group.popleft())
    return result


def release_poses(box,option,remaining,*,resolution=.002):
    p=pose_of(box);lo,hi=np.asarray(option['center_limits_xy_m'])
    ys=[float(p[1,3])]
    if option['requires_open_channel']:
        sign=option['segment_directions_world'][0][1]
        a,b=lo[1],hi[1]
        if sign>0:a=max(a,p[1,3]+resolution)
        else:b=min(b,p[1,3]-resolution)
        ys=[y for y in interval_samples(a,b,resolution) if not channel_blockers(box,y,remaining)]
    xs=list(interval_samples(lo[0],min(hi[0],p[0,3]-resolution),resolution))
    # Fair diagonals over the finite region; no nearest/centre-only shortcut.
    for rank in range(len(xs)+len(ys)-1):
        for yi in range(max(0,rank-len(xs)+1),min(rank,len(ys)-1)+1):
            pose=p.copy();pose[:2,3]=[xs[rank-yi],ys[yi]]
            pose[2,3]+=option['release_bottom_world_m']-corners(box,p)[:,2].min()
            # Occupied landing footprints cannot borrow adjacent belt area.
            projected=corners(box,pose);a,b=projected.min(0),projected.max(0)
            occupied=False
            for other in remaining:
                if other['id']==box['id']:continue
                v=corners(other)
                xy=np.minimum(b[:2],v.max(0)[:2])-np.maximum(a[:2],v.min(0)[:2])
                if np.all(xy>1e-8) and v.min(0)[2]<b[2] and v.max(0)[2]>option['height_m']+1e-8:occupied=True;break
            if not occupied:yield pose
