"""Source-bound soft link-direction preference, plus the optional upper-arm
branch gate (user decision 2026-11-25), which only removes approach IK
branches before they are tried and never rejects a planned state."""
import math
import numpy as np
from unloading_sim.robot import _axis_angle_transform


WRIST_CLUSTERS=('ANY','UP','DOWN')
# Implementer's values (2026-11-28): a wrist link within 10 deg of horizontal
# is NEUTRAL (both clusters); a wrist centre closer than 5 cm to the J1 axis
# has no defined facing along a path (the direction flips for any motion).
WRIST_NEUTRAL_DEG=10.
FACING_MIN_RADIUS_M=.05


def settings(config):
    value=config.get('posture_preference')
    if value is None:return None
    keys={'mode','upper_up_weight','forearm_horizontal_weight','joint_travel_weight','basis'}
    # Optional (user decision 2026-11-21): soft preference for the wrist link
    # J4->J5 (link_4 -> link_5 origins) pointing to base -Z. Absent keeps the
    # original score exactly.
    # Optional (user decision 2026-11-25, "近似即可，不要写死成固定的值"):
    # tolerance_deg = {upper_up, forearm_horizontal[, wrist_down]}: a link
    # direction within its tolerance of the preferred direction costs nothing;
    # only the excess angle is scored. Absent keeps the original score exactly.
    # Optional (user decision 2026-11-25, "降为次要"): wrist_down_priority=
    # SECONDARY orders IK branches by the arm score (upper arm, forearm, joint
    # travel) first; the J4->J5 term only orders branches whose arm scores lie
    # within wrist_down_secondary_band of the best of their group. Absent or
    # WEIGHTED keeps the summed score.
    # Optional (user decision 2026-11-25, "加上朝向偏好"): arm_facing=
    # FRONT_FIRST puts every IK branch whose wrist centre lies behind the arm
    # (more than 90 deg from the shoulder-offset side, i.e. reaching over its
    # back) after all front-facing branches; order inside each class unchanged.
    # Optional (user decision 2026-11-25, "大臂都不是竖直，这个不好，不好的逆解为什么
    # 要尝试"; threshold chosen "偏竖直 >45° 不尝试"): upper_arm_max_from_vertical_deg
    # removes approach IK branches whose upper arm (joint_2 -> joint_3 origins)
    # leans more than this from base +Z before any of them is checked.
    # Optional (user question 2026-11-28 "是腕部向上的姿态簇好，还是腕部向下的姿态簇
    # 好？"): wrist_cluster = ANY | UP | DOWN keeps only approach IK branches
    # whose wrist link J4->J5 (link_4 -> link_5 origins) points up / down
    # (|z| >= sin(WRIST_NEUTRAL_DEG); a near-horizontal wrist link, e.g. a
    # tool pointing down, belongs to both). Absent or ANY removes nothing.
    optional={'wrist_down_weight','tolerance_deg','wrist_down_priority','wrist_down_secondary_band','arm_facing',
              'upper_arm_max_from_vertical_deg','wrist_cluster'}
    weights=['upper_up_weight','forearm_horizontal_weight','joint_travel_weight']+[k for k in ('wrist_down_weight',) if isinstance(value,dict) and k in value]
    if (not isinstance(value,dict) or not keys<=set(value)<=keys|optional or value['mode']!='SOFT_LINK_DIRECTIONS'
            or not isinstance(value['basis'],str) or not value['basis'].strip()
            or any(type(value[k]) not in (int,float) or not math.isfinite(value[k]) or not 0<value[k]<=100
                   for k in weights)):
        raise ValueError('POSTURE_PREFERENCE_INVALID')
    if 'tolerance_deg' in value:
        tolerance=value['tolerance_deg'];allowed={'upper_up','forearm_horizontal'}|({'wrist_down'} if 'wrist_down_weight' in value else set())
        if (not isinstance(tolerance,dict) or not tolerance or not set(tolerance)<=allowed
                or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=60 for v in tolerance.values())):
            raise ValueError('POSTURE_PREFERENCE_TOLERANCE_INVALID')
    priority=value.get('wrist_down_priority','WEIGHTED')
    if priority not in ('WEIGHTED','SECONDARY') or (priority=='SECONDARY' and 'wrist_down_weight' not in value):
        raise ValueError('POSTURE_PREFERENCE_WRIST_PRIORITY_INVALID')
    if 'wrist_down_secondary_band' in value:
        band=value['wrist_down_secondary_band']
        if priority!='SECONDARY' or type(band) not in (int,float) or not math.isfinite(band) or not 0<=band<=1:
            raise ValueError('POSTURE_PREFERENCE_WRIST_BAND_INVALID')
    # FRONT_ONLY (user decision 2026-11-26, "取消掉朝后分支"): reach-over
    # approach branches are removed before any is checked (front_gate).
    if value.get('arm_facing','ANY') not in ('ANY','FRONT_FIRST','FRONT_ONLY'):
        raise ValueError('POSTURE_PREFERENCE_ARM_FACING_INVALID')
    if value.get('wrist_cluster','ANY') not in WRIST_CLUSTERS:
        raise ValueError('POSTURE_PREFERENCE_WRIST_CLUSTER_INVALID')
    if 'upper_arm_max_from_vertical_deg' in value:
        gate=value['upper_arm_max_from_vertical_deg']
        if type(gate) not in (int,float) or not math.isfinite(gate) or not 0<gate<=180:
            raise ValueError('POSTURE_PREFERENCE_UPPER_ARM_GATE_INVALID')
    return dict(value)


def upper_arm_gate(policy,solutions,reference):
    """(kept, removed) by upper_arm_max_from_vertical_deg; all kept when unset."""
    if policy is None or 'upper_arm_max_from_vertical_deg' not in policy:return list(solutions),[]
    limit=float(policy['upper_arm_max_from_vertical_deg']);kept=[];removed=[]
    for x in solutions:
        angle=metrics(policy,x,reference)['upper_from_up_deg']
        if angle>limit+1e-9:removed.append(dict(q=list(x),upper_from_up_deg=angle))
        else:kept.append(x)
    return kept,removed


def facing(policy):
    return policy is not None and policy.get('arm_facing') in ('FRONT_FIRST','FRONT_ONLY')


def front_only(policy):
    return policy is not None and policy.get('arm_facing')=='FRONT_ONLY'


def front_gate(policy,solutions,reference):
    """(kept, removed) by arm_facing=FRONT_ONLY; all kept otherwise."""
    if not front_only(policy):return list(solutions),[]
    kept=[];removed=[]
    for x in solutions:
        m=metrics(policy,x,reference)
        if m['arm_faces_front']:kept.append(x)
        else:removed.append(dict(q=list(x),arm_facing_deg=m['arm_facing_deg']))
    return kept,removed


def wrist_cluster(policy):
    return 'ANY' if policy is None else policy.get('wrist_cluster','ANY')


def wrist_gate(policy,solutions,reference):
    """(kept, removed) by wrist_cluster UP / DOWN; all kept for ANY."""
    wanted=wrist_cluster(policy)
    if wanted=='ANY':return list(solutions),[]
    kept=[];removed=[]
    for x in solutions:
        m=metrics(policy,x,reference)
        if m['wrist_cluster'] in (wanted,'NEUTRAL'):kept.append(x)
        else:removed.append(dict(q=list(x),wrist_cluster=m['wrist_cluster'],wrist_link_z=m['wrist_link_z']))
    return kept,removed


def goal_violation(policy,q,reference):
    """The approach-branch gates (upper arm, FRONT_ONLY, wrist cluster) applied to
    one configuration that did not come from the screened IK branches (a pose-
    goal approach, user 2026-11-28 "这种姿态以后不要再规划了"); None when it passes."""
    if policy is None:return None
    for gate,name in ((upper_arm_gate,'UPPER_ARM_GATE'),(front_gate,'ARM_FACING_GATE'),(wrist_gate,'WRIST_CLUSTER_GATE')):
        _,removed=gate(policy,[list(q)],reference)
        if removed:return dict(gate=name,**removed[0])
    return None


def _batch_transforms(policy,states):
    """Base transforms of every chain link for many configurations (N x 4 x 4 each)."""
    Q=np.asarray(states,float).reshape(-1,6);n=len(Q);T=np.broadcast_to(np.asarray(policy['base'],float),(n,4,4)).copy();out={}
    for i,j in enumerate(policy['chain']):
        k=np.asarray(j['axis'],float);k=k/np.linalg.norm(k);K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
        s=np.sin(Q[:,i])[:,None,None];c=np.cos(Q[:,i])[:,None,None]
        R=np.zeros((n,4,4));R[:,:3,:3]=np.eye(3)+s*K+(1-c)*(K@K);R[:,3,3]=1.
        T=T@np.asarray(j['origin'],float)@R;out[j['child']]=T.copy()
    return out


def reach_over_violation(policy,states):
    """arm_facing=FRONT_ONLY along a path (user 2026-11-28, the C5 "仰泳"
    plan reached over the arm's back through a pose-goal approach): the first
    state whose wrist centre lies more than 90 deg from the arm front, or None.
    The same facing as metrics(); states whose wrist centre is within
    FACING_MIN_RADIUS_M of the J1 axis have no defined facing and pass."""
    if not front_only(policy):return None
    Q=np.asarray(states,float).reshape(-1,6)
    if not len(Q):return None
    frames=_batch_transforms(policy,Q);chain=policy['chain']
    link1=frames[chain[0]['child']];shoulder=(link1@np.asarray(chain[1]['origin'],float))[:,:2,3]
    front=shoulder-link1[:,:2,3];wrist=frames[chain[4]['child']][:,:2,3]-link1[:,:2,3]
    nf=np.linalg.norm(front,axis=1);nw=np.linalg.norm(wrist,axis=1)
    cos=np.einsum('ij,ij->i',front,wrist)/np.maximum(nf*nw,1e-12)
    angle=np.degrees(np.arccos(np.clip(cos,-1.,1.)));angle[(nf<1e-6)|(nw<FACING_MIN_RADIUS_M)]=0.
    bad=np.flatnonzero(angle>90.)
    if not len(bad):return None
    k=int(bad[0])
    return dict(reason='ARM_FACING_REACH_OVER_STATE',index=k,q=Q[k].tolist(),arm_facing_deg=float(angle[k]),
                max_arm_facing_deg=float(angle.max()),states=len(Q),wrist_radius_m=float(nw[k]))


def secondary(policy):
    return policy is not None and policy.get('wrist_down_priority')=='SECONDARY'


def bind(config,coordinates):
    policy=settings(config)
    if policy is None:return None
    robot=coordinates.robot
    chain=[dict(name=j.name,child=j.child,origin=j.origin.tolist(),axis=j.axis.tolist(),type=j.joint_type)
           for j in robot.joints]
    if ([j['name'] for j in chain]!=[f'joint_{i}' for i in range(1,7)] or
            any(j['type']!='revolute' for j in chain)):
        raise ValueError('POSTURE_REQUIRES_SOURCE_ECO65_CHAIN')
    # Directions are measured in robot base, independently of any saved
    # external world mounting transform; joint origins already use this base.
    bound=dict(policy,chain=chain,base=np.eye(4).tolist(),
               metric_links=['link_2','link_3','link_4'],frame='base_link',hard_constraint=False)
    if 'wrist_down_weight' in policy or policy.get('wrist_cluster','ANY')!='ANY':bound['wrist_links']=['link_4','link_5']
    return bound


def metrics(policy,q,reference):
    t=np.array(policy['base'],float);origins={};frames={}
    for i,j in enumerate(policy['chain']):
        t=t@np.asarray(j['origin'])@_axis_angle_transform(np.asarray(j['axis']),q[i],j['type'])
        origins[j['child']]=t[:3,3].copy();frames[j['child']]=t.copy()
    a,b,c=(origins[k] for k in policy['metric_links'])
    upper=b-a;fore=c-b;upper/=np.linalg.norm(upper);fore/=np.linalg.norm(fore)
    up=float(np.clip(upper[2],-1.,1.));horizontal=float(np.clip(fore[2],-1.,1.))
    travel=float(np.linalg.norm(np.asarray(q)-reference))
    tolerance=policy.get('tolerance_deg')
    upper_deg=float(np.rad2deg(np.arccos(up)));fore_deg=float(np.rad2deg(np.arcsin(abs(horizontal))))
    if tolerance is None:
        score=policy['upper_up_weight']*(1-up)+policy['forearm_horizontal_weight']*horizontal**2+policy['joint_travel_weight']*travel
    else:
        excess=lambda angle,key:np.deg2rad(max(0.,angle-float(tolerance.get(key,0.))))
        score=(policy['upper_up_weight']*(1-math.cos(excess(upper_deg,'upper_up')))
               +policy['forearm_horizontal_weight']*math.sin(excess(fore_deg,'forearm_horizontal'))**2
               +policy['joint_travel_weight']*travel)
    out=dict(upper_from_up_deg=upper_deg,forearm_from_horizontal_deg=fore_deg,raw_joint_travel_rad=travel)
    if 'wrist_links' in policy:
        a,b=(origins[k] for k in policy['wrist_links']);wrist=(b-a)/np.linalg.norm(b-a)
        limit=math.sin(math.radians(WRIST_NEUTRAL_DEG))
        out.update(wrist_link_z=float(wrist[2]),wrist_cluster='UP' if wrist[2]>=limit else 'DOWN' if wrist[2]<=-limit else 'NEUTRAL')
    if 'wrist_down_weight' in policy:
        a,b=(origins[k] for k in policy['wrist_links']);wrist=(b-a)/np.linalg.norm(b-a)
        down=float(np.clip(-wrist[2],-1.,1.))
        wrist_deg=float(np.rad2deg(np.arccos(down)))
        if tolerance is None:wrist_score=policy['wrist_down_weight']*(1-down)
        else:wrist_score=policy['wrist_down_weight']*(1-math.cos(excess(wrist_deg,'wrist_down')))
        out.update(wrist_from_down_deg=wrist_deg,arm_score=score,wrist_score=wrist_score)
        score+=wrist_score
    if facing(policy):
        # Arm front = the horizontal direction from the J1 axis to the joint_2
        # origin (the source URDF shoulder offset, link_1 -X for ECO65); facing
        # = its angle to the horizontal direction from the J1 axis to the wrist
        # centre (link_5 origin).
        link1=frames[policy['chain'][0]['child']]
        shoulder=(link1@np.asarray(policy['chain'][1]['origin']))[:3,3]
        front=shoulder[:2]-link1[:2,3];wrist_c=origins[policy['chain'][4]['child']][:2]-link1[:2,3]
        if np.linalg.norm(front)<1e-6 or np.linalg.norm(wrist_c)<1e-6:angle=0.
        else:angle=float(np.degrees(np.arccos(np.clip(front@wrist_c/np.linalg.norm(front)/np.linalg.norm(wrist_c),-1.,1.))))
        out.update(arm_facing_deg=angle,arm_faces_front=angle<=90.)
    return dict(score=score,**out)


def key(policy,q,reference):
    if policy is None:return (float(np.linalg.norm(np.asarray(q)-reference)),)
    m=metrics(policy,q,reference)
    back=(0 if m['arm_faces_front'] else 1,) if facing(policy) else ()
    if secondary(policy):
        # Arm first, then the J4->J5 term (single solutions; IK branch lists
        # use order(), which also groups arm scores within the band).
        return back+(math.floor(m['arm_score']*1e9+.5),math.floor(m['wrist_score']*1e9+.5),m['raw_joint_travel_rad'])
    # Stable cross-language ordering, retaining raw finite joint windings.
    return back+(math.floor(m['score']*1e9+.5),m['raw_joint_travel_rad'])


def order(policy,solutions,reference):
    """IK branches in preference order.

    WEIGHTED (or no wrist term): sorted by key(). SECONDARY: sorted by the arm
    score; consecutive branches whose arm score is within the band of their
    group's first (best) branch form one group, ordered inside by the J4->J5
    term, then the arm score and raw travel. Deterministic; never a gate.
    """
    if facing(policy):
        # FRONT_FIRST: front-facing branches, then the reach-over ones, each
        # class in the order below (stable; nothing is removed).
        inner=dict(policy,arm_facing='ANY')
        classes=[[],[]]
        for x in solutions:classes[0 if metrics(policy,x,reference)['arm_faces_front'] else 1].append(x)
        return order(inner,classes[0],reference)+order(inner,classes[1],reference)
    if not secondary(policy):
        return sorted(solutions,key=lambda x:key(policy,x,reference))
    band=float(policy.get('wrist_down_secondary_band',.05))
    rows=sorted(((metrics(policy,x,reference),x) for x in solutions),
                key=lambda r:(math.floor(r[0]['arm_score']*1e9+.5),r[0]['raw_joint_travel_rad']))
    out=[];group=[]
    for r in rows:
        if group and r[0]['arm_score']-group[0][0]['arm_score']>band+1e-12:
            out+=sorted(group,key=lambda g:(math.floor(g[0]['wrist_score']*1e9+.5),math.floor(g[0]['arm_score']*1e9+.5),g[0]['raw_joint_travel_rad']))
            group=[]
        group.append(r)
    out+=sorted(group,key=lambda g:(math.floor(g[0]['wrist_score']*1e9+.5),math.floor(g[0]['arm_score']*1e9+.5),g[0]['raw_joint_travel_rad']))
    return [x for _,x in out]
