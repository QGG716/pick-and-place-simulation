"""Source-bound soft link-direction preference; never a state rejection gate."""
import math
import numpy as np
from unloading_sim.robot import _axis_angle_transform


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
    optional={'wrist_down_weight','tolerance_deg'}
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
    return dict(value)


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
    if 'wrist_down_weight' in policy:bound['wrist_links']=['link_4','link_5']
    return bound


def metrics(policy,q,reference):
    t=np.array(policy['base'],float);origins={}
    for i,j in enumerate(policy['chain']):
        t=t@np.asarray(j['origin'])@_axis_angle_transform(np.asarray(j['axis']),q[i],j['type'])
        origins[j['child']]=t[:3,3].copy()
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
    if 'wrist_down_weight' in policy:
        a,b=(origins[k] for k in policy['wrist_links']);wrist=(b-a)/np.linalg.norm(b-a)
        down=float(np.clip(-wrist[2],-1.,1.))
        wrist_deg=float(np.rad2deg(np.arccos(down)))
        if tolerance is None:score+=policy['wrist_down_weight']*(1-down)
        else:score+=policy['wrist_down_weight']*(1-math.cos(excess(wrist_deg,'wrist_down')))
        out['wrist_from_down_deg']=wrist_deg
    return dict(score=score,**out)


def key(policy,q,reference):
    if policy is None:return (float(np.linalg.norm(np.asarray(q)-reference)),)
    m=metrics(policy,q,reference)
    # Stable cross-language ordering, retaining raw finite joint windings.
    return (math.floor(m['score']*1e9+.5),m['raw_joint_travel_rad'])
