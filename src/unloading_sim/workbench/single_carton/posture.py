"""Source-bound soft link-direction preference; never a state rejection gate."""
import math
import numpy as np
from unloading_sim.robot import _axis_angle_transform


def settings(config):
    value=config.get('posture_preference')
    if value is None:return None
    keys={'mode','upper_up_weight','forearm_horizontal_weight','joint_travel_weight','basis'}
    if (not isinstance(value,dict) or set(value)!=keys or value['mode']!='SOFT_LINK_DIRECTIONS'
            or not isinstance(value['basis'],str) or not value['basis'].strip()
            or any(type(value[k]) not in (int,float) or not math.isfinite(value[k]) or not 0<value[k]<=100
                   for k in ('upper_up_weight','forearm_horizontal_weight','joint_travel_weight'))):
        raise ValueError('POSTURE_PREFERENCE_INVALID')
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
    return dict(policy,chain=chain,base=np.eye(4).tolist(),
                metric_links=['link_2','link_3','link_4'],frame='base_link',hard_constraint=False)


def metrics(policy,q,reference):
    t=np.array(policy['base'],float);origins={}
    for i,j in enumerate(policy['chain']):
        t=t@np.asarray(j['origin'])@_axis_angle_transform(np.asarray(j['axis']),q[i],j['type'])
        origins[j['child']]=t[:3,3].copy()
    a,b,c=(origins[k] for k in policy['metric_links'])
    upper=b-a;fore=c-b;upper/=np.linalg.norm(upper);fore/=np.linalg.norm(fore)
    up=float(np.clip(upper[2],-1.,1.));horizontal=float(np.clip(fore[2],-1.,1.))
    travel=float(np.linalg.norm(np.asarray(q)-reference))
    score=policy['upper_up_weight']*(1-up)+policy['forearm_horizontal_weight']*horizontal**2+policy['joint_travel_weight']*travel
    return dict(score=score,upper_from_up_deg=float(np.rad2deg(np.arccos(up))),
                forearm_from_horizontal_deg=float(np.rad2deg(np.arcsin(abs(horizontal)))),raw_joint_travel_rad=travel)


def key(policy,q,reference):
    if policy is None:return (float(np.linalg.norm(np.asarray(q)-reference)),)
    m=metrics(policy,q,reference)
    # Stable cross-language ordering, retaining raw finite joint windings.
    return (math.floor(m['score']*1e9+.5),m['raw_joint_travel_rad'])
