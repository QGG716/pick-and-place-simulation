"""Soft ECO65 shoulder/elbow search preference, measured in the workcell frame.

This orders seeds and native IK goals. It is neither a collision exemption nor
a joint/path constraint; a seed is never a solved endpoint or a robot motion.
"""
import numpy as np

POLICY = dict(schema='eco65_upright_open_ik_preference_v1',mode='SOFT_SEARCH_PRIORITY',
    shoulder_link='link_2',elbow_link='link_3',wrist_link='link_6',
    preferred_upper_arm_cone_deg=60.,
    forward_reference='SHOULDER_TO_FROZEN_STACK_CENTER_XY',
    goal_order='POSTURE_BAND_THEN_RAW_JOINT_CONTINUITY',
    candidates_removed=False,path_constraint=False,collision_acceptance_changed=False)


def posture_metrics(robot, q, stack_center):
    frames=robot.named_link_frames(np.asarray(q,float))
    shoulder=frames['link_2'][:3,3];elbow=frames['link_3'][:3,3]
    wrist=frames['link_6'][:3,3]
    tcp=(frames['link_6']@robot.tip_from_tcp)[:3,3]
    upper=elbow-shoulder;length=float(np.linalg.norm(upper))
    toward=np.asarray(stack_center,float)[:2]-shoulder[:2]
    norm=float(np.linalg.norm(toward))
    if length<1e-9 or norm<1e-9:raise ValueError('POSTURE_REFERENCE_DEGENERATE')
    toward/=norm
    up_cos=float(np.clip(upper[2]/length,-1.,1.))
    wrist_forward=float((wrist-elbow)[:2]@toward)
    tcp_forward=float((tcp-elbow)[:2]@toward)
    tilt_excess=max(0.,float(np.cos(np.deg2rad(POLICY['preferred_upper_arm_cone_deg'])))-up_cos)
    foldback=max(0.,-min(wrist_forward,tcp_forward))
    penalty=tilt_excess+foldback/length
    return dict(upper_arm_tilt_deg=float(np.rad2deg(np.arccos(up_cos))),
        wrist_forward_of_elbow_m=wrist_forward,tcp_forward_of_elbow_m=tcp_forward,
        foldback_m=foldback,penalty=penalty,preferred=bool(penalty<=1e-9),
        shoulder_D=shoulder.tolist(),elbow_D=elbow.tolist(),wrist_D=wrist.tolist(),tcp_D=tcp.tolist())


def goal_cost(robot, start, goal, stack_center):
    metric=posture_metrics(robot,goal,stack_center)
    delta=np.abs(np.asarray(goal)-np.asarray(start))
    # Within the broad upright/open band, preserve the nearby shoulder/elbow
    # and wrist branch. Raw finite angles are never wrapped or edited.
    return (not metric['preferred'],metric['penalty'],float(delta.sum()),float(delta[0]))


def _upright_seed(seed, limits, shoulder, elbow, wrist=None):
    q=np.asarray(seed,float).copy();pitch=float(q[1:4].sum())
    q[1]=shoulder;q[2]=elbow;q[3]=pitch-shoulder-elbow
    if wrist is not None:q[4]=wrist
    return np.clip(q,limits[:,0],limits[:,1])


def ik_seeds(robot, seed, count, rng_seed, stack_center):
    if count<1:raise ValueError('IK_SEED_COUNT_MUST_BE_POSITIVE')
    q=np.asarray(seed,float).copy()
    # Keep the supplied current/native seed first. Three bounded open-elbow
    # seeds replace part of the old random allocation, not additional budget.
    preferred=[_upright_seed(q,robot.joint_limits,s,e)
               for s,e in ((0.,-1.),(.45,-1.4),(-.35,-.8))][:count-1]
    preferred.sort(key=lambda x:goal_cost(robot,q,x,stack_center))
    rng=np.random.default_rng(rng_seed)
    random=list(rng.uniform(robot.joint_limits[:,0],robot.joint_limits[:,1],(count-1-len(preferred),6)))
    random.sort(key=lambda x:goal_cost(robot,q,x,stack_center))
    return [q,*preferred,*random]


def transit_seeds(robot, q, stack_center):
    # ECO65's zero J2 points its upper arm up in the pinned official model.
    # These are finite IK starts, not archived solutions or a fixed-HOME route.
    sign=1. if q[4]>=0. else -1.
    seeds=[_upright_seed(q,robot.joint_limits,s,e,w*np.pi/2)
           for s,e in ((0.,-1.),(.45,-1.4)) for w in (sign,-sign)]
    return sorted(seeds,key=lambda x:goal_cost(robot,q,x,stack_center))
