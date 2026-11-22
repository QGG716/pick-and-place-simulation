"""Start/release separated single-carton cycle (user decision 2026-11-22).

``cycle_structure`` absent keeps the original cycle: release at the frozen
q_start, segments approach / contact / detach / return (loaded).

``cycle_structure.mode = START_RELEASE_SEPARATE``: the frozen q_start S is the
start of every round; the release pose R is S with J1 offset by
``release_j1_offset_deg`` (user decision 2026-11-21: J1 rotated 180 deg
clockwise, i.e. -180). One round is

    approach (S -> precontact, joint space)
    contact  (LIN)            -> vacuum established
    detach   (LIN)
    transfer (loaded, detach end -> R; raw J1 non-increasing = CLOCKWISE)
    release at R              -> pressure-confirmed release
    withdraw (empty LIN, back off the released carton along the gripped face
              normal by contact_search.precontact_m, the approach standoff)
    return_empty (empty, withdraw end -> S; raw J1 non-decreasing =
              COUNTERCLOCKWISE)

The next round starts from S again. The direction rule is a raw-joint sign
check of every state of the exact sent path (cumulative reverse motion at most
``J1_REVERSE_BUDGET_RAD``), plus a goal sign check before planning; it is not
a MoveIt path constraint and never relaxes a collision rule. The released
carton is kept, conservatively, as a collision object at its release pose for
withdraw and return_empty (see ``released_task``): the withdraw LIN keeps the
native contact-stage rules on it, return_empty has no contact permission. Where it goes after release
is not modelled (receiver qualification stays deferred by the user).
"""
import math
import numpy as np

MODE='START_RELEASE_SEPARATE'
LEGACY_STAGES=('approach','contact','detach','return')
SEPARATE_STAGES=('approach','contact','detach','transfer','withdraw','return_empty')
LEGACY_MOVEMENTS=('APPROACH','CONTACT','DETACH','RETURN_LOADED')
SEPARATE_MOVEMENTS=('APPROACH','CONTACT','DETACH','TRANSFER_LOADED','WITHDRAW_EMPTY','RETURN_EMPTY')
# Raw J1 sign of the user's words, seen from base +Z (2026-11-21: "J1 rotated
# 180 deg clockwise" is J1 - 180 deg).
DIRECTIONS={'CLOCKWISE':-1,'COUNTERCLOCKWISE':1}
# Same cumulative reverse budget as the reference branch's directed-J1 cycle
# (ECO65 cycle policy V2, 2026-10-20); a numerical tolerance, not a motion limit.
J1_REVERSE_BUDGET_RAD=1e-6


def settings(c):
    """Validated ``cycle_structure`` or None (original cycle)."""
    value=c.get('cycle_structure') if isinstance(c,dict) else None
    if value is None:return None
    if not isinstance(value,dict):raise ValueError('CYCLE_STRUCTURE_OBJECT_REQUIRED')
    allowed={'mode','release_j1_offset_deg','transfer_j1','return_j1','basis'}
    extra=sorted(set(value)-allowed)
    if extra:raise ValueError('CYCLE_STRUCTURE_UNKNOWN_FIELDS:'+','.join(extra))
    if value.get('mode')!=MODE:raise ValueError('CYCLE_STRUCTURE_MODE: '+MODE)
    offset=value.get('release_j1_offset_deg')
    if (isinstance(offset,bool) or not isinstance(offset,(int,float)) or not math.isfinite(offset)
            or not 0<abs(offset)<=360):
        raise ValueError('CYCLE_STRUCTURE_RELEASE_J1_OFFSET: 0 < |deg| <= 360')
    for key in ('transfer_j1','return_j1'):
        if value.get(key) not in DIRECTIONS:raise ValueError(f'CYCLE_STRUCTURE_{key.upper()}: CLOCKWISE or COUNTERCLOCKWISE')
    if value['transfer_j1']==value['return_j1']:raise ValueError('CYCLE_STRUCTURE_RETURN_MUST_REVERSE_TRANSFER')
    if not isinstance(value.get('basis'),str) or not value['basis'].strip():raise ValueError('CYCLE_STRUCTURE_BASIS_REQUIRED')
    return dict(mode=MODE,release_j1_offset_deg=float(offset),transfer_j1=value['transfer_j1'],
                return_j1=value['return_j1'],basis=value['basis'],j1_reverse_budget_rad=J1_REVERSE_BUDGET_RAD,
                stages=list(SEPARATE_STAGES))


def from_plan(value):
    """Re-validate the policy bound into a plan (its configured fields), or None."""
    if not value:return None
    keys=('mode','release_j1_offset_deg','transfer_j1','return_j1','basis')
    policy=settings(dict(cycle_structure={k:value.get(k) for k in keys}))
    if any(value.get(k)!=policy[k] for k in ('j1_reverse_budget_rad','stages')):raise ValueError('CYCLE_STRUCTURE_PLAN_POLICY_CHANGED')
    return policy


def stages(policy):
    return SEPARATE_STAGES if policy else LEGACY_STAGES


def movements(policy):
    return SEPARATE_MOVEMENTS if policy else LEGACY_MOVEMENTS


def release_q(policy,q_start,lower=None,upper=None):
    """R = S with the configured J1 offset, inside the official joint bounds."""
    q=np.array(q_start,float).copy()
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('INVALID_Q_START')
    q[0]+=np.deg2rad(policy['release_j1_offset_deg'])
    if lower is not None and upper is not None and (np.any(q<np.asarray(lower)-1e-12) or np.any(q>np.asarray(upper)+1e-12)):
        raise ValueError('RELEASE_POSE_OUTSIDE_JOINT_LIMITS: q_release(deg)='+str(np.rad2deg(q).round(3).tolist()))
    return q


def goal_violation(policy,key,q_from,q_goal):
    """None, or why a joint goal already moves J1 against the configured direction."""
    sign=DIRECTIONS[policy[key]];reverse=max(0.,-sign*(float(q_goal[0])-float(q_from[0])))
    if reverse<=policy['j1_reverse_budget_rad']:return None
    return dict(reason='J1_GOAL_AGAINST_DIRECTION',direction=policy[key],from_j1_deg=float(np.rad2deg(q_from[0])),
                goal_j1_deg=float(np.rad2deg(q_goal[0])),reverse_rad=reverse)


def path_violation(policy,key,states):
    """None, or the first sent state where cumulative reverse J1 motion exceeds the budget."""
    sign=DIRECTIONS[policy[key]];q=np.asarray(states,float).reshape(-1,6)
    reverse=np.cumsum(np.maximum(0.,-sign*np.diff(q[:,0])))
    over=np.flatnonzero(reverse>policy['j1_reverse_budget_rad'])
    if not len(over):return None
    k=int(over[0])
    return dict(reason='J1_PATH_AGAINST_DIRECTION',direction=policy[key],state=k+1,
                reverse_rad=float(reverse[k]),total_reverse_rad=float(reverse[-1]),
                j1_deg=float(np.rad2deg(q[k+1,0])))


def j1_summary(states):
    q=np.asarray(states,float).reshape(-1,6)
    return dict(start_j1_deg=float(np.rad2deg(q[0,0])),end_j1_deg=float(np.rad2deg(q[-1,0])),
                increase_rad=float(np.maximum(0.,np.diff(q[:,0])).sum()),decrease_rad=float(np.maximum(0.,-np.diff(q[:,0])).sum()))


def released_task(box,candidate,world,release_pose):
    """Target, gripped face and world after the release, for withdraw and return_empty.

    The released carton keeps the target id and its full collision envelope,
    rigidly moved to its release pose (the attached pose at q_release), so the
    unchanged native single-carton contact rules apply to it: the withdraw LIN
    is the contact stage reversed (lip penetration and normal guards, rigid
    inserts and every other pair at their clearances), return_empty has no
    contact permission. Its original stack volume is vacated by the transfer:
    the carried envelope was checked at every loaded state, and by the scene's
    own claim rules every return inside that envelope was the target's, so the
    volume is no longer a collision object (recorded as VACATED_BY_TRANSFER;
    the remaining stack, supports, shadows and unknown-space policy are
    unchanged). Every world-frame field the native policy
    binds to the target pose (target pose, face centre/normal/u/v, measured box
    pose, corners) moves with the same rigid transform.
    """
    T=np.asarray(box['pose'],float);R=np.asarray(release_pose,float);G=R@np.linalg.inv(T)
    point=lambda x:(G[:3,:3]@np.asarray(x,float)+G[:3,3]).tolist()
    vector=lambda x:(G[:3,:3]@np.asarray(x,float)).tolist()
    released=dict(box,pose=R.tolist(),corners_m=[point(c) for c in box['corners_m']])
    if box.get('measured_box'):released['measured_box']=dict(box['measured_box'],pose=(G@np.asarray(box['measured_box']['pose'],float)).tolist())
    face=dict(candidate['face'],center_m=point(candidate['face']['center_m']),
              normal=vector(candidate['face']['normal']),u=vector(candidate['face']['u']),v=vector(candidate['face']['v']))
    moved=dict(candidate,face=face,tcp_pose=(G@np.asarray(candidate['tcp_pose'],float)).tolist(),
               precontact_pose=(G@np.asarray(candidate['precontact_pose'],float)).tolist())
    vacated=dict(id=box['id'],size=list(box['size_m']),pose=T.tolist(),status='VACATED_BY_TRANSFER',
                 basis='TARGET_ENVELOPE_CARRIED_AND_CHECKED_AT_EVERY_LOADED_STATE_RETURNS_INSIDE_IT_WERE_CLAIMED_BY_THE_TARGET')
    carton=dict(id=box['id'],size=list(box['size_m']),pose=R.tolist(),category='released_carton',
                basis='CONSERVATIVE_RELEASED_CARTON_STAYS_AT_RELEASE_POSE_RECEIVER_NOT_MODELLED')
    if sum(o['id']==box['id'] for o in world)!=1:raise ValueError('RELEASED_TASK_TARGET_NOT_UNIQUE_IN_WORLD')
    out=[o for o in world if o['id']!=box['id']]
    return released,moved,out+[carton],dict(released_carton=carton,vacated_volume=vacated,rigid_motion=G.tolist())


def withdraw_goal(tcp_release,tcp_contact,face_normal_world,distance_m):
    """TCP goal backing off the released carton along the gripped face normal.

    The face normal is fixed in the TCP frame by the actual contact pose, so the
    withdraw is the approach->contact LIN reversed in the carton's frame.
    """
    A=np.asarray(tcp_release,float);C=np.asarray(tcp_contact,float);n=np.asarray(face_normal_world,float)
    n_tcp=C[:3,:3].T@(n/np.linalg.norm(n))
    goal=A.copy();goal[:3,3]+=A[:3,:3]@n_tcp*float(distance_m)
    return goal
