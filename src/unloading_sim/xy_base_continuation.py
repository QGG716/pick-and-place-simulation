"""Explicit, unloaded XY relocation between accepted carton cycles (SI units).

The six-axis configuration, fixed equipment and cartons do not change during
relocation. Only the robot and its existing solid plinth translate. Paths are
sampled geometric assumptions, not physical actuation or continuous-sweep proof.
"""
from dataclasses import replace
import math
import numpy as np
from .joint_search_layout import LayoutCandidate,candidate_data
from .moveit2_backend import box_message,digest
from .coupled_lift_checks import _pair_geometry

XY_SCHEMA='accepted_prefix_xy_base_continuation_20261029_v1'
MAXIMUM_XY_STEP_M=.005


def xy_path(start,end,*,order=(0,1),step=MAXIMUM_XY_STEP_M):
    a=np.asarray(start,float);b=np.asarray(end,float)
    if (a.shape!=(2,) or b.shape!=(2,) or not np.isfinite(np.r_[a,b,step]).all()
            or not 0<step<=MAXIMUM_XY_STEP_M or tuple(sorted(order))!=(0,1)):
        raise ValueError('XY_PATH_SCOPE')
    if sum(math.ceil(abs(v)/step) for v in b-a)>4096:raise ValueError('XY_PATH_RESOURCE_LIMIT')
    points=[a.copy()];current=a.copy()
    for axis in order:
        count=math.ceil(abs(b[axis]-current[axis])/step)
        for value in np.linspace(current[axis],b[axis],count+1)[1:]:
            q=current.copy();q[axis]=value;points.append(q)
        current[axis]=b[axis]
    return np.asarray(points)


def xy_candidate(original,xy,baseline):
    c=LayoutCandidate(*xy,original.z,original.minimum,original.maximum,original.profile)
    candidate_data(baseline,c)
    return c


def translated_body(box,delta):
    """Retain ConceptSolid's actual vertices; never replace it with its OBB."""
    shift=np.asarray(delta,float)
    if shift.shape!=(3,) or not np.isfinite(shift).all():raise ValueError('XY_TRANSLATION_NUMERIC')
    return replace(box,center=box.center+shift)


def relocation_world(initial_world,delta,*,evaluation_frame=False):
    shift=np.asarray(delta,float)
    if shift.shape!=(3,) or not np.isfinite(shift).all() or shift[2]!=0.:
        raise ValueError('XY_TRANSLATION_ONLY')
    if sum(b.name=='mount_plinth' for b in initial_world)!=1:raise ValueError('XY_SOLID_PLINTH_REQUIRED')
    # In the native evaluation frame the unchanged robot model stays at its
    # original mount. Translating all other bodies by -delta is an exact rigid
    # coordinate change, not an environment-motion or collision exemption.
    world=tuple(translated_body(b,shift if b.name=='mount_plinth' else np.zeros(3)) for b in initial_world)
    return tuple(translated_body(b,-shift) for b in world) if evaluation_frame else world


def check_plinth_world(world,*,clearance=.005):
    if clearance!=.005:raise ValueError('XY_PLINTH_CLEARANCE_CHANGED')
    by_name={b.name:b for b in world}
    if len(by_name)!=len(world) or 'mount_plinth' not in by_name or 'chassis' not in by_name:
        raise ValueError('XY_WORLD_IDENTITY')
    plinth=by_name['mount_plinth'];deck=by_name['chassis']
    if not np.array_equal(plinth.rotation,np.eye(3)) or not np.array_equal(deck.rotation,np.eye(3)):
        raise ValueError('XY_HORIZONTAL_SUPPORT_REQUIRED')
    lo,hi=plinth.center-plinth.half_extents,plinth.center+plinth.half_extents
    dl,dh=deck.center-deck.half_extents,deck.center+deck.half_extents
    supported=abs(lo[2]-dh[2])<=1e-10 and np.all(lo[:2]>=dl[:2]-1e-12) and np.all(hi[:2]<=dh[:2]+1e-12)
    failures=[]
    if not supported:failures.append(dict(reason='XY_PLINTH_NOT_FULLY_SUPPORTED'))
    for other in world:
        if other.name in {'mount_plinth','chassis'}:continue
        # Disjoint AABBs are only a lower-bound broad phase; close pairs use
        # the unchanged exact OBB / concept-solid authority.
        lower=np.maximum(np.abs(plinth.center-other.center)-plinth.half_extents-np.abs(other.rotation)@other.half_extents,0.)
        if np.linalg.norm(lower)>=clearance+1e-9:continue
        check=_pair_geometry(plinth,other)
        if check['surface_distance_m']+1e-9<clearance or check['signed_sat_separation_m']<0:
            failures.append(dict(reason='XY_PLINTH_WORLD_CLEARANCE',**check))
    return dict(passed=not failures,full_deck_support=bool(supported),failures=failures,
        permitted_support_pair=['mount_plinth','chassis'],permitted_support_scope='FULL_NONPENETRATING_BOTTOM_PLANE_ONLY')


def relocation_receipt(before,source,destination,evidence_sha256):
    if (before.get('assumed_removed') or before['velocity_rad_s']!=[0.]*6
            or source.z!=destination.z or source.minimum!=destination.minimum
            or source.maximum!=destination.maximum or source.profile!=destination.profile):
        raise ValueError('XY_RECEIPT_FROZEN_STATE_CHANGED')
    return dict(schema=XY_SCHEMA,operation='UNLOADED_BASE_AND_SOLID_PLINTH_XY_TRANSLATION',
        parent_root_sha256=digest(before),parent_final_state_receipt=before['predecessor_final_state_receipt'],
        from_layout=source.evidence(),to_layout=destination.evidence(),q_rad=list(before['q_rad']),
        height_m=before['height_m'],world_cartons_sha256=digest(before['world_cartons']),
        completed=list(before['completed']),evidence_sha256=evidence_sha256,
        base_actuation='KINEMATIC_ASSUMPTION_NATIVE_DISCRETE_COLLISION_CHECKED',
        physical_execution='NOT_RUN',continuous_swept_proof=False)
