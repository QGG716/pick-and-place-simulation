"""Explicit field parameters; missing values are reported, never guessed."""
import json
from pathlib import Path
import math
import numpy as np
import hashlib
from ..controlled.planner import ROOT,load_config as load_execution

STEP4_SWITCHES=('resident_workers','scene_handles','async_evidence','path_endpoint_reuse','binding_hash_cache')
REQUIRED_VACUUM=('grip_below_kpa','release_above_kpa','hold_below_kpa',
                'grip_dwell_s','release_dwell_s','grip_timeout_s','release_timeout_s',
                'max_age_s','establish_commands','release_commands','field_basis')


def release_policy_evidence(c):
    policy=c.get('release_policy')
    if policy=='MEASURED_SITE_RECEIVER_REQUIRED':
        return dict(policy=policy,qualification_deferred=False,basis='MEASURED_SITE_GEOMETRY_REQUIRED')
    if policy=='USER_DEFERRED_RECEIVER_QUALIFICATION':
        basis=c.get('receiver_qualification_basis')
        if not isinstance(basis,str) or not basis.strip():
            raise ValueError('RECEIVER_DEFERRAL_BASIS_REQUIRED: 延期承接区资格核验须保留用户明确要求的依据')
        return dict(policy=policy,qualification_deferred=True,basis=basis,
                    status='RECEIVER_QUALIFICATION_DEFERRED_BY_USER',support_confirmed=False)
    raise ValueError('INVALID_RELEASE_POLICY: 必须明确选择实测承接检查或用户已要求的资格核验延期策略')


# Pressure gate approved on 2026-11-11 (two fresh heartbeats over >= 1 s, 3 s
# grip/release timeouts) and the reviewed freshness bound of the example file.
VACUUM_MIN_DWELL_S=1.
VACUUM_MAX_TIMEOUT_S=3.
VACUUM_MAX_AGE_S=3.


def pre_execute_visual_recheck(c):
    """Whether execute re-captures RGB-D and compares it with the planning frame.

    ENABLED (default when absent) keeps the stationary re-capture plus the
    whole-scene and target-local depth/appearance comparison. TARGET_ONLY keeps
    the re-capture, session/start/camera checks and the target-local comparison.
    DISABLED_BY_USER skips the re-capture. Both non-default values require a
    recorded basis. The non-visual interlocks are never disabled:
    task record, session generation, file binding, and Cycle.start()'s fresh
    live joints within the configured start tolerance, enabled and alarm-free.
    """
    value=c.get('pre_execute_visual_recheck','ENABLED')
    if value=='ENABLED':return dict(enabled=True,scope='SCENE_AND_TARGET')
    if value=='TARGET_ONLY':
        # Skips only the whole-view depth comparison, which far-range depth
        # noise trips on a static scene; the target-local check remains.
        basis=c.get('pre_execute_visual_recheck_basis')
        if not isinstance(basis,str) or not basis.strip():
            raise ValueError('PRE_EXECUTE_RECHECK_SCOPE_BASIS_REQUIRED: 只复核目标须记录依据')
        return dict(enabled=True,scope='TARGET_ONLY',basis=basis)
    if value=='DISABLED_BY_USER':
        basis=c.get('pre_execute_visual_recheck_basis')
        if not isinstance(basis,str) or not basis.strip():
            raise ValueError('PRE_EXECUTE_RECHECK_DISABLE_BASIS_REQUIRED: 关闭执行前画面复核须记录用户要求的依据')
        return dict(enabled=False,status='VISUAL_RECHECK_DISABLED_BY_USER',basis=basis,
                    retained=['TASK_RECORD_LIVE','SESSION_GENERATION','FILE_VERSION_BINDING',
                              'FRESH_LIVE_JOINTS_CONFIGURED_START_TOLERANCE_ENABLED_NO_ALARM'])
    raise ValueError('PRE_EXECUTE_RECHECK_POLICY_INVALID: ENABLED、TARGET_ONLY 或 DISABLED_BY_USER')


def start_joint_check(c):
    """Whether the cycle compares fresh live joints with the frozen q_start.

    ENABLED (default when absent): configured tolerance (default 0.01 deg).
    DISABLED_BY_USER needs a recorded basis; the live sample must still be
    fresh, enabled, alarm-free and stationary, and every later segment keeps
    the arrival / tracking / feedback / stop supervision.
    """
    from ..controlled.start_policy import start_limits
    policy=dict(tolerance_deg=c.get('start_joint_tolerance_deg',.01),
        hard_cap_deg=c.get('start_hard_cap_deg',.5),
        tolerance_basis=c.get('start_joint_tolerance_basis','Historical default 0.01 degree'),
        hard_cap_basis=c.get('start_hard_cap_basis','User decision 2026-11-19: non-waivable 0.5 degree cap'))
    start_limits(policy)
    for key in ('tolerance_basis','hard_cap_basis'):
        if not isinstance(policy[key],str) or not policy[key].strip():raise ValueError('START_JOINT_LIMIT_BASIS_REQUIRED')
    if 'start_joint_tolerance_deg' in c and not c.get('start_joint_tolerance_basis'):
        raise ValueError('START_JOINT_LIMIT_BASIS_REQUIRED')
    if 'start_hard_cap_deg' in c and not c.get('start_hard_cap_basis'):
        raise ValueError('START_JOINT_LIMIT_BASIS_REQUIRED')
    if policy['tolerance_deg']>.02:raise ValueError('START_JOINT_TOLERANCE_LOOSER_THAN_USER_APPROVED')
    value=c.get('start_joint_check','ENABLED')
    if value=='ENABLED':return dict(policy,enabled=True)
    if value=='DISABLED_BY_USER':
        basis=c.get('start_joint_check_basis')
        if not isinstance(basis,str) or not basis.strip():
            raise ValueError('START_JOINT_CHECK_DISABLE_BASIS_REQUIRED: 取消起点核对须记录用户要求的依据')
        return dict(policy,enabled=False,status='START_JOINT_CHECK_DISABLED_BY_USER',basis=basis)
    raise ValueError('START_JOINT_CHECK_POLICY_INVALID: ENABLED 或 DISABLED_BY_USER')


def plan_validity(c):
    value=c.get('plan_validity_s',120.)
    basis=c.get('plan_validity_basis','User decision 2026-11-19: frozen exposure receipt, at most 120 s')
    if (type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=120
            or not isinstance(basis,str) or not basis.strip()):raise ValueError('PLAN_VALIDITY_INVALID')
    if 'plan_validity_s' in c and not c.get('plan_validity_basis'):raise ValueError('PLAN_VALIDITY_BASIS_REQUIRED')
    return dict(seconds=float(value),basis=basis,clock='HOST_MONOTONIC_FROZEN_EXPOSURE_RECEIPT')


def observation_mode(c):
    """observation_mode (user decision 2026-11-25, default unchanged).

    OPERATOR_BUTTON (absent): the operator's observe button freezes q_start and
    one RGB-D, then detection, segmentation, geometry and scene run in order.
    CONTINUOUS_STANDING: from start-up the continuous detection/segmentation
    loop itself takes frames that pass the identical stationary/pairing gate
    (perception.FreezeGate), and keeps one immutable standing observation
    (q_start, frame, masks, boxes, scene). It is rebuilt only when missing,
    after a session change, when a joint leaves q_start by more than the
    existing 0.02 deg stationary range, or after ``refresh_s``. Planning binds
    the standing observation and only then writes its records. It pauses while
    a task is active, a load may be held, a stop is unconfirmed, or a failed or
    cancelled cycle still offers the reset to its q_start.
    """
    mode=c.get('observation_mode','OPERATOR_BUTTON')
    if mode not in ('OPERATOR_BUTTON','CONTINUOUS_STANDING'):raise ValueError('OBSERVATION_MODE_INVALID')
    if mode=='OPERATOR_BUTTON':
        if 'standing_observation' in c:raise ValueError('STANDING_OBSERVATION_REQUIRES_CONTINUOUS_STANDING')
        return dict(mode=mode)
    standing=c.get('standing_observation')
    if not isinstance(standing,dict):raise ValueError('STANDING_OBSERVATION_SETTINGS_REQUIRED')
    refresh=standing.get('refresh_s');basis=standing.get('basis')
    validity=plan_validity(c)['seconds']
    if (type(refresh) not in (int,float) or not math.isfinite(refresh) or not 1<=refresh<validity):
        raise ValueError('STANDING_OBSERVATION_REFRESH_INVALID')
    if not isinstance(basis,str) or not basis.strip():raise ValueError('STANDING_OBSERVATION_BASIS_REQUIRED')
    return dict(mode=mode,refresh_s=float(refresh),basis=basis,
                joint_drift_deg=.02,joint_drift_basis='EXISTING_STATIONARY_RANGE_0_02_DEG')


def scene_build(c):
    """scene_build (user decision 2026-11-25): SEQUENTIAL (absent) or SPLIT_PARALLEL.

    SPLIT_PARALLEL computes the mask-independent part of the site scene (voxel
    free space, base_link points, robot self-ownership of every valid return
    and every voxel at q_capture) in a child process as soon as the frame is
    frozen, while detection and segmentation run; the mask-dependent part then
    reuses those per-pixel/per-voxel results. Same functions, same thresholds.
    """
    mode=c.get('scene_build','SEQUENTIAL')
    if mode not in ('SEQUENTIAL','SPLIT_PARALLEL'):raise ValueError('SCENE_BUILD_INVALID')
    return mode


def execution_lock(c):
    """Hard lock after the 2026-11-14 incident: no motion may be submitted.

    Absent or {"locked": false} = unlocked. Locked needs a basis. Planning and
    preview stay available; stop paths are never affected. Unlocking is an
    explicit configuration change followed by a restart.
    """
    lock=c.get('execution_lock')
    if lock is None:return None
    if not isinstance(lock,dict) or type(lock.get('locked'))!=bool:raise ValueError('EXECUTION_LOCK_INVALID')
    if not lock['locked']:return None
    if not isinstance(lock.get('basis'),str) or not lock['basis'].strip():raise ValueError('EXECUTION_LOCK_BASIS_REQUIRED')
    return dict(locked=True,status='EXECUTION_LOCKED_BY_USER',basis=lock['basis'])


def tool_robot_clearance(c,tool_links=None):
    """Clearance between owned tool solids and non-tool robot links (metres).

    Absent = the source policy's 2.4 mm. Exemptions must name the fixed mount
    link and specific motion-invariant (tool part, link) pairs only.
    """
    t=c.get('tool_robot_clearance')
    if t is None:return None
    if (not isinstance(t,dict) or not isinstance(t.get('clearance_m'),(int,float)) or not math.isfinite(t['clearance_m'])
            or not .0024<=t['clearance_m']<=.1 or not isinstance(t.get('basis'),str) or not t['basis'].strip()):
        raise ValueError('TOOL_ROBOT_CLEARANCE_INVALID: clearance_m 0.0024～0.1 m，并须记录依据')
    links=t.get('exempt_robot_links',[]);pairs=t.get('exempt_pairs',[])
    if not isinstance(links,list) or not all(isinstance(x,str) for x in links) or not isinstance(pairs,list) or \
            not all(isinstance(x,list) and len(x)==2 and all(isinstance(y,str) for y in x) for x in pairs):
        raise ValueError('TOOL_ROBOT_CLEARANCE_EXEMPTIONS_INVALID')
    if tool_links is not None:
        tools=set(tool_links)
        if any(x in tools for x in links) or any(a not in tools or b in tools for a,b in pairs):
            raise ValueError('TOOL_ROBOT_CLEARANCE_EXEMPTIONS_NOT_TOOL_ROBOT')
    return dict(clearance_m=float(t['clearance_m']),exempt_robot_links=sorted(links),exempt_pairs=[list(x) for x in pairs],basis=t['basis'])


def pair_clearance(c):
    """pair_clearance (user decision 2026-11-25, "把要求2.4mm，降低为1.5mm"): the
    required gap of every collision pair (self pairs included) in the physical
    single-carton planning, {clearance_m 0.0005-0.0024, basis}. Absent = the
    source policy's 2.4 mm. The tool/robot reserve and the seal rules are
    separate and unchanged. PYTHON flow only."""
    p=c.get('pair_clearance')
    if p is None:return None
    if (not isinstance(p,dict) or set(p)!={'clearance_m','basis'} or type(p['clearance_m']) not in (int,float)
            or not math.isfinite(p['clearance_m']) or not .0005<=p['clearance_m']<=.0024
            or not isinstance(p['basis'],str) or not p['basis'].strip()):
        raise ValueError('PAIR_CLEARANCE_INVALID: {clearance_m: 0.0005～0.0024 m, basis: 依据}')
    if c.get('planning_module')=='NATIVE':raise ValueError('PAIR_CLEARANCE_PYTHON_FLOW_ONLY')
    return dict(clearance_m=float(p['clearance_m']),basis=p['basis'])


def absent_tool_parts(c,tool_links=None):
    """absent_tool_parts (user decision 2026-11-25, "把吸具零件 part_001从layout中去掉";
    confirmed "实物没有，去掉"): rigid tool solids of the CAD model that the
    field tool does not have, {parts: [ids], basis}. They take part in no
    collision pair (native absent_tool_parts_v1); every other solid is
    unchanged. Cup lips can never be named. PYTHON flow only."""
    a=c.get('absent_tool_parts')
    if a is None:return None
    if (not isinstance(a,dict) or set(a)!={'parts','basis'} or not isinstance(a['parts'],list) or not a['parts']
            or not all(isinstance(x,str) and x.startswith('part_') and '_lip' not in x for x in a['parts'])
            or len(set(a['parts']))!=len(a['parts']) or not isinstance(a['basis'],str) or not a['basis'].strip()):
        raise ValueError('ABSENT_TOOL_PARTS_INVALID: {parts: [吸具刚性零件 id], basis: 依据}')
    if tool_links is not None and not set(a['parts'])<=set(tool_links):raise ValueError('ABSENT_TOOL_PARTS_NOT_TOOL_LINKS')
    if c.get('planning_module')=='NATIVE':raise ValueError('ABSENT_TOOL_PARTS_PYTHON_FLOW_ONLY')
    return dict(parts=list(a['parts']),basis=a['basis'])


WRIST_DEFAULTS=dict(keep_j5_sign=True,j5_min_abs_deg=10.,max_j4_travel_deg=90.,max_j6_travel_deg=180.)


def wrist_branch_limits(c):
    """Wrist-branch limits relative to the frozen q_start, for every sent state.

    Absent = no limit (historical). keep_j5_sign forbids crossing the wrist
    singularity J5=0 (a wrist flip); travel limits bound J4/J6 rotation.
    Explicit max_j4_travel_deg=null or max_j6_travel_deg=null disables only that
    relative travel gate (user decisions 2026-11-18 for J4, 2026-11-21 for J6).
    Official joint bounds are checked independently in native FK/planning.
    """
    w=c.get('wrist_branch_limits')
    if w is None:return None
    if not isinstance(w,dict) or set(w)-set(WRIST_DEFAULTS)-{'basis'}:raise ValueError('WRIST_BRANCH_LIMITS_INVALID')
    out=dict(WRIST_DEFAULTS,**{k:v for k,v in w.items() if k!='basis'})
    if (type(out['keep_j5_sign'])!=bool or not all((k in ('max_j4_travel_deg','max_j6_travel_deg') and out[k] is None) or
            (type(out[k]) in (int,float) and math.isfinite(out[k]) and out[k]>=0)
            for k in ('j5_min_abs_deg','max_j4_travel_deg','max_j6_travel_deg')) or out['j5_min_abs_deg']>60
            or not isinstance(w.get('basis'),str) or not w['basis'].strip()):
        raise ValueError('WRIST_BRANCH_LIMITS_INVALID')
    out['basis']=w['basis'];return out


def wrist_branch_violation(limits,q_start,states):
    """First violation of the wrist limits along states, or None."""
    if not limits:return None
    q0=np.rad2deg(np.asarray(q_start,float));q=np.rad2deg(np.asarray(states,float)).reshape(-1,6)
    if limits['keep_j5_sign']:
        if abs(q0[4])<limits['j5_min_abs_deg']:return dict(reason='Q_START_J5_INSIDE_SINGULAR_BAND',j5_deg=float(q0[4]))
        bad=np.flatnonzero((np.sign(q[:,4])!=np.sign(q0[4]))|(abs(q[:,4])<limits['j5_min_abs_deg']))
        if len(bad):k=int(bad[0]);return dict(reason='J5_WRIST_FLIP_OR_SINGULAR',index=k,j5_deg=float(q[k,4]),j5_start_deg=float(q0[4]),
                                              j5_min_abs_deg=limits['j5_min_abs_deg'])
    for j,key in ((3,'max_j4_travel_deg'),(5,'max_j6_travel_deg')):
        if limits[key] is None:continue
        travel=abs(q[:,j]-q0[j]);k=int(np.argmax(travel))
        if travel[k]>limits[key]:return dict(reason=f'J{j+1}_TRAVEL_EXCEEDED',index=k,travel_deg=float(travel[k]),limit_deg=limits[key])
    return None


def issues(c):
    errors=[]
    from .site_layout import registration_limits,load_layout
    try:
        registration_limits(c)
        if c.get('site_layout') is not None:load_layout(c,ROOT)
    except (ValueError,KeyError,OSError) as e:errors.append('site_layout: '+str(e))
    age=c.get('scene_max_age_s',300.)
    if type(age) not in (int,float) or not math.isfinite(age) or not 0<age<=300:
        errors.append('scene_max_age_s: INVALID_OR_LOOSER_THAN_300_S')
    if 'scene_max_age_s' in c and (not isinstance(c.get('scene_max_age_basis'),str) or not c['scene_max_age_basis'].strip()):
        errors.append('scene_max_age_s: BASIS_REQUIRED')
    from .holding_capacity import settings as capacity_settings
    try:capacity_settings(c)
    except ValueError as e:errors.append('holding_capacity: '+str(e))
    try:release_policy_evidence(c)
    except ValueError as e:errors.append('release_policy：'+str(e))
    try:pre_execute_visual_recheck(c)
    except ValueError as e:errors.append('pre_execute_visual_recheck：'+str(e))
    try:start_joint_check(c)
    except ValueError as e:errors.append('start_joint_check：'+str(e))
    for name,check in (('plan_validity',plan_validity),('observation_mode',observation_mode),('scene_build',scene_build),('execution_lock',execution_lock),('tool_robot_clearance',tool_robot_clearance),('pair_clearance',pair_clearance),('absent_tool_parts',absent_tool_parts),('wrist_branch_limits',wrist_branch_limits)):
        try:check(c)
        except ValueError as e:errors.append(name+'：'+str(e))
    from .posture import settings as posture_settings
    from .geometry import contact_search_settings
    for name,check in (('posture_preference',posture_settings),('contact_search',lambda x:contact_search_settings(x.get('contact_search')))):
        try:check(c)
        except ValueError as e:errors.append(name+'：'+str(e))
    # The C++ composite (planning_module NATIVE) implements neither the J6-null
    # travel gate nor the wrist-down preference term: fail closed instead of
    # letting the native order or wrist filter differ from the Python flow.
    if c.get('planning_module')=='NATIVE':
        w=c.get('wrist_branch_limits') or {}
        if 'max_j6_travel_deg' in w and w['max_j6_travel_deg'] is None:
            errors.append('wrist_branch_limits：max_j6_travel_deg=null 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
        if 'wrist_down_weight' in (c.get('posture_preference') or {}):
            errors.append('posture_preference：wrist_down_weight 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
        if 'single_contact' in (c.get('contact_search') or {}):
            errors.append('contact_search：single_contact 仅 PYTHON 规划流程支持')
        if 'arm_facing' in (c.get('posture_preference') or {}):
            errors.append('posture_preference：arm_facing 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
        if 'wrist_down_priority' in (c.get('posture_preference') or {}):
            errors.append('posture_preference：wrist_down_priority 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
        if 'tolerance_deg' in (c.get('posture_preference') or {}):
            errors.append('posture_preference：tolerance_deg 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
        if 'upper_arm_max_from_vertical_deg' in (c.get('posture_preference') or {}):
            errors.append('posture_preference：upper_arm_max_from_vertical_deg 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
    # Start/release separated cycle (user decision 2026-11-22): PYTHON flow only.
    from .cycle_structure import settings as cycle_settings
    try:
        if cycle_settings(c) is not None and c.get('planning_module')=='NATIVE':
            errors.append('cycle_structure：START_RELEASE_SEPARATE 仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
    except ValueError as e:errors.append('cycle_structure：'+str(e))
    geometry=c.get('geometry',{})
    from .stack_support import policy as stack_policy
    try:stack_policy(geometry)
    except ValueError as e:errors.append('geometry.confirmed_stack_support：'+str(e))
    from .coplanar_seam import policy as seam_policy
    try:seam_policy(geometry)
    except ValueError as e:errors.append('geometry.coplanar_seam_policy: '+str(e))
    from .stereo_shadow import policy as shadow_policy
    try:shadow_policy(geometry)
    except ValueError as e:errors.append('geometry.stereo_shadow_boundary_policy: '+str(e))
    from .rgb_color_boundary import policy as colour_policy
    try:colour_policy(geometry)
    except ValueError as e:errors.append('geometry.rgb_color_boundary_policy: '+str(e))
    from .measured_envelope import max_carton_extent
    try:max_carton_extent(geometry)
    except ValueError as e:errors.append('geometry.max_carton_extent_m: '+str(e))
    lip=c.get('neighbor_lip_contact')
    if lip is not None and (not isinstance(lip,dict) or lip.get('mode') not in ('DISABLED','CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS')
                            or (lip.get('mode')!='DISABLED' and not str(lip.get('basis','')).strip())):
        errors.append('neighbor_lip_contact: NEIGHBOR_LIP_CONTACT_POLICY_INVALID')
    if isinstance(lip,dict) and 'observed_support_basis' in lip and (lip.get('mode')!='CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS'
            or not isinstance(lip['observed_support_basis'],str) or not lip['observed_support_basis'].strip()):
        errors.append('neighbor_lip_contact.observed_support_basis：非空依据，且仅在 mode=CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS 时可用（唇边可碰目标下方实测支撑）')
    if isinstance(lip,dict) and 'cup_body_basis' in lip and (lip.get('mode')!='CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS'
            or not isinstance(lip['cup_body_basis'],str) or not lip['cup_body_basis'].strip()):
        errors.append('neighbor_lip_contact.cup_body_basis：非空依据，且仅在 mode=CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS 时可用（整个吸盘体可碰邻箱）')
    if isinstance(lip,dict) and 'cup_body_basis' in lip and c.get('planning_module')=='NATIVE':
        errors.append('neighbor_lip_contact.cup_body_basis 仅 PYTHON 规划流程支持')
    nb=c.get('contact_failure_next_branch')
    if nb is not None and (not isinstance(nb,dict) or nb.get('mode') not in ('DISABLED','NEXT_IK_BRANCH') or set(nb)-{'mode','basis'}
                           or (nb.get('mode')=='NEXT_IK_BRANCH' and not str(nb.get('basis','')).strip())):
        errors.append('contact_failure_next_branch：mode 为 DISABLED 或 NEXT_IK_BRANCH，后者需非空 basis')
    if isinstance(nb,dict) and nb.get('mode')=='NEXT_IK_BRANCH' and c.get('planning_module')=='NATIVE':
        errors.append('contact_failure_next_branch 仅 PYTHON 规划流程支持')
    tc=c.get('target_cup_approach_contact')
    # User decision 2026-11-25 ("吸盘是柔性的，所以只记录，不拒绝"): in the approach
    # stage the connected cups may touch the target envelope; recorded, not rejected.
    if tc is not None and (not isinstance(tc,dict) or set(tc)-{'mode','basis'}
                           or tc.get('mode') not in ('DISABLED','CUPS_MAY_TOUCH_TARGET_DURING_APPROACH_RECORD_ONLY')
                           or (tc.get('mode')!='DISABLED' and not str(tc.get('basis','')).strip())):
        errors.append('target_cup_approach_contact：mode 为 DISABLED 或 CUPS_MAY_TOUCH_TARGET_DURING_APPROACH_RECORD_ONLY，后者需非空 basis')
    if isinstance(tc,dict) and tc.get('mode')=='CUPS_MAY_TOUCH_TARGET_DURING_APPROACH_RECORD_ONLY' and c.get('planning_module')=='NATIVE':
        errors.append('target_cup_approach_contact 仅 PYTHON 规划流程支持')
    mc=c.get('mast_contact')
    # User decision 2026-11-25 ("所有与视觉桅杆的碰撞都只记录，不否决"; asked about a
    # depth bound, chose "全部只记录，不设上限"): every moving robot link, the tool
    # and the carried carton may touch the vision mast; recorded, not rejected.
    if mc is not None and (not isinstance(mc,dict) or set(mc)-{'mode','basis'}
                           or mc.get('mode') not in ('DISABLED','ROBOT_MAST_CONTACT_RECORD_ONLY')
                           or (mc.get('mode')!='DISABLED' and not str(mc.get('basis','')).strip())):
        errors.append('mast_contact：mode 为 DISABLED 或 ROBOT_MAST_CONTACT_RECORD_ONLY，后者需非空 basis')
    if isinstance(mc,dict) and mc.get('mode')=='ROBOT_MAST_CONTACT_RECORD_ONLY' and c.get('planning_module')=='NATIVE':
        errors.append('mast_contact 仅 PYTHON 规划流程支持')
    from .passive_cups import policy as passive_cup_policy
    try:passive_cup_policy(c)
    except ValueError as e:errors.append('passive_cup_neighbour: '+str(e))
    from .cycle import dry_run_policy
    try:dry_run_policy(c)
    except ValueError as e:errors.append('dry_run_no_vacuum: '+str(e))
    from .debug_obstacles import policy as debug_obstacles_policy
    try:debug_obstacles_policy(c)
    except ValueError as e:errors.append('debug_near_target_obstacles: '+str(e))
    from .measured_envelope import single_face_policy,continuation_policy
    from .non_interpenetration import policy as ni_policy
    from .stack_support import resting_policy
    from .site_scene import own_mask_claim_policy
    from .adjacent_contact import policy as adjacent_policy
    from .edge_mixed_pixels import policy as mixed_policy
    for name,check in (('single_face_depth_policy',single_face_policy),('coplanar_continuation_policy',continuation_policy),
                       ('non_interpenetration_policy',ni_policy),('resting_contact_policy',resting_policy),
                       ('own_mask_claim_policy',own_mask_claim_policy),('adjacent_contact_policy',adjacent_policy),
                       ('edge_mixed_pixel_policy',mixed_policy)):
        try:check(geometry)
        except ValueError as e:errors.append(f'geometry.{name}: '+str(e))
    from .size_policy import size_policy
    try:policy=size_policy(geometry)
    except ValueError as e:errors.append('geometry.size_policy：'+str(e));policy=None
    prior=geometry.get('prior_size_m')
    if policy=='CONFIGURED_SINGLE_TYPE' and (not isinstance(prior,list) or len(prior)!=3 or
            any(not isinstance(x,(int,float)) or not math.isfinite(x) or not .005<x<=2 for x in prior)):
        errors.append('geometry.prior_size_m：明确的现场箱型尺寸先验，单位 m')
    # The prior bounds hidden thickness in the collision envelope, so its
    # source must be recorded explicitly (tape measurement or RGB-D task id).
    basis=geometry.get('prior_measurement_basis')
    if policy=='CONFIGURED_SINGLE_TYPE' and (not isinstance(basis,str) or not basis.strip()):
        errors.append('geometry.prior_measurement_basis：填写箱型尺寸来源（卷尺实测或 RGB-D 实测任务编号）')
    tol=geometry.get('dimension_tolerance_m')
    if policy=='CONFIGURED_SINGLE_TYPE' and (not isinstance(tol,(int,float)) or not math.isfinite(tol) or not 0<=tol<=.03):
        errors.append('geometry.dimension_tolerance_m：尺寸先验误差界，0 至 0.03 m')
    detector=c.get('detector',{});backend=detector.get('backend','local')
    if backend=='remote_ssh':
        from .remote_detector import remote_config_issues
        errors += remote_config_issues(detector.get('remote'))
    elif backend!='local':errors.append('detector.backend：local 或 remote_ssh')
    if not c.get('camera_extrinsics'):errors.append('camera_extrinsics：现场 link_1 外参路径')
    elif 'training_extrinsics.example' in str(c['camera_extrinsics']):errors.append('训练外参示例不能用于实机任务')
    mass=c.get('carton_mass_kg')
    if not isinstance(mass,(int,float)) or not math.isfinite(mass) or not 0<mass<=3.5:
        errors.append('carton_mass_kg：本次箱子质量，0 < kg ≤ 3.5')
    if not c.get('controller_loaded_payload_basis'):errors.append('controller_loaded_payload_basis：现场带载控制器设定依据')
    payload=c.get('controller_payload_kg')
    if (not isinstance(payload,(int,float)) or not math.isfinite(payload) or
            not isinstance(mass,(int,float)) or not math.isfinite(mass) or
            abs(payload-(1.5+mass))>1e-9 or payload>5):
        errors.append('controller_payload_kg：本轮带载设定须明确等于 1.5 kg 吸具 + 箱子质量，且不超过 5 kg')
    scale=c.get('time_scale')
    if not isinstance(scale,(int,float)) or not math.isfinite(scale) or not 1<=scale<=100:
        errors.append('time_scale：时间放大倍数，1 至 100；发送周期保持 5 ms')
    workers=c.get('native_check_workers',0)
    if type(workers)!=int or not 0<=workers<=12:
        errors.append('native_check_workers：并行检查发送路径的额外原生进程数，0 至 12（0 为不并行）')
    lanes=c.get('parallel_candidate_lanes',1)
    if type(lanes)!=int or not 1<=lanes<=13:
        errors.append('parallel_candidate_lanes：同时规划的候选数（每个占一个原生进程），1 至 13（1 为逐个规划）')
    solver=c.get('native_solver_budget_s',25.)
    if type(solver) not in (int,float) or not math.isfinite(solver) or not 0<solver<=100:
        errors.append('native_solver_budget_s：单次原生路径搜索预算（秒），0 < s ≤ 100；仍受统一规划截止时间约束')
    # Optional OMPL-only search budget (user decision 2026-11-21): the Pilz request
    # keeps native_solver_budget_s; only the OMPL fallback after a failed direct
    # plan uses this. A search setting, not a motion or monitor threshold.
    if 'native_ompl_budget_s' in c:
        ompl=c['native_ompl_budget_s']
        if type(ompl) not in (int,float) or not math.isfinite(ompl) or not 0<ompl<=100:
            errors.append('native_ompl_budget_s：OMPL 避让搜索预算（秒），0 < s ≤ 100；仍受统一规划截止时间约束')
        if not isinstance(c.get('native_ompl_budget_basis'),str) or not c['native_ompl_budget_basis'].strip():
            errors.append('native_ompl_budget_basis：设置 native_ompl_budget_s 时必须写明依据')
        if c.get('planning_module')=='NATIVE':
            errors.append('native_ompl_budget_s：仅 PYTHON 规划流程支持（planning_module=NATIVE 的复合请求未实现）')
    # Optional: absent keeps the per-request budgets; present sets one absolute
    # planning deadline from prepare_plan entry, and expiry is budget exhaustion.
    deadline=c.get('planning_deadline_s')
    if deadline is not None and (isinstance(deadline,bool) or not isinstance(deadline,(int,float))
                                 or not math.isfinite(deadline) or not 0<deadline<=3600):
        errors.append('planning_deadline_s：可选的统一规划截止时间（秒），0 < s ≤ 3600；不填则保持各请求原有预算')
    # Step-4 mechanism switches: optional booleans, absent or false keeps the current behaviour.
    for key in STEP4_SWITCHES:
        if key in c and type(c[key]) is not bool:
            errors.append(f'{key}：可选开关，true 或 false（缺省 false，保持现行为）')
    if c.get('collision_check_mode','EXACT') not in ('EXACT','TWO_LEVEL'):
        errors.append('collision_check_mode：EXACT（缺省）或 TWO_LEVEL（保守代理只证明通过，其余仍由精确检查决定）')
    if c.get('planning_module','PYTHON') not in ('PYTHON','NATIVE'):
        errors.append('planning_module：PYTHON（缺省）或 NATIVE（复合原生请求、worker 内 prepare、二进制帧；结论由原 Python 流程回放给出）')
    # 2026-11-22 latency options (PYTHON flow only; absent keeps the current behaviour).
    if c.get('sent_path_probe','SENT_GRID') not in ('SENT_GRID','GEOMETRIC'):
        errors.append('sent_path_probe：SENT_GRID（缺省，含每个 5 ms 发送点）或 GEOMETRIC（按节点区间解析速度上界取检查点，相邻检查状态任一物体对位移仍 ≤ 1 mm）')
    for key in ('parallel_screening','concurrent_audits','native_ik_batch','compact_requests','binary_transport'):
        if key in c and type(c[key]) is not bool:errors.append(f'{key}：可选开关，true 或 false（缺省 false）')
    if c.get('solution_path_check','PIPELINE') not in ('PIPELINE','CLEARANCE_PREDICATE'):
        errors.append('solution_path_check：PIPELINE（缺省，MoveIt 管线对规划器返回点做网格相交 + 间隙判据）或 CLEARANCE_PREDICATE（同一批点只做间隙判据；正间隙在包含网格的凸体上已蕴含不相交）')
    lane_clients=c.get('lane_clients',1)
    if type(lane_clients)!=int or not 1<=lane_clients<=3:
        errors.append('lane_clients：每条候选车道使用的原生进程数，1 至 3（缺省 1；>1 时同一候选内互不依赖的段并行规划）')
    if c.get('planning_module')=='NATIVE' and (c.get('sent_path_probe','SENT_GRID')!='SENT_GRID' or c.get('parallel_screening')
                                               or c.get('concurrent_audits') or lane_clients!=1
                                               or c.get('solution_path_check','PIPELINE')!='PIPELINE' or c.get('native_ik_batch')):
        errors.append('sent_path_probe / parallel_screening / concurrent_audits / lane_clients / solution_path_check：仅 PYTHON 规划流程支持（planning_module=NATIVE 未实现）')
    # 2-second planning step 1 (2026-11-22, scheduling only; absent keeps the current behaviour).
    for key in ('pipelined_audits','early_scene_upload','process_geometry_ref','native_lin_ladder','structured_plan_digest','binary_plan_record','early_holding_postcheck','workspace_pair_cache'):
        if key in c and type(c[key]) is not bool:errors.append(f'{key}：可选开关，true 或 false（缺省 false）')
    mode=c.get('pipelined_audit_mode','RESERVED_WORKERS')
    if mode not in ('RESERVED_WORKERS','IDLE_SPECULATIVE_WORKERS'):
        errors.append('pipelined_audit_mode：RESERVED_WORKERS（缺省，固定一部分进程只做审计）或 IDLE_SPECULATIVE_WORKERS'
                      '（不划出进程；审计块在非首位候选车道的进程上于请求间隙执行，推测性规划让路）')
    if 'pipelined_audit_mode' in c and not c.get('pipelined_audits'):
        errors.append('pipelined_audit_mode：需要 pipelined_audits')
    if c.get('pipelined_audits') and mode=='IDLE_SPECULATIVE_WORKERS':
        if 'pipelined_audit_workers' in c:
            errors.append('pipelined_audit_workers：IDLE_SPECULATIVE_WORKERS 不保留审计进程，不能同时设置')
        if c.get('planning_module')=='NATIVE':
            errors.append('pipelined_audits：仅 PYTHON 规划流程支持')
    elif c.get('pipelined_audits'):
        reserved=c.get('pipelined_audit_workers')
        if type(reserved)!=int or reserved<1:
            errors.append('pipelined_audit_workers：pipelined_audits 打开时为审计保留的原生进程数（≥ 1，须少于检查进程总数）')
        if c.get('planning_module')=='NATIVE' or int(c.get('parallel_candidate_lanes',1))<1:
            errors.append('pipelined_audits：仅 PYTHON 规划流程支持')
    if c.get('planning_module')=='NATIVE' and (c.get('process_geometry_ref') or c.get('native_lin_ladder') or c.get('workspace_pair_cache')):
        errors.append('process_geometry_ref / native_lin_ladder / workspace_pair_cache：仅 PYTHON 规划流程支持')
    if c.get('planning_module')=='NATIVE' and c.get('structured_plan_digest'):
        errors.append('structured_plan_digest：仅 PYTHON 规划流程支持（NATIVE 使用规范二进制摘要）')
    if c.get('adaptive_constraints','PER_GROUP') not in ('PER_GROUP','PER_BODY'):
        errors.append('adaptive_constraints：PER_GROUP（缺省，按刚体组的间隙下界与最大力臂）或 PER_BODY（近距离物体对 / 实体各用自己的下界与力臂，远处保留组约束）')
    if c.get('adaptive_constraints','PER_GROUP')=='PER_BODY' and (c.get('sent_path_audit') not in ('ADAPTIVE','ADAPTIVE_CONTACT') or c.get('planning_module')=='NATIVE'):
        errors.append('adaptive_constraints：PER_BODY 需要 sent_path_audit=ADAPTIVE 或 ADAPTIVE_CONTACT（PYTHON 规划流程）')
    # User decisions B and E (2026-11-23), PYTHON flow only; absent keeps the current behaviour.
    if 'defer_pilz_solution_check' in c and type(c['defer_pilz_solution_check']) is not bool:
        errors.append('defer_pilz_solution_check：可选开关，true 或 false（缺省 false）')
    if c.get('defer_pilz_solution_check') and c.get('planning_module')=='NATIVE':
        errors.append('defer_pilz_solution_check：仅 PYTHON 规划流程支持（Pilz 返回路点的间隙判据改由必做的发送路径审计检查）')
    if 'pilz_sent_path_ompl_fallback' in c:
        # User decision 2026-11-25 ("加 OMPL 回退"): a Pilz PTP approach whose
        # sent path the audit rejects goes to the unchanged OMPL fallback of
        # the same goal before the next IK branch.
        if type(c['pilz_sent_path_ompl_fallback']) is not bool:
            errors.append('pilz_sent_path_ompl_fallback：可选开关，true 或 false（缺省 false）')
        elif c['pilz_sent_path_ompl_fallback'] and not c.get('defer_pilz_solution_check'):
            errors.append('pilz_sent_path_ompl_fallback：需要 defer_pilz_solution_check=true（否则 Pilz 路点未通过时本来就转 OMPL）')
        if c['pilz_sent_path_ompl_fallback'] and c.get('planning_module')=='NATIVE':
            errors.append('pilz_sent_path_ompl_fallback：仅 PYTHON 规划流程支持')
    if c.get('deadline_ompl_fallback','CAPPED_BY_DEADLINE') not in ('CAPPED_BY_DEADLINE','NOT_STARTED_WHEN_SHORT'):
        errors.append('deadline_ompl_fallback：CAPPED_BY_DEADLINE（缺省，OMPL 回退的预算受统一截止时间截短）或 NOT_STARTED_WHEN_SHORT'
                      '（剩余时间不足 OMPL 预算时不启动回退，该候选记为未搜索，不算不可行）')
    if c.get('deadline_ompl_fallback','CAPPED_BY_DEADLINE')!='CAPPED_BY_DEADLINE' and c.get('planning_module')=='NATIVE':
        errors.append('deadline_ompl_fallback：仅 PYTHON 规划流程支持')
    lin=c.get('lin_time_parametrization')
    if lin is not None:
        # User decision D (2026-11-23): retime Pilz LIN paths along the line; limits are only read.
        if not isinstance(lin,dict) or lin.get('mode') not in ('PILZ_PROFILE','PATH_TOPP') or not str(lin.get('basis','')).strip():
            errors.append('lin_time_parametrization：{mode: PILZ_PROFILE（缺省行为）或 PATH_TOPP（沿直线时间最优参数化，关节与 Pilz 笛卡尔限值不变）, basis: 依据}')
        else:
            f=lin.get('joint_limit_fraction',.95)
            if type(f) not in (int,float) or not .5<=f<=1:
                errors.append('lin_time_parametrization.joint_limit_fraction：0.5 至 1（只能收紧执行关节限值，缺省 0.95）')
        if c.get('planning_module')=='NATIVE':
            errors.append('lin_time_parametrization：仅 PYTHON 规划流程支持')
    # Candidate strategy switches (user decisions 2026-11-25), PYTHON flow only; absent keeps the current behaviour.
    from .candidate_strategy import config_issues as strategy_issues
    errors+=strategy_issues(c)
    twins=c.get('ik_winding_twins')
    if twins is not None:
        # User decision 2026-11-25 ("先去重分支"): IK solutions equal modulo 2*pi on every joint
        # (J6 +-360 deg windings) are one arm configuration; later windings move behind the distinct ones.
        if not isinstance(twins,dict) or twins.get('mode') not in ('KEEP_ORDER','AFTER_DISTINCT_CONFIGURATIONS') or not str(twins.get('basis','')).strip():
            errors.append('ik_winding_twins：{mode: KEEP_ORDER（缺省行为）或 AFTER_DISTINCT_CONFIGURATIONS（各关节模 2π 相同的绕圈孪生解排在全部不同构型之后，不删除）, basis: 依据}')
        if c.get('planning_module')=='NATIVE':
            errors.append('ik_winding_twins：仅 PYTHON 规划流程支持')
    if 'analytic_send_slowdown' in c and type(c['analytic_send_slowdown']) is not bool:
        errors.append('analytic_send_slowdown：可选开关，true 或 false（缺省 false）')
    if c.get('analytic_send_slowdown') and c.get('planning_module')=='NATIVE':
        errors.append('analytic_send_slowdown：仅 PYTHON 规划流程支持（同一放慢倍数搜索，跳过必然越限的倍数）')
    if 'digest_memo' in c and type(c['digest_memo']) is not bool:
        errors.append('digest_memo：可选开关，true 或 false（缺省 false）')
    if c.get('digest_memo') and (not c.get('structured_plan_digest') or c.get('planning_module')=='NATIVE'):
        errors.append('digest_memo：需要 structured_plan_digest（PYTHON 规划流程；只用于签发，检查一律重新读取全部内容）')
    if c.get('adaptive_rule_transforms','FULL_UPDATE') not in ('FULL_UPDATE','LINK_TRANSFORMS_ONLY'):
        errors.append('adaptive_rule_transforms：FULL_UPDATE（缺省）或 LINK_TRANSFORMS_ONLY（自适应审计被跳过的采样点只更新连杆变换再判单箱规则；规则只读连杆变换，结果相同）')
    if c.get('adaptive_rule_transforms','FULL_UPDATE')=='LINK_TRANSFORMS_ONLY' and (c.get('sent_path_audit') not in ('ADAPTIVE','ADAPTIVE_CONTACT') or c.get('planning_module')=='NATIVE'):
        errors.append('adaptive_rule_transforms：LINK_TRANSFORMS_ONLY 需要 sent_path_audit=ADAPTIVE 或 ADAPTIVE_CONTACT（PYTHON 规划流程）')
    if c.get('audit_chunking','SUBSETS') not in ('SUBSETS','WORK_QUEUE'):
        errors.append('audit_chunking：SUBSETS（缺省，每段分一组进程、每进程一块）或 WORK_QUEUE（各段切成更小的连续块排队，空闲进程取下一块；只改调度）')
    if c.get('audit_chunking','SUBSETS')=='WORK_QUEUE' and (not c.get('concurrent_audits') or c.get('planning_module')=='NATIVE'):
        errors.append('audit_chunking：WORK_QUEUE 需要 concurrent_audits（PYTHON 规划流程）')
    if c.get('prepare_implementation','PYTHON') not in ('PYTHON','NATIVE_LIBRARY'):
        errors.append('prepare_implementation：PYTHON（缺省）或 NATIVE_LIBRARY（Python 流程调用 worker 同一份 C++ prepare，libm710_prepare.so）')
    if c.get('prepare_implementation','PYTHON')=='NATIVE_LIBRARY' and (c.get('prepare_processes') or c.get('planning_module')=='NATIVE'):
        errors.append('prepare_implementation：NATIVE_LIBRARY 不与 prepare_processes 同时设置，且仅 PYTHON 规划流程支持')
    if 'prepare_processes' in c and (type(c['prepare_processes']) is not int or not 1<=c['prepare_processes']<=8 or c.get('planning_module')=='NATIVE'):
        errors.append('prepare_processes：可选，发送网格准备（prepare）在常驻子进程中执行的进程数，1 至 8（PYTHON 规划流程；缺省在规划线程内执行）')
    if 'python_switch_interval_s' in c:
        v=c['python_switch_interval_s']
        if type(v) not in (int,float) or not np.isfinite(v) or not 1e-4<=v<=5e-3:
            errors.append('python_switch_interval_s：可选，规划期间的解释器线程切换间隔（秒），1e-4 至 5e-3（缺省不设，保持解释器缺省 5e-3）')
    if c.get('early_scene_upload') and not c.get('scene_handles'):
        errors.append('early_scene_upload：需要 scene_handles')
    if c.get('sent_path_audit')=='ADAPTIVE_CONTACT' and c.get('planning_module')=='NATIVE':
        errors.append('sent_path_audit：ADAPTIVE_CONTACT 仅 PYTHON 规划流程支持（planning_module=NATIVE 未实现）')
    if c.get('sent_path_audit','SAMPLED') not in ('SAMPLED','ADAPTIVE','ADAPTIVE_CONTACT'):
        errors.append('sent_path_audit：SAMPLED（缺省）、ADAPTIVE（只跳过由逐对位移上界证明通过的采样点；接触段和带支撑段始终 SAMPLED）或 ADAPTIVE_CONTACT（接触段和带支撑段也跳过已证明的物体对距离，接触/支撑规则逐点精确计算）')
    topology=c.get('topology',{})
    if topology.get('mode')!='COMMON_GRIPPER_VALVE' or not topology.get('connected_cup_ids') or not topology.get('field_basis'):
        errors.append('topology：总阀下实际连通杯及物理封堵杯列表')
    vacuum=c.get('vacuum',{})
    for k in REQUIRED_VACUUM:
        if vacuum.get(k) is None or vacuum.get(k)=='' or vacuum.get(k)==[]:errors.append('vacuum.'+k)
    if not any(e.startswith('vacuum.') for e in errors):
        try:
            if not -100<vacuum['grip_below_kpa']<=vacuum['hold_below_kpa']<vacuum['release_above_kpa']<=10:raise ValueError()
            for k in ('grip_dwell_s','release_dwell_s','grip_timeout_s','release_timeout_s','max_age_s'):
                if not math.isfinite(vacuum[k]) or vacuum[k]<=0:raise ValueError()
            if vacuum['grip_dwell_s']>=vacuum['grip_timeout_s'] or vacuum['release_dwell_s']>=vacuum['release_timeout_s']:raise ValueError()
            for seq in ('establish_commands','release_commands'):
                if not all(type(x)==int and x in {0x12,0x13,0x51,0x52,0x53,0x54,0x55,0x56} for x in vacuum[seq]):raise ValueError()
        except (ValueError,TypeError):errors.append('真空压力、时间或阀门指令参数非法')
        else:
            # User-approved pressure gate: a file may only tighten these.
            try:
                looser=[k for k in ('grip_dwell_s','release_dwell_s') if vacuum[k]<VACUUM_MIN_DWELL_S]
                looser+=[k for k in ('grip_timeout_s','release_timeout_s') if vacuum[k]>VACUUM_MAX_TIMEOUT_S]
                looser+=['max_age_s'] if vacuum['max_age_s']>VACUUM_MAX_AGE_S else []
            except TypeError:looser=['vacuum']
            if looser:
                errors.append('vacuum：'+'、'.join(looser)+f' 比用户批准值宽松；停留须 ≥ {VACUUM_MIN_DWELL_S:g} s，'
                              f'抓取/释放超时须 ≤ {VACUUM_MAX_TIMEOUT_S:g} s，压力数据年龄须 ≤ {VACUUM_MAX_AGE_S:g} s（只能收紧）')
    return errors


def load(path, code_defaults=True):
    path=Path(path).resolve();c=json.loads(path.read_text())
    if c.get('schema')!='eco65_physical_single_carton_v1':raise ValueError('EXPLICIT_SINGLE_CARTON_CONFIG_REQUIRED')
    if not c.get('site_scene'):raise ValueError('REAL_SITE_SCENE_REQUIRED_USE_CURRENT_SINGLE_CARTON_CONFIG')
    # User decision 2026-11-25: optimisation switches default ON in code; a
    # value written in the file still wins (optimization_defaults.apply).
    # code_defaults=False reproduces the pre-2026-11-25 defaults (tests that pin
    # the earlier behaviour with fake workers); the field always applies them.
    from .optimization_defaults import apply as apply_code_defaults
    if code_defaults:apply_code_defaults(c)
    else:c['code_defaults']=dict(applied=[],skipped='CODE_DEFAULTS_DISABLED_BY_CALLER')
    c['config_path']=str(path)
    for k in ('execution_config','camera_extrinsics','record_root','site_scene','device_config'):
        if c.get(k):c[k]=str((ROOT/c[k]).resolve())
    for k in ('dino_directory','sam_directory'):c['detector'][k]=str((ROOT/c['detector'][k]).resolve())
    c['execution']=load_execution(c['execution_config'])
    c['execution']['record_root']=c['record_root']
    if c.get('device_config') and Path(c['device_config']).is_file():
        device=json.loads(Path(c['device_config']).read_text())
        c['execution']['host']=device.get('robot_host',c['execution']['host'])
        c['execution']['port']=device.get('robot_port',c['execution']['port'])
        c['serial_port']=device.get('serial_port',c['serial_port'])
        c['robot_network']=device.get('robot_network',c.get('robot_network',{}))
        c.setdefault('camera',{})['serial']=device.get('camera_serial',c.get('camera',{}).get('serial',''))
    # Remote GPU host/port/user/key live in a separate ignored local file, so
    # connection details never enter the repository or the robot device config.
    remote_file=c['detector'].get('remote_config')
    if remote_file:
        remote_file=c['detector']['remote_config']=str((ROOT/remote_file).resolve())
        if Path(remote_file).is_file():
            c['detector']['remote']=dict(c['detector'].get('remote') or {},**json.loads(Path(remote_file).read_text()))
    if c.get('site_scene'):
        from .site_scene import load_site
        c['site']=load_site(c['site_scene'])
    # Preserve separate measured tool mass and the user-requested controller setting.
    # A loaded task needs its own explicit basis; an empty-test basis does not qualify.
    c['execution']['controller_payload_kg']=c.get('controller_payload_kg')
    c['execution']['controller_payload_basis']=c.get('controller_loaded_payload_basis')
    paths=[c['config_path'],c['execution_config'],c['site_scene'],c['camera_extrinsics'],
           c['execution']['installation'],c['execution']['mapping'],c.get('device_config')]
    c['loaded_file_sha256']={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths if p and Path(p).is_file()}
    from .pointcloud_artifacts import load_manifest
    artifacts=load_manifest(c['site'])
    if artifacts:c['loaded_file_sha256'][artifacts['path']]=artifacts['sha256']
    return c
