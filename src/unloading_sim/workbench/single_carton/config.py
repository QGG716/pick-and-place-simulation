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
    for name,check in (('plan_validity',plan_validity),('execution_lock',execution_lock),('tool_robot_clearance',tool_robot_clearance),('wrist_branch_limits',wrist_branch_limits)):
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
    if c.get('sent_path_audit','SAMPLED') not in ('SAMPLED','ADAPTIVE'):
        errors.append('sent_path_audit：SAMPLED（缺省）或 ADAPTIVE（只跳过由逐对位移上界证明通过的采样点；接触段和带支撑段始终 SAMPLED）')
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


def load(path):
    path=Path(path).resolve();c=json.loads(path.read_text())
    if c.get('schema')!='eco65_physical_single_carton_v1':raise ValueError('EXPLICIT_SINGLE_CARTON_CONFIG_REQUIRED')
    if not c.get('site_scene'):raise ValueError('REAL_SITE_SCENE_REQUIRED_USE_CURRENT_SINGLE_CARTON_CONFIG')
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
