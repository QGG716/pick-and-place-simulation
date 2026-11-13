"""Explicit field parameters; missing values are reported, never guessed."""
import json
from pathlib import Path
import math
import hashlib
from ..controlled.planner import ROOT,load_config as load_execution

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
    live joints within 0.01 deg of q_start with the robot enabled and no alarm.
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
                              'FRESH_LIVE_JOINTS_WITHIN_0_01_DEG_OF_Q_START_ENABLED_NO_ALARM'])
    raise ValueError('PRE_EXECUTE_RECHECK_POLICY_INVALID: ENABLED、TARGET_ONLY 或 DISABLED_BY_USER')


def start_joint_check(c):
    """Whether the cycle compares fresh live joints with the frozen q_start.

    ENABLED (default when absent): within 0.01 deg, else START_CHANGED_REPLAN.
    DISABLED_BY_USER needs a recorded basis; the live sample must still be
    fresh, enabled, alarm-free and stationary, and every later segment keeps
    the arrival / tracking / feedback / stop supervision.
    """
    value=c.get('start_joint_check','ENABLED')
    if value=='ENABLED':return dict(enabled=True)
    if value=='DISABLED_BY_USER':
        basis=c.get('start_joint_check_basis')
        if not isinstance(basis,str) or not basis.strip():
            raise ValueError('START_JOINT_CHECK_DISABLE_BASIS_REQUIRED: 取消起点核对须记录用户要求的依据')
        return dict(enabled=False,status='START_JOINT_CHECK_DISABLED_BY_USER',basis=basis)
    raise ValueError('START_JOINT_CHECK_POLICY_INVALID: ENABLED 或 DISABLED_BY_USER')


def issues(c):
    errors=[]
    try:release_policy_evidence(c)
    except ValueError as e:errors.append('release_policy：'+str(e))
    try:pre_execute_visual_recheck(c)
    except ValueError as e:errors.append('pre_execute_visual_recheck：'+str(e))
    try:start_joint_check(c)
    except ValueError as e:errors.append('start_joint_check：'+str(e))
    geometry=c.get('geometry',{})
    prior=geometry.get('prior_size_m')
    if (not isinstance(prior,list) or len(prior)!=3 or
            any(not isinstance(x,(int,float)) or not math.isfinite(x) or not .005<x<=2 for x in prior)):
        errors.append('geometry.prior_size_m：明确的现场箱型尺寸先验，单位 m')
    # The prior bounds hidden thickness in the collision envelope, so its
    # source must be recorded explicitly (tape measurement or RGB-D task id).
    basis=geometry.get('prior_measurement_basis')
    if not isinstance(basis,str) or not basis.strip():
        errors.append('geometry.prior_measurement_basis：填写箱型尺寸来源（卷尺实测或 RGB-D 实测任务编号）')
    tol=geometry.get('dimension_tolerance_m')
    if not isinstance(tol,(int,float)) or not math.isfinite(tol) or not 0<=tol<=.03:
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
    return c
