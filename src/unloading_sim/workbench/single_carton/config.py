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


def issues(c):
    errors=[]
    try:release_policy_evidence(c)
    except ValueError as e:errors.append('release_policy：'+str(e))
    geometry=c.get('geometry',{})
    prior=geometry.get('prior_size_m')
    if (not isinstance(prior,list) or len(prior)!=3 or
            any(not isinstance(x,(int,float)) or not math.isfinite(x) or not .005<x<=2 for x in prior)):
        errors.append('geometry.prior_size_m：明确的现场箱型尺寸先验，单位 m')
    tol=geometry.get('dimension_tolerance_m')
    if not isinstance(tol,(int,float)) or not math.isfinite(tol) or not 0<=tol<=.03:
        errors.append('geometry.dimension_tolerance_m：尺寸先验误差界，0 至 0.03 m')
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
