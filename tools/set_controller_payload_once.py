"""One explicitly user-authorized controller payload update. No motion, stop, enable or IO calls.

2026-11-13: the user authorized one SDK write so the controller payload matches the
single-carton setting (controller_payload_kg = 1.5 kg tool + carton mass). The current
tool frame is cloned (pose and centre of mass unchanged) under a new name with the
required payload and selected; existing tools are never modified or deleted. Before and
after states, every SDK call and stationary feedback are recorded under
outputs/controller_configuration/. The ordinary workbench never writes tool settings.

Refuses to run while another process is connected to the controller or bound to the
feedback UDP port (close the workbench first). Without --apply it only reads.
Same method as the 2026-11-11 Empty3kg setup.
"""
import argparse, ctypes, datetime, json, socket, subprocess, time
import numpy as np
from unloading_sim.workbench.controlled.planner import ROOT
from unloading_sim.workbench.controlled.transport import RealmanTransport
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.ui import load_mapping
from Robotic_Arm.rm_robot_interface import rm_frame_t
from Robotic_Arm.rm_ctypes_wrap import rm_get_current_tool_frame as get_raw_frame

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--config', default=str(ROOT/'configs/workbench/single_carton.example.json'))
parser.add_argument('--expected-tool', required=True, help='name of the tool currently selected on the controller')
parser.add_argument('--name', default='Load3p5kg', help='new tool name (< 12 bytes)')
parser.add_argument('--apply', action='store_true', help='create and select the tool; otherwise read only')
args = parser.parse_args()
single = load(args.config); c = single['execution']
payload = float(single['controller_payload_kg'])
if abs(payload-(1.5+single['carton_mass_kg'])) > 1e-9 or not 0 < payload <= 5: raise SystemExit('CONTROLLER_PAYLOAD_SETTING_INVALID')
if not 0 < len(args.name.encode()) < 12: raise SystemExit('TOOL_NAME_TOO_LONG')

# Another session on the controller (e.g. the workbench) would lose its UDP feedback.
busy = subprocess.run(['ss', '-tnpH', 'dst', f"{c['host']}:{c['port']}"], stdout=subprocess.PIPE, text=True).stdout.strip()
probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try: probe.bind(('', int(c.get('udp_port', 18089))))
except OSError: busy += '\nUDP_PORT_IN_USE'
finally: probe.close()
if busy: raise SystemExit('CONTROLLER_IN_USE_CLOSE_THE_WORKBENCH_FIRST:\n'+busy)

out = ROOT/'outputs/controller_configuration'/(f'payload-{payload:g}kg-'.replace('.', 'p')+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir(parents=True)
report = dict(operation='USER_AUTHORIZED_2026_11_13_SINGLE_CARTON_CONTROLLER_PAYLOAD', motion_commands_sent=0,
              physical_tool_mass_kg=1.5, carton_mass_kg=single['carton_mass_kg'], requested_controller_payload_kg=payload,
              basis=single['controller_loaded_payload_basis'], record_dir=str(out), sdk_calls=[])
t = RealmanTransport(load_mapping(c['mapping']), c)


def frame():
    value = rm_frame_t(); code = get_raw_frame(t.sender.handle, ctypes.byref(value))
    report['sdk_calls'].append(dict(api='rm_get_current_tool_frame', code=code, time_monotonic_s=time.monotonic()))
    if code: raise RuntimeError(f'FRAME_QUERY_FAILED:{code}')
    return value


def fields(f):
    return dict(name=f.frame_name.decode(), payload=float(f.payload),
                pose=[float(x) for x in (f.pose.position.x, f.pose.position.y, f.pose.position.z, f.pose.euler.rx, f.pose.euler.ry, f.pose.euler.rz)],
                x=float(f.x), y=float(f.y), z=float(f.z))


def save():
    (out/'record.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))


def stationary():
    begin = time.monotonic(); samples = []; last = None
    while time.monotonic()-begin < .6:
        s = t.latest; now = time.monotonic()
        if s is None or s['source'] != 'LIVE' or not 0 <= now-s['received_s'] <= .1: raise RuntimeError('FRESH_FEEDBACK_REQUIRED')
        if s['alarm']: raise RuntimeError('ROBOT_ALARM')
        if s['raw'].get('arm_current_status') != 'idle': raise RuntimeError('CONTROLLER_NOT_IDLE')
        v = np.asarray(s['velocity'], float)
        if v.shape != (6,) or not np.isfinite(v).all() or np.max(abs(v)) > np.deg2rad(.5): raise RuntimeError('ROBOT_NOT_STATIONARY')
        if s['sequence'] != last: samples.append(s); last = s['sequence']
        time.sleep(.01)
    if len(samples) < 30: raise RuntimeError('INSUFFICIENT_STATIONARY_FEEDBACK')
    return dict(samples=len(samples), duration_s=samples[-1]['received_s']-samples[0]['received_s'], first=samples[0], last=samples[-1])


try:
    report['capabilities'] = t.connect(c['host'], c['port'])
    assert t._send_disabled.is_set(), 'Sending must remain disabled throughout configuration'
    report['stationary_before'] = stationary()
    original = frame(); report['before'] = fields(original); report['before_raw_bytes_hex'] = bytes(original).hex()
    if report['before']['name'] != args.expected_tool: raise RuntimeError('CURRENT_TOOL_CHANGED:'+report['before']['name'])
    if not np.isfinite(list(report['before']['pose'])+[report['before'][k] for k in ('payload', 'x', 'y', 'z')]).all():
        raise RuntimeError('INVALID_TOOL_CONFIGURATION')
    names = t.sender.rm_get_total_tool_frame(); report['tool_inventory_before'] = names; save()
    if names['return_code']: raise RuntimeError('TOOL_INVENTORY_QUERY_FAILED')
    if not args.apply:
        report['status'] = 'READ_ONLY'; save(); print(json.dumps(dict(before=report['before'], tools=names['tool_names']), ensure_ascii=False)); raise SystemExit(0)
    existing = set(names['tool_names'])
    if args.name in existing: raise RuntimeError('TOOL_NAME_EXISTS_NOT_OVERWRITTEN')
    if len(existing) >= 10: raise RuntimeError('NO_FREE_TOOL_SLOT_NO_EXISTING_TOOLS_DELETED')
    target = rm_frame_t.from_buffer_copy(bytes(original)); target.frame_name = args.name.encode(); target.payload = payload
    report['requested'] = fields(target); save()
    if fields(frame()) != report['before']: raise RuntimeError('CURRENT_TOOL_CHANGED_BEFORE_WRITE')
    code = t.sender.rm_set_manual_tool_frame(target)
    report['sdk_calls'].append(dict(api='rm_set_manual_tool_frame', tool=args.name, code=code, time_monotonic_s=time.monotonic())); save()
    if code: raise RuntimeError(f'CREATE_TOOL_REJECTED:{code}')
    current = frame(); report['after_create'] = fields(current); save()
    if current.frame_name != target.frame_name:
        code = t.sender.rm_change_tool_frame(args.name)
        report['sdk_calls'].append(dict(api='rm_change_tool_frame', tool=args.name, code=code, time_monotonic_s=time.monotonic())); save()
        if code: raise RuntimeError(f'TOOL_SELECTION_REJECTED:{code}')
    report['after'] = fields(frame())
    if report['after']['name'] != args.name or abs(report['after']['payload']-payload) > 1e-4 or any(
            abs(a-b) > 1e-6 for a, b in zip(report['after']['pose']+[report['after'][k] for k in 'xyz'],
                                              report['before']['pose']+[report['before'][k] for k in 'xyz'])):
        raise RuntimeError('PAYLOAD_OR_PRESERVED_FRAME_READBACK_MISMATCH')
    report['tool_inventory_after'] = t.sender.rm_get_total_tool_frame()
    report['stationary_after'] = stationary()
    assert t._send_disabled.is_set()
    report['status'] = 'PASS'; save()
    print(json.dumps({k: report[k] for k in ('status', 'before', 'after', 'motion_commands_sent', 'record_dir')}, ensure_ascii=False), flush=True)
except BaseException as exc:
    if report.get('status') != 'READ_ONLY': report['status'] = 'FAILED'; report['error'] = repr(exc)
    save(); raise
finally:
    t.disconnect(); report['disconnected'] = True; save()
