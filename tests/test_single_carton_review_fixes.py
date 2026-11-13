"""Regressions for the ECO65-B field-workbench review fixes.

Fake IO, fake clocks and local socket pairs only: no robot, serial port,
network or SDK is opened. Physical results remain NOT_RUN.
"""
from collections import deque
from dataclasses import replace
import json
import multiprocessing as mp
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.workbench.backend import SampleHistory
from unloading_sim.workbench.contracts import Stamp, DetectionResult
from unloading_sim.workbench.controlled.engine import MonitorConfig, field_monitor_config
from unloading_sim.workbench.controlled.planner import load_config
from unloading_sim.workbench.controlled.service import ExecutionService
from unloading_sim.workbench.controlled.trajectory import prepare, Authority
from unloading_sim.workbench.controlled.transport import StopChannel
from unloading_sim.workbench.devices import GaugeReading
from unloading_sim.workbench.single_carton.config import load, issues
from unloading_sim.workbench.single_carton.live import SingleBackend
from unloading_sim.workbench.single_carton.partial_geometry import recover_partial
from unloading_sim.workbench.single_carton.runtime import TaskManager
from unloading_sim.workbench.single_carton.site_scene import build_scene
from unloading_sim.workbench.single_carton.vacuum import VacuumOwner, valve_frame
from test_controlled_execution import Harness, LIMITS
from test_single_carton_cycle import attached
from test_single_carton_geometry import plane
from test_single_carton_observation import observation
from test_single_carton_partial_site import small_site

ROOT = Path(__file__).resolve().parents[1]
REPLY_TRUE = b'{"command":"set_arm_stop","arm_stop":true}\r\n'
REPLY_FALSE = b'{"command":"set_arm_stop","arm_stop":false}\r\n'


# --- Segment start continuity (review item 4) -------------------------------

def succeeded(h):
    h.start();h.finish_sends()
    for _ in range(200):
        if h.engine.state=='SUCCEEDED':return
        h.tick()
    raise AssertionError('segment did not succeed')


def segment(first):
    points=[dict(t=0.,q=[first]*6,v=[0.]*6,a=[0.]*6),dict(t=.5,q=[first+.001]*6,v=[0.]*6,a=[0.]*6)]
    return prepare(points,LIMITS,time_scale=1)


def test_chained_segment_starts_from_held_command_within_arrival_tolerance():
    h=Harness();succeeded(h)
    held=h.engine.hold_command[0];rest=held+np.deg2rad(.3)  # loaded rest error
    task=h.engine.submit(segment(held),h.sample([rest]*6))
    accepted=[e for e in h.logs if e['kind']=='accepted'][-1]
    assert task and accepted['start_rule']=='CONTINUES_HELD_COMMAND_OF_PREVIOUS_SUCCEEDED_SEGMENT'
    assert accepted['start_limit_rad']==MonitorConfig().arrival_rad


@pytest.mark.parametrize('case',['beyond_arrival','different_first_point','after_stop','after_reset'])
def test_chained_start_rule_is_refused_outside_its_preconditions(case):
    h=Harness();succeeded(h);held=h.engine.hold_command[0];first=held;rest=held+np.deg2rad(.3)
    if case=='beyond_arrival':rest=held+np.deg2rad(.6)
    if case=='different_first_point':first=held+1e-5;rest=first+np.deg2rad(.3)
    if case=='after_stop':h.engine.stop('OPERATOR_SOFTWARE_STOP')
    if case=='after_reset':h.engine.reset_continuity()
    with pytest.raises(ValueError,match='START_CHANGED_REPLAN_REQUIRED'):h.engine.submit(segment(first),h.sample([rest]*6))
    assert h.engine.state=='SUCCEEDED'


def test_single_carton_return_rest_error_within_arrival_reaches_release():
    c,t,r,v,events,_=attached();r.finish();c.tick();assert c.state=='RETURN_LOADED'
    r.finish();r.q=[np.deg2rad(.3)]+[0.]*5;c.tick()
    assert c.state=='RELEASE' and v.commands==['grip','release'] and c.status()['cycle_release_command_sent']


def test_single_carton_return_beyond_arrival_keeps_load_without_release():
    c,t,r,v,events,_=attached();r.finish();c.tick()
    r.finish();r.q=[np.deg2rad(.6)]+[0.]*5;c.tick()
    assert c.state=='STOPPING' and c.reason=='RETURN_START_NOT_CONFIRMED' and v.commands==['grip'] and v.holding
    assert not c.status()['cycle_release_command_sent']


def test_stop_during_release_records_that_the_cycle_release_was_sent():
    c,t,r,v,events,_=attached();r.finish();c.tick();r.finish();c.tick();assert c.state=='RELEASE'
    c.stop(cancelled=True)
    stopping=[e for e in events if e.get('state')=='STOPPING'][-1]
    assert stopping['cycle_release_command_sent'] and stopping['automatic_release_sent'] is False
    assert c.status()['cycle_release_command_sent'] and v.commands==['grip','release']


# --- Send timing and stalled stop (review items 7, low 3) -------------------

def test_send_deadline_is_judged_at_the_vendor_call_not_tick_start():
    h=Harness();h.start();due=h.engine.start_time;h.clock.now=due
    def slow_record(event):
        h.logs.append(event)
        if event['kind']=='feedback':h.clock.now+=.003  # work before the send
    h.engine.record=slow_record
    h.engine.tick(h.sample())
    assert h.io.sent==[] and h.engine.state=='STOPPING' and h.engine.reason=='SEND_DEADLINE_MISSED'


def test_stalled_vendor_call_latches_stop_unconfirmed_and_reasserts_stop_on_return():
    h=Harness();h.start()
    def stalled(q):
        h.engine.stop('SEND_THREAD_STALLED')   # watchdog while the call blocks
        h.engine.expire_stalled_stop();assert h.engine.state=='STOPPING'  # not yet timed out
        h.clock.now+=h.engine.c.stop_timeout_s+.1
        h.engine.expire_stalled_stop()           # watchdog latch, timed thread still blocked
        return 0
    h.io.send=stalled
    for _ in range(10):
        h.tick()
        if h.engine.state=='STOP_UNCONFIRMED':break
    assert h.engine.state=='STOP_UNCONFIRMED' and h.io.stops==2
    assert [e for e in h.logs if e['kind']=='result'][-1]['timed_thread_stalled'] is True
    h.tick();assert h.engine.state=='STOP_UNCONFIRMED'


# --- Field configuration cannot loosen reviewed gates (low 5) ---------------

def test_field_monitor_config_only_tightens_reviewed_defaults():
    assert field_monitor_config({})==MonitorConfig()
    assert field_monitor_config(dict(arrival_rad=.005,dwell_s=.6)).arrival_rad==.005
    for looser in (dict(arrival_rad=.02),dict(stop_timeout_s=3.),dict(dwell_s=.2),dict(feedback_timeout_s=.2)):
        with pytest.raises(ValueError,match='LOOSER_THAN_REVIEWED'):field_monitor_config(looser)
    with pytest.raises(ValueError,match='UNKNOWN_MONITOR_FIELDS'):field_monitor_config(dict(arival_rad=.001))


def test_controlled_config_file_with_looser_monitor_is_rejected(tmp_path):
    raw=json.loads((ROOT/'configs/workbench/controlled.example.json').read_text())
    load_config(ROOT/'configs/workbench/controlled.example.json')
    raw['monitor']=dict(tracking_rad=.1);path=tmp_path/'controlled.json';path.write_text(json.dumps(raw))
    with pytest.raises(ValueError,match='MONITOR_THRESHOLDS_LOOSER_THAN_REVIEWED:tracking_rad'):load_config(path)


def test_vacuum_timing_cannot_be_loosened_by_the_field_file():
    c=load(ROOT/'configs/workbench/single_carton.example.json')
    assert not [e for e in issues(c) if e.startswith('vacuum：')]
    for key,value in (('grip_timeout_s',5.),('release_dwell_s',.5),('max_age_s',4.)):
        changed=dict(c,vacuum=dict(c['vacuum'],**{key:value}))
        assert any(e.startswith('vacuum：') and key in e for e in issues(changed))
    tighter=dict(c,vacuum=dict(c['vacuum'],grip_timeout_s=2.5,grip_dwell_s=1.5,max_age_s=2.))
    assert not [e for e in issues(tighter) if e.startswith('vacuum：')]


# --- Software stop channel (low 1, 2) ----------------------------------------

def pairs(monkeypatch,count):
    made=[socket.socketpair() for _ in range(count)];local=iter(a for a,_ in made)
    monkeypatch.setattr(socket,'create_connection',lambda *args,**kwargs:next(local))
    return [b for _,b in made]


def answer(remote,reply,delay=0.):
    def run():
        remote.recv(4096);time.sleep(delay);remote.sendall(reply)
    thread=threading.Thread(target=run,daemon=True);thread.start();return thread


def test_stop_reply_deadline_is_half_a_second_for_the_whole_reply(monkeypatch):
    remote,=pairs(monkeypatch,1);channel=StopChannel('fake',8080)
    answer(remote,REPLY_TRUE,delay=.3)
    assert channel.stop()==0  # previously one recv timed out after 0.25 s
    channel.close()


def test_late_reply_to_a_timed_out_stop_never_confirms_the_next_stop(monkeypatch):
    first,second=pairs(monkeypatch,2);channel=StopChannel('fake',8080)
    started=time.monotonic()
    with pytest.raises(TimeoutError):channel.stop()
    assert .45<=time.monotonic()-started<1.
    try:first.sendall(REPLY_TRUE)  # the old request's late success
    except OSError:pass
    answer(second,REPLY_FALSE)
    assert channel.stop()==1 and channel.reconnects==1
    channel.close()


def test_bytes_received_before_a_stop_request_are_discarded(monkeypatch):
    remote,=pairs(monkeypatch,1);channel=StopChannel('fake',8080)
    remote.sendall(REPLY_TRUE);time.sleep(.05)  # stale success left in the socket
    answer(remote,REPLY_FALSE)
    assert channel.stop()==1 and channel.discarded_bytes==len(REPLY_TRUE)
    channel.close()


# --- Execution service (low 4, record 3) --------------------------------------

def test_ipc_timeout_advances_cancel_epoch_so_a_pending_submit_cannot_start():
    class SilentPipe:
        def send(self,message):pass
        def poll(self,timeout):return False
    service=ExecutionService(None,{},Authority(),lambda:{});service.process=object()
    service.stop_event=threading.Event();service.cancel_epoch=mp.Value('Q',0);service.pipe=SilentPipe()
    with pytest.raises(RuntimeError,match='TIMEOUT'):service._call('feedback')
    assert service.cancel_epoch.value==1 and service.stop_event.is_set() and service._poisoned


def test_event_record_open_failure_rejects_submit_before_any_send(tmp_path,monkeypatch):
    import unloading_sim.workbench.controlled.service as svc
    made=[]
    class FakeTransport:
        def __init__(self,mapping,config):
            self.connected=False;self.latest=None;self.capabilities={};self.sender=self;self.sent=0;made.append(self)
        def connect(self,host,port):
            self.connected=True
            self.latest=dict(source='LIVE',sequence=1,received_s=time.monotonic(),q=[0.]*6,velocity=[0.]*6,enabled=True,alarm=False)
        def rm_get_current_tool_frame(self):return 0,dict(name='Fake_tip',payload=1.5)
        def check_limits(self,prepared):pass
        def arm(self):pass
        def send(self,q):self.sent+=1;return 0
        def request_stop(self):pass
        def stop_result(self):return 0
        def disconnect(self):self.connected=False
    def refuse(*args,**kwargs):raise PermissionError('read-only record root')
    monkeypatch.setattr(svc,'RealmanTransport',FakeTransport);monkeypatch.setattr(svc,'open',refuse,raising=False)
    parent,child=mp.Pipe();stop=threading.Event()
    config=dict(record_root=str(tmp_path),tool_mass_kg=1.5,controller_payload_kg=1.5)
    thread=threading.Thread(target=svc.worker,args=(child,None,config,stop),daemon=True);thread.start()
    def call(ident,op,arg=None):
        parent.send((ident,op,arg));assert parent.poll(3);_,ok,value=parent.recv();return ok,value
    try:
        assert call(1,'connect',('fake',8080))[0]
        ok,why=call(2,'submit',dict(prepared=segment(0.)))
        assert not ok and why.startswith('LOG_OPEN_FAILED_NO_MOTION') and made[0].sent==0
        result=[json.loads(p.read_text()) for p in tmp_path.glob('*/result.json')]
        assert result and result[0]['state']=='REJECTED' and result[0]['hardware_motion']=='NOT_STARTED'
    finally:
        call(3,'close');thread.join(2)


def test_execute_requires_a_live_task_record_before_motion():
    m=SimpleNamespace(cycle=SimpleNamespace(state='PLAN_READY',active=False),journal=SimpleNamespace(error='disk full',thread=SimpleNamespace(is_alive=lambda:False)),
                      _enter=lambda:None,busy=True,planner=SimpleNamespace(authority=SimpleNamespace(invalidate=lambda:None)),
                      rejected=[])
    m.reject=lambda e:m.rejected.append(str(e))
    with pytest.raises(ValueError,match='TASK_RECORD_UNAVAILABLE_NO_MOTION'):TaskManager.execute(m)
    assert m.rejected and m.busy is False


# --- Camera faults during loaded motion (review item 5) ----------------------
# User confirmed on 2026-11-13: a camera fault aborts loaded motion as well.

@pytest.mark.parametrize('holding',[False,True])
def test_camera_session_change_stops_even_a_loaded_cycle_and_keeps_ticking(holding):
    stops=[];ticks=[]
    c=SimpleNamespace(state='DETACH' if holding else 'APPROACH',active=True,reason='',
                      stop=lambda reason='OPERATOR_SOFTWARE_STOP',cancelled=False:stops.append(reason),tick=lambda:ticks.append(1))
    m=SimpleNamespace(cycle=c,stop_event=threading.Event(),busy=False,observation=SimpleNamespace(generation=1),
                      backend=SimpleNamespace(observation_generation=2),journal=SimpleNamespace(error=None),
                      vacuum=SimpleNamespace(holding=holding),_save_result=lambda cycle,saved:None)
    TaskManager._monitor_once(m,[None])
    # Stopped, and still ticked so the stop is confirmed or latched; no release.
    assert stops==['CONNECTION_OR_CONFIG_CHANGED'] and ticks==[1]


def bare_backend():
    b=object.__new__(SingleBackend);owner=SimpleNamespace(min_received_s=0.)
    b.workers={'camera':None,'robot':None,'vacuum':SimpleNamespace(device=SimpleNamespace(owner=owner))}
    b.observation_generation=0;b.sample_cutoff={k:0 for k in b.workers}
    b.manual_stationary=False;b.frame=b.joint=b.pressure=b.detections=None
    b.joint_history=SampleHistory(maxlen=4);b.vision=None;b.logs=deque(maxlen=20)
    return b,owner


def test_camera_invalidation_revokes_observations_but_keeps_current_gripper_pressure():
    b,owner=bare_backend()
    b.invalidate_device('camera')
    assert b.observation_generation==1 and owner.min_received_s==0.
    b.invalidate_device('robot')
    assert b.observation_generation==2 and owner.min_received_s>0


# --- Thread robustness (low 6) -----------------------------------------------

def test_device_pump_survives_a_failing_cycle():
    b,_=bare_backend();b.pump_stop=threading.Event();b.pump_errors=0;calls=[]
    def once(state):
        calls.append(state)
        if len(calls)==1:raise KeyError('malformed event')
        b.pump_stop.set()
    b._pump_once=once;b._pump()
    assert len(calls)==2 and b.pump_errors==1 and any('KeyError' in line for line in b.logs)


def test_task_monitor_thread_survives_errors_and_keeps_ticking_the_cycle():
    class Flaky:
        state='APPROACH';reason='';active=True
        def __init__(self):self.ticks=0;self.stops=[];self.stop_requested=threading.Event()
        def tick(self):
            self.ticks+=1
            if self.ticks==1:raise RuntimeError('boom')
        def stop(self,reason='OPERATOR_SOFTWARE_STOP',cancelled=False):self.stops.append(reason)
    notices=[]
    m=TaskManager(SimpleNamespace(observation_generation=1),SimpleNamespace(request_stop=lambda:None),
                  SimpleNamespace(holding=False,inhibit=lambda:None),SimpleNamespace(authority=SimpleNamespace(invalidate=lambda:None)),
                  None,None,{},lambda kind,value:notices.append((kind,value)))
    try:
        m.observation=SimpleNamespace(generation=1);m.journal=SimpleNamespace(error=None)
        cycle=m.cycle=Flaky();deadline=time.monotonic()+3
        while cycle.ticks<3 and time.monotonic()<deadline:time.sleep(.01)
        assert cycle.ticks>=3 and m.thread.is_alive()
        assert any(s.startswith('SUPERVISOR_ERROR:RuntimeError:boom') for s in cycle.stops)
        assert any(kind=='error' and 'boom' in value for kind,value in notices)
    finally:
        m.closing.set();m.thread.join(1)


def test_joint_history_iteration_is_safe_against_concurrent_appends():
    history=SampleHistory(maxlen=128);stop=threading.Event()
    def writer():
        i=0
        while not stop.is_set():history.append(i);i+=1;time.sleep(0)
    thread=threading.Thread(target=writer,daemon=True);thread.start()
    try:
        for _ in range(100):
            seen=0
            # Yield the GIL per element, as joint_for_frame's per-sample
            # Python stamp check does, so the writer appends mid-iteration.
            for _sample in history:seen+=1;time.sleep(0)
            assert seen<=128
    finally:
        stop.set();thread.join(1)
    history.clear();assert list(history)==[]


# --- Envelope tightness and unreconstructed neighbours (tests gap, perception 3)

def test_exact_prior_face_keeps_a_bounded_envelope_that_covers_the_prior():
    p,m=plane(width=.15,height=.1);box=recover_partial(p,m,np.eye(4),prior=[.15,.1,.1])
    size=np.asarray(box['size_m'])
    assert np.all(size>=[.15,.1,.1]) and np.all(size<=[.19,.145,.11])  # was 217 x 169 x 108 mm


def neighbor_result():
    obs=observation();mask=np.zeros(obs.frame.depth.shape,bool);mask[:,15:]=True
    items=({'label':'box','score':.8,'frame_id':'camera_optical'},)
    return obs,DetectionResult(obs.frame.observation_id,obs.frame.stamp.source,1,items,(mask,))


def test_user_clearance_keeps_the_hidden_volume_behind_an_unreconstructed_neighbor():
    obs,result=neighbor_result();c=small_site()
    c.update(unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC_TEST_USER_CONFIRMATION')
    without=build_scene(obs,result,[],c);scene=build_scene(obs,result,[],c,neighbor_depth_m=.1)
    assert without['neighbor_shadow_voxels']==0
    shadow=[o for o in scene['world'] if o['category']=='neighbor_shadow']
    assert scene['unreconstructed_detections']==[0] and scene['neighbor_shadow_voxels']>0 and shadow
    observed=[o for o in scene['world'] if o['category']=='observed']
    nearest=min(np.asarray(o['pose'])[2,3]-np.asarray(o['size'])[2]/2 for o in observed)
    # Shadow lies behind the measured surface, never between it and the camera.
    assert all(np.asarray(o['pose'])[2,3]-np.asarray(o['size'])[2]/2>=nearest-1e-9 for o in shadow)
    assert scene['user_confirmed_clear_unknown_voxels']==scene['unknown_voxels']-scene['neighbor_shadow_voxels']


def test_occupied_policy_needs_no_neighbor_shadow():
    obs,result=neighbor_result()
    scene=build_scene(obs,result,[],small_site(),neighbor_depth_m=.1)
    assert scene['neighbor_shadow_voxels']==0 and scene['unknown_collision_voxels']>0


# --- Common-valve board safety logic (tests gap) -----------------------------

class Port:
    def __init__(self):self.writes=[];self.short=False;self.closed=False
    def read(self,n):time.sleep(.005);return b''
    def write(self,data):self.writes.append(bytes(data));return len(data)-1 if self.short else len(data)
    def close(self):self.closed=True


def owner(**extra):
    port=Port();config=dict(max_age_s=3.,hold_below_kpa=-30.,establish_commands=[0x12],release_commands=[0x13],**extra)
    v=VacuumOwner(config,open_port=lambda path:port);v.connect('/fake/serial');return v,port


def pressure(v,kpa,age_s=0.):
    stamp=Stamp.receive();stamp=replace(stamp,received_monotonic_ns=stamp.received_monotonic_ns-int(age_s*1e9))
    v.observer.binary[1]=GaugeReading(1,kpa,stamp,'CHECKED_BINARY','')


def test_vacuum_owner_writes_only_owned_reviewed_commands_and_never_on_stop():
    v,port=owner()
    try:
        assert port.writes==[]
        with pytest.raises(RuntimeError,match='NOT_OWNED'):v.establish('task')
        with pytest.raises(ValueError,match='FRESH_CHECKED'):v.claim('task',lambda **e:None,lambda r:None)
        pressure(v,-1.);v.claim('task',lambda **e:None,lambda r:None)
        with pytest.raises(RuntimeError,match='NOT_OWNED'):v.establish('other')
        v.establish('task');assert port.writes==[valve_frame(0x12)] and v.holding
        v.inhibit();assert port.writes==[valve_frame(0x12)]  # stop writes nothing
        with pytest.raises(RuntimeError,match='STOPPING'):v.release('task')
        v.unclaim();assert v.task_id=='task' and v.holding  # held load keeps ownership
        with pytest.raises(RuntimeError,match='HELD_LOAD'):v.disconnect()
        assert port.writes==[valve_frame(0x12)]
        v.unclaim(released=True);assert v.task_id is None and v.inhibited and not v.holding
    finally:
        v.holding=False;v.task_id=None;v.disconnect()
    assert port.closed


def test_vacuum_pressure_gates_freshness_session_and_partial_writes():
    v,port=owner()
    try:
        pressure(v,-1.,age_s=4.)
        with pytest.raises(ValueError,match='STALE'):v.pressure()
        pressure(v,-1.);v.min_received_s=time.monotonic()+1
        with pytest.raises(ValueError,match='PREVIOUS_SESSION'):v.pressure()
        v.min_received_s=0.;port.short=True;v.claim('task',lambda **e:None,lambda r:None)
        with pytest.raises(RuntimeError,match='PARTIAL_WRITE'):v.establish('task')
        for command in (0x99,True,18.):
            with pytest.raises(ValueError,match='NOT_A_REVIEWED_VALVE_COMMAND'):valve_frame(command)
    finally:
        v.holding=False;v.task_id=None;v.disconnect()


def test_holding_monitor_reports_pressure_loss_without_any_valve_write():
    v,port=owner();faults=[]
    try:
        pressure(v,-65.);v.claim('task',lambda **e:None,faults.append);v.establish('task');v.monitor_holding=True
        pressure(v,-10.);deadline=time.monotonic()+2
        while not faults and time.monotonic()<deadline:time.sleep(.01)
        assert 'GRIPPER_PRESSURE_LOST' in faults and port.writes==[valve_frame(0x12)]
    finally:
        v.monitor_holding=False;v.holding=False;v.task_id=None;v.disconnect()
