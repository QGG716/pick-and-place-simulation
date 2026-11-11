"""No physical devices. Time advances only when the test clock advances."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import threading
import numpy as np
import pytest
from unloading_sim.workbench.controlled.engine import Supervisor,MonitorConfig
from unloading_sim.workbench.controlled.trajectory import prepare,Limits,Authority,inverse_mapping
from unloading_sim.workbench.controlled.transport import RealmanTransport
from unloading_sim.workbench.controlled.service import ExecutionService
from unloading_sim.workbench.ui import load_mapping
from unloading_sim.workbench.controlled.planner import ROOT,load_config,EmptyPlanner

LIMITS=Limits((-3.,)*6,(3.,)*6,(.2,)*6,(1.,)*6)

def points():
    return [dict(t=t,q=[q]*6,v=[0.]*6,a=[0.]*6) for t,q in [(0.,0.),(.13,.001),(.5,.002)]]

class Clock:
    def __init__(self):self.now=10.
    def __call__(self):return self.now
class FakeSDK:
    def __init__(self,clock):self.clock=clock;self.sent=[];self.stops=0;self.code=0;self.ack=0;self.delay=0
    def send(self,q):self.sent.append((self.clock(),list(q)));self.clock.now+=self.delay;return self.code
    def request_stop(self):self.stops+=1
    def stop_result(self):return self.ack if self.stops else None
class Harness:
    def __init__(self):
        self.clock=Clock();self.io=FakeSDK(self.clock);self.logs=[];self.engine=Supervisor(self.io,self.clock,self.logs.append);self.seq=0;self.prepared=prepare(points(),LIMITS,time_scale=1)
    def sample(self,q=None,velocity=None,**kw):
        self.seq+=1
        return dict(source='LIVE',sequence=self.seq,received_s=self.clock(),q=[0.]*6 if q is None else q,velocity=[0.]*6 if velocity is None else velocity,alarm=False,enabled=True,**kw)
    def start(self):return self.engine.submit(self.prepared,self.sample())
    def tick(self,q=None,velocity=None,dt=.005):
        self.clock.now+=dt;self.engine.tick(self.sample(q,velocity))
    def finish_sends(self):
        while self.engine.index<len(self.prepared['points']) and self.engine.state=='EXECUTING':self.tick()


def test_nonuniform_timing_scale_and_derivatives():
    p=points();a=prepare(p,LIMITS,1);b=prepare(p,LIMITS,4)
    assert b['scaled_duration_s']==2
    # Interior knot is at 0.52 s, NOT uniformly reassigned to 1.0 s.
    assert np.allclose(b['points'][104]['q'],p[1]['q'])
    for index in (4,10,30,60):
        assert np.allclose(a['points'][index]['q'],b['points'][4*index]['q'])
        assert np.allclose(a['points'][index]['v'],np.array(b['points'][4*index]['v'])*4)
        assert np.allclose(a['points'][index]['a'],np.array(b['points'][4*index]['a'])*16)
    assert len(b['collision_path'])>len(b['points'])


def test_inverse_units_order_signs_and_zero():
    mapping=load_mapping(ROOT/'configs/workbench/joint_mapping.json')
    mapping=replace(mapping,source_indices=(5,3,1,4,2,0),signs=(-1,1,-1,1,1,-1),zero_offsets_rad=(.1,.2,.3,.4,.5,.6))
    raw=[30,20,10,-20,-30,40];q=mapping.apply(raw,mapping.source_names)
    assert np.allclose(inverse_mapping(q,mapping),raw)

@pytest.mark.parametrize('change', ['unordered','nan','speed','overshoot','endpoint','scale'])
def test_illegal_final_trajectories_rejected(change):
    p=points();scale=1
    if change=='unordered':p[1]['t']=0
    if change=='nan':p[1]['q'][0]=float('nan')
    if change=='speed':p[1]['q']=[2.]*6
    if change=='overshoot':p[1]['v']=[100.]*6
    if change=='endpoint':p[0]['v'][0]=.1
    if change=='scale':scale=.5
    with pytest.raises(ValueError):prepare(p,LIMITS,scale)

@pytest.mark.parametrize('bad', ['start','stale','replay','disabled','alarm','moving','nan_time'])
def test_start_rejections_never_send(bad):
    h=Harness();s=h.sample()
    if bad=='start':s['q'][0]=.02
    if bad=='stale':s['received_s']-=.2
    if bad=='replay':s['source']='REPLAY'
    if bad=='disabled':s['enabled']=False
    if bad=='alarm':s['alarm']=True
    if bad=='moving':s['velocity'][0]=.1
    if bad=='nan_time':s['received_s']=float('nan')
    with pytest.raises(ValueError):h.engine.submit(h.prepared,s)
    assert not h.io.sent and not h.io.stops

def test_duplicate_task_rejected():
    h=Harness();task=h.start()
    with pytest.raises(RuntimeError):h.start()
    assert h.engine.task_id==task and not h.io.sent

@pytest.mark.parametrize('failure,reason',[('sdk','SDK_SEND_ERROR'),('late','SEND_DEADLINE_MISSED'),('slow','SDK_SEND_OVERRUN'),('feedback','FEEDBACK_EXPIRED'),('alarm','CONTROLLER_ALARM_OR_DISABLED')])
def test_fault_stops_without_backlog(failure,reason):
    h=Harness();h.start()
    if failure=='sdk':h.io.code=-2
    if failure=='slow':h.io.delay=.003
    if failure=='late':h.tick(dt=.03)
    elif failure=='feedback':h.clock.now+=.2;h.engine.tick(None)
    elif failure=='alarm':s=h.sample();s['alarm']=True;h.engine.tick(s)
    else:h.tick(dt=.01)
    assert h.engine.state=='STOPPING' and h.engine.reason==reason and h.io.stops==1
    count=len(h.io.sent)
    for _ in range(100):h.tick()
    assert len(h.io.sent)==count and h.engine.state=='FAILED'

def test_sustained_tracking_error():
    h=Harness();h.start()
    for _ in range(35):h.tick(q=[.1]*6)
    assert h.engine.state=='STOPPING' and h.engine.reason=='TRACKING_ERROR'

def test_send_completion_is_not_arrival_and_real_dwell_required():
    h=Harness();h.start();h.finish_sends()
    assert h.engine.state=='EXECUTING'
    for _ in range(30):h.tick(q=[.002]*6,velocity=[.01]*6)
    assert h.engine.state=='EXECUTING'
    for _ in range(85):h.tick(q=[.002]*6)
    assert h.engine.state=='SUCCEEDED'
    assert h.engine.feedback(h.sample())['arrival_confirmed']

def test_last_send_at_goal_does_not_succeed_on_stale_receipt():
    h=Harness();h.start();h.finish_sends();stale=h.sample(q=[.002]*6)
    for _ in range(30):h.clock.now+=.005;h.engine.tick(stale)
    assert h.engine.state=='STOPPING' and h.engine.reason=='FEEDBACK_EXPIRED'

def test_cancel_idempotent_and_requires_feedback_ack_dwell():
    h=Harness();h.start();h.tick(dt=.01);h.engine.cancel();h.engine.cancel()
    assert h.io.stops==1 and h.engine.state=='STOPPING'
    for _ in range(20):h.tick(velocity=[.1]*6)
    assert h.engine.state=='STOPPING'
    for _ in range(85):h.tick()
    assert h.engine.state=='CANCELLED';h.engine.cancel();assert h.io.stops==1

@pytest.mark.parametrize('failure',['network','stop_error'])
def test_stop_unconfirmed_latches_and_cannot_resubmit(failure):
    h=Harness();h.start();h.engine.cancel()
    if failure=='stop_error':h.io.ack=-2
    for _ in range(410):
        h.clock.now+=.005;h.engine.tick(None if failure=='network' else h.sample())
    assert h.engine.state=='STOP_UNCONFIRMED'
    with pytest.raises(RuntimeError):h.start()
    assert sum(x['kind']=='stop_return' for x in h.logs)<=1

def test_preview_permits_cannot_be_loaded_toggled_or_reused():
    a=Authority();h=Harness();binding={'model':'abc'}
    with pytest.raises(ValueError):a.verify({'status':'EXECUTABLE'},binding)
    with pytest.raises(ValueError):a.register(h.prepared,binding,{'native_plan':'PASS'})
    p=a.register(h.prepared,binding,{'native_plan':'PASS','sent_path_collision':'PASS'})
    a.verify(p,binding)
    corrupt=deepcopy(p);corrupt['prepared']['points'][0]['q'][0]=.1
    with pytest.raises(ValueError):a.verify(corrupt,binding)
    with pytest.raises(ValueError):a.verify(p,{'model':'changed'})
    a.consume(p)
    with pytest.raises(ValueError):a.verify(p,binding)
    p=a.register(h.prepared,binding,{'native_plan':'PASS','sent_path_collision':'PASS'});a.invalidate()
    with pytest.raises(ValueError):a.verify(p,binding)

def test_construction_configuration_preview_do_not_initialize_sdk(tmp_path):
    calls=[];mapping=load_mapping(ROOT/'configs/workbench/joint_mapping.json')
    transport=RealmanTransport(mapping,{'record_root':str(tmp_path)},lambda:calls.append('init'))
    authority=Authority();service=ExecutionService(mapping,{},authority,lambda:{})
    prepare(points(),LIMITS);config=load_config(ROOT/'configs/workbench/controlled.example.json')
    assert not calls and service.process is None and not transport.connected
    assert config['tool_mass_kg']==1.5 and config['robot_rated_payload_kg']==5

def test_cancel_signal_does_not_wait_for_query_lock():
    service=ExecutionService(None,{},Authority(),lambda:{});service.process=object();service.stop_event=threading.Event();service._active_id='task'
    with service._lock:service.cancel('task')
    assert service.stop_event.is_set()

def test_rejected_submit_consumes_permit():
    a=Authority();h=Harness();p=a.register(h.prepared,{},dict(native_plan='PASS',sent_path_collision='PASS'))
    service=ExecutionService(None,{},a,lambda:{})
    def reject(*_):raise RuntimeError('START_CHANGED_REPLAN_REQUIRED')
    service._call=reject
    with pytest.raises(RuntimeError):service.submit(p)
    with pytest.raises(ValueError):a.verify(p,{})

def test_disconnect_reconnect_invalidates_old_task_permits():
    a=Authority();h=Harness();p=a.register(h.prepared,{},dict(native_plan='PASS',sent_path_collision='PASS'))
    service=ExecutionService(None,{},a,lambda:{});calls=[];service._call=lambda *x:calls.append(x)
    service.disconnect();service.connect('localhost',8080)
    with pytest.raises(ValueError):service.submit(p)
    assert [c[0] for c in calls]==['disconnect','connect']

def test_sdk_boundary_exact_canfd_no_filter_and_stop_guard():
    mapping=load_mapping(ROOT/'configs/workbench/joint_mapping.json');transport=RealmanTransport(mapping,{})
    class SDK:
        def __init__(self):self.calls=[]
        def rm_movej_canfd(self,*args):self.calls.append(args);return 0
    sdk=transport.sender=SDK()
    assert transport.send([0]*6)==-998
    transport.arm();assert transport.send([np.pi/2]*6)==0
    assert sdk.calls==[([90.]*6,True,0,0,0)]
    transport.external_stop=threading.Event();transport.external_stop.set()
    assert transport.send([0]*6)==-998 and len(sdk.calls)==1


def test_stop_channel_has_only_fixed_stop_command_and_parses_split_reply(monkeypatch):
    from unloading_sim.workbench.controlled.transport import StopChannel
    class Socket:
        def __init__(self):self.sent=[];self.replies=[b'{"command":"set_arm_',b'stop","arm_stop":true}\r\n'];self.closed=False
        def settimeout(self,x):pass
        def sendall(self,b):self.sent.append(b)
        def recv(self,n):return self.replies.pop(0)
        def close(self):self.closed=True
    sock=Socket();monkeypatch.setattr('socket.create_connection',lambda *a,**k:sock)
    channel=StopChannel('fake',8080);assert not sock.sent
    assert channel.stop()==0 and sock.sent==[b'{"command":"set_arm_stop"}\r\n']
    channel.close();assert sock.closed


def test_worker_close_stops_own_task_then_disconnects_and_records(tmp_path,monkeypatch):
    """Exercise actual IPC/record lifecycle with fake transport; never load SDK."""
    import multiprocessing as mp
    import time
    import unloading_sim.workbench.controlled.service as svc
    instances=[]
    class FakeTransport:
        def __init__(self,mapping,config):
            self.connected=False;self.capabilities={};self.sender=self;self.stops=0;self.deletes=0;self.seq=0;self.q=[0.]*6;instances.append(self)
        def connect(self,*args):self.connected=True;return self.capabilities
        @property
        def latest(self):
            if not self.connected:return None
            self.seq+=1
            return dict(source='LIVE',sequence=self.seq,received_s=time.monotonic(),q=self.q,velocity=[0.]*6,enabled=True,alarm=False)
        def rm_get_current_tool_frame(self):return 0,dict(payload=1.5)
        def arm(self):pass
        def check_limits(self,p):pass
        def send(self,q):self.q=q;return 0
        def request_stop(self):self.stops+=1
        def stop_result(self):return 0 if self.stops else None
        def disconnect(self):self.connected=False;self.deletes+=1
    monkeypatch.setattr(svc,'RealmanTransport',FakeTransport)
    parent,child=mp.Pipe();stop=threading.Event()
    config=dict(record_root=str(tmp_path),tool_mass_kg=1.5,monitor=dict(dwell_s=.02,jitter_s=.004,max_send_gap_s=.01))
    thread=threading.Thread(target=svc.worker,args=(child,None,config,stop),daemon=True);thread.start()
    ident=0
    def call(op,arg=None):
        nonlocal ident
        ident+=1;parent.send((ident,op,arg));assert parent.poll(3);_,ok,value=parent.recv();return ok,value
    try:
        assert call('connect',('fake',8080))[0]
        ok,task=call('submit',dict(prepared=prepare(points(),LIMITS,10)));assert ok
        ok,why=call('close');assert not ok and why=='STOPPING_WAIT_FOR_FEEDBACK'
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            _,status=call('feedback')
            if status['state']=='CANCELLED':break
            time.sleep(.005)
        assert status['state']=='CANCELLED' and instances[0].deletes==0
        assert call('close')[0];thread.join(2);assert not thread.is_alive()
        assert instances[0].stops==1 and instances[0].deletes>=1
        result=json.loads((tmp_path/task/'result.json').read_text())
        assert result['state']=='CANCELLED' and result['task_id']==task
        assert (tmp_path/task/'trajectory-package.json').is_file()
    finally:parent.close();child.close()


def test_collision_sampling_bounds_intermediate_motion():
    p=prepare(points(),LIMITS,10,reach_bound_m=1.5)
    sampling=p['collision_sampling'];step=p['period_s']/sampling['subdivisions_per_send']
    assert 4*1.5*sum(sampling['analytic_speed_bounds_rad_s'])*step<=.001+1e-12
    assert p['points'][0]['q']==p['collision_path'][0]

def test_authority_single_atomic_claim_under_double_click():
    a=Authority();p=a.register(prepare(points(),LIMITS),{},dict(native_plan='PASS',sent_path_collision='PASS'))
    barrier=threading.Barrier(2);results=[]
    def claim():
        barrier.wait()
        try:a.claim(p,{});results.append('accepted')
        except ValueError:results.append('rejected')
    ts=[threading.Thread(target=claim) for _ in range(2)]
    for t in ts:t.start()
    for t in ts:t.join()
    assert sorted(results)==['accepted','rejected']

def test_controller_limits_do_not_widen_native_limits():
    m=load_mapping(ROOT/'configs/workbench/joint_mapping.json');t=RealmanTransport(m,{})
    t.capabilities={name:{'value':value} for name,value in [('rm_get_joint_min_pos',[-180.]*6),('rm_get_joint_max_pos',[180.]*6),('rm_get_joint_max_speed',[60.]*6),('rm_get_joint_max_acc',[90.]*6)]}
    p=prepare(points(),LIMITS);t.check_limits(p)
    t.capabilities['rm_get_joint_max_speed']['value'][0]=1.
    with pytest.raises(ValueError,match='NARROWER'):t.check_limits(p)

@pytest.mark.real_eco65_asset
def test_full_geometry_and_mass_are_preserved_without_simulation_exemptions():
    import xml.etree.ElementTree as E
    c=load_config(ROOT/'configs/workbench/controlled.example.json')
    if not Path(c['snapshot']).is_file():pytest.skip('local full-geometry source unavailable')
    p=EmptyPlanner(c,load_mapping(c['mapping']),Authority());p.build()
    source=p.source['native_model']['init'];old=E.fromstring(source['parameters']['robot_description']);new=E.fromstring(p.init['parameters']['robot_description'])
    assert len(new.findall('.//collision'))==len(old.findall('.//collision'))==715
    assert len(p.world)==928
    assert p.mass['mass_kg']==1.5 and min(np.linalg.eigvalsh(p.mass['inertia_com_flange_kg_m2']))>0
    for link in old.findall('link'):
        if link.find('inertial') is not None:
            new_link=new.find(f"link[@name='{link.get('name')}']")
            assert E.tostring(link.find('inertial'))==E.tostring(new_link.find('inertial'))
    sr=E.fromstring(p.init['parameters']['robot_description_semantic'])
    assert all('simulation' not in e.get('reason').lower() for e in sr.findall('disable_collisions'))
    assert p.removed==2416 and 0<p.reach_bound<10
    assert p.init['process_geometry'] is None and not p.init.get('directed_j1_enabled')
    assert len(p.source['tool']['seals'])==12


def test_cancel_during_vendor_call_reasserts_stop_after_return():
    h=Harness();h.start()
    def send(q):
        h.io.sent.append((h.clock(),q));h.engine.cancel();h.clock.now+=.02;return 0
    h.io.send=send;h.tick(dt=.01)
    assert h.engine.state=='STOPPING' and h.io.stops==2
    assert any(x['kind']=='stop_reasserted_after_inflight_send' for x in h.logs)
    for _ in range(85):h.tick()
    assert h.engine.state=='CANCELLED' and len(h.io.sent)==1


def test_feedback_ipc_loss_is_visible_and_latched():
    a=Authority();s=ExecutionService(None,{},a,lambda:{});s.process=object();s.stop_event=threading.Event();s._active_id='old'
    def fail(*args):raise BrokenPipeError('worker vanished')
    s._call=fail
    result=s.feedback('old')
    assert result['state']=='STOP_UNCONFIRMED' and s.stop_event.is_set() and s._poisoned


def test_stop_during_submit_preflight_cannot_be_cleared(monkeypatch,tmp_path):
    import multiprocessing as mp
    import unloading_sim.workbench.controlled.service as module
    events=[]
    class Transport:
        connected=True;capabilities={};latest=None
        def __init__(self,*args):pass
        def check_limits(self,*args):events.append('check')
        def stop_result(self):return None
        def disconnect(self):pass
    monkeypatch.setattr(module,'RealmanTransport',Transport)
    parent,child=mp.Pipe();stop=threading.Event();epoch=mp.Value('Q',1)
    thread=threading.Thread(target=module.worker,args=(child,None,{'record_root':str(tmp_path)},stop,epoch),daemon=True);thread.start()
    parent.send((1,'submit',dict(package={'prepared':prepare(points(),LIMITS)},cancel_epoch=0)))
    assert parent.poll(3);_,ok,error=parent.recv()
    assert not ok and error=='SUBMISSION_CANCELLED_BEFORE_MOTION'
    parent.send((2,'close',None));assert parent.poll(3);parent.recv();thread.join(2);parent.close();child.close()
    result=json.loads(next(tmp_path.glob('*/result.json')).read_text())
    assert result['state']=='REJECTED' and result['hardware_motion']=='NOT_STARTED'


def test_nonfinite_or_zero_payload_cannot_pass():
    from unloading_sim.workbench.controlled.transport import payload_matches
    assert payload_matches({'payload':1.5},1.5)
    for value in (float('nan'),float('inf'),0.,5.,None,'invalid'):
        assert not payload_matches({'payload':value},1.5)
