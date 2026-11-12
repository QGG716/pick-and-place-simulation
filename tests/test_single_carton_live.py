"""Fake-only continuous observation contracts. Never open SDK/serial/camera."""
from dataclasses import replace
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.contracts import DetectionResult,Stamp
from unloading_sim.workbench.devices import StreamProfile
from unloading_sim.workbench.single_carton.live import LiveVision,SingleBackend,overlay
from test_single_carton_observation import frame


def exposure(i):return replace(frame(),observation_id=f'frame-{i}',rgb_number=i,depth_number=i)


def until(predicate,timeout=3):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.005)
    assert predicate()


class Model:
    def __init__(self,**kw):self.seen=[];self.gate=threading.Event();self.gate.set()
    def detect_boxes(self,f):
        self.seen.append(f.rgb_number)
        return DetectionResult(f.observation_id,f.stamp.source,1,
            ({'label':'box','score':.9,'bbox_xyxy':[0,0,20,15],'center_m':None,'frame_id':'camera_optical'},))
    def segment(self,f,d):
        assert d.observation_id==f.observation_id
        self.gate.wait(2)
        return replace(d,masks=(np.ones(f.depth.shape,bool),))


def test_detection_publishes_before_sam_and_backlog_is_one_latest_frame():
    model=Model();model.gate.clear();loads=[]
    v=LiveVision({},factory=lambda **kw:(loads.append(1) or model));v.start()
    try:
        v.offer(exposure(1),None,0)
        until(lambda:'detection' in v.snapshot()[0])
        assert 'segmentation' not in v.snapshot()[0]
        for i in range(2,302):v.offer(exposure(i),None,0)
        assert v.current[0].rgb_number==1 and v.pending[0].rgb_number==301
        assert v.dropped==299
        model.gate.set();until(lambda:v.completed>=2)
        assert model.seen==[1,301] and loads==[1]
        for packet in v.snapshot()[0].values():assert packet.frame.observation_id==packet.result.observation_id
    finally:v.close();model.gate.set()


def test_reconnect_discards_inflight_results_without_reloading_models():
    model=Model();model.gate.clear();v=LiveVision({},factory=lambda **_:model);v.start()
    try:
        v.offer(exposure(1),None,0);until(lambda:'detection' in v.snapshot()[0])
        v.invalidate(2);assert v.snapshot()[0]=={}
        v.offer(exposure(2),None,0);assert v.pending is None
        model.gate.set();v.offer(exposure(3),None,2)
        until(lambda:v.snapshot()[0].get('segmentation') is not None)
        assert {p.frame.rgb_number for p in v.snapshot()[0].values()}=={3}
    finally:v.close();model.gate.set()


def test_frozen_task_frame_cannot_be_replaced_and_cancel_does_not_wait_gpu():
    model=Model();model.gate.clear();v=LiveVision({},factory=lambda **_:model);v.start();cancel=threading.Event();errors=[];progress=[]
    v.offer(exposure(1),None,0);until(lambda:v.current is not None)
    def task():
        try:v.infer_frozen(exposure(55),0,cancel.is_set,progress=progress.append)
        except ValueError as e:errors.append(str(e))
    thread=threading.Thread(target=task);thread.start();until(lambda:v.task_waiter is not None)
    for i in range(60,80):v.offer(exposure(i),None,0)
    assert v.pending[0].rgb_number==55
    assert progress and '等待当前帧推理结束' in progress[0] and '#55/55' in progress[0]
    start=time.monotonic();cancel.set();thread.join(.2)
    assert not thread.is_alive() and time.monotonic()-start<.2 and errors==['CANCELLED']
    assert not model.gate.is_set()
    v.close();model.gate.set()


def test_model_error_is_explicit_and_no_demo_output():
    def fail(**_):raise OSError('missing verified weights')
    v=LiveVision({},factory=fail);v.start();v.offer(exposure(1),None,0)
    try:
        until(lambda:bool(v.error));assert 'missing verified weights' in v.error
        assert v.snapshot()[0]=={} and v.model is None
        with pytest.raises(RuntimeError,match='missing verified weights'):v.infer_frozen(exposure(2),0)
    finally:v.close()


def test_overlay_rejects_old_boxes_on_new_rgb():
    with pytest.raises(ValueError,match='FRAME_MISMATCH'):overlay(exposure(2),Model().detect_boxes(exposure(1)),False)


class Camera:
    connected=False;streaming=False
    def __init__(self,profiles=(StreamProfile(640,480,30),)):
        self.profiles=profiles;self.calls=[];self.number=0
    def connect(self,serial):self.calls.append(('connect',serial));self.connected=True;return self.profiles
    def start(self,p):self.calls.append(('start',p));self.streaming=True
    def stop(self):self.streaming=False
    def disconnect(self):self.connected=False;self.streaming=False
    def read(self):time.sleep(.02);self.number+=1;return exposure(self.number)


class Device:
    connected=False;alarms={}
    def __init__(self,fail=False):self.fail=fail;self.calls=[]
    def connect(self,*args):
        self.calls.append(args)
        if self.fail:raise OSError('endpoint unreachable')
        self.connected=True
    def read(self):return None
    def disconnect(self):self.connected=False
    def gauges(self):return {i:None for i in range(1,5)}


def test_independent_connection_failures_do_not_stop_camera_or_inference():
    camera=Camera();robot=Device(True);vacuum=Device(True)
    b=SingleBackend(camera,robot,vacuum,None);v=LiveVision({},factory=Model);b.start_observation(v)
    try:
        b.command('camera','connect','saved-camera');b.command('robot','connect','192.0.2.19',8080);b.command('vacuum','connect','/fake/serial')
        until(lambda:v.completed>1 and b.device_errors['robot'] and b.device_errors['vacuum'])
        assert camera.calls==[('connect','saved-camera'),('start',StreamProfile(640,480,30))]
        assert b.device_state('camera')=='数据有效'
        assert b.device_state('robot')==b.device_state('vacuum')=='连接失败'
        assert b.endpoints['robot']=='192.0.2.19:8080' and b.endpoints['vacuum']=='/fake/serial'
        assert 'endpoint unreachable' in b.device_errors['robot'] and b.frame.rgb_number>1
        locked=object();b.manager=SimpleNamespace(active=True,vacuum=SimpleNamespace(holding=False),cycle=None,
            config={'vacuum':{'max_age_s':3}},result=locked)
        before=v.completed;until(lambda:v.completed>=before+2)
        assert b.manager.result is locked
        with pytest.raises(ValueError,match='任务正在运行'):b.command('camera','stop')
    finally:b.close()


def test_unavailable_requested_profile_never_silently_falls_back():
    camera=Camera((StreamProfile(1280,720,15),));b=SingleBackend(camera,Device(),Device(),None)
    v=LiveVision({},factory=Model);b.start_observation(v)
    try:
        b.command('camera','connect','saved');until(lambda:bool(b.device_errors['camera']))
        assert '640×480@30fps' in b.device_errors['camera'] and not camera.streaming
        assert camera.calls==[('connect','saved')]
    finally:b.close()


def test_waiting_fresh_and_stale_are_distinct():
    camera=Camera();b=SingleBackend(camera,Device(),Device(),None)
    try:
        camera.connected=True
        assert b.device_state('camera')=='等待数据'
        b.frame=exposure(1);assert b.device_state('camera')=='数据有效'
        b.sample_cutoff['camera']=time.monotonic_ns()-3_000_000_000
        b.frame=replace(b.frame,stamp=replace(Stamp.receive(),received_monotonic_ns=time.monotonic_ns()-2_000_000_000))
        assert b.device_state('camera')=='数据过期'
        b.invalidate_device('camera');assert b.frame is None and b.device_state('camera')=='等待数据'
    finally:b.close()


def test_stop_request_never_waits_for_cycle_lock_or_gpu():
    from unloading_sim.workbench.single_carton.runtime import TaskManager
    calls=[];m=TaskManager.__new__(TaskManager);m.stop_event=threading.Event()
    m.service=SimpleNamespace(request_stop=lambda:calls.append('stop_signal'))
    m.planner=SimpleNamespace(authority=SimpleNamespace(invalidate=lambda:calls.append('revoke')))
    m.vacuum=SimpleNamespace(inhibit=lambda:calls.append('inhibit'))
    # Calling Cycle.stop here would wait on its coordinator lock. The manager
    # must only signal; its independent monitor performs the state transition.
    def blocked_stop(**kw):raise AssertionError('UI_MUST_NOT_WAIT_FOR_CYCLE_LOCK')
    m.cycle=SimpleNamespace(stop_requested=threading.Event(),stop=blocked_stop)
    start=time.monotonic();m.stop()
    assert time.monotonic()-start<.05 and m.cycle.stop_requested.is_set()
    assert calls==['stop_signal','revoke','inhibit']
