from dataclasses import replace
import threading,time
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workbench.contracts import RGBDFrame,Stamp,Source,DeviceTime,DetectionResult
from unloading_sim.workbench.backend import WorkbenchBackend,DeviceWorker,WorkerSample
from unloading_sim.workbench.single_carton.perception import Observation,reconstruct,unchanged,freeze
from unloading_sim.workbench.single_carton.vacuum import valve_frame,VacuumOwner
from unloading_sim.workbench.reference.vacuum_receive import FrameParser


def frame(source=Source.LIVE):
    return RGBDFrame('frame-1','serial',{},10,10,DeviceTime(100,'camera'),DeviceTime(100,'camera'),Stamp.receive(source),
        dict(width=30,height=20,fx=300,fy=300,ppx=15,ppy=10,model='SYNTHETIC_PINHOLE',coeffs=[0]*5),.001,
        np.zeros((20,30,3),np.uint8),np.full((20,30),500,np.uint16))

def observation(f=None):
    return Observation(f or frame(),(0,)*6,(0,)*6,{},(),1,tuple(map(tuple,np.eye(4))),tuple(map(tuple,np.eye(4))))

def test_detection_and_depth_same_identity_required():
    obs=observation();det=DetectionResult('other-frame',Source.LIVE,1,())
    with pytest.raises(ValueError,match='SAME_FRAME'):reconstruct(obs,det)
    det=DetectionResult('frame-1',Source.REPLAY,1,())
    with pytest.raises(ValueError,match='SAME_FRAME'):reconstruct(obs,det)
    assert not obs.frame.rgb.flags.writeable and not obs.frame.depth.flags.writeable

def test_old_observation_timestamps_are_not_refreshed():
    a=observation();b=observation(replace(a.frame,observation_id='new-frame',stamp=Stamp.receive()))
    original=a.frame.stamp;check=unchanged(a,b)
    assert check['original_observation_id']=='frame-1' and check['new_observation_id']=='new-frame'
    assert a.frame.stamp is original
    with pytest.raises(ValueError,match='CONNECTION'):unchanged(a,replace(b,generation=2))
    with pytest.raises(ValueError,match='START'):unchanged(a,replace(b,q_start=(.01,)*6))
    moved=replace(b.frame,depth=np.full((20,30),510,np.uint16))
    with pytest.raises(ValueError,match='SCENE'):unchanged(a,replace(b,frame=moved))

def test_frozen_capture_uses_capture_joint_not_inference_end_joint():
    from unloading_sim.workbench.single_carton.perception import validate_robot
    obs=observation();mutable=[0.]*6
    later=dict(q=mutable,source='LIVE',received_s=time.monotonic(),alarm=False,enabled=True)
    mutable[0]=1.;validate_robot(later,time.monotonic())
    assert obs.q_capture==(0,)*6 and obs.q_start==(0,)*6
    assert obs.metadata()['rgb_number']==obs.metadata()['depth_number']==10

def test_worker_discards_inflight_old_connection_read():
    entered=threading.Event();finish=threading.Event()
    class Device:
        connected=True;streaming=True
        def connect(self):self.connected=True
        def disconnect(self):self.connected=False
        def start(self):self.streaming=True
        def stop(self):self.streaming=False
        def read(self):entered.set();finish.wait(1);return frame()
    d=Device();w=DeviceWorker('camera',d,.1)
    try:
        assert entered.wait(1)
        w.command('stop');finish.set();time.sleep(.15)
        events=[]
        while not w.events.empty():events.append(w.events.get_nowait())
        assert not any(kind=='sample' for kind,value in events)
    finally:w.close()

@pytest.mark.parametrize('cmd,expected',[(0x12,'ee1602031215'),(0x13,'ee1602031316'),(0x53,'ee1602035356'),(0x54,'ee1602035457')])
def test_audited_reference_command_bytes(cmd,expected):
    packet=valve_frame(cmd);assert packet.hex()==expected
    assert FrameParser().feed(packet)==[(cmd,b'')]

@pytest.mark.parametrize('cmd',[0x11,0x14,0x15,0x31,0x71,0x72])
def test_no_conveyor_threshold_or_guessed_query_command(cmd):
    with pytest.raises(ValueError):valve_frame(cmd)


def test_vacuum_constructor_and_disconnected_lifecycle_never_open_or_write():
    calls=[]
    v=VacuumOwner({},open_port=lambda p:calls.append(p))
    assert calls==[];v.disconnect();assert calls==[]

@pytest.mark.parametrize('source',['REPLAY','SIMULATED'])
def test_real_execution_service_rejects_nonlive_cycle_before_connection(source):
    from unloading_sim.workbench.controlled.service import ExecutionService
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.contracts import HOST_SESSION
    authority=Authority('eco65_single_carton_segment_v1')
    binding=dict(physical_cycle=dict(observation=dict(stamp=dict(source=source,host_session=HOST_SESSION),robot_sample=dict(source=source))))
    package=authority.register({},binding,dict(native_plan='PASS',sent_path_collision='PASS'))
    service=ExecutionService(None,{},authority,lambda:binding)
    with pytest.raises(ValueError,match='REPLAY_OR_FOREIGN'):service.submit(package)
    assert service.process is None and not authority._plans

@pytest.mark.parametrize('unpaired',[False,True])
def test_freeze_pairs_original_frameset_with_nearest_measured_joint_after_rpc(monkeypatch,unpaired):
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton import perception
    class Clock:
        t=10.
        def __call__(self):return self.t
        def sleep(self,s):self.t+=s
    clock=Clock();camera_q=[]
    class Backend:
        observation_generation=7
        def sample_state(self,*args):return 'LIVE_VALID'
        @property
        def frame(self):
            return replace(frame(),stamp=replace(Stamp.receive(),received_monotonic_ns=int((clock()-.007)*1e9)),
                depth_device_time=DeviceTime(200 if unpaired else 100,'camera'))
    class Service:
        def snapshot(self):
            # A real interprocess query can finish after a new UDP sample arrives.
            clock.sleep(.002)
            return dict(source='LIVE',q=[(clock()-10)*.0001]*6,received_s=clock(),
                        sequence=clock(),alarm=False,enabled=True)
    class Model:
        def camera(self,q,c):camera_q.append(q);return {'T_world_camera':np.eye(4)}
        def tcp(self,q):return np.eye(4)
    monkeypatch.setattr(perception,'monotonic_ns',lambda:int(clock()*1e9))
    call=lambda:freeze(Backend(),Service(),Model(),SimpleNamespace(parent_frame='link_1'),clock=clock,sleep=clock.sleep)
    if unpaired:
        with pytest.raises(ValueError,match='UNTRACEABLE_OR_UNPAIRED'):call()
    else:
        observed=call()
        assert observed.q_capture!=observed.q_start
        assert observed.q_capture==camera_q[-1]
        assert observed.robot_sample['q']==list(observed.q_capture)
        assert abs(observed.robot_sample['received_s']-observed.frame.stamp.received_monotonic_ns/1e9)<=.03
        assert observed.generation==7
