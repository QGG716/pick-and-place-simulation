"""Stopped capture and operator progress. Synthetic clocks/IO only."""
from dataclasses import replace
from types import SimpleNamespace
import json
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import perception, runtime
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.controlled.trajectory import Authority
from test_single_carton_observation import frame, observation
from eco65_test_inputs import use_synthetic_remote_detector


def samples():
    # 0.004 degree quantization jump in a 1 ms interval: derived 4 deg/s,
    # controller-reported speed zero, total position range only 0.004 degree.
    return [dict(q=np.deg2rad([q]*6).tolist(),received_s=t,
                 velocity=[0.]*6,velocity_source='MEASURED_UDP_SPEED')
            for t,q in [(0.,0.),(.001,.004),(.09,0.),(.18,.004),(.27,0.),(.36,.004),(.45,0.)]]


def test_measured_speed_avoids_quantization_spike_without_relaxing_position_bound():
    history=samples();check=perception.stationary_checks(history)
    assert check['stationary'] and check['range_deg']==pytest.approx(.004)
    assert check['speed_deg_s']==0. and check['speed_source']=='MEASURED_UDP_SPEED'
    history[2]['q']=np.deg2rad([.021]*6).tolist()
    assert not perception.stationary_checks(history)['stationary']


@pytest.mark.parametrize('change',['missing','invalid','derived','moving','gap'])
def test_stationary_requires_evidence_not_assumed_zero_speed(change):
    history=samples()
    if change=='missing':history[1]['velocity']=None
    elif change=='invalid':history[1]['velocity']=[float('nan')]*6
    elif change=='derived':history[1]['velocity_source']='FINITE_DIFFERENCE_OF_ACTUAL_UDP_SAMPLES'
    elif change=='moving':history[1]['velocity']=np.deg2rad([.51]*6).tolist()
    elif change=='gap':history.pop(3)
    assert not perception.stationary_checks(history)['stationary']


@pytest.mark.parametrize('fault',['none','no_speed','stale_frame','future_frame','skew','no_camera','session'])
def test_freeze_reports_exact_wait_condition_and_preserves_original_clock(monkeypatch,fault):
    class Clock:
        t=10.
        def __call__(self):return self.t
        def sleep(self,s):self.t+=s
    clock=Clock();progress=[];sequence=0
    class Backend:
        observation_generation=3
        def sample_state(self,*_):return 'LIVE_VALID'
        @property
        def frame(self):
            if fault=='no_camera':return None
            if fault=='session':self.observation_generation=4
            offset={'stale_frame':-.2,'future_frame':.02}.get(fault,0.)
            f=frame()
            return replace(f,stamp=replace(f.stamp,received_monotonic_ns=int((clock()+offset)*1e9)))
    class Service:
        def snapshot(self):
            nonlocal sequence
            clock.sleep(.002);sequence+=1
            return dict(source='LIVE',sequence=sequence,received_s=clock()-(.045 if fault=='skew' else 0.),
                        q=np.deg2rad([.008*(sequence%2)]*6).tolist(),alarm=False,enabled=True,
                        velocity=None if fault=='no_speed' else [0.]*6,velocity_source='MEASURED_UDP_SPEED')
    model=SimpleNamespace(camera=lambda q,c:{'T_world_camera':np.eye(4)},tcp=lambda q:np.eye(4))
    monkeypatch.setattr(perception,'monotonic_ns',lambda:int(clock()*1e9))
    def capture():
        return perception.freeze(Backend(),Service(),model,SimpleNamespace(parent_frame='link_1'),
                                 clock=clock,sleep=clock.sleep,progress=progress.append)
    if fault=='none':
        obs=capture()
        assert obs.capture_checks['stationary'] and obs.capture_checks['window_s']>=.4
        assert obs.frame.stamp.received_monotonic_ns==int(clock()*1e9)
        assert obs.metadata()['capture_checks']['speed_source']=='MEASURED_UDP_SPEED'
    elif fault=='session':
        with pytest.raises(ValueError,match='CONNECTION_CHANGED'):capture()
    else:
        with pytest.raises(perception.CaptureRejected) as error:capture()
        d=error.value.evidence
        expected={'no_speed':'速度超过','stale_frame':'年龄不合格','future_frame':'年龄不合格',
                  'skew':'配对差超过','no_camera':'本会话新 RGB-D'}[fault]
        assert expected in d['waiting'] and expected in str(error.value)
        assert d['elapsed_s']>=4.
        if fault=='skew':assert d['pair_skew_s']==pytest.approx(.045)
    assert progress and '停稳窗口' in progress[0] and len(progress)<=17


def test_task_clears_previous_observation_shows_capture_before_inference_and_records_failure(monkeypatch,tmp_path):
    config=use_synthetic_remote_detector(load(__import__('pathlib').Path('configs/workbench/single_carton.example.json')),tmp_path)
    config['record_root']=str(tmp_path);config.pop('site',None)
    notifications=[];obs=observation();evidence=dict(waiting='测试配对超限',pair_skew_s=.045)
    manager=None
    class Vision:
        def infer_frozen(self,f,g,cancelled,*,progress):
            assert manager.result is None and manager.boxes==[] and manager.scene is None
            assert manager.observation is obs and any(k=='capture' for k,v in notifications)
            progress('本次冻结帧 SAM 处理中')
            raise perception.CaptureRejected('TEST_INFERENCE_FAILURE',evidence)
    def capture(*args,progress,**kwargs):
        assert manager.observation is None and manager.reason==''
        progress('等待停稳');return obs
    monkeypatch.setattr(runtime,'freeze',capture)
    manager=runtime.TaskManager(SimpleNamespace(vision=Vision()),None,None,
        SimpleNamespace(authority=Authority('eco65_single_carton_segment_v1')),None,None,config,
        lambda k,v:notifications.append((k,v)))
    manager.vacuum=SimpleNamespace(holding=False)
    manager.observation=object();manager.result=object();manager.boxes=[object()];manager.scene=object();manager.reason='previous failure'
    try:
        with pytest.raises(perception.CaptureRejected,match='TEST_INFERENCE_FAILURE'):manager.observe()
        assert not manager.busy and manager.phase=='FAILED'
        result=json.loads((manager.folder/'result.json').read_text())
        assert result['evidence']==evidence and result['physical_result']=='NOT_RUN'
        assert any(k=='progress' and 'SAM' in v for k,v in notifications)
    finally:
        manager.closing.set();manager.thread.join(1)
        if manager.journal:manager.journal.close()


def test_record_creation_failure_does_not_leave_preparation_busy(monkeypatch,tmp_path):
    config={'record_root':str(tmp_path/'file')};(tmp_path/'file').write_text('not a directory')
    manager=runtime.TaskManager(None,None,SimpleNamespace(holding=False),
        SimpleNamespace(authority=Authority()),None,None,config)
    try:
        with pytest.raises(OSError):manager.observe()
        assert not manager.busy and manager.phase=='FAILED'
    finally:manager.closing.set();manager.thread.join(1)
