"""Regression of real Tk read controls with a fake service; no device IO."""
import os
import time

import numpy as np
import pytest

from unloading_sim.workbench.backend import DeviceRequestRejected,DeviceWorker
from unloading_sim.workbench.controlled.ui import ManagedRobotView,live_target_sample
from unloading_sim.workbench.controlled.planner import ROOT,EmptyPlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping


class FakeService:
    def __init__(self):
        self.process=None;self.connected=False;self.calls=[];self.sample_mode='live'
    def connect(self,*endpoint):
        self.calls.append(('connect',endpoint));self.process=object();self.connected=True
        return {'test_only':True}
    def disconnect(self):
        self.calls.append(('disconnect',));self.connected=False
    def snapshot(self):
        if self.sample_mode=='none':return None
        if not self.connected:raise RuntimeError('DISCONNECTED')
        return dict(source='LIVE',sequence=1,received_s=time.monotonic()-(1 if self.sample_mode=='stale' else 0),
                    q=np.deg2rad([10,-20,30,-40,50,-60]).tolist(),received_utc_ns=time.time_ns())
    def feedback(self):
        return dict(connected=self.connected,state='READY',task_id=None)
    def request_stop(self):self.calls.append(('stop_signal',))
    def close(self):self.process=None;self.connected=False
    def submit(self,*args):raise AssertionError('A read button must never submit motion')


def wait_until(predicate):
    end=time.monotonic()+2
    while not predicate():
        if time.monotonic()>end:raise AssertionError('Timed out')
        time.sleep(.01)


def test_repeated_connect_is_idempotent_through_device_worker():
    service=FakeService();view=ManagedRobotView(service,load_mapping(ROOT/'configs/workbench/joint_mapping.json'))
    worker=DeviceWorker('robot',view,.01)
    try:
        worker.command('connect','192.168.1.19',8080)
        wait_until(lambda:not worker.busy)
        worker.command('connect','192.168.1.19',8080)
        wait_until(lambda:not worker.busy)
        assert view.connected and service.calls==[('connect',('192.168.1.19',8080))]
        worker.command('connect','192.168.1.20',8080)
        wait_until(lambda:not worker.busy)
        assert view.connected and view.endpoint==('192.168.1.19',8080)
        assert len(service.calls)==1  # Rejected address change must not disconnect.
        events=[]
        while not worker.events.empty():events.append(worker.events.get_nowait())
        assert any(k=='error' and 'DeviceRequestRejected' in v for k,v in events)
        assert not any(k=='error' and 'ALREADY_CONNECTED' in v for k,v in events)
    finally:worker.close()
    assert service.calls[-1]==('disconnect',)


@pytest.mark.parametrize('sample,match',[
    (None,'没有机械臂实时数据'),
    ({'source':'LIVE','q':[0]*6,'received_s':8.},'过期'),
    ({'source':'REPLAY','q':[0]*6,'received_s':10.},'无效'),
    ({'source':'LIVE','q':[0]*5,'received_s':10.},'无效'),
    ({'source':'LIVE','q':[float('nan')]*6,'received_s':10.},'无效'),
    ({'source':'LIVE','q':[0]*6,'received_s':float('nan')},'无效'),
    ({'source':'LIVE','q':[0]*6,'received_s':11.},'过期'),
])
def test_fill_refuses_empty_stale_or_invalid_feedback(sample,match):
    with pytest.raises(RuntimeError,match=match):live_target_sample(sample,now=10.)


def test_live_target_uses_real_angles_in_sdk_joint_order():
    result=live_target_sample(dict(source='LIVE',q=np.deg2rad([86.455,14.412,-69.186,-30.715,86.948,77.313]),received_s=9.998),now=10.)
    assert result['target_degrees']=='86.4550 14.4120 -69.1860 -30.7150 86.9480 77.3130'
    assert result['feedback_age_s']==pytest.approx(.002)


@pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Actual Tk display required')
def test_actual_read_button_reports_disconnection_then_fills_then_rejects_stale():
    import tkinter as tk
    from unloading_sim.workbench.controlled.ui import ControlledWorkbench,ControlledBackend
    from unloading_sim.workbench.devices import D435iObserver,VacuumObserver
    service=FakeService();mapping=load_mapping(ROOT/'configs/workbench/joint_mapping.json')
    view=ManagedRobotView(service,mapping)
    backend=ControlledBackend(D435iObserver(),view,VacuumObserver(),service)
    root=tk.Tk();root.withdraw()
    config=dict(snapshot=None,installation=ROOT/'configs/workbench/link1_base.example.json',host='192.168.1.19',port=8080,time_scale=10)
    planner=EmptyPlanner(config,mapping,Authority())
    def unexpected_build():raise AssertionError('A current-as-target click must not start native planning')
    planner.build=unexpected_build
    ui=ControlledWorkbench(root,backend,planner,config)
    def pump_until(predicate):
        end=time.monotonic()+3
        while not predicate():
            root.update()
            if time.monotonic()>end:raise AssertionError(ui.summary.get())
            time.sleep(.01)
    try:
        ui.goal.set('operator value')
        ui.fill_button.invoke()
        pump_until(lambda:not ui.busy)
        assert '未连接' in ui.summary.get() and ui.goal.get()=='operator value'
        ui.safe(ui.connect_robot);pump_until(lambda:not ui.waiting_connection)
        ui.fill_button.invoke();pump_until(lambda:not ui.busy)
        assert ui.goal.get()=='10.0000 -20.0000 30.0000 -40.0000 50.0000 -60.0000'
        assert '已填入当前实测' in ui.summary.get()
        assert str(ui.execute_button['state'])=='disabled'
        pump_until(lambda:ui.live_status.get('connected'))
        ui.plan_button.invoke();pump_until(lambda:not ui.busy)
        assert '规划未完成：目标与当前实测姿态基本相同' in ui.summary.get()
        assert 'INVALID_TIMED_TRAJECTORY' not in ui.summary.get()
        assert ui.package is None and str(ui.preview_button['state'])=='disabled'
        assert str(ui.execute_button['state'])=='disabled'
        ui.fill_button.invoke();pump_until(lambda:not ui.busy)
        assert '已填入当前实测' in ui.summary.get()  # A later read clears the plan message.
        ui.safe(ui.connect_robot);pump_until(lambda:not ui.waiting_connection)
        assert len([c for c in service.calls if c[0]=='connect'])==1 and view.connected
        service.sample_mode='stale';ui.fill_button.invoke();pump_until(lambda:not ui.busy)
        assert '过期' in ui.summary.get() and ui.goal.get().startswith('10.0000')
        service.sample_mode='none';ui.fill_button.invoke();pump_until(lambda:not ui.busy)
        assert '没有机械臂实时数据' in ui.summary.get()
        ui.busy=True;ui.fill_button.invoke()
        assert '当前操作尚未完成' in ui.summary.get()  # Error visible on execution page.
        assert not any(c[0]=='stop_signal' for c in service.calls)
    finally:
        ui._closed=True;backend.close();root.destroy()
