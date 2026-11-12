"""Fake devices and a controlled clock: never opens a serial/SDK connection."""
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.cycle import Cycle

class Clock:
    def __init__(self):self.t=10.
    def __call__(self):return self.t
    def step(self,t=.1):self.t+=t
class Robot:
    def __init__(self,clock):
        self.clock=clock;self.calls=[];self.q=[0.]*6;self.state='READY';self.id=None;self.arrived=False;self.stopped=False;self.alarm=False;self.age=0.;self.ticket=0
    def submission_ticket(self):return self.ticket
    def submit(self,p,**kwargs):
        assert kwargs['cancel_ticket']==self.ticket
        self.calls.append(p);self.goal=p['prepared']['points'][-1]['q'];self.id=str(len(self.calls));self.state='EXECUTING';self.arrived=False;return self.id
    def snapshot(self):return dict(source='LIVE',q=self.q,velocity=[0.]*6,received_s=self.clock()-self.age,sequence=len(self.calls),alarm=self.alarm,enabled=True)
    def feedback(self):return dict(task_id=self.id,state=self.state,arrival_confirmed=self.arrived,stop_confirmed=self.stopped)
    def finish(self):self.q=self.goal;self.state='SUCCEEDED';self.arrived=True
    def request_stop(self):self.ticket+=1
class Vacuum:
    def __init__(self,clock):
        self.clock=clock;self.holding=False;self.monitor_holding=False;self.task_id=None;self.inhibited=True;self.commands=[];self.kpa=0.;self.age=0.;self.stamp=None
    def pressure(self):return dict(source='LIVE',channel='gripper',integrity='CHECKED_BINARY',kpa=self.kpa,received_s=self.clock()-self.age if self.stamp is None else self.stamp)
    def claim(self,task,record,on_fault):
        if self.task_id:raise ValueError('TASK_ACTIVE')
        self.task_id=task;self.inhibited=False
    def establish(self,task):assert not self.inhibited;self.commands.append('grip');self.holding=True
    def release(self,task):assert not self.inhibited;self.commands.append('release');self.monitor_holding=False
    def inhibit(self):self.inhibited=True
    def unclaim(self,released=False):
        if released:self.holding=False
        if not self.holding:self.task_id=None

def make():
    clock=Clock();robot=Robot(clock);vacuum=Vacuum(clock);records=[]
    config=dict(grip_below_kpa=-60.,hold_below_kpa=-30.,release_above_kpa=-10.,grip_dwell_s=1.,release_dwell_s=1.,grip_timeout_s=3.,release_timeout_s=3.,max_age_s=3.)
    cycle=Cycle(robot,vacuum,config,lambda **event:records.append(event),lambda *a:None,lambda *a:None,clock)
    segments=[]
    for i,stage in enumerate(['approach','contact','detach','return']):
        segments.append(dict(stage=stage,package=dict(prepared=dict(points=[dict(q=[0.]*6),dict(q=[.01*(i+1) if i!=3 else 0.]*6)]))))
    plan=dict(full_path_validated=True,segments=segments,q_start=[0.]*6)
    cycle.ready('one-box',plan)
    return cycle,clock,robot,vacuum,records,plan

def at_vacuum():
    x=make();c,t,r,v,_,_=x;c.start();r.finish();c.tick();r.finish();c.tick();assert c.state=='ESTABLISH_VACUUM';return x

def attached():
    x=at_vacuum();c,t,r,v,_,_=x;v.kpa=-65;t.step(.1);c.tick();t.step(1.01);c.tick();assert c.state=='DETACH';return x

def test_complete_requires_real_arrival_and_two_new_pressure_samples():
    c,t,r,v,events,plan=make()
    plan['release']=dict(status='RECEIVER_QUALIFICATION_DEFERRED_BY_USER',support_confirmed=False)
    c.ready('one-box',plan)
    assert r.calls==[] and v.commands==[]
    c.start();r.state='SUCCEEDED';c.tick();assert c.state=='APPROACH' # receipt is not arrival
    r.finish();c.tick();r.finish();c.tick();assert v.commands==['grip']
    v.kpa=-65;t.step();v.stamp=t();c.tick();t.step(1.1);c.tick();assert c.state=='ESTABLISH_VACUUM'
    v.stamp=t();c.tick();assert c.state=='DETACH'
    r.finish();c.tick();assert c.state=='RETURN_LOADED' and v.commands==['grip']
    r.finish();v.stamp=None;c.tick();assert c.state=='RELEASE' and v.commands==['grip','release']
    v.kpa=0;t.step();c.tick();assert c.state=='RELEASE'
    t.step(1.01);c.tick();assert c.state=='COMPLETE' and not v.holding
    assert not c.physical_complete and c.status()['physical_result']!='PHYSICAL_CYCLE_COMPLETE'
    assert events[-1]['result']=='FAKE_DEVICE_CYCLE_COMPLETE'
    assert c.status()['receiver_qualification']=='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'
    assert c.status()['support_confirmed'] is False

@pytest.mark.parametrize('fault',['pressure_loss','pressure_stale','alarm','robot_stale','sdk_failed'])
def test_loaded_fault_never_releases_or_returns_blindly(fault):
    c,t,r,v,events,_=attached()
    if fault=='pressure_loss':v.kpa=-20
    if fault=='pressure_stale':v.age=4
    if fault=='alarm':r.alarm=True
    if fault=='robot_stale':r.age=1
    if fault=='sdk_failed':r.state='FAILED'
    c.tick();assert c.state=='STOPPING';assert v.commands==['grip'] and len(r.calls)==3
    c.tick();assert v.holding

@pytest.mark.parametrize('phase',['APPROACH','ESTABLISH_VACUUM','DETACH','RETURN_LOADED','RELEASE'])
def test_cancel_is_idempotent_and_no_automatic_release(phase):
    x=make() if phase=='APPROACH' else at_vacuum() if phase=='ESTABLISH_VACUUM' else attached()
    c,t,r,v,events,_=x
    if phase=='APPROACH':c.start()
    if phase in ('RETURN_LOADED','RELEASE'):r.finish();c.tick()
    if phase=='RELEASE':r.finish();c.tick()
    before=list(v.commands);c.stop(cancelled=True);c.stop(cancelled=True)
    assert c.state=='STOPPING' and v.commands==before
    r.state='CANCELLED';r.stopped=True;c.tick();assert c.state=='CANCELLED' and v.commands==before
    assert sum(e.get('state')=='STOPPING' for e in events)==1

def test_network_loss_reports_stop_unconfirmed_and_reconnect_never_resumes():
    c,t,r,v,_,_=attached();c.stop(cancelled=True);r.age=100;t.step(2.1);c.tick()
    assert c.state=='STOP_UNCONFIRMED' and v.holding and v.commands==['grip']
    r.age=0;r.state='READY';before=len(r.calls);c.tick();assert len(r.calls)==before
    with pytest.raises(ValueError):c.start()

def test_grip_timeout_does_not_allow_detach():
    c,t,r,v,_,_=at_vacuum();t.step(3.1);c.tick();assert c.state=='STOPPING';assert len(r.calls)==2

def test_release_timeout_is_failure_not_completion():
    c,t,r,v,_,_=attached();r.finish();c.tick();r.finish();c.tick();t.step(3.1);c.tick()
    assert c.state=='STOPPING' and not c.physical_complete and v.holding

def test_start_changes_and_duplicate_start_and_full_return_gate():
    c,t,r,v,_,plan=make();r.q=[.1]*6
    with pytest.raises(ValueError,match='START_CHANGED'):c.start()
    assert not r.calls and not v.commands
    c,t,r,v,_,plan=make();c.start()
    with pytest.raises(ValueError):c.start()
    assert len(r.calls)==1
    c,t,r,v,_,plan=make();plan['q_start']=[.2]*6
    with pytest.raises(ValueError,match='RETURN_NOT_Q_START'):c.ready('bad',plan)

def test_mutated_plan_and_stop_before_start_never_submit():
    c,t,r,v,_,plan=make();plan['q_start'][0]=1
    with pytest.raises(ValueError,match='PLAN_CHANGED'):c.start()
    c,t,r,v,_,plan=make();c.stop(cancelled=True)
    with pytest.raises(ValueError):c.start()
    assert not r.calls and not v.commands

def test_contact_actual_fk_check_before_vacuum():
    c,t,r,v,_,_=make();c.contact_check=lambda *a:(_ for _ in ()).throw(ValueError('BAD_SEAL'))
    c.start();r.finish();c.tick();r.finish();c.tick();assert c.state=='STOPPING' and not v.commands

def test_monitor_ages_sample_after_snapshot_rpc_returns():
    c,t,r,v,_,_=make();c.start();original=r.snapshot
    def delayed():
        t.step(.002)
        return original()
    r.snapshot=delayed;c.tick()
    assert c.state=='APPROACH' and not c.reason
