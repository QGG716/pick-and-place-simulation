"""Deterministic single-task state machine; tests inject robot, pressure and clock."""
import threading
from time import monotonic
import numpy as np
from ..controlled.trajectory import digest
from .perception import validate_robot

TERMINAL={'COMPLETE','FAILED','CANCELLED','STOP_UNCONFIRMED'}
MOVEMENTS=('APPROACH','CONTACT','DETACH','RETURN_LOADED')


class Cycle:
    def __init__(self,robot,vacuum,config,record,contact_check,release_check,clock=monotonic,physical_io=False):
        self.robot,self.vacuum,self.config,self.record=robot,vacuum,config,record
        self.contact_check,self.release_check,self.clock=contact_check,release_check,clock
        self.state='IDLE';self.reason='';self.task_id=None;self.motion_id=None;self.plan=None
        self.lock=threading.RLock();self.stable_since=None;self.last_pressure=None;self.entered=clock()
        self.cancelled=False;self.physical_complete=False;self.last_feedback=None;self.plan_hash=None
        self.stop_requested=threading.Event()
        self.physical_io=physical_io
        self.vacuum_after_s=0.
    @property
    def active(self):return self.state not in TERMINAL|{'IDLE','PLAN_READY'}
    def transition(self,state,**details):
        now=self.clock();self.record(kind='stage',previous=self.state,state=state,elapsed_s=now-self.entered,**details)
        self.state=state;self.entered=now;self.stable_since=None;self.last_pressure=None
    def ready(self,task_id,plan):
        with self.lock:
            if self.active or self.vacuum.holding or self.state=='STOP_UNCONFIRMED':raise ValueError('TASK_ACTIVE_OR_LOAD_HELD')
            if plan.get('full_path_validated') is not True or len(plan.get('segments',[]))!=4:raise ValueError('FULL_PATH_REQUIRED')
            if [s['stage'] for s in plan['segments']]!=['approach','contact','detach','return']:raise ValueError('INVALID_STAGE_CHAIN')
            if np.max(abs(np.asarray(plan['segments'][-1]['package']['prepared']['points'][-1]['q'])-plan['q_start']))>1e-7:raise ValueError('RETURN_NOT_Q_START')
            self.task_id=task_id;self.plan=plan;self.plan_hash=digest(plan);self.physical_complete=False
            self.stop_requested.clear()
            self.transition('PLAN_READY')
    def start(self):
        with self.lock:
            if self.state!='PLAN_READY':raise ValueError('PLAN_NOT_READY_OR_TASK_ACTIVE')
            if digest(self.plan)!=self.plan_hash:raise ValueError('PLAN_CHANGED')
            q=validate_robot(self.robot.snapshot(),self.clock())
            if np.max(abs(q-self.plan['q_start']))>np.deg2rad(.01):
                self.transition('FAILED',reason='START_CHANGED_REPLAN');raise ValueError('START_CHANGED_REPLAN')
            self.vacuum.claim(self.task_id,self.record,lambda r:self.stop(r))
            self.cancelled=False;self.reason=''
            try:self._motion(0)
            except Exception as e:self.stop(str(e));raise
    def _motion(self,index):
        ticket=self.robot.submission_ticket()
        if self.state=='STOPPING' or self.stop_requested.is_set():raise ValueError('STOPPING')
        self.transition(MOVEMENTS[index])
        self.motion_id=self.robot.submit(self.plan['segments'][index]['package'],cancel_ticket=ticket)
        self.record(kind='motion_task',cycle_task_id=self.task_id,motion_task_id=self.motion_id,segment=index)
    def stop(self,reason='OPERATOR_SOFTWARE_STOP',cancelled=False):
        # Signal first, without waiting for a status RPC or the coordinator lock.
        self.stop_requested.set();self.vacuum.inhibit();self.robot.request_stop()
        with self.lock:
            if self.state in TERMINAL or self.state=='STOPPING':return
            if self.state in {'IDLE','PLAN_READY'}:
                self.cancelled=cancelled;self.reason=reason;self.transition('CANCELLED' if cancelled else 'FAILED',reason=reason);return
            self.cancelled=cancelled;self.reason=reason;self.transition('STOPPING',reason=reason,load_may_be_held=self.vacuum.holding,
                valve_state_not_changed_by_stop=True,automatic_release_sent=False)
    def tick(self):
        if self.state in TERMINAL|{'IDLE','PLAN_READY'}:return
        now=self.clock()
        try:
            status=self.robot.feedback();self.last_feedback=status
            self.record(kind='robot_status',feedback=status)
            if self.state=='STOPPING':
                # A previous arrival confirmation also proves a stationary arm
                # during vacuum dwell; it is rechecked against current real q.
                sample=self.robot.snapshot();now=self.clock();q=validate_robot(sample,now)
                stopped=status.get('stop_confirmed') or (status.get('arrival_confirmed') and np.max(abs(np.asarray(sample['velocity'])))<=np.deg2rad(.5))
                if stopped:
                    self.vacuum.unclaim();self.transition('CANCELLED' if self.cancelled else 'FAILED',reason=self.reason,stop_confirmed=True)
                elif status.get('state')=='STOP_UNCONFIRMED' or now-self.entered>2.:
                    self.transition('STOP_UNCONFIRMED',reason=self.reason,stop_confirmed=False)
                return
            sample=self.robot.snapshot();now=self.clock();q=validate_robot(sample,now)
            pressure=self.vacuum.pressure()
            now=self.clock()
            if (pressure.get('source')!='LIVE' or pressure.get('channel')!='gripper' or
                    pressure.get('integrity')!='CHECKED_BINARY' or
                    not 0<=now-pressure.get('received_s',-1)<=self.config['max_age_s']):
                raise ValueError('REAL_FRESH_GRIPPER_PRESSURE_REQUIRED')
            if self.stop_requested.is_set():return
            if status['state'] in {'FAILED','CANCELLED','STOP_UNCONFIRMED'}:raise ValueError('ROBOT_'+status['state']+':'+str(status.get('reason')))
            if self.state in {'DETACH','RETURN_LOADED'} and pressure['kpa']>self.config['hold_below_kpa']:raise ValueError('GRIPPER_PRESSURE_LOST')
            if self.state in MOVEMENTS:
                if status.get('task_id')!=self.motion_id:raise ValueError('ROBOT_TASK_ID_CHANGED')
                if status.get('state')!='SUCCEEDED' or not status.get('arrival_confirmed'):return
                if self.state=='APPROACH':self._motion(1)
                elif self.state=='CONTACT':
                    self.contact_check(q,self.plan)
                    self.transition('ESTABLISH_VACUUM');self.vacuum.establish(self.task_id)
                    self.vacuum_after_s=self.clock()
                elif self.state=='DETACH':self._motion(3)
                else:
                    if np.max(abs(q-self.plan['q_start']))>np.deg2rad(.01):raise ValueError('RETURN_START_NOT_CONFIRMED')
                    self.release_check(q,self.plan)
                    self.transition('RELEASE');self.vacuum.release(self.task_id)
                    self.vacuum_after_s=self.clock()
            elif self.state in {'ESTABLISH_VACUUM','RELEASE'}:
                gripping=self.state=='ESTABLISH_VACUUM';prefix='grip' if gripping else 'release'
                if gripping:self.contact_check(q,self.plan)
                elif np.max(abs(q-self.plan['q_start']))>np.deg2rad(.01):raise ValueError('ROBOT_MOVED_DURING_RELEASE')
                if now-self.entered>self.config[prefix+'_timeout_s']:raise ValueError(prefix.upper()+'_PRESSURE_TIMEOUT')
                if pressure['received_s']<=self.vacuum_after_s or pressure['received_s']==self.last_pressure:return
                self.last_pressure=pressure['received_s']
                met=pressure['kpa']<=self.config['grip_below_kpa'] if gripping else pressure['kpa']>=self.config['release_above_kpa']
                if not met:self.stable_since=None;return
                if self.stable_since is None:self.stable_since=pressure['received_s']
                if pressure['received_s']-self.stable_since>=self.config[prefix+'_dwell_s']:
                    if gripping:
                        self.transition('ATTACHMENT_CONFIRMED',pressure=pressure)
                        self.vacuum.monitor_holding=True;self._motion(2)
                    else:
                        self.vacuum.unclaim(released=True);self.physical_complete=self.physical_io
                        self.transition('COMPLETE',result='PHYSICAL_CYCLE_COMPLETE' if self.physical_io else 'FAKE_DEVICE_CYCLE_COMPLETE',release_pressure=pressure)
        except Exception as e:
            if self.state=='STOPPING':
                if now-self.entered>2.:self.transition('STOP_UNCONFIRMED',reason=str(e),stop_confirmed=False)
            else:self.stop(str(e))
    def status(self):
        return dict(task_id=self.task_id,state=self.state,reason=self.reason,holding=self.vacuum.holding,
                    physical_result='PHYSICAL_CYCLE_COMPLETE' if self.physical_complete else 'NOT_COMPLETE',
                    robot=self.last_feedback,stage_elapsed_s=self.clock()-self.entered)
