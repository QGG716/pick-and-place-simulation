"""Deterministic motion supervisor. SDK success is never arrival evidence."""
from dataclasses import dataclass, asdict
import math
import uuid
import numpy as np

TERMINAL={'SUCCEEDED','CANCELLED','FAILED','STOP_UNCONFIRMED'}


class SubmissionRejected(ValueError, RuntimeError):
    """Definitive pre-motion refusal: the timed sender never accepted this package.

    Raised only where no point can have been sent for it (host-side permit or
    observation checks, or an explicit reply from the execution process).
    Timeouts and IPC failures are unknown outcomes and never use this type.
    """

@dataclass(frozen=True)
class MonitorConfig:
    start_rad: float=.0001745329252  # 0.01 degree: quiet-state repeat noise allowance
    tracking_rad: float=.05235987756  # 3 degrees, sustained for tracking_dwell_s
    tracking_dwell_s: float=.10
    arrival_rad: float=.00872664626
    stopped_rad_s: float=.00872664626
    feedback_timeout_s: float=.10
    dwell_s: float=.4
    completion_timeout_s: float=5.
    stop_timeout_s: float=2.
    jitter_s: float=.002
    max_send_gap_s: float=.009
    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in asdict(self).values()):raise ValueError('INVALID_MONITOR_CONFIG')
        if self.max_send_gap_s>.010 or self.jitter_s>=.005:raise ValueError('HIGH_FOLLOW_TIMING_UNSUPPORTED')

# The reviewed defaults are the loosest values a field configuration may use.
# A larger value of these delays or widens a fault/arrival decision; a smaller
# dwell shortens the stationary evidence. Field files may only tighten them.
LOOSER_WHEN_LARGER=('start_rad','tracking_rad','tracking_dwell_s','arrival_rad','stopped_rad_s',
                    'feedback_timeout_s','completion_timeout_s','stop_timeout_s','jitter_s','max_send_gap_s')
LOOSER_WHEN_SMALLER=('dwell_s',)
# A following segment may start from the command the controller still holds
# only when its first point is that command (planner chains to 1e-7 rad).
CONTINUATION_RAD=1e-7


def field_monitor_config(values):
    """Monitor thresholds from a field configuration file: never looser than reviewed."""
    values=dict(values or {});reviewed=MonitorConfig()
    unknown=set(values)-set(asdict(reviewed))
    if unknown:raise ValueError('UNKNOWN_MONITOR_FIELDS:'+','.join(sorted(unknown)))
    config=MonitorConfig(**values)
    looser=[k for k in LOOSER_WHEN_LARGER if getattr(config,k)>getattr(reviewed,k)]
    looser+=[k for k in LOOSER_WHEN_SMALLER if getattr(config,k)<getattr(reviewed,k)]
    if looser:raise ValueError('MONITOR_THRESHOLDS_LOOSER_THAN_REVIEWED:'+','.join(looser))
    if config.dwell_s>=config.stop_timeout_s:raise ValueError('MONITOR_DWELL_MUST_FIT_STOP_TIMEOUT')
    return config

class Supervisor:
    def __init__(self, io, clock, record, config=MonitorConfig()):
        self.io,self.clock,self.record,self.c=io,clock,record,config
        self.state='READY';self.task_id=None;self.reason='';self.last_seq=None
        self.start_time=None;self.last_send=None;self.last_feedback=None
        self.start_q=None
        self.command=None;self.actual=None;self.error=None;self.index=0
        self.stable_since=None;self.excess_since=None;self.stop_time=None
        self.stop_target=None;self.stop_ack=False;self.stop_reply_seen=False;self.sent=False
        # Last command of a task that SUCCEEDED with every send accepted: the
        # controller is still holding exactly this target. Cleared by any stop,
        # failure, new task, connection change or unknown outcome.
        self.hold_command=None
    def emit(self, kind, **values):self.record(dict(kind=kind,monotonic_s=self.clock(),task_id=self.task_id,**values))
    def reset_continuity(self):self.hold_command=None
    def fresh(self, sample):
        return sample is not None and sample.get('source')=='LIVE' and isinstance(sample.get('received_s'),(int,float)) and math.isfinite(sample['received_s']) and isinstance(sample.get('sequence'),int) and 0<=self.clock()-sample['received_s']<=self.c.feedback_timeout_s and len(sample.get('q',[]))==6 and np.isfinite(sample['q']).all()
    def submit(self, prepared, sample, task_id=None):
        if self.state not in {'READY','SUCCEEDED','CANCELLED','FAILED'}:raise RuntimeError('TASK_ACTIVE_OR_STOP_UNCONFIRMED')
        if not self.fresh(sample):raise ValueError('FRESH_FEEDBACK_REQUIRED')
        if sample.get('alarm') or not sample.get('enabled',False):raise ValueError('CONTROLLER_NOT_READY')
        first=np.asarray(prepared['points'][0]['q'],float)
        # A chained segment begins at the command the controller already holds,
        # so its first send is no step even though the loaded arm rests within
        # the arrival tolerance of it. Any other start must match measured q.
        continuing=(self.state=='SUCCEEDED' and self.hold_command is not None and
                    np.max(abs(first-np.asarray(self.hold_command,float)))<=CONTINUATION_RAD)
        waived=(not continuing and isinstance(prepared.get('start_joint_check'),dict) and
                prepared['start_joint_check'].get('status')=='START_JOINT_CHECK_DISABLED_BY_USER' and
                bool(prepared['start_joint_check'].get('basis')))
        limit=self.c.arrival_rad if continuing else self.c.start_rad
        start_error=float(np.max(abs(np.asarray(sample['q'])-first)))
        if start_error>limit and not waived:raise ValueError('START_CHANGED_REPLAN_REQUIRED')
        if not self.stationary(sample):raise ValueError('START_NOT_STATIONARY')
        self.io.arm() if hasattr(self.io,'arm') else None
        self.task_id=task_id or uuid.uuid4().hex;self.prepared=prepared;self.index=0
        self.start_q=list(sample['q']);self.hold_command=None
        self.state='PREPARING';self.reason='';self.stable_since=self.excess_since=self.last_send=None
        self.last_seq=None;self.stop_time=None;self.stop_ack=False;self.stop_reply_seen=False;self.sent=False
        self.start_time=self.clock()+prepared['period_s']*2
        self.emit('accepted',monitor=asdict(self.c),start_feedback=sample,start_error_rad=start_error,start_limit_rad=limit,
                  start_rule='CONTINUES_HELD_COMMAND_OF_PREVIOUS_SUCCEEDED_SEGMENT' if continuing else
                  'USER_DISABLED_START_JOINT_CHECK' if waived else 'MEASURED_STATIONARY_START',
                  start_joint_check=prepared.get('start_joint_check'))
        self.state='EXECUTING';return self.task_id
    def stationary(self,s):
        v=s.get('velocity')
        return v is not None and len(v)==6 and np.isfinite(v).all() and np.max(abs(np.asarray(v)))<=self.c.stopped_rad_s
    def stop(self, reason, cancelled=False):
        self.hold_command=None
        if self.state in TERMINAL or self.state in {'READY','STOPPING'}:return
        self.reason=reason;self.state='STOPPING';self.stop_time=self.clock();self.stable_since=None;self.last_seq=None
        self.stop_target='CANCELLED' if cancelled else 'FAILED';self.stop_ack=False;self.stop_reply_seen=False
        self.emit('stop_requested',reason=reason)
        self.io.request_stop()  # independent channel; must return immediately
    def cancel(self):self.stop('OPERATOR_SOFTWARE_STOP',True)
    def after_inflight_stop(self):
        # A call already in the vendor library may return after an independent
        # stop was issued. Reassert stop only after that call has left the SDK;
        # discard earlier stop evidence and require a new reply + feedback dwell.
        self.stop_time=self.clock();self.stable_since=None;self.last_seq=None
        self.stop_ack=False;self.stop_reply_seen=False
        self.emit('stop_reasserted_after_inflight_send')
        self.io.request_stop()
    def after_inflight_send(self):
        """A vendor call returned after a stop: re-request it, keep any latch."""
        if self.state=='STOPPING':self.after_inflight_stop()
        else:
            self.emit('stop_reasserted_after_inflight_send',state=self.state)
            self.io.request_stop()
    def expire_stalled_stop(self):
        """Watchdog path while the timed thread is blocked inside a vendor call.

        Without it a hung send keeps STOPPING forever, since only tick() applies
        the stop timeout. A send that returns later still re-requests the stop.
        """
        if self.state=='STOPPING' and self.clock()-self.stop_time>=self.c.stop_timeout_s:
            self.state='STOP_UNCONFIRMED';self.hold_command=None
            self.emit('result',state=self.state,reason=self.reason,timed_thread_stalled=True)
    def tick(self,sample):
        if self.state in TERMINAL or self.state in {'READY','PREPARING'}:return
        now=self.clock()
        ack=self.io.stop_result()
        if ack is not None and self.state=='STOPPING' and not self.stop_reply_seen:
            self.stop_reply_seen=True;self.stop_ack=ack==0;self.emit('stop_return',return_code=ack,transport=getattr(self.io,'stop_api','FAKE_SDK'))
        fresh=self.fresh(sample)
        new=fresh and sample.get('sequence')!=self.last_seq
        if new:
            self.last_seq=sample['sequence'];self.last_feedback=sample['received_s'];self.actual=sample['q']
            self.emit('feedback',sample=sample,feedback_age_s=now-sample['received_s'])
        if self.state=='STOPPING':
            if new and sample['received_s']>=self.stop_time and self.stationary(sample):
                if self.stable_since is None:self.stable_since=sample['received_s']
                if self.stop_ack and sample['received_s']-self.stable_since>=self.c.dwell_s:
                    self.state=self.stop_target;self.emit('result',state=self.state,reason=self.reason);return
            elif not fresh or (new and not self.stationary(sample)):self.stable_since=None
            if now-self.stop_time>=self.c.stop_timeout_s:
                self.state='STOP_UNCONFIRMED';self.emit('result',state=self.state,reason=self.reason)
            return
        if not fresh:self.stop('FEEDBACK_EXPIRED');return
        if sample.get('alarm') or not sample.get('enabled',False):self.stop('CONTROLLER_ALARM_OR_DISABLED');return
        elapsed=now-self.start_time;pts=self.prepared['points'];period=self.prepared['period_s']
        if new and self.last_send is not None:
            # Compare to the command at feedback receipt, not a later GUI time.
            k=max(0,min(self.index-1,int(max(0,sample['received_s']-self.start_time)/period)))
            self.error=(np.asarray(sample['q'])-pts[k]['q']).tolist()
            self.emit('tracking',error_rad=self.error,command_index=k)
            if max(abs(x) for x in self.error)>self.c.tracking_rad:
                if self.excess_since is None:self.excess_since=now
                if now-self.excess_since>=self.c.tracking_dwell_s:self.stop('TRACKING_ERROR');return
            else:self.excess_since=None
        if self.index<len(pts):
            due=self.start_time+pts[self.index]['t']
            # Deadline and gap are judged at the vendor call itself, after the
            # feedback/record work above, not at the start of this tick.
            begin=self.clock()
            if begin<due:return
            if begin-due>self.c.jitter_s or (self.last_send is not None and begin-self.last_send>self.c.max_send_gap_s):
                self.stop('SEND_DEADLINE_MISSED');return
            p=pts[self.index]
            try:code=self.io.send(p['q'])
            except Exception as e:
                self.emit('send_exception',error=str(e))
                if self.state in {'STOPPING','STOP_UNCONFIRMED'}:self.after_inflight_send()
                else:self.stop('SDK_SEND_EXCEPTION')
                return
            end=self.clock();self.last_send=begin;self.sent=True;self.command=p['q']
            self.emit('send',index=self.index,due_s=due,sent_s=begin,returned_s=end,jitter_s=begin-due,q_rad=p['q'],sdk_code=code)
            self.index+=1
            if self.state in {'STOPPING','STOP_UNCONFIRMED'}:self.after_inflight_send();return
            if code!=0:self.stop('SDK_SEND_ERROR');return
            if end-begin>self.c.jitter_s:self.stop('SDK_SEND_OVERRUN');return
        else:
            if new and sample['received_s']>=self.last_send and self.stationary(sample) and max(abs(np.asarray(sample['q'])-pts[-1]['q']))<=self.c.arrival_rad:
                if self.stable_since is None:self.stable_since=sample['received_s']
                if sample['received_s']-self.stable_since>=self.c.dwell_s and self.state=='EXECUTING':
                    self.state='SUCCEEDED';self.hold_command=list(pts[-1]['q'])
                    self.emit('result',state=self.state);return
            elif new:self.stable_since=None
            if elapsed>self.prepared['send_duration_s']+self.c.completion_timeout_s:self.stop('ARRIVAL_TIMEOUT')
    def feedback(self,sample):
        goal=self.prepared['points'][-1]['q'] if hasattr(self,'prepared') else None
        return dict(task_id=self.task_id,state=self.state,reason=self.reason,command_q=self.command,
                    start_q=self.start_q,
                    goal_q=goal,goal_error_rad=None if goal is None or sample is None else (np.asarray(sample['q'])-goal).tolist(),
                    actual_q=None if sample is None else sample['q'],tracking_error_rad=self.error,
                    feedback_age_s=None if sample is None else self.clock()-sample['received_s'],
                    sent_points=self.index,total_points=len(self.prepared['points']) if hasattr(self,'prepared') else 0,
                    arrival_confirmed=self.state=='SUCCEEDED',stop_confirmed=self.state in {'CANCELLED','FAILED'})
