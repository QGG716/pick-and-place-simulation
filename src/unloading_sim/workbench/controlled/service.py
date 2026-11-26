"""Separate execution process: GUI/inference/planning cannot hold its GIL."""
import json
import multiprocessing as mp
from pathlib import Path
from queue import SimpleQueue
import signal
import threading
import time
import uuid
from .engine import Supervisor,MonitorConfig,TERMINAL,SubmissionRejected
from .transport import RealmanTransport,payload_matches
from .trajectory import expected_controller_payload
from .. import timing


def read_controller_payload(transport,mass):
    """Called only by the owner while idle, never on the timed sending thread."""
    code,tool=transport.sender.rm_get_current_tool_frame()
    result=dict(sdk_code=code,tool=tool,matches=code==0 and payload_matches(tool,mass),
                required_kg=mass,received_s=time.monotonic())
    transport.capabilities['payload_check']=result
    return result


def payload_error(result):
    if result['sdk_code']:
        return f"CONTROLLER_TOOL_QUERY_FAILED: 工具负载读取失败，SDK 返回 {result['sdk_code']}；未发送运动。"
    tool=result['tool'] if isinstance(result['tool'],dict) else {}
    mass_code=f"{result['required_kg']:g}".replace('.','_')
    return (f"CONTROLLER_PAYLOAD_MUST_MATCH_{mass_code}_KG: 控制器当前工具 {tool.get('name','—')} 的负载为 "
            f"{tool.get('payload','未知')} kg，需要 {result['required_kg']:g} kg。请在控制器中选择/设置本次确认的控制器负载，"
            '然后点击“刷新工具负载（只读）”并重新规划。5 kg 是机械臂额定能力。未发送运动。')


SENDER_REALTIME_PRIORITY=20


def sender_realtime(setting=None):
    """Ask SCHED_FIFO for the calling (sender) thread; fall back to the normal
    class and say so. Returns the record stored in the controller capabilities."""
    import os
    setting=setting or {}
    if setting.get('mode')=='DISABLED':
        return dict(requested='NONE',obtained='SCHED_OTHER',reason='DISABLED_BY_CONFIGURATION')
    priority=int(setting.get('priority',SENDER_REALTIME_PRIORITY))
    record=dict(requested='SCHED_FIFO',priority=priority)
    try:
        os.sched_setscheduler(0,os.SCHED_FIFO,os.sched_param(priority))
        record.update(obtained='SCHED_FIFO')
    except (PermissionError,OSError,AttributeError,ValueError) as error:
        record.update(obtained='SCHED_OTHER',error=f'{type(error).__name__}: {error}',
                      hint='grant rtprio (e.g. /etc/security/limits.d) to the field user and log in again')
    try:record['policy_now']=int(os.sched_getscheduler(0))
    except Exception:pass
    return record


SENDER_SWITCH_INTERVAL_S=.0005


def sender_switch_interval(setting=None):
    """Interpreter thread switch interval of the execution process.

    The timed sender shares this process with the record writer and the
    feedback receiver; with the interpreter default of 5 ms a woken sender may
    wait that long for the lock held by another thread (field 2026-11-26 task
    121c71bb: the sender woke 2.6 ms late after 4672 of 7000 loaded-return
    points, SEND_DEADLINE_MISSED; jitter otherwise <= 0.7 ms). 0.5 ms, as the
    planning process uses since 2026-11-23 (python_switch_interval_s). No
    monitor threshold changes."""
    import sys
    value=SENDER_SWITCH_INTERVAL_S if setting is None else float(setting)
    if not 1e-4<=value<=5e-3:raise ValueError('SENDER_SWITCH_INTERVAL_OUT_OF_RANGE: 1e-4 .. 5e-3 s')
    previous=sys.getswitchinterval();sys.setswitchinterval(value)
    return dict(requested_s=value,obtained_s=sys.getswitchinterval(),previous_s=previous)


def worker(pipe, mapping, config, stop_event, cancel_epoch=None):
    switch=sender_switch_interval(config.get('sender_switch_interval_s'))
    transport=RealmanTransport(mapping,config);transport.external_stop=stop_event;events=SimpleQueue();record_file=None
    transport.capabilities['sender_switch_interval']=switch
    def write_records():
        nonlocal record_file
        while True:
            msg=events.get()
            if msg is None:return
            if '_file' in msg:
                # Opened synchronously by the submit path before the engine
                # accepts the package; a failure there rejects the submit.
                try:
                    if record_file:record_file.close()
                except Exception:pass
                record_file=msg['_file']
            elif '_timing' in msg:
                # Measurement only (ECO65_TIMING=1): submit steps of this task,
                # written by this record thread after the engine accepted it.
                try:Path(msg['_timing']['task_dir'],'submit-timing.json').write_text(json.dumps(msg['_timing'],default=str))
                except Exception:pass
            elif record_file:
                try:
                    record_file.write(json.dumps(msg,allow_nan=False)+'\n')
                    if msg['kind']=='result':
                        (Path(record_file.name).parent/'result.json').write_text(json.dumps(msg,indent=2))
                except Exception:engine.stop('LOG_WRITE_FAILED')
    writer=threading.Thread(target=write_records,daemon=True);writer.start()
    engine=Supervisor(transport,time.monotonic,events.put,MonitorConfig(**config.get('monitor',{})))
    closing=threading.Event();last_tick=time.monotonic()
    def run():
        nonlocal last_tick
        # Real-time priority for the timed sender thread where the system
        # grants it (AGENTS 2026-11-16 section 4: no system change, report the
        # class obtained). Field 2026-11-26 task 3ffe1c25: one >2 ms stall of
        # the normal-priority sender stopped the loaded return
        # (SEND_DEADLINE_MISSED; p99 jitter otherwise 0.6 ms). Thresholds unchanged.
        transport.capabilities['sender_scheduling']=sender_realtime(config.get('sender_realtime'))
        while not closing.is_set():
            try:
                if stop_event.is_set():engine.cancel()
                engine.tick(transport.latest)
            except Exception as e:engine.stop('SUPERVISOR_ERROR:'+str(e))
            last_tick=time.monotonic()
            # No backlog loop: at most one command per tick.
            time.sleep(.0005)
    thread=threading.Thread(target=run,daemon=True,name='timed-canfd');thread.start()
    active_package=None
    try:
        while True:
            if stop_event.is_set():
                engine.cancel()
            # Independent watchdog still requests stop if a vendor call stalls,
            # and latches STOP_UNCONFIRMED when the stop timeout passes while
            # the timed thread cannot run tick() itself.
            if engine.state=='EXECUTING' and time.monotonic()-last_tick>.02:engine.stop('SEND_THREAD_STALLED')
            if engine.state=='STOPPING' and time.monotonic()-last_tick>.02:engine.expire_stalled_stop()
            if not pipe.poll(.01):continue
            ident,op,arg=pipe.recv()
            received=time.monotonic();lap=timing.Laps('child.submit.') if op=='submit' else None
            attempt_dir=None;attempt_id=None
            try:
                if op=='connect':
                    if engine.state=='STOP_UNCONFIRMED':raise RuntimeError('STOP_UNCONFIRMED_REQUIRES_LOCAL_INTERVENTION')
                    engine.reset_continuity();value=transport.connect(*arg)
                elif op=='snapshot':value=transport.latest
                elif op=='feedback':value=dict(engine.feedback(transport.latest),connected=transport.connected,capabilities=transport.capabilities)
                elif op=='check_payload':
                    if not transport.connected:raise RuntimeError('DISCONNECTED')
                    if engine.state not in {'READY','SUCCEEDED','CANCELLED','FAILED'}:raise RuntimeError('TASK_ACTIVE_OR_STOP_UNCONFIRMED')
                    value=read_controller_payload(transport,expected_controller_payload(config))
                elif op=='submit':
                    sent=arg.get('timing_sent_monotonic')
                    ticket=arg.get('cancel_epoch');arg=arg.get('package',arg)
                    if not transport.connected:raise RuntimeError('DISCONNECTED')
                    if engine.state not in {'READY','SUCCEEDED','CANCELLED','FAILED'}:raise RuntimeError('TASK_ACTIVE_OR_STOP_UNCONFIRMED')
                    task_id=uuid.uuid4().hex;record_root=Path(config['record_root'])
                    if arg.get('schema')=='eco65_single_carton_segment_v1':
                        cycle_id=arg['binding']['physical_cycle']['task_id']
                        if len(cycle_id)!=32 or any(x not in '0123456789abcdef' for x in cycle_id):raise ValueError('INVALID_CYCLE_TASK_ID')
                        record_root=record_root/cycle_id/'motion'
                    task_dir=record_root/task_id;task_dir.mkdir(parents=True)
                    attempt_dir,attempt_id=task_dir,task_id
                    lap('mkdir')
                    if timing.ENABLED:
                        dumping=time.monotonic();text=json.dumps(arg,indent=2,allow_nan=False);dumps_s=time.monotonic()-dumping
                        writing=time.monotonic();(task_dir/'trajectory-package.json').write_text(text)
                        timing.add('child.submit.trajectory_package_json_dumps',dumping,dumps_s,bytes=len(text))
                        timing.add('child.submit.trajectory_package_json_write',writing,time.monotonic()-writing,bytes=len(text))
                        lap.t=time.monotonic()
                    else:(task_dir/'trajectory-package.json').write_text(json.dumps(arg,indent=2,allow_nan=False))
                    (task_dir/'task.json').write_text(json.dumps({'task_id':task_id,'directory':str(task_dir)}))
                    lap('task_json_write')
                    transport.check_limits(arg['prepared'])
                    lap('check_limits')
                    if cancel_epoch is not None:
                        with cancel_epoch.get_lock():
                            if ticket!=cancel_epoch.value:raise RuntimeError('SUBMISSION_CANCELLED_BEFORE_MOTION')
                            stop_event.clear()
                    else:stop_event.clear()
                    lap('cancel_epoch_check')
                    # No active sender while this bounded TCP query occurs.
                    payload=read_controller_payload(transport,expected_controller_payload(config))
                    lap('controller_payload_query')
                    transport.capabilities['submit_tool_frame']=payload['tool']
                    (task_dir/'controller.json').write_text(json.dumps(transport.capabilities,indent=2))
                    lap('controller_json_write')
                    if not payload['matches']:raise RuntimeError(payload_error(payload))
                    if stop_event.is_set():raise RuntimeError('SUBMISSION_CANCELLED_BEFORE_MOTION')
                    # A task without its event record never starts.
                    try:handle=open(task_dir/'events.jsonl','a',buffering=1)
                    except OSError as error:raise RuntimeError('LOG_OPEN_FAILED_NO_MOTION:'+str(error)) from error
                    events.put({'_file':handle})
                    lap('events_file_open')
                    if stop_event.is_set():raise RuntimeError('SUBMISSION_CANCELLED_BEFORE_MOTION')
                    value=engine.submit(arg['prepared'],transport.latest,task_id);active_package=arg
                    lap('engine_submit')
                    if timing.ENABLED:
                        # Only this submit's own spans: never serialize unrelated spans here.
                        spans,overhead=timing.recorder.take(since=received)
                        events.put({'_timing':dict(schema='eco65_timing_v1',phase='child_submit',task_id=task_id,task_dir=str(task_dir),
                            clock='time.monotonic',parent_sent_monotonic_s=sent,child_received_monotonic_s=received,
                            pipe_transfer_s=None if sent is None else received-sent,first_send_due_monotonic_s=engine.start_time,
                            measurement_overhead_s=overhead,events=spans)})
                elif op=='cancel':
                    if arg!=engine.task_id:raise ValueError('UNKNOWN_TASK')
                    engine.cancel();value=None
                elif op in ('disconnect','close'):
                    if engine.state not in TERMINAL|{'READY'}:
                        engine.cancel();raise RuntimeError('STOPPING_WAIT_FOR_FEEDBACK')
                    if engine.state=='STOP_UNCONFIRMED':raise RuntimeError('STOP_UNCONFIRMED_KEEP_CONNECTION_AND_USE_LOCAL_STOP')
                    engine.reset_continuity();transport.disconnect();value=None
                    if op=='close':pipe.send((ident,True,value));break
                else:raise ValueError('UNKNOWN_OPERATION')
                pipe.send((ident,True,value))
            except Exception as e:
                if attempt_dir is not None and engine.task_id!=attempt_id:
                    (attempt_dir/'result.json').write_text(json.dumps(dict(task_id=attempt_id,state='REJECTED',reason=str(e),hardware_motion='NOT_STARTED')))
                pipe.send((ident,False,str(e)))
    except (EOFError,BrokenPipeError,KeyboardInterrupt) as e:
        if isinstance(e,KeyboardInterrupt):
            # Terminal Ctrl-C reaches the whole process group. Stop the active
            # segment and wait for feedback; a repeated Ctrl-C must not skip it.
            signal.signal(signal.SIGINT,signal.SIG_IGN)
            engine.stop('TERMINAL_INTERRUPT_SOFTWARE_STOP',True)
        else:engine.cancel()
        deadline=time.monotonic()+engine.c.stop_timeout_s+.2
        while engine.state not in TERMINAL|{'READY'} and time.monotonic()<deadline:time.sleep(.01)
    finally:
        closing.set();thread.join(.2)
        # Never close transport while a vendor send or stop is still running.
        if not thread.is_alive() and engine.state!='STOP_UNCONFIRMED':transport.disconnect()
        events.put(None);writer.join(1)
        if record_file:record_file.close()


class ExecutionService:
    def __init__(self,mapping,config,authority,binding):
        self.mapping,self.config,self.authority,self.binding=mapping,config,authority,binding
        self.process=None;self._lock=threading.Lock();self._cache={'state':'DISCONNECTED','connected':False};self._id=0
        self._poisoned=False;self._active_id=None
    def _call(self,op,arg=None):
        with self._lock:
            if self._poisoned:raise RuntimeError('IPC_FAILED_STOP_UNCONFIRMED_NO_RECONNECT')
            if self.process is None:
                if op!='connect':raise RuntimeError('DISCONNECTED')
                ctx=mp.get_context('spawn');self.pipe,child=ctx.Pipe()
                self.stop_event=ctx.Event()
                self.cancel_epoch=ctx.Value('Q',0)
                self.process=ctx.Process(target=worker,args=(child,self.mapping,self.config,self.stop_event,self.cancel_epoch),daemon=False);self.process.start();child.close()
            self._id+=1;ident=self._id
            if timing.ENABLED:
                if op=='submit' and isinstance(arg,dict):arg=dict(arg,timing_sent_monotonic=time.monotonic())
                with timing.aggregated('ipc.pipe_send',op):self.pipe.send((ident,op,arg))
            else:self.pipe.send((ident,op,arg))
            waiting=timing.now()
            if not self.pipe.poll(10):
                # Advance the cancel epoch too: a submit still inside its
                # pre-motion checks compares the epoch and would otherwise
                # clear a bare stop flag and start after this timeout.
                self._poisoned=True;self.request_stop();self.authority.invalidate()
                raise RuntimeError('EXECUTION_PROCESS_TIMEOUT_STOP_STATUS_UNKNOWN')
            got,ok,value=self.pipe.recv()
            timing.aggregate('ipc.wait_and_receive_reply',op,timing.now()-waiting)
            if got!=ident:raise RuntimeError('IPC_SEQUENCE_ERROR')
            # An explicit submit reply of failure is raised before engine
            # acceptance in the child: nothing was sent for that package.
            if not ok:raise (SubmissionRejected if op=='submit' else RuntimeError)(value)
            return value
    def connect(self,host,port):
        self.authority.invalidate();return self._call('connect',(host,port))
    def snapshot(self):return self._call('snapshot')
    def check_payload(self):return self._call('check_payload')
    def submission_ticket(self):
        return self.cancel_epoch.value if hasattr(self,'cancel_epoch') else None
    def submit(self,trajectory,*,cancel_ticket=None):
        # Source files rehashed here, immediately before the child rechecks start.
        ticket=self.submission_ticket() if cancel_ticket is None else cancel_ticket
        if trajectory.get('schema')=='eco65_single_carton_segment_v1':
            from ..contracts import HOST_SESSION
            observation=trajectory.get('binding',{}).get('physical_cycle',{}).get('observation',{})
            stamp=observation.get('stamp',{})
            if (stamp.get('source')!='LIVE' or stamp.get('host_session')!=HOST_SESSION or
                    observation.get('robot_sample',{}).get('source')!='LIVE'):
                self.authority.invalidate()
                raise SubmissionRejected('SINGLE_CARTON_REPLAY_OR_FOREIGN_OBSERVATION_REJECTED')
        # Consume even rejected starts: an old preview can never be retried.
        try:
            if timing.ENABLED:
                with timing.span('service.submit.binding_rehash'):binding=self.binding()
                with timing.span('service.submit.authority_claim'):self.authority.claim(trajectory,binding)
            else:self.authority.claim(trajectory,self.binding())
        except Exception as e:raise SubmissionRejected(str(e)) from e
        with timing.span('service.submit.ipc'):task=self._call('submit',dict(package=trajectory,cancel_epoch=ticket))
        self._active_id=task;return task
    def feedback(self,task_id=None):
        try:self._cache=self._call('feedback')
        except (RuntimeError,EOFError,BrokenPipeError,OSError) as e:
            self.request_stop();self._poisoned=True;self.authority.invalidate()
            self._cache=dict(state='STOP_UNCONFIRMED',task_id=self._active_id,connected=False,reason=str(e),feedback_age_s=None,arrival_confirmed=False,stop_confirmed=False)
        if task_id and self._cache['task_id']!=task_id:raise ValueError('UNKNOWN_TASK')
        return self._cache
    def cancel(self,task_id):
        if task_id!=self._active_id:raise ValueError('UNKNOWN_TASK')
        self.request_stop()
    def request_stop(self):
        # Dedicated signal: never waits on the GUI/status RPC lock.
        if self.process is not None:
            if hasattr(self,'cancel_epoch'):
                with self.cancel_epoch.get_lock():
                    self.cancel_epoch.value+=1;self.stop_event.set()
            else:self.stop_event.set()
    def disconnect(self):self._call('disconnect');self.authority.invalidate()
    def close(self):
        if self.process is not None:
            self._call('close');self.process.join(2);self.process=None;self.authority.invalidate()
