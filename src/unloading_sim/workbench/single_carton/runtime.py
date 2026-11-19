"""Task-bound observation, native planning, pre-execution recheck and records."""
from dataclasses import asdict
from pathlib import Path
from queue import SimpleQueue,Empty
import json
import threading
import time
import uuid
import numpy as np
from .config import issues,pre_execute_visual_recheck,execution_lock
from .cycle import Cycle,TERMINAL
from .geometry import candidate_contact_check,contact_candidates
from .size_policy import neighbor_depth_bound,size_policy,mark_incomplete_scene
from .perception import make_detector,freeze,reconstruct,unchanged,target_unchanged,same_session_start_and_camera
from .. import timing


class Journal:
    def __init__(self,folder,task_id):
        self.folder=folder;self.task_id=task_id;self.events=SimpleQueue();self.error=None
        self.post_lock=threading.Lock();self.closed=False
        self.thread=threading.Thread(target=self._write,daemon=True,name='single-carton-records');self.thread.start()
    def __call__(self,**event):
        self.events.put(dict(task_id=self.task_id,host_monotonic_s=time.monotonic(),host_utc_ns=time.time_ns(),**event))
    def _write(self):
        try:
            with (self.folder/'events.jsonl').open('a',buffering=1) as f:
                while True:
                    event=self.events.get()
                    if event is None:break
                    if isinstance(event,threading.Event):
                        f.flush();event.set();continue
                    if callable(event):event();continue
                    f.write(json.dumps(event,allow_nan=False)+'\n')
        except Exception as e:
            with self.post_lock:self.error=str(e)
            # Accepted writes must finish with an observable error if the
            # journal dies before dispatching them, rather than wait forever.
            while True:
                try:pending=self.events.get_nowait()
                except Empty:break
                if isinstance(pending,timing.WriteReceipt):pending.fail(ValueError('TASK_RECORD_UNAVAILABLE:'+self.error))
    def defer(self,write):
        # Only queue operations hold this lock. Never wait for disk IO or
        # start a thread on a planning caller that has exhausted its budget.
        with self.post_lock:
            if self.closed or self.error or not self.thread.is_alive():
                raise ValueError('TASK_RECORD_UNAVAILABLE:'+str(self.error))
            self.events.put(write)
    def flush(self,timeout=120,cancelled=lambda:False):
        done=threading.Event();self.events.put(done);end=time.monotonic()+timeout
        while True:
            if cancelled():raise ValueError('CANCELLED')
            if self.error or not self.thread.is_alive():raise ValueError('TASK_RECORD_UNAVAILABLE:'+str(self.error))
            if done.is_set():return
            left=end-time.monotonic()
            if left<=0:raise ValueError('TASK_RECORD_FLUSH_TIMEOUT')
            done.wait(min(.01,left))
    def close(self):
        with self.post_lock:
            if not self.closed:self.closed=True;self.events.put(None)
        self.thread.join(2)


def flush_task_records(manager):
    """One startup barrier covers observation, planning and task journal writes."""
    try:
        for writer in (getattr(manager,'evidence',None),getattr(manager.planner,'evidence',None),manager.journal):
            if writer is not None:writer.flush(timeout=120,cancelled=manager.stop_event.is_set)
    except Exception as error:
        raise ValueError('TASK_RECORD_UNAVAILABLE_NO_MOTION:'+str(error)) from error


class TaskManager:
    def __init__(self,backend,service,vacuum,planner,model,calibration,config,notify=lambda *a:None):
        self.backend,self.service,self.vacuum,self.planner=backend,service,vacuum,planner
        self.model,self.calibration,self.config,self.notify=model,calibration,config,notify
        self.detector=None;self.cycle=None;self.observation=None;self.plan=None;self.boxes=[];self.result=None
        self.scene=None;self.self_filter=None
        self.task_id=None;self.folder=None;self.journal=None;self.phase='IDLE';self.reason=''
        self.stop_event=threading.Event();self.closing=threading.Event();self.busy=False;self.lock=threading.Lock()
        self.evidence=None  # observation evidence writer (async_evidence only)
        self.thread=threading.Thread(target=self._monitor,daemon=True,name='single-carton-supervisor');self.thread.start()
        # resident_workers: start and initialise the native workers in the
        # background as soon as the entry starts (no motion or valve command).
        if (config or {}).get('resident_workers') is True and hasattr(planner,'prewarm'):planner.prewarm()
    @property
    def active(self):return self.busy or (self.cycle is not None and self.cycle.active)
    def progress(self,text):self.notify('progress',text)
    def _enter(self):
        with self.lock:
            if self.active or self.vacuum.holding or (self.cycle and self.cycle.state=='STOP_UNCONFIRMED'):raise ValueError('TASK_ACTIVE_OR_HELD_LOAD')
            self.busy=True
    def save_observation(self,observation,name):
        timing.write_json(self.folder/(name+'.json'),observation.metadata(),name+'.json',indent=2)
        if self.evidence is not None:
            self.evidence.write_npz(self.folder/(name+'.npz'),dict(rgb=observation.frame.rgb,depth=observation.frame.depth),name+'.npz')
            return
        with timing.span('disk.write',file=name+'.npz'):
            np.savez_compressed(self.folder/(name+'.npz'),rgb=observation.frame.rgb,depth=observation.frame.depth)
    def write_record(self,name,value,**options):
        """Observation evidence: through the async writer when enabled, else written now."""
        if self.evidence is not None:self.evidence.write_json(self.folder/name,value,name,**options)
        else:timing.write_json(self.folder/name,value,name,**options)
    def evidence_flush(self):
        """async_evidence: every observation record on disk, or refuse (no motion without records)."""
        if self.evidence is not None:self.evidence.flush(timeout=120)
    def observe(self):
        self._enter();self.stop_event.clear();self.planner.authority.invalidate();self.plan=None;self.cycle=None
        self.observation=None;self.result=None;self.boxes=[];self.scene=None;self.reason=''
        entered=timing.now()
        try:
            if self.journal:self.journal.close()
            self.journal=None;self.folder=None
            self.task_id=uuid.uuid4().hex;folder=Path(self.config['record_root'])/self.task_id;folder.mkdir(parents=True)
            self.folder=folder;self.journal=Journal(folder,self.task_id)
            if self.evidence is not None:self.evidence.close();self.evidence=None
            if self.config.get('async_evidence') is True:
                from .evidence import EvidenceWriter
                self.evidence=EvidenceWriter('observation-evidence')
            timing.write_json(folder/'result.json',dict(task_id=self.task_id,physical_result='NOT_RUN'),'result.json')
            missing=issues(self.config)
            if missing:raise ValueError('缺少现场参数：\n'+'\n'.join(missing))
            self.phase='OBSERVING';started=time.monotonic();self.progress('等待机械臂停稳，锁定 q_start 与同帧 RGB-D')
            with timing.span('observe.freeze'):
                self.observation=freeze(self.backend,self.service,self.model,self.calibration,
                                        cancelled=self.stop_event.is_set,progress=self.progress)
            self.notify('capture',self.observation)
            self.save_observation(self.observation,'observation')
            self.progress('q_start 与原始 RGB-D 已锁定；等待本次冻结帧的检测 / 分割')
            vision=getattr(self.backend,'vision',None)
            if vision:
                with timing.span('observe.vision_infer_frozen'):
                    self.result=vision.infer_frozen(self.observation.frame,self.observation.generation,
                                                  self.stop_event.is_set,progress=self.progress)
            else:
                with timing.span('observe.detector_detect'):
                    if self.detector is None:self.detector=make_detector(**self.config['detector'])
                    self.result=self.detector.detect(self.observation.frame)
            if self.stop_event.is_set() or self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.progress('本次同帧检测 / 分割已完成；恢复箱体与实测接触面')
            with timing.span('observe.reconstruct'):
                self.boxes,rejected=reconstruct(self.observation,self.result,self.config.get('geometry'))
            if self.config.get('site'):
                from .site_scene import build_scene
                from .self_filter import RobotSelfFilter
                clear=self.config['site']['unknown_policy']=='USER_CONFIRMED_CLEAR'
                self.progress('构建同帧现场碰撞场景：保留邻物及实测障碍；'+
                              ('未观测区域采用用户已清空确认' if clear else '未知区域保留为障碍'))
                with timing.span('observe.planner_build'):self.planner.build()
                if self.self_filter is None:
                    with timing.span('observe.self_filter_construct'):
                        self.self_filter=RobotSelfFilter(self.planner.init['parameters']['robot_description'],self.model)
                # Only an applicable known type bounds a hidden neighbour.
                # Mixed unknown sizes must not inherit an older carton's depth.
                geometry=self.config.get('geometry') or {}
                neighbor_depth=neighbor_depth_bound(geometry)
                with timing.span('observe.build_scene'):
                    self.scene=build_scene(self.observation,self.result,self.boxes,self.config['site'],self.self_filter,
                                           neighbor_depth_m=neighbor_depth)
                if size_policy(geometry)=='UNKNOWN_PER_INSTANCE':
                    mark_incomplete_scene(self.scene,geometry,rejected)
                    for item in rejected:
                        evidence=item.get('evidence') or {};face=evidence.get('observed_face')
                        if not face:continue
                        try:
                            candidates=contact_candidates(dict(faces=[face]),self.planner.source['tool'],
                                self.config['topology'],self.config.get('contact_search'))
                            evidence['contact_candidate_check']=dict(count=len(candidates),
                                anticipated_seal_counts=sorted({len(c['active_ids']) for c in candidates}),
                                all_observed_coverage=all(c['seal_check']['observed_surface_covered'] for c in candidates),
                                scope='OBSERVED_CONTACT_GEOMETRY_ONLY_NOT_A_PLAN')
                        except ValueError as e:evidence['contact_candidate_check']=dict(count=0,reason=str(e))
                if self.stop_event.is_set() or self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
                self.write_record('site-scene.json',self.scene)
            if self.evidence is not None:
                self.evidence.write_npz(self.folder/'segmentation.npz',{f'mask_{i}':m for i,m in enumerate(self.result.masks)},'segmentation.npz')
            else:
                with timing.span('disk.write',file='segmentation.npz'):
                    np.savez_compressed(self.folder/'segmentation.npz',**{f'mask_{i}':m for i,m in enumerate(self.result.masks)})
            self.write_record('geometry.json',dict(boxes=self.boxes,rejected=rejected,detections=self.result.detections),indent=2)
            self.journal(kind='perception',duration_s=time.monotonic()-started,observation_id=self.observation.frame.observation_id,boxes=len(self.boxes),rejected=rejected)
            self.notify('observation',(self.observation,self.result,self.boxes))
            if not self.boxes and any((r.get('evidence') or {}).get('geometry_state')=='OBSERVED_SURFACE_ONLY' for r in rejected):
                self.phase='GEOMETRY_UNRESOLVED'
                self.reason=('各箱尺寸未知：已保留各自实测接触面，历史统一尺寸先验未使用；'
                    '完整箱体及邻箱隐藏体积尚无可靠边界，不能据此验证带载路径。')
                self.journal(kind='geometry_unresolved',reason=self.reason,physical_result='NOT_RUN')
                self.progress(self.reason)
                return
            if rejected and not self.config.get('site'):
                labels=dict(BOX_THICKNESS_UNOBSERVED_SHOW_SIDE='已有可见面，后方厚度未测出，请露出侧面',
                    BOX_ENVELOPE_CLIPPED_BY_IMAGE='箱体被画面边缘截断',
                    BOX_FAR_BOUNDARY_OCCLUDED_SHOW_SIDE_EDGE='侧面边界被前方物体遮挡',
                    VISIBLE_FACE_INSUFFICIENT_BOUNDARY_COVERAGE='可见面边界不完整',
                    MASK_MIXED_NEIGHBOR_OR_NONCUBOID='掩码可能混入邻箱或不是完整箱体')
                reasons=[]
                for x in rejected:
                    dimensions=(np.asarray(x['evidence']['measured_face_size_m'])*1000).round(1).tolist() if x.get('evidence') else None
                    reasons.append(f"编号 {x['detection_index']}：{labels.get(x['reason'],x['reason'])}"+(f'；已测面 {dimensions} mm' if dimensions else ''))
                raise ValueError('不限制固定箱型；本次完整碰撞包络尚未确定：\n'+'\n'.join(reasons))
            if not self.boxes:
                def explain(r):
                    e=r.get('evidence') or {}
                    if r['reason'].startswith('DIMENSION_PRIOR_CONFLICT') and e.get('observed_patch_m'):
                        seen=(np.asarray(e['observed_patch_m'])*1000).round(1).tolist();prior=(np.asarray(e['prior_m'])*1000).round(1).tolist()
                        return (f"{r['detection_index']}: {r['reason']}（可见面实测 {seen} mm 大于箱型先验 {prior} mm 的任一面；"
                                '请在“现场配置”中把 geometry.prior_size_m 改为实际纸箱尺寸并重启）')
                    return f"{r['detection_index']}: {r['reason']}"
                raise ValueError('NO_REAL_BOX_WITH_TRUSTWORTHY_GEOMETRY: '+'; '.join(map(explain,rejected)))
            self.phase='TARGET_READY';self.reason=''
            timing.add('observe.entry_to_target_ready',entered,timing.now()-entered)
        except Exception as e:self.reject(e);raise
        finally:
            self.busy=False
            if self.folder:timing.write_later(self.folder/'timing-observe.json','observe',dict(task_id=self.task_id,final_phase=self.phase),since=entered)
    def prepare_plan(self,selected=None):
        self._enter();entered=timing.now()
        try:
            if self.phase!='TARGET_READY' or self.observation is None:
                raise ValueError('请先完成有效观测；'+(self.reason or '点击准备观测并提供足够密封的实测可见面'))
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.phase='PLANNING';start=time.monotonic()
            # Optional unified planning deadline: one absolute monotonic time
            # from this entry; absent keeps every request and timeout unchanged.
            budget=self.config.get('planning_deadline_s')
            deadline=None if budget is None else entered+float(budget)
            prepared_cycle={}
            journal=self.journal;task_id=self.task_id
            observation=self.observation;generation=observation.generation
            def finish(plan,guard):
                guard('CYCLE_READY')
                def pending_record(**event):
                    if event.get('state')=='PLAN_READY':event=dict(event,state='PREPARED_PENDING_COMMIT')
                    journal(**event)
                cycle=Cycle(self.service,self.vacuum,self.config['vacuum'],pending_record,self.check_contact,self.check_release,physical_io=True)
                with timing.span('prepare_plan.cycle_ready'):cycle.ready(task_id,plan)
                guard('TASK_EVIDENCE')
                for writer in (getattr(self,'evidence',None),journal):
                    if writer is not None:
                        remaining=120 if deadline is None else max(0.,deadline-time.monotonic())
                        writer.flush(timeout=remaining,cancelled=self.stop_event.is_set)
                guard('READY_TO_PUBLISH')
                if generation!=self.backend.observation_generation:raise ValueError('PLAN_INVALIDATED')
                cycle.record=journal;prepared_cycle['cycle']=cycle
            with timing.span('prepare_plan.plan_cycle'):
                self.plan=self.planner.plan_cycle(observation,self.boxes,self.folder,self.progress,selected,self.stop_event.is_set,
                                                  scene=self.scene,finalize=finish,**({} if deadline is None else dict(deadline=deadline)))
            if self.stop_event.is_set() or generation!=self.backend.observation_generation:raise ValueError('PLAN_INVALIDATED')
            if deadline is not None and time.monotonic()>=deadline:
                from .planner import PlanningBudgetExhausted
                raise PlanningBudgetExhausted('PLAN_READY_PUBLICATION')
            self.cycle=prepared_cycle['cycle']
            self.phase='PLAN_READY'
            timing.add('prepare_plan.entry_to_plan_ready',entered,timing.now()-entered)
            self.journal(kind='planning',duration_s=time.monotonic()-start,full_path='PASS')
            self.notify('plan',self.plan)
        except Exception as e:self.reject(e,background=True);raise
        finally:
            self.busy=False
            timing.add('prepare_plan.entry_to_return',entered,timing.now()-entered)
            outcome='PLANNING_BUDGET_EXHAUSTED' if 'PLANNING_BUDGET_EXHAUSTED' in self.reason else ('CANCELLED' if self.stop_event.is_set() else self.phase)
            if self.folder:
                self.planning_timing_write=timing.write_later(self.folder/'timing-prepare_plan.json','prepare_plan',
                    dict(task_id=self.task_id,final_phase=self.phase,outcome=outcome),since=entered,
                    dispatch=getattr(self.journal,'defer',None))
    def execute(self):
        self._enter();self.execute_entered=timing.now();timing.add('execute.entry',self.execute_entered,0.)
        try:
            lock=execution_lock(getattr(self,'config',None) or {}) or (getattr(self,'plan',None) or {}).get('execution_lock')
            if lock:
                if self.journal:self.journal(kind='execution_refused',reason='EXECUTION_LOCKED_BY_USER',execution_lock=lock)
                raise ValueError('EXECUTION_LOCKED_BY_USER: 执行已锁定（'+str(lock.get('basis'))+'）；未发送任何运动指令')
            if not self.cycle or self.cycle.state!='PLAN_READY':raise ValueError('PLAN_NOT_READY')
            # No cycle starts without its task record or with a revoked observation.
            if self.journal is None or self.journal.error or not self.journal.thread.is_alive():
                raise ValueError('TASK_RECORD_UNAVAILABLE_NO_MOTION:'+str(getattr(self.journal,'error','')))
            # async_evidence: every observation and plan record must be on disk first.
            flush_task_records(self)
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            recheck=pre_execute_visual_recheck(self.config)
            recheck_timing=timing.span('execute.pre_execute_recheck',enabled=recheck['enabled']);recheck_timing.__enter__()
            if recheck['enabled']:
                self.progress('停稳重新取景：核对目标、场景、起点及版本')
                current=freeze(self.backend,self.service,self.model,self.calibration,
                               cancelled=self.stop_event.is_set,progress=self.progress)
                # Keep the new frame even when the comparison rejects it.
                self.save_observation(current,'pre_execute_observation')
                if recheck['scope']=='TARGET_ONLY':
                    same_session_start_and_camera(self.observation,current)
                    check=dict(status='PASS',scope='TARGET_ONLY',basis=recheck['basis'],whole_scene_comparison='SKIPPED_BY_CONFIGURATION')
                else:check=unchanged(self.observation,current)
                # Target-local check prevents a small target movement being diluted
                # by unchanged background pixels in the whole-view scene test.
                mask=self.result.masks[self.plan['target']['detection_index']]
                target_unchanged(self.observation,current,mask)
                self.journal(kind='pre_execute_recheck',evidence=check)
            else:
                # User instruction: no visual re-capture. The operator is
                # responsible for an unchanged target and scene since planning.
                self.progress('执行前画面复核已按用户要求关闭；核对会话、版本及实时起点关节后启动')
                self.journal(kind='pre_execute_recheck',evidence=recheck)
            recheck_timing.__exit__(None,None,None)
            with timing.span('execute.binding'):self.planner.binding()
            hashed=getattr(self.planner,'binding_evidence',None)
            if hashed is not None:self.journal(kind='binding_hash',evidence={k:v for k,v in hashed.items() if k!='files'},
                                               files=[(r['file'],r['status']) for r in hashed['files']])
            if self.stop_event.is_set():raise ValueError('CANCELLED')
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            flush_task_records(self)
            # Disk barriers may block across a device/session transition. Order
            # the final check and startup against all backend generation writes.
            with self.backend.observation_lock:
                if self.stop_event.is_set():raise ValueError('CANCELLED')
                if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
                with self.planner.authority._lock:
                    for segment in self.plan['segments']:
                        self.planner.authority.verify(segment['package'],self.plan['binding'])
                # submit still claims its permit atomically; a revocation after
                # verification is refused there. Never hold its lock across IO.
                if self.stop_event.is_set():raise ValueError('CANCELLED')
                with timing.span('execute.cycle_start'):self.cycle.start()
                self.phase=self.cycle.state
        except Exception as e:
            self.planner.authority.invalidate()
            if self.cycle and self.cycle.active:self.cycle.stop(str(e))
            else:self.reject(e)
            raise
        finally:self.busy=False
    def check_contact(self,q,plan):
        check=candidate_contact_check(self.model.tcp(q),plan['candidate'])
        self.journal(kind='actual_contact_fk',q_actual=q.tolist(),seal=check)
        if not check['valid']:raise ValueError('ACTUAL_CONTACT_POSE_NOT_SEALED')
    def check_release(self,q,plan):
        b=plan['binding']['physical_cycle'];result=self.planner.release_check(q,np.asarray(b['attachment']),plan['target'],b.get('release_world',[]))
        self.journal(kind='actual_return_release_pose',q_actual=q.tolist(),check=result)
    def reject(self,error,background=False):
        self.reason=str(error);self.phase='FAILED';self.planner.authority.invalidate()
        evidence=getattr(error,'evidence',None)
        if self.journal:self.journal(kind='rejected',reason=self.reason,evidence=evidence,physical_result='NOT_RUN')
        if self.folder:
            path=self.folder/'result.json'
            record=dict(task_id=self.task_id,state='REJECTED',reason=self.reason,evidence=evidence,physical_result='NOT_RUN')
            def write():
                try:path.write_text(json.dumps(record,indent=2))
                except Exception as failure:self.rejection_record_error=str(failure)
            if background:
                dispatch=getattr(self.journal,'defer',None)
                if dispatch is not None:
                    self.rejection_record_thread=timing.dispatch_write(write,dispatch)
                    if self.rejection_record_thread.error is not None:self.rejection_record_error=str(self.rejection_record_thread.error)
                else:
                    self.rejection_record_thread=threading.Thread(target=write,daemon=True,name='planning-rejection-record')
                    self.rejection_record_thread.start()
            else:path.write_text(json.dumps(record,indent=2))
        self.notify('error',self.reason)
    def stop(self):
        self.stop_event.set()
        if self.cycle:self.cycle.stop_requested.set()
        self.service.request_stop();self.planner.authority.invalidate();self.vacuum.inhibit()
        # A Cycle tick may currently hold its lock across an IPC read. Signal
        # immediately; the independent supervisor performs the locked transition.
        # Tk must not wait for either this lock or inference completion.
    def _monitor(self):
        # This thread is the cycle's only ticker: it must survive any error.
        # A failure stops an active cycle (never releasing a load) and the next
        # iteration keeps ticking so the stop can still be confirmed or latched.
        saved=[None];last_error=None
        while not self.closing.wait(.02):
            try:self._monitor_once(saved);last_error=None
            except Exception as e:
                message=f'SUPERVISOR_ERROR:{type(e).__name__}:{e}'
                try:
                    c=self.cycle
                    if c and c.state not in TERMINAL:c.stop(message)
                except Exception:pass
                if message!=last_error:
                    last_error=message
                    try:self.notify('error','任务监视线程异常（已停止当前循环并继续监视）：'+message)
                    except Exception:pass
    def _monitor_once(self,saved):
        c=self.cycle
        if c and self.stop_event.is_set() and c.state not in TERMINAL|{'STOPPING'}:
            c.stop(cancelled=True)
        if c and c.active and not self.busy:
            try:
                # Any session transition, camera included, stops the cycle
                # (user confirmed 2026-11-13 that a camera fault aborts loaded
                # motion too). The stop never releases a held load.
                if self.observation.generation!=self.backend.observation_generation:
                    c.stop('CONNECTION_OR_CONFIG_CHANGED')
                if self.journal.error or not self.journal.thread.is_alive():c.stop('LOG_WRITE_FAILED:'+str(self.journal.error))
                for writer in (getattr(self,'evidence',None),getattr(self.planner,'evidence',None)):
                    if writer is not None and not writer.alive:c.stop('LOG_WRITE_FAILED:'+str(writer.error))
            except Exception as e:c.stop(f'SUPERVISOR_ERROR:{type(e).__name__}:{e}')
            c.tick();self.phase=c.state;self.reason=c.reason
        self._save_result(c,saved)
    def _save_result(self,c,saved):
        """Write each terminal cycle state once; saved[0] holds the last written key."""
        if c and c.state in TERMINAL and saved[0]!=(self.task_id,c.state):
            saved[0]=(self.task_id,c.state);status=c.status()
            try:
                (self.folder/'result.json').write_text(json.dumps(status,indent=2))
                self.notify('result',status)
            except Exception as e:c.stop('RESULT_WRITE_FAILED:'+str(e))
            # Measurement only: this cycle's spans, written after it is terminal.
            timing.write_later(self.folder/'timing-execute.json','execute',dict(task_id=self.task_id,final_state=c.state),
                               since=getattr(self,'execute_entered',None))
    def close(self):
        self.stop()
        if self.cycle and (self.cycle.active or self.cycle.state=='STOP_UNCONFIRMED'):raise ValueError('等待实测停止确认，连接暂时保留')
        if self.vacuum.holding:raise ValueError('吸具可能仍持箱：保持真空，请现场处置，窗口暂时保留')
        self.closing.set();self.thread.join(.3)
        if self.journal:self.journal.close()
        if self.evidence is not None:self.evidence.close();self.evidence=None
        self.planner.close();self.service.close()
