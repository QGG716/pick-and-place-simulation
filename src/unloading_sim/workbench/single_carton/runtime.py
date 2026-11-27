"""Task-bound observation, native planning, pre-execution recheck and records."""
from dataclasses import asdict
from pathlib import Path
from queue import SimpleQueue,Empty
from types import SimpleNamespace
import json
import threading
import time
import uuid
import numpy as np
from .config import issues,pre_execute_visual_recheck,execution_lock,start_joint_check,plan_validity,observation_mode,scene_build
from .cycle import Cycle,TERMINAL
from .geometry import candidate_contact_check,contact_candidates
from .size_policy import neighbor_depth_bound,size_policy,mark_incomplete_scene,NO_PRIOR_POLICIES
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
        self.rejected=[]  # unresolved detections of the current observation (continuous job target order)
        self.scene=None;self.self_filter=None
        self.task_id=None;self.folder=None;self.journal=None;self.phase='IDLE';self.reason='';self.reset=None
        self.stop_event=threading.Event();self.closing=threading.Event();self.busy=False;self.lock=threading.Lock()
        self.evidence=None  # observation evidence writer (async_evidence only)
        self.thread=threading.Thread(target=self._monitor,daemon=True,name='single-carton-supervisor');self.thread.start()
        # resident_workers: start and initialise the native workers in the
        # background as soon as the entry starts (no motion or valve command).
        if (config or {}).get('resident_workers') is True and hasattr(planner,'prewarm'):planner.prewarm()
        # observation_mode / scene_build (user decision 2026-11-25; defaults unchanged).
        self.standing=None;self.standing_hold=None;self.standing_observer=None;self._standing_issues=None;self.standing_count=0
        self.prewarm_lock=threading.Lock();self.precompute_lock=threading.Lock();self.scene_pool=None;self.prewarm_error=None
        mode=observation_mode(config or {})
        if mode['mode']=='CONTINUOUS_STANDING':
            from .standing import StandingObserver
            self.standing_observer=StandingObserver(self,mode)
        if mode['mode']=='CONTINUOUS_STANDING' or scene_build(config or {})=='SPLIT_PARALLEL':self._observation_prewarm()
        # Site STEP static layer: built once at startup, reused by every task.
        self.layout_prebuild=None
        if (config or {}).get('site_layout') is not None:self._layout_prebuild()
    def _layout_prebuild(self):
        def run():
            from .site_layout import prebuild
            started=time.monotonic()
            try:self.layout_prebuild=dict(status='READY',**prebuild(self.config,Path(__file__).resolve().parents[4]))
            except Exception as e:self.layout_prebuild=dict(status='FAILED',error=f'{type(e).__name__}: {e}')
            self.layout_prebuild['elapsed_s']=time.monotonic()-started
        threading.Thread(target=run,daemon=True,name='site-layout-prebuild').start()
    @property
    def active(self):return self.busy or (self.cycle is not None and self.cycle.active)
    # -- standing observation (observation_mode=CONTINUOUS_STANDING) ------------
    def vision(self):return getattr(self.backend,'vision',None)
    def start_standing(self):
        """Start the standing observation thread (after the continuous vision has started)."""
        if self.standing_observer is not None:self.standing_observer.start()
    def standing_status(self):
        return None if self.standing_observer is None else self.standing_observer.status
    def standing_config_issues(self):
        # The configuration is fixed until restart (a saved change requires one).
        if self._standing_issues is None:self._standing_issues=list(issues(self.config))
        return self._standing_issues
    def standing_pause_reason(self,force=False):
        """Why no standing observation may be taken or installed now (None: allowed)."""
        if self.standing_hold:return self.standing_hold
        if self.closing.is_set():return '正在关闭'
        if self.busy:return '任务操作进行中'
        if self.vacuum.holding:return '吸具可能持箱'
        if self.reset is not None:return '复位计划待确认'
        c=self.cycle
        if c is not None:
            if c.active:return '循环运行中'
            if c.state=='STOP_UNCONFIRMED':return '停止未确认'
            # A stopped cycle keeps its q_start for the operator reset until the
            # operator asks for a new observation (which gives that reset up).
            if c.state in ('FAILED','CANCELLED') and not force:return '循环已停止，可复位回本次起点；点“立即重新观测”则放弃本次复位'
            if c.state=='PLAN_READY':
                try:remaining=self.plan_remaining_s()
                except ValueError:remaining=0.
                if remaining>0:return '计划就绪，等待执行'
        if self.phase=='RESETTING':return '复位进行中'
        return None
    def install_standing(self,standing,forced=False):
        """Atomically make ``standing`` the task observation, unless anything forbids it now."""
        with self.lock:
            if self.standing_pause_reason(force=forced) is not None:return False
            if standing.observation.generation!=self.backend.observation_generation:return False
            old=self.journal
            self.planner.authority.invalidate();self.plan=None;self.cycle=None;self.result=None
            self.journal=None;self.folder=None;self.task_id=None
            self.observation=standing.observation;self.result=standing.result;self.boxes=standing.boxes;self.rejected=list(standing.rejected)
            self.scene=standing.scene;self.phase=standing.phase;self.reason=standing.reason
            standing.installed_monotonic_s=time.monotonic()
            self.standing_count+=1;standing.standing_id=self.standing_count
            self.standing=standing
        if old is not None:old.close()
        self.notify('capture',standing.observation)
        self.notify('observation',(standing.observation,standing.result,standing.boxes))
        return True
    def drop_standing(self,reason):
        with self.lock:
            if self.busy or self.standing is None or self.observation is not self.standing.observation:return
            self.standing=None;self.observation=None;self.result=None;self.boxes=[];self.scene=None
            self.phase='IDLE';self.reason=reason
    def request_standing_refresh(self):
        """Operator: take a new standing observation now (never while a load may be held or a stop is unconfirmed)."""
        if self.standing_observer is None:raise ValueError('STANDING_OBSERVATION_NOT_CONFIGURED')
        if self.active or self.vacuum.holding or (self.cycle and self.cycle.state=='STOP_UNCONFIRMED'):
            raise ValueError('TASK_ACTIVE_OR_HELD_LOAD')
        with self.lock:
            self.standing=None;self.observation=None;self.boxes=[];self.scene=None;self.phase='OBSERVING'
            self.reason='已请求立即重新观测；等待停稳取景'
        self.standing_observer.request_refresh()
        return dict(state='STANDING_REFRESH_REQUESTED')
    def _bind_standing(self):
        """Planning entry: bind the installed standing observation to a new task record."""
        s=self.standing;settings=observation_mode(self.config)
        try:
            if s is None or self.observation is not s.observation:raise ValueError('STANDING_OBSERVATION_NOT_READY: 常驻观测尚未就绪')
            if s.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            from .perception import validate_robot
            from .standing import joint_offset_deg
            q=validate_robot(self.service.snapshot(),time.monotonic());offset=joint_offset_deg(q,s.observation.q_start)
            if offset>settings['joint_drift_deg']:raise ValueError(f'STANDING_OBSERVATION_ROBOT_MOVED: 机械臂已离开常驻观测起点 {offset:.4f}°')
            age=s.receipt_age_s()
            if not np.isfinite(age) or age<0 or age>=plan_validity(self.config)['seconds']:raise ValueError('STANDING_OBSERVATION_EXPIRED')
        except Exception:
            self.standing=None;raise
        self._new_task_records()
        self.save_observation(s.observation,'observation')
        s.records.write(self.write_record,self.write_npz)
        for path in s.records.tracked:self.planner.track(path)
        self.journal(kind='standing_observation',mode='CONTINUOUS_STANDING',standing_id=s.standing_id,
            observation_id=s.observation.frame.observation_id,receipt_age_at_bind_s=age,joint_offset_at_bind_deg=offset,
            build_s=s.build_s,refresh_s=settings['refresh_s'],basis=settings['basis'],forced=s.extra.get('forced',False),
            gate='perception.FreezeGate (same as the operator observe button)',physical_result='NOT_RUN')
        self.journal(kind='perception',duration_s=s.build_s,observation_id=s.observation.frame.observation_id,boxes=len(s.boxes),rejected=s.rejected)
        timing.write_later(self.folder/'timing-observe.json','observe',dict(task_id=self.task_id,final_phase=s.phase,
            observation_mode='CONTINUOUS_STANDING'),since=s.started_timing_s)
        self.standing=None
        return s
    def write_standing_diagnostic(self,standing,reason,error=None):
        """A standing build that cannot be planned keeps its inputs in its own record."""
        task_id=uuid.uuid4().hex;folder=Path(self.config['record_root'])/task_id;folder.mkdir(parents=True)
        def write_json(name,value,**options):timing.write_json(folder/name,value,name,**options)
        def write_npz(name,arrays):np.savez_compressed(folder/name,**arrays)
        o=standing.observation
        write_json('observation.json',o.metadata(),indent=2)
        write_npz('observation.npz',dict(rgb=o.frame.rgb,depth=o.frame.depth))
        standing.records.write(write_json,write_npz)
        write_json('result.json',dict(task_id=task_id,state='STANDING_OBSERVATION_NOT_PLANNABLE',phase=standing.phase,
            reason=reason,evidence=getattr(error,'evidence',None),observation_mode='CONTINUOUS_STANDING',
            physical_result='NOT_RUN'),indent=2)
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
    def _new_task_records(self):
        """A new task id, folder, journal and (async_evidence) writer; closes the previous ones."""
        if self.journal:self.journal.close()
        self.journal=None;self.folder=None
        self.task_id=uuid.uuid4().hex;folder=Path(self.config['record_root'])/self.task_id;folder.mkdir(parents=True)
        self.folder=folder;self.journal=Journal(folder,self.task_id)
        if self.evidence is not None:self.evidence.close();self.evidence=None
        if self.config.get('async_evidence') is True:
            from .evidence import EvidenceWriter
            self.evidence=EvidenceWriter('observation-evidence')
        timing.write_json(folder/'result.json',dict(task_id=self.task_id,physical_result='NOT_RUN'),'result.json')
    def write_npz(self,name,arrays):
        if self.evidence is not None:
            self.evidence.write_npz(self.folder/name,arrays,name)
        else:
            with timing.span('disk.write',file=name):
                np.savez_compressed(self.folder/name,**arrays)
    def _observation_prewarm(self):
        """Background, no device access: planner build, self filter and (SPLIT_PARALLEL) the scene child."""
        def run():
            try:
                with self.prewarm_lock:
                    self.planner.build()
                    if self.self_filter is None and self.config.get('site'):
                        from .self_filter import RobotSelfFilter
                        self.self_filter=RobotSelfFilter(self.planner.init['parameters']['robot_description'],self.model)
                    self._scene_precompute()
            except Exception as e:
                self.prewarm_error=f'{type(e).__name__}: {e}'
        threading.Thread(target=run,daemon=True,name='observation-prewarm').start()
    def _scene_precompute(self):
        """The resident SPLIT_PARALLEL child (created once), or None for SEQUENTIAL / no site."""
        if scene_build(self.config)!='SPLIT_PARALLEL' or not self.config.get('site'):return None
        with self.precompute_lock:
            if self.scene_pool is None:
                self.planner.build()
                from .scene_precompute import ScenePrecompute
                self.scene_pool=ScenePrecompute(self.planner.init['parameters']['robot_description'],self.model)
            return self.scene_pool
    def submit_scene_precompute(self,observation):
        pool=self._scene_precompute()
        if pool is None:return None
        return pool.submit(observation.frame,observation.world_camera,observation.q_capture,self.config['site'])
    def perceive(self,observation,result,target,record,record_npz,track,cancelled,progress,precomputed=None):
        """Box recovery and the same-frame site scene (identical in both observation modes).

        ``target`` receives .boxes and .scene as soon as each exists; records go
        through ``record``/``record_npz``; planner file tracking through ``track``.
        """
        if cancelled() or observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
        progress('本次同帧检测 / 分割已完成；恢复箱体与实测接触面')
        with timing.span('observe.reconstruct'):
            target.boxes,rejected=reconstruct(observation,result,self.config.get('geometry'),
                (self.config.get('site') or {}).get('workspace_bounds_m'))
        # Persist the verified same-frame inference before scene/model work.
        # A later scene failure must not lose the original masks or require
        # another inference just to diagnose the failed frozen observation.
        record('detections.json',dict(observation_id=observation.frame.observation_id,
            detections=result.detections,provenance=getattr(result,'provenance','EXISTING_DETECTOR_OUTPUT')))
        record_npz('segmentation.npz',{f'mask_{i}':m for i,m in enumerate(result.masks)})
        if self.config.get('site'):
            from .site_scene import build_scene
            from .self_filter import RobotSelfFilter
            clear=self.config['site']['unknown_policy']=='USER_CONFIRMED_CLEAR'
            progress('构建同帧现场碰撞场景：保留邻物及实测障碍；'+
                     ('未观测区域采用用户已清空确认' if clear else '未知区域保留为障碍'))
            with timing.span('observe.planner_build'):self.planner.build()
            if self.self_filter is None:
                with timing.span('observe.self_filter_construct'):
                    self.self_filter=RobotSelfFilter(self.planner.init['parameters']['robot_description'],self.model)
            # Mixed sizes use the user's site-wide maximum, never an older
            # carton's size prior. Unresolved objects are static shadows.
            geometry=self.config.get('geometry') or {}
            neighbor_depth=neighbor_depth_bound(geometry)
            pre=None
            if precomputed is not None:
                with timing.span('observe.scene_precompute_wait'):
                    try:pre=precomputed.result()
                    except Exception as error:
                        # A broken child is replaced for the next observation; this one fails closed.
                        from concurrent.futures.process import BrokenProcessPool
                        if isinstance(error,BrokenProcessPool):
                            with self.precompute_lock:self.scene_pool=None
                        raise ValueError(f'SCENE_PRECOMPUTE_FAILED:{type(error).__name__}:{error}') from error
            site=self.config['site']
            if self.config.get('site_layout') is not None:
                # Static STEP layer: built once per process (prebuilt at startup), reused here.
                from .site_layout import load_layout,replaced_volumes
                with timing.span('observe.site_layout_cached'):
                    layout=load_layout(self.config,Path(__file__).resolve().parents[4],getattr(observation,'lift_state',None))
                volumes=replaced_volumes(layout,self.config['site_layout'])
                if volumes:site=dict(site,layout_replaced_volumes=volumes)
            with timing.span('observe.build_scene'):
                target.scene=build_scene(observation,result,target.boxes,site,self.self_filter,
                    neighbor_depth_m=neighbor_depth,
                    unresolved_cartons=rejected if size_policy(geometry) in NO_PRIOR_POLICIES else (),geometry=geometry,
                    precomputed=pre)
            from .site_layout import merge_registered_layout
            target.scene=merge_registered_layout(target.scene,observation,result,self.config,
                SimpleNamespace(track=track),self.self_filter,record)
            if size_policy(geometry) in NO_PRIOR_POLICIES:
                mark_incomplete_scene(target.scene,geometry,rejected)
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
            if cancelled() or observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            record('site-scene.json',target.scene)
        record('geometry.json',dict(boxes=target.boxes,rejected=rejected,detections=result.detections),indent=2)
        return rejected
    def classify(self,boxes,rejected,scene):
        """(phase, reason) after perception, or the same refusal as before."""
        if not boxes:
            reason=(f'可规划目标 {len(boxes)} 箱，{len(rejected)} 箱保留可见面及隐藏体积约束；未使用尺寸先验。'
                '当前 RGB-D 尚不能为待抓目标给出完整带载包络，接触候选仍已保存；不要求所有邻箱先测出完整尺寸。\n'+
                '\n'.join(f"编号 {r['detection_index']}：{r['reason']}" for r in rejected))
            return 'GEOMETRY_UNRESOLVED',reason
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
        reason=(f'可规划目标 {len(boxes)} 箱；未恢复邻箱以保守阴影保留，工作范围外检测已单独忽略。' if rejected else '')
        if any(r.get('status')=='APPLIED' for r in (scene or {}).get('confirmed_stack_support',[])):
            reason+=' 已按现场确认的叠放关系约束下箱支撑体，未将其包络尺寸冒充实测箱尺寸。'
        return 'TARGET_READY',reason
    def observe(self):
        if observation_mode(self.config or {})['mode']=='CONTINUOUS_STANDING':
            # The button asks for an immediate new standing observation (same gate and steps).
            return self.request_standing_refresh()
        self._enter();self.stop_event.clear();self.planner.authority.invalidate();self.plan=None;self.cycle=None
        self.observation=None;self.result=None;self.boxes=[];self.rejected=[];self.scene=None;self.reason=''
        entered=timing.now()
        try:
            self._new_task_records()
            missing=issues(self.config)
            if missing:raise ValueError('缺少现场参数：\n'+'\n'.join(missing))
            self.phase='OBSERVING';started=time.monotonic();self.progress('等待机械臂停稳，锁定 q_start 与同帧 RGB-D')
            with timing.span('observe.freeze'):
                self.observation=freeze(self.backend,self.service,self.model,self.calibration,
                                        cancelled=self.stop_event.is_set,progress=self.progress)
            self.notify('capture',self.observation)
            # SPLIT_PARALLEL: the mask-independent scene part starts now, beside inference.
            precomputed=self.submit_scene_precompute(self.observation)
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
            rejected=self.perceive(self.observation,self.result,self,self.write_record,self.write_npz,lambda path:self.planner.track(path),
                                   self.stop_event.is_set,self.progress,precomputed)
            self.rejected=list(rejected)
            self.journal(kind='perception',duration_s=time.monotonic()-started,observation_id=self.observation.frame.observation_id,boxes=len(self.boxes),rejected=rejected)
            self.notify('observation',(self.observation,self.result,self.boxes))
            self.phase,self.reason=self.classify(self.boxes,rejected,self.scene)
            if self.phase=='GEOMETRY_UNRESOLVED':
                self.journal(kind='geometry_unresolved',reason=self.reason,physical_result='NOT_RUN')
                self.progress(self.reason)
                return
            timing.add('observe.entry_to_target_ready',entered,timing.now()-entered)
        except Exception as e:self.reject(e);raise
        finally:
            self.busy=False
            if self.folder:timing.write_later(self.folder/'timing-observe.json','observe',dict(task_id=self.task_id,final_phase=self.phase),since=entered)
    def prepare_plan(self,selected=None):
        self._enter();entered=timing.now()
        try:
            if observation_mode(self.config or {})['mode']=='CONTINUOUS_STANDING':
                with timing.span('prepare_plan.bind_standing'):self._bind_standing()
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
                from .cycle import dry_run_policy
                cycle=Cycle(self.service,self.vacuum,self.config['vacuum'],pending_record,self.check_contact,self.check_release,physical_io=True,dry_run=dry_run_policy(self.config))
                with timing.span('prepare_plan.cycle_ready'):cycle.ready(task_id,plan,digest_memo=getattr(self.planner,'digest_memo',lambda:None)())
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
                    same_session_start_and_camera(self.observation,current,max_joint_rad=np.deg2rad(start_joint_check(self.config)['tolerance_deg']))
                    check=dict(status='PASS',scope='TARGET_ONLY',basis=recheck['basis'],whole_scene_comparison='SKIPPED_BY_CONFIGURATION')
                else:check=unchanged(self.observation,current,max_joint_rad=np.deg2rad(start_joint_check(self.config)['tolerance_deg']))
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
                if self.plan_remaining_s()<0:raise ValueError('PLAN_EXPIRED_REPLAN')
                with timing.span('execute.cycle_start'):self.cycle.start()
                self.phase=self.cycle.state
        except Exception as e:
            self.planner.authority.invalidate()
            if self.cycle and self.cycle.active:self.cycle.stop(str(e))
            else:self.reject(e)
            raise
        finally:self.busy=False
    def _reset_record(self,**event):
        try:
            if self.journal is not None and not self.journal.error:self.journal(**event)
        except Exception:pass
    def reset_plan(self):
        """Operator reset after a stop (user decision 2026-11-24): plan only.

        Allowed only after this task's cycle ended FAILED/CANCELLED (stop
        confirmed by feedback), with no load possibly held, the arm enabled,
        alarm-free and stationary. Plans a new audited empty path back to the
        frozen q_start (``SinglePlanner.plan_reset``); nothing moves here.
        """
        self._enter()
        try:
            self.reset=None
            if self.plan is None or self.folder is None:raise ValueError('RESET_REQUIRES_A_PLANNED_TASK')
            if self.cycle is None or self.cycle.state not in {'FAILED','CANCELLED'}:
                raise ValueError('RESET_ONLY_AFTER_A_CONFIRMED_STOP: 当前状态 '+str(self.cycle.state if self.cycle else self.phase))
            if self.vacuum.holding:raise ValueError('吸具可能仍持箱：不允许复位，请现场处置')
            status=self.service.feedback()
            if status.get('state') not in {'READY','SUCCEEDED','CANCELLED','FAILED'}:raise ValueError('ROBOT_NOT_IDLE:'+str(status.get('state')))
            from .perception import validate_robot
            sample=self.service.snapshot();q=validate_robot(sample,time.monotonic())
            if np.max(abs(np.asarray(sample.get('velocity',[1.]*6),float)))>np.deg2rad(.5):raise ValueError('ROBOT_NOT_STATIONARY')
            q_start=np.asarray(self.plan['q_start'],float)
            if np.max(abs(q-q_start))<=np.deg2rad(.01):raise ValueError('已在起点，无需复位')
            self.stop_event.clear()
            self.progress('复位：从实测关节规划回到本任务起点 q_start（完整碰撞审计）')
            self.reset=self.planner.plan_reset(q,self.plan,self.folder,self.progress,self.stop_event.is_set)
            self._reset_record(kind='reset_plan',stages=self.reset['stages'],q_now=self.reset['q_now'],q_start=self.reset['q_start'],
                               duration_s=self.reset['duration_s'],physical_result='NOT_RUN')
            return dict(stages=self.reset['stages'],duration_s=self.reset['duration_s'],
                        max_joint_offset_deg=float(np.rad2deg(np.max(abs(q-q_start)))))
        except Exception as e:
            self._reset_record(kind='reset_rejected',reason=str(e));raise
        finally:self.busy=False
    def move_plan(self):
        """Move to the configured observation start (user decision 2026-11-25):
        plan only. Allowed with no running cycle, no load possibly held, the arm
        enabled, alarm-free and stationary, after this session's latest task
        was observed and planned (its frozen scene is used). The motion runs
        only through reset_execute after the operator confirms."""
        self._enter()
        try:
            self.reset=None
            from .config import observation_start
            goal=observation_start(self.config)
            if goal is None:raise ValueError('配置中没有 observation_start（观测起点关节角）')
            if self.cycle is not None and self.cycle.state not in {'FAILED','CANCELLED','COMPLETE'}:
                raise ValueError('循环未结束，不能移动：当前状态 '+str(self.cycle.state))
            if self.observation is None or self.folder is None or not self.boxes:
                raise ValueError('请先“准备观测”并“规划所选箱”一次（失败也可），再移到观测起点')
            if self.vacuum.holding:raise ValueError('吸具可能仍持箱：不允许移动，请现场处置')
            status=self.service.feedback()
            if status.get('state') not in {'READY','SUCCEEDED','CANCELLED','FAILED'}:raise ValueError('ROBOT_NOT_IDLE:'+str(status.get('state')))
            from .perception import validate_robot
            sample=self.service.snapshot();q=validate_robot(sample,time.monotonic())
            if np.max(abs(np.asarray(sample.get('velocity',[1.]*6),float)))>np.deg2rad(.5):raise ValueError('ROBOT_NOT_STATIONARY')
            target=np.deg2rad(goal['q_deg'])
            if np.max(abs(q-target))<=np.deg2rad(.01):raise ValueError('已在观测起点，无需移动')
            box=next((b for b in self.boxes if b['id']==getattr(self.planner,'task_target_id',None)),self.boxes[0])
            self.stop_event.clear()
            self.progress('移到观测起点：从实测关节规划（完整碰撞审计），确认后才运动')
            self.reset=self.planner.plan_move_to(q,target,box,self.folder,self.progress,self.stop_event.is_set)
            self._reset_record(kind='move_plan',q_now=self.reset['q_now'],goal=self.reset['q_start'],
                               duration_s=self.reset['duration_s'],basis=goal['basis'],physical_result='NOT_RUN')
            return dict(stages=self.reset['stages'],duration_s=self.reset['duration_s'],goal_deg=goal['q_deg'],
                        q_now_deg=np.rad2deg(q).round(2).tolist(),max_joint_offset_deg=float(np.rad2deg(np.max(abs(q-target)))))
        except Exception as e:
            self._reset_record(kind='move_rejected',reason=str(e));raise
        finally:self.busy=False
    def reset_execute(self):
        """Operator-confirmed execution of the reset plan, segment by segment."""
        self._enter()
        try:
            reset,self.reset=getattr(self,'reset',None),None
            if not reset:raise ValueError('RESET_PLAN_REQUIRED')
            if self.vacuum.holding:raise ValueError('吸具可能仍持箱：不允许复位')
            if reset['binding']!=self.planner.binding():raise ValueError('RESET_PLAN_BINDING_CHANGED')
            self.stop_event.clear();self.phase='RESETTING'
            for k,segment in enumerate(reset['segments']):
                if self.stop_event.is_set():raise ValueError('CANCELLED')
                ticket=self.service.submission_ticket()
                task=self.service.submit(segment['package'],cancel_ticket=ticket)
                self._reset_record(kind='reset_motion',segment=k,stage=segment['stage'],motion_task_id=task)
                self.progress(f"复位第 {k+1}/{len(reset['segments'])} 段（{segment['stage']}）执行中")
                while True:
                    time.sleep(.02)
                    status=self.service.feedback()
                    if status.get('task_id')!=task:raise ValueError('ROBOT_TASK_ID_CHANGED')
                    if status.get('state')=='SUCCEEDED' and status.get('arrival_confirmed'):break
                    if status.get('state') in {'FAILED','CANCELLED','STOP_UNCONFIRMED'}:
                        raise ValueError('RESET_STOPPED:'+str(status.get('state'))+':'+str(status.get('reason')))
            from .perception import validate_robot
            q=validate_robot(self.service.snapshot(),time.monotonic())
            error=float(np.rad2deg(np.max(abs(q-np.asarray(reset['q_start'])))))
            self._reset_record(kind='reset_complete',q_actual=q.tolist(),max_error_deg=error,physical_result='NOT_RUN')
            if reset.get('kind')=='MOVE_TO_OBSERVATION_START':
                self.phase='AT_OBSERVATION_START';self.reason=f'已到观测起点（最大关节偏差 {error:.3f}°）；请重新准备观测'
                return dict(state='AT_OBSERVATION_START',max_error_deg=error)
            self.phase='RESET_AT_START';self.reason=f'已复位到起点（最大关节偏差 {error:.3f}°）；请重新准备观测'
            return dict(state='RESET_AT_START',max_error_deg=error)
        except Exception as e:
            self.phase='FAILED';self.reason='复位失败：'+str(e)
            self._reset_record(kind='reset_failed',reason=str(e));raise
        finally:
            self.planner.authority.invalidate();self.busy=False
    def plan_remaining_s(self):
        if self.observation is None:return 0.
        age=time.monotonic()-self.observation.frame.stamp.received_monotonic_ns/1e9
        if not np.isfinite(age) or age<0:raise ValueError('PLAN_EXPOSURE_CLOCK_INVALID')
        return plan_validity(self.config)['seconds']-age
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
        if self.standing_observer is not None:self.standing_observer.close()
        if self.scene_pool is not None:self.scene_pool.close()
        if self.journal:self.journal.close()
        if self.evidence is not None:self.evidence.close();self.evidence=None
        self.planner.close();self.service.close()
