"""Task-bound observation, native planning, pre-execution recheck and records."""
from dataclasses import asdict
from pathlib import Path
from queue import SimpleQueue
import json
import threading
import time
import uuid
import numpy as np
from .config import issues,pre_execute_visual_recheck
from .cycle import Cycle,TERMINAL
from .geometry import candidate_contact_check
from .perception import make_detector,freeze,reconstruct,unchanged,target_unchanged,same_session_start_and_camera


class Journal:
    def __init__(self,folder,task_id):
        self.folder=folder;self.task_id=task_id;self.events=SimpleQueue();self.error=None
        self.thread=threading.Thread(target=self._write,daemon=True,name='single-carton-records');self.thread.start()
    def __call__(self,**event):
        self.events.put(dict(task_id=self.task_id,host_monotonic_s=time.monotonic(),host_utc_ns=time.time_ns(),**event))
    def _write(self):
        try:
            with (self.folder/'events.jsonl').open('a',buffering=1) as f:
                while True:
                    event=self.events.get()
                    if event is None:break
                    f.write(json.dumps(event,allow_nan=False)+'\n')
        except Exception as e:self.error=str(e)
    def close(self):self.events.put(None);self.thread.join(2)


class TaskManager:
    def __init__(self,backend,service,vacuum,planner,model,calibration,config,notify=lambda *a:None):
        self.backend,self.service,self.vacuum,self.planner=backend,service,vacuum,planner
        self.model,self.calibration,self.config,self.notify=model,calibration,config,notify
        self.detector=None;self.cycle=None;self.observation=None;self.plan=None;self.boxes=[];self.result=None
        self.scene=None;self.self_filter=None
        self.task_id=None;self.folder=None;self.journal=None;self.phase='IDLE';self.reason=''
        self.stop_event=threading.Event();self.closing=threading.Event();self.busy=False;self.lock=threading.Lock()
        self.thread=threading.Thread(target=self._monitor,daemon=True,name='single-carton-supervisor');self.thread.start()
    @property
    def active(self):return self.busy or (self.cycle is not None and self.cycle.active)
    def progress(self,text):self.notify('progress',text)
    def _enter(self):
        with self.lock:
            if self.active or self.vacuum.holding or (self.cycle and self.cycle.state=='STOP_UNCONFIRMED'):raise ValueError('TASK_ACTIVE_OR_HELD_LOAD')
            self.busy=True
    def save_observation(self,observation,name):
        (self.folder/(name+'.json')).write_text(json.dumps(observation.metadata(),indent=2))
        np.savez_compressed(self.folder/(name+'.npz'),rgb=observation.frame.rgb,depth=observation.frame.depth)
    def observe(self):
        self._enter();self.stop_event.clear();self.planner.authority.invalidate();self.plan=None;self.cycle=None
        self.observation=None;self.result=None;self.boxes=[];self.scene=None;self.reason=''
        try:
            if self.journal:self.journal.close()
            self.journal=None;self.folder=None
            self.task_id=uuid.uuid4().hex;folder=Path(self.config['record_root'])/self.task_id;folder.mkdir(parents=True)
            self.folder=folder;self.journal=Journal(folder,self.task_id)
            (folder/'result.json').write_text(json.dumps(dict(task_id=self.task_id,physical_result='NOT_RUN')))
            missing=issues(self.config)
            if missing:raise ValueError('缺少现场参数：\n'+'\n'.join(missing))
            self.phase='OBSERVING';started=time.monotonic();self.progress('等待机械臂停稳，锁定 q_start 与同帧 RGB-D')
            self.observation=freeze(self.backend,self.service,self.model,self.calibration,
                                    cancelled=self.stop_event.is_set,progress=self.progress)
            self.notify('capture',self.observation)
            self.save_observation(self.observation,'observation')
            self.progress('q_start 与原始 RGB-D 已锁定；等待本次冻结帧的检测 / 分割')
            vision=getattr(self.backend,'vision',None)
            if vision:
                self.result=vision.infer_frozen(self.observation.frame,self.observation.generation,
                                              self.stop_event.is_set,progress=self.progress)
            else:
                if self.detector is None:self.detector=make_detector(**self.config['detector'])
                self.result=self.detector.detect(self.observation.frame)
            if self.stop_event.is_set() or self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.progress('本次同帧检测 / 分割已完成；恢复箱体与实测接触面')
            self.boxes,rejected=reconstruct(self.observation,self.result,self.config.get('geometry'))
            if self.config.get('site'):
                from .site_scene import build_scene
                from .self_filter import RobotSelfFilter
                clear=self.config['site']['unknown_policy']=='USER_CONFIRMED_CLEAR'
                self.progress('构建同帧现场碰撞场景：保留邻物及实测障碍；'+
                              ('未观测区域采用用户已清空确认' if clear else '未知区域保留为障碍'))
                self.planner.build()
                if self.self_filter is None:
                    self.self_filter=RobotSelfFilter(self.planner.init['parameters']['robot_description'],self.model)
                # Ray depth behind an unreconstructed neighbour's visible surface:
                # the prior's space diagonal plus its tolerance bounds any carton.
                geometry=self.config.get('geometry') or {}
                neighbor_depth=(float(np.linalg.norm(geometry['prior_size_m']))+float(geometry.get('dimension_tolerance_m',0.))
                                if geometry.get('prior_size_m') else None)
                self.scene=build_scene(self.observation,self.result,self.boxes,self.config['site'],self.self_filter,
                                       neighbor_depth_m=neighbor_depth)
                if self.stop_event.is_set() or self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
                (self.folder/'site-scene.json').write_text(json.dumps(self.scene))
            np.savez_compressed(self.folder/'segmentation.npz',**{f'mask_{i}':m for i,m in enumerate(self.result.masks)})
            (self.folder/'geometry.json').write_text(json.dumps(dict(boxes=self.boxes,rejected=rejected,detections=self.result.detections),indent=2))
            self.journal(kind='perception',duration_s=time.monotonic()-started,observation_id=self.observation.frame.observation_id,boxes=len(self.boxes),rejected=rejected)
            self.notify('observation',(self.observation,self.result,self.boxes))
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
        except Exception as e:self.reject(e);raise
        finally:self.busy=False
    def prepare_plan(self,selected=None):
        self._enter()
        try:
            if self.phase!='TARGET_READY' or self.observation is None:
                raise ValueError('请先完成有效观测；'+(self.reason or '点击准备观测并提供足够密封的实测可见面'))
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.phase='PLANNING';start=time.monotonic()
            self.plan=self.planner.plan_cycle(self.observation,self.boxes,self.folder,self.progress,selected,self.stop_event.is_set,scene=self.scene)
            if self.stop_event.is_set() or self.observation.generation!=self.backend.observation_generation:raise ValueError('PLAN_INVALIDATED')
            self.cycle=Cycle(self.service,self.vacuum,self.config['vacuum'],self.journal,self.check_contact,self.check_release,physical_io=True)
            self.cycle.ready(self.task_id,self.plan);self.phase='PLAN_READY'
            self.journal(kind='planning',duration_s=time.monotonic()-start,full_path='PASS')
            self.notify('plan',self.plan)
        except Exception as e:self.reject(e);raise
        finally:self.busy=False
    def execute(self):
        self._enter()
        try:
            if not self.cycle or self.cycle.state!='PLAN_READY':raise ValueError('PLAN_NOT_READY')
            # No cycle starts without its task record or with a revoked observation.
            if self.journal is None or self.journal.error or not self.journal.thread.is_alive():
                raise ValueError('TASK_RECORD_UNAVAILABLE_NO_MOTION:'+str(getattr(self.journal,'error','')))
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            recheck=pre_execute_visual_recheck(self.config)
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
            self.planner.binding()
            if self.stop_event.is_set():raise ValueError('CANCELLED')
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.cycle.start();self.phase=self.cycle.state
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
    def reject(self,error):
        self.reason=str(error);self.phase='FAILED';self.planner.authority.invalidate()
        evidence=getattr(error,'evidence',None)
        if self.journal:self.journal(kind='rejected',reason=self.reason,evidence=evidence,physical_result='NOT_RUN')
        if self.folder:(self.folder/'result.json').write_text(json.dumps(dict(task_id=self.task_id,state='REJECTED',reason=self.reason,evidence=evidence,physical_result='NOT_RUN'),indent=2))
        self.notify('error',self.reason)
    def stop(self):
        self.stop_event.set();self.service.request_stop();self.planner.authority.invalidate();self.vacuum.inhibit()
        # A Cycle tick may currently hold its lock across an IPC read. Signal
        # immediately; the independent supervisor performs the locked transition.
        # Tk must not wait for either this lock or inference completion.
        if self.cycle:self.cycle.stop_requested.set()
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
                if self.journal.error:c.stop('LOG_WRITE_FAILED:'+self.journal.error)
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
    def close(self):
        self.stop()
        if self.cycle and (self.cycle.active or self.cycle.state=='STOP_UNCONFIRMED'):raise ValueError('等待实测停止确认，连接暂时保留')
        if self.vacuum.holding:raise ValueError('吸具可能仍持箱：保持真空，请现场处置，窗口暂时保留')
        self.closing.set();self.thread.join(.3)
        if self.journal:self.journal.close()
        self.planner.close();self.service.close()
