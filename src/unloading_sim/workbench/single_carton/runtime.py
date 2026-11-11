"""Task-bound observation, native planning, pre-execution recheck and records."""
from dataclasses import asdict
from pathlib import Path
from queue import SimpleQueue
import json
import threading
import time
import uuid
import numpy as np
from .config import issues
from .cycle import Cycle,TERMINAL
from .geometry import seal_check
from .perception import HuggingFaceGroundingSAM,freeze,reconstruct,unchanged


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
        if self.journal:self.journal.close()
        self.task_id=uuid.uuid4().hex;self.folder=Path(self.config['record_root'])/self.task_id;self.folder.mkdir(parents=True)
        self.journal=Journal(self.folder,self.task_id)
        (self.folder/'result.json').write_text(json.dumps(dict(task_id=self.task_id,physical_result='NOT_RUN')))
        try:
            missing=issues(self.config)
            if missing:raise ValueError('缺少现场参数：\n'+'\n'.join(missing))
            self.phase='OBSERVING';started=time.monotonic();self.progress('等待机械臂停稳，锁定 q_start 与同帧 RGB-D')
            self.observation=freeze(self.backend,self.service,self.model,self.calibration,cancelled=self.stop_event.is_set)
            self.save_observation(self.observation,'observation')
            self.progress('加载本地 GroundingDINO / SAM 并执行真实推理')
            if self.detector is None:self.detector=HuggingFaceGroundingSAM(**self.config['detector'])
            self.result=self.detector.detect(self.observation.frame)
            if self.stop_event.is_set() or self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.boxes,rejected=reconstruct(self.observation,self.result)
            np.savez_compressed(self.folder/'segmentation.npz',**{f'mask_{i}':m for i,m in enumerate(self.result.masks)})
            (self.folder/'geometry.json').write_text(json.dumps(dict(boxes=self.boxes,rejected=rejected,detections=self.result.detections),indent=2))
            self.journal(kind='perception',duration_s=time.monotonic()-started,observation_id=self.observation.frame.observation_id,boxes=len(self.boxes),rejected=rejected)
            self.notify('observation',(self.observation,self.result,self.boxes))
            if rejected:raise ValueError('UNRESOLVED_DETECTED_NEIGHBOR_GEOMETRY:'+json.dumps(rejected,ensure_ascii=False))
            if not self.boxes:raise ValueError('NO_REAL_BOX_WITH_TRUSTWORTHY_GEOMETRY')
            self.phase='TARGET_READY';self.reason=''
        except Exception as e:self.reject(e);raise
        finally:self.busy=False
    def prepare_plan(self,selected=None):
        self._enter()
        try:
            if self.phase!='TARGET_READY' or self.observation is None:raise ValueError('PREPARE_FROZEN_OBSERVATION_FIRST')
            if self.observation.generation!=self.backend.observation_generation:raise ValueError('OBSERVATION_INVALIDATED')
            self.phase='PLANNING';start=time.monotonic()
            self.plan=self.planner.plan_cycle(self.observation,self.boxes,self.folder,self.progress,selected,self.stop_event.is_set)
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
            self.progress('停稳重新取景：核对目标、场景、起点及版本')
            current=freeze(self.backend,self.service,self.model,self.calibration,cancelled=self.stop_event.is_set)
            check=unchanged(self.observation,current)
            # Target-local check prevents a small target movement being diluted
            # by unchanged background pixels in the whole-view scene test.
            mask=self.result.masks[self.plan['target']['detection_index']]
            a=self.observation.frame;b=current.frame;valid=mask&(a.depth>0)&(b.depth>0)
            changed=abs(a.depth.astype(float)-b.depth)*a.depth_scale_m>.004
            if valid.sum()<120 or changed[valid].mean()>.05:raise ValueError('LOCKED_TARGET_CHANGED_REPLAN')
            self.save_observation(current,'pre_execute_observation');self.journal(kind='pre_execute_recheck',evidence=check)
            self.planner.binding()
            if self.stop_event.is_set():raise ValueError('CANCELLED')
            self.cycle.start();self.phase=self.cycle.state
        except Exception as e:
            self.planner.authority.invalidate()
            if self.cycle and self.cycle.active:self.cycle.stop(str(e))
            else:self.reject(e)
            raise
        finally:self.busy=False
    def check_contact(self,q,plan):
        c=plan['candidate'];check=seal_check(self.model.tcp(q),c['face'],c['rings_tcp'],c['suction_normal_tcp'])
        self.journal(kind='actual_contact_fk',q_actual=q.tolist(),seal=check)
        if not check['valid']:raise ValueError('ACTUAL_CONTACT_POSE_NOT_SEALED')
    def check_release(self,q,plan):
        b=plan['binding']['physical_cycle'];result=self.planner.release_check(q,np.asarray(b['attachment']),plan['target'],[])
        self.journal(kind='actual_return_release_pose',q_actual=q.tolist(),check=result)
    def reject(self,error):
        self.reason=str(error);self.phase='FAILED';self.planner.authority.invalidate()
        if self.journal:self.journal(kind='rejected',reason=self.reason,physical_result='NOT_RUN')
        if self.folder:(self.folder/'result.json').write_text(json.dumps(dict(task_id=self.task_id,state='REJECTED',reason=self.reason,physical_result='NOT_RUN'),indent=2))
        self.notify('error',self.reason)
    def stop(self):
        self.stop_event.set();self.service.request_stop();self.planner.authority.invalidate();self.vacuum.inhibit()
        if self.cycle:self.cycle.stop(cancelled=True)
    def _monitor(self):
        saved=None
        while not self.closing.wait(.02):
            c=self.cycle
            if c and c.active and not self.busy:
                if self.observation.generation!=self.backend.observation_generation:c.stop('CONNECTION_OR_CONFIG_CHANGED')
                if self.journal.error:c.stop('LOG_WRITE_FAILED:'+self.journal.error)
                c.tick();self.phase=c.state;self.reason=c.reason
            if c and c.state in TERMINAL and saved!=(self.task_id,c.state):
                saved=(self.task_id,c.state);status=c.status()
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
