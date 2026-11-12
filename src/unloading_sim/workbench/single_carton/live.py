"""Bounded real vision and independent device supervision. No motion/valve calls."""
from collections import deque
from dataclasses import dataclass
import threading
import time
import cv2
import numpy as np
from ..controlled.ui import ControlledBackend
from ..devices import StreamProfile
from .perception import HuggingFaceGroundingSAM


def connection_error(error):
    message=str(error)
    explanations={
        'ROBOT_NETWORK_':'已保存的机器人网络配置未就绪，请检查网络名称、网卡和系统网络权限',
        'ROBOT_ALREADY_OWNED':'机械臂已被其他工作台占用，请先按原会话流程结束操作',
        'SERIAL_IN_USE':'串口已被其他程序占用，请先结束原会话',
        'PermissionError':'当前账户没有设备访问权限，请检查串口权限',
        'CONNECTION_FAILED':'无法建立控制器通信，请核对已保存地址、端口和网络',
        'NO_LIVE_UDP_FEEDBACK':'控制器通信已建立，但未收到有效 UDP 关节反馈',
        'UDP_MONITORING_CADENCE_UNSUITABLE':'关节反馈频率或间隔不满足执行监测要求',
        'ModuleNotFoundError':'当前 Python 环境缺少已配置设备的适配器依赖',
        'No such file':'配置的设备路径不存在',
        'EXECUTION_PROCESS_TIMEOUT':'机械臂通信进程未按时回复，停止状态未知；请现场检查',
    }
    prefix=next((text for code,text in explanations.items() if code in message),'设备通信失败，请检查下方原始原因')
    return prefix+'；'+message


@dataclass(frozen=True)
class VisionPacket:
    generation: int
    frame: object
    joint: object
    result: object
    pixels: object


def overlay(frame, result, segmentation):
    if frame.observation_id != result.observation_id:
        raise ValueError('OVERLAY_FRAME_MISMATCH')
    pixels = frame.rgb.copy()
    for i, item in enumerate(result.detections):
        color = (60+(i*53)%170, 220-(i*37)%150, 100+(i*71)%150)
        if segmentation:
            mask = result.masks[i]
            pixels[mask] = (pixels[mask]*.6+np.asarray(color)*.4).astype('uint8')
            contours, _ = cv2.findContours(mask.astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(pixels, contours, -1, color, 1)
        else:
            x1,y1,x2,y2 = np.round(item['bbox_xyxy']).astype(int)
            cv2.rectangle(pixels, (x1,y1), (x2,y2), color, 2)
            cv2.putText(pixels, f"{i} {item['label']} {item['score']:.3f}",
                        (max(0,x1),max(15,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
    pixels.setflags(write=False)
    return pixels


class LiveVision:
    """One model owner, current frame + one latest pending frame.

    A task can reserve that one pending slot for an immutable frozen exposure;
    live offers resume as soon as it is consumed. Cancellation never joins GPU.
    Results are two replaceable slots, not an ever-growing UI event queue.
    """
    def __init__(self, config, factory=HuggingFaceGroundingSAM):
        self.config=config;self.factory=factory;self.model=None
        self.condition=threading.Condition();self.generation=0;self.pending=None
        self.current=None;self.task_waiter=None;self.closed=False;self.thread=None
        self.outputs={};self.status='等待相机';self.error='';self.dropped=0
        self.stage_status={'detection':'等待相机','segmentation':'等待相机'}
        self.last_offer=None;self.completed=0
    def start(self):
        with self.condition:
            if self.thread:return
            self.thread=threading.Thread(target=self._run,daemon=True,name='continuous-dino-sam')
            self.thread.start()
    def invalidate(self, generation):
        with self.condition:
            self.generation=generation;self.pending=None;self.outputs.clear();self.last_offer=None
            self.stage_status={stage:'新会话 · 等待结果' for stage in self.stage_status}
            if self.task_waiter:
                self.task_waiter['error']='OBSERVATION_INVALIDATED';self.task_waiter['done']=True
            self.condition.notify_all()
    def offer(self,frame,joint,generation):
        with self.condition:
            if generation!=self.generation or self.closed or self.error:return
            if self.last_offer==frame.observation_id:return
            self.last_offer=frame.observation_id
            if self.pending and self.pending[3] is not None:
                self.dropped+=1;return
            if self.pending:self.dropped+=1
            self.pending=(frame,joint,generation,None);self.condition.notify_all()
    def infer_frozen(self,frame,generation,cancelled=lambda:False):
        waiter={'done':False}
        with self.condition:
            if generation!=self.generation:raise ValueError('OBSERVATION_INVALIDATED')
            if self.task_waiter:raise ValueError('VISION_TASK_ALREADY_PENDING')
            if self.error:raise RuntimeError(self.error)
            self.task_waiter=waiter;self.pending=(frame,None,generation,waiter);self.condition.notify_all()
            try:
                while not waiter['done']:
                    if cancelled() or self.closed:raise ValueError('CANCELLED')
                    if self.error:raise RuntimeError(self.error)
                    self.condition.wait(.02)
                if 'error' in waiter:raise ValueError(waiter['error'])
                return waiter['result']
            finally:
                if self.pending and self.pending[3] is waiter:self.pending=None
                self.task_waiter=None
    def snapshot(self):
        with self.condition:return dict(self.outputs),self.status,self.error
    def retry(self):
        with self.condition:
            self.error='';self.condition.notify_all()
    def _publish(self,stage,frame,joint,generation,result):
        packet=VisionPacket(generation,frame,joint,result,overlay(frame,result,stage=='segmentation'))
        with self.condition:
            if generation==self.generation and not self.closed:self.outputs[stage]=packet
    def _run(self):
        while True:
            with self.condition:
                while not self.closed and (self.error or self.pending is None):self.condition.wait(.1)
                if self.closed:return
                job=self.pending;self.pending=None;self.current=job
            frame,joint,generation,waiter=job
            try:
                if self.model is None:
                    self.status='正在加载本地模型';self.model=self.factory(**self.config)
                self.status='检测处理中'
                self.stage_status['detection']='检测处理中';self.stage_status['segmentation']='等待同帧检测'
                detected=self.model.detect_boxes(frame)
                self._publish('detection',frame,joint,generation,detected)
                self.stage_status['detection']='检测已完成 · 持续运行'
                with self.condition:
                    if generation!=self.generation:continue
                self.status='SAM 分割处理中'
                self.stage_status['segmentation']='SAM 分割处理中'
                segmented=self.model.segment(frame,detected)
                self._publish('segmentation',frame,joint,generation,segmented)
                with self.condition:
                    if waiter and generation==self.generation:
                        waiter.update(done=True,result=segmented)
                    self.completed+=1;self.status='持续运行 · 等待最新帧'
                    self.stage_status['segmentation']='分割已完成 · 持续运行'
            except Exception as exc:
                with self.condition:
                    if generation==self.generation:
                        self.error=f'{type(exc).__name__}: {exc}';self.status='模型加载 / 推理失败'
                    if waiter:waiter.update(done=True,error=str(exc))
            finally:
                with self.condition:self.current=None;self.condition.notify_all()
    def close(self):
        with self.condition:
            self.closed=True;self.pending=None;self.outputs.clear();self.condition.notify_all()
        # GPU work owns no hardware/Tk handles and may finish in the background.


class SingleBackend(ControlledBackend):
    def __init__(self,*args):
        super().__init__(*args);self.manager=None;self.vision=None;self.pump=None
        self.workers['robot'].interval=.02
        self.device_errors={k:'' for k in self.workers}
        self.states={k:'已断开' for k in self.workers};self.endpoints={k:'' for k in self.workers}
        self.ui_events=deque(maxlen=32);self.pump_stop=threading.Event()
        self.auto_profile=StreamProfile(640,480,30);self.auto_camera=False
        self.logs.clear();self.log('实机单箱：自动观测；运动与阀门仅由执行按钮启动')
    def start_observation(self,vision):
        self.vision=vision;vision.invalidate(self.observation_generation);vision.start()
        self.pump=threading.Thread(target=self._pump,daemon=True,name='live-device-handoff');self.pump.start()
    def command(self,device,action,*args):
        if self.manager and (self.manager.active or self.manager.vacuum.holding or
                (self.manager.cycle and self.manager.cycle.state=='STOP_UNCONFIRMED')):
            raise ValueError('任务正在运行、可能持箱或停止未确认；不得切换设备')
        if action=='connect':
            self.states[device]='连接中';self.device_errors[device]=''
            self.endpoints[device]=':'.join(map(str,args)) or 'D435i 自动唯一设备'
            if device=='camera':self.auto_camera=True
        return super().command(device,action,*args)
    def invalidate_device(self,device):
        super().invalidate_device(device)
        # Revoke the association session, not just the last visible result.
        # Every device must contribute a receipt newer than this transition.
        cutoff=time.monotonic_ns()
        self.frame=self.joint=self.pressure=self.detections=None;self.joint_history.clear()
        self.sample_cutoff={name:cutoff for name in self.workers}
        owner=getattr(self.workers['vacuum'].device,'owner',None)
        if owner is not None:owner.min_received_s=cutoff/1e9
        if self.vision:self.vision.invalidate(self.observation_generation)
    def sample_state(self,stamp,device):
        if stamp.received_monotonic_ns<self.sample_cutoff[device]:return 'WAITING_CURRENT_SESSION'
        return super().sample_state(stamp,device)
    def _pump(self):
        previous={k:False for k in self.workers};last_frame=None
        while not self.pump_stop.wait(.005):
            events=super().poll()
            for name,kind,value in events:
                if kind=='sample':continue
                self.ui_events.append((name,kind,value))
                if kind=='error':
                    self.device_errors[name]=connection_error(value);self.states[name]='连接失败'
                elif kind=='state':
                    action=value['action']
                    self.states[name]='已连接' if action in {'connect','start'} else '已断开'
                    if name=='camera' and action=='connect' and self.auto_camera:
                        if self.auto_profile not in value['value']:
                            p=self.auto_profile
                            self.device_errors[name]=f'{p.width}×{p.height}@{p.fps}fps 无可用 RGB-D 配对；请在相机配置中选择设备或明确修改采集配置'
                            self.states[name]='连接失败';self.log(self.device_errors[name])
                        else:
                            # Use the same lifecycle invalidation as a manual restart.
                            try:self.command('camera','start',self.auto_profile)
                            except Exception as e:
                                self.device_errors[name]=str(e);self.states[name]='连接失败'
            for name,w in self.workers.items():
                connected=w.device.connected
                if previous[name] and not connected:
                    self.invalidate_device(name)
                    self.states[name]='已断开'
                    if name=='vacuum':self.device_errors[name]=connection_error(w.device.owner.last_error)
                previous[name]=connected
            frame=self.frame
            if self.vision and frame is not None and frame is not last_frame:
                last_frame=frame
                self.vision.offer(frame,self.joint_for_frame(frame),self.observation_generation)
    def poll(self):
        if self.pump is None:return super().poll()
        events=[]
        while self.ui_events:
            try:events.append(self.ui_events.popleft())
            except IndexError:break
        return events
    def device_state(self,name):
        w=self.workers[name];state=self.states[name]
        if w.busy:return '连接中'
        if self.device_errors[name] and not w.device.connected:return '连接失败'
        if not w.device.connected:return '已断开'
        if state=='连接失败':return state
        sample=self.frame if name=='camera' else self.joint if name=='robot' else w.device.gauges().get(1)
        if sample is None or sample.stamp.received_monotonic_ns<self.sample_cutoff[name]:return '等待数据'
        limit=.1 if name=='robot' else self.manager.config['vacuum']['max_age_s'] if name=='vacuum' and self.manager else 1.
        return '数据有效' if sample.stamp.age_s() is not None and sample.stamp.age_s()<=limit else '数据过期'
    def close(self):
        self.pump_stop.set()
        if self.vision:self.vision.close()
        if self.pump:self.pump.join(.2)
        super().close()
