"""Ubuntu explicit visual single-carton entry. Automatic observation; motion requires an operator click."""
import argparse
import json
from pathlib import Path
from queue import Queue,Empty
import threading
import time
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image,ImageTk
from ..ui import Workbench,load_mapping,ROOT
from .network import NetworkedRobotView
from ..stable_ui import StableStringVar, fixed_text, constrain_labels
from .live import SingleBackend, LiveVision
from .result_window import ResultWindow
from ..controlled.service import ExecutionService
from ..controlled.trajectory import Authority
from ..devices import D435iObserver,StreamProfile
from ..coordinates import Extrinsics,CoordinateModel,Installation
from ..ui import URDF,URDF_SHA
from .config import load,issues
from .planner import SinglePlanner
from .vacuum import VacuumOwner,VacuumView
from .runtime import TaskManager
from .geometry import transform


def project_camera(points,intrinsics):
    """Same Brown distortion convention as RGB-D deprojection."""
    from ..reference.camera_geometry import ProjectionInfo,opencv_brown_coefficients
    k=intrinsics
    info=ProjectionInfo('realsense',k['width'],k['height'],k['fx'],k['fy'],k['ppx'],k['ppy'],
                        distortion=tuple(k['coeffs']),distortion_model=k['model'])
    coefficients=(0.,)*5 if k['model']=='SYNTHETIC_PINHOLE' else opencv_brown_coefficients(info)
    K=np.array([[k['fx'],0,k['ppx']],[0,k['fy'],k['ppy']],[0,0,1.]])
    uv,_=cv2.projectPoints(np.asarray(points,float),np.zeros(3),np.zeros(3),K,np.asarray(coefficients))
    return uv.reshape(-1,2)

LABELS=dict(IDLE='待机',OBSERVING='停稳观测 / 推理',TARGET_READY='观测已冻结，选择目标后规划',PLANNING='规划与完整路径检查',
    PLAN_READY='计划就绪',APPROACH='接近',CONTACT='接触',ESTABLISH_VACUUM='建立真空',ATTACHMENT_CONFIRMED='吸附已确认',
    DETACH='脱离原位置',RETURN_LOADED='带载返回本次 q_start',RELEASE='释放并等待压力确认',COMPLETE='完成',
    STOPPING='正在停止，等待实测确认',FAILED='失败',CANCELLED='已取消',STOP_UNCONFIRMED='停止未确认')


class SingleWorkbench(Workbench):
    def safe(self,function,*args):
        try:return function(*args)
        except Exception as exc:
            self.backend.log(f'ERROR {type(exc).__name__}: {exc}')
            self.notice.set(str(exc))
            if hasattr(self,'details'):self.details.set(str(exc))
    def __init__(self,root,backend,planner,service,vacuum,config,model,calibration,*,auto_start=True):
        self.jobs=Queue();self.config_single=config;self.previewed=False;self.closing_single=False;self.feedback_pending=False
        self.service=service;self.planner=planner;self.last_status={}
        self.restart_required=False;self.job_pending=False;self.job_started=0.;self.job_label=''
        super().__init__(root,backend,installation=config['execution']['installation'])
        self.model=model;self.calibration=calibration
        self.manager=TaskManager(backend,service,vacuum,planner,model,calibration,config,lambda k,v:self.jobs.put((k,v)))
        backend.manager=self.manager
        self.robot_host.set(config['execution']['host']);self.robot_port.set(str(config['execution']['port']))
        self.serial_port.set(config['serial_port'])
        self.camera_serial.set(config.get('camera',{}).get('serial',''))
        root.title('ECO65-B · 视觉单箱抓取并返回释放 · 实机模式')
        labels=[w for w in root.winfo_children() if isinstance(w,ttk.Label)]
        labels[0].configure(text='ECO65-B / 视觉单箱抓取并返回释放')
        labels[1].configure(text='自动连接与持续视觉；仅执行按钮启动运动。吸具 1.5 kg + 箱子 2 kg；统一阀门、被动止回吸盘。软件停止不是硬件急停。')
        self.calibration_text.set('现场确认：link_1 相机；world=base。加载参考原文件，历史标定状态原样保留。')
        tab=ttk.Frame(self.tabs,padding=10);self.task_tab=tab;self.tabs.add(tab,text='视觉单箱任务');self.tabs.select(tab)
        self.task_status=StableStringVar(value='待机 · 设备未连接 · 实机执行 NOT_RUN')
        fixed_text(tab,self.task_status,height=2,pady=4)
        row=ttk.Frame(tab);row.pack(fill='x')
        self.button(row,'重试机械臂',lambda:self.retry_device('robot'))
        self.button(row,'重试真空板',lambda:self.retry_device('vacuum'))
        self.button(row,'相机配置 / 原始预览',lambda:self.tabs.select(self.observe))
        self.button(row,'现场配置',self.open_config)
        self.button(row,'刷新控制器负载（只读）',lambda:self.job('payload',service.check_payload))
        row=ttk.Frame(tab);row.pack(fill='x',pady=10)
        self.observe_button=self.button(row,'准备观测 / 锁定起始点',lambda:self.job('observe_done',self.manager.observe))
        ttk.Label(row,text='目标检测编号（空=自动选择）').pack(side='left',padx=5)
        self.target=StableStringVar();ttk.Entry(row,textvariable=self.target,width=5).pack(side='left')
        self.plan_button=self.button(row,'规划所选箱',self.plan_selected);self.plan_button.configure(state='disabled')
        self.preview_button=self.button(row,'查看完整轨迹预览',self.preview)
        self.execute_button=self.button(row,'执行单箱',self.execute_cycle);self.execute_button.configure(state='disabled')
        self.button(row,'停止（不自动释放）',self.stop_cycle)
        receiver_notice=('本轮承接区资格核验已按用户要求延期；仍检查完整带箱回程及释放包络，支撑未确认。'
            if config.get('release_policy')=='USER_DEFERRED_RECEIVER_QUALIFICATION' else
            ('实测承接策略：规划前请填写现场 receiver。' if not config.get('site',{}).get('receiver') else '实测承接策略：已加载现场 receiver。'))
        receiver_notice+='\n'+('未观测区域：用户已确认清空；实测障碍与完整箱体仍检查碰撞。'
            if config.get('site',{}).get('unknown_policy')=='USER_CONFIRMED_CLEAR' else '未观测区域：按占用处理。')
        self.details=StableStringVar(value='启动后自动连接三类设备，640×480@30fps；检测和分割持续运行。\n支持部分出画面与遮挡；尺寸先验可配置，完整不确定包络参与碰撞。\n'+('\n'.join(issues(config)) or receiver_notice))
        fixed_text(tab,self.details,height=4,pady=4)
        self.bound_details=StableStringVar(value='本次起始点、目标和释放位置：尚未锁定')
        fixed_text(tab,self.bound_details,height=4,pady=4)
        self.health=StableStringVar();fixed_text(tab,self.health,height=4)
        self.preview_label=StableStringVar(value='轨迹预览与实测执行分别显示；预览不是机器人反馈')
        ttk.Label(tab,textvariable=self.preview_label,width=1).pack(side='bottom',fill='x')
        visual=ttk.Frame(tab);visual.pack(fill='both',expand=True,pady=8)
        frozen=ttk.Frame(visual,width=520,height=390);frozen.pack(side='left',fill='y');frozen.pack_propagate(False)
        self.frozen_image=ttk.Label(frozen,text='本次冻结观测、长方体与选中面',width=1);self.frozen_image.pack(fill='both',expand=True)
        self.canvas=tk.Canvas(visual,width=650,height=260,bg='#142333',highlightthickness=0);self.canvas.pack(side='right',fill='both',expand=True)
        self.live_vision=LiveVision(config['detector'])
        self.result_windows={stage:ResultWindow(root,stage) for stage in ('detection','segmentation')}
        def adjust_controls(parent):
            for child in parent.winfo_children():
                if isinstance(child,ttk.Button):
                    title=child.cget('text')
                    if title in {'加载回放目录','下一条回放','停止回放'}:child.configure(state='disabled')
                    elif title=='停止检测':child.configure(text='持续检测状态',command=lambda:self.notice.set(self.live_vision.status))
                    elif title=='开始检测':child.configure(text='重试视觉',command=self.live_vision.retry)
                    elif title=='连接 / 查询配置':child.configure(text='重试相机',command=lambda:self.safe(self.retry_device,'camera'))
                    elif title=='查看完整场景':child.configure(text='历史仿真查看器（独立）')
                adjust_controls(child)
        adjust_controls(self.observe)
        for child in self.coords.winfo_children():
            if isinstance(child,ttk.Label) and 'NativeSceneWorld' in str(child.cget('text')):
                child.configure(text='当前实机坐标采用 base_link。历史仿真查看器请通过独立只读入口打开；不进入实机碰撞场景。',width=1)
        self.frozen_image.bind('<Configure>',lambda _:self.draw_observation())
        livebar=ttk.Frame(root);livebar.pack(side='top',fill='x',before=self.tabs,padx=12)
        self.device_summary=StableStringVar(value='相机：已断开 | 机械臂：已断开 | 真空：已断开')
        ttk.Label(livebar,textvariable=self.device_summary,width=1).pack(side='left',fill='x',expand=True)
        self.button(livebar,'检测窗口',self.result_windows['detection'].open)
        self.button(livebar,'分割窗口',self.result_windows['segmentation'].open)
        self.button(livebar,'重试模型',self.live_vision.retry)
        self.device_details=StableStringVar()
        fixed_text(root,self.device_details,height=3,before=self.tabs,padx=12)
        constrain_labels(root)
        if auto_start:root.after_idle(self.autostart)
        root.after(100,self.update_task)
    def autostart(self):
        camera=self.config_single.get('camera',{})
        self.backend.auto_profile=StreamProfile(camera.get('width',640),camera.get('height',480),camera.get('fps',30))
        self.backend.start_observation(self.live_vision)
        for device in ('camera','robot','vacuum'):
            self.safe(self.retry_device,device)
    def retry_device(self,device):
        worker=self.backend.workers[device]
        if worker.busy:raise ValueError('连接操作仍在进行，请等待当前结果')
        if worker.device.connected:self.backend.command(device,'disconnect')
        if device=='camera':
            self.backend.command('camera','connect',self.camera_serial.get())
        elif device=='robot':
            self.backend.command('robot','connect',self.robot_host.get(),int(self.robot_port.get()))
        else:self.backend.command('vacuum','connect',self.serial_port.get())
    def open_config(self):
        # Fixed scrollable editor; apply is a file change, never a device write.
        # Existing in-flight plans are invalidated by source hashes.
        from tkinter.scrolledtext import ScrolledText
        window=tk.Toplevel(self.root);window.title('现场配置（保存后重启应用）');window.geometry('820x620')
        paths=[self.config_single['config_path'],self.config_single.get('site_scene'),self.config_single.get('device_config')]
        paths=[p for p in paths if p]
        selected=StableStringVar(value=paths[0]);combo=ttk.Combobox(window,textvariable=selected,values=paths,state='readonly',width=1)
        combo.pack(fill='x');editor=ScrolledText(window,width=1,height=1,wrap='none');editor.pack(fill='both',expand=True)
        def read(*_):
            editor.delete('1.0','end');editor.insert('end',Path(selected.get()).read_text() if Path(selected.get()).exists() else '{}')
        def save():
            if self.manager.active or self.manager.vacuum.holding:raise ValueError('任务活动或可能持箱时不可修改配置')
            raw=editor.get('1.0','end');json.loads(raw)
            Path(selected.get()).write_text(raw);self.planner.authority.invalidate();self.manager.plan=None
            self.restart_required=True;self.manager.phase='IDLE'
            self.previewed=False;self.execute_button.configure(state='disabled')
            self.notice.set('配置已保存；请关闭并重启工作台以应用新配置')
        combo.bind('<<ComboboxSelected>>',read);read()
        self.button(window,'保存配置（重启生效）',save)
        self.button(window,'复制全部',lambda:(self.root.clipboard_clear(),self.root.clipboard_append(editor.get('1.0','end'))))

    def job(self,kind,fn):
        if self.restart_required:raise ValueError('配置已更改，请重启应用以重新绑定数据与模型')
        if self.manager.active:raise ValueError('已有活动任务或操作，请勿重复启动')
        self.previewed=False;self.execute_button.configure(state='disabled')
        # UI reservation happens before thread startup to reject double-clicks.
        if getattr(self,'job_pending',False):raise ValueError('操作尚未完成')
        self.job_pending=True;self.job_started=time.monotonic()
        self.job_label={'observe_done':'准备观测 / 锁定起始点','plan_done':'完整路径规划',
                        'execute_done':'执行前复核','payload':'读取控制器负载'}.get(kind,kind)
        self.observe_button.configure(state='disabled');self.plan_button.configure(state='disabled')
        self.tabs.select(self.task_tab)
        message='已受理：'+self.job_label+' · 正在后台处理'
        self.task_status.set(message);self.details.set(message);self.notice.set(message)
        if kind=='observe_done':
            self.frozen_image.configure(image='',text='正在取景；锁定后先显示原始冻结帧，再更新同帧分割与几何')
            self._photos.pop('frozen',None)
            self.bound_details.set('正在等待停稳取景；本次起始点尚未锁定')
        def run():
            try:self.jobs.put((kind,fn()))
            except Exception as e:self.jobs.put(('error',str(e)))
            finally:self.jobs.put(('job_finished',None))
        threading.Thread(target=run,daemon=True,name='single-carton-ui-job').start()
    def plan_selected(self):
        target=self.target.get().strip();selected=int(target) if target else None
        self.job('plan_done',lambda:self.manager.prepare_plan(selected))
    def preview(self):
        plan=self.manager.plan
        if plan is None or self.manager.phase!='PLAN_READY':raise ValueError('请先生成完整计划')
        if self.manager.observation.generation!=self.backend.observation_generation:
            raise ValueError('观测会话已失效，请重新准备任务')
        self.canvas.delete('all');offset=0.;times=[];joints=[];marks=[]
        for s in plan['segments']:
            p=s['package']['prepared'];times.extend(offset+x['t'] for x in p['points']);joints.extend(x['q'] for x in p['points']);marks.append((offset,s['stage']));offset+=p['send_duration_s']
        t=np.asarray(times);q=np.rad2deg(joints);w=max(self.canvas.winfo_width(),600);h=max(self.canvas.winfo_height(),250);lo=q.min()-1;hi=q.max()+1
        for j,color in enumerate(('#ffbf69','#66dce6','#9be564','#df80ff','#ff7080','#ffffff')):
            xy=np.c_[20+(w-40)*t/t[-1],h-25-(h-60)*(q[:,j]-lo)/(hi-lo)][::max(1,len(t)//900)]
            self.canvas.create_line(*xy.ravel(),fill=color,width=2)
        for t0,name in marks:self.canvas.create_text(30+(w-80)*t0/offset,15,text=name,fill='#fff',anchor='nw')
        scales='/'.join(f"{s['package']['prepared']['time_scale']:g}" for s in plan['segments'])
        self.preview_label.set(f'规划预览：4 段运动总时长 {offset:.2f} s；真空等待另计。各段发送时间缩放 ×{scales}（1 = 原生规划速度）')
        self.previewed=True;self.execute_button.configure(state='normal');self.draw_observation()
    def execute_cycle(self):
        if not self.previewed:raise ValueError('请先查看本次完整轨迹预览')
        self.job('execute_done',self.manager.execute)
    def stop_cycle(self):
        self.manager.stop();self.previewed=False;self.execute_button.configure(state='disabled')
        self.details.set('已请求软件停止。保持阀门状态，继续核对实际停止反馈；不会自动返回或释放。')
    def draw_observation(self):
        m=self.manager
        obs,result,boxes,plan=m.observation,m.result,m.boxes,m.plan
        if obs is None:return
        f=obs.frame;pixels=f.rgb.copy()
        matched=result is not None and result.observation_id==f.observation_id
        for i,mask in enumerate(result.masks if matched else ()):
            pixels[mask]=(pixels[mask]*.65+np.array([30,180,250])*.35).astype('uint8')
            b=result.detections[i]['bbox_xyxy'];x,y=int(b[0]),int(b[1]);cv2.putText(pixels,str(i),(x,y),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,220,50),2)
        camera_world=np.linalg.inv(np.asarray(obs.world_camera))
        for box in boxes if matched else ():
            p=transform(camera_world,box['corners_m'])
            if np.any(p[:,2]<=0):continue
            uv=project_camera(p,f.intrinsics).astype(int)
            selected=plan is not None and box['id']==plan['target']['id'];color=(255,200,20) if selected else (60,255,120)
            for a in range(8):
                for b in range(a+1,8):
                    if (a^b) in (1,2,4):cv2.line(pixels,tuple(uv[a]),tuple(uv[b]),color,2)
            if selected:
                face=plan['candidate']['face'];c=np.asarray(face['center_m']);p=transform(camera_world,[c,c+.04*np.asarray(face['normal'])]);uv=project_camera(p,f.intrinsics).astype(int)
                cv2.arrowedLine(pixels,tuple(uv[0]),tuple(uv[1]),(255,70,60),3)
        im=Image.fromarray(pixels);im.thumbnail((max(1,self.frozen_image.winfo_width()),max(1,self.frozen_image.winfo_height())))
        photo=ImageTk.PhotoImage(im);self._photos['frozen']=photo;self.frozen_image.configure(image=photo,text='')
    def update_task(self):
        if self._closed:return
        outputs,vision_status,vision_error=self.live_vision.snapshot()
        for stage,window in self.result_windows.items():
            packet=outputs.get(stage)
            if packet and packet.generation!=self.backend.observation_generation:packet=None
            window.update(packet,self.live_vision.stage_status[stage]+' · '+vision_status,vision_error)
        segmentation=outputs.get('segmentation')
        if segmentation and segmentation.generation==self.backend.observation_generation:
            self.original_detection_frame=segmentation.frame;self.original_detection_joint=segmentation.joint
            self.backend.detections=segmentation.result
        labels={'camera':'相机','robot':'机械臂','vacuum':'真空'}
        self.device_summary.set(' | '.join(labels[k]+'：'+self.backend.device_state(k) for k in labels))
        network=getattr(self.backend.workers['robot'].device,'network',None)
        self.device_details.set('\n'.join(labels[k]+' · '+self.backend.endpoints[k]+' · '+
            ('通信已连接' if self.backend.workers[k].device.connected else '通信未连接')+' · '+self.backend.device_state(k)+' · '+
            ((network.status+' · ') if k=='robot' and network else '')+self.backend.device_errors[k] for k in labels))
        while True:
            try:kind,value=self.jobs.get_nowait()
            except Empty:break
            if kind=='job_finished':self.job_pending=False
            elif kind=='progress':self.details.set(value);self.notice.set(value)
            elif kind=='error':self.details.set(value);self.notice.set(value);self.backend.log(value)
            elif kind=='capture':
                self.bound_details.set('锁定 q_start（°）：'+str(np.rad2deg(value.q_start).round(3).tolist())+
                    '\n采集 q_capture（°）：'+str(np.rad2deg(value.q_capture).round(3).tolist())+
                    f'\n本次 RGB/depth #{value.frame.rgb_number}/{value.frame.depth_number} · 等待同帧检测 / 分割'+
                    '\n起始 TCP（m）：'+str(np.asarray(value.world_tcp_start)[:3,3].round(5).tolist()))
                self.draw_observation()
            elif kind=='observation':
                obs,result,boxes=value
                self.details.set('本次 q_start（度）：'+' '.join(f'{x:.3f}' for x in np.rad2deg(obs.q_start))+f'\n恢复 {len(boxes)} 个箱体；选择检测编号或留空自动选择。')
                if boxes:self.details.set(self.details.get()+'\n碰撞包络尺寸（mm，含先验不确定性）：'+'；'.join(str(b['detection_index'])+' → '+str((np.asarray(b['size_m'])*1000).round(1).tolist()) for b in boxes))
                self.bound_details.set('锁定 q_start（°）：'+str(np.rad2deg(obs.q_start).round(3).tolist())+
                    '\n起始 TCP（m）：'+str(np.asarray(obs.world_tcp_start)[:3,3].round(5).tolist()))
                self.draw_observation()
                self.notice.set(self.details.get())
            elif kind=='plan':
                qualification=('承接区资格核验延期；支撑未确认。' if value['release']['status']=='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'
                               else '承接区域按现场实测配置检查；不代表传感器证实支撑。')
                self.details.set('目标：'+value['target']['id']+'\n接触 TCP（m）：'+str(np.array(value['candidate']['tcp_pose'])[:3,3].round(5).tolist())+'\n返回同一组 q_start；释放包络已检查。'+qualification)
                self.bound_details.set('锁定 q_start（°）：'+str(np.rad2deg(value['q_start']).round(3).tolist())+
                    '\n目标 '+value['target']['id']+' · 接触 TCP（m） '+str(np.asarray(value['candidate']['tcp_pose'])[:3,3].round(5).tolist())+
                    '\n释放箱心（m） '+str(np.asarray(value['release']['pose'])[:3,3].round(5).tolist())+
                    f" · 预计密封 {len(value['candidate']['active_ids'])}/12 杯 · 总阀控制、其余杯被动关闭"+
                    '\n成功条件：实测到位 → 压力稳定吸牢 → 脱离并返回原关节 → 压力稳定释放')
                self.draw_observation()
            elif kind=='payload':self.details.set(json.dumps(value,ensure_ascii=False))
            elif kind=='feedback':self.last_status=value;self.feedback_pending=False
            elif kind=='result':self.details.set(json.dumps(value,ensure_ascii=False));self.execute_button.configure(state='disabled')
            elif kind=='close_error':self.details.set(value);self.closing_single=False
            elif kind=='close_done':self._closed=True;self.root.destroy();return
        m=self.manager;state=m.cycle.state if m.cycle else m.phase
        stale=m.observation is not None and m.observation.generation!=self.backend.observation_generation
        if stale:
            self.previewed=False;self.execute_button.configure(state='disabled')
        self.plan_button.configure(state='normal' if m.phase=='TARGET_READY' and not stale and not m.active and not getattr(self,'job_pending',False) else 'disabled')
        self.observe_button.configure(state='disabled' if self.job_pending or m.active or m.vacuum.holding or
                                      (m.cycle and m.cycle.state=='STOP_UNCONFIRMED') else 'normal')
        state_text='观测会话已失效，请重新准备任务' if stale and not m.active else LABELS.get(state,state)
        elapsed=f' · {self.job_label} 已用 {time.monotonic()-self.job_started:.1f} s' if self.job_pending else ''
        self.task_status.set(f'实机单箱模式 · {state_text}{elapsed} · task_id {m.task_id or "—"}\n原因：{m.reason or (m.cycle.reason if m.cycle else "—")}')
        status=m.cycle.last_feedback if m.cycle and m.cycle.active else self.last_status
        status=status or {};q=status.get('actual_q');age=status.get('feedback_age_s');error=status.get('tracking_error_rad')
        joint=self.backend.joint
        if joint is not None:
            q=joint.q_rad;age=joint.stamp.age_s()
        try:p=self.manager.vacuum.pressure();pressure=f"{p['kpa']:.2f} kPa · 年龄 {__import__('time').monotonic()-p['received_s']:.3f} s"
        except Exception as e:pressure=str(e)
        faults=joint.raw.get('raw',{}) if joint else {}
        alarm=dict(alarm=joint.raw.get('alarm') if joint else None,system=faults.get('err'),joints=faults.get('joint_status',{}).get('joint_err_code'))
        command=status.get('command_q');goal=status.get('goal_q')
        def numbers(values):return '—' if values is None else ' '.join(f'{x:+09.3f}' for x in np.rad2deg(values))
        capabilities=self.last_status.get('capabilities',{})
        payload_ok=capabilities.get('payload_check',{}).get('matches',capabilities.get('payload_agrees',False))
        executable=bool(joint and joint.raw.get('enabled') and not joint.raw.get('alarm') and
                        self.backend.device_state('robot')=='数据有效' and self.backend.device_state('vacuum')=='数据有效' and
                        payload_ok and m.phase=='PLAN_READY' and self.previewed and m.observation is not None and
                        m.observation.generation==self.backend.observation_generation)
        self.health.set(f"执行准备：{'待执行前复核' if executable else '尚未满足执行条件'} · 实测 ° {numbers(q)}\n指令 ° {numbers(command)} · 本段目标 ° {numbers(goal)}\n跟踪误差 ° {numbers(error)} · 反馈年龄 {format(age,'.3f') if isinstance(age,(int,float)) else '—'} s · 吸具 {pressure}\n控制器报警：{alarm}")
        if self.service.process is not None and not self.feedback_pending and not m.active:
            self.feedback_pending=True
            def read():self.jobs.put(('feedback',self.service.feedback()))
            threading.Thread(target=read,daemon=True).start()
        self.root.after(100,self.update_task)
    def choose_tool(self):raise ValueError('实机工具由本次配置绑定；更改后重启并重新准备任务')
    def change_tool(self):raise ValueError('实机模式不能切换工具参考')
    def choose_calibration(self):raise ValueError('实机外参由现场配置绑定；更改配置后重新启动')
    def choose_config(self):self.open_config()
    def load_plan(self):raise ValueError('实机单箱入口只接受本次冻结观测生成的完整计划；预览 JSON 不能导入执行')
    def start_detection(self):
        self.live_vision.retry();self.notice.set('持续检测和分割已启用；任务使用独立冻结帧')
    def stop_detection(self):
        # The base UI session reset must not stop continuous inference.
        self.detecting=False
    def _sync_observations(self):
        if self._observed_generation!=self.backend.observation_generation:
            self._observed_generation=self.backend.observation_generation
            self.original_detection_frame=None;self.original_detection_joint=None
            self._shown_detection=None
    def close(self):
        if self.closing_single:return
        if not hasattr(self,'manager'):return super().close()
        self.manager.stop();self.closing_single=True
        def finish():
            try:
                self.manager.close();self.live_vision.close();self.backend.close()
                self.jobs.put(('close_done',None))
            except Exception as e:self.jobs.put(('close_error',str(e)))
        threading.Thread(target=finish,daemon=True,name='single-carton-close').start()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'configs/workbench/single_carton.example.json');p.add_argument('--ui-smoke',action='store_true');p.add_argument('--check-config',action='store_true');args=p.parse_args()
    config=load(args.config)
    if args.check_config:
        print(json.dumps(dict(issues=issues(config),release_policy=config.get('release_policy'),
            receiver_qualification_deferred=config.get('release_policy')=='USER_DEFERRED_RECEIVER_QUALIFICATION',
            receiver_configured=bool(config.get('site',{}).get('receiver')),unknown_space_policy=config.get('site',{}).get('unknown_policy'),detector_backend=config['detector'].get('backend','local'),
            detector_remote_configured=bool((config['detector'].get('remote') or {}).get('host')),hardware_execution='NOT_RUN'),ensure_ascii=False,indent=2));return
    installation=Installation.load(config['execution']['installation']);model=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
    calibration=Extrinsics.load(config['camera_extrinsics']) if config.get('camera_extrinsics') else None
    if calibration:installation.check_camera(calibration)
    mapping=load_mapping(config['execution']['mapping']);authority=Authority('eco65_single_carton_segment_v1');planner=SinglePlanner(config,mapping,authority,model)
    service=ExecutionService(mapping,config['execution'],authority,planner.binding);vacuum=VacuumOwner(config['vacuum'])
    backend=SingleBackend(D435iObserver(),NetworkedRobotView(service,mapping,config.get('robot_network',{})),VacuumView(vacuum),service)
    root=tk.Tk();ui=SingleWorkbench(root,backend,planner,service,vacuum,config,model,calibration,auto_start=not args.ui_smoke)
    if args.ui_smoke:root.after(700,ui.close)
    root.mainloop()

if __name__=='__main__':main()
