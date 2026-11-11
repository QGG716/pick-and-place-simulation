"""Ubuntu explicit visual single-carton entry. No automatic device connection."""
import argparse
import json
from pathlib import Path
from queue import Queue,Empty
import threading
import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
from PIL import Image,ImageTk
from ..ui import Workbench,load_mapping,ROOT
from ..controlled.ui import ControlledBackend,ManagedRobotView
from ..controlled.service import ExecutionService
from ..controlled.trajectory import Authority
from ..devices import D435iObserver
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


class SingleBackend(ControlledBackend):
    def __init__(self,*args):
        super().__init__(*args);self.manager=None
        # This facade reads the owner's already-received UDP samples; it does not
        # increase SDK TCP polling. Capture pairs directly with original receipts.
        self.workers['robot'].interval=.02
    def command(self,device,action,*args):
        if self.manager and (self.manager.active or self.manager.vacuum.holding):
            raise ValueError('任务正在运行或仍可能持箱；请先使用任务区停止，不切换设备')
        return super().command(device,action,*args)


class SingleWorkbench(Workbench):
    def __init__(self,root,backend,planner,service,vacuum,config,model,calibration):
        self.jobs=Queue();self.config_single=config;self.previewed=False;self.closing_single=False;self.feedback_pending=False
        self.service=service;self.planner=planner;self.last_status={}
        super().__init__(root,backend,snapshot=config['execution']['snapshot'],installation=config['execution']['installation'],snapshot_geometry_only=True)
        self.model=model;self.calibration=calibration
        self.manager=TaskManager(backend,service,vacuum,planner,model,calibration,config,lambda k,v:self.jobs.put((k,v)))
        backend.manager=self.manager
        self.robot_host.set(config['execution']['host']);self.robot_port.set(str(config['execution']['port']))
        self.serial_port.set(config['serial_port'])
        root.title('ECO65-B · 视觉单箱抓取并返回释放 · 实机模式')
        labels=[w for w in root.winfo_children() if isinstance(w,ttk.Label)]
        labels[0].configure(text='ECO65-B / 视觉单箱抓取并返回释放')
        labels[1].configure(text='手动连接；准备和预览不运动。吸具 1.5 kg + 箱子 2 kg；统一阀门、被动止回吸盘。软件停止不是硬件急停。')
        self.calibration_text.set('现场确认：link_1 相机；world=base。加载参考原文件，历史标定状态原样保留。')
        tab=ttk.Frame(self.tabs,padding=10);self.tabs.add(tab,text='视觉单箱任务');self.tabs.select(tab)
        self.task_status=tk.StringVar(value='待机 · 设备未连接 · 实机执行 NOT_RUN')
        ttk.Label(tab,textvariable=self.task_status,wraplength=1300,foreground='#66dce6').pack(fill='x',pady=8)
        row=ttk.Frame(tab);row.pack(fill='x')
        self.button(row,'连接机械臂（不运动）',lambda:backend.command('robot','connect',self.robot_host.get(),int(self.robot_port.get())))
        self.button(row,'连接真空板（不写阀门）',lambda:backend.command('vacuum','connect',self.serial_port.get()))
        self.button(row,'刷新控制器负载（只读）',lambda:self.job('payload',service.check_payload))
        row=ttk.Frame(tab);row.pack(fill='x',pady=10)
        self.observe_button=self.button(row,'准备观测 / 锁定起始点',lambda:self.job('observe_done',self.manager.observe))
        ttk.Label(row,text='目标检测编号（空=自动选择）').pack(side='left',padx=5)
        self.target=tk.StringVar();ttk.Entry(row,textvariable=self.target,width=5).pack(side='left')
        self.plan_button=self.button(row,'规划所选箱',self.plan_selected);self.plan_button.configure(state='disabled')
        self.preview_button=self.button(row,'查看完整轨迹预览',self.preview)
        self.execute_button=self.button(row,'执行单箱',self.execute_cycle);self.execute_button.configure(state='disabled')
        self.button(row,'停止（不自动释放）',self.stop_cycle)
        self.details=tk.StringVar(value='相机在“观测与检测”页手动连接并开始采集。\n箱型不限；根据可见面密封条件选箱，优先从正面与侧面自动测量尺寸。\n'+('\n'.join(issues(config)) or '现场配置已加载。'))
        ttk.Label(tab,textvariable=self.details,wraplength=1300).pack(fill='x',pady=8)
        self.bound_details=tk.StringVar(value='本次起始点、目标和释放位置：尚未锁定')
        ttk.Label(tab,textvariable=self.bound_details,wraplength=1300).pack(fill='x',pady=5)
        self.health=tk.StringVar();ttk.Label(tab,textvariable=self.health,wraplength=1300).pack(fill='x')
        self.preview_label=tk.StringVar(value='轨迹预览与实测执行分别显示；预览不是机器人反馈')
        ttk.Label(tab,textvariable=self.preview_label).pack(side='bottom')
        visual=ttk.Frame(tab);visual.pack(fill='both',expand=True,pady=8)
        self.frozen_image=ttk.Label(visual,text='本次冻结观测、长方体与选中面');self.frozen_image.pack(side='left',padx=6)
        self.canvas=tk.Canvas(visual,width=650,height=260,bg='#142333',highlightthickness=0);self.canvas.pack(side='right',fill='both',expand=True)
        root.after(100,self.update_task)
    def job(self,kind,fn):
        if self.manager.active:raise ValueError('已有活动任务或操作，请勿重复启动')
        self.previewed=False;self.execute_button.configure(state='disabled')
        # UI reservation happens before thread startup to reject double-clicks.
        if getattr(self,'job_pending',False):raise ValueError('操作尚未完成')
        self.job_pending=True
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
        self.canvas.delete('all');offset=0.;times=[];joints=[];marks=[]
        for s in plan['segments']:
            p=s['package']['prepared'];times.extend(offset+x['t'] for x in p['points']);joints.extend(x['q'] for x in p['points']);marks.append((offset,s['stage']));offset+=p['send_duration_s']
        t=np.asarray(times);q=np.rad2deg(joints);w=max(self.canvas.winfo_width(),600);h=max(self.canvas.winfo_height(),250);lo=q.min()-1;hi=q.max()+1
        for j,color in enumerate(('#ffbf69','#66dce6','#9be564','#df80ff','#ff7080','#ffffff')):
            xy=np.c_[20+(w-40)*t/t[-1],h-25-(h-60)*(q[:,j]-lo)/(hi-lo)][::max(1,len(t)//900)]
            self.canvas.create_line(*xy.ravel(),fill=color,width=2)
        for t0,name in marks:self.canvas.create_text(30+(w-80)*t0/offset,15,text=name,fill='#fff',anchor='nw')
        self.preview_label.set(f'规划预览：4 段运动总时长 {offset:.2f} s；真空等待另计。低速时间缩放 ×{self.config_single["time_scale"]:g}')
        self.previewed=True;self.execute_button.configure(state='normal');self.draw_observation()
    def execute_cycle(self):
        if not self.previewed:raise ValueError('请先查看本次完整轨迹预览')
        self.job('execute_done',self.manager.execute)
    def stop_cycle(self):
        self.manager.stop();self.previewed=False;self.execute_button.configure(state='disabled')
        self.details.set('已请求软件停止。保持阀门状态，继续核对实际停止反馈；不会自动返回或释放。')
    def draw_observation(self):
        m=self.manager
        if m.observation is None or m.result is None:return
        f=m.observation.frame;pixels=f.rgb.copy()
        for i,mask in enumerate(m.result.masks):
            pixels[mask]=(pixels[mask]*.65+np.array([30,180,250])*.35).astype('uint8')
            b=m.result.detections[i]['bbox_xyxy'];x,y=int(b[0]),int(b[1]);cv2.putText(pixels,str(i),(x,y),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,220,50),2)
        camera_world=np.linalg.inv(np.asarray(m.observation.world_camera))
        for box in m.boxes:
            p=transform(camera_world,box['corners_m'])
            if np.any(p[:,2]<=0):continue
            uv=project_camera(p,f.intrinsics).astype(int)
            selected=m.plan is not None and box['id']==m.plan['target']['id'];color=(255,200,20) if selected else (60,255,120)
            for a in range(8):
                for b in range(a+1,8):
                    if (a^b) in (1,2,4):cv2.line(pixels,tuple(uv[a]),tuple(uv[b]),color,2)
            if selected:
                face=m.plan['candidate']['face'];c=np.asarray(face['center_m']);p=transform(camera_world,[c,c+.04*np.asarray(face['normal'])]);uv=project_camera(p,f.intrinsics).astype(int)
                cv2.arrowedLine(pixels,tuple(uv[0]),tuple(uv[1]),(255,70,60),3)
        photo=ImageTk.PhotoImage(Image.fromarray(pixels).resize((520,390)));self._photos['frozen']=photo;self.frozen_image.configure(image=photo,text='')
    def update_task(self):
        if self._closed:return
        while True:
            try:kind,value=self.jobs.get_nowait()
            except Empty:break
            if kind=='job_finished':self.job_pending=False
            elif kind=='progress':self.details.set(value)
            elif kind=='error':self.details.set(value);self.backend.log(value)
            elif kind=='observation':
                obs,result,boxes=value;self.original_detection_frame=obs.frame;self.backend.detections=result
                from ..contracts import JointSample,Stamp
                from dataclasses import replace
                stamp=replace(Stamp.receive(),received_monotonic_ns=int(obs.robot_sample['received_s']*1e9))
                self.original_detection_joint=JointSample(str(obs.robot_sample['sequence']),obs.q_capture,stamp,obs.robot_sample,'CAPTURE_ASSOCIATED')
                self.details.set('本次 q_start（度）：'+' '.join(f'{x:.3f}' for x in np.rad2deg(obs.q_start))+f'\n恢复 {len(boxes)} 个箱体；选择检测编号或留空自动选择。')
                if boxes:self.details.set(self.details.get()+'\n实测尺寸（mm）：'+'；'.join(str(b['detection_index'])+' → '+str((np.asarray(b['size_m'])*1000).round(1).tolist()) for b in boxes))
                self.bound_details.set('锁定 q_start（°）：'+str(np.rad2deg(obs.q_start).round(3).tolist())+
                    '\n起始 TCP（m）：'+str(np.asarray(obs.world_tcp_start)[:3,3].round(5).tolist()))
                self.draw_observation()
            elif kind=='plan':
                self.details.set('目标：'+value['target']['id']+'\n接触 TCP（m）：'+str(np.array(value['candidate']['tcp_pose'])[:3,3].round(5).tolist())+'\n返回同一组 q_start；释放包络已检查。承接资格按用户要求暂不核验。')
                self.bound_details.set('锁定 q_start（°）：'+str(np.rad2deg(value['q_start']).round(3).tolist())+
                    '\n目标 '+value['target']['id']+' · 接触 TCP（m） '+str(np.asarray(value['candidate']['tcp_pose'])[:3,3].round(5).tolist())+
                    '\n释放箱心（m） '+str(np.asarray(value['release']['pose'])[:3,3].round(5).tolist())+
                    f" · 预计密封 {len(value['candidate']['active_ids'])}/12 杯 · 总阀控制、其余杯被动关闭"+
                    '\n成功条件：实测到位 → 压力稳定吸牢 → 脱离并返回原关节 → 压力稳定释放')
                self.draw_observation()
            elif kind=='payload':self.details.set(json.dumps(value,ensure_ascii=False))
            elif kind=='feedback':self.last_status=value;self.feedback_pending=False
            elif kind=='result':self.details.set(json.dumps(value,ensure_ascii=False));self.execute_button.configure(state='disabled')
        m=self.manager;state=m.cycle.state if m.cycle else m.phase
        self.plan_button.configure(state='normal' if m.phase=='TARGET_READY' and not m.active and not getattr(self,'job_pending',False) else 'disabled')
        self.task_status.set(f'实机单箱模式 · {LABELS.get(state,state)} · task_id {m.task_id or "—"}\n原因：{m.reason or (m.cycle.reason if m.cycle else "—")}')
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
        self.health.set(f"机器人连接 {self.backend.workers['robot'].device.connected} · 实测 ° {None if q is None else np.round(np.rad2deg(q),3).tolist()}\n指令 ° {None if command is None else np.round(np.rad2deg(command),3).tolist()} · 本段目标 ° {None if goal is None else np.round(np.rad2deg(goal),3).tolist()}\n跟踪误差 ° {None if error is None else np.round(np.rad2deg(error),3).tolist()} · 反馈年龄 {age} s · 吸具 {pressure}\n控制器报警：{alarm}")
        if self.service.process is not None and not self.feedback_pending and not m.active:
            self.feedback_pending=True
            def read():self.jobs.put(('feedback',self.service.feedback()))
            threading.Thread(target=read,daemon=True).start()
        if self.closing_single:
            try:self.manager.close()
            except Exception as e:self.details.set(str(e));self.closing_single=False
            else:super().close();return
        self.root.after(100,self.update_task)
    def choose_tool(self):raise ValueError('实机工具由本次配置绑定；更改后重启并重新准备任务')
    def change_tool(self):raise ValueError('实机模式不能切换工具参考')
    def choose_calibration(self):raise ValueError('实机外参由现场配置绑定；更改配置后重新启动')
    def choose_config(self):raise ValueError('本入口使用 --config 指定的单箱配置；更改后重新启动')
    def load_plan(self):raise ValueError('实机单箱入口只接受本次冻结观测生成的完整计划；预览 JSON 不能导入执行')
    def start_detection(self):raise ValueError('单箱任务请使用“准备观测”，检测输入会与采集关节一同冻结')
    def close(self):
        if hasattr(self,'manager'):self.manager.stop();self.closing_single=True
        else:super().close()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'configs/workbench/single_carton.example.json');p.add_argument('--ui-smoke',action='store_true');p.add_argument('--check-config',action='store_true');args=p.parse_args()
    config=load(args.config)
    if args.check_config:
        print(json.dumps(dict(issues=issues(config),hardware_execution='NOT_RUN'),ensure_ascii=False,indent=2));return
    installation=Installation.load(config['execution']['installation']);model=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
    calibration=Extrinsics.load(config['camera_extrinsics']) if config.get('camera_extrinsics') else None
    if calibration:installation.check_camera(calibration)
    mapping=load_mapping(config['execution']['mapping']);authority=Authority('eco65_single_carton_segment_v1');planner=SinglePlanner(config,mapping,authority,model)
    service=ExecutionService(mapping,config['execution'],authority,planner.binding);vacuum=VacuumOwner(config['vacuum'])
    backend=SingleBackend(D435iObserver(),ManagedRobotView(service,mapping),VacuumView(vacuum),service)
    root=tk.Tk();ui=SingleWorkbench(root,backend,planner,service,vacuum,config,model,calibration)
    if args.ui_smoke:root.after(700,ui.close)
    root.mainloop()

if __name__=='__main__':main()
