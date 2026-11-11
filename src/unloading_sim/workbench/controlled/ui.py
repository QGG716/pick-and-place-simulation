"""Explicit operator entry for one controlled empty trajectory. Never auto-connect."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
from queue import Queue,Empty
import threading
import time
import tkinter as tk
from tkinter import ttk
import numpy as np
from ..ui import Workbench,load_mapping
from ..backend import WorkbenchBackend,DeviceRequestRejected
from ..contracts import JointSample,Stamp
from ..devices import D435iObserver,VacuumObserver
from .planner import ROOT,EmptyPlanner,load_config
from .trajectory import Authority,expected_controller_payload
from .service import ExecutionService,payload_error
from .transport import payload_matches
from .engine import TERMINAL


def live_target_sample(sample, now=None, max_age_s=.1):
    """Validate the sample at read time; never silently reuse an old target."""
    if sample is None:
        raise RuntimeError('没有机械臂实时数据，请先点击“连接受控会话（不运动）”。')
    now=time.monotonic() if now is None else now
    received=sample.get('received_s')
    q=np.asarray(sample.get('q',[]),float)
    if (sample.get('source')!='LIVE' or q.shape!=(6,) or not np.isfinite(q).all()
            or not isinstance(received,(float,int)) or not np.isfinite(received)):
        raise RuntimeError('机械臂实测反馈无效，未填入目标。')
    age=now-received
    if not 0<=age<=max_age_s:
        raise RuntimeError(f'机械臂反馈已过期（{age*1000:.1f} ms），未填入目标。请检查连接；必要时先断开，再手动连接。')
    return dict(target_degrees=' '.join(f'{x:.4f}' for x in np.rad2deg(q)),
                feedback_age_s=age,received_s=received)


class ManagedRobotView:
    """Observer facade over the SAME owner; no second SDK or state polling."""
    def __init__(self,service,mapping):
        self.service=service;self.mapping=mapping;self.connected=False
        self.endpoint=None;self.capabilities=None;self.last_connection_error=''
    def connect(self,host,port):
        endpoint=(str(host),int(port));self.last_connection_error=''
        if self.connected:
            if endpoint!=self.endpoint:
                self.last_connection_error=f'仍连接 {self.endpoint[0]}:{self.endpoint[1]}；切换地址前请先断开。'
                raise DeviceRequestRejected(self.last_connection_error)
            # DeviceWorker used to interpret ALREADY_CONNECTED as a transport
            # failure and disconnect the working session on a repeated click.
            return dict(self.capabilities,already_connected=True)
        try:
            result=self.service.connect(*endpoint)
        except Exception as exc:
            self.last_connection_error=str(exc)
            raise
        self.connected=True;self.endpoint=endpoint;self.capabilities=result
        return result
    def disconnect(self):
        if self.connected:
            if self.service.process is not None:self.service.disconnect()
            self.connected=False;self.endpoint=None
    def read(self):
        s=self.service.snapshot()
        if s is None:raise RuntimeError('NO_LIVE_FEEDBACK')
        stamp=replace(Stamp.receive(),received_monotonic_ns=int(s['received_s']*1e9),received_utc_ns=s.get('received_utc_ns',0))
        return JointSample(str(s['sequence']),tuple(s['q']),stamp,s,self.mapping.mapping_id)

class ControlledBackend(WorkbenchBackend):
    def __init__(self,camera,robot,vacuum,service):
        super().__init__(camera,robot,vacuum);self.execution=service
        self.logs.clear();self.log('CONTROLLED_EMPTY · 手动连接、规划、预览、执行；未发出运动')
    def load_replay(self,directory):raise RuntimeError('回放请使用默认只读入口；受控模式只接受实测关节')

class ControlledWorkbench(Workbench):
    def __init__(self,root,backend,planner,config):
        self.planner,self.control_config=planner,config
        self.service=backend.execution;self.package=None;self.task_id=None
        self.jobs=Queue();self.busy=False;self.closing=False;self.feedback_busy=False
        self.live_status={'state':'DISCONNECTED'};self.previewed=False;self.goal_generation=0
        self.waiting_connection=False
        self.planning_progress=None
        super().__init__(root,backend,snapshot=config['snapshot'],installation=config['installation'])
        root.title('ECO65-B · 单条低速空载轨迹 · 受控执行')
        labels=[w for w in root.winfo_children() if isinstance(w,ttk.Label)]
        labels[0].configure(text='ECO65-B / 单条低速空载执行')
        labels[1].configure(text=f"现场设备与坐标标定按用户已确认配置；吸具模型 {config.get('tool_mass_kg',1.5):g} kg，控制器设定 {expected_controller_payload(config):g} kg。软件停止不是硬件急停。")
        self.calibration_text.set('沿用现场已确认关系：相机在 link_1，world=base；本任务为关节目标，不调用训练相机外参。')
        self.robot_host.set(config['host']);self.robot_port.set(str(config['port']))
        tab=ttk.Frame(self.tabs,padding=12);self.tabs.add(tab,text='受控执行');self.tabs.select(tab)
        self.execution_status=tk.StringVar(value='受控模式 · 未连接 · 无活动任务')
        ttk.Label(tab,textvariable=self.execution_status,wraplength=1250).pack(fill='x',pady=8)
        connection=ttk.Frame(tab);connection.pack(fill='x',pady=(0,8))
        self.button(connection,'连接受控会话（不运动）',self.connect_robot)
        self.button(connection,'断开（活动任务先停止）',lambda:self.backend.command('robot','disconnect'))
        self.payload_button=self.button(connection,'刷新工具负载（只读）',self.refresh_payload)
        self.payload_label=ttk.Label(tab,text='工具负载：等待连接后读取',wraplength=1250)
        self.payload_label.pack(fill='x',pady=(0,8))
        row=ttk.Frame(tab);row.pack(fill='x')
        self.goal=tk.StringVar();self.scale=tk.StringVar(value=str(config.get('time_scale',10.)))
        ttk.Label(row,text='目标 J1—J6（度，空格分隔）').pack(side='left')
        ttk.Entry(row,textvariable=self.goal,width=64).pack(side='left',padx=8)
        self.fill_button=self.button(row,'填入当前实测',self.fill_current)
        row=ttk.Frame(tab);row.pack(fill='x',pady=10)
        ttk.Label(row,text='时间放大倍数（≥1，默认 10）').pack(side='left')
        ttk.Entry(row,textvariable=self.scale,width=8).pack(side='left',padx=8)
        self.plan_button=self.button(row,'从实测起点规划',self.plan_empty)
        self.preview_button=self.button(row,'查看轨迹预览',self.preview)
        self.preview_button.configure(state='disabled')
        self.execute_button=self.button(row,'执行这一条',self.execute)
        self.execute_button.configure(state='disabled')
        self.button(row,'软件停止',self.stop_task)
        self.summary=tk.StringVar(value='选择邻近目标：每关节相对起点 ≤10°。当前没有可执行轨迹。')
        ttk.Label(tab,textvariable=self.summary,wraplength=1250).pack(fill='x',pady=6)
        self.actual_text=tk.StringVar(value='实测 / 指令 / 跟踪误差 / 反馈年龄：等待实测')
        ttk.Label(tab,textvariable=self.actual_text,font=('DejaVu Sans Mono',10),wraplength=1250).pack(fill='x')
        self.canvas=tk.Canvas(tab,height=260,bg='#0b1420',highlightthickness=0);self.canvas.pack(fill='both',expand=True,pady=8)
        self.preview_time=tk.DoubleVar(value=0.)
        ttk.Label(tab,text='规划预览时间（独立于实机进度）').pack(anchor='w')
        self.preview_slider=ttk.Scale(tab,from_=0,to=1,variable=self.preview_time,command=self.preview_point);self.preview_slider.pack(fill='x')
        self.preview_label=tk.StringVar(value='尚未生成轨迹')
        ttk.Label(tab,textvariable=self.preview_label).pack(fill='x')
        self.button(tab,'查看该预览姿态的完整三维场景',self.render_preview)
        for variable in (self.goal,self.scale):variable.trace_add('write',self.invalidate)
        root.after(100,self.update_execution)
    def _robot_controls(self,box):
        ttk.Entry(box,textvariable=self.robot_host,width=16).pack(side='left',padx=4)
        ttk.Entry(box,textvariable=self.robot_port,width=6).pack(side='left',padx=4)
        self.button(box,'连接受控会话（不运动）',self.connect_robot)
        self.button(box,'断开（活动任务先停止）',lambda:self.backend.command('robot','disconnect'))
        self.joints_text=tk.StringVar(value='未连接');ttk.Label(box,textvariable=self.joints_text,wraplength=800).pack(side='left')
    def safe(self,function,*args):
        try:return function(*args)
        except Exception as exc:
            message=f'{type(exc).__name__}: {exc}'
            self.backend.log(message)
            if hasattr(self,'notice'):self.notice.set(str(exc))
            if hasattr(self,'summary'):self.summary.set(str(exc))
    def connect_robot(self):
        self.backend.command('robot','connect',self.robot_host.get(),int(self.robot_port.get()))
        self.waiting_connection=True
        self.summary.set('正在连接机械臂并检查实时反馈…')
    def invalidate(self,*_):
        self.package=None;self.previewed=False;self.goal_generation+=1
        self.planning_progress=None
        self.planner.authority.invalidate()
        if hasattr(self,'execute_button'):self.execute_button.configure(state='disabled')
        if hasattr(self,'preview_button'):self.preview_button.configure(state='disabled')
    def job(self,kind,function):
        if self.busy:raise RuntimeError('当前操作尚未完成')
        self.busy=True
        def run():
            try:self.jobs.put((kind,function(),None))
            except Exception as e:self.jobs.put((kind,None,str(e)))
        threading.Thread(target=run,daemon=True).start()
    def fill_current(self):
        def read():
            try:sample=self.service.snapshot()
            except RuntimeError as exc:
                if str(exc)=='DISCONNECTED':raise RuntimeError('机械臂未连接，请先点击“连接受控会话（不运动）”。') from exc
                raise
            return live_target_sample(sample)
        self.job('fill',read)
        self.summary.set('正在读取当前实测关节…')
    def plan_empty(self):
        if self.busy:raise RuntimeError('当前操作尚未完成')
        if not self.live_status.get('connected') or self.live_status['state'] not in {'READY','SUCCEEDED','CANCELLED','FAILED'}:raise RuntimeError('请先连接，并结束当前任务')
        q=np.deg2rad([float(v) for v in self.goal.get().split()]);scale=float(self.scale.get());self.invalidate()
        generation=self.goal_generation
        self.summary.set('正在读取实测起点、生成原生 MoveIt 轨迹并检查最终发送路径…')
        self.job('plan',lambda:(generation,self.planner.plan(self.service.snapshot(),q,scale,
            progress=lambda message:self.jobs.put(('plan_progress',(generation,message),None)))))
    def controller_payload(self):
        caps=self.live_status.get('capabilities',{})
        if 'payload_check' in caps:return caps['payload_check']
        tool=caps.get('submit_tool_frame',caps.get('rm_get_current_tool_frame',{}).get('value',{}))
        mass=expected_controller_payload(self.control_config)
        return dict(tool=tool,matches=payload_matches(tool,mass),required_kg=mass,sdk_code=0)
    def refresh_payload(self):
        if self.busy:raise RuntimeError('当前操作尚未完成')
        if not self.live_status.get('connected'):raise RuntimeError('请先连接机械臂')
        self.invalidate()
        self.job('payload',self.service.check_payload)
        self.summary.set('正在只读核对控制器当前工具负载…')
    def preview(self):
        if self.package is None:raise RuntimeError('请先完成规划')
        self.previewed=True;self.draw_plan();self.preview_point()
        self.execute_button.configure(state='normal' if self.controller_payload()['matches'] else 'disabled')
    def draw_plan(self):
        pts=self.package['prepared']['points'];t=np.array([p['t'] for p in pts]);q=np.rad2deg([p['q'] for p in pts]);self.canvas.delete('all')
        width=max(900,self.canvas.winfo_width());height=max(230,self.canvas.winfo_height());lo=q.min()-1;hi=q.max()+1
        colors=['#ffbf69','#66dce6','#9be564','#df80ff','#ff7080','#ffffff']
        stride=max(1,len(t)//1200)
        for j in range(6):
            points=np.column_stack((30+(width-60)*t/t[-1],height-30-(height-60)*(q[:,j]-lo)/(hi-lo)))[::stride]
            self.canvas.create_line(*points.ravel(),fill=colors[j],width=2)
            self.canvas.create_text(65+j*125,15,text=f'J{j+1} / °',fill=colors[j])
        self.preview_slider.configure(to=t[-1]);self.preview_time.set(0)
    def preview_point(self,*_):
        if self.package is None:return
        p=self.package['prepared'];index=min(len(p['points'])-1,round(self.preview_time.get()/p['period_s']));point=p['points'][index]
        self.preview_label.set(f"预览 {point['t']:.3f} / {p['send_duration_s']:.3f} s · J1—J6 °: "+' '.join(f'{v:.3f}' for v in np.rad2deg(point['q'])))
    def execute(self):
        if self.package is None or not self.previewed:raise RuntimeError('当前轨迹未预览或已失效，请重新规划')
        package=self.package;self.execute_button.configure(state='disabled');self.package=None;self.previewed=False
        self.job('submit',lambda:self.service.submit(package))
    def stop_task(self):
        self.service.request_stop();self.summary.set('已请求软件停止；等待真实反馈确认。')
    def render_preview(self):
        if self.package is None:raise RuntimeError('无当前预览轨迹')
        p=self.package['prepared'];i=min(len(p['points'])-1,round(self.preview_time.get()/p['period_s']))
        self._render_checked(p['points'][i]['q'],'PLANNED_PREVIEW / NOT EXECUTION')
    def open_scene(self):
        if self.pose_source.get()=='PLANNED_PREVIEW':return self.render_preview()
        if self.backend.joint is None:raise RuntimeError('无当前实测姿态')
        self._render_checked(self.backend.joint.q_rad,'LIVE JOINT OBSERVATION')
    def _render_checked(self,q,label):
        generation=(self.backend.observation_generation,self.scene_generation)
        def render():
            renderer=None
            try:
                self.planner.build();self.planner.binding()
                from ..scene import SceneRenderer,display_scene
                from unloading_sim.eco65.native_review_world import NativeSceneWorld
                renderer=SceneRenderer.__new__(SceneRenderer)
                renderer.world=NativeSceneWorld(self.planner.source['tool'],display_scene(self.planner.source['scene']))
                self.results.put(('scene',(generation,renderer.render(q),label)))
            except Exception as e:self.results.put(('scene_error',(generation,str(e))))
            finally:
                if renderer:renderer.close()
        threading.Thread(target=render,daemon=True).start()
    def load_plan(self):
        self.invalidate();super().load_plan()
    def choose_tool(self):raise RuntimeError('本模式工具由已确认配置绑定；修改配置后重新启动和规划')
    def update_execution(self):
        if self._closed:return
        while True:
            try:kind,value,error=self.jobs.get_nowait()
            except Empty:break
            if kind=='plan_progress':
                generation,message=value
                if generation==self.goal_generation:self.planning_progress=(message,time.monotonic())
                continue
            if kind=='feedback':self.feedback_busy=False
            else:self.busy=False
            if kind=='plan':self.planning_progress=None
            if error:
                label={'fill':'读取实测未完成','plan':'规划未完成','submit':'执行提交未完成','feedback':'实时反馈读取失败','payload':'负载核对未完成','close':'退出未完成'}.get(kind,kind)
                message=f'{label}：{error}'
                self.summary.set(message);self.backend.log(message)
                if kind in ('submit','plan'):self.invalidate()
                if kind=='close':self.closing=False
                continue
            if kind=='fill':
                self.goal.set(value['target_degrees'])
                self.summary.set(f"已填入当前实测 J1—J6（度）；读取时反馈年龄 {value['feedback_age_s']*1000:.1f} ms。此时目标等于当前位置，请修改一个邻近关节目标后再规划。")
            elif kind=='plan':
                generation,package=value
                if generation==self.goal_generation:
                    self.package=package;p=package['prepared']
                    self.preview_button.configure(state='normal')
                    self.summary.set(f"规划成功 · 原时长 {p['source_duration_s']:.3f} s · ×{p['time_scale']:g} 后 {p['send_duration_s']:.3f} s · 最终发送路径碰撞检查 PASS · 请预览")
            elif kind=='submit':self.task_id=value;self.summary.set('任务已提交；正在监测实际运动。')
            elif kind=='feedback':self.live_status=value
            elif kind=='payload':
                self.live_status.setdefault('capabilities',{})['payload_check']=value
                self.summary.set('控制器负载核对通过；请重新规划并预览。' if value['matches'] else payload_error(value))
            elif kind=='close':
                super().close();return
        robot_worker=self.backend.workers['robot']
        if self.waiting_connection and not robot_worker.busy:
            self.waiting_connection=False
            robot=robot_worker.device
            if robot.last_connection_error:self.summary.set('连接请求未完成：'+robot.last_connection_error)
            elif robot.connected:self.summary.set('机械臂已连接，可以点击“填入当前实测”。重复连接不会断开现有会话。')
        s=self.live_status
        payload_check=self.controller_payload();tool=payload_check['tool']
        if not isinstance(tool,dict):tool={}
        payload=tool.get('payload','—')
        required=expected_controller_payload(self.control_config)
        if not s.get('connected'):
            self.payload_label.configure(text='工具负载：等待连接后读取')
        elif payload_check['matches']:
            self.payload_label.configure(text=f"最近读取工具 {tool.get('name','—')} · 控制器负载 {payload} kg（本次要求 {required:g} kg），核对通过；提交前仍会重新读取。")
        else:
            self.payload_label.configure(text=f"执行未就绪：最近读取工具 {tool.get('name','—')} 负载 {payload} kg，需要 {required:g} kg。请在控制器选择/设置本次确认的负载，再刷新工具负载。仍可规划和预览。")
            self.execute_button.configure(state='disabled')
        if self.planning_progress:
            message,started=self.planning_progress
            self.summary.set(f'{message} · 本阶段已用 {time.monotonic()-started:.1f} s（尚未运动）')
        self.execution_status.set(f"受控模式 · {'已连接' if s.get('connected') else '未连接'} · task_id={s.get('task_id') or '—'} · {s['state']} · {s.get('reason','')} · 最近读取工具负载 {payload} kg（需要 {required:g} kg）")
        def fmt(x):return '—' if x is None else ' '.join(f'{v:.3f}' for v in np.rad2deg(x))
        age=s.get('feedback_age_s')
        self.actual_text.set(f"实测 °: {fmt(s.get('actual_q'))}\n指令 °: {fmt(s.get('command_q'))}\n跟踪误差 °: {fmt(s.get('tracking_error_rad'))}\n距目标误差 °: {fmt(s.get('goal_error_rad'))}\n反馈年龄: {'—' if age is None else f'{age*1000:.1f} ms'} · 发送 {s.get('sent_points',0)}/{s.get('total_points',0)} · 到位确认: {s.get('arrival_confirmed',False)}")
        if self.service.process is not None and not self.feedback_busy and not self.closing:
            self.feedback_busy=True
            def feedback():
                try:self.jobs.put(('feedback',self.service.feedback(),None))
                except Exception as e:self.jobs.put(('feedback',None,str(e)))
            threading.Thread(target=feedback,daemon=True).start()
        if self.closing and not self.busy:
            if s['state']=='STOP_UNCONFIRMED':
                self.closing=False;self.summary.set('停止未确认：保留连接。请现场使用控制器/硬件停止，禁止自动续跑。')
            elif s['state'] in TERMINAL|{'READY','DISCONNECTED'}:
                self.job('close',self.service.close)
            elif not self.feedback_busy:
                # Closing needs feedback until actual stop confirmation.
                self.feedback_busy=True
                def closing_feedback():
                    try:self.jobs.put(('feedback',self.service.feedback(),None))
                    except Exception as e:self.jobs.put(('feedback',None,str(e)))
                threading.Thread(target=closing_feedback,daemon=True).start()
        self.root.after(100,self.update_execution)
    def close(self):
        if self.closing:return
        self.closing=True;self.stop_task()
        if self.service.process is None:super().close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/workbench/controlled.example.json')
    parser.add_argument('--ui-smoke',action='store_true',help='Open and close disconnected UI; no SDK connection')
    args=parser.parse_args();config=load_config(args.config);mapping=load_mapping(config['mapping']);authority=Authority()
    planner=EmptyPlanner(config,mapping,authority);service=ExecutionService(mapping,config,authority,planner.binding)
    backend=ControlledBackend(D435iObserver(),ManagedRobotView(service,mapping),VacuumObserver(),service)
    root=tk.Tk();ui=ControlledWorkbench(root,backend,planner,config)
    if args.ui_smoke:root.after(600,ui.close)
    root.mainloop()

if __name__=='__main__':main()
