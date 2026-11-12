"""Tk pixel geometry and result-window lifecycle, with fake devices only."""
import gc
import time
import tkinter as tk
from types import SimpleNamespace
from pathlib import Path
import pytest
from unloading_sim.workbench.single_carton.ui import SingleWorkbench
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.live import SingleBackend,VisionPacket,overlay
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation,CoordinateModel,Extrinsics
from unloading_sim.workbench.ui import URDF,URDF_SHA
from unloading_sim.workbench.stable_ui import StableStringVar,append_log
from test_single_carton_live import Camera,Device,Model,exposure


@pytest.fixture
def ui():
    c=load(Path('configs/workbench/single_carton.example.json'))
    install=Installation.load(c['execution']['installation'])
    model=CoordinateModel(URDF,URDF_SHA,install.world_from_base,install.tool)
    service=SimpleNamespace(process=None,request_stop=lambda:None,close=lambda:None,check_payload=lambda:{})
    vacuum=SimpleNamespace(holding=False,inhibit=lambda:None,pressure=lambda:dict(kpa=-65.,received_s=time.monotonic()))
    planner=SimpleNamespace(authority=Authority('eco65_single_carton_segment_v1'),close=lambda:None)
    backend=SingleBackend(Camera(),Device(True),Device(True),service)
    root=tk.Tk();errors=[];root.report_callback_exception=lambda *e:errors.append(str(e))
    workbench=SingleWorkbench(root,backend,planner,service,vacuum,c,model,Extrinsics.load(c['camera_extrinsics']),auto_start=False)
    yield workbench,errors
    workbench._closed=True;workbench.manager.close();backend.close()
    for ident in root.tk.call('after','info'):root.after_cancel(ident)
    root.destroy()
    del workbench,backend,root
    gc.collect()


def geometry(widget):
    return (widget.winfo_x(),widget.winfo_y(),widget.winfo_width(),widget.winfo_height())


def widgets(parent):
    result=[parent]
    for child in parent.winfo_children():result.extend(widgets(child))
    return result


def test_status_numeric_paths_and_long_errors_never_shift_widgets(ui):
    w,errors=ui;root=w.root
    for tab in w.tabs.tabs():
        w.tabs.select(tab);root.update()
        fixed={str(x):geometry(x) for x in widgets(root)}
        for text in ['短','-123.456 / +0.000',('中文错误 /dev/'+'long-path/'*50+'\n')*15]:
            for var in (w.details,w.bound_details,w.task_status,w.health,w.device_details,w.device_summary,
                        w.notice,w.calibration_text,w.pairing_text,w.joints_text,w.pressure_text,w.metadata,w.detection_status):
                var.set(text)
            root.update_idletasks()
            assert fixed=={str(x):geometry(x) for x in widgets(root)}
    assert not errors


def test_closing_result_window_hides_only_and_uses_original_image(ui):
    w,errors=ui;root=w.root;window=w.result_windows['detection'];root.update()
    f=exposure(7);result=Model().detect_boxes(f);packet=VisionPacket(0,f,None,result,overlay(f,result,False))
    window.update(packet,'检测完成','');root.update()
    image_id=window.item;before=geometry(window.canvas)
    window.update(packet,'错误'*1000,'路径'*1000);root.update_idletasks()
    assert geometry(window.canvas)==before and window.item==image_id
    root.tk.call(window.window.protocol('WM_DELETE_WINDOW'))
    assert window.window.state()=='withdrawn' and not w.live_vision.closed
    window.open();root.update();assert window.window.state()=='normal'
    assert window.packet.frame.rgb_number==7 and window.item==image_id and not errors


def test_unchanged_text_is_not_rewritten_and_log_preserves_scroll(ui):
    w,_=ui;root=w.root;var=StableStringVar(value='same');writes=[]
    var.trace_add('write',lambda *_:writes.append(1));var.set('same');assert writes==[]
    var.set('new');var.set('new');assert writes==[1]
    rows=tuple(f'line {i}' for i in range(150));previous=append_log(w.logbox,rows,())
    root.update();w.logbox.yview_moveto(.3);before=w.logbox.index('@0,0')
    append_log(w.logbox,rows+('new line',),previous);root.update_idletasks()
    assert w.logbox.index('@0,0')==before


def test_ui_autostart_is_nonblocking_and_failure_is_visible(ui):
    w,errors=ui;w.live_vision.factory=Model
    start=time.monotonic();w.autostart();assert time.monotonic()-start<.1
    end=time.monotonic()+2
    while time.monotonic()<end:
        w.root.update();time.sleep(.005)
        if w.live_vision.completed>1 and '连接失败' in w.device_summary.get():break
    assert '连接失败' in w.device_summary.get() and 'endpoint unreachable' in w.device_details.get()
    assert w.live_vision.completed>0 and w.manager.task_id is None and not errors
    assert w.backend.workers['camera'].device.calls[1][1].fps==30


def test_session_change_disables_old_plan_and_labels_it_invalid(ui):
    w,errors=ui
    w.manager.phase='PLAN_READY';w.manager.plan=object()
    w.manager.observation=SimpleNamespace(generation=w.backend.observation_generation-1)
    w.previewed=True;w.execute_button.configure(state='normal')
    w.update_task();w.root.update_idletasks()
    assert not w.previewed and str(w.execute_button.cget('state'))=='disabled'
    assert '观测会话已失效' in w.task_status.get()
    with pytest.raises(ValueError,match='观测会话已失效'):w.preview()
    assert not errors
