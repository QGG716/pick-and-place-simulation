"""Field UI issues of 2026-11-26 (task 01fc3f5f): a confirmed reset never ran,
the continuous panel squeezed the task-tab images to 4 px, the device fold
button moved under the opened panel, and the observation tab's third pane
shows live detection + segmentation together. No hardware, no motion."""
import gc
import time
import tkinter as tk
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workbench.continuous.ui import ContinuousWorkbench
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.live import SingleBackend, combined_overlay, latest_live_packet
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel, Extrinsics
from unloading_sim.workbench.ui import URDF, URDF_SHA
from test_single_carton_live import Camera, Device
from eco65_test_inputs import use_committed_installation, use_synthetic_remote_detector


@pytest.fixture
def ui(tmp_path):
    c = use_synthetic_remote_detector(use_committed_installation(load(Path('configs/workbench/single_carton.example.json'))), tmp_path)
    c.update(start_joint_check='ENABLED', start_joint_tolerance_deg=0.02, start_joint_tolerance_basis='test',
             continuous_job=dict(max_cartons=3, receiver_clearing=dict(mode='USER_CONFIRMED_SELF_CLEARING', clearing_time_s=3, basis='test')))
    install = Installation.load(c['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, install.world_from_base, install.tool)
    service = SimpleNamespace(process=None, request_stop=lambda: None, close=lambda: None, check_payload=lambda: {})
    vacuum = SimpleNamespace(holding=False, inhibit=lambda: None, pressure=lambda: dict(kpa=-2., received_s=time.monotonic()))
    planner = SimpleNamespace(authority=Authority('eco65_single_carton_segment_v1'), close=lambda: None)
    backend = SingleBackend(Camera(), Device(True), Device(True), service)
    root = tk.Tk()
    root.geometry('1920x1043+0+0')
    workbench = ContinuousWorkbench(root, backend, planner, service, vacuum, c, model, Extrinsics.load(c['camera_extrinsics']), auto_start=False)
    yield workbench
    workbench._closed = True; workbench.manager.close(); backend.close()
    for ident in root.tk.call('after', 'info'): root.after_cancel(ident)
    root.destroy(); del workbench, backend, root; gc.collect()


def pump(root, seconds=.3):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        root.update(); time.sleep(.01)


def drain(workbench, until, timeout=5.):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not until():
        workbench.update_task(); workbench.root.update(); time.sleep(.02)
    return until()


def test_confirmed_reset_runs_after_its_planning_job(ui, monkeypatch):
    """Field 2026-11-26: OK in '确认复位回起点' raised '操作尚未完成' and nothing moved."""
    calls = []
    monkeypatch.setattr('tkinter.messagebox.askokcancel', lambda *a, **k: True)
    monkeypatch.setattr(ui.manager, 'reset_execute', lambda: calls.append('execute') or dict(state='RESET_AT_START', max_error_deg=.001))
    ui.job('reset_ready', lambda: dict(stages=['return_empty'], duration_s=9.7, max_joint_offset_deg=41.8))
    assert drain(ui, lambda: calls == ['execute'] and not ui.job_pending)
    assert '已复位到起点' in ui.details.get()
    assert '操作尚未完成' not in ui.notice.get()


def test_planning_job_marker_does_not_release_the_follow_up(ui, monkeypatch):
    """The reset execution keeps its reservation until its own job finishes."""
    import threading
    gate = threading.Event()
    monkeypatch.setattr('tkinter.messagebox.askokcancel', lambda *a, **k: True)
    monkeypatch.setattr(ui.manager, 'reset_execute', lambda: gate.wait(5) and dict(state='RESET_AT_START', max_error_deg=0.))
    ui.job('reset_ready', lambda: dict(stages=['return_empty'], duration_s=1., max_joint_offset_deg=1.))
    assert drain(ui, lambda: ui.job_label == '复位 / 移动执行')
    pump(ui.root, .2); ui.update_task()
    assert ui.job_pending, 'the planning job finished marker released the running reset'
    gate.set()
    assert drain(ui, lambda: not ui.job_pending)


def test_cancelled_reset_sends_nothing(ui, monkeypatch):
    calls = []
    monkeypatch.setattr('tkinter.messagebox.askokcancel', lambda *a, **k: False)
    monkeypatch.setattr(ui.manager, 'reset_execute', lambda: calls.append('execute'))
    ui.job('reset_ready', lambda: dict(stages=['return_empty'], duration_s=1., max_joint_offset_deg=1.))
    assert drain(ui, lambda: not ui.job_pending)
    assert calls == [] and '已取消复位' in ui.details.get()


def test_task_tab_keeps_image_and_preview_height_with_the_job_panel(ui):
    ui.tabs.select(ui.task_tab); pump(ui.root)
    assert ui.frozen_image.winfo_height() >= 200 and ui.canvas.winfo_height() >= 200


def test_device_fold_button_stays_put(ui):
    ui.tabs.select(ui.observe); pump(ui.root)
    y = ui.device_toggle.winfo_rooty()
    ui.device_toggle.invoke(); pump(ui.root)
    assert ui._device_panel_shown and ui.device_toggle.winfo_rooty() == y
    ui.device_toggle.invoke(); pump(ui.root)
    assert not ui._device_panel_shown and ui.device_toggle.winfo_rooty() == y


def _packet(stage_masks, number, generation=0):
    rgb = np.zeros((48, 64, 3), np.uint8)
    frame = SimpleNamespace(observation_id=f'obs{number}', rgb=rgb, rgb_number=number)
    detections = [dict(label='carton', score=.9, bbox_xyxy=[5, 5, 30, 30])]
    masks = None
    if stage_masks:
        m = np.zeros((48, 64), bool); m[10:20, 10:20] = True; masks = [m]
    result = SimpleNamespace(observation_id=f'obs{number}', detections=detections, masks=masks)
    return SimpleNamespace(generation=generation, frame=frame, result=result)


def test_combined_overlay_draws_boxes_and_masks():
    det = _packet(False, 1); seg = _packet(True, 1)
    box_only = combined_overlay(det.frame, det.result)
    both = combined_overlay(seg.frame, seg.result)
    assert box_only[5, 15].any() and not box_only[15, 15].any()  # box edge, no mask fill
    assert both[5, 15].any() and both[15, 15].any()              # box edge and mask fill


def test_latest_live_packet_prefers_newest_frame_then_segmentation():
    seg1, det1, det2 = _packet(True, 1), _packet(False, 1), _packet(False, 2)
    assert latest_live_packet(dict(segmentation=seg1, detection=det1), 0)[0] == 'segmentation'
    assert latest_live_packet(dict(segmentation=seg1, detection=det2), 0)[1] is det2
    assert latest_live_packet(dict(segmentation=seg1, detection=det2), 1) == (None, None)


def test_observation_third_pane_shows_live_result(ui):
    seg = _packet(True, 7)
    ui._draw_live_pane(dict(segmentation=seg))
    pane = ui.images['detection'].master
    assert '实时检测与分割' in str(pane.cget('text')) and '#7' in str(pane.cget('text'))
    assert ui._image_pixels['detection'][15, 15].any()
