"""Continuous job status line follows the start gate on every refresh (2026-11-26)."""
import gc
import time
import tkinter as tk
from types import SimpleNamespace
from pathlib import Path
import pytest
from unloading_sim.workbench.continuous.ui import ContinuousWorkbench
from unloading_sim.workbench.single_carton.ui import SingleWorkbench
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.live import SingleBackend
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel, Extrinsics
from unloading_sim.workbench.ui import URDF, URDF_SHA
from test_single_carton_live import Camera, Device
from eco65_test_inputs import use_committed_installation, use_synthetic_remote_detector


@pytest.fixture
def ui(tmp_path, monkeypatch):
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
    workbench = ContinuousWorkbench(root, backend, planner, service, vacuum, c, model, Extrinsics.load(c['camera_extrinsics']), auto_start=False)
    yield workbench, monkeypatch
    workbench._closed = True; workbench.manager.close(); backend.close()
    for ident in root.tk.call('after', 'info'): root.after_cancel(ident)
    root.destroy(); del workbench, backend, root; gc.collect()


def test_startup_reason_is_replaced_once_the_gate_clears(ui):
    workbench, monkeypatch = ui
    state = dict(blockers=['机械臂未使能', '机械臂数据无效'])
    def base_refresh(self): self.execute_blockers = list(state['blockers'])
    monkeypatch.setattr(SingleWorkbench, '_update_task_once', base_refresh)
    workbench._update_task_once()
    assert '尚不能开始：机械臂未使能、机械臂数据无效' in workbench.job_text.get()
    assert str(workbench.job_start_button.cget('state')) == 'disabled'
    state['blockers'] = ['无有效计划']  # planning is part of each round, not a start blocker
    workbench._update_task_once()
    assert str(workbench.job_start_button.cget('state')) == 'normal'
    assert '可以开始' in workbench.job_text.get() and '尚不能开始' not in workbench.job_text.get()
