"""Disconnected Tk entry, no hardware connection or motion during UI testing."""
import tkinter as tk
from pathlib import Path
import pytest
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.ui import SingleWorkbench,SingleBackend
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.vacuum import VacuumOwner,VacuumView
from unloading_sim.workbench.controlled.ui import ManagedRobotView
from unloading_sim.workbench.controlled.service import ExecutionService
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.devices import D435iObserver
from unloading_sim.workbench.ui import load_mapping,URDF,URDF_SHA
from unloading_sim.workbench.coordinates import Installation,CoordinateModel,Extrinsics

ROOT=Path(__file__).resolve().parents[1]

def test_single_carton_ui_starts_inert_and_rejects_preview_import(monkeypatch):
    c=load(ROOT/'configs/workbench/single_carton.example.json')
    if not Path(c['execution']['snapshot']).exists():pytest.skip('Local model assets unavailable')
    def forbidden(*args,**kwargs):raise AssertionError('TEST_MUST_NOT_OPEN_HARDWARE')
    monkeypatch.setattr('unloading_sim.workbench.devices.realman_sdk',forbidden)
    installation=Installation.load(c['execution']['installation']);model=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
    mapping=load_mapping(c['execution']['mapping']);authority=Authority('eco65_single_carton_segment_v1')
    planner=SinglePlanner(c,mapping,authority,model);service=ExecutionService(mapping,c['execution'],authority,planner.binding)
    vacuum=VacuumOwner(c['vacuum'],open_port=forbidden);backend=SingleBackend(D435iObserver(),ManagedRobotView(service,mapping),VacuumView(vacuum),service)
    root=tk.Tk();ui=SingleWorkbench(root,backend,planner,service,vacuum,c,model,Extrinsics.load(c['camera_extrinsics']),auto_start=False)
    try:
        root.update();assert '实机模式' in root.title()
        assert str(ui.execute_button.cget('state'))=='disabled'
        assert str(ui.plan_button.cget('state'))=='disabled'
        assert '尺寸先验可配置' in ui.details.get()
        assert set(ui.result_windows)=={'detection','segmentation'}
        assert ui.snapshot is None
        with pytest.raises(ValueError,match='预览'):ui.execute_cycle()
        with pytest.raises(ValueError,match='不能导入执行'):ui.load_plan()
        assert service.process is None and not vacuum.connected and not backend.workers['camera'].device.connected
        assert ui.manager.task_id is None
    finally:
        ui.manager.close();backend.close();root.destroy()
        # Collect Tk callback cycles on their owning thread, before the next
        # fake IPC test starts worker threads (Tk destructors may otherwise block).
        del ui,backend,root
        __import__('gc').collect()

def test_overlay_projection_uses_nonzero_camera_distortion():
    import numpy as np
    from unloading_sim.workbench.single_carton.ui import project_camera
    k=dict(width=640,height=480,fx=400.,fy=400.,ppx=320.,ppy=240.,model='brown_conrady',coeffs=[.1,0,0,0,0])
    actual=project_camera([[.5,0,1.]],k)
    # r²=.25; Brown radial factor=1.025, versus pinhole pixel x=520.
    np.testing.assert_allclose(actual,[[525.,240.]],atol=1e-10)
