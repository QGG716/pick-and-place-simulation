"""Open the actual Ubuntu Tk UI under DISPLAY, exercise it and capture its windows.

Only authored replay and inert adapters are used. This is not hardware acceptance.
"""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import platform
import time
import tkinter as tk
import numpy as np

from PIL import ImageGrab

from unloading_sim.workbench.backend import WorkbenchBackend
from unloading_sim.workbench.devices import D435iObserver, RobotObserver, VacuumObserver
from unloading_sim.workbench.replay import generate_demo
from unloading_sim.workbench.ui import ROOT, Workbench, load_mapping


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--scene-snapshot', type=Path)
    p.add_argument('--scene-input-root', type=Path)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    replay = a.output/'demo'
    if not (replay/'manifest.json').exists(): generate_demo(replay)
    backend = WorkbenchBackend(D435iObserver(), RobotObserver(load_mapping(ROOT/'configs/workbench/joint_mapping.json')), VacuumObserver())
    root = tk.Tk()
    errors = []
    root.report_callback_exception = lambda kind, value, traceback: errors.append(str(value))
    ui = Workbench(root, backend, snapshot=a.scene_snapshot, scene_input_root=a.scene_input_root)
    checks = []

    def pump(seconds=.25):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            root.update(); time.sleep(.01)
        assert not errors, errors

    def capture(name):
        root.update_idletasks()
        ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(a.output/(name+'.png'))

    try:
        pump()
        assert all(not w.device.connected for w in backend.workers.values())
        capture('ubuntu_no_devices'); checks.append('no_device_startup')
        legacy = a.output/'legacy.json'
        legacy.write_text(json.dumps({'enable_hardware':True,'auto_connect':True,'mode':'EXECUTE'}))
        ui.load_config(legacy)
        assert all(not w.device.connected for w in backend.workers.values())
        checks.append('old_configuration_does_not_connect')
        backend.load_replay(replay); ui.replay_step(); ui.start_detection(); pump()
        assert 'REPLAY' in ui.metadata.get() and 'SIMULATED' in ui.metadata.get()
        assert len(ui.detection_list.get_children()) == 1
        capture('ubuntu_replay'); checks.append('rgbd_joint_pressure_segmentation_replay')
        ui.load_calibration(ROOT/'configs/workbench/training_extrinsics.example.json')
        if a.scene_snapshot:
            np.testing.assert_allclose(ui.world_from_base[:3,3], [-.12,-.1,1.04],atol=1e-12)
            zero = ui.model.camera(np.zeros(6),ui.calibration)
            np.testing.assert_allclose(zero['T_world_camera'][:3,3],[-.0984467240171534,-.24914158671415392,1.4934768314852272])
        ui.replay_step()
        ui.stationary.set(True); ui.confirm_stationary(); ui.tabs.select(ui.coords); pump(.12)
        assert 'MANUAL_STATIC' in ui.pairing_text.get()
        assert '待独立验证' in ui.calibration_text.get()
        assert 'T_world_camera' in ui.coordinate_text.get('1.0','end')
        ui.coordinate_text.yview_moveto(.5); pump(.12)
        assert ui.coordinate_text.yview()[0] > .2
        ui.coordinate_text.yview_moveto(0.)
        checks.append('coordinate_scroll_survives_refresh')
        capture('ubuntu_coordinates'); checks.append('manual_static_transform_training_label')
        invalid = a.output/'invalid.json'
        invalid.write_text(json.dumps({'matrix':[[1,.5,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}))
        ui.safe(ui.load_calibration, invalid)
        assert ui.calibration is None and '被拒绝' in ui.calibration_text.get()
        checks.append('invalid_extrinsic_clears_previous_calibration')
        ui.load_calibration(ROOT/'configs/workbench/training_extrinsics.example.json')
        backend.frame = replace(backend.frame, stamp=replace(backend.frame.stamp,
            presented_monotonic_ns=backend.frame.stamp.presented_monotonic_ns-10_000_000_000))
        pump()
        assert 'STALE' in ui.camera_status.get() and 'STALE' in ui.pairing_text.get()
        ui.tabs.select(ui.observe); pump(); capture('ubuntu_stale')
        checks.append('expired_data_not_live_or_paired')
        ui.stop_replay(); pump()
        assert ui.camera_status.get() == 'DISCONNECTED'
        assert ui.metadata.get().startswith('No RGB-D') and not ui.detection_list.get_children()
        checks.append('stop_replay_clears_previews')
        if a.scene_snapshot:
            ui.tabs.select(ui.coords); ui.open_scene()
            end = time.monotonic()+55
            while time.monotonic() < end and not any(isinstance(w, tk.Toplevel) for w in root.winfo_children()):
                pump(.1)
            windows = [w for w in root.winfo_children() if isinstance(w,tk.Toplevel)]
            assert windows, list(backend.logs)
            windows[0].geometry('+20+20'); windows[0].lift(); pump()
            capture('ubuntu_complete_scene'); checks.append('original_complete_scene_rendered')
    finally:
        ui.close()
    assert all(not w.thread.is_alive() for w in backend.workers.values())
    checks.append('clean_shutdown')
    report = {'status':'PASS','os':platform.platform(),'python':platform.python_version(),
              'tk':tk.TkVersion,'checks':checks,'hardware_validation':'NOT_RUN',
              'display':os.environ.get('DISPLAY'),'screenshots':sorted(p.name for p in a.output.glob('*.png'))}
    (a.output/'ui_acceptance.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))


if __name__ == '__main__': main()
