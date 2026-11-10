"""Open the actual Ubuntu Tk UI under DISPLAY, exercise it and capture its windows.

Only authored replay and inert adapters are used. This is not hardware acceptance.
"""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import platform
import threading
import time
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from PIL import ImageGrab

from unloading_sim.workbench.backend import WorkbenchBackend
from unloading_sim.workbench.contracts import Source, Stamp
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
    checks, skips = [], []

    def pump(seconds=.25):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            root.update(); time.sleep(.01)
        assert not errors, errors

    def capture(name):
        root.update_idletasks()
        ImageGrab.grab(xdisplay=os.environ.get('DISPLAY')).save(a.output/(name+'.png'))

    def assert_cleared():
        assert not backend.replay_active and not backend.replay_exhausted and backend.replay is None
        assert backend.frame is backend.joint is backend.pressure is backend.detections is None
        assert not backend.joint_history and not backend.manual_stationary
        assert ui.original_detection_frame is ui.original_detection_joint is None
        assert not ui.detection_list.get_children()
        assert all(not label.cget('image') for label in ui.images.values())
        assert ui.metadata.get().startswith('No RGB-D')
        assert '°' not in ui.joints_text.get() and 'kPa' not in ui.pressure_text.get()
        assert 'T_world_camera:' not in ui.coordinate_text.get('1.0','end')
        assert 'ORIGINAL DETECTION /' not in ui.coordinate_text.get('1.0','end')
        assert 'LIVE_VALID' not in (ui.camera_status.get()+ui.joints_text.get()+ui.pressure_text.get())

    def write_fixture(name, edit):
        path = a.output/name
        # Acceptance may be repeated without reusing an edited record.
        path.mkdir(exist_ok=True)
        (path/'manifest.json').write_bytes((replay/'manifest.json').read_bytes())
        for filename in ('rgbd.npz','mask.npy'):
            (path/filename).write_bytes((replay/filename).read_bytes())
        manifest = path/'manifest.json'; data = json.loads(manifest.read_text())
        edit(data)
        manifest.write_text(json.dumps(data))
        return path

    try:
        pump()
        assert all(not w.device.connected for w in backend.workers.values())
        capture('ubuntu_no_devices'); checks.append('no_device_startup')
        legacy = a.output/'legacy.json'
        legacy.write_text(json.dumps({'enable_hardware':True,'auto_connect':True,'mode':'EXECUTE'}))
        ui.load_config(legacy)
        assert all(not w.device.connected for w in backend.workers.values())
        checks.append('old_configuration_does_not_connect')
        ui.load_replay(replay); ui.start_detection(); pump()
        assert backend.frame.observation_id == 'synthetic_rgbd_0' and backend.replay.index == 1
        assert 'REPLAY' in ui.metadata.get() and 'SIMULATED' in ui.metadata.get()
        assert len(ui.detection_list.get_children()) == 1
        capture('ubuntu_replay'); checks.append('rgbd_joint_pressure_segmentation_replay')
        ui.load_calibration(ROOT/'configs/workbench/training_extrinsics.example.json')
        if a.scene_snapshot and ui.model:
            np.testing.assert_allclose(ui.world_from_base[:3,3], [-.12,-.1,1.04],atol=1e-12)
            zero = ui.model.camera(np.zeros(6),ui.calibration)
            np.testing.assert_allclose(zero['T_world_camera'][:3,3],[-.0984467240171534,-.24914158671415392,1.4934768314852272])
        ui.replay_step()
        assert backend.frame.observation_id == 'synthetic_rgbd_1'
        ui.stationary.set(True); ui.confirm_stationary(); ui.tabs.select(ui.coords); pump(.12)
        assert 'MANUAL_STATIC' in ui.pairing_text.get()
        assert '待独立验证' in ui.calibration_text.get()
        checks.append('first_and_next_records_not_skipped_or_duplicated')
        checks.append('manual_static_pair_and_training_label')
        if ui.model:
            assert 'T_world_camera:' in ui.coordinate_text.get('1.0','end')
            checks.append('manual_static_coordinate_transform')
            if ui.coordinate_text.yview()[1] < 1.:
                ui.coordinate_text.yview_moveto(.5)
                scroll = ui.coordinate_text.yview()[0]
                pump(.12)
                assert scroll > 0 and abs(ui.coordinate_text.yview()[0]-scroll) < .02
                ui.coordinate_text.yview_moveto(0.)
                checks.append('coordinate_scroll_survives_refresh')
            else:
                skips.append('Coordinate scrolling NOT_RUN: all content fits without a scene snapshot')
        else:
            skips.append('Official ECO65-B model unavailable: dynamic coordinate display/clear assertions NOT_RUN')
        capture('ubuntu_coordinates')
        for unit in ('mm', 'MISSING', None, '', 'unknown'):
            ui.load_replay(replay)
            ui.load_calibration(ROOT/'configs/workbench/training_extrinsics.example.json')
            ui.stationary.set(True); ui.confirm_stationary(); ui.refresh(schedule=False)
            if ui.model: assert 'T_world_camera:' in ui.coordinate_text.get('1.0','end')
            old_scene = tk.Toplevel(root)
            ui.scene_windows.append(old_scene)
            old_scene_token = (backend.observation_generation, ui.scene_generation)
            data = dict(ui.calibration.raw)
            if unit == 'MISSING': data.pop('unit')
            else: data['unit'] = unit
            if unit == 'mm':
                data['matrix'] = [list(row) for row in data['matrix']]
                for row in data['matrix'][:3]: row[3] *= 1000
            invalid = a.output/'invalid_unit.json'; invalid.write_text(json.dumps(data))
            ui.safe(ui.load_calibration, invalid)
            assert ui.calibration is None and 'EXTRINSICS_UNIT_M_REQUIRED' in ui.calibration_text.get()
            assert 'T_world_camera:' not in ui.coordinate_text.get('1.0','end')
            assert 'ORIGINAL DETECTION /' not in ui.coordinate_text.get('1.0','end')
            assert not old_scene.winfo_exists()
            ui.results.put(('scene', (old_scene_token, np.zeros((10,10,3),np.uint8), 'obsolete render')))
            pump(.03)
            assert not ui.scene_windows
            checks.append('invalid_unit_clears_calibration_and_scene_'+repr(unit))
        capture('ubuntu_invalid_unit')
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
        # Cancel never enters the backend source transition and preserves confirmation.
        ui.load_replay(replay)
        ui.stationary.set(True); ui.confirm_stationary()
        before, generation = backend.frame, backend.observation_generation
        history = tuple(backend.joint_history)
        with patch('unloading_sim.workbench.ui.filedialog.askdirectory', return_value=''):
            ui.choose_replay()
        assert backend.frame is before and backend.observation_generation == generation
        assert backend.manual_stationary and tuple(backend.joint_history) == history
        checks.append('cancel_chooser_preserves_observations_and_confirmation')
        ui.replay_step(); ui.replay_step(); ui.replay_step(); pump(.05)
        assert backend.frame.observation_id == 'synthetic_rgbd_2' and not backend.replay_active
        assert backend.replay_exhausted and not backend.manual_stationary
        for status in (ui.camera_status, ui.joints_text, ui.pressure_text, ui.detection_status):
            assert 'HISTORICAL_END_OF_REPLAY' in status.get()
        assert ui.images['rgb'].cget('image') and ui.detection_list.get_children()
        assert 'T_world_camera:' not in ui.coordinate_text.get('1.0','end')
        generation = backend.observation_generation
        ui.replay_step()
        assert backend.observation_generation == generation
        capture('ubuntu_replay_end'); checks.append('eof_retains_explicit_history_without_pairing')
        ui.stop_replay(); pump(.05); assert_cleared()
        assert ui.camera_status.get() == 'DISCONNECTED'
        assert ui.metadata.get().startswith('No RGB-D') and not ui.detection_list.get_children()
        checks.append('stop_replay_clears_previews')
        for origin in ('SIMULATED', None):
            def mixed(data):
                data['records'][0]['frame']['stamp'].update(source='REPLAY', original_source='LIVE')
                data['records'][0]['joint']['stamp'].update(source='REPLAY', original_source=origin)
            ui.load_replay(write_fixture('origin_'+str(origin), mixed))
            ui.stationary.set(True); ui.confirm_stationary(); ui.refresh(schedule=False)
            assert 'NO_JOINT_SAMPLE' in ui.pairing_text.get()
            assert 'T_world_camera:' not in ui.coordinate_text.get('1.0','end')
            checks.append('incompatible_origin_not_paired_'+str(origin))
        # Delay the actual inference worker across a failed switch, testing both
        # late success and late error without loading neural-network weights.
        for fault in ('missing_npz', 'invalid_joint'):
            ui.load_replay(replay)
            frame, joint, pressure, result = backend.frame, backend.joint, backend.pressure, backend.detections
            ui.stop_replay()
            live = Stamp.receive()
            frame, joint = replace(frame, stamp=live), replace(joint, stamp=live)
            result = replace(result, source=Source.LIVE)
            backend.frame, backend.joint = frame, joint
            backend.pressure, backend.detections = replace(pressure, stamp=live), result
            backend.joint_history.append(joint); backend.manual_stationary = True
            ui.original_detection_frame, ui.original_detection_joint = frame, joint
            ui.refresh(schedule=False)
            assert 'LIVE_VALID' not in ui.camera_status.get()  # Actual devices are disconnected.
            entered, release = threading.Event(), threading.Event()
            def delayed(_):
                entered.set()
                if not release.wait(5): raise RuntimeError('test worker timed out')
                if fault == 'invalid_joint': raise RuntimeError('obsolete inference failure')
                return result
            ui.detector = SimpleNamespace(detect=delayed)
            ui.detecting = ui.detect_busy = True
            worker = threading.Thread(target=ui._infer, args=(frame,joint,ui._inference_token()))
            worker.start()
            assert entered.wait(1)
            def broken(data):
                if fault == 'missing_npz': data['records'][0]['frame']['arrays'] = 'missing.npz'
                else: data['records'][0]['joint']['q_rad'] = [0]*5
            try:
                ui.safe(ui.load_replay, write_fixture(fault, broken))
                assert_cleared()
                error = ui.notice.get()
                assert error and any('Replay load failed' in line for line in backend.logs)
            finally:
                release.set(); worker.join(timeout=2)
            assert not worker.is_alive()
            pump(.05); assert_cleared()
            assert ui.notice.get() == error and not ui.detect_busy
            checks.append('failed_switch_clears_live_and_discards_late_inference_'+fault)
        capture('ubuntu_failed_replay')
        def corrupt_next(data): data['records'][1]['joint']['q_rad'] = [0]*5
        ui.load_replay(write_fixture('invalid_next', corrupt_next))
        ui.safe(ui.replay_step); assert_cleared()
        assert any('Replay read failed' in line for line in backend.logs)
        checks.append('read_error_clears_instead_of_retaining_eof_frame')
        if a.scene_snapshot:
            ui.tabs.select(ui.coords); ui.open_scene()
            end = time.monotonic()+55
            while time.monotonic() < end and not any(isinstance(w, tk.Toplevel) for w in root.winfo_children()):
                pump(.1)
            windows = [w for w in root.winfo_children() if isinstance(w,tk.Toplevel)]
            assert windows, list(backend.logs)
            windows[0].geometry('+20+20'); windows[0].lift(); pump()
            capture('ubuntu_complete_scene'); checks.append('original_complete_scene_rendered')
        else:
            skips.append('Complete scene render NOT_RUN: no --scene-snapshot; unrelated to this patch')
    finally:
        ui.close()
    assert all(not w.thread.is_alive() for w in backend.workers.values())
    checks.append('clean_shutdown')
    report = {'status':'PASS','os':platform.platform(),'python':platform.python_version(),
              'tk':tk.TkVersion,'checks':checks,'skips':skips,'hardware_validation':'NOT_RUN',
              'display':os.environ.get('DISPLAY'),'screenshots':sorted(p.name for p in a.output.glob('*.png'))}
    (a.output/'ui_acceptance.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))


if __name__ == '__main__': main()
