"""Physical mounting contracts, using explicitly synthetic calibration fixtures."""
import json
import os
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.workbench.coordinates import CoordinateModel, Extrinsics, Installation

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/urdf/ECO65-B.urdf'
SHA = '08b33a0588839f9fef4f08e13bb04ff27071c332f5ac40e688949a1ea5e9970b'


def installation_path(tmp_path):
    path = tmp_path / 'installation.json'
    path.write_bytes((ROOT / 'configs/workbench/external_fixed_base.example.json').read_bytes())
    return path


def fixed_calibration(tmp_path):
    raw = dict(parent_frame='base_link', child_frame='camera_optical',
               camera_mount='external_fixed', unit='m', model_sha256=SHA,
               matrix=[[1, 0, 0, .4], [0, 1, 0, -.1], [0, 0, 1, .8], [0, 0, 0, 1]],
               status='SYNTHETIC_TEST_ONLY', usage='SOFTWARE_TEST_NOT_HARDWARE', session_file=None)
    path = tmp_path / 'camera.json'
    path.write_text(json.dumps(raw))
    return path


def model(world=None):
    if not URDF.exists():
        pytest.skip('Pinned official ECO65-B model not installed')
    return CoordinateModel(URDF, SHA, np.eye(4) if world is None else world, None)


def test_fixed_camera_is_invariant_to_all_joints_and_supports_no_joint_input(tmp_path):
    calibration = Extrinsics.load(fixed_calibration(tmp_path))
    world = np.diag([-1., -1., 1., 1.])
    world[:3, 3] = [.2, .3, .4]
    m = model(world)
    for q in (None, np.zeros(6), np.deg2rad([80, -20, 40, 30, -15, 90])):
        transforms = m.camera(q, calibration)
        np.testing.assert_allclose(transforms['T_base_camera'][:3, 3], [.4, -.1, .8])
        np.testing.assert_allclose(transforms['T_world_camera'][:3, 3], [-.2, .4, 1.2])
    assert calibration.session_status == 'MISSING'
    assert '待独立验证' in calibration.label


def test_unknown_suction_tcp_does_not_default_to_probe_or_reference_tool(tmp_path):
    setup = Installation.load(installation_path(tmp_path))
    assert setup.tool is None and setup.tool_name == 'installed_suction_tool'
    np.testing.assert_array_equal(setup.world_from_base, np.eye(4))
    with pytest.raises(ValueError, match='TOOL_TCP_NOT_CALIBRATED'):
        model().tcp(np.zeros(6))


def test_fixed_installation_refuses_historical_link1_camera(tmp_path):
    setup = Installation.load(installation_path(tmp_path))
    setup.check_camera(Extrinsics.load(fixed_calibration(tmp_path)))
    with pytest.raises(ValueError, match='CAMERA_MOUNT_MISMATCH'):
        setup.check_camera(Extrinsics.load(ROOT / 'configs/workbench/training_extrinsics.example.json'))


def test_link1_installation_uses_capture_j1_and_world_equals_base(tmp_path):
    setup = Installation.load(ROOT / 'configs/workbench/link1_base.example.json')
    calibration = Extrinsics.load(ROOT / 'configs/workbench/training_extrinsics.example.json')
    setup.check_camera(calibration)
    with pytest.raises(ValueError, match='CAMERA_MOUNT_MISMATCH'):
        setup.check_camera(Extrinsics.load(fixed_calibration(tmp_path)))
    m = model(setup.world_from_base)
    zero = m.camera(np.zeros(6), calibration)
    quarter = m.camera([np.pi / 2, 0, 0, 0, 0, 0], calibration)
    np.testing.assert_allclose(zero['T_world_camera'][:3, 3],
                               [-.021553275982846603, .14914158671415392, .4534768314852272])
    np.testing.assert_allclose(quarter['T_world_camera'][:3, 3],
                               [-.14914158671415392, -.021553275982846603, .4534768314852272])
    np.testing.assert_array_equal(quarter['T_world_camera'], quarter['T_base_camera'])
    np.testing.assert_allclose(m.camera([0,.2,-.3,.4,-.5,.6], calibration)['T_world_camera'], zero['T_world_camera'])
    assert calibration.status == 'SOLVED_REQUIRES_VALIDATION'
    assert calibration.usage == 'TRAINING_ONLY_TEST'
    assert calibration.session_status == 'MISSING'


def test_base_extrinsics_require_explicit_fixed_mount(tmp_path):
    path = fixed_calibration(tmp_path)
    raw = json.loads(path.read_text())
    del raw['camera_mount']
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match='BASE_CAMERA_REQUIRES_EXTERNAL_FIXED_MOUNT'):
        Extrinsics.load(path)


@pytest.mark.parametrize('fault', ['world_offset', 'world_unit', 'camera_parent', 'tool_unit', 'tool_unknown_status'])
def test_inconsistent_installations_rejected(tmp_path, fault):
    path = installation_path(tmp_path)
    raw = json.loads(path.read_text())
    if fault == 'world_offset': raw['world']['matrix'][0][3] = 1
    if fault == 'world_unit': raw['world']['unit'] = 'mm'
    if fault == 'camera_parent': raw['camera']['parent_frame'] = 'link_1'
    if fault == 'tool_unit': raw['tool']['unit'] = 'mm'
    if fault == 'tool_unknown_status': raw['tool']['status'] = 'VALIDATED'
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError): Installation.load(path)


def test_mounting_difference_does_not_compare_base_and_link1_as_same_parent(tmp_path):
    with pytest.raises(ValueError, match='DIFFERENT_CAMERA_MOUNTING_FRAMES'):
        model().installation_difference(np.zeros(6), Extrinsics.load(fixed_calibration(tmp_path)), np.eye(4), np.eye(4))


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Actual Tk display required')
def test_actual_ui_fixed_installation_and_failed_replacements(tmp_path):
    import tkinter as tk
    from unloading_sim.workbench.backend import WorkbenchBackend
    from unloading_sim.workbench.devices import D435iObserver, RobotObserver, VacuumObserver
    from unloading_sim.workbench.ui import Workbench, load_mapping
    model()  # Skip explicitly if pinned robot assets are absent.
    backend = WorkbenchBackend(D435iObserver(), RobotObserver(load_mapping(ROOT / 'configs/workbench/joint_mapping.json')), VacuumObserver())
    root = tk.Tk()
    ui = Workbench(root, backend, installation=installation_path(tmp_path))
    try:
        ui.refresh(schedule=False)
        assert ui.model.tool is None and not ui.tools
        assert 'TCP 待标定' in ui.tool_name.get()
        assert '与机器人关节角无关' in ui.camera_chain_text.get()
        ui.load_calibration(fixed_calibration(tmp_path))
        with pytest.raises(ValueError, match='CAMERA_MOUNT_MISMATCH'):
            ui.load_calibration(ROOT / 'configs/workbench/training_extrinsics.example.json')
        assert ui.calibration is None
        tool = dict(name='synthetic_measured_tool', parent_frame='link_6', unit='m',
                    matrix=[[1,0,0,.01],[0,1,0,.02],[0,0,1,.13],[0,0,0,1]],
                    source='Synthetic numeric fixture', status='SYNTHETIC_TEST_ONLY')
        path = tmp_path / 'tool.json'; path.write_text(json.dumps(tool))
        ui.load_tool(path)
        np.testing.assert_allclose(ui.model.tool.matrix[:3, 3], [.01, .02, .13])
        tool['unit'] = 'mm'; path.write_text(json.dumps(tool))
        with pytest.raises(ValueError, match='METRES'):
            ui.load_tool(path)
        assert ui.model.tool is None and not ui.tools
        assert all(not worker.device.connected for worker in backend.workers.values())
    finally:
        ui.close()
        del ui,backend,root
        __import__('gc').collect()


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Actual Tk display required')
def test_scene_display_conversion_preserves_original_geometry_with_base_world(tmp_path):
    import tkinter as tk
    from unloading_sim.workbench.backend import WorkbenchBackend
    from unloading_sim.workbench.devices import D435iObserver, RobotObserver, VacuumObserver
    from unloading_sim.workbench.ui import Workbench, load_mapping
    model()
    world_scene = np.eye(4); world_scene[:3, 3] = [.32, -.1, .8]
    # Coordinate display now shares the renderer's complete snapshot checks.
    # The old three-field pseudo-snapshot was never valid scene evidence.
    path = ROOT / 'outputs/workbench/scene_snapshot.json'
    if not path.is_file(): pytest.skip('Pinned full scene assets unavailable')
    before = path.read_bytes()
    backend = WorkbenchBackend(D435iObserver(), RobotObserver(load_mapping(ROOT / 'configs/workbench/joint_mapping.json')), VacuumObserver())
    root = tk.Tk(); ui = Workbench(root, backend, snapshot=path, installation=installation_path(tmp_path), snapshot_geometry_only=True)
    try:
        np.testing.assert_array_equal(ui.world_from_base, np.eye(4))
        np.testing.assert_allclose(ui.world_from_scene[:3, 3], [-.44, 0, -.24], atol=1e-12)
        np.testing.assert_allclose(ui.world_from_scene[:3, :3], np.diag([-1., -1., 1.]), atol=1e-12)
        assert path.read_bytes() == before
    finally:
        ui.close()
        del ui,backend,root
        __import__('gc').collect()


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='Actual Tk display required')
def test_link1_ui_replay_camera_works_without_assuming_tool_tcp(tmp_path):
    import tkinter as tk
    from unloading_sim.workbench.backend import WorkbenchBackend
    from unloading_sim.workbench.devices import D435iObserver, RobotObserver, VacuumObserver
    from unloading_sim.workbench.replay import generate_demo
    from unloading_sim.workbench.ui import Workbench, load_mapping
    model()
    backend = WorkbenchBackend(D435iObserver(), RobotObserver(load_mapping(ROOT / 'configs/workbench/joint_mapping.json')), VacuumObserver())
    root = tk.Tk()
    ui = Workbench(root, backend, installation=ROOT / 'configs/workbench/link1_base.example.json')
    try:
        ui.load_calibration(ROOT / 'configs/workbench/training_extrinsics.example.json')
        replay = tmp_path / 'synthetic_replay'; generate_demo(replay)
        ui.load_replay(replay)
        ui.stationary.set(True); ui.confirm_stationary(); ui.refresh(schedule=False)
        assert '随 link_1 转动' in ui.coordinate_basis
        assert 'T_base_link1(q_capture)' in ui.camera_chain_text.get()
        details = ui.coordinate_text.get('1.0', 'end')
        assert 'T_world_camera:' in details
        assert 'T_link6_tcp: 未标定' in details
        assert '待独立验证' in ui.calibration_text.get()
        assert all(not worker.device.connected for worker in backend.workers.values())
    finally:
        ui.close()
        del ui,backend,root
        __import__('gc').collect()
