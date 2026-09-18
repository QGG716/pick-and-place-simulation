"""File-boundary regression; synthetic captures and masks, no model inference."""
import importlib
import hashlib
import json
from pathlib import Path
import sys
from types import ModuleType
import subprocess

import numpy as np
import pytest

from ros2_ws.src.unloading_ros_bridge.test.isaac_joint_fixture import capture_fixture
from unloading_perception.isaac_payload import (CapturePayloadError, load_capture_payload,
    require_capture_payload, write_capture_snapshot)


@pytest.fixture
def geometry(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools'))
    # Pointmap/observation tests never call OpenCV; keep it optional on CPU.
    monkeypatch.setitem(sys.modules, 'cv2', ModuleType('cv2_display_test_substitute'))
    monkeypatch.delitem(sys.modules, 'run_isaac_rgbd_geometry', raising=False)
    return importlib.import_module('run_isaac_rgbd_geometry')


def write_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def algorithm_fixture(root, *, camera_index=0, frame=7):
    manifest, path, binding = capture_fixture(root, camera_index=camera_index, frame=frame)
    annotations = json.loads((root/'gt_annotations.json').read_text(encoding='utf-8'))
    identity = manifest.objects[0]['simulation_object_id']
    annotations['objects'] = [{'simulation_object_id': identity, 'mask_key': identity,
        'bbox_xyxy': [0, 0, 3, 2], 'visible': True, 'occluded': False}]
    write_json(root/'gt_annotations.json', annotations)
    np.savez(root/'gt_instance_masks.npz', **{identity: np.ones((2, 3), bool)})
    # New synthetic acquisition only. Tests below never repair negative inputs.
    for field, name in [('gt_snapshot_sha256', 'gt_annotations.json'), ('instance_masks_sha256', 'gt_instance_masks.npz')]:
        binding[field] = hashlib.sha256((root/name).read_bytes()).hexdigest()
    binding.pop('robot_state')  # image qualification must not require articulation state
    write_json(root/'capture_binding.json', binding)
    return manifest, path, binding


FILTER = dict(boundary_erosion_px=0, depth_percentile_low=0, depth_percentile_high=100,
    discontinuity_mad_scale=3, discontinuity_floor_m=.02, local_depth_component_selection=False,
    retain_multiple_depth_components=True, minimum_component_points=1, minimum_points=1)


def test_real_pointmap_entry_rejects_modified_positive_depth(tmp_path, geometry):
    manifest, _, binding = capture_fixture(tmp_path)
    depth = np.load(tmp_path / 'metric_depth_m.npy', allow_pickle=False)
    depth[0, 0] += .125
    np.save(tmp_path / 'metric_depth_m.npy', depth)
    masks = tmp_path / 'synthetic_sam_masks.npz'
    np.savez(masks, masks=np.ones((1, 2, 3), bool), labels=['box'], mask_ids=[1])
    with pytest.raises(CapturePayloadError, match='HASH_MISMATCH.*metric_depth_m.npy'):
        geometry._build_pointmap(tmp_path, manifest, masks, FILTER)


@pytest.mark.parametrize('rgb_npy', ['missing', 'corrupt', 'different'])
def test_shared_ros_boundary_and_algorithm_pixels_geometry_identity(tmp_path, geometry, rgb_npy):
    manifest, path, binding = algorithm_fixture(tmp_path)
    if rgb_npy == 'missing': (tmp_path/'sensor_rgb.npy').unlink()
    elif rgb_npy == 'corrupt': (tmp_path/'sensor_rgb.npy').write_bytes(b'bad')
    else: np.save(tmp_path/'sensor_rgb.npy', np.full((2, 3, 3), 222, np.uint8))
    (tmp_path/'pointcloud_world_m.npz').unlink()
    ros = load_capture_payload(tmp_path, path)  # exact shared entry used by the ROS adapter
    payload = load_capture_payload(tmp_path, manifest, with_instance_masks=True,
                                   expected_module_id=manifest.cameras[0]['module_id'])
    assert payload.points is None and ros.instance_masks is None
    np.testing.assert_array_equal(payload.rgb, ros.rgb)
    np.testing.assert_array_equal(payload.depth, ros.depth)
    assert payload.rgb[0, 0].tolist() == [33, 32, 32]
    assert payload.metadata == ros.metadata and payload.camera['K'] == ros.camera['K']
    assert payload.camera['T_W_C'] == ros.camera['T_W_C']
    with pytest.raises(TypeError): payload.camera['module_id'] = 'bad'
    assert not payload.rgb.flags.writeable and not payload.depth.flags.writeable
    assert all(not v.flags.writeable for v in payload.instance_masks.values())
    masks = tmp_path/'synthetic_sam_masks.npz'
    np.savez(masks, masks=np.ones((1, 2, 3), bool), labels=['box'], mask_ids=[1])
    pointmap, audits = geometry._build_pointmap(tmp_path, manifest, masks, FILTER, payload=payload)
    y, x = np.indices((2, 3))
    expected = np.stack(((x-1)/2, (y-.5)/2, np.ones((2, 3))), axis=-1)
    np.testing.assert_allclose(pointmap.points_camera_xyz_m[pointmap.valid_mask], expected[pointmap.valid_mask])
    assert pointmap.depth_identity == binding['metric_depth_sha256']
    assert pointmap.capture_id == payload.metadata.capture_id and audits[0]['status'] == 'PASS'


@pytest.mark.parametrize('filename', ['metric_depth_m.npy', 'sensor_rgb.png', 'capture_metadata.json', 'camera_info.json'])
def test_each_original_file_change_is_rejected_before_geometry(tmp_path, geometry, filename):
    manifest, _, _ = algorithm_fixture(tmp_path)
    original = (tmp_path/'capture_binding.json').read_bytes()
    if filename == 'metric_depth_m.npy':
        depth = np.load(tmp_path/filename, allow_pickle=False); depth[0, 0] += .125
        np.save(tmp_path/filename, depth)
    elif filename == 'camera_info.json':
        info = json.loads((tmp_path/filename).read_text(encoding='utf-8')); info['K'][0] += .25
        write_json(tmp_path/filename, info)
    else:
        (tmp_path/filename).write_bytes((tmp_path/filename).read_bytes()+b' ')
    with pytest.raises(CapturePayloadError, match='HASH_MISMATCH'):
        geometry._run_secondary_module(scene='synthetic', module_dir=tmp_path, manifest=manifest,
            artifacts={}, config={}, vision_root=tmp_path, upstream_python=Path(sys.executable), timeout=1)
    assert (tmp_path/'capture_binding.json').read_bytes() == original
    assert not (tmp_path/'geometry-run').exists()


def test_wrong_but_internally_consistent_module_and_unverified_dict_rejected(tmp_path):
    manifest, _, _ = algorithm_fixture(tmp_path, camera_index=1)
    with pytest.raises(CapturePayloadError, match='IDENTITY_MISMATCH.*expected_module'):
        load_capture_payload(tmp_path, manifest, expected_module_id=manifest.cameras[0]['module_id'])
    with pytest.raises(CapturePayloadError, match='IDENTITY_MISMATCH'):
        require_capture_payload(tmp_path, manifest, payload={'verified': True})
    loaded = load_capture_payload(tmp_path, manifest)
    with pytest.raises(CapturePayloadError, match='IDENTITY_MISMATCH'):
        require_capture_payload(tmp_path/'different', manifest, payload=loaded)


def test_internally_consistent_different_capture_cannot_replace_expected_manifest(tmp_path):
    expected, _, _ = algorithm_fixture(tmp_path/'expected')
    other, _, _ = algorithm_fixture(tmp_path/'other', frame=8)
    load_capture_payload(tmp_path/'other', other, with_instance_masks=True)
    original = (tmp_path/'other/capture_binding.json').read_bytes()
    with pytest.raises(CapturePayloadError, match='IDENTITY_MISMATCH.*frame_sequence'):
        load_capture_payload(tmp_path/'other', expected, with_instance_masks=True)
    assert (tmp_path/'other/capture_binding.json').read_bytes() == original


@pytest.mark.parametrize('field', ['rgb_sha256', 'metric_depth_sha256', 'capture_metadata_sha256', 'instance_masks_sha256'])
def test_absent_original_hash_is_not_replaced_by_actual_hash(tmp_path, field):
    manifest, _, binding = algorithm_fixture(tmp_path)
    del binding[field]
    write_json(tmp_path/'capture_binding.json', binding)
    original = (tmp_path/'capture_binding.json').read_bytes()
    with pytest.raises(CapturePayloadError, match='BINDING_MISSING'):
        load_capture_payload(tmp_path, manifest, with_instance_masks=True)
    assert (tmp_path/'capture_binding.json').read_bytes() == original


@pytest.mark.parametrize('invalid', ['hash', 'shape', 'object', 'pickle'])
def test_gt_mask_boundary_is_optional_and_checks_bound_arrays(tmp_path, invalid):
    manifest, _, binding = algorithm_fixture(tmp_path)
    identity = manifest.objects[0]['simulation_object_id']
    if invalid == 'object': arrays = {'unbound-object': np.ones((2, 3), bool)}
    elif invalid == 'shape': arrays = {identity: np.ones((3, 2), bool)}
    elif invalid == 'pickle': arrays = {identity: np.array([object()], dtype=object)}
    else: arrays = {identity: np.zeros((2, 3), bool)}
    np.savez(tmp_path/'gt_instance_masks.npz', **arrays)
    if invalid != 'hash':  # author malformed-but-hashed synthetic format cases
        binding['instance_masks_sha256'] = hashlib.sha256((tmp_path/'gt_instance_masks.npz').read_bytes()).hexdigest()
        write_json(tmp_path/'capture_binding.json', binding)
    load_capture_payload(tmp_path, manifest)  # ROS default does not consume masks
    with pytest.raises(CapturePayloadError, match='HASH_MISMATCH' if invalid == 'hash' else 'FORMAT_INVALID'):
        load_capture_payload(tmp_path, manifest, with_instance_masks=True)


def test_verified_buffers_survive_source_replacement_and_snapshot_is_exclusive(tmp_path, geometry):
    source = tmp_path/'source'
    manifest, _, binding = algorithm_fixture(source)
    payload = load_capture_payload(source, manifest, with_instance_masks=True)
    originals = dict(payload.raw_files)
    for name in payload.raw_files: (source/name).write_bytes(b'replaced after trusted load')
    snapshot = write_capture_snapshot(payload, tmp_path/'isolated')
    assert dict(snapshot.raw_files) == originals
    assert all((snapshot.directory/name).read_bytes() == raw for name, raw in originals.items())
    assert not (snapshot.directory/'sensor_rgb.npy').exists()
    with pytest.raises(FileExistsError): write_capture_snapshot(payload, snapshot.directory)
    masks = tmp_path/'masks.npz'
    np.savez(masks, masks=np.ones((1, 2, 3), bool), labels=['box'], mask_ids=[1])
    pointmap, _ = geometry._build_pointmap(snapshot.directory, manifest, masks, FILTER, payload=snapshot)
    np.testing.assert_array_equal(pointmap.depth_optical_z_m, np.ones((2, 3), np.float32))
    assert pointmap.depth_identity == binding['metric_depth_sha256']


def test_no_visible_gt_or_robot_state_adds_no_new_quality_gate(tmp_path):
    manifest, _, binding = algorithm_fixture(tmp_path)
    annotations = json.loads((tmp_path/'gt_annotations.json').read_text(encoding='utf-8'))
    annotations['objects'][0].update(visible=False, mask_key=None)
    write_json(tmp_path/'gt_annotations.json', annotations)
    np.savez(tmp_path/'gt_instance_masks.npz')
    for key, filename in [('instance_masks_sha256', 'gt_instance_masks.npz'), ('gt_snapshot_sha256', 'gt_annotations.json')]:
        binding[key] = hashlib.sha256((tmp_path/filename).read_bytes()).hexdigest()
    write_json(tmp_path/'capture_binding.json', binding)  # author a new legal synthetic empty capture
    payload = load_capture_payload(tmp_path, manifest, with_instance_masks=True)
    assert not payload.instance_masks and 'robot_state' not in payload.binding.extensions
    with pytest.raises(TypeError): payload.binding.extensions['metric_depth_sha256'] = 'a'*64


@pytest.mark.parametrize('bad_count', [1, 2])
def test_real_once_cli_rejects_tampered_input_before_inference(tmp_path, bad_count):
    from workcell_once_fakes import capture_fixture as once_fixture
    capture, models, modules = once_fixture(tmp_path, ('normal', 'normal'))
    for module in modules[:bad_count]:
        path = capture/'FULL_STACK_NOMINAL/modules'/module/'metric_depth_m.npy'
        data = np.load(path, allow_pickle=False); data[0, 0] += .125; np.save(path, data)
    launcher = Path(__file__).with_name('workcell_once_cli.py')
    result = subprocess.run([sys.executable, str(launcher), '--capture', str(capture), '--vision', str(tmp_path),
                             '--models', str(models)], capture_output=True, text=True, encoding='utf-8', timeout=20)
    assert result.returncode == 1 and 'HASH_MISMATCH' in result.stderr
    summary = json.loads((capture/'perception-once/summary.json').read_text(encoding='utf-8'))
    assert summary['exit_code'] == 1 and summary['module_counts']['failed'] == bad_count
    for row in summary['runs'][:bad_count]:
        assert row['failure_stage'] == 'payload_validation'
        assert row['sam_attempts'] == row['metric_attempts'] == 0
        assert not (Path(row['artifact_directory'])/'rgbd_cuboids.json').exists()
    if bad_count == 2:
        assert not (capture/'test-calls.jsonl').exists()  # no runtime initialization either
    else:
        assert summary['module_counts']['completed'] == 1


def test_real_geometry_cli_cannot_bypass_raw_binding(tmp_path):
    # Actual child process, no replacement main/loader and no OpenCV dependency.
    capture = tmp_path/'captures'; scene = capture/'synthetic'
    manifest, path, _ = algorithm_fixture(scene)
    bundle = tmp_path/'bundle'; bundle.mkdir()
    write_json(bundle/'index.json', {'scenes': [{'scene': 'synthetic', 'path': 'manifest.json'}]})
    (bundle/'manifest.json').write_bytes(path.read_bytes())
    np.save(scene/'metric_depth_m.npy', np.full((2, 3), 9., np.float32))
    script = Path(__file__).resolve().parents[1]/'tools/run_isaac_rgbd_geometry.py'
    result = subprocess.run([sys.executable, str(script), '--capture-directory', str(capture),
        '--bundle-directory', str(bundle), '--vision-root', str(tmp_path), '--upstream-python', sys.executable],
        capture_output=True, text=True, encoding='utf-8', timeout=20)
    assert result.returncode != 0 and 'HASH_MISMATCH file=metric_depth_m.npy' in result.stderr
    assert not list((capture/'rgbd-geometry-run').rglob('rgbd_cuboids.json'))
