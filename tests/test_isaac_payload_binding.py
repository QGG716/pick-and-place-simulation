"""Small synthetic payload fixtures; never real Isaac capture acceptance."""
import json
from pathlib import Path

import numpy as np
import pytest

from ros2_ws.src.unloading_ros_bridge.test.isaac_joint_fixture import capture_fixture
from unloading_perception.isaac_payload import (
    CapturePayloadError, begin_capture_write, camera_content_identity,
    finalize_capture_binding, load_capture_payload, write_capture_pointcloud,
)
from unloading_perception.isaac_validation import IsaacCaptureBinding, sha256_file


def save_binding(root, binding):
    (root/'capture_binding.json').write_text(json.dumps(binding))


def test_same_frame_and_formal_binding_round_trip(tmp_path):
    manifest, path, binding = capture_fixture(tmp_path)
    binding['instance_masks_sha256'] = 'c'*64  # retained, but masks are not consumed by this adapter
    save_binding(tmp_path, binding)
    assert IsaacCaptureBinding.from_dict(binding).to_dict() == binding
    loaded = load_capture_payload(tmp_path, path, with_pointcloud=True)
    np.testing.assert_array_equal(loaded.rgb, np.load(tmp_path/'sensor_rgb.npy'))
    assert loaded.depth.dtype == np.float32 and loaded.depth.shape == (2, 3)
    assert loaded.points.shape == (1, 3)
    assert loaded.manifest == manifest
    assert loaded.binding.extensions['robot_state'] == binding['robot_state']
    assert not loaded.rgb.flags.writeable and not loaded.depth.flags.writeable


@pytest.mark.parametrize('filename,field', [
    ('sensor_rgb.png', 'rgb_sha256'), ('metric_depth_m.npy', 'metric_depth_sha256'),
    ('capture_metadata.json', 'capture_metadata_sha256'), ('gt_annotations.json', 'gt_snapshot_sha256'),
    ('pointcloud_world_m.npz', 'pointcloud_sha256'),
])
def test_file_replacement_rejected(tmp_path, filename, field):
    _, path, binding = capture_fixture(tmp_path)
    if filename == 'metric_depth_m.npy':
        depth = np.load(tmp_path/filename)
        depth[0, 0] += .125  # original bug: still the same dtype and dimensions
        np.save(tmp_path/filename, depth)
    elif filename == 'pointcloud_world_m.npz':
        np.savez(tmp_path/filename, xyz_m=np.ones((1, 3), dtype=np.float32))
    else:
        (tmp_path/filename).write_bytes((tmp_path/filename).read_bytes()+b' ')
    with pytest.raises(CapturePayloadError, match='HASH_MISMATCH'):
        load_capture_payload(tmp_path, path, with_pointcloud=True)
    assert json.loads((tmp_path/'capture_binding.json').read_text())[field] == binding[field]


def test_png_only_authority_and_disabled_cloud_not_read(tmp_path, monkeypatch):
    _, path, _ = capture_fixture(tmp_path)
    (tmp_path/'sensor_rgb.npy').write_bytes(b'not a numpy file')
    (tmp_path/'pointcloud_world_m.npz').unlink()
    original = Path.read_bytes
    def read(file):
        assert file.name not in ('sensor_rgb.npy', 'pointcloud_world_m.npz')
        return original(file)
    monkeypatch.setattr(Path, 'read_bytes', read)
    loaded = load_capture_payload(tmp_path, path)
    assert loaded.rgb[0, 0].tolist() == [33, 32, 32]
    assert loaded.points is None


@pytest.mark.parametrize('field', ['metric_depth_sha256', 'capture_metadata_sha256', 'pointcloud_sha256'])
def test_missing_binding_is_never_created_by_reader(tmp_path, field):
    _, path, binding = capture_fixture(tmp_path)
    del binding[field]
    save_binding(tmp_path, binding)
    original = (tmp_path/'capture_binding.json').read_bytes()
    with pytest.raises(CapturePayloadError, match='BINDING_MISSING'):
        load_capture_payload(tmp_path, path, with_pointcloud=True)
    assert (tmp_path/'capture_binding.json').read_bytes() == original


def test_missing_file_has_distinct_diagnostic(tmp_path):
    _, path, _ = capture_fixture(tmp_path)
    (tmp_path/'metric_depth_m.npy').unlink()
    with pytest.raises(CapturePayloadError, match='FILE_MISSING.*metric_depth.*module=.*capture='):
        load_capture_payload(tmp_path, path)


@pytest.mark.parametrize('values', [
    np.array([[np.nan, np.inf, 0.], [-1., 1., 2.]], dtype=np.float32),
    np.full((2, 3), np.nan, dtype=np.float32),
])
def test_no_hit_depth_preserved_without_geometry_threshold(tmp_path, values):
    _, path, binding = capture_fixture(tmp_path)
    np.save(tmp_path/'metric_depth_m.npy', values)
    # Author a new synthetic fixture; this is not a replay-side repair.
    binding['metric_depth_sha256'] = sha256_file(tmp_path/'metric_depth_m.npy')
    save_binding(tmp_path, binding)
    np.testing.assert_array_equal(load_capture_payload(tmp_path, path).depth, values)


@pytest.mark.parametrize('values', [np.ones((2, 3), dtype=np.float64),
    np.ones((2, 3), dtype=np.int32), np.ones((3, 2), dtype=np.float32)])
def test_raw_depth_dtype_and_shape_checked_before_conversion(tmp_path, values):
    _, path, binding = capture_fixture(tmp_path)
    np.save(tmp_path/'metric_depth_m.npy', values)
    binding['metric_depth_sha256'] = sha256_file(tmp_path/'metric_depth_m.npy')
    save_binding(tmp_path, binding)
    with pytest.raises(CapturePayloadError, match='FORMAT_INVALID.*dtype/shape'):
        load_capture_payload(tmp_path, path)


@pytest.mark.parametrize('patch', [
    {'module_id': 'other'}, {'frame_sequence': 8}, {'simulation_epoch': 'other'},
    {'manifest_fingerprint': 'd'*64}, {'capture_id': 'other'}, {'clock_domain': 'ros'},
])
def test_binding_identity_mismatch(tmp_path, patch):
    _, path, binding = capture_fixture(tmp_path)
    binding.update(patch)
    save_binding(tmp_path, binding)
    with pytest.raises(CapturePayloadError, match='IDENTITY_MISMATCH'):
        load_capture_payload(tmp_path, path)


@pytest.mark.parametrize('patch', [{'depth_frame_id': 'wrong'}, {'clock_domain': 'ros'},
    {'sensor_epoch': 'wrong'}, {'frame_sequence': 8}, {'capture_id': 'wrong'},
    {'capture_center_time': 10.}, {'T_W_C_at_capture': np.eye(4).tolist()}])
def test_internally_hashed_metadata_must_match_capture(tmp_path, patch):
    _, path, binding = capture_fixture(tmp_path)
    metadata = json.loads((tmp_path/'capture_metadata.json').read_text())
    metadata.update(patch)
    (tmp_path/'capture_metadata.json').write_text(json.dumps(metadata))
    binding['capture_metadata_sha256'] = sha256_file(tmp_path/'capture_metadata.json')
    save_binding(tmp_path, binding)
    with pytest.raises(CapturePayloadError, match='IDENTITY_MISMATCH|FORMAT_INVALID'):
        load_capture_payload(tmp_path, path)


@pytest.mark.parametrize('rehash', [False, True])
def test_calibration_content_not_just_matching_identity_strings(tmp_path, rehash):
    _, path, binding = capture_fixture(tmp_path)
    info = json.loads((tmp_path/'camera_info.json').read_text())
    info['K'][0] += 1.
    (tmp_path/'camera_info.json').write_text(json.dumps(info))
    if rehash:
        binding['camera_calibration_identity'] = camera_content_identity(info)
        metadata = json.loads((tmp_path/'capture_metadata.json').read_text())
        metadata['calibration_identity'] = binding['camera_calibration_identity']
        (tmp_path/'capture_metadata.json').write_text(json.dumps(metadata))
        binding['capture_metadata_sha256'] = sha256_file(tmp_path/'capture_metadata.json')
        save_binding(tmp_path, binding)
    with pytest.raises(CapturePayloadError, match='HASH_MISMATCH|IDENTITY_MISMATCH'):
        load_capture_payload(tmp_path, path)


@pytest.mark.parametrize('key,values', [('xyz_m', np.ones((1, 3), dtype=np.float64)),
    ('xyz_m', np.ones((3,), dtype=np.float32)), ('wrong_key', np.ones((1, 3), dtype=np.float32))])
def test_cloud_keys_shape_dtype(tmp_path, key, values):
    _, path, binding = capture_fixture(tmp_path)
    np.savez(tmp_path/'pointcloud_world_m.npz', **{key: values})
    binding['pointcloud_sha256'] = sha256_file(tmp_path/'pointcloud_world_m.npz')
    save_binding(tmp_path, binding)
    with pytest.raises(CapturePayloadError, match='FORMAT_INVALID.*pointcloud'):
        load_capture_payload(tmp_path, path, with_pointcloud=True)


@pytest.mark.parametrize('field,value', [('metric_depth_unit', 'mm'), ('metric_depth_semantics', 'range_m'),
    ('pointcloud_frame_id', 'camera'), ('pointcloud_unit', 'mm')])
def test_units_and_frames_are_not_inferred_from_filename(tmp_path, field, value):
    _, path, binding = capture_fixture(tmp_path)
    binding[field] = value
    save_binding(tmp_path, binding)
    with pytest.raises(CapturePayloadError, match='FORMAT_INVALID'):
        load_capture_payload(tmp_path, path, with_pointcloud=True)


def test_single_buffer_hash_and_decode_even_if_path_replaced_after_read(tmp_path, monkeypatch):
    _, path, _ = capture_fixture(tmp_path)
    original = Path.read_bytes
    reads = []
    def read(file):
        raw = original(file)
        reads.append(file.name)
        if file.name == 'metric_depth_m.npy':
            np.save(file, np.full((2, 3), 99., dtype=np.float32))
        return raw
    monkeypatch.setattr(Path, 'read_bytes', read)
    sample = load_capture_payload(tmp_path, path)
    np.testing.assert_array_equal(sample.depth, np.ones((2, 3), dtype=np.float32))
    assert reads.count('metric_depth_m.npy') == 1


@pytest.mark.parametrize('camera_index', [0, 1], ids=['upper', 'lower'])
def test_acquisition_finalizer_publishes_complete_binding_last(tmp_path, camera_index):
    manifest, path, original = capture_fixture(tmp_path, camera_index=camera_index)
    camera = manifest.cameras[camera_index]
    begin_capture_write(tmp_path)
    assert not (tmp_path/'capture_binding.json').exists()
    # An unfinished/invalid write cannot expose a completion record.
    np.save(tmp_path/'metric_depth_m.npy', np.ones((1, 1), dtype=np.float32))
    with pytest.raises(CapturePayloadError):
        finalize_capture_binding(tmp_path, manifest, camera, original, with_pointcloud=True)
    assert not (tmp_path/'capture_binding.json').exists()
    depth = np.ones((2, 3), dtype=np.float32)
    np.save(tmp_path/'metric_depth_m.npy', depth)
    write_capture_pointcloud(tmp_path, depth, camera)
    result = finalize_capture_binding(tmp_path, manifest, camera, original, with_pointcloud=True)
    assert result['robot_state'] == original['robot_state']
    assert result['metric_depth_sha256'] == sha256_file(tmp_path/'metric_depth_m.npy')
    assert result['pointcloud_sha256'] == sha256_file(tmp_path/'pointcloud_world_m.npz')
    assert IsaacCaptureBinding.from_dict(result).to_dict() == result
    sample = load_capture_payload(tmp_path, path, with_pointcloud=True)
    optical = np.array([[-.5, -.25, 1.], [.5, -.25, 1.]])
    transform = np.asarray(camera['T_W_C'])
    np.testing.assert_allclose(sample.points, ((transform[:3, :3] @ optical.T).T+transform[:3, 3]).astype(np.float32))
    assert not list(tmp_path.glob('*.tmp'))


def test_legacy_bound_depth_metadata_and_png_remain_usable(tmp_path):
    _, path, binding = capture_fixture(tmp_path)
    for key in ('manifest_fingerprint', 'capture_id', 'clock_domain', 'module_id'):
        binding.pop(key)
    save_binding(tmp_path, binding)
    (tmp_path/'sensor_rgb.npy').unlink()
    load_capture_payload(tmp_path, path)


def test_legacy_compatibility_alias_uses_actual_bound_camera(tmp_path):
    _, path, binding = capture_fixture(tmp_path)
    for key in ('manifest_fingerprint', 'capture_id', 'clock_domain', 'module_id'):
        binding.pop(key)
    info = json.loads((tmp_path/'camera_info.json').read_text())
    info.pop('module_id')
    (tmp_path/'camera_info.json').write_text(json.dumps(info))
    binding['camera_calibration_identity'] = camera_content_identity(info)
    metadata = json.loads((tmp_path/'capture_metadata.json').read_text())
    metadata.update(capture_id='synthetic-joint-capture:module_0_main:7',
                    calibration_identity=binding['camera_calibration_identity'])
    (tmp_path/'capture_metadata.json').write_text(json.dumps(metadata))
    binding['capture_metadata_sha256'] = sha256_file(tmp_path/'capture_metadata.json')
    save_binding(tmp_path, binding)
    sample = load_capture_payload(tmp_path, path)
    assert sample.camera['module_id'] == 'module_0_upper'
    assert sample.metadata.capture_id == metadata['capture_id']
