"""Small file-level Isaac payload boundary; no simulator or ROS imports.

PNG is authoritative RGB. Hashes authenticate file bytes, not numpy values.
Bindings detect accidental replacement/incomplete writes, not hostile edits.
"""
from dataclasses import dataclass
import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from .isaac_validation import IsaacCaptureBinding, IsaacSceneManifest, sha256_file
from .rgbd import CaptureMetadata, RegisteredRgbdFrame


class CapturePayloadError(RuntimeError):
    pass


@dataclass(frozen=True)
class VerifiedCapturePayload:
    manifest: IsaacSceneManifest
    binding: IsaacCaptureBinding
    camera: dict
    annotations: dict
    metadata: CaptureMetadata
    rgb: np.ndarray
    depth: np.ndarray
    points: np.ndarray | None


def camera_content_identity(info):
    # Same canonical JSON convention as the existing acquisition writer.
    return hashlib.sha256(json.dumps(info, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def load_capture_payload(directory, manifest, *, with_pointcloud=False, binding_payload=None):
    """Read each consumed file once, validate that buffer, then parse it.

    A supplied binding is reserved for acquisition finalization before its
    atomic publication; the replay adapter always reads the saved binding.
    """
    directory = Path(directory)
    context = f'module=unknown capture={directory.name}'

    def fail(code, file, detail=''):
        raise CapturePayloadError(f'{code} file={file} {context} {detail}')

    def read(file, expected=None, *, bound=False):
        if bound and (not isinstance(expected, str) or len(expected) != 64
                      or any(c not in '0123456789abcdef' for c in expected)):
            fail('BINDING_MISSING', file, 'expected SHA-256 is absent or invalid')
        try:
            raw = (directory/file).read_bytes()
        except OSError as exc:
            fail('FILE_MISSING', file, type(exc).__name__)
        if bound and hashlib.sha256(raw).hexdigest() != expected:
            fail('HASH_MISMATCH', file)
        return raw

    def parse(file, raw, parser):
        try:
            return parser(raw)
        except (ValueError, TypeError, KeyError, IndexError, OSError, EOFError) as exc:
            fail('FORMAT_INVALID', file, str(exc))

    if binding_payload is None:
        binding_payload = parse('capture_binding.json', read('capture_binding.json'), json.loads)
    if not isinstance(binding_payload, dict):
        fail('FORMAT_INVALID', 'capture_binding.json', 'expected object')
    context = (f"module={binding_payload.get('module_id', 'legacy-camera')} "
               f"capture={binding_payload.get('capture_id', directory.name)} "
               f"epoch={binding_payload.get('simulation_epoch')} frame={binding_payload.get('frame_sequence')}")
    binding = parse('capture_binding.json', binding_payload, IsaacCaptureBinding.from_dict)
    if not isinstance(manifest, IsaacSceneManifest):
        try:
            manifest = IsaacSceneManifest.from_dict(json.loads(Path(manifest).read_bytes()))
        except OSError as exc:
            fail('FILE_MISSING', 'manifest', str(exc))
        except (ValueError, TypeError, KeyError) as exc:
            fail('FORMAT_INVALID', 'manifest', str(exc))
    b = binding.to_dict()
    for key, expected in (('simulation_epoch', manifest.timing['simulation_epoch']),
                          ('frame_sequence', manifest.timing['simulation_frame']),
                          ('simulation_time', manifest.timing['simulation_time'])):
        if b[key] != expected:
            fail('IDENTITY_MISMATCH', 'capture_binding.json', key)
    if b.get('manifest_fingerprint', manifest.manifest_fingerprint) != manifest.manifest_fingerprint:
        fail('IDENTITY_MISMATCH', 'capture_binding.json', 'manifest_fingerprint')
    metadata = parse('capture_metadata.json',
        read('capture_metadata.json', b.get('capture_metadata_sha256'), bound=True),
        lambda raw: CaptureMetadata.from_dict(json.loads(raw)))
    annotations = parse('gt_annotations.json', read('gt_annotations.json', binding.gt_snapshot_sha256, bound=True), json.loads)
    info = parse('camera_info.json', read('camera_info.json'), json.loads)
    if not isinstance(info, dict):
        fail('FORMAT_INVALID', 'camera_info.json', 'expected object')
    identity = parse('camera_info.json', info, camera_content_identity)
    if identity != binding.camera_calibration_identity:
        fail('HASH_MISMATCH', 'camera_info.json', 'canonical calibration content')
    cameras = [c for c in manifest.cameras if c['frame_id'] == metadata.rgb_frame_id]
    if len(cameras) != 1:
        fail('IDENTITY_MISMATCH', 'capture_metadata.json', 'RGB camera frame')
    camera = dict(cameras[0])
    module = camera['module_id']
    expected_id = f'{binding.simulation_epoch}:{module}:{binding.frame_sequence}'
    # Old compatibility roots used the explicit module_0_main alias. Their
    # hashed metadata + hashed calibration + GT manifest identity still bind
    # the actual camera. No missing file hash is inferred here.
    legacy_id = f'{binding.simulation_epoch}:module_0_main:{binding.frame_sequence}'
    ids = {expected_id}
    if 'module_id' not in b and 'module_id' not in info:
        ids.add(legacy_id)
    if (metadata.capture_id not in ids or b.get('capture_id', metadata.capture_id) != metadata.capture_id
            or b.get('module_id', module) != module or info.get('module_id', module) != module
            or metadata.sensor_epoch != binding.simulation_epoch or metadata.frame_sequence != binding.frame_sequence
            or metadata.capture_center_time != binding.simulation_time
            or metadata.clock_domain != 'ros_sim_time' or b.get('clock_domain', 'ros_sim_time') != metadata.clock_domain
            or metadata.depth_frame_id != camera['depth_frame_id']
            or metadata.calibration_identity != identity or metadata.sync_status != 'SYNCHRONIZED'
            or metadata.q1_at_capture_rad != camera['q1_at_capture_rad']
            or metadata.T_W_C_at_capture != tuple(tuple(row) for row in camera['T_W_C'])):
        fail('IDENTITY_MISMATCH', 'capture_metadata.json', 'capture/camera/time')
    for key in ('camera_id', 'frame_id', 'K', 'T_W_C', 'distortion_model', 'registration_mode', 'depth_semantics'):
        if info.get(key) != camera[key]:
            fail('IDENTITY_MISMATCH', 'camera_info.json', key)
    if (info.get('D') != camera['distortion'] or [info.get('width'), info.get('height')] != list(camera['resolution'])):
        fail('IDENTITY_MISMATCH', 'camera_info.json', 'distortion/resolution')
    for key, expected in (('simulation_epoch', binding.simulation_epoch), ('simulation_frame', binding.frame_sequence),
                          ('simulation_time', binding.simulation_time), ('manifest_fingerprint', manifest.manifest_fingerprint)):
        if not isinstance(annotations, dict) or annotations.get(key) != expected:
            fail('IDENTITY_MISMATCH', 'gt_annotations.json', key)
    if annotations.get('module_id', module) != module or not isinstance(annotations.get('objects'), list):
        fail('IDENTITY_MISMATCH', 'gt_annotations.json', 'module/objects')
    if (camera['depth_semantics'] != 'optical_z_m'
            or b.get('metric_depth_semantics', 'optical_z_m') != 'optical_z_m'
            or b.get('metric_depth_unit', 'm') != 'm' or b.get('metric_depth_dtype', 'float32') != 'float32'):
        fail('FORMAT_INVALID', 'metric_depth_m.npy', 'expected float32 optical-Z metres')

    def png(raw):
        from PIL import Image
        with Image.open(BytesIO(raw)) as image:
            if image.format != 'PNG' or image.mode != 'RGB':
                raise ValueError('expected RGB PNG; no BGR or implicit mode conversion')
            return np.array(image)

    rgb = parse('sensor_rgb.png', read('sensor_rgb.png', binding.rgb_sha256, bound=True), png)
    depth = parse('metric_depth_m.npy', read('metric_depth_m.npy', b.get('metric_depth_sha256'), bound=True),
                  lambda raw: np.load(BytesIO(raw), allow_pickle=False))
    height, width = int(info['height']), int(info['width'])
    if (rgb.dtype != np.uint8 or rgb.shape != (height, width, 3)
            or not isinstance(depth, np.ndarray) or depth.dtype.kind != 'f' or depth.dtype.itemsize != 4
            or depth.shape != (height, width)):
        fail('FORMAT_INVALID', 'RGB/depth', 'dtype/shape differs from calibration')
    # Reuse the registered-frame contract after checking raw dtype; all-invalid
    # frames and NaN/+Inf no-hit values remain unchanged (no geometry threshold).
    frame = parse('RGB/depth', None, lambda _: RegisteredRgbdFrame(metadata, rgb, depth,
        np.isfinite(depth) & (depth > 0), tuple(camera['K']), tuple(tuple(r) for r in camera['T_rgb_depth']),
        camera['registration_mode']))
    if camera['registration_mode'] != 'SIMULATION_IDEAL_REGISTERED_DEPTH' or not np.array_equal(camera['T_rgb_depth'], np.eye(4)):
        fail('FORMAT_INVALID', 'camera_info.json', 'Isaac registered depth must be coaxial')
    points = None
    if with_pointcloud:
        for key, expected in (('pointcloud_array_key', 'xyz_m'), ('pointcloud_frame_id', 'world'),
                              ('pointcloud_unit', 'm'), ('pointcloud_dtype', 'float32')):
            if key not in b:
                fail('BINDING_MISSING', 'pointcloud_world_m.npz', key)
            if b[key] != expected:
                fail('FORMAT_INVALID', 'pointcloud_world_m.npz', key)
        def cloud(raw):
            with np.load(BytesIO(raw), allow_pickle=False) as arrays:
                return arrays['xyz_m']
        points = parse('pointcloud_world_m.npz', read('pointcloud_world_m.npz', b.get('pointcloud_sha256'), bound=True), cloud)
        if points.dtype.kind != 'f' or points.dtype.itemsize != 4 or points.ndim != 2 or points.shape[1] != 3:
            fail('FORMAT_INVALID', 'pointcloud_world_m.npz', 'expected Nx3 float32 xyz_m')
        points.setflags(write=False)
    return VerifiedCapturePayload(manifest, binding, camera, annotations, metadata, frame.rgb, frame.depth_optical_z_m, points)


def begin_capture_write(directory):
    """Invalidate any old completion record before overwriting payload files."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'capture_binding.json').unlink(missing_ok=True)


def write_capture_pointcloud(directory, depth, camera):
    """Acquisition helper shared by both modules and the compatibility root.

    Preserve the existing stride-two optical-Z to world conversion.
    """
    k = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    rows, columns = np.indices(depth.shape)
    selected = np.isfinite(depth) & (depth > 0) & (rows % 2 == 0) & (columns % 2 == 0)
    z = depth[selected]
    optical = np.column_stack(((columns[selected]-k[0, 2])*z/k[0, 0],
                               (rows[selected]-k[1, 2])*z/k[1, 1], z))
    transform = np.asarray(camera['T_W_C'], dtype=float)
    points = ((transform[:3, :3] @ optical.T).T + transform[:3, 3]).astype(np.float32)
    np.savez_compressed(Path(directory)/'pointcloud_world_m.npz', xyz_m=points)


def finalize_capture_binding(directory, manifest, camera, binding, *, with_pointcloud=False):
    """Acquisition-only: hash closed files, validate, atomically publish last.

    Replay never calls this function or fills in a missing expected hash.
    """
    directory = Path(directory)
    result = dict(binding)
    result.update(manifest_fingerprint=manifest.manifest_fingerprint, module_id=camera['module_id'],
        capture_id=f"{manifest.timing['simulation_epoch']}:{camera['module_id']}:{manifest.timing['simulation_frame']}",
        clock_domain='ros_sim_time', metric_depth_dtype='float32', metric_depth_unit='m', metric_depth_semantics='optical_z_m')
    for field, filename in (('rgb_sha256', 'sensor_rgb.png'), ('gt_snapshot_sha256', 'gt_annotations.json'),
                            ('metric_depth_sha256', 'metric_depth_m.npy'), ('capture_metadata_sha256', 'capture_metadata.json')):
        result[field] = sha256_file(directory/filename)
    if with_pointcloud:
        result.update(pointcloud_sha256=sha256_file(directory/'pointcloud_world_m.npz'),
            pointcloud_array_key='xyz_m', pointcloud_frame_id='world', pointcloud_unit='m', pointcloud_dtype='float32')
    load_capture_payload(directory, manifest, with_pointcloud=with_pointcloud, binding_payload=result)
    # Formal serialization preserves all additive payload fields and robot_state.
    result = IsaacCaptureBinding.from_dict(result).to_dict()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=directory, suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, directory/'capture_binding.json')
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return result
