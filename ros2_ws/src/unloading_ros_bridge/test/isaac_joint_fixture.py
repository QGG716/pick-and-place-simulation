"""Tiny synthetic capture, never evidence of a real Isaac acquisition."""
import json
from pathlib import Path

import numpy as np

from unloading_perception.isaac_validation import IsaacSceneManifest, canonical_digest, sha256_file
from unloading_perception.rgbd import CaptureMetadata, IlluminationState


def capture_fixture(directory, *, velocity=(0.25, -0.5), stamp=10.125, frame=7, pixel=32, camera_offset=0., camera_index=0):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # Reuse the validated camera/scene envelope, but explicitly create a new
    # synthetic, two-joint robot identity. No simulation/rendering is performed.
    root = Path(__file__).resolve().parents[4]
    payload = json.loads((root / 'docs/validation/evidence/roof-mast-lit-20260916/manifest.json').read_text())
    payload['run_id'] = 'SYNTHETIC_JOINT_CONTRACT_FIXTURE'
    payload['robot'].update(joint_names=['fixture_a', 'fixture_b'], q_rad=[0.1, -0.2],
                            robot_model_identity='synthetic-two-joint-robot', robot_asset_hash='a'*64)
    payload['timing'].update(simulation_epoch='synthetic-joint-capture', simulation_frame=frame, simulation_time=stamp)
    for camera in payload['cameras']:
        camera.update(resolution=[3, 2], K=[2., 0., 1., 0., 2., .5, 0., 0., 1.])
        camera['T_W_C'][0][3] += camera_offset
    payload['dynamic_scene_fingerprint'] = canonical_digest({
        'objects': payload['objects'], 'mechanisms': payload['mechanisms'],
        'camera_calibration': tuple({key: c[key] for key in ('camera_id', 'frame_id', 'resolution',
            'K', 'distortion_model', 'distortion', 'T_W_C', 'near_clip_m', 'far_clip_m')} for c in payload['cameras'])})
    payload['world_fingerprint'] = canonical_digest({
        'layout_fingerprint': payload['layout']['layout_fingerprint'],
        'dynamic_scene_fingerprint': payload['dynamic_scene_fingerprint'], 'robot': payload['robot']})
    payload.pop('manifest_fingerprint')
    payload['manifest_fingerprint'] = canonical_digest(payload)
    manifest = IsaacSceneManifest.from_dict(payload)
    manifest_path = directory / 'manifest.json'
    manifest_path.write_text(json.dumps(payload))
    camera = manifest.cameras[camera_index]
    rgb = np.full((2, 3, 3), pixel, dtype=np.uint8)
    rgb[..., 0] = pixel+1  # distinguish RGB from BGR
    np.save(directory / 'sensor_rgb.npy', rgb)
    np.save(directory / 'metric_depth_m.npy', np.ones((2, 3), dtype=np.float32))
    np.savez(directory / 'pointcloud_world_m.npz', xyz_m=np.zeros((1, 3), dtype=np.float32))
    from PIL import Image
    Image.fromarray(rgb).save(directory / 'sensor_rgb.png')
    info = {key: camera[key] for key in ('module_id', 'camera_id', 'frame_id', 'K',
        'T_W_C', 'distortion_model', 'depth_semantics', 'registration_mode')}
    info.update(width=3, height=2, D=camera['distortion'])
    (directory / 'camera_info.json').write_text(json.dumps(info))
    calibration = canonical_digest(info)
    (directory / 'gt_annotations.json').write_text(json.dumps({'objects': [],
        'module_id': camera['module_id'], 'simulation_epoch': 'synthetic-joint-capture',
        'simulation_frame': frame, 'simulation_time': stamp, 'manifest_fingerprint': manifest.manifest_fingerprint}))
    capture_id = f"synthetic-joint-capture:{camera['module_id']}:{frame}"
    metadata = CaptureMetadata(capture_id, 'synthetic-joint-capture', frame,
        stamp, stamp, stamp, stamp, 'ros_sim_time', camera['frame_id'], camera['depth_frame_id'],
        calibration, IlluminationState.LIGHT_ON_NOMINAL, 'synthetic-j1', float(camera['q1_at_capture_rad']),
        tuple(tuple(row) for row in camera['T_W_C']))
    (directory / 'capture_metadata.json').write_text(json.dumps(metadata.to_dict()))
    state = dict(schema_version='isaac_robot_state_v1',
        source='ARTICULATION_READBACK', synthetic_fixture=True,
        manifest_fingerprint=manifest.manifest_fingerprint,
        robot_model_identity=manifest.robot['robot_model_identity'], robot_asset_hash=manifest.robot['robot_asset_hash'],
        simulation_epoch='synthetic-joint-capture', frame_sequence=frame, sample_time=stamp,
        clock_domain='ros_sim_time', joint_names=['fixture_a', 'fixture_b'],
        position_rad=[0.125, -0.225], velocity_rad_s=list(velocity))
    binding = dict(schema_version='isaac_capture_binding_v1', simulation_epoch='synthetic-joint-capture',
        frame_sequence=frame, simulation_time=stamp, rgb_sha256=sha256_file(directory/'sensor_rgb.png'),
        gt_snapshot_sha256=sha256_file(directory/'gt_annotations.json'), camera_calibration_identity=calibration,
        metric_depth_sha256=sha256_file(directory/'metric_depth_m.npy'),
        capture_metadata_sha256=sha256_file(directory/'capture_metadata.json'),
        manifest_fingerprint=manifest.manifest_fingerprint, module_id=camera['module_id'],
        capture_id=capture_id, clock_domain='ros_sim_time',
        pointcloud_sha256=sha256_file(directory/'pointcloud_world_m.npz'),
        pointcloud_array_key='xyz_m', pointcloud_frame_id='world', pointcloud_unit='m', pointcloud_dtype='float32',
        robot_state=state)
    (directory/'capture_binding.json').write_text(json.dumps(binding))
    return manifest, manifest_path, binding
