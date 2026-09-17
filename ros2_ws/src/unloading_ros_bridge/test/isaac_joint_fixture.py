"""Tiny synthetic capture, never evidence of a real Isaac acquisition."""
import json
from pathlib import Path

import numpy as np

from unloading_perception.isaac_validation import IsaacSceneManifest, canonical_digest, sha256_file
from unloading_perception.rgbd import CaptureMetadata, IlluminationState


def capture_fixture(directory, *, velocity=(0.25, -0.5), stamp=10.125):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # Reuse the validated camera/scene envelope, but explicitly create a new
    # synthetic, two-joint robot identity. No simulation/rendering is performed.
    root = Path(__file__).resolve().parents[4]
    payload = json.loads((root / 'docs/validation/evidence/roof-mast-lit-20260916/manifest.json').read_text())
    payload['run_id'] = 'SYNTHETIC_JOINT_CONTRACT_FIXTURE'
    payload['robot'].update(joint_names=['fixture_a', 'fixture_b'], q_rad=[0.1, -0.2],
                            robot_model_identity='synthetic-two-joint-robot', robot_asset_hash='a'*64)
    payload['timing'].update(simulation_epoch='synthetic-joint-capture', simulation_frame=7, simulation_time=stamp)
    payload['world_fingerprint'] = canonical_digest({
        'layout_fingerprint': payload['layout']['layout_fingerprint'],
        'dynamic_scene_fingerprint': payload['dynamic_scene_fingerprint'], 'robot': payload['robot']})
    payload.pop('manifest_fingerprint')
    payload['manifest_fingerprint'] = canonical_digest(payload)
    manifest = IsaacSceneManifest.from_dict(payload)
    manifest_path = directory / 'manifest.json'
    manifest_path.write_text(json.dumps(payload))
    camera = manifest.cameras[0]
    np.save(directory / 'sensor_rgb.npy', np.zeros((2, 3, 3), dtype=np.uint8))
    np.save(directory / 'metric_depth_m.npy', np.ones((2, 3), dtype=np.float32))
    np.savez(directory / 'pointcloud_world_m.npz', xyz_m=np.zeros((1, 3), dtype=np.float32))
    # Valid minimal PNG; adapter's pre-existing path verifies its byte hash.
    import base64
    (directory / 'sensor_rgb.png').write_bytes(base64.b64decode(
        'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jRZkAAAAASUVORK5CYII='))
    (directory / 'gt_annotations.json').write_text(json.dumps({'objects': []}))
    metadata = CaptureMetadata('synthetic-joint-capture:7', 'synthetic-joint-capture', 7,
        stamp, stamp, stamp, stamp, 'ros_sim_time', camera['frame_id'], camera['depth_frame_id'],
        'b'*64, IlluminationState.LIGHT_ON_NOMINAL, 'synthetic-j1', 0.1,
        tuple(tuple(row) for row in camera['T_W_C']))
    (directory / 'capture_metadata.json').write_text(json.dumps(metadata.to_dict()))
    state = dict(schema_version='isaac_robot_state_v1',
        source='ARTICULATION_READBACK', synthetic_fixture=True,
        manifest_fingerprint=manifest.manifest_fingerprint,
        robot_model_identity=manifest.robot['robot_model_identity'], robot_asset_hash=manifest.robot['robot_asset_hash'],
        simulation_epoch='synthetic-joint-capture', frame_sequence=7, sample_time=stamp,
        clock_domain='ros_sim_time', joint_names=['fixture_a', 'fixture_b'],
        position_rad=[0.125, -0.225], velocity_rad_s=list(velocity))
    binding = dict(schema_version='isaac_capture_binding_v1', simulation_epoch='synthetic-joint-capture',
        frame_sequence=7, simulation_time=stamp, rgb_sha256=sha256_file(directory/'sensor_rgb.png'),
        gt_snapshot_sha256=sha256_file(directory/'gt_annotations.json'), camera_calibration_identity='b'*64,
        robot_state=state)
    (directory/'capture_binding.json').write_text(json.dumps(binding))
    return manifest, manifest_path, binding
