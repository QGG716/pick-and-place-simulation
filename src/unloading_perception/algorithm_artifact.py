"""Single-run domain artifact and read-only replay, without model or ROS imports."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path

from unloading_contracts import (PerceptionObservation, ObservationStatus, UnknownRegion,
                                SCHEMA_VERSION, canonical_fingerprint, dumps, loads, to_wire)
from .algorithm_handoff import fused_algorithm_observation, validate_algorithm_capture
from .fusion import ModuleFaceBatch, fuse_module_face_batches
from .replay import source_metadata, validate_replay_observation


def file_reference(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def empty_module_observation(payload):
    """Verified SAM returned no masks; absence is explicitly unknown space."""
    m = payload.metadata
    return PerceptionObservation(SCHEMA_VERSION, 'rgbd-empty-' + m.capture_id,
        m.sensor_epoch, m.frame_sequence, m.capture_center_time, m.capture_center_time,
        m.clock_domain, 'staged-registered-rgbd-geometry',
        '1d208f2ed380a207e6e46b4a62d2ac640edfe477',
        'sam+pinned-plane-cuboid+registered-depth', m.calibration_identity,
        ObservationStatus.PARTIAL, None, None, (),
        (UnknownRegion('empty-' + m.capture_id, 'world', 'EMPTY_SEGMENTATION_UNKNOWN_VOLUME'),),
        {'proposal_source': 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL', 'raw_image_automatic': False,
         'absence_means_free_space': False, 'input_provenance': payload.input_provenance(),
         'module_binding': {'module_id': payload.camera['module_id'], 'capture_id': m.capture_id,
                            'calibration_identity': m.calibration_identity, 'T_W_C_at_capture': m.T_W_C_at_capture}}, False)


def publish_algorithm_run(output, results, *, expected_modules, run_id, models, config, write_json):
    """Fuse only this call's complete results; atomically publish index last."""
    if not expected_modules or set(results) != set(expected_modules):
        raise ValueError('MISSING_CURRENT_RUN_MODULE')
    batches, observations, references = [], [], {}
    for module in expected_modules:
        result = results[module]
        observation = result['observation']
        if observation.coverage['module_binding']['module_id'] != module:
            raise ValueError('ALGORITHM_MODULE_IDENTITY_MISMATCH')
        observations.append(observation)
        batches.append(ModuleFaceBatch(module, observation.source_epoch, observation.source_sequence,
            observation.capture_time, tuple(result['observed_face_sets'])))
        path = output/module/'mode_b_rgbd_observation.json'
        write_json(path, json.loads(dumps(observation)))
        references[module] = file_reference(path)
    fusion = fuse_module_face_batches(batches, expected_modules=expected_modules)
    observation = fused_algorithm_observation(observations, fusion)
    fusion_path = output/'fusion_result.json'
    write_json(fusion_path, to_wire(fusion))
    fusion_ref = file_reference(fusion_path)
    provenance = {'schema_version': 'single_capture_algorithm_v1', 'run_id': run_id,
        'module_observations': references, 'fusion_result': fusion_ref,
        'model_manifest': models, 'model_manifest_identity': canonical_fingerprint(models),
        'config_identity': canonical_fingerprint(config), 'config': config,
        'upstream_commit': observation.upstream_commit}
    observation = replace(observation, coverage={**observation.coverage, 'algorithm_run': provenance})
    path = output/'fused_algorithm_observation.json'
    write_json(path, json.loads(dumps(observation)))
    index = {**provenance, 'observation': file_reference(path)}
    index_path = output/'algorithm_artifact.json'
    write_json(index_path, index)
    return file_reference(index_path)


def _read_reference(reference, root):
    path = Path(reference['path']).resolve()
    if not path.is_relative_to(root):
        raise ValueError('ALGORITHM_ARTIFACT_OUTSIDE_RUN')
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != reference['sha256']:
        raise ValueError('ALGORITHM_ARTIFACT_HASH_MISMATCH: ' + path.name)
    return raw


def load_algorithm_artifact(index_path, expected_sha256):
    """Check the pinned index and all referenced domain artifacts before replay."""
    index_path = Path(index_path).resolve()
    raw = _read_reference({'path': str(index_path), 'sha256': expected_sha256}, index_path.parent)
    index = json.loads(raw.decode('utf-8'))
    if index['schema_version'] != 'single_capture_algorithm_v1' or not index['run_id']:
        raise ValueError('ALGORITHM_ARTIFACT_TYPE_INVALID')
    original = _read_reference(index['observation'], index_path.parent)
    observation = loads(original.decode('utf-8'))
    if (not isinstance(observation, PerceptionObservation) or observation.synthetic
            or observation.provider != 'registered-rgbd-fused-algorithm'
            or observation.status not in (ObservationStatus.COMPLETE, ObservationStatus.PARTIAL)):
        raise ValueError('ALGORITHM_OBSERVATION_REQUIRED')
    validate_algorithm_capture(observation)
    if (canonical_fingerprint(observation.coverage['algorithm_run']) != canonical_fingerprint({k: v for k, v in index.items() if k != 'observation'})
            or set(index['module_observations']) != set(observation.coverage['expected_modules'])
            or set(observation.coverage['received_modules']) != set(observation.coverage['expected_modules'])
            or observation.coverage.get('raw_image_automatic') is not False
            or observation.coverage.get('proposal_source') != 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL'
            or any(c.candidate_eligible for c in observation.cargo)):
        raise ValueError('ALGORITHM_ARTIFACT_PROVENANCE_MISMATCH')
    for reference in (*index['module_observations'].values(), index['fusion_result']):
        _read_reference(reference, index_path.parent)
    return observation, index


def algorithm_replay_record(index_path, expected_sha256, session):
    original, index = load_algorithm_artifact(index_path, expected_sha256)
    identity = {key: getattr(original, key) for key in ('observation_id', 'provider', 'source_epoch',
        'source_sequence', 'capture_time', 'processed_time', 'clock_domain')}
    meta = source_metadata(identity, Path(index['observation']['path']).as_uri(),
                           index['observation']['sha256'], session, 0, 0.)
    meta.update(original_identity=identity, original_fingerprint=canonical_fingerprint(original),
                artifact_index={'path': str(Path(index_path).resolve()), 'sha256': expected_sha256},
                auxiliary_state='DISPLAY_ONLY_MOCK_NOT_MEASURED_AT_CAPTURE')
    record = replace(original, observation_id='replay-record-' + index['observation']['sha256'],
        source_epoch=session, source_sequence=0, capture_time=0., processed_time=0., clock_domain='replay',
        coverage={**original.coverage, 'replay': meta})
    validate_algorithm_replay(record)
    return record


def validate_algorithm_replay(record):
    """Restore original identity for algorithm validation; never retime surfaces."""
    meta = validate_replay_observation(record)
    original = replace(record, **dict(meta['original_identity']),
        coverage={k: v for k, v in record.coverage.items() if k not in ('replay', 'source_restart')})
    validate_algorithm_capture(original)
    if canonical_fingerprint(original) != meta['original_fingerprint']:
        raise ValueError('ALGORITHM_REPLAY_ORIGINAL_CHANGED')
    return original
