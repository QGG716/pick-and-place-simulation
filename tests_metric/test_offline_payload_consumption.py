"""Real geometry/observation/evaluation plumbing with explicit external-model substitutes."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np


def test_complete_geometry_chain_consumes_verified_snapshot_after_source_replacement(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'tools'))
    monkeypatch.syspath_prepend(str(root/'tests'))
    from test_offline_payload_binding import algorithm_fixture, write_json, FILTER
    from unloading_perception.isaac_payload import load_capture_payload
    from unloading_perception.metric_faces import MetricFitConfig
    from unloading_perception.upstream_v4 import UPSTREAM_V4_SHA
    import run_isaac_rgbd_geometry as geometry
    from run_metric_small_matrix import oracle_proposals
    import diagnose_metric_calibration

    source = tmp_path/'capture'
    manifest, _, binding = algorithm_fixture(source)
    payload = load_capture_payload(source, manifest, with_instance_masks=True)
    oracle_proposals(source, manifest.cameras[0], payload=payload)
    artifacts_dir = tmp_path/'synthetic-sam-output'; artifacts_dir.mkdir()
    masks = artifacts_dir/'cargo_masks.npz'
    np.savez(masks, masks=np.ones((1, 2, 3), bool), mask_ids=[1], labels=['box'],
        boxes=[[0, 0, 3, 2]], scores=[.8], sources=['CPU_TEST_SUBSTITUTE'])
    write_json(artifacts_dir/'cargo_instances.json', {
        'source': str(source/'sensor_rgb.png'), 'box_source': str(source/'oracle_proposals.json'),
        'instances': [dict(instance_id=1, proposal_id=1, bbox=[0, 0, 3, 2], label='box',
            boundary_source='CPU_TEST_SUBSTITUTE', validation_score=.8, sam_iou_score=.9,
            sam_prompt_stability=.99, mask_area=6)],
        'proposal_audit': [dict(status='accepted', proposal_id=1, instance_id=1)]})
    write_json(artifacts_dir/'box_geometry_2d.json', {'instances': []})
    artifacts = {name: {'path': str(artifacts_dir/name), 'sha256': geometry.sha256(artifacts_dir/name)}
                 for name in ('cargo_masks.npz', 'cargo_instances.json', 'box_geometry_2d.json')}
    metrics = artifacts_dir/'metrics.json'
    write_json(metrics, {'artifacts': artifacts, 'upstream_commit': UPSTREAM_V4_SHA, 'config': {
        'proposal_sha256': geometry.sha256(source/'oracle_proposals.json'),
        'sam_revision': '70c1a07f894ebb5b307fd9eaaee97b9dfc16068f'}})
    write_json(source/'mode_b1_worker_response.json', {'status': 'COMPLETE', 'input_sha256': binding['rgb_sha256'],
        'metrics_reference': {'path': str(metrics), 'sha256': geometry.sha256(metrics)}})
    verified_artifacts = geometry._worker_artifacts(source, payload=payload)
    calls = []
    original_read = Path.read_bytes

    def external_geometry_substitute(command, **kwargs):
        # Only the external pinned executable is substituted; pointmap, metric
        # strategy, final geometry validation, lineage and evaluation stay real.
        calls.append(command)
        assert Path(command[2]).parent != source
        assert Path(command[2]).read_bytes() == payload.raw_files['sensor_rgb.png']
        for name in payload.raw_files: (source/name).write_bytes(b'replaced after verification')
        def guarded_read(path):
            assert not (path.parent == source and path.name in payload.raw_files), 're-read original capture'
            return original_read(path)
        monkeypatch.setattr(Path, 'read_bytes', guarded_read)
        write_json(Path(command[command.index('--json-output')+1]), {'instances': []})
        return SimpleNamespace(returncode=0, stderr='')

    def unused_extractor(*args):
        raise AssertionError('six-pixel fixture must not invoke a plane/model extractor')
    monkeypatch.setattr(geometry.subprocess, 'run', external_geometry_substitute)
    monkeypatch.setattr(diagnose_metric_calibration, 'load_extractor', lambda _: unused_extractor)
    result = geometry._run_secondary_module(scene='synthetic', module_dir=source, manifest=manifest,
        artifacts=verified_artifacts, config={'vision': {'pointcloud_filter': FILTER}},
        vision_root=tmp_path, upstream_python=Path(sys.executable), timeout=1, payload=payload)
    assert len(calls) == 1
    output = result['module_directory']
    record = json.loads((output/'rgbd_cuboids.json').read_text(encoding='utf-8'))['instances'][0]
    assert record['config'] == asdict(MetricFitConfig(positive_infinity_is_no_hit=True))
    assert not record['accepted'] and record['camera_facing_faces'] == []
    assert record['support_capture_binding']['depth_float32'] == hashlib.sha256(payload.depth.tobytes()).hexdigest()
    observation = result['observation']
    assert observation.capture_time == payload.metadata.capture_center_time
    assert observation.coverage['input_provenance']['original_binding']['metric_depth_sha256'] == binding['metric_depth_sha256']
    assert all(not c.candidate_eligible for c in observation.cargo)
    assert result['report']['mode'] == 'STAGED_RGBD'
    assert np.array_equal(result['payload'].depth, np.ones((2, 3), np.float32))
    assert (output/'sensor_rgb.png').read_bytes() == payload.raw_files['sensor_rgb.png']
    assert not (output/'sensor_rgb.npy').exists()
    external_inputs = json.loads((output/'geometry_input_provenance.json').read_text(encoding='utf-8'))
    assert external_inputs['derived_pointmap']['parent_depth_sha256'] == binding['metric_depth_sha256']
