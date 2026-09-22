import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
from test_offline_payload_binding import geometry


@pytest.mark.parametrize('failure', ['returncode', 'timeout', 'missing_output'])
def test_requested_diagnostic_failure_is_persisted_and_raised(tmp_path, geometry, monkeypatch, failure):
    def fail(*args, **kwargs):
        if failure == 'timeout':
            raise subprocess.TimeoutExpired(['legacy'], 1)
        return SimpleNamespace(returncode=9 if failure == 'returncode' else 0, stdout='output', stderr='failure')
    monkeypatch.setattr(geometry.subprocess, 'run', fail)
    with pytest.raises((RuntimeError, subprocess.TimeoutExpired, FileNotFoundError)):
        geometry._legacy_cuboid_diagnostic(['legacy'], directory=tmp_path, vision_root=tmp_path, timeout=1, enabled=True)
    record = json.loads((tmp_path/'legacy_cuboid_diagnostic.json').read_text())
    assert record['status'] == 'FAILED' and record['error'] and record['seconds'] >= 0


def test_disabled_diagnostic_does_not_read_historical_outputs(tmp_path, geometry, monkeypatch):
    (tmp_path/'rgbd_cuboids_baseline_raw.json').write_text('invalid historical JSON')
    monkeypatch.setattr(geometry.subprocess, 'run', lambda *a, **kw: pytest.fail('legacy invoked'))
    result = geometry._legacy_cuboid_diagnostic(['legacy'], directory=tmp_path, vision_root=tmp_path, timeout=1, enabled=False)
    assert result['status'] == 'DISABLED' and result['command'] is None


def test_pinned_extractor_check_is_not_removed(tmp_path, geometry, monkeypatch):
    from diagnose_metric_calibration import load_extractor
    monkeypatch.setattr(subprocess, 'check_output', lambda *a, **kw: 'wrong-sha')
    with pytest.raises(ValueError, match='UPSTREAM_SHA_MISMATCH'):
        load_extractor(tmp_path)


@pytest.mark.parametrize('enabled', [True, False])
def test_one_shot_production_entry_passes_diagnostic_config_and_publishes_identity(tmp_path, monkeypatch, enabled):
    from test_workcell_perception_once import prepare, report
    import sys
    from unloading_perception.algorithm_artifact import load_algorithm_artifact
    capture, modules, calls, entry, argv = prepare(tmp_path, monkeypatch)
    module = sys.modules['run_isaac_rgbd_geometry']
    original = module._run_secondary_module
    seen = []
    def checked(**kwargs):
        seen.append(kwargs['config']['vision']['legacy_cuboid_diagnostic'])
        return original(**kwargs)
    monkeypatch.setattr(module, '_run_secondary_module', checked)
    assert entry(argv + ['--legacy-cuboid-diagnostic' if enabled else '--no-legacy-cuboid-diagnostic']) == 0
    assert seen == [enabled, enabled]
    summary = report(capture)
    reference = summary['algorithm_artifact']
    _, index = load_algorithm_artifact(reference['path'], reference['sha256'])
    assert index['config']['vision']['legacy_cuboid_diagnostic'] is enabled


def test_one_shot_diagnostic_failure_returns_nonzero_without_artifact(tmp_path, monkeypatch):
    from test_workcell_perception_once import prepare, report
    import sys
    capture, _, _, entry, argv = prepare(tmp_path, monkeypatch)
    def failed(**kwargs):
        (kwargs['module_dir']/'legacy_cuboid_diagnostic.json').write_text(json.dumps(
            {'enabled': True, 'status': 'FAILED', 'error': 'CPU_TEST_REQUESTED_DIAGNOSTIC_FAILURE'}))
        raise RuntimeError('CPU_TEST_REQUESTED_DIAGNOSTIC_FAILURE')
    monkeypatch.setattr(sys.modules['run_isaac_rgbd_geometry'], '_run_secondary_module', failed)
    assert entry(argv+['--legacy-cuboid-diagnostic']) == 1
    summary = report(capture)
    assert all(r['legacy_cuboid_diagnostic']['status'] == 'FAILED' for r in summary['runs'])
    assert not (capture/'perception-once/algorithm_artifact.json').exists()


def test_metric_runner_raw_compatibility_is_not_a_file_dependency(tmp_path, geometry, monkeypatch):
    import numpy as np
    import metric_depth_runner as runner
    import diagnose_metric_calibration as calibration
    from test_rgbd_pipeline import _frame
    from unloading_perception.rgbd import masked_metric_pointmap, PointCloudFilterConfig
    frame = _frame()
    pointmap = tmp_path/'pointmap.npz'
    masked_metric_pointmap(frame, np.ones((6, 6), bool), depth_identity='test',
        config=PointCloudFilterConfig(boundary_erosion_px=0, minimum_points=1)).write_npz(pointmap)
    masks = tmp_path/'cargo_masks.npz'
    np.savez(masks, mask_ids=[1], masks=np.ones((1, 6, 6), bool))
    (tmp_path/'cargo_instances.json').write_text('{"instances":[{"instance_id":1}]}')
    counts = {'extractor': 0, 'labels': 0, 'fit': 0}
    def extractor(path): counts['extractor'] += 1; return object()
    def labels(*a): counts['labels'] += 1; return None, None, {'test': 'SYNTHETIC'}
    def fit(*a, **kw): counts['fit'] += 1; return {'mask_id': kw['mask_id'], 'camera_facing_faces': [], 'accepted': False}
    monkeypatch.setattr(calibration, 'load_extractor', extractor)
    monkeypatch.setattr(runner, 'extract_observation_labels', labels)
    monkeypatch.setattr(runner, 'fit_metric_faces', fit)
    monkeypatch.setattr(runner.cv2, 'imwrite', lambda p, image: True, raising=False)
    class UnreadableRaw:
        def __getitem__(self, key): raise AssertionError('legacy raw read')
    common = dict(source=tmp_path/'not-read.png', masks=masks, pointmap=pointmap,
        depth=frame.depth_optical_z_m, K=frame.K, metadata=frame.metadata, vision_root=tmp_path, rgb=frame.rgb)
    a = runner.run_metric_depth(**common, output=tmp_path/'a', raw=UnreadableRaw())
    b = runner.run_metric_depth(**common, output=tmp_path/'b')
    assert a == b and counts == {'extractor': 2, 'labels': 2, 'fit': 2}
    assert not (tmp_path/'rgbd_cuboids_baseline_raw.json').exists()


def test_legacy_paired_entry_requests_diagnostic_and_consumes_current_output(tmp_path, geometry, monkeypatch):
    import sys
    from types import ModuleType
    import numpy as np
    import run_metric_small_matrix as paired
    import metric_v4_runner
    from test_rgbd_pipeline import _frame
    frame = _frame()
    camera = {'module_id': 'camera', 'K': frame.K}
    folder = tmp_path/'capture/scene/modules/camera'
    folder.mkdir(parents=True)
    np.save(folder/'metric_depth_m.npy', frame.depth_optical_z_m)
    np.savez(folder/'gt_instance_masks.npz', masks=[])
    (folder/'rgbd_cuboids_baseline_raw.json').write_text('invalid historical raw')
    bundle = tmp_path/'bundle'; bundle.mkdir()
    (bundle/'index.json').write_text('{"scenes":[{"scene":"scene","path":"manifest.json"}]}')
    (bundle/'manifest.json').write_text('{}')
    (tmp_path/'models.json').write_text('{}')
    worker = ModuleType('vision_resident_worker')
    worker.ResidentRuntime = worker.parser = None  # reuse-SAM never initializes a worker
    monkeypatch.setitem(sys.modules, 'vision_resident_worker', worker)
    monkeypatch.setattr(paired, 'load_extractor', lambda p: None)
    monkeypatch.setattr(paired, 'IsaacSceneManifest', SimpleNamespace(from_dict=lambda d: SimpleNamespace(cameras=[camera])))
    monkeypatch.setattr(paired, 'load_capture_payload', lambda *a, **kw: object())
    monkeypatch.setattr(paired, 'oracle_proposals', lambda *a, **kw: (frame.metadata.to_dict(), {'objects': []}, {}))
    monkeypatch.setattr(paired, '_worker_artifacts', lambda p: {'cargo_masks.npz': {'path': str(folder/'sam.npz')}})
    current = tmp_path/'current-geometry'; current.mkdir()
    (current/'rgbd_cuboids.json').write_text('{"instances":[]}')
    (current/'rgbd_cuboids_baseline_raw.json').write_text('{"current_run":true}')
    calls = []
    def secondary(**kw):
        assert kw['config']['vision']['legacy_cuboid_diagnostic'] is True
        calls.append('current-diagnostic')
        return {'module_directory': current}
    class LegacyConsumerReached(Exception): pass
    def consume(**kw):
        assert kw['raw'] == {'current_run': True}
        assert kw['pointmap'].parent == kw['source'].parent == current
        calls.append('legacy-consumer')
        raise LegacyConsumerReached()
    monkeypatch.setattr(paired, '_run_secondary_module', secondary)
    monkeypatch.setattr(metric_v4_runner, 'run_metric_v4', consume)
    monkeypatch.setattr(sys, 'argv', ['paired', '--capture', str(tmp_path/'capture'), '--bundle', str(bundle),
        '--vision', str(tmp_path), '--models', str(tmp_path/'models.json'), '--output', str(tmp_path/'paired'), '--reuse-sam'])
    with pytest.raises(LegacyConsumerReached):
        paired.main()
    assert calls == ['current-diagnostic', 'legacy-consumer']
