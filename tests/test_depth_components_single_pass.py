"""Production depth-component regressions and same-call point-map reuse."""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from unloading_perception import rgbd
from unloading_perception.rgbd import _depth_continuous_components
from test_rgbd_pipeline import _frame
from test_offline_payload_binding import geometry, algorithm_fixture, FILTER


@pytest.mark.parametrize('split,expected', [(False, [100]), (True, [50, 50])])
def test_dense_depth_components_partition_each_pixel_once(split, expected):
    mask = np.ones((10, 10), dtype=bool)
    depth = np.ones(mask.shape)
    if split:
        depth[:, 5:] = 2.
    components = _depth_continuous_components(mask, depth, .02)
    sizes = [len(c) for c in components]
    assert sizes == expected, f'components={len(sizes)}, pixel_memberships={sum(sizes)}'
    assert sum(sizes) == np.count_nonzero(mask)


def reference_partition(mask, depth, threshold):
    # Independent undirected edge graph via union-find; no BFS/start snapshot.
    parents = {tuple(p): tuple(p) for p in np.argwhere(mask)}

    def root(p):
        while parents[p] != p:
            p = parents[p]
        return p

    for y, x in parents:
        for q in ((y + 1, x), (y, x + 1)):
            if q in parents and abs(float(depth[y, x]) - float(depth[q])) <= threshold:
                parents[root(q)] = root((y, x))
    groups = {}
    for p in parents:
        groups.setdefault(root(p), set()).add(p)
    return {frozenset(g) for g in groups.values()}


def assert_partition(mask, depth, threshold):
    actual = _depth_continuous_components(mask, depth, threshold)
    flat = [p for c in actual for p in c]
    assert len(flat) == len(set(flat)) == np.count_nonzero(mask)
    assert set(flat) == {tuple(p) for p in np.argwhere(mask)}
    assert {frozenset(c) for c in actual} == reference_partition(mask, depth, threshold)
    assert actual == _depth_continuous_components(mask, depth, threshold)
    return actual


@pytest.mark.parametrize('mask', [
    np.zeros((0, 0), bool), np.zeros((4, 5), bool), np.ones((1, 1), bool),
    np.eye(5, dtype=bool),  # diagonal-only contact
    np.array([[1, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0]], bool),
    np.pad(np.zeros((2, 3), bool), 1, constant_values=True),  # boundary ring + hole
])
def test_spatial_partition_edges_holes_isolated_and_diagonal(mask):
    assert_partition(mask, np.ones(mask.shape), .02)


def test_threshold_inclusive_and_adjacent_edge_not_seed_depth():
    mask = np.ones((1, 5), bool)
    depth = np.array([[1., 1.25, 1.5, 1.75, 2.25]])
    assert [len(c) for c in assert_partition(mask, depth, .25)] == [4, 1]
    # 0.25 is exact in binary; first-to-last spread .75 must not split the chain.
    assert _depth_continuous_components(mask, depth, .25)[0] == [(0, 0), (0, 1), (0, 2), (0, 3)]


def test_seeded_random_partitions_match_independent_edge_graph():
    rng = np.random.default_rng(71923)
    for _ in range(80):
        mask = rng.random((7, 9)) > .35
        depth = rng.integers(0, 10, mask.shape) / 4 + 1
        assert_partition(mask, depth, .25)


def config(**kwargs):
    return rgbd.PointCloudFilterConfig(**{**FILTER, 'local_depth_component_selection': True, **kwargs})


def test_retention_ties_small_components_minimum_total_and_invalid_depth():
    frame = _frame()
    mask = np.zeros((6, 6), bool)
    mask[:2, :2] = True
    mask[4:, 4:] = True
    mask[0, 5] = True
    both = rgbd.filter_registered_instance_depth(frame, mask, config(minimum_component_points=4, minimum_points=8))
    assert [c['point_count'] for c in both.evidence['components']] == [4, 4, 1]
    assert both.retained_mask.sum() == 8 and both.rejected_reason[0, 5] == 5
    first = rgbd.filter_registered_instance_depth(frame, mask, config(retain_multiple_depth_components=False))
    assert first.retained_mask[:2, :2].all() and first.retained_mask.sum() == 4
    with pytest.raises(ValueError, match='too few filtered points: 8'):
        rgbd.filter_registered_instance_depth(frame, mask, config(minimum_component_points=4, minimum_points=9))
    with pytest.raises(ValueError, match='too few filtered points: 0'):
        rgbd.filter_registered_instance_depth(frame, mask, config(minimum_component_points=5))
    depth = frame.depth_optical_z_m.copy()
    depth[0, :4] = [np.nan, np.inf, 0, -1]
    frame = replace(frame, depth_optical_z_m=depth, valid_depth_mask=np.isfinite(depth) & (depth > 0))
    filtered = rgbd.filter_registered_instance_depth(frame, np.ones((6, 6), bool), config())
    assert filtered.raw_valid_mask.sum() == 32 and (filtered.rejected_reason[0, :4] == 2).all()
    assert sum(c['point_count'] for c in filtered.evidence['components']) == 32


def test_compatibility_wrapper_and_combined_result_keep_arrays_and_identity():
    frame, mask = _frame(), np.ones((6, 6), bool)
    old_interface = rgbd.masked_metric_pointmap(frame, mask, depth_identity='bound-depth', config=config())
    combined, filtered = rgbd.masked_metric_pointmap_with_filter(frame, mask, depth_identity='bound-depth', config=config())
    assert combined.valid_mask is filtered.retained_mask
    for name in ('points_camera_xyz_m', 'depth_optical_z_m', 'valid_mask'):
        np.testing.assert_array_equal(getattr(old_interface, name), getattr(combined, name))
    assert combined.points_camera_xyz_m.dtype == np.float32
    assert combined.capture_id == frame.metadata.capture_id
    assert combined.depth_identity == 'bound-depth'
    assert combined.filter_evidence == old_interface.filter_evidence
    with pytest.raises(ValueError, match='resolution differ'):
        rgbd.masked_metric_pointmap_with_filter(frame, mask[:2], depth_identity='bad', config=config())


def test_real_build_entry_filters_once_per_processed_mask_and_shares_audits(tmp_path, geometry, monkeypatch):
    manifest, _, binding = algorithm_fixture(tmp_path)
    masks = np.ones((4, 2, 3), bool)
    masks[1, 0, 0] = False
    masks[2] = False  # processed but rejected: exactly one attempt
    path = tmp_path / 'sam.npz'
    np.savez(path, masks=masks, labels=['box', 'cardboard_box', 'box', 'wall'], mask_ids=[1, 2, 3, 4])
    calls, results = [], []
    original = rgbd.filter_registered_instance_depth

    def counted(frame, mask, configuration):
        calls.append(mask.copy())
        result = original(frame, mask, configuration)
        results.append(result)
        return result

    monkeypatch.setattr(rgbd, 'filter_registered_instance_depth', counted)
    pointmap, audits = geometry._build_pointmap(tmp_path, manifest, path, {**FILTER, 'local_depth_component_selection': True})
    assert len(calls) == 3 and len(results) == 2
    assert [a['status'] for a in audits] == ['PASS', 'PASS', 'REJECTED']
    assert 'no valid registered depth' in audits[2]['reason']
    for audit, result in zip(audits, results):
        with np.load(audit['audit_npz']) as data:
            np.testing.assert_array_equal(data['filtered_mask'], result.retained_mask)
            np.testing.assert_array_equal(data['raw_valid_mask'], result.raw_valid_mask)
            np.testing.assert_array_equal(data['rejected_reason'], result.rejected_reason)
            assert np.isnan(data['filtered_points_camera_xyz_m'][~result.retained_mask]).all()
        assert audit['components'] == result.evidence['components']
    assert pointmap.depth_identity == binding['metric_depth_sha256']
    # A second invocation must recompute even with exactly the same mask IDs.
    geometry._build_pointmap(tmp_path, manifest, path, FILTER)
    assert len(calls) == 6


def test_real_build_entry_preserves_all_rejected_failure(tmp_path, geometry, monkeypatch):
    manifest, _, _ = algorithm_fixture(tmp_path)
    path = tmp_path / 'sam.npz'
    np.savez(path, masks=np.zeros((1, 2, 3), bool), labels=['box'], mask_ids=[1])
    with pytest.raises(RuntimeError, match='no SAM instance retained'):
        geometry._build_pointmap(tmp_path, manifest, path, FILTER)


@pytest.mark.parametrize('diagnostic', [True, False, None])
def test_secondary_timing_preserves_legacy_interval_and_adds_preprocessing(tmp_path, geometry, monkeypatch, diagnostic):
    from run_metric_small_matrix import oracle_proposal_document
    from unloading_perception.isaac_payload import load_capture_payload

    manifest, _, _ = algorithm_fixture(tmp_path)
    payload = load_capture_payload(tmp_path, manifest, with_instance_masks=True)
    (tmp_path / 'oracle_proposals.json').write_text(json.dumps(oracle_proposal_document(payload)))
    masks = tmp_path / 'sam.npz'
    np.savez(masks, masks=np.ones((1, 2, 3), bool), labels=['box'], mask_ids=[1])
    clock = [0.]
    calls = {'legacy': 0, 'metric': 0}
    monkeypatch.setattr(geometry, 'perf_counter', lambda: clock[0])
    build, write_npz = geometry._build_pointmap, rgbd.MetricPointMap.write_npz

    def timed_build(*a, **kw):
        clock[0] += 7
        return build(*a, **kw)

    def timed_write(*a, **kw):
        clock[0] += 3
        return write_npz(*a, **kw)

    def fake_external(command, **kwargs):
        calls['legacy'] += 1
        clock[0] += 11
        Path(command[command.index('--json-output') + 1]).write_text('{}')
        Path(command[command.index('--output') + 1]).write_bytes(b'synthetic-image')
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    def fake_metric(**kwargs):
        calls['metric'] += 1
        assert 'raw' not in kwargs
        clock[0] += 13
        return {'instances': []}

    def fake_observation(*a, **kw):
        clock[0] += 5
        from unloading_perception.algorithm_artifact import empty_module_observation
        return empty_module_observation(payload), {}, {}, ()

    def fake_evaluation(*a, **kw):
        clock[0] += 2
        return {'world_transform_evaluation': {}}

    metric = ModuleType('metric_depth_runner')
    metric.run_metric_depth = fake_metric
    monkeypatch.setitem(sys.modules, 'metric_depth_runner', metric)
    monkeypatch.setattr(geometry, '_build_pointmap', timed_build)
    monkeypatch.setattr(rgbd.MetricPointMap, 'write_npz', timed_write)
    monkeypatch.setattr(geometry.subprocess, 'run', fake_external)
    monkeypatch.setattr(geometry, '_observation', fake_observation)
    monkeypatch.setattr(geometry, 'ground_truth_observation', lambda *a, **kw: {})
    monkeypatch.setattr(geometry, 'evaluate_observations', fake_evaluation)
    monkeypatch.setattr(geometry, 'write_evaluation', lambda *a: None)
    vision_config = {'pointcloud_filter': FILTER}
    if diagnostic is not None:
        vision_config['legacy_cuboid_diagnostic'] = diagnostic
    diagnostic = bool(diagnostic)
    result = geometry._run_secondary_module(scene='synthetic', module_dir=tmp_path, manifest=manifest,
        artifacts={'cargo_masks.npz': {'path': str(masks)}, 'box_geometry_2d.json': {'path': 'unused'}},
        config={'vision': vision_config}, vision_root=tmp_path,
        upstream_python=Path(sys.executable), timeout=1, payload=payload,
        output_directory=tmp_path / 'new-run')
    assert calls == {'legacy': int(diagnostic), 'metric': 1}
    assert result['elapsed_seconds'] == (24 if diagnostic else 13)
    assert {k: result['timing_seconds'][k] for k in (
        'pointmap_build_including_filter_audits', 'pointmap_write', 'legacy_geometry',
        'module_total_before_timing_record')} == {
        'pointmap_build_including_filter_audits': 7, 'pointmap_write': 3,
        'legacy_geometry': 24 if diagnostic else 13, 'module_total_before_timing_record': 41 if diagnostic else 30}
    assert result['legacy_cuboid_diagnostic']['status'] == ('COMPLETED' if diagnostic else 'DISABLED')
    assert (tmp_path/'new-run/rgbd_cuboids_baseline_raw.json').exists() == diagnostic
    assert (tmp_path/'new-run/rgbd_cuboids_baseline.png').exists() == diagnostic
    provenance = json.loads((tmp_path/'new-run/geometry_input_provenance.json').read_text())
    assert (provenance['external_geometry_command'] is not None) == diagnostic
    assert json.loads((tmp_path / 'new-run/rgbd_stage_timing.json').read_text()) == result['timing_seconds']
