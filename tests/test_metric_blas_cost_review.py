"""Synthetic fault injection of returned diagnostics, NOT solver reproduction."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

from test_metric_faces_ab_comparison import thread_fixture, write
from tools.compare_metric_faces_ab import compare, main as strict_main
from tools.compare_depth_preprocessing import sha
from tools.review_metric_blas_cost import (ExhaustiveDifferences, OBJECTIVE,
    numeric_difference, review, solver_cost, main)


def refresh(root):
    """Re-publish synthetic fixture references; never used on historical data."""
    index = json.loads((root/'algorithm_artifact.json').read_text())
    for ref in index['module_observations'].values(): ref['sha256'] = sha(__import__('pathlib').Path(ref['path']))
    observation = json.loads((root/'fused_algorithm_observation.json').read_text())
    observation['payload']['coverage']['algorithm_run'] = {k: v for k, v in index.items() if k != 'observation'}
    write(root/'fused_algorithm_observation.json', observation)
    index['observation']['sha256'] = sha(root/'fused_algorithm_observation.json')
    write(root/'algorithm_artifact.json', index)
    for name in ('ab_summary.json', 'thread_policy_report.json'):
        value = json.loads((root/name).read_text()); value['artifact']['sha256'] = sha(root/'algorithm_artifact.json'); write(root/name, value)


def fixture(tmp_path, changed=True):
    plan, a, b = thread_fixture(tmp_path)
    for root in (a, b):
        for m in json.loads(plan.read_text())['modules']:
            folder = root/m['module_id']
            final = json.loads((folder/'rgbd_cuboids.json').read_text())
            obs = json.loads((folder/'mode_b_rgbd_observation.json').read_text())
            for r, c in zip(final['instances'], obs['payload']['cargo']):
                r.update(geometry_version='DEPTH_METRIC_PATCHES_V1', metric_solver={
                    'cost': math.nextafter(1., 2.) if root == b and changed else 1.,
                    'success': True, 'objective': OBJECTIVE})
                c['raw_result']['record'] = deepcopy(r)
            write(folder/'rgbd_cuboids.json', final); write(folder/'mode_b_rgbd_observation.json', obs)
        refresh(root)
    return plan, a, b


def test_diagnostic_classification_does_not_relax_strict_failure(tmp_path):
    plan, a, b = fixture(tmp_path)
    result = review(a, b, plan)
    assert result['plan_complete'] and not result['records_exact_equal']
    assert result['protected_geometry_exact_equal'] and result['decisions_and_support_exact_equal']
    assert result['diagnostic_differences'] and all(d['ulp_distance'] == 1 for d in result['diagnostic_differences'])
    assert result['diagnostic_review_status'] == 'UNEXPLAINED_REQUIRES_REAL_SOLVER_EVIDENCE'
    assert strict_main(['--before', str(a), '--after', str(b), '--plan', str(plan), '--mode', 'blas-threads',
        '--output', str(tmp_path/'strict.json')]) == 1
    assert not compare(a, b, plan, mode='blas-threads', difference_collector=lambda *args: None)['results_equal']


@pytest.mark.parametrize('field', ['corners_3d_m', 'plane_normal', 'plane_residual_m', 'point_support_count', 'accepted', 'unexpected_diagnostic'])
def test_protected_fields_never_masked(tmp_path, field):
    plan, a, b = fixture(tmp_path)
    p = b/'m0/rgbd_cuboids.json'; value = json.loads(p.read_text()); r = value['instances'][0]
    if field == 'accepted': r[field] = True
    elif field == 'unexpected_diagnostic': r['metric_solver'][field] = 2.
    elif field == 'corners_3d_m': r['camera_facing_faces'][0][field][0][0] += 1e-12
    elif field == 'plane_normal': r['camera_facing_faces'][0][field][0] += 1e-12
    else: r['camera_facing_faces'][0][field] += 1
    write(p, value)
    result = review(a, b, plan)
    assert not result['protected_geometry_exact_equal'] and not result['decisions_and_support_exact_equal']


@pytest.mark.parametrize('cost', [None, float('nan'), float('inf'), -1., 1e200, '1.0'])
def test_invalid_missing_or_abnormal_cost_fails(cost):
    record = {'geometry_version': 'DEPTH_METRIC_PATCHES_V1', 'metric_solver': {'objective': OBJECTIVE, 'cost': cost}}
    with pytest.raises(ValueError, match='COST'): solver_cost(record, 'synthetic')


def test_copies_must_agree_even_when_both_runs_have_same_corruption(tmp_path):
    plan, a, b = fixture(tmp_path, False)
    for root in (a, b):
        p = root/'m0/mode_b_rgbd_observation.json'; value = json.loads(p.read_text())
        value['payload']['cargo'][0]['raw_result']['record']['metric_solver']['cost'] = 2.
        write(p, value); refresh(root)
    assert compare(a, b, plan, mode='blas-threads')['results_equal']
    with pytest.raises(ValueError, match='COPY_MISMATCH'): review(a, b, plan)


def test_exhaustive_count_continues_beyond_detail_limit():
    a = {'metric_solver': {'other': list(range(90))}, 'z_geometry': [0.]}
    b = {'metric_solver': {'other': list(range(1, 91))}, 'z_geometry': [1e-16]}
    collector = ExhaustiveDifferences(2); strict = []
    collector(a, b, 'm/final/instances/0', strict)
    assert collector.count == collector.protected_count == 91
    assert len(collector.details) == 2 and len(strict) == 40
    assert collector.visited > 91


@pytest.mark.parametrize('damage', ['module', 'instance', 'duplicate', 'hash', 'file'])
def test_incomplete_or_unbound_artifacts_fail(tmp_path, damage):
    plan, a, b = fixture(tmp_path)
    if damage == 'hash': (b/'fusion_result.json').write_text('{}')
    elif damage == 'file': (b/'m0/rgbd_cuboids.json').unlink()
    else:
        p = b/('ab_summary.json' if damage == 'module' else 'm0/rgbd_cuboids.json'); v = json.loads(p.read_text())
        if damage == 'module': v['modules'].pop()
        elif damage == 'instance': v['instances'].pop()
        else: v['instances'][-1]['mask_id'] = v['instances'][0]['mask_id']
        write(p, v)
    with pytest.raises((ValueError, FileNotFoundError)): review(a, b, plan)


def test_historical_binary64_distance_is_descriptive_only():
    d = numeric_difference(1.0654886446379104, 1.0654886446379102)
    assert d['ulp_distance'] == 1 and d['absolute_difference'] == 2.220446049250313e-16


def test_returned_cost_fault_injection_preserves_decisions_but_changes_fingerprint(tmp_path):
    from dataclasses import replace
    from unloading_contracts import loads, to_wire, canonical_fingerprint
    from unloading_perception.scene import build_scene_update
    plan, a, b = fixture(tmp_path, False)
    original = loads((a/'m0/mode_b_rgbd_observation.json').read_text())
    raw = to_wire(original.cargo[0])['raw_result']
    raw['record']['metric_solver']['cost'] = math.nextafter(1., 2.)
    changed = replace(original, cargo=(replace(original.cargo[0], raw_result=raw), *original.cargo[1:]))
    before, after = build_scene_update(original), build_scene_update(changed)
    assert before.planning_admissible == after.planning_admissible
    assert before.unknown_regions == after.unknown_regions
    assert [c.candidate_eligible for c in original.cargo] == [c.candidate_eligible for c in changed.cargo]
    assert before.geometry_fingerprint != after.geometry_fingerprint
    assert canonical_fingerprint(original) != canonical_fingerprint(changed)


def test_independent_final_patch_validation_not_bypassed_by_cost():
    from unloading_perception.metric_faces import _binding
    from unloading_perception.final_geometry import validate_final_record
    depth = np.full((40, 40), 2., np.float32); mask = np.ones((40, 40), bool)
    K = np.array([[100., 0, 20], [0, 100., 20], [0, 0, 1.]])
    record = {'geometry_version': 'DEPTH_METRIC_PATCHES_V1', 'accepted': False,
        'support_capture_binding': _binding(depth, mask, K), 'metric_solver': {'cost': 1.},
        'frozen_support_regions': {'0': [[y, 0, 40] for y in range(40)]},
        'camera_facing_faces': [{'support_label': 0, 'corners_3d_m': [[-.3,-.3,2.],[.28,-.3,2.],[.28,.28,2.],[-.3,.28,2.]]}]}
    valid = validate_final_record(record, depth, mask, K)
    assert len(valid['camera_facing_faces']) == 1
    bad = deepcopy(record); bad['metric_solver']['cost'] = 0.
    bad['camera_facing_faces'][0]['corners_3d_m'][0][2] += .1
    rejected = validate_final_record(bad, depth, mask, K)
    assert not rejected['camera_facing_faces']


@pytest.mark.parametrize('changed', ['depth', 'mask', 'K'])
def test_independent_validation_keeps_capture_binding(changed):
    from unloading_perception.metric_faces import _binding
    from unloading_perception.final_geometry import validate_final_record
    depth = np.full((10, 10), 2., np.float32); mask = np.ones((10, 10), bool); K = np.eye(3)
    record = {'geometry_version': 'DEPTH_METRIC_PATCHES_V1', 'support_capture_binding': _binding(depth, mask, K),
              'camera_facing_faces': [], 'metric_solver': {'cost': 1.}}
    if changed == 'depth': depth[0, 0] += .1
    elif changed == 'mask': mask[0, 0] = False
    else: K[0, 0] += .1
    with pytest.raises(ValueError, match='CAPTURE_BINDING_MISMATCH'): validate_final_record(record, depth, mask, K)


def test_missing_real_evidence_cli_is_explicit_nonzero(tmp_path):
    out = tmp_path/'failed.json'
    assert main(['--archive-root',str(tmp_path/'missing'), '--evidence-manifest',str(tmp_path/'missing.json'),
        '--plan',str(tmp_path/'plan.json'), '--code-root',str(tmp_path/'code'), '--output',str(out)]) == 1
    result = json.loads(out.read_text())
    assert result['plan_complete'] is False and result['protected_geometry_exact_equal'] is None
