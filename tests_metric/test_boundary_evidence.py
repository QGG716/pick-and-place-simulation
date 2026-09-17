"""Deterministic optical-Z boundary evidence, no sensor/model runtime."""
import json
from itertools import permutations
import numpy as np
import pytest

from unloading_perception.metric_faces import _boundary_kinds, _reduce_boundary_probe_evidence


def test_original_near_occluder_then_far_background_is_not_physical():
    polygon = np.array([[20., 20.], [60., 20.], [60., 60.], [20., 60.]])
    K = np.array([[100., 0, 39.5], [0, 120., 39.5], [0, 0, 1.]])
    depth = np.full((80, 80), 3.)
    depth[18, :] = 1.5  # top edge: 2 px occluder; 4/6 px background
    edge = _boundary_kinds(polygon, np.array([0., 0., 1.]), -2., depth, K)[0]
    assert edge['kind'] == 'UNCLASSIFIED_BOUNDARY', edge
    assert edge['reason'] == 'DEPTH_EVIDENCE_CONFLICT'
    assert edge['sample_counts']['conflict'] == 15 and edge['support_fraction'] == 0.


def top_edge(profiles, *, no_hit=False, offset=-2.):
    depth = np.full((80, 80), np.nan)
    for x, values in zip(range(26, 55, 2), profiles):
        depth[[18, 16, 14], x] = values
    original = depth.copy()
    polygon = np.array([[20., 20.], [60., 20.], [60., 60.], [20., 60.]])
    K = np.array([[100., 0, 39.5], [0, 120., 39.5], [0, 0, 1.]])
    edge = _boundary_kinds(polygon, np.array([0., 0., 1.]), offset, depth, K,
                           positive_infinity_is_no_hit=no_hit)[0]
    np.testing.assert_array_equal(depth, original)
    assert edge['sample_count'] == sum(edge['sample_counts'].values()) == 15
    assert edge['support_fraction_denominator'] == 15
    assert edge['support_fraction_numerator'] == edge['sample_counts']['physical_support']
    return edge


def test_probe_reduction_does_not_depend_on_order_or_majority():
    physical, occlusion, unknown = 'PHYSICAL_EDGE_SUPPORTED', 'OCCLUSION_BOUNDARY', 'UNCLASSIFIED_BOUNDARY'
    for evidence in ((physical, physical, occlusion), (physical, occlusion, occlusion), (unknown, occlusion, physical)):
        assert {_reduce_boundary_probe_evidence(p) for p in permutations(evidence)} == {
            (unknown, 'DEPTH_EVIDENCE_CONFLICT')}
    for depths in set(permutations((1.5, 3., 3.))):
        edge = top_edge([depths]*15)
        assert edge['kind'] == unknown and edge['sample_counts']['conflict'] == 15


@pytest.mark.parametrize('depths,kind,category', [
    ((3., 3., 3.), 'PHYSICAL_EDGE_SUPPORTED', 'physical_support'),
    ((1.5, 1.5, 1.5), 'OCCLUSION_BOUNDARY', 'occlusion'),
    ((2., 2., 2.), 'UNCLASSIFIED_BOUNDARY', 'unknown'),
    ((np.nan, 3., np.nan), 'PHYSICAL_EDGE_SUPPORTED', 'physical_support'),
    ((2., 3., np.nan), 'PHYSICAL_EDGE_SUPPORTED', 'physical_support'),
    ((np.nan, 1.5, np.nan), 'OCCLUSION_BOUNDARY', 'occlusion'),
    ((np.nan, -np.inf, 0.), 'UNCLASSIFIED_BOUNDARY', 'unknown'),
    ((-1., -np.inf, np.nan), 'UNCLASSIFIED_BOUNDARY', 'unknown'),
])
def test_consistent_decisive_depth_and_unknown(depths, kind, category):
    edge = top_edge([depths]*15)
    assert edge['kind'] == kind and edge['sample_counts'][category] == 15
    assert edge['support_fraction'] == (1. if category == 'physical_support' else 0.)


@pytest.mark.parametrize('no_hit,depths,kind,category', [
    (False, (np.inf, np.inf, np.inf), 'UNCLASSIFIED_BOUNDARY', 'unknown'),
    (True, (np.inf, np.inf, np.inf), 'PHYSICAL_EDGE_SUPPORTED', 'physical_support'),
    (True, (1.5, np.inf, np.inf), 'UNCLASSIFIED_BOUNDARY', 'conflict'),
    (False, (1.5, np.inf, np.inf), 'OCCLUSION_BOUNDARY', 'occlusion'),
])
def test_explicit_no_hit_cannot_overwrite_occluder(no_hit, depths, kind, category):
    edge = top_edge([depths]*15, no_hit=no_hit)
    assert edge['kind'] == kind and edge['sample_counts'][category] == 15
    if category == 'conflict':
        assert edge['reason'] == 'DEPTH_EVIDENCE_CONFLICT'


@pytest.mark.parametrize('offset', [0., 2., np.nan, np.inf])
def test_no_hit_requires_finite_positive_predicted_optical_z(offset):
    edge = top_edge([(np.inf, np.inf, np.inf)]*15, no_hit=True, offset=offset)
    assert edge['kind'] == 'UNCLASSIFIED_BOUNDARY'
    assert edge['sample_counts']['unknown'] == 15 and edge['support_fraction'] == 0.
    assert all(p['reason'] == 'INVALID_PREDICTED_OPTICAL_Z' for s in edge['samples'] for p in s['probes'])


@pytest.mark.parametrize('z,kind', [(1.99, 'UNCLASSIFIED_BOUNDARY'), (2.01, 'UNCLASSIFIED_BOUNDARY'),
    (np.nextafter(1.99, -np.inf), 'OCCLUSION_BOUNDARY'),
    (np.nextafter(2.01, np.inf), 'PHYSICAL_EDGE_SUPPORTED')])
def test_existing_ten_millimetre_comparison_unchanged(z, kind):
    assert top_edge([(z, z, z)]*15)['kind'] == kind


@pytest.mark.parametrize('top,expected,cropped', [
    (5., 'PHYSICAL_EDGE_SUPPORTED', False),  # only farther 6px probe is outside
    (1., 'UNCLASSIFIED_BOUNDARY', False),  # all probes outside, but candidate inside
    (0., 'IMAGE_CROP_BOUNDARY', True),  # candidate itself reaches image limit
])
def test_candidate_crop_distinct_from_out_of_image_probe(top, expected, cropped):
    depth = np.full((80, 80), 3.)
    polygon = np.array([[20., top], [60., top], [60., 60.], [20., 60.]])
    edges = _boundary_kinds(polygon, np.array([0., 0., 1.]), -2., depth, np.eye(3))
    assert edges[0]['kind'] == expected
    assert all(s['candidate_at_image_crop'] == cropped for s in edges[0]['samples'])
    assert all(e['kind'] == 'PHYSICAL_EDGE_SUPPORTED' for e in edges[1:])
    assert all(s['probes'][-1]['reason'] == 'OUTER_PROBE_OUTSIDE_IMAGE' for s in edges[0]['samples'])


@pytest.mark.parametrize('supported,unknown,conflicts,kind', [
    (10, 3, 2, 'PHYSICAL_EDGE_SUPPORTED'), (9, 4, 2, 'UNCLASSIFIED_BOUNDARY'),
    (1, 14, 0, 'UNCLASSIFIED_BOUNDARY'),
])
def test_along_edge_votes_keep_all_samples_and_allow_local_pollution(supported, unknown, conflicts, kind):
    profiles = [(3., 3., 3.)]*supported + [(np.nan, np.nan, np.nan)]*unknown + [(1.5, 3., 3.)]*conflicts
    for ordered in (profiles, profiles[::-1]):
        edge = top_edge(ordered)
        assert edge['kind'] == kind
        assert edge['sample_counts'] == dict(physical_support=supported, occlusion=0, crop=0, unknown=unknown, conflict=conflicts)
        assert edge['support_fraction'] == supported/15
        assert len(edge['conflict_sample_indices']) == conflicts
        assert all(edge['samples'][i]['reason'] == 'DEPTH_EVIDENCE_CONFLICT' for i in edge['conflict_sample_indices'])


def analytic_conflict_pair():
    """Perturb only outside-mask 2px probes of an actual completion edge.

    The baseline's geometry locates probes; no classifier is replaced. Oracle
    labels belong to the existing analytic fixture, not a perception claim.
    """
    from test_depth_solver import analytic_box_fixture
    from unloading_perception.metric_faces import fit_metric_faces, MetricFitConfig
    from unloading_perception.metric_support import backproject_pixels
    depth, mask, K, labels, seeds, _ = analytic_box_fixture()
    config = MetricFitConfig(positive_infinity_is_no_hit=True)
    good = fit_metric_faces(depth, mask, K, labels, seeds, mask_id=1, config=config)
    assert good['accepted'], 'normal analytic complete cuboid must pass'
    patches = {f['support_label']: f for f in good['camera_facing_faces']}
    for face in good['completion_diagnostics']['candidate_faces']:
        polygon = np.asarray(face['corners_2d'])
        patch = patches[face['support_label']]
        for edge_index, (a, b) in enumerate(zip(polygon, np.roll(polygon, -1, axis=0))):
            tangent = b-a
            outward = np.array([tangent[1], -tangent[0]])
            if outward @ ((a+b)/2-polygon.mean(axis=0)) < 0: outward *= -1
            outward /= np.linalg.norm(outward)
            along = a+np.linspace(.15, .85, 15)[:, None]*(b-a)
            probes = np.rint(along[:, None, :]+outward*np.array([2, 4, 6])[None, :, None]).astype(int)
            x, y = probes[..., 0], probes[..., 1]
            if not ((x >= 0) & (x < depth.shape[1]) & (y >= 0) & (y < depth.shape[0])).all(): continue
            if mask[y, x].any() or not np.isposinf(depth[y, x]).all(): continue
            near = {tuple(p) for p in probes[:, 0]}
            far = {tuple(p) for p in probes[:, 1:].reshape(-1, 2)}
            if near & far: continue
            rays = backproject_pixels(probes[:, 0], np.ones(15), K)
            predicted = -patch['plane_offset_m']/(rays @ np.asarray(patch['plane_normal']))
            assert (predicted > .5).all()
            changed = depth.copy()
            changed[y[:, 0], x[:, 0]] = predicted-.5
            np.testing.assert_array_equal(changed[mask], depth[mask])
            bad = fit_metric_faces(changed, mask, K, labels, seeds, mask_id=1, config=config)
            return good, bad, changed, mask, K, face['support_label'], edge_index
    raise AssertionError('analytic fixture needs one fully observed background edge')


def test_conflicting_external_depth_rejects_complete_cuboid_not_measured_faces():
    good, bad, depth, mask, K, label, edge_index = analytic_conflict_pair()
    edge = next(f for f in bad['completion_diagnostics']['candidate_faces'] if f['support_label'] == label)['boundary_evidence'][edge_index]
    assert not bad['accepted'], {'accepted': bad['accepted'], 'target_edge': edge}
    assert good['complete_observability'] == 'OBSERVABLE_METRIC_MULTIFACE'
    assert bad['complete_observability'] == 'UNRESOLVED_PHYSICAL_BOUNDARIES'
    assert good['completion_diagnostics']['all_visible_edges_physical']
    diagnostic = bad['completion_diagnostics']
    assert not diagnostic['all_visible_edges_physical']
    assert diagnostic['rejection'] == 'UNSUPPORTED_PHYSICAL_BOUNDARY'
    assert any(item['support_label'] == label and item['edge_index'] == edge_index
               and item['reason'] == 'DEPTH_EVIDENCE_CONFLICT' for item in diagnostic['rejected_boundaries'])
    assert edge['sample_counts']['conflict'] == 15 and edge['support_fraction'] == 0.
    assert bad['frozen_support_regions'] == good['frozen_support_regions']
    assert len(good['camera_facing_faces']) == len(bad['camera_facing_faces']) == 3
    for original, retained in zip(good['camera_facing_faces'], bad['camera_facing_faces']):
        for field in ('corners_3d_m', 'corners_2d', 'plane_normal', 'plane_offset_m'):
            np.testing.assert_array_equal(original[field], retained[field])
        assert retained['final_support'] == original['final_support']
        assert retained['final_support']['status'] == 'PASS'
        assert len(retained['boundary_evidence']) == 4
        assert retained['evidence'] == 'registered_metric_depth_plane'
    # Exercise actual serialization, final-validation dispatch and observed-face
    # wrapping: completion remains rejected, independent measured faces survive.
    from unloading_perception.final_geometry import validate_final_record
    from unloading_perception.observed_faces import observed_faces_from_geometry_record
    restored = validate_final_record(json.loads(json.dumps(bad, allow_nan=False)), depth, mask, K)
    assert not restored['accepted'] and restored['complete_observability'] == bad['complete_observability']
    assert restored['completion_diagnostics'] == json.loads(json.dumps(diagnostic, allow_nan=False))
    observed = observed_faces_from_geometry_record(restored, source_instance_id='analytic-box',
        module_id='synthetic-module', capture_id='analytic-boundary', capture_time=1., frame_id='camera')
    assert len(observed.faces) == 3
    assert observed.complete_cuboid_status == 'MULTIFACE_OBSERVED_VOLUME_UNRESOLVED'
    assert all('boundary_evidence' in f.diagnostics for f in observed.faces)
