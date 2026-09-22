"""CPU constraint/envelope tests; synthetic fixtures are not real reconstruction evidence."""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from unloading_contracts import canonical_fingerprint, to_wire
from unloading_perception.algorithm_artifact import load_algorithm_artifact, algorithm_replay_record
from unloading_perception.planning_geometry import adapt_artifact, derive_geometry, envelope_to_obb, main
from unloading_perception.scene import build_scene_update
from test_algorithm_artifact import artifact_fixture


def prior(*, error=0., unknown=False):
    value = json.loads(Path('configs/perception/nominal_size_prior_offline.json').read_text())
    value['enabled'] = True
    value['size_prior']['axis_correspondence'] = 'unknown' if unknown else [0, 1, 2]
    value['measurement_assumptions']['coordinate_error_bound_m'] = error
    return value


def face(axis=0, sign=-1, *, center=(1., 2., 3.), size=(.6, .4, .3), offset=(0., 0., 0.), width=.04):
    center = np.array(center) + offset
    normal = np.eye(3)[axis] * sign
    center[axis] = np.array(center)[axis] + sign * size[axis] / 2
    a, b = [i for i in range(3) if i != axis]
    points = np.tile(center, (4, 1))
    points[:, a] += np.array([-1, 1, 1, -1]) * width
    points[:, b] += np.array([-1, -1, 1, 1]) * width
    transform = np.eye(4)
    transform[:3, 3] = center + 2 * normal
    return {'schema_version': 'observed_surface_v1', 'frame_id': 'world', 'volume_status': 'UNKNOWN',
        'face_id': f'face-{axis}-{sign}', 'module_id': 'module', 'source_instance_id': 'instance',
        'capture_id': 'capture', 'capture_time': 1., 'sensor_epoch': 'epoch', 'clock_domain': 'test',
        'calibration_identity': 'explicit-test-calibration', 'T_W_C_at_capture': transform.tolist(),
        'corners_3d_m': points.tolist(), 'plane_normal': normal.tolist(),
        'plane_offset_m': -float(center @ normal), 'plane_residual_m': .0001,
        'point_support_count': 100, 'point_support_ratio': .8, 'evidence': 'registered_metric_depth_plane',
        'boundary_2d_px': [[0, 0], [1, 0], [1, 1], [0, 1]],
        'boundary_kind': 'OBSERVED_PATCH_UNCLASSIFIED',
        'diagnostics': {'physical_corners_certified': False, 'occluded': True}}


def derive(faces, config=None):
    return derive_geometry(faces, config or prior(), enabled=True)


def test_three_independent_local_faces_determine_box_without_physical_corners():
    result = derive([face(i) for i in range(3)])
    assert result['status'] == 'CONSERVATIVE_VOLUME'
    assert result['unique_reconstruction'] and result['independent_directions'] == 3
    box = envelope_to_obb(result['planning_geometry'])
    np.testing.assert_allclose(box.center, [1, 2, 3], atol=1e-10)
    np.testing.assert_allclose(box.half_extents, [.3, .2, .15], atol=1e-10)
    assert not result['planning_admissible'] and not result['candidate_eligible']


def test_single_inscribed_patch_has_continuous_ambiguity_and_requires_axes():
    patch = face(offset=(0, .06, .04))
    config = prior()
    del config['environment_prior']
    assert derive([patch], config)['status'] == 'INSUFFICIENT_INFORMATION'
    result = derive([patch])
    assert result['status'] == 'MULTIPLE_HYPOTHESES' and not result['unique_reconstruction']
    box = envelope_to_obb(result['planning_geometry'])
    assert box.half_extents[1] > .3  # all possible full faces, not patch +/- half size
    assert box.contains([1.3, 2.3, 3.25])
    assert 'CONTINUOUS_FACE_POSITION_OR_SIZE_UNCERTAINTY' in result['ambiguities']
    # Full physical corners are never inferred from patch annotations.
    patch['diagnostics']['physical_corners_certified'] = True
    assert derive([patch])['planning_geometry'] == result['planning_geometry']


def test_unknown_axis_dimensions_retain_all_permutations_and_union():
    result = derive([face()], prior(unknown=True))
    assert len(result['candidates']) == 6
    assert 'MULTIPLE_DIMENSION_AXIS_ASSIGNMENTS' in result['ambiguities']
    union = envelope_to_obb(result['planning_geometry'])
    for candidate in result['candidates']:
        for corner in envelope_to_obb(candidate['envelope']).corners():
            assert union.contains(corner)


@pytest.mark.parametrize('faces', [
    [face(width=.4)],
    [face(0, -1), face(0, 1, offset=(.1, 0, 0))],
    [face(0, -1), face(0, -1, offset=(.02, 0, 0))],
])
def test_conflicting_support_or_plane_position_is_not_moved_or_removed(faces):
    original = deepcopy(faces)
    result = derive(faces, prior(error=.001))
    assert result['status'] == 'PRIOR_OBSERVATION_CONFLICT'
    assert result['planning_geometry'] is None and result['rejected_hypotheses']
    assert faces == original


def test_duplicate_coplanar_patches_are_one_independent_direction():
    result = derive([face(), face(offset=(0, .05, 0))])
    assert result['face_count'] == 2 and result['independent_directions'] == 1
    assert result['status'] == 'MULTIPLE_HYPOTHESES'


@pytest.mark.parametrize('key,value', [
    ('calibration_identity', ''), ('frame_id', 'camera'),
    ('corners_3d_m', [[float('nan'), 0, 0]] * 4),
    ('plane_normal', [2, 0, 0]), ('plane_normal', [float('inf'), 0, 0]),
    ('point_support_count', 0), ('point_support_ratio', 0),
    ('T_W_C_at_capture', np.zeros((4, 4)).tolist()),
    ('corners_3d_m', [[.7, 2, 3]] * 4), ('plane_offset_m', 100),
])
def test_invalid_measurement_is_not_geometry(key, value):
    patch = face()
    patch[key] = value
    result = derive([patch])
    assert result['status'] == 'INVALID_INPUT' and result['planning_geometry'] is None


def test_wrong_normal_sign_and_incompatible_environment_axes():
    patch = face()
    patch['plane_normal'] = [1, 0, 0]
    patch['plane_offset_m'] *= -1
    assert derive([patch])['status'] == 'INVALID_INPUT'
    config = prior()
    a = .1
    config['environment_prior']['rotation_world_from_box_axes'] = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
    assert derive([face()], config)['status'] == 'PRIOR_OBSERVATION_CONFLICT'


@pytest.mark.parametrize('interval', [[[0, 1]] * 3, [[-.1, .1]] * 3, [[2, 1]] * 3, [[1, float('inf')]] * 3])
def test_invalid_dimension_intervals(interval):
    config = prior()
    config['size_prior']['full_dimensions_interval_m'] = interval
    assert derive([face()], config)['status'] == 'INVALID_INPUT'


def test_missing_faces_and_disabled_mode_never_generate_geometry():
    assert derive([])['status'] == 'INSUFFICIENT_INFORMATION'
    assert derive_geometry([face()], prior())['status'] == 'DISABLED'
    config = prior()
    config['enabled'] = False
    assert derive([face()], config)['status'] == 'DISABLED'


def test_rotation_full_size_and_world_center_semantics():
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    config = prior()
    config['environment_prior']['rotation_world_from_box_axes'] = rotation.tolist()
    patches = [face(i) for i in range(3)]
    for p in patches:
        p['corners_3d_m'] = (np.array(p['corners_3d_m']) @ rotation.T).tolist()
        p['plane_normal'] = (rotation @ p['plane_normal']).tolist()
        t = np.array(p['T_W_C_at_capture'])
        t[:3, :3] = rotation
        t[:3, 3] = rotation @ t[:3, 3]
        p['T_W_C_at_capture'] = t.tolist()
    box = envelope_to_obb(derive(patches, config)['planning_geometry'])
    np.testing.assert_allclose(box.center, [-2, 1, 3], atol=1e-10)
    np.testing.assert_allclose(box.half_extents, [.3, .2, .15], atol=1e-10)
    np.testing.assert_allclose(box.rotation, rotation)


def test_envelope_contains_continuous_size_and_translation_extremes():
    config = prior(error=.002)
    config['size_prior']['full_dimensions_interval_m'] = [[.59, .61], [.39, .41], [.29, .31]]
    result = derive([face()], config)
    box = envelope_to_obb(result['planning_geometry'])
    # Independently construct feasible boxes at dimension and face-position extrema.
    for d0 in (.59, .60, .61):
        for d1 in (.39, .405, .41):
            for d2 in (.29, .3, .31):
                for ysign in (-1, 1):
                    for zsign in (-1, 1):
                        low = np.array([.698, 2 + ysign * (d1 / 2 - .038) - d1 / 2,
                                       3 + zsign * (d2 / 2 - .038) - d2 / 2])
                        for bits in np.ndindex(2, 2, 2):
                            assert box.contains(low + np.array(bits) * [d0, d1, d2])
    assert result['status'] == 'MULTIPLE_HYPOTHESES'


def test_artifact_selection_immutability_and_original_online_gates(tmp_path):
    source = tmp_path / 'source'
    ref = artifact_fixture(source)
    observation, _ = load_algorithm_artifact(ref['path'], ref['sha256'])
    before = {str(p): p.read_bytes() for p in source.rglob('*') if p.is_file()}
    selection = {'rule': 'all synthetic fixtures', 'artifact_sha256': ref['sha256'],
                 'source_instance_ids': [c.source_instance_id for c in observation.cargo]}
    result = adapt_artifact(ref['path'], ref['sha256'], prior(), tmp_path / 'derived',
                            enabled=True, selection=selection)
    assert result['source_capture_time'] == observation.capture_time
    assert result['original_unknown_regions'] == to_wire(observation.unknown_regions)
    assert result['source_observation_fingerprint'] == canonical_fingerprint(observation)
    assert not result['planning_admissible'] and result['raw_image_automatic'] is False
    assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in result['blocking_reasons']
    assert result['objects'][0]['raw_observed_surfaces'] == to_wire(observation.cargo[0].observed_surfaces)
    assert before == {str(p): p.read_bytes() for p in source.rglob('*') if p.is_file()}
    replay = algorithm_replay_record(ref['path'], ref['sha256'], 'test')
    update = build_scene_update(replay, replay_display_only=True)
    assert not update.planning_admissible and update.unknown_regions
    assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in update.blocking_reasons
    with pytest.raises(FileExistsError):
        adapt_artifact(ref['path'], ref['sha256'], prior(), tmp_path / 'derived')
    with pytest.raises(ValueError, match='outside'):
        adapt_artifact(ref['path'], ref['sha256'], prior(), source / 'overwrite')


def test_cli_defaults_disabled_and_verifies_hash(tmp_path):
    ref = artifact_fixture(tmp_path / 'source')
    args = ['--artifact', ref['path'], '--artifact-sha256', ref['sha256'], '--prior',
            'configs/perception/nominal_size_prior_offline.json', '--output-directory', str(tmp_path / 'out')]
    assert main(args) == 0
    result = json.loads((tmp_path / 'out' / 'derived_geometry.json').read_text())
    assert not result['experiment_enabled']
    assert all(o['status'] == 'DISABLED' for o in result['objects'])
    args[3] = '0' * 64
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        main(args)
