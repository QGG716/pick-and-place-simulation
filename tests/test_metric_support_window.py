"""Exact support/audit equivalence to the pre-optimization full-frame operation."""
from copy import deepcopy
import numpy as np
import pytest

from unloading_perception.metric_support import observation_support
from unloading_perception.rgbd import _erode_mask


def full_frame_reference(depth, instance, face, erosion_px=2, discontinuity_m=.02):
    # Frozen 02a20ad reference, kept only in tests. No production cache/context.
    selected = instance & face
    valid = np.isfinite(depth) & (depth > 0)
    interior = _erode_mask(selected, erosion_px)
    jumps = np.zeros(depth.shape, bool)
    for axis in (0, 1):
        a, b = [slice(None)]*2, [slice(None)]*2
        a[axis], b[axis] = slice(1, None), slice(None, -1)
        a, b = tuple(a), tuple(b)
        difference = np.zeros(depth[a].shape, dtype=float)
        np.subtract(depth[a], depth[b], out=difference, where=valid[a] & valid[b])
        bad = (~valid[a] | ~valid[b] | (np.abs(difference) > discontinuity_m))
        jumps[a] |= bad; jumps[b] |= bad
    retained = selected & valid & interior & ~jumps
    return selected & valid, retained, {
        'selection': 'FROZEN_OBSERVATION_FACE_LABEL_INTERSECT_INSTANCE',
        'candidate_plane_used_for_selection': False, 'selected_pixels': int(selected.sum()),
        'excluded_invalid': int((selected & ~valid).sum()),
        'excluded_boundary': int((selected & valid & ~interior).sum()),
        'excluded_discontinuity': int((selected & valid & interior & jumps).sum()),
        'retained_pixels': int(retained.sum())}


def exact(depth, instance, face, erosion=2, threshold=.02):
    originals = [v.copy() for v in (depth, instance, face)]
    expected = full_frame_reference(depth, instance, face, erosion, threshold)
    actual = observation_support(depth, instance, face, erosion_px=erosion, discontinuity_m=threshold)
    for a, b in zip(actual[:2], expected[:2]):
        assert a.dtype == b.dtype == np.bool_ and a.shape == depth.shape
        np.testing.assert_array_equal(a, b)
    assert actual[2] == expected[2]
    for a, b in zip((depth, instance, face), originals): np.testing.assert_array_equal(a, b)
    return actual


@pytest.mark.parametrize('shape', [(0, 0), (0, 5), (5, 0), (1, 1), (1, 13), (17, 1), (24, 31)])
@pytest.mark.parametrize('erosion', [0, 1, 2, 5])
def test_empty_full_edges_holes_disconnected_and_thin_regions(shape, erosion):
    depth = np.full(shape, 2., np.float32)
    mask = np.ones(shape, bool)
    exact(depth, mask, mask, erosion)
    exact(depth, mask, ~mask, erosion)
    if shape == (24, 31):
        face = np.zeros(shape, bool); face[0:9, 0:8] = True; face[15:24, 20:31] = True
        face[3:5, 3:5] = False
        exact(depth, mask, face, erosion)


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_fixed_seed_random_masks_depths_and_strided_inputs(dtype):
    rng = np.random.default_rng(571823)
    for trial in range(80):
        depth = rng.choice([0., -1., np.nan, np.inf, -np.inf, 2., 2.01, 2.02, 2.02001], size=(29, 37)).astype(dtype)
        instance = rng.random(depth.shape) > .15
        face = np.zeros(depth.shape, bool)
        y0, y1 = sorted(rng.integers(0, 30, size=2)); x0, x1 = sorted(rng.integers(0, 38, size=2))
        face[y0:y1, x0:x1] = rng.random((y1-y0, x1-x0)) > .2
        if trial % 2: depth, instance, face = depth[::-1, ::2], instance[::-1, ::2], face[::-1, ::2]
        exact(depth, instance, face, trial % 5)


def test_one_pixel_halo_uses_depth_outside_instance_and_exact_threshold():
    mask = np.zeros((12, 16), bool); mask[4:8, 5:11] = True
    depth = np.full(mask.shape, 2.)
    # Selected support's neighbour need not be selected, or even valid.
    depth[3, 5:11] = np.nan
    depth[8, 5:11] = 2.25
    raw, retained, audit = exact(depth, mask, mask, 0, .25)
    assert raw[4, 7] and not retained[4, 7] and retained[7, 7]
    depth[8, 7] = np.nextafter(2.25, np.inf)
    assert not exact(depth, mask, mask, 0, .25)[1][7, 7]


def test_work_window_is_bounded_without_cropping_outputs(monkeypatch):
    import unloading_perception.rgbd as rgbd
    sizes = []
    def erode(mask, count):
        sizes.append(mask.shape)
        return _erode_mask(mask, count)
    monkeypatch.setattr(rgbd, '_erode_mask', erode)
    depth = np.ones((300, 400), np.float32)
    mask = np.zeros(depth.shape, bool); mask[80:100, 120:150] = True
    result = observation_support(depth, mask, mask)
    assert sizes == [(22, 32)] and result[1].shape == (300, 400)
    assert result[2]['retained_pixels'] == 16*26


@pytest.mark.parametrize('mutation', ['face', 'depth', 'K', 'mask', 'frozen_support'])
def test_independent_validation_never_reuses_old_pass_after_mutation(mutation):
    from test_metric_patch_adapter import fixture
    from unloading_perception.final_geometry import validate_final_record
    record, depth, mask, K = fixture()
    assert len(validate_final_record(record, depth, mask, K)['camera_facing_faces']) == 1
    if mutation == 'face':
        record['camera_facing_faces'][0]['corners_3d_m'][0][2] += .05
    elif mutation == 'depth': depth[20, 20] += .1
    elif mutation == 'K': K[0, 0] += 1
    elif mutation == 'mask': mask[20, 20] = False
    else: record['frozen_support_regions']['0'] = [[20, 20, 25]]
    if mutation in ('depth', 'K', 'mask'):
        with pytest.raises(ValueError, match='BINDING'):
            validate_final_record(record, depth, mask, K)
    else:
        checked = validate_final_record(record, depth, mask, K)
        assert not checked['camera_facing_faces'] and checked['final_face_validation'][0]['status'] == 'REJECTED'
