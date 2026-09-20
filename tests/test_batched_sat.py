"""Batched box screening must preserve the scalar separating-axis predicate."""
import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rpy


def scalar_sat(first, second):
    axes = [first.rotation[:, i] for i in range(3)]
    axes += [second.rotation[:, i] for i in range(3)]
    for a in first.rotation.T:
        for b in second.rotation.T:
            axis = np.cross(a, b)
            norm = np.linalg.norm(axis)
            if norm > 1e-12:
                axes.append(axis / norm)
    gaps = []
    for raw in axes:
        axis = raw / np.linalg.norm(raw)
        radius = (first.half_extents @ np.abs(first.rotation.T @ axis)
                  + second.half_extents @ np.abs(second.rotation.T @ axis))
        gaps.append(abs((second.center - first.center) @ axis) - radius)
    return max(gaps)


@pytest.mark.parametrize("gap", [-.003, 0., .004999, .005, .0052, .006])
def test_literal_gaps_and_frozen_inputs(gap):
    first = OBB(np.zeros(3), np.array([.3, .2, .15]), np.eye(3))
    second = OBB(np.array([.6 + gap, 0., 0.]), first.half_extents.copy(), np.eye(3))
    for box in (first, second):
        for value in (box.center, box.half_extents, box.rotation):
            value.setflags(write=False)
    assert first.signed_distance_obb(second) == pytest.approx(gap, abs=2e-15)
    np.testing.assert_array_equal(first.rotation, np.eye(3))


def test_all_axes_match_scalar_for_seeded_skew_and_near_parallel_boxes():
    rng = np.random.default_rng(20260920)
    for index in range(240):
        rotation = rotation_matrix_from_rpy(*rng.uniform(-np.pi, np.pi, 3))
        other_rotation = (rotation @ rotation_matrix_from_rpy(10. ** (-14 + index % 9), 0., 0.)
                          if index % 3 == 0 else rotation_matrix_from_rpy(*rng.uniform(-np.pi, np.pi, 3)))
        first = OBB(rng.uniform(-2, 2, 3), rng.uniform(.01, .7, 3), rotation)
        second = OBB(first.center + rng.uniform(-1, 1, 3), rng.uniform(.01, .7, 3), other_rotation)
        expected = scalar_sat(first, second)
        actual = first.signed_distance_obb(second)
        assert actual == pytest.approx(expected, abs=2e-14)
        assert (actual > 0) == (expected > 0)


def test_corner_results_are_owned_and_keep_original_order():
    box = OBB(np.zeros(3), np.array([1., 2., 3.]), np.eye(3))
    corners = box.corners()
    np.testing.assert_array_equal(corners[0], [-1., -2., -3.])
    np.testing.assert_array_equal(corners[-1], [1., 2., 3.])
    corners[:] = 99
    np.testing.assert_array_equal(box.corners()[0], [-1., -2., -3.])
