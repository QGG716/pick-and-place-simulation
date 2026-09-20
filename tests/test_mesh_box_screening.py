"""Simplified enclosing-box screening cannot skip a near mesh pair."""
import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.pinocchio_backend import _separated_from_local_box
from unloading_sim.pair_clearance import obb_surface_distance


def test_rotated_thin_boxes_with_overlapping_world_aabbs_are_screened():
    rotation = rotation_matrix_from_rpy(0, 0, np.pi/4)
    half = np.array([1., .02, .02])
    obstacle = OBB(rotation @ [0., .06, 0.], half.copy(), rotation.copy())
    assert _separated_from_local_box((-half, half), rotation, np.zeros(3), [obstacle], .005).tolist() == [True]


@pytest.mark.parametrize('gap,expected', [(0., False), (.0049, False), (.005, False), (.0050000001, True), (.006, True)])
def test_literal_clearance_boundary(gap, expected):
    half = np.array([.3, .2, .15])
    obstacle = OBB([.6+gap, 0., 0.], half, np.eye(3))
    assert bool(_separated_from_local_box((-half, half), np.eye(3), np.zeros(3), [obstacle], .005)[0]) is expected


def test_seeded_screening_never_skips_pair_inside_euclidean_clearance():
    rng = np.random.default_rng(20260920)
    for _ in range(160):
        half = rng.uniform(.01, .6, 3)
        rotation = rotation_matrix_from_rpy(*rng.uniform(-3, 3, 3))
        center = rng.uniform(-1, 1, 3)
        first = OBB(center, half, rotation)
        obstacles = [OBB(center+rng.uniform(-1, 1, 3), rng.uniform(.01, .6, 3),
                         rotation_matrix_from_rpy(*rng.uniform(-3, 3, 3))) for _ in range(4)]
        screened = _separated_from_local_box((-half, half), rotation, center, obstacles, .005)
        for skipped, other in zip(screened, obstacles):
            if skipped:
                assert obb_surface_distance(first, other) > .005
