import numpy as np

from unloading_sim.geometry import OBB, rotation_matrix_from_rotation_vector
from unloading_sim.layout_trajectory import possible_inflated_obb_pairs


def test_inflated_rotated_broadphase_retains_every_sat_collision():
    rng = np.random.default_rng(71070)
    boxes = [OBB(rng.uniform(-0.3, 0.3, 3), rng.uniform(0.005, 0.2, 3),
                 rotation_matrix_from_rotation_vector(rng.normal(size=3)), str(i))
             for i in range(32)]
    for margin in (0.0, 0.01):
        candidates = set(possible_inflated_obb_pairs(boxes, boxes, margin))
        for i, first in enumerate(boxes):
            for j, second in enumerate(boxes):
                if first.intersects_obb(second, margin=margin):
                    assert (i, j) in candidates
        assert len(candidates) < len(boxes) ** 2


def test_broadphase_retains_engineering_margin_and_handles_empty_sets():
    a = OBB(np.zeros(3), np.full(3, 0.1), np.eye(3), "a")
    b = OBB(np.array([0.219, 0, 0]), np.full(3, 0.1), np.eye(3), "b")
    assert possible_inflated_obb_pairs([a], [b], 0.01) == [(0, 0)]
    assert possible_inflated_obb_pairs([], [b], 0.01) == []
