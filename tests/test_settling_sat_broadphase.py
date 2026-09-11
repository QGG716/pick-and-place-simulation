from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.m710_replay_physics import obb_aabb_definitely_separated, obb_penetration_depth


def test_randomized_broadphase_never_discards_original_sat_penetration():
    rng = np.random.default_rng(20260911)
    rejected = 0
    for _ in range(500):
        rotations = []
        for _ in range(2):
            rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
            rotation[:, 0] *= np.linalg.det(rotation)
            rotations.append(rotation)
        args = (rng.uniform(-2, 2, 3), rng.uniform(0.02, 0.5, 3), rotations[0],
                rng.uniform(-2, 2, 3), rng.uniform(0.02, 0.5, 3), rotations[1])
        original = obb_penetration_depth(*args, use_aabb_broadphase=False)
        if obb_aabb_definitely_separated(*args):
            rejected += 1
            assert original == 0.0
        assert obb_penetration_depth(*args) == original
    assert rejected > 350


@pytest.mark.parametrize("offset", [0.0, 1e-15, -1e-15, -0.0005, -0.02])
def test_touch_near_touch_and_penetration_keep_original_sat_exactly(offset):
    args = (np.zeros(3), np.array([0.3, 0.2, 0.15]), np.eye(3),
            np.array([0.6 + offset, 0, 0]), np.array([0.3, 0.2, 0.15]), np.eye(3))
    assert not obb_aabb_definitely_separated(*args)
    assert obb_penetration_depth(*args) == obb_penetration_depth(*args, use_aabb_broadphase=False)


def test_outward_epsilon_scales_with_world_coordinates():
    center = np.array([1e6, 1e6, 1e6])
    args = (center, np.array([0.3, 0.2, 0.15]), np.eye(3),
            center + [0.6 + 1e-9, 0, 0], np.array([0.3, 0.2, 0.15]), np.eye(3))
    assert not obb_aabb_definitely_separated(*args)
    assert obb_penetration_depth(*args) == obb_penetration_depth(*args, use_aabb_broadphase=False)
