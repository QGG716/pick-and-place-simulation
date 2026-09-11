from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.pinocchio_backend import PinocchioHppFclBackend, _local_geometry_aabb


def _backend(size=(1.0, 1.0, 1.0), rotation=None):
    coal = pytest.importorskip("coal")
    backend = object.__new__(PinocchioHppFclBackend)
    backend.coal = coal
    backend.base_transform = np.eye(4)
    backend.model = SimpleNamespace(frames=[SimpleNamespace(name="test_link")])
    geometry = coal.Box(*size)
    backend.geometry_model = SimpleNamespace(geometryObjects=[SimpleNamespace(name="mesh", parentFrame=0, geometry=geometry)])
    backend.geometry_data = SimpleNamespace(oMg=[SimpleNamespace(rotation=np.eye(3) if rotation is None else rotation, translation=np.zeros(3))])
    backend._geometry_local_aabbs = (_local_geometry_aabb(geometry),)
    backend.exact_call_count = 0
    exact = backend._distance

    def counted(*args):
        backend.exact_call_count += 1
        return exact(*args)

    backend._distance = counted
    return backend


def _check(backend, obstacles, margin=0.02):
    return backend._environment_collision(obstacles, set(), margin, set())


def test_far_obstacles_skip_exact_queries_but_missing_bounds_fall_back():
    backend = _backend()
    obstacle = OBB([5, 0, 0], [0.1, 0.1, 0.1], np.eye(3), "far")
    assert not _check(backend, [obstacle]).in_collision
    assert backend.exact_call_count == 0
    backend._geometry_local_aabbs = (None,)
    assert not _check(backend, [obstacle]).in_collision
    assert backend.exact_call_count == 1
    assert _local_geometry_aabb(object()) is None


def test_rotated_robot_bounds_do_not_skip_near_margin():
    backend = _backend((1.0, 4.0, 1.0), rotation_matrix_from_rpy(0, 0, np.pi / 2))
    obstacle = OBB([2.115, 0, 0], [0.1, 0.1, 0.1], np.eye(3), "within_twenty_mm")
    assert _check(backend, [obstacle]).in_collision
    assert backend.exact_call_count == 1


def test_rotated_obstacle_bounds_do_not_skip_near_margin():
    backend = _backend()
    obstacle = OBB([2.015, 0, 0], [0.1, 1.5, 0.1], rotation_matrix_from_rpy(0, 0, np.pi / 2), "rotated_near")
    assert _check(backend, [obstacle]).in_collision
    assert backend.exact_call_count == 1


def test_seeded_rotated_box_states_match_all_exact_results():
    backend = _backend((0.4, 0.9, 0.3), rotation_matrix_from_rpy(0.2, 0.4, 0.7))
    cached = backend._geometry_local_aabbs
    rng = np.random.default_rng(710)
    for index in range(48):
        obstacle = OBB(rng.uniform(-1.2, 1.2, 3), rng.uniform(0.02, 0.2, 3), rotation_matrix_from_rpy(*rng.uniform(-np.pi, np.pi, 3)), f"box_{index}")
        backend._geometry_local_aabbs = cached
        optimized = _check(backend, [obstacle])
        backend._geometry_local_aabbs = (None,)
        reference = _check(backend, [obstacle])
        assert optimized == reference
