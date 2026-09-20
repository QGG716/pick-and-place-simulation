import numpy as np
import pytest

from unloading_sim.geometry import make_transform, rotation_matrix_from_rpy
from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.layout_single_carton import load_layout_motion_policy
from unloading_sim.planning_profile import DEFAULT_MOTION


def test_motion_certificate_reuses_clear_pairs_but_detects_approach_and_rotation():
    coal = pytest.importorskip('coal')
    backend = object.__new__(PinocchioHppFclBackend)
    backend.coal = coal
    policy = SimulationCollisionPolicy.from_mapping(
        load_layout_motion_policy(DEFAULT_MOTION).layout_validation.data['collision_policy'])
    first, second = coal.Box(.2, .2, .2), coal.Box(.2, .4, .2)
    original_distance = backend._distance
    calls = []
    def counted(*args):
        calls.append(1)
        return original_distance(*args)
    backend._distance = counted
    origin = np.eye(4)
    def query(pose, required=.005, reuse=True, shape=second):
        backend.clearance_motion_cache_enabled = reuse
        return backend._poc_pair_result(first, coal.Transform3s(np.eye(3), np.zeros(3)),
            shape, coal.Transform3s(pose[:3, :3], pose[:3, 3]), required, policy,
            ('robot', 'tool'), 'transit', poses=(origin, pose))
    initial = make_transform(translation=[.23, 0, 0])
    assert not query(initial).in_collision
    exact_count = len(calls)
    assert not query(make_transform(translation=[.229, 0, 0])).in_collision
    assert len(calls) == exact_count
    assert backend.performance_counters['rigid_motion_clearance_certificates'] == 1
    # Dense inward steps must not repeatedly reset the original distance.
    for x in np.linspace(.228, .198, 81):
        pose = make_transform(translation=[x, 0, 0])
        assert query(pose).in_collision == query(pose, reuse=False).in_collision
    for angle in np.linspace(0, 1.1, 41):
        pose = make_transform(rotation_matrix_from_rpy(0, 0, angle), [.25, 0, 0])
        assert query(pose).in_collision == query(pose, reuse=False).in_collision
    assert query(initial, required=.04).in_collision
    assert query(initial, shape=coal.Box(.4, .4, .4)).in_collision
    # Changing geometry generation cannot reuse a prior certificate.
    query(initial)
    exact_count = len(calls)
    backend.geometry_revision = 1
    query(initial)
    assert len(calls) == exact_count + 1


def test_unknown_exact_query_is_not_cached_as_clear():
    coal = pytest.importorskip('coal')
    backend = object.__new__(PinocchioHppFclBackend)
    backend.coal = coal
    policy = SimulationCollisionPolicy.from_mapping(
        load_layout_motion_policy(DEFAULT_MOTION).layout_validation.data['collision_policy'])
    backend._distance = lambda *args: float('nan')
    first, second = coal.Box(.2, .2, .2), coal.Box(.2, .2, .2)
    a, b = np.eye(4), make_transform(translation=[.3, 0, 0])
    for _ in range(2):
        result = backend._poc_pair_result(first, coal.Transform3s(a[:3, :3], a[:3, 3]),
            second, coal.Transform3s(b[:3, :3], b[:3, 3]), .005, policy,
            ('a', 'b'), 'transit', poses=(a, b))
        assert result.in_collision and result.evidence['classification'] == 'UNKNOWN'
    assert not backend._clearance_motion_cache
