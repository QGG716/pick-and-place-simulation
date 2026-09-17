"""Distance queries must not rewrite the actual scene they are validating."""
import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.pair_clearance import obb_surface_distance


# carton_l03_c00 from the original fifth-carton actual-state quaternion.
# Normalizing a column in place changes these valid rotation entries by one ULP.
ARCHIVED_ROTATION = np.array([
    [.9999999999955079, -9.167632368966419e-7, -2.8537012742113936e-6],
    [9.167801282204824e-7, .9999999999820619, 5.91909753135287e-6],
    [2.8536958477491915e-6, -5.919100147542902e-6, .9999999999784104],
])


def boxes(readonly=False):
    first = OBB([0., 0., 0.], [.3, .2, .15], ARCHIVED_ROTATION.copy(), "first")
    second = OBB([.605, 0., 0.], [.3, .2, .15], np.eye(3), "second")
    if readonly:
        first.rotation.setflags(write=False)
        second.rotation.setflags(write=False)
    return first, second


def state_bytes(items):
    return [(b.center.tobytes(), b.half_extents.tobytes(), b.rotation.tobytes()) for b in items]


@pytest.mark.parametrize("readonly", [False, True])
@pytest.mark.parametrize("query", [OBB.signed_distance_obb, obb_surface_distance])
def test_distance_queries_preserve_both_frozen_boxes(query, readonly):
    items = boxes(readonly)
    before = state_bytes(items)
    first = query(*items)
    assert np.isfinite(first) and first > 0
    assert state_bytes(items) == before
    assert query(*items) == first
    assert state_bytes(items) == before


def test_production_state_query_keeps_validation_context_identity():
    from test_search_diagnostics import connector
    items = boxes()
    def validate(q, obstacles, **kwargs):
        distance = obb_surface_distance(*obstacles)
        if distance < .005:
            return dict(reason="CLEARANCE_INSUFFICIENT", pair=[b.name for b in obstacles],
                        surface_distance_m=distance, required_pair_clearance_m=.005)
    c = connector(validate)
    before = c._context_identity(items, stage="pregrasp")
    failure = c.validate_unloaded_state(np.zeros(6), items, stage="pregrasp")
    assert failure["reason"] == "CLEARANCE_INSUFFICIENT"
    assert c._context_identity(items, stage="pregrasp") == before
