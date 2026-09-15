import copy

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.post_landing_transport import (
    LANDED, OUTFED, advance_ideal_transport, begin_ideal_transport,
    ideal_transport_ids,
)


POLICY = {
    "mode": "ideal_outfeed", "tail_protection_enabled": False,
    "evaluate_tipping": False,
    "evaluate_post_landing_environment_collisions": False,
    "downstream_capacity": "assumed_sufficient",
    "output_plane": {"name": "trailer_opening_x", "direction": "-X", "x_m": -3.2},
}


def begin(receiver="transverse", **gates):
    # A quarter-turn carton has 0.2 m world-X half extent, not its local 0.3 m.
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    box = OBB([-0.5, 0.6, 1.0], [0.3, 0.2, 0.25], rotation, "selected", "carton")
    receivers = {
        "transverse": OBB([-0.5, 0., 0.5], [0.4, 1., 0.1], np.eye(3), "transverse"),
        "longitudinal": OBB([-1.5, -0.6, 0.5], [1., 0.4, 0.1], np.eye(3), "longitudinal"),
    }
    proof = dict(attachment_removed=True, top_contact_observed=True, support_geometry_accepted=True)
    proof.update(gates)
    return begin_ideal_transport(box, receiver_name=receiver, receivers=receivers,
        directions={"transverse": [0., -1., 0.], "longitudinal": [-1., 0., 0.]},
        time_s=10., policy=POLICY, **proof)


def test_takeover_requires_each_actual_release_and_landing_gate():
    for gate in ("attachment_removed", "top_contact_observed", "support_geometry_accepted"):
        with pytest.raises(ValueError, match="actual release"):
            begin(**{gate: False})
    record = begin()
    assert ideal_transport_ids(POLICY, {"selected": record}) == {"selected"}
    with pytest.raises(ValueError):
        ideal_transport_ids({"mode": "strict_physics"}, {"selected": record})
    with pytest.raises(ValueError):
        ideal_transport_ids(POLICY, {"different_carton": record})


def test_transverse_corner_and_exit_use_simulation_time_and_full_rotated_envelope():
    record = begin()
    before = copy.deepcopy(record)
    assert advance_ideal_transport(record, dt_s=0., speed_m_s=0.3) is None
    assert record == before  # Offline planning cannot advance the cargo.
    advance_ideal_transport(record, dt_s=4., speed_m_s=0.3)
    np.testing.assert_allclose(np.asarray(record["pose_world"])[:3, 3], [-0.5, -0.6, 1.0])
    advance_ideal_transport(record, dt_s=9., speed_m_s=0.3)
    assert record["state"] == LANDED  # Centre at -3.2 is not a full-body handoff.
    event = advance_ideal_transport(record, dt_s=0.67, speed_m_s=0.3)
    assert event["event"] == OUTFED
    assert event["full_envelope_max_x_m"] == pytest.approx(-3.201)
    assert record["time_s"] == pytest.approx(23.67)
    np.testing.assert_array_equal(np.asarray(record["pose_world"])[:3, :3],
                                  np.asarray(record["takeover_pose_world"])[:3, :3])
    assert not record["post_landing_physics_qualified"]


def test_direct_longitudinal_reception_does_not_route_back_to_transverse():
    record = begin("longitudinal")
    np.testing.assert_allclose(record["route_world_m"], [[-3.401, 0.6, 1.0]])
    advance_ideal_transport(record, dt_s=1., speed_m_s=0.3)
    np.testing.assert_allclose(np.asarray(record["pose_world"])[:3, 3], [-0.8, 0.6, 1.0])
