"""A user timeout exception never fabricates actual geometric clearance."""
from unloading_sim.m710_replay_physics import BoundedFreeTransitGate


def test_default_still_stops_after_finite_wait():
    gate = BoundedFreeTransitGate(3., 1.)
    assert gate.evaluate(3., 5., False) == {"hold": True, "reason": None}
    assert gate.evaluate(3., 6., False)["reason"] == "ACTUAL_PAYLOAD_NOT_CLEAR_FOR_FREE_TRANSIT"
    assert not gate.passed and not gate.timeout_waived


def test_waiver_releases_command_hold_without_claiming_clearance():
    gate = BoundedFreeTransitGate(3., 1., waive_timeout=True)
    assert gate.advance(2.9, .2, 10.) == 3.
    assert gate.evaluate(3., 5., False)["hold"]
    assert gate.evaluate(3., 6., False) == {"hold": False, "reason": None}
    assert gate.timeout_waived and not gate.passed
    assert gate.advance(3., .2, 10.) == 3.2
    assert gate.evaluate(3.2, 6.2, False) == {"hold": False, "reason": None}
    assert not gate.passed
    assert [e["event"] for e in gate.events].count("USER_WAIVED_FREE_TRANSIT_TIMEOUT") == 1
    assert gate.evaluate(3.4, 6.4, True) == {"hold": False, "reason": None}
    assert gate.passed
    assert gate.events[-1]["event"] == "actual_free_transit_gate_passed"
