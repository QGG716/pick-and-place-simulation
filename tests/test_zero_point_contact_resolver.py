from __future__ import annotations

import pytest

from unloading_sim.isaac_collision_policy import ZeroPointContactResolver


KEY = ("/Robot/J6", "/Scene/target", "/Robot/J6/Tool/SoftCup17", "/Scene/target")
SECOND = ("/Robot/J4", "/Scene/wall", "/Robot/J4/collision", "/Scene/wall")
CONTACT = ("contact", "/Scene/target", False, False)
EXTRACTION = ("extraction", "/Scene/target", True, False)


def test_empty_found_resolves_only_on_finite_persist_without_granting_permission():
    resolver = ZeroPointContactResolver()
    assert resolver.observe(KEY, [], False, CONTACT) == "pending"
    assert resolver.pending_keys == (KEY,)
    assert resolver.unresolved_outside(CONTACT) == ()
    assert resolver.observe(KEY, [.02, .01], False, CONTACT) == "resolved"
    assert resolver.pending_keys == ()
    assert resolver.snapshot()["grants_contact_permission"] is False


def test_lost_clears_only_its_exact_collider_key():
    resolver = ZeroPointContactResolver()
    same_actor_other_shape = (*KEY[:2], "/Robot/J6/Tool/SoftCup18", KEY[3])
    resolver.observe(KEY, [], False, CONTACT)
    resolver.observe(same_actor_other_shape, [], False, CONTACT)
    assert resolver.observe(KEY, [], True, EXTRACTION) == "lost"
    assert resolver.pending_keys == (same_actor_other_shape,)
    assert resolver.observe(KEY, [], True, EXTRACTION) == "lost"


def test_scope_change_and_run_end_preserve_all_unresolved_keys():
    resolver = ZeroPointContactResolver()
    resolver.observe(KEY, [], False, CONTACT)
    resolver.observe(SECOND, [], False, EXTRACTION)
    assert resolver.unresolved_outside(EXTRACTION) == (KEY,)
    # Another empty header must not move the first pending event to a new scope.
    resolver.observe(KEY, [], False, EXTRACTION)
    assert resolver.unresolved_outside(EXTRACTION) == (KEY,)
    snapshot = resolver.snapshot()
    assert snapshot["pending_count"] == 2
    assert snapshot["unresolved_scope_exit_or_run_end"] == "FAIL_CLOSED"
    assert {tuple(item["key"]) for item in snapshot["pending"]} == {KEY, SECOND}


@pytest.mark.parametrize("values", [[float("nan")], [0, float("inf")], [-float("inf")], [None], None])
def test_nonfinite_or_malformed_measurement_is_invalid_not_finite_evidence(values):
    resolver = ZeroPointContactResolver()
    resolver.observe(KEY, [], False, CONTACT)
    assert resolver.observe(KEY, values, False, CONTACT) == "invalid"
    assert resolver.pending_keys == (KEY,)


def test_rigid_pair_empty_header_is_not_allowed_and_finite_penetration_needs_classifier():
    resolver = ZeroPointContactResolver()
    assert resolver.observe(SECOND, [], False, CONTACT) == "pending"
    assert resolver.observe(SECOND, [-.02], False, CONTACT) == "resolved"
    assert resolver.snapshot()["grants_contact_permission"] is False


def test_snapshot_and_scope_input_do_not_share_mutable_pending_state():
    resolver = ZeroPointContactResolver()
    scope = ["contact", "target"]
    resolver.observe(KEY, [], False, scope)
    scope[0] = "withdrawal"
    assert resolver.unresolved_outside(scope) == (KEY,)
    snapshot = resolver.snapshot()
    snapshot["pending"][0]["scope_token"][0] = "mutated"
    assert resolver.unresolved_outside(["contact", "target"]) == ()

