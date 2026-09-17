"""POC continuation wiring and source-preserving cumulative state contracts."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.layout_single_carton import (
    _build_automatic_trajectory_connector, build_verified_motion_input, load_layout_motion_policy,
)
from unloading_sim.planning_profile import DEFAULT_MOTION
from unloading_sim.post_landing_transport import begin_ideal_transport, advance_ideal_transport
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.workcell_layout import canonical_digest
from test_serial_unloading import actual, scene


@pytest.fixture
def poc_scene(scene):
    data = deepcopy(scene.policy.data)
    data["search_strategy"] = {"post_landing_transport":
        load_layout_motion_policy(DEFAULT_MOTION).data["search_strategy"]["post_landing_transport"],
        "surface_directions_world": {"receiver": [-1., 0., 0.]}}
    return replace(scene, policy=replace(scene.policy, data=data))


def ideal_state(scene, names):
    state = actual(scene)
    state["q_rad"] = scene.policy.layout_validation.initial_q.tolist()
    strategy = scene.policy.data["search_strategy"]
    receivers = {b.name: b for b in scene.fixed_components if b.category == "conveyor"}
    receiver = next(b for b in receivers.values()
                    if np.allclose(strategy["surface_directions_world"][b.name], [-1., 0., 0.]))
    records = {}
    for name in names:
        original = next(b for b in scene.cartons if b.name == name)
        center = receiver.center.copy()
        center[2] += receiver.half_extents[2] + original.half_extents[2] + .02
        box = OBB(center, original.half_extents, np.eye(3), name, "carton")
        records[name] = begin_ideal_transport(box, receiver_name=receiver.name, receivers=receivers,
            directions=strategy["surface_directions_world"], time_s=2.,
            policy=strategy["post_landing_transport"], attachment_removed=True,
            top_contact_observed=False, support_geometry_accepted=False,
            expected_target=name, actual_attachment_observed=True)
        next(r for r in state["cartons"] if r["name"] == name)["position_m"] = center.tolist()
    state.update(receiver_transport_state=records, ideal_received_ids=list(names), processed_carton_ids=list(names))
    return state


@pytest.mark.parametrize("config,event,iterations", [
    (DEFAULT_MOTION, "empty", 600),
    (DEFAULT_MOTION, "ideal", 1800),
    ("configs/validation/m710id70_layout_v1_single_carton.yaml", "actual", 1800),
])
def test_real_production_connector_selects_first_or_continuation_budget(config, event, iterations):
    # No connector, state validator, loader or official robot is substituted.
    original = build_verified_motion_input(load_layout_motion_policy(config))
    name = original.cartons[-1].name
    if event == "ideal":
        state = ideal_state(original, [name])
    else:
        state = actual(original, [name] if event == "actual" else [])
        state["q_rad"] = original.policy.layout_validation.initial_q.tolist()
        if event == "empty":
            state.update(ideal_received_ids=[], processed_carton_ids=[])
    updated = apply_actual_motion_state(original, state)
    built = _build_automatic_trajectory_connector(updated, updated.policy.layout_validation.layout.robot())
    assert built.status == "AVAILABLE"
    assert built.connector.budget.stage_connection_iterations == iterations
    context = updated.snapshot["actual_state_context"]
    if event == "ideal":
        assert context["completed_carton_ids"] == []
        assert context["ideal_received_ids"] == context["processed_carton_ids"] == [name]
        assert name not in updated.removable_cartons
        assert built.connector.budget.planning_wall_time_s is None
    if event == "actual":
        assert "processed_carton_ids" not in state and "ideal_received_ids" not in state
        assert context["completed_carton_ids"] == [name]


def test_connector_rejects_unverified_processed_list():
    original = build_verified_motion_input(load_layout_motion_policy(DEFAULT_MOTION))
    state = actual(original)
    state["q_rad"] = original.policy.layout_validation.initial_q.tolist()
    updated = apply_actual_motion_state(original, state)
    snapshot = deepcopy(updated.snapshot)
    snapshot["actual_state_context"]["processed_carton_ids"] = [original.cartons[-1].name]
    snapshot.pop("scene_fingerprint")
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    forged = replace(updated, snapshot=snapshot)
    with pytest.raises(ValueError, match="processed identities disagree"):
        _build_automatic_trajectory_connector(forged, forged.policy.layout_validation.layout.robot())


def test_ideal_state_is_idempotent_keeps_active_identity_and_allows_new_events(poc_scene):
    names = [poc_scene.cartons[-1].name, poc_scene.cartons[-2].name]
    state = ideal_state(poc_scene, names[:1])
    first = apply_actual_motion_state(poc_scene, state)
    repeated = apply_actual_motion_state(first, deepcopy(state))
    for current in (first, repeated):
        assert len(current.cartons) == 40
        assert names[0] in {b.name for b in current.cartons}
        assert names[0] not in current.remaining_stack_names
        assert names[0] not in current.removable_cartons
        assert current.snapshot["actual_state_context"]["processed_carton_ids"] == names[:1]
    new = ideal_state(poc_scene, names)
    new["time_s"] = 3.
    added = apply_actual_motion_state(repeated, new)
    assert len(added.remaining_stack_names) == 38
    assert added.snapshot["actual_state_context"]["completed_carton_ids"] == []
    assert set(added.snapshot["actual_state_context"]["ideal_received_ids"]) == set(names)


@pytest.mark.parametrize("omit_fields", [False, True])
def test_in_transit_ideal_event_cannot_disappear(poc_scene, omit_fields):
    name = poc_scene.cartons[-1].name
    first = apply_actual_motion_state(poc_scene, ideal_state(poc_scene, [name]))
    lost = actual(first)  # Still has all 40 active bodies, including the received box.
    if not omit_fields:
        lost.update(ideal_received_ids=[], processed_carton_ids=[], receiver_transport_state={})
    with pytest.raises(ValueError, match="ideal reception events must not regress"):
        apply_actual_motion_state(first, lost)


def test_processed_field_cannot_regress_or_be_omitted_for_ideal_events(poc_scene):
    name = poc_scene.cartons[-1].name
    state = ideal_state(poc_scene, [name])
    first = apply_actual_motion_state(poc_scene, state)
    for changed in ([], None, True, [name, name]):
        bad = deepcopy(state)
        bad["processed_carton_ids"] = changed
        with pytest.raises(ValueError, match="processed"):
            apply_actual_motion_state(first, bad)
    bad = deepcopy(state)
    del bad["processed_carton_ids"]
    with pytest.raises(ValueError, match="explicit processed_carton_ids"):
        apply_actual_motion_state(first, bad)


def test_actual_reception_cannot_regress_and_legacy_updates_remain_compatible(scene):
    names = [scene.cartons[-1].name, scene.cartons[-2].name]
    state = actual(scene, names[:1])
    first = apply_actual_motion_state(scene, state)
    repeated = apply_actual_motion_state(first, deepcopy(state))
    with pytest.raises(ValueError, match="actual reception events must not regress"):
        apply_actual_motion_state(repeated, actual(repeated))
    added = apply_actual_motion_state(repeated, actual(scene, names))
    assert set(added.snapshot["actual_state_context"]["completed_carton_ids"]) == set(names)
    # A snapshot written before the new fields existed remains supported.
    snapshot = deepcopy(first.snapshot)
    for key in ("ideal_received_ids", "processed_carton_ids"):
        snapshot["actual_state_context"].pop(key)
    snapshot.pop("scene_fingerprint")
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    legacy = apply_actual_motion_state(replace(first, snapshot=snapshot), state)
    assert legacy.snapshot["actual_state_context"]["processed_carton_ids"] == names[:1]


@pytest.mark.parametrize("initial_source", ["actual", "ideal"])
def test_reception_source_cannot_be_rewritten(poc_scene, initial_source):
    name = poc_scene.cartons[-1].name
    measured, assumed = actual(poc_scene, [name]), ideal_state(poc_scene, [name])
    before, after = (measured, assumed) if initial_source == "actual" else (assumed, measured)
    first = apply_actual_motion_state(poc_scene, before)
    with pytest.raises(ValueError, match="reception event source must not change"):
        apply_actual_motion_state(first, after)


def test_outfed_events_cannot_regress_or_resurrect(poc_scene):
    name = poc_scene.cartons[-1].name
    state = ideal_state(poc_scene, [name])
    in_transit = deepcopy(state)
    event = advance_ideal_transport(state["receiver_transport_state"][name], dt_s=20., speed_m_s=.3)
    assert event is not None
    state["handed_off_ids"] = [name]
    state["cartons"] = [r for r in state["cartons"] if r["name"] != name]
    first = apply_actual_motion_state(poc_scene, state)
    repeated = apply_actual_motion_state(first, deepcopy(state))
    assert len(repeated.cartons) == 39
    with pytest.raises(ValueError, match="handoff events must not regress"):
        apply_actual_motion_state(first, in_transit)
    resurrected = deepcopy(state)
    resurrected["cartons"] = in_transit["cartons"]
    with pytest.raises(ValueError, match="identity mismatch"):
        apply_actual_motion_state(first, resurrected)
