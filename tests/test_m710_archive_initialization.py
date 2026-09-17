"""Archive events initialize a new world without inventing new completions."""
from copy import deepcopy

import pytest

from unloading_sim.m710_replay_physics import archived_replay_initialization, archived_world_actual_state
from unloading_sim.serial_unloading import apply_actual_motion_state
from test_serial_unloading import scene, actual
from test_poc_continuation_state import poc_scene, ideal_state


def metadata_for(updated):
    context = deepcopy(updated.snapshot["actual_state_context"])
    return dict(initial_actual_state_context=context,
        joint_names=updated.snapshot["robot"]["joint_names"],
        post_landing_transport=updated.policy.data["search_strategy"]["post_landing_transport"],
        scene_primitives=[dict(name=b.name, dynamic=True) for b in updated.cartons],
        target=next(name for name in updated.remaining_stack_names),
        **{k: deepcopy(context[k]) for k in ("completed_carton_ids", "processed_carton_ids",
                                            "handed_off_ids", "receiver_transport_state")})


def test_archive_ideal_context_keeps_active_body_and_does_not_mutate_bound_metadata(poc_scene):
    name = poc_scene.cartons[-1].name
    updated = apply_actual_motion_state(poc_scene, ideal_state(poc_scene, [name]))
    metadata = metadata_for(updated)
    before = deepcopy(metadata)
    context = archived_replay_initialization(metadata)
    assert context["active_ideal_transport_ids"] == [name]
    assert context["historical_counts"] == dict(actual_received=0, ideal_received=1, processed=1, outfed=0)
    assert context["world_scope"] == "NEW_WORLD_RECONSTRUCTED_FROM_ARCHIVED_ACTUAL_STATE"
    context["receiver_transport_state"][name]["time_s"] += 1.
    assert metadata == before


def test_old_physical_reception_stays_physical_in_new_poc_world(poc_scene):
    name = poc_scene.cartons[-1].name
    state = actual(poc_scene, [name])  # Old schema has no ideal/processed lists.
    updated = apply_actual_motion_state(poc_scene, state)
    context = archived_replay_initialization(metadata_for(updated))
    assert context["historical_counts"] == dict(actual_received=1, ideal_received=0, processed=1, outfed=0)
    assert context["completed_carton_ids"] == [name]
    assert context["active_ideal_transport_ids"] == []


@pytest.mark.parametrize("damage", ["missing_context", "source", "fingerprint", "events",
                                    "active_identity", "velocity", "already_processed_target", "transport"])
def test_archive_initialization_fails_closed_on_lost_identity_or_provenance(poc_scene, damage):
    name = poc_scene.cartons[-1].name
    metadata = metadata_for(apply_actual_motion_state(poc_scene, ideal_state(poc_scene, [name])))
    context = metadata["initial_actual_state_context"]
    if damage == "missing_context":
        metadata.pop("initial_actual_state_context")
    elif damage == "source":
        context["source"] = "CPU_PLANNED_ROLLOUT_NOT_PHYSICAL"
    elif damage == "fingerprint":
        context["actual_state_fingerprint"] = "unknown"
    elif damage == "events":
        metadata["processed_carton_ids"] = []
    elif damage == "active_identity":
        metadata["scene_primitives"].pop()
    elif damage == "velocity":
        context["carton_velocities"].pop(name)
    elif damage == "already_processed_target":
        metadata["target"] = name
    elif damage == "transport":
        metadata["receiver_transport_state"] = {}
    with pytest.raises(ValueError):
        archived_replay_initialization(metadata)


def test_first_world_without_archive_remains_supported():
    assert archived_replay_initialization({}) is None


def test_archive_context_cannot_be_added_by_rehashing_only_the_bundle():
    from test_m710_replay_contract import _ready_preflight, _bundle
    from unloading_sim.m710_replay_contract import add_bundle_payload_sha256, verify_m710_replay_bundle
    bundle = _bundle(_ready_preflight())
    bundle["metadata"]["initial_actual_state_context"] = {"completed_carton_ids": ["forged"]}
    changed = add_bundle_payload_sha256(bundle)
    with pytest.raises(ValueError, match="initial_actual_state_context differs from bound preflight"):
        verify_m710_replay_bundle(changed)


@pytest.mark.parametrize("damage", [None, "lost_body", "nan_q", "old_world"])
def test_measured_new_world_state_enters_real_actual_state_pipeline(poc_scene, damage):
    name = poc_scene.cartons[-1].name
    raw = ideal_state(poc_scene, [name])
    updated = apply_actual_motion_state(poc_scene, raw)
    metadata = metadata_for(updated)
    initialization = archived_replay_initialization(metadata)
    feedback = [dict(name=c["name"], center_m=c["position_m"], quaternion_wxyz=c["orientation_wxyz"],
                     linear_velocity_m_s=c["linear_velocity_m_s"], angular_velocity_rad_s=c["angular_velocity_rad_s"])
                for c in raw["cartons"]]
    kwargs = dict(q_rad=list(raw["q_rad"]), joint_names=metadata["joint_names"],
                  cartons=feedback, world_session_id="new-archive-world")
    if damage == "lost_body":
        feedback.pop()
    elif damage == "nan_q":
        kwargs["q_rad"][0] = float("nan")
    elif damage == "old_world":
        kwargs["world_session_id"] = raw["world_session_id"]
    if damage:
        with pytest.raises(ValueError):
            archived_world_actual_state(metadata, initialization, **kwargs)
        return
    captured = archived_world_actual_state(metadata, initialization, **kwargs)
    current = apply_actual_motion_state(poc_scene, captured)
    assert current.snapshot["actual_state_context"]["world_session_id"] == "new-archive-world"
    assert name not in current.removable_cartons
    assert name in {b.name for b in current.cartons}
    assert captured["completed_carton_ids"] == []
    assert captured["ideal_received_ids"] == captured["processed_carton_ids"] == [name]
    assert captured["receiver_transport_state"] == raw["receiver_transport_state"]
