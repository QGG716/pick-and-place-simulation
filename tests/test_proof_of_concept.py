"""Focused profile/region/scheduling contracts, not physical success evidence."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rotation_vector
from unloading_sim.planning_profile import DEFAULT_MOTION, profile_evidence, deadline_after
from unloading_sim.layout_single_carton import load_layout_motion_policy
from unloading_sim.layout_trajectory import LayoutTrajectoryBudget, LayoutTrajectoryConnector
from unloading_sim.post_landing_transport import (
    begin_ideal_transport, ideal_transport_ids, advance_ideal_transport, OUTFED, RECEPTION_SOURCE)
from unloading_sim.release_motion import (
    ideal_reception_region, predict_release, IDEAL_RECEPTION_RELEASE, SUPPORTED_RELEASE)
from unloading_sim.contact_scheduler import ContactCandidateScheduler
from unloading_sim.history_candidates import history_policy, HistorySource
from test_serial_unloading import scene, actual
from ideal_handoff_fixture import measured_handoff


def objects():
    receiver = OBB([0, 0, .5], [1, .6, .1], np.eye(3), "belt", "conveyor")
    rotation = rotation_matrix_from_rotation_vector([.002, 0, 0])
    box = OBB([0, 0, .771], [.3, .2, .15], rotation, "target", "carton")
    return box, receiver


def policy():
    return load_layout_motion_policy(DEFAULT_MOTION).data["search_strategy"]["post_landing_transport"]


def test_profile_is_explicit_and_old_config_preserves_physical_reception():
    new = load_layout_motion_policy(DEFAULT_MOTION)
    old = load_layout_motion_policy("configs/validation/m710id70_layout_v1_single_carton.yaml")
    assert profile_evidence(new.data)["ideal_reception"]
    assert not profile_evidence(old.data)["ideal_reception"]
    assert new.data["search_strategy"]["planning_wall_time_s"] is None
    assert old.data["search_strategy"]["planning_wall_time_s"] == 900.
    bad = deepcopy(new.data)
    bad["search_strategy"]["post_landing_transport"]["reception_mode"] = "physical"
    with pytest.raises(ValueError):
        profile_evidence(bad)
    assert new.policy_fingerprint != old.policy_fingerprint


@pytest.mark.parametrize("seconds,expected", [(None, None), (0, 100.), (4, 104.)])
def test_none_zero_positive_deadlines(seconds, expected):
    assert deadline_after(100., seconds) == expected
    LayoutTrajectoryBudget(planning_wall_time_s=seconds)


@pytest.mark.parametrize("value", [-1, float("inf"), float("nan"), True])
def test_invalid_budget_rejected(value):
    with pytest.raises(ValueError):
        deadline_after(0, value)


def test_production_request_clock_has_no_hidden_deadline(monkeypatch):
    from unloading_sim import layout_trajectory as module
    c = object.__new__(LayoutTrajectoryConnector)
    c.budget = LayoutTrajectoryBudget(proof_of_concept=True, candidate_wall_time_s=None,
        stage_wall_time_s=None, postprocess_wall_time_s=None, planning_wall_time_s=None)
    c._deadline_monotonic = None
    monkeypatch.setattr(module, "perf_counter", lambda: 1_000_000.)
    assert c._remaining_wall_time() is None
    assert not c._deadline_reached()
    assert c._optional_deadline() is None
    c._deadline_monotonic = 0.
    assert c._deadline_reached()


def test_region_is_not_simultaneous_contact_or_physical_landing():
    box, receiver = objects()
    region = ideal_reception_region(box, [receiver])
    assert region["accepted"] and not region["actual_top_contact_observed"]
    assert region["vertical_correction_m"] < 0
    predicted = predict_release(box, [receiver], mode=IDEAL_RECEPTION_RELEASE)
    assert predicted["accepted"] and predicted["actual_landing_state"] is None
    assert predicted["actual_support"] is None
    assert not predict_release(box, [receiver], mode=SUPPORTED_RELEASE)["accepted"]
    obstacle = OBB(box.center, [.01]*3, np.eye(3), "frame", "fixed")
    assert not predict_release(box, [receiver], mode=IDEAL_RECEPTION_RELEASE, obstacles=[obstacle])["accepted"]
    for displacement in ([2., 0, 0], [0, 0, .1], [0, 0, -.04]):
        outside = OBB(box.center + displacement, box.half_extents, box.rotation, box.name, box.category)
        assert not ideal_reception_region(outside, [receiver])["accepted"]


def takeover(**overrides):
    box, receiver = objects()
    gates = dict(attachment_removed=True, top_contact_observed=False,
        support_geometry_accepted=False, expected_target="target", actual_attachment_observed=True)
    gates.update(overrides)
    return begin_ideal_transport(box, receiver_name="belt", receivers={"belt": receiver},
        directions={"belt": [-1., 0, 0]}, time_s=0., policy=policy(),
        released_handoff=measured_handoff(box,receiver,policy(),0.), **gates)


@pytest.mark.parametrize("gates", [{"attachment_removed": False}, {"expected_target": "other"},
                                   {"actual_attachment_observed": False}])
def test_takeover_requires_real_release_and_same_attached_target(gates):
    with pytest.raises(ValueError):
        takeover(**gates)


def test_source_isolation_continuous_same_body_and_idempotent_outfeed():
    record = takeover()
    assert record["completion_source"] == RECEPTION_SOURCE
    assert not record["actual_top_contact_observed"]
    assert ideal_transport_ids(policy(), {"target": record}) == {"target"}
    legacy = {**policy(), "reception_mode": "physical"}
    with pytest.raises(ValueError):
        ideal_transport_ids(legacy, {"target": record})
    before = np.asarray(record["pose_world"])[:3, 3].copy()
    advance_ideal_transport(record, dt_s=.01, speed_m_s=.3)
    after = np.asarray(record["pose_world"])[:3, 3]
    assert np.linalg.norm(after-before) <= .003000001
    assert record["state"] != OUTFED
    assert advance_ideal_transport(record, dt_s=20., speed_m_s=.3)["event"] == OUTFED
    assert advance_ideal_transport(record, dt_s=20., speed_m_s=.3) is None


def test_full_pool_and_retries_both_execute_with_new_seeds():
    box, _ = objects()
    poses = []
    for i in range(65):
        pose = np.eye(4)
        pose[0, 3] = i*.001
        poses.append(("front", (0, pose, {"index": i})))
    scheduler = ContactCandidateScheduler(poses, target=box, context={}, request_seed=71070,
        batch_size=12, attempt_limit=195, complete_connection_limit=195, fair_retries=True)
    while (attempt := scheduler.next_attempt()) is not None:
        _, record = attempt
        scheduler.finish(record, {"failure_stage": "ik", "failure_reason": "NO_IK"},
                         complete_connection_attempted=False, elapsed_s=0.)
    assert len(scheduler.visited) == 65
    assert scheduler.records[1]["retry_index"] == 1
    assert len({r["ik_seed"] for r in scheduler.records}) == 195
    assert scheduler.summary()["pending_retry_count"] == 0
    assert scheduler.summary()["remaining_unsearched_count"] == 0


def test_history_without_deadline_preserves_input_resource_limits():
    config = history_policy(dict(screening_wall_time_s=None, wall_time_s=None, candidate_wall_time_s=None))
    assert config.maximum_file_bytes == 4_000_000 and config.maximum_files == 16
    assert HistorySource(config).evidence()["files"][0]["status"] == "NOT_CONFIGURED"


def test_continuation_keeps_all_identities_but_does_not_regrasp_assumed_reception(scene):
    from unloading_sim.serial_unloading import apply_actual_motion_state
    from unloading_sim.qualification import reception_counts
    name = scene.cartons[-3].name
    data = deepcopy(scene.policy.data)
    data["search_strategy"] = {"post_landing_transport": policy(),
        "surface_directions_world": {"receiver": [-1., 0, 0]}}
    scene = replace(scene, policy=replace(scene.policy, data=data))
    state = actual(scene)
    box = OBB([-.55, .35, .77], [.3, .2, .15], np.eye(3), name, "carton")
    record = begin_ideal_transport(box, receiver_name="receiver", receivers={"receiver": scene.receiver},
        directions={"receiver": [-1., 0, 0]}, time_s=0, policy=policy(), attachment_removed=True,
        top_contact_observed=False, support_geometry_accepted=False,
        expected_target=name, actual_attachment_observed=True,
        released_handoff=measured_handoff(box,scene.receiver,policy(),0.))
    state.update(receiver_transport_state={name: record}, ideal_received_ids=[name], processed_carton_ids=[name])
    item = next(item for item in state["cartons"] if item["name"] == name)
    item["position_m"] = box.center.tolist()
    updated = apply_actual_motion_state(scene, state)
    assert len(updated.cartons) == 40 and name not in updated.remaining_stack_names
    assert updated.snapshot["actual_state_context"]["completed_carton_ids"] == []
    assert updated.snapshot["actual_state_context"]["ideal_received_ids"] == [name]
    counts = reception_counts([], {name: record})
    assert counts["actual_reception"] == counts["ideal_outfeed"] == 0
    assert counts["ideal_reception"] == counts["workflow_processed"] == 1
    assert reception_counts([], {name: record}) == counts


def test_ordinary_entry_dispatches_beyond_48_and_executes_retries(monkeypatch):
    """Injected failures verify dispatch only; not robot planning success."""
    from unloading_sim import layout_single_carton as production
    policy = load_layout_motion_policy(DEFAULT_MOTION)
    scene = production.build_verified_motion_input(policy)
    c = production._build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot()).connector
    first_target, evaluated, connected = [], [], []
    def pool(scene, target, faces):
        if not first_target:
            first_target.append(target.name)
        if target.name != first_target[0]:
            return []
        result = []
        for index in range(65):
            pose = np.eye(4)
            pose[0, 3] = index*.001
            result.append(("front", (0, pose, {"index": index})))
        return result
    def evaluate(scene, target, face, roll, physical, virtual, variant, seed, *args, **kwargs):
        assert kwargs["deadline_monotonic"] is None
        evaluated.append((variant["index"], seed))
        return dict(face=face, failure_stage="trajectory_search", failure_reason="PENDING",
            search_status="WIRING_TEST_ONLY", strict_grasp_candidates=[{
                "q_rad": scene.policy.layout_validation.initial_q.tolist(), "candidate_id": "injected"}],
            ik_stream=None, path_connection_attempts=0, complete_trajectory=False,
            planning_timing_seconds={"coverage_and_candidate_qualification": 0.,
                                     "grasp_ik_inclusive_of_endpoint_validation": 0.})
    def branch(**kwargs):
        assert c._deadline_monotonic is None
        connected.append(kwargs["seed"])
        return None, {"reason": "INJECTED_FINITE_SEARCH_FAILURE", "stage": "transit"}, {}
    monkeypatch.setattr(production, "_scheduled_contact_poses", pool)
    monkeypatch.setattr(production, "_audit_pose", evaluate)
    monkeypatch.setattr(c, "_plan_branch", branch)
    result = production.run_layout_single_carton_audit(policy, motion_input=scene, trajectory_connector=c)
    assert len(evaluated) == len(connected) == 195
    assert {index for index, seed in evaluated} == set(range(65))
    assert len(set(connected)) == 195
    assert not result["planning_success"]


def test_extraction_expanded_routes_are_finite_and_not_prefix_truncated(monkeypatch):
    from types import SimpleNamespace
    from unloading_sim import layout_trajectory as module
    box, _ = objects()
    c = object.__new__(LayoutTrajectoryConnector)
    c.budget = LayoutTrajectoryBudget(proof_of_concept=True)
    c.collision_margin_m = .01
    c.contact_tolerance_m = .0002
    from unloading_sim.collision_policy import SimulationCollisionPolicy
    c.collision_policy = SimulationCollisionPolicy()
    c.ik = dict(position_tolerance_m=.0001, orientation_tolerance_rad=.0002)
    c.robot = SimpleNamespace(fk=lambda q: np.eye(4))
    directions = []
    def distance(box, direction, *args, **kwargs):
        assert np.all(np.isfinite(direction)) and np.isclose(np.linalg.norm(direction), 1.)
        directions.append(direction.copy())
        return .1
    monkeypatch.setattr(module, "minimum_clearance_extraction_distance", distance)
    c._cartesian = lambda *args, **kwargs: ([], {"reason": "INJECTED"}, {})
    tracker = SimpleNamespace(clone=lambda: SimpleNamespace(fully_released=False, evidence=lambda: {}))
    list(c._extraction_options(np.zeros(6), SimpleNamespace(box_at=lambda q: box), [], tracker,
                              np.array([0., 0., -1.]), seed=7))
    assert len(directions) == 6  # Opposing up/down sum is discarded, remaining routes all visited.


def test_zero_request_budget_is_unsearched_not_an_invalid_physical_start():
    from unloading_sim import layout_single_carton as production
    policy = load_layout_motion_policy(DEFAULT_MOTION)
    data = deepcopy(policy.data)
    data["search_strategy"]["planning_wall_time_s"] = 0.
    policy = replace(policy, data=data)
    scene = production.build_verified_motion_input(policy)
    result = production.run_layout_single_carton_audit(policy, motion_input=scene)
    assert result["initial_state_audit"]["status"] == "NOT_EVALUATED"
    assert result["initial_state_audit"]["failure"]["reason"] == "PLANNING_WALL_CLOCK_DEADLINE"
    assert result["statistics"]["candidate_pose_attempts"] == 0
    assert not result["planning_success"]
