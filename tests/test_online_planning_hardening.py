from __future__ import annotations

from dataclasses import replace

import pytest

from unloading_sim.online_planning import (
    ContinuousPlanningSession,
    DeterministicAsyncPlanningExecutor,
    ExecutionState,
    PlanStatus,
    PlanningCandidate,
    PlanningPath,
    PlanningRequest,
    PlanningResult,
    PlannerBackend,
    PlanningWorldSnapshot,
    RobotStateRevision,
    SceneRevision,
    SessionState,
)


def world(scene_sequence=0, cartons=("a",), q=(0.0, 0.0), robot_sequence=None):
    scene = {"cartons": list(cartons)}
    revision = SceneRevision.from_scene(scene, scene_sequence)
    robot_revision = RobotStateRevision(
        scene_sequence if robot_sequence is None else robot_sequence,
        q,
        robot_state={"mode": "AUTO", "drives": "ON"},
    )
    return PlanningWorldSnapshot(
        revision,
        scene,
        robot_revision,
        tool_attachment={"id": "three-zone-tool", "revision": "tool-v1"},
        payload_attachment=None,
        base_state={"x_m": -0.5, "z_m": 0.7},
        conveyor_state={"extension_m": 0.0, "z_m": 0.2},
        config_identity={"sha256": "config-v3"},
    )


def request(request_id, snapshot, candidates=(("a", "top"),), *, speculative=False):
    return PlanningRequest(
        request_id,
        snapshot,
        tuple(PlanningCandidate(target, candidate) for target, candidate in candidates),
        seed=11,
        speculative=speculative,
    )


class HardeningBackend(PlannerBackend):
    def __init__(self, result_factory=None, *, validate=None):
        super().__init__(0)
        self.result_factory = result_factory
        self.validator = validate
        self.calls = []

    def plan(self, planning_request, candidate, planning_path):
        self.calls.append((planning_request, candidate, planning_path))
        if self.result_factory is not None:
            return self.result_factory(planning_request, candidate, planning_path)
        q = planning_request.world_snapshot.current_q
        return PlanningResult.succeeded(candidate, [q, tuple(value + 0.1 for value in q)])

    def validate_plan(self, envelope, snapshot):
        if self.validator is not None:
            return self.validator(envelope, snapshot)
        return super().validate_plan(envelope, snapshot)


def ready_session(backend=None, snapshot=None):
    backend = HardeningBackend() if backend is None else backend
    snapshot = world() if snapshot is None else snapshot
    session = ContinuousPlanningSession(backend, rolling_horizon=2)
    session.submit(request("k", snapshot))
    session.run_until_stable()
    assert session.state is SessionState.READY
    return session, backend, snapshot


def test_wrong_trajectory_start_cannot_execute():
    def wrong_start(planning_request, candidate, _path):
        q = planning_request.world_snapshot.current_q
        return PlanningResult.succeeded(candidate, [(q[0] + 0.4, q[1]), (0.5, 0.0)])

    backend = HardeningBackend(wrong_start)
    session = ContinuousPlanningSession(backend)
    session.submit(request("wrong-start", world()))
    session.run_until_stable()

    assert session.state is SessionState.RECOVERY
    assert session.start_execution() is None
    assert not session.ready_plans


def test_current_q_drift_invalidates_and_replans_from_actual_q():
    session, backend, snapshot = ready_session()
    drifted = replace(snapshot, robot_state_revision=RobotStateRevision(1, (0.03, -0.02), robot_state=snapshot.robot_state_revision.robot_state))
    session.update_scene(drifted)

    assert session.start_execution() is None
    session.run_until_stable()
    assert backend.calls[-1][0].world_snapshot.current_q == pytest.approx((0.03, -0.02))


@pytest.mark.parametrize(
    ("field", "changed_value"),
    [
        ("tool_attachment", {"id": "different-tool"}),
        ("payload_attachment", {"box": "unexpected-payload"}),
        ("base_state", {"x_m": -0.4, "z_m": 0.7}),
        ("conveyor_state", {"extension_m": 0.1, "z_m": 0.2}),
        ("config_identity", {"sha256": "different-config"}),
    ],
)
def test_execution_validation_rejects_changed_planning_context(field, changed_value):
    session, _, snapshot = ready_session()
    changed = replace(snapshot, **{field: changed_value})
    session.update_scene(changed)

    assert session.start_execution() is None
    assert session.invalidated_plans[-1].invalidated_by is not None
    assert session.state is SessionState.PLANNING


def test_execution_validation_rejects_changed_robot_mode():
    session, _, snapshot = ready_session()
    changed = replace(
        snapshot,
        robot_state_revision=RobotStateRevision(1, snapshot.current_q, {"mode": "MANUAL", "drives": "ON"}),
    )
    session.update_scene(changed)

    assert session.start_execution() is None
    assert session.state is SessionState.PLANNING


def test_scene_change_while_planning_discards_late_old_generation_result():
    backend = HardeningBackend()
    executor = DeterministicAsyncPlanningExecutor()
    first = world(0, ("a",))
    second = world(1, ("a", "new"), robot_sequence=1)
    session = ContinuousPlanningSession(backend, executor=executor)
    session.submit(request("pending", first))

    session.update_scene(second)
    session.advance(1)  # Completes generation 0 after generation 1 exists.

    assert not session.ready_plans
    assert any(event.kind == "stale_result_discarded" for event in session.events)
    session.run_until_stable()
    assert session.ready_plans[0].planning_generation == 1


def test_scene_change_during_execution_waits_for_stop_ack_before_replan():
    session, backend, _ = ready_session()
    session.start_execution()
    calls_before = len(backend.calls)

    session.update_scene(world(1, ("a", "intrusion"), robot_sequence=1))

    assert session.state is SessionState.STOPPING
    assert session.execution.state is ExecutionState.STOPPING
    session.advance(10)
    assert len(backend.calls) == calls_before
    assert not session.ready_plans


def test_stop_ack_replans_from_real_stopped_q_and_new_scene():
    session, backend, _ = ready_session()
    session.start_execution()
    changed = world(1, ("a", "intrusion"), q=(0.0, 0.0), robot_sequence=1)
    session.update_scene(changed)
    stopped = replace(changed, robot_state_revision=RobotStateRevision(2, (0.27, -0.16), robot_state=changed.robot_state_revision.robot_state))

    session.acknowledge_stop(stopped)
    session.run_until_stable()

    assert backend.calls[-1][0].world_snapshot.current_q == pytest.approx((0.27, -0.16))
    assert session.ready_plans[0].expected_start_state == pytest.approx((0.27, -0.16))


def test_success_without_new_scene_waits_and_does_not_reuse_pre_execution_scene():
    session, _, original = ready_session()
    plan = session.start_execution()
    session.complete_execution(success=True, stopped_q=plan.expected_end_state)

    assert session.state is SessionState.WAITING_FOR_SCENE
    assert session.current_world.scene_revision == original.scene_revision
    assert session.start_execution() is None


def test_success_with_new_scene_invalidates_other_non_speculative_ready_plan():
    snapshot = world(0, ("a", "b"))
    session = ContinuousPlanningSession(HardeningBackend(), rolling_horizon=2)
    session.submit(request("first", snapshot, (("a", "top"),)))
    session.submit(request("second", snapshot, (("b", "top"),)))
    session.run_until_stable()
    plan = session.start_execution()
    next_world = world(1, ("b",), q=plan.expected_end_state, robot_sequence=1)

    session.complete_execution(
        success=True,
        stopped_q=plan.expected_end_state,
        world_snapshot=next_world,
    )

    assert any(item.request.request_id == "second" for item in session.invalidated_plans)
    session.run_until_stable()
    assert session.ready_plans[0].planned_snapshot == next_world


def test_same_scene_sequence_with_different_fingerprint_rejected_at_all_entry_points():
    session, _, _ = ready_session()
    conflicting = world(0, ("different",), robot_sequence=1)
    with pytest.raises(ValueError, match="same sequence"):
        session.update_scene(conflicting)

    session.start_execution()
    with pytest.raises(ValueError, match="same sequence"):
        session.complete_execution(success=True, stopped_q=(0.1, 0.1), world_snapshot=conflicting)

    stopping, _, _ = ready_session()
    stopping.start_execution()
    latest = world(1, ("a", "intrusion"), robot_sequence=1)
    stopping.update_scene(latest)
    conflicting_ack = world(1, ("other",), robot_sequence=2)
    with pytest.raises(ValueError, match="same sequence"):
        stopping.acknowledge_stop(conflicting_ack)


def test_one_failed_request_and_one_successful_request_is_not_globally_blocked():
    def outcomes(planning_request, candidate, _path):
        if planning_request.request_id == "fails":
            return PlanningResult.failed(PlanStatus.COLLISION, candidate)
        q = planning_request.world_snapshot.current_q
        return PlanningResult.succeeded(candidate, [q, (0.1, 0.1)])

    snapshot = world()
    session = ContinuousPlanningSession(HardeningBackend(outcomes), rolling_horizon=2)
    session.submit(request("fails", snapshot))
    session.submit(request("succeeds", snapshot))
    session.run_until_stable()

    assert session.state is SessionState.READY
    assert session.ready_plans[0].request.request_id == "succeeds"


@pytest.mark.parametrize("failure", ["wrong_type", "timeout"])
def test_backend_contract_error_and_timeout_do_not_deadlock(failure):
    if failure == "wrong_type":
        backend = HardeningBackend(lambda *_: {"status": "SUCCESS"})
    else:
        backend = HardeningBackend(lambda _request, candidate, _path: PlanningResult.failed(PlanStatus.TIMEOUT, candidate))
    session = ContinuousPlanningSession(backend)
    session.submit(request(failure, world()))
    session.run_until_stable()
    assert session.state in {SessionState.RECOVERY, SessionState.BLOCKED}
    assert session.state is not SessionState.PLANNING


@pytest.mark.parametrize("failure", ["wrong_candidate", "wrong_dof"])
def test_backend_candidate_identity_and_trajectory_contract_fail_closed(failure):
    def invalid(planning_request, candidate, _path):
        if failure == "wrong_candidate":
            other = PlanningCandidate(candidate.target_id, "other")
            return PlanningResult.succeeded(other, [planning_request.start_state, (0.1, 0.1)])
        return PlanningResult.succeeded(candidate, [(0.0, 0.0, 0.0), (0.1, 0.1, 0.1)])

    session = ContinuousPlanningSession(HardeningBackend(invalid))
    session.submit(request(failure, world()))
    session.run_until_stable()

    assert session.state is SessionState.RECOVERY
    assert not session.ready_plans


def test_world_snapshot_detaches_mutable_inputs_and_request_requires_it():
    scene = {"cartons": ["a"]}
    revision = SceneRevision.from_scene(scene, 0)
    base = {"x_m": -0.5}
    snapshot = PlanningWorldSnapshot(
        revision,
        scene,
        RobotStateRevision(0, (0.0, 0.0), {"mode": "AUTO"}),
        {"id": "tool"},
        None,
        base,
        {"extension_m": 0.0},
        {"sha256": "cfg"},
    )
    scene["cartons"].append("mutated")
    base["x_m"] = 99.0

    assert snapshot.scene_snapshot["cartons"] == ("a",)
    assert snapshot.base_state["x_m"] == -0.5
    with pytest.raises(TypeError):
        snapshot.base_state["x_m"] = 0.0
    with pytest.raises(TypeError, match="PlanningWorldSnapshot"):
        PlanningRequest("bad", revision, (PlanningCandidate("a", "top"),))


def test_speculative_start_must_equal_plan_k_predicted_end():
    session, _, _ = ready_session()
    plan = session.start_execution()
    predicted_scene = world(1, (), q=(9.0, 9.0), robot_sequence=1)

    with pytest.raises(ValueError, match="predicted end"):
        session.submit_speculative(request("k+1", predicted_scene, speculative=True))

    matching = replace(predicted_scene, robot_state_revision=RobotStateRevision(1, plan.expected_end_state, robot_state=predicted_scene.robot_state_revision.robot_state))
    session.submit_speculative(request("k+1-valid", matching, speculative=True))
    session.run_until_stable()
    assert session.speculative_plans[0].predecessor_plan_id == plan.plan_id


def test_all_failures_still_return_blocked_without_manufactured_trajectory():
    backend = HardeningBackend(lambda _request, candidate, _path: PlanningResult.failed(PlanStatus.NO_IK, candidate))
    session = ContinuousPlanningSession(backend)
    session.submit(request("all-fail", world(), (("a", "top"), ("a", "side"))))
    session.run_until_stable()

    assert session.state is SessionState.BLOCKED
    assert not session.ready_plans
    assert not session.speculative_plans


def test_request_id_is_unique_for_session_lifetime():
    snapshot = world()
    session = ContinuousPlanningSession(HardeningBackend(), rolling_horizon=2)
    session.submit(request("unique", snapshot))
    with pytest.raises(ValueError, match="duplicate request id"):
        session.submit(request("unique", snapshot))
