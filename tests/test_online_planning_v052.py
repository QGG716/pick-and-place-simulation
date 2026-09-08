from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from unloading_sim.online_planning import (
    BoundaryMode,
    ContinuousPlanningSession,
    DeterministicAsyncPlanningExecutor,
    MotionBoundaryState,
    PlanArtifactKind,
    PlanValidationResult,
    PlanValidator,
    PlanningCandidate,
    PlanningRequest,
    PlanningResult,
    PlanningWorldSnapshot,
    PlannerBackend,
    RobotStateRevision,
    SceneRevision,
    SessionEvent,
    SessionState,
    ValidationStatus,
)


def world(sequence=0, cartons=("a",), q=(0.0, 0.0)):
    scene = {"cartons": list(cartons)}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, sequence),
        scene,
        RobotStateRevision(sequence, q, {"mode": "AUTO"}),
        {"id": "tool"},
        None,
        {"x": 0.0},
        {"extension": 0.0},
        {
            "id": "dev0",
            "robot_model_fingerprint": "robot-dev0",
            "world_model_fingerprint": "world-dev0",
        },
    )


def stop_boundary(q, *, time_seconds=0.0, predecessor=None):
    q = tuple(q)
    zeros = tuple(0.0 for _ in q)
    return MotionBoundaryState(
        q=q,
        qd=zeros,
        qdd=zeros,
        time_seconds=time_seconds,
        boundary_mode=BoundaryMode.STOP_BOUNDARY,
        predecessor_plan_id=predecessor,
    )


def request(request_id, snapshot, *, boundary=None, speculative=False):
    return PlanningRequest(
        request_id,
        snapshot,
        (PlanningCandidate("a", "top"),),
        speculative=speculative,
        motion_boundary=boundary or stop_boundary(snapshot.current_q),
    )


class GeometricBackend(PlannerBackend):
    def plan(self, planning_request, candidate, planning_path):
        q = planning_request.motion_boundary.q
        end = tuple(value + 0.1 for value in q)
        return PlanningResult.succeeded(candidate, [q, end])


def ready_session(*, executor=None, rolling_horizon=2):
    snapshot = world(0, ("a", "b"))
    session = ContinuousPlanningSession(
        GeometricBackend(), executor=executor, rolling_horizon=rolling_horizon
    )
    session.submit(request("k", snapshot))
    session.run_until_stable()
    return session, snapshot


def test_nested_metadata_is_recursively_immutable_and_detached():
    candidate_metadata = {"nested": {"values": [1, np.int64(2)], "array": np.array([3.0, 4.0])}}
    request_metadata = {"nested": [{"enabled": True}]}
    result_metadata = {"native": {"codes": ["a", "b"]}}
    event_details = {"diagnostics": {"samples": np.array([1, 2])}}

    candidate = PlanningCandidate("a", "top", candidate_metadata)
    planning_request = PlanningRequest(
        "immutable",
        world(),
        (candidate,),
        metadata=request_metadata,
    )
    result = PlanningResult.succeeded(candidate, [(0.0, 0.0), (0.1, 0.1)], metadata=result_metadata)
    event = SessionEvent(1, "test", "immutable", "fixture", event_details)

    candidate_metadata["nested"]["values"].append(99)
    candidate_metadata["nested"]["array"][0] = 99
    request_metadata["nested"][0]["enabled"] = False
    result_metadata["native"]["codes"].append("changed")
    event_details["diagnostics"]["samples"][0] = 99

    assert candidate.metadata["nested"]["values"] == (1, 2)
    assert candidate.metadata["nested"]["array"] == (3.0, 4.0)
    assert planning_request.metadata["nested"][0]["enabled"] is True
    assert result.metadata["native"]["codes"] == ("a", "b")
    assert event.details["diagnostics"]["samples"] == (1, 2)
    with pytest.raises(TypeError):
        candidate.metadata["new"] = "forbidden"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"q": (0.0, np.nan), "qd": (0.0, 0.0), "qdd": (0.0, 0.0)},
        {"q": (0.0, 0.0), "qd": (0.0,), "qdd": (0.0, 0.0)},
        {"q": (0.0, 0.0), "qd": (0.0, 0.0), "qdd": (0.0,)},
        {"q": (), "qd": (), "qdd": ()},
    ],
)
def test_motion_boundary_rejects_nonfinite_empty_or_dof_mismatch(kwargs):
    with pytest.raises(ValueError):
        MotionBoundaryState(
            **kwargs,
            time_seconds=0.0,
            boundary_mode=BoundaryMode.STOP_BOUNDARY,
        )


@pytest.mark.parametrize(
    ("qd", "qdd"),
    [((1e-3, 0.0), (0.0, 0.0)), ((0.0, 0.0), (0.0, -1e-3))],
)
def test_stop_boundary_rejects_nonzero_velocity_or_acceleration(qd, qdd):
    with pytest.raises(ValueError, match="STOP_BOUNDARY"):
        MotionBoundaryState((0.0, 0.0), qd, qdd, 0.0, BoundaryMode.STOP_BOUNDARY)


def test_motion_boundary_match_includes_velocity_acceleration_and_time():
    first = MotionBoundaryState(
        (0.0, 0.0), (0.2, -0.1), (0.01, 0.02), 1.0, BoundaryMode.CONTINUOUS_BOUNDARY
    )
    different_velocity = replace(first, qd=(0.25, -0.1))
    close = replace(first, qd=(0.2000001, -0.1), time_seconds=1.0000001)

    assert not first.matches(different_velocity)
    assert first.matches(close, qd_atol=1e-6, time_atol=1e-6)


def test_geometric_path_cannot_declare_continuous_boundaries():
    candidate = PlanningCandidate("a", "top")
    start = MotionBoundaryState(
        (0.0, 0.0), (0.1, 0.0), (0.0, 0.0), 0.0, BoundaryMode.CONTINUOUS_BOUNDARY
    )
    end = replace(start, q=(0.1, 0.1), time_seconds=1.0)

    with pytest.raises(ValueError, match="GEOMETRIC_PATH"):
        PlanningResult.succeeded(
            candidate,
            [start.q, end.q],
            artifact_kind=PlanArtifactKind.GEOMETRIC_PATH,
            expected_start_boundary=start,
            expected_end_boundary=end,
        )


def test_speculative_boundary_rejects_same_q_with_different_qd():
    session, _ = ready_session()
    plan = session.start_execution()
    predicted_world = world(1, ("b",), plan.expected_end_boundary.q)
    wrong = MotionBoundaryState(
        q=plan.expected_end_boundary.q,
        qd=(0.1, 0.0),
        qdd=(0.0, 0.0),
        time_seconds=plan.expected_end_boundary.time_seconds,
        boundary_mode=BoundaryMode.CONTINUOUS_BOUNDARY,
    )

    with pytest.raises(ValueError, match="predicted end boundary"):
        session.submit_speculative(request("k+1", predicted_world, boundary=wrong, speculative=True))


def test_unknown_predecessor_is_rejected_without_polluting_session():
    session = ContinuousPlanningSession(GeometricBackend(), rolling_horizon=2)
    snapshot = world()
    orphan = request("orphan", snapshot, boundary=stop_boundary(snapshot.current_q, predecessor="missing"))
    before = session.snapshot()

    with pytest.raises(ValueError, match="predecessor"):
        session.submit(orphan)

    assert session.snapshot() == before
    session.submit(request("orphan", snapshot))
    session.run_until_stable()
    assert session.state is SessionState.READY


def test_predecessor_failure_cascades_to_inflight_successor():
    executor = DeterministicAsyncPlanningExecutor()
    session, _ = ready_session(executor=executor)
    plan = session.start_execution()
    predicted = world(1, ("b",), plan.expected_end_boundary.q)
    session.submit_speculative(
        request(
            "k+1",
            predicted,
            boundary=replace(plan.expected_end_boundary, predecessor_plan_id=plan.plan_id),
            speculative=True,
        )
    )

    session.complete_execution(success=False, stopped_q=plan.expected_end_boundary.q)
    session.advance(10)

    assert session.state is SessionState.RECOVERY
    assert not session.ready_plans
    assert not session.speculative_plans
    assert any(event.kind == "lineage_invalidated" for event in session.events)


def test_horizon_two_allows_only_one_successor_while_k_executes():
    session, _ = ready_session(rolling_horizon=2)
    plan = session.start_execution()
    predicted = world(1, ("b",), plan.expected_end_boundary.q)
    successor = request(
        "k+1",
        predicted,
        boundary=replace(plan.expected_end_boundary, predecessor_plan_id=plan.plan_id),
        speculative=True,
    )
    session.submit_speculative(successor)
    before = session.snapshot()

    with pytest.raises(RuntimeError, match="rolling horizon|live successor"):
        session.submit_speculative(replace(successor, request_id="k+1-duplicate"))

    assert session.snapshot() == before
    assert "k+1-duplicate" not in session.known_request_ids
    assert session.occupancy == 2


def test_successful_predecessor_is_recorded_before_successor_becomes_ready():
    session, _ = ready_session()
    plan = session.start_execution()
    predicted = world(1, ("b",), plan.expected_end_boundary.q)
    successor = request(
        "k+1",
        predicted,
        boundary=replace(plan.expected_end_boundary, predecessor_plan_id=plan.plan_id),
        speculative=True,
    )
    session.submit_speculative(successor)
    session.run_until_stable()
    assert not session.ready_plans
    assert not session.speculative_plans[0].executable

    session.complete_execution(
        success=True,
        stopped_q=plan.expected_end_boundary.q,
        world_snapshot=predicted,
    )

    assert plan.plan_id in session.successful_completed_plan_ids
    assert session.last_successful_plan_id == plan.plan_id
    assert session.ready_plans[0].predecessor_plan_id == plan.plan_id
    assert session.ready_plans[0].plan_id in session.plan_children[plan.plan_id]


def test_validator_result_with_wrong_model_binding_fails_closed():
    class WrongBindingValidator(PlanValidator):
        def __init__(self):
            super().__init__("wrong-binding", "1")

        def validate(self, envelope, snapshot, boundary):
            return PlanValidationResult(
                ValidationStatus.VALID,
                snapshot,
                boundary,
                "claims valid",
                self.name,
                self.version,
                robot_model_fingerprint="wrong-robot-model",
            )

    session = ContinuousPlanningSession(GeometricBackend(), validator=WrongBindingValidator())
    session.submit(request("wrong-validator-binding", world()))
    session.run_until_stable()

    assert session.state is SessionState.RECOVERY
    assert not session.ready_plans


def test_default_compatibility_validator_proxies_legacy_backend_hook():
    class CountingBackend(GeometricBackend):
        def __init__(self):
            super().__init__()
            self.validation_calls = 0

        def validate_plan(self, envelope, snapshot):
            self.validation_calls += 1
            return super().validate_plan(envelope, snapshot)

    backend = CountingBackend()
    session = ContinuousPlanningSession(backend)
    session.submit(request("compat-validator", world()))
    session.run_until_stable()

    assert backend.validation_calls == 1
    assert session.state is SessionState.READY
