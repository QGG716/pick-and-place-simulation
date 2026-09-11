from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import re

import pytest

from unloading_sim.online_planning import (
    ArtifactKind,
    BackendHealth,
    BackendProvenance,
    BoundaryMode,
    ContinuousPlanningSession,
    OperationalOutcome,
    PlanStatus,
    PlanningPath,
    SessionState,
)
from tests.online_contract_fixtures import (
    ROBOT_MODEL,
    WORLD_MODEL,
    SemanticBackend,
    SemanticValidator,
    semantic_capabilities as capabilities,
    semantic_identity as identity,
    semantic_request as request,
    semantic_world as world,
)


def test_backend_identity_provenance_and_capabilities_are_immutable_and_saved():
    backend = SemanticBackend("cpu-stop", capabilities(PlanningPath.COLD))
    validator = SemanticValidator()
    session = ContinuousPlanningSession(backend, validator=validator)
    session.submit(request("trace", world()))
    session.run_until_stable()

    envelope = session.ready_plans[0]
    assert isinstance(envelope.proposed_by, BackendProvenance)
    assert envelope.proposed_by.backend_name == "cpu-stop"
    assert envelope.proposed_by.deterministic_seed == 0
    assert envelope.artifact_kind is ArtifactKind.GEOMETRIC_PATH
    assert envelope.robot_model_fingerprint == ROBOT_MODEL
    assert envelope.world_model_fingerprint == WORLD_MODEL
    assert envelope.validated_by == "authoritative-test-validator@1.0"
    assert envelope.validated_scene_revision == envelope.planned_scene_revision
    with pytest.raises((AttributeError, TypeError)):
        envelope.proposed_by.backend_name = "changed"


def test_scheduler_skips_unsupported_paths_without_counting_planning_failure():
    backend = SemanticBackend("cold-only", capabilities(PlanningPath.COLD))
    session = ContinuousPlanningSession(backend, validator=SemanticValidator())
    session.submit(request("supported", world()))
    session.run_until_stable()

    assert backend.calls == [PlanningPath.COLD]
    report = session.statistics.report()
    assert report["planning_status_counts"] == {"SUCCESS": 1}
    assert report["unsupported_route_skip_count"] == 2


def test_fast_cache_miss_falls_through_to_warm_without_blocking():
    library = SemanticBackend(
        "library",
        capabilities(PlanningPath.FAST),
        {PlanningPath.FAST: OperationalOutcome.CACHE_MISS},
    )
    warm = SemanticBackend(
        "warm",
        capabilities(
            PlanningPath.WARM,
            artifacts=(ArtifactKind.TIME_PARAMETERIZED_TRAJECTORY,),
        ),
    )
    session = ContinuousPlanningSession((library, warm), validator=SemanticValidator())
    session.submit(request("fallback", world()))
    session.run_until_stable()

    assert session.state is SessionState.READY
    assert library.calls == [PlanningPath.FAST]
    assert warm.calls == [PlanningPath.WARM]
    report = session.statistics.report()
    assert report["fallback_count_by_path"]["FAST"] == 1
    assert report["outcome_count_by_backend_category"]["library"]["OPERATIONAL:CACHE_MISS"] == 1


def test_geometric_path_cannot_satisfy_continuous_motion_boundary():
    geometric = SemanticBackend(
        "geometric",
        capabilities(
            PlanningPath.FAST,
            boundaries=(BoundaryMode.CONTINUOUS_BOUNDARY,),
            artifacts=(ArtifactKind.GEOMETRIC_PATH,),
        ),
    )
    session = ContinuousPlanningSession(geometric, validator=SemanticValidator())
    session.submit(request("continuous", world(), boundary=BoundaryMode.CONTINUOUS_BOUNDARY))
    session.run_until_stable()

    assert not geometric.calls
    assert session.state is SessionState.BLOCKED
    assert any(event.kind == "unsupported_route_skipped" for event in session.events)


def test_cpu_geometric_stop_only_profile_is_valid_for_stop_boundary():
    geometric = SemanticBackend(
        "cpu-geometric",
        capabilities(
            PlanningPath.COLD,
            boundaries=(BoundaryMode.STOP_BOUNDARY,),
            artifacts=(ArtifactKind.GEOMETRIC_PATH,),
        ),
    )
    session = ContinuousPlanningSession(geometric, validator=SemanticValidator())
    session.submit(request("stop-path", world()))
    session.run_until_stable()

    assert session.state is SessionState.READY
    assert session.ready_plans[0].artifact_kind is ArtifactKind.GEOMETRIC_PATH


def test_continuous_execution_requires_actual_motion_boundary_state():
    backend = SemanticBackend(
        "continuous-trajectory",
        capabilities(
            PlanningPath.WARM,
            boundaries=(BoundaryMode.CONTINUOUS_BOUNDARY,),
            artifacts=(ArtifactKind.TIME_PARAMETERIZED_TRAJECTORY,),
        ),
    )
    snapshot = world()
    planning_request = request("continuous-actual", snapshot, boundary=BoundaryMode.CONTINUOUS_BOUNDARY)
    session = ContinuousPlanningSession(backend, validator=SemanticValidator())
    session.submit(planning_request)
    session.run_until_stable()

    assert session.start_execution() is None
    assert session.state is SessionState.RECOVERY

    accepted = ContinuousPlanningSession(backend, validator=SemanticValidator())
    accepted.submit(replace(planning_request, request_id="continuous-provided"))
    accepted.run_until_stable()
    assert accepted.start_execution(planning_request.motion_boundary) is not None


def test_backend_success_must_pass_authoritative_validator_before_ready():
    validator = SemanticValidator(valid=False)
    session = ContinuousPlanningSession(
        SemanticBackend("proposal", capabilities(PlanningPath.FAST)), validator=validator
    )
    session.submit(request("rejected", world()))
    session.run_until_stable()

    assert validator.calls == 1
    assert not session.ready_plans
    assert session.state is SessionState.BLOCKED
    assert session.invalidated_plans[-1].validation_message == "authoritative rejection"


def test_backend_model_fingerprint_mismatch_never_reaches_validator_or_ready():
    backend = SemanticBackend("wrong-model", capabilities(PlanningPath.FAST))
    backend.identity = identity("wrong-model", robot_model="different")
    validator = SemanticValidator()
    session = ContinuousPlanningSession(backend, validator=validator)
    session.submit(request("mismatch", world()))
    session.run_until_stable()

    assert validator.calls == 0
    assert not session.ready_plans
    assert session.state is SessionState.BLOCKED


@pytest.mark.parametrize(
    "outcome",
    [
        OperationalOutcome.BACKEND_UNAVAILABLE,
        OperationalOutcome.DEADLINE_EXCEEDED,
        OperationalOutcome.BACKEND_ERROR,
    ],
)
def test_operational_failures_fallback_and_do_not_deadlock(outcome):
    failing = SemanticBackend("failing", capabilities(PlanningPath.FAST), {PlanningPath.FAST: outcome})
    succeeding = SemanticBackend("fallback", capabilities(PlanningPath.COLD))
    session = ContinuousPlanningSession((failing, succeeding), validator=SemanticValidator())
    session.submit(request(outcome.value, world()))
    session.run_until_stable()

    assert session.state is SessionState.READY
    assert session.state is not SessionState.PLANNING


def test_backend_exception_is_operational_and_falls_back_without_deadlock():
    failing = SemanticBackend(
        "raises", capabilities(PlanningPath.FAST), {PlanningPath.FAST: RuntimeError("boom")}
    )
    succeeding = SemanticBackend("fallback", capabilities(PlanningPath.COLD))
    session = ContinuousPlanningSession((failing, succeeding), validator=SemanticValidator())
    session.submit(request("exception", world()))
    session.run_until_stable()

    assert session.state is SessionState.READY
    report = session.statistics.report()
    assert report["outcome_count_by_backend_category"]["raises"]["OPERATIONAL:BACKEND_ERROR"] == 1


def test_unavailable_health_profile_falls_back_without_calling_plan():
    unavailable = SemanticBackend(
        "offline",
        capabilities(PlanningPath.FAST),
        health=BackendHealth.UNAVAILABLE,
    )
    fallback = SemanticBackend("fallback", capabilities(PlanningPath.COLD))
    session = ContinuousPlanningSession((unavailable, fallback), validator=SemanticValidator())
    session.submit(request("health", world()))
    session.run_until_stable()

    assert unavailable.calls == []
    assert session.state is SessionState.READY


def test_validation_exception_fails_closed_to_recovery():
    session = ContinuousPlanningSession(
        SemanticBackend("proposal", capabilities(PlanningPath.FAST)),
        validator=SemanticValidator(raises=True),
    )
    session.submit(request("validation-error", world()))
    session.run_until_stable()

    assert session.state is SessionState.RECOVERY
    assert not session.ready_plans


def test_model_fingerprint_change_invalidates_speculative_plan():
    backend = SemanticBackend("planner", capabilities(PlanningPath.FAST))
    session = ContinuousPlanningSession(backend, validator=SemanticValidator(), rolling_horizon=2)
    first = world(0, ("a", "b"))
    session.submit(request("k", first))
    session.run_until_stable()
    plan = session.start_execution()
    predicted = world(1, ("b",), q=plan.expected_end_state)
    session.submit_speculative(request("k+1", predicted))
    session.run_until_stable()

    changed_model = replace(
        predicted,
        config_identity={
            "config": "test",
            "robot_model_fingerprint": ROBOT_MODEL,
            "world_model_fingerprint": "world-model-test-v2",
        },
    )
    session.complete_execution(
        success=True,
        stopped_q=plan.expected_end_state,
        world_snapshot=changed_model,
    )

    assert not session.ready_plans
    assert any("fingerprint" in item.validation_message for item in session.invalidated_plans)


def test_lifecycle_and_latency_metrics_are_separate():
    class Clock:
        value = 0.0

        def __call__(self):
            return self.value

    clock = Clock()
    backend = SemanticBackend(
        "lifecycle",
        capabilities(PlanningPath.COLD, initialize=True, prewarm=True),
        health=BackendHealth.NEW,
        clock=clock,
    )
    session = ContinuousPlanningSession(backend, validator=SemanticValidator(), clock=clock)
    session.initialize_backends()
    session.submit(request("latency", world()))
    session.run_until_stable()
    report = session.statistics.report()

    assert report["backend_init_latency"]["lifecycle"]["total_seconds"] == pytest.approx(2.0)
    assert report["backend_warmup_latency"]["lifecycle"]["total_seconds"] == pytest.approx(3.0)
    assert report["planning_compute_latency"]["total_seconds"] == pytest.approx(0.0)
    assert report["production_throughput"] is None


def test_all_backends_exhausted_is_explicit_blocked_without_trajectory():
    unavailable = SemanticBackend(
        "unavailable",
        capabilities(PlanningPath.FAST),
        {PlanningPath.FAST: OperationalOutcome.BACKEND_UNAVAILABLE},
    )
    failed = SemanticBackend(
        "domain-failure", capabilities(PlanningPath.COLD), {PlanningPath.COLD: PlanStatus.COLLISION}
    )
    session = ContinuousPlanningSession((unavailable, failed), validator=SemanticValidator())
    session.submit(request("exhausted", world()))
    session.run_until_stable()

    assert session.state is SessionState.BLOCKED
    assert not session.ready_plans
    assert all(not attempt.success for attempt in session.terminal_results)


def test_control_plane_has_no_specific_planner_or_accelerator_dependency():
    source = Path("src/unloading_sim/online_planning.py").read_text(encoding="utf-8").lower()
    project = Path("pyproject.toml").read_text(encoding="utf-8").lower()
    forbidden = ("vamp", "curobo", "coad", "ompl", "cuda")
    assert not any(re.search(rf"\b{token}\b", source) for token in forbidden)
    assert not any(re.search(rf"\b{token}\b", project) for token in forbidden)
