"""Development acceptance runner for provider-neutral online backends.

This module is tooling, not part of the online runtime dependency graph.  A
provider supplies factories and deterministic scenes; this runner exercises
the existing session/runtime rather than implementing another scheduler.
"""

from __future__ import annotations

import argparse
import importlib
import json
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import subprocess
from typing import Any, Callable, Mapping, Sequence

from unloading_sim.online_execution import (
    ExecutionBackend,
    ExecutionBackendCapabilities,
    ExecutionBackendHealth,
    ExecutionBackendIdentity,
)
from unloading_sim.online_planning import (
    BackendCapabilities,
    BackendIdentity,
    ContinuousPlanningSession,
    CooperativePlanningExecutor,
    OperationalOutcome,
    PlanStatus,
    PlanValidationResult,
    PlanValidator,
    PlanningCandidate,
    PlanningExecutor,
    PlanningPath,
    PlanningRequest,
    PlanningWorldSnapshot,
    PlannerBackend,
    ReplanReason,
    SessionState,
    ValidationStatus,
)
from unloading_sim.online_runtime import (
    ContinuousPlanningRuntime,
    ObservationAuthority,
    RuntimeState,
    SuccessorRequestFactory,
    WorldObservation,
)


class AcceptanceStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    NOT_EVALUATED = "NOT_EVALUATED"


PlannerFactory = Callable[[str, int], PlannerBackend | Sequence[PlannerBackend] | None]
ValidatorFactory = Callable[[], PlanValidator]
ExecutionFactory = Callable[[], ExecutionBackend]
WorldFactory = Callable[[int, tuple[str, ...], tuple[float, ...]], PlanningWorldSnapshot]
RequestFactory = Callable[[str, PlanningWorldSnapshot], PlanningRequest]
ExecutorFactory = Callable[[], PlanningExecutor]


@dataclass(frozen=True)
class BackendAcceptanceProfile:
    profile_id: str
    planner_factory: PlannerFactory
    validator_factory: ValidatorFactory
    execution_factory: ExecutionFactory
    world_factory: WorldFactory
    request_factory: RequestFactory
    seed: int
    executor_factory: ExecutorFactory = CooperativePlanningExecutor
    declared_optional_cases: frozenset[str] = field(default_factory=frozenset)
    replay_parameters: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.profile_id:
            raise ValueError("acceptance profile_id must be non-empty")
        object.__setattr__(self, "seed", int(self.seed))
        object.__setattr__(self, "declared_optional_cases", frozenset(self.declared_optional_cases))


@dataclass(frozen=True)
class AcceptanceCase:
    category: str
    case_id: str
    profile_id: str | None
    required: bool
    status: AcceptanceStatus
    reason: str
    backend_identity: Mapping[str, Any] | None = None
    capabilities: Mapping[str, Any] | None = None
    seed: int | None = None
    replay: Mapping[str, Any] = field(default_factory=dict)


class _RejectingValidator(PlanValidator):
    def __init__(self) -> None:
        super().__init__("acceptance-reject-all", "1")

    def validate(self, envelope, snapshot, boundary):
        return PlanValidationResult(
            ValidationStatus.INVALID,
            snapshot,
            boundary,
            "intentional acceptance rejection",
            self.name,
            self.version,
        )


class _SuccessorFactory(SuccessorRequestFactory):
    def __init__(self, profile: BackendAcceptanceProfile) -> None:
        self.profile = profile
        self.predicted: PlanningWorldSnapshot | None = None
        self.calls = 0

    def create_successor(self, active_plan, current_world_snapshot):
        self.calls += 1
        cartons = tuple(current_world_snapshot.scene_snapshot["cartons"])
        if len(cartons) < 2:
            return None
        self.predicted = self.profile.world_factory(
            current_world_snapshot.scene_revision.sequence + 1,
            cartons[1:],
            active_plan.expected_end_boundary.q,
        )
        request = self.profile.request_factory(
            f"{active_plan.request.request_id}-successor",
            self.predicted,
        )
        return PlanningRequest(
            request.request_id,
            request.world_snapshot,
            request.candidates,
            seed=request.seed,
            horizon_index=1,
            speculative=True,
            replan_reason=ReplanReason.ROLLING_HORIZON,
            metadata=request.metadata,
            motion_boundary=active_plan.expected_end_boundary,
        )


def _planner_list(value: PlannerBackend | Sequence[PlannerBackend] | None) -> tuple[PlannerBackend, ...]:
    if value is None:
        raise LookupError("profile does not provide this deterministic scenario")
    if isinstance(value, PlannerBackend):
        return (value,)
    result = tuple(value)
    if not result or not all(isinstance(item, PlannerBackend) for item in result):
        raise TypeError("planner_factory must return PlannerBackend or a non-empty sequence")
    return result


def _identity(backend: PlannerBackend | ExecutionBackend) -> dict[str, Any]:
    return asdict(backend.identity)


def _capabilities(value: BackendCapabilities | ExecutionBackendCapabilities) -> dict[str, Any]:
    data = asdict(value)
    return {
        key: sorted(item.value for item in val) if isinstance(val, frozenset) else val
        for key, val in data.items()
    }


class _Recorder:
    def __init__(self, profile: BackendAcceptanceProfile | None) -> None:
        self.profile = profile
        self.results: list[AcceptanceCase] = []

    def run(self, category: str, case_id: str, required: bool, operation: Callable[[], str]) -> None:
        profile_id = None if self.profile is None else self.profile.profile_id
        replay = {} if self.profile is None else dict(self.profile.replay_parameters)
        identity = capabilities = None
        seed = None if self.profile is None else self.profile.seed
        try:
            reason = operation()
            status = AcceptanceStatus.PASS
            if self.profile is not None and category == "backend_compliance":
                backends = _planner_list(self.profile.planner_factory("success", self.profile.seed))
                identity = _identity(backends[0])
                capabilities = _capabilities(backends[0].capabilities)
        except LookupError as exc:
            status = AcceptanceStatus.NOT_EVALUATED
            reason = str(exc)
            if required:
                status = AcceptanceStatus.FAIL
        except Exception as exc:
            status = AcceptanceStatus.FAIL
            reason = f"{type(exc).__name__}: {exc}"
        self.results.append(
            AcceptanceCase(
                category,
                case_id,
                profile_id,
                required,
                status,
                reason,
                identity,
                capabilities,
                seed,
                replay,
            )
        )

    def not_evaluated(self, category: str, case_id: str, reason: str) -> None:
        self.results.append(
            AcceptanceCase(
                category,
                case_id,
                None if self.profile is None else self.profile.profile_id,
                False,
                AcceptanceStatus.NOT_EVALUATED,
                reason,
                seed=None if self.profile is None else self.profile.seed,
                replay={} if self.profile is None else dict(self.profile.replay_parameters),
            )
        )


def _new_session(profile: BackendAcceptanceProfile, scenario: str, *, validator=None, executor=None):
    planners = _planner_list(profile.planner_factory(scenario, profile.seed))
    return ContinuousPlanningSession(
        planners,
        validator=profile.validator_factory() if validator is None else validator,
        executor=profile.executor_factory() if executor is None else executor,
        rolling_horizon=2,
    )


def _success_request(profile: BackendAcceptanceProfile, request_id: str, cartons=("a",)):
    world = profile.world_factory(0, tuple(cartons), (0.0, 0.0))
    return profile.request_factory(request_id, world)


def _case_identity_and_lifecycle(profile: BackendAcceptanceProfile) -> str:
    planners = _planner_list(profile.planner_factory("success", profile.seed))
    for backend in planners:
        if not isinstance(backend.identity, BackendIdentity):
            raise TypeError("planner identity is not BackendIdentity")
        if not isinstance(backend.capabilities, BackendCapabilities):
            raise TypeError("planner capabilities are not BackendCapabilities")
        if backend.capabilities.supports_deterministic_seed and backend.seed != profile.seed:
            raise AssertionError("deterministic seed was not preserved by planner factory")
        backend.initialize()
        backend.prewarm()
        backend.shutdown()
    execution = profile.execution_factory()
    if not isinstance(execution.identity, ExecutionBackendIdentity):
        raise TypeError("execution identity is not ExecutionBackendIdentity")
    if not isinstance(execution.capabilities, ExecutionBackendCapabilities):
        raise TypeError("execution capabilities are not ExecutionBackendCapabilities")
    execution.initialize()
    execution.shutdown()
    execution.shutdown()
    if execution.health is not ExecutionBackendHealth.SHUTDOWN:
        raise AssertionError("execution shutdown did not expose SHUTDOWN health")
    return "identity, capabilities, deterministic seed, and lifecycle are contract-compliant"


def _case_success_validation(profile: BackendAcceptanceProfile) -> str:
    request = _success_request(profile, "accept-success")
    session = _new_session(profile, "success")
    session.submit(request)
    session.run_until_stable()
    if session.state is not SessionState.READY or not session.ready_plans:
        raise AssertionError(f"feasible proposal did not reach READY: {session.state.value}")
    envelope = session.ready_plans[0]
    if not envelope.validated_by or not envelope.executable:
        raise AssertionError("successful proposal bypassed or failed authoritative validation")
    session.executor.shutdown(wait=True, cancel_pending=True)

    rejected = _new_session(profile, "success", validator=_RejectingValidator())
    rejected.submit(_success_request(profile, "accept-rejected"))
    rejected.run_until_stable()
    if rejected.ready_plans or rejected.state is not SessionState.BLOCKED:
        raise AssertionError("backend SUCCESS bypassed an authoritative rejection")
    rejected.executor.shutdown(wait=True, cancel_pending=True)
    return "SUCCESS reaches READY only after the supplied validator and cannot bypass rejection"


def _case_path_capabilities(profile: BackendAcceptanceProfile) -> str:
    session = _new_session(profile, "success")
    session.submit(_success_request(profile, "accept-paths"))
    session.run_until_stable()
    supported = set().union(*(backend.capabilities.supported_planning_paths for backend in session.backends))
    expected_skips = len(set(PlanningPath) - supported)
    actual_skips = session.statistics.report()["unsupported_route_skip_count"]
    if actual_skips < expected_skips:
        raise AssertionError("unsupported planning paths were not explicitly skipped")
    if session.statistics.report()["planning_status_counts"].get("SUCCESS", 0) != 1:
        raise AssertionError("unsupported routes were counted as planning failures or extra successes")
    session.executor.shutdown(wait=True, cancel_pending=True)
    return f"scheduler used declared paths and recorded at least {expected_skips} unsupported skips"


def _case_failure(profile: BackendAcceptanceProfile, scenario: str, expected: str, operational: bool) -> str:
    session = _new_session(profile, scenario)
    session.submit(_success_request(profile, f"accept-{scenario.replace(':', '-') }"))
    session.run_until_stable()
    results = session.terminal_results
    allowed_states = (
        {SessionState.WAITING_FOR_SCENE, SessionState.BLOCKED}
        if expected == PlanStatus.NOT_EVALUATED.value
        else {SessionState.BLOCKED}
    )
    if session.ready_plans or session.state not in allowed_states or not results:
        raise AssertionError(
            f"failure scenario ended in {session.state.value}, expected "
            f"{sorted(state.value for state in allowed_states)} without READY"
        )
    last = results[-1]
    actual = last.operational_outcome.value if operational and last.operational_outcome else last.status.value
    if actual != expected:
        raise AssertionError(f"expected {expected}, received {actual}")
    if bool(last.operational_outcome) is not operational:
        raise AssertionError("domain and operational outcome categories were conflated")
    if last.trajectory:
        raise AssertionError("failed result manufactured a trajectory")
    session.executor.shutdown(wait=True, cancel_pending=True)
    return f"{expected} remained an explicit {'operational' if operational else 'domain'} failure"


def _case_candidate_fallback(profile: BackendAcceptanceProfile) -> str:
    session = _new_session(profile, "candidate-fallback")
    base = _success_request(profile, "accept-candidate-fallback")
    session.submit(
        replace(
            base,
            candidates=(
                PlanningCandidate("a", "bad"),
                PlanningCandidate("a", "good"),
            ),
        )
    )
    session.run_until_stable()
    if session.state is not SessionState.READY or not session.ready_plans:
        raise AssertionError("candidate-local failure did not permit the next candidate")
    if session.ready_plans[0].result.candidate_id != "good":
        raise AssertionError("candidate fallback selected an unexpected proposal")
    session.executor.shutdown(wait=True, cancel_pending=True)
    return "candidate-local failure advanced to the next candidate without manufacturing success"


def _case_runtime_handoff(profile: BackendAcceptanceProfile) -> str:
    executor = profile.executor_factory()
    session = _new_session(profile, "success", executor=executor)
    execution = profile.execution_factory()
    successor = _SuccessorFactory(profile)
    runtime = ContinuousPlanningRuntime(
        session,
        execution,
        successor,
        owns_executor=True,
        owns_execution_backend=True,
    )
    runtime.submit_initial(_success_request(profile, "accept-k", cartons=("a", "b")))
    state = runtime.state
    for _ in range(100):
        state = runtime.step()
        if state is RuntimeState.WAITING_FOR_SCENE:
            break
        if not isinstance(executor, CooperativePlanningExecutor) and (
            executor.pending_count or executor.running_count
        ):
            executor.wait_for_completion(3.0)
    if state is not RuntimeState.WAITING_FOR_SCENE or successor.predicted is None:
        raise AssertionError(f"k/k+1 scenario did not await authoritative scene: {state.value}")
    if not session.speculative_plans:
        raise AssertionError("k+1 was not planned while k executed")
    predicted = successor.predicted
    observed = profile.world_factory(
        predicted.scene_revision.sequence,
        tuple(predicted.scene_snapshot["cartons"]),
        session.last_actual_execution_boundary.q,
    )
    ingress = runtime.observe_world(
        WorldObservation(
            ObservationAuthority.AUTHORITATIVE,
            "acceptance-perception",
            "world",
            0,
            observed.scene_revision.sequence,
            float(observed.scene_revision.sequence + 1),
            observed,
            "acceptance observed scene",
        )
    )
    if ingress.status.value != "ACCEPTED":
        raise AssertionError(f"authoritative observation rejected: {ingress.status.value}")
    for _ in range(20):
        runtime.step()
        if runtime.metrics.execution_start_command_count == 2:
            break
    if runtime.metrics.execution_start_command_count != 2:
        raise AssertionError("matching authoritative scene did not reuse/start k+1 exactly once")
    if not any(event.kind == "speculative_reused" for event in session.events):
        raise AssertionError("speculative reuse was not audited")
    runtime.shutdown()
    return "factory substitution preserved k execution, speculative k+1, scene reconciliation, and one handoff"


def _case_runtime_scene_mismatch(profile: BackendAcceptanceProfile) -> str:
    executor = profile.executor_factory()
    session = _new_session(profile, "success", executor=executor)
    execution = profile.execution_factory()
    successor = _SuccessorFactory(profile)
    runtime = ContinuousPlanningRuntime(
        session,
        execution,
        successor,
        owns_executor=True,
        owns_execution_backend=True,
    )
    runtime.submit_initial(_success_request(profile, "accept-mismatch-k", cartons=("a", "b")))
    for _ in range(100):
        state = runtime.step()
        if state is RuntimeState.WAITING_FOR_SCENE:
            break
        if not isinstance(executor, CooperativePlanningExecutor) and (
            executor.pending_count or executor.running_count
        ):
            executor.wait_for_completion(3.0)
    if runtime.state is not RuntimeState.WAITING_FOR_SCENE or successor.predicted is None:
        raise AssertionError("mismatch scenario did not produce a speculative successor")
    actual = profile.world_factory(
        successor.predicted.scene_revision.sequence,
        ("b", "unexpected"),
        session.last_actual_execution_boundary.q,
    )
    runtime.observe_world(
        WorldObservation(
            ObservationAuthority.AUTHORITATIVE,
            "acceptance-perception",
            "world",
            0,
            actual.scene_revision.sequence,
            float(actual.scene_revision.sequence + 1),
            actual,
            "acceptance mismatching scene",
        )
    )
    for _ in range(100):
        runtime.step()
        if runtime.metrics.execution_start_command_count == 2:
            break
        if not isinstance(executor, CooperativePlanningExecutor) and (
            executor.pending_count or executor.running_count
        ):
            executor.wait_for_completion(3.0)
    if not any(
        plan.invalidated_by is ReplanReason.SPECULATIVE_MISMATCH
        for plan in session.invalidated_plans
    ):
        raise AssertionError("mismatching authoritative scene did not invalidate speculation")
    if runtime.metrics.execution_start_command_count != 2:
        raise AssertionError("scene mismatch did not replan and start exactly one replacement")
    runtime.shutdown()
    return "authoritative mismatch invalidated speculation and replanned through the configured factories"


def _run_reused_test_case(callable_spec: str) -> str:
    module_name, _, name = callable_spec.partition(":")
    operation = getattr(importlib.import_module(module_name), name)
    operation()
    return f"reused public-interface contract passed: {callable_spec}"


RUNTIME_TOLERANCE_CASES = {
    "runtime.unknown_start_contained": "tests.test_online_stop_lifecycle:test_problem_b_start_response_loss_is_contained_without_retry_or_fake_identity",
    "runtime.faulted_requires_correlated_stop": "tests.test_online_stop_lifecycle:test_case_b_faulted_task_can_later_accept_correlated_stopped_evidence",
    "runtime.stop_conflict_blocks_handoff": "tests.test_online_stop_lifecycle:test_received_stop_conflict_backlog_blocks_successor_before_evidence_retirement",
    "runtime.finite_feedback_backlog": "tests.test_online_stop_lifecycle:test_finite_duplicate_backlog_clears_before_single_handoff_and_late_conflict_is_history",
    "runtime.reconfirm_and_explicit_recovery": "tests.test_online_stop_lifecycle:test_reconciled_conflict_can_reestablish_evidence_then_retire_on_accepted_handoff",
    "runtime.feedback_capacity_bounded": "tests.test_online_stop_lifecycle:test_feedback_capacity_boundary_remains_bounded_and_cannot_hide_conflict",
    "runtime.explicit_start_rejection": "tests.test_online_stop_lifecycle:test_explicit_start_rejection_is_not_treated_as_an_ambiguous_start",
    "runtime.late_planning_completion_isolated": "tests.test_online_threaded_executor:test_scene_change_discards_late_old_generation_result_after_logical_cancel",
}

NEUTRAL_CONTRACT_CASES = {
    "planner.model_fingerprint_mismatch_rejected": "tests.test_online_backend_contracts:test_backend_model_fingerprint_mismatch_never_reaches_validator_or_ready",
    "planner.speculative_model_change_invalidates": "tests.test_online_backend_contracts:test_model_fingerprint_change_invalidates_speculative_plan",
    "planner.geometric_continuous_boundary_rejected": "tests.test_online_backend_contracts:test_geometric_path_cannot_satisfy_continuous_motion_boundary",
}


def _check_bottom_alternative_fixture() -> str:
    path = Path(__file__).resolve().parents[1] / "tests" / "fixtures" / "m710id70_v3_bottom_alternative.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "online_planning_integration_fixture_v1":
        raise AssertionError("unexpected bottom-alternative fixture schema")
    if payload.get("recorded_result", {}).get("status") != "SUCCESS":
        raise AssertionError("bottom-alternative recorded result is missing")
    return (
        "record integrity verified for independently recorded bottom-alternative witness; "
        f"archive_sha256={payload['evidence']['archive_sha256']}"
    )


def run_acceptance(
    profiles: Sequence[BackendAcceptanceProfile],
    *,
    include_runtime_tolerance: bool = True,
) -> dict[str, Any]:
    results: list[AcceptanceCase] = []
    domain = (
        PlanStatus.NO_IK,
        PlanStatus.GRASP_CONSTRAINT_FAILED,
        PlanStatus.INITIAL_CLEARANCE_FAILED,
        PlanStatus.COLLISION,
        PlanStatus.NOT_EVALUATED,
    )
    operational = (
        OperationalOutcome.BACKEND_ERROR,
        OperationalOutcome.DEADLINE_EXCEEDED,
        OperationalOutcome.CACHE_MISS,
        OperationalOutcome.CANCELLED,
        OperationalOutcome.BACKEND_UNAVAILABLE,
        OperationalOutcome.PATH_UNSUPPORTED,
    )
    for profile in profiles:
        recorder = _Recorder(profile)
        recorder.run("backend_compliance", "backend.identity_capabilities_lifecycle", True, lambda p=profile: _case_identity_and_lifecycle(p))
        recorder.run("backend_compliance", "planner.success_requires_validation", True, lambda p=profile: _case_success_validation(p))
        recorder.run("backend_compliance", "planner.path_capability_selection", True, lambda p=profile: _case_path_capabilities(p))
        recorder.run("backend_compliance", "runtime.configured_backend_handoff", True, lambda p=profile: _case_runtime_handoff(p))
        recorder.run("backend_compliance", "runtime.scene_mismatch_invalidate_replan", True, lambda p=profile: _case_runtime_scene_mismatch(p))
        candidate_case = "planner.candidate_fallback"
        recorder.run(
            "backend_compliance",
            candidate_case,
            candidate_case in profile.declared_optional_cases,
            lambda p=profile: _case_candidate_fallback(p),
        )
        for outcome in domain:
            case_id = f"planner.domain.{outcome.value}"
            recorder.run(
                "backend_compliance",
                case_id,
                case_id in profile.declared_optional_cases,
                lambda p=profile, o=outcome: _case_failure(p, f"domain:{o.value}", o.value, False),
            )
        for outcome in operational:
            case_id = f"planner.operational.{outcome.value}"
            recorder.run(
                "backend_compliance",
                case_id,
                case_id in profile.declared_optional_cases,
                lambda p=profile, o=outcome: _case_failure(p, f"operational:{o.value}", o.value, True),
            )
        results.extend(recorder.results)

    if include_runtime_tolerance:
        neutral = _Recorder(None)
        for case_id, callable_spec in NEUTRAL_CONTRACT_CASES.items():
            neutral.run(
                "backend_compliance",
                case_id,
                True,
                lambda spec=callable_spec: _run_reused_test_case(spec),
            )
        results.extend(neutral.results)
        tolerance = _Recorder(None)
        for case_id, node_id in RUNTIME_TOLERANCE_CASES.items():
            tolerance.run("runtime_tolerance", case_id, True, lambda node=node_id: _run_reused_test_case(node))
        tolerance.run(
            "geometry_evidence",
            "v3.bottom_alternative.fixture_integrity",
            False,
            _check_bottom_alternative_fixture,
        )
        tolerance.not_evaluated(
            "geometry_evidence",
            "v3.bottom_alternative.geometry_rerun",
            "recorded integration witness only; this command does not rerun grasp, IK, collision, or geometric planning",
        )
        results.extend(tolerance.results)

    required = [item for item in results if item.required]
    overall = bool(required) and all(item.status is AcceptanceStatus.PASS for item in required)
    try:
        package_version = version("unloading-layer1-sim")
    except PackageNotFoundError:
        package_version = "unknown"
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).resolve().parents[1],
            text=True,
            capture_output=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        revision = "unknown"
    counts = {status.value: sum(item.status is status for item in results) for status in AcceptanceStatus}
    profile_records = []
    for profile in profiles:
        try:
            planner = _planner_list(profile.planner_factory("success", profile.seed))[0]
            execution = profile.execution_factory()
            profile_records.append(
                {
                    "profile_id": profile.profile_id,
                    "seed": profile.seed,
                    "planner_identity": _identity(planner),
                    "planner_capabilities": _capabilities(planner.capabilities),
                    "execution_identity": _identity(execution),
                    "execution_capabilities": _capabilities(execution.capabilities),
                    "declared_optional_cases": sorted(profile.declared_optional_cases),
                    "replay_parameters": dict(profile.replay_parameters),
                }
            )
            execution.shutdown()
        except Exception as exc:
            profile_records.append(
                {
                    "profile_id": profile.profile_id,
                    "seed": profile.seed,
                    "inspection_error": f"{type(exc).__name__}: {exc}",
                    "replay_parameters": dict(profile.replay_parameters),
                }
            )
    return {
        "schema_version": 1,
        "package_version": package_version,
        "code_revision": revision or "unknown",
        "required_contracts_passed": overall,
        "counts": counts,
        "profiles": profile_records,
        "runtime_tolerance_replay": dict(RUNTIME_TOLERANCE_CASES),
        "neutral_contract_replay": dict(NEUTRAL_CONTRACT_CASES),
        "latency_scope": {
            "source": "existing ContinuousPlanningSession/Runtime control-plane metrics",
            "feedback_or_fault_wait_in_waiting_for_planner": False,
            "production_throughput": None,
        },
        "cases": [
            {
                **asdict(item),
                "status": item.status.value,
            }
            for item in results
        ],
    }


def _load_profiles(spec: str) -> Sequence[BackendAcceptanceProfile]:
    module_name, separator, attribute = spec.partition(":")
    if not separator or not module_name or not attribute:
        raise ValueError("profile spec must use module:callable")
    factory = getattr(importlib.import_module(module_name), attribute)
    profiles = tuple(factory())
    required = (
        "profile_id",
        "planner_factory",
        "validator_factory",
        "execution_factory",
        "world_factory",
        "request_factory",
        "seed",
        "executor_factory",
        "declared_optional_cases",
        "replay_parameters",
    )
    if not profiles or not all(all(hasattr(item, name) for name in required) for item in profiles):
        raise TypeError("profile factory must return BackendAcceptanceProfile objects")
    return profiles


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profiles",
        default="tests.online_acceptance_profiles:default_profiles",
        help="module:callable returning BackendAcceptanceProfile objects",
    )
    parser.add_argument("--output", type=Path, required=True, help="untracked JSON report path")
    args = parser.parse_args(argv)
    report = run_acceptance(_load_profiles(args.profiles))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(
        f"online backend acceptance: {'PASS' if report['required_contracts_passed'] else 'FAIL'} "
        f"PASS={report['counts']['PASS']} FAIL={report['counts']['FAIL']} "
        f"SKIP={report['counts']['SKIP']} NOT_EVALUATED={report['counts']['NOT_EVALUATED']}"
    )
    print(f"report: {args.output}")
    return 0 if report["required_contracts_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
