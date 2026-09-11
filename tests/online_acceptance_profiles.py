"""Built-in CPU-only profiles for the online backend acceptance command."""

from __future__ import annotations

from dataclasses import replace

from unloading_sim.online_execution import DeterministicSimExecutionBackend
from unloading_sim.online_planning import (
    CooperativePlanningExecutor,
    OperationalOutcome,
    PlanStatus,
    PlanningPath,
    ThreadedPlanningExecutor,
)
from tests.online_contract_fixtures import (
    ColdOnlyBackend,
    ScriptedExecutionBackend,
    SemanticBackend,
    SemanticValidator,
    semantic_capabilities,
    semantic_request,
    semantic_world,
)
from tools.online_backend_acceptance import BackendAcceptanceProfile


DOMAIN_CASES = frozenset(
    f"planner.domain.{status.value}"
    for status in (
        PlanStatus.NO_IK,
        PlanStatus.GRASP_CONSTRAINT_FAILED,
        PlanStatus.INITIAL_CLEARANCE_FAILED,
        PlanStatus.COLLISION,
        PlanStatus.NOT_EVALUATED,
    )
)
OPERATIONAL_CASES = frozenset(
    f"planner.operational.{outcome.value}"
    for outcome in (
        OperationalOutcome.BACKEND_ERROR,
        OperationalOutcome.DEADLINE_EXCEEDED,
        OperationalOutcome.CACHE_MISS,
        OperationalOutcome.CANCELLED,
        OperationalOutcome.BACKEND_UNAVAILABLE,
        OperationalOutcome.PATH_UNSUPPORTED,
    )
)


def _semantic_factory(scenario: str, seed: int):
    paths = tuple(PlanningPath)
    if scenario == "success":
        return SemanticBackend("semantic-multipath", semantic_capabilities(*paths), seed=seed)
    if scenario == "candidate-fallback":
        return SemanticBackend(
            "semantic-multipath",
            semantic_capabilities(*paths),
            {("bad", path): PlanStatus.NO_IK for path in paths},
            seed=seed,
        )
    category, separator, value = scenario.partition(":")
    if not separator:
        return None
    if category == "domain":
        outcome = PlanStatus(value)
    elif category == "operational":
        outcome = OperationalOutcome(value)
    else:
        return None
    return SemanticBackend(
        "semantic-multipath",
        semantic_capabilities(*paths),
        {path: outcome for path in paths},
        seed=seed,
    )


def _cold_factory(scenario: str, seed: int):
    if scenario == "success":
        return ColdOnlyBackend(seed=seed)
    category, separator, value = scenario.partition(":")
    if category == "domain" and separator:
        return ColdOnlyBackend(seed=seed, outcome=PlanStatus(value))
    return None


def _world(sequence: int, cartons: tuple[str, ...], q: tuple[float, ...]):
    return semantic_world(sequence, cartons, q)


def default_profiles():
    """Return independent planner/executor combinations used by CI and adapters."""

    return (
        BackendAcceptanceProfile(
            "semantic-multipath+deterministic-execution+cooperative",
            _semantic_factory,
            SemanticValidator,
            lambda: DeterministicSimExecutionBackend(execution_steps=2),
            _world,
            semantic_request,
            seed=17,
            executor_factory=CooperativePlanningExecutor,
            declared_optional_cases=(
                DOMAIN_CASES | OPERATIONAL_CASES | {"planner.candidate_fallback"}
            ),
            replay_parameters={"scene": "synthetic-two-dof", "seed": 17},
        ),
        BackendAcceptanceProfile(
            "independent-cold+scripted-execution+threaded",
            _cold_factory,
            SemanticValidator,
            lambda: ScriptedExecutionBackend(auto_advance=True),
            _world,
            semantic_request,
            seed=23,
            executor_factory=ThreadedPlanningExecutor,
            declared_optional_cases=DOMAIN_CASES,
            replay_parameters={"scene": "synthetic-two-dof", "seed": 23},
        ),
    )


def intentional_invalid_profiles():
    """One replayable contract violation used to prove nonzero CLI behavior."""

    return (
        replace(
            default_profiles()[0],
            profile_id="intentional-invalid-planner-factory",
            planner_factory=lambda scenario, seed: object(),
            replay_parameters={"fault": "planner factory returned wrong type"},
        ),
    )
