"""Reusable deterministic fixtures for online backend contract acceptance.

These fixtures intentionally live under ``tests``.  Production runtime modules
do not import them; the development acceptance command and pytest suites do.
"""

from __future__ import annotations

from collections import deque

from unloading_sim.online_execution import (
    ExecutionBackend,
    ExecutionBackendCapabilities,
    ExecutionBackendHealth,
    ExecutionBackendIdentity,
    ExecutionBackendState,
    ExecutionCommandResult,
    ExecutionCommandStatus,
    ExecutionFeedback,
    ExecutionFeedbackStatus,
)
from unloading_sim.online_planning import (
    ArtifactKind,
    BackendCapabilities,
    BackendHealth,
    BackendIdentity,
    BoundaryMode,
    FailureDetails,
    FailureScope,
    MotionBoundaryState,
    OperationalOutcome,
    PlanArtifactKind,
    PlanStatus,
    PlanValidationResult,
    PlanValidator,
    PlanningCandidate,
    PlanningPath,
    PlanningRequest,
    PlanningResult,
    PlanningWorldSnapshot,
    PlannerBackend,
    RobotStateRevision,
    SceneRevision,
    ValidationStatus,
)
from unloading_sim.online_runtime import ObservationAuthority, WorldObservation


ROBOT_MODEL = "robot-model-test-v1"
WORLD_MODEL = "world-model-test-v1"


class FakeClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def semantic_world(
    sequence: int = 0,
    cartons=("a",),
    q=(0.0, 0.0),
    *,
    robot_model: str = ROBOT_MODEL,
    world_model: str = WORLD_MODEL,
    source: str = "semantic-contract-test",
) -> PlanningWorldSnapshot:
    scene = {"cartons": list(cartons)}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, sequence, source=source),
        scene,
        RobotStateRevision(sequence, q, {"mode": "AUTO"}),
        {"id": "tool-v1"},
        None,
        {"x_m": -0.5},
        {"extension_m": 0.0},
        {
            "config": "test",
            "robot_model_fingerprint": robot_model,
            "world_model_fingerprint": world_model,
        },
    )


def semantic_request(
    request_id: str,
    snapshot: PlanningWorldSnapshot,
    *,
    boundary: BoundaryMode = BoundaryMode.STOP_BOUNDARY,
    candidates: tuple[PlanningCandidate, ...] | None = None,
) -> PlanningRequest:
    velocity = (
        tuple(0.0 for _ in snapshot.current_q)
        if boundary is BoundaryMode.STOP_BOUNDARY
        else tuple(0.1 for _ in snapshot.current_q)
    )
    acceleration = tuple(0.0 for _ in snapshot.current_q)
    return PlanningRequest(
        request_id,
        snapshot,
        candidates or (PlanningCandidate("a", "top"),),
        motion_boundary=MotionBoundaryState(
            snapshot.current_q,
            velocity,
            acceleration,
            0.0,
            boundary,
        ),
    )


def semantic_identity(
    name: str = "semantic",
    *,
    robot_model: str = ROBOT_MODEL,
    world_model: str = WORLD_MODEL,
) -> BackendIdentity:
    return BackendIdentity(name, "1.0", "1.0", robot_model, world_model)


def semantic_capabilities(
    *paths: PlanningPath,
    boundaries=(BoundaryMode.STOP_BOUNDARY,),
    artifacts=(ArtifactKind.GEOMETRIC_PATH,),
    initialize: bool = False,
    prewarm: bool = False,
) -> BackendCapabilities:
    return BackendCapabilities(
        supported_planning_paths=frozenset(paths),
        supported_boundary_modes=frozenset(boundaries),
        supported_artifact_kinds=frozenset(artifacts),
        supports_warm_start=PlanningPath.WARM in paths,
        supports_deterministic_seed=True,
        supports_attached_object=True,
        supports_incremental_world_update=False,
        supports_logical_cancel=True,
        supports_hard_deadline=False,
        supports_revalidation=False,
        initialization_required=initialize,
        prewarm_required=prewarm,
    )


class SemanticBackend(PlannerBackend):
    """Configurable semantic planner used only for contract acceptance."""

    def __init__(self, name, caps, outcomes=None, *, health=BackendHealth.READY, clock=None, seed=7):
        super().__init__(identity=semantic_identity(name), capabilities=caps, seed=seed)
        self.outcomes = {} if outcomes is None else dict(outcomes)
        self.calls = []
        self._health = health
        self.clock = clock
        self.cancelled = []

    @property
    def health(self):
        return self._health

    def initialize(self):
        if self.clock:
            self.clock.value += 2.0
        self._health = BackendHealth.READY

    def prewarm(self):
        if self.clock:
            self.clock.value += 3.0

    def logical_cancel(self, request_id):
        self.cancelled.append(request_id)

    def plan(self, planning_request, candidate, planning_path):
        self.calls.append(planning_path)
        outcome = self.outcomes.get(
            (candidate.candidate_id, planning_path),
            self.outcomes.get(planning_path, PlanStatus.SUCCESS),
        )
        if isinstance(outcome, Exception):
            raise outcome
        if isinstance(outcome, OperationalOutcome):
            return PlanningResult.operational_failure(
                outcome,
                candidate,
                failure=FailureDetails(
                    retryable=True,
                    scope=FailureScope.BACKEND_LOCAL,
                    native_code=outcome.value,
                    native_message="fake operational result",
                    allow_path_fallback=True,
                    allow_backend_fallback=True,
                ),
            )
        if outcome is not PlanStatus.SUCCESS:
            return PlanningResult.failed(outcome, candidate)
        q = planning_request.motion_boundary.current_q
        end_q = tuple(value + 0.1 for value in q)
        artifact = next(iter(self.capabilities.supported_artifact_kinds))
        if artifact is ArtifactKind.GEOMETRIC_PATH:
            return PlanningResult.succeeded(candidate, [q, end_q], artifact_kind=artifact)
        start = planning_request.motion_boundary
        end = MotionBoundaryState(
            end_q,
            start.qd,
            start.qdd,
            start.time_seconds + 1.0,
            start.boundary_mode,
        )
        return PlanningResult.succeeded(
            candidate,
            [q, end_q],
            artifact_kind=artifact,
            expected_start_boundary=start,
            expected_end_boundary=end,
        )


class ColdOnlyBackend(PlannerBackend):
    """Independent cold-path implementation for backend substitution checks."""

    def __init__(self, *, seed: int = 11, outcome: PlanStatus = PlanStatus.SUCCESS) -> None:
        super().__init__(
            seed=seed,
            identity=semantic_identity("independent-cold"),
            capabilities=semantic_capabilities(PlanningPath.COLD),
        )
        self.outcome = PlanStatus(outcome)
        self.calls: list[PlanningPath] = []

    def plan(self, request, candidate, planning_path):
        self.calls.append(PlanningPath(planning_path))
        if self.outcome is not PlanStatus.SUCCESS:
            return PlanningResult.failed(self.outcome, candidate)
        start = request.motion_boundary
        end = MotionBoundaryState.stopped(
            tuple(value + 0.05 for value in start.q),
            time_seconds=start.time_seconds + 0.5,
        )
        return PlanningResult.succeeded(
            candidate,
            (start.q, end.q),
            expected_start_boundary=start,
            expected_end_boundary=end,
        )


class SemanticValidator(PlanValidator):
    def __init__(self, *, valid: bool = True, raises: bool = False):
        super().__init__("authoritative-test-validator", "1.0")
        self.valid = valid
        self.raises = raises
        self.calls = 0

    def validate(self, plan_envelope, current_world_snapshot, current_motion_boundary):
        self.calls += 1
        if self.raises:
            raise RuntimeError("validator unavailable")
        return PlanValidationResult(
            ValidationStatus.VALID if self.valid else ValidationStatus.INVALID,
            current_world_snapshot,
            current_motion_boundary,
            "accepted" if self.valid else "authoritative rejection",
            validator_name=self.name,
            validator_version=self.version,
        )


class ScriptedExecutionBackend(ExecutionBackend):
    """Manually driven execution fixture shared by lifecycle and acceptance tests."""

    def __init__(
        self,
        *,
        stop_status: ExecutionCommandStatus = ExecutionCommandStatus.ACCEPTED,
        stop_error: Exception | None = None,
        start_error_after_accept: Exception | None = None,
        start_result: object | None = None,
        auto_advance: bool = False,
    ) -> None:
        self._identity = ExecutionBackendIdentity("scripted-stop", "1", "1")
        self._capabilities = ExecutionBackendCapabilities(
            frozenset({PlanArtifactKind.GEOMETRIC_PATH}),
            frozenset({BoundaryMode.STOP_BOUNDARY}),
            True,
            True,
            True,
            True,
            True,
            True,
            True,
            False,
        )
        self.feedback = deque()
        self.plan = None
        self.execution_id = "execution-0"
        self.stop_calls = 0
        self.stop_command_id: str | None = None
        self.start_calls = 0
        self.stop_status = stop_status
        self.stop_error = stop_error
        self.start_error_after_accept = start_error_after_accept
        self.start_result = start_result
        self.auto_advance = bool(auto_advance)
        self._health = ExecutionBackendHealth.READY
        self._auto_phase = 0
        self._auto_sequence = 0
        self._last_boundary = None
        self._stop_requested = False

    @property
    def identity(self):
        return self._identity

    @property
    def capabilities(self):
        return self._capabilities

    @property
    def health(self):
        return self._health

    @property
    def state(self):
        if self._health is ExecutionBackendHealth.SHUTDOWN:
            return ExecutionBackendState.SHUTDOWN
        if self.plan is None:
            return ExecutionBackendState.IDLE
        return ExecutionBackendState.STOPPING if self._stop_requested else ExecutionBackendState.RUNNING

    def start(self, plan_envelope):
        self.start_calls += 1
        if self._health is not ExecutionBackendHealth.READY:
            return ExecutionCommandResult(
                ExecutionCommandStatus.BACKEND_UNAVAILABLE,
                f"start-{self.start_calls}",
                None,
                plan_envelope.plan_id,
            )
        self.execution_id = f"execution-{self.start_calls}"
        self.plan = plan_envelope
        self._last_boundary = plan_envelope.expected_start_boundary
        self._auto_phase = 0
        self._stop_requested = False
        if self.start_error_after_accept is not None:
            raise self.start_error_after_accept
        if self.start_result is not None:
            if callable(self.start_result):
                return self.start_result(plan_envelope, self.execution_id)
            return self.start_result
        result = ExecutionCommandResult(
            ExecutionCommandStatus.ACCEPTED,
            f"start-{self.start_calls}",
            self.execution_id,
            plan_envelope.plan_id,
        )
        if self.auto_advance:
            self.emit(
                self._auto_sequence,
                ExecutionFeedbackStatus.ACCEPTED,
                0.0,
                plan_envelope.expected_start_boundary,
            )
            self._auto_sequence += 1
        return result

    def poll(self):
        return self.feedback.popleft() if self.feedback else None

    def request_stop(self, plan_id, reason):
        del reason
        self.stop_calls += 1
        if self.stop_error is not None:
            raise self.stop_error
        self.stop_command_id = f"stop-{self.stop_calls}"
        result = ExecutionCommandResult(
            self.stop_status,
            self.stop_command_id,
            self.execution_id,
            plan_id,
        )
        self._stop_requested = result.status is ExecutionCommandStatus.ACCEPTED
        return result

    def current_boundary(self):
        return self._last_boundary

    def shutdown(self):
        self._health = ExecutionBackendHealth.SHUTDOWN

    def advance(self, max_steps=1):
        if not self.auto_advance or self.plan is None:
            return 0
        advanced = 0
        while self.plan is not None and advanced < max_steps:
            if self._stop_requested:
                if self._auto_phase == 0:
                    self.emit(
                        self._auto_sequence,
                        ExecutionFeedbackStatus.STOPPING,
                        0.0,
                        self._last_boundary,
                        stop_command_id=self.stop_command_id,
                    )
                    self._auto_phase = 1
                else:
                    stopped = MotionBoundaryState.stopped(
                        self._last_boundary.q,
                        time_seconds=self._last_boundary.time_seconds,
                    )
                    self.emit(
                        self._auto_sequence,
                        ExecutionFeedbackStatus.STOPPED,
                        0.0,
                        stopped,
                        stop_command_id=self.stop_command_id,
                    )
                    self._last_boundary = stopped
                    self.plan = None
                self._auto_sequence += 1
                advanced += 1
                continue
            if self._auto_phase == 0:
                self.emit(
                    self._auto_sequence,
                    ExecutionFeedbackStatus.RUNNING,
                    0.25,
                    self.plan.expected_start_boundary,
                )
                self._auto_phase = 1
            else:
                end = self.plan.expected_end_boundary
                self.emit(
                    self._auto_sequence,
                    ExecutionFeedbackStatus.SUCCEEDED,
                    1.0,
                    end,
                )
                self._last_boundary = end
                self.plan = None
            self._auto_sequence += 1
            advanced += 1
        return advanced

    def emit(self, sequence, status, progress, boundary=None, *, stop_command_id=None):
        assert self.plan is not None
        feedback = ExecutionFeedback(
            sequence,
            self.execution_id,
            self.plan.plan_id,
            status,
            progress,
            boundary or self.plan.expected_start_boundary,
            observed_at_monotonic_seconds=float(sequence),
            feedback_stream_id="scripted-feedback",
            producer_epoch=7,
            stop_command_id=stop_command_id,
        )
        self.feedback.append(feedback)
        return feedback


def authoritative_observation(snapshot: PlanningWorldSnapshot) -> WorldObservation:
    return WorldObservation(
        authority=ObservationAuthority.AUTHORITATIVE,
        producer_id="synthetic-perception",
        stream_id="world",
        producer_epoch=0,
        sequence=snapshot.scene_revision.sequence,
        observed_at_monotonic_seconds=float(snapshot.scene_revision.sequence + 1),
        snapshot=snapshot,
        source="online contract fixture",
    )
