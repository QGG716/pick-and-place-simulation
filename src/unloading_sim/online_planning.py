"""Fail-closed control plane for online continuous planning.

This module deliberately contains no grasp, IK, depalletizing, conveyor, or
base-pose algorithms.  A :class:`PlannerBackend` owns those decisions and may
fail normally.  The control plane owns revision binding, FAST/WARM/COLD
escalation, rolling-horizon bookkeeping, speculative reuse, and execution
invalidation.

The asynchronous executor is intentionally cooperative: work is queued in a
stable order and is run only when ``advance`` is called.  It therefore exposes
the same transition trace as the synchronous executor without scheduler races,
while still allowing plan k+1 to be computed while plan k is marked running.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter, deque
from dataclasses import dataclass, field, replace
from enum import Enum
import hashlib
import json
from math import ceil, isfinite
from time import perf_counter
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np


class PlanStatus(str, Enum):
    SUCCESS = "SUCCESS"
    NO_IK = "NO_IK"
    GRASP_CONSTRAINT_FAILED = "GRASP_CONSTRAINT_FAILED"
    INITIAL_CLEARANCE_FAILED = "INITIAL_CLEARANCE_FAILED"
    COLLISION = "COLLISION"
    NOT_EVALUATED = "NOT_EVALUATED"


class PlanningPath(str, Enum):
    FAST = "FAST"
    WARM = "WARM"
    COLD = "COLD"


class ReplanReason(str, Enum):
    INITIAL_REQUEST = "INITIAL_REQUEST"
    ROLLING_HORIZON = "ROLLING_HORIZON"
    SPECULATIVE_MISMATCH = "SPECULATIVE_MISMATCH"
    SCENE_REVISION_CHANGED = "SCENE_REVISION_CHANGED"
    PLAN_VALIDATION_FAILED = "PLAN_VALIDATION_FAILED"
    PLAN_INVALIDATED = "PLAN_INVALIDATED"
    EXECUTION_DEVIATION = "EXECUTION_DEVIATION"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    PLANNER_FAILURE = "PLANNER_FAILURE"
    HORIZON_EXHAUSTED = "HORIZON_EXHAUSTED"


class FailureAction(str, Enum):
    ESCALATE_PATH = "ESCALATE_PATH"
    TRY_NEXT_CANDIDATE = "TRY_NEXT_CANDIDATE"
    TRY_NEXT_TARGET = "TRY_NEXT_TARGET"
    WAIT_FOR_SCENE = "WAIT_FOR_SCENE"
    ENTER_RECOVERY = "ENTER_RECOVERY"
    BLOCK = "BLOCK"


class SessionState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    WAITING_FOR_SCENE = "WAITING_FOR_SCENE"
    RECOVERY = "RECOVERY"
    BLOCKED = "BLOCKED"


class ExecutionState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


def _canonical(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _canonical(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _canonical(item) for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if hasattr(value, "center") and hasattr(value, "half_extents") and hasattr(value, "rotation"):
        return {
            "name": str(getattr(value, "name", "")),
            "category": str(getattr(value, "category", "")),
            "center": _canonical(value.center),
            "half_extents": _canonical(value.half_extents),
            "rotation": _canonical(value.rotation),
        }
    if hasattr(value, "obstacles") and hasattr(value, "cartons"):
        return {
            "obstacles": sorted((_canonical(item) for item in value.obstacles), key=lambda item: item["name"]),
            "cartons": sorted((_canonical(item) for item in value.cartons), key=lambda item: item["name"]),
            "metadata": _canonical(getattr(value, "metadata", {})),
        }
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"cannot create a deterministic scene fingerprint for {type(value).__name__}")


def scene_fingerprint(scene: Any) -> str:
    payload = json.dumps(_canonical(scene), sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


@dataclass(frozen=True, order=True)
class SceneRevision:
    """Monotonic scene version plus a content identity.

    Sequence numbers order observations; fingerprints determine whether a
    speculative observation is geometrically the one that was predicted.
    """

    sequence: int
    fingerprint: str
    source: str = field(default="perception", compare=False)
    parent_fingerprint: str | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("scene revision sequence must be non-negative")
        if not self.fingerprint:
            raise ValueError("scene revision fingerprint must be non-empty")
        if not self.source:
            raise ValueError("scene revision source must be non-empty")

    @classmethod
    def from_scene(
        cls,
        scene: Any,
        sequence: int,
        *,
        source: str = "perception",
        parent: SceneRevision | None = None,
    ) -> SceneRevision:
        return cls(
            sequence=sequence,
            fingerprint=scene_fingerprint(scene),
            source=source,
            parent_fingerprint=None if parent is None else parent.fingerprint,
        )

    def same_scene(self, other: SceneRevision) -> bool:
        return self.fingerprint == other.fingerprint


@dataclass(frozen=True)
class PlanningCandidate:
    target_id: str
    candidate_id: str
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        if not self.target_id or not self.candidate_id:
            raise ValueError("planning target and candidate ids must be non-empty")


@dataclass(frozen=True)
class PlanningRequest:
    request_id: str
    scene_revision: SceneRevision
    start_state: tuple[float, ...]
    candidates: tuple[PlanningCandidate, ...]
    seed: int = 0
    horizon_index: int = 0
    speculative: bool = False
    replan_reason: ReplanReason = ReplanReason.INITIAL_REQUEST
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        if not self.request_id:
            raise ValueError("planning request id must be non-empty")
        state = tuple(float(value) for value in self.start_state)
        if not state or not all(isfinite(value) for value in state):
            raise ValueError("planning start state must be finite and non-empty")
        candidates = tuple(self.candidates)
        if not candidates:
            raise ValueError("planning request must contain at least one candidate")
        if self.horizon_index < 0:
            raise ValueError("planning horizon index must be non-negative")
        object.__setattr__(self, "start_state", state)
        object.__setattr__(self, "candidates", candidates)


@dataclass(frozen=True)
class PlanningResult:
    status: PlanStatus
    trajectory: tuple[tuple[float, ...], ...] = ()
    target_id: str | None = None
    candidate_id: str | None = None
    message: str = ""
    latency_seconds: float | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        status = PlanStatus(self.status)
        trajectory = tuple(tuple(float(value) for value in point) for point in self.trajectory)
        if self.latency_seconds is not None and (
            not isfinite(self.latency_seconds) or self.latency_seconds < 0.0
        ):
            raise ValueError("planning latency must be finite and non-negative")
        if status is PlanStatus.SUCCESS:
            if not trajectory:
                raise ValueError("a successful planning result must contain a trajectory")
            dof = len(trajectory[0])
            if dof == 0 or any(len(point) != dof for point in trajectory):
                raise ValueError("trajectory points must have a consistent non-zero dimension")
            if not all(isfinite(value) for point in trajectory for value in point):
                raise ValueError("trajectory must contain only finite values")
            if not self.target_id or not self.candidate_id:
                raise ValueError("a successful result must identify its target and candidate")
        elif trajectory:
            raise ValueError("a failed planning result cannot carry an executable trajectory")
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "trajectory", trajectory)

    @property
    def success(self) -> bool:
        return self.status is PlanStatus.SUCCESS

    @classmethod
    def succeeded(
        cls,
        candidate: PlanningCandidate,
        trajectory: Sequence[Sequence[float]],
        *,
        latency_seconds: float | None = None,
        message: str = "",
        metadata: Mapping[str, Any] | None = None,
    ) -> PlanningResult:
        return cls(
            PlanStatus.SUCCESS,
            tuple(tuple(point) for point in trajectory),
            candidate.target_id,
            candidate.candidate_id,
            message,
            latency_seconds,
            {} if metadata is None else metadata,
        )

    @classmethod
    def failed(
        cls,
        status: PlanStatus,
        candidate: PlanningCandidate,
        *,
        latency_seconds: float | None = None,
        message: str = "",
        metadata: Mapping[str, Any] | None = None,
    ) -> PlanningResult:
        if PlanStatus(status) is PlanStatus.SUCCESS:
            raise ValueError("use PlanningResult.succeeded for success")
        return cls(
            PlanStatus(status),
            (),
            candidate.target_id,
            candidate.candidate_id,
            message,
            latency_seconds,
            {} if metadata is None else metadata,
        )


@dataclass(frozen=True)
class PlanValidation:
    valid: bool
    revision: SceneRevision
    message: str
    reused: bool = False


@dataclass(frozen=True)
class PlanEnvelope:
    plan_id: str
    request: PlanningRequest
    candidate: PlanningCandidate
    planning_path: PlanningPath
    result: PlanningResult
    planned_revision: SceneRevision
    validated_revision: SceneRevision
    speculative: bool = False
    invalidated_by: ReplanReason | None = None
    validation_message: str = "planned against exact revision"

    def __post_init__(self) -> None:
        if not self.plan_id:
            raise ValueError("plan id must be non-empty")
        if not self.result.success:
            raise ValueError("only successful results can be placed in a plan envelope")
        if (self.result.target_id, self.result.candidate_id) != (
            self.candidate.target_id,
            self.candidate.candidate_id,
        ):
            raise ValueError("plan result does not match its candidate")

    @property
    def executable(self) -> bool:
        return self.invalidated_by is None and self.result.success

    def invalidate(self, reason: ReplanReason, message: str = "") -> PlanEnvelope:
        return replace(self, invalidated_by=reason, validation_message=message or reason.value)

    def revalidated(self, validation: PlanValidation) -> PlanEnvelope:
        if not validation.valid:
            raise ValueError("cannot mark a failed validation as valid")
        return replace(
            self,
            validated_revision=validation.revision,
            invalidated_by=None,
            validation_message=validation.message,
        )


class PlannerBackend(ABC):
    """Replaceable bottom-layer planner contract with deterministic seeding."""

    def __init__(self, seed: int = 0) -> None:
        self.seed = int(seed)

    def set_seed(self, seed: int) -> None:
        self.seed = int(seed)

    @abstractmethod
    def plan(
        self,
        request: PlanningRequest,
        candidate: PlanningCandidate,
        planning_path: PlanningPath,
    ) -> PlanningResult:
        """Return SUCCESS or an explicit fail-closed status."""

    def validate_plan(
        self, envelope: PlanEnvelope, revision: SceneRevision
    ) -> PlanValidation:
        valid = envelope.planned_revision.same_scene(revision)
        return PlanValidation(
            valid,
            revision,
            "scene fingerprint matches planned revision" if valid else "scene fingerprint changed",
            reused=valid and revision != envelope.planned_revision,
        )


def normalize_plan_status(value: str | PlanStatus) -> PlanStatus:
    """Map legacy planner reasons into the control-plane status vocabulary."""

    if isinstance(value, PlanStatus):
        return value
    reason = str(value).upper()
    if reason in {"OK", "SUCCESS"}:
        return PlanStatus.SUCCESS
    if "NO_IK" in reason:
        return PlanStatus.NO_IK
    if "GRASP" in reason and ("FAIL" in reason or "CONSTRAINT" in reason):
        return PlanStatus.GRASP_CONSTRAINT_FAILED
    if "INITIAL_CLEARANCE" in reason or "INITIAL_ATTACHED_STATE" in reason:
        return PlanStatus.INITIAL_CLEARANCE_FAILED
    if "COLLISION" in reason:
        return PlanStatus.COLLISION
    return PlanStatus.NOT_EVALUATED


class FailurePolicy:
    """Deterministic retry policy; it never promotes a failure to success."""

    def decide(
        self,
        status: PlanStatus,
        *,
        has_next_path: bool,
        has_next_candidate: bool,
        has_next_target: bool,
    ) -> FailureAction:
        if has_next_path:
            return FailureAction.ESCALATE_PATH
        if has_next_candidate:
            return FailureAction.TRY_NEXT_CANDIDATE
        if has_next_target:
            return FailureAction.TRY_NEXT_TARGET
        if status is PlanStatus.NOT_EVALUATED:
            return FailureAction.WAIT_FOR_SCENE
        return FailureAction.BLOCK


@dataclass(frozen=True)
class PlanningLatencySample:
    request_id: str
    planning_path: PlanningPath
    status: PlanStatus
    seconds: float
    speculative: bool


class PlanningLatencyStatistics:
    """Planning and robot-idle metrics only; no production cycle claim."""

    def __init__(self, clock: Callable[[], float] = perf_counter) -> None:
        self._clock = clock
        self.samples: list[PlanningLatencySample] = []
        self._idle_started: float | None = None
        self.robot_idle_waiting_seconds = 0.0

    def record(self, sample: PlanningLatencySample) -> None:
        if not isfinite(sample.seconds) or sample.seconds < 0.0:
            raise ValueError("planning latency must be finite and non-negative")
        self.samples.append(sample)

    def begin_robot_idle(self) -> None:
        if self._idle_started is None:
            self._idle_started = self._clock()

    def end_robot_idle(self) -> None:
        if self._idle_started is not None:
            elapsed = self._clock() - self._idle_started
            if elapsed < 0.0:
                raise RuntimeError("statistics clock moved backwards")
            self.robot_idle_waiting_seconds += elapsed
            self._idle_started = None

    @staticmethod
    def _summary(values: Sequence[float]) -> dict[str, float | int | None]:
        if not values:
            return {"count": 0, "total_seconds": 0.0, "mean_seconds": None, "p95_seconds": None, "max_seconds": None}
        ordered = sorted(values)
        p95_index = max(0, ceil(0.95 * len(ordered)) - 1)
        return {
            "count": len(ordered),
            "total_seconds": float(sum(ordered)),
            "mean_seconds": float(sum(ordered) / len(ordered)),
            "p95_seconds": float(ordered[p95_index]),
            "max_seconds": float(ordered[-1]),
        }

    def report(self) -> dict[str, Any]:
        idle = self.robot_idle_waiting_seconds
        if self._idle_started is not None:
            idle += max(0.0, self._clock() - self._idle_started)
        by_path = {
            path.value: self._summary([sample.seconds for sample in self.samples if sample.planning_path is path])
            for path in PlanningPath
        }
        return {
            "model": "online_control_plane_latency_v1",
            "planning_latency": self._summary([sample.seconds for sample in self.samples]),
            "planning_latency_by_path": by_path,
            "planning_status_counts": dict(sorted(Counter(sample.status.value for sample in self.samples).items())),
            "speculative_planning_count": sum(sample.speculative for sample in self.samples),
            "robot_idle_waiting_for_planner_seconds": float(idle),
            "production_throughput": None,
            "timing_scope": "planning latency and robot idle waiting for planner only",
        }


@dataclass
class _ExecutorTask:
    task_id: str
    operation: Callable[[], PlanningResult]


@dataclass
class _ExecutorCompletion:
    task_id: str
    result: PlanningResult | None
    elapsed_seconds: float
    error: Exception | None = None


class PlanningExecutor(ABC):
    @abstractmethod
    def submit(self, task_id: str, operation: Callable[[], PlanningResult]) -> None: ...

    @abstractmethod
    def advance(self, max_tasks: int = 1) -> int: ...

    @abstractmethod
    def pop_completed(self) -> _ExecutorCompletion | None: ...

    @property
    @abstractmethod
    def pending_count(self) -> int: ...


class DeterministicAsyncPlanningExecutor(PlanningExecutor):
    """FIFO cooperative executor with deterministic completion ordering."""

    def __init__(self, clock: Callable[[], float] = perf_counter) -> None:
        self._clock = clock
        self._pending: deque[_ExecutorTask] = deque()
        self._completed: deque[_ExecutorCompletion] = deque()
        self._ids: set[str] = set()

    def submit(self, task_id: str, operation: Callable[[], PlanningResult]) -> None:
        if task_id in self._ids:
            raise ValueError(f"duplicate planning task id: {task_id}")
        self._ids.add(task_id)
        self._pending.append(_ExecutorTask(task_id, operation))

    def advance(self, max_tasks: int = 1) -> int:
        if max_tasks < 0:
            raise ValueError("max_tasks must be non-negative")
        completed = 0
        while self._pending and completed < max_tasks:
            task = self._pending.popleft()
            started = self._clock()
            try:
                result = task.operation()
                item = _ExecutorCompletion(task.task_id, result, self._clock() - started)
            except Exception as exc:  # Backend failure is a recovery event, not a deadlock.
                item = _ExecutorCompletion(task.task_id, None, self._clock() - started, exc)
            self._completed.append(item)
            completed += 1
        return completed

    def pop_completed(self) -> _ExecutorCompletion | None:
        if not self._completed:
            return None
        item = self._completed.popleft()
        self._ids.remove(item.task_id)
        return item

    @property
    def pending_count(self) -> int:
        return len(self._pending) + len(self._completed)


class SynchronousPlanningExecutor(DeterministicAsyncPlanningExecutor):
    """Same FIFO semantics, but submission computes the task immediately."""

    def submit(self, task_id: str, operation: Callable[[], PlanningResult]) -> None:
        super().submit(task_id, operation)
        self.advance(1)


class ExecutionMonitor:
    """Tracks the sole executable plan and blocks unvalidated continuation."""

    def __init__(self) -> None:
        self.state = ExecutionState.IDLE
        self.active_plan: PlanEnvelope | None = None
        self.progress = 0.0
        self.message = ""

    def start(self, envelope: PlanEnvelope) -> None:
        if self.state in {ExecutionState.RUNNING, ExecutionState.PAUSED}:
            raise RuntimeError("an execution is already active")
        if not envelope.executable:
            raise ValueError("cannot execute an invalidated plan")
        self.active_plan = envelope
        self.progress = 0.0
        self.message = "execution started"
        self.state = ExecutionState.RUNNING

    def update_progress(self, progress: float) -> None:
        if self.state is not ExecutionState.RUNNING:
            raise RuntimeError("execution is not running")
        if not isfinite(progress) or progress < self.progress or progress > 1.0:
            raise ValueError("execution progress must be finite, monotonic, and inside [0, 1]")
        self.progress = float(progress)

    def apply_validation(self, validation: PlanValidation, envelope: PlanEnvelope) -> None:
        if self.state is not ExecutionState.RUNNING:
            return
        if validation.valid:
            self.active_plan = envelope.revalidated(validation)
            self.message = validation.message
        else:
            self.active_plan = envelope.invalidate(ReplanReason.SCENE_REVISION_CHANGED, validation.message)
            self.state = ExecutionState.PAUSED
            self.message = validation.message

    def complete(self, success: bool, message: str = "") -> PlanEnvelope:
        if self.state is not ExecutionState.RUNNING or self.active_plan is None:
            raise RuntimeError("no running execution to complete")
        plan = self.active_plan
        self.progress = 1.0 if success else self.progress
        self.state = ExecutionState.COMPLETED if success else ExecutionState.FAILED
        self.message = message or ("execution completed" if success else "execution failed")
        self.active_plan = None
        return plan

    def fail_for_deviation(self, message: str) -> PlanEnvelope:
        if self.state is not ExecutionState.RUNNING or self.active_plan is None:
            raise RuntimeError("no running execution")
        plan = self.active_plan.invalidate(ReplanReason.EXECUTION_DEVIATION, message)
        self.active_plan = None
        self.state = ExecutionState.FAILED
        self.message = message
        return plan

    def abort_paused(self, message: str = "paused execution stopped") -> PlanEnvelope:
        if self.state is not ExecutionState.PAUSED or self.active_plan is None:
            raise RuntimeError("no paused execution to stop")
        plan = self.active_plan
        self.active_plan = None
        self.state = ExecutionState.FAILED
        self.message = message
        return plan


@dataclass
class _RequestProgress:
    request: PlanningRequest
    candidate_index: int = 0
    path_index: int = 0
    attempts: list[PlanningResult] = field(default_factory=list)


@dataclass(frozen=True)
class SessionEvent:
    sequence: int
    kind: str
    request_id: str | None
    reason: str
    details: Mapping[str, Any] = field(default_factory=dict)


class ContinuousPlanningSession:
    """Rolling-horizon state machine independent of geometric success rate."""

    def __init__(
        self,
        backend: PlannerBackend,
        *,
        executor: PlanningExecutor | None = None,
        rolling_horizon: int = 2,
        planning_paths: Sequence[PlanningPath] = (
            PlanningPath.FAST,
            PlanningPath.WARM,
            PlanningPath.COLD,
        ),
        failure_policy: FailurePolicy | None = None,
        clock: Callable[[], float] = perf_counter,
    ) -> None:
        if rolling_horizon < 1:
            raise ValueError("rolling horizon must be at least one")
        paths = tuple(PlanningPath(path) for path in planning_paths)
        if not paths or len(set(paths)) != len(paths):
            raise ValueError("planning paths must be non-empty and unique")
        self.backend = backend
        self.executor = SynchronousPlanningExecutor(clock) if executor is None else executor
        self.rolling_horizon = int(rolling_horizon)
        self.planning_paths = paths
        self.failure_policy = FailurePolicy() if failure_policy is None else failure_policy
        self.statistics = PlanningLatencyStatistics(clock)
        self.execution = ExecutionMonitor()
        self.state = SessionState.IDLE
        self.current_revision: SceneRevision | None = None
        self.current_state: tuple[float, ...] | None = None
        self.ready_plans: deque[PlanEnvelope] = deque()
        self.speculative_plans: deque[PlanEnvelope] = deque()
        self.invalidated_plans: list[PlanEnvelope] = []
        self.events: list[SessionEvent] = []
        self._requests: deque[_RequestProgress] = deque()
        self._active_progress: _RequestProgress | None = None
        self._submitted_task: str | None = None
        self._event_sequence = 0
        self._task_sequence = 0
        self._replan_sequence = 0
        self._last_exhausted_request: PlanningRequest | None = None

    def _event(self, kind: str, reason: str, request_id: str | None = None, **details: Any) -> None:
        self._event_sequence += 1
        self.events.append(SessionEvent(self._event_sequence, kind, request_id, reason, details))

    @property
    def blocked(self) -> bool:
        return self.state is SessionState.BLOCKED

    @property
    def robot_idle_waiting_for_planner(self) -> bool:
        return self.state is SessionState.PLANNING and self.execution.state is not ExecutionState.RUNNING

    def submit(self, request: PlanningRequest) -> None:
        occupied = len(self.ready_plans) + len(self.speculative_plans) + len(self._requests)
        occupied += int(self._active_progress is not None)
        if occupied >= self.rolling_horizon:
            raise RuntimeError("rolling horizon is full")
        if self.current_revision is None:
            self.current_revision = request.scene_revision
            self.current_state = request.start_state
        elif not request.speculative and not request.scene_revision.same_scene(self.current_revision):
            raise ValueError("non-speculative request is not bound to the current scene")
        self._requests.append(_RequestProgress(request))
        self._event("request_submitted", request.replan_reason.value, request.request_id, speculative=request.speculative)
        if self.execution.state is not ExecutionState.RUNNING:
            self.state = SessionState.PLANNING
            self.statistics.begin_robot_idle()
        self._schedule_if_possible()

    def submit_speculative(self, request: PlanningRequest) -> None:
        if self.execution.state is not ExecutionState.RUNNING:
            raise RuntimeError("speculative next planning requires plan k to be executing")
        if not request.speculative:
            request = replace(request, speculative=True, replan_reason=ReplanReason.ROLLING_HORIZON)
        self.submit(request)

    def _schedule_if_possible(self) -> None:
        if self._submitted_task is not None:
            return
        if self._active_progress is None:
            if not self._requests:
                return
            self._active_progress = self._requests.popleft()
        progress = self._active_progress
        candidate = progress.request.candidates[progress.candidate_index]
        path = self.planning_paths[progress.path_index]
        self._task_sequence += 1
        task_id = f"{progress.request.request_id}:{candidate.candidate_id}:{path.value}:{self._task_sequence}"
        self._submitted_task = task_id
        request = progress.request

        def operation() -> PlanningResult:
            self.backend.set_seed(request.seed)
            return self.backend.plan(request, candidate, path)

        self.executor.submit(task_id, operation)
        self._event("planning_started", path.value, request.request_id, target=candidate.target_id, candidate=candidate.candidate_id)

    def advance(self, max_tasks: int = 1) -> SessionState:
        if max_tasks < 0:
            raise ValueError("max_tasks must be non-negative")
        remaining = max_tasks
        while remaining > 0:
            self._schedule_if_possible()
            if self._submitted_task is None:
                break
            self.executor.advance(1)
            completion = self.executor.pop_completed()
            if completion is None:
                break
            remaining -= 1
            self._consume(completion)
        self._refresh_state()
        return self.state

    def run_until_stable(self, max_tasks: int = 1000) -> SessionState:
        for _ in range(max_tasks):
            before = (self._submitted_task, len(self._requests), self.state, len(self.ready_plans), len(self.speculative_plans))
            self.advance(1)
            after = (self._submitted_task, len(self._requests), self.state, len(self.ready_plans), len(self.speculative_plans))
            if self._submitted_task is None and not self._requests and self._active_progress is None:
                return self.state
            if before == after and self.executor.pending_count == 0:
                return self.state
        raise RuntimeError("planning session did not stabilize within max_tasks")

    def _consume(self, completion: _ExecutorCompletion) -> None:
        if completion.task_id != self._submitted_task or self._active_progress is None:
            raise RuntimeError("executor returned an out-of-order completion")
        self._submitted_task = None
        progress = self._active_progress
        request = progress.request
        candidate = request.candidates[progress.candidate_index]
        path = self.planning_paths[progress.path_index]
        if completion.error is not None:
            self._event("backend_error", ReplanReason.PLANNER_FAILURE.value, request.request_id, error=repr(completion.error))
            self._active_progress = None
            self.state = SessionState.RECOVERY
            self.statistics.end_robot_idle()
            return
        assert completion.result is not None
        result = completion.result
        if (result.target_id is not None and result.target_id != candidate.target_id) or (
            result.candidate_id is not None and result.candidate_id != candidate.candidate_id
        ):
            self._event("backend_contract_error", ReplanReason.PLANNER_FAILURE.value, request.request_id)
            self._active_progress = None
            self.state = SessionState.RECOVERY
            self.statistics.end_robot_idle()
            return
        latency = completion.elapsed_seconds if result.latency_seconds is None else result.latency_seconds
        result = replace(result, latency_seconds=latency)
        progress.attempts.append(result)
        self.statistics.record(PlanningLatencySample(request.request_id, path, result.status, latency, request.speculative))
        self._event("planning_finished", result.status.value, request.request_id, path=path.value, latency_seconds=latency)
        if result.success:
            envelope = self._envelope(request, candidate, path, result)
            (self.speculative_plans if request.speculative else self.ready_plans).append(envelope)
            self._active_progress = None
            self._event("plan_ready", path.value, request.request_id, plan_id=envelope.plan_id, speculative=request.speculative)
            self._schedule_if_possible()
            return

        next_path = progress.path_index + 1 < len(self.planning_paths)
        next_candidate_index = progress.candidate_index + 1
        has_next_candidate = False
        has_next_target = False
        if next_candidate_index < len(request.candidates):
            next_candidate = request.candidates[next_candidate_index]
            has_next_candidate = next_candidate.target_id == candidate.target_id
            has_next_target = next_candidate.target_id != candidate.target_id
        action = self.failure_policy.decide(
            result.status,
            has_next_path=next_path,
            has_next_candidate=has_next_candidate,
            has_next_target=has_next_target,
        )
        self._event("failure_decision", action.value, request.request_id, status=result.status.value)
        if action is FailureAction.ESCALATE_PATH:
            progress.path_index += 1
        elif action in {FailureAction.TRY_NEXT_CANDIDATE, FailureAction.TRY_NEXT_TARGET}:
            progress.candidate_index += 1
            progress.path_index = 0
        else:
            self._active_progress = None
            self._last_exhausted_request = request
            all_not_evaluated = all(
                attempt.status is PlanStatus.NOT_EVALUATED for attempt in progress.attempts
            )
            if action is FailureAction.WAIT_FOR_SCENE and all_not_evaluated:
                self.state = SessionState.WAITING_FOR_SCENE
            elif action is FailureAction.ENTER_RECOVERY:
                self.state = SessionState.RECOVERY
            else:
                self.state = SessionState.BLOCKED
            self.statistics.end_robot_idle()
            self._event("request_exhausted", self.state.value, request.request_id, attempts=len(progress.attempts))
        self._schedule_if_possible()

    def _envelope(
        self,
        request: PlanningRequest,
        candidate: PlanningCandidate,
        path: PlanningPath,
        result: PlanningResult,
    ) -> PlanEnvelope:
        identity = {
            "request": request.request_id,
            "revision": request.scene_revision.fingerprint,
            "target": candidate.target_id,
            "candidate": candidate.candidate_id,
            "path": path.value,
            "trajectory": result.trajectory,
        }
        plan_id = hashlib.sha256(
            json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()[:20]
        return PlanEnvelope(
            plan_id,
            request,
            candidate,
            path,
            result,
            request.scene_revision,
            request.scene_revision,
            request.speculative,
        )

    def _refresh_state(self) -> None:
        if self.state in {SessionState.RECOVERY, SessionState.BLOCKED, SessionState.WAITING_FOR_SCENE}:
            return
        if self.execution.state is ExecutionState.RUNNING:
            self.state = SessionState.EXECUTING
            self.statistics.end_robot_idle()
        elif self.ready_plans:
            self.state = SessionState.READY
            self.statistics.end_robot_idle()
        elif self._submitted_task is not None or self._active_progress is not None or self._requests:
            self.state = SessionState.PLANNING
            self.statistics.begin_robot_idle()
        else:
            self.state = SessionState.IDLE
            self.statistics.end_robot_idle()

    def _validate(self, envelope: PlanEnvelope, revision: SceneRevision) -> tuple[PlanEnvelope, PlanValidation]:
        validation = self.backend.validate_plan(envelope, revision)
        return (envelope.revalidated(validation) if validation.valid else envelope, validation)

    def start_execution(self) -> PlanEnvelope | None:
        if self.execution.state in {ExecutionState.RUNNING, ExecutionState.PAUSED}:
            raise RuntimeError("an execution is already active")
        if not self.ready_plans or self.current_revision is None:
            self._refresh_state()
            return None
        envelope = self.ready_plans.popleft()
        envelope, validation = self._validate(envelope, self.current_revision)
        if not validation.valid:
            invalid = envelope.invalidate(ReplanReason.PLAN_VALIDATION_FAILED, validation.message)
            self.invalidated_plans.append(invalid)
            self._event("plan_invalidated", ReplanReason.PLAN_VALIDATION_FAILED.value, envelope.request.request_id)
            self._queue_replan(envelope.request, self.current_revision, ReplanReason.PLAN_VALIDATION_FAILED)
            self._refresh_state()
            return None
        self.execution.start(envelope)
        self.statistics.end_robot_idle()
        self.state = SessionState.EXECUTING
        self._event("execution_started", "validated", envelope.request.request_id, plan_id=envelope.plan_id)
        return envelope

    def update_scene(self, revision: SceneRevision) -> None:
        previous = self.current_revision
        if previous is not None:
            if revision.sequence < previous.sequence:
                raise ValueError("scene revision cannot move backwards")
            if revision.sequence == previous.sequence and not revision.same_scene(previous):
                raise ValueError("one scene sequence cannot identify two different scenes")
        self.current_revision = revision
        if self.state in {SessionState.WAITING_FOR_SCENE, SessionState.BLOCKED}:
            self.state = SessionState.IDLE
            if self._last_exhausted_request is not None and (
                previous is None or not revision.same_scene(previous)
            ):
                request = self._last_exhausted_request
                self._last_exhausted_request = None
                self._queue_replan(request, revision, ReplanReason.SCENE_REVISION_CHANGED)
        self._event("scene_updated", ReplanReason.SCENE_REVISION_CHANGED.value, None, sequence=revision.sequence)
        if self.execution.state is ExecutionState.RUNNING and self.execution.active_plan is not None:
            active = self.execution.active_plan
            validated, result = self._validate(active, revision)
            self.execution.apply_validation(result, validated)
            if not result.valid:
                self.invalidated_plans.append(active.invalidate(ReplanReason.SCENE_REVISION_CHANGED, result.message))
                self._queue_replan(active.request, revision, ReplanReason.SCENE_REVISION_CHANGED)
                self.state = SessionState.PLANNING
                self.statistics.begin_robot_idle()
                return
        if self.execution.state is not ExecutionState.RUNNING and self.speculative_plans:
            self._reconcile_speculative()
        retained: deque[PlanEnvelope] = deque()
        while self.ready_plans:
            plan = self.ready_plans.popleft()
            validated, result = self._validate(plan, revision)
            if result.valid:
                retained.append(validated)
            else:
                self.invalidated_plans.append(plan.invalidate(ReplanReason.SCENE_REVISION_CHANGED, result.message))
                self._queue_replan(plan.request, revision, ReplanReason.SCENE_REVISION_CHANGED)
        self.ready_plans = retained
        self._refresh_state()

    def complete_execution(
        self,
        *,
        success: bool,
        end_state: Sequence[float] | None = None,
        scene_revision: SceneRevision | None = None,
        message: str = "",
    ) -> PlanEnvelope:
        completed = self.execution.complete(success, message)
        self._event(
            "execution_completed" if success else "execution_failed",
            "SUCCESS" if success else ReplanReason.EXECUTION_FAILED.value,
            completed.request.request_id,
        )
        if end_state is not None:
            state = tuple(float(value) for value in end_state)
            if not state or not all(isfinite(value) for value in state):
                raise ValueError("execution end state must be finite and non-empty")
            self.current_state = state
        if not success:
            while self.speculative_plans:
                plan = self.speculative_plans.popleft().invalidate(ReplanReason.EXECUTION_FAILED)
                self.invalidated_plans.append(plan)
            self.state = SessionState.RECOVERY
            return completed
        if scene_revision is not None:
            if self.current_revision is not None and scene_revision.sequence < self.current_revision.sequence:
                raise ValueError("scene revision cannot move backwards")
            self.current_revision = scene_revision
            self._reconcile_speculative()
        elif self.speculative_plans:
            self.state = SessionState.WAITING_FOR_SCENE
            return completed
        self._refresh_state()
        if not self.ready_plans and self.current_revision is None:
            self.state = SessionState.WAITING_FOR_SCENE
        return completed

    def stop_invalidated_execution(self, message: str = "scene-invalidated execution stopped") -> PlanEnvelope:
        """Acknowledge that the robot stopped after a revision invalidation."""

        plan = self.execution.abort_paused(message)
        self.state = SessionState.PLANNING if (
            self._submitted_task is not None or self._active_progress is not None or self._requests
        ) else SessionState.RECOVERY
        self._event("invalidated_execution_stopped", ReplanReason.PLAN_INVALIDATED.value, plan.request.request_id)
        self._refresh_state()
        return plan

    def _reconcile_speculative(self) -> None:
        if not self.speculative_plans:
            return
        if self.current_revision is None:
            self.state = SessionState.WAITING_FOR_SCENE
            return
        while self.speculative_plans:
            plan = self.speculative_plans.popleft()
            validated, result = self._validate(plan, self.current_revision)
            if result.valid:
                self.ready_plans.append(replace(validated, speculative=False))
                self._event("speculative_reused", "scene_match", plan.request.request_id, plan_id=plan.plan_id)
            else:
                invalid = plan.invalidate(ReplanReason.SPECULATIVE_MISMATCH, result.message)
                self.invalidated_plans.append(invalid)
                self._event("plan_invalidated", ReplanReason.SPECULATIVE_MISMATCH.value, plan.request.request_id)
                self._queue_replan(plan.request, self.current_revision, ReplanReason.SPECULATIVE_MISMATCH)

    def notify_execution_deviation(self, message: str) -> PlanEnvelope:
        plan = self.execution.fail_for_deviation(message)
        self.invalidated_plans.append(plan)
        if self.current_revision is not None:
            self._queue_replan(plan.request, self.current_revision, ReplanReason.EXECUTION_DEVIATION)
        while self.speculative_plans:
            self.invalidated_plans.append(
                self.speculative_plans.popleft().invalidate(ReplanReason.EXECUTION_DEVIATION)
            )
        self.state = SessionState.RECOVERY
        self._event("execution_deviation", ReplanReason.EXECUTION_DEVIATION.value, plan.request.request_id)
        return plan

    def _queue_replan(
        self,
        request: PlanningRequest,
        revision: SceneRevision,
        reason: ReplanReason,
    ) -> None:
        self._replan_sequence += 1
        start = self.current_state or request.start_state
        replanned = replace(
            request,
            request_id=f"{request.request_id}:replan:{self._replan_sequence}",
            scene_revision=revision,
            start_state=start,
            speculative=False,
            replan_reason=reason,
        )
        self._requests.append(_RequestProgress(replanned))
        self._event("replan_queued", reason.value, replanned.request_id)
        self._schedule_if_possible()

    def resume_after_recovery(self) -> SessionState:
        if self.state is not SessionState.RECOVERY:
            raise RuntimeError("session is not in recovery")
        self.state = SessionState.PLANNING if (
            self._submitted_task is not None or self._active_progress is not None or self._requests
        ) else SessionState.IDLE
        if self.state is SessionState.PLANNING:
            self.statistics.begin_robot_idle()
            self._schedule_if_possible()
        return self.state

    def snapshot(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "scene_revision": None if self.current_revision is None else {
                "sequence": self.current_revision.sequence,
                "fingerprint": self.current_revision.fingerprint,
            },
            "execution_state": self.execution.state.value,
            "ready_plan_ids": [plan.plan_id for plan in self.ready_plans],
            "speculative_plan_ids": [plan.plan_id for plan in self.speculative_plans],
            "invalidated_plan_ids": [plan.plan_id for plan in self.invalidated_plans],
            "pending_requests": len(self._requests) + int(self._active_progress is not None),
            "blocked": self.blocked,
            "metrics": self.statistics.report(),
        }


__all__ = [
    "ContinuousPlanningSession",
    "DeterministicAsyncPlanningExecutor",
    "ExecutionMonitor",
    "ExecutionState",
    "FailureAction",
    "FailurePolicy",
    "PlanEnvelope",
    "PlanningCandidate",
    "PlanningExecutor",
    "PlanningLatencySample",
    "PlanningLatencyStatistics",
    "PlanningPath",
    "PlanningRequest",
    "PlanningResult",
    "PlannerBackend",
    "PlanStatus",
    "PlanValidation",
    "ReplanReason",
    "SceneRevision",
    "SessionEvent",
    "SessionState",
    "SynchronousPlanningExecutor",
    "normalize_plan_status",
    "scene_fingerprint",
]
