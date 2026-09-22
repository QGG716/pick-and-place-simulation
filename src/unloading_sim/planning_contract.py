"""Small backend-neutral contract for a frozen, joint-space free motion."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
from typing import Any, Callable, Mapping

import numpy as np


def fingerprint(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


class PlanningStatus(str, Enum):
    VERIFIED = "VERIFIED"
    INVALID_START = "INVALID_START"
    INVALID_GOAL = "INVALID_GOAL"
    BUDGET_EXHAUSTED = "BUDGET_EXHAUSTED"
    CANCELLED = "CANCELLED"
    STALE_SCENE = "STALE_SCENE"
    UNSUPPORTED_CONSTRAINT = "UNSUPPORTED_CONSTRAINT"
    BACKEND_UNAVAILABLE = "BACKEND_UNAVAILABLE"
    AUTHORITY_REJECTED = "AUTHORITY_REJECTED"
    INTERNAL_ERROR = "INTERNAL_ERROR"


@dataclass(frozen=True)
class PlanningBudget:
    # Native state evaluations and legacy iterations are different work units.
    max_state_checks: int = 100000
    legacy_iterations: int = 600
    max_attempts: int = 2
    wall_time_s: float | None = None

    def __post_init__(self):
        if any(type(v) is not int or v < 1 for v in (
                self.max_state_checks, self.legacy_iterations, self.max_attempts)):
            raise ValueError("resource budgets must be positive integers")
        if self.max_attempts > 8:
            raise ValueError("at most eight repair attempts")
        if self.wall_time_s is not None and (
                not np.isfinite(self.wall_time_s) or self.wall_time_s <= 0):
            raise ValueError("explicit wall budget must be positive and finite")


@dataclass(frozen=True)
class FreeMotionRequest:
    request_id: str
    scene_revision: str
    scene_fingerprint: str
    model_fingerprint: str
    tool_fingerprint: str
    policy_fingerprint: str
    stage: str
    joint_names: tuple[str, ...]
    q_start: tuple[float, ...]
    q_goal: tuple[float, ...]
    constraints: Mapping[str, Any]
    frames: Mapping[str, Any]
    attachment: Mapping[str, Any] | None
    seed: int
    budget: PlanningBudget = field(default_factory=PlanningBudget)
    cancelled: Callable[[], bool] = field(default=lambda: False, repr=False, compare=False)
    current_revision: Callable[[], str] | None = field(default=None, repr=False, compare=False)


@dataclass
class FreeMotionResult:
    status: PlanningStatus
    backend: str
    candidate_found: bool = False
    exact_solution: bool = False
    native_validated: bool = False
    authority_validated: bool = False
    path: list[list[float]] = field(default_factory=list)
    diagnostics: dict[str, Any] = field(default_factory=dict)
    timings: dict[str, float | None] = field(default_factory=dict)
    counters: dict[str, int | None] = field(default_factory=dict)

    @property
    def deliverable(self):
        return (self.status == PlanningStatus.VERIFIED and self.exact_solution
                and self.authority_validated and bool(self.path))

    def to_mapping(self):
        return {"status": self.status.value, "backend": self.backend,
                "candidate_found": self.candidate_found, "exact_solution": self.exact_solution,
                "native_validated": self.native_validated, "authority_validated": self.authority_validated,
                "path": self.path, "diagnostics": self.diagnostics,
                "timings": self.timings, "counters": self.counters,
                "deliverable_stage_path": self.deliverable, "complete_task_executable": False}


def validate_request(request: FreeMotionRequest, limits) -> PlanningStatus | None:
    if request.cancelled():
        return PlanningStatus.CANCELLED
    if request.current_revision is not None and request.current_revision() != request.scene_revision:
        return PlanningStatus.STALE_SCENE
    if request.constraints.get("motion") != "free_joint_space" or request.constraints.get("events", []):
        return PlanningStatus.UNSUPPORTED_CONSTRAINT
    if (request.stage not in {"pregrasp", "transit"} or type(request.seed) is not int
            or not 0 <= request.seed <= 0xffffffff):
        return PlanningStatus.UNSUPPORTED_CONSTRAINT
    supported = {"motion", "events", "joint_margin", "maximum_jacobian_condition",
                 "radial_limit", "edge_resolution_rad", "point_motion_bound_m", "lever_arm_m"}
    if set(request.constraints) - supported:
        return PlanningStatus.UNSUPPORTED_CONSTRAINT
    if len(set(request.joint_names)) != len(request.joint_names):
        return PlanningStatus.UNSUPPORTED_CONSTRAINT
    bounds = np.asarray(limits, float)
    for q, status in ((request.q_start, PlanningStatus.INVALID_START),
                      (request.q_goal, PlanningStatus.INVALID_GOAL)):
        a = np.asarray(q, float)
        if (a.shape != (len(request.joint_names),) or not np.isfinite(a).all()
                or bounds.shape != (len(a), 2) or not np.isfinite(bounds).all()
                or np.any(a < bounds[:, 0]) or np.any(a > bounds[:, 1])):
            return status
    return None
