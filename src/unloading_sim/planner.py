"""Joint-space RRT-Connect planner for fast geometric simulation."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import Callable, Sequence

import numpy as np


@dataclass
class PlanResult:
    success: bool
    path: list[np.ndarray]
    iterations: int
    message: str = ""
    search_evidence: dict = field(default_factory=dict)


class _Tree:
    def __init__(self, root: np.ndarray, reverse: bool = False) -> None:
        self.reverse = reverse
        self.nodes = [np.asarray(root, dtype=float).copy()]
        self.parents = [-1]

    def nearest_index(self, q: np.ndarray) -> int:
        points = np.asarray(self.nodes)
        return int(np.argmin(np.linalg.norm(points - q, axis=1)))

    def add(self, q: np.ndarray, parent: int) -> int:
        self.nodes.append(np.asarray(q, dtype=float).copy())
        self.parents.append(parent)
        return len(self.nodes) - 1

    def path_to_root(self, index: int) -> list[np.ndarray]:
        path: list[np.ndarray] = []
        while index >= 0:
            path.append(self.nodes[index])
            index = self.parents[index]
        path.reverse()
        return path


class RRTConnectPlanner:
    def __init__(
        self,
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
        is_state_valid: Callable[[np.ndarray], bool],
        step_size: float = 0.18,
        edge_resolution: float = 0.055,
        max_iterations: int = 4000,
        goal_bias: float = 0.12,
        rng: np.random.Generator | None = None,
        diagnostic_context: bool = False,
        motion_validator=None,
        request_budget=None,
        candidate_check=None,
    ) -> None:
        self.diagnostic_context = diagnostic_context
        self.sample_context = None
        self.lower = np.asarray(lower_limits, dtype=float)
        self.upper = np.asarray(upper_limits, dtype=float)
        self.is_state_valid = is_state_valid
        self.step_size = float(step_size)
        self.edge_resolution = float(edge_resolution)
        self.max_iterations = int(max_iterations)
        self.goal_bias = float(goal_bias)
        self.rng = rng or np.random.default_rng(0)
        self.motion_validator = motion_validator
        self.request_budget = request_budget
        self.candidate_check = candidate_check
        self._reset_search_evidence()

    def _reset_search_evidence(self) -> None:
        self._state_validations = 0
        self._edge_validation_calls = 0
        self._edge_state_samples = 0
        self._extension_attempts = 0
        self._candidate_failures = []
        self._restarts = 0
        self._direct_rejected = 0
        self._last_edge_result = None
        self._direct_edge_result = None
        self._last_state_result = None
        self._validation_stop_result = None
        self._rejected_candidates = set()

    def _state_valid(self, q: np.ndarray) -> bool:
        self._state_validations += 1
        if self.motion_validator is not None:
            self._last_state_result = self.motion_validator.check_states([q],self._validation_budget())[0]
            return self._last_state_result.valid
        return bool(self.is_state_valid(q))

    def _result(self, success: bool, path: list[np.ndarray], iterations: int, message: str) -> PlanResult:
        return PlanResult(success, path, iterations, message, {
            "planning_iteration_budget": self.max_iterations,
            "planning_iterations_consumed": iterations,
            "extension_attempts": self._extension_attempts,
            "state_validations": self._state_validations,
            "edge_validation_calls": self._edge_validation_calls,
            "edge_state_samples": self._edge_state_samples,
            "state_validation_budget": None,
            "edge_validation_budget": None,
            "termination": message.upper().replace(" ", "_"),
            "validation_status": "VALID" if success else "CANCELLED" if "cancel" in message else "INDETERMINATE" if message in {"maximum iterations reached", "time limit reached", "validation budget exhausted", "state evidence incomplete"} else "INVALID",
            "direct_rejected_continue_search": self._direct_rejected,
            "direct_edge_validation": None if self._direct_edge_result is None else self._direct_edge_result.evidence(),
            "candidate_failures": self._candidate_failures,
            "bounded_tree_restarts": self._restarts,
            "motion_validation": None if self.motion_validator is None else dict(self.motion_validator.statistics),
            "validation_failure": None if self._validation_stop_result is None else self._validation_stop_result.evidence(),
        })

    def _validation_budget(self, deadline=None):
        from .motion_validation import RequestBudget
        if self.request_budget is None:
            self.request_budget = RequestBudget(deadline=deadline)
        elif deadline is not None:
            old = self.request_budget.deadline
            self.request_budget.deadline = deadline if old is None else min(old, deadline)
        return self.request_budget

    def _interruption(self):
        from .motion_validation import Status
        status = self.request_budget.status() if self.request_budget is not None else None
        if status:
            return "cancelled" if status == Status.CANCELLED else "validation budget exhausted"
        for result in (self._validation_stop_result, self._last_state_result, self._last_edge_result):
            if result is not None and result.status in {Status.INDETERMINATE, Status.CANCELLED}:
                self._validation_stop_result = result
                return 'cancelled' if result.status == Status.CANCELLED else 'state evidence incomplete'
        return None

    def _accept_candidate(self, path):
        from .motion_validation import Status
        if self.motion_validator is not None:
            result = self.motion_validator.check_path(path, self._validation_budget())
            if not result.valid:
                self._candidate_failures.append(result.evidence())
                if result.status != Status.INVALID:
                    self._validation_stop_result = result
                if result.status == Status.INVALID:
                    index = result.failure.get('edge', 0)
                    self.motion_validator.feedback_failure(path[index], path[index+1], result)
                return False
        if self.candidate_check is not None:
            # Optional downstream geometric parameterization. Exact path blacklist,
            # never a blacklist of its endpoints or neighboring configuration space.
            key = tuple(np.asarray(q,float).tobytes() for q in path)
            if key in self._rejected_candidates:
                return False
            result = self.candidate_check(path, self._validation_budget())
            if not result.valid:
                self._candidate_failures.append(result.evidence())
                if result.status != Status.INVALID:
                    self._validation_stop_result = result
                failure = result.failure or {}
                if result.status == Status.INVALID and failure.get('type', 'GEOMETRY') == 'GEOMETRY':
                    index = failure.get('edge', 0)
                    if self.motion_validator is not None:
                        self.motion_validator.feedback_failure(path[index],path[index+1],result,
                            parameters=failure.get('motion_parameters'))
                    self._rejected_candidates.add(key)
                return False
        return True

    @staticmethod
    def _deadline_reached(deadline: float | None) -> bool:
        return deadline is not None and perf_counter() >= deadline

    def _edge_valid(self, a: np.ndarray, b: np.ndarray, deadline: float | None = None) -> bool:
        self._edge_validation_calls += 1
        if self.motion_validator is not None:
            self._last_edge_result = self.motion_validator.check_motion(a,b,self._validation_budget(deadline))
            self._edge_state_samples += self._last_edge_result.statistics.get('state_samples',0)
            return self._last_edge_result.valid
        delta = b - a
        n = max(1, int(np.ceil(np.max(np.abs(delta)) / self.edge_resolution)))
        for i in range(1, n + 1):
            if self._deadline_reached(deadline):
                return False
            q = a + (i / n) * delta
            self._edge_state_samples += 1
            if self.diagnostic_context:
                self.sample_context = dict(start_q_rad=a.tolist(), end_q_rad=b.tolist(), fraction=i/n)
            if not self._state_valid(q):
                return False
        return True

    def _steer(self, source: np.ndarray, target: np.ndarray) -> np.ndarray:
        delta = target - source
        distance = float(np.linalg.norm(delta))
        if distance <= self.step_size:
            return target.copy()
        return source + delta * (self.step_size / distance)

    def _extend(self, tree: _Tree, target: np.ndarray, deadline: float | None = None) -> tuple[str, int | None]:
        self._extension_attempts += 1
        if self._deadline_reached(deadline) or self._interruption():
            return "timeout", None
        nearest_idx = tree.nearest_index(target)
        nearest = tree.nodes[nearest_idx]
        new_q = self._steer(nearest, target)
        a,b = (new_q,nearest) if tree.reverse else (nearest,new_q)
        if not self._edge_valid(a, b, deadline):
            if self._deadline_reached(deadline) or self._interruption():
                return "timeout", None
            return "trapped", None
        new_idx = tree.add(new_q, nearest_idx)
        if np.linalg.norm(new_q - target) < 1e-8:
            return "reached", new_idx
        return "advanced", new_idx

    def _connect(self, tree: _Tree, target: np.ndarray, deadline: float | None = None) -> tuple[str, int | None]:
        last_idx: int | None = None
        while True:
            status, idx = self._extend(tree, target, deadline)
            if status == "timeout":
                return "timeout", last_idx
            if status == "trapped":
                return "trapped", last_idx
            last_idx = idx
            if status == "reached":
                return "reached", idx

    def plan(self, start: np.ndarray, goal: np.ndarray, time_limit_seconds: float | None = None) -> PlanResult:
        self._reset_search_evidence()
        deadline = None if time_limit_seconds is None else perf_counter() + max(0.0, float(time_limit_seconds))
        if self.motion_validator is not None:
            budget = self._validation_budget(deadline)
            deadline = budget.deadline
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if self._interruption():
            return self._result(False, [], 0, self._interruption())
        if self._deadline_reached(deadline):
            return self._result(False, [], 0, "time limit reached")
        if not self._state_valid(start):
            incomplete = self._last_state_result is not None and self._last_state_result.status.value != 'INVALID'
            return self._result(False, [], 0, self._interruption() or ('state evidence incomplete' if incomplete else "start state is invalid"))
        if not self._state_valid(goal):
            incomplete = self._last_state_result is not None and self._last_state_result.status.value != 'INVALID'
            return self._result(False, [], 0, self._interruption() or ('state evidence incomplete' if incomplete else "goal state is invalid"))
        if self._deadline_reached(deadline):
            return self._result(False, [], 0, "time limit reached")
        direct_valid = self._edge_valid(start, goal, deadline)
        self._direct_edge_result = self._last_edge_result
        if direct_valid and self._accept_candidate([start,goal]):
            return self._result(True, [start, goal], 0, "direct edge")
        self._direct_rejected += 1
        if self._interruption():
            return self._result(False, [], 0, self._interruption())
        if self._deadline_reached(deadline):
            return self._result(False, [], 0, "time limit reached")

        tree_a = _Tree(start)
        tree_b = _Tree(goal, reverse=True)
        a_is_start = True

        for iteration in range(1, self.max_iterations + 1):
            if self._interruption():
                return self._result(False, [], iteration-1, self._interruption())
            if self._deadline_reached(deadline):
                return self._result(False, [], iteration - 1, "time limit reached")
            if self.rng.random() < self.goal_bias:
                sample = tree_b.nodes[0]
            else:
                sample = self.rng.uniform(self.lower, self.upper)

            status_a, idx_a = self._extend(tree_a, sample, deadline)
            if status_a == "timeout":
                return self._result(False, [], iteration - 1, self._interruption() or "time limit reached")
            if status_a != "trapped" and idx_a is not None:
                q_new = tree_a.nodes[idx_a]
                status_b, idx_b = self._connect(tree_b, q_new, deadline)
                if status_b == "timeout":
                    return self._result(False, [], iteration, self._interruption() or "time limit reached")
                if status_b == "reached" and idx_b is not None:
                    path_a = tree_a.path_to_root(idx_a)
                    path_b = tree_b.path_to_root(idx_b)
                    if a_is_start:
                        path = path_a + list(reversed(path_b[:-1]))
                    else:
                        path = path_b + list(reversed(path_a[:-1]))
                    if self._accept_candidate(path):
                        return self._result(True, path, iteration, "connected")
                    # No dangling descendants: discard only this search tree.
                    # The validator retains exact failed motions, and the shared
                    # request deadline/check counter and iteration loop continue.
                    self._restarts += 1
                    tree_a,tree_b = _Tree(start),_Tree(goal,reverse=True)
                    a_is_start = True
                    continue

            tree_a, tree_b = tree_b, tree_a
            a_is_start = not a_is_start

        return self._result(False, [], self.max_iterations, "maximum iterations reached")

    def edge_valid(self, start: np.ndarray, goal: np.ndarray) -> bool:
        """Return whether both endpoints and their interpolated edge are valid."""
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if self.motion_validator is not None:
            return self._edge_valid(start,goal)
        return bool(
            self.is_state_valid(start)
            and self.is_state_valid(goal)
            and self._edge_valid(start, goal)
        )

    def shortcut(self, path: Sequence[np.ndarray], attempts: int = 180) -> list[np.ndarray]:
        result = [np.asarray(q, dtype=float).copy() for q in path]
        if len(result) <= 2:
            return result
        for _ in range(attempts):
            if len(result) <= 2:
                break
            i, j = sorted(self.rng.integers(0, len(result), size=2).tolist())
            if j <= i + 1:
                continue
            if self._edge_valid(result[i], result[j]):
                result = result[: i + 1] + result[j:]
        return result

    def bounded_shortcut(self, path, *, deadline, attempts=32, state_budget=1200, protected=()):
        """Simplify an already validated path; never publish a partially checked edge.

        Protected indices partition contact/permission domains. The caller must
        supply only an unloaded free segment, or explicitly protect its boundaries.
        Longest skips (including the direct edge) are attempted first, without RNG.
        """
        result = [(i, np.asarray(q, dtype=float).copy()) for i, q in enumerate(path)]
        protected = set(protected)
        checked = tried = accepted = 0
        rejected = set()
        termination = "NO_MORE_SHORTCUTS"
        while len(result) > 2:
            choices = [(j-i, i, j) for i in range(len(result)-2) for j in range(i+2, len(result))
                       if (result[i][0], result[j][0]) not in rejected
                       and not any(result[i][0] < k < result[j][0] for k in protected)]
            if not choices:
                break
            if tried == 0:
                _, i, j = max(choices, key=lambda item: (item[0], -item[1]))
            else:
                # Alternate scales after the direct probe. Spending every
                # attempt on nearly identical full-span edges starves local
                # detour removal when a central obstacle blocks all of them.
                scales = [max(2, (len(result)-1)//4), 2,
                          max(2, (len(result)-1)//2), max(2, (len(result)-1)//8)]
                desired = scales[(tried-1) % len(scales)]
                _, i, j = min(choices, key=lambda item: (abs(item[0]-desired),
                    abs(item[1]-((tried//len(scales)) % len(result))), item[1]))
            if self._deadline_reached(deadline) or tried >= attempts or checked >= state_budget:
                termination = "POSTPROCESS_BUDGET_EXHAUSTED"
                break
            tried += 1
            a, b = result[i][1], result[j][1]
            n = max(1, int(np.ceil(np.max(np.abs(b-a)) / self.edge_resolution)))
            valid = True
            if self.motion_validator is not None:
                budget = self._validation_budget(deadline)
                before = budget.checks
                # The optimization allowance is nested inside, never resets the request.
                previous_limit = budget.max_checks
                budget.max_checks = min(previous_limit if previous_limit is not None else float('inf'),
                                        budget.checks+state_budget-checked)
                try:
                    valid = self._edge_valid(a,b,deadline)
                finally:
                    budget.max_checks = previous_limit
                checked += budget.checks-before
            for k in range(0 if self.motion_validator is not None else n+1):
                if checked >= state_budget or self._deadline_reached(deadline):
                    valid = False
                    termination = "POSTPROCESS_BUDGET_EXHAUSTED"
                    break
                checked += 1
                if not self._state_valid(a + k/n*(b-a)):
                    valid = False
                    break
                if self._deadline_reached(deadline):
                    valid = False
                    termination = "POSTPROCESS_BUDGET_EXHAUSTED"
                    break
            if valid:
                result = result[:i+1] + result[j:]
                accepted += 1
            else:
                rejected.add((result[i][0], result[j][0]))
        return [q for _, q in result], dict(attempts=tried, state_checks=checked, accepted=accepted,
            attempts_budget=attempts, state_budget=state_budget, termination=termination,
            retained_source_indices=[i for i, _ in result])

    def densify(self, path: Sequence[np.ndarray], resolution: float = 0.035) -> list[np.ndarray]:
        if not path:
            return []
        dense = [np.asarray(path[0], dtype=float)]
        for a, b in zip(path[:-1], path[1:]):
            a = np.asarray(a, dtype=float)
            b = np.asarray(b, dtype=float)
            n = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
            for i in range(1, n + 1):
                dense.append(a + (i / n) * (b - a))
        return dense
