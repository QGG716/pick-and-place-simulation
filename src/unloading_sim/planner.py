"""Joint-space RRT-Connect planner for fast geometric simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np


@dataclass
class PlanResult:
    success: bool
    path: list[np.ndarray]
    iterations: int
    message: str = ""


class _Tree:
    def __init__(self, root: np.ndarray) -> None:
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
    ) -> None:
        self.lower = np.asarray(lower_limits, dtype=float)
        self.upper = np.asarray(upper_limits, dtype=float)
        self.is_state_valid = is_state_valid
        self.step_size = float(step_size)
        self.edge_resolution = float(edge_resolution)
        self.max_iterations = int(max_iterations)
        self.goal_bias = float(goal_bias)
        self.rng = rng or np.random.default_rng(0)

    def _edge_valid(self, a: np.ndarray, b: np.ndarray) -> bool:
        delta = b - a
        n = max(1, int(np.ceil(np.max(np.abs(delta)) / self.edge_resolution)))
        for i in range(1, n + 1):
            q = a + (i / n) * delta
            if not self.is_state_valid(q):
                return False
        return True

    def _steer(self, source: np.ndarray, target: np.ndarray) -> np.ndarray:
        delta = target - source
        distance = float(np.linalg.norm(delta))
        if distance <= self.step_size:
            return target.copy()
        return source + delta * (self.step_size / distance)

    def _extend(self, tree: _Tree, target: np.ndarray) -> tuple[str, int | None]:
        nearest_idx = tree.nearest_index(target)
        nearest = tree.nodes[nearest_idx]
        new_q = self._steer(nearest, target)
        if not self._edge_valid(nearest, new_q):
            return "trapped", None
        new_idx = tree.add(new_q, nearest_idx)
        if np.linalg.norm(new_q - target) < 1e-8:
            return "reached", new_idx
        return "advanced", new_idx

    def _connect(self, tree: _Tree, target: np.ndarray) -> tuple[str, int | None]:
        last_idx: int | None = None
        while True:
            status, idx = self._extend(tree, target)
            if status == "trapped":
                return "trapped", last_idx
            last_idx = idx
            if status == "reached":
                return "reached", idx

    def plan(self, start: np.ndarray, goal: np.ndarray) -> PlanResult:
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if not self.is_state_valid(start):
            return PlanResult(False, [], 0, "start state is invalid")
        if not self.is_state_valid(goal):
            return PlanResult(False, [], 0, "goal state is invalid")
        if self._edge_valid(start, goal):
            return PlanResult(True, [start, goal], 0, "direct edge")

        tree_a = _Tree(start)
        tree_b = _Tree(goal)
        a_is_start = True

        for iteration in range(1, self.max_iterations + 1):
            if self.rng.random() < self.goal_bias:
                sample = tree_b.nodes[0]
            else:
                sample = self.rng.uniform(self.lower, self.upper)

            status_a, idx_a = self._extend(tree_a, sample)
            if status_a != "trapped" and idx_a is not None:
                q_new = tree_a.nodes[idx_a]
                status_b, idx_b = self._connect(tree_b, q_new)
                if status_b == "reached" and idx_b is not None:
                    path_a = tree_a.path_to_root(idx_a)
                    path_b = tree_b.path_to_root(idx_b)
                    if a_is_start:
                        path = path_a + list(reversed(path_b[:-1]))
                    else:
                        path = path_b + list(reversed(path_a[:-1]))
                    return PlanResult(True, path, iteration, "connected")

            tree_a, tree_b = tree_b, tree_a
            a_is_start = not a_is_start

        return PlanResult(False, [], self.max_iterations, "maximum iterations reached")

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
