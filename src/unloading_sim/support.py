"""Carton support, face-occlusion, and deterministic removal ordering."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from .geometry import OBB


def _bounds(box: OBB) -> tuple[np.ndarray, np.ndarray]:
    corners = box.corners()
    return np.min(corners, axis=0), np.max(corners, axis=0)


def _overlap(lo_a: float, hi_a: float, lo_b: float, hi_b: float) -> float:
    return max(0.0, min(hi_a, hi_b) - max(lo_a, lo_b))


@dataclass(frozen=True)
class SupportEdge:
    supporter: str
    supported: str
    vertical_gap_m: float
    footprint_overlap_ratio: float


class SupportRelationGraph:
    """Directed graph whose edges point from a carton to boxes it supports."""

    def __init__(self, cartons: Sequence[OBB], edges: Sequence[SupportEdge]) -> None:
        names = [carton.name for carton in cartons]
        if len(names) != len(set(names)):
            raise ValueError("carton names must be unique")
        self.cartons = {carton.name: carton for carton in cartons}
        self.edges = list(edges)
        self.supports = {name: set() for name in names}
        self.supported_by = {name: set() for name in names}
        for edge in edges:
            if edge.supporter not in self.cartons or edge.supported not in self.cartons:
                raise ValueError("support edge references an unknown carton")
            self.supports[edge.supporter].add(edge.supported)
            self.supported_by[edge.supported].add(edge.supporter)

    @classmethod
    def build(
        cls,
        cartons: Sequence[OBB],
        *,
        contact_tolerance_m: float = 0.008,
        minimum_overlap_ratio: float = 0.08,
    ) -> "SupportRelationGraph":
        if contact_tolerance_m < 0.0 or not 0.0 < minimum_overlap_ratio <= 1.0:
            raise ValueError("invalid support graph tolerances")
        bounds = {carton.name: _bounds(carton) for carton in cartons}
        edges: list[SupportEdge] = []
        for lower in cartons:
            lower_min, lower_max = bounds[lower.name]
            lower_area = max(1e-12, (lower_max[0] - lower_min[0]) * (lower_max[1] - lower_min[1]))
            for upper in cartons:
                if lower.name == upper.name:
                    continue
                upper_min, upper_max = bounds[upper.name]
                gap = float(upper_min[2] - lower_max[2])
                if abs(gap) > contact_tolerance_m:
                    continue
                overlap_area = _overlap(lower_min[0], lower_max[0], upper_min[0], upper_max[0]) * _overlap(
                    lower_min[1], lower_max[1], upper_min[1], upper_max[1]
                )
                upper_area = max(1e-12, (upper_max[0] - upper_min[0]) * (upper_max[1] - upper_min[1]))
                ratio = overlap_area / min(lower_area, upper_area)
                if ratio >= minimum_overlap_ratio:
                    edges.append(SupportEdge(lower.name, upper.name, gap, float(ratio)))
        return cls(cartons, edges)

    def face_blockers(self, name: str, face: str, remaining: Iterable[str] | None = None) -> list[str]:
        """Find boxes between a requested face and its open approach direction."""
        if name not in self.cartons:
            raise KeyError(name)
        active = set(self.cartons) if remaining is None else set(remaining)
        active.discard(name)
        target_min, target_max = _bounds(self.cartons[name])
        blockers: list[str] = []
        for other_name in sorted(active):
            other_min, other_max = _bounds(self.cartons[other_name])
            if face == "top":
                blocked = (
                    other_min[2] >= target_max[2] - 1e-6
                    and _overlap(target_min[0], target_max[0], other_min[0], other_max[0]) > 1e-6
                    and _overlap(target_min[1], target_max[1], other_min[1], other_max[1]) > 1e-6
                )
            elif face == "front":
                blocked = (
                    other_max[0] <= target_min[0] + 1e-6
                    and _overlap(target_min[1], target_max[1], other_min[1], other_max[1]) > 1e-6
                    and _overlap(target_min[2], target_max[2], other_min[2], other_max[2]) > 1e-6
                )
            elif face == "left":
                blocked = (
                    other_min[1] >= target_max[1] - 1e-6
                    and _overlap(target_min[0], target_max[0], other_min[0], other_max[0]) > 1e-6
                    and _overlap(target_min[2], target_max[2], other_min[2], other_max[2]) > 1e-6
                )
            elif face == "right":
                blocked = (
                    other_max[1] <= target_min[1] + 1e-6
                    and _overlap(target_min[0], target_max[0], other_min[0], other_max[0]) > 1e-6
                    and _overlap(target_min[2], target_max[2], other_min[2], other_max[2]) > 1e-6
                )
            else:
                raise ValueError("face must be top, front, left, or right")
            if blocked:
                blockers.append(other_name)
        return blockers

    def removable_cartons(
        self,
        remaining: Iterable[str] | None = None,
        face_modes: Sequence[str] = ("front", "side", "top"),
    ) -> list[str]:
        active = set(self.cartons) if remaining is None else set(remaining)
        unknown = active.difference(self.cartons)
        if unknown:
            raise KeyError(f"unknown cartons: {sorted(unknown)}")
        removable: list[str] = []
        for name in active:
            if self.supports[name].intersection(active):
                continue
            exposed = (
                ("top" in face_modes and not self.face_blockers(name, "top", active))
                or ("front" in face_modes and not self.face_blockers(name, "front", active))
                or (
                    "side" in face_modes
                    and (
                        not self.face_blockers(name, "left", active)
                        or not self.face_blockers(name, "right", active)
                    )
                )
            )
            if exposed:
                removable.append(name)
        return sorted(
            removable,
            key=lambda item: (
                -_bounds(self.cartons[item])[1][2],
                self.cartons[item].center[0],
                abs(self.cartons[item].center[1]),
                item,
            ),
        )

    def removal_order(self, face_modes: Sequence[str] = ("front", "side", "top")) -> list[str]:
        remaining = set(self.cartons)
        order: list[str] = []
        while remaining:
            candidates = self.removable_cartons(remaining, face_modes)
            if not candidates:
                raise RuntimeError(f"support/occlusion graph has no removable carton: {sorted(remaining)}")
            selected = candidates[0]
            order.append(selected)
            remaining.remove(selected)
        return order

    def audit(self) -> dict:
        return {
            "model": "conservative_world_projection_support_graph_v1",
            "cartons": len(self.cartons),
            "support_edges": [edge.__dict__ for edge in self.edges],
        }
