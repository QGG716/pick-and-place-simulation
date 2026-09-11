"""Stateful, deterministic row-first heuristics over actual remaining cartons.

This module chooses what to try, never declares a motion feasible.  The caller
supplies the current rigid-body OBBs after every task; placed cartons belong in
receiver occupancy rather than in the remaining stack passed here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
from typing import Any, Iterable, Mapping, Sequence, TypeVar

import numpy as np

from .geometry import OBB, rotation_vector_from_matrix
from .support import SupportRelationGraph

_UNCHANGED_CONTEXT = object()

@dataclass(frozen=True)
class RowSequencePolicy:
    row_height_fraction: float = 0.05
    minimum_row_tolerance_m: float = 0.003
    state_translation_tolerance_m: float = 0.002
    state_rotation_tolerance_rad: float = 0.005
    support_contact_tolerance_m: float = 0.008
    center_tie_tolerance_m: float = 0.002
    actual_cost_weight: float = 0.15

    def __post_init__(self) -> None:
        if not 0 < self.row_height_fraction < 0.5:
            raise ValueError("row height fraction must be between zero and one half")
        for name in ("minimum_row_tolerance_m", "state_translation_tolerance_m",
                     "state_rotation_tolerance_rad", "support_contact_tolerance_m",
                     "center_tie_tolerance_m"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 <= self.actual_cost_weight <= 0.25:
            raise ValueError("actual cost weight must be normalized and at most 0.25")


@dataclass(frozen=True)
class RankedCarton:
    carton: OBB
    row_id: str
    row_center_y_m: float
    center_distance_m: float
    normalized_center_cost: float
    normalized_actual_cost: float
    score: float
    available_faces: tuple[str, ...]
    support_names: tuple[str, ...]
    reasons: tuple[str, ...]

    @property
    def name(self) -> str:
        return self.carton.name

    def as_dict(self) -> dict[str, Any]:
        return {"carton_name": self.name, "row_id": self.row_id,
                "row_center_y_m": self.row_center_y_m,
                "center_distance_m": self.center_distance_m,
                "normalized_center_cost": self.normalized_center_cost,
                "normalized_actual_cost": self.normalized_actual_cost,
                "score": self.score, "available_faces": list(self.available_faces),
                "support_names": list(self.support_names), "reasons": list(self.reasons)}


@dataclass(frozen=True)
class RowSelection:
    status: str
    row_id: str | None
    row_center_y_m: float | None
    candidates: tuple[RankedCarton, ...]
    scene_fingerprint: str
    row_remaining_names: tuple[str, ...]
    remaining_count: int
    blocked_reasons: Mapping[str, str] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {"status": self.status, "row_id": self.row_id,
                "row_center_y_m": self.row_center_y_m,
                "candidates": [item.as_dict() for item in self.candidates],
                "scene_fingerprint": self.scene_fingerprint,
                "row_remaining_names": list(self.row_remaining_names),
                "remaining_count": self.remaining_count,
                "blocked_reasons": dict(self.blocked_reasons)}


def _bounds(box: OBB) -> tuple[np.ndarray, np.ndarray]:
    points = box.corners()
    return points.min(axis=0), points.max(axis=0)


def actual_tcp_approach_costs(
    cartons: Sequence[OBB], current_tcp_world: np.ndarray,
    exposed_faces: Mapping[str, Iterable[str]], *, pregrasp_standoff_m: float,
    virtual_contact_offset_m: float = 0.0,
) -> dict[str, Any]:
    """Cheap translation lower bounds, never IK or collision feasibility.

    Project the current TCP onto each complete exposed carton face. Expanding
    that face by the pregrasp/contact-frame offset encloses every normal/tilt
    direction of the approach TCP. Distance to this larger region is a lower
    bound; using a face centre instead would not be a lower bound when in-face
    grasp offsets are available. Physical geometry and search are unchanged.
    """
    pose = np.asarray(current_tcp_world, dtype=float)
    if (pose.shape != (4, 4) or not np.all(np.isfinite(pose))
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-12, rtol=0)):
        raise ValueError("current TCP must be a finite homogeneous world pose")
    if any(not np.isfinite(value) or value < 0
           for value in (pregrasp_standoff_m, virtual_contact_offset_m)):
        raise ValueError("approach and frame offsets must be finite and nonnegative")
    face_axes = {"front": (0, -1), "top": (2, 1), "left": (1, 1), "right": (1, -1)}
    # The virtual TCP extends inward from physical contact while pregrasp
    # extends outward along the same axis, including any candidate tilt.
    reach_radius = abs(float(pregrasp_standoff_m) - float(virtual_contact_offset_m))
    records = {}
    for box in cartons:
        if box.name in records:
            raise ValueError("approach cost carton identities must be unique")
        local = box.rotation.T @ (pose[:3, 3] - box.center)
        distances = {}
        for face in dict.fromkeys(exposed_faces.get(box.name, ())):
            if face not in face_axes:
                raise ValueError(f"unknown exposed face: {face}")
            axis, sign = face_axes[face]
            nearest = np.clip(local, -box.half_extents, box.half_extents)
            nearest[axis] = sign * box.half_extents[axis]
            distances[face] = max(0.0, float(np.linalg.norm(local - nearest)) - reach_radius)
        if distances:
            records[box.name] = {"face_lower_bounds_m": distances,
                                 "translation_lower_bound_m": min(distances.values())}
    normalization = max((record["translation_lower_bound_m"] for record in records.values()), default=0.0)
    for record in records.values():
        record["normalized_cost"] = (record["translation_lower_bound_m"] / normalization
                                     if normalization > 0 else 0.0)
    return {"metric": "ACTUAL_TCP_TO_EXPOSED_APPROACH_REGION_TRANSLATION_LOWER_BOUND",
            "feasibility_claim": "NONE_NO_IK_OR_COLLISION_QUERY",
            "tcp_position_world_m": pose[:3, 3].tolist(),
            "pregrasp_standoff_m": float(pregrasp_standoff_m),
            "virtual_contact_offset_m": float(virtual_contact_offset_m),
            "approach_region_expansion_m": reach_radius,
            "normalization_distance_m": normalization, "candidates": records}


def cluster_carton_rows(cartons: Sequence[OBB], policy: RowSequencePolicy | None = None
                        ) -> tuple[tuple[OBB, ...], ...]:
    """Cluster actual centre heights with bounded cluster diameter, top first."""
    policy = policy or RowSequencePolicy()
    if not cartons:
        return ()
    tolerance = max(policy.minimum_row_tolerance_m,
                    policy.row_height_fraction * min(2 * box.half_extents[2] for box in cartons))
    rows: list[list[OBB]] = []
    for box in sorted(cartons, key=lambda b: (-b.center[2], b.center[0], b.center[1], b.name)):
        # Compare with the highest member, not the previous point: noisy chains
        # cannot merge two true layers into one arbitrarily tall cluster.
        if not rows or rows[-1][0].center[2] - box.center[2] > tolerance:
            rows.append([])
        rows[-1].append(box)
    return tuple(tuple(row) for row in rows)


class RowUnloadingState:
    """Keep row centres and failure memory across sequential physical tasks."""

    def __init__(self, config: RowSequencePolicy | None = None) -> None:
        self.config = config or RowSequencePolicy()
        self._rows: list[dict[str, Any]] = []
        self._active_row_id: str | None = None
        self._next_row = 0
        self._failures: dict[tuple[str, str], str] = {}
        self._reference_boxes: dict[str, OBB] = {}
        self._reference_context = ""
        self._fingerprint = ""

    def _scene_identity(self, cartons: Sequence[OBB], context: Any) -> str:
        current = {box.name: box for box in cartons}
        context_key = json.dumps(context, sort_keys=True, separators=(",", ":"))
        unchanged = set(current) == set(self._reference_boxes) and context_key == self._reference_context
        if unchanged:
            for name, box in current.items():
                old = self._reference_boxes[name]
                if (np.linalg.norm(box.center - old.center) > self.config.state_translation_tolerance_m
                    or np.linalg.norm(rotation_vector_from_matrix(box.rotation @ old.rotation.T))
                    > self.config.state_rotation_tolerance_rad
                    or not np.array_equal(box.half_extents, old.half_extents)):
                    unchanged = False
                    break
        if unchanged and self._fingerprint:
            return self._fingerprint
        encoded = {"context": context, "cartons": [
            {"name": name, "pose": box.world_from_local.tolist(),
             "half_extents": box.half_extents.tolist()}
            for name, box in sorted(current.items())]}
        self._fingerprint = hashlib.sha256(json.dumps(encoded, sort_keys=True,
                                                      separators=(",", ":")).encode()).hexdigest()
        self._reference_boxes = {name: OBB(box.center.copy(), box.half_extents.copy(),
                                         box.rotation.copy(), box.name, box.category)
                                 for name, box in current.items()}
        self._reference_context = context_key
        return self._fingerprint

    def record_failure(self, carton_name: str, scene_fingerprint: str, reason: str) -> None:
        """Call after bounded alternative faces/poses/places have been tried."""
        self._failures[(str(scene_fingerprint), str(carton_name))] = str(reason)

    def rank(self, cartons: Iterable[OBB], *, support_graph: SupportRelationGraph | None = None,
             candidate_costs: Mapping[str, float] | None = None,
             scene_context: Any = _UNCHANGED_CONTEXT) -> RowSelection:
        boxes = tuple(cartons)
        names = {box.name for box in boxes}
        if len(names) != len(boxes):
            raise ValueError("remaining carton identities must be unique")
        if scene_context is _UNCHANGED_CONTEXT:
            scene_context = json.loads(self._reference_context) if self._reference_context else None
        fingerprint = self._scene_identity(boxes, scene_context)
        if not boxes:
            self._active_row_id = None
            return RowSelection("COMPLETE", None, None, (), fingerprint, (), 0)
        by_name = {box.name: box for box in boxes}
        assigned = {name for row in self._rows for name in row["names"]}
        for cluster in cluster_carton_rows(boxes, self.config):
            fresh = {box.name for box in cluster} - assigned
            if not fresh:
                continue
            overlapping = [row for row in self._rows if row["names"] & {box.name for box in cluster}]
            if overlapping:
                # A newly observed carton joins the geometric row, without
                # changing the centre established from that row's first view.
                overlapping[0]["names"].update(fresh)
            else:
                lows, highs = zip(*[_bounds(box) for box in cluster])
                self._rows.append({"id": f"row_{self._next_row:03d}", "names": set(fresh),
                                   "center_y_m": float(0.5 * (min(p[1] for p in lows) + max(p[1] for p in highs))),
                                   "width_m": float(max(p[1] for p in highs) - min(p[1] for p in lows))})
                self._next_row += 1
            assigned.update(fresh)
        live_rows = [row for row in self._rows if row["names"] & names]
        row = next((item for item in live_rows if item["id"] == self._active_row_id), None)
        if row is None:
            row = max(live_rows, key=lambda item: max(by_name[name].center[2] for name in item["names"] & names))
            self._active_row_id = row["id"]
        row_names = row["names"] & names
        graph = support_graph or SupportRelationGraph.build(
            boxes, contact_tolerance_m=self.config.support_contact_tolerance_m)
        if set(graph.cartons) != names:
            raise ValueError("support graph must describe the current remaining carton identities")
        blocked: dict[str, str] = {}
        candidates = []
        for name in sorted(row_names):
            failure = self._failures.get((fingerprint, name))
            if failure is not None:
                blocked[name] = f"FAILED_IN_UNCHANGED_SCENE: {failure}"
                continue
            supported = graph.supports[name] & names
            if supported:
                blocked[name] = "SUPPORTS_REMAINING_CARTONS: " + ",".join(sorted(supported))
                continue
            faces = tuple(face for face in ("front", "top", "left", "right")
                          if not graph.face_blockers(name, face, names))
            if not faces:
                blocked[name] = "NO_EXPOSED_FACE"
                continue
            distance = abs(float(by_name[name].center[1]) - row["center_y_m"])
            normalized = distance / max(row["width_m"] / 2, 1e-12)
            actual = float((candidate_costs or {}).get(name, 0.0))
            if not np.isfinite(actual):
                raise ValueError("candidate costs must be finite normalized scores")
            actual = float(np.clip(actual, 0.0, 1.0))
            candidates.append(RankedCarton(
                by_name[name], row["id"], row["center_y_m"], distance, normalized, actual,
                normalized + self.config.actual_cost_weight * actual, faces,
                tuple(sorted(graph.supported_by[name] & names)),
                ("HIGHEST_UNFINISHED_ROW", "STABLE_INITIAL_ROW_CENTER",
                 "LATEST_GEOMETRY_AND_SUPPORT", "ALTERNATIVE_FACES_RETAINED")))
        # Centre distance has a clear geometric tier; normalized actual costs
        # resolve near-symmetric sides without overpowering centre expansion.
        candidates.sort(key=lambda item: (
            int(np.floor(item.center_distance_m / self.config.center_tie_tolerance_m + 0.5)),
            self.config.actual_cost_weight * item.normalized_actual_cost,
            item.carton.center[0], -item.carton.center[1], item.name))
        return RowSelection("READY" if candidates else "ROW_BLOCKED", row["id"], row["center_y_m"],
                            tuple(candidates), fingerprint, tuple(sorted(row_names)), len(boxes), blocked)


def height_face_prior(target: OBB, available_faces: Iterable[str], *, shoulder_height_m: float,
                      transition_halfwidth_m: float = 0.35,
                      necessary_reach: Mapping[str, bool] | None = None) -> dict[str, float]:
    """Higher score means earlier search, from fixed physical height only.

    ``necessary_reach`` may exclude a face only when a geometric necessary
    condition was checked; this function itself makes no reachability claims.
    """
    if not np.isfinite(shoulder_height_m) or not np.isfinite(transition_halfwidth_m) or transition_halfwidth_m <= 0:
        raise ValueError("face transition requires finite physical shoulder height and positive width")
    blend = float(np.clip((target.center[2] - shoulder_height_m) / transition_halfwidth_m, -1, 1))
    front = 0.5 + 0.4 * np.sin(0.5 * np.pi * blend)
    scores = {"front": float(front), "top": float(1.0 - front), "left": 0.15, "right": 0.15}
    return dict(sorted(((face, scores[face]) for face in dict.fromkeys(available_faces)
                        if face in scores and (necessary_reach or {}).get(face, True)),
                       key=lambda item: (-item[1], item[0])))


T = TypeVar("T")


def fair_face_candidates(candidates_by_face: Mapping[str, Iterable[T]], face_scores: Mapping[str, float],
                         budget: int) -> Iterable[tuple[str, T]]:
    """Lazy round-robin gives alternatives a real share of the shared budget.

    Require at least one slot per nonempty/possibly-lazy face stream; callers
    should use a larger per-task budget for subsequent offsets and rolls.
    """
    if budget < len(candidates_by_face):
        raise ValueError("face budget must reserve one attempt for every available face")
    streams = [(face, iter(items)) for face, items in sorted(candidates_by_face.items(),
               key=lambda item: (-face_scores.get(item[0], 0.0), item[0]))]
    produced = 0
    while streams and produced < budget:
        next_streams = []
        for face, stream in streams:
            try:
                item = next(stream)
            except StopIteration:
                continue
            yield face, item
            produced += 1
            next_streams.append((face, stream))
            if produced >= budget:
                break
        streams = next_streams
