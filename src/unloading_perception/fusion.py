"""World-frame, non-oracle association and fusion for RGB-D modules."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp, isfinite
from typing import Sequence

import numpy as np

from .observed_faces import ObservedFace, ObservedFaceSet


@dataclass(frozen=True)
class ModuleFaceBatch:
    module_id: str
    sensor_epoch: str
    frame_sequence: int
    capture_time: float
    face_sets: tuple[ObservedFaceSet, ...]

    def __post_init__(self) -> None:
        if not self.module_id or not self.sensor_epoch or self.frame_sequence < 0 or not isfinite(self.capture_time):
            raise ValueError("module face batch identity/time is invalid")
        if any(item.module_id != self.module_id or item.frame_id != "world" for item in self.face_sets):
            raise ValueError("fusion accepts module-bound world-frame face sets only")
        if any(abs(item.capture_time - self.capture_time) > 1e-9 for item in self.face_sets):
            raise ValueError("face set and batch capture times differ")


@dataclass(frozen=True)
class FusedObservedObject:
    fusion_id: str
    source_members: tuple[tuple[str, str], ...]
    observed_faces: tuple[ObservedFace, ...]
    contributing_modules: tuple[str, ...]
    association_status: str
    coverage_status: str
    diagnostics: dict


@dataclass(frozen=True)
class FusionResult:
    sensor_epoch: str
    capture_time_range: tuple[float, float]
    expected_modules: tuple[str, ...]
    received_modules: tuple[str, ...]
    objects: tuple[FusedObservedObject, ...]
    coverage_status: str


def _points(face_set: ObservedFaceSet) -> np.ndarray:
    if not face_set.faces:
        return np.empty((0, 3), dtype=float)
    return np.vstack([np.asarray(face.corners_3d_m, dtype=float) for face in face_set.faces])


def _face_distance(first: ObservedFace, second: ObservedFace) -> tuple[float, float, float]:
    left = np.asarray(first.corners_3d_m, dtype=float)
    right = np.asarray(second.corners_3d_m, dtype=float)
    normal_alignment = abs(float(np.dot(first.plane_normal, second.plane_normal)))
    plane_distance = abs(float(first.plane_offset_m - second.plane_offset_m))
    corner_distance = max(
        float(np.max(np.min(np.linalg.norm(left[:, None, :] - right[None, :, :], axis=2), axis=1))),
        float(np.max(np.min(np.linalg.norm(right[:, None, :] - left[None, :, :], axis=2), axis=1))),
    )
    return normal_alignment, plane_distance, corner_distance


def _association_score(first: ObservedFaceSet, second: ObservedFaceSet) -> tuple[float, dict]:
    left, right = _points(first), _points(second)
    if not left.size or not right.size:
        return 0.0, {"reason": "NO_OBSERVED_FACE"}
    center_distance = float(np.linalg.norm(np.mean(left, axis=0) - np.mean(right, axis=0)))
    same_face = []
    for left_face in first.faces:
        for right_face in second.faces:
            alignment, plane_distance, corner_distance = _face_distance(left_face, right_face)
            if alignment >= 0.94 and plane_distance <= 0.06:
                same_face.append((corner_distance, alignment, plane_distance))
    if not same_face:
        return 0.0, {"reason": "NO_COPLANAR_OVERLAP", "center_distance_m": center_distance}
    corner_distance, alignment, plane_distance = min(same_face)
    score = exp(-center_distance / 0.30) * exp(-corner_distance / 0.15) * alignment
    diagnostics = {
        "center_distance_m": center_distance,
        "matched_face_corner_distance_m": corner_distance,
        "normal_alignment": alignment,
        "plane_distance_m": plane_distance,
        "score": score,
    }
    return (score if center_distance <= 0.45 and corner_distance <= 0.20 else 0.0), diagnostics


def _merge_faces(face_sets: Sequence[ObservedFaceSet], tolerance_m: float) -> tuple[tuple[ObservedFace, ...], bool]:
    fused: list[ObservedFace] = []
    conflict = False
    for face_set in face_sets:
        for face in face_set.faces:
            duplicate = False
            for retained in fused:
                alignment, plane_distance, corner_distance = _face_distance(face, retained)
                if alignment >= 0.94 and plane_distance <= 0.06:
                    if corner_distance <= tolerance_m:
                        duplicate = True
                        # Keep the higher-support actual observation; never average.
                        if face.point_support_count > retained.point_support_count:
                            fused[fused.index(retained)] = face
                    elif corner_distance <= 0.20:
                        conflict = True
                    break
            if not duplicate:
                fused.append(face)
    return tuple(fused), conflict


def fuse_module_face_batches(
    batches: Sequence[ModuleFaceBatch], *, expected_modules: Sequence[str],
    maximum_capture_skew_s: float = 1.0 / 30.0,
    minimum_association_score: float = 0.20,
    duplicate_face_tolerance_m: float = 0.04,
) -> FusionResult:
    """Associate by measured world geometry; oracle IDs are neither accepted nor used."""

    if not batches:
        raise ValueError("at least one module batch is required")
    modules = tuple(batch.module_id for batch in batches)
    if len(set(modules)) != len(modules):
        raise ValueError("one batch per module is required")
    epochs = {batch.sensor_epoch for batch in batches}
    if len(epochs) != 1:
        raise ValueError("STALE_OR_MIXED_SENSOR_EPOCH")
    times = tuple(batch.capture_time for batch in batches)
    if max(times) - min(times) > maximum_capture_skew_s + 1e-12:
        raise ValueError("NON_SIMULTANEOUS_MODULE_CAPTURES")

    groups: list[list[ObservedFaceSet]] = []
    association_diagnostics: list[dict] = []
    for batch in sorted(batches, key=lambda item: item.module_id):
        for face_set in batch.face_sets:
            best_group = None
            best_score = 0.0
            best_diagnostics = {}
            for group_index, group in enumerate(groups):
                if any(item.module_id == face_set.module_id for item in group):
                    continue
                for existing in group:
                    score, diagnostics = _association_score(existing, face_set)
                    if score > best_score:
                        best_group, best_score, best_diagnostics = group_index, score, diagnostics
            if best_group is not None and best_score >= minimum_association_score:
                groups[best_group].append(face_set)
                association_diagnostics.append({
                    "members": [(item.module_id, item.source_instance_id) for item in groups[best_group]],
                    **best_diagnostics,
                })
            else:
                groups.append([face_set])

    expected = tuple(sorted(set(str(item) for item in expected_modules)))
    received = tuple(sorted(modules))
    coverage = "COMPLETE_MODULE_SET" if received == expected else "DEGRADED_MISSING_MODULE"
    objects = []
    for index, group in enumerate(groups):
        faces, conflict = _merge_faces(group, duplicate_face_tolerance_m)
        contributors = tuple(sorted(item.module_id for item in group))
        members = tuple(sorted((item.module_id, item.source_instance_id) for item in group))
        objects.append(FusedObservedObject(
            f"fused-{index:04d}", members, faces, contributors,
            "CONFLICT_RETAINED_NO_AVERAGE" if conflict else "ASSOCIATED_BY_WORLD_FACE_GEOMETRY",
            coverage,
            {"oracle_identity_used": False, "association_records": [
                item for item in association_diagnostics if all(member in item["members"] for member in members)
            ]},
        ))
    return FusionResult(next(iter(epochs)), (min(times), max(times)), expected, received, tuple(objects), coverage)
