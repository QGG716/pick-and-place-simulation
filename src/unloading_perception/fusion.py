"""World-frame, non-oracle association and fusion for RGB-D modules."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
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
    # Fix floating-point reduction order without changing association geometry.
    return np.asarray(sorted(point for face in face_set.faces for point in face.corners_3d_m), dtype=float)


def _face_distance(first: ObservedFace, second: ObservedFace) -> tuple[float, float, float]:
    left = np.asarray(first.corners_3d_m, dtype=float)
    right = np.asarray(second.corners_3d_m, dtype=float)
    normal_alignment = abs(float(np.dot(first.plane_normal, second.plane_normal)))
    sign = 1.0 if np.dot(first.plane_normal, second.plane_normal) >= 0 else -1.0
    plane_distance = abs(float(first.plane_offset_m - sign * second.plane_offset_m))
    corner_distance = max(
        float(np.max(np.min(np.linalg.norm(left[:, None, :] - right[None, :, :], axis=2), axis=1))),
        float(np.max(np.min(np.linalg.norm(right[:, None, :] - left[None, :, :], axis=2), axis=1))),
    )
    return normal_alignment, plane_distance, corner_distance


def _intersection(first: ObservedFace, second: ObservedFace) -> tuple[float, tuple]:
    """Intersection ratio and polygon projected onto the first measured plane."""
    p=np.asarray(first.corners_3d_m); q=np.asarray(second.corners_3d_m)
    u=p[1]-p[0]; u/=np.linalg.norm(u)
    v=np.cross(first.plane_normal,u)
    a=np.column_stack(((p-p[0])@u,(p-p[0])@v))
    b=np.column_stack(((q-p[0])@u,(q-p[0])@v))
    def signed(poly):
        if len(poly)<3: return 0.
        poly=np.asarray(poly); return float(np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1))/2)
    if signed(a)<0: a=a[::-1]
    if signed(b)<0: b=b[::-1]
    poly=list(a)
    for start,end in zip(b,np.roll(b,-1,axis=0)):
        previous=poly; poly=[]
        if not previous: break
        edge=end-start
        def side(point): return edge[0]*(point[1]-start[1])-edge[1]*(point[0]-start[0])
        for s,e in zip(previous,previous[1:]+previous[:1]):
            ds,de=side(s),side(e)
            if (ds>=0)!=(de>=0): poly.append(s+(e-s)*ds/(ds-de))
            if de>=0: poly.append(e)
    ratio = abs(signed(poly))/max(1e-12,min(abs(signed(a)),abs(signed(b))))
    polygon = tuple(tuple(float(x) for x in p[0] + point[0]*u + point[1]*v) for point in poly)
    return ratio, polygon


def _overlap(first: ObservedFace, second: ObservedFace) -> float:
    """Convex intersection / smaller patch area, in their common plane."""
    return _intersection(first, second)[0]


def _association_score(first: ObservedFaceSet, second: ObservedFaceSet) -> tuple[float, dict]:
    left, right = _points(first), _points(second)
    if not left.size or not right.size:
        return 0.0, {"reason": "NO_OBSERVED_FACE"}
    center_distance = float(np.linalg.norm(np.mean(left, axis=0) - np.mean(right, axis=0)))
    same_face = []
    for left_face in first.faces:
        for right_face in second.faces:
            alignment, plane_distance, corner_distance = _face_distance(left_face, right_face)
            overlap = _overlap(left_face, right_face)
            if alignment >= 0.94 and plane_distance <= 0.06 and overlap >= 0.20:
                same_face.append((overlap, alignment, plane_distance, corner_distance))
    if not same_face:
        return 0.0, {"reason": "NO_COPLANAR_OVERLAP", "center_distance_m": center_distance}
    overlap, alignment, plane_distance, corner_distance = max(same_face)
    score = overlap * alignment * exp(-plane_distance / .06)
    diagnostics = {
        "center_distance_m": center_distance,
        "matched_face_corner_distance_m": corner_distance,
        "normal_alignment": alignment,
        "plane_distance_m": plane_distance,
        "score": score,
        "intersection_over_smaller_patch": overlap,
    }
    return score, diagnostics


def _reduce_faces(face_sets: Sequence[ObservedFaceSet], tolerance_m: float) -> tuple[tuple[ObservedFace, ...], dict]:
    """Reduce only an associated group; conflicts always compare raw observations.

    Highest support wins, then lexicographic acquisition/source identity. A
    removed member never acts as a bridge in a non-transitive duplicate chain.
    """
    records = sorted([
        ((item.module_id, item.capture_id, item.source_instance_id, face.face_id), face)
        for item in face_sets for face in item.faces
    ], key=lambda record: record[0])
    keys = [key for key, _ in records]
    if len(set(keys)) != len(keys):
        raise ValueError("DUPLICATE_FACE_SOURCE_IDENTITY")

    def source(key):
        return dict(zip(("module_id", "capture_id", "source_instance_id", "face_id"), key))

    duplicates = set()
    conflicts = []
    for i, (left_key, left) in enumerate(records):
        for right_key, right in records[i + 1:]:
            alignment, plane_distance, corner_distance = _face_distance(left, right)
            if alignment < .94 or plane_distance > .06:
                continue
            overlap, polygon = _intersection(left, right)
            if corner_distance <= tolerance_m and plane_distance <= .01 and overlap > 0.:
                duplicates.add((left_key, right_key))
            elif overlap >= .20 and plane_distance > .01:
                conflicts.append({
                    "first": source(left_key), "second": source(right_key),
                    "reason": "OVERLAPPING_PLANE_POSITION_CONFLICT",
                    "normal_alignment": alignment, "plane_distance_m": plane_distance,
                    "corner_distance_m": corner_distance,
                    "intersection_over_smaller_patch": overlap,
                    "overlap_polygon_on_first_plane_m": polygon,
                })

    retained = []
    members = []
    for key, face in sorted(records, key=lambda record: (-record[1].point_support_count, record[0])):
        # Check every surviving representative, never the first unrelated plane.
        matches = [rep_key for rep_key, _ in retained
                   if tuple(sorted((key, rep_key))) in duplicates]
        if matches:
            members.append({"member": source(key), "representative": source(matches[0])})
        else:
            retained.append((key, face))
    retained.sort(key=lambda record: record[0])
    members.sort(key=lambda item: tuple(item["member"].values()))
    return tuple(face for _, face in retained), {
        "input_face_count": len(records), "representative_count": len(retained),
        "representatives": [source(key) for key, _ in retained],
        "duplicate_members": members, "conflicts": conflicts,
    }


def _merge_faces(face_sets: Sequence[ObservedFaceSet], tolerance_m: float) -> tuple[tuple[ObservedFace, ...], bool]:
    faces, diagnostics = _reduce_faces(face_sets, tolerance_m)
    return faces, bool(diagnostics["conflicts"])


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
    if not set(modules).issubset(expected_modules):
        raise ValueError("UNKNOWN_MODULE")
    if len({batch.frame_sequence for batch in batches}) != 1:
        raise ValueError("MIXED_CAPTURE_GROUP")
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
    values = sorted((v for b in batches for v in b.face_sets), key=lambda v:(v.module_id,v.capture_id,v.source_instance_id))
    keys = [(v.module_id,v.capture_id,v.source_instance_id) for v in values]
    if len(set(keys)) != len(keys): raise ValueError("DUPLICATE_INSTANCE_IN_BATCH")
    candidates = {i:[] for i in range(len(values))}
    for i,first in enumerate(values):
        for j in range(i+1,len(values)):
            if first.module_id == values[j].module_id: continue
            score, diagnostics = _association_score(first,values[j])
            if score >= minimum_association_score:
                candidates[i].append((score,j,diagnostics)); candidates[j].append((score,i,diagnostics))
    choices={}; ambiguous=set()
    for i,options in candidates.items():
        options.sort(key=lambda x:(-x[0],keys[x[1]]))
        if len(options)>1 and options[0][0]-options[1][0] <= .05:
            ambiguous.add(i)
        elif options: choices[i]=options[0][1]
    used=set()
    for i,first in enumerate(values):
        if i in used: continue
        j=choices.get(i)
        if j is not None and choices.get(j)==i and j not in used:
            group=[first,values[j]]; used.update((i,j))
            association_diagnostics.append({'members':[(v.module_id,v.source_instance_id) for v in group],**candidates[i][0][2]})
        else: group=[first]; used.add(i)
        groups.append(group)

    expected = tuple(sorted(set(str(item) for item in expected_modules)))
    received = tuple(sorted(modules))
    coverage = "COMPLETE_MODULE_SET" if received == expected else "DEGRADED_MISSING_MODULE"
    objects = []
    for index, group in enumerate(groups):
        faces, reduction = _reduce_faces(group, duplicate_face_tolerance_m)
        conflict = bool(reduction["conflicts"])
        contributors = tuple(sorted(item.module_id for item in group))
        members = tuple(sorted((item.module_id, item.source_instance_id) for item in group))
        objects.append(FusedObservedObject(
            "fusion-" + hashlib.sha256(json.dumps([(v.module_id,v.capture_id,v.source_instance_id) for v in group]).encode()).hexdigest()[:20], members, faces, contributors,
            "CONFLICT_RETAINED_NO_AVERAGE" if conflict else "AMBIGUOUS_RETAINED" if len(group)==1 and candidates[values.index(group[0])] else "ASSOCIATED_BY_WORLD_FACE_GEOMETRY" if len(group)>1 else "UNASSOCIATED_RETAINED",
            coverage,
            {"oracle_identity_used": False, "face_reduction": reduction, "association_records": [
                item for item in association_diagnostics if all(member in item["members"] for member in members)
            ]},
        ))
    return FusionResult(next(iter(epochs)), (min(times), max(times)), expected, received, tuple(objects), coverage)
