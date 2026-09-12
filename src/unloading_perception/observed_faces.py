"""First-class observed-face geometry, separate from cuboid completion."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import isfinite
from typing import Any, Mapping, Sequence

import numpy as np

from .geometry import validate_transform_parent_child
from .upstream_v4 import REGISTERED_DEPTH_FACE, REGISTERED_JOINT_FACE


DIRECT_FACE_EVIDENCE = frozenset({
    REGISTERED_DEPTH_FACE,
    REGISTERED_JOINT_FACE,
    "registered_sam_visible_face",
    "monocular_depth_plane",
    "joint_sam_depth_multiplane_pnp",
})


@dataclass(frozen=True)
class ObservedFace:
    face_id: str
    corner_indices: tuple[int, int, int, int]
    boundary_2d_px: tuple[tuple[float, float], ...]
    corners_3d_m: tuple[tuple[float, float, float], ...]
    plane_normal: tuple[float, float, float]
    plane_offset_m: float
    point_support_count: int
    point_support_ratio: float
    plane_residual_m: float
    evidence: str
    frame_id: str
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.face_id or not self.frame_id or self.evidence not in DIRECT_FACE_EVIDENCE:
            raise ValueError("observed face identity/evidence is invalid")
        if len(set(self.corner_indices)) != 4 or len(self.boundary_2d_px) != 4 or len(self.corners_3d_m) != 4:
            raise ValueError("an observed face requires four distinct shared cuboid corners")
        normal = np.asarray(self.plane_normal, dtype=float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or abs(float(np.linalg.norm(normal)) - 1.0) > 1e-5:
            raise ValueError("observed face normal must be a finite unit vector")
        if self.point_support_count < 0 or not 0.0 <= self.point_support_ratio <= 1.0:
            raise ValueError("observed face support is invalid")
        if not isfinite(self.plane_offset_m) or not isfinite(self.plane_residual_m) or self.plane_residual_m < 0.0:
            raise ValueError("observed face plane values are invalid")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ObservedFaceSet:
    source_instance_id: str
    module_id: str
    capture_id: str
    capture_time: float
    frame_id: str
    faces: tuple[ObservedFace, ...]
    shared_edges: tuple[tuple[str, str, tuple[int, int]], ...]
    complete_cuboid_status: str

    def __post_init__(self) -> None:
        if not all((self.source_instance_id, self.module_id, self.capture_id, self.frame_id)):
            raise ValueError("observed face-set identities are required")
        if not isfinite(float(self.capture_time)):
            raise ValueError("observed face-set capture time is invalid")
        if any(face.frame_id != self.frame_id for face in self.faces):
            raise ValueError("all observed faces must share the declared frame")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _face_plane(points: np.ndarray) -> tuple[tuple[float, float, float], float]:
    first = points[1] - points[0]
    second = points[3] - points[0]
    normal = np.cross(first, second)
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-9:
        raise ValueError("observed face corners are degenerate")
    normal /= norm
    center = np.mean(points, axis=0)
    if float(np.dot(normal, center)) > 0.0:
        normal *= -1.0
    return tuple(float(value) for value in normal), -float(np.dot(normal, center))


def observed_faces_from_geometry_record(
    record: Mapping[str, Any], *, source_instance_id: str, module_id: str,
    capture_id: str, capture_time: float, frame_id: str,
) -> ObservedFaceSet:
    """Adapt only directly observed faces; completion faces are ignored."""

    corners = np.asarray(record.get("corners_3d", ()), dtype=float)
    faces = []
    plane_counts = tuple(int(value) for value in record.get("plane_inlier_counts", ()))
    overall_support = min(1.0, max(0.0, float(record.get("plane_inlier_ratio", 0.0))))
    residual = max(0.0, float(record.get("plane_residual_mean", 0.0)))
    for ordinal, source_face in enumerate(record.get("camera_facing_faces", ())):
        evidence = str(source_face.get("evidence", ""))
        if evidence not in DIRECT_FACE_EVIDENCE:
            continue
        indices = tuple(int(value) for value in source_face.get("corner_indices", ()))
        boundary = tuple(tuple(float(value) for value in point) for point in source_face.get("corners_2d", ()))
        if corners.shape != (8, 3) or len(indices) != 4 or any(index < 0 or index >= 8 for index in indices):
            continue
        points = corners[np.asarray(indices)]
        normal, offset = _face_plane(points)
        axis = int(source_face.get("axis_index", -1))
        support_count = plane_counts[axis] if 0 <= axis < len(plane_counts) else 0
        faces.append(ObservedFace(
            f"{source_instance_id}:face:{ordinal}", indices, boundary,
            tuple(tuple(float(value) for value in point) for point in points),
            normal, offset, support_count, overall_support, residual, evidence, frame_id,
            {
                key: source_face[key] for key in (
                    "mask_precision", "mask_coverage", "mask_iou", "metrics_before_joint_refinement"
                ) if key in source_face
            },
        ))
    shared = []
    for first_index, first in enumerate(faces):
        for second in faces[first_index + 1:]:
            common = tuple(sorted(set(first.corner_indices) & set(second.corner_indices)))
            if len(common) == 2:
                shared.append((first.face_id, second.face_id, common))
    complete_status = (
        "SUFFICIENT_MULTIFACE_EVIDENCE" if len(faces) >= 2
        else "INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR" if len(faces) == 1
        else "NO_CERTIFIED_OBSERVED_FACE"
    )
    return ObservedFaceSet(
        source_instance_id, module_id, capture_id, float(capture_time), frame_id,
        tuple(faces), tuple(shared), complete_status,
    )


def transform_observed_face_set(
    face_set: ObservedFaceSet, T_parent_child: Sequence[Sequence[float]], parent_frame: str,
) -> ObservedFaceSet:
    matrix = np.asarray(validate_transform_parent_child(T_parent_child), dtype=float)
    rotation, translation = matrix[:3, :3], matrix[:3, 3]
    transformed = []
    for face in face_set.faces:
        points = (rotation @ np.asarray(face.corners_3d_m).T).T + translation
        normal = rotation @ np.asarray(face.plane_normal)
        offset = -float(np.dot(normal, np.mean(points, axis=0)))
        transformed.append(ObservedFace(**{
            **face.__dict__,
            "corners_3d_m": tuple(tuple(float(value) for value in point) for point in points),
            "plane_normal": tuple(float(value) for value in normal),
            "plane_offset_m": offset,
            "frame_id": parent_frame,
        }))
    return ObservedFaceSet(**{
        **face_set.__dict__, "frame_id": parent_frame, "faces": tuple(transformed),
    })
