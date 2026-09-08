"""Small SI-unit transform helpers with explicit frame direction."""

from __future__ import annotations

from math import isfinite, sqrt
from typing import Sequence

from unloading_contracts import EvidenceKind, Pose3D


def _matrix(values: Sequence[Sequence[float]], rows: int, cols: int, name: str) -> tuple[tuple[float, ...], ...]:
    result = tuple(tuple(float(value) for value in row) for row in values)
    if len(result) != rows or any(len(row) != cols for row in result):
        raise ValueError(f"{name} must be {rows}x{cols}")
    if not all(isfinite(value) for row in result for value in row):
        raise ValueError(f"{name} must be finite")
    return result


def validate_transform_parent_child(transform: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    """Validate T_parent_child, which maps child coordinates into parent."""

    matrix = _matrix(transform, 4, 4, "transform")
    if any(abs(matrix[3][index] - expected) > 1e-9 for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))):
        raise ValueError("transform homogeneous row is invalid")
    rotation = tuple(row[:3] for row in matrix[:3])
    for first in range(3):
        for second in range(3):
            dot = sum(rotation[index][first] * rotation[index][second] for index in range(3))
            expected = 1.0 if first == second else 0.0
            if abs(dot - expected) > 1e-6:
                raise ValueError("transform rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-6:
        raise ValueError("transform rotation must be right handed")
    return matrix


def quaternion_from_rotation(rotation: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    matrix = _matrix(rotation, 3, 3, "rotation")
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
        w = 0.25 * scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        x, y, z, w = 0.25 * scale, (matrix[0][1] + matrix[1][0]) / scale, (matrix[0][2] + matrix[2][0]) / scale, (matrix[2][1] - matrix[1][2]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        x, y, z, w = (matrix[0][1] + matrix[1][0]) / scale, 0.25 * scale, (matrix[1][2] + matrix[2][1]) / scale, (matrix[0][2] - matrix[2][0]) / scale
    else:
        scale = sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        x, y, z, w = (matrix[0][2] + matrix[2][0]) / scale, (matrix[1][2] + matrix[2][1]) / scale, 0.25 * scale, (matrix[1][0] - matrix[0][1]) / scale
    norm = sqrt(x*x + y*y + z*z + w*w)
    if norm <= 1e-12:
        raise ValueError("rotation produced an invalid quaternion")
    return (x / norm, y / norm, z / norm, w / norm)


def rotation_from_quaternion(quaternion_xyzw: Sequence[float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = sqrt(x*x + y*y + z*z + w*w)
    if not isfinite(norm) or abs(norm - 1.0) > 1e-6:
        raise ValueError("quaternion must be finite and normalized")
    return (
        (1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)),
        (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)),
        (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)),
    )


def compose_rotation(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(sum(left[row][k] * right[k][col] for k in range(3)) for col in range(3)) for row in range(3))


def transform_pose(transform_parent_child: Sequence[Sequence[float]], pose_child: Pose3D, parent_frame: str) -> Pose3D:
    matrix = validate_transform_parent_child(transform_parent_child)
    rotation = tuple(row[:3] for row in matrix[:3])
    position = tuple(sum(rotation[row][col] * pose_child.position_m[col] for col in range(3)) + matrix[row][3] for row in range(3))
    orientation = quaternion_from_rotation(compose_rotation(rotation, rotation_from_quaternion(pose_child.orientation_xyzw)))
    covariance = pose_child.covariance
    if covariance is not None:
        source = tuple(tuple(covariance[row * 6 + col] for col in range(6)) for row in range(6))
        adjoint = tuple(tuple(rotation[row][col] if row < 3 and col < 3 else rotation[row-3][col-3] if row >= 3 and col >= 3 else 0.0 for col in range(6)) for row in range(6))
        covariance = tuple(sum(adjoint[row][a] * source[a][b] * adjoint[col][b] for a in range(6) for b in range(6)) for row in range(6) for col in range(6))
    return Pose3D(position, orientation, parent_frame, pose_child.axis_convention, pose_child.evidence, covariance)


def pose_from_axes_rows(center_m: Sequence[float], axes_rows: Sequence[Sequence[float]], frame_id: str, evidence: EvidenceKind) -> Pose3D:
    rows = _matrix(axes_rows, 3, 3, "axis rows")
    # Upstream records each local axis as a row vector in camera coordinates;
    # rotation matrices expose those vectors as columns.
    rotation = tuple(tuple(rows[col][row] for col in range(3)) for row in range(3))
    validate_transform_parent_child(tuple(tuple(rotation[row][col] for col in range(3)) + (0.0,) for row in range(3)) + ((0.0, 0.0, 0.0, 1.0),))
    return Pose3D(tuple(float(value) for value in center_m), quaternion_from_rotation(rotation), frame_id, "right_handed_local_axes_stored_as_rows", evidence)
