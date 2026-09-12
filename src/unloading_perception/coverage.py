"""Deterministic camera-frustum coverage checks for carton front faces.

The routines in this module deliberately depend only on the Python standard
library.  They operate on the same manifest dictionaries consumed by Isaac,
so coverage drawings and acceptance JSON can be derived from capture-time
camera transforms instead of a second, hand-authored diagram.
"""

from __future__ import annotations

from math import isfinite
from typing import Any, Mapping, Sequence


Point3 = tuple[float, float, float]
Point2 = tuple[float, float]


def _point3(value: Sequence[float], name: str) -> Point3:
    if len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    result = tuple(float(item) for item in value)
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{name} must be finite")
    return result  # type: ignore[return-value]


def _transform(value: Sequence[Sequence[float]], name: str) -> tuple[tuple[float, ...], ...]:
    if len(value) != 4 or any(len(row) != 4 for row in value):
        raise ValueError(f"{name} must be 4x4")
    result = tuple(tuple(float(item) for item in row) for row in value)
    if not all(isfinite(item) for row in result for item in row):
        raise ValueError(f"{name} must be finite")
    if any(abs(result[3][index] - expected) > 1e-9 for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))):
        raise ValueError(f"{name} must be homogeneous")
    return result


def transform_point(T_parent_child: Sequence[Sequence[float]], point_child: Sequence[float]) -> Point3:
    """Transform one three-dimensional point into the parent frame."""

    transform = _transform(T_parent_child, "transform")
    point = _point3(point_child, "point")
    return tuple(
        sum(transform[row][column] * point[column] for column in range(3)) + transform[row][3]
        for row in range(3)
    )  # type: ignore[return-value]


def world_to_camera(T_W_C: Sequence[Sequence[float]], point_world: Sequence[float]) -> Point3:
    """Apply the inverse of a rigid capture-time optical transform."""

    transform = _transform(T_W_C, "T_W_C")
    point = _point3(point_world, "world point")
    delta = tuple(point[index] - transform[index][3] for index in range(3))
    return tuple(
        sum(transform[row][column] * delta[row] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


def project_world_point(camera: Mapping[str, Any], point_world: Sequence[float]) -> dict[str, Any]:
    """Project a world point using the declared output ``K`` and ``T_W_C``."""

    width, height = (int(value) for value in camera["resolution"])
    if width <= 0 or height <= 0:
        raise ValueError("camera resolution must be positive")
    K = tuple(float(value) for value in camera["K"])
    if len(K) != 9 or not all(isfinite(value) for value in K) or K[0] <= 0.0 or K[4] <= 0.0:
        raise ValueError("camera K must be a finite positive 3x3 pinhole matrix")
    x, y, z = world_to_camera(camera["T_W_C"], point_world)
    if z <= float(camera.get("near_clip_m", 0.0)) or z >= float(camera.get("far_clip_m", float("inf"))):
        return {"camera_xyz_m": [x, y, z], "pixel_xy": None, "inside": False, "edge_margin_px": None}
    u = K[0] * x / z + K[2]
    v = K[4] * y / z + K[5]
    margin = min(u, (width - 1.0) - u, v, (height - 1.0) - v)
    return {
        "camera_xyz_m": [x, y, z],
        "pixel_xy": [u, v],
        "inside": bool(margin >= 0.0),
        "edge_margin_px": margin,
    }


def carton_front_face(object_record: Mapping[str, Any]) -> tuple[Point3, Point3, Point3, Point3]:
    """Return the physical -X (trailer-front) face in winding order.

    The layout convention is +X into the trailer and the nominal cameras view
    the stack from lower X.  No layer/column name or fixed carton count is used.
    """

    dimensions = _point3(object_record["full_dimensions_m"], "carton dimensions")
    if any(value <= 0.0 for value in dimensions):
        raise ValueError("carton dimensions must be positive")
    hx, hy, hz = (value / 2.0 for value in dimensions)
    local = ((-hx, -hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (-hx, -hy, hz))
    return tuple(transform_point(object_record["T_W_object"], point) for point in local)  # type: ignore[return-value]


def sample_quad(corners: Sequence[Sequence[float]], density: int = 17) -> tuple[Point3, ...]:
    """Sample a quadrilateral densely, including all four edges and corners."""

    if len(corners) != 4 or density < 2:
        raise ValueError("quad sampling requires four corners and density >= 2")
    p00, p10, p11, p01 = (_point3(point, "quad corner") for point in corners)
    points = []
    for row in range(density):
        v = row / (density - 1.0)
        for column in range(density):
            u = column / (density - 1.0)
            point = tuple(
                (1.0 - u) * (1.0 - v) * p00[axis]
                + u * (1.0 - v) * p10[axis]
                + u * v * p11[axis]
                + (1.0 - u) * v * p01[axis]
                for axis in range(3)
            )
            points.append(point)
    return tuple(points)


def derive_stack_bounds(objects: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Derive the finite front-surface region from actual manifest cartons."""

    if not objects:
        raise ValueError("at least one carton is required")
    faces = [carton_front_face(item) for item in objects]
    points = [point for face in faces for point in face]
    return {
        "front_x_range_m": [min(point[0] for point in points), max(point[0] for point in points)],
        "y_range_m": [min(point[1] for point in points), max(point[1] for point in points)],
        "z_range_m": [min(point[2] for point in points), max(point[2] for point in points)],
        "carton_count": len(objects),
        "derivation": "manifest carton transforms and full dimensions; physical local -X faces",
    }


def evaluate_front_face_coverage(
    cameras: Sequence[Mapping[str, Any]],
    objects: Sequence[Mapping[str, Any]],
    *,
    density: int = 17,
) -> dict[str, Any]:
    """Evaluate per-camera and union coverage over every front-face sample."""

    if not cameras:
        raise ValueError("at least one camera is required")
    camera_ids = [str(camera["module_id"]) for camera in cameras]
    if len(camera_ids) != len(set(camera_ids)):
        raise ValueError("camera module IDs must be unique")
    rows = []
    all_union = []
    all_per_camera = {module_id: [] for module_id in camera_ids}
    for item in objects:
        corners = carton_front_face(item)
        samples = sample_quad(corners, density)
        camera_inside = {
            module_id: [project_world_point(camera, point)["inside"] for point in samples]
            for module_id, camera in zip(camera_ids, cameras)
        }
        union_inside = [any(camera_inside[module_id][index] for module_id in camera_ids) for index in range(len(samples))]
        for module_id in camera_ids:
            all_per_camera[module_id].extend(camera_inside[module_id])
        all_union.extend(union_inside)
        center = transform_point(item["T_W_object"], (0.0, 0.0, 0.0))
        rows.append({
            "simulation_object_id": str(item["simulation_object_id"]),
            "center_world_m": list(center),
            "front_face_world_m": [list(point) for point in corners],
            "per_module_sample_coverage": {
                module_id: sum(camera_inside[module_id]) / len(samples) for module_id in camera_ids
            },
            "union_sample_coverage": sum(union_inside) / len(samples),
            "fully_covered_by_union": all(union_inside),
            "uncovered_sample_count": len(samples) - sum(union_inside),
        })
    total = len(all_union)
    return {
        "schema_version": "front_face_frustum_coverage_v1",
        "claim_boundary": "FRUSTUM_COVERAGE_ONLY; rendered visibility and occlusion require semantic renders",
        "sampling_density_per_axis": density,
        "stack_region": derive_stack_bounds(objects),
        "per_module_sample_coverage": {
            module_id: sum(values) / len(values) for module_id, values in all_per_camera.items()
        },
        "union_sample_coverage": sum(all_union) / total,
        "fully_covered_carton_count": sum(row["fully_covered_by_union"] for row in rows),
        "carton_count": len(rows),
        "cartons": rows,
    }
