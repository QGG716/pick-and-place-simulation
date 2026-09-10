"""Pure geometry for the M-710 ideal independent-cup suction mode.

The contact model in this module deliberately answers only geometric questions.
It does not waive or perform robot, rigid-tool, payload, or environment collision
checks.  Callers must continue to apply those existing checks independently.

The physical contact-frame convention is the one used by the layout-bound
M-710 planner: local ``+Z`` points from the suction head into the selected carton
face, while local ``X/Y`` span the 6 x 12 cup array.  Cup identifiers and bit
positions are stable: ``index = column * rows + row``.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos
from typing import Any, Mapping, Sequence

import numpy as np

from .geometry import OBB


IDEAL_INDEPENDENT_CUPS_MODE = "ideal_independent_cups"
HOLDING_CAPACITY_ASSUMPTION = "sufficient_for_selected_geometric_contacts"
M710_CUP_ROWS = 6
M710_CUP_COLUMNS = 12
M710_CUP_COUNT = M710_CUP_ROWS * M710_CUP_COLUMNS
M710_CUP_PITCH_M = (0.048, 0.048)
M710_CUP_RADIUS_M = 0.0215
M710_CUP_ZONE_COUNT = 3

_FACE_AXIS_SIGN = {
    "front": (0, -1),
    "left": (1, 1),
    "right": (1, -1),
    "top": (2, 1),
}
_COMPARISON_EPSILON_M = 1e-12


def _finite_se3(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(
        transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0
    ):
        raise ValueError(f"{name} must be a rigid homogeneous transform")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0.0):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0.0):
        raise ValueError(f"{name} rotation must be proper")
    return transform.copy()


@dataclass(frozen=True)
class IndependentCup:
    """One physical FG42 cup with a stable identity in the contact frame."""

    index: int
    cup_id: str
    row: int
    column: int
    zone: int
    center_contact_frame_m: tuple[float, float, float]
    seal_radius_m: float

    @property
    def contact_frame_from_cup(self) -> np.ndarray:
        """Pose of the cup centre; cup-local +Z is the working-face normal."""

        result = np.eye(4)
        result[:3, 3] = np.asarray(self.center_contact_frame_m, dtype=float)
        return result

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "cup_id": self.cup_id,
            "row": self.row,
            "column": self.column,
            "zone": self.zone,
            "center_contact_frame_m": list(self.center_contact_frame_m),
            "T_contact_frame_cup": self.contact_frame_from_cup.tolist(),
            "seal_radius_m": self.seal_radius_m,
            "working_surface": {
                "frame": "nominal_compressed_physical_contact",
                "cup_local_plane": "z=0",
                "cup_local_inward_normal": [0.0, 0.0, 1.0],
                "shape": "complete_circular_seal_ring",
            },
        }


@dataclass(frozen=True)
class IndependentCupArray:
    """Validated rectangular array; M-710 construction fixes it at 72 cups."""

    rows: int
    columns: int
    pitch_row_column_m: tuple[float, float]
    seal_radius_m: float
    zone_count: int
    cups: tuple[IndependentCup, ...]

    def __post_init__(self) -> None:
        if (self.rows, self.columns, self.zone_count) != (
            M710_CUP_ROWS,
            M710_CUP_COLUMNS,
            M710_CUP_ZONE_COUNT,
        ):
            raise ValueError("the M-710 cup array topology must remain 6 x 12 x 3 zones")
        if len(self.cups) != M710_CUP_COUNT:
            raise ValueError("the M-710 cup array must preserve all 72 physical cups")
        expected_ids = tuple(
            f"cup_r{row:02d}_c{column:02d}"
            for column in range(self.columns)
            for row in range(self.rows)
        )
        if tuple(cup.index for cup in self.cups) != tuple(range(M710_CUP_COUNT)):
            raise ValueError("cup indices must be the stable contiguous range 0..71")
        if tuple(cup.cup_id for cup in self.cups) != expected_ids:
            raise ValueError("cup IDs must follow the stable column-major M-710 convention")
        pitch = np.asarray(self.pitch_row_column_m, dtype=float)
        if not np.allclose(pitch, M710_CUP_PITCH_M, atol=1e-12, rtol=0.0):
            raise ValueError("the M-710 cup array must preserve the confirmed 48 mm pitch")
        if not np.isclose(
            self.seal_radius_m, M710_CUP_RADIUS_M, atol=1e-12, rtol=0.0
        ):
            raise ValueError("the M-710 cup array must preserve the FG42 21.5 mm radius")
        row_offsets = (np.arange(self.rows) - 0.5 * (self.rows - 1)) * pitch[0]
        column_offsets = (
            np.arange(self.columns) - 0.5 * (self.columns - 1)
        ) * pitch[1]
        columns_per_zone = self.columns // self.zone_count
        for cup in self.cups:
            expected_row = cup.index % self.rows
            expected_column = cup.index // self.rows
            expected_center = (
                float(row_offsets[expected_row]),
                float(column_offsets[expected_column]),
                0.0,
            )
            if (
                cup.row != expected_row
                or cup.column != expected_column
                or cup.zone != expected_column // columns_per_zone
                or not np.allclose(
                    cup.center_contact_frame_m,
                    expected_center,
                    atol=1e-12,
                    rtol=0.0,
                )
                or not np.isclose(
                    cup.seal_radius_m,
                    self.seal_radius_m,
                    atol=1e-12,
                    rtol=0.0,
                )
            ):
                raise ValueError("cup local poses, zones, and radii must follow stable IDs")

    @property
    def cup_ids(self) -> tuple[str, ...]:
        return tuple(cup.cup_id for cup in self.cups)

    def to_dict(self) -> dict[str, Any]:
        return {
            "rows": self.rows,
            "columns": self.columns,
            "cup_count": len(self.cups),
            "pitch_row_column_m": list(self.pitch_row_column_m),
            "seal_radius_m": self.seal_radius_m,
            "zone_count": self.zone_count,
            "index_order": "column_major_index_equals_column_times_rows_plus_row",
            "array_frame": {
                "frame": "nominal_compressed_physical_contact",
                "row_axis": "+X",
                "column_axis": "+Y",
                "inward_working_normal": "+Z",
            },
            "cups": [cup.to_dict() for cup in self.cups],
        }


def build_m710_independent_cup_array(
    *,
    rows: int = M710_CUP_ROWS,
    columns: int = M710_CUP_COLUMNS,
    pitch_m: Sequence[float] = M710_CUP_PITCH_M,
    cup_radius_m: float = M710_CUP_RADIUS_M,
    zone_count: int = M710_CUP_ZONE_COUNT,
) -> IndependentCupArray:
    """Build the confirmed 6 x 12 FG42 array with deterministic stable IDs."""

    if isinstance(rows, bool) or isinstance(columns, bool):
        raise ValueError("cup rows and columns must be integers")
    if int(rows) != rows or int(columns) != columns:
        raise ValueError("cup rows and columns must be integers")
    rows, columns = int(rows), int(columns)
    if rows != M710_CUP_ROWS or columns != M710_CUP_COLUMNS:
        raise ValueError("the M-710 independent-cup contract requires exactly 6 x 12 cups")
    if isinstance(zone_count, bool) or int(zone_count) != zone_count:
        raise ValueError("zone_count must be an integer")
    zone_count = int(zone_count)
    if zone_count != M710_CUP_ZONE_COUNT or columns % zone_count != 0:
        raise ValueError("the M-710 array requires three equal column zones")
    pitch = np.asarray(pitch_m, dtype=float)
    radius = float(cup_radius_m)
    if (
        pitch.shape != (2,)
        or not np.all(np.isfinite(pitch))
        or np.any(pitch <= 0.0)
        or not np.isfinite(radius)
        or radius <= 0.0
    ):
        raise ValueError("cup pitch and seal radius must be finite and positive")
    if not np.allclose(pitch, M710_CUP_PITCH_M, atol=1e-12, rtol=0.0):
        raise ValueError("the confirmed M-710 cup pitch is exactly 48 mm on both axes")
    if not np.isclose(radius, M710_CUP_RADIUS_M, atol=1e-12, rtol=0.0):
        raise ValueError("the confirmed FG42 seal radius is exactly 21.5 mm")

    row_offsets = (np.arange(rows) - 0.5 * (rows - 1)) * pitch[0]
    column_offsets = (np.arange(columns) - 0.5 * (columns - 1)) * pitch[1]
    columns_per_zone = columns // zone_count
    cups: list[IndependentCup] = []
    for column, offset_y in enumerate(column_offsets):
        for row, offset_x in enumerate(row_offsets):
            index = column * rows + row
            cups.append(
                IndependentCup(
                    index=index,
                    cup_id=f"cup_r{row:02d}_c{column:02d}",
                    row=row,
                    column=column,
                    zone=column // columns_per_zone,
                    center_contact_frame_m=(float(offset_x), float(offset_y), 0.0),
                    seal_radius_m=radius,
                )
            )
    if len(cups) != M710_CUP_COUNT or any(cup.index != i for i, cup in enumerate(cups)):
        raise AssertionError("internal M-710 cup indexing error")
    return IndependentCupArray(
        rows=rows,
        columns=columns,
        pitch_row_column_m=(float(pitch[0]), float(pitch[1])),
        seal_radius_m=radius,
        zone_count=zone_count,
        cups=tuple(cups),
    )


@dataclass(frozen=True)
class CupFaceContact:
    """Geometric result for one complete circular sealing ring."""

    index: int
    cup_id: str
    target_id: str
    target_face: str
    geometrically_eligible: bool
    reason: str | None
    center_signed_gap_m: float | None
    ring_signed_gap_range_m: tuple[float, float] | None
    normal_alignment: float
    face_contact_center_world_m: tuple[float, float, float] | None
    projected_ring_half_extents_on_face_m: tuple[float, float] | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "cup_id": self.cup_id,
            "target_id": self.target_id,
            "target_face": self.target_face,
            "geometrically_eligible": self.geometrically_eligible,
            "reason": self.reason,
            "center_signed_gap_m": self.center_signed_gap_m,
            "ring_signed_gap_range_m": (
                None
                if self.ring_signed_gap_range_m is None
                else list(self.ring_signed_gap_range_m)
            ),
            "normal_alignment": self.normal_alignment,
            "face_contact_center_world_m": (
                None
                if self.face_contact_center_world_m is None
                else list(self.face_contact_center_world_m)
            ),
            "projected_ring_half_extents_on_face_m": (
                None
                if self.projected_ring_half_extents_on_face_m is None
                else list(self.projected_ring_half_extents_on_face_m)
            ),
        }


@dataclass(frozen=True)
class IndependentCupGeometry:
    """All 72 per-cup geometric results at one physical contact pose."""

    target_id: str
    target_face: str
    physical_contact_pose_world: np.ndarray
    cups: tuple[IndependentCup, ...]
    contacts: tuple[CupFaceContact, ...]
    geometrically_eligible_mask: tuple[bool, ...]
    max_attachment_gap_m: float
    maximum_penetration_m: float
    max_normal_misalignment_rad: float
    suction_edge_margin_m: float

    def __post_init__(self) -> None:
        if (
            len(self.cups) != M710_CUP_COUNT
            or len(self.contacts) != M710_CUP_COUNT
            or len(self.geometrically_eligible_mask) != M710_CUP_COUNT
        ):
            raise ValueError("independent-cup geometry must preserve 72 ordered entries")
        if any(type(value) is not bool for value in self.geometrically_eligible_mask):
            raise ValueError("geometrically_eligible_mask must contain 72 booleans")
        if tuple(contact.index for contact in self.contacts) != tuple(range(M710_CUP_COUNT)):
            raise ValueError("cup contact entries must retain stable index order")
        if tuple(contact.cup_id for contact in self.contacts) != tuple(
            cup.cup_id for cup in self.cups
        ):
            raise ValueError("cup contact entries must retain stable cup IDs")
        if tuple(contact.geometrically_eligible for contact in self.contacts) != (
            self.geometrically_eligible_mask
        ):
            raise ValueError("cup contact records and eligible mask disagree")

    @property
    def geometrically_eligible_ids(self) -> tuple[str, ...]:
        return tuple(
            cup.cup_id
            for cup, eligible in zip(self.cups, self.geometrically_eligible_mask)
            if eligible
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "target_id": self.target_id,
            "target_face": self.target_face,
            "physical_contact_pose_world": self.physical_contact_pose_world.tolist(),
            "geometrically_eligible_mask": list(self.geometrically_eligible_mask),
            "geometrically_eligible_ids": list(self.geometrically_eligible_ids),
            "geometrically_eligible_count": sum(self.geometrically_eligible_mask),
            "max_attachment_gap_m": self.max_attachment_gap_m,
            "maximum_penetration_m": self.maximum_penetration_m,
            "max_normal_misalignment_rad": self.max_normal_misalignment_rad,
            "suction_edge_margin_m": self.suction_edge_margin_m,
            "contacts": [contact.to_dict() for contact in self.contacts],
        }


def evaluate_independent_cup_geometry(
    physical_contact_pose_world: Sequence[Sequence[float]],
    target: OBB,
    target_face: str,
    cup_array: IndependentCupArray,
    *,
    max_attachment_gap_m: float = 0.002,
    maximum_penetration_m: float = 0.0002,
    max_normal_misalignment_rad: float = np.deg2rad(5.0),
    suction_edge_margin_m: float = 0.0,
) -> IndependentCupGeometry:
    """Evaluate every full sealing ring against one named target-carton face.

    Each ring is projected along contact-frame ``+Z`` onto the named face.  The
    analytic test checks the *entire* circular ring's face footprint and its
    minimum/maximum signed gap, rather than accepting a center-ray hit alone.
    """

    if target_face not in _FACE_AXIS_SIGN:
        raise ValueError(f"unsupported target face {target_face!r}")
    pose = _finite_se3(physical_contact_pose_world, "physical_contact_pose_world")
    tolerances = np.asarray(
        [
            max_attachment_gap_m,
            maximum_penetration_m,
            max_normal_misalignment_rad,
            suction_edge_margin_m,
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(tolerances)) or np.any(tolerances < 0.0):
        raise ValueError("cup contact tolerances must be finite and non-negative")
    if max_normal_misalignment_rad >= 0.5 * np.pi:
        raise ValueError("max_normal_misalignment_rad must be below pi/2")
    if len(cup_array.cups) != M710_CUP_COUNT:
        raise ValueError("ideal_independent_cups requires a fixed 72-cup array")

    face_axis, face_sign = _FACE_AXIS_SIGN[target_face]
    tangent_axes = tuple(axis for axis in range(3) if axis != face_axis)
    contact_rotation = pose[:3, :3]
    box_from_contact_rotation = target.rotation.T @ contact_rotation
    ray_local = box_from_contact_rotation[:, 2]
    inward_local = np.zeros(3, dtype=float)
    inward_local[face_axis] = -float(face_sign)
    alignment = float(np.clip(ray_local @ inward_local, -1.0, 1.0))
    minimum_alignment = cos(float(max_normal_misalignment_rad))
    denominator = float(ray_local[face_axis])
    points_contact = np.asarray(
        [cup.center_contact_frame_m for cup in cup_array.cups], dtype=float
    )
    points_world = (
        contact_rotation @ points_contact.T
    ).T + pose[:3, 3]
    points_box = (
        target.rotation.T @ (points_world - target.center).T
    ).T

    contacts: list[CupFaceContact] = []
    for cup, center_box in zip(cup_array.cups, points_box):
        common: dict[str, Any] = {
            "index": cup.index,
            "cup_id": cup.cup_id,
            "target_id": target.name,
            "target_face": target_face,
            "normal_alignment": alignment,
        }
        if denominator * (-float(face_sign)) <= 1e-12:
            contacts.append(
                CupFaceContact(
                    **common,
                    geometrically_eligible=False,
                    reason="CUP_RAY_NOT_DIRECTED_INTO_TARGET_FACE",
                    center_signed_gap_m=None,
                    ring_signed_gap_range_m=None,
                    face_contact_center_world_m=None,
                    projected_ring_half_extents_on_face_m=None,
                )
            )
            continue

        center_gap = float(
            (face_sign * target.half_extents[face_axis] - center_box[face_axis])
            / denominator
        )
        # Every point of the ring has a distinct ray origin when the tool plane
        # is slightly tilted.  This is the exact min/max gap over that circle.
        ring_gap_amplitude = cup.seal_radius_m * float(
            np.hypot(
                box_from_contact_rotation[face_axis, 0],
                box_from_contact_rotation[face_axis, 1],
            )
        ) / abs(denominator)
        gap_range = (
            center_gap - ring_gap_amplitude,
            center_gap + ring_gap_amplitude,
        )
        contact_center_box = center_box + center_gap * ray_local

        projected_extents: list[float] = []
        ring_inside = True
        for tangent_axis in tangent_axes:
            # Project every cup-ring point along the cup ray onto the named
            # target plane, then take the analytic support radius on this axis.
            projected_u = (
                box_from_contact_rotation[tangent_axis, 0]
                - box_from_contact_rotation[face_axis, 0]
                * ray_local[tangent_axis]
                / denominator
            )
            projected_v = (
                box_from_contact_rotation[tangent_axis, 1]
                - box_from_contact_rotation[face_axis, 1]
                * ray_local[tangent_axis]
                / denominator
            )
            extent = cup.seal_radius_m * float(np.hypot(projected_u, projected_v))
            projected_extents.append(extent)
            if (
                abs(float(contact_center_box[tangent_axis]))
                + extent
                + suction_edge_margin_m
                > float(target.half_extents[tangent_axis]) + _COMPARISON_EPSILON_M
            ):
                ring_inside = False

        contact_center_world = target.to_world(contact_center_box)
        if alignment < minimum_alignment:
            reason = "CUP_NORMAL_MISALIGNED"
        elif gap_range[0] < -maximum_penetration_m - _COMPARISON_EPSILON_M:
            reason = "CUP_RING_PENETRATES_TARGET"
        elif gap_range[1] > max_attachment_gap_m + _COMPARISON_EPSILON_M:
            reason = "CUP_RING_GAP_EXCEEDS_LIMIT"
        elif not ring_inside:
            reason = "CUP_SEAL_RING_NOT_FULLY_ON_TARGET_FACE"
        else:
            reason = None
        contacts.append(
            CupFaceContact(
                **common,
                geometrically_eligible=reason is None,
                reason=reason,
                center_signed_gap_m=center_gap,
                ring_signed_gap_range_m=(float(gap_range[0]), float(gap_range[1])),
                face_contact_center_world_m=tuple(float(v) for v in contact_center_world),
                projected_ring_half_extents_on_face_m=tuple(projected_extents),
            )
        )

    eligible = tuple(contact.geometrically_eligible for contact in contacts)
    if len(eligible) != M710_CUP_COUNT:
        raise AssertionError("independent-cup evaluation did not preserve 72 mask bits")
    return IndependentCupGeometry(
        target_id=target.name,
        target_face=target_face,
        physical_contact_pose_world=pose,
        cups=cup_array.cups,
        contacts=tuple(contacts),
        geometrically_eligible_mask=eligible,
        max_attachment_gap_m=float(max_attachment_gap_m),
        maximum_penetration_m=float(maximum_penetration_m),
        max_normal_misalignment_rad=float(max_normal_misalignment_rad),
        suction_edge_margin_m=float(suction_edge_margin_m),
    )


@dataclass(frozen=True)
class IdealIndependentCupSelection:
    """A non-empty deterministic cup command and its actual geometric contacts."""

    geometry: IndependentCupGeometry
    commanded_active_mask: tuple[bool, ...]
    actual_contact_mask: tuple[bool, ...]
    pose_source: str
    actual_q_rad: tuple[float, ...] | None = None
    actual_virtual_task_tcp_pose_world: np.ndarray | None = None

    def __post_init__(self) -> None:
        for name, mask in (
            ("commanded_active_mask", self.commanded_active_mask),
            ("actual_contact_mask", self.actual_contact_mask),
        ):
            if len(mask) != M710_CUP_COUNT or any(type(value) is not bool for value in mask):
                raise ValueError(f"{name} must contain 72 booleans")
        if not any(self.commanded_active_mask):
            raise ValueError("commanded_active_mask must be non-empty")
        if any(
            actual and not commanded
            for commanded, actual in zip(
                self.commanded_active_mask, self.actual_contact_mask
            )
        ):
            raise ValueError("actual_contact_mask must be a subset of commanded_active_mask")
        if not self.pose_source:
            raise ValueError("pose_source must be non-empty")

    @property
    def geometrically_eligible_mask(self) -> tuple[bool, ...]:
        return self.geometry.geometrically_eligible_mask

    @property
    def commanded_active_ids(self) -> tuple[str, ...]:
        return tuple(
            cup.cup_id
            for cup, active in zip(self.geometry.cups, self.commanded_active_mask)
            if active
        )

    @property
    def commanded_active_indices(self) -> tuple[int, ...]:
        return tuple(
            cup.index
            for cup, active in zip(self.geometry.cups, self.commanded_active_mask)
            if active
        )

    @property
    def actual_contact_ids(self) -> tuple[str, ...]:
        return tuple(
            cup.cup_id
            for cup, contact in zip(self.geometry.cups, self.actual_contact_mask)
            if contact
        )

    @property
    def actual_contact_indices(self) -> tuple[int, ...]:
        return tuple(
            cup.index
            for cup, contact in zip(self.geometry.cups, self.actual_contact_mask)
            if contact
        )

    @property
    def actual_contacts_per_zone(self) -> tuple[int, ...]:
        counts = [0] * (max(cup.zone for cup in self.geometry.cups) + 1)
        for cup, contact in zip(self.geometry.cups, self.actual_contact_mask):
            if contact:
                counts[cup.zone] += 1
        return tuple(counts)

    def to_dict(self) -> dict[str, Any]:
        return {
            "suction_mode": IDEAL_INDEPENDENT_CUPS_MODE,
            "holding_capacity_assumption": HOLDING_CAPACITY_ASSUMPTION,
            "enforce_vacuum_force_capacity": False,
            "enforce_vacuum_break_force": False,
            "enforce_vacuum_break_torque": False,
            "load_bearing_minimum_cup_count": None,
            "command_nonempty_required": True,
            "pose_source": self.pose_source,
            "target_id": self.geometry.target_id,
            "target_face": self.geometry.target_face,
            "mask_bit_order_cup_ids": [cup.cup_id for cup in self.geometry.cups],
            "geometrically_eligible_mask": list(self.geometrically_eligible_mask),
            "commanded_active_mask": list(self.commanded_active_mask),
            "actual_contact_mask": list(self.actual_contact_mask),
            "geometrically_eligible_ids": list(
                self.geometry.geometrically_eligible_ids
            ),
            "commanded_active_ids": list(self.commanded_active_ids),
            "commanded_active_indices": list(self.commanded_active_indices),
            "actual_contact_ids": list(self.actual_contact_ids),
            "actual_contact_indices": list(self.actual_contact_indices),
            "actual_contacts_per_zone": list(self.actual_contacts_per_zone),
            "commanded_active_count": sum(self.commanded_active_mask),
            "actual_contact_count": sum(self.actual_contact_mask),
            "actual_q_rad": (
                None if self.actual_q_rad is None else list(self.actual_q_rad)
            ),
            "actual_virtual_task_tcp_pose_world": (
                None
                if self.actual_virtual_task_tcp_pose_world is None
                else self.actual_virtual_task_tcp_pose_world.tolist()
            ),
            "actual_physical_contact_pose_world": (
                self.geometry.physical_contact_pose_world.tolist()
            ),
            "cups": [cup.to_dict() for cup in self.geometry.cups],
            "contacts": [contact.to_dict() for contact in self.geometry.contacts],
            "rigid_collision_semantics": (
                "NOT_EVALUATED_HERE_CALLER_MUST_RETAIN_ALL_EXISTING_RIGID_COLLISION_CHECKS"
            ),
        }


def select_ideal_independent_cups(
    geometry: IndependentCupGeometry,
    *,
    commanded_cup_ids: Sequence[str] | None = None,
    pose_source: str = "provided_actual_physical_contact_pose",
    actual_q_rad: Sequence[float] | None = None,
    actual_virtual_task_tcp_pose_world: Sequence[Sequence[float]] | None = None,
) -> IdealIndependentCupSelection:
    """Activate all eligible cups, or a caller-selected eligible non-empty subset.

    There is deliberately no load-bearing cup-count threshold.  A one-cup
    command is geometrically valid in this ideal mode; holding capacity is an
    explicit assumption outside this geometry result.
    """

    ids = geometry.cups
    by_id = {cup.cup_id: cup.index for cup in ids}
    if len(by_id) != M710_CUP_COUNT:
        raise ValueError("cup IDs must be unique and preserve all 72 mask positions")
    if commanded_cup_ids is None:
        commanded = geometry.geometrically_eligible_mask
    else:
        requested = tuple(str(cup_id) for cup_id in commanded_cup_ids)
        if len(requested) != len(set(requested)):
            raise ValueError("commanded cup IDs must be unique")
        unknown = sorted(set(requested) - set(by_id))
        if unknown:
            raise ValueError(f"unknown commanded cup IDs: {unknown}")
        commanded_list = [False] * M710_CUP_COUNT
        for cup_id in requested:
            index = by_id[cup_id]
            if not geometry.geometrically_eligible_mask[index]:
                raise ValueError(f"commanded cup is not geometrically eligible: {cup_id}")
            commanded_list[index] = True
        commanded = tuple(commanded_list)
    if len(commanded) != M710_CUP_COUNT or not any(commanded):
        raise ValueError(
            "ideal_independent_cups requires a non-empty geometrically eligible command"
        )
    if any(active and not eligible for active, eligible in zip(
        commanded, geometry.geometrically_eligible_mask
    )):
        raise ValueError("commanded_active_mask must be a subset of eligible cups")

    # In this CPU model, an actual contact means an enabled cup whose complete
    # ring passed the geometry test at the supplied actual pose.  Inactive cups
    # remain physical geometry but provide no attachment action.
    actual_contact = tuple(
        active and eligible
        for active, eligible in zip(commanded, geometry.geometrically_eligible_mask)
    )
    q_tuple = None
    if actual_q_rad is not None:
        q = np.asarray(actual_q_rad, dtype=float)
        if q.ndim != 1 or len(q) == 0 or not np.all(np.isfinite(q)):
            raise ValueError("actual_q_rad must be a non-empty finite vector")
        q_tuple = tuple(float(value) for value in q)
    virtual_pose = None
    if actual_virtual_task_tcp_pose_world is not None:
        virtual_pose = _finite_se3(
            actual_virtual_task_tcp_pose_world,
            "actual_virtual_task_tcp_pose_world",
        )
    return IdealIndependentCupSelection(
        geometry=geometry,
        commanded_active_mask=tuple(commanded),
        actual_contact_mask=actual_contact,
        pose_source=str(pose_source),
        actual_q_rad=q_tuple,
        actual_virtual_task_tcp_pose_world=virtual_pose,
    )


def audit_actual_independent_cup_contacts(
    geometry: IndependentCupGeometry,
    commanded_cup_ids: Sequence[str],
    *,
    pose_source: str = "actual_execution_physical_contact_pose",
) -> IdealIndependentCupSelection:
    """Audit a previously issued command at a new actual physical pose.

    Unlike planning-time selection, this function does not pretend that every
    commanded cup still touches.  It preserves the 72-bit command verbatim and
    reports ``actual_contact_mask = commanded_active_mask & current_geometry``.
    An empty actual-contact mask is valid diagnostic evidence, but a caller must
    reject attachment because no ideal contact exists.
    """

    by_id = {cup.cup_id: cup.index for cup in geometry.cups}
    requested = tuple(str(cup_id) for cup_id in commanded_cup_ids)
    if not requested:
        raise ValueError("commanded_active_mask must be non-empty")
    if len(requested) != len(set(requested)):
        raise ValueError("commanded cup IDs must be unique")
    unknown = sorted(set(requested) - set(by_id))
    if unknown:
        raise ValueError(f"unknown commanded cup IDs: {unknown}")
    commanded = [False] * M710_CUP_COUNT
    for cup_id in requested:
        commanded[by_id[cup_id]] = True
    actual = tuple(
        active and eligible
        for active, eligible in zip(commanded, geometry.geometrically_eligible_mask)
    )
    return IdealIndependentCupSelection(
        geometry=geometry,
        commanded_active_mask=tuple(commanded),
        actual_contact_mask=actual,
        pose_source=str(pose_source),
    )


def select_ideal_independent_cups_from_actual_fk(
    robot: Any,
    actual_q_rad: Sequence[float],
    target: OBB,
    target_face: str,
    cup_array: IndependentCupArray,
    flange_from_virtual_task_tcp: Sequence[Sequence[float]],
    flange_from_physical_contact: Sequence[Sequence[float]],
    *,
    commanded_cup_ids: Sequence[str] | None = None,
    max_attachment_gap_m: float = 0.002,
    maximum_penetration_m: float = 0.0002,
    max_normal_misalignment_rad: float = np.deg2rad(5.0),
    suction_edge_margin_m: float = 0.0,
) -> IdealIndependentCupSelection:
    """Recompute cup masks from the successful solution's actual FK pose.

    ``robot.fk(actual_q_rad)`` is treated as the virtual task TCP.  The physical
    compressed cup plane is recovered with the explicit full-SE(3) frame
    relation; no tool length is added a second time.
    """

    q = np.asarray(actual_q_rad, dtype=float)
    if q.ndim != 1 or len(q) == 0 or not np.all(np.isfinite(q)):
        raise ValueError("actual_q_rad must be a non-empty finite vector")
    actual_virtual = _finite_se3(robot.fk(q), "robot.fk(actual_q_rad)")
    flange_from_virtual = _finite_se3(
        flange_from_virtual_task_tcp, "flange_from_virtual_task_tcp"
    )
    flange_from_contact = _finite_se3(
        flange_from_physical_contact, "flange_from_physical_contact"
    )
    actual_physical_contact = (
        actual_virtual @ np.linalg.inv(flange_from_virtual) @ flange_from_contact
    )
    geometry = evaluate_independent_cup_geometry(
        actual_physical_contact,
        target,
        target_face,
        cup_array,
        max_attachment_gap_m=max_attachment_gap_m,
        maximum_penetration_m=maximum_penetration_m,
        max_normal_misalignment_rad=max_normal_misalignment_rad,
        suction_edge_margin_m=suction_edge_margin_m,
    )
    return select_ideal_independent_cups(
        geometry,
        commanded_cup_ids=commanded_cup_ids,
        pose_source="actual_fk_virtual_tcp_converted_to_physical_contact",
        actual_q_rad=q,
        actual_virtual_task_tcp_pose_world=actual_virtual,
    )


def m710_cup_array_from_mapping(value: Mapping[str, Any]) -> IndependentCupArray:
    """Read confirmed geometry fields without accepting a hidden cup threshold."""

    forbidden = {
        "minimum_sealed_cups",
        "minimum_active_cups",
        "minimum_contact_cups",
    } & set(value)
    if forbidden:
        raise ValueError(
            "ideal_independent_cups does not accept a load-bearing cup-count threshold: "
            + ", ".join(sorted(forbidden))
        )
    return build_m710_independent_cup_array(
        rows=value.get("cup_rows", value.get("rows", M710_CUP_ROWS)),
        columns=value.get("cup_columns", value.get("columns", M710_CUP_COLUMNS)),
        pitch_m=value.get("cup_pitch_m", value.get("pitch_m", M710_CUP_PITCH_M)),
        cup_radius_m=value.get(
            "cup_radius_m", value.get("seal_radius_m", M710_CUP_RADIUS_M)
        ),
        zone_count=value.get("zone_count", M710_CUP_ZONE_COUNT),
    )


__all__ = [
    "HOLDING_CAPACITY_ASSUMPTION",
    "IDEAL_INDEPENDENT_CUPS_MODE",
    "M710_CUP_COLUMNS",
    "M710_CUP_COUNT",
    "M710_CUP_PITCH_M",
    "M710_CUP_RADIUS_M",
    "M710_CUP_ROWS",
    "M710_CUP_ZONE_COUNT",
    "CupFaceContact",
    "IdealIndependentCupSelection",
    "IndependentCup",
    "IndependentCupArray",
    "IndependentCupGeometry",
    "build_m710_independent_cup_array",
    "audit_actual_independent_cup_contacts",
    "evaluate_independent_cup_geometry",
    "m710_cup_array_from_mapping",
    "select_ideal_independent_cups",
    "select_ideal_independent_cups_from_actual_fk",
]
