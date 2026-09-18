"""Bounded complete-cycle connector for the frozen M-710 unloading layout.

This module owns search order and stage semantics, but not a collision
approximation.  A caller must supply a validator backed by the execution
collision model (official per-link meshes plus the rigid-tool compound).  The
separation is deliberate: a proxy robot can still be used for a kinematic
audit, but can never be promoted to an executable trajectory by this class.
"""

from __future__ import annotations

from .pair_clearance import obb_pair_failure, obb_surface_distance

from contextlib import contextmanager, nullcontext
from copy import deepcopy
from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path
from time import perf_counter
from typing import Any, Callable, Mapping, Protocol, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from .planning_profile import deadline_after, optional_seconds
from .depalletizing import minimum_clearance_extraction_distance
from .collision_policy import SimulationCollisionPolicy, PhysicsCheckedStackTracker
from .conveyor_placement import (
    ConveyorSupport,
    PlacementPolicy,
    effective_process_and_receiver,
    generate_conveyor_placements,
    support_union_audit,
    placement_working_normal, PLACEMENT_SEMANTICS,
)
from .release_motion import (MOTION_SEMANTICS, ReleasePolicy, predict_release,
                             SUPPORTED_RELEASE, SHORT_DROP_RELEASE, IDEAL_RECEPTION_RELEASE, departure_sweep, verify_release_prediction,
                             receiver_transport_support, receiver_footprint_reserve)
from .geometry import OBB, rotation_matrix_from_rotation_vector, rotation_vector_from_matrix
from .ik import iter_ik_solutions, pose_error, solve_ik_multistart
from .release_motion import release_flight_envelope
from .motion_quality import path_quality, QualityDeadline, quality_improves
from .planner import RRTConnectPlanner
from .pinocchio_backend import PinocchioHppFclBackend
from .validation_physics import (
    InitialProximityTracker,
    RigidAttachment,
    contact_separated,
    support_audit,
)


class LayoutTrajectoryRobot(Protocol):
    dof: int
    joint_limits: np.ndarray
    base_transform: np.ndarray

    def fk(self, q: np.ndarray) -> np.ndarray: ...

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-9) -> bool: ...

    def clamp(self, q: np.ndarray) -> np.ndarray: ...

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray: ...

    def named_link_frames(self, q: np.ndarray) -> Mapping[str, np.ndarray]: ...

    def tool_collision_obbs(self, q: np.ndarray) -> list[OBB]: ...


RobotStateValidator = Callable[..., Mapping[str, Any] | None]


def possible_inflated_obb_pairs(first: Sequence[OBB], second: Sequence[OBB],
                                margin: float) -> list[tuple[int, int]]:
    """Conservative broad phase preserving per-body local-axis inflation."""
    if not first or not second:
        return []
    def arrays(boxes):
        centers = np.asarray([box.center for box in boxes])
        extents = np.asarray([np.abs(box.rotation) @ (box.half_extents + margin)
                              for box in boxes])
        return centers, extents
    centers_a, extents_a = arrays(first)
    centers_b, extents_b = arrays(second)
    overlap = np.all(np.abs(centers_a[:, None, :] - centers_b[None, :, :]) <=
                     extents_a[:, None, :] + extents_b[None, :, :] + 1e-10, axis=2)
    return [tuple(map(int, pair)) for pair in np.argwhere(overlap)]


TRAJECTORY_SEGMENT_SCHEMA = "m710id70_layout_complete_trajectory_segment_v1"
TRAJECTORY_STAGES = (
    "home",
    "pregrasp",
    "contact",
    "support-release",
    "extraction",
    "transit",
    "place",
    "withdrawal",
)
OFFICIAL_MODEL_MANIFEST = Path(
    "assets/robots/fanuc_m710id_70/official/provenance.yaml"
)
OFFICIAL_MODEL_URDF = Path(
    "assets/robots/fanuc_m710id_70/official/"
    "fanuc_m710_description/urdf/m710id_70_official.urdf"
)
OFFICIAL_MODEL_SRDF = Path(
    "assets/robots/fanuc_m710id_70/official/m710id_70_official.srdf"
)
_OFFICIAL_DISABLED_SELF_COLLISION_PAIRS = (
    ("base_link", "J1_link"),
    ("J1_link", "J2_link"),
    ("J2_link", "J3_link"),
    ("J3_link", "J4_link"),
    ("J4_link", "J5_link"),
    ("J5_link", "J6_link"),
)


def validate_layout_trajectory_stage_contract(
    segment: Mapping[str, Any],
    *, historical_intent: bool = False,
) -> Mapping[str, Any]:
    """Reject an incomplete or internally inconsistent replay segment."""

    if segment.get("schema") != TRAJECTORY_SEGMENT_SCHEMA:
        raise ValueError("unsupported layout trajectory segment schema")
    path = np.asarray(segment.get("path"), dtype=float)
    if path.ndim != 2 or path.shape[1:] != (6,) or len(path) < 2:
        raise ValueError("complete trajectory path must contain at least two 6-DOF states")
    if not np.all(np.isfinite(path)):
        raise ValueError("complete trajectory path must be finite")
    ranges = segment.get("stage_ranges")
    adaptive = segment.get("motion_semantics") == MOTION_SEMANTICS
    required = {"home", "contact", "extraction", "transit", "place", "withdrawal"}
    if not isinstance(ranges, Mapping) or (not adaptive and set(ranges) != set(TRAJECTORY_STAGES)) or (adaptive and (not required <= set(ranges) or not set(ranges) <= set(TRAJECTORY_STAGES))):
        raise ValueError("complete trajectory must contain exactly the eight required stages")
    previous_end = 0
    for stage in TRAJECTORY_STAGES:
        if stage not in ranges:
            continue
        interval = ranges[stage]
        if (
            not isinstance(interval, (list, tuple))
            or len(interval) != 2
            or any(isinstance(item, bool) or not isinstance(item, int) for item in interval)
        ):
            raise ValueError(f"stage {stage} range must contain two integer indices")
        begin, end = interval
        if begin != previous_end or not begin <= end < len(path):
            raise ValueError(f"stage {stage} is not contiguous and in range")
        previous_end = end
    if list(ranges["home"]) != [0, 0] or previous_end != len(path) - 1:
        raise ValueError("stage ranges must cover the full path from home through withdrawal")
    expected_indices = {
        "grasp_index": int(ranges["contact"][1]),
        "release_index": int(ranges["place"][1]),
        "release_retreat_index": int(ranges["withdrawal"][1]),
    }
    for name, expected in expected_indices.items():
        if segment.get(name) != expected:
            raise ValueError(f"{name} does not match its stage endpoint")
    expected_events = [
        {"index": expected_indices["grasp_index"], "event": "ATTACH"},
        {"index": expected_indices["release_index"], "event": "RELEASE"},
        {
            "index": expected_indices["release_retreat_index"],
            "event": "RELEASE_RETREAT_COMPLETE",
        },
    ]
    if segment.get("events") != expected_events:
        raise ValueError("trajectory events must exactly match the three ordered stage endpoints")
    contact = segment.get("contact")
    if not isinstance(contact, Mapping):
        raise ValueError("trajectory contact evidence is missing")
    for name in (
        "requested_virtual_task_tcp_pose_world",
        "requested_physical_contact_pose_world",
        "actual_virtual_task_tcp_pose_world",
        "actual_physical_contact_pose_world",
    ):
        LayoutTrajectoryConnector._se3(np.asarray(contact.get(name)), f"contact.{name}")
    actual_q = np.asarray(contact.get("actual_q_rad"), dtype=float)
    if actual_q.shape != (6,) or not np.allclose(
        actual_q, path[expected_indices["grasp_index"]], atol=1e-12, rtol=0.0
    ):
        raise ValueError("contact actual_q_rad must equal the grasp path state")
    selection = contact.get("cup_selection")
    actual_contact_count = (
        selection.get("actual_contact_count") if isinstance(selection, Mapping) else None
    )
    if (
        not isinstance(selection, Mapping)
        or selection.get("suction_mode") != "ideal_independent_cups"
        or selection.get("load_bearing_minimum_cup_count") is not None
        or isinstance(actual_contact_count, bool)
        or not isinstance(actual_contact_count, (int, np.integer))
        or int(actual_contact_count) <= 0
    ):
        raise ValueError("trajectory attachment requires a non-empty ideal cup selection")
    mask_names = (
        "geometrically_eligible_mask",
        "commanded_active_mask",
        "actual_contact_mask",
    )
    masks: dict[str, tuple[bool, ...]] = {}
    for name in mask_names:
        raw_mask = selection.get(name)
        if (
            not isinstance(raw_mask, list)
            or len(raw_mask) != 72
            or any(type(value) is not bool for value in raw_mask)
        ):
            raise ValueError(f"contact cup_selection.{name} must contain 72 booleans")
        masks[name] = tuple(raw_mask)
    if masks["actual_contact_mask"] != masks["commanded_active_mask"] or any(
        command_bit and not eligible_bit
        for eligible_bit, command_bit in zip(
            masks["geometrically_eligible_mask"], masks["commanded_active_mask"]
        )
    ):
        raise ValueError(
            "contact cup selection counts, target, or enforcement policy disagree: "
            "every commanded cup must retain complete-ring actual contact"
        )
    if (
        sum(masks["actual_contact_mask"]) != int(actual_contact_count)
        or selection.get("commanded_active_count")
        != sum(masks["commanded_active_mask"])
        or selection.get("target_id") != segment.get("target")
        or selection.get("target_face") != segment.get("face")
        or selection.get("enforce_vacuum_force_capacity") is not False
        or selection.get("enforce_vacuum_break_force") is not False
        or selection.get("enforce_vacuum_break_torque") is not False
    ):
        raise ValueError("contact cup selection counts, target, or enforcement policy disagree")
    selection_q = np.asarray(selection.get("actual_q_rad"), dtype=float)
    if selection_q.shape != (6,) or not np.allclose(
        selection_q, actual_q, atol=1e-12, rtol=0.0
    ):
        raise ValueError("contact cup selection actual_q_rad disagrees with contact evidence")
    for selection_name, contact_name in (
        (
            "actual_virtual_task_tcp_pose_world",
            "actual_virtual_task_tcp_pose_world",
        ),
        (
            "actual_physical_contact_pose_world",
            "actual_physical_contact_pose_world",
        ),
    ):
        selection_pose = LayoutTrajectoryConnector._se3(
            np.asarray(selection.get(selection_name)),
            f"contact.cup_selection.{selection_name}",
        )
        contact_pose = np.asarray(contact[contact_name], dtype=float)
        if not np.allclose(selection_pose, contact_pose, atol=1e-12, rtol=0.0):
            raise ValueError(
                f"contact cup selection {selection_name} disagrees with contact evidence"
            )
    place = segment.get("place")
    support = place.get("support") if isinstance(place, Mapping) else None
    drop = isinstance(place, Mapping) and place.get("release_mode") in {SHORT_DROP_RELEASE, IDEAL_RECEPTION_RELEASE}
    prediction = place.get("release_prediction", {}) if isinstance(place, Mapping) else {}
    if drop and (not adaptive or prediction.get("accepted") is not True
                 or prediction.get("mode") != place.get("release_mode")
                 or prediction.get("actual_landing_state") is not None):
        raise ValueError("short drop requires qualified prediction, never fabricated actual landing")
    if not isinstance(support, Mapping) or (not drop and support.get("supported") is not True):
        raise ValueError("trajectory place endpoint must pass the actual-pose support audit")
    actual_box_pose = LayoutTrajectoryConnector._se3(
        np.asarray(place.get("actual_box_pose_world")),
        "place.actual_box_pose_world",
    )
    if drop and not historical_intent:
        verify_release_prediction(place, str(segment["target"]))
    if adaptive and segment.get("placement_semantics") != PLACEMENT_SEMANTICS:
        raise ValueError("adaptive trajectory requires current placement semantics")
    release_center = np.asarray(place.get("release_center_world_m"), dtype=float)
    if release_center.shape != (3,) or not np.allclose(
        release_center, actual_box_pose[:3, 3], atol=1e-12, rtol=0.0
    ):
        raise ValueError("place release center must equal the actual box pose translation")
    if place.get("place_surface") != place.get("receiver"):
        raise ValueError("place surface and receiver must identify the same support")
    validation = segment.get("validation")
    if not isinstance(validation, Mapping) or validation.get("execution_qualified") is not True:
        raise ValueError("trajectory validation must remain execution-qualified")
    return segment


@dataclass(frozen=True)
class LayoutTrajectoryBudget:
    """Deterministic search limits; none of these values relax geometry."""

    proof_of_concept: bool = False
    candidate_wall_time_s: float | None = 30.0
    stage_wall_time_s: float = 12.0
    postprocess_wall_time_s: float = 3.0
    grasp_branches: int = 4
    task_pose_connection_attempts: int = 12  # Compatibility name: batch size, not pool truncation.
    stage_ik_candidates: int = 3
    stage_connection_attempts: int = 3
    stage_connection_iterations: int = 600
    rrt_step_rad: float = 0.20
    rrt_goal_bias: float = 0.20
    edge_resolution_rad: float = 0.04
    cartesian_step_m: float = 0.015
    cartesian_orientation_step_rad: float = np.deg2rad(3.0)
    cartesian_max_branch_step_rad: float = 0.45
    cartesian_max_samples_per_stage: int = 80
    approach_mode: str = "auto"
    pregrasp_standoff_m: float = 0.10  # historical candidate only
    preplace_standoff_m: float = 0.10
    withdrawal_distance_m: float = 0.0  # optional historical comparison
    post_release_vertical_lift_m: float = 0.0
    maximum_drop_m: float = 0.05
    ideal_release_min_height_m: float = 0.020
    ideal_release_max_height_m: float = 0.050
    ideal_release_height_reserve_m: float = 0.005
    approach_runtime_clearance_reserve_m: float = 0.0
    receiver_runtime_clearance_reserve_m: float = 0.0
    departure_runtime_clearance_reserve_m: float = 0.0
    receiver_edge_reserve_m: float = 0.01
    extraction_scan_step_m: float = 0.01
    maximum_extraction_m: float = 0.80
    extraction_direction_attempts: int = 5
    extraction_runtime_clearance_reserve_m: float = 0.0
    local_transit_cartesian_sample_budget: int = 240
    local_transit_outward_step_m: float = 0.03
    local_transit_outward_attempts: int = 3
    planning_wall_time_s: float | None = None

    def __post_init__(self) -> None:
        for name in ("approach_runtime_clearance_reserve_m", "receiver_runtime_clearance_reserve_m",
                     "departure_runtime_clearance_reserve_m"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.proof_of_concept:
            self.release_policy().ideal_heights()
        integer_names = (
            "grasp_branches",
            "task_pose_connection_attempts",
            "stage_ik_candidates",
            "stage_connection_attempts",
            "stage_connection_iterations",
            "extraction_direction_attempts",
            "cartesian_max_samples_per_stage",
            "local_transit_outward_attempts",
        )
        for name in integer_names:
            value = getattr(self, name)
            if isinstance(value, bool) or int(value) != value or int(value) <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if (isinstance(self.local_transit_cartesian_sample_budget, bool)
                or int(self.local_transit_cartesian_sample_budget) != self.local_transit_cartesian_sample_budget
                or self.local_transit_cartesian_sample_budget < 0):
            raise ValueError("local transit sample budget must be a nonnegative integer")
        for name in ("candidate_wall_time_s", "stage_wall_time_s", "postprocess_wall_time_s", "planning_wall_time_s"):
            optional_seconds(getattr(self, name), name)
        positive_names = (

            "rrt_step_rad",
            "edge_resolution_rad",
            "cartesian_step_m",
            "cartesian_orientation_step_rad",
            "cartesian_max_branch_step_rad",
            "pregrasp_standoff_m",
            "preplace_standoff_m",
            "extraction_scan_step_m",
            "maximum_extraction_m",
            "local_transit_outward_step_m",
        )
        for name in positive_names:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.approach_mode not in {"auto", "direct", "adaptive_pregrasp"}:
            raise ValueError("unsupported approach mode")
        for name in ("withdrawal_distance_m", "post_release_vertical_lift_m", "maximum_drop_m", "receiver_edge_reserve_m"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not 0.0 <= float(self.rrt_goal_bias) <= 1.0:
            raise ValueError("rrt_goal_bias must be in [0, 1]")
        if (
            not np.isfinite(self.extraction_runtime_clearance_reserve_m)
            or self.extraction_runtime_clearance_reserve_m < 0.0
        ):
            raise ValueError("extraction_runtime_clearance_reserve_m must be finite and non-negative")
        if (self.planning_wall_time_s is not None
                and (not np.isfinite(self.planning_wall_time_s)
                     or self.planning_wall_time_s < 0.0)):
            raise ValueError("planning_wall_time_s must be finite and positive when configured")


    @property
    def task_pose_batch_size(self):
        return self.task_pose_connection_attempts

    def release_policy(self):
        return ReleasePolicy(maximum_drop_m=self.maximum_drop_m,
            ideal_release_min_height_m=self.ideal_release_min_height_m,
            ideal_release_max_height_m=self.ideal_release_max_height_m,
            ideal_release_height_reserve_m=self.ideal_release_height_reserve_m)

    def execution_reserves(self):
        return {name: getattr(self, name) for name in (
            "approach_runtime_clearance_reserve_m", "receiver_runtime_clearance_reserve_m",
            "departure_runtime_clearance_reserve_m", "extraction_runtime_clearance_reserve_m")}

    @property
    def task_complete_connection_attempt_limit(self):
        return None if self.proof_of_concept else 3 * self.task_pose_connection_attempts


@dataclass(frozen=True)
class PhysicalContactAttachment:
    """A rigid box attachment captured at the actual physical cup plane."""

    robot: LayoutTrajectoryRobot
    rigid: RigidAttachment
    flange_from_virtual_task_tcp: np.ndarray
    flange_from_physical_contact: np.ndarray

    def physical_contact_pose(self, q: Sequence[float]) -> np.ndarray:
        virtual = self.robot.fk(np.asarray(q, dtype=float))
        return (
            virtual
            @ np.linalg.inv(self.flange_from_virtual_task_tcp)
            @ self.flange_from_physical_contact
        )

    def box_at(self, q: Sequence[float]) -> OBB:
        return self.rigid.box_at(self.physical_contact_pose(q))


@dataclass(frozen=True)
class LayoutTrajectorySearchResult:
    success: bool
    segment: Mapping[str, Any] | None
    failure: Mapping[str, Any] | None
    attempts: tuple[Mapping[str, Any], ...]
    statistics: Mapping[str, Any]


@dataclass(frozen=True)
class LayoutTrajectoryConnectorBuildResult:
    """Fail-closed optional-backend construction result."""

    connector: "LayoutTrajectoryConnector | None"
    status: str
    failure_reason: str | None
    evidence: Mapping[str, Any]


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repository_root_from(path: Path) -> Path | None:
    """Find the repository that owns one fixed official-model input."""

    for candidate in (path.parent, *path.parents):
        if (candidate / "pyproject.toml").is_file() and (
            candidate / "src" / "unloading_sim"
        ).is_dir():
            return candidate.resolve()
    return None


def _audit_official_srdf_policy(path: Path) -> dict[str, Any]:
    """Accept only the fixed adjacent-link exclusions used by this connector.

    The SRDF is a project policy rather than an upstream FANUC source asset, so
    its byte hash is recorded while its full collision semantics are checked
    here. Adding a non-adjacent waiver cannot silently create a new qualified
    backend identity.
    """

    root = ET.parse(path).getroot()
    if root.tag != "robot" or root.attrib != {"name": "fanuc_m710id_70"}:
        raise ValueError("official SRDF robot identity is not exact")
    children = list(root)
    groups = [child for child in children if child.tag == "group"]
    disabled = [child for child in children if child.tag == "disable_collisions"]
    if len(groups) != 1 or len(disabled) != len(_OFFICIAL_DISABLED_SELF_COLLISION_PAIRS):
        raise ValueError("official SRDF must contain one group and six adjacent exclusions")
    if len(children) != len(groups) + len(disabled):
        raise ValueError("official SRDF contains an unsupported policy element")
    group = groups[0]
    if group.attrib != {"name": "manipulator"} or len(group) != 1:
        raise ValueError("official SRDF manipulator group is not exact")
    chain = group[0]
    if (
        chain.tag != "chain"
        or chain.attrib
        != {"base_link": "base_link", "tip_link": "tool0"}
        or len(chain)
    ):
        raise ValueError("official SRDF chain must remain base_link through tool0")
    actual_pairs: list[tuple[str, str]] = []
    for entry in disabled:
        if (
            set(entry.attrib) != {"link1", "link2", "reason"}
            or entry.attrib["reason"] != "Adjacent"
            or len(entry)
        ):
            raise ValueError("official SRDF collision exclusion is malformed")
        actual_pairs.append((entry.attrib["link1"], entry.attrib["link2"]))
    if tuple(actual_pairs) != _OFFICIAL_DISABLED_SELF_COLLISION_PAIRS:
        raise ValueError("official SRDF collision exclusions are not the fixed adjacent pairs")
    return {
        "sha256": _sha256_file(path),
        "group": {
            "name": "manipulator",
            "base_link": "base_link",
            "tip_link": "tool0",
        },
        "disabled_self_collision_pairs": [list(pair) for pair in actual_pairs],
        "non_adjacent_exclusions_allowed": False,
    }


class ExactM710LayoutStateValidator:
    """Compose official robot meshes with the source-audited rigid-tool compound.

    The official robot backend owns robot self/environment and robot/payload
    tests.  The lightweight URDF instance is used only to transform the
    CAD-derived rigid-solid bounds that are already expressed in the virtual
    task-TCP frame.  It does not substitute its capsule collision model.
    """

    def __init__(
        self,
        mesh_robot: PinocchioHppFclBackend,
        tool_transform_robot: LayoutTrajectoryRobot,
        *,
        collision_margin_m: float,
        floor_z_m: float,
        right_wall_y_m: float,
        left_wall_y_m: float,
        robot_world_boxes: Callable[[np.ndarray], Sequence[OBB]],
        base_support_obstacle_name: str = "chassis",
        tool_mount_link_name: str = "J6_link",
        collision_policy: Mapping[str, Any] | None = None,
        nominal_cup_compression_m: float = 0.0,
    ) -> None:
        self.mesh_robot = mesh_robot
        self.tool_transform_robot = tool_transform_robot
        self.collision_margin_m = float(collision_margin_m)
        self.floor_z_m = float(floor_z_m)
        self.right_wall_y_m = float(right_wall_y_m)
        self.left_wall_y_m = float(left_wall_y_m)
        self.robot_world_boxes = robot_world_boxes
        self.base_support_obstacle_name = str(base_support_obstacle_name)
        self.tool_mount_link_name = str(tool_mount_link_name)
        self.collision_policy = SimulationCollisionPolicy.from_mapping(collision_policy)
        self.nominal_cup_compression_m = float(nominal_cup_compression_m)
        if not 0.0 <= self.nominal_cup_compression_m <= 0.015:
            raise ValueError("nominal cup compression outside CAD travel")
        scalars = np.asarray(
            [
                self.collision_margin_m,
                self.floor_z_m,
                self.right_wall_y_m,
                self.left_wall_y_m,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(scalars)) or self.collision_margin_m < 0.0:
            raise ValueError("exact validator bounds and margin must be finite")
        if not self.right_wall_y_m < self.left_wall_y_m:
            raise ValueError("trailer side-wall bounds must be ordered")
        if not self.base_support_obstacle_name or not self.tool_mount_link_name:
            raise ValueError("fixed-contact pair names must be non-empty")
        zero = np.zeros(int(mesh_robot.dof), dtype=float)
        tool_boxes = list(tool_transform_robot.tool_all_physical_obbs(zero))
        self.required_tool_names = frozenset(box.name for box in tool_boxes)
        if not tool_boxes or len(self.required_tool_names) != len(tool_boxes):
            raise ValueError(
                "execution validator requires a nonempty uniquely named audited tool compound"
            )
        self._geometry_cache = {}
        self._static_cache = {}
        self._geometry_signature = None
        self.performance_counters = {}
        self.commanded_cup_mask = None
        self.stack_carton_names = set()
        self.contact_target_name = None

    def _profile_call(self, name, function, *args, **kwargs):
        started = perf_counter()
        try:
            return function(*args, **kwargs)
        finally:
            self.performance_counters[name + "_seconds"] = self.performance_counters.get(name + "_seconds", 0.) + perf_counter()-started
            self.performance_counters[name + "_calls"] = self.performance_counters.get(name + "_calls", 0) + 1

    @staticmethod
    def _collision_failure(result, reason: str) -> Mapping[str, Any] | None:
        if not result.in_collision:
            return None
        return {
            "reason": reason,
            "collision_kind": str(result.reason),
            **(getattr(result, "evidence", None) or {}),
            "pair": [result.first_link, result.first_obstacle],
        }

    def _mesh_pair_options(self, stage, proxy=False):
        return ({"margin": 0., "pair_policy": self.collision_policy, "stage": stage, "proxy": proxy}
                if self.collision_policy.poc_pair_clearance else {"margin": self.collision_margin_m})

    def _compliant_boxes(self, q):
        # Source bounds are uncompressed. Match Isaac's 72 nominally
        # compressed bellows; the independent rigid inserts remain unchanged.
        result = []
        for box in self.tool_transform_robot.tool_compliant_collision_obbs(q):
            half = box.half_extents.copy()
            half[2] -= self.nominal_cup_compression_m / 2
            result.append(OBB(box.center - box.rotation[:, 2] * self.nominal_cup_compression_m / 2,
                              half, box.rotation, box.name, box.category))
        return result

    def _plane_failure(
        self, q: np.ndarray, payload: OBB | None, stage: str
    ) -> Mapping[str, Any] | None:
        robot_bounds_provider = getattr(self.mesh_robot, "collision_world_axis_extrema", None)
        robot_bounds = robot_bounds_provider(q) if robot_bounds_provider is not None else {}
        robot_boxes = list(self.robot_world_boxes(q))
        body_bounds = [(box.name, *(
            (np.asarray(robot_bounds[box.name]["lower_m"]), np.asarray(robot_bounds[box.name]["upper_m"]))
            if box.name in robot_bounds else (box.corners().min(axis=0), box.corners().max(axis=0))))
            for box in robot_boxes]
        cached = self._geometry_cache.get(np.asarray(q, float).tobytes())
        bodies = ([*cached[0], *cached[1]] if cached is not None else
                  [*self.tool_transform_robot.tool_collision_obbs(q), *self._compliant_boxes(q)])
        if payload is not None:
            bodies.append(payload)
        for body in bodies:
            corners = body.corners()
            body_bounds.append((body.name, corners.min(axis=0), corners.max(axis=0)))
        for name, lower, upper in body_bounds:
            y_min, y_max, z_min = float(lower[1]), float(upper[1]), float(lower[2])
            if not self.collision_policy.poc_pair_clearance and (
                y_min < self.right_wall_y_m + self.collision_margin_m
                or y_max > self.left_wall_y_m - self.collision_margin_m
            ):
                return {
                    "reason": "TRAILER_SIDE_CLEARANCE",
                    "body": name,
                    "y_bounds_m": [y_min, y_max],
                }
            floor_margin = (self.collision_policy.pair_clearance("external", self.collision_margin_m)
                            if self.collision_policy.poc_pair_clearance else self.collision_margin_m)
            if payload is not None and name == payload.name and self.collision_policy.allows_stack_planning_contact(stage):
                floor_margin = -0.0002
            if z_min < self.floor_z_m + floor_margin:
                return {
                    "reason": "FLOOR_CLEARANCE",
                    "body": name,
                    "minimum_z_m": z_min,
                }
        return None

    def __call__(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        *,
        payload: OBB | None,
        target_contact: OBB | None,
        stage: str,
    ) -> Mapping[str, Any] | None:
        q = np.asarray(q, dtype=float)
        names = [box.name for box in obstacles]
        if len(names) != len(set(names)):
            return {"reason": "DUPLICATE_OBSTACLE_NAME"}
        environment = list(obstacles)
        if payload is not None and payload.name not in set(names):
            environment.append(payload)
        signature = (getattr(self.mesh_robot, "geometry_revision", 0),
            np.asarray(getattr(self.tool_transform_robot, "tool_collision_local_boxes", [])).tobytes(),
            np.asarray(getattr(self.tool_transform_robot, "tool_compliant_collision_local_boxes", [])).tobytes(),
            np.asarray(getattr(self.tool_transform_robot, "base_transform", [])).tobytes(),
            self.nominal_cup_compression_m)
        if signature != self._geometry_signature:
            self._geometry_cache.clear(); self._static_cache.clear()
            self._geometry_signature = signature
        q_key = q.tobytes()
        if q_key not in self._geometry_cache:
            if len(self._geometry_cache) >= 4096:
                self._geometry_cache.clear()
            self._geometry_cache[q_key] = (
                list(self.tool_transform_robot.tool_collision_obbs(q)), self._compliant_boxes(q))
        rigid_boxes, compliant_boxes = self._geometry_cache[q_key]
        tool_boxes = [*rigid_boxes, *compliant_boxes]
        compliant_names = {box.name for box in compliant_boxes}
        compliant_indices = {box.name: index for index, box in enumerate(compliant_boxes)}
        if frozenset(box.name for box in tool_boxes) != self.required_tool_names:
            return {
                "reason": "RIGID_TOOL_COMPOUND_INCOMPLETE",
                "actual_solid_count": len(tool_boxes),
                "required_solid_count": len(self.required_tool_names),
            }
        fixed = [box for box in environment if box.category not in {"carton", "payload"}]
        dynamic = [box for box in environment if box.category in {"carton", "payload"}]
        static_key = (q_key, repr(self.collision_policy), self.collision_margin_m,
                      self.floor_z_m, self.right_wall_y_m, self.left_wall_y_m,
                      getattr(self.mesh_robot, "geometry_revision", 0), tuple((box.name, box.world_from_local.tobytes(), box.half_extents.tobytes()) for box in fixed))
        if static_key not in self._static_cache:
            if len(self._static_cache) >= 4096:
                self._static_cache.clear()
            result = self._profile_call("robot_fixed_including_self", self.mesh_robot.collision_result, q, fixed, **self._mesh_pair_options(stage),
                ignored_geometry_obstacle_pairs={("base_link", self.base_support_obstacle_name)}, check_self=True)
            failure = self._collision_failure(result, "ROBOT_MESH_COLLISION")
            if failure is None:
                result = self._profile_call("robot_tool", self.mesh_robot.collision_result, q, tool_boxes, **self._mesh_pair_options(stage, proxy=True),
                    ignored_geometry_obstacle_pairs=self.collision_policy.wrist_tool_pairs([box.name for box in tool_boxes]),
                    check_self=False)
                failure = self._collision_failure(result, "ROBOT_RIGID_TOOL_COLLISION")
            if failure is None:
                for i, j in self._profile_call("tool_fixed_broadphase", possible_inflated_obb_pairs, tool_boxes, fixed, self.collision_margin_m):
                    failure = obb_pair_failure(tool_boxes[i], fixed[j], self.collision_policy,
                        self.collision_margin_m, stage=stage, proxy=True, reason="RIGID_TOOL_COLLISION")
                    if failure is not None:
                        break
            if failure is None:
                failure = self._profile_call("plane_bounds", self._plane_failure, q, None, stage)
            self._static_cache[static_key] = failure
        else:
            self.performance_counters["static_cache_hits"] = self.performance_counters.get("static_cache_hits", 0) + 1
        failure = self._static_cache[static_key]
        if failure is not None:
            return failure
        result = self._profile_call("robot_dynamic", self.mesh_robot.collision_result, q, dynamic, **self._mesh_pair_options(stage), check_self=False)
        failure = self._collision_failure(result, "ROBOT_MESH_COLLISION")
        if failure is not None:
            return failure
        # Conservative broad phase around the *locally inflated* OBBs.  It
        # only skips disjoint world AABBs; the unchanged SAT remains final.
        for tool_index, obstacle_index in self._profile_call("tool_dynamic_broadphase", possible_inflated_obb_pairs,
            tool_boxes, dynamic, self.collision_margin_m
        ):
            tool, obstacle = tool_boxes[tool_index], dynamic[obstacle_index]
            if tool.name in compliant_names:
                target_name = (payload.name if payload is not None else
                               target_contact.name if target_contact is not None else self.contact_target_name)
                if (self.collision_policy.compliant_cup_neighbor_contact_mode == "ignore"
                        and target_name is not None and obstacle.name != target_name
                        and obstacle.name in self.stack_carton_names):
                    continue
                current_target = (payload is not None and obstacle.name == payload.name) or (
                    target_contact is not None and obstacle.name == target_contact.name
                    and stage in {"contact", "contact_endpoint", "next_contact", "withdrawal"})
                cup_index = compliant_indices[tool.name]
                inactive = (self.commanded_cup_mask is not None
                            and not self.commanded_cup_mask[cup_index])
                stack_contact = (inactive and obstacle.name in self.stack_carton_names
                    and self.collision_policy.inactive_compliant_cup_stack_contact_mode == "physical_contact_within_compression"
                    and (stage in {"contact", "contact_endpoint", "next_contact"}
                         or payload is not None and self.collision_policy.allows_stack_planning_contact(stage)))
                if (current_target or stack_contact) and tool.signed_distance_obb(obstacle) >= (
                    -self.collision_policy.maximum_compliant_cup_additional_compression_m):
                    continue
            failure = obb_pair_failure(tool, obstacle, self.collision_policy, self.collision_margin_m,
                                       stage=stage, proxy=True, reason="RIGID_TOOL_COLLISION")
            if failure is not None:
                return failure

        if payload is not None:
            lower, upper = payload.corners().min(axis=0), payload.corners().max(axis=0)
            floor_margin = (-0.0002 if self.collision_policy.allows_stack_planning_contact(stage) else
                self.collision_policy.pair_clearance("external", self.collision_margin_m) if self.collision_policy.poc_pair_clearance else self.collision_margin_m)
            if lower[2] < self.floor_z_m + floor_margin:
                return {"reason": "FLOOR_CLEARANCE", "body": payload.name}
            if not self.collision_policy.poc_pair_clearance and (lower[1] < self.right_wall_y_m + self.collision_margin_m or upper[1] > self.left_wall_y_m - self.collision_margin_m):
                return {"reason": "TRAILER_SIDE_CLEARANCE", "body": payload.name}
        return None


class LayoutTrajectoryConnector:
    """Connect one strict contact candidate through a complete pick/place cycle.

    ``robot_state_validator`` must validate official robot meshes, the complete
    rigid tool and robot/payload pairs.  Payload/environment contact is also
    checked here so support and initial-proximity semantics remain explicit.
    """

    def __init__(
        self,
        robot: LayoutTrajectoryRobot,
        robot_state_validator: RobotStateValidator,
        *,
        flange_from_virtual_task_tcp: np.ndarray,
        flange_from_physical_contact: np.ndarray,
        ik_policy: Mapping[str, Any],
        collision_margin_m: float,
        contact_tolerance_m: float,
        joint_margin_rad: float,
        maximum_jacobian_condition: float,
        validator_identity: str,
        execution_qualified: bool,
        budget: LayoutTrajectoryBudget | None = None,
        official_radial_reach_m: float | None = None,
        radial_guard_tolerance_m: float = 0.0,
        collision_policy: Mapping[str, Any] | None = None,
        surface_directions_world: Mapping[str, Sequence[float]] | None = None,
        tool_collision_obbs_provider: Callable[[np.ndarray], Sequence[OBB]] | None = None,
        post_landing_transport: Mapping[str, Any] | None = None,
    ) -> None:
        self.robot = robot
        from .post_landing_transport import transport_policy
        self.post_landing_transport = transport_policy(post_landing_transport)
        self.collision_policy = SimulationCollisionPolicy.from_mapping(collision_policy)
        self.robot_state_validator = robot_state_validator
        self.flange_from_virtual_task_tcp = self._se3(
            flange_from_virtual_task_tcp, "flange_from_virtual_task_tcp"
        )
        self.flange_from_physical_contact = self._se3(
            flange_from_physical_contact, "flange_from_physical_contact"
        )
        self.ik = dict(ik_policy)
        self.collision_margin_m = self._nonnegative(
            collision_margin_m, "collision_margin_m"
        )
        self.contact_tolerance_m = self._nonnegative(
            contact_tolerance_m, "contact_tolerance_m"
        )
        self.joint_margin_rad = self._nonnegative(
            joint_margin_rad, "joint_margin_rad"
        )
        self.maximum_jacobian_condition = float(maximum_jacobian_condition)
        if not np.isfinite(self.maximum_jacobian_condition) or self.maximum_jacobian_condition <= 0:
            raise ValueError("maximum_jacobian_condition must be finite and positive")
        if not validator_identity.strip():
            raise ValueError("validator_identity must be non-empty")
        self.validator_identity = str(validator_identity)
        self.execution_qualified = bool(execution_qualified)
        if not self.execution_qualified:
            raise ValueError("complete trajectory connector requires execution-qualified collision validation")
        if int(robot.dof) != 6 or np.asarray(robot.joint_limits).shape != (6, 2):
            raise ValueError("M-710 complete trajectory connector requires six scalar joints")
        self.official_radial_reach_m = (
            None
            if official_radial_reach_m is None
            else self._nonnegative(official_radial_reach_m, "official_radial_reach_m")
        )
        self.radial_guard_tolerance_m = self._nonnegative(
            radial_guard_tolerance_m, "radial_guard_tolerance_m"
        )
        self.surface_directions_world: dict[str, tuple[float, float, float]] = {}
        for name, raw_direction in dict(surface_directions_world or {}).items():
            direction = np.asarray(raw_direction, dtype=float)
            if direction.shape != (3,) or not np.all(np.isfinite(direction)):
                raise ValueError(f"surface direction for {name!r} must have three finite values")
            norm = float(np.linalg.norm(direction))
            if norm <= 1e-12:
                raise ValueError(f"surface direction for {name!r} must be non-zero")
            self.surface_directions_world[str(name)] = tuple((direction / norm).tolist())
        self.tool_collision_obbs_provider = (
            tool_collision_obbs_provider
            or getattr(self.robot, "tool_collision_obbs", lambda _q: ())
        )
        self.budget = budget or LayoutTrajectoryBudget()
        self.placement_policy = PlacementPolicy(maximum_candidates=12)
        self._statistics = {
            "state_validations": 0,
            "edge_validation_calls": 0,
            "edge_state_samples": 0,
            "ik_calls": 0,
            "ik_seeds_attempted": 0,
            "ik_iterations_consumed": 0,
            "connection_attempts": 0,
            "rrt_iterations_consumed": 0,
            "cartesian_samples": 0,
            "placement_candidates_generated": 0,
            "placement_candidate_generation_wall_seconds": 0.0,
            "state_cache_hits": 0,
            "state_cache_misses": 0,
            "trajectory_ik_wall_seconds": 0.0,
            "path_connection_wall_seconds_inclusive": 0.0,
            "collision_validation_wall_seconds_nested": 0.0,
            "kinematics_and_joint_checks_wall_seconds": 0.0,
            "final_recheck_wall_seconds": 0.0,
        }
        self._state_cache: dict[tuple[Any, ...], Mapping[str, Any] | None] = {}
        self._state_cache_limit = 4096
        self._deadline_monotonic: float | None = None
        self.next_contact_provider = None
        self.stack_carton_names = None
        self._request_generation = 0
        self._completed_tasks = {}
        self._budget_events = []

    def start_planning_request(self, start_monotonic: float | None = None) -> None:
        """Reset bounded reuse and bind every nested search to one deadline."""
        start = perf_counter() if start_monotonic is None else float(start_monotonic)
        self._request_deadline_monotonic = (None if self.budget.planning_wall_time_s is None
                                    else start + self.budget.planning_wall_time_s)
        self._final_export_reserve_s = (0. if self.budget.planning_wall_time_s is None else
                                       min(15., .05*self.budget.planning_wall_time_s))
        self._deadline_monotonic = (None if self._request_deadline_monotonic is None else
                                    self._request_deadline_monotonic-self._final_export_reserve_s)
        self._state_cache.clear()
        self._lookahead_remaining_s = 12.0
        self._request_generation += 1
        self._completed_tasks.clear()
        self._budget_events.clear()
        self._search_deadline_monotonic = self._deadline_monotonic

    @staticmethod
    def _limit(*deadlines):
        finite = [value for value in deadlines if value is not None]
        return min(finite) if finite else None

    @contextmanager
    def _budget_scope(self, deadline, *, finalization=False):
        """Ordinary children only shorten time; finalization is bound separately.

        Only a fully constructed task may enter finalization. No search runs in
        that scope. It uses the original candidate/branch hard limit, never a
        newly issued reserve. Lookahead has no access to it.
        """
        outer = self._deadline_monotonic
        self._deadline_monotonic = (deadline if finalization else self._limit(outer, deadline))
        try:
            yield
        finally:
            self._deadline_monotonic = outer

    def _optional_deadline(self, *, comparison=False):
        # Bounded allowance for the necessary continuation, inside parent time.
        now = perf_counter()
        remaining = self._remaining_wall_time()
        reserve = 0. if self.budget.stage_wall_time_s is None else min(3., self.budget.stage_wall_time_s)
        return self._limit(self._deadline_monotonic,
            None if comparison else deadline_after(now, self.budget.postprocess_wall_time_s),
            None if remaining is None else now + max(0., remaining - reserve))

    def _context_identity(self, obstacles, *, attachment=None, support_names=(),
                          target_contact=None, stage):
        """Exact same-request identity for retained evidence, never a global cache."""
        def box(value):
            return None if value is None else (value.name, value.category,
                value.world_from_local.tolist(), value.half_extents.tolist())
        v = self.robot_state_validator
        values = (self._request_generation, getattr(self, "_candidate_identity", None),
            self.validator_identity, id(self.robot), id(getattr(v, "mesh_robot", None)),
            self.collision_policy.to_mapping(), self.budget.proof_of_concept, self.post_landing_transport,
            self.budget.execution_reserves(), self.budget.release_policy().to_mapping(),
            self.collision_margin_m, self.contact_tolerance_m, self.joint_margin_rad,
            self.maximum_jacobian_condition, self.official_radial_reach_m,
            self.radial_guard_tolerance_m, self.budget.edge_resolution_rad, dict(self.ik),
            np.asarray(getattr(self.robot, "base_transform", np.eye(4))).tolist(),
            np.asarray(self.robot.joint_limits).tolist(),
            self.flange_from_virtual_task_tcp.tolist(), self.flange_from_physical_contact.tolist(),
            getattr(getattr(v, "mesh_robot", None), "geometry_revision", 0),
            np.asarray(getattr(getattr(v, "tool_transform_robot", None), "tool_collision_local_boxes", [])).tolist(),
            np.asarray(getattr(getattr(v, "tool_transform_robot", None), "tool_compliant_collision_local_boxes", [])).tolist(),
            getattr(v, "nominal_cup_compression_m", None),
            None if getattr(v, "commanded_cup_mask", None) is None else list(v.commanded_cup_mask),
            getattr(v, "contact_target_name", None), sorted(getattr(v, "stack_carton_names", ())),
            sorted(self.stack_carton_names or ()), [box(b) for b in obstacles], box(target_contact),
            None if attachment is None else attachment.rigid.tcp_from_box.tolist(),
            list(support_names), stage)
        return hashlib.sha256(repr(values).encode()).hexdigest()

    def _remember_path(self, path, context, stage, level, quality=None):
        completed = perf_counter()
        snapshot = dict(path=tuple(tuple(float(x) for x in q) for q in path),
            context=context, stage=stage, validation_level=level,
            start_q_rad=np.asarray(path[0]).tolist(), end_q_rad=np.asarray(path[-1]).tolist(),
            stage_ranges={stage: [0, len(path)-1]},
            target_identity=getattr(self.robot_state_validator, "contact_target_name", None),
            completed_monotonic=completed, deadline_monotonic=self._deadline_monotonic,
            candidate_identity=getattr(self, "_candidate_identity", None),
            quality=deepcopy(quality))
        self._budget_events.append({key: value for key, value in snapshot.items()
                                   if key != "path"})
        return snapshot

    def _optional_quality(self, path, deadline):
        if deadline is not None and perf_counter() >= deadline:
            return None
        try:
            return self._path_quality(path, deadline=deadline)
        except QualityDeadline:
            return None

    @staticmethod
    def _task_digest(segment):
        content = deepcopy(segment)
        content.get("validation", {}).pop("completion", None)
        # Selection accounting is written after comparison; it grants no permission.
        content.get("place", {}).pop("release_selection", None)
        return hashlib.sha256(json.dumps(content, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def _finalize_task(self, segment, obstacles, target):
        """Finalize only after every required stage and envelope has succeeded."""
        started = perf_counter()
        try:
            return self._finalize_task_checked(segment, obstacles, target)
        finally:
            self._statistics["final_recheck_wall_seconds"] += perf_counter() - started

    def _finalize_task_checked(self, segment, obstacles, target):
        deadline = getattr(self, "_branch_final_deadline", None)
        if deadline is None:
            deadline = self._deadline_monotonic
        deadline = self._limit(deadline, getattr(self, "_request_deadline_monotonic", None))
        completion = dict(validation_level="A_UNVERIFIED_GEOMETRY", completed_monotonic=None,
            deadline_monotonic=deadline, final_validation_completed=False, execution_ready=False)
        with self._budget_scope(deadline, finalization=True):
            if self._deadline_reached():
                return {"reason": "FINAL_VALIDATION_DEADLINE", "stage": "final_validation",
                        "completion": completion}
            if segment.get("target") != target.name:
                return {"reason": "FINAL_VALIDATION_TARGET_MISMATCH", "stage": "final_validation"}
            # All original incremental FK/contact/edge/envelope gates precede
            # this call. This original final contract gate must still execute.
            validate_layout_trajectory_stage_contract(segment)
            identity = self._context_identity(obstacles, target_contact=target, stage="complete_task")
            digest = self._task_digest(segment)
            completed = perf_counter()
            completion["completed_monotonic"] = completed
            if deadline is not None and completed >= deadline:
                return {"reason": "FINAL_VALIDATION_DEADLINE", "stage": "final_validation",
                        "completion": completion}
            completion.update(validation_level="D_COMPLETE_TASK", final_validation_completed=True,
                context=identity, content_sha256=digest, request_generation=self._request_generation)
            segment["validation"]["completion"] = deepcopy(completion)
            self._completed_tasks[digest] = deepcopy(completion)
            self._budget_events.append(deepcopy(completion))
        return None

    def _task_completed(self, segment, obstacles, target):
        if segment is None:
            return False
        digest = self._task_digest(segment)
        proof = self._completed_tasks.get(digest)
        return bool(proof is not None and proof == segment.get("validation", {}).get("completion")
            and proof["request_generation"] == self._request_generation
            and proof["context"] == self._context_identity(obstacles,
                target_contact=target, stage="complete_task"))

    def _remaining_wall_time(self) -> float | None:
        if self._deadline_monotonic is None:
            return None
        return max(0.0, self._deadline_monotonic - perf_counter())

    def _deadline_reached(self) -> bool:
        remaining = self._remaining_wall_time()
        return remaining is not None and remaining <= 0.0

    @staticmethod
    def _se3(value: np.ndarray, name: str) -> np.ndarray:
        result = np.asarray(value, dtype=float)
        if result.shape != (4, 4) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be a finite 4x4 transform")
        if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0):
            raise ValueError(f"{name} has an invalid homogeneous bottom row")
        rotation = result[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0.0) \
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0.0):
            raise ValueError(f"{name} must contain a proper rotation")
        return result.copy()

    @staticmethod
    def _nonnegative(value: float, name: str) -> float:
        result = float(value)
        if not np.isfinite(result) or result < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return result

    def physical_from_virtual(self, virtual_pose: np.ndarray) -> np.ndarray:
        return (
            self._se3(virtual_pose, "virtual_pose")
            @ np.linalg.inv(self.flange_from_virtual_task_tcp)
            @ self.flange_from_physical_contact
        )

    def budget_evidence(self) -> dict[str, Any]:
        return {
            "task_pose_connection_attempts": self.budget.task_pose_connection_attempts,
            "task_pose_batch_size": self.budget.task_pose_batch_size,
            "task_complete_connection_attempt_limit": self.budget.task_complete_connection_attempt_limit,
            "candidate_schedule": "UNVISITED_FAMILY_VARIANTS_THEN_BOUNDED_RETRIES",
            "stage_wall_time_s": self.budget.stage_wall_time_s,
            "postprocess_wall_time_s": self.budget.postprocess_wall_time_s,
            "final_export_reserve_s": getattr(self, "_final_export_reserve_s", 0.),
            "final_reserve_scope": "COMPLETE_TASK_VALIDATION_AND_IN_PROCESS_BINDING_ONLY",
            "independent_preflight_export_in_request_budget": False,
            "optional_continuation_reserve_s": 0. if self.budget.stage_wall_time_s is None else min(3., self.budget.stage_wall_time_s),
            "deadline_boundary": "completion < deadline; new work requires now < deadline",
            "task_pose_scope": "shared_by_one_task_across_faces_rolls_and_task_set_variants",
            "grasp_branches_per_pose": self.budget.grasp_branches,
            "stage_ik_candidates": self.budget.stage_ik_candidates,
            "stage_connection_attempts": self.budget.stage_connection_attempts,
            "stage_connection_iterations": self.budget.stage_connection_iterations,
            "stage_connection_scope": "shared_across_lazy_ik_candidates_for_one_stage_endpoint",
            "cartesian_policy": "one_continuation_seed_per_bounded_cartesian_sample",
            "cartesian_max_samples_per_stage": self.budget.cartesian_max_samples_per_stage,
            "extraction_direction_attempts": self.budget.extraction_direction_attempts,
            "local_transit_cartesian_sample_budget": self.budget.local_transit_cartesian_sample_budget,
            "local_transit_budget_scope": "shared_by_all_placement_candidates_of_one_grasp_prefix",
            "local_transit_outward_step_m": self.budget.local_transit_outward_step_m,
            "local_transit_outward_attempts": self.budget.local_transit_outward_attempts,
            "maximum_extraction_m": self.budget.maximum_extraction_m,
            "planning_execution_reserves": self.budget.execution_reserves(),
            "release_policy": self.budget.release_policy().to_mapping(),
            "extraction_runtime_clearance_reserve_m": (
                self.budget.extraction_runtime_clearance_reserve_m
            ),
            "withdrawal_distance_m": self.budget.withdrawal_distance_m,
            "post_release_vertical_lift_m": self.budget.post_release_vertical_lift_m,
            "post_release_conveyor_escape_clearance_m": (
                self.collision_policy.pair_clearance("external", self.collision_margin_m)
                + self.budget.departure_runtime_clearance_reserve_m + self.contact_tolerance_m
                + 2 * float(self.ik["position_tolerance_m"])
            ),
            "planning_wall_time_s": self.budget.planning_wall_time_s,
            "wall_clock_scope": "one_shared_first_carton_planning_request",
        }

    def virtual_from_physical(self, physical_pose: np.ndarray) -> np.ndarray:
        return (
            self._se3(physical_pose, "physical_pose")
            @ np.linalg.inv(self.flange_from_physical_contact)
            @ self.flange_from_virtual_task_tcp
        )

    def validate_unloaded_state(
        self,
        q: Sequence[float],
        obstacles: Sequence[OBB],
        *,
        target_contact: OBB | None = None,
        stage: str = "unloaded",
    ) -> Mapping[str, Any] | None:
        """Expose the exact endpoint predicate used by later path edges."""

        return self._state_failure(
            q,
            obstacles,
            target_contact=target_contact,
            stage=stage,
        )

    def _diagnostic_event(self, item):
        diagnostics = getattr(self, "diagnostics", None)
        if diagnostics is not None:
            callback = getattr(self, "progress_callback", None)
            (callback or diagnostics.event)({"candidate": dict(diagnostics.candidate), **item})

    def _state_failure(self, q, obstacles, *, diagnostic_origin="state", diagnostic_edge=None, **kwargs):
        failure = self._checked_state_failure(q, obstacles, **kwargs)
        diagnostics = getattr(self, "diagnostics", None)
        if diagnostics is not None:
            diagnostics.observe(kwargs["stage"], diagnostic_origin, failure is not None)
        if failure is not None and diagnostics is not None:
            def context():
                def box(b):
                    return None if b is None else dict(name=b.name, category=b.category,
                        pose_world=b.world_from_local.tolist(), half_extents_m=b.half_extents.tolist())
                attachment = kwargs.get("attachment")
                v = self.robot_state_validator
                return dict(stage=kwargs["stage"], obstacles=[box(b) for b in obstacles],
                    target_contact=box(kwargs.get("target_contact")),
                    support_names=list(kwargs.get("support_names", ())),
                    attachment=None if attachment is None else dict(
                        tcp_from_box=attachment.rigid.tcp_from_box.tolist(),
                        payload=box(attachment.box_at(q))),
                    initial_proximity=(None if kwargs.get("initial_proximity") is None else
                        kwargs["initial_proximity"].evidence()),
                    commanded_cup_mask=getattr(v, "commanded_cup_mask", None),
                    contact_target_name=getattr(v, "contact_target_name", None),
                    stack_carton_names=sorted(getattr(v, "stack_carton_names", ())),
                    collision_policy=self.collision_policy.to_mapping(),
                    validator_identity=self.validator_identity,
                    joint_limits=self.robot.joint_limits, joint_margin_rad=self.joint_margin_rad,
                    maximum_jacobian_condition=self.maximum_jacobian_condition,
                    path_seed=getattr(self, "diagnostic_path_seed", None),
                    ik_seed=getattr(self, "diagnostic_ik_seed", None),
                    cartesian_sample_index=getattr(self, "diagnostic_cartesian_sample", None),
                    ik_seed_kind=("PREVIOUS_Q_NO_RANDOM_RESTARTS" if
                        getattr(self, "diagnostic_cartesian_sample", None) is not None else "LAZY_IK_STREAM"),
                    branch_seed=getattr(self, "diagnostic_branch_seed", None))
            diagnostics.reject(failure, np.asarray(q), origin=diagnostic_origin,
                               edge=diagnostic_edge, context=context)
        return failure

    def _checked_state_failure(
        self,
        q: Sequence[float],
        obstacles: Sequence[OBB],
        *,
        attachment: PhysicalContactAttachment | None = None,
        support_names: Sequence[str] = (),
        target_contact: OBB | None = None,
        initial_proximity: InitialProximityTracker | None = None,
        stage: str,
    ) -> Mapping[str, Any] | None:
        if self._deadline_reached():
            return {"reason": "PLANNING_WALL_CLOCK_DEADLINE", "stage": stage}
        q_array = np.asarray(q, dtype=float)
        cacheable = initial_proximity is None
        cache_key = None
        if cacheable and q_array.shape == (6,) and np.all(np.isfinite(q_array)):
            commanded_mask = getattr(self.robot_state_validator, "commanded_cup_mask", None)
            attachment_key = () if attachment is None else tuple(
                attachment.rigid.tcp_from_box.flatten()
            )
            cache_key = (
                stage, q_array.tobytes(), attachment_key,
                repr(self.collision_policy), self.collision_margin_m, self.contact_tolerance_m,
                tuple(self.budget.execution_reserves().items()),
                self.joint_margin_rad, self.maximum_jacobian_condition, self.official_radial_reach_m,
                self.radial_guard_tolerance_m,
                getattr(self.robot_state_validator, "nominal_cup_compression_m", None),
                None if commanded_mask is None else tuple(commanded_mask),
                getattr(self.robot_state_validator, "contact_target_name", None),
                tuple(sorted(getattr(self.robot_state_validator, "stack_carton_names", []))),
                self.validator_identity, np.asarray(self.robot.base_transform).tobytes(),
                getattr(getattr(self.robot_state_validator, "mesh_robot", None), "geometry_revision", 0),
                np.asarray(getattr(getattr(self.robot_state_validator, "tool_transform_robot", None), "tool_collision_local_boxes", [])).tobytes(),
                np.asarray(getattr(getattr(self.robot_state_validator, "tool_transform_robot", None), "tool_compliant_collision_local_boxes", [])).tobytes(),
                tuple((box.name, box.category, box.world_from_local.tobytes(), box.half_extents.tobytes()) for box in obstacles), tuple(support_names),
                None if target_contact is None else (target_contact.name, target_contact.world_from_local.tobytes(), target_contact.half_extents.tobytes()),
            )
            if cache_key in self._state_cache:
                self._statistics["state_cache_hits"] += 1
                cached = self._state_cache[cache_key]
                return None if cached is None else dict(cached)
            self._statistics["state_cache_misses"] += 1
        self._statistics["state_validations"] += 1
        validation_started = perf_counter()

        def finish(value: Mapping[str, Any] | None):
            self._statistics["collision_validation_wall_seconds_nested"] += (
                perf_counter() - validation_started
            )
            if value is None and self._deadline_reached():
                value = {"reason": "PLANNING_WALL_CLOCK_DEADLINE", "stage": stage,
                         "timeout_work": "necessary_validation"}
            if cache_key is not None and not (value and value.get("timeout_work")):
                if len(self._state_cache) >= self._state_cache_limit:
                    self._state_cache.pop(next(iter(self._state_cache)))
                self._state_cache[cache_key] = None if value is None else dict(value)
            return value

        if q_array.shape != (6,) or not np.all(np.isfinite(q_array)):
            return finish({"reason": "JOINT_VECTOR_INVALID", "stage": stage})
        if not self.robot.within_limits(q_array):
            return finish({"reason": "JOINT_LIMIT", "stage": stage})
        limit_margin = float(
            np.min(
                np.minimum(
                    q_array - np.asarray(self.robot.joint_limits)[:, 0],
                    np.asarray(self.robot.joint_limits)[:, 1] - q_array,
                )
            )
        )
        if limit_margin < self.joint_margin_rad:
            return finish({
                "reason": "JOINT_MARGIN",
                "stage": stage,
                "actual_margin_rad": limit_margin,
            })
        condition = float(np.linalg.cond(self.robot.geometric_jacobian(q_array)))
        if not np.isfinite(condition) or condition > self.maximum_jacobian_condition:
            return finish({
                "reason": "SINGULARITY",
                "stage": stage,
                "jacobian_condition": condition,
            })
        if self.official_radial_reach_m is not None:
            frames = self.robot.named_link_frames(q_array)
            if "flange" not in frames:
                return finish({"reason": "FLANGE_FRAME_MISSING", "stage": stage})
            flange = np.asarray(frames["flange"], dtype=float)[:3, 3]
            base = np.asarray(self.robot.base_transform, dtype=float)
            local_flange = base[:3, :3].T @ (flange - base[:3, 3])
            radial = float(np.linalg.norm(local_flange[:2]))
            limit = self.official_radial_reach_m + self.radial_guard_tolerance_m
            if radial > limit:
                return finish({
                    "reason": "RADIAL_REACH",
                    "stage": stage,
                    "radial_m": radial,
                    "limit_m": limit,
                })

        self._statistics["kinematics_and_joint_checks_wall_seconds"] += perf_counter()-validation_started
        payload = None if attachment is None else attachment.box_at(q_array)
        backend_failure = self.robot_state_validator(
            q_array,
            tuple(obstacles),
            payload=payload,
            target_contact=target_contact,
            stage=stage,
        )
        if backend_failure is not None:
            return finish({**dict(backend_failure), "stage": stage})

        # Scoped planning reserve, never a runtime tolerance or contact exemption.
        # Receiver payload gaps are checked at every production state/edge sample.
        # Legal cup/stack/support contact stages retain their existing permissions.
        reserve_pairs = []
        if payload is not None and stage == "transit":
            reserve_pairs = [(payload, b, self.budget.receiver_runtime_clearance_reserve_m)
                             for b in obstacles if b.category == "conveyor"]
        if payload is None and stage == "residence" and target_contact is not None:
            reserve_pairs = [(b, target_contact, self.budget.departure_runtime_clearance_reserve_m)
                             for b in self.tool_collision_obbs_provider(q_array)]
        for first, second, reserve in reserve_pairs:
            if reserve <= 0:
                continue
            distance = obb_surface_distance(first, second)
            required = self.collision_policy.pair_clearance("external", self.collision_margin_m) + reserve
            if distance + 1e-9 < required:
                return finish(dict(reason="PLANNING_EXECUTION_RESERVE", classification="PLANNING_RESERVE_INSUFFICIENT",
                    stage=stage, pair=[first.name, second.name], surface_distance_m=distance,
                    required_pair_clearance_m=required, execution_reserve_m=reserve,
                    runtime_pair_clearance_m=required-reserve))
        if payload is None:
            return finish(None)
        if initial_proximity is not None and (
            not isinstance(initial_proximity, PhysicsCheckedStackTracker)
            or self.collision_policy.stack_contact_mode == "planner_relaxed_physics_checked"
        ):
            proximity_failure = initial_proximity.state_failure(
                payload, list(obstacles), support_names
            )
            if proximity_failure is not None:
                return finish({**proximity_failure, "stage": stage})
            return finish(None)
        support_set = set(support_names)
        for obstacle in obstacles:
            if obstacle.name in support_set and contact_separated(
                payload, obstacle, self.contact_tolerance_m
            ):
                continue
            failure = obb_pair_failure(payload, obstacle, self.collision_policy, self.collision_margin_m,
                                       stage=stage, reason="PAYLOAD_COLLISION")
            if failure is not None:
                return finish(failure)
        return finish(None)

    def _path_failure(
        self,
        path: Sequence[Sequence[float]],
        obstacles: Sequence[OBB],
        *,
        attachment: PhysicalContactAttachment | None = None,
        support_names: Sequence[str] = (),
        target_contact: OBB | None = None,
        initial_proximity: InitialProximityTracker | None = None,
        stage: str,
        diagnostic_origin: str = "full_edge_recheck",
    ) -> Mapping[str, Any] | None:
        self._statistics["edge_validation_calls"] += 1
        arrays = [np.asarray(q, dtype=float) for q in path]
        if len(arrays) == 1:
            self._statistics["edge_state_samples"] += 1
            return self._state_failure(
                arrays[0],
                obstacles,
                attachment=attachment,
                support_names=support_names,
                target_contact=target_contact,
                initial_proximity=initial_proximity,
                stage=stage,
            )
        for edge, (start, goal) in enumerate(zip(arrays[:-1], arrays[1:])):
            samples = max(
                1,
                int(
                    np.ceil(
                        np.max(np.abs(goal - start))
                        / self.budget.edge_resolution_rad
                    )
                ),
            )
            if self.collision_policy.poc_pair_clearance:
                # <= 1.25 mm point-motion bound per final sample (4 m conservative
                # lever arm, all six joint increments). Never coarsen legacy checks.
                samples = max(samples, int(np.ceil(4.0*np.sum(np.abs(goal-start))/.0025)))
            for fraction in np.linspace(0.0, 1.0, 2 * samples + 1):
                self._statistics["edge_state_samples"] += 1
                q = start + float(fraction) * (goal - start)
                failure = self._state_failure(
                    q,
                    obstacles,
                    attachment=attachment,
                    support_names=support_names,
                    target_contact=target_contact,
                    initial_proximity=initial_proximity,
                    stage=stage, diagnostic_origin=diagnostic_origin,
                    diagnostic_edge=dict(start_q_rad=start.tolist(), end_q_rad=goal.tolist(),
                                         index=edge, fraction=float(fraction)),
                )
                if failure is not None:
                    return {
                        **dict(failure),
                        "edge": edge,
                        "fraction": float(fraction),
                        "q_rad": q.tolist(),
                    }
        return None

    def _transit(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: Sequence[OBB],
        *,
        seed: int,
        iteration_budget: int,
        attachment: PhysicalContactAttachment | None = None,
        support_names: Sequence[str] = (),
        target_contact: OBB | None = None,
        stage: str,
    ) -> tuple[list[np.ndarray], Mapping[str, Any] | None, Mapping[str, Any]]:
        if self._deadline_reached():
            return [], {"reason": "STAGE_CONNECTION_DEADLINE", "stage": stage}, {
                "stage": stage, "planning_iterations_consumed": 0,
                "validation_level": "A_UNVERIFIED_GEOMETRY", "search_started": False}
        self.diagnostic_path_seed = int(seed)
        self.diagnostic_cartesian_sample = None
        state = lambda q: self._state_failure(
            q,
            obstacles,
            attachment=attachment,
            support_names=support_names,
            target_contact=target_contact,
            stage=stage, diagnostic_origin="rrt_internal",
            diagnostic_edge=planner.sample_context,
        ) is None
        callback = getattr(self, "progress_callback", None)
        if callback is not None:
            callback({"event": "connection_started", "stage": stage,
                      "iteration_budget": int(iteration_budget), "seed": int(seed)})
        planner = RRTConnectPlanner(
            np.asarray(self.robot.joint_limits)[:, 0],
            np.asarray(self.robot.joint_limits)[:, 1],
            state,
            step_size=self.budget.rrt_step_rad,
            edge_resolution=0.5 * self.budget.edge_resolution_rad,
            max_iterations=int(iteration_budget),
            goal_bias=self.budget.rrt_goal_bias,
            rng=np.random.default_rng(seed),
            diagnostic_context=getattr(self, "diagnostics", None) is not None,
        )
        connection_started = perf_counter()
        result = planner.plan(
            np.asarray(start, dtype=float), np.asarray(goal, dtype=float),
            time_limit_seconds=self._limit(self._remaining_wall_time(), self.budget.stage_wall_time_s),
        )
        self._statistics["path_connection_wall_seconds_inclusive"] += (
            perf_counter() - connection_started
        )
        if callback is not None:
            callback({"event": "connection_finished", "stage": stage,
                      "success": bool(result.success), "iterations": int(result.iterations),
                      "termination": result.message,
                      "statistics": dict(self._statistics)})
        self._statistics["connection_attempts"] += 1
        self._statistics["rrt_iterations_consumed"] += int(result.iterations)
        evidence = {
            "stage": stage,
            "seed": int(seed),
            "iteration_budget": int(iteration_budget),
            "success": bool(result.success),
            "search_success": bool(result.success),
            **dict(result.search_evidence),
        }
        if not result.success:
            return [], {
                "reason": "STAGE_CONNECTION_DEADLINE" if "time limit" in result.message else "PATH_SEARCH_EXHAUSTED",
                "stage": stage,
                "detail": result.message,
                "iterations": int(result.iterations),
            }, evidence
        context = lambda: self._context_identity(obstacles, attachment=attachment,
            support_names=support_names, target_contact=target_contact, stage=stage)
        identity = context()
        evidence["validation_level"] = "A_UNVERIFIED_GEOMETRY"
        # The caller's edge grid differs from RRT's: both checks remain required.
        failure = self._path_failure(
            result.path,
            obstacles,
            attachment=attachment,
            support_names=support_names,
            target_contact=target_contact,
            stage=stage, diagnostic_origin="rrt_success_full_edge_recheck",
        )
        if failure is not None:
            evidence["success"] = False
            return [], failure, evidence
        if context() != identity:
            evidence["success"] = False
            return [], {"reason": "VALIDATION_CONTEXT_CHANGED", "stage": stage}, evidence
        baseline = self._remember_path(result.path, identity, stage, "B_STRICT_LOCAL_CONNECTION")
        evidence.update(validation_level=baseline["validation_level"],
            validation_completed_monotonic=baseline["completed_monotonic"],
            validation_context=identity, fallback_to_verified=False)
        if attachment is None and target_contact is None and stage == "pregrasp":
            path, simplify = self._improve_free_path(result.path, obstacles, planner=planner)
            evidence["simplification"] = simplify
            evidence["fallback_to_verified"] = not simplify.get("adopted", False)
            if simplify.get("adopted"):
                baseline = self._remember_path(path, identity, stage,
                    "B_STRICT_LOCAL_CONNECTION", simplify["after"])
        if context() != baseline["context"]:
            evidence["success"] = False
            return [], {"reason": "VALIDATION_CONTEXT_CHANGED", "stage": stage}, evidence
        return [np.array(q) for q in baseline["path"]], None, evidence

    def _ik_stream(
        self,
        pose: np.ndarray,
        seeds: Sequence[np.ndarray],
        obstacles: Sequence[OBB],
        *,
        seed: int,
        attachment: PhysicalContactAttachment | None,
        support_names: Sequence[str],
        target_contact: OBB | None,
        stage: str,
        contact_candidate: Mapping[str, Any] | None = None,
        endpoint_checks: list[dict[str, Any]] | None = None,
    ):
        self.diagnostic_ik_seed = int(seed)
        self.diagnostic_cartesian_sample = None
        # Only this named contact search shares the normal grasp endpoint.
        # Unknown stages (including other *_ik_endpoint names) stay strict.
        endpoint_stage = "contact_endpoint" if stage == "next_contact" else f"{stage}_ik_endpoint"
        if stage == "next_contact" and (target_contact is None or contact_candidate is None):
            raise ValueError("next_contact IK requires candidate target, face and suction context")

        def endpoint_valid(q):
            with self._contact_context() if stage == "next_contact" else nullcontext():
                cups = None
                failure = None
                if stage == "next_contact":
                    try:
                        cups = self._contact_selection(q, target_contact,
                            contact_candidate["face"], contact_candidate["suction"])
                    except ValueError as exc:
                        failure = {"reason": "NEXT_CONTACT_COVERAGE_FAILED", "detail": str(exc)}
                if failure is None:
                    failure = self._state_failure(q, obstacles, attachment=attachment,
                        support_names=support_names, target_contact=target_contact, stage=endpoint_stage)
                if endpoint_checks is not None:
                    endpoint_checks.append({"requested_stage": stage, "endpoint_stage": endpoint_stage,
                        "target": None if target_contact is None else target_contact.name,
                        "q_rad": np.asarray(q).tolist(),
                        "mask_source": "candidate_fk_target_face_suction" if cups is not None else None,
                        "geometrically_eligible_mask": None if cups is None else cups["geometrically_eligible_mask"],
                        "commanded_active_mask": None if cups is None else cups["commanded_active_mask"],
                        "failure": failure})
                return failure is None

        unique = list(
            {
                tuple(np.asarray(q, dtype=float)): np.asarray(q, dtype=float)
                for q in seeds
            }.values()
        )
        stream = iter_ik_solutions(
            self.robot,
            pose,
            unique,
            random_restarts=int(self.ik["random_restarts"]),
            rng=np.random.default_rng(seed),
            candidate_limit=self.budget.stage_ik_candidates,
            dedup_tolerance_rad=float(self.ik["candidate_dedup_tolerance_rad"]),
            dedup_tolerance_m=float(self.ik["candidate_dedup_tolerance_m"]),
            max_iterations=int(self.ik["max_iterations"]),
            damping=float(self.ik["damping"]),
            max_step=float(self.ik["max_step_rad"]),
            position_tolerance=float(self.ik["position_tolerance_m"]),
            orientation_tolerance=float(self.ik["orientation_tolerance_rad"]),
            orientation_weight=float(self.ik["orientation_weight"]),
            extra_state_valid=endpoint_valid,
            collision_check_stride=int(self.ik["max_iterations"]) + 1,
            deadline_monotonic=self._deadline_monotonic,
            deadline_provider=lambda: self._limit(self._deadline_monotonic,
                getattr(self, "_request_deadline_monotonic", None)),
        )
        return stream

    def _path_quality(self, path, *, deadline=None):
        return path_quality(path, fk=self.robot.fk, joint_limits=self.robot.joint_limits,
            velocity_limits=getattr(getattr(self.robot, "model", None), "velocityLimit", None),
            joint_names=getattr(self.robot, "active_joint_names", None),
            jacobian=self.robot.geometric_jacobian,
            deadline=self._limit(self._deadline_monotonic, deadline))

    def _improve_free_path(self, path, obstacles, *, planner=None):
        """Optional common optimizer; caller has strictly validated this free domain.

        Only pregrasp paths without attachment/contact permissions enter here.
        A separate working copy, both edge grids, complete scores and unchanged
        endpoints/context are required before replacing the retained baseline.
        """
        baseline = [np.asarray(q).copy() for q in path]
        started = perf_counter()
        if self.budget.proof_of_concept:
            return [np.asarray(q).copy() for q in path], {"adopted": False, "reason": "DEFERRED_UNTIL_BASELINE_DELIVERY"}
        deadline = self._optional_deadline()
        identity = self._context_identity(obstacles, stage="pregrasp")
        evidence = dict(started_monotonic=started, deadline_monotonic=deadline,
            domain="UNLOADED_FREE_CONNECTION_ONLY", before=None, after=None,
            adopted=False, skipped=False, reason="NECESSARY_CONTINUATION_RESERVE")
        if deadline is not None and started >= deadline:
            evidence["skipped"] = True
        else:
            if planner is None:
                planner = RRTConnectPlanner(np.asarray(self.robot.joint_limits)[:, 0],
                    np.asarray(self.robot.joint_limits)[:, 1],
                    lambda q: self._state_failure(q, obstacles, stage="pregrasp") is None,
                    edge_resolution=.5*self.budget.edge_resolution_rad,
                    rng=np.random.default_rng(0))  # Shortcut is deterministic; no search.
            with self._budget_scope(deadline):
                before = self._optional_quality(baseline, deadline)
                evidence["before"] = before
                if before is not None:
                    candidate, shortcut = planner.bounded_shortcut(
                        [q.copy() for q in baseline], deadline=deadline,
                        attempts=8, state_budget=300)
                    evidence.update(shortcut)
                    if (not candidate or not np.array_equal(candidate[0], baseline[0])
                            or not np.array_equal(candidate[-1], baseline[-1])):
                        failure = {"reason": "SHORTCUT_ENDPOINT_CHANGED", "stage": "pregrasp"}
                    else:
                        failure = self._path_failure(candidate, obstacles, stage="pregrasp")
                    after = None if failure else self._optional_quality(candidate, deadline)
                    evidence.update(after=after, recheck_failure=failure)
                    if (quality_improves(before, after) and identity ==
                            self._context_identity(obstacles, stage="pregrasp")):
                        baseline = [q.copy() for q in candidate]
                        evidence.update(adopted=True, reason="FULLY_RECHECKED_IMPROVEMENT")
                    else:
                        evidence["reason"] = "IMPROVEMENT_INCOMPLETE_REJECTED_OR_NOT_BETTER"
                else:
                    evidence["reason"] = "QUALITY_DEADLINE"
        evidence["finished_monotonic"] = perf_counter()
        return baseline, evidence

    def _connect_pose(
        self,
        pose: np.ndarray,
        seeds: Sequence[np.ndarray],
        start: np.ndarray,
        obstacles: Sequence[OBB],
        *,
        ik_seed: int,
        connection_seed: int,
        attachment: PhysicalContactAttachment | None = None,
        support_names: Sequence[str] = (),
        target_contact: OBB | None = None,
        stage: str,
    ) -> tuple[np.ndarray | None, list[np.ndarray], Mapping[str, Any] | None, Mapping[str, Any]]:
        stream = self._ik_stream(
            pose,
            seeds,
            obstacles,
            seed=ik_seed,
            attachment=attachment,
            support_names=support_names,
            target_contact=target_contact,
            stage=stage,
        )
        attempts: list[dict[str, Any]] = []
        remaining = (min(getattr(self, "_placement_remaining", self.budget.stage_connection_iterations),
                         getattr(self, "_placement_candidate_allowance", self.budget.stage_connection_iterations))
                     if stage == "transit" else self.budget.stage_connection_iterations)
        selected: np.ndarray | None = None
        selected_path: list[np.ndarray] = []
        failure: Mapping[str, Any] | None = None
        feasible = []
        improvement_deadline = None
        improvement_started = None
        initial_context = self._context_identity(obstacles, attachment=attachment,
            support_names=support_names, target_contact=target_contact, stage=stage)
        for index in range(self.budget.stage_connection_attempts):
            if remaining <= 0 or self._deadline_reached():
                break
            if feasible and improvement_deadline is not None and perf_counter() >= improvement_deadline:
                break
            if feasible and improvement_started is None:
                improvement_started = perf_counter()
            # Stop before next(stream): lazy IK itself can consume the reserve.
            slots = self.budget.stage_connection_attempts - index
            allocation = max(1, int(np.ceil(remaining / slots)))
            with self._budget_scope(improvement_deadline if feasible else self._deadline_monotonic):
                ik_started = perf_counter()
                try:
                    candidate = next(stream)
                except StopIteration:
                    break
                finally:
                    self._statistics["trajectory_ik_wall_seconds"] += perf_counter() - ik_started
                path, candidate_failure, connection = self._transit(
                    start, candidate.q, obstacles, seed=connection_seed + index * 1009,
                    iteration_budget=allocation, attachment=attachment,
                    support_names=support_names, target_contact=target_contact, stage=stage)
            consumed = int(connection.get("planning_iterations_consumed", connection.get("iterations", 0)))
            remaining = max(0, remaining - consumed)
            if stage == "transit":
                self._placement_remaining = max(
                    0,
                    getattr(self, "_placement_remaining", self.budget.stage_connection_iterations)
                    - consumed,
                )
                self._placement_candidate_allowance = max(
                    0, getattr(self, "_placement_candidate_allowance",
                               self.budget.stage_connection_iterations) - consumed)
            attempts.append(
                {
                    "candidate_index": index,
                    "candidate_id": candidate.search_evidence.get("candidate_id"),
                    "q_rad": candidate.q.tolist(),
                    "position_error_m": float(candidate.position_error),
                    "orientation_error_rad": float(candidate.orientation_error),
                    "connection": connection,
                    "failure": candidate_failure,
                }
            )
            if candidate_failure is None:
                if not feasible:
                    improvement_deadline = self._optional_deadline()
                quality = self._optional_quality(path, improvement_deadline)
                attempts[-1]["path_quality"] = quality
                attempts[-1]["quality_status"] = "COMPLETE" if quality is not None else "NOT_COMPUTED_BUDGET"
                feasible.append((None if quality is None else quality["soft_score"],
                    candidate.q.copy(), [q.copy() for q in path]))
                selected = candidate.q.copy()
                selected_path = path
                failure = None
                if self.budget.proof_of_concept or attachment is not None or len(feasible) >= 2 or self._deadline_reached():
                    break
            elif not feasible:
                failure = candidate_failure
        if feasible:
            # Unknown is not zero or optimal. Without both scores keep baseline.
            best = feasible[0]
            for item in feasible[1:]:
                if best[0] is not None and item[0] is not None and item[0] < best[0]:
                    best = item
            _, selected, selected_path = best
            failure = None
            if initial_context != self._context_identity(obstacles, attachment=attachment,
                    support_names=support_names, target_contact=target_contact, stage=stage):
                selected, selected_path = None, []
                failure = {"reason": "VALIDATION_CONTEXT_CHANGED", "stage": stage}
        stream_evidence = stream.evidence()
        self._statistics["ik_calls"] += 1
        self._statistics["ik_seeds_attempted"] += int(stream_evidence.get("seeds_attempted", 0))
        self._statistics["ik_iterations_consumed"] += int(stream_evidence.get("iterations_consumed", 0))
        if selected is not None:
            termination = "SUCCESS"
        elif remaining <= 0 and not attempts:
            termination = "SHARED_CONNECTION_BUDGET_EXHAUSTED"
            failure = {"reason": termination, "stage": stage}
        elif failure is not None and failure.get("reason") == "VALIDATION_CONTEXT_CHANGED":
            termination = "VALIDATION_CONTEXT_CHANGED"
        elif self._deadline_reached():
            termination = "STAGE_IK_DEADLINE"
            failure = {"reason": termination, "stage": stage}
        elif not attempts:
            termination = "NO_VALID_IK"
            failure = {"reason": f"{stage.upper()}_NO_IK", "stage": stage}
        elif remaining <= 0:
            termination = "SHARED_CONNECTION_BUDGET_EXHAUSTED"
        elif len(attempts) >= self.budget.stage_connection_attempts:
            termination = "CONNECTION_ATTEMPT_LIMIT_REACHED"
        else:
            termination = "IK_STREAM_EXHAUSTED"
        return selected, selected_path, failure, {
            "stage": stage,
            "shared_connection_iteration_budget": self.budget.stage_connection_iterations,
            "remaining_connection_iterations": remaining,
            "attempts": attempts,
            "ik_stream": stream_evidence,
            "termination": termination,
            "validation_level": "B_STRICT_LOCAL_CONNECTION" if selected is not None else "A_UNVERIFIED_GEOMETRY",
            "improvement_deadline_monotonic": improvement_deadline,
            "optional_comparison_skipped": bool(feasible and improvement_started is None),
            "optional_comparison": {
                "started_monotonic": improvement_started,
                "finished_monotonic": None if improvement_started is None else perf_counter(),
                "skip_reason": ("NECESSARY_CONTINUATION_RESERVE" if feasible and improvement_started is None
                    and improvement_deadline is not None and perf_counter() >= improvement_deadline else None),
                "termination": (stream_evidence.get("termination") if improvement_started is not None else None),
                "fallback_to_verified": bool(feasible and (len(attempts) > len(feasible)
                    or (improvement_started is not None
                        and stream_evidence.get("termination") == "PLANNING_WALL_CLOCK_DEADLINE"))),
            },
        }

    def _cartesian(
        self,
        start: np.ndarray,
        destination: np.ndarray,
        obstacles: Sequence[OBB],
        *,
        seed: int,
        attachment: PhysicalContactAttachment | None = None,
        support_names: Sequence[str] = (),
        target_contact: OBB | None = None,
        initial_proximity: InitialProximityTracker | None = None,
        stage: str,
    ) -> tuple[list[np.ndarray], Mapping[str, Any] | None, Mapping[str, Any]]:
        origin = self.robot.fk(np.asarray(start, dtype=float))
        _, distance, angle = pose_error(origin, destination)
        count = max(
            1,
            int(np.ceil(distance / self.budget.cartesian_step_m)),
            int(np.ceil(angle / self.budget.cartesian_orientation_step_rad)),
        )
        if count > self.budget.cartesian_max_samples_per_stage:
            failure = {
                "reason": "CARTESIAN_SAMPLE_BUDGET_EXCEEDED",
                "stage": stage,
                "required_samples": count,
                "sample_limit": self.budget.cartesian_max_samples_per_stage,
            }
            return [np.asarray(start, dtype=float)], failure, {
                "stage": stage,
                "samples": [],
                "termination": "CARTESIAN_SAMPLE_BUDGET_EXCEEDED",
                "required_samples": count,
                "sample_limit": self.budget.cartesian_max_samples_per_stage,
            }
        rotation_vector = rotation_vector_from_matrix(
            destination[:3, :3] @ origin[:3, :3].T
        )
        path = [np.asarray(start, dtype=float)]
        samples: list[dict[str, Any]] = []
        for index in range(1, count + 1):
            if self._deadline_reached():
                return path, {"reason": "PLANNING_WALL_CLOCK_DEADLINE", "stage": stage}, {
                    "stage": stage, "samples": samples, "termination": "PLANNING_WALL_CLOCK_DEADLINE"
                }
            fraction = index / count
            pose = origin.copy()
            pose[:3, 3] = origin[:3, 3] + fraction * (
                destination[:3, 3] - origin[:3, 3]
            )
            pose[:3, :3] = (
                rotation_matrix_from_rotation_vector(rotation_vector * fraction)
                @ origin[:3, :3]
            )
            self.diagnostic_ik_seed = int(seed) + index
            self.diagnostic_cartesian_sample = index
            ik = solve_ik_multistart(
                self.robot,
                pose,
                [path[-1]],
                random_restarts=0,
                rng=np.random.default_rng(seed + index),
                max_iterations=int(self.ik["max_iterations"]),
                damping=float(self.ik["damping"]),
                max_step=float(self.ik["max_step_rad"]),
                position_tolerance=float(self.ik["position_tolerance_m"]),
                orientation_tolerance=float(self.ik["orientation_tolerance_rad"]),
                orientation_weight=float(self.ik["orientation_weight"]),
                collision_check_stride=int(self.ik["max_iterations"]) + 1,
                deadline_monotonic=self._deadline_monotonic,
            )
            self._statistics["ik_calls"] += 1
            self._statistics["ik_seeds_attempted"] += int(
                ik.search_evidence.get("seeds_attempted", 0)
            )
            self._statistics["ik_iterations_consumed"] += int(
                ik.search_evidence.get("iterations_consumed", 0)
            )
            self._statistics["cartesian_samples"] += 1
            sample = {
                "sample": index,
                "fraction": fraction,
                "success": bool(ik.success),
                "position_error_m": float(ik.position_error),
                "orientation_error_rad": float(ik.orientation_error),
            }
            samples.append(sample)
            if not ik.success:
                self._diagnostic_event({"event": "CARTESIAN_IK_REJECTED", "stage": stage,
                    "failure": {"reason": "NO_IK", **sample}, "q_rad": np.asarray(ik.q).tolist(),
                    "seed_q_rad": path[-1].tolist(), "ik_seed": int(seed) + index,
                    "requested_pose_world": pose.tolist(), "ik_policy": dict(self.ik),
                    "random_restarts": 0})
                return path, {
                    "reason": "NO_IK",
                    "stage": stage,
                    **sample,
                }, {"stage": stage, "samples": samples}
            branch_step = float(np.max(np.abs(ik.q - path[-1])))
            sample["maximum_joint_step_rad"] = branch_step
            if branch_step > self.budget.cartesian_max_branch_step_rad:
                self._diagnostic_event({"event": "CARTESIAN_IK_REJECTED", "stage": stage,
                    "failure": {"reason": "IK_BRANCH_JUMP", "maximum_joint_step_rad": branch_step,
                                "required_maximum_joint_step_rad": self.budget.cartesian_max_branch_step_rad},
                    "q_rad": np.asarray(ik.q).tolist(), "seed_q_rad": path[-1].tolist(),
                    "ik_seed": int(seed) + index, "requested_pose_world": pose.tolist(),
                    "ik_policy": dict(self.ik), "random_restarts": 0})
                return path, {
                    "reason": "IK_BRANCH_JUMP",
                    "stage": stage,
                    "sample": index,
                    "maximum_joint_step_rad": branch_step,
                }, {"stage": stage, "samples": samples}
            failure = self._path_failure(
                [path[-1], ik.q],
                obstacles,
                attachment=attachment,
                support_names=support_names,
                target_contact=target_contact,
                initial_proximity=initial_proximity,
                stage=stage,
            )
            if failure is not None:
                return path, failure, {"stage": stage, "samples": samples}
            path.append(ik.q.copy())
        return path, None, {"stage": stage, "samples": samples}

    @contextmanager
    def _contact_context(self):
        """Isolate speculative cup permissions, including mutable input values."""
        validator = self.robot_state_validator
        saved = {name: deepcopy(getattr(validator, name))
                 for name in ("commanded_cup_mask", "stack_carton_names", "contact_target_name")
                 if hasattr(validator, name)}
        saved_stack = deepcopy(self.stack_carton_names)
        try:
            yield
        finally:
            self.stack_carton_names = saved_stack
            for name, value in saved.items():
                setattr(validator, name, value)
            self._state_cache.clear()

    def _contact_selection(
        self, q: np.ndarray, target: OBB, face: str, suction: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        from .independent_cups import (
            m710_cup_array_from_mapping,
            select_ideal_independent_cups_from_actual_fk,
        )

        array = m710_cup_array_from_mapping(suction)
        selection = select_ideal_independent_cups_from_actual_fk(
            self.robot,
            q,
            target,
            face,
            array,
            self.flange_from_virtual_task_tcp,
            self.flange_from_physical_contact,
            max_attachment_gap_m=0.002,
            maximum_penetration_m=self.contact_tolerance_m,
            max_normal_misalignment_rad=np.deg2rad(5.0),
            suction_edge_margin_m=float(suction.get("suction_edge_margin_m", 0.0)),
        )
        result = selection.to_dict()
        if isinstance(self.robot_state_validator, ExactM710LayoutStateValidator):
            self.robot_state_validator.commanded_cup_mask = tuple(result["commanded_active_mask"])
            self.robot_state_validator.stack_carton_names = set(self.stack_carton_names or ())
            self.robot_state_validator.contact_target_name = target.name
            # Contact pair permissions depend on the commanded mask.
            self._state_cache.clear()
        return result

    def _initial_proximity(
        self,
        target: OBB,
        obstacles: Sequence[OBB],
        support_names: Sequence[str],
    ) -> tuple[InitialProximityTracker, Mapping[str, Any] | None]:
        if self.collision_policy.stack_contact_mode == "planner_relaxed_physics_checked":
            return PhysicsCheckedStackTracker(
                target, [o for o in obstacles if o.category == "carton"
                         and (self.stack_carton_names is None or o.name in self.stack_carton_names)], self.collision_policy,
                self.collision_margin_m, self.contact_tolerance_m), None
        supports = set(support_names)
        neighbors = [
            obstacle
            for obstacle in obstacles
            if obstacle.category == "carton"
            and obstacle.name != target.name
            and obstacle.name not in supports
        ]
        return InitialProximityTracker.capture(
            target,
            neighbors,
            self.collision_margin_m,
            self.contact_tolerance_m,
            self.contact_tolerance_m,
        )

    def _support_release(
        self,
        start: np.ndarray,
        attachment: PhysicalContactAttachment,
        obstacles: Sequence[OBB],
        support_names: Sequence[str],
        tracker: InitialProximityTracker,
        *,
        seed: int,
    ) -> tuple[list[np.ndarray], Mapping[str, Any] | None, Mapping[str, Any]]:
        names = tuple(dict.fromkeys(str(name) for name in support_names if name))
        if self.collision_policy.allows_stack_planning_contact("support-release"):
            return [start.copy()], None, {"stage": "support-release", "required": False,
                "lift_m": 0.0, "support_names": list(names),
                "motion_policy": "allow_supported_sliding_or_lift_in_extraction"}
        if not names:
            return [start.copy()], None, {
                "stage": "support-release",
                "required": False,
                "lift_m": 0.0,
            }
        by_name = {obstacle.name: obstacle for obstacle in obstacles}
        missing = sorted(set(names) - set(by_name))
        if missing:
            failure = {
                "reason": "SUPPORT_RELEASE_OBSTACLE_MISSING",
                "stage": "support-release",
                "support_names": missing,
            }
            return [], failure, failure
        box = attachment.box_at(start)
        clearance = self.collision_policy.pair_clearance("external", self.collision_margin_m) + 2.0 * self.contact_tolerance_m
        bottom = float(np.min(box.corners()[:, 2]))
        lifts = {
            name: max(
                0.0,
                by_name[name].center[2]
                + by_name[name].half_extents[2]
                + clearance
                - bottom,
            )
            for name in names
        }
        lift = max(lifts.values(), default=0.0)
        destination = self.robot.fk(start).copy()
        destination[2, 3] += lift
        path, failure, search = self._cartesian(
            start,
            destination,
            obstacles,
            seed=seed,
            attachment=attachment,
            support_names=names,
            initial_proximity=tracker,
            stage="support-release",
        )
        evidence = {
            "stage": "support-release",
            "required": True,
            "support_names": list(names),
            "lift_m": lift,
            "lift_by_support_m": lifts,
            "search": search,
        }
        if failure is None:
            released = attachment.box_at(path[-1])
            blocked = [
                name
                for name in names
                if obb_pair_failure(released, by_name[name], self.collision_policy, self.collision_margin_m)
            ]
            if blocked:
                failure = {
                    "reason": "SUPPORT_RELEASE_MARGIN_NOT_RESTORED",
                    "stage": "support-release",
                    "support_names": blocked,
                }
        evidence["success"] = failure is None
        evidence["failure"] = failure
        return path, failure, evidence

    def _extraction_options(
        self,
        start: np.ndarray,
        attachment: PhysicalContactAttachment,
        obstacles: Sequence[OBB],
        tracker: InitialProximityTracker,
        outward: np.ndarray,
        *,
        seed: int,
    ) -> tuple[list[np.ndarray], InitialProximityTracker, Mapping[str, Any] | None, Mapping[str, Any]]:
        box = attachment.box_at(start)
        up = np.asarray([0.0, 0.0, 1.0])
        directions = [outward, up, outward + up, outward + np.asarray([0.0, 1.0, 0.0]),
                      outward + np.asarray([0.0, -1.0, 0.0])]
        unique: list[np.ndarray] = []
        for direction in directions:
            direction = np.asarray(direction, dtype=float)
            norm = float(np.linalg.norm(direction))
            if not np.isfinite(norm) or norm <= 1e-12:
                continue
            direction /= norm
            if not any(np.allclose(direction, other, atol=1e-12, rtol=0.0) for other in unique):
                unique.append(direction)
        constraints = [
            obstacle
            for obstacle in obstacles
            if obstacle.category == "carton" and obstacle.name != box.name
        ]
        attempts: list[dict[str, Any]] = []
        free_clearance = max(self.collision_policy.pair_clearance("external", self.collision_margin_m) + self.contact_tolerance_m,
                             self.collision_policy.free_space_clearance_m)
        runtime_clearance_goal = (
            free_clearance + self.budget.extraction_runtime_clearance_reserve_m
        )
        # Search beyond, not exactly on, the acceptance boundary. A strict
        # but nonzero FK residual can otherwise leave every successful path
        # a fraction of a micron short of the unchanged physical clearance.
        fk_boundary_guard = (2.0 * float(self.ik["position_tolerance_m"])
            + 2.0 * float(np.linalg.norm(box.half_extents)) * float(self.ik["orientation_tolerance_rad"])
            + self.contact_tolerance_m)
        # Bias the bent exit away from actual remaining neighbors. Rotation
        # happens while the same attachment is still under stack monitoring.
        lateral = sum((box.center[1] - other.center[1]) /
                      max(0.01, np.linalg.norm(box.center - other.center)) for other in constraints)
        turn_sign = 1. if lateral >= 0 else -1.
        routes = [(unique[0], "straight")]
        routes += [(unique[0], "outward_then_turn"), (unique[0], "coupled_lift_turn")]
        routes += [(direction, "straight") for direction in unique[1:]]
        for index, (direction, route) in enumerate(routes if self.budget.proof_of_concept else routes[: self.budget.extraction_direction_attempts]):
            distance = minimum_clearance_extraction_distance(
                box,
                direction,
                constraints,
                free_space_clearance_m=runtime_clearance_goal + fk_boundary_guard,
                scan_step_m=self.budget.extraction_scan_step_m,
                maximum_distance_m=self.budget.maximum_extraction_m,
            )
            if distance is None:
                attempts.append(
                    {
                        "index": index,
                        "direction_world": direction.tolist(),
                        "status": "NO_GEOMETRIC_RELEASE_WITHIN_BOUND",
                    }
                )
                continue
            branch_tracker = tracker.clone()
            destination = self.robot.fk(start).copy()
            destination[:3, 3] += direction * float(distance)
            goals = []
            if route != "straight":
                intermediate = self.robot.fk(start).copy()
                intermediate[:3, 3] += direction * float(distance) / 3
                goals.append(intermediate)
                if route == "coupled_lift_turn":
                    destination[2, 3] += min(float(distance) / 3, box.half_extents[2] / 3)
                destination[:3, :3] = rotation_matrix_from_rotation_vector(
                    np.array([0., 0., turn_sign * min(self.budget.cartesian_orientation_step_rad * 2, 0.12)])
                ) @ destination[:3, :3]
            goals.append(destination)
            path, searches, failure = [start.copy()], [], None
            for goal in goals:
                part, failure, search = self._cartesian(path[-1], goal, obstacles,
                    seed=seed + index * 101 + len(searches), attachment=attachment,
                    initial_proximity=branch_tracker, stage="extraction")
                searches.append(search)
                if failure is not None:
                    break
                path.extend(part[1:])
            search = {"route": route, "parts": searches}
            released = failure is None and branch_tracker.fully_released
            if released:
                failure = self._extraction_reserve_failure(path[-1], attachment, obstacles)
                released = failure is None
            if failure is None and not released:
                failure = {"reason": "ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED", "stage": "extraction"}
            attempt = {
                "index": index,
                "direction_world": direction.tolist(),
                "distance_m": float(distance),
                "free_clearance_m": free_clearance,
                "runtime_clearance_goal_m": runtime_clearance_goal,
                "runtime_clearance_reserve_m": (
                    self.budget.extraction_runtime_clearance_reserve_m
                ),
                "fk_boundary_guard_m": fk_boundary_guard,
                "search": search,
                "initial_proximity": branch_tracker.evidence(),
                "status": "ACCEPTED" if released else "REJECTED",
                "failure": failure,
            }
            attempts.append(attempt)
            if released:
                yield path, branch_tracker, None, {
                    "stage": "extraction",
                    "attempts": attempts,
                    "selected_attempt": index,
                }
        self._last_extraction_attempts = attempts

    def _extraction_reserve_failure(self, q, attachment, obstacles):
        required = max(self.collision_policy.pair_clearance("external", self.collision_margin_m)
                       + self.contact_tolerance_m, self.collision_policy.free_space_clearance_m)
        required += self.budget.extraction_runtime_clearance_reserve_m
        payload = attachment.box_at(q)
        short = [(b.name, obb_surface_distance(payload, b)) for b in obstacles
                 if b.category == "carton" and b.name != payload.name
                 and obb_surface_distance(payload, b) + 1e-9 < required]
        return (dict(reason="EXTRACTION_EXECUTION_RESERVE_NOT_RETAINED", stage="extraction",
                     required_clearance_m=required, pairs=short) if short else None)

    def _approach_reserve_failure(self, q, target):
        required = self.collision_policy.pair_clearance("external", self.collision_margin_m)
        required += self.budget.approach_runtime_clearance_reserve_m
        if self.budget.approach_runtime_clearance_reserve_m <= 0:
            return None
        for tool in self.tool_collision_obbs_provider(q):
            gap = obb_surface_distance(tool, target)
            if gap + 1e-9 < required:
                return dict(reason="APPROACH_EXECUTION_RESERVE_NOT_RETAINED", stage="pregrasp",
                    pair=[tool.name,target.name], surface_distance_m=gap, required_pair_clearance_m=required)
        return None

    def _extraction(self, *args, **kwargs):
        # Compatibility for focused callers; production consumes all exits lazily.
        for option in self._extraction_options(*args, **kwargs):
            return option
        tracker = args[3]
        failure = {"reason": "NO_BOUNDED_EXTRACTION_PATH", "stage": "extraction"}
        return [], tracker, failure, {"attempts": getattr(self, "_last_extraction_attempts", []),
                                     "selected_attempt": None}

    @staticmethod
    def _append_stage(
        full: list[np.ndarray],
        stages: dict[str, list[int]],
        name: str,
        path: Sequence[np.ndarray],
    ) -> None:
        if not path:
            raise ValueError(f"stage {name} has no path")
        arrays = [np.asarray(q, dtype=float) for q in path]
        if not np.allclose(full[-1], arrays[0], atol=1e-10, rtol=0.0):
            raise ValueError(f"stage {name} does not start at the preceding endpoint")
        begin = len(full) - 1
        full.extend(q.copy() for q in arrays[1:])
        stages[name] = [begin, len(full) - 1]

    def _approach(self, start, grasp_q, requested, obstacles, target, *, seed):
        """Both routes share strict collision edges and the controlled terminal arc.

        Direct approach merges the free connector and terminal arc into contact;
        its transition is a geometric sample, not a process station. Adaptive
        branches retry the entire approach when a terminal arc fails.
        """
        physical = self.physical_from_virtual(requested)
        outward = -physical[:3, 2]
        tool = list(self.tool_collision_obbs_provider(grasp_q))
        depth = max((float(np.ptp(b.corners() @ outward)) for b in tool), default=0.03)
        error = 2 * float(self.ik["position_tolerance_m"]) + self.contact_tolerance_m
        terminal = max(self.collision_policy.pair_clearance("external", self.collision_margin_m)
                       + self.budget.approach_runtime_clearance_reserve_m + error,
                       min(depth / 4, self.budget.pregrasp_standoff_m))
        current_distance = float(np.linalg.norm(self.robot.fk(start)[:3, 3] - requested[:3, 3]))
        adaptive_distances = sorted({terminal, min(max(terminal, current_distance / 3), depth),
                                     self.budget.pregrasp_standoff_m})
        modes = ([self.budget.approach_mode] if self.budget.approach_mode != "auto"
                 else ["direct", "adaptive_pregrasp"])
        attempts = []
        last_failure = {"reason": "APPROACH_NOT_SEARCHED", "stage": "contact"}
        for mode in modes:
            for distance in ([terminal] if mode == "direct" else adaptive_distances):
                before = perf_counter()
                gate = physical.copy()
                gate[:3, 3] += outward * distance
                q, prefix, failure, search = self._connect_pose(self.virtual_from_physical(gate),
                    [start, grasp_q], start, obstacles, ik_seed=seed + len(attempts) * 100,
                    connection_seed=seed + len(attempts) * 100 + 1, stage="pregrasp")
                terminal_path = []
                if failure is None and q is not None:
                    failure = self._approach_reserve_failure(q, target)
                if failure is None and q is not None:
                    terminal_path, failure, terminal_search = self._cartesian(q, requested, obstacles,
                        seed=seed + len(attempts) * 100 + 2, target_contact=target, stage="contact")
                    search = {"connection": search, "terminal": terminal_search}
                attempts.append({"mode": mode, "terminal_distance_m": distance,
                    "planning_wall_seconds": perf_counter() - before, "failure": failure, "search": search})
                if failure is None and terminal_path:
                    if self._deadline_reached():
                        return [], [], {"reason": "PLANNING_WALL_CLOCK_DEADLINE", "stage": "contact"}, {"attempts": attempts}
                    approach = [*prefix, *terminal_path[1:]]
                    evidence = {"selected_mode": mode, "attempts": attempts,
                        "joint_path_length_rad": float(np.sum(np.linalg.norm(np.diff(approach, axis=0), axis=1))),
                        "independent_pregrasp_station": mode != "direct",
                        "free_connection_end_index": len(prefix) - 1,
                        "terminal_contact_start_index": len(prefix) - 1}
                    if mode == "direct":
                        return [start.copy()], approach, None, evidence
                    return prefix, terminal_path, None, evidence
                last_failure = failure or {"reason": "APPROACH_CONNECTION_FAILED", "stage": "contact"}
        return [], [], last_failure, {"attempts": attempts}

    def _plan_branch(
        self, **kwargs,
    ):
        outer = self._deadline_monotonic
        final = getattr(self, "_branch_final_deadline", None)
        try:
            return self._plan_branch_search(**kwargs)
        finally:
            self._deadline_monotonic = outer
            self._branch_final_deadline = final

    def _plan_branch_search(
        self,
        *,
        target: OBB,
        face: str,
        requested_virtual_contact: np.ndarray,
        grasp_q: np.ndarray,
        home_q: np.ndarray,
        all_obstacles: Sequence[OBB],
        receiver: OBB,
        support_names: Sequence[str],
        suction: Mapping[str, Any],
        seed: int,
    ) -> tuple[Mapping[str, Any] | None, Mapping[str, Any] | None, Mapping[str, Any]]:
        self.diagnostic_branch_seed = int(seed)
        trace: dict[str, Any] = {"seed": int(seed), "stages": {}}
        actual = self.robot.fk(grasp_q)
        _, position_error, orientation_error = pose_error(actual, requested_virtual_contact)
        if position_error > float(self.ik["position_tolerance_m"]) or orientation_error > float(
            self.ik["orientation_tolerance_rad"]
        ):
            failure = {
                "reason": "GRASP_FK_RESIDUAL_FAILED",
                "stage": "contact",
                "position_error_m": float(position_error),
                "orientation_error_rad": float(orientation_error),
            }
            return None, failure, trace

        actual_physical = self.physical_from_virtual(actual)
        try:
            self._contact_selection(grasp_q, target, face, suction)
        except ValueError as exc:
            return None, {"reason": "FINAL_CONTACT_GEOMETRY_FAILED", "detail": str(exc)}, trace
        outward = -actual_physical[:3, 2]
        pregrasp, contact, failure, evidence = self._approach(
            home_q, grasp_q, requested_virtual_contact, all_obstacles, target, seed=seed + 10)
        trace["stages"]["approach"] = evidence
        if failure is not None:
            return None, failure, trace
        contact_q = contact[-1]
        try:
            selection = self._contact_selection(contact_q, target, face, suction)
        except ValueError as exc:
            return None, {
                "reason": "FINAL_CONTACT_GEOMETRY_FAILED",
                "stage": "contact",
                "detail": str(exc),
            }, trace
        rigid_failure = self._state_failure(
            contact_q,
            all_obstacles,
            target_contact=target,
            stage="contact_endpoint",
        )
        if rigid_failure is not None:
            return None, {
                "reason": "FINAL_CONTACT_COLLISION_FAILED",
                "stage": "contact",
                "collision": rigid_failure,
            }, trace
        self._remember_path([*pregrasp, *contact[1:]],
            self._context_identity(all_obstacles, target_contact=target, stage="contact"),
            "contact", "C_COMPLETE_APPROACH")
        physical_contact = self.physical_from_virtual(self.robot.fk(contact_q))
        rigid = RigidAttachment.capture(physical_contact, target)
        attachment = PhysicalContactAttachment(
            self.robot,
            rigid,
            self.flange_from_virtual_task_tcp,
            self.flange_from_physical_contact,
        )
        if not np.allclose(
            attachment.box_at(contact_q).world_from_local,
            target.world_from_local,
            atol=1e-10,
            rtol=0.0,
        ):
            raise RuntimeError("physical contact attachment is not pose-continuous")
        payload_obstacles = [
            obstacle for obstacle in all_obstacles if obstacle.name != target.name
        ]
        tracker, proximity_failure = self._initial_proximity(
            target, payload_obstacles, support_names
        )
        if proximity_failure is not None:
            return None, {**dict(proximity_failure), "stage": "contact"}, trace

        support_release, failure, evidence = self._support_release(
            contact_q,
            attachment,
            payload_obstacles,
            support_names,
            tracker,
            seed=seed + 40,
        )
        trace["stages"]["support-release"] = evidence
        if failure is not None:
            return None, failure, trace

        extraction_options = self._extraction_options(
            support_release[-1], attachment, payload_obstacles, tracker, outward, seed=seed + 50)
        all_exit_attempts = []
        self._placement_remaining = self.budget.stage_connection_iterations
        self._local_transit_remaining = self.budget.local_transit_cartesian_sample_budget
        for exit_index, (extraction, released_tracker, failure, evidence) in enumerate(extraction_options):
            trace["stages"]["extraction"] = evidence
            supports = tuple(
                ConveyorSupport(box, self.surface_directions_world.get(box.name))
                for box in all_obstacles
                if box.category == "conveyor"
            ) or (ConveyorSupport(receiver, self.surface_directions_world.get(receiver.name)),)
            process_rank = {
                name: index
                for index, name in enumerate(self.placement_policy.overlap_process_priority)
            }
            supports = tuple(sorted(supports, key=lambda item: (
                process_rank.get(self.placement_policy.process_family_by_support.get(item.name, ""), 999),
                item.name,
            )))
            support_bodies = tuple(item.body for item in supports)
            occupied = [box for box in payload_obstacles if box.category == "carton"
                        and any(contact_separated(box, surface, self.contact_tolerance_m)
                                and support_union_audit(box, support_bodies)["supported"]
                                for surface in support_bodies)]
            placement_generation_started = perf_counter()
            placements = generate_conveyor_placements(target, supports or (receiver,),
                occupied=occupied, preferred_point_world=attachment.box_at(extraction[-1]).center,
                policy=replace(self.placement_policy, sampling_edge_reserve_m=self.budget.receiver_edge_reserve_m),
                contact_normal_local=(np.linalg.inv(rigid.tcp_from_box)[:3, :3]
                                      @ np.array([0.0, 0.0, 1.0])))
            placements = sorted(placements, key=lambda placement: (
                not receiver_footprint_reserve(placement.payload, support_bodies,
                    self.budget.receiver_edge_reserve_m,
                    contact_tolerance_m=self.contact_tolerance_m,
                    edge_tolerance_m=self.placement_policy.edge_tolerance_m)["supported"],))
            self._statistics["placement_candidate_generation_wall_seconds"] += (
                perf_counter() - placement_generation_started
            )
            self._statistics["placement_candidates_generated"] += len(placements)
            placement_attempts = []
            residence_fallback = None
            for placement_index, placement in enumerate(placements):
                if self.budget.proof_of_concept:
                    self._placement_remaining = self.budget.stage_connection_iterations
                    self._local_transit_remaining = self.budget.local_transit_cartesian_sample_budget
                if self._deadline_reached():
                    break
                # One shared downstream budget; reserve real connection work for
                # alternate supports instead of allowing the first location to
                # consume all of it. Unused direct-edge budget remains available.
                self._placement_candidate_allowance = min(self._placement_remaining,
                    max(1, self.budget.stage_connection_iterations // 3))
                branch_trace = {**trace, "stages": dict(trace["stages"])}
                segment = None
                release_attempts, release_choices, transit_hint = [], [], None
                upper_drop_goal = max(0., self.budget.maximum_drop_m - 2 * float(self.ik["position_tolerance_m"])
                                      - self.contact_tolerance_m)
                heights = (self.budget.release_policy().ideal_heights() if self.budget.proof_of_concept
                           else tuple(dict.fromkeys((0., self.budget.maximum_drop_m / 2, upper_drop_goal))))
                for height in heights:
                    if self._deadline_reached():
                        break
                    remaining_wall = self._remaining_wall_time()
                    if release_choices and remaining_wall is not None and remaining_wall < 30.:
                        break
                    branch_trace = {**trace, "stages": dict(trace["stages"])}
                    segment, failure, branch_trace = self._finish_place_branch(
                        target=target, face=face, requested_virtual_contact=requested_virtual_contact,
                        home_q=home_q, contact_q=contact_q, physical_contact=physical_contact,
                        rigid=rigid, attachment=attachment, selection=selection, pregrasp=pregrasp,
                        contact=contact, support_release=support_release, extraction=extraction,
                        released_tracker=released_tracker, payload_obstacles=payload_obstacles,
                        placement=placement, selected_supports=[box for box in support_bodies if box.name in placement.receiver_names],
                        trace=branch_trace, seed=seed + placement_index * 1000, release_height=height,
                        transit_hint=transit_hint)
                    record = {"height_m": height, "failure": failure}
                    if segment is not None:
                        if self.budget.proof_of_concept:
                            return segment, None, branch_trace
                        # All later release/placement alternatives are optional.
                        # One window, shared by their search, scoring and final
                        # checks; no new reserve per alternative.
                        if not release_choices and residence_fallback is None:
                            optional_deadline = self._optional_deadline(comparison=True)
                            self._deadline_monotonic = self._limit(self._deadline_monotonic, optional_deadline)
                            self._branch_final_deadline = self._limit(
                                getattr(self, "_branch_final_deadline", None), optional_deadline)
                        transit_start, transit_end = segment["stage_ranges"]["transit"]
                        transit_hint = [np.asarray(q) for q in segment["path"][transit_start:transit_end + 1]]
                        departure = segment["post_release_safe_residence"]
                        lookahead = departure.get("next_contact", {})
                        cost = (None if self._deadline_reached() else
                            float(np.sum(np.linalg.norm(np.diff(segment["path"], axis=0), axis=1))))
                        if self._deadline_reached():
                            cost = None
                        next_cost = lookahead.get("joint_path_length_rad")
                        record["current_motion_cost_rad"] = cost
                        record["two_task_cost_rad"] = None if next_cost is None or cost is None else cost + next_cost
                        record["quality_status"] = "COMPLETE" if cost is not None else "NOT_COMPUTED_BUDGET"
                        rank = (next_cost is None, None if cost is None else cost + (next_cost or 0.),
                                segment["place"]["release_prediction"]["flight_time_s"])
                        release_choices.append((rank, deepcopy(segment), deepcopy(branch_trace)))
                    release_attempts.append(record)
                    # Compare supported release with one feasible bounded drop;
                    # higher candidates are fallback, never compulsory motions.
                    if len(release_choices) >= 2:
                        break
                if release_choices:
                    best = release_choices[0]
                    for choice in release_choices[1:]:
                        if best[0][1] is not None and choice[0][1] is not None and choice[0] < best[0]:
                            best = choice
                    _, segment, branch_trace = best
                    failure = None
                    segment["place"]["release_selection"] = {
                        "objective": "prefer_checked_next_connection_then_current_cost_unknown_next_cost_explicit",
                        "attempts": release_attempts}
                branch_trace["release_attempts"] = release_attempts
                placement_attempts.append({"candidate": placement.as_dict(), "failure": failure,
                    "allocated_connection_iterations": self._placement_candidate_allowance,
                    "shared_remaining_connection_iterations": self._placement_remaining,
                    "downstream_stages": {key: value for key, value in branch_trace["stages"].items()
                                          if key in {"local_transit", "transit", "place", "withdrawal"}}})
                if segment is not None:
                    branch_trace["placement_attempts"] = placement_attempts
                    lookahead = segment["post_release_safe_residence"].get("next_contact", {})
                    if lookahead.get("status") == "BOUNDED_NEXT_CONTACT_SEARCH_FAILED":
                        if residence_fallback is None:
                            residence_fallback = (deepcopy(segment), None, deepcopy(branch_trace))
                            continue
                        return residence_fallback
                    return segment, None, branch_trace
            if residence_fallback is not None:
                return residence_fallback
            all_exit_attempts.append({"exit": exit_index, "extraction": evidence,
                                      "placements": placement_attempts})

        trace["exit_attempts"] = all_exit_attempts
        return None, {"reason": "EXTRACTION_AND_PLACEMENT_CANDIDATES_EXHAUSTED", "stage": "place",
                      "attempts": all_exit_attempts,
                      "extraction_attempts": getattr(self, "_last_extraction_attempts", [])}, trace

    def _local_cartesian_transit(self, start, destination, obstacles, attachment, *, seed):
        """Reuse strict Cartesian continuation with a shared finite sample pool.

        First try the direct SE(3) edge. If its first turn/descending edge cannot
        leave the stack safely, test small *normally collision-checked* outward
        motions before retrying. No attached-neighbor exception applies here.
        """
        evidence = {"stage": "transit", "method": "BOUNDED_EXISTING_CARTESIAN_WITH_OUTWARD_ESCAPE",
                    "attempts": [], "shared_sample_budget": self.budget.local_transit_cartesian_sample_budget}
        prefix = [np.asarray(start, dtype=float).copy()]

        def connect(origin_q, goal, attempt_seed):
            origin = self.robot.fk(origin_q)
            _, distance, angle = pose_error(origin, goal)
            needed = max(1, int(np.ceil(distance / self.budget.cartesian_step_m)),
                         int(np.ceil(angle / self.budget.cartesian_orientation_step_rad)))
            if needed > self._local_transit_remaining:
                return [], {"reason": "SHARED_LOCAL_TRANSIT_SAMPLE_BUDGET", "required": needed}, []
            # Leave one sample of headroom for the previous strict FK residual
            # when determining the next chunk's actual required sample count.
            capacity = max(1, self.budget.cartesian_max_samples_per_stage - 1)
            chunks = max(1, int(np.ceil(needed / capacity)))
            omega = rotation_vector_from_matrix(goal[:3, :3] @ origin[:3, :3].T)
            full, searches = [origin_q.copy()], []
            for chunk in range(1, chunks + 1):
                fraction = chunk / chunks
                waypoint = origin.copy()
                waypoint[:3, 3] += fraction * (goal[:3, 3] - origin[:3, 3])
                waypoint[:3, :3] = rotation_matrix_from_rotation_vector(fraction * omega) @ origin[:3, :3]
                _, chunk_distance, chunk_angle = pose_error(self.robot.fk(full[-1]), waypoint)
                chunk_needed = max(1, int(np.ceil(chunk_distance / self.budget.cartesian_step_m)),
                                   int(np.ceil(chunk_angle / self.budget.cartesian_orientation_step_rad)))
                if chunk_needed > self._local_transit_remaining:
                    return [], {"reason": "SHARED_LOCAL_TRANSIT_SAMPLE_BUDGET", "required": chunk_needed}, searches
                before = self._statistics["cartesian_samples"]
                path, failure, search = self._cartesian(full[-1], waypoint, obstacles,
                    attachment=attachment, seed=attempt_seed + chunk, stage="transit")
                self._local_transit_remaining -= self._statistics["cartesian_samples"] - before
                searches.append(search)
                if failure is not None:
                    return [], failure, searches
                full.extend(path[1:])
            return full, None, searches

        last_failure = {"reason": "LOCAL_TRANSIT_NOT_RUN"}
        for index in range(self.budget.local_transit_outward_attempts + 1):
            path, last_failure, searches = connect(prefix[-1], destination, seed + index * 1000)
            evidence["attempts"].append({"outward_steps": index, "failure": last_failure,
                                         "searches": searches})
            if last_failure is None:
                evidence["remaining_samples"] = self._local_transit_remaining
                self._diagnostic_event({"event": "LOCAL_TRANSIT_FINISHED", "stage": "transit",
                    "success": True, "seed": seed, "remaining_samples": self._local_transit_remaining})
                return [*prefix, *path[1:]], None, evidence
            if index == self.budget.local_transit_outward_attempts or self._local_transit_remaining <= 0:
                break
            collision_pair = last_failure.get("pair", [])
            stack_names = {box.name for box in obstacles if box.category == "carton"}
            if (last_failure.get("reason") != "PAYLOAD_COLLISION"
                    or not stack_names.intersection(collision_pair)):
                # Outward buffering addresses the nearby stack, not a failed
                # robot posture at a distant placement. Preserve local work
                # for a different receiving position instead of repeating it.
                break
            outward = -attachment.physical_contact_pose(prefix[-1])[:3, 2]
            waypoint = self.robot.fk(prefix[-1]).copy()
            waypoint[:3, 3] += outward * self.budget.local_transit_outward_step_m
            step, failure, searches = connect(prefix[-1], waypoint, seed + index * 1000 + 500)
            evidence["attempts"].append({"outward_step_m": self.budget.local_transit_outward_step_m,
                                         "failure": failure, "searches": searches})
            if failure is not None:
                last_failure = failure
                break
            prefix.extend(step[1:])
        evidence["remaining_samples"] = self._local_transit_remaining
        self._diagnostic_event({"event": "LOCAL_TRANSIT_FINISHED", "stage": "transit",
            "success": False, "failure": last_failure, "seed": seed,
            "configured_sample_budget": self.budget.local_transit_cartesian_sample_budget,
            "remaining_samples": self._local_transit_remaining})
        return [], last_failure, evidence

    def _next_contact_cost(self, start, placed, obstacles, sweep, *, seed):
        """One bounded lookahead through the production contact/path predicates.

        This is a cost preview, never an executable next task. The executor
        still replans from measured poses and preserves the released rigid body.
        """
        candidates = ([] if self.next_contact_provider is None else
                      self.next_contact_provider(placed.name))
        if not candidates:
            return {"status": "NOT_EVALUATED", "reason": "NO_NEXT_CONTACT_CANDIDATE", "joint_path_length_rad": None, "attempts": []}
        remaining = None if self.budget.proof_of_concept else getattr(self, "_lookahead_remaining_s", 12.0)
        if remaining is not None and remaining <= 0:
            return {"status": "NOT_EVALUATED", "joint_path_length_rad": None,
                    "reason": "REQUEST_LOOKAHEAD_BUDGET_EXHAUSTED", "attempts": [],
                    "requires_actual_state_replan": True}
        # The provider already encodes the next real row/center order.
        candidates = [c for c in candidates if c["target"].name == candidates[0]["target"].name][:2]
        before = perf_counter()
        outer_deadline = self._deadline_monotonic
        # A single conservative union encloses zero progress and all bounded
        # belt progress. It cannot assume the box already took the belt speed.
        delta = sweep[-1].center - sweep[0].center
        future = OBB(sweep[0].center + delta / 2,
            sweep[0].half_extents + np.abs(sweep[0].rotation.T @ delta) / 2,
            sweep[0].rotation, placed.name, placed.category)
        world = [*obstacles, future]
        attempts = []
        try:
            self._deadline_monotonic = self._limit(outer_deadline, None if self.budget.proof_of_concept else before + min(4., remaining))
            for index, candidate in enumerate(candidates):
                with self._contact_context():
                    if self._deadline_reached():
                        break
                    target = candidate["target"]
                    requested = candidate["requested_virtual_contact"]
                    endpoint_checks = []
                    stream = self._ik_stream(requested, [start], world, seed=seed + index * 100,
                        attachment=None, support_names=(), target_contact=target, stage="next_contact",
                        contact_candidate=candidate, endpoint_checks=endpoint_checks)
                    solved = next(stream, None)
                    self._statistics["ik_calls"] += 1
                    self._statistics["ik_seeds_attempted"] += int(stream.evidence().get("seeds_attempted", 0))
                    self._statistics["ik_iterations_consumed"] += int(stream.evidence().get("iterations_consumed", 0))
                    failure = {"reason": "LOOKAHEAD_DEADLINE" if self._deadline_reached() else "NO_NEXT_CONTACT_IK"}
                    if solved is not None:
                        try:
                            selected_cups = self._contact_selection(solved.q, target, candidate["face"], candidate["suction"])
                            failure = self._state_failure(solved.q, world, target_contact=target, stage="contact_endpoint")
                        except ValueError as exc:
                            failure = {"reason": "NEXT_CONTACT_COVERAGE_FAILED", "detail": str(exc)}
                        if failure is not None:
                            attempts.append({"target": target.name, "face": candidate["face"], "failure": failure,
                                             "endpoint_checks": endpoint_checks})
                            continue
                        prefix, terminal, failure, approach = self._approach(start, solved.q,
                            requested, world, target, seed=seed + index * 100 + 1)
                        if failure is None:
                            try:
                                cups = self._contact_selection(terminal[-1], target,
                                    candidate["face"], candidate["suction"])
                            except ValueError as exc:
                                failure = {"reason": "NEXT_CONTACT_COVERAGE_FAILED", "detail": str(exc)}
                            if failure is None:
                                path = [*prefix, *terminal[1:]]
                                failure = self._state_failure(terminal[-1], world,
                                    target_contact=target, stage="contact_endpoint")
                                if cups["commanded_active_mask"] != selected_cups["commanded_active_mask"]:
                                    # Cartesian IK can end at a slightly different q. Recheck
                                    # the complete approach under the final command if it changed.
                                    gate = approach["free_connection_end_index"]
                                    failure = failure or self._path_failure(path[:gate + 1], world, stage="pregrasp")
                                    failure = failure or self._path_failure(path[gate:], world,
                                        target_contact=target, stage="contact")
                            if failure is None:
                                cost = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
                                verified = self._remember_path(path,
                                    self._context_identity(world, target_contact=target, stage="contact"),
                                    "contact", "C_COMPLETE_APPROACH")
                                return {"status": "CHECKED_NEXT_CONTACT_CONNECTION", "target": target.name,
                                    "validation_level": verified["validation_level"],
                                    "validation_completed_monotonic": verified["completed_monotonic"],
                                    "execution_ready": False,
                                    "row_id": candidate["row_id"], "face": candidate["face"],
                                    "approach_mode": approach["selected_mode"], "joint_path_length_rad": cost,
                                    "start_q_rad": start.tolist(), "contact_q_rad": terminal[-1].tolist(),
                                    "commanded_active_mask": cups["commanded_active_mask"],
                                    "endpoint_checks": endpoint_checks,
                                    "planning_wall_seconds": perf_counter() - before,
                                    "attempts": attempts, "requires_actual_state_replan": True,
                                    "released_carton_swept_occupancy_retained": True}
                    attempts.append({"target": target.name, "face": candidate["face"], "failure": failure,
                                     "endpoint_checks": endpoint_checks})
            return {"status": "BOUNDED_NEXT_CONTACT_SEARCH_FAILED", "cost_status": "UNKNOWN",
                "requires_actual_state_replan": True, "joint_path_length_rad": None, "attempts": attempts,
                "planning_wall_seconds": perf_counter() - before,
                "safe_current_residence_remains_valid": True}
        finally:
            self._lookahead_remaining_s = None if remaining is None else max(0., remaining - (perf_counter() - before))
            self._deadline_monotonic = outer_deadline

    def _departure(self, start, placed, obstacles, direction, *, seed, working_normal,
                   release_prediction, verified_baseline=None):
        outer = self._deadline_monotonic
        identity = self._context_identity(obstacles, target_contact=placed, stage="withdrawal")
        try:
            result = self._departure_search(start, placed, obstacles, direction, seed=seed,
                working_normal=working_normal, release_prediction=release_prediction,
                verified_baseline=verified_baseline)
            if identity != self._context_identity(obstacles, target_contact=placed, stage="withdrawal"):
                return [], {"reason": "VALIDATION_CONTEXT_CHANGED", "stage": "withdrawal"}, {}
            return result
        finally:
            self._deadline_monotonic = outer

    def _departure_search(self, start, placed, obstacles, direction, *, seed, working_normal,
                   release_prediction, verified_baseline=None):
        if direction is None:
            return [], {"reason": "POST_RELEASE_CONVEYOR_DIRECTION_UNAVAILABLE"}, {}
        direction = np.asarray(direction, float)
        tool = list(self.tool_collision_obbs_provider(start))
        if not tool:
            return [], {"reason": "POST_RELEASE_TOOL_ENVELOPE_UNAVAILABLE"}, {}
        clearance = (self.collision_policy.pair_clearance("external", self.collision_margin_m)
                     + self.budget.departure_runtime_clearance_reserve_m + self.contact_tolerance_m
                     + 2 * float(self.ik["position_tolerance_m"]))
        # Sweep far enough to pass the entire tool, including a stalled carton.
        span = max(float(np.max(b.corners() @ direction)) for b in tool) - float(np.min(placed.corners() @ direction))
        flight_envelope = release_flight_envelope(placed, release_prediction)
        sweep = departure_sweep(flight_envelope, direction, distance_m=max(clearance, span + clearance),
                                resolution_m=self.budget.cartesian_step_m)
        if self.post_landing_transport["mode"] == "ideal_outfeed":
            # Keep the entire release-to-first-landing flight envelope. Beyond
            # first reception the same carton is explicitly outside collision
            # planning; no full-belt sweep or tail residence is needed.
            sweep = [flight_envelope]
        candidates = [-np.asarray(working_normal), np.array([0., 0., 1.]), -direction]
        candidates += [candidates[0] + candidates[1], candidates[0] - direction]
        future_contacts = ([] if self.next_contact_provider is None else self.next_contact_provider(placed.name))
        if future_contacts:
            candidates.insert(0, future_contacts[0]["target"].center - placed.center)
        # Compare translation-only and a few gradual reorientation departures.
        # Every interpolated pose is still checked against the flight envelope.
        variants = [(vector, turn) for i, vector in enumerate(candidates)
                    for turn in ((0., 1.) if future_contacts and i < 2 else (0.,))]
        attempts, safe_choices, tested_directions = [], [], []
        def score_departure(path, evidence, candidate_seed):
            if self.budget.proof_of_concept:
                evidence.update(next_contact={"status": "DEFERRED_UNTIL_BASELINE_DELIVERY"},
                                two_task_cost_rad=None, departure_quality=None)
                return ((True, None), [q.copy() for q in path], deepcopy(evidence))
            lookahead = self._next_contact_cost(path[-1], placed, obstacles, sweep, seed=candidate_seed)
            quality = self._optional_quality(path, self._deadline_monotonic)
            departure_cost = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
            next_cost = lookahead.get("joint_path_length_rad")
            evidence.update(next_contact=lookahead, departure_quality=quality,
                departure_joint_path_length_rad=departure_cost,
                two_task_cost_rad=None if next_cost is None else departure_cost + next_cost)
            score = None if quality is None or next_cost is None else quality["soft_score"] + next_cost
            return ((next_cost is None, score), [q.copy() for q in path], deepcopy(evidence))
        if verified_baseline is not None:
            path, evidence = verified_baseline
            self._deadline_monotonic = self._limit(self._deadline_monotonic,
                self._optional_deadline(comparison=True))
            baseline_evidence = deepcopy(evidence)
            baseline_evidence.update(selected_direction_world=None, baseline_source="HISTORY_CURRENT_RECHECK")
            safe_choices.append(score_departure(path, baseline_evidence, seed))
            # Baseline counts toward the three-candidate bound. No Cartesian
            # alternative is useful if the baseline next-target cost is unknown.
            variants = variants[:2] if safe_choices[0][0][1] is not None else []
        for index, (vector, turn) in enumerate(variants):
            if self.budget.proof_of_concept and safe_choices:
                break
            if self._deadline_reached():
                break
            if np.linalg.norm(vector) < 1e-10:
                continue
            vector = vector / np.linalg.norm(vector)
            if any(turn == old_turn and np.allclose(vector, tested, atol=1e-10, rtol=0)
                   for tested, old_turn in tested_directions):
                continue
            tested_directions.append((vector, turn))
            # Project complete solids, rather than imposing a normal retreat
            # or a vertical lift. Each direction derives its own distance.
            minimum_tool = min(float(np.min(b.corners() @ vector)) for b in tool)
            maximum_box = max(float(np.max(b.corners() @ vector)) for b in sweep)
            distance = max(0., maximum_box + clearance - minimum_tool)
            if distance > self.budget.maximum_extraction_m:
                continue
            destination = self.robot.fk(start).copy()
            destination[:3, 3] += distance * vector
            orientation_change = np.zeros(3)
            if turn:
                desired = np.asarray(future_contacts[0]["requested_virtual_contact"])
                orientation_change = rotation_vector_from_matrix(desired[:3, :3] @ destination[:3, :3].T)
                angle = np.linalg.norm(orientation_change)
                orientation_change *= min(1., np.deg2rad(30.) / max(angle, 1e-12))
                destination[:3, :3] = rotation_matrix_from_rotation_vector(orientation_change) @ destination[:3, :3]
            path, failure, search = self._cartesian(start, destination, [*obstacles, placed],
                seed=seed + index, target_contact=placed, stage="withdrawal")
            if failure is None:
                for predicted in sweep:
                    failure = self._path_failure(path, [*obstacles, predicted],
                        target_contact=predicted, stage="withdrawal")
                    if failure is not None:
                        break
            if failure is None:
                for predicted in sweep:
                    failure = self._state_failure(path[-1], [*obstacles, predicted], target_contact=predicted, stage="residence")
                    if failure is not None:
                        break
            attempts.append({"direction_world": vector.tolist(), "distance_m": distance,
                             "orientation_change_rotvec_rad": orientation_change.tolist(),
                             "failure": failure, "search": search})
            if failure is None:
                if not safe_choices:
                    # Safety is established before optional lookahead/quality.
                    self._deadline_monotonic = self._limit(self._deadline_monotonic,
                        self._optional_deadline(comparison=True))
                evidence = {"model": "bounded_departure_swept_occupancy_v2",
                    "selected_direction_world": vector.tolist(), "distance_m": distance,
                    "selected_orientation_change_rotvec_rad": orientation_change.tolist(),
                    "attempts": attempts, "conveyor_surface": release_prediction["landing_support"]["receiver_names"],
                    "stationary_carton_included": True, "sweep_samples": len(sweep),
                    "next_approach_start_q_rad": path[-1].tolist(),
                    "fixed_normal_retreat_or_vertical_lift": False}
                safe_choices.append(score_departure(path, evidence, seed + (index+1)*1000))
                if not future_contacts or len(safe_choices) >= 3:
                    break
        if safe_choices:
            best = safe_choices[0]
            for choice in safe_choices[1:]:
                comparable = (best[2]["next_contact"].get("target"), best[2]["next_contact"].get("row_id")) == (
                    choice[2]["next_contact"].get("target"), choice[2]["next_contact"].get("row_id"))
                if (comparable and best[0][1] is not None and choice[0][1] is not None
                        and choice[0] < best[0] and choice[2]["two_task_cost_rad"] < best[2]["two_task_cost_rad"]):
                    best = choice
            _, selected_path, selected_evidence = best
            selected_evidence["comparison_scope"] = "SAME_NEXT_TARGET_ROW_FIRST_TWO_CONTACT_CANDIDATES"
            if verified_baseline is not None:
                selected_evidence["candidate_limit_including_baseline"] = 3
                selected_evidence["alternatives_status"] = (
                    "NOT_EVALUATED_BASELINE_NEXT_COST_UNKNOWN" if not variants else
                    "BOUNDED_COMPARISON_OR_PARENT_DEADLINE")
            selected_evidence["compared_safe_departures"] = [{"selection_score": rank[1], "two_task_cost_rad": evidence["two_task_cost_rad"],
                "next_contact_status": evidence["next_contact"]["status"],
                "next_contact_target": evidence["next_contact"].get("target"),
                "next_contact_row": evidence["next_contact"].get("row_id"),
                "next_contact_face": evidence["next_contact"].get("face"),
                "departure_quality": evidence.get("departure_quality"),
                "direction_world": evidence["selected_direction_world"]}
                for rank, _, evidence in safe_choices]
            return selected_path, None, selected_evidence
        # A bounded wait is a real fallback only if the entire uncertain
        # moving-carton sweep is safe at this pose, including legal cup contact,
        # and a later measured separation can restore the ordinary rules.
        wait_failure = None
        for predicted in sweep:
            wait_failure = self._state_failure(start, [*obstacles, predicted],
                target_contact=predicted, stage="withdrawal")
            if wait_failure is not None:
                break
        if wait_failure is None:
            wait_failure = self._state_failure(start, [*obstacles, sweep[-1]], stage="residence")
        if wait_failure is None:
            return [start.copy()], None, {"model": "bounded_wait_for_actual_carton_separation_v2",
                "attempts": attempts, "required_observed_progress_m": max(clearance, span + clearance),
                "stationary_carton_included": True, "next_approach_start_q_rad": start.tolist(),
                "fixed_normal_retreat_or_vertical_lift": False, "actual_separation_required": True}
        failure = {"reason": "NO_SAFE_MOVING_CARTON_DEPARTURE", "stage": "withdrawal"}
        return [], failure, {"attempts": attempts, "failure": failure}

    def _finish_place_branch(self, *, target, face, requested_virtual_contact, home_q,
            contact_q, physical_contact, rigid, attachment, selection, pregrasp, contact,
            support_release, extraction, released_tracker, payload_obstacles, placement,
            selected_supports, trace, seed, release_height=0., transit_hint=None, history_hint=None):
        if self.budget.proof_of_concept:
            policy = self.budget.release_policy()
            policy.ideal_heights()
            if not policy.ideal_release_min_height_m <= release_height <= policy.ideal_release_max_height_m:
                return None, {"reason": "IDEAL_RELEASE_HEIGHT_OUT_OF_BOUNDS", "stage": "place"}, trace
        desired_box = placement.payload.world_from_local.copy()
        desired_box[2, 3] += release_height
        support_z = float(np.min(placement.payload.corners()[:, 2]))
        # A footprint may end exactly at an adjoining conveyor seam. Its
        # zero-area contact with that second coplanar top is still a legal
        # support-face contact, even though it bears none of the bottom area.
        # Keep both real bodies and contact_separated's bottom-plane test;
        # robot/tool margins and support-side penetration remain unchanged.
        coplanar_supports = tuple(box for box in payload_obstacles
            if box.category == "conveyor"
            and np.allclose(box.rotation[:, 2], [0., 0., 1.], atol=1e-9, rtol=0)
            and abs(float(np.max(box.corners()[:, 2])) - support_z) <= self.contact_tolerance_m)
        selected_supports = coplanar_supports or tuple(selected_supports)
        support_contact_names = tuple(
            box.name for box in selected_supports
            if box.name in set(placement.receiver_names)
        ) or tuple(placement.receiver_names)
        desired_physical = desired_box @ np.linalg.inv(rigid.tcp_from_box)
        preplace_physical = desired_physical.copy()
        # Receiver approach and final working normal are independent.
        # Derive clearance from the unchanged margins and strict FK residual.
        receiver_clearance = (self.collision_policy.pair_clearance("external", self.collision_margin_m)
                              + self.budget.receiver_runtime_clearance_reserve_m + self.contact_tolerance_m
                              + 2 * float(self.ik["position_tolerance_m"]))
        preplace_physical[2, 3] += max(0., receiver_clearance - release_height)
        preplace_virtual = self.virtual_from_physical(preplace_physical)
        transit = []
        place_virtual = self.virtual_from_physical(desired_physical)
        if history_hint is not None:
            from .history_adaptation import loaded_suffix
            transit, history_place, failure = loaded_suffix(self, history_hint, extraction,
                preplace_virtual, place_virtual, payload_obstacles, attachment, support_contact_names)
            if failure is not None:
                return None, failure, trace
            local_evidence = {"source": "HISTORY_NODES_NEW_ATTACHMENT_FULL_EDGE_RECHECK"}
        if history_hint is None and transit_hint:
            suffix, reuse_failure, local_evidence = self._cartesian(
                transit_hint[-1], preplace_virtual, payload_obstacles, seed=seed + 54,
                attachment=attachment, stage="transit")
            if reuse_failure is None:
                transit = [*transit_hint, *suffix[1:]]
                local_evidence = {"reused_verified_loaded_prefix": True, "suffix": local_evidence}
        if not transit and getattr(self, "_local_transit_remaining", 0) > 0:
            transit, local_failure, local_evidence = self._local_cartesian_transit(
                extraction[-1], preplace_virtual, payload_obstacles, attachment, seed=seed + 55)
            trace["stages"]["local_transit"] = local_evidence
        if transit:
            preplace_q, failure, evidence = transit[-1], None, local_evidence
        else:
            preplace_q, transit, failure, evidence = self._connect_pose(
                preplace_virtual,
                [extraction[-1], contact_q, home_q],
                extraction[-1],
                payload_obstacles,
                ik_seed=seed + 60,
                connection_seed=seed + 70,
                attachment=attachment,
                stage="transit",
            )
        trace["stages"]["transit"] = evidence
        if preplace_q is None or failure is not None:
            return None, failure, trace

        if history_hint is not None:
            place, failure, evidence = history_place, None, {"source": "CURRENT_ATTACHMENT_RECEIVER_IK_AND_EDGE_RECHECK"}
        else:
            place, failure, evidence = self._cartesian(
                preplace_q,
                place_virtual,
                payload_obstacles,
                seed=seed + 80,
                attachment=attachment,
                support_names=support_contact_names,
                stage="place",
            )
        trace["stages"]["place"] = evidence
        if failure is not None:
            return None, failure, trace
        placed = attachment.box_at(place[-1])
        support = support_union_audit(
            placed,
            selected_supports,
            contact_tolerance_m=self.contact_tolerance_m,
            edge_tolerance_m=self.placement_policy.edge_tolerance_m,
            engineering_edge_margin_m=self.placement_policy.engineering_edge_margin_m,
        )
        release_mode = (IDEAL_RECEPTION_RELEASE if self.budget.proof_of_concept else
                        SHORT_DROP_RELEASE if release_height > 0 else SUPPORTED_RELEASE)
        release_prediction = predict_release(placed, selected_supports, mode=release_mode,
            obstacles=payload_obstacles, policy=self.budget.release_policy(),
            contact_tolerance_m=self.contact_tolerance_m,
            edge_tolerance_m=self.placement_policy.edge_tolerance_m)
        if not release_prediction["accepted"]:
            return None, {
                "reason": ("ACTUAL_FK_SUPPORT_FAILED" if release_mode == SUPPORTED_RELEASE
                           else release_prediction["reason"]),
                "stage": "place",
                "support": support,
                "release_prediction": release_prediction,
            }, trace

        actual_support_names = tuple(str(name) for name in
            release_prediction["landing_support"]["receiver_names"])
        support_descriptors = tuple(
            ConveyorSupport(box, self.surface_directions_world.get(box.name))
            for box in selected_supports
        )
        actual_process_family, actual_effective_receiver = effective_process_and_receiver(
            actual_support_names, support_descriptors, self.placement_policy
        )
        if actual_process_family is None or actual_effective_receiver is None:
            return None, {
                "reason": "ACTUAL_SUPPORT_PROCESS_UNRESOLVED",
                "stage": "place",
                "actual_support_names": list(actual_support_names),
            }, trace

        place_physical = attachment.physical_contact_pose(place[-1])
        actual_working_normal = place_physical[:3, 2]
        relative_payload = placed.center - place_physical[:3, 3]
        actual_placement_family = {
            process: tuple(families)
            for process, families in self.placement_policy.allowed_families_by_process.items()
        }.get(actual_process_family, ())
        required_normal = placement_working_normal(placement.placement_family)
        minimum_alignment = float(np.cos(self.placement_policy.normal_tolerance_rad))
        if (required_normal is None
                or placement.placement_family not in actual_placement_family
                or float(actual_working_normal @ required_normal) < minimum_alignment
                or float(relative_payload @ actual_working_normal) <= 0.0):
            failure = {
                "reason": "ACTUAL_PLACEMENT_PROCESS_RELATION_FAILED",
                "stage": "place",
                "placement_family": placement.placement_family,
                "actual_working_normal_world": actual_working_normal.tolist(),
                "payload_relative_to_tool_world_m": relative_payload.tolist(),
            }
            trace["stages"]["place"]["process_relation_failure"] = failure
            return None, failure, trace
        place_surface = actual_effective_receiver
        direction = self.surface_directions_world.get(place_surface)
        if direction is None:
            return None, {"reason": "POST_RELEASE_CONVEYOR_DIRECTION_UNAVAILABLE", "stage": "place"}, trace
        landing_pose = np.asarray(release_prediction["predicted_landing_pose_world"])
        landing_box = OBB(landing_pose[:3, 3], placed.half_extents, landing_pose[:3, :3], placed.name, placed.category)
        edge_reserve = receiver_footprint_reserve(landing_box, selected_supports, self.budget.receiver_edge_reserve_m,
            contact_tolerance_m=self.contact_tolerance_m, edge_tolerance_m=self.placement_policy.edge_tolerance_m,
            ideal_region=self.budget.proof_of_concept)
        if not edge_reserve["supported"]:
            return None, {"reason": "RECEIVER_EDGE_RESERVE_UNAVAILABLE", "stage": "place",
                          "edge_reserve": edge_reserve}, trace
        if self.post_landing_transport["mode"] == "ideal_outfeed":
            transport_support = {"accepted": True, "source": "EXPLICIT_POST_LANDING_IDEAL_OUTFEED",
                "physical_transport_checked": False, "first_landing_footprint_checked": True}
        else:
            transport_support = receiver_transport_support(landing_box,
                next(box for box in selected_supports if box.name == place_surface), selected_supports, direction,
                contact_tolerance_m=self.contact_tolerance_m, edge_tolerance_m=self.placement_policy.edge_tolerance_m)
        if not transport_support["accepted"]:
            return None, {"reason": transport_support["reason"], "stage": "place",
                          "transport_support": transport_support}, trace
        if history_hint is not None:
            from .history_adaptation import checked_departure
            withdrawal, failure, escape_audit = checked_departure(self, history_hint,
                place[-1], placed, payload_obstacles, direction, release_prediction)
        else:
            withdrawal, failure, escape_audit = self._departure(
                place[-1], placed, payload_obstacles, direction, seed=seed + 90,
                working_normal=place_physical[:3, 2], release_prediction=release_prediction)
        trace["stages"]["withdrawal"] = escape_audit
        if failure is not None:
            return None, failure, trace

        full = [home_q.copy()]
        stages: dict[str, list[int]] = {"home": [0, 0]}
        if len(pregrasp) > 1:
            self._append_stage(full, stages, "pregrasp", pregrasp)
        self._append_stage(full, stages, "contact", contact)
        grasp_index = stages["contact"][1]
        if len(support_release) > 1:
            self._append_stage(full, stages, "support-release", support_release)
        self._append_stage(full, stages, "extraction", extraction)
        self._append_stage(full, stages, "transit", transit)
        self._append_stage(full, stages, "place", place)
        release_index = stages["place"][1]
        self._append_stage(full, stages, "withdrawal", withdrawal)
        release_retreat_index = stages["withdrawal"][1]
        segment: dict[str, Any] = {
            "schema": TRAJECTORY_SEGMENT_SCHEMA,
            "motion_semantics": MOTION_SEMANTICS,
            "placement_semantics": PLACEMENT_SEMANTICS,
            "approach": trace["stages"].get("approach", {}),
            "planning_execution_reserves": self.budget.execution_reserves(),
            "target": target.name,
            "face": face,
            "path": [q.tolist() for q in full],
            "grasp_index": int(grasp_index),
            "release_index": int(release_index),
            "release_retreat_index": int(release_retreat_index),
            "stage_ranges": stages,
            "events": [
                {"index": int(grasp_index), "event": "ATTACH"},
                {"index": int(release_index), "event": "RELEASE"},
                {
                    "index": int(release_retreat_index),
                    "event": "RELEASE_RETREAT_COMPLETE",
                },
            ],
            "contact": {
                "requested_virtual_task_tcp_pose_world": requested_virtual_contact.tolist(),
                "requested_physical_contact_pose_world": self.physical_from_virtual(
                    requested_virtual_contact
                ).tolist(),
                "actual_q_rad": contact_q.tolist(),
                "actual_virtual_task_tcp_pose_world": self.robot.fk(contact_q).tolist(),
                "actual_physical_contact_pose_world": physical_contact.tolist(),
                "physical_contact_from_box": rigid.tcp_from_box.tolist(),
                "cup_selection": selection,
            },
            "place": {
                "release_mode": release_mode,
                "release_prediction": release_prediction,
                "receiver_transport_support": transport_support,
                "receiver_edge_reserve_m": self.budget.receiver_edge_reserve_m,
                "actual_box_pose_world": placed.world_from_local.tolist(),
                "release_center_world_m": placed.center.tolist(),
                "place_surface": place_surface,
                "support_names": [box.name for box in selected_supports],
                "load_bearing_support_names": list(actual_support_names),
                "selection": placement.as_dict(),
                "support": support,
                "receiver": place_surface,
                "effective_receiver": place_surface,
                "process_family": actual_process_family,
                "placement_family": placement.placement_family,
                "actual_working_normal_world": actual_working_normal.tolist(),
                "payload_relative_to_tool_world_m": relative_payload.tolist(),
            },
            "post_release_safe_residence": escape_audit,
            "validation": {
                "validator_identity": self.validator_identity,
                "execution_qualified": True,
                "collision_margin_per_body_m": self.collision_margin_m,
                "contact_tolerance_m": self.contact_tolerance_m,
                "initial_proximity": released_tracker.evidence(),
                "collision_policy": self.collision_policy.to_mapping(),
            },
        }
        trace["selected"] = {
            "grasp_index": int(grasp_index),
            "release_index": int(release_index),
            "release_retreat_index": int(release_retreat_index),
        }
        if history_hint is not None:
            segment["history"] = deepcopy(trace["history"])
            segment["history"].update(new_release_q_rad=place[-1].tolist(),
                new_release_box_pose_world=placed.world_from_local.tolist(),
                validation_inherited=False, all_required_stages_rechecked=True)
        failure = self._finalize_task(segment, [*payload_obstacles, target], target)
        if failure is not None:
            return None, failure, trace
        return segment, None, trace

    def plan(self, **kwargs):
        """One bounded candidate slice nested inside the shared request deadline."""
        outer = self._deadline_monotonic
        outer_final = getattr(self, "_candidate_final_deadline", None)
        allowance = getattr(self, "candidate_slice_s", self.budget.candidate_wall_time_s)
        started = perf_counter()
        self._candidate_final_deadline = self._limit(
            getattr(self, "_request_deadline_monotonic", outer), deadline_after(started, allowance))
        if kwargs.pop("optional_quality", False) or kwargs.get("history_hint") is not None:
            # History's share cannot borrow the ordinary-search/final reserve.
            self._candidate_final_deadline = self._limit(self._candidate_final_deadline, outer)
        self._deadline_monotonic = self._limit(outer, self._candidate_final_deadline)
        self._completed_tasks.clear()
        event_start = len(self._budget_events)
        try:
            result = self._plan_candidate(**kwargs)
            if not result.success and self._deadline_reached() and (outer is None or perf_counter() < outer):
                result.statistics["termination"] = "CANDIDATE_WALL_CLOCK_DEADLINE"
                result.failure["budget_termination"] = "CANDIDATE_WALL_CLOCK_DEADLINE"
            result.statistics["candidate_wall_seconds"] = perf_counter() - started
            result.statistics["candidate_wall_budget_s"] = allowance
            result.statistics["request_final_export_reserve_s"] = getattr(self, "_final_export_reserve_s", 0.)
            result.statistics["result_retention"] = {
                "events": deepcopy(self._budget_events[event_start:]),
                "planning_success": result.success, "execution_ready": False,
                "execution_readiness_status": "INDEPENDENT_PREFLIGHT_NOT_RUN",
                "request_deadline_monotonic": getattr(self, "_request_deadline_monotonic", None),
                "search_deadline_monotonic": outer,
                "candidate_search_deadline_monotonic": self._deadline_monotonic,
                "candidate_final_deadline_monotonic": self._candidate_final_deadline,
                "returned_monotonic": perf_counter(),
                "final_reserve_scope": "COMPLETE_TASK_VALIDATION_AND_IN_PROCESS_BINDING_ONLY"}
            retention = result.statistics["result_retention"]
            returned = retention["returned_monotonic"]
            retention["return_after_search_deadline"] = outer is not None and returned >= outer
            hard = retention["request_deadline_monotonic"]
            retention["return_after_request_hard_deadline"] = hard is not None and returned >= hard
            result.statistics["validation_profile_request_cumulative"] = {
                "validator": dict(getattr(self.robot_state_validator, "performance_counters", {})),
                "mesh_backend": dict(getattr(getattr(self.robot_state_validator, "mesh_robot", None), "performance_counters", {})),
                "nested_timers_overlap": True}
            return result
        finally:
            self._deadline_monotonic = outer
            self._candidate_final_deadline = outer_final

    def _plan_candidate(
        self,
        *,
        target: OBB,
        face: str,
        requested_virtual_contact: np.ndarray,
        grasp_candidates: Sequence[Mapping[str, Any]],
        home_q: Sequence[float],
        all_obstacles: Sequence[OBB],
        receiver: OBB,
        support_names: Sequence[str],
        suction: Mapping[str, Any],
        seed: int,
        history_hint=None,
    ) -> LayoutTrajectorySearchResult:
        """Lazily try strict grasp branches and stop at the first full cycle."""

        self._statistics = {name: 0 for name in self._statistics}
        if self._deadline_reached():
            return LayoutTrajectorySearchResult(False, None,
                {"reason": "PLANNING_WALL_CLOCK_DEADLINE", "stage": "request"}, (), {
                    **self._statistics, "termination": "PLANNING_WALL_CLOCK_DEADLINE",
                    "budget_policy": self.budget_evidence()})
        home = np.asarray(home_q, dtype=float)
        attempts: list[Mapping[str, Any]] = []
        last_failure: Mapping[str, Any] | None = None
        for index, candidate in enumerate(grasp_candidates[: self.budget.grasp_branches]):
            if self._deadline_reached():
                break
            q = np.asarray(candidate["q_rad"], dtype=float)
            candidate_deadline = self._deadline_monotonic
            slots = min(len(grasp_candidates), self.budget.grasp_branches)-index
            remaining_s = self._remaining_wall_time()
            branch_seconds = (None if self.budget.proof_of_concept else
                self.budget.candidate_wall_time_s/slots if remaining_s is None else remaining_s/slots)
            self._deadline_monotonic = self._limit(candidate_deadline, deadline_after(perf_counter(), branch_seconds))
            old_final = getattr(self, "_branch_final_deadline", None)
            old_identity = getattr(self, "_candidate_identity", None)
            final_limit = getattr(self, "_candidate_final_deadline", candidate_deadline)
            self._branch_final_deadline = self._limit(final_limit,
                None if final_limit is None else perf_counter() + max(0., final_limit-perf_counter())/slots)
            self._candidate_identity = hashlib.sha256(repr((candidate.get("candidate_id"), target.name,
                np.asarray(requested_virtual_contact).tolist(), q.tolist(), home.tolist(), face, suction)).encode()).hexdigest()
            try:
                branch = self._plan_branch
                if history_hint is not None:
                    from .history_adaptation import adapt_branch
                    branch = lambda **values: adapt_branch(self, hint=history_hint, **values)
                segment, failure, trace = branch(
                    target=target,
                    face=face,
                    requested_virtual_contact=self._se3(
                        requested_virtual_contact, "requested_virtual_contact"
                    ),
                    grasp_q=q,
                    home_q=home,
                    all_obstacles=all_obstacles,
                    receiver=receiver,
                    support_names=support_names,
                    suction=suction,
                    seed=int(seed) + index * 10000,
                )
                completed = self._task_completed(segment,
                    [box for box in all_obstacles if box.name != target.name] + [target], target)
                if segment is not None and not completed:
                    segment = None
                    failure = {"reason": "COMPLETE_TASK_VALIDATION_MISSING_OR_STALE", "stage": "final_validation"}
                if segment is None and self._deadline_reached():
                    failure = {**dict(failure or {}), "budget_termination": "IK_BRANCH_SLICE_EXHAUSTED"}
            finally:
                self._deadline_monotonic = candidate_deadline
                self._branch_final_deadline = old_final
                self._candidate_identity = old_identity
            attempts.append(
                {
                    "branch": index,
                    "candidate_id": candidate.get("candidate_id"),
                    "success": segment is not None,
                    "failure": failure,
                    "trace": trace,
                }
            )
            if segment is not None:
                return LayoutTrajectorySearchResult(
                    True,
                    segment,
                    None,
                    tuple(attempts),
                    {
                        **self._statistics,
                        "termination": "SUCCESS",
                        "budget_policy": self.budget_evidence(),
                    },
                )
            last_failure = failure
            if self._deadline_reached():
                break
        if not attempts and self._deadline_reached():
            last_failure = {"reason": "PLANNING_WALL_CLOCK_DEADLINE", "stage": "request"}
        elif not attempts:
            last_failure = {
                "reason": "NO_STRICT_GRASP_CANDIDATE",
                "stage": "contact",
            }
        return LayoutTrajectorySearchResult(
            False,
            None,
            last_failure,
            tuple(attempts),
            {
                **self._statistics,
                "termination": (
                    "PLANNING_WALL_CLOCK_DEADLINE"
                    if self._deadline_reached()
                    else "GRASP_BRANCH_LIMIT_REACHED"
                    if len(grasp_candidates) > len(attempts)
                    else "GRASP_CANDIDATES_EXHAUSTED"
                ),
                "budget_policy": self.budget_evidence(),
            },
        )


def build_m710_layout_trajectory_connector(
    *,
    lightweight_robot: LayoutTrajectoryRobot,
    urdf_path: str | Path,
    srdf_path: str | Path,
    package_dirs: Sequence[str | Path],
    base_transform: np.ndarray,
    flange_from_virtual_task_tcp: np.ndarray,
    flange_from_physical_contact: np.ndarray,
    ik_policy: Mapping[str, Any],
    collision_margin_m: float,
    contact_tolerance_m: float,
    joint_margin_rad: float,
    maximum_jacobian_condition: float,
    floor_z_m: float,
    right_wall_y_m: float,
    left_wall_y_m: float,
    official_radial_reach_m: float,
    radial_guard_tolerance_m: float,
    budget: LayoutTrajectoryBudget | None = None,
    collision_policy: Mapping[str, Any] | None = None,
    surface_directions_world: Mapping[str, Sequence[float]] | None = None,
    post_landing_transport: Mapping[str, Any] | None = None,
) -> LayoutTrajectoryConnectorBuildResult:
    """Build the exact optional connector and prove its FK-frame agreement.

    Missing Pinocchio/Coal, missing assets, an incomplete tool compound, or a
    disagreement between the official backend and the lightweight chain all
    return a concrete fail-closed result.  No proxy connector is substituted.
    """

    urdf = Path(urdf_path).resolve()
    srdf = Path(srdf_path).resolve()
    evidence: dict[str, Any] = {
        "schema": "m710id70_exact_layout_trajectory_backend_v1",
        "collision_contract": {
            "robot": "official_urdf_per_link_triangle_mesh_via_pinocchio_coal",
            "tool": "source_audited_rigid_structures_and_inserts_as_compound_obbs",
            "payload": "actual_fk_physical_contact_rigid_attachment",
            "fixed_body_aggregation_pairs": {
                "base_mount": ["base_link", "chassis"],
                "tool_mount": ["J6_link", "tool_rigid_*"],
            },
            "target_rigid_geometry_deleted": False,
            "engineering_margin_reduced": False,
        },
        "urdf_path": str(urdf),
        "srdf_path": str(srdf),
        "package_dirs": [str(Path(item).resolve()) for item in package_dirs],
        "kinematic_consistency_probes": [],
    }

    def unavailable(reason: str, detail: str) -> LayoutTrajectoryConnectorBuildResult:
        evidence["status"] = "UNAVAILABLE"
        evidence["failure_reason"] = reason
        evidence["detail"] = detail
        return LayoutTrajectoryConnectorBuildResult(
            connector=None,
            status="UNAVAILABLE",
            failure_reason=reason,
            evidence=evidence,
        )

    if not urdf.is_file():
        return unavailable("OFFICIAL_URDF_NOT_AVAILABLE", str(urdf))
    if not srdf.is_file():
        return unavailable("OFFICIAL_SRDF_NOT_AVAILABLE", str(srdf))
    try:
        flange_from_virtual = LayoutTrajectoryConnector._se3(
            np.asarray(flange_from_virtual_task_tcp, dtype=float),
            "flange_from_virtual_task_tcp",
        )
        flange_from_physical = LayoutTrajectoryConnector._se3(
            np.asarray(flange_from_physical_contact, dtype=float),
            "flange_from_physical_contact",
        )
    except ValueError as exc:
        return unavailable("TOOL_FRAME_CONTRACT_INVALID", str(exc))
    planner_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        dtype=float,
    )
    expected_virtual = np.eye(4)
    expected_virtual[:3, :3] = planner_rotation
    expected_virtual[0, 3] = 0.2500
    expected_physical = np.eye(4)
    expected_physical[:3, :3] = planner_rotation
    expected_physical[0, 3] = flange_from_physical[0, 3]
    if not np.allclose(flange_from_virtual, expected_virtual, atol=1e-12, rtol=0.0):
        return unavailable(
            "VIRTUAL_TASK_TCP_IDENTITY_MISMATCH",
            "the fixed virtual task TCP must remain 0.250 m along the resolved planner axis",
        )
    if not np.allclose(flange_from_physical, expected_physical, atol=1e-12, rtol=0.0) or not 0.2125 <= flange_from_physical[0, 3] <= 0.2275:
        return unavailable(
            "PHYSICAL_CONTACT_FRAME_IDENTITY_MISMATCH",
            "the physical plane must remain within the source-defined 0-15 mm cup compression range on the resolved axis",
        )
    evidence["tool_frame_contract"] = {
        "T_flange_virtual_task_tcp": flange_from_virtual.tolist(),
        "T_flange_nominal_compressed_contact": flange_from_physical.tolist(),
        "virtual_to_physical_offset_m": float(flange_from_virtual[0, 3] - flange_from_physical[0, 3]),
    }
    repository_root = _repository_root_from(urdf)
    if repository_root is None:
        return unavailable(
            "OFFICIAL_MODEL_REPOSITORY_IDENTITY_UNAVAILABLE",
            f"could not infer repository root from {urdf}",
        )
    expected_manifest = (repository_root / OFFICIAL_MODEL_MANIFEST).resolve()
    expected_urdf = (repository_root / OFFICIAL_MODEL_URDF).resolve()
    expected_srdf = (repository_root / OFFICIAL_MODEL_SRDF).resolve()
    expected_package_dir = expected_urdf.parents[2]
    if urdf != expected_urdf:
        return unavailable(
            "OFFICIAL_URDF_IDENTITY_MISMATCH",
            f"expected {expected_urdf}, got {urdf}",
        )
    if srdf != expected_srdf:
        return unavailable(
            "OFFICIAL_SRDF_IDENTITY_MISMATCH",
            f"expected {expected_srdf}, got {srdf}",
        )
    actual_package_dirs = tuple(Path(item).resolve() for item in package_dirs)
    if actual_package_dirs != (expected_package_dir,):
        return unavailable(
            "OFFICIAL_PACKAGE_SEARCH_PATH_MISMATCH",
            f"expected only {(str(expected_package_dir),)}, got "
            f"{tuple(str(item) for item in actual_package_dirs)}",
        )
    try:
        from .asset_audit import audit_m710id70_official_model

        official_audit = audit_m710id70_official_model(
            repository_root,
            expected_manifest,
        ).to_mapping()
        if official_audit.get("execution_qualified") is not True:
            return unavailable(
                "OFFICIAL_MODEL_ASSET_NOT_EXECUTION_QUALIFIED",
                "official asset audit did not qualify the robot description",
            )
        srdf_policy = _audit_official_srdf_policy(srdf)
    except (OSError, ET.ParseError, ValueError) as exc:
        return unavailable("OFFICIAL_MODEL_IDENTITY_AUDIT_FAILED", str(exc))
    evidence.update(
        repository_root=str(repository_root),
        official_model_audit=official_audit,
        official_srdf_policy=srdf_policy,
    )
    try:
        tip_from_tcp = np.asarray(
            getattr(lightweight_robot, "tip_from_tcp"), dtype=float
        )
        mesh_robot = PinocchioHppFclBackend(
            urdf,
            tip_frame="tool0",
            package_dirs=package_dirs,
            srdf_path=srdf,
            base_transform=base_transform,
            tip_from_tcp=tip_from_tcp,
            name="fanuc_m710id_70_official_mesh",
        )
    except (AttributeError, FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        message = str(exc)
        reason = (
            "PINOCCHIO_COAL_DEPENDENCY_UNAVAILABLE"
            if "Pinocchio" in message or "Coal" in message or "hpp-fcl" in message
            else "OFFICIAL_MESH_BACKEND_CONSTRUCTION_FAILED"
        )
        return unavailable(reason, message)

    expected_joint_names = tuple(f"J{index}" for index in range(1, 7))
    actual_joint_names = tuple(getattr(mesh_robot, "active_joint_names", ()))
    if actual_joint_names != expected_joint_names:
        return unavailable(
            "OFFICIAL_JOINT_ORDER_MISMATCH",
            f"expected {expected_joint_names}, got {actual_joint_names}",
        )
    if not np.allclose(
        np.asarray(mesh_robot.joint_limits),
        np.asarray(lightweight_robot.joint_limits),
        atol=1e-12,
        rtol=0.0,
    ):
        return unavailable(
            "PINOCCHIO_LIGHTWEIGHT_JOINT_LIMIT_MISMATCH",
            "the two official-URDF readers produced different joint limits",
        )

    probes = (
        np.zeros(6),
        np.asarray([0.17, -0.31, 0.24, 0.19, -0.22, 0.27]),
        np.asarray([-0.23, 0.18, -0.27, -0.31, 0.25, -0.19]),
    )
    try:
        for index, q in enumerate(probes):
            if not mesh_robot.within_limits(q) or not lightweight_robot.within_limits(q):
                return unavailable(
                    "KINEMATIC_CONSISTENCY_PROBE_OUTSIDE_LIMITS",
                    f"probe {index} is outside the official joint limits",
                )
            exact_frames = mesh_robot.named_link_frames(q)
            light_frames = lightweight_robot.named_link_frames(q)
            flange_error = float(
                np.max(np.abs(exact_frames["flange"] - light_frames["flange"]))
            )
            tcp_error = float(
                np.max(np.abs(mesh_robot.fk(q) - lightweight_robot.fk(q)))
            )
            jacobian_error = float(
                np.max(
                    np.abs(
                        mesh_robot.geometric_jacobian(q)
                        - lightweight_robot.geometric_jacobian(q)
                    )
                )
            )
            contract_tcp = light_frames["flange"] @ np.asarray(
                flange_from_virtual_task_tcp, dtype=float
            )
            frame_contract_error = float(
                np.max(np.abs(lightweight_robot.fk(q) - contract_tcp))
            )
            evidence["kinematic_consistency_probes"].append(
                {
                    "index": index,
                    "q_rad": q.tolist(),
                    "flange_pose_max_abs": flange_error,
                    "virtual_tcp_pose_max_abs": tcp_error,
                    "virtual_tcp_geometric_jacobian_max_abs": jacobian_error,
                    "flange_contract_tcp_pose_max_abs": frame_contract_error,
                }
            )
            if (
                flange_error > 1e-9
                or tcp_error > 1e-9
                or jacobian_error > 1e-9
                or frame_contract_error > 1e-9
            ):
                return unavailable(
                    "PINOCCHIO_LIGHTWEIGHT_FRAME_MISMATCH",
                    "probe "
                    f"{index}: flange={flange_error}, virtual_tcp={tcp_error}, "
                    f"jacobian={jacobian_error}, frame_contract={frame_contract_error}",
                )
    except (AttributeError, KeyError, RuntimeError, ValueError) as exc:
        return unavailable("KINEMATIC_CONSISTENCY_CHECK_FAILED", str(exc))

    try:
        from .validation_physics import urdf_collision_shapes, world_link_boxes

        shapes = urdf_collision_shapes(lightweight_robot)
        validator = ExactM710LayoutStateValidator(
            mesh_robot,
            lightweight_robot,
            collision_margin_m=collision_margin_m,
            floor_z_m=floor_z_m,
            right_wall_y_m=right_wall_y_m,
            left_wall_y_m=left_wall_y_m,
            robot_world_boxes=lambda q: world_link_boxes(
                lightweight_robot, np.asarray(q, dtype=float), shapes
            ),
            collision_policy=collision_policy,
            nominal_cup_compression_m=0.2275 - float(flange_from_physical[0, 3]),
        )
    except (AttributeError, FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        return unavailable("EXACT_STATE_VALIDATOR_CONSTRUCTION_FAILED", str(exc))

    zero_tool_boxes = lightweight_robot.tool_collision_obbs(np.zeros(6))
    tool_compound_q0 = [
        {
            "name": box.name,
            "center": box.center.tolist(),
            "half_extents": box.half_extents.tolist(),
            "rotation": box.rotation.tolist(),
        }
        for box in zero_tool_boxes
    ]
    tool_compound_sha256 = hashlib.sha256(
        json.dumps(tool_compound_q0, sort_keys=True, separators=(",", ":")).encode(
            "utf-8"
        )
    ).hexdigest()
    source_identity = {
        "urdf_sha256": _sha256_file(urdf),
        "srdf_sha256": _sha256_file(srdf),
        "official_manifest_sha256": official_audit["manifest_sha256"],
        "official_upstream_commit": official_audit["upstream_commit"],
        "collision_geometry_count": len(mesh_robot.geometry_model.geometryObjects),
        "collision_link_names": list(mesh_robot.collision_link_names),
        "rigid_tool_solid_count": len(zero_tool_boxes),
        "rigid_tool_compound_q0_sha256": tool_compound_sha256,
        "tip_from_tcp": tip_from_tcp.tolist(),
        "base_transform": np.asarray(base_transform, dtype=float).tolist(),
        "flange_from_virtual_task_tcp": np.asarray(
            flange_from_virtual_task_tcp, dtype=float
        ).tolist(),
        "flange_from_physical_contact": np.asarray(
            flange_from_physical_contact, dtype=float
        ).tolist(),
        "collision_margin_m": float(collision_margin_m),
        "contact_tolerance_m": float(contact_tolerance_m),
        "collision_policy": SimulationCollisionPolicy.from_mapping(collision_policy).to_mapping(),
        "joint_margin_rad": float(joint_margin_rad),
        "maximum_jacobian_condition": float(maximum_jacobian_condition),
        "official_radial_reach_m": float(official_radial_reach_m),
        "radial_guard_tolerance_m": float(radial_guard_tolerance_m),
        "surface_directions_world": {
            str(name): list(direction)
            for name, direction in dict(surface_directions_world or {}).items()
        },
        "floor_z_m": float(floor_z_m),
        "right_wall_y_m": float(right_wall_y_m),
        "left_wall_y_m": float(left_wall_y_m),
    }
    identity = hashlib.sha256(
        json.dumps(source_identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    connector = LayoutTrajectoryConnector(
        mesh_robot,
        validator,
        flange_from_virtual_task_tcp=flange_from_virtual_task_tcp,
        flange_from_physical_contact=flange_from_physical_contact,
        ik_policy=ik_policy,
        collision_margin_m=collision_margin_m,
        contact_tolerance_m=contact_tolerance_m,
        joint_margin_rad=joint_margin_rad,
        maximum_jacobian_condition=maximum_jacobian_condition,
        validator_identity=f"m710id70_exact_mesh_tool:{identity}",
        execution_qualified=True,
        budget=budget,
        official_radial_reach_m=official_radial_reach_m,
        radial_guard_tolerance_m=radial_guard_tolerance_m,
        collision_policy=collision_policy,
        surface_directions_world=surface_directions_world,
        tool_collision_obbs_provider=lightweight_robot.tool_all_physical_obbs,
        post_landing_transport=post_landing_transport,
    )
    evidence.update(
        status="AVAILABLE",
        failure_reason=None,
        source_identity=source_identity,
        validator_identity=connector.validator_identity,
        search_budget=connector.budget_evidence(),
    )
    return LayoutTrajectoryConnectorBuildResult(
        connector=connector,
        status="AVAILABLE",
        failure_reason=None,
        evidence=evidence,
    )


__all__ = [
    "ExactM710LayoutStateValidator",
    "LayoutTrajectoryBudget",
    "LayoutTrajectoryConnector",
    "LayoutTrajectoryConnectorBuildResult",
    "LayoutTrajectorySearchResult",
    "PhysicalContactAttachment",
    "TRAJECTORY_SEGMENT_SCHEMA",
    "TRAJECTORY_STAGES",
    "build_m710_layout_trajectory_connector",
    "validate_layout_trajectory_stage_contract",
]
