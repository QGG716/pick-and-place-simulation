"""Bounded complete-cycle connector for the frozen M-710 unloading layout.

This module owns search order and stage semantics, but not a collision
approximation.  A caller must supply a validator backed by the execution
collision model (official per-link meshes plus the rigid-tool compound).  The
separation is deliberate: a proxy robot can still be used for a kinematic
audit, but can never be promoted to an executable trajectory by this class.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from .depalletizing import minimum_clearance_extraction_distance
from .geometry import OBB, rotation_matrix_from_rotation_vector, rotation_vector_from_matrix
from .ik import iter_ik_solutions, pose_error, solve_ik_multistart
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
    if not isinstance(ranges, Mapping) or set(ranges) != set(TRAJECTORY_STAGES):
        raise ValueError("complete trajectory must contain exactly the eight required stages")
    previous_end = 0
    for stage in TRAJECTORY_STAGES:
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
    if not isinstance(support, Mapping) or support.get("supported") is not True:
        raise ValueError("trajectory place endpoint must pass the actual-pose support audit")
    actual_box_pose = LayoutTrajectoryConnector._se3(
        np.asarray(place.get("actual_box_pose_world")),
        "place.actual_box_pose_world",
    )
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

    grasp_branches: int = 4
    task_pose_connection_attempts: int = 12
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
    pregrasp_standoff_m: float = 0.10
    preplace_standoff_m: float = 0.10
    withdrawal_distance_m: float = 0.10
    extraction_scan_step_m: float = 0.01
    maximum_extraction_m: float = 0.80
    extraction_direction_attempts: int = 5

    def __post_init__(self) -> None:
        integer_names = (
            "grasp_branches",
            "task_pose_connection_attempts",
            "stage_ik_candidates",
            "stage_connection_attempts",
            "stage_connection_iterations",
            "extraction_direction_attempts",
            "cartesian_max_samples_per_stage",
        )
        for name in integer_names:
            value = getattr(self, name)
            if isinstance(value, bool) or int(value) != value or int(value) <= 0:
                raise ValueError(f"{name} must be a positive integer")
        positive_names = (
            "rrt_step_rad",
            "edge_resolution_rad",
            "cartesian_step_m",
            "cartesian_orientation_step_rad",
            "cartesian_max_branch_step_rad",
            "pregrasp_standoff_m",
            "preplace_standoff_m",
            "withdrawal_distance_m",
            "extraction_scan_step_m",
            "maximum_extraction_m",
        )
        for name in positive_names:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0.0 <= float(self.rrt_goal_bias) <= 1.0:
            raise ValueError("rrt_goal_bias must be in [0, 1]")


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
    """Compose official robot meshes with the 58-solid rigid-tool compound.

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
        tool_boxes = list(tool_transform_robot.tool_collision_obbs(zero))
        if len(tool_boxes) != 58:
            raise ValueError(
                "execution validator requires all 58 CAD-derived rigid-tool solids"
            )

    @staticmethod
    def _collision_failure(result, reason: str) -> Mapping[str, Any] | None:
        if not result.in_collision:
            return None
        return {
            "reason": reason,
            "collision_kind": str(result.reason),
            "pair": [result.first_link, result.first_obstacle],
        }

    def _plane_failure(
        self, q: np.ndarray, payload: OBB | None
    ) -> Mapping[str, Any] | None:
        bodies = [*self.robot_world_boxes(q), *self.tool_transform_robot.tool_collision_obbs(q)]
        if payload is not None:
            bodies.append(payload)
        for body in bodies:
            corners = body.corners()
            y_min = float(np.min(corners[:, 1]))
            y_max = float(np.max(corners[:, 1]))
            z_min = float(np.min(corners[:, 2]))
            if (
                y_min < self.right_wall_y_m + self.collision_margin_m
                or y_max > self.left_wall_y_m - self.collision_margin_m
            ):
                return {
                    "reason": "TRAILER_SIDE_CLEARANCE",
                    "body": body.name,
                    "y_bounds_m": [y_min, y_max],
                }
            if z_min < self.floor_z_m + self.collision_margin_m:
                return {
                    "reason": "FLOOR_CLEARANCE",
                    "body": body.name,
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
        del target_contact, stage  # A named contact never deletes rigid geometry.
        q = np.asarray(q, dtype=float)
        names = [box.name for box in obstacles]
        if len(names) != len(set(names)):
            return {"reason": "DUPLICATE_OBSTACLE_NAME"}
        environment = list(obstacles)
        if payload is not None and payload.name not in set(names):
            environment.append(payload)
        result = self.mesh_robot.collision_result(
            q,
            environment,
            margin=self.collision_margin_m,
            ignored_geometry_obstacle_pairs={
                ("base_link", self.base_support_obstacle_name)
            },
            check_self=True,
        )
        failure = self._collision_failure(result, "ROBOT_MESH_COLLISION")
        if failure is not None:
            return failure

        tool_boxes = list(self.tool_transform_robot.tool_collision_obbs(q))
        if len(tool_boxes) != 58:
            return {
                "reason": "RIGID_TOOL_COMPOUND_INCOMPLETE",
                "actual_solid_count": len(tool_boxes),
                "required_solid_count": 58,
            }
        for tool in tool_boxes:
            for obstacle in environment:
                if tool.intersects_obb(obstacle, margin=self.collision_margin_m):
                    return {
                        "reason": "RIGID_TOOL_COLLISION",
                        "pair": [tool.name, obstacle.name],
                    }

        # The only robot/tool exception is the immutable mounting pair.  All
        # other official link meshes retain the normal engineering margin.
        result = self.mesh_robot.collision_result(
            q,
            tool_boxes,
            margin=self.collision_margin_m,
            ignored_geometry_obstacle_pairs={
                (self.tool_mount_link_name, box.name) for box in tool_boxes
            },
            check_self=False,
        )
        failure = self._collision_failure(result, "ROBOT_RIGID_TOOL_COLLISION")
        if failure is not None:
            return failure
        return self._plane_failure(q, payload)


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
    ) -> None:
        self.robot = robot
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
        self.budget = budget or LayoutTrajectoryBudget()
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
        }

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
            "task_pose_scope": "shared_by_one_task_across_faces_rolls_and_task_set_variants",
            "grasp_branches_per_pose": self.budget.grasp_branches,
            "stage_ik_candidates": self.budget.stage_ik_candidates,
            "stage_connection_attempts": self.budget.stage_connection_attempts,
            "stage_connection_iterations": self.budget.stage_connection_iterations,
            "stage_connection_scope": "shared_across_lazy_ik_candidates_for_one_stage_endpoint",
            "cartesian_policy": "one_continuation_seed_per_bounded_cartesian_sample",
            "cartesian_max_samples_per_stage": self.budget.cartesian_max_samples_per_stage,
            "extraction_direction_attempts": self.budget.extraction_direction_attempts,
            "maximum_extraction_m": self.budget.maximum_extraction_m,
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

    def _state_failure(
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
        self._statistics["state_validations"] += 1
        q_array = np.asarray(q, dtype=float)
        if q_array.shape != (6,) or not np.all(np.isfinite(q_array)):
            return {"reason": "JOINT_VECTOR_INVALID", "stage": stage}
        if not self.robot.within_limits(q_array):
            return {"reason": "JOINT_LIMIT", "stage": stage}
        limit_margin = float(
            np.min(
                np.minimum(
                    q_array - np.asarray(self.robot.joint_limits)[:, 0],
                    np.asarray(self.robot.joint_limits)[:, 1] - q_array,
                )
            )
        )
        if limit_margin < self.joint_margin_rad:
            return {
                "reason": "JOINT_MARGIN",
                "stage": stage,
                "actual_margin_rad": limit_margin,
            }
        condition = float(np.linalg.cond(self.robot.geometric_jacobian(q_array)))
        if not np.isfinite(condition) or condition > self.maximum_jacobian_condition:
            return {
                "reason": "SINGULARITY",
                "stage": stage,
                "jacobian_condition": condition,
            }
        if self.official_radial_reach_m is not None:
            frames = self.robot.named_link_frames(q_array)
            if "flange" not in frames:
                return {"reason": "FLANGE_FRAME_MISSING", "stage": stage}
            flange = np.asarray(frames["flange"], dtype=float)[:3, 3]
            base = np.asarray(self.robot.base_transform, dtype=float)
            local_flange = base[:3, :3].T @ (flange - base[:3, 3])
            radial = float(np.linalg.norm(local_flange[:2]))
            limit = self.official_radial_reach_m + self.radial_guard_tolerance_m
            if radial > limit:
                return {
                    "reason": "RADIAL_REACH",
                    "stage": stage,
                    "radial_m": radial,
                    "limit_m": limit,
                }

        payload = None if attachment is None else attachment.box_at(q_array)
        backend_failure = self.robot_state_validator(
            q_array,
            tuple(obstacles),
            payload=payload,
            target_contact=target_contact,
            stage=stage,
        )
        if backend_failure is not None:
            return {**dict(backend_failure), "stage": stage}

        if payload is None:
            return None
        if initial_proximity is not None:
            proximity_failure = initial_proximity.state_failure(
                payload, list(obstacles), support_names
            )
            if proximity_failure is not None:
                return {**proximity_failure, "stage": stage}
            return None
        support_set = set(support_names)
        for obstacle in obstacles:
            if obstacle.name in support_set and contact_separated(
                payload, obstacle, self.contact_tolerance_m
            ):
                continue
            if payload.intersects_obb(obstacle, margin=self.collision_margin_m):
                return {
                    "reason": "PAYLOAD_COLLISION",
                    "pair": [payload.name, obstacle.name],
                    "stage": stage,
                }
        return None

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
                    stage=stage,
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
        state = lambda q: self._state_failure(
            q,
            obstacles,
            attachment=attachment,
            support_names=support_names,
            target_contact=target_contact,
            stage=stage,
        ) is None
        planner = RRTConnectPlanner(
            np.asarray(self.robot.joint_limits)[:, 0],
            np.asarray(self.robot.joint_limits)[:, 1],
            state,
            step_size=self.budget.rrt_step_rad,
            edge_resolution=0.5 * self.budget.edge_resolution_rad,
            max_iterations=int(iteration_budget),
            goal_bias=self.budget.rrt_goal_bias,
            rng=np.random.default_rng(seed),
        )
        result = planner.plan(np.asarray(start, dtype=float), np.asarray(goal, dtype=float))
        self._statistics["connection_attempts"] += 1
        self._statistics["rrt_iterations_consumed"] += int(result.iterations)
        evidence = {
            "stage": stage,
            "seed": int(seed),
            "iteration_budget": int(iteration_budget),
            "success": bool(result.success),
            **dict(result.search_evidence),
        }
        if not result.success:
            return [], {
                "reason": "PATH_SEARCH_EXHAUSTED",
                "stage": stage,
                "detail": result.message,
                "iterations": int(result.iterations),
            }, evidence
        failure = self._path_failure(
            result.path,
            obstacles,
            attachment=attachment,
            support_names=support_names,
            target_contact=target_contact,
            stage=stage,
        )
        return [np.asarray(q, dtype=float) for q in result.path], failure, evidence

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
    ):
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
            extra_state_valid=lambda q: self._state_failure(
                q,
                obstacles,
                attachment=attachment,
                support_names=support_names,
                target_contact=target_contact,
                stage=f"{stage}_ik_endpoint",
            ) is None,
            collision_check_stride=int(self.ik["max_iterations"]) + 1,
        )
        return stream

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
        remaining = self.budget.stage_connection_iterations
        selected: np.ndarray | None = None
        selected_path: list[np.ndarray] = []
        failure: Mapping[str, Any] | None = None
        for index in range(self.budget.stage_connection_attempts):
            if remaining <= 0:
                break
            try:
                candidate = next(stream)
            except StopIteration:
                break
            slots = self.budget.stage_connection_attempts - index
            allocation = max(1, int(np.ceil(remaining / slots)))
            path, candidate_failure, connection = self._transit(
                start,
                candidate.q,
                obstacles,
                seed=connection_seed + index * 1009,
                iteration_budget=allocation,
                attachment=attachment,
                support_names=support_names,
                target_contact=target_contact,
                stage=stage,
            )
            consumed = int(connection.get("planning_iterations_consumed", connection.get("iterations", 0)))
            remaining = max(0, remaining - consumed)
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
                selected = candidate.q.copy()
                selected_path = path
                failure = None
                break
            failure = candidate_failure
        stream_evidence = stream.evidence()
        self._statistics["ik_calls"] += 1
        self._statistics["ik_seeds_attempted"] += int(stream_evidence.get("seeds_attempted", 0))
        self._statistics["ik_iterations_consumed"] += int(stream_evidence.get("iterations_consumed", 0))
        if selected is not None:
            termination = "SUCCESS"
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
            fraction = index / count
            pose = origin.copy()
            pose[:3, 3] = origin[:3, 3] + fraction * (
                destination[:3, 3] - origin[:3, 3]
            )
            pose[:3, :3] = (
                rotation_matrix_from_rotation_vector(rotation_vector * fraction)
                @ origin[:3, :3]
            )
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
                return path, {
                    "reason": "NO_IK",
                    "stage": stage,
                    **sample,
                }, {"stage": stage, "samples": samples}
            branch_step = float(np.max(np.abs(ik.q - path[-1])))
            sample["maximum_joint_step_rad"] = branch_step
            if branch_step > self.budget.cartesian_max_branch_step_rad:
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
        return selection.to_dict()

    def _initial_proximity(
        self,
        target: OBB,
        obstacles: Sequence[OBB],
        support_names: Sequence[str],
    ) -> tuple[InitialProximityTracker, Mapping[str, Any] | None]:
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
        clearance = 2.0 * self.collision_margin_m + 2.0 * self.contact_tolerance_m
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
                if released.intersects_obb(
                    by_name[name], margin=self.collision_margin_m
                )
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

    def _extraction(
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
            direction /= np.linalg.norm(direction)
            if not any(np.allclose(direction, other, atol=1e-12, rtol=0.0) for other in unique):
                unique.append(direction)
        constraints = [
            obstacle
            for obstacle in obstacles
            if obstacle.category == "carton" and obstacle.name != box.name
        ]
        attempts: list[dict[str, Any]] = []
        free_clearance = 2.0 * self.collision_margin_m + self.contact_tolerance_m
        for index, direction in enumerate(unique[: self.budget.extraction_direction_attempts]):
            distance = minimum_clearance_extraction_distance(
                box,
                direction,
                constraints,
                free_space_clearance_m=free_clearance,
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
            path, failure, search = self._cartesian(
                start,
                destination,
                obstacles,
                seed=seed + index * 101,
                attachment=attachment,
                initial_proximity=branch_tracker,
                stage="extraction",
            )
            released = failure is None and branch_tracker.fully_released
            attempt = {
                "index": index,
                "direction_world": direction.tolist(),
                "distance_m": float(distance),
                "search": search,
                "initial_proximity": branch_tracker.evidence(),
                "status": "ACCEPTED" if released else "REJECTED",
                "failure": failure,
            }
            attempts.append(attempt)
            if released:
                return path, branch_tracker, None, {
                    "stage": "extraction",
                    "attempts": attempts,
                    "selected_attempt": index,
                }
        failure = {
            "reason": "NO_BOUNDED_EXTRACTION_PATH",
            "stage": "extraction",
            "attempts": attempts,
        }
        return [], tracker, failure, {
            "stage": "extraction",
            "attempts": attempts,
            "selected_attempt": None,
        }

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

    def _plan_branch(
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
        outward = -actual_physical[:3, 2]
        pregrasp_physical = actual_physical.copy()
        pregrasp_physical[:3, 3] += outward * self.budget.pregrasp_standoff_m
        pregrasp_virtual = self.virtual_from_physical(pregrasp_physical)
        pre_q, pregrasp, failure, evidence = self._connect_pose(
            pregrasp_virtual,
            [grasp_q, home_q],
            home_q,
            all_obstacles,
            ik_seed=seed + 10,
            connection_seed=seed + 20,
            stage="pregrasp",
        )
        trace["stages"]["pregrasp"] = evidence
        if pre_q is None or failure is not None:
            return None, failure, trace

        contact, failure, evidence = self._cartesian(
            pre_q,
            requested_virtual_contact,
            all_obstacles,
            seed=seed + 30,
            target_contact=target,
            stage="contact",
        )
        trace["stages"]["contact"] = evidence
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

        extraction, released_tracker, failure, evidence = self._extraction(
            support_release[-1],
            attachment,
            payload_obstacles,
            tracker,
            outward,
            seed=seed + 50,
        )
        trace["stages"]["extraction"] = evidence
        if failure is not None:
            return None, failure, trace

        desired_box = target.world_from_local.copy()
        desired_box[:3, 3] = [
            receiver.center[0],
            receiver.center[1],
            receiver.center[2] + receiver.half_extents[2] + target.half_extents[2],
        ]
        desired_physical = desired_box @ np.linalg.inv(rigid.tcp_from_box)
        preplace_physical = desired_physical.copy()
        preplace_physical[2, 3] += self.budget.preplace_standoff_m
        preplace_virtual = self.virtual_from_physical(preplace_physical)
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

        place_virtual = self.virtual_from_physical(desired_physical)
        place, failure, evidence = self._cartesian(
            preplace_q,
            place_virtual,
            payload_obstacles,
            seed=seed + 80,
            attachment=attachment,
            support_names=[receiver.name],
            stage="place",
        )
        trace["stages"]["place"] = evidence
        if failure is not None:
            return None, failure, trace
        placed = attachment.box_at(place[-1])
        support = support_audit(
            placed,
            receiver,
            self.contact_tolerance_m,
            self.collision_margin_m,
        )
        if not support["supported"]:
            return None, {
                "reason": "ACTUAL_FK_SUPPORT_FAILED",
                "stage": "place",
                "support": support,
            }, trace

        place_physical = attachment.physical_contact_pose(place[-1])
        withdrawal_virtual = self.robot.fk(place[-1]).copy()
        withdrawal_virtual[:3, 3] -= (
            place_physical[:3, 2] * self.budget.withdrawal_distance_m
        )
        withdrawal_obstacles = [*payload_obstacles, placed]
        withdrawal, failure, evidence = self._cartesian(
            place[-1],
            withdrawal_virtual,
            withdrawal_obstacles,
            seed=seed + 90,
            target_contact=placed,
            stage="withdrawal",
        )
        trace["stages"]["withdrawal"] = evidence
        if failure is not None:
            return None, failure, trace

        full = [home_q.copy()]
        stages: dict[str, list[int]] = {"home": [0, 0]}
        self._append_stage(full, stages, "pregrasp", pregrasp)
        self._append_stage(full, stages, "contact", contact)
        grasp_index = stages["contact"][1]
        self._append_stage(full, stages, "support-release", support_release)
        self._append_stage(full, stages, "extraction", extraction)
        self._append_stage(full, stages, "transit", transit)
        self._append_stage(full, stages, "place", place)
        release_index = stages["place"][1]
        self._append_stage(full, stages, "withdrawal", withdrawal)
        release_retreat_index = stages["withdrawal"][1]
        segment: dict[str, Any] = {
            "schema": TRAJECTORY_SEGMENT_SCHEMA,
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
                "actual_box_pose_world": placed.world_from_local.tolist(),
                "release_center_world_m": placed.center.tolist(),
                "place_surface": receiver.name,
                "support": support,
                "receiver": receiver.name,
            },
            "validation": {
                "validator_identity": self.validator_identity,
                "execution_qualified": True,
                "collision_margin_per_body_m": self.collision_margin_m,
                "contact_tolerance_m": self.contact_tolerance_m,
                "initial_proximity": released_tracker.evidence(),
            },
        }
        trace["selected"] = {
            "grasp_index": int(grasp_index),
            "release_index": int(release_index),
            "release_retreat_index": int(release_retreat_index),
        }
        validate_layout_trajectory_stage_contract(segment)
        return segment, None, trace

    def plan(
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
    ) -> LayoutTrajectorySearchResult:
        """Lazily try strict grasp branches and stop at the first full cycle."""

        self._statistics = {name: 0 for name in self._statistics}
        home = np.asarray(home_q, dtype=float)
        attempts: list[Mapping[str, Any]] = []
        last_failure: Mapping[str, Any] | None = None
        for index, candidate in enumerate(grasp_candidates[: self.budget.grasp_branches]):
            q = np.asarray(candidate["q_rad"], dtype=float)
            segment, failure, trace = self._plan_branch(
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
        if not attempts:
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
                "termination": "GRASP_BRANCH_LIMIT_REACHED"
                if len(grasp_candidates) > len(attempts)
                else "GRASP_CANDIDATES_EXHAUSTED",
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
            "tool": "58_step_rigid_solid_bounds_as_compound_obbs",
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
    expected_physical[0, 3] = 0.2125
    if not np.allclose(flange_from_virtual, expected_virtual, atol=1e-12, rtol=0.0):
        return unavailable(
            "VIRTUAL_TASK_TCP_IDENTITY_MISMATCH",
            "the fixed virtual task TCP must remain 0.250 m along the resolved planner axis",
        )
    if not np.allclose(flange_from_physical, expected_physical, atol=1e-12, rtol=0.0):
        return unavailable(
            "PHYSICAL_CONTACT_FRAME_IDENTITY_MISMATCH",
            "the nominal compressed physical plane must remain 0.2125 m along the resolved planner axis",
        )
    evidence["tool_frame_contract"] = {
        "T_flange_virtual_task_tcp": flange_from_virtual.tolist(),
        "T_flange_nominal_compressed_contact": flange_from_physical.tolist(),
        "virtual_to_physical_offset_m": 0.0375,
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
        "joint_margin_rad": float(joint_margin_rad),
        "maximum_jacobian_condition": float(maximum_jacobian_condition),
        "official_radial_reach_m": float(official_radial_reach_m),
        "radial_guard_tolerance_m": float(radial_guard_tolerance_m),
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
