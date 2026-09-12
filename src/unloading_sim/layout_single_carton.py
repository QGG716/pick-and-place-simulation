"""Frozen-layout single-carton search with an execution-qualified path gate.

The confirmed M-710iD/70 layout is first built into, and verified as, a
content-addressed scene snapshot.  This module then searches strict suction,
IK and bounded complete-cycle candidates for the cartons that the support
graph says are currently removable.  It intentionally does not reinterpret
either legacy V3 task population and it never promotes proxy collision
geometry to an executable trajectory.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
import copy
import json
import platform
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .fanuc_m710id70 import target_pose
from .geometry import OBB
from .collision_policy import SimulationCollisionPolicy
from .unloading_sequence import (
    RowUnloadingState, RowSequencePolicy, height_face_prior, fair_face_candidates,
    actual_tcp_approach_costs,
)
from .conveyor_placement import PlacementPolicy
from .independent_cups import (
    HOLDING_CAPACITY_ASSUMPTION,
    IDEAL_INDEPENDENT_CUPS_MODE,
    evaluate_independent_cup_geometry,
    m710_cup_array_from_mapping,
    select_ideal_independent_cups,
    select_ideal_independent_cups_from_actual_fk,
)
from .ik import IKCandidateStream, iter_ik_solutions
from .layout_trajectory import (
    LayoutTrajectoryBudget,
    LayoutTrajectoryConnector,
    LayoutTrajectoryConnectorBuildResult,
    build_m710_layout_trajectory_connector,
)
from .support import SupportRelationGraph
from .validation_motion import grasp_seed_configurations, grasp_task_set
from .validation_physics import urdf_collision_shapes, world_link_boxes
from .workcell_layout import (
    LayoutValidationConfig,
    audit_initial_state,
    audit_snapshot_consistency,
    build_scene_snapshot,
    canonical_digest,
    load_layout_validation_config,
    robot_chassis_support_contact_allowed,
    sha256_file,
    verify_scene_snapshot,
)


MOTION_SCHEMA = "m710id70_layout_single_carton_motion_v1"
RESULT_SCHEMA = "m710id70_layout_single_carton_motion_audit_v1"
EXPECTED_LAYOUT_ID = "m710id70_unloading_layout_v1"
NO_IK_SEARCH_STATUS = "BUDGET_EXHAUSTED_NOT_INFEASIBILITY_PROOF"
EXECUTION_GATE_REASON = "EXECUTION_COLLISION_GEOMETRY_NOT_QUALIFIED"
PATH_BACKEND_UNAVAILABLE_REASON = "EXECUTION_PATH_BACKEND_UNAVAILABLE"
TOOL_FRAME_SCHEMA = "m710id70_planner_tool_frames_v1"
MOTION_IMPLEMENTATION_FILES = (
    "src/unloading_sim/collision_policy.py",
    "src/unloading_sim/conveyor_placement.py",
    "src/unloading_sim/unloading_sequence.py",
    "src/unloading_sim/tool_geometry.py",
    "src/unloading_sim/depalletizing.py",
    "src/unloading_sim/fanuc_m710id70.py",
    "src/unloading_sim/geometry.py",
    "src/unloading_sim/grasp.py",
    "src/unloading_sim/independent_cups.py",
    "src/unloading_sim/ik.py",
    "src/unloading_sim/layout_single_carton.py",
    "src/unloading_sim/layout_trajectory.py",
    "src/unloading_sim/pinocchio_backend.py",
    "src/unloading_sim/planner.py",
    "src/unloading_sim/robot.py",
    "src/unloading_sim/robot_load/model.py",
    "src/unloading_sim/robot_load/spatial.py",
    "src/unloading_sim/robot_load/task.py",
    "src/unloading_sim/scene.py",
    "src/unloading_sim/serial_unloading.py",
    "src/unloading_sim/support.py",
    "src/unloading_sim/timing.py",
    "src/unloading_sim/validation_motion.py",
    "src/unloading_sim/validation_physics.py",
    "src/unloading_sim/workcell_layout.py",
)


def motion_implementation_identity(project_root: str | Path) -> dict[str, Any]:
    """Return portable hashes for every CPU module participating in this audit."""

    root = Path(project_root).resolve()
    files: dict[str, str] = {}
    for relative in MOTION_IMPLEMENTATION_FILES:
        source = root / relative
        if not source.is_file():
            raise FileNotFoundError(f"motion implementation source is missing: {relative}")
        files[relative] = sha256_file(source)
    return {
        "source_sha256": files,
        "runtime": {
            "python": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "platform": platform.platform(),
            "numpy": np.__version__,
        },
    }


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _keys(value: Mapping[str, Any], expected: set[str], name: str) -> None:
    unknown = set(value) - expected
    missing = expected - set(value)
    if unknown or missing:
        raise ValueError(f"{name} unknown={sorted(unknown)} missing={sorted(missing)}")


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    result = float(value)
    if not np.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(f"{name} must be finite and >= {minimum}")
    return result


def _integer(value: Any, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or int(value) != value or int(value) < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return int(value)


def _resolve(config_path: Path, declared: str) -> Path:
    path = Path(declared)
    result = path.resolve() if path.is_absolute() else (config_path.parent / path).resolve()
    if not result.is_file():
        raise FileNotFoundError(f"declared motion input is missing: {declared}")
    return result


def _se3(value: Any, name: str) -> np.ndarray:
    """Return a validated rigid transform, rejecting affine approximations."""

    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0):
        raise ValueError(f"{name} must have homogeneous bottom row [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0.0):
        raise ValueError(f"{name} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0.0):
        raise ValueError(f"{name} rotation must be proper")
    return transform.copy()


def virtual_tcp_from_physical_contact(
    physical_contact_pose: np.ndarray,
    flange_from_virtual_task_tcp: np.ndarray,
    flange_from_physical_contact: np.ndarray,
) -> np.ndarray:
    """Map a physical cup-plane pose to the virtual TCP used by strict IK.

    With ``F_V = T_flange_virtual`` and ``F_C = T_flange_contact``, the
    explicit frame relation is ``W_V = W_C @ inv(F_C) @ F_V``.
    """

    world_from_contact = _se3(physical_contact_pose, "physical_contact_pose")
    flange_from_virtual = _se3(
        flange_from_virtual_task_tcp, "flange_from_virtual_task_tcp"
    )
    flange_from_contact = _se3(
        flange_from_physical_contact, "flange_from_physical_contact"
    )
    return world_from_contact @ np.linalg.inv(flange_from_contact) @ flange_from_virtual


def physical_contact_from_virtual_tcp(
    virtual_task_tcp_pose: np.ndarray,
    flange_from_virtual_task_tcp: np.ndarray,
    flange_from_physical_contact: np.ndarray,
) -> np.ndarray:
    """Recover the actual physical cup plane from a virtual-TCP FK pose."""

    world_from_virtual = _se3(virtual_task_tcp_pose, "virtual_task_tcp_pose")
    flange_from_virtual = _se3(
        flange_from_virtual_task_tcp, "flange_from_virtual_task_tcp"
    )
    flange_from_contact = _se3(
        flange_from_physical_contact, "flange_from_physical_contact"
    )
    return world_from_virtual @ np.linalg.inv(flange_from_virtual) @ flange_from_contact


@dataclass(frozen=True)
class ToolFrameContract:
    flange_from_virtual_task_tcp: np.ndarray
    flange_from_uncompressed_cup_plane: np.ndarray
    flange_from_nominal_compressed_contact: np.ndarray
    tool0_clocking_status: str
    execution_qualified: bool

    @property
    def flange_from_physical_contact(self) -> np.ndarray:
        return self.flange_from_nominal_compressed_contact

    @property
    def virtual_to_physical_contact_offset_m(self) -> float:
        delta = (
            self.flange_from_virtual_task_tcp[:3, 3]
            - self.flange_from_physical_contact[:3, 3]
        )
        return float(np.linalg.norm(delta))

    def evidence(self) -> dict[str, Any]:
        return {
            "schema": TOOL_FRAME_SCHEMA,
            "transform_convention": "T_parent_child_maps_child_coordinates_into_parent",
            "planner_axes": {
                "x": "cup_array_row_tangent",
                "y": "cup_array_column_tangent",
                "z": "flange_toward_carton_inward_normal_at_contact",
            },
            "active_physical_contact_frame": "nominal_compressed_contact",
            "T_flange_virtual_task_tcp": self.flange_from_virtual_task_tcp.tolist(),
            "T_flange_uncompressed_cup_plane": self.flange_from_uncompressed_cup_plane.tolist(),
            "T_flange_nominal_compressed_contact": (
                self.flange_from_nominal_compressed_contact.tolist()
            ),
            "virtual_to_physical_contact_offset_m": self.virtual_to_physical_contact_offset_m,
            "virtual_task_tcp_role": "strict_ik_fk_residual_only_never_attachment",
            "attachment_frame_role": "actual_physical_contact_only",
            "tool0_clocking_status": self.tool0_clocking_status,
            "execution_qualified": self.execution_qualified,
        }


def _load_tool_frame_contract(value: Any) -> ToolFrameContract:
    frame = _mapping(value, "tool_frame_contract")
    _keys(
        frame,
        {
            "schema",
            "transform_convention",
            "planner_axes",
            "T_flange_virtual_task_tcp",
            "T_flange_uncompressed_cup_plane",
            "T_flange_nominal_compressed_contact",
            "active_physical_contact_frame",
            "virtual_task_tcp_role",
            "attachment_frame_role",
            "tool0_clocking_status",
            "execution_qualified",
        },
        "tool_frame_contract",
    )
    if frame["schema"] != TOOL_FRAME_SCHEMA:
        raise ValueError("unsupported tool frame contract schema")
    if frame["transform_convention"] != "T_parent_child_maps_child_coordinates_into_parent":
        raise ValueError("unsupported tool frame transform convention")
    axes = _mapping(frame["planner_axes"], "tool_frame_contract.planner_axes")
    _keys(axes, {"x", "y", "z"}, "tool_frame_contract.planner_axes")
    if dict(axes) != {
        "x": "cup_array_row_tangent",
        "y": "cup_array_column_tangent",
        "z": "flange_toward_carton_inward_normal_at_contact",
    }:
        raise ValueError("tool frame planner axes do not match the CPU contact convention")
    expected_literals = {
        "T_flange_virtual_task_tcp": 0.2500,
        "T_flange_uncompressed_cup_plane": 0.2275,
        "T_flange_nominal_compressed_contact": float(np.asarray(frame["T_flange_nominal_compressed_contact"])[0, 3]),
    }
    flange_from_planner_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        dtype=float,
    )
    transforms: dict[str, np.ndarray] = {}
    compression = expected_literals["T_flange_uncompressed_cup_plane"] - expected_literals["T_flange_nominal_compressed_contact"]
    if not 0.0 <= compression <= 0.015:
        raise ValueError("physical contact must lie within the 15 mm flexible cup compression range")
    for name, distance in expected_literals.items():
        transform = _se3(frame[name], f"tool_frame_contract.{name}")
        expected = np.eye(4)
        expected[:3, :3] = flange_from_planner_rotation
        expected[0, 3] = distance
        if not np.allclose(transform, expected, atol=1e-12, rtol=0.0):
            raise ValueError(f"{name} must retain the confirmed planner-frame transform")
        transforms[name] = transform
    if frame["active_physical_contact_frame"] != "nominal_compressed_contact":
        raise ValueError("attachment must use the nominal compressed physical contact plane")
    if frame["virtual_task_tcp_role"] != "strict_ik_fk_residual_only_never_attachment":
        raise ValueError("virtual task TCP may only be used for strict IK/FK residuals")
    if frame["attachment_frame_role"] != "actual_physical_contact_only":
        raise ValueError("attachment must be evaluated at the actual physical contact frame")
    clocking = str(frame["tool0_clocking_status"])
    if clocking != "OFFICIAL_FLANGE_TO_PROJECT_TOOL0_ADAPTER_RESOLVED":
        raise ValueError("tool0 must use the resolved official-flange project adapter")
    if frame["execution_qualified"] is not True:
        raise ValueError("the resolved full-SE(3) frame contract must be execution-qualified")
    return ToolFrameContract(
        transforms["T_flange_virtual_task_tcp"],
        transforms["T_flange_uncompressed_cup_plane"],
        transforms["T_flange_nominal_compressed_contact"],
        clocking,
        True,
    )


@dataclass(frozen=True)
class LayoutMotionPolicy:
    config_path: Path
    data: Mapping[str, Any]
    layout_validation: LayoutValidationConfig
    tool_frames: ToolFrameContract

    @property
    def project_root(self) -> Path:
        return self.config_path.parents[2]

    @property
    def policy_fingerprint(self) -> str:
        return canonical_digest(self.data)


@dataclass(frozen=True)
class FrozenLayoutMotionInput:
    policy: LayoutMotionPolicy
    snapshot: Mapping[str, Any]
    snapshot_verification: Mapping[str, Any]
    snapshot_consistency: Mapping[str, Any]
    fixed_components: tuple[OBB, ...]
    cartons: tuple[OBB, ...]
    receiver: OBB
    support_graph: SupportRelationGraph
    removable_cartons: tuple[str, ...]
    remaining_stack_names: tuple[str, ...] | None = None

    @property
    def occupied(self) -> tuple[OBB, ...]:
        """Completed cartons still physically present remain receiver obstacles."""
        if self.remaining_stack_names is None:
            return ()
        remaining = set(self.remaining_stack_names)
        return tuple(box for box in self.cartons if box.name not in remaining)

    @property
    def all_obstacles(self) -> tuple[OBB, ...]:
        # The target is not deleted here.  Contact semantics are applied only
        # to that named target during the final state check.
        return (*self.fixed_components, *self.cartons)


def load_layout_motion_policy(path: str | Path) -> LayoutMotionPolicy:
    """Load the small execution policy without importing legacy scene data."""
    config_path = Path(path).resolve()
    data = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "motion config")
    _keys(
        data,
        {
            "schema",
            "layout_validation_config",
            "task_population",
            "receiver",
            "fixed_cell",
            "tool_frame_contract",
            "suction",
            "ik",
            "state_validity",
            "execution_collision",
            "search_strategy",
        },
        "motion config",
    )
    if data["schema"] != MOTION_SCHEMA:
        raise ValueError("unsupported layout motion schema")

    population = _mapping(data["task_population"], "task_population")
    _keys(
        population,
        {
            "selector",
            "expected_scene_cartons",
            "expected_removable_cartons",
            "face_modes",
            "support_contact_tolerance_m",
            "support_minimum_overlap_ratio",
        },
        "task_population",
    )
    if population["selector"] != "support_relation_graph_removable":
        raise ValueError("layout task population must use SupportRelationGraph removability")
    _integer(population["expected_scene_cartons"], "expected_scene_cartons", minimum=1)
    _integer(population["expected_removable_cartons"], "expected_removable_cartons", minimum=1)
    face_modes = tuple(population["face_modes"])
    if face_modes != ("front", "top", "side"):
        raise ValueError("layout v1 audit face_modes must be [front, top, side]")
    _finite(population["support_contact_tolerance_m"], "support_contact_tolerance_m", minimum=0.0)
    ratio = _finite(population["support_minimum_overlap_ratio"], "support_minimum_overlap_ratio", minimum=0.0)
    if ratio <= 0.0 or ratio > 1.0:
        raise ValueError("support_minimum_overlap_ratio must be in (0, 1]")

    receiver = _mapping(data["receiver"], "receiver")
    _keys(receiver, {"name", "fixed"}, "receiver")
    if not isinstance(receiver["name"], str) or receiver["fixed"] is not True:
        raise ValueError("receiver must identify a fixed supported conveyor surface")

    fixed = _mapping(data["fixed_cell"], "fixed_cell")
    forbidden = {
        "lift_enabled",
        "conveyor_extension_enabled",
        "conveyor_z_optimization_enabled",
        "base_scan_enabled",
    }
    _keys(fixed, forbidden, "fixed_cell")
    if any(fixed[name] is not False for name in forbidden):
        raise ValueError("layout v1 forbids lift, conveyor extension/Z optimization, and base scans")

    strategy = _mapping(data["search_strategy"], "search_strategy")
    directions = _mapping(
        strategy.get("surface_directions_world"),
        "search_strategy.surface_directions_world",
    )
    expected_directions = {
        "conveyor_transverse": np.array([0.0, -1.0, 0.0]),
        "conveyor_longitudinal": np.array([-1.0, 0.0, 0.0]),
    }
    if set(directions) != set(expected_directions):
        raise ValueError("both physical conveyor surface directions must be explicit")
    for name, expected in expected_directions.items():
        direction = np.asarray(directions[name], dtype=float)
        if direction.shape != (3,) or not np.allclose(
            direction, expected, atol=1e-12, rtol=0.0
        ):
            raise ValueError(f"{name} transport direction disagrees with the dynamics contract")

    tool_frames = _load_tool_frame_contract(data["tool_frame_contract"])

    suction = _mapping(data["suction"], "suction")
    _keys(
        suction,
        {
            "mode",
            "holding_capacity_assumption",
            "enforce_vacuum_force_capacity",
            "enforce_vacuum_break_force",
            "enforce_vacuum_break_torque",
            "selection_policy",
            "require_nonempty_geometric_contact",
            "cup_rows",
            "cup_columns",
            "cup_pitch_m",
            "cup_radius_m",
            "suction_edge_margin_m",
        },
        "suction",
    )
    if suction["mode"] != IDEAL_INDEPENDENT_CUPS_MODE:
        raise ValueError("layout v1 must use ideal_independent_cups")
    if suction["holding_capacity_assumption"] != HOLDING_CAPACITY_ASSUMPTION:
        raise ValueError("ideal independent cups require the explicit holding-capacity assumption")
    for name in (
        "enforce_vacuum_force_capacity",
        "enforce_vacuum_break_force",
        "enforce_vacuum_break_torque",
    ):
        if suction[name] is not False:
            raise ValueError(f"suction.{name} must remain false for the ideal model")
    if suction["selection_policy"] != "command_all_geometrically_eligible_cups":
        raise ValueError("ideal cup selection must command all geometrically eligible cups")
    if suction["require_nonempty_geometric_contact"] is not True:
        raise ValueError("ideal cup geometry must require a non-empty contact mask")
    rows = _integer(suction["cup_rows"], "suction.cup_rows", minimum=1)
    columns = _integer(suction["cup_columns"], "suction.cup_columns", minimum=1)
    if (rows, columns) != (6, 12):
        raise ValueError("the accepted independent-cup geometry is exactly 6x12")
    pitch = np.asarray(suction["cup_pitch_m"], dtype=float)
    if pitch.shape != (2,) or not np.all(np.isfinite(pitch)) or np.any(pitch <= 0.0):
        raise ValueError("suction.cup_pitch_m must contain two positive SI values")
    _finite(suction["cup_radius_m"], "suction.cup_radius_m", minimum=0.0)
    _finite(suction["suction_edge_margin_m"], "suction.suction_edge_margin_m", minimum=0.0)
    # Also rejects any hidden load-bearing cup-count threshold and verifies all
    # 72 stable cup identities, zones, pitch and seal radius.
    m710_cup_array_from_mapping(suction)

    ik = _mapping(data["ik"], "ik")
    _keys(
        ik,
        {
            "seed",
            "roll_candidates_deg",
            "grasp_face_offset_candidates_m",
            "grasp_tilt_candidates_rad",
            "random_restarts",
            "candidate_limit",
            "max_iterations",
            "damping",
            "max_step_rad",
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "orientation_weight",
            "candidate_dedup_tolerance_rad",
            "candidate_dedup_tolerance_m",
        },
        "ik",
    )
    for name, minimum in (("random_restarts", 0), ("candidate_limit", 1), ("max_iterations", 1)):
        _integer(ik[name], f"ik.{name}", minimum=minimum)
    _integer(ik["seed"], "ik.seed")
    if tuple(int(value) for value in ik["roll_candidates_deg"]) != (0, 90, 180, 270):
        raise ValueError("strict layout audit must retain all four deterministic wrist rolls")
    for name in ("grasp_face_offset_candidates_m", "grasp_tilt_candidates_rad"):
        values = np.asarray(ik[name], dtype=float)
        if values.ndim != 1 or not len(values) or not np.all(np.isfinite(values)):
            raise ValueError(f"ik.{name} must be a nonempty finite sequence")
    for name in (
        "damping",
        "max_step_rad",
        "position_tolerance_m",
        "orientation_tolerance_rad",
        "orientation_weight",
        "candidate_dedup_tolerance_rad",
        "candidate_dedup_tolerance_m",
    ):
        _finite(ik[name], f"ik.{name}", minimum=0.0)
    if float(ik["position_tolerance_m"]) > 1e-4 or float(ik["orientation_tolerance_rad"]) > 2e-4:
        raise ValueError("layout IK tolerances may not be looser than the strict V3 FK contract")

    validity = _mapping(data["state_validity"], "state_validity")
    _keys(
        validity,
        {
            "collision_margin_m",
            "contact_tolerance_m",
            "joint_margin_rad",
            "maximum_jacobian_condition",
            "official_radial_reach_m",
            "radial_guard_tolerance_m",
        },
        "state_validity",
    )
    for name in validity:
        _finite(validity[name], f"state_validity.{name}", minimum=0.0)

    execution = _mapping(data["execution_collision"], "execution_collision")
    _keys(
        execution,
        {"require_execution_qualified_cad_collision", "qualification_manifest"},
        "execution_collision",
    )
    if execution["require_execution_qualified_cad_collision"] is not True:
        raise ValueError("complete trajectories must require execution-qualified CAD collision geometry")

    layout_validation = load_layout_validation_config(
        _resolve(config_path, str(data["layout_validation_config"]))
    )
    if layout_validation.layout.data["layout_id"] != EXPECTED_LAYOUT_ID:
        raise ValueError("motion policy is bound only to m710id70_unloading_layout_v1")
    if float(validity["collision_margin_m"]) < layout_validation.collision_margin_m:
        raise ValueError("motion collision margin may not reduce the accepted layout margin")
    if float(validity["contact_tolerance_m"]) > layout_validation.contact_tolerance_m:
        raise ValueError("motion contact tolerance may not relax the accepted layout tolerance")
    if float(validity["joint_margin_rad"]) < float(layout_validation.data["collision"]["joint_margin_rad"]):
        raise ValueError("motion joint margin may not reduce the accepted layout margin")
    return LayoutMotionPolicy(
        config_path,
        copy.deepcopy(dict(data)),
        layout_validation,
        tool_frames,
    )


def _obb_from_snapshot(record: Mapping[str, Any]) -> OBB:
    if record.get("shape") != "box":
        raise ValueError(f"unsupported frozen collision shape for {record.get('name')}: {record.get('shape')}")
    pose = np.asarray(record["pose_world"], dtype=float)
    center = np.asarray(record["center_m"], dtype=float)
    half = np.asarray(record["half_extents_m"], dtype=float)
    if pose.shape != (4, 4) or not np.allclose(pose[:3, 3], center, atol=1e-12, rtol=0.0):
        raise ValueError(f"inconsistent frozen OBB pose for {record.get('name')}")
    return OBB(center, half, pose[:3, :3], str(record["name"]), str(record["category"]))


def build_verified_motion_input(
    policy: LayoutMotionPolicy,
    project_root: str | Path | None = None,
) -> FrozenLayoutMotionInput:
    """Build and verify the frozen scene before exposing any planning input."""
    root = policy.project_root if project_root is None else Path(project_root).resolve()
    snapshot = build_scene_snapshot(policy.layout_validation)
    verification = verify_scene_snapshot(snapshot, root)
    consistency = audit_snapshot_consistency(policy.layout_validation, snapshot)
    if verification["status"] != "PASS" or consistency["status"] != "PASS":
        raise ValueError("frozen layout snapshot did not pass verification and consistency checks")
    if snapshot["layout_id"] != EXPECTED_LAYOUT_ID:
        raise ValueError("unexpected frozen layout id")

    fixed = tuple(_obb_from_snapshot(item) for item in snapshot["assembly"]["fixed_components"])
    cartons = tuple(_obb_from_snapshot(item) for item in snapshot["cartons"])
    population = policy.data["task_population"]
    receiver_name = str(policy.data["receiver"]["name"])
    receiver = next((box for box in fixed if box.name == receiver_name), None)
    if receiver is None:
        raise ValueError("the fixed transverse receiver is absent from the frozen snapshot")

    graph = SupportRelationGraph.build(
        cartons,
        contact_tolerance_m=float(population["support_contact_tolerance_m"]),
        minimum_overlap_ratio=float(population["support_minimum_overlap_ratio"]),
    )
    removable = tuple(graph.removable_cartons(face_modes=tuple(population["face_modes"])))
    row = RowUnloadingState(config=RowSequencePolicy(row_height_fraction=float(
        policy.data.get("search_strategy", {}).get("row_height_fraction", 0.05)))).rank(cartons, support_graph=graph)
    removable = tuple(candidate.name for candidate in row.candidates if candidate.name in removable)
    return FrozenLayoutMotionInput(
        policy,
        snapshot,
        verification,
        consistency,
        fixed,
        cartons,
        receiver,
        graph,
        removable,
    )


def audit_execution_collision_geometry(
    scene: FrozenLayoutMotionInput,
    project_root: str | Path | None = None,
) -> dict[str, Any]:
    """Require qualified CAD geometry; mere mesh presence is not approval."""
    root = scene.policy.project_root if project_root is None else Path(project_root).resolve()
    urdf_record = scene.snapshot["robot"]["urdf"]
    urdf_path = root / str(urdf_record["repository_path"])
    tree = ET.parse(urdf_path)
    shape_types: list[str] = []
    links_with_collision: list[str] = []
    for link in tree.getroot().findall("link"):
        collisions = link.findall("collision")
        if collisions:
            links_with_collision.append(str(link.attrib.get("name", "")))
        for collision in collisions:
            geometry = collision.find("geometry")
            children = [] if geometry is None else list(geometry)
            shape_types.extend(child.tag.split("}")[-1] for child in children)

    manifest = scene.policy.data["execution_collision"]["qualification_manifest"]
    official_audit: Mapping[str, Any] | None = None
    official_audit_failure: str | None = None
    manifest_path: Path | None = None
    if isinstance(manifest, str) and manifest:
        try:
            from .asset_audit import audit_m710id70_official_model

            manifest_path = _resolve(scene.policy.config_path, manifest)
            official_audit = audit_m710id70_official_model(
                root,
                manifest_path,
            ).to_mapping()
        except (OSError, ValueError) as exc:
            official_audit_failure = str(exc)
    expected_manifest = (
        root / "assets/robots/fanuc_m710id_70/official/provenance.yaml"
    ).resolve()
    expected_urdf = (
        root
        / "assets/robots/fanuc_m710id_70/official/"
        "fanuc_m710_description/urdf/m710id_70_official.urdf"
    ).resolve()
    checks = {
        "qualification_manifest_declared": manifest_path is not None,
        "qualification_manifest_is_fixed_official_identity": (
            manifest_path == expected_manifest
        ),
        "official_model_asset_audit_execution_qualified": (
            official_audit is not None
            and official_audit.get("execution_qualified") is True
            and official_audit.get("source_integrity") is True
            and official_audit.get("static_urdf_integrity") is True
            and official_audit.get("model_semantics") is True
        ),
        "scene_urdf_is_fixed_audited_official_identity": urdf_path.resolve()
        == expected_urdf,
        "robot_links_use_cad_collision_meshes": bool(shape_types) and all(item == "mesh" for item in shape_types),
        "tool_rigid_solid_no_false_negative_coverage_proven": scene.snapshot["tool"]["geometry_status"]
        == "CONSERVATIVE_RIGID_SOLID_COVERAGE_PROVEN",
        "robot_mounting_geometry_execution_qualified": scene.snapshot["robot"]["mounting_reference"]["status"]
        == "EXECUTION_QUALIFIED_CAD_COLLISION",
    }
    qualified = all(checks.values())
    return {
        "required_for_complete_trajectory": True,
        "qualified": qualified,
        "checks": checks,
        "robot_links_with_collision": links_with_collision,
        "robot_collision_shape_types": dict(sorted(Counter(shape_types).items())),
        "tool_geometry_status": scene.snapshot["tool"]["geometry_status"],
        "mounting_geometry_status": scene.snapshot["robot"]["mounting_reference"]["status"],
        "official_model_audit": official_audit,
        "official_model_audit_failure": official_audit_failure,
        "failure_reason": None if qualified else EXECUTION_GATE_REASON,
        "proxy_geometry_use": "DIAGNOSTIC_AND_SEARCH_REJECTION_ONLY" if not qualified else "NOT_APPLICABLE",
    }


def _exposed_faces(scene: FrozenLayoutMotionInput, target_name: str) -> tuple[str, ...]:
    active = tuple(box.name for box in scene.cartons)
    faces: list[str] = []
    modes = scene.policy.data["task_population"]["face_modes"]
    # The currently removable population is the exposed top layer.  Try the
    # direct lift-compatible face first, while retaining every configured face.
    if "top" in modes and not scene.support_graph.face_blockers(target_name, "top", active):
        faces.append("top")
    if "front" in modes and not scene.support_graph.face_blockers(target_name, "front", active):
        faces.append("front")
    if "side" in modes:
        for face in ("left", "right"):
            if not scene.support_graph.face_blockers(target_name, face, active):
                faces.append(face)
    return tuple(faces)


def _state_failure(
    scene: FrozenLayoutMotionInput,
    target: OBB,
    face: str,
    robot,
    shapes: Sequence[tuple[str, np.ndarray, np.ndarray]],
    q: np.ndarray,
) -> dict[str, Any] | None:
    """Apply proxy checks with a face-scoped physical-contact exception."""
    validity = scene.policy.data["state_validity"]
    q = np.asarray(q, dtype=float)
    if q.shape != (robot.dof,) or not np.all(np.isfinite(q)) or not robot.within_limits(q):
        return {"reason": "JOINT_LIMIT_OR_NONFINITE"}
    margin_actual = float(
        np.min(np.minimum(q - robot.joint_limits[:, 0], robot.joint_limits[:, 1] - q))
    )
    if margin_actual < float(validity["joint_margin_rad"]):
        return {"reason": "JOINT_MARGIN", "actual_margin_rad": margin_actual}
    jacobian_condition = float(np.linalg.cond(robot.geometric_jacobian(q)))
    if not np.isfinite(jacobian_condition) or jacobian_condition > float(
        validity["maximum_jacobian_condition"]
    ):
        return {"reason": "SINGULARITY", "jacobian_condition": jacobian_condition}
    flange = robot.named_link_frames(q)["flange"][:3, 3]
    local_flange = robot.base_transform[:3, :3].T @ (flange - robot.base_transform[:3, 3])
    radial = float(np.linalg.norm(local_flange[:2]))
    if radial > float(validity["official_radial_reach_m"]) + float(
        validity["radial_guard_tolerance_m"]
    ):
        return {"reason": "RADIAL_REACH", "radial_m": radial}
    self_result = robot.collision_result(q, [], check_self=True)
    if self_result.in_collision:
        return {"reason": "SELF_COLLISION", "detail": str(self_result)}

    collision_margin = float(validity["collision_margin_m"])
    contact_tolerance = float(validity["contact_tolerance_m"])
    links = world_link_boxes(robot, q, shapes)
    chassis = next(box for box in scene.fixed_components if box.name == "chassis")
    for link in links:
        for obstacle in scene.all_obstacles:
            if not link.intersects_obb(obstacle, margin=collision_margin):
                continue
            if (
                link.name in {"base_link", "J1_link"}
                and obstacle.name == chassis.name
                and robot_chassis_support_contact_allowed(link, chassis, contact_tolerance)
            ):
                continue
            return {"reason": "ROBOT_COLLISION", "pair": [link.name, obstacle.name]}

    virtual_tcp = robot.fk(q)
    physical_contact = physical_contact_from_virtual_tcp(
        virtual_tcp,
        scene.policy.tool_frames.flange_from_virtual_task_tcp,
        scene.policy.tool_frames.flange_from_physical_contact,
    )
    contact_selection, contact_geometry = _independent_cup_selection_at_pose(
        physical_contact,
        target,
        face,
        scene.policy.data["suction"],
        contact_tolerance,
        pose_source="proxy_actual_fk_physical_contact",
    )
    if contact_selection is None:
        return {
            "reason": "TARGET_PHYSICAL_CONTACT_INVALID",
            "face": face,
            "geometrically_eligible_cups": int(
                sum(contact_geometry.geometrically_eligible_mask)
            ),
            "load_bearing_minimum_cup_count": None,
        }
    tools = list(robot.tool_collision_obbs(q))
    if not tools:
        legacy_tool = robot.tool_collision_obb(q)
        tools = [] if legacy_tool is None else [legacy_tool]
    for tool in tools:
        for obstacle in scene.all_obstacles:
            if tool.intersects_obb(obstacle, margin=collision_margin):
                return {"reason": "TOOL_COLLISION", "pair": [tool.name, obstacle.name]}
        for link in links:
            if link.name not in scene.policy.layout_validation.data.get("collision_policy", {}).get("wrist_tool_exempt_links", []) and tool.intersects_obb(link, margin=collision_margin):
                return {"reason": "TOOL_SELF_COLLISION", "pair": [tool.name, link.name]}

    right = float(scene.snapshot["trailer"]["right_wall_y_m"])
    left = float(scene.snapshot["trailer"]["left_wall_y_m"])
    floor = float(scene.snapshot["world"]["floor_z_m"])
    for body in [*links, *tools]:
        corners = body.corners()
        y_bounds = [float(np.min(corners[:, 1])), float(np.max(corners[:, 1]))]
        if y_bounds[0] < right + collision_margin or y_bounds[1] > left - collision_margin:
            return {"reason": "TRAILER_SIDE_CLEARANCE", "body": body.name, "y_bounds_m": y_bounds}
        if float(np.min(corners[:, 2])) < floor + collision_margin:
            return {"reason": "FLOOR_CLEARANCE", "body": body.name}
    return None


def _scheduled_contact_poses(scene, target, faces):
    robot = scene.policy.layout_validation.layout.robot()
    shoulder = robot.named_link_frames(np.zeros(robot.dof))["J2_link"][2, 3]
    strategy = scene.policy.data.get("search_strategy", {})
    scores = height_face_prior(target, faces, shoulder_height_m=float(shoulder),
        transition_halfwidth_m=float(strategy.get("face_height_transition_m", 0.35)))
    by_face = {}
    reference_q = np.zeros(robot.dof)
    reference_virtual = robot.fk(reference_q)
    rigid_vertices_virtual = np.concatenate([
        (box.corners() - reference_virtual[:3, 3]) @ reference_virtual[:3, :3]
        for box in robot.tool_collision_obbs(reference_q)])
    physical_from_virtual = (np.linalg.inv(scene.policy.tool_frames.flange_from_physical_contact)
                             @ scene.policy.tool_frames.flange_from_virtual_task_tcp)
    target_bottom = float(np.min(target.corners()[:, 2]))
    required_margin = 2.0 * float(scene.policy.data["state_validity"]["collision_margin_m"])
    for face in faces:
        rolls = []
        for roll in scene.policy.data["ik"]["roll_candidates_deg"]:
            nominal, _ = target_pose(-float(target.half_extents[0]), 0.0, 0.0,
                                    2.0 * target.half_extents, face, int(roll))
            variants = grasp_task_set(nominal, scene.policy.data["ik"]["grasp_face_offset_candidates_m"],
                                     scene.policy.data["ik"]["grasp_tilt_candidates_rad"])
            rolls.append([(int(roll), target.world_from_local @ pose,
                           {**dict(info), "height_face_prior": scores.get(face, 0.0)}) for pose, info in variants])
        interleaved = [items[i] for i in range(max(map(len, rolls), default=0))
                       for items in rolls if i < len(items)]
        ranked = []
        for original_index, (roll, physical_pose, info) in enumerate(interleaved):
            virtual_pose = physical_pose @ physical_from_virtual
            vertices = rigid_vertices_virtual @ virtual_pose[:3, :3].T + virtual_pose[:3, 3]
            clearance = float(np.min(vertices[:, 2]) - target_bottom)
            # A whole-action soft prior: tools protruding below a box's base
            # tend to hit receiving structures. Do not discard them globally
            # (overhang may be legal); prefer existing offsets/rolls that keep
            # every rigid part above a horizontal receiving plane's margin.
            info = {**info, "rigid_tool_bottom_above_box_bottom_m": clearance,
                    "placement_clearance_prior": "PREFER_LEVEL_SUPPORT_CLEARANCE_NOT_A_HARD_GATE"}
            roll_preference = list(scene.policy.data["ik"]["roll_candidates_deg"]).index(roll)
            offset_radius = float(np.linalg.norm(info["face_offset_local_xy_m"]))
            tilt_radius = float(np.linalg.norm(info["orientation_offset_local_xy_rad"]))
            ranked.append(((clearance < required_margin, tilt_radius, offset_radius,
                            roll_preference, original_index), (roll, physical_pose, info)))
        by_face[face] = [candidate for _, candidate in sorted(ranked, key=lambda item: item[0])]
    return fair_face_candidates(by_face, scores, int(strategy.get("grasp_poses_per_task", 48)))


def _independent_cup_selection_at_pose(
    physical_contact_pose: np.ndarray,
    target: OBB,
    face: str,
    suction: Mapping[str, Any],
    contact_tolerance_m: float,
    *,
    pose_source: str,
):
    cup_array = m710_cup_array_from_mapping(suction)
    geometry = evaluate_independent_cup_geometry(
        physical_contact_pose,
        target,
        face,
        cup_array,
        max_attachment_gap_m=0.002,
        maximum_penetration_m=float(contact_tolerance_m),
        max_normal_misalignment_rad=np.deg2rad(5.0),
        suction_edge_margin_m=float(suction["suction_edge_margin_m"]),
    )
    try:
        selection = select_ideal_independent_cups(
            geometry,
            pose_source=pose_source,
        )
    except ValueError as exc:
        if "non-empty geometrically eligible command" not in str(exc):
            raise
        selection = None
    return selection, geometry


def _compact_coverage(selection, geometry) -> dict[str, Any]:
    selected = 0 if selection is None else len(selection.actual_contact_ids)
    eligible = len(geometry.geometrically_eligible_ids)
    return {
        "suction_mode": IDEAL_INDEPENDENT_CUPS_MODE,
        "geometrically_eligible_cups": int(eligible),
        "actual_contact_cups": int(selected),
        "geometric_coverage": bool(selected > 0),
        "require_nonempty_geometric_contact": True,
        "load_bearing_minimum_cup_count": None,
        "holding_capacity_assumption": HOLDING_CAPACITY_ASSUMPTION,
        "vacuum_force_capacity_enforced": False,
        "vacuum_break_force_enforced": False,
        "vacuum_break_torque_enforced": False,
    }


def _best_failure(stream: IKCandidateStream) -> dict[str, Any] | None:
    result = stream.best_failure
    if result is None:
        return None
    return {
        "q_rad": np.asarray(result.q, dtype=float).tolist(),
        "position_error_m": float(result.position_error),
        "orientation_error_rad": float(result.orientation_error),
        "iterations": int(result.iterations),
        "message": result.message,
    }


def _audit_pose(
    scene: FrozenLayoutMotionInput,
    target: OBB,
    face: str,
    roll: int,
    requested_physical_contact_pose: np.ndarray,
    requested_virtual_task_tcp_pose: np.ndarray,
    task_set: Mapping[str, Any],
    rng_seed: int,
    execution_qualified: bool,
    robot,
    shapes: Sequence[tuple[str, np.ndarray, np.ndarray]],
    exact_state_failure: Callable[[np.ndarray], Mapping[str, Any] | None] | None = None,
) -> dict[str, Any]:
    policy = scene.policy.data
    requested_selection, requested_geometry = _independent_cup_selection_at_pose(
        requested_physical_contact_pose,
        target,
        face,
        policy["suction"],
        float(policy["state_validity"]["contact_tolerance_m"]),
        pose_source="requested_physical_contact",
    )
    attempt: dict[str, Any] = {
        "face": face,
        "roll_deg": int(roll),
        "task_set": dict(task_set),
        # Compatibility alias: the old key now explicitly means virtual TCP.
        "requested_tcp_pose_world": np.asarray(
            requested_virtual_task_tcp_pose, dtype=float
        ).tolist(),
        "requested_virtual_task_tcp_pose_world": np.asarray(
            requested_virtual_task_tcp_pose, dtype=float
        ).tolist(),
        "requested_physical_contact_pose_world": np.asarray(
            requested_physical_contact_pose, dtype=float
        ).tolist(),
        "coverage_frame": "requested_physical_contact",
        "coverage": _compact_coverage(requested_selection, requested_geometry),
        "rng_seed": int(rng_seed),
        "strict_grasp_candidates": [],
        "path_connection_attempts": 0,
        "complete_trajectory": None,
    }
    if requested_selection is None:
        attempt.update(
            failure_stage="coverage",
            failure_reason="NO_GEOMETRIC_CUP_CONTACT",
            search_status="NOT_RUN_COVERAGE_GATE",
            ik_stream=None,
            path_search="NOT_RUN_NO_STRICT_GRASP",
            complete_trajectory=False,
        )
        return attempt

    ik = policy["ik"]
    seeds = grasp_seed_configurations(robot, [scene.policy.layout_validation.initial_q])
    stream = iter_ik_solutions(
        robot,
        requested_virtual_task_tcp_pose,
        seeds,
        random_restarts=int(ik["random_restarts"]),
        rng=np.random.default_rng(rng_seed),
        candidate_limit=int(ik["candidate_limit"]),
        dedup_tolerance_rad=float(ik["candidate_dedup_tolerance_rad"]),
        dedup_tolerance_m=float(ik["candidate_dedup_tolerance_m"]),
        max_iterations=int(ik["max_iterations"]),
        damping=float(ik["damping"]),
        max_step=float(ik["max_step_rad"]),
        position_tolerance=float(ik["position_tolerance_m"]),
        orientation_tolerance=float(ik["orientation_tolerance_rad"]),
        orientation_weight=float(ik["orientation_weight"]),
        collision_check_stride=int(ik["max_iterations"]) + 1,
        extra_state_valid=lambda q: (
            exact_state_failure(np.asarray(q, dtype=float))
            if exact_state_failure is not None
            else _state_failure(scene, target, face, robot, shapes, q)
        ) is None,
    )
    for result in stream:
        actual_virtual_tcp = robot.fk(result.q)
        actual_physical_contact = physical_contact_from_virtual_tcp(
            actual_virtual_tcp,
            scene.policy.tool_frames.flange_from_virtual_task_tcp,
            scene.policy.tool_frames.flange_from_physical_contact,
        )
        try:
            actual_selection = select_ideal_independent_cups_from_actual_fk(
                robot,
                result.q,
                target,
                face,
                m710_cup_array_from_mapping(policy["suction"]),
                scene.policy.tool_frames.flange_from_virtual_task_tcp,
                scene.policy.tool_frames.flange_from_physical_contact,
                max_attachment_gap_m=0.002,
                maximum_penetration_m=float(
                    policy["state_validity"]["contact_tolerance_m"]
                ),
                max_normal_misalignment_rad=np.deg2rad(5.0),
                suction_edge_margin_m=float(
                    policy["suction"]["suction_edge_margin_m"]
                ),
            )
        except ValueError as exc:
            if "non-empty geometrically eligible command" not in str(exc):
                raise
            actual_selection = None
        state_failure = (
            exact_state_failure(np.asarray(result.q, dtype=float))
            if exact_state_failure is not None
            else _state_failure(scene, target, face, robot, shapes, result.q)
        )
        if actual_selection is not None and state_failure is None:
            attempt["strict_grasp_candidates"].append(
                {
                    "candidate_id": result.search_evidence.get("candidate_id"),
                    "q_rad": result.q.tolist(),
                    "position_error_m": float(result.position_error),
                    "orientation_error_rad": float(result.orientation_error),
                    "fk_residual_frame": "virtual_task_tcp",
                    "actual_virtual_task_tcp_pose_world": actual_virtual_tcp.tolist(),
                    "actual_physical_contact_pose_world": actual_physical_contact.tolist(),
                    "actual_coverage": _compact_coverage(
                        actual_selection, actual_selection.geometry
                    ),
                    "actual_coverage_frame": "actual_physical_contact",
                    "selected_cup_ids": list(actual_selection.actual_contact_ids),
                    "state_validation": (
                        "PASS_EXECUTION_QUALIFIED_MESH_AND_RIGID_TOOL"
                        if exact_state_failure is not None
                        else "PASS_PROXY_AUDIT_ONLY"
                    ),
                }
            )
    attempt["ik_stream"] = {**stream.evidence(), "best_failure": _best_failure(stream)}
    if not attempt["strict_grasp_candidates"]:
        attempt.update(
            failure_stage="grasp_ik",
            failure_reason="NO_IK",
            search_status=NO_IK_SEARCH_STATUS,
            path_search="NOT_RUN_NO_STRICT_GRASP",
            complete_trajectory=False,
        )
        return attempt
    if not execution_qualified:
        attempt.update(
            failure_stage="execution_collision_qualification",
            failure_reason=EXECUTION_GATE_REASON,
            search_status="STRICT_PROXY_GRASP_FOUND_EXECUTION_GATE_CLOSED",
            path_search="NOT_RUN_FAIL_CLOSED_BEFORE_COMPLETE_TRAJECTORY",
            complete_trajectory=False,
        )
        return attempt
    attempt.update(
        failure_stage="trajectory_search",
        failure_reason="TRAJECTORY_SEARCH_PENDING",
        search_status="STRICT_GRASP_FOUND_TRAJECTORY_PENDING",
        path_search="PENDING_EXECUTION_QUALIFIED_CONNECTOR",
    )
    return attempt


def _build_automatic_trajectory_connector(
    scene: FrozenLayoutMotionInput | LayoutMotionPolicy,
    lightweight_robot,
) -> LayoutTrajectoryConnectorBuildResult:
    # The policy form breaks the snapshot/home-validation dependency cycle.
    # Both forms derive identical geometry from the confirmed immutable layout.
    policy = scene if isinstance(scene, LayoutMotionPolicy) else scene.policy
    layout = policy.layout_validation.layout
    robot_config_path = _resolve(
        layout.config_path, str(layout.data["robot"]["model_config"])
    )
    robot_config = _mapping(
        yaml.safe_load(robot_config_path.read_text(encoding="utf-8")),
        "robot model config",
    )
    kinematics = _mapping(robot_config.get("kinematics"), "robot model kinematics")
    urdf_path = _resolve(robot_config_path, str(kinematics["urdf_path"]))
    srdf_path = _resolve(robot_config_path, str(kinematics["srdf_path"]))
    # ``package://fanuc_m710_description`` resolves from the directory that
    # contains that package, rather than from the URDF directory itself.
    package_root = next(
        (
            parent
            for parent in urdf_path.parents
            if (parent / "package.xml").is_file()
        ),
        None,
    )
    if package_root is None:
        return LayoutTrajectoryConnectorBuildResult(
            None,
            "UNAVAILABLE",
            "OFFICIAL_URDF_PACKAGE_ROOT_NOT_AVAILABLE",
            {
                "status": "UNAVAILABLE",
                "failure_reason": "OFFICIAL_URDF_PACKAGE_ROOT_NOT_AVAILABLE",
                "urdf_path": str(urdf_path),
            },
        )
    validity = policy.data["state_validity"]
    strategy = policy.data.get("search_strategy", {})
    return build_m710_layout_trajectory_connector(
        lightweight_robot=lightweight_robot,
        urdf_path=urdf_path,
        srdf_path=srdf_path,
        package_dirs=[package_root.parent],
        base_transform=layout.robot_base_transform(),
        flange_from_virtual_task_tcp=(
            policy.tool_frames.flange_from_virtual_task_tcp
        ),
        flange_from_physical_contact=(
            policy.tool_frames.flange_from_physical_contact
        ),
        ik_policy=policy.data["ik"],
        collision_margin_m=float(validity["collision_margin_m"]),
        contact_tolerance_m=float(validity["contact_tolerance_m"]),
        joint_margin_rad=float(validity["joint_margin_rad"]),
        maximum_jacobian_condition=float(validity["maximum_jacobian_condition"]),
        floor_z_m=float(layout.data["world"]["floor_z_m"]),
        right_wall_y_m=float(layout.data["trailer"]["right_wall_y_m"]),
        left_wall_y_m=float(layout.data["trailer"]["left_wall_y_m"]),
        official_radial_reach_m=float(validity["official_radial_reach_m"]),
        radial_guard_tolerance_m=float(validity["radial_guard_tolerance_m"]),
        budget=LayoutTrajectoryBudget(
            extraction_runtime_clearance_reserve_m=float(
                strategy.get("extraction_runtime_clearance_reserve_m", 0.0)
            )
        ),
        collision_policy=policy.layout_validation.data.get("collision_policy"),
        surface_directions_world=strategy.get("surface_directions_world", {}),
    )


def _blocked_initial_state_result(
    policy: LayoutMotionPolicy,
    root: Path,
    initial: Mapping[str, Any],
) -> dict[str, Any]:
    """Record a rejected home without creating a snapshot or searching paths.

    The optional exact backend is diagnostic only here: even a passing exact
    check cannot override the rejected production initial-state predicate.
    """
    layout = policy.layout_validation.layout
    cartons = layout.cartons()
    fixed = tuple(layout.fixed_components())
    population = policy.data["task_population"]
    graph = SupportRelationGraph.build(
        cartons,
        contact_tolerance_m=float(population["support_contact_tolerance_m"]),
        minimum_overlap_ratio=float(population["support_minimum_overlap_ratio"]),
    )
    removable = tuple(graph.removable_cartons(face_modes=tuple(population["face_modes"])))
    reason = "INITIAL_STATE_INVALID"
    backend: dict[str, Any] = {"status": "NOT_RUN", "failure_reason": reason}
    diagnostic: dict[str, Any] = {
        "status": "NOT_RUN", "failure_reason": reason,
        "scope": "single_unloaded_home_state_only_no_ik_or_path_search",
        "can_override_initial_state_gate": False,
        "state_validations": 0,
    }
    try:
        built = _build_automatic_trajectory_connector(policy, layout.robot())
        backend = dict(built.evidence)
        if built.connector is not None:
            diagnostic["state_validations"] = 1
            failure = built.connector.validate_unloaded_state(
                policy.layout_validation.initial_q,
                [*fixed, *cartons],
                stage="initial_state_diagnostic",
            )
            diagnostic.update(
                status="PASS" if failure is None else "FAIL",
                failure_reason=None if failure is None else failure.get("reason"),
                failure=None if failure is None else dict(failure),
            )
        else:
            diagnostic["failure_reason"] = built.failure_reason
    except Exception as exc:
        # Preserve the primary failure and write evidence even if an optional
        # backend cannot initialize or diagnose the rejected state.
        diagnostic.update(status="ERROR", failure_reason=str(exc), exception_type=type(exc).__name__)
    zero_counters = (
        "task_success_count", "candidate_pose_attempts", "coverage_rejected",
        "ik_calls", "ik_seed_pool_available", "ik_seeds_attempted",
        "ik_iterations_consumed", "ik_converged_pose_results", "ik_valid_solutions",
        "ik_deduplicated_candidates", "path_connection_attempts", "trajectory_pose_attempts",
        "trajectory_ik_calls", "trajectory_ik_seeds_attempted", "trajectory_ik_iterations_consumed",
        "trajectory_state_validations", "trajectory_edge_validation_calls",
        "trajectory_edge_state_samples", "trajectory_rrt_iterations_consumed",
        "trajectory_cartesian_samples", "complete_trajectory_success_count",
    )
    result: dict[str, Any] = {
        "schema": RESULT_SCHEMA,
        "run_status": "BLOCKED",
        "layout_id": layout.data["layout_id"],
        "layout_fingerprint": layout.layout_fingerprint,
        "scene_fingerprint": None,
        "policy_fingerprint": policy.policy_fingerprint,
        "implementation_identity": motion_implementation_identity(root),
        "input_assets": copy.deepcopy(dict(layout.assets)),
        "effective_motion_policy": copy.deepcopy(dict(policy.data)),
        "initial_state_audit": copy.deepcopy(dict(initial)),
        "initial_state_exact_diagnostic": diagnostic,
        "snapshot_verification": {"status": "NOT_RUN", "failure_reason": reason},
        "snapshot_consistency_status": "NOT_RUN",
        "scene": {
            "source": "confirmed_layout_only_snapshot_refused_invalid_initial_state",
            "carton_count": len(cartons),
            "carton_ids": [box.name for box in cartons],
            "fixed_component_names": [box.name for box in fixed],
            "planning_obstacle_count": len(cartons) + len(fixed),
            "all_cartons_retained_during_candidate_checks": None,
        },
        "task_population": {
            "selector": "SupportRelationGraph.removable_cartons",
            "carton_ids": list(removable),
            "population_source": "actual_highest_remaining_row_and_support_graph",
            "legacy_104_and_129_denominators_used": False,
            "support_graph": graph.audit(),
        },
        "trajectory_backend": backend,
        "search": {"status": "NOT_RUN", "failure_reason": reason},
        "tasks": [{
            "task_id": name, "attempts": [], "complete_trajectory": False,
            "scene_cartons_retained": len(cartons), "failure_reason": reason,
            "search_status": "NOT_RUN_INITIAL_STATE_INVALID",
        } for name in removable],
        "statistics": {
            **dict.fromkeys(zero_counters, 0),
            "task_count": len(removable),
            "tasks_searched": 0,
            "task_failure_counts": {reason: len(removable)},
            "candidate_failure_counts": {},
            "initial_state_diagnostic_validations": diagnostic["state_validations"],
        },
        "selected_trajectory_segment": None,
        "complete_trajectory_status": "FAIL_CLOSED",
        "complete_trajectory_failure_reason": reason,
    }
    result["evidence_fingerprint"] = canonical_digest(result)
    return result


def run_layout_single_carton_audit(
    config_path: str | Path | LayoutMotionPolicy,
    *,
    project_root: str | Path | None = None,
    trajectory_connector: LayoutTrajectoryConnector | None = None,
    progress_callback: Callable[[Mapping[str, Any]], None] | None = None,
    motion_input: FrozenLayoutMotionInput | None = None,
    row_state: RowUnloadingState | None = None,
) -> dict[str, Any]:
    """Search the initial top layer and expose one replay-ready full segment.

    Unless an already-qualified connector is injected, this function attempts
    to construct the optional Pinocchio/Coal backend from the frozen official
    assets.  Dependency or consistency failures are serialized and fail closed;
    the legacy proxy audit is never used to certify a complete path.
    """
    policy = (
        config_path
        if isinstance(config_path, LayoutMotionPolicy)
        else load_layout_motion_policy(config_path)
    )
    if motion_input is not None:
        if motion_input.policy.policy_fingerprint != policy.policy_fingerprint:
            raise ValueError("actual motion input belongs to a different search policy")
        policy = motion_input.policy
    root = policy.project_root if project_root is None else Path(project_root).resolve()
    initial = (audit_initial_state(policy.layout_validation) if motion_input is None
               else dict(motion_input.snapshot["initial_state_audit"]))
    if motion_input is None and initial["status"] != "PASS":
        return _blocked_initial_state_result(policy, root, initial)
    scene = motion_input or build_verified_motion_input(policy, root)
    strategy = policy.data.get("search_strategy", {})
    sequence = row_state or RowUnloadingState(config=RowSequencePolicy(
        row_height_fraction=float(strategy.get("row_height_fraction", 0.05)),
        actual_cost_weight=float(strategy.get("actual_cost_weight", 0.15))))
    remaining = scene.cartons if scene.remaining_stack_names is None else tuple(
        box for box in scene.cartons if box.name in scene.remaining_stack_names)
    approach_budget = getattr(trajectory_connector, "budget", None) or LayoutTrajectoryBudget()
    approach_cost_evidence = actual_tcp_approach_costs(
        remaining, np.asarray(scene.snapshot["robot"]["tcp_pose_world"]),
        {box.name: _exposed_faces(scene, box.name) for box in remaining},
        pregrasp_standoff_m=approach_budget.pregrasp_standoff_m,
        virtual_contact_offset_m=policy.tool_frames.virtual_to_physical_contact_offset_m)
    row_selection = sequence.rank(remaining, support_graph=scene.support_graph,
        candidate_costs={name: item["normalized_cost"]
                         for name, item in approach_cost_evidence["candidates"].items()},
        scene_context={"q_rad": policy.layout_validation.initial_q.tolist(),
                       "receiver_occupancy": [box.name for box in scene.occupied]})
    scene = replace(scene, removable_cartons=tuple(item.name for item in row_selection.candidates))
    execution = audit_execution_collision_geometry(scene, root)
    lightweight_robot = policy.layout_validation.layout.robot()
    if trajectory_connector is None and execution["qualified"]:
        connector_build = _build_automatic_trajectory_connector(
            scene, lightweight_robot
        )
        trajectory_connector = connector_build.connector
    elif trajectory_connector is not None:
        connector_build = LayoutTrajectoryConnectorBuildResult(
            trajectory_connector,
            "INJECTED",
            None,
            {
                "status": "INJECTED",
                "validator_identity": trajectory_connector.validator_identity,
                "execution_qualified": trajectory_connector.execution_qualified,
            },
        )
    else:
        connector_build = LayoutTrajectoryConnectorBuildResult(
            None,
            "NOT_RUN",
            EXECUTION_GATE_REASON,
            {"status": "NOT_RUN", "failure_reason": EXECUTION_GATE_REASON},
        )
    robot = (
        trajectory_connector.robot
        if trajectory_connector is not None
        else lightweight_robot
    )
    if trajectory_connector is not None:
        trajectory_connector.progress_callback = progress_callback
        strategy = policy.data.get("search_strategy", {})
        trajectory_connector.placement_policy = PlacementPolicy(
            maximum_candidates=int(strategy.get("placement_candidates", 12)),
            coarse_samples_per_axis=int(strategy.get("coarse_place_samples_per_axis", 3)),
            fine_samples_per_axis=int(strategy.get("fine_place_samples_per_axis", 7)),
            contact_tolerance_m=float(policy.data["state_validity"]["contact_tolerance_m"]),
            occupancy_clearance_m=2.0 * float(policy.data["state_validity"]["collision_margin_m"]))
    if motion_input is not None:
        if trajectory_connector is None:
            raise ValueError("actual-state continuation requires the exact state validator")
        initial_failure = trajectory_connector.validate_unloaded_state(
            policy.layout_validation.initial_q, scene.all_obstacles, stage="actual_task_start")
        if initial_failure is not None:
            raise ValueError(f"ACTUAL_TASK_START_INVALID: {initial_failure}")
        initial = {"status": "PASS", "source": "ACTUAL_RIGID_BODY_STATE",
                   "validator_identity": trajectory_connector.validator_identity,
                   "q_rad": policy.layout_validation.initial_q.tolist()}
    shapes = (
        []
        if trajectory_connector is not None
        else urdf_collision_shapes(lightweight_robot)
    )
    trajectory_backend_failure = (
        connector_build.failure_reason or PATH_BACKEND_UNAVAILABLE_REASON
    )
    cartons_by_name = {box.name: box for box in scene.cartons}
    tasks: list[dict[str, Any]] = []
    selected_trajectory_segment: Mapping[str, Any] | None = None
    base_seed = int(policy.data["ik"]["seed"])
    pose_index = 0
    for task_index, target_name in enumerate(scene.removable_cartons):
        target = cartons_by_name[target_name]
        attempts: list[dict[str, Any]] = []
        trajectory_pose_attempts = 0
        task_search_budget_exhausted = False
        faces = _exposed_faces(scene, target_name)
        if selected_trajectory_segment is not None:
            tasks.append(
                {
                    "task_id": target_name,
                    "target_pose_world": target.world_from_local.tolist(),
                    "scene_cartons_retained": len(scene.cartons),
                    "exposed_faces": list(faces),
                    "attempts": [],
                    "face_summary": {},
                    "strict_grasp_candidate_count": 0,
                    "complete_trajectory": False,
                    "failure_reason": "NOT_SEARCHED_AFTER_FIRST_COMPLETE_TRAJECTORY",
                }
            )
            continue
        for face, (roll, requested_physical_contact, task_set) in _scheduled_contact_poses(scene, target, faces):
            requested_virtual_task_tcp = virtual_tcp_from_physical_contact(
                requested_physical_contact,
                policy.tool_frames.flange_from_virtual_task_tcp,
                policy.tool_frames.flange_from_physical_contact,
            )
            rng_seed = base_seed + task_index * 10000 + pose_index * 101
            attempt = _audit_pose(
                    scene,
                    target,
                    face,
                    int(roll),
                    requested_physical_contact,
                    requested_virtual_task_tcp,
                    task_set,
                    rng_seed,
                    bool(execution["qualified"]),
                    robot,
                    shapes,
                    (
                        lambda q, current_target=target: (
                            trajectory_connector.validate_unloaded_state(
                                q,
                                scene.all_obstacles,
                                target_contact=current_target,
                                stage="grasp_contact_endpoint",
                            )
                        )
                        if trajectory_connector is not None
                        else None
                    ),
                )
            attempts.append(attempt)
            if progress_callback is not None:
                progress_callback({"target": target_name, "face": face, "roll": roll,
                    "pose": pose_index, "valid_grasps": len(attempt["strict_grasp_candidates"]),
                    "failure": attempt.get("failure_reason"), "ik": attempt.get("ik_stream")})
            if attempt["strict_grasp_candidates"] and execution["qualified"]:
                if trajectory_connector is None:
                    attempt.update(
                        complete_trajectory=False,
                        failure_stage="trajectory_backend",
                        failure_reason=trajectory_backend_failure,
                        search_status="STRICT_GRASP_FOUND_PATH_BACKEND_UNAVAILABLE",
                        path_search="NOT_RUN_NO_EXECUTION_QUALIFIED_CONNECTOR",
                    )
                else:
                    if trajectory_pose_attempts >= (
                        trajectory_connector.budget.task_pose_connection_attempts
                    ):
                        task_search_budget_exhausted = True
                        attempt.update(
                            complete_trajectory=False,
                            failure_stage="task_search_budget",
                            failure_reason="TASK_POSE_CONNECTION_BUDGET_EXHAUSTED",
                            search_status="TASK_POSE_CONNECTION_BUDGET_EXHAUSTED",
                            path_search="NOT_RUN_SHARED_TASK_BUDGET_EXHAUSTED",
                        )
                        pose_index += 1
                        break
                    trajectory_pose_attempts += 1
                    support_names = sorted(
                        scene.support_graph.supported_by[target.name]
                    )
                    outcome = trajectory_connector.plan(
                        target=target,
                        face=face,
                        requested_virtual_contact=requested_virtual_task_tcp,
                        grasp_candidates=attempt["strict_grasp_candidates"],
                        home_q=policy.layout_validation.initial_q,
                        all_obstacles=scene.all_obstacles,
                        receiver=scene.receiver,
                        support_names=support_names,
                        suction=policy.data["suction"],
                        seed=rng_seed + 50000,
                    )
                    attempt["trajectory_search"] = {
                        "attempts": list(outcome.attempts),
                        "statistics": dict(outcome.statistics),
                        "failure": outcome.failure,
                    }
                    if progress_callback is not None:
                        progress_callback({"target": target_name, "face": face, "roll": roll,
                            "trajectory_success": outcome.success, "failure": outcome.failure,
                            "statistics": outcome.statistics})
                    attempt["path_connection_attempts"] = int(
                        outcome.statistics.get("connection_attempts", 0)
                    )
                    if outcome.success:
                        if outcome.segment is None:
                            raise RuntimeError(
                                "successful trajectory search did not return a segment"
                            )
                        selected_trajectory_segment = copy.deepcopy(
                            dict(outcome.segment)
                        )
                        attempt.update(
                            complete_trajectory=True,
                            failure_stage="complete",
                            failure_reason=None,
                            search_status="COMPLETE_TRAJECTORY_FOUND",
                            path_search="PASS",
                        )
                    else:
                        failure = dict(outcome.failure or {})
                        attempt.update(
                            complete_trajectory=False,
                            failure_stage=str(
                                failure.get("stage", "trajectory_search")
                            ),
                            failure_reason=str(
                                failure.get(
                                    "reason", "NO_COMPLETE_LAYOUT_BOUND_PATH"
                                )
                            ),
                            search_status=str(
                                outcome.statistics.get(
                                    "termination", "TRAJECTORY_SEARCH_EXHAUSTED"
                                )
                            ),
                            path_search="FAIL",
                        )
            pose_index += 1
            if selected_trajectory_segment is not None:
                break
            if selected_trajectory_segment is not None or task_search_budget_exhausted:
                break
        strict_candidates = sum(len(item["strict_grasp_candidates"]) for item in attempts)
        task_complete = any(item.get("complete_trajectory") is True for item in attempts)
        if task_complete:
            task_reason = "OK"
        elif task_search_budget_exhausted:
            task_reason = "TASK_POSE_CONNECTION_BUDGET_EXHAUSTED"
        elif strict_candidates and not execution["qualified"]:
            task_reason = EXECUTION_GATE_REASON
        elif strict_candidates and trajectory_connector is None:
            task_reason = trajectory_backend_failure
        elif strict_candidates:
            task_reason = next(
                (
                    str(item["failure_reason"])
                    for item in reversed(attempts)
                    if item.get("trajectory_search") is not None
                ),
                "NO_STRICT_GRASP_IK",
            )
        elif any(item["failure_reason"] == "NO_IK" for item in attempts):
            task_reason = "NO_STRICT_GRASP_IK"
        else:
            task_reason = "NO_GEOMETRIC_CUP_CONTACT"
        face_counts: dict[str, dict[str, int]] = {}
        for face in faces:
            subset = [item for item in attempts if item["face"] == face]
            face_counts[face] = {
                "candidate_poses": len(subset),
                "coverage_rejected": sum(item["failure_reason"] == "NO_GEOMETRIC_CUP_CONTACT" for item in subset),
                "no_ik": sum(item["failure_reason"] == "NO_IK" for item in subset),
                "strict_grasp_candidates": sum(len(item["strict_grasp_candidates"]) for item in subset),
            }
        tasks.append(
            {
                "task_id": target_name,
                "target_pose_world": target.world_from_local.tolist(),
                "scene_cartons_retained": len(scene.cartons),
                "exposed_faces": list(faces),
                "attempts": attempts,
                "face_summary": face_counts,
                "strict_grasp_candidate_count": strict_candidates,
                "trajectory_pose_attempts": trajectory_pose_attempts,
                "trajectory_pose_attempt_limit": (
                    None
                    if trajectory_connector is None
                    else trajectory_connector.budget.task_pose_connection_attempts
                ),
                "complete_trajectory": task_complete,
                "failure_reason": task_reason,
            }
        )
        if not task_complete:
            sequence.record_failure(target_name, row_selection.scene_fingerprint, task_reason)

    attempts = [attempt for task in tasks for attempt in task["attempts"]]
    streams = [attempt["ik_stream"] for attempt in attempts if attempt["ik_stream"] is not None]
    trajectory_statistics = [
        attempt["trajectory_search"]["statistics"]
        for attempt in attempts
        if attempt.get("trajectory_search") is not None
    ]
    successful_tasks = sum(task["complete_trajectory"] for task in tasks)
    statistics = {
        "task_count": len(tasks),
        "task_success_count": successful_tasks,
        "candidate_pose_attempts": len(attempts),
        "coverage_rejected": sum(item["failure_reason"] == "NO_GEOMETRIC_CUP_CONTACT" for item in attempts),
        "ik_calls": len(streams),
        "ik_seed_pool_available": sum(int(item["seed_pool_available"]) for item in streams),
        "ik_seeds_attempted": sum(int(item["seeds_attempted"]) for item in streams),
        "ik_iterations_consumed": sum(int(item["iterations_consumed"]) for item in streams),
        "ik_converged_pose_results": sum(int(item["converged_pose_results"]) for item in streams),
        "ik_valid_solutions": sum(int(item["valid_solutions"]) for item in streams),
        "ik_deduplicated_candidates": sum(int(item["deduplicated_candidates"]) for item in streams),
        "path_connection_attempts": sum(int(item["path_connection_attempts"]) for item in attempts),
        "trajectory_pose_attempts": sum(
            int(task.get("trajectory_pose_attempts", 0)) for task in tasks
        ),
        "trajectory_pose_attempt_budget_scope": "per_task_shared_across_faces_rolls_and_task_set_variants",
        "trajectory_pose_attempt_limit_per_task": (
            None
            if trajectory_connector is None
            else trajectory_connector.budget.task_pose_connection_attempts
        ),
        "trajectory_ik_calls": sum(int(item.get("ik_calls", 0)) for item in trajectory_statistics),
        "trajectory_ik_seeds_attempted": sum(
            int(item.get("ik_seeds_attempted", 0)) for item in trajectory_statistics
        ),
        "trajectory_ik_iterations_consumed": sum(
            int(item.get("ik_iterations_consumed", 0)) for item in trajectory_statistics
        ),
        "trajectory_state_validations": sum(
            int(item.get("state_validations", 0)) for item in trajectory_statistics
        ),
        "trajectory_edge_validation_calls": sum(
            int(item.get("edge_validation_calls", 0)) for item in trajectory_statistics
        ),
        "trajectory_edge_state_samples": sum(
            int(item.get("edge_state_samples", 0)) for item in trajectory_statistics
        ),
        "trajectory_rrt_iterations_consumed": sum(
            int(item.get("rrt_iterations_consumed", 0)) for item in trajectory_statistics
        ),
        "trajectory_cartesian_samples": sum(
            int(item.get("cartesian_samples", 0)) for item in trajectory_statistics
        ),
        "complete_trajectory_success_count": successful_tasks,
        "task_failure_counts": dict(
            sorted(
                Counter(
                    task["failure_reason"]
                    for task in tasks
                    if task["failure_reason"] != "OK"
                ).items()
            )
        ),
        "candidate_failure_counts": dict(
            sorted(
                Counter(
                    "OK" if item.get("failure_reason") is None else item["failure_reason"]
                    for item in attempts
                ).items()
            )
        ),
    }
    result: dict[str, Any] = {
        "schema": RESULT_SCHEMA,
        "run_status": "COMPLETED",
        "layout_id": scene.snapshot["layout_id"],
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "policy_fingerprint": policy.policy_fingerprint,
        "implementation_identity": motion_implementation_identity(root),
        "snapshot_verification": scene.snapshot_verification,
        "snapshot_consistency_status": scene.snapshot_consistency["status"],
        "initial_state_audit": initial,
        "scene": {
            "source": "build_then_verify_content_addressed_snapshot",
            "carton_count": len(scene.cartons),
            "fixed_component_names": [box.name for box in scene.fixed_components],
            "planning_obstacle_count": len(scene.all_obstacles),
            "all_cartons_retained_during_candidate_checks": True,
        },
        "task_population": {
            "selector": "SupportRelationGraph.removable_cartons",
            "carton_ids": list(scene.removable_cartons),
            "population_source": "actual_highest_remaining_row_and_support_graph",
            "legacy_104_and_129_denominators_used": False,
            "support_graph": scene.support_graph.audit(),
            "row_selection": {**row_selection.as_dict(), "actual_cost_evidence": approach_cost_evidence,
                              "actual_cost_weight": sequence.config.actual_cost_weight},
        },
        "fixed_cell_contract": {
            "receiver": scene.receiver.name,
            "lift_enabled": False,
            "conveyor_extension_enabled": False,
            "conveyor_z_optimization_enabled": False,
            "base_scan_enabled": False,
        },
        "strict_contract": {
            "suction_mode": IDEAL_INDEPENDENT_CUPS_MODE,
            "require_nonempty_geometric_contact": True,
            "load_bearing_minimum_cup_count": None,
            "holding_capacity_assumption": HOLDING_CAPACITY_ASSUMPTION,
            "vacuum_force_capacity_enforced": False,
            "vacuum_break_force_enforced": False,
            "vacuum_break_torque_enforced": False,
            "ik_position_tolerance_m": float(policy.data["ik"]["position_tolerance_m"]),
            "ik_orientation_tolerance_rad": float(policy.data["ik"]["orientation_tolerance_rad"]),
            "collision_margin_m": float(policy.data["state_validity"]["collision_margin_m"]),
            "joint_margin_rad": float(policy.data["state_validity"]["joint_margin_rad"]),
            "tool_frames": policy.tool_frames.evidence(),
        },
        "execution_collision_qualification": execution,
        "trajectory_backend": dict(connector_build.evidence),
        "tasks": tasks,
        "statistics": statistics,
        "selected_trajectory_segment": selected_trajectory_segment,
        "complete_trajectory_status": (
            "PASS" if selected_trajectory_segment is not None else "FAIL_CLOSED"
        ),
        "complete_trajectory_failure_reason": (
            None
            if selected_trajectory_segment is not None
            else EXECUTION_GATE_REASON
            if not execution["qualified"]
            else trajectory_backend_failure
            if trajectory_connector is None
            else "NO_COMPLETE_LAYOUT_BOUND_PATH"
        ),
    }
    result["evidence_fingerprint"] = canonical_digest(result)
    return result


def write_layout_single_carton_audit(result: Mapping[str, Any], path: str | Path) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return destination
