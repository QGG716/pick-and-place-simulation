"""Frozen-layout single-carton kinematic audit with a fail-closed path gate.

The confirmed M-710iD/70 layout is first built into, and verified as, a
content-addressed scene snapshot.  This module then searches strict suction
and IK candidates for the cartons that the support graph says are currently
removable.  It intentionally does not reinterpret either legacy V3 task
population and it never promotes proxy collision geometry to an executable
trajectory.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import copy
import json
import platform
from pathlib import Path
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .fanuc_m710id70 import target_pose
from .geometry import OBB
from .ik import IKCandidateStream, iter_ik_solutions
from .support import SupportRelationGraph
from .validation_motion import grasp_seed_configurations, grasp_task_set
from .validation_physics import suction_coverage, urdf_collision_shapes, world_link_boxes
from .workcell_layout import (
    LayoutValidationConfig,
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
EXPECTED_TOP_CARTONS = tuple(f"carton_l07_c{column:02d}" for column in range(5))
NO_IK_SEARCH_STATUS = "BUDGET_EXHAUSTED_NOT_INFEASIBILITY_PROOF"
EXECUTION_GATE_REASON = "EXECUTION_COLLISION_GEOMETRY_NOT_QUALIFIED"
TOOL_FRAME_SCHEMA = "m710id70_planner_tool_frames_v1"
MOTION_IMPLEMENTATION_FILES = (
    "src/unloading_sim/depalletizing.py",
    "src/unloading_sim/fanuc_m710id70.py",
    "src/unloading_sim/geometry.py",
    "src/unloading_sim/grasp.py",
    "src/unloading_sim/ik.py",
    "src/unloading_sim/layout_single_carton.py",
    "src/unloading_sim/planner.py",
    "src/unloading_sim/robot.py",
    "src/unloading_sim/robot_load/model.py",
    "src/unloading_sim/robot_load/spatial.py",
    "src/unloading_sim/robot_load/task.py",
    "src/unloading_sim/scene.py",
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
            "execution_qualified": False,
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
        "T_flange_nominal_compressed_contact": 0.2125,
    }
    flange_from_planner_rotation = np.asarray(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        dtype=float,
    )
    transforms: dict[str, np.ndarray] = {}
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
    if clocking != "PROVISIONAL_180_DEGREE_DISCREPANCY_UNRESOLVED":
        raise ValueError("the unresolved tool0 clocking discrepancy must remain explicit")
    if frame["execution_qualified"] is not False:
        raise ValueError("unresolved tool0 clocking must remain fail-closed for execution")
    return ToolFrameContract(
        transforms["T_flange_virtual_task_tcp"],
        transforms["T_flange_uncompressed_cup_plane"],
        transforms["T_flange_nominal_compressed_contact"],
        clocking,
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
    if _integer(population["expected_scene_cartons"], "expected_scene_cartons", minimum=1) != 40:
        raise ValueError("layout v1 motion audit must retain all 40 cartons")
    if _integer(population["expected_removable_cartons"], "expected_removable_cartons", minimum=1) != 5:
        raise ValueError("layout v1 initial task population must be the five removable top cartons")
    face_modes = tuple(population["face_modes"])
    if face_modes != ("front", "top", "side"):
        raise ValueError("layout v1 audit face_modes must be [front, top, side]")
    _finite(population["support_contact_tolerance_m"], "support_contact_tolerance_m", minimum=0.0)
    ratio = _finite(population["support_minimum_overlap_ratio"], "support_minimum_overlap_ratio", minimum=0.0)
    if ratio <= 0.0 or ratio > 1.0:
        raise ValueError("support_minimum_overlap_ratio must be in (0, 1]")

    receiver = _mapping(data["receiver"], "receiver")
    _keys(receiver, {"name", "fixed"}, "receiver")
    if receiver != {"name": "conveyor_transverse", "fixed": True}:
        raise ValueError("layout v1 receiver must be the fixed conveyor_transverse")

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

    tool_frames = _load_tool_frame_contract(data["tool_frame_contract"])

    suction = _mapping(data["suction"], "suction")
    _keys(
        suction,
        {
            "cup_rows",
            "cup_columns",
            "cup_pitch_m",
            "cup_radius_m",
            "minimum_sealed_cups",
            "suction_edge_margin_m",
        },
        "suction",
    )
    rows = _integer(suction["cup_rows"], "suction.cup_rows", minimum=1)
    columns = _integer(suction["cup_columns"], "suction.cup_columns", minimum=1)
    required = _integer(suction["minimum_sealed_cups"], "suction.minimum_sealed_cups", minimum=60)
    if (rows, columns, required) != (6, 12, 60):
        raise ValueError("the accepted strict suction contract is exactly 6x12 cups with 60 required")
    pitch = np.asarray(suction["cup_pitch_m"], dtype=float)
    if pitch.shape != (2,) or not np.all(np.isfinite(pitch)) or np.any(pitch <= 0.0):
        raise ValueError("suction.cup_pitch_m must contain two positive SI values")
    _finite(suction["cup_radius_m"], "suction.cup_radius_m", minimum=0.0)
    _finite(suction["suction_edge_margin_m"], "suction.suction_edge_margin_m", minimum=0.0)

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
    if len(cartons) != int(population["expected_scene_cartons"]):
        raise ValueError("frozen layout motion scene must retain exactly 40 cartons")
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
    if len(removable) != int(population["expected_removable_cartons"]):
        raise ValueError(f"expected five removable cartons, got {list(removable)}")
    if tuple(sorted(removable)) != EXPECTED_TOP_CARTONS:
        raise ValueError(f"initial removable population is not the literal top layer: {list(removable)}")
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
    checks = {
        "qualification_manifest_declared": manifest is not None,
        "robot_links_use_cad_collision_meshes": bool(shape_types) and all(item == "mesh" for item in shape_types),
        "tool_collision_geometry_execution_qualified": scene.snapshot["tool"]["geometry_status"]
        == "EXECUTION_QUALIFIED_CAD_COLLISION",
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
        "failure_reason": None if qualified else EXECUTION_GATE_REASON,
        "proxy_geometry_use": "KINEMATIC_AUDIT_ONLY" if not qualified else "NOT_APPLICABLE",
    }


def _exposed_faces(scene: FrozenLayoutMotionInput, target_name: str) -> tuple[str, ...]:
    active = tuple(box.name for box in scene.cartons)
    faces: list[str] = []
    modes = scene.policy.data["task_population"]["face_modes"]
    if "front" in modes and not scene.support_graph.face_blockers(target_name, "front", active):
        faces.append("front")
    if "top" in modes and not scene.support_graph.face_blockers(target_name, "top", active):
        faces.append("top")
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
    contact_coverage = suction_coverage(
        physical_contact,
        target,
        face,
        scene.policy.data["suction"],
        contact_tolerance,
    )
    if not contact_coverage["geometric_coverage"]:
        return {
            "reason": "TARGET_PHYSICAL_CONTACT_INVALID",
            "face": face,
            "sealed_cups": int(contact_coverage["sealed_cups"]),
            "required_cups": int(contact_coverage["required_cups"]),
            "normal_alignment": float(contact_coverage["normal_alignment"]),
        }
    tool = robot.tool_collision_obb(q)
    if tool is not None:
        for obstacle in scene.all_obstacles:
            if obstacle.name == target.name:
                # The conservative proxy includes compliant cups up to the
                # virtual TCP.  Waive that proxy overlap only after the actual
                # nominal-compressed cup plane has passed the named face's
                # strict normal, plane-distance and sealed-cup checks above.
                # Robot links and every non-target pair retain normal margins.
                continue
            if tool.intersects_obb(obstacle, margin=collision_margin):
                return {"reason": "TOOL_COLLISION", "pair": [tool.name, obstacle.name]}
        for link in links:
            if link.name != "J6_link" and tool.intersects_obb(link, margin=collision_margin):
                return {"reason": "TOOL_SELF_COLLISION", "pair": [tool.name, link.name]}

    right = float(scene.snapshot["trailer"]["right_wall_y_m"])
    left = float(scene.snapshot["trailer"]["left_wall_y_m"])
    floor = float(scene.snapshot["world"]["floor_z_m"])
    for body in [*links, *([] if tool is None else [tool])]:
        corners = body.corners()
        y_bounds = [float(np.min(corners[:, 1])), float(np.max(corners[:, 1]))]
        if y_bounds[0] < right + collision_margin or y_bounds[1] > left - collision_margin:
            return {"reason": "TRAILER_SIDE_CLEARANCE", "body": body.name, "y_bounds_m": y_bounds}
        if float(np.min(corners[:, 2])) < floor + collision_margin:
            return {"reason": "FLOOR_CLEARANCE", "body": body.name}
    return None


def _compact_coverage(coverage: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "sealed_cups": int(coverage["sealed_cups"]),
        "required_cups": int(coverage["required_cups"]),
        "geometric_coverage": bool(coverage["geometric_coverage"]),
        "normal_alignment": float(coverage["normal_alignment"]),
        "suction_force_status": coverage["suction_force_status"],
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
) -> dict[str, Any]:
    policy = scene.policy.data
    requested_coverage = suction_coverage(
        requested_physical_contact_pose,
        target,
        face,
        policy["suction"],
        float(policy["state_validity"]["contact_tolerance_m"]),
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
        "coverage": _compact_coverage(requested_coverage),
        "rng_seed": int(rng_seed),
        "strict_grasp_candidates": [],
        "path_connection_attempts": 0,
        "complete_trajectory": False,
    }
    if not requested_coverage["geometric_coverage"]:
        attempt.update(
            failure_stage="coverage",
            failure_reason="INSUFFICIENT_SEALED_CUPS",
            search_status="NOT_RUN_COVERAGE_GATE",
            ik_stream=None,
            path_search="NOT_RUN_NO_STRICT_GRASP",
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
        extra_state_valid=lambda q: _state_failure(
            scene, target, face, robot, shapes, q
        )
        is None,
    )
    for result in stream:
        actual_virtual_tcp = robot.fk(result.q)
        actual_physical_contact = physical_contact_from_virtual_tcp(
            actual_virtual_tcp,
            scene.policy.tool_frames.flange_from_virtual_task_tcp,
            scene.policy.tool_frames.flange_from_physical_contact,
        )
        actual_coverage = suction_coverage(
            actual_physical_contact,
            target,
            face,
            policy["suction"],
            float(policy["state_validity"]["contact_tolerance_m"]),
        )
        state_failure = _state_failure(scene, target, face, robot, shapes, result.q)
        if actual_coverage["geometric_coverage"] and state_failure is None:
            attempt["strict_grasp_candidates"].append(
                {
                    "candidate_id": result.search_evidence.get("candidate_id"),
                    "q_rad": result.q.tolist(),
                    "position_error_m": float(result.position_error),
                    "orientation_error_rad": float(result.orientation_error),
                    "fk_residual_frame": "virtual_task_tcp",
                    "actual_virtual_task_tcp_pose_world": actual_virtual_tcp.tolist(),
                    "actual_physical_contact_pose_world": actual_physical_contact.tolist(),
                    "actual_coverage": _compact_coverage(actual_coverage),
                    "actual_coverage_frame": "actual_physical_contact",
                    "state_validation": "PASS_PROXY_AUDIT",
                }
            )
    attempt["ik_stream"] = {**stream.evidence(), "best_failure": _best_failure(stream)}
    if not attempt["strict_grasp_candidates"]:
        attempt.update(
            failure_stage="grasp_ik",
            failure_reason="NO_IK",
            search_status=NO_IK_SEARCH_STATUS,
            path_search="NOT_RUN_NO_STRICT_GRASP",
        )
        return attempt
    if not execution_qualified:
        attempt.update(
            failure_stage="execution_collision_qualification",
            failure_reason=EXECUTION_GATE_REASON,
            search_status="STRICT_PROXY_GRASP_FOUND_EXECUTION_GATE_CLOSED",
            path_search="NOT_RUN_FAIL_CLOSED_BEFORE_COMPLETE_TRAJECTORY",
        )
        return attempt
    # This branch is deliberately explicit.  A future CAD-qualified backend
    # must add its connector here; proxy states can never fall through to a
    # claimed complete path.
    attempt.update(
        failure_stage="path_planner_not_integrated",
        failure_reason="CAD_QUALIFIED_PATH_CONNECTOR_NOT_IMPLEMENTED",
        search_status="STRICT_GRASP_FOUND",
        path_search="NOT_IMPLEMENTED",
    )
    return attempt


def run_layout_single_carton_audit(
    config_path: str | Path | LayoutMotionPolicy,
    *,
    project_root: str | Path | None = None,
) -> dict[str, Any]:
    """Audit the entire initial five-carton population deterministically."""
    policy = (
        config_path
        if isinstance(config_path, LayoutMotionPolicy)
        else load_layout_motion_policy(config_path)
    )
    root = policy.project_root if project_root is None else Path(project_root).resolve()
    scene = build_verified_motion_input(policy, root)
    execution = audit_execution_collision_geometry(scene, root)
    robot = policy.layout_validation.layout.robot()
    shapes = urdf_collision_shapes(robot)
    cartons_by_name = {box.name: box for box in scene.cartons}
    tasks: list[dict[str, Any]] = []
    base_seed = int(policy.data["ik"]["seed"])
    pose_index = 0
    for task_index, target_name in enumerate(scene.removable_cartons):
        target = cartons_by_name[target_name]
        if not np.allclose(target.rotation, np.eye(3), atol=1e-12, rtol=0.0):
            raise ValueError("layout v1 target_pose adapter requires the frozen axis-aligned cartons")
        attempts: list[dict[str, Any]] = []
        faces = _exposed_faces(scene, target_name)
        for face in faces:
            for roll in policy.data["ik"]["roll_candidates_deg"]:
                nominal_physical_contact, _ = target_pose(
                    float(target.center[0] - target.half_extents[0]),
                    float(target.center[1]),
                    float(target.center[2]),
                    2.0 * target.half_extents,
                    face,
                    int(roll),
                )
                variants = grasp_task_set(
                    nominal_physical_contact,
                    policy.data["ik"]["grasp_face_offset_candidates_m"],
                    policy.data["ik"]["grasp_tilt_candidates_rad"],
                )
                for requested_physical_contact, task_set in variants:
                    requested_virtual_task_tcp = virtual_tcp_from_physical_contact(
                        requested_physical_contact,
                        policy.tool_frames.flange_from_virtual_task_tcp,
                        policy.tool_frames.flange_from_physical_contact,
                    )
                    rng_seed = base_seed + task_index * 10000 + pose_index * 101
                    attempts.append(
                        _audit_pose(
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
                        )
                    )
                    pose_index += 1
        strict_candidates = sum(len(item["strict_grasp_candidates"]) for item in attempts)
        task_reason = (
            EXECUTION_GATE_REASON
            if strict_candidates and not execution["qualified"]
            else "NO_STRICT_GRASP_IK"
            if any(item["failure_reason"] == "NO_IK" for item in attempts)
            else "INSUFFICIENT_SEALED_CUPS"
        )
        face_counts: dict[str, dict[str, int]] = {}
        for face in faces:
            subset = [item for item in attempts if item["face"] == face]
            face_counts[face] = {
                "candidate_poses": len(subset),
                "coverage_rejected": sum(item["failure_reason"] == "INSUFFICIENT_SEALED_CUPS" for item in subset),
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
                "complete_trajectory": False,
                "failure_reason": task_reason,
            }
        )

    attempts = [attempt for task in tasks for attempt in task["attempts"]]
    streams = [attempt["ik_stream"] for attempt in attempts if attempt["ik_stream"] is not None]
    statistics = {
        "task_count": len(tasks),
        "task_success_count": 0,
        "candidate_pose_attempts": len(attempts),
        "coverage_rejected": sum(item["failure_reason"] == "INSUFFICIENT_SEALED_CUPS" for item in attempts),
        "ik_calls": len(streams),
        "ik_seed_pool_available": sum(int(item["seed_pool_available"]) for item in streams),
        "ik_seeds_attempted": sum(int(item["seeds_attempted"]) for item in streams),
        "ik_iterations_consumed": sum(int(item["iterations_consumed"]) for item in streams),
        "ik_converged_pose_results": sum(int(item["converged_pose_results"]) for item in streams),
        "ik_valid_solutions": sum(int(item["valid_solutions"]) for item in streams),
        "ik_deduplicated_candidates": sum(int(item["deduplicated_candidates"]) for item in streams),
        "path_connection_attempts": sum(int(item["path_connection_attempts"]) for item in attempts),
        "complete_trajectory_success_count": 0,
        "task_failure_counts": dict(sorted(Counter(task["failure_reason"] for task in tasks).items())),
        "candidate_failure_counts": dict(sorted(Counter(item["failure_reason"] for item in attempts).items())),
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
            "literal_expected_top_layer": list(EXPECTED_TOP_CARTONS),
            "legacy_104_and_129_denominators_used": False,
            "support_graph": scene.support_graph.audit(),
        },
        "fixed_cell_contract": {
            "receiver": scene.receiver.name,
            "lift_enabled": False,
            "conveyor_extension_enabled": False,
            "conveyor_z_optimization_enabled": False,
            "base_scan_enabled": False,
        },
        "strict_contract": {
            "minimum_sealed_cups": int(policy.data["suction"]["minimum_sealed_cups"]),
            "ik_position_tolerance_m": float(policy.data["ik"]["position_tolerance_m"]),
            "ik_orientation_tolerance_rad": float(policy.data["ik"]["orientation_tolerance_rad"]),
            "collision_margin_m": float(policy.data["state_validity"]["collision_margin_m"]),
            "joint_margin_rad": float(policy.data["state_validity"]["joint_margin_rad"]),
            "tool_frames": policy.tool_frames.evidence(),
        },
        "execution_collision_qualification": execution,
        "tasks": tasks,
        "statistics": statistics,
        "complete_trajectory_status": "FAIL_CLOSED",
        "complete_trajectory_failure_reason": (
            EXECUTION_GATE_REASON if not execution["qualified"] else "NO_COMPLETE_LAYOUT_BOUND_PATH"
        ),
    }
    result["evidence_fingerprint"] = canonical_digest(result)
    return result


def write_layout_single_carton_audit(result: Mapping[str, Any], path: str | Path) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return destination
