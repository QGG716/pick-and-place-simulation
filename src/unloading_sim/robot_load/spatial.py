"""Pose-specific rigid-body wrist loading using URDF/FK joint axes.

This module keeps four frames/points deliberately separate:

``F``
    The physical robot flange frame supplied by the URDF.
``T``
    A massless task/TCP frame rigidly attached to ``F``.
``C_tool``
    The tool centre of mass, defined in ``F`` independently of ``T``.
``C_box``
    The box centre of mass, defined in the box frame.  The box pose maps into
    ``T`` and is then composed with the FK flange pose.

The v2 model rotates every centre-of-mass inertia tensor into the world frame,
uses the tensor parallel-axis theorem, and projects the result on actual joint
axes.  Static gravity load uses ``a dot ((r-r0) cross m*g)``.  No controller
torque rating or complete vendor payload-versus-CoM diagram is inferred.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from unloading_sim.robot import URDFRobot

from .model import (
    AXIS_NAMES,
    GRAVITY_M_S2,
    LoadQualificationResult,
    RobotLoadLimits,
    ToolLoad,
    _inertia3,
    _positive,
    _vector3,
    cuboid_inertia_at_com,
)


LOAD_MODEL_VERSION = "spatial_inertia_joint_axis_v2"
RIGID_BODY_INERTIA_LAYER = "RIGID_BODY_ENGINEERING_INERTIA"
JOINT_ESTIMATE_LAYER = "POSE_SPECIFIC_GRAVITY_JOINT_LOAD"
JOINT_ESTIMATE_EVIDENCE = "ENGINEERING_ESTIMATE"
VENDOR_DIAGRAM_STATUS = "NOT_EVALUATED_VENDOR_LOAD_DIAGRAM"


def _rotation3(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 rotation matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-9) or not np.isclose(
        np.linalg.det(result), 1.0, atol=1e-9
    ):
        raise ValueError(f"{name} must be a proper orthonormal rotation matrix")
    return result


def rigid_transform(
    rotation: Sequence[Sequence[float]] | np.ndarray = np.eye(3),
    translation: Sequence[float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    """Build a homogeneous transform that maps local coordinates to parent."""
    result = np.eye(4)
    result[:3, :3] = _rotation3(rotation, "rotation")
    result[:3, 3] = _vector3(translation, "translation")
    return result


def _transform4(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 4x4 rigid transform")
    _rotation3(result[:3, :3], f"{name}.rotation")
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise ValueError(f"{name} must have homogeneous final row [0,0,0,1]")
    return result


def rotate_inertia(inertia_tensor_com: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Rotate a centre-of-mass inertia tensor into a common frame."""
    inertia = _inertia3(inertia_tensor_com, "inertia_tensor_com")
    rot = _rotation3(rotation, "rotation")
    return rot @ inertia @ rot.T


def spatial_inertia_at_point(
    inertia_tensor_com_common: np.ndarray,
    mass_kg: float,
    com_xyz_common: Sequence[float],
    reference_xyz_common: Sequence[float],
) -> np.ndarray:
    """Translate a rotated inertia tensor from its CoM to a reference point.

    Implements ``I_O = I_C + m[(r^T r)E - r r^T]``.  It intentionally does
    not apply the scalar ``I + m*r^2`` independently to three axes.
    """
    inertia = _inertia3(inertia_tensor_com_common, "inertia_tensor_com_common")
    mass = _positive(mass_kg, "mass_kg")
    offset = _vector3(com_xyz_common, "com_xyz_common") - _vector3(
        reference_xyz_common, "reference_xyz_common"
    )
    return inertia + mass * ((offset @ offset) * np.eye(3) - np.outer(offset, offset))


def inertia_about_axis(inertia_at_axis_origin: np.ndarray, axis_xyz: Sequence[float]) -> float:
    """Project a point-referenced inertia tensor on a unit rotation axis."""
    inertia = _inertia3(inertia_at_axis_origin, "inertia_at_axis_origin")
    axis = _vector3(axis_xyz, "axis_xyz")
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-15:
        raise ValueError("axis_xyz must have non-zero length")
    unit = axis / norm
    return float(unit @ inertia @ unit)


def gravity_moment_about_axis(
    mass_kg: float,
    com_xyz_common: Sequence[float],
    axis_origin_xyz_common: Sequence[float],
    axis_xyz_common: Sequence[float],
    gravity_xyz_m_s2: Sequence[float] = (0.0, 0.0, -GRAVITY_M_S2),
) -> float:
    """Return signed static gravity moment projected on one physical axis."""
    mass = _positive(mass_kg, "mass_kg")
    com = _vector3(com_xyz_common, "com_xyz_common")
    origin = _vector3(axis_origin_xyz_common, "axis_origin_xyz_common")
    axis = _vector3(axis_xyz_common, "axis_xyz_common")
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-15:
        raise ValueError("axis_xyz_common must have non-zero length")
    force = mass * _vector3(gravity_xyz_m_s2, "gravity_xyz_m_s2")
    return float((axis / norm) @ np.cross(com - origin, force))


@dataclass(frozen=True)
class BoxSpatialLoad:
    """Box mass properties and its rigid pose in the massless TCP frame.

    ``pose_box_to_tcp`` maps box-frame coordinates into TCP-frame coordinates.
    The box CoM is never inferred from the TCP position.
    """

    mass_kg: float
    size_xyz_m: np.ndarray
    com_in_box_frame_m: np.ndarray
    pose_box_to_tcp: np.ndarray
    grasp: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "mass_kg", _positive(self.mass_kg, "box.mass_kg"))
        size = _vector3(self.size_xyz_m, "box.size_xyz_m")
        if np.any(size <= 0.0):
            raise ValueError("box.size_xyz_m must be positive")
        object.__setattr__(self, "size_xyz_m", size)
        object.__setattr__(
            self,
            "com_in_box_frame_m",
            _vector3(self.com_in_box_frame_m, "box.com_in_box_frame_m"),
        )
        object.__setattr__(
            self,
            "pose_box_to_tcp",
            _transform4(self.pose_box_to_tcp, "box.pose_box_to_tcp"),
        )


@dataclass(frozen=True)
class PoseSpecificLoadCase:
    robot: RobotLoadLimits
    kinematics: URDFRobot
    tool: ToolLoad
    box: BoxSpatialLoad
    joint_pose_q: np.ndarray
    tcp_rotation_in_flange: np.ndarray = field(default_factory=lambda: np.eye(3))
    gravity_xyz_m_s2: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -GRAVITY_M_S2])
    )

    def __post_init__(self) -> None:
        q = np.asarray(self.joint_pose_q, dtype=float)
        if q.shape != (self.kinematics.dof,) or not np.all(np.isfinite(q)):
            raise ValueError(f"joint_pose_q must contain {self.kinematics.dof} finite radians")
        if not self.kinematics.within_limits(q):
            raise ValueError("joint_pose_q violates URDF joint limits")
        object.__setattr__(self, "joint_pose_q", q)
        object.__setattr__(
            self,
            "tcp_rotation_in_flange",
            _rotation3(self.tcp_rotation_in_flange, "tcp_rotation_in_flange"),
        )
        gravity = _vector3(self.gravity_xyz_m_s2, "gravity_xyz_m_s2")
        if np.linalg.norm(gravity) <= 1e-15:
            raise ValueError("gravity_xyz_m_s2 must be non-zero")
        object.__setattr__(self, "gravity_xyz_m_s2", gravity)


@dataclass(frozen=True)
class _BodyInWorld:
    mass_kg: float
    com_xyz: np.ndarray
    inertia_tensor_com: np.ndarray


def _body_in_world(
    mass_kg: float,
    com_local: np.ndarray,
    inertia_tensor_com_local: np.ndarray,
    pose_local_to_world: np.ndarray,
) -> _BodyInWorld:
    rotation = pose_local_to_world[:3, :3]
    com = rotation @ com_local + pose_local_to_world[:3, 3]
    return _BodyInWorld(
        mass_kg,
        com,
        rotate_inertia(inertia_tensor_com_local, rotation),
    )


def box_spatial_load(size_xyz_m: Sequence[float], mass_kg: float, grasp: str) -> BoxSpatialLoad:
    """Build the three deterministic study grasps without merging TCP and CoM."""
    size = _vector3(size_xyz_m, "size_xyz_m")
    depth, width, height = size
    com = np.array([0.5 * depth, 0.0, 0.0])
    if grasp == "front_center":
        rotation = np.eye(3)
        grasp_point = np.zeros(3)
    elif grasp == "front_offset":
        rotation = np.eye(3)
        grasp_point = np.array([0.0, -0.20 * width, -0.20 * height])
    elif grasp == "side_center":
        rotation = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        grasp_point = np.array([0.5 * depth, -0.5 * width, 0.0])
    else:
        raise ValueError(f"unsupported grasp {grasp!r}")
    # The contacted box-frame point coincides with the TCP origin.  This is a
    # pose relation only; the massless TCP itself contributes no mass property.
    pose = rigid_transform(rotation, -rotation @ grasp_point)
    return BoxSpatialLoad(float(mass_kg), size, com, pose, grasp)


def build_kinematics_from_limits(robot: RobotLoadLimits) -> URDFRobot:
    if robot.urdf_path is None:
        raise ValueError("robot limits config must provide kinematics.urdf_path for v2")
    return URDFRobot.from_urdf(
        robot.urdf_path,
        active_joint_names=robot.active_joint_names,
        base_link=robot.base_link,
        tip_link=robot.tip_link,
        tool_length=0.0,
        name=robot.model,
    )


def qualify_load_v2(case: PoseSpecificLoadCase) -> LoadQualificationResult:
    """Calculate pose-specific engineering quantities without vendor overclaim.

    The published numeric values are useful engineering references, but the
    public documents do not establish that these URDF joint-axis projections
    are the quantities defined by FANUC's wrist load diagram.
    """
    robot, kinematics, tool, box = case.robot, case.kinematics, case.tool, case.box
    frames = kinematics.named_link_frames(case.joint_pose_q)
    if robot.flange_link not in frames:
        raise ValueError(f"URDF chain does not contain flange link {robot.flange_link!r}")
    flange_pose = frames[robot.flange_link]
    tcp_pose_in_flange = rigid_transform(case.tcp_rotation_in_flange, tool.tcp_xyz_m)
    tcp_pose = flange_pose @ tcp_pose_in_flange
    box_pose = tcp_pose @ box.pose_box_to_tcp

    tool_body = _body_in_world(
        tool.mass_kg,
        tool.com_xyz_m,
        tool.inertia_tensor_com_kg_m2,
        flange_pose,
    )
    box_body = _body_in_world(
        box.mass_kg,
        box.com_in_box_frame_m,
        cuboid_inertia_at_com(box.mass_kg, box.size_xyz_m),
        box_pose,
    )
    bodies = (tool_body, box_body)
    total_mass = sum(body.mass_kg for body in bodies)
    combined_com = sum(body.mass_kg * body.com_xyz for body in bodies) / total_mass

    joint_frames = kinematics.joint_axis_frames(case.joint_pose_q)
    missing = [name for name in AXIS_NAMES if name not in joint_frames]
    if missing:
        raise ValueError(f"URDF active joint set lacks required wrist axes: {missing}")

    gravity_moments: list[float] = []
    inertias: list[float] = []
    axes: list[np.ndarray] = []
    origins: list[np.ndarray] = []
    inertia_tensors: dict[str, np.ndarray] = {}
    for name in AXIS_NAMES:
        origin, axis = joint_frames[name]
        axes.append(axis)
        origins.append(origin)
        inertia_tensor = sum(
            (
                spatial_inertia_at_point(
                    body.inertia_tensor_com,
                    body.mass_kg,
                    body.com_xyz,
                    origin,
                )
                for body in bodies
            ),
            start=np.zeros((3, 3)),
        )
        inertia_tensors[name] = inertia_tensor
        inertias.append(inertia_about_axis(inertia_tensor, axis))
        gravity_moments.append(
            sum(
                gravity_moment_about_axis(
                    body.mass_kg,
                    body.com_xyz,
                    origin,
                    axis,
                    case.gravity_xyz_m_s2,
                )
                for body in bodies
            )
        )

    payload_util = total_mass / robot.rated_payload_kg
    moment_util = np.abs(gravity_moments) / np.asarray(robot.allowable_moment_nm)
    inertia_util = np.asarray(inertias) / np.asarray(robot.allowable_inertia_kg_m2)
    payload_pass = payload_util <= 1.0 + 1e-12
    moment_reference_pass = bool(np.max(moment_util) <= 1.0 + 1e-12)
    inertia_reference_pass = bool(np.max(inertia_util) <= 1.0 + 1e-12)
    if not payload_pass:
        qualification = "FAIL_KNOWN_PAYLOAD"
        reasons = (f"external mass {total_mass:.3f} kg exceeds {robot.rated_payload_kg:.3f} kg",)
    elif not moment_reference_pass:
        index = int(np.argmax(moment_util))
        qualification = "POSE_SPECIFIC_GRAVITY_MOMENT_REFERENCE_EXCEEDED"
        reasons = (
            f"{AXIS_NAMES[index]} pose-specific gravity-moment engineering reference ratio is "
            f"{moment_util[index]:.3f}; manufacturer moment definition is not verified",
        )
    elif not inertia_reference_pass:
        index = int(np.argmax(inertia_util))
        qualification = "ENGINEERING_AXIS_INERTIA_REFERENCE_EXCEEDED"
        reasons = (
            f"{AXIS_NAMES[index]} rigid-body axis-inertia engineering reference ratio is "
            f"{inertia_util[index]:.3f}; vendor inertia is not evaluated",
        )
    else:
        qualification = "ENGINEERING_REFERENCES_WITHIN_PUBLIC_VALUES"
        reasons = (
            "payload passes and engineering projections are within published numeric values; "
            "vendor wrist-load qualification remains not evaluated",
        )

    criteria = {
        "manufacturer_payload_limit": "PASS" if payload_pass else "FAIL",
        "pose_specific_gravity_moment_reference": (
            "WITHIN_PUBLIC_VALUE" if moment_reference_pass else "REFERENCE_EXCEEDED"
        ),
        "manufacturer_moment_definition": "MANUFACTURER_MOMENT_DEFINITION_NOT_VERIFIED",
        "engineering_axis_inertia_screen": (
            "WITHIN_PUBLIC_VALUE" if inertia_reference_pass else "REFERENCE_EXCEEDED"
        ),
        "vendor_inertia": "VENDOR_INERTIA_NOT_EVALUATED",
        "vendor_load_diagram": VENDOR_DIAGRAM_STATUS,
    }
    intermediate: Mapping[str, Any] = {
        "load_model_version": LOAD_MODEL_VERSION,
        "model_status": "CURRENT_AFTER_REQUIRED_TESTS_PASS",
        "rigid_body_inertia_layer": RIGID_BODY_INERTIA_LAYER,
        "joint_load_layer": JOINT_ESTIMATE_LAYER,
        "joint_load_evidence": JOINT_ESTIMATE_EVIDENCE,
        "tool_inertia_evidence": tool.inertia_evidence_status,
        "tool_mass_properties_source": tool.mass_properties_source.value,
        "tool_mass_properties_reference_frame": tool.mass_properties_reference_frame,
        "joint_pose_q": case.joint_pose_q,
        "gravity_xyz_m_s2": case.gravity_xyz_m_s2,
        "flange_pose_world": flange_pose,
        "tcp_pose_world": tcp_pose,
        "tool_com_xyz": tool_body.com_xyz,
        "box_com_xyz": box_body.com_xyz,
        "combined_com_xyz": combined_com,
        "total_external_mass_kg": total_mass,
        "j4_axis_xyz": axes[0],
        "j5_axis_xyz": axes[1],
        "j6_axis_xyz": axes[2],
        "j4_origin_xyz": origins[0],
        "j5_origin_xyz": origins[1],
        "j6_origin_xyz": origins[2],
        "gravity_moment_j4": gravity_moments[0],
        "gravity_moment_j5": gravity_moments[1],
        "gravity_moment_j6": gravity_moments[2],
        "inertia_j4": inertias[0],
        "inertia_j5": inertias[1],
        "inertia_j6": inertias[2],
        "inertia_tensor_at_j4_origin": inertia_tensors["J4"],
        "inertia_tensor_at_j5_origin": inertia_tensors["J5"],
        "inertia_tensor_at_j6_origin": inertia_tensors["J6"],
        "payload_utilization": payload_util,
        "pose_gravity_moment_reference_ratio_j4_j5_j6": moment_util,
        "engineering_axis_inertia_reference_ratio_j4_j5_j6": inertia_util,
        "maximum_engineering_reference_ratio": max(
            payload_util, float(np.max(moment_util)), float(np.max(inertia_util))
        ),
        "vendor_load_diagram_status": VENDOR_DIAGRAM_STATUS,
    }
    return LoadQualificationResult(
        qualification=qualification,
        reasons=reasons,
        criteria=criteria,
        intermediate=intermediate,
        sources={
            "robot": dict(robot.source),
            "tool": dict(tool.source),
            "urdf": str(robot.urdf_path),
        },
    )
