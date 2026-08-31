"""Six-axis industrial robot kinematics and capsule collision model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Protocol, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import Capsule, OBB, capsules_collide, make_transform, rotation_matrix_from_rpy


def _dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _axis_angle_transform(axis: np.ndarray, value: float, joint_type: str) -> np.ndarray:
    t = np.eye(4)
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("joint axis must be non-zero")
    axis = axis / norm
    if joint_type == "prismatic":
        t[:3, 3] = axis * value
        return t
    x, y, z = axis
    c, s = np.cos(value), np.sin(value)
    c1 = 1.0 - c
    t[:3, :3] = np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ],
        dtype=float,
    )
    return t


def _parse_xyz(text: str | None, default: Sequence[float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    return np.asarray([float(v) for v in text.split()], dtype=float)


@dataclass
class CollisionResult:
    in_collision: bool
    reason: str = ""
    first_link: str | None = None
    first_obstacle: str | None = None


class RobotBackend(Protocol):
    """Kinematics and collision contract for an arbitrary-DOF robot."""

    name: str
    joint_limits: np.ndarray
    base_transform: np.ndarray

    @property
    def dof(self) -> int: ...

    def clamp(self, q: np.ndarray) -> np.ndarray: ...

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-9) -> bool: ...

    def frames(self, q: np.ndarray, include_tool: bool = True) -> list[np.ndarray]: ...

    def fk(self, q: np.ndarray) -> np.ndarray: ...

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray: ...

    def link_elevation_degrees(self, q: np.ndarray, link_index: int) -> float: ...

    def link_capsules(self, q: np.ndarray) -> list[Capsule]: ...

    def collision_result(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.015,
        ignored_obstacle_names: set[str] | None = None,
        check_self: bool = True,
    ) -> CollisionResult: ...

    def is_collision_free(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.015,
        ignored_obstacle_names: set[str] | None = None,
    ) -> bool: ...


# Compatibility alias for callers that still import the original name.
RobotKinematics6 = RobotBackend


class DHRobot6:
    """Generic 6R robot using standard Denavit-Hartenberg parameters."""

    def __init__(
        self,
        a: Sequence[float],
        alpha: Sequence[float],
        d: Sequence[float],
        joint_offsets: Sequence[float],
        joint_limits: np.ndarray,
        link_radii: Sequence[float],
        base_transform: np.ndarray | None = None,
        tool_length: float = 0.16,
        name: str = "industrial_6r",
    ) -> None:
        self.a = np.asarray(a, dtype=float)
        self.alpha = np.asarray(alpha, dtype=float)
        self.d = np.asarray(d, dtype=float)
        self.joint_offsets = np.asarray(joint_offsets, dtype=float)
        self.joint_limits = np.asarray(joint_limits, dtype=float)
        self.link_radii = np.asarray(link_radii, dtype=float)
        self.base_transform = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
        self.tool_length = float(tool_length)
        self.name = name
        if any(x.shape != (6,) for x in (self.a, self.alpha, self.d, self.joint_offsets, self.link_radii)):
            raise ValueError("DH arrays and link radii must each have six elements")
        if self.joint_limits.shape != (6, 2):
            raise ValueError("joint_limits must have shape (6,2)")

    @property
    def dof(self) -> int:
        return 6

    @classmethod
    def ur5e_like(
        cls,
        base_position: Iterable[float] = (-0.55, 0.0, 0.62),
        base_rpy: Iterable[float] = (0.0, 0.0, 0.0),
        tool_length: float = 0.18,
    ) -> "DHRobot6":
        """Create a UR5e-scale model suitable for algorithm prototyping.

        The dimensions follow publicly known UR-family scale conventions, but
        this is deliberately a geometric planning model rather than a certified
        manufacturer model.  Replace the DH table with the selected production
        arm before hardware deployment.
        """
        roll, pitch, yaw = [float(v) for v in base_rpy]
        base = make_transform(rotation_matrix_from_rpy(roll, pitch, yaw), base_position)
        # Standard-DH dimensions in metres, close to a UR5e-sized 6R arm.
        a = [0.0, -0.4250, -0.3922, 0.0, 0.0, 0.0]
        alpha = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]
        d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
        offsets = [0.0] * 6
        limits = np.array(
            [
                [-2.0 * np.pi, 2.0 * np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
            ]
        )
        radii = [0.078, 0.068, 0.060, 0.052, 0.044, 0.038]
        return cls(a, alpha, d, offsets, limits, radii, base, tool_length, name="ur5e_like")

    def clamp(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-9) -> bool:
        q = np.asarray(q, dtype=float)
        return bool(
            np.all(q >= self.joint_limits[:, 0] - tolerance)
            and np.all(q <= self.joint_limits[:, 1] + tolerance)
        )

    def frames(self, q: np.ndarray, include_tool: bool = True) -> list[np.ndarray]:
        """Return base frame, six post-joint frames, and optionally tool frame."""
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"q must be shape ({self.dof},)")
        result = [self.base_transform.copy()]
        t = self.base_transform.copy()
        for i in range(6):
            t = t @ _dh_transform(self.a[i], self.alpha[i], self.d[i], q[i] + self.joint_offsets[i])
            result.append(t.copy())
        if include_tool:
            tool = t.copy()
            tool[:3, 3] += t[:3, 2] * self.tool_length
            result.append(tool)
        return result

    def fk(self, q: np.ndarray) -> np.ndarray:
        return self.frames(q, include_tool=True)[-1]

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray:
        frames = self.frames(q, include_tool=True)
        p_end = frames[-1][:3, 3]
        j = np.zeros((6, 6), dtype=float)
        # Joint i rotates around z-axis of frame before applying joint i.
        for i in range(6):
            origin = frames[i][:3, 3]
            axis = frames[i][:3, 2]
            j[:3, i] = np.cross(axis, p_end - origin)
            j[3:, i] = axis
        return j

    def link_elevation_degrees(self, q: np.ndarray, link_index: int) -> float:
        frames = self.frames(q, include_tool=False)
        if not 1 <= link_index < len(frames):
            raise ValueError(f"link_index must be in [1, {len(frames) - 1}]")
        vector = frames[link_index][:3, 3] - frames[link_index - 1][:3, 3]
        return float(np.degrees(np.arctan2(vector[2], np.linalg.norm(vector[:2]))))

    def link_capsules(self, q: np.ndarray) -> list[Capsule]:
        frames = self.frames(q, include_tool=True)
        points = [f[:3, 3] for f in frames]
        capsules: list[Capsule] = []
        # Six structural capsules plus a thinner flange-to-tool capsule.
        for i in range(6):
            capsules.append(Capsule(points[i], points[i + 1], float(self.link_radii[i]), f"link_{i + 1}"))
        capsules.append(Capsule(points[6], points[7], 0.034, "tool"))
        return capsules

    def collision_result(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.015,
        ignored_obstacle_names: set[str] | None = None,
        check_self: bool = True,
    ) -> CollisionResult:
        if not self.within_limits(q):
            return CollisionResult(True, "joint_limit")
        ignored = ignored_obstacle_names or set()
        capsules = self.link_capsules(q)
        for capsule in capsules:
            for obstacle in obstacles:
                if obstacle.name in ignored:
                    continue
                if capsule.collides_obb(obstacle, margin=margin):
                    return CollisionResult(True, "robot_obstacle", capsule.name, obstacle.name)
        if check_self:
            for i, a in enumerate(capsules):
                for j in range(i + 2, len(capsules)):
                    # Adjacent links share a joint; link 1/link 3 proximity near
                    # the shoulder is normal for this capsule approximation.
                    if (i, j) in {(0, 2), (4, 6)}:
                        continue
                    if capsules_collide(a, capsules[j], margin=0.003):
                        return CollisionResult(True, "self_collision", a.name, capsules[j].name)
        return CollisionResult(False)

    def is_collision_free(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.015,
        ignored_obstacle_names: set[str] | None = None,
    ) -> bool:
        return not self.collision_result(q, obstacles, margin, ignored_obstacle_names).in_collision

    @property
    def max_reach(self) -> float:
        return float(np.sum(np.abs(self.a)) + np.sum(np.abs(self.d)) + self.tool_length)


@dataclass(frozen=True)
class URDFJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    limit: tuple[float, float]


class URDFRobot:
    """Arbitrary-DOF serial robot parsed from a URDF link/joint chain."""

    def __init__(
        self,
        joints: Sequence[URDFJoint],
        active_joint_names: Sequence[str],
        base_link: str,
        tip_link: str,
        base_transform: np.ndarray | None = None,
        tool_length: float = 0.16,
        link_radii: Sequence[float] | None = None,
        tool_collision_size: Sequence[float] | None = None,
        tool_collision_center_offset: float | None = None,
        tool_collision_local_boxes: Sequence[Sequence[float]] | None = None,
        name: str = "urdf_6r",
        urdf_path: str | Path | None = None,
    ) -> None:
        self.joints = list(joints)
        self.active_joint_names = list(active_joint_names)
        self.base_link = base_link
        self.tip_link = tip_link
        self.base_transform = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
        self.tool_length = float(tool_length)
        default_radii = [0.12] * len(self.active_joint_names)
        self.link_radii = np.asarray(default_radii if link_radii is None else link_radii, dtype=float)
        self.tool_collision_size = (
            None if tool_collision_size is None else np.asarray(tool_collision_size, dtype=float)
        )
        self.tool_collision_center_offset = (
            None
            if tool_collision_center_offset is None
            else float(tool_collision_center_offset)
        )
        self.tool_collision_local_boxes = (
            np.empty((0, 6), dtype=float)
            if tool_collision_local_boxes is None
            else np.asarray(tool_collision_local_boxes, dtype=float)
        )
        self.name = name
        self.urdf_path = Path(urdf_path) if urdf_path else None
        if not self.active_joint_names:
            raise ValueError("URDFRobot requires at least one active joint")
        self.self_collision_exclusions = {
            pair for pair in {(0, 2), (4, 6)} if max(pair) <= len(self.active_joint_names)
        }
        if self.link_radii.shape != (len(self.active_joint_names),):
            raise ValueError("link_radii must contain one radius per active joint")
        if self.tool_collision_size is not None and (
            self.tool_collision_size.shape != (3,)
            or not np.all(np.isfinite(self.tool_collision_size))
            or np.any(self.tool_collision_size <= 0.0)
        ):
            raise ValueError("tool_collision_size must contain three finite positive dimensions")
        if self.tool_collision_center_offset is not None and (
            self.tool_collision_size is None
            or not np.isfinite(self.tool_collision_center_offset)
            or self.tool_collision_center_offset <= 0.0
        ):
            raise ValueError(
                "tool_collision_center_offset requires a tool size and must be finite and positive"
            )
        if (
            self.tool_collision_local_boxes.ndim != 2
            or self.tool_collision_local_boxes.shape[1] != 6
            or not np.all(np.isfinite(self.tool_collision_local_boxes))
            or np.any(self.tool_collision_local_boxes[:, 3:] <= 0.0)
        ):
            raise ValueError(
                "tool_collision_local_boxes must contain finite [center_xyz, size_xyz] rows"
            )
        joint_by_name = {joint.name: joint for joint in self.joints}
        self.active_joints = [joint_by_name[name] for name in self.active_joint_names]
        self.joint_limits = np.asarray([joint.limit for joint in self.active_joints], dtype=float)

    @property
    def dof(self) -> int:
        return len(self.active_joint_names)

    @classmethod
    def from_urdf(
        cls,
        urdf_path: str | Path,
        active_joint_names: Sequence[str] | None = None,
        base_link: str = "base_link",
        tip_link: str = "flange",
        base_position: Iterable[float] = (0.0, 0.0, 0.0),
        base_rpy: Iterable[float] = (0.0, 0.0, 0.0),
        tool_length: float = 0.18,
        link_radii: Sequence[float] | None = None,
        tool_collision_size: Sequence[float] | None = None,
        tool_collision_center_offset: float | None = None,
        tool_collision_local_boxes: Sequence[Sequence[float]] | None = None,
        name: str = "urdf_6r",
    ) -> "URDFRobot":
        urdf_path = Path(urdf_path)
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        joints: list[URDFJoint] = []
        for elem in root.findall("joint"):
            joint_type = str(elem.attrib.get("type", "fixed"))
            parent = elem.find("parent")
            child = elem.find("child")
            if parent is None or child is None:
                continue
            origin_elem = elem.find("origin")
            xyz = _parse_xyz(origin_elem.attrib.get("xyz") if origin_elem is not None else None, (0.0, 0.0, 0.0))
            rpy = _parse_xyz(origin_elem.attrib.get("rpy") if origin_elem is not None else None, (0.0, 0.0, 0.0))
            origin = make_transform(rotation_matrix_from_rpy(*rpy), xyz)
            axis_elem = elem.find("axis")
            axis = _parse_xyz(axis_elem.attrib.get("xyz") if axis_elem is not None else None, (0.0, 0.0, 1.0))
            limit_elem = elem.find("limit")
            if limit_elem is not None:
                lower = float(limit_elem.attrib.get("lower", "0.0"))
                upper = float(limit_elem.attrib.get("upper", "0.0"))
            elif joint_type == "continuous":
                lower, upper = -2.0 * np.pi, 2.0 * np.pi
            else:
                lower, upper = 0.0, 0.0
            joints.append(
                URDFJoint(
                    name=str(elem.attrib["name"]),
                    joint_type=joint_type,
                    parent=str(parent.attrib["link"]),
                    child=str(child.attrib["link"]),
                    origin=origin,
                    axis=axis,
                    limit=(lower, upper),
                )
            )
        if active_joint_names is None:
            active_joint_names = [joint.name for joint in joints if joint.joint_type in {"revolute", "continuous", "prismatic"}]
        roll, pitch, yaw = [float(v) for v in base_rpy]
        base = make_transform(rotation_matrix_from_rpy(roll, pitch, yaw), base_position)
        chain = cls._chain_joints(joints, base_link, tip_link)
        chain_names = {joint.name for joint in chain}
        missing = [name for name in active_joint_names if name not in chain_names]
        if missing:
            raise ValueError(f"active joints are not on URDF chain {base_link}->{tip_link}: {missing}")
        return cls(
            chain,
            active_joint_names,
            base_link,
            tip_link,
            base,
            tool_length,
            link_radii,
            tool_collision_size,
            tool_collision_center_offset,
            tool_collision_local_boxes,
            name,
            urdf_path,
        )

    @classmethod
    def kuka_kr50_r2500(
        cls,
        urdf_path: str | Path = "assets/robots/kuka_kr50_r2500/kr_50_r2500.urdf",
        base_position: Iterable[float] = (-1.15, 0.0, 0.0),
        base_rpy: Iterable[float] = (0.0, 0.0, 0.0),
        tool_length: float = 0.20,
    ) -> "URDFRobot":
        robot = cls.from_urdf(
            urdf_path=urdf_path,
            active_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            base_link="base_link",
            tip_link="flange",
            base_position=base_position,
            base_rpy=base_rpy,
            tool_length=tool_length,
            link_radii=[0.18, 0.16, 0.14, 0.11, 0.09, 0.075],
            name="kuka_kr50_r2500",
        )
        # Link 3 and link 5 overlap in the conservative capsule model at
        # valid KR 50 wrist-fold configurations. Mesh collision is deferred to
        # the optional high-fidelity backend.
        robot.self_collision_exclusions.add((2, 4))
        return robot

    @classmethod
    def fanuc_m20id35(
        cls,
        urdf_path: str | Path = "assets/robots/fanuc_m20id35/m20_35_18d.urdf",
        base_position: Iterable[float] = (-0.70, 0.0, 0.30),
        base_rpy: Iterable[float] = (0.0, 0.0, 0.0),
        tool_length: float = 0.20,
        tool_collision_size: Sequence[float] | None = None,
        tool_collision_center_offset: float | None = None,
        tool_collision_local_boxes: Sequence[Sequence[float]] | None = None,
    ) -> "URDFRobot":
        """Build the FANUC M-20iD/35 from FANUC's M-20/35-18D URDF."""
        robot = cls.from_urdf(
            urdf_path=urdf_path,
            active_joint_names=["J1", "J2", "J3", "J4", "J5", "J6"],
            base_link="base_link",
            tip_link="tool0",
            base_position=base_position,
            base_rpy=base_rpy,
            tool_length=tool_length,
            link_radii=[0.24, 0.20, 0.18, 0.13, 0.10, 0.08],
            tool_collision_size=tool_collision_size,
            tool_collision_center_offset=tool_collision_center_offset,
            tool_collision_local_boxes=tool_collision_local_boxes,
            name="fanuc_m20id35",
        )
        # J5 rotates at the end of the long J4 forearm. Their conservative
        # centerline capsules overlap at valid wrist configurations.
        robot.self_collision_exclusions.add((2, 4))
        return robot

    @staticmethod
    def _chain_joints(joints: Sequence[URDFJoint], base_link: str, tip_link: str) -> list[URDFJoint]:
        by_parent: dict[str, list[URDFJoint]] = {}
        for joint in joints:
            by_parent.setdefault(joint.parent, []).append(joint)

        def visit(link: str, path: list[URDFJoint]) -> list[URDFJoint] | None:
            if link == tip_link:
                return path
            for joint in by_parent.get(link, []):
                result = visit(joint.child, path + [joint])
                if result is not None:
                    return result
            return None

        chain = visit(base_link, [])
        if chain is None:
            raise ValueError(f"No URDF joint chain found from {base_link!r} to {tip_link!r}")
        return chain

    def clamp(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-9) -> bool:
        q = np.asarray(q, dtype=float)
        return bool(
            np.all(q >= self.joint_limits[:, 0] - tolerance)
            and np.all(q <= self.joint_limits[:, 1] + tolerance)
        )

    def _frames_and_axes(self, q: np.ndarray) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"q must be shape ({self.dof},)")
        q_by_name = {name: float(value) for name, value in zip(self.active_joint_names, q)}
        frames = [self.base_transform.copy()]
        axes: list[np.ndarray] = []
        origins: list[np.ndarray] = []
        active_child_frames: list[np.ndarray] = []
        transform = self.base_transform.copy()
        for joint in self.joints:
            joint_frame = transform @ joint.origin
            if joint.name in q_by_name:
                axis_world = joint_frame[:3, :3] @ joint.axis
                axes.append(axis_world / np.linalg.norm(axis_world))
                origins.append(joint_frame[:3, 3].copy())
                transform = joint_frame @ _axis_angle_transform(joint.axis, q_by_name[joint.name], joint.joint_type)
                active_child_frames.append(transform.copy())
            else:
                transform = joint_frame
        frames.extend(active_child_frames)
        tool = transform.copy()
        tool[:3, 3] += tool[:3, 2] * self.tool_length
        frames.append(tool)
        return frames, origins, axes

    def frames(self, q: np.ndarray, include_tool: bool = True) -> list[np.ndarray]:
        frames, _, _ = self._frames_and_axes(q)
        if include_tool:
            return frames
        return frames[:-1]

    def named_link_frames(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Return world transforms for every link on the parsed URDF chain."""
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"q must be shape ({self.dof},)")
        q_by_name = {name: float(value) for name, value in zip(self.active_joint_names, q)}
        transform = self.base_transform.copy()
        result = {self.base_link: transform.copy()}
        for joint in self.joints:
            transform = transform @ joint.origin
            if joint.name in q_by_name:
                transform = transform @ _axis_angle_transform(
                    joint.axis, q_by_name[joint.name], joint.joint_type
                )
            result[joint.child] = transform.copy()
        return result

    def fk(self, q: np.ndarray) -> np.ndarray:
        return self.frames(q, include_tool=True)[-1]

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray:
        frames, origins, axes = self._frames_and_axes(q)
        p_end = frames[-1][:3, 3]
        jacobian = np.zeros((6, self.dof), dtype=float)
        for idx, (origin, axis) in enumerate(zip(origins, axes)):
            jacobian[:3, idx] = np.cross(axis, p_end - origin)
            jacobian[3:, idx] = axis
        return jacobian

    def link_elevation_degrees(self, q: np.ndarray, link_index: int) -> float:
        frames = self.frames(q, include_tool=False)
        if not 1 <= link_index < len(frames):
            raise ValueError(f"link_index must be in [1, {len(frames) - 1}]")
        vector = frames[link_index][:3, 3] - frames[link_index - 1][:3, 3]
        return float(np.degrees(np.arctan2(vector[2], np.linalg.norm(vector[:2]))))

    def link_capsules(self, q: np.ndarray) -> list[Capsule]:
        frames = self.frames(q, include_tool=True)
        points = [f[:3, 3] for f in frames]
        capsules: list[Capsule] = []
        for i in range(self.dof):
            capsules.append(Capsule(points[i], points[i + 1], float(self.link_radii[i]), f"link_{i + 1}"))
        capsules.append(Capsule(points[self.dof], points[self.dof + 1], 0.045, "tool"))
        return capsules

    def tool_collision_obb(self, q: np.ndarray) -> OBB | None:
        """Return the configured rigid-head envelope in the planner tool frame."""
        if self.tool_collision_size is None:
            return None
        tool = self.fk(q)
        size = self.tool_collision_size
        # FK is located at the suction working plane; the flange and rigid
        # head occupy the negative local-Z interval behind that plane.
        center_offset = (
            0.5 * size[2]
            if self.tool_collision_center_offset is None
            else self.tool_collision_center_offset
        )
        local_center = np.asarray([0.0, 0.0, -center_offset], dtype=float)
        return OBB(
            center=tool[:3, :3] @ local_center + tool[:3, 3],
            half_extents=0.5 * size,
            rotation=tool[:3, :3],
            name="tool_envelope",
            category="robot",
        )

    def tool_collision_obbs(self, q: np.ndarray) -> list[OBB]:
        """Return audited rigid-tool subcomponents in the planner tool frame."""
        tool = self.fk(q)
        return [
            OBB(
                center=tool[:3, :3] @ row[:3] + tool[:3, 3],
                half_extents=0.5 * row[3:],
                rotation=tool[:3, :3],
                name=f"tool_rigid_{index}",
                category="robot",
            )
            for index, row in enumerate(self.tool_collision_local_boxes)
        ]

    def collision_result(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.015,
        ignored_obstacle_names: set[str] | None = None,
        check_self: bool = True,
    ) -> CollisionResult:
        if not self.within_limits(q):
            return CollisionResult(True, "joint_limit")
        ignored = ignored_obstacle_names or set()
        capsules = self.link_capsules(q)
        tool_envelopes = self.tool_collision_obbs(q)
        if not tool_envelopes:
            legacy_envelope = self.tool_collision_obb(q)
            tool_envelopes = [] if legacy_envelope is None else [legacy_envelope]
        for tool_envelope in tool_envelopes:
            for obstacle in obstacles:
                if obstacle.name in ignored:
                    continue
                if tool_envelope.intersects_obb(obstacle, margin=margin):
                    return CollisionResult(
                        True, "robot_obstacle", tool_envelope.name, obstacle.name
                    )
        for capsule in capsules:
            for obstacle in obstacles:
                if obstacle.name in ignored:
                    continue
                if capsule.collides_obb(obstacle, margin=margin):
                    return CollisionResult(True, "robot_obstacle", capsule.name, obstacle.name)
        if check_self:
            for i, a in enumerate(capsules):
                for j in range(i + 2, len(capsules)):
                    if (i, j) in self.self_collision_exclusions:
                        continue
                    if capsules_collide(a, capsules[j], margin=0.003):
                        return CollisionResult(True, "self_collision", a.name, capsules[j].name)
        return CollisionResult(False)

    def is_collision_free(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.015,
        ignored_obstacle_names: set[str] | None = None,
    ) -> bool:
        return not self.collision_result(q, obstacles, margin, ignored_obstacle_names).in_collision

    @property
    def max_reach(self) -> float:
        points = [frame[:3, 3] for frame in self.frames(np.mean(self.joint_limits, axis=1), include_tool=True)]
        return float(sum(np.linalg.norm(b - a) for a, b in zip(points[:-1], points[1:])))


# Backward-compatible concrete name for existing six-axis configurations.
URDFRobot6 = URDFRobot
