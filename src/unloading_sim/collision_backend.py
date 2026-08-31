"""Optional exact-geometry collision backends and approximation regression."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np

from .geometry import OBB
from .robot import RobotBackend, URDFRobot6


@dataclass(frozen=True)
class CollisionQuery:
    in_collision: bool
    reason: str = ""
    first_link: str | None = None
    first_obstacle: str | None = None


class CollisionBackend(Protocol):
    name: str

    def query(self, q: np.ndarray, obstacles: Sequence[OBB]) -> CollisionQuery: ...


class CapsuleCollisionBackend:
    name = "capsule_obb"

    def __init__(self, robot: RobotBackend, margin: float = 0.015) -> None:
        self.robot = robot
        self.margin = float(margin)

    def query(self, q: np.ndarray, obstacles: Sequence[OBB]) -> CollisionQuery:
        result = self.robot.collision_result(q, obstacles, margin=self.margin)
        return CollisionQuery(result.in_collision, result.reason, result.first_link, result.first_obstacle)


class PyBulletMeshCollisionBackend:
    """URDF mesh-vs-OBB collision query using an isolated DIRECT client.

    PyBullet remains an optional dependency and is imported only when this
    adapter is constructed.  The core planner therefore remains lightweight.
    """

    name = "pybullet_urdf_mesh"

    def __init__(
        self,
        robot: URDFRobot6,
        obstacles: Sequence[OBB],
        *,
        package_roots: Sequence[str | Path] = (),
        distance: float = 0.0,
    ) -> None:
        if robot.urdf_path is None:
            raise ValueError("mesh collision backend requires a robot URDF path")
        if not np.isfinite(distance) or distance < 0.0:
            raise ValueError("distance must be finite and non-negative")
        from .pybullet_sim import _import_pybullet, add_obb, load_robot

        self._p = _import_pybullet()
        self._client = self._p.connect(self._p.DIRECT)
        if self._client < 0:
            raise RuntimeError("failed to create PyBullet collision client")
        self._distance = float(distance)
        self._robot = robot
        self._p.resetSimulation(physicsClientId=self._client)
        base_position = robot.base_transform[:3, 3].tolist()
        # XYZ Euler extraction is delegated to PyBullet from the equivalent quaternion.
        from .pybullet_sim import quaternion_from_matrix

        base_rpy = self._p.getEulerFromQuaternion(quaternion_from_matrix(robot.base_transform[:3, :3]))
        self._model = load_robot(
            self._p,
            robot.urdf_path,
            [Path(root) for root in package_roots],
            base_position,
            base_rpy,
        )
        if set(self._model.joint_names) != set(robot.active_joint_names):
            self.close()
            raise ValueError("PyBullet active joints do not match the core URDF chain")
        index_by_name = dict(zip(self._model.joint_names, self._model.joint_indices))
        self._ordered_joint_indices = [index_by_name[name] for name in robot.active_joint_names]
        self._obstacle_ids = {
            obstacle.name: add_obb(self._p, obstacle, (0.5, 0.5, 0.5, 0.2)) for obstacle in obstacles
        }

    def close(self) -> None:
        if getattr(self, "_client", -1) >= 0:
            self._p.disconnect(self._client)
            self._client = -1

    def __enter__(self) -> "PyBulletMeshCollisionBackend":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def query(self, q: np.ndarray, obstacles: Sequence[OBB]) -> CollisionQuery:
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self._ordered_joint_indices),) or not np.all(np.isfinite(q)):
            raise ValueError("q has an invalid shape or contains non-finite values")
        if not self._robot.within_limits(q):
            return CollisionQuery(True, "joint_limit")
        requested = {obstacle.name for obstacle in obstacles}
        unknown = requested.difference(self._obstacle_ids)
        if unknown:
            raise KeyError(f"collision backend was not initialized with obstacles: {sorted(unknown)}")
        for joint_index, value in zip(self._ordered_joint_indices, q):
            self._p.resetJointState(self._model.body_id, joint_index, float(value))
        self._p.performCollisionDetection()
        for name in sorted(requested):
            contacts = self._p.getClosestPoints(
                self._model.body_id,
                self._obstacle_ids[name],
                self._distance,
            )
            if contacts:
                link_index = int(contacts[0][3])
                link_name = next((key for key, value in self._model.link_names.items() if value == link_index), None)
                return CollisionQuery(True, "robot_obstacle", link_name, name)
        self_contacts = self._p.getContactPoints(self._model.body_id, self._model.body_id)
        for contact in self_contacts:
            link_a, link_b = int(contact[3]), int(contact[4])
            if link_a >= 0 and link_b >= 0 and abs(link_a - link_b) > 1:
                return CollisionQuery(True, "self_collision", str(link_a), str(link_b))
        return CollisionQuery(False)


def compare_collision_backends(
    path: Sequence[Sequence[float] | np.ndarray],
    obstacles: Sequence[OBB],
    approximate: CollisionBackend,
    reference: CollisionBackend,
) -> dict:
    """Return a deterministic confusion matrix for two collision models."""
    matrix = {"both_free": 0, "both_collision": 0, "approx_only": 0, "reference_only": 0}
    mismatches: list[dict] = []
    for index, waypoint in enumerate(path):
        q = np.asarray(waypoint, dtype=float)
        approx = approximate.query(q, obstacles)
        exact = reference.query(q, obstacles)
        if approx.in_collision and exact.in_collision:
            matrix["both_collision"] += 1
        elif not approx.in_collision and not exact.in_collision:
            matrix["both_free"] += 1
        elif approx.in_collision:
            matrix["approx_only"] += 1
            mismatches.append(
                {
                    "index": index,
                    "kind": "approx_only",
                    "approx_reason": approx.reason,
                    "approx_link": approx.first_link,
                    "approx_obstacle": approx.first_obstacle,
                }
            )
        else:
            matrix["reference_only"] += 1
            mismatches.append(
                {
                    "index": index,
                    "kind": "reference_only",
                    "reference_reason": exact.reason,
                    "reference_link": exact.first_link,
                    "reference_obstacle": exact.first_obstacle,
                }
            )
    total = len(path)
    return {
        "approximate_backend": approximate.name,
        "reference_backend": reference.name,
        "samples": total,
        **matrix,
        "agreement_rate": 1.0 if total == 0 else (matrix["both_free"] + matrix["both_collision"]) / total,
        "mismatches": mismatches,
    }
