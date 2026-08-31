"""Optional Pinocchio + Coal/hpp-fcl arbitrary-DOF robot backend.

Recent Pinocchio releases renamed hpp-fcl to Coal.  This adapter accepts either
Python module name and keeps both packages outside the lightweight core.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from .geometry import Capsule, OBB
from .robot import CollisionResult


def _optional_imports():
    try:
        import pinocchio as pin
    except ImportError as exc:
        raise RuntimeError(
            "Pinocchio is required for the exact backend. On Linux install the "
            "optional extra or use conda-forge: conda install pinocchio coal -c conda-forge"
        ) from exc
    try:
        import coal
    except ImportError:
        try:
            import hppfcl as coal
        except ImportError as exc:
            raise RuntimeError("Coal (formerly hpp-fcl) is required for mesh collision checking") from exc
    return pin, coal


def _transform(coal, rotation: np.ndarray, translation: np.ndarray):
    transform_type = getattr(coal, "Transform3s", None) or getattr(coal, "Transform3f", None)
    if transform_type is None:
        raise RuntimeError("installed Coal/hpp-fcl has no supported Transform3 type")
    return transform_type(np.asarray(rotation, dtype=float), np.asarray(translation, dtype=float))


class PinocchioHppFclBackend:
    """Fixed-base serial/tree robot using URDF collision meshes and Pinocchio FK."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        tip_frame: str,
        package_dirs: Sequence[str | Path] = (),
        srdf_path: str | Path | None = None,
        base_transform: np.ndarray | None = None,
        tool_length: float = 0.0,
        link_radii: Sequence[float] | None = None,
        name: str = "pinocchio_robot",
    ) -> None:
        self.pin, self.coal = _optional_imports()
        self.urdf_path = Path(urdf_path)
        if not self.urdf_path.exists():
            raise FileNotFoundError(self.urdf_path)
        self.model = self.pin.buildModelFromUrdf(str(self.urdf_path))
        if self.model.nq != self.model.nv:
            raise ValueError("this backend currently requires fixed-base scalar joints with nq == nv")
        self.data = self.model.createData()
        package_paths = [str(Path(path)) for path in package_dirs]
        self.geometry_model = self.pin.buildGeomFromUrdf(
            self.model,
            str(self.urdf_path),
            self.pin.GeometryType.COLLISION,
            package_dirs=package_paths,
        )
        self.geometry_model.addAllCollisionPairs()
        if srdf_path is not None:
            self.pin.removeCollisionPairs(
                self.model, self.geometry_model, str(Path(srdf_path)), False
            )
            self.collision_pair_filter = "srdf"
        else:
            self._remove_kinematic_neighbor_pairs()
            self.collision_pair_filter = "automatic_direct_kinematic_neighbors"
        self.geometry_data = self.pin.GeometryData(self.geometry_model)
        if not self.model.existFrame(tip_frame):
            raise ValueError(f"URDF has no tip frame {tip_frame!r}")
        self.tip_frame = tip_frame
        self.tip_frame_id = self.model.getFrameId(tip_frame)
        self.base_transform = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
        if self.base_transform.shape != (4, 4):
            raise ValueError("base_transform must have shape (4,4)")
        self.tool_length = float(tool_length)
        self.name = name
        self.joint_limits = np.column_stack((self.model.lowerPositionLimit, self.model.upperPositionLimit))
        default_radii = np.full(self.dof, 0.10)
        self.link_radii = default_radii if link_radii is None else np.asarray(link_radii, dtype=float)
        if self.link_radii.shape != (self.dof,) or np.any(self.link_radii <= 0.0):
            raise ValueError("link_radii must contain one positive radius per DOF")

    def _remove_kinematic_neighbor_pairs(self) -> None:
        """Remove only same-body and direct parent-child geometry pairs.

        Adjacent URDF collision meshes commonly overlap at their joint by
        design.  SRDF remains the preferred source for model-specific filters;
        this deterministic fallback leaves every non-neighbour pair active.
        """
        retained: list[tuple[int, int]] = []
        for pair in list(self.geometry_model.collisionPairs):
            first_geometry = int(pair.first)
            second_geometry = int(pair.second)
            first_joint = int(self.geometry_model.geometryObjects[first_geometry].parentJoint)
            second_joint = int(self.geometry_model.geometryObjects[second_geometry].parentJoint)
            same_body = first_joint == second_joint
            directly_connected = (
                first_joint > 0 and int(self.model.parents[first_joint]) == second_joint
            ) or (
                second_joint > 0 and int(self.model.parents[second_joint]) == first_joint
            )
            if not same_body and not directly_connected:
                retained.append((first_geometry, second_geometry))
        # Rebuild atomically. Removing pairs one by one mutates the Boost.Python
        # vector and can invalidate wrappers for the remaining pair objects.
        self.geometry_model.removeAllCollisionPairs()
        for first_geometry, second_geometry in retained:
            self.geometry_model.addCollisionPair(
                self.pin.CollisionPair(first_geometry, second_geometry)
            )

    @property
    def dof(self) -> int:
        return int(self.model.nq)

    def clamp(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"q must be shape ({self.dof},)")
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def within_limits(self, q: np.ndarray, tolerance: float = 1e-9) -> bool:
        q = np.asarray(q, dtype=float)
        return bool(
            q.shape == (self.dof,)
            and np.all(np.isfinite(q))
            and np.all(q >= self.joint_limits[:, 0] - tolerance)
            and np.all(q <= self.joint_limits[:, 1] + tolerance)
        )

    @staticmethod
    def _matrix(se3) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = np.asarray(se3.rotation)
        transform[:3, 3] = np.asarray(se3.translation).reshape(3)
        return transform

    def _update(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,):
            raise ValueError(f"q must be shape ({self.dof},)")
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)

    def fk(self, q: np.ndarray) -> np.ndarray:
        self._update(q)
        transform = self.base_transform @ self._matrix(self.data.oMf[self.tip_frame_id])
        transform[:3, 3] += transform[:3, 2] * self.tool_length
        return transform

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        jacobian = np.asarray(
            self.pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.tip_frame_id,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            ),
            dtype=float,
        )
        if jacobian.shape != (6, self.dof):
            raise RuntimeError(f"unexpected Pinocchio Jacobian shape: {jacobian.shape}")
        return jacobian

    def frames(self, q: np.ndarray, include_tool: bool = True) -> list[np.ndarray]:
        self._update(q)
        frames = [self.base_transform.copy()]
        for joint_id in range(1, self.model.njoints):
            frames.append(self.base_transform @ self._matrix(self.data.oMi[joint_id]))
        if include_tool:
            frames.append(self.fk(q))
        return frames

    def link_capsules(self, q: np.ndarray) -> list[Capsule]:
        frames = self.frames(q, include_tool=True)
        # Multi-DOF joints are rejected in __init__, so one non-universe joint
        # corresponds to each configured scalar DOF.
        capsules = [
            Capsule(frames[index][:3, 3], frames[index + 1][:3, 3], float(self.link_radii[index]), f"link_{index + 1}")
            for index in range(self.dof)
        ]
        capsules.append(Capsule(frames[-2][:3, 3], frames[-1][:3, 3], 0.045, "tool"))
        return capsules

    def link_elevation_degrees(self, q: np.ndarray, link_index: int) -> float:
        frames = self.frames(q, include_tool=False)
        if not 1 <= link_index < len(frames):
            raise ValueError(f"link_index must be in [1, {len(frames) - 1}]")
        vector = frames[link_index][:3, 3] - frames[link_index - 1][:3, 3]
        return float(np.degrees(np.arctan2(vector[2], np.linalg.norm(vector[:2]))))

    def _environment_collision(self, obstacles: Sequence[OBB], ignored: set[str]) -> CollisionResult:
        request = self.coal.CollisionRequest()
        for geometry_index, geometry_object in enumerate(self.geometry_model.geometryObjects):
            placement = self.base_transform @ self._matrix(self.geometry_data.oMg[geometry_index])
            robot_tf = _transform(self.coal, placement[:3, :3], placement[:3, 3])
            for obstacle in obstacles:
                if obstacle.name in ignored:
                    continue
                box = self.coal.Box(*(2.0 * obstacle.half_extents).tolist())
                box_tf = _transform(self.coal, obstacle.rotation, obstacle.center)
                result = self.coal.CollisionResult()
                self.coal.collide(geometry_object.geometry, robot_tf, box, box_tf, request, result)
                if result.isCollision():
                    return CollisionResult(True, "robot_obstacle", geometry_object.name, obstacle.name)
        return CollisionResult(False)

    def collision_result(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.0,
        ignored_obstacle_names: set[str] | None = None,
        check_self: bool = True,
    ) -> CollisionResult:
        if margin != 0.0:
            raise ValueError("Pinocchio mesh backend does not silently approximate non-zero collision margins")
        q = np.asarray(q, dtype=float)
        if not self.within_limits(q):
            return CollisionResult(True, "joint_limit")
        if check_self and self.pin.computeCollisions(
            self.model, self.data, self.geometry_model, self.geometry_data, q, True
        ):
            for index, result in enumerate(self.geometry_data.collisionResults):
                if result.isCollision():
                    pair = self.geometry_model.collisionPairs[index]
                    first = self.geometry_model.geometryObjects[pair.first].name
                    second = self.geometry_model.geometryObjects[pair.second].name
                    return CollisionResult(True, "self_collision", first, second)
        self.pin.updateGeometryPlacements(
            self.model, self.data, self.geometry_model, self.geometry_data, q
        )
        return self._environment_collision(obstacles, ignored_obstacle_names or set())

    def is_collision_free(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.0,
        ignored_obstacle_names: set[str] | None = None,
    ) -> bool:
        return not self.collision_result(q, obstacles, margin, ignored_obstacle_names).in_collision


# Modern package terminology while preserving the requested hpp-fcl name.
PinocchioCoalBackend = PinocchioHppFclBackend
