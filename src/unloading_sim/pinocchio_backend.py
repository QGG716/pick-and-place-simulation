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


def _local_geometry_aabb(geometry) -> tuple[np.ndarray, np.ndarray] | None:
    """Read the exact geometry's enclosing Coal bounds, or disable broadphase.

    Some supported hpp-fcl builds do not expose these fields.  Missing,
    invalid, or infinite bounds must fall back to the original exact query.
    URDF geometry is immutable after backend construction.
    """
    try:
        geometry.computeLocalAABB()
        lower = np.asarray(geometry.aabb_local.min_, dtype=float).reshape(3)
        upper = np.asarray(geometry.aabb_local.max_, dtype=float).reshape(3)
    except (AttributeError, TypeError, ValueError, RuntimeError):
        return None
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower > upper):
        return None
    return lower.copy(), upper.copy()


def _world_aabb(
    local_bounds: tuple[np.ndarray, np.ndarray], rotation: np.ndarray, translation: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Enclose all eight transformed corners, including rounding allowance."""
    lower, upper = local_bounds
    center = rotation @ (0.5 * (lower + upper)) + translation
    half = np.abs(rotation) @ (0.5 * (upper - lower))
    # This expands the broadphase only; exact collision distance is untouched.
    pad = 32.0 * np.finfo(float).eps * (1.0 + np.max(np.abs(center)) + np.max(half))
    return np.nextafter(center - half - pad, -np.inf), np.nextafter(center + half + pad, np.inf)


def _local_geometry_vertices(geometry) -> np.ndarray | None:
    """Copy vertices of the very Coal mesh used by the collision backend.

    Coal has already applied the URDF mesh scale. Geometry placements contain
    its collision origin, so neither scale nor origin is applied a second time.
    Unsupported hpp-fcl bindings/primitives retain the conservative AABB path.
    """
    try:
        accessor = getattr(geometry, "vertices")
        vertices = np.asarray(accessor() if callable(accessor) else accessor, dtype=float)
        count = int(geometry.num_vertices)
    except (AttributeError, TypeError, ValueError, RuntimeError):
        return None
    if vertices.shape != (count, 3) or count == 0 or not np.all(np.isfinite(vertices)):
        return None
    vertices = np.ascontiguousarray(vertices).copy()
    vertices.setflags(write=False)
    return vertices


def _world_vertex_extrema(vertices, rotation, translation):
    """Plane extrema of every mesh triangle, outward-rounded in world axes.

    A linear functional reaches its extrema at triangle vertices, making this
    exact for the piecewise-linear collision surface without enclosing empty
    rotated local-AABB corners. The pad covers the floating-point transforms.
    """
    points = vertices @ rotation.T + translation
    magnitude = (1.0 + float(np.max(np.abs(translation)))
                 + float(np.max(np.abs(vertices))) * float(np.max(np.sum(np.abs(rotation), axis=1))))
    pad = 64.0 * np.finfo(float).eps * magnitude
    return (np.nextafter(np.min(points, axis=0) - pad, -np.inf),
            np.nextafter(np.max(points, axis=0) + pad, np.inf))


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
        tip_from_tcp: np.ndarray | None = None,
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
        self.active_joint_names = tuple(str(name) for name in self.model.names[1:])
        if len(self.active_joint_names) != self.model.nq:
            raise ValueError(
                "this backend requires exactly one scalar configuration per non-universe joint"
            )
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
        self._geometry_local_aabbs = tuple(
            _local_geometry_aabb(item.geometry)
            for item in self.geometry_model.geometryObjects
        )
        self._geometry_local_vertices = tuple(
            _local_geometry_vertices(item.geometry)
            for item in self.geometry_model.geometryObjects
        )
        if not self.model.existFrame(tip_frame):
            raise ValueError(f"URDF has no tip frame {tip_frame!r}")
        self.tip_frame = tip_frame
        self.tip_frame_id = self.model.getFrameId(tip_frame)
        self.base_transform = np.eye(4) if base_transform is None else np.asarray(base_transform, dtype=float)
        if self.base_transform.shape != (4, 4) or not np.all(np.isfinite(self.base_transform)):
            raise ValueError("base_transform must be a finite 4x4 transform")
        if not np.allclose(
            self.base_transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0
        ):
            raise ValueError("base_transform must have a homogeneous bottom row")
        base_rotation = self.base_transform[:3, :3]
        if not np.allclose(
            base_rotation.T @ base_rotation, np.eye(3), atol=1e-12, rtol=0.0
        ) or not np.isclose(np.linalg.det(base_rotation), 1.0, atol=1e-12, rtol=0.0):
            raise ValueError("base_transform must contain a proper orthonormal rotation")
        self.tool_length = float(tool_length)
        if not np.isfinite(self.tool_length) or self.tool_length < 0.0:
            raise ValueError("tool_length must be finite and non-negative")
        if tip_from_tcp is not None and self.tool_length != 0.0:
            raise ValueError("tip_from_tcp and a non-zero tool_length may not be combined")
        if tip_from_tcp is None:
            self.tip_from_tcp = np.eye(4)
            self.tip_from_tcp[2, 3] = self.tool_length
        else:
            transform = np.asarray(tip_from_tcp, dtype=float)
            if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
                raise ValueError("tip_from_tcp must be a finite 4x4 transform")
            if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0):
                raise ValueError("tip_from_tcp must have a homogeneous bottom row")
            rotation = transform[:3, :3]
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0.0) \
                    or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0.0):
                raise ValueError("tip_from_tcp must contain a proper orthonormal rotation")
            self.tip_from_tcp = transform.copy()
        self.name = name
        self.collision_geometry_kind = "urdf_mesh"
        self.collision_margin_semantics = "minimum_surface_distance_two_times_per_body_margin"
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
        tip = self.base_transform @ self._matrix(self.data.oMf[self.tip_frame_id])
        return tip @ self.tip_from_tcp

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
        base_rotation = self.base_transform[:3, :3]
        jacobian[:3] = base_rotation @ jacobian[:3]
        jacobian[3:] = base_rotation @ jacobian[3:]
        # Pinocchio returns the Jacobian at ``tip_frame``.  Strict IK is solved
        # at the configured virtual task TCP, which may have a full SE(3)
        # offset (the official M-710 flange points along +X, not TCP +Z).
        tip = self.base_transform @ self._matrix(self.data.oMf[self.tip_frame_id])
        tcp = tip @ self.tip_from_tcp
        offset_world = tcp[:3, 3] - tip[:3, 3]
        jacobian[:3] += np.cross(jacobian[3:].T, offset_world).T
        return jacobian

    def named_link_frames(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Return every URDF frame plus the explicit virtual TCP frame."""

        self._update(q)
        result = {
            str(frame.name): self.base_transform @ self._matrix(self.data.oMf[index])
            for index, frame in enumerate(self.model.frames)
        }
        result["virtual_task_tcp"] = self.fk(q)
        return result

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

    def _distance(self, first_geometry, first_tf, second_geometry, second_tf) -> float:
        request = self.coal.DistanceRequest()
        result = self.coal.DistanceResult()
        value = self.coal.distance(
            first_geometry, first_tf, second_geometry, second_tf, request, result
        )
        recorded = getattr(result, "min_distance", value)
        return float(recorded)

    def _environment_collision(
        self,
        obstacles: Sequence[OBB],
        ignored: set[str],
        minimum_distance: float,
        ignored_pairs: set[tuple[str, str]],
    ) -> CollisionResult:
        active_obstacles = [obstacle for obstacle in obstacles if obstacle.name not in ignored]
        if not active_obstacles:
            return CollisionResult(False)
        obstacle_aabbs = [
            _world_aabb((-obstacle.half_extents, obstacle.half_extents), obstacle.rotation, obstacle.center)
            for obstacle in active_obstacles
        ]
        obstacle_lower = np.asarray([bounds[0] for bounds in obstacle_aabbs])
        obstacle_upper = np.asarray([bounds[1] for bounds in obstacle_aabbs])
        # Construct exact Coal boxes lazily and reuse them across robot links.
        # A skipped pair never changes first-collision ordering among queries.
        obstacle_exact: dict[int, tuple[object, object]] = {}
        for geometry_index, geometry_object in enumerate(self.geometry_model.geometryObjects):
            parent_frame = int(getattr(geometry_object, "parentFrame", -1))
            link_name = (
                str(self.model.frames[parent_frame].name)
                if 0 <= parent_frame < len(self.model.frames)
                else str(geometry_object.name)
            )
            placement = self.base_transform @ self._matrix(self.geometry_data.oMg[geometry_index])
            local_bounds = self._geometry_local_aabbs[geometry_index]
            if local_bounds is None:
                near_indices = range(len(active_obstacles))
            else:
                lower, upper = _world_aabb(local_bounds, placement[:3, :3], placement[:3, 3])
                # Axis separation is a lower bound on Euclidean surface
                # distance.  Only provably farther pairs skip Coal; the 2x
                # per-body margin comparison below remains unchanged.
                separated = np.any(
                    (obstacle_lower - upper > minimum_distance + 1e-12)
                    | (lower - obstacle_upper > minimum_distance + 1e-12), axis=1,
                )
                near_indices = np.flatnonzero(~separated)
            robot_tf = _transform(self.coal, placement[:3, :3], placement[:3, 3])
            for obstacle_index in near_indices:
                obstacle = active_obstacles[obstacle_index]
                if (link_name, obstacle.name) in ignored_pairs or (
                    str(geometry_object.name), obstacle.name
                ) in ignored_pairs:
                    continue
                if obstacle_index not in obstacle_exact:
                    obstacle_exact[obstacle_index] = (
                        self.coal.Box(*(2.0 * obstacle.half_extents).tolist()),
                        _transform(self.coal, obstacle.rotation, obstacle.center),
                    )
                box, box_tf = obstacle_exact[obstacle_index]
                if self._distance(geometry_object.geometry, robot_tf, box, box_tf) <= minimum_distance:
                    return CollisionResult(True, "robot_obstacle", link_name, obstacle.name)
        return CollisionResult(False)

    @property
    def collision_link_names(self) -> tuple[str, ...]:
        """Return link-frame names that own collision geometry in the URDF."""

        names: list[str] = []
        for geometry_object in self.geometry_model.geometryObjects:
            parent_frame = int(getattr(geometry_object, "parentFrame", -1))
            name = (
                str(self.model.frames[parent_frame].name)
                if 0 <= parent_frame < len(self.model.frames)
                else str(geometry_object.name)
            )
            if name not in names:
                names.append(name)
        return tuple(names)

    def collision_world_axis_extrema(self, q: np.ndarray) -> dict[str, dict]:
        """World-axis bounds of the actual collision geometry, grouped by link.

        This provider is for plane tests only. Pairwise mesh collision queries
        and their engineering margins stay unchanged. Mesh vertices are cached
        once from the same immutable Coal objects, never loaded per query.
        """
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,) or not np.all(np.isfinite(q)):
            raise ValueError(f"q must contain {self.dof} finite joint values")
        self.pin.updateGeometryPlacements(self.model, self.data, self.geometry_model, self.geometry_data, q)
        result: dict[str, dict] = {}
        for index, geometry_object in enumerate(self.geometry_model.geometryObjects):
            parent_frame = int(getattr(geometry_object, "parentFrame", -1))
            link = (str(self.model.frames[parent_frame].name)
                    if 0 <= parent_frame < len(self.model.frames) else str(geometry_object.name))
            placement = self.base_transform @ self._matrix(self.geometry_data.oMg[index])
            vertices = self._geometry_local_vertices[index]
            if vertices is not None:
                lower, upper = _world_vertex_extrema(vertices, placement[:3, :3], placement[:3, 3])
                source = "SAME_COAL_COLLISION_MESH_VERTICES"
            else:
                local_bounds = self._geometry_local_aabbs[index]
                if local_bounds is None:
                    raise RuntimeError(f"collision geometry has no safe plane-bound provider: {geometry_object.name}")
                lower, upper = _world_aabb(local_bounds, placement[:3, :3], placement[:3, 3])
                source = "CONSERVATIVE_SAME_COAL_LOCAL_AABB_FALLBACK"
            if link not in result:
                result[link] = {"lower_m": lower.copy(), "upper_m": upper.copy(),
                                "exact_mesh": vertices is not None, "geometry_names": [],
                                "vertex_count": 0, "sources": [],
                                "numeric_padding": "OUTWARD_64_EPS_TRANSFORM_BOUND_AND_NEXTAFTER"}
            else:
                result[link]["lower_m"] = np.minimum(result[link]["lower_m"], lower)
                result[link]["upper_m"] = np.maximum(result[link]["upper_m"], upper)
                result[link]["exact_mesh"] &= vertices is not None
            result[link]["geometry_names"].append(str(geometry_object.name))
            result[link]["vertex_count"] += 0 if vertices is None else len(vertices)
            result[link]["sources"].append(source)
        return result

    def _self_clearance_failure(self, minimum_distance: float) -> CollisionResult:
        for pair in self.geometry_model.collisionPairs:
            first = self.geometry_model.geometryObjects[int(pair.first)]
            second = self.geometry_model.geometryObjects[int(pair.second)]
            first_pose = self.base_transform @ self._matrix(
                self.geometry_data.oMg[int(pair.first)]
            )
            second_pose = self.base_transform @ self._matrix(
                self.geometry_data.oMg[int(pair.second)]
            )
            first_tf = _transform(self.coal, first_pose[:3, :3], first_pose[:3, 3])
            second_tf = _transform(self.coal, second_pose[:3, :3], second_pose[:3, 3])
            if self._distance(first.geometry, first_tf, second.geometry, second_tf) <= minimum_distance:
                return CollisionResult(True, "self_collision", first.name, second.name)
        return CollisionResult(False)

    def collision_result(
        self,
        q: np.ndarray,
        obstacles: Sequence[OBB],
        margin: float = 0.0,
        ignored_obstacle_names: set[str] | None = None,
        ignored_geometry_obstacle_pairs: set[tuple[str, str]] | None = None,
        check_self: bool = True,
    ) -> CollisionResult:
        if not np.isfinite(margin) or margin < 0.0:
            raise ValueError("collision margin must be finite and non-negative")
        q = np.asarray(q, dtype=float)
        if not self.within_limits(q):
            return CollisionResult(True, "joint_limit")
        self.pin.updateGeometryPlacements(
            self.model, self.data, self.geometry_model, self.geometry_data, q
        )
        # Repository margins are per body.  Requiring a surface distance of
        # twice that value preserves the same pairwise engineering clearance
        # without convexifying or scaling either official mesh.
        minimum_distance = 2.0 * float(margin)
        if check_self:
            failure = self._self_clearance_failure(minimum_distance)
            if failure.in_collision:
                return failure
        return self._environment_collision(
            obstacles,
            ignored_obstacle_names or set(),
            minimum_distance,
            ignored_geometry_obstacle_pairs or set(),
        )

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
