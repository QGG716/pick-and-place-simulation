"""Run the independent FANUC trailer cross-section reachability study."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Iterable, Sequence

import numpy as np

from unloading_sim.geometry import OBB
from unloading_sim.ik import IKResult, pose_error, solve_ik
from unloading_sim.robot import URDFRobot6

try:
    from studies.fanuc_m20id35_cross_section.model import (
        STATE_A,
        STATE_B,
        STATE_C,
        STATE_LABELS,
        WristLoadResult,
        aggregate_cell_states,
        carton_inside_trailer,
        inclusive_grid,
        load_config,
        load_gripper_model,
        neighboring_carton_centers,
        resolve_project_path,
        stable_seed,
        target_pose,
        target_rotations,
        transform_matrix,
        wrist_load_check,
    )
except ModuleNotFoundError:  # direct script execution from its own directory
    from model import (  # type: ignore
        STATE_A,
        STATE_B,
        STATE_C,
        STATE_LABELS,
        WristLoadResult,
        aggregate_cell_states,
        carton_inside_trailer,
        inclusive_grid,
        load_config,
        load_gripper_model,
        neighboring_carton_centers,
        resolve_project_path,
        stable_seed,
        target_pose,
        target_rotations,
        transform_matrix,
        wrist_load_check,
    )


@dataclass
class GeometryResult:
    passed: bool
    derated: bool
    ik_count: int
    q: np.ndarray | None
    path: list[np.ndarray]
    poses: list[np.ndarray]
    collision_status: str
    failure_reason: str
    normal_error_deg: float
    joint_margin_deg: float
    jacobian_condition: float
    support_insertion_clear: bool


@dataclass
class ScenarioResult:
    state: int
    geometry: GeometryResult
    load: WristLoadResult | None


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _matrix_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Numerically stable matrix-to-quaternion conversion for PyBullet."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
            0.25 * s,
        )
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return (0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s)
    if index == 1:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return ((matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s)
    s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return ((matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s, (matrix[1, 0] - matrix[0, 1]) / s)


class CollisionWorld:
    """Collision adapter; PyBullet uses official robot meshes, core mode is a test fallback."""

    def __init__(self, cfg: dict, config_path: Path, robot: URDFRobot6, tool_boxes: list[np.ndarray], backend: str) -> None:
        self.cfg = cfg
        self.robot = robot
        self.tool_boxes = tool_boxes
        self.backend = backend
        self.margin = float(cfg["thresholds"]["collision_margin_m"])
        self.wall_obbs = self._wall_obbs()
        self._p = None
        self.client = -1
        if backend == "pybullet":
            try:
                import pybullet as p
            except ImportError as exc:
                raise RuntimeError("--backend pybullet requires the optional pybullet package") from exc
            self._p = p
            self.client = p.connect(p.DIRECT)
            p.resetSimulation(physicsClientId=self.client)
            urdf = resolve_project_path(config_path, cfg["robot"]["urdf"])
            self.robot_body = p.loadURDF(str(urdf), useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=self.client)
            self.joint_indices = []
            self.link_names: dict[int, str] = {-1: "base_link"}
            for index in range(p.getNumJoints(self.robot_body, physicsClientId=self.client)):
                info = p.getJointInfo(self.robot_body, index, physicsClientId=self.client)
                self.link_names[index] = info[12].decode("utf-8")
                if info[1].decode("utf-8") in robot.active_joint_names:
                    self.joint_indices.append(index)
            if len(self.joint_indices) != robot.dof:
                raise RuntimeError("PyBullet joint mapping does not match FANUC URDF")
            self.tip_index = next(index for index, name in self.link_names.items() if name == robot.tip_link)
            self.wall_bodies = [self._add_box(2.0 * wall.half_extents, wall.center) for wall in self.wall_obbs]
            shape = p.createCollisionShapeArray(
                shapeTypes=[p.GEOM_BOX] * len(tool_boxes),
                halfExtents=[(0.5 * box[3:]).tolist() for box in tool_boxes],
                collisionFramePositions=[box[:3].tolist() for box in tool_boxes],
                collisionFrameOrientations=[[0.0, 0.0, 0.0, 1.0]] * len(tool_boxes),
                physicsClientId=self.client,
            )
            self.tool_body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, basePosition=[10, 10, 10], physicsClientId=self.client)
            blade_cfg = cfg["gripper"]
            blade_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[0.5 * blade_cfg["support_blade_width_m"], 0.5 * blade_cfg["support_blade_thickness_m"], 0.5 * blade_cfg["support_blade_length_m"]],
                physicsClientId=self.client,
            )
            self.blade_body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=blade_shape, basePosition=[10, 10, 10], physicsClientId=self.client)
            self.box_sets: dict[tuple[float, float, float], tuple[int, list[int]]] = {}

    def inverse_kinematics(self, target: np.ndarray, seeds: Sequence[np.ndarray], position_tolerance: float, orientation_tolerance: float) -> tuple[list[IKResult], IKResult | None]:
        if self._p is None:
            raise RuntimeError("PyBullet IK requested from the geometric fallback")
        p = self._p
        p.resetBasePositionAndOrientation(self.robot_body, [0, 0, 0], [0, 0, 0, 1], physicsClientId=self.client)
        lower = self.robot.joint_limits[:, 0].tolist()
        upper = self.robot.joint_limits[:, 1].tolist()
        ranges = (self.robot.joint_limits[:, 1] - self.robot.joint_limits[:, 0]).tolist()
        solutions: list[IKResult] = []
        best_failure: IKResult | None = None
        raw_candidates: list[np.ndarray] = []
        for seed_index, seed in enumerate(seeds):
            flange_target = target[:3, 3] - target[:3, 2] * float(self.robot.tool_length)
            values = p.calculateInverseKinematics(
                self.robot_body,
                self.tip_index,
                flange_target.tolist(),
                _matrix_quaternion_xyzw(target[:3, :3]),
                lowerLimits=lower,
                upperLimits=upper,
                jointRanges=ranges,
                restPoses=np.asarray(seed, dtype=float).tolist(),
                jointDamping=[0.02] * self.robot.dof,
                solver=p.IK_DLS,
                maxNumIterations=140,
                residualThreshold=1e-7,
                physicsClientId=self.client,
            )
            q = np.asarray(values[: self.robot.dof], dtype=float)
            if not any(np.linalg.norm(np.arctan2(np.sin(q - prior), np.cos(q - prior))) < 0.15 for prior in raw_candidates):
                raw_candidates.append(q)
        for q in raw_candidates:
            result = solve_ik(
                self.robot,
                target,
                q,
                max_iterations=100,
                damping=0.035,
                max_step=0.22,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
                orientation_weight=0.7,
            )
            if result.success:
                solutions.append(result)
            elif best_failure is None or result.position_error + 0.25 * result.orientation_error < best_failure.position_error + 0.25 * best_failure.orientation_error:
                best_failure = result
        return solutions, best_failure

    def close(self) -> None:
        if self._p is not None and self.client >= 0:
            self._p.disconnect(self.client)
            self.client = -1

    def _wall_obbs(self) -> list[OBB]:
        trailer = self.cfg["trailer"]
        width, height = float(trailer["inside_width_m"]), float(trailer["inside_height_m"])
        thickness = float(trailer["wall_thickness_m"])
        x_min, x_max = trailer["longitudinal_collision_span_m"]
        x_size = float(x_max - x_min)
        x_center = 0.5 * float(x_min + x_max)
        def box(center, size, name):
            return OBB(np.asarray(center, dtype=float), 0.5 * np.asarray(size, dtype=float), np.eye(3), name=name)

        return [
            box([x_center, -0.5 * width - 0.5 * thickness, 0.5 * height], [x_size, thickness, height + 2 * thickness], "right_wall"),
            box([x_center, 0.5 * width + 0.5 * thickness, 0.5 * height], [x_size, thickness, height + 2 * thickness], "left_wall"),
            box([x_center, 0.0, -0.5 * thickness], [x_size, width, thickness], "floor"),
            box([x_center, 0.0, height + 0.5 * thickness], [x_size, width, thickness], "roof"),
        ]

    def _add_box(self, size: Sequence[float], center: Sequence[float]) -> int:
        assert self._p is not None
        shape = self._p.createCollisionShape(self._p.GEOM_BOX, halfExtents=(0.5 * np.asarray(size)).tolist(), physicsClientId=self.client)
        return self._p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, basePosition=list(center), physicsClientId=self.client)

    def _box_set(self, size: Sequence[float]) -> tuple[int, list[int]]:
        key = tuple(float(v) for v in size)
        if key not in self.box_sets:
            # World axes are longitudinal depth, cross-section width, height.
            world_size = [key[2], key[0], key[1]]
            self.box_sets[key] = (self._add_box(world_size, [10, 10, 10]), [self._add_box(world_size, [10, 10, 10]) for _ in range(4)])
        return self.box_sets[key]

    @staticmethod
    def _obb_from_local_box(pose: np.ndarray, row: np.ndarray, name: str) -> OBB:
        return OBB(pose[:3, :3] @ row[:3] + pose[:3, 3], 0.5 * row[3:], pose[:3, :3], name=name)

    def check(
        self,
        q: np.ndarray,
        mount: tuple[float, float],
        working_pose: np.ndarray,
        box_pose: np.ndarray | None,
        box_size: Sequence[float] | None,
        neighbor_centers: Sequence[np.ndarray],
        blade_progress: float | None = None,
    ) -> tuple[bool, str]:
        if self.backend == "geometric":
            self.robot.base_transform[:3, 3] = [self.cfg["robot"]["base_longitudinal_x_m"], mount[0], mount[1]]
            obstacles = list(self.wall_obbs)
            if box_size is not None:
                world_size = [box_size[2], box_size[0], box_size[1]]
                obstacles.extend(OBB(np.asarray(center), 0.5 * np.asarray(world_size), np.eye(3), name=f"neighbor_{index}") for index, center in enumerate(neighbor_centers))
            collision = self.robot.collision_result(q, obstacles, margin=self.margin)
            if collision.in_collision:
                return False, "robot_self_collision" if collision.reason == "self_collision" else "robot_trailer_collision"
            for index, row in enumerate(self.tool_boxes):
                tool = self._obb_from_local_box(working_pose, row, f"tool_{index}")
                if any(tool.intersects_obb(obstacle, margin=self.margin) for obstacle in obstacles):
                    return False, "tool_neighbor_collision"
            if box_pose is not None and box_size is not None:
                carried = OBB(box_pose[:3, 3], 0.5 * np.asarray([box_size[2], box_size[0], box_size[1]]), box_pose[:3, :3], name="carried")
                if any(carried.intersects_obb(obstacle, margin=self.margin) for obstacle in obstacles):
                    return False, "carried_carton_collision"
            return True, "clear"

        assert self._p is not None
        p = self._p
        base_position = [self.cfg["robot"]["base_longitudinal_x_m"], mount[0], mount[1]]
        p.resetBasePositionAndOrientation(self.robot_body, base_position, [0, 0, 0, 1], physicsClientId=self.client)
        for joint_index, value in zip(self.joint_indices, q):
            p.resetJointState(self.robot_body, joint_index, float(value), physicsClientId=self.client)
        quaternion = _matrix_quaternion_xyzw(working_pose[:3, :3])
        p.resetBasePositionAndOrientation(self.tool_body, working_pose[:3, 3], quaternion, physicsClientId=self.client)
        active_bodies = list(self.wall_bodies)
        target_body = None
        neighbor_bodies: list[int] = []
        if box_size is not None:
            target_body, all_neighbors = self._box_set(box_size)
            neighbor_bodies = all_neighbors[: len(neighbor_centers)]
            for body, center in zip(neighbor_bodies, neighbor_centers):
                p.resetBasePositionAndOrientation(body, center, [0, 0, 0, 1], physicsClientId=self.client)
            for body in all_neighbors[len(neighbor_centers) :]:
                p.resetBasePositionAndOrientation(body, [10, 10, 10], [0, 0, 0, 1], physicsClientId=self.client)
            if box_pose is not None:
                p.resetBasePositionAndOrientation(target_body, box_pose[:3, 3], _matrix_quaternion_xyzw(box_pose[:3, :3]), physicsClientId=self.client)
            else:
                p.resetBasePositionAndOrientation(target_body, [10, 10, 10], [0, 0, 0, 1], physicsClientId=self.client)
            active_bodies.extend(neighbor_bodies)
        if blade_progress is None:
            p.resetBasePositionAndOrientation(self.blade_body, [10, 10, 10], [0, 0, 0, 1], physicsClientId=self.client)
        else:
            assert box_size is not None
            width = min(float(box_size[0]) * 0.9, float(self.cfg["gripper"]["support_blade_width_m"]))
            # The fixed shape is conservative when a small carton needs a narrower blade.
            local = np.array([0.0, -0.5 * float(box_size[1]) - 0.5 * self.cfg["gripper"]["support_blade_thickness_m"], -0.5 * self.cfg["gripper"]["support_extension_m"] + blade_progress * self.cfg["gripper"]["support_extension_m"]])
            blade_position = working_pose[:3, :3] @ local + working_pose[:3, 3]
            p.resetBasePositionAndOrientation(self.blade_body, blade_position, quaternion, physicsClientId=self.client)
        p.performCollisionDetection(physicsClientId=self.client)

        disabled_names = {
            frozenset(pair)
            for pair in (("base_link", "J1_link"), ("J1_link", "J2_link"), ("J2_link", "J3_link"), ("J3_link", "J4_link"), ("J4_link", "J5_link"), ("J5_link", "J6_link"), ("J4_link", "J6_link"))
        }
        for point in p.getClosestPoints(self.robot_body, self.robot_body, self.margin, physicsClientId=self.client):
            first, second = self.link_names.get(int(point[3]), str(point[3])), self.link_names.get(int(point[4]), str(point[4]))
            if first != second and frozenset((first, second)) not in disabled_names:
                return False, "robot_self_collision"
        for body in self.wall_bodies:
            if p.getClosestPoints(self.robot_body, body, self.margin, physicsClientId=self.client):
                return False, "robot_trailer_collision"
        for body in neighbor_bodies:
            if p.getClosestPoints(self.robot_body, body, self.margin, physicsClientId=self.client):
                return False, "robot_neighbor_collision"
        for body in active_bodies:
            if p.getClosestPoints(self.tool_body, body, self.margin, physicsClientId=self.client):
                return False, "tool_neighbor_collision"
            if blade_progress is not None and p.getClosestPoints(self.blade_body, body, self.margin, physicsClientId=self.client):
                return False, "support_insertion_collision"
        if target_body is not None and box_pose is not None:
            for wall in self.wall_bodies:
                if p.getClosestPoints(target_body, wall, self.margin, physicsClientId=self.client):
                    return False, "carried_carton_trailer_collision"
            for neighbor in neighbor_bodies:
                if p.getClosestPoints(target_body, neighbor, self.margin, physicsClientId=self.client):
                    return False, "carried_carton_neighbor_collision"
            # Ignore J6/flange proximity, but not contact with the structural arm.
            for point in p.getClosestPoints(self.robot_body, target_body, self.margin, physicsClientId=self.client):
                if self.link_names.get(int(point[3])) not in {"J5_link", "J6_link", "flange", "tool0"}:
                    return False, "robot_carried_carton_collision"
        return True, "clear"


class StudyRunner:
    def __init__(self, cfg: dict, config_path: Path, backend: str, quick: bool = False) -> None:
        self.cfg, self.config_path, self.quick = cfg, config_path, quick
        self.tool_boxes, self.mass_properties = load_gripper_model(cfg, config_path)
        urdf = resolve_project_path(config_path, cfg["robot"]["urdf"])
        self.robot = URDFRobot6.fanuc_m20id35(
            urdf_path=urdf,
            base_position=[0.0, 0.0, 0.0],
            base_rpy=cfg["robot"]["base_rpy_rad"],
            tool_length=float(cfg["gripper"]["requested_axial_length_m"]),
            tool_collision_local_boxes=self.tool_boxes,
        )
        self.world = CollisionWorld(cfg, config_path, self.robot, self.tool_boxes, backend)
        self.ik_cache: dict[tuple, tuple[list[IKResult], IKResult | None]] = {}
        self.geometry_cache: dict[tuple, GeometryResult] = {}
        grid_cfg = cfg["grid"]
        self.x_values = inclusive_grid(*grid_cfg["cross_section_x_range_m"], grid_cfg["spacing_m"])
        self.z_values = inclusive_grid(*grid_cfg["z_range_m"], grid_cfg["spacing_m"])
        if quick:
            self.x_values = self.x_values[np.linspace(0, len(self.x_values) - 1, 3, dtype=int)]
            self.z_values = self.z_values[np.linspace(0, len(self.z_values) - 1, 3, dtype=int)]
        self.home = np.asarray([-0.3766, -0.7955, -0.8172, 0.8419, -1.3, -0.3599])

    def close(self) -> None:
        self.world.close()

    def _relative_target(self, pose: np.ndarray, mount: tuple[float, float]) -> np.ndarray:
        relative = pose.copy()
        relative[:3, 3] -= [self.cfg["robot"]["base_longitudinal_x_m"], mount[0], mount[1]]
        return relative

    def solve_branches(self, target: np.ndarray, key_hint: tuple, preferred: np.ndarray | None = None) -> tuple[list[IKResult], IKResult | None]:
        decimals = tuple(np.round(target[:3, 3], 6)) + tuple(np.round(target[:3, :3].reshape(-1), 6))
        key = (decimals, key_hint)
        if key in self.ik_cache:
            return self.ik_cache[key]
        thresholds = self.cfg["thresholds"]
        seed_value = stable_seed(int(self.cfg["seed"]), *decimals, *key_hint)
        rng = np.random.default_rng(seed_value)
        seeds = [self.home, np.mean(self.robot.joint_limits, axis=1)]
        if preferred is not None:
            seeds.insert(0, preferred)
        count = 5 if self.quick else 12
        seeds.extend(rng.uniform(self.robot.joint_limits[:, 0], self.robot.joint_limits[:, 1]) for _ in range(count))
        solutions: list[IKResult] = []
        best_failure: IKResult | None = None
        if self.world.backend == "pybullet":
            raw_solutions, best_failure = self.world.inverse_kinematics(
                target,
                seeds,
                float(thresholds["ik_position_tolerance_m"]),
                math.radians(float(thresholds["ik_orientation_tolerance_deg"])),
            )
            # PyBullet rapidly supplies a nearby branch for path tracking. At
            # the grasp pose, retain a sparse set of independent core starts
            # so the reported branch count and collision alternatives are not
            # reduced to whichever null-space branch Bullet returns first.
            if key_hint[-1] != "path":
                for seed in seeds[::3]:
                    candidate = solve_ik(
                        self.robot,
                        target,
                        np.asarray(seed),
                        max_iterations=140,
                        damping=0.035,
                        max_step=0.22,
                        position_tolerance=float(thresholds["ik_position_tolerance_m"]),
                        orientation_tolerance=math.radians(float(thresholds["ik_orientation_tolerance_deg"])),
                        orientation_weight=0.7,
                    )
                    if candidate.success:
                        raw_solutions.append(candidate)
                    elif best_failure is None or candidate.position_error + 0.25 * candidate.orientation_error < best_failure.position_error + 0.25 * best_failure.orientation_error:
                        best_failure = candidate
        else:
            raw_solutions = []
            for seed in seeds:
                result = solve_ik(
                    self.robot,
                    target,
                    np.asarray(seed),
                    max_iterations=90 if self.quick else 180,
                    damping=0.035,
                    max_step=0.22,
                    position_tolerance=float(thresholds["ik_position_tolerance_m"]),
                    orientation_tolerance=math.radians(float(thresholds["ik_orientation_tolerance_deg"])),
                    orientation_weight=0.7,
                )
                if result.success:
                    raw_solutions.append(result)
                elif best_failure is None or result.position_error + 0.25 * result.orientation_error < best_failure.position_error + 0.25 * best_failure.orientation_error:
                    best_failure = result
        for result in raw_solutions:
            if not any(np.linalg.norm(np.arctan2(np.sin(result.q - prior.q), np.cos(result.q - prior.q))) < float(thresholds["joint_branch_dedup_rad"]) for prior in solutions):
                solutions.append(result)
        output = (solutions, best_failure)
        self.ik_cache[key] = output
        return output

    def _joint_metrics(self, q: np.ndarray) -> tuple[float, float]:
        margin = np.min(np.minimum(q - self.robot.joint_limits[:, 0], self.robot.joint_limits[:, 1] - q))
        singular_values = np.linalg.svd(self.robot.geometric_jacobian(q), compute_uv=False)
        condition = float("inf") if singular_values[-1] < 1e-9 else float(singular_values[0] / singular_values[-1])
        return math.degrees(float(margin)), condition

    def _path_poses(self, contact_pose: np.ndarray, outward: np.ndarray) -> list[np.ndarray]:
        thresholds = self.cfg["thresholds"]
        pre, retreat = float(thresholds["prepull_distance_m"]), float(thresholds["initial_retreat_distance_m"])
        step = float(thresholds["ccd_cartesian_step_m"])
        poses: list[np.ndarray] = [contact_pose]
        for distance in np.linspace(step, pre, max(2, int(math.ceil(pre / step)))):
            pose = contact_pose.copy()
            pose[:3, 3] += outward * distance
            poses.append(pose)
        # The transfer zone is at the trailer opening. A pure -X segment is
        # reproducible, cache-friendly and does not sweep a carried carton
        # sideways across neighbouring columns.
        direction = np.array([-1.0, 0.0, 0.0])
        start = poses[-1].copy()
        for distance in np.linspace(step, retreat, max(2, int(math.ceil(retreat / step)))):
            pose = start.copy()
            pose[:3, 3] += direction * distance
            poses.append(pose)
        return poses

    def geometry(self, mount: tuple[float, float], lateral_x: float, z: float, size: Sequence[float] | None, face: str) -> GeometryResult:
        size_key = None if size is None else tuple(float(v) for v in size)
        cache_key = (round(mount[0], 4), round(mount[1], 4), round(lateral_x, 4), round(z, 4), size_key, face, self.world.backend)
        if cache_key in self.geometry_cache:
            return self.geometry_cache[cache_key]
        if size is not None and not carton_inside_trailer(self.cfg, lateral_x, z, size):
            result = GeometryResult(False, False, 0, None, [], [], "not_checked", "box_outside_trailer", 0.0, 0.0, float("inf"), False)
            self.geometry_cache[cache_key] = result
            return result
        contact, box_center, outward = target_pose(self.cfg, lateral_x, z, size, face)
        rotations = target_rotations(face, lateral_x)
        all_solutions: list[tuple[IKResult, np.ndarray]] = []
        best_failure: IKResult | None = None
        for roll_index, rotation in enumerate(rotations):
            world_pose = transform_matrix(rotation, contact)
            relative = self._relative_target(world_pose, mount)
            solutions, failure = self.solve_branches(relative, (face, roll_index))
            all_solutions.extend((solution, world_pose) for solution in solutions)
            if failure is not None and (best_failure is None or failure.position_error < best_failure.position_error):
                best_failure = failure
        ik_count = len(all_solutions)
        if not all_solutions:
            near_limit = best_failure is not None and np.min(np.minimum(best_failure.q - self.robot.joint_limits[:, 0], self.robot.joint_limits[:, 1] - best_failure.q)) < 1e-4
            reason = "joint_limit" if near_limit else "ik_unreachable"
            result = GeometryResult(False, False, 0, None, [], [], "not_checked", reason, float("inf"), 0.0, float("inf"), False)
            self.geometry_cache[cache_key] = result
            return result
        neighbor_centers = [] if size is None else neighboring_carton_centers(self.cfg, box_center, size)
        collision_failures: list[str] = []
        for solution, world_pose in sorted(all_solutions, key=lambda item: np.linalg.norm(item[0].q - self.home)):
            q0 = solution.q
            current_fk = self.robot.fk(q0)
            normal_error = math.degrees(math.acos(float(np.clip(current_fk[:3, 2] @ world_pose[:3, 2], -1.0, 1.0))))
            if normal_error > float(self.cfg["thresholds"]["surface_normal_error_deg"]):
                continue
            if size is None:
                clear, reason = self.world.check(q0, mount, world_pose, None, None, [])
                if not clear:
                    collision_failures.append(reason)
                    continue
                margin, condition = self._joint_metrics(q0)
                derated = margin < float(self.cfg["thresholds"]["joint_limit_derate_margin_deg"]) or condition > float(self.cfg["thresholds"]["jacobian_condition_derate"])
                result = GeometryResult(True, derated, ik_count, q0, [q0], [world_pose], "clear", "", normal_error, margin, condition, True)
                self.geometry_cache[cache_key] = result
                return result

            box_world = transform_matrix(np.eye(3), box_center)
            tool_to_box = np.linalg.inv(world_pose) @ box_world
            poses = self._path_poses(world_pose, outward)
            path: list[np.ndarray] = []
            checked_poses: list[np.ndarray] = []
            previous = q0
            path_failed = ""
            maximum_condition = 0.0
            minimum_margin = float("inf")
            prepull_pose_index = max(2, int(math.ceil(self.cfg["thresholds"]["prepull_distance_m"] / self.cfg["thresholds"]["ccd_cartesian_step_m"])))
            insertion_q = insertion_pose = carried_pose_at_insertion = None
            base_translation = np.asarray([self.cfg["robot"]["base_longitudinal_x_m"], mount[0], mount[1]], dtype=float)
            for pose_index, pose in enumerate(poses):
                if pose_index == 0:
                    q = q0
                else:
                    relative = self._relative_target(pose, mount)
                    solutions, _failure = self.solve_branches(relative, (face, "path"), preferred=previous)
                    if not solutions:
                        path_failed = "loaded_retreat_ik"
                        break
                    q = min(solutions, key=lambda item: np.linalg.norm(np.arctan2(np.sin(item.q - previous), np.cos(item.q - previous)))).q
                delta = np.arctan2(np.sin(q - previous), np.cos(q - previous))
                subdivisions = max(1, int(math.ceil(float(np.max(np.abs(delta))) / float(self.cfg["thresholds"]["ccd_joint_step_rad"]))))
                for alpha in np.linspace(1.0 / subdivisions, 1.0, subdivisions):
                    sample_q = previous + alpha * delta
                    actual_pose = self.robot.fk(sample_q)
                    actual_pose[:3, 3] += base_translation
                    carried_pose = actual_pose @ tool_to_box
                    clear, reason = self.world.check(sample_q, mount, actual_pose, carried_pose, size, neighbor_centers)
                    if not clear:
                        path_failed = reason
                        break
                    margin, condition = self._joint_metrics(sample_q)
                    minimum_margin, maximum_condition = min(minimum_margin, margin), max(maximum_condition, condition)
                    path.append(sample_q.copy())
                    checked_poses.append(actual_pose.copy())
                if path_failed:
                    break
                if pose_index == prepull_pose_index:
                    insertion_q = q.copy()
                    insertion_pose = checked_poses[-1].copy()
                    carried_pose_at_insertion = insertion_pose @ tool_to_box
                previous = q
            if path_failed:
                collision_failures.append(path_failed)
                continue
            insertion_clear = True
            assert insertion_q is not None and insertion_pose is not None and carried_pose_at_insertion is not None
            for progress in np.linspace(0.0, 1.0, int(self.cfg["thresholds"]["support_extension_steps"])):
                clear, reason = self.world.check(insertion_q, mount, insertion_pose, carried_pose_at_insertion, size, neighbor_centers, float(progress))
                if not clear:
                    collision_failures.append(reason)
                    insertion_clear = False
                    break
            if not insertion_clear:
                continue
            derated = minimum_margin < float(self.cfg["thresholds"]["joint_limit_derate_margin_deg"]) or maximum_condition > float(self.cfg["thresholds"]["jacobian_condition_derate"])
            result = GeometryResult(True, derated, ik_count, q0, path, checked_poses, "clear", "", normal_error, minimum_margin, maximum_condition, True)
            self.geometry_cache[cache_key] = result
            return result
        reason = collision_failures[0] if collision_failures else "surface_normal_error"
        result = GeometryResult(False, False, ik_count, None, [], [], "collision" if collision_failures else "not_checked", reason, float("inf"), 0.0, float("inf"), False)
        self.geometry_cache[cache_key] = result
        return result

    def scenario(self, mount: tuple[float, float], lateral_x: float, z: float, size: Sequence[float] | None, face: str, mass: float | None) -> ScenarioResult:
        geometry = self.geometry(mount, lateral_x, z, size, face)
        if not geometry.passed:
            return ScenarioResult(STATE_A, geometry, None)
        if mass is None or size is None:
            return ScenarioResult(STATE_B if geometry.derated else STATE_C, geometry, None)
        worst: WristLoadResult | None = None
        for q, pose in zip(geometry.path, geometry.poses):
            load = wrist_load_check(self.cfg, self.robot, q, pose, self.mass_properties, mass, size, face)
            if worst is None or load.maximum_utilization > worst.maximum_utilization:
                worst = load
        assert worst is not None
        if not worst.passed:
            failed_geometry = GeometryResult(**{**asdict(geometry), "passed": False, "failure_reason": worst.reason})
            if failed_geometry.q is not None:
                failed_geometry.q = np.asarray(failed_geometry.q)
            failed_geometry.path = [np.asarray(q) for q in failed_geometry.path]
            failed_geometry.poses = [np.asarray(pose) for pose in failed_geometry.poses]
            return ScenarioResult(STATE_A, failed_geometry, worst)
        return ScenarioResult(STATE_B if geometry.derated or worst.derated else STATE_C, geometry, worst)

    @staticmethod
    def _failure_bucket(reason: str) -> str:
        if reason in {"joint_limit"}:
            return "joint_limit"
        if reason.startswith("wrist") or reason == "payload_mass":
            return "wrist_load"
        if reason.startswith("robot"):
            return "robot_collision"
        if "carton" in reason or reason.startswith("tool") or reason.startswith("support"):
            return "loaded_carton_collision"
        return "other"

    def mount_metrics(self, mount: tuple[float, float], mass: float) -> tuple[dict, dict[tuple[float, float, tuple[float, ...]], ScenarioResult]]:
        sizes = [tuple(size) for size in self.cfg["cartons"]["sizes_m"]]
        records: dict[tuple[float, float, tuple[float, ...]], ScenarioResult] = {}
        for z in self.z_values:
            for x in self.x_values:
                for size in sizes:
                    records[(float(x), float(z), size)] = self.scenario(mount, float(x), float(z), size, "front", mass)
        total = len(records)
        reachable = sum(result.state > STATE_A for result in records.values())
        normal = sum(result.state == STATE_C for result in records.values())

        def region_rate(predicate) -> float:
            region = [result for (x, z, _size), result in records.items() if predicate(x, z)]
            return 0.0 if not region else sum(result.state > STATE_A for result in region) / len(region)

        width = float(self.cfg["trailer"]["inside_width_m"])
        height = float(self.cfg["trailer"]["inside_height_m"])
        corner_rates = [
            region_rate(lambda x, z, sx=sx, sz=sz: (x <= -0.5 * width + 0.15 if sx < 0 else x >= 0.5 * width - 0.15) and (z <= 0.2 if sz < 0 else z >= height - 0.2))
            for sx in (-1, 1)
            for sz in (-1, 1)
        ]
        failures = {name: 0 for name in ("joint_limit", "wrist_load", "robot_collision", "loaded_carton_collision", "other")}
        for result in records.values():
            if result.state == STATE_A:
                failures[self._failure_bucket(result.geometry.failure_reason)] += 1
        metric = {
            "mount_lateral_offset_m": mount[0],
            "mounting_surface_height_m": mount[1],
            "j1_center_height_m": mount[1] + float(self.cfg["robot"]["mounting_surface_to_j1_center_m"]),
            "mass_kg": mass,
            "task_reachability_rate": reachable / total,
            "normal_task_rate": normal / total,
            "top_200mm_rate": region_rate(lambda _x, z: z >= height - 0.2),
            "bottom_200mm_rate": region_rate(lambda _x, z: z <= 0.2),
            "left_150mm_rate": region_rate(lambda x, _z: x >= 0.5 * width - 0.15),
            "right_150mm_rate": region_rate(lambda x, _z: x <= -0.5 * width + 0.15),
            "minimum_corner_rate": min(corner_rates),
            "average_feasible_ik_branches": float(np.mean([result.geometry.ik_count for result in records.values()])),
            **{f"failure_{name}_ratio": count / total for name, count in failures.items()},
        }
        return metric, records


def _write_heatmap(path: Path, title: str, states: np.ndarray, x_values: np.ndarray, z_values: np.ndarray) -> None:
    from matplotlib import pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch

    cmap = ListedColormap(["#4b5563", "#f59e0b", "#16a34a"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    fig, ax = plt.subplots(figsize=(8.0, 7.2), constrained_layout=True)
    dx = 0.5 * (x_values[1] - x_values[0]) if len(x_values) > 1 else 0.025
    dz = 0.5 * (z_values[1] - z_values[0]) if len(z_values) > 1 else 0.025
    ax.imshow(states, origin="lower", interpolation="nearest", aspect="equal", extent=[x_values[0] - dx, x_values[-1] + dx, z_values[0] - dz, z_values[-1] + dz], cmap=cmap, norm=norm)
    ax.set(xlim=(-1.15, 1.15), ylim=(0.0, 2.7), xlabel="Trailer cross-section x (m; + left)", ylabel="Height z (m)", title=title)
    ax.legend(handles=[Patch(color="#4b5563", label="A unreachable"), Patch(color="#f59e0b", label="B derated / partial"), Patch(color="#16a34a", label="C normal")], loc="lower center", ncol=3, framealpha=0.95)
    ax.grid(color="white", alpha=0.12, linewidth=0.4)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _summary_grid(runner: StudyRunner, records: dict, mass: float) -> np.ndarray:
    sizes = [tuple(size) for size in runner.cfg["cartons"]["sizes_m"]]
    matrix = np.zeros((len(runner.z_values), len(runner.x_values)), dtype=np.int8)
    for zi, z in enumerate(runner.z_values):
        for xi, x in enumerate(runner.x_values):
            matrix[zi, xi] = aggregate_cell_states(records[(float(x), float(z), sizes[0])].state for _size in [0]) if len(sizes) == 1 else aggregate_cell_states(records[(float(x), float(z), size)].state for size in sizes)
    return matrix


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError(f"no rows for {path}")
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _scan_offset_worker(payload: tuple[str, str, bool, float, list[float]]) -> list[dict]:
    """Process one lateral offset so its seven heights share the IK cache."""
    config_name, backend, quick, offset, heights = payload
    cfg, config_path = load_config(config_name)
    runner = StudyRunner(cfg, config_path, backend, quick)
    metrics: list[dict] = []
    try:
        for height in heights:
            mount = (float(offset), float(height))
            for mass in cfg["cartons"]["masses_kg"]:
                metric, _records = runner.mount_metrics(mount, float(mass))
                metrics.append(metric)
                print(f"scan mount_y={mount[0]:+.2f} m height={mount[1]:.2f} m mass={mass:.0f} kg rate={metric['task_reachability_rate']:.3f}", flush=True)
    finally:
        runner.close()
    return metrics


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="studies/fanuc_m20id35_cross_section/study_config.json")
    parser.add_argument("--backend", choices=("pybullet", "geometric"), default="pybullet")
    parser.add_argument("--quick", action="store_true", help="small deterministic smoke grid")
    parser.add_argument("--probe", action="store_true", help="evaluate one representative centre/large-carton scenario")
    parser.add_argument("--workers", type=int, default=1, help="parallel lateral-offset workers (recommended: 9)")
    args = parser.parse_args(argv)
    cfg, config_path = load_config(args.config)
    output = resolve_project_path(config_path, cfg["output_directory"])
    output.mkdir(parents=True, exist_ok=True)
    runner = StudyRunner(cfg, config_path, args.backend, args.quick)
    start = time.perf_counter()
    if args.probe:
        try:
            size = tuple(cfg["cartons"]["sizes_m"][-1])
            result = runner.scenario((0.0, 0.8), 0.0, 1.35, size, "front", 25.0)
            print(json.dumps(_jsonable({
                "state": STATE_LABELS[result.state],
                "geometry_passed": result.geometry.passed,
                "failure_reason": result.geometry.failure_reason,
                "ik_branch_count": result.geometry.ik_count,
                "checked_path_samples": len(result.geometry.path),
                "collision_status": result.geometry.collision_status,
                "load": None if result.load is None else asdict(result.load),
                "ik_failure_residuals": [
                    [failure.position_error, failure.orientation_error]
                    for _solutions, failure in runner.ik_cache.values()
                    if failure is not None
                ][:8],
            }), indent=2))
        finally:
            runner.close()
        return
    metrics: list[dict] = []
    scan_records: dict[tuple[float, float, float], dict] = {}
    offsets = cfg["mount_scan"]["lateral_offsets_m"]
    heights = cfg["mount_scan"]["mounting_surface_heights_m"]
    if args.quick:
        offsets, heights = offsets[::4], heights[::3]
    try:
        if args.workers > 1:
            from concurrent.futures import ProcessPoolExecutor

            payloads = [(str(config_path), args.backend, args.quick, float(offset), [float(height) for height in heights]) for offset in offsets]
            with ProcessPoolExecutor(max_workers=min(args.workers, len(payloads))) as pool:
                for group_metrics in pool.map(_scan_offset_worker, payloads):
                    metrics.extend(group_metrics)
        else:
            for height in heights:
                for offset in offsets:
                    mount = (float(offset), float(height))
                    for mass in cfg["cartons"]["masses_kg"]:
                        metric, records = runner.mount_metrics(mount, float(mass))
                        metrics.append(metric)
                        scan_records[(mount[0], mount[1], float(mass))] = records
                        print(f"scan mount_y={mount[0]:+.2f} m height={mount[1]:.2f} m mass={mass:.0f} kg rate={metric['task_reachability_rate']:.3f}", flush=True)
        candidates = [metric for metric in metrics if metric["mass_kg"] == 25.0]
        best = max(candidates, key=lambda metric: (metric["task_reachability_rate"], metric["normal_task_rate"], metric["minimum_corner_rate"], -abs(metric["mount_lateral_offset_m"]), -metric["mounting_surface_height_m"]))
        best_mount = (float(best["mount_lateral_offset_m"]), float(best["mounting_surface_height_m"]))

        best_records: dict[float, dict] = {}
        for mass in cfg["cartons"]["masses_kg"]:
            key = (best_mount[0], best_mount[1], float(mass))
            if key in scan_records:
                best_records[float(mass)] = scan_records[key]
            else:
                metric, records = runner.mount_metrics(best_mount, float(mass))
                best_records[float(mass)] = records

        empty_states = np.zeros((len(runner.z_values), len(runner.x_values)), dtype=np.int8)
        empty_results: dict[tuple[float, float], ScenarioResult] = {}
        for zi, z in enumerate(runner.z_values):
            for xi, x in enumerate(runner.x_values):
                result = runner.scenario(best_mount, float(x), float(z), None, "front", None)
                empty_results[(float(x), float(z))] = result
                empty_states[zi, xi] = result.state
        _write_heatmap(output / "heatmap_empty.png", "Empty geometric reachability", empty_states, runner.x_values, runner.z_values)
        for mass in cfg["cartons"]["masses_kg"]:
            states = _summary_grid(runner, best_records[float(mass)], float(mass))
            _write_heatmap(output / f"heatmap_{int(mass)}kg.png", f"{int(mass)} kg front-grasp task reachability (3 carton sizes)", states, runner.x_values, runner.z_values)

        # Side-face results are intentionally separate and retained in the detailed CSV.
        side_records: dict[tuple[float, float, tuple[float, ...], float], ScenarioResult] = {}
        for mass in cfg["cartons"]["masses_kg"]:
            for z in runner.z_values:
                for x in runner.x_values:
                    for size in cfg["cartons"]["sizes_m"]:
                        side_records[(float(x), float(z), tuple(size), float(mass))] = runner.scenario(best_mount, float(x), float(z), size, "side", float(mass))

        # Z axis selects the best task state for each scenario; non-zero is counted only when selected.
        z_offsets = cfg["z_axis"]["offsets_m"]
        fixed_25 = best_records[25.0]
        zaxis_25: dict = {}
        zaxis_calls = 0
        for key, fixed_result in fixed_25.items():
            x, z, size = key
            options = [(0.0, fixed_result)]
            for delta in z_offsets:
                if abs(float(delta)) < 1e-12:
                    continue
                mount = (best_mount[0], best_mount[1] + float(delta))
                options.append((float(delta), runner.scenario(mount, x, z, size, "front", 25.0)))
            selected_delta, selected = max(options, key=lambda item: (item[1].state, -abs(item[0])))
            zaxis_25[key] = selected
            zaxis_calls += int(abs(selected_delta) > 1e-12 and selected.state > fixed_result.state)
        fixed_matrix = _summary_grid(runner, fixed_25, 25.0)
        zaxis_matrix = _summary_grid(runner, zaxis_25, 25.0)
        from matplotlib import pyplot as plt
        from matplotlib.colors import BoundaryNorm, ListedColormap
        cmap = ListedColormap(["#4b5563", "#f59e0b", "#16a34a"])
        norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
        fig, axes = plt.subplots(1, 2, figsize=(12, 6.6), sharex=True, sharey=True, constrained_layout=True)
        for ax, matrix, title in zip(axes, (fixed_matrix, zaxis_matrix), ("Best fixed base — 25 kg", "±200 mm Z axis — 25 kg")):
            ax.imshow(matrix, origin="lower", interpolation="nearest", aspect="equal", extent=[-1.175, 1.175, -0.025, 2.725], cmap=cmap, norm=norm)
            ax.set(xlim=(-1.15, 1.15), ylim=(0, 2.7), xlabel="Cross-section x (m)", title=title)
        axes[0].set_ylabel("Height z (m)")
        fig.savefig(output / "fixed_vs_zaxis.png", dpi=180)
        plt.close(fig)

        reach_rows: list[dict] = []
        failure_rows: list[dict] = []

        def append_row(x: float, z: float, size, mass, face: str, result: ScenarioResult) -> None:
            geometry, load = result.geometry, result.load
            q_text = "" if geometry.q is None else json.dumps([round(float(v), 8) for v in geometry.q], separators=(",", ":"))
            size_text = "" if size is None else "x".join(f"{float(v):.3f}" for v in size)
            reach_rows.append({
                "x_m": f"{x:.3f}", "z_m": f"{z:.3f}", "mass_kg": "" if mass is None else f"{mass:.1f}", "carton_size_w_h_d_m": size_text, "grasp_face": face,
                "reachability_state": STATE_LABELS[result.state], "ik_branch_count": geometry.ik_count, "best_joint_angles_rad": q_text,
                "payload_status": "not_applicable" if load is None else load.reason, "maximum_wrist_utilization": "" if load is None else f"{load.maximum_utilization:.6f}",
                "collision_status": geometry.collision_status, "support_insertion_clear": geometry.support_insertion_clear, "failure_reason": geometry.failure_reason,
            })
            bucket = runner._failure_bucket(geometry.failure_reason) if geometry.failure_reason else "none"
            failure_rows.append({
                "x_m": f"{x:.3f}", "z_m": f"{z:.3f}", "mass_kg": "" if mass is None else f"{mass:.1f}", "carton_size_w_h_d_m": size_text, "grasp_face": face,
                "failed": result.state == STATE_A, "primary_failure_reason": geometry.failure_reason, "joint_limit_failure": bucket == "joint_limit", "wrist_load_or_inertia_failure": bucket == "wrist_load",
                "robot_collision_failure": bucket == "robot_collision", "loaded_carton_or_tool_collision_failure": bucket == "loaded_carton_collision",
            })

        for (x, z), result in empty_results.items():
            append_row(x, z, None, None, "front", result)
        for mass, records in best_records.items():
            for (x, z, size), result in records.items():
                append_row(x, z, size, mass, "front", result)
        for (x, z, size, mass), result in side_records.items():
            append_row(x, z, size, mass, "side", result)
        _write_csv(output / "reachability_matrix.csv", reach_rows)
        _write_csv(output / "failure_reason_matrix.csv", failure_rows)
        _write_csv(output / "mounting_scan_metrics.csv", metrics)

        def rate(records, predicate=lambda _x, _z: True) -> float:
            selected = [result for (x, z, _size), result in records.items() if predicate(x, z)]
            return 0.0 if not selected else sum(result.state > STATE_A for result in selected) / len(selected)

        height, width = cfg["trailer"]["inside_height_m"], cfg["trailer"]["inside_width_m"]
        z_compare = {
            "fixed_25kg_rate": rate(fixed_25),
            "zaxis_25kg_rate": rate(zaxis_25),
            "top_200mm_improvement": rate(zaxis_25, lambda _x, z: z >= height - 0.2) - rate(fixed_25, lambda _x, z: z >= height - 0.2),
            "bottom_200mm_improvement": rate(zaxis_25, lambda _x, z: z <= 0.2) - rate(fixed_25, lambda _x, z: z <= 0.2),
            "corner_improvement": rate(zaxis_25, lambda x, z: abs(x) >= 0.5 * width - 0.15 and (z <= 0.2 or z >= height - 0.2)) - rate(fixed_25, lambda x, z: abs(x) >= 0.5 * width - 0.15 and (z <= 0.2 or z >= height - 0.2)),
            "z_axis_called_grid_scenario_ratio": zaxis_calls / len(zaxis_25),
        }
        recommendation = z_compare["zaxis_25kg_rate"] - z_compare["fixed_25kg_rate"] >= 0.05 or z_compare["corner_improvement"] >= 0.10
        best_payload = {
            "schema_version": cfg["schema_version"],
            "backend": args.backend,
            "quick_mode": args.quick,
            "best_fixed_mount": best,
            "best_mount_lateral_offset_m": best_mount[0],
            "best_mounting_surface_height_m": best_mount[1],
            "best_j1_center_height_m": best_mount[1] + cfg["robot"]["mounting_surface_to_j1_center_m"],
            "z_axis_comparison": z_compare,
            "recommend_short_z_axis": recommendation,
            "optimization_objective": "maximize 25 kg front-grasp reachable scenarios over all three required carton sizes; tie-break normal rate, worst corner, centering and lower mount",
        }
        (output / "best_mounting_parameters.json").write_text(json.dumps(_jsonable(best_payload), ensure_ascii=False, indent=2), encoding="utf-8")
        parameters = {**cfg, "resolved": {"backend": args.backend, "quick_mode": args.quick, "grid_shape_z_x": [len(runner.z_values), len(runner.x_values)], "robot_joint_limits_rad": runner.robot.joint_limits.tolist(), "gripper_rigid_proxy_count": len(runner.tool_boxes), "run_seconds": time.perf_counter() - start}}
        (output / "simulation_parameters.json").write_text(json.dumps(_jsonable(parameters), ensure_ascii=False, indent=2), encoding="utf-8")

        best_mass_metrics = {metric["mass_kg"]: metric for metric in metrics if metric["mount_lateral_offset_m"] == best_mount[0] and metric["mounting_surface_height_m"] == best_mount[1]}
        if len(best_mass_metrics) < 3:
            best_mass_metrics = {mass: next(metric for metric in metrics if metric["mass_kg"] == mass and metric["mount_lateral_offset_m"] == best_mount[0] and metric["mounting_surface_height_m"] == best_mount[1]) for mass in cfg["cartons"]["masses_kg"]}
        failure_keys = ["failure_joint_limit_ratio", "failure_wrist_load_ratio", "failure_robot_collision_ratio", "failure_loaded_carton_collision_ratio", "failure_other_ratio"]
        main_failure = max(failure_keys, key=lambda key: best_mass_metrics[25.0][key]).replace("failure_", "").replace("_ratio", "")
        region_rates = {key: best_mass_metrics[25.0][key] for key in ("top_200mm_rate", "bottom_200mm_rate", "left_150mm_rate", "right_150mm_rate", "minimum_corner_rate")}
        worst_region = min(region_rates, key=region_rates.get)
        print("\n技术结论")
        print(f"1. 最优固定安装面高度: {best_mount[1]:.3f} m (J1中心 {best_payload['best_j1_center_height_m']:.3f} m)")
        print(f"2. 最优横向位置: {best_mount[0]:+.3f} m")
        print("3. 5/15/25 kg任务可达率: " + ", ".join(f"{mass:.0f} kg={best_mass_metrics[mass]['task_reachability_rate']:.1%}" for mass in (5.0, 15.0, 25.0)))
        print(f"4. 最差区域: {worst_region} ({region_rates[worst_region]:.1%})")
        print(f"5. 主要不可达原因: {main_failure}")
        print(f"6. 增加±200 mm Z轴后的25 kg提升: {z_compare['zaxis_25kg_rate'] - z_compare['fixed_25kg_rate']:+.1%}")
        print(f"7. 是否建议首代机器人设置升降轴: {'建议' if recommendation else '不建议'}")
    finally:
        runner.close()


if __name__ == "__main__":
    main()
