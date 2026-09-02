"""Deterministic load-aware inverse-kinematics task qualification.

This layer joins geometric IK to :mod:`unloading_sim.robot_load.spatial`.
Payload, configured engineering reference screens, collisions, joint limits,
and optional extraction checks are hard task-selection constraints.  They are
not silently promoted to FANUC load-diagram criteria.  A vendor payload/CoM
diagram is fail-closed through an explicit evaluator interface; absence of
vendor data is never converted to a pass or fail.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from typing import Any, Callable, Mapping, Protocol, Sequence

import numpy as np

from unloading_sim.geometry import OBB
from unloading_sim.ik import IKResult, solve_ik
from unloading_sim.robot import URDFRobot

from .model import LoadQualificationResult, RobotLoadLimits, ToolLoad, ToolMassPropertiesSource
from .spatial import BoxSpatialLoad, PoseSpecificLoadCase, qualify_load_v2


class VendorLoadStatus(str, Enum):
    NOT_EVALUATED = "NOT_EVALUATED"
    PASS = "PASS"
    FAIL = "FAIL"


@dataclass(frozen=True)
class VendorLoadEvaluation:
    status: VendorLoadStatus
    reason: str
    evidence: Mapping[str, Any] = field(default_factory=dict)


class VendorLoadDiagramEvaluator(Protocol):
    """Interface for official curves, ROBOGUIDE checks, or entered boundaries."""

    def evaluate(
        self,
        *,
        robot: RobotLoadLimits,
        tool: ToolLoad,
        box: BoxSpatialLoad,
        joint_pose_q: np.ndarray,
        known_result: LoadQualificationResult,
    ) -> VendorLoadEvaluation: ...


class UnevaluatedVendorLoadDiagram:
    """Default evaluator used until traceable vendor load data is supplied."""

    def evaluate(self, **_: Any) -> VendorLoadEvaluation:
        return VendorLoadEvaluation(
            VendorLoadStatus.NOT_EVALUATED,
            "the public FANUC evidence does not define the reference frame and load-diagram boundary",
            {
                "reference": "NOT_EVALUATED_VENDOR_REFERENCE",
                "moment": "MANUFACTURER_MOMENT_DEFINITION_NOT_VERIFIED",
                "inertia": "VENDOR_INERTIA_NOT_EVALUATED",
                "diagram": "NOT_EVALUATED_VENDOR_LOAD_DIAGRAM",
            },
        )


@dataclass(frozen=True)
class TaskScoreWeights:
    load: float = 1.0
    joint_limit: float = 0.08
    singularity: float = 0.04
    collision: float = 0.04
    motion: float = 0.04
    extraction_continuity: float = 0.04

    def __post_init__(self) -> None:
        values = tuple(float(v) for v in asdict(self).values())
        if any(not np.isfinite(v) or v < 0.0 for v in values) or self.load <= 0.0:
            raise ValueError("score weights must be finite and non-negative; load must be positive")


@dataclass(frozen=True)
class TaskLoadSearchOptions:
    rng_seed: int = 20260901
    minimum_candidate_count: int = 20
    random_seeds_per_orientation: int = 3
    solutions_per_orientation: int = 1
    normal_tolerance_rad: float = np.deg2rad(3.0)
    roll_samples_rad: tuple[float, ...] = (0.0, np.pi / 2.0, np.pi, -np.pi / 2.0)
    position_tolerance_m: float = 0.003
    orientation_tolerance_rad: float = np.deg2rad(1.0)
    joint_dedup_rad: float = 1e-3
    max_iterations: int = 180
    damping: float = 0.035
    max_step_rad: float = 0.22

    def __post_init__(self) -> None:
        if self.minimum_candidate_count < 1:
            raise ValueError("minimum_candidate_count must be positive")
        if self.random_seeds_per_orientation < 1 or self.solutions_per_orientation < 1:
            raise ValueError("IK search counts must be positive")
        if not 0.0 <= self.normal_tolerance_rad < np.pi / 2.0:
            raise ValueError("normal_tolerance_rad must be in [0, pi/2)")
        if not self.roll_samples_rad:
            raise ValueError("at least one roll sample is required")


CollisionEvaluator = Callable[[np.ndarray], tuple[bool, float]]
ExtractionEvaluator = Callable[[np.ndarray], tuple[bool, float]]


@dataclass(frozen=True)
class TaskLoadRequest:
    robot: RobotLoadLimits
    kinematics: URDFRobot
    tool: ToolLoad
    box: BoxSpatialLoad
    tcp_pose_world: np.ndarray
    grasp_normal_world: np.ndarray
    current_q: np.ndarray | None = None
    obstacles: tuple[OBB, ...] = ()
    collision_evaluator: CollisionEvaluator | None = None
    extraction_evaluator: ExtractionEvaluator | None = None

    def __post_init__(self) -> None:
        pose = np.asarray(self.tcp_pose_world, dtype=float)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            raise ValueError("tcp_pose_world must be a finite 4x4 transform")
        if not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-8):
            raise ValueError("tcp_pose_world rotation must be orthonormal")
        normal = np.asarray(self.grasp_normal_world, dtype=float)
        if normal.shape != (3,) or not np.all(np.isfinite(normal)) or np.linalg.norm(normal) < 1e-12:
            raise ValueError("grasp_normal_world must be a finite non-zero vector")
        q = None if self.current_q is None else np.asarray(self.current_q, dtype=float)
        if q is not None and (q.shape != (self.kinematics.dof,) or not self.kinematics.within_limits(q)):
            raise ValueError("current_q must be a legal robot pose")
        object.__setattr__(self, "tcp_pose_world", pose)
        object.__setattr__(self, "grasp_normal_world", normal / np.linalg.norm(normal))
        object.__setattr__(self, "current_q", q)
        if abs(float(pose[:3, 0] @ (normal / np.linalg.norm(normal)))) < 0.99:
            raise ValueError("tcp local X axis must be parallel to the supplied grasp normal")


@dataclass(frozen=True)
class LoadAwareIKCandidate:
    q: np.ndarray
    tcp_pose_world: np.ndarray
    ik_position_error_m: float
    ik_orientation_error_rad: float
    known_result: LoadQualificationResult
    vendor_result: VendorLoadEvaluation
    moment_utilization_j4_j5_j6: np.ndarray
    inertia_utilization_j4_j5_j6: np.ndarray
    known_utilization: float
    joint_limit_margin_rad: float
    singularity_margin: float
    collision_clearance_m: float
    joint_motion_distance_rad: float
    extraction_continuity_cost: float
    hard_failure_reasons: tuple[str, ...]
    score: float

    @property
    def known_limits_pass(self) -> bool:
        """Compatibility property: now means engineering references pass."""
        return self.known_result.qualification == "ENGINEERING_REFERENCES_WITHIN_PUBLIC_VALUES"

    @property
    def hard_constraints_pass(self) -> bool:
        return not self.hard_failure_reasons


@dataclass(frozen=True)
class TaskLoadQualificationResult:
    qualification: str
    known_limits_status: str
    vendor_load_status: str
    candidates: tuple[LoadAwareIKCandidate, ...]
    best_candidate: LoadAwareIKCandidate | None
    second_best_candidate: LoadAwareIKCandidate | None
    load_optimal_candidate: LoadAwareIKCandidate | None
    all_failure_reasons: tuple[str, ...]
    primary_failure_reason: str | None
    search_coverage: Mapping[str, Any]

    @property
    def best_q(self) -> np.ndarray | None:
        return None if self.best_candidate is None else self.best_candidate.q

    @property
    def second_best_q(self) -> np.ndarray | None:
        return None if self.second_best_candidate is None else self.second_best_candidate.q

    @property
    def best_U_known(self) -> float | None:
        return None if self.best_candidate is None else self.best_candidate.known_utilization

    @property
    def load_margin(self) -> float | None:
        return None if self.best_U_known is None else 1.0 - self.best_U_known

    @property
    def minimum_j4_j5_inertia_utilization(self) -> float | None:
        if not self.candidates:
            return None
        return min(float(np.max(c.inertia_utilization_j4_j5_j6[:2])) for c in self.candidates)


def scale_engineering_tool_model(reference: ToolLoad, mass_kg: float) -> ToolLoad:
    """Scale an equal-geometry engineering tool model to a requested mass.

    CoM and normalized mass distribution are held constant, so the full 3x3
    CoM inertia tensor scales linearly with mass.  The provenance is explicit
    and does not claim that CAD or a manufacturer certificate exists.
    """
    target = float(mass_kg)
    if not np.isfinite(target) or target <= 0.0:
        raise ValueError("mass_kg must be finite and positive")
    ratio = target / reference.mass_kg
    source = {
        "type": "ENGINEERING_TOOL_MODEL",
        "reference_source": dict(reference.source),
        "reference_mass_kg": reference.mass_kg,
        "target_mass_kg": target,
        "mass_scale": ratio,
        "inertia_scale": ratio,
        "scaling_assumption": "unchanged geometry, CoM, and normalized mass distribution",
        "cad_mass_properties_status": "NOT_AVAILABLE_FOR_THIS_TARGET_MASS",
    }
    return replace(
        reference,
        mass_kg=target,
        inertia_at_com_kg_m2=reference.inertia_at_com_kg_m2 * ratio,
        source=source,
        mass_properties_source=ToolMassPropertiesSource.ENGINEERING_MODEL,
    )


def _axis_rotation(axis: int, angle: float) -> np.ndarray:
    vector = np.zeros(3)
    vector[axis] = 1.0
    x, y, z = vector
    c, s = np.cos(angle), np.sin(angle)
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return c * np.eye(3) + (1.0 - c) * np.outer(vector, vector) + s * skew


def _orientation_variants(pose: np.ndarray, options: TaskLoadSearchOptions) -> list[np.ndarray]:
    # Local +X is the box depth / suction-normal axis in the load model.
    tilts = ((0.0, 0.0),)
    if options.normal_tolerance_rad > 0.0:
        a = options.normal_tolerance_rad
        tilts = ((0.0, 0.0), (a, 0.0), (-a, 0.0), (0.0, a), (0.0, -a))
    variants: list[np.ndarray] = []
    for tilt_y, tilt_z in tilts:
        for roll_x in options.roll_samples_rad:
            variant = pose.copy()
            variant[:3, :3] = (
                pose[:3, :3]
                @ _axis_rotation(1, tilt_y)
                @ _axis_rotation(2, tilt_z)
                @ _axis_rotation(0, float(roll_x))
            )
            variants.append(variant)
    return variants


def _tip_adapter_rotation(request: TaskLoadRequest) -> np.ndarray:
    q = np.mean(request.kinematics.joint_limits, axis=1)
    frames = request.kinematics.named_link_frames(q)
    flange = frames[request.robot.flange_link]
    tip = frames[request.robot.tip_link]
    return flange[:3, :3].T @ tip[:3, :3]


def _ik_target(request: TaskLoadRequest, tcp_pose: np.ndarray, adapter: np.ndarray) -> np.ndarray:
    target = np.eye(4)
    target[:3, :3] = tcp_pose[:3, :3] @ adapter
    target[:3, 3] = tcp_pose[:3, 3] - tcp_pose[:3, :3] @ request.tool.tcp_xyz_m
    return target


def _wrapped_distance(a: np.ndarray, b: np.ndarray) -> float:
    delta = np.arctan2(np.sin(a - b), np.cos(a - b))
    return float(np.linalg.norm(delta))


def _known_failure_reasons(result: LoadQualificationResult) -> list[str]:
    i = result.intermediate
    reasons: list[str] = []
    if float(i["payload_utilization"]) > 1.0 + 1e-12:
        reasons.append("FAIL_PAYLOAD")
    for index, name in enumerate(("J4", "J5", "J6")):
        if float(i["pose_gravity_moment_reference_ratio_j4_j5_j6"][index]) > 1.0 + 1e-12:
            reasons.append(f"POSE_GRAVITY_MOMENT_{name}_PUBLIC_REFERENCE_EXCEEDED")
    for index, name in enumerate(("J4", "J5", "J6")):
        if float(i["engineering_axis_inertia_reference_ratio_j4_j5_j6"][index]) > 1.0 + 1e-12:
            reasons.append(f"ENGINEERING_{name}_AXIS_INERTIA_PUBLIC_REFERENCE_EXCEEDED")
    return reasons


class LoadAwareIKQualifier:
    """Search and rank multiple IK poses for one physical grasp task."""

    def __init__(
        self,
        options: TaskLoadSearchOptions | None = None,
        weights: TaskScoreWeights | None = None,
        vendor_evaluator: VendorLoadDiagramEvaluator | None = None,
    ) -> None:
        self.options = options or TaskLoadSearchOptions()
        self.weights = weights or TaskScoreWeights()
        self.vendor_evaluator = vendor_evaluator or UnevaluatedVendorLoadDiagram()
        self._ik_cache: dict[tuple[Any, ...], tuple[list[tuple[IKResult, np.ndarray]], dict[str, Any]]] = {}

    def _search_ik(self, request: TaskLoadRequest) -> tuple[list[tuple[IKResult, np.ndarray]], dict[str, Any]]:
        options = self.options
        cache_key = (
            id(request.kinematics),
            request.tcp_pose_world.tobytes(),
            request.tool.tcp_xyz_m.tobytes(),
            None if request.current_q is None else request.current_q.tobytes(),
        )
        cached = self._ik_cache.get(cache_key)
        if cached is not None:
            solutions, coverage = cached
            return list(solutions), {**coverage, "cache_hit": True}
        rng = np.random.default_rng(options.rng_seed)
        lower, upper = request.kinematics.joint_limits.T
        base_seeds = [np.mean(request.kinematics.joint_limits, axis=1)]
        if request.robot.qualification_joint_pose_q:
            base_seeds.insert(0, np.asarray(request.robot.qualification_joint_pose_q))
        if request.current_q is not None:
            base_seeds.insert(0, request.current_q)
        random_seeds = [rng.uniform(lower, upper) for _ in range(options.random_seeds_per_orientation)]
        variants = _orientation_variants(request.tcp_pose_world, options)
        adapter = _tip_adapter_rotation(request)
        solutions: list[tuple[IKResult, np.ndarray]] = []
        attempts = 0
        orientations_with_solution = 0
        continuation_seeds: list[np.ndarray] = []
        for tcp_pose in variants:
            target = _ik_target(request, tcp_pose, adapter)
            found_here = 0
            seeds = [*continuation_seeds[-3:], *base_seeds, *random_seeds]
            for seed in seeds:
                attempts += 1
                result = solve_ik(
                    request.kinematics,
                    target,
                    seed,
                    max_iterations=options.max_iterations,
                    damping=options.damping,
                    max_step=options.max_step_rad,
                    position_tolerance=options.position_tolerance_m,
                    orientation_tolerance=options.orientation_tolerance_rad,
                    orientation_weight=0.7,
                )
                if not result.success:
                    continue
                if any(float(np.linalg.norm(result.q - prior.q)) < options.joint_dedup_rad for prior, _ in solutions):
                    continue
                solutions.append((result, tcp_pose.copy()))
                continuation_seeds.append(result.q.copy())
                found_here += 1
                if found_here >= options.solutions_per_orientation:
                    break
            orientations_with_solution += int(found_here > 0)
        # Revolute coordinates separated by 2*pi are distinct legal controller
        # branches when both values lie inside the configured URDF range.  J6
        # on this FANUC spans 5*pi, so retaining these exact equivalents is
        # important and avoids asking a local DLS solver to rediscover wraps.
        direct_solutions = list(solutions)
        periodic_added = 0
        for result, tcp_pose in direct_solutions:
            for index in range(request.kinematics.dof):
                for delta in (-2.0 * np.pi, 2.0 * np.pi):
                    q = result.q.copy()
                    q[index] += delta
                    if not request.kinematics.within_limits(q):
                        continue
                    if any(float(np.linalg.norm(q - prior.q)) < options.joint_dedup_rad for prior, _ in solutions):
                        continue
                    solutions.append((replace(result, q=q), tcp_pose.copy()))
                    periodic_added += 1
        coverage = {
            "rng_seed": options.rng_seed,
            "orientation_variants": len(variants),
            "orientations_with_solution": orientations_with_solution,
            "ik_attempts": attempts,
            "minimum_candidate_target": options.minimum_candidate_count,
            "candidate_target_met": len(solutions) >= options.minimum_candidate_count,
            "search_space_exhausted": True,
            "periodic_equivalent_solutions_added": periodic_added,
            "normal_tolerance_rad": options.normal_tolerance_rad,
            "roll_samples_rad": list(options.roll_samples_rad),
            "cache_hit": False,
        }
        self._ik_cache[cache_key] = (list(solutions), dict(coverage))
        return solutions, coverage

    def evaluate(self, request: TaskLoadRequest) -> TaskLoadQualificationResult:
        solutions, coverage = self._search_ik(request)
        if not solutions:
            return TaskLoadQualificationResult(
                "NO_IK", "NO_IK", VendorLoadStatus.NOT_EVALUATED.value, (), None, None, None,
                ("NO_IK",), "NO_IK", coverage,
            )

        candidates: list[LoadAwareIKCandidate] = []
        for ik_result, tcp_pose in solutions:
            known = qualify_load_v2(
                PoseSpecificLoadCase(request.robot, request.kinematics, request.tool, request.box, ik_result.q)
            )
            vendor = self.vendor_evaluator.evaluate(
                robot=request.robot,
                tool=request.tool,
                box=request.box,
                joint_pose_q=ik_result.q,
                known_result=known,
            )
            moment = np.asarray(known.intermediate["pose_gravity_moment_reference_ratio_j4_j5_j6"], dtype=float)
            inertia = np.asarray(known.intermediate["engineering_axis_inertia_reference_ratio_j4_j5_j6"], dtype=float)
            known_util = float(known.intermediate["maximum_engineering_reference_ratio"])
            q = ik_result.q
            margins = np.minimum(q - request.kinematics.joint_limits[:, 0], request.kinematics.joint_limits[:, 1] - q)
            spans = request.kinematics.joint_limits[:, 1] - request.kinematics.joint_limits[:, 0]
            normalized_margin = float(np.min(margins / spans))
            jacobian_sv = np.linalg.svd(request.kinematics.geometric_jacobian(q), compute_uv=False)
            singularity_margin = float(jacobian_sv[-1] / max(jacobian_sv[0], 1e-12))
            collision_clearance = float("inf")
            collision_pass = True
            if request.collision_evaluator is not None:
                collision_pass, collision_clearance = request.collision_evaluator(q)
            elif request.obstacles:
                collision_pass = request.kinematics.is_collision_free(q, request.obstacles)
                collision_clearance = 0.0 if not collision_pass else float("inf")
            extraction_pass, extraction_cost = True, 0.0
            if request.extraction_evaluator is not None:
                extraction_pass, extraction_cost = request.extraction_evaluator(q)
            motion = 0.0 if request.current_q is None else _wrapped_distance(q, request.current_q)
            failures = _known_failure_reasons(known)
            if not request.kinematics.within_limits(q):
                failures.append("JOINT_LIMIT")
            if not collision_pass:
                failures.append("COLLISION_FAIL")
            if not extraction_pass:
                failures.append("EXTRACTION_FAIL")
            if vendor.status == VendorLoadStatus.FAIL:
                failures.append("FAIL_VENDOR_LOAD_DIAGRAM")
            joint_cost = 1.0 / max(normalized_margin, 1e-6)
            singularity_cost = 1.0 / max(singularity_margin, 1e-6)
            collision_cost = 0.0 if np.isinf(collision_clearance) else 1.0 / max(collision_clearance, 1e-6)
            w = self.weights
            score = (
                w.load * known_util
                + w.joint_limit * joint_cost
                + w.singularity * singularity_cost
                + w.collision * collision_cost
                + w.motion * motion
                + w.extraction_continuity * float(extraction_cost)
            )
            candidates.append(
                LoadAwareIKCandidate(
                    q.copy(), tcp_pose, ik_result.position_error, ik_result.orientation_error,
                    known, vendor, moment, inertia, known_util, float(np.min(margins)),
                    singularity_margin, collision_clearance, motion, float(extraction_cost),
                    tuple(failures), float(score),
                )
            )

        load_optimal = min(candidates, key=lambda c: c.known_utilization)
        feasible = [c for c in candidates if c.hard_constraints_pass]
        # The requested primary objective is q*=argmin U_known.  The composite
        # score breaks utilization ties and never compensates a hard failure.
        ranked = sorted(feasible, key=lambda c: (c.known_utilization, c.score, tuple(c.q)))
        diagnostic_ranked = sorted(candidates, key=lambda c: (c.known_utilization, c.score, tuple(c.q)))
        best = ranked[0] if ranked else diagnostic_ranked[0]
        second = ranked[1] if len(ranked) > 1 else (diagnostic_ranked[1] if len(diagnostic_ranked) > 1 else None)
        engineering_pass = [c for c in candidates if c.known_limits_pass]
        task_feasible = [c for c in engineering_pass if c.hard_constraints_pass]
        reasons = sorted({reason for candidate in candidates for reason in candidate.hard_failure_reasons})
        if not engineering_pass:
            qualification = "ENGINEERING_LOAD_RISK_VENDOR_QUALIFICATION_PENDING"
            known_status = "ENGINEERING_LOAD_RISK"
            reasons.append("VENDOR_QUALIFICATION_PENDING")
        else:
            known_status = "ENGINEERING_REFERENCES_WITHIN_PUBLIC_VALUES"
            vendor_statuses = {c.vendor_result.status for c in task_feasible}
            if not task_feasible:
                qualification = "TASK_GEOMETRIC_CONSTRAINT_FAIL"
            elif VendorLoadStatus.PASS in vendor_statuses:
                qualification = "TASK_LOAD_QUALIFIED"
            elif VendorLoadStatus.NOT_EVALUATED in vendor_statuses:
                qualification = "ENGINEERING_REFERENCES_PASS_VENDOR_QUALIFICATION_PENDING"
                reasons.append("VENDOR_QUALIFICATION_PENDING")
            else:
                qualification = "VENDOR_LOAD_FAIL"
                reasons.append("FAIL_VENDOR_LOAD_DIAGRAM")
        priority = (
            "NO_IK", "FAIL_PAYLOAD",
            "POSE_GRAVITY_MOMENT_J4_PUBLIC_REFERENCE_EXCEEDED",
            "POSE_GRAVITY_MOMENT_J5_PUBLIC_REFERENCE_EXCEEDED",
            "POSE_GRAVITY_MOMENT_J6_PUBLIC_REFERENCE_EXCEEDED",
            "ENGINEERING_J4_AXIS_INERTIA_PUBLIC_REFERENCE_EXCEEDED",
            "ENGINEERING_J5_AXIS_INERTIA_PUBLIC_REFERENCE_EXCEEDED",
            "ENGINEERING_J6_AXIS_INERTIA_PUBLIC_REFERENCE_EXCEEDED",
            "FAIL_VENDOR_LOAD_DIAGRAM", "COLLISION_FAIL", "EXTRACTION_FAIL",
            "VENDOR_QUALIFICATION_PENDING",
        )
        primary = next((reason for reason in priority if reason in reasons), None)
        return TaskLoadQualificationResult(
            qualification,
            known_status,
            best.vendor_result.status.value,
            tuple(candidates),
            best,
            second,
            load_optimal,
            tuple(dict.fromkeys(reasons)),
            primary,
            coverage,
        )
