"""Pure geometry policies consumed by the M-710 Isaac replay adapter.

The functions in this module deliberately have no Isaac/Kit dependencies.  The
CPU test suite therefore exercises the exact attachment and conveyor-selection
logic that the heavyweight runtime consumes, even when Isaac is unavailable.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos
from typing import Any, Mapping, Sequence

import numpy as np


def verify_physics_backend_readback(requested, actual):
    """The configured engine/device policy must be the one PhysX reports."""
    fields = {"mode", "device", "broadphase_type", "gpu_dynamics_enabled", "fabric_enabled", "ccd_enabled"}
    if not isinstance(requested, Mapping) or set(requested) != fields:
        raise ValueError("physics execution backend requires the complete bound policy")
    # This milestone preserves the previous CPU world's CCD. The installed
    # GPU PhysX backend cannot preserve CCD and is therefore not an allowed
    # execution policy, even when GPU rendering remains enabled.
    expected = {"mode": "physx_cpu", "device": "cpu", "broadphase_type": "MBP",
                "gpu_dynamics_enabled": False, "fabric_enabled": True, "ccd_enabled": True}
    if dict(requested) != expected:
        raise ValueError("unsupported or contradictory physics execution backend")
    if not isinstance(actual, Mapping) or dict(actual) != expected:
        raise ValueError(f"physics execution backend readback mismatch: expected {expected}, actual {actual}")
    return {"status": "PASS", "requested": dict(requested), "actual": dict(actual),
            "source": "SimulationManager_and_PhysicsContext_runtime_readback"}


def obb_aabb_definitely_separated(center_a, half_a, rotation_a, center_b, half_b, rotation_b):
    """Conservative world-axis rejection only, with outward roundoff padding."""
    extent_a = np.abs(rotation_a) @ half_a
    extent_b = np.abs(rotation_b) @ half_b
    scale = max(1.0, float(np.max(np.abs(center_a))), float(np.max(np.abs(center_b))),
                float(np.max(extent_a)), float(np.max(extent_b)))
    padding = 64.0 * np.finfo(float).eps * scale
    return bool(np.any(np.abs(center_b - center_a) > extent_a + extent_b + padding))


def obb_penetration_depth(center_a, half_a, rotation_a, center_b, half_b, rotation_b,
                          *, use_aabb_broadphase=True):
    """The original SAT depth; skip it only for proved-separated world AABBs."""
    center_a = np.asarray(center_a, dtype=float)
    center_b = np.asarray(center_b, dtype=float)
    half_a = np.asarray(half_a, dtype=float)
    half_b = np.asarray(half_b, dtype=float)
    rotation_a = np.asarray(rotation_a, dtype=float)
    rotation_b = np.asarray(rotation_b, dtype=float)
    if use_aabb_broadphase and obb_aabb_definitely_separated(
            center_a, half_a, rotation_a, center_b, half_b, rotation_b):
        return 0.0
    axes = [*rotation_a.T, *rotation_b.T]
    axes.extend(np.cross(first, second) for first in rotation_a.T for second in rotation_b.T)
    center_delta = center_b - center_a
    minimum_overlap = float("inf")
    for raw_axis in axes:
        magnitude = float(np.linalg.norm(raw_axis))
        if magnitude <= 1e-10:
            continue
        axis = raw_axis / magnitude
        radius_a = float(np.sum(half_a * np.abs(rotation_a.T @ axis)))
        radius_b = float(np.sum(half_b * np.abs(rotation_b.T @ axis)))
        overlap = radius_a + radius_b - abs(float(center_delta @ axis))
        if overlap <= 0.0:
            return 0.0
        minimum_overlap = min(minimum_overlap, overlap)
    return 0.0 if not np.isfinite(minimum_overlap) else minimum_overlap


def replay_command_arrays(bundle, expected_joint_names):
    """Consume the exporter schema identically for initial and continued tasks."""
    if "timestamps_seconds" not in bundle or "positions_rad" not in bundle:
        raise ValueError("replay commands require timestamps_seconds and positions_rad")
    timestamps = np.asarray(bundle["timestamps_seconds"], dtype=float)
    positions = np.asarray(bundle["positions_rad"], dtype=float)
    if (timestamps.ndim != 1 or len(timestamps) < 2
            or positions.shape != (len(timestamps), len(expected_joint_names))
            or not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(positions))
            or abs(timestamps[0]) > 1e-12 or np.any(np.diff(timestamps) <= 0)):
        raise ValueError("replay bundle command dimensions or times are invalid")
    return timestamps, positions


def validate_continuation_request(request, *, world_session_id, actual_state_sha256):
    """Reject stale queue messages when they carry a world/state identity."""
    if not isinstance(request, Mapping):
        raise ValueError("continuation request must be a JSON object")
    expected = {"world_session_id": world_session_id, "actual_state_sha256": actual_state_sha256}
    for field, value in expected.items():
        if field in request and request[field] != value:
            raise ValueError(f"continuation request has stale {field}")
    return {"world_session_id": world_session_id, "actual_state_sha256": actual_state_sha256}


def finite_gravity_compensated_drive_target(reference_q, gravity_nm, stiffness_nm_rad,
                                             effort_limits_nm):
    """Bias a finite-force position drive by g/Kp without adding an uncapped force.

    The PhysX position drive remains the sole actuator and retains its official
    maxForce. This is equivalent to a gravity feedforward inside that actuator's
    saturation, not an independent torque source beyond the effort limit.
    """
    values = [np.asarray(value, dtype=float) for value in
              (reference_q, gravity_nm, stiffness_nm_rad, effort_limits_nm)]
    q, gravity, stiffness, limits = values
    if (q.ndim != 1 or any(value.shape != q.shape or not np.all(np.isfinite(value)) for value in values)
            or np.any(stiffness <= 0) or np.any(limits <= 0)):
        raise ValueError("gravity-compensated drive inputs must be finite matching vectors")
    compensated = np.clip(gravity, -limits, limits)
    return q + compensated / stiffness


def sample_joint_reference(timestamps, positions, time_s, *, held=False):
    """Same piecewise-linear position path and its segment derivative.

    Interior knots use the right segment. Outside the path and at its two
    endpoints the velocity is zero. A clock hold commands zero velocity without
    changing position. No trajectory resampling or velocity-limit clipping is
    performed: a bound violation must be rejected by the caller.
    """
    times, values = np.asarray(timestamps), np.asarray(positions)
    if not np.isfinite(time_s):
        raise ValueError("reference time must be finite")
    t = float(np.clip(time_s, times[0], times[-1]))
    position = np.asarray([np.interp(t, times, values[:, j]) for j in range(values.shape[1])])
    velocity = np.zeros(values.shape[1], dtype=float)
    if not held and times[0] < t < times[-1]:
        index = min(int(np.searchsorted(times, t, side="right")) - 1, len(times) - 2)
        velocity = (values[index + 1] - values[index]) / (times[index + 1] - times[index])
    return position, velocity


def resolve_actual_task_stage(
    stage_windows,
    trajectory_time_s,
    *,
    grasp_commanded,
    grasp_event_time_s,
    contact_wait_started_s,
    attached=False,
    release_commanded=False,
    release_event_time_s=None,
    support_wait_started_s=None,
):
    """Contact and place follow the actual attachment/release lifecycle.

    CPU stage windows end at arrival, whereas the exported controller schedule
    adds a settling dwell before the grasp command. That dwell is still contact,
    not unattached extraction. Only the runtime's existing bounded contact wait
    can extend it beyond the scheduled grasp event; arbitrary late unattached
    motion never receives a contact-stage exception.  The equivalent release
    dwell remains place while the real constraint is attached; withdrawal starts
    only after the release command removes it.  A bounded support wait may extend
    that state, but arbitrary late attached motion receives no permission.
    """
    def stage_name(window):
        return str(window.get("stage", window.get("name", "motion")))
    planned = next((stage_name(window) for window in stage_windows
                    if float(window.get("start_time_s", 0.)) <= trajectory_time_s
                    <= float(window.get("end_time_s", 0.))), "motion")
    contact = next((window for window in stage_windows if stage_name(window) == "contact"), None)
    if (not grasp_commanded and contact is not None and grasp_event_time_s is not None
            and trajectory_time_s >= float(contact["start_time_s"])
            and (trajectory_time_s <= float(grasp_event_time_s) or contact_wait_started_s is not None)):
        return "contact"
    place = next((window for window in stage_windows if stage_name(window) == "place"), None)
    if (grasp_commanded and attached and not release_commanded and place is not None
            and release_event_time_s is not None
            and trajectory_time_s >= float(place["start_time_s"])
            and (trajectory_time_s <= float(release_event_time_s)
                 or support_wait_started_s is not None)):
        return "place"
    return planned


def payload_gravity_compensation(jacobian_world, tcp_position_world, payload_com_world,
                                 mass_kg, gravity_world):
    """External attached rigid-body gravity feedforward in world coordinates.

    J is [linear; angular] at the TCP. F=-m*g is the upward compensating force;
    tau=Jv.T@F + Jw.T@((COM-TCP) cross F). This equals d(-m*g.COM)/dq.
    Only gravity is modeled here, not acceleration, contact or measured effort.
    The caller adds this inside the existing finite-force drive saturation.
    """
    jacobian = np.asarray(jacobian_world, dtype=float)
    tcp, com, gravity = [np.asarray(value, dtype=float) for value in
                         (tcp_position_world, payload_com_world, gravity_world)]
    if (jacobian.ndim != 2 or jacobian.shape[0] != 6 or not np.all(np.isfinite(jacobian))
            or any(value.shape != (3,) or not np.all(np.isfinite(value)) for value in (tcp, com, gravity))
            or not np.isfinite(mass_kg) or mass_kg <= 0):
        raise ValueError("payload gravity requires a finite world Jacobian, COM, gravity and positive mass")
    force = -float(mass_kg) * gravity
    return jacobian[:3].T @ force + jacobian[3:].T @ np.cross(com - tcp, force)


class BoundedFreeTransitGate:
    """Pause command time, never physics, until unchanged actual clearance passes."""

    def __init__(self, boundary_time_s, maximum_wait_s):
        self.boundary_time_s = float(boundary_time_s)
        self.maximum_wait_s = float(maximum_wait_s)
        if (not np.isfinite(self.boundary_time_s) or self.boundary_time_s < 0
                or not np.isfinite(self.maximum_wait_s) or self.maximum_wait_s < 0):
            raise ValueError("free-transit boundary and wait must be finite and nonnegative")
        self.wait_started_s = None
        self.passed = False
        self.events = []

    def evaluate(self, trajectory_time_s, physical_time_s, actual_free):
        if self.passed or trajectory_time_s < self.boundary_time_s:
            return {"hold": False, "reason": None}
        if actual_free:
            self.passed = True
            self.events.append({"event": "actual_free_transit_gate_passed", "time_s": float(physical_time_s),
                                "wait_elapsed_s": 0.0 if self.wait_started_s is None else physical_time_s - self.wait_started_s})
            return {"hold": False, "reason": None}
        if self.wait_started_s is None:
            self.wait_started_s = float(physical_time_s)
            self.events.append({"event": "actual_free_transit_wait_started", "time_s": float(physical_time_s),
                                "trajectory_time_s": self.boundary_time_s, "maximum_wait_s": self.maximum_wait_s})
        elapsed = physical_time_s - self.wait_started_s
        reason = "ACTUAL_PAYLOAD_NOT_CLEAR_FOR_FREE_TRANSIT" if elapsed >= self.maximum_wait_s else None
        if reason:
            self.events.append({"event": "actual_free_transit_wait_timeout", "time_s": float(physical_time_s),
                                "wait_elapsed_s": float(elapsed), "reason": reason})
        return {"hold": True, "reason": reason}

    def advance(self, trajectory_time_s, dt_s, duration_s):
        following = min(float(duration_s), trajectory_time_s + dt_s)
        if not self.passed and trajectory_time_s < self.boundary_time_s <= following:
            following = self.boundary_time_s
        return following


class BoundedTargetCupReleaseClearance:
    """Separate constraint independence from restoration of target-cup margin.

    The caller keeps moving the verified withdrawal while this latch is open.
    Only actual LOST for every known compliant-cup/selected-target proximity
    closes it. No robot, rigid insert or neighbor obtains this permission.
    """

    def __init__(self, maximum_wait_s):
        self.maximum_wait_s = float(maximum_wait_s)
        if not np.isfinite(self.maximum_wait_s) or self.maximum_wait_s < 0:
            raise ValueError("release clearance wait must be finite and nonnegative")
        self.started_s = None
        self.pending = False
        self.events = []

    def begin(self, time_s):
        if self.started_s is not None:
            raise ValueError("release clearance latch cannot be restarted")
        self.started_s = float(time_s)
        self.pending = True
        self.events.append({"event": "target_cup_release_clearance_started", "time_s": float(time_s),
                            "maximum_wait_s": self.maximum_wait_s})

    def observe(self, time_s, active_headers, *, target_path, compliant_paths):
        if not self.pending:
            return None
        keys = [key for key in active_headers if target_path in key[:2]
                and any(path in compliant_paths for path in key[2:])]
        elapsed = float(time_s) - self.started_s
        if not keys:
            self.pending = False
            self.events.append({"event": "target_cup_release_clearance_restored", "time_s": float(time_s),
                                "elapsed_s": elapsed, "active_target_compliant_pairs": 0})
            return None
        if elapsed >= self.maximum_wait_s:
            reason = "TARGET_CUP_RELEASE_CLEARANCE_TIMEOUT"
            self.events.append({"event": "target_cup_release_clearance_timeout", "time_s": float(time_s),
                                "elapsed_s": elapsed, "active_target_compliant_pairs": len(keys), "reason": reason})
            return reason
        return None


def validate_same_world_continuation(previous_metadata, next_bundle, actual_state, *,
                                     position_tolerance_m=0.001,
                                     joint_tolerance_rad=0.001):
    """Validate a new offline plan against the live, retained physical world.

    This does not restore the scene to the plan. A stale plan is rejected while
    preserving every existing body and the current articulation state.
    """
    if actual_state.get("attached") or actual_state.get("attachment_target"):
        raise ValueError("continuation requires confirmed release of the previous carton")
    if next_bundle.get("format") != "isaacsim_fanuc_replay_v1":
        raise ValueError("unsupported continuation bundle")
    metadata = next_bundle["metadata"]
    for field in ("robot_model", "joint_names", "base_position_m", "base_rpy_rad",
                  "urdf_path", "robot_srdf_path", "robot_srdf_sha256", "physics", "collision_policy",
                  "official_model_manifest_sha256", "robot_link_dynamics", "robot_tool_mass_accounting",
                  "joint_effort_limits_nm", "joint_velocity_limits_rad_s",
                  "joint_position_lower_limits_rad", "joint_position_upper_limits_rad",
                  "joint_drive_stiffness_nm_rad", "joint_drive_damping_nm_s_rad",
                  "joint_gravity_feedforward_enabled", "joint_velocity_feedforward_enabled",
                  "attached_payload_gravity_feedforward_enabled", "tool_length_m",
                  "isaac_grasp_body_path_suffix", "flange_offset_from_grasp_body_m",
                  "conveyor", "actual_state_gates", "rendering", "camera"):
        if metadata.get(field) != previous_metadata.get(field):
            raise ValueError(f"continuation changed fixed physical input: {field}")
    for field in ("physical_cup_compression_m", "cup_radius_m", "task_tcp_from_flange_m",
                  "compressed_contact_plane_from_flange_m", "mass_properties_path",
                  "physical_cup_count", "mask_bit_order_cup_ids", "cup_centers_tool_yz_m",
                  "gripper_mass_kg", "center_of_mass_from_flange_m", "inertia_at_com_kg_m2",
                  "step_path", "visual_mesh_path", "collision_mesh_path",
                  "flange_origin_step_mm", "step_from_tool_rotation_matrix", "frame_contract",
                  "suction_mode", "holding_capacity_assumption",
                  "qualified_rigid_collision_boxes_tool_frame",
                  "max_grip_distance_m", "max_normal_misalignment_rad", "maximum_contact_penetration_m"):
        if metadata.get("gripper", {}).get(field) != previous_metadata.get("gripper", {}).get(field):
            raise ValueError(f"continuation changed tool physical input: {field}")
    if (not metadata.get("simulation_execution_ready") or metadata.get("execution_blockers")):
        raise ValueError("continuation bundle is not simulation ready")
    current = {item["name"]: item for item in actual_state["cartons"]}
    dynamic = {item["name"]: item for item in metadata["scene_primitives"] if item.get("dynamic")}
    if set(current) != set(dynamic):
        raise ValueError("continuation must retain every live carton identity")
    if metadata.get("target") in actual_state.get("completed_carton_ids", []):
        raise ValueError("continuation selected an already completed carton")
    for event_field in ("completed_carton_ids", "handed_off_ids"):
        if set(metadata.get(event_field, [])) != set(actual_state.get(event_field, [])):
            raise ValueError(f"continuation must preserve cumulative execution events: {event_field}")
    for name, record in current.items():
        if np.linalg.norm(np.asarray(record["position_m"]) - dynamic[name]["center_m"]) > position_tolerance_m:
            raise ValueError(f"continuation plan has stale actual carton position: {name}")
        quaternion = np.asarray(record["orientation_wxyz"], dtype=float)
        quaternion /= np.linalg.norm(quaternion)
        w, x, y, z = quaternion
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        if np.linalg.norm(rotation - dynamic[name]["rotation_matrix"]) > 0.002:
            raise ValueError(f"continuation plan has stale actual carton rotation: {name}")
    previous_static = {item["name"]: item for item in previous_metadata["scene_primitives"] if not item.get("dynamic")}
    next_static = {item["name"]: item for item in metadata["scene_primitives"] if not item.get("dynamic")}
    if previous_static != next_static:
        raise ValueError("continuation changed fixed scene structure")
    actual_order = list(actual_state.get("joint_names", metadata["joint_names"]))
    actual_q = np.asarray(actual_state["q_rad"])[[actual_order.index(name) for name in metadata["joint_names"]]]
    _, commands = replay_command_arrays(next_bundle, metadata["joint_names"])
    first = commands[0]
    if first.shape != actual_q.shape or np.max(np.abs(first - actual_q)) > joint_tolerance_rad:
        raise ValueError("continuation path does not start at actual joint configuration")
    return {"accepted": True, "retained_carton_count": len(current),
            "world_session_id": actual_state.get("world_session_id"),
            "scene_restored_or_teleported": False}


class ActualStackContactMonitor:
    """Bound actual contact/sliding without suppressing PhysX carton shapes.

    The geometric free-space transition latches only when every current
    neighbor clears the policy distance. Monitoring uses actual world states;
    neither a stage label nor a fixed extraction distance proves separation.
    """

    def __init__(self, target, neighbors, policy):
        self.initial_target = target
        self.initial_neighbors = {item.name: item for item in neighbors if item.name != target.name}
        self.policy = policy
        self.free_space_reached = not self.initial_neighbors
        self.progress_reference = np.asarray(target.center).copy()
        self.progress_clock = None
        self.instability_clock = None
        self.peak_penetration_m = 0.0
        self.peak_neighbor_displacement_m = 0.0
        self.peak_neighbor_tilt_rad = 0.0
        self.first_free_space_time_s = None
        self.observations = 0

    def observe(self, time_s, target, neighbors, *, commanded_motion):
        current = {item.name: item for item in neighbors}
        missing = set(self.initial_neighbors) - set(current)
        if missing:
            raise ValueError(f"actual stack state lost carton identities: {sorted(missing)}")
        clearance = min((target.signed_distance_obb(current[name])
                         for name in self.initial_neighbors), default=float("inf"))
        penetration = max(0.0, -clearance)
        drift = max((float(np.linalg.norm(current[name].center - original.center))
                     for name, original in self.initial_neighbors.items()), default=0.0)
        tilt = max((float(np.arccos(np.clip(current[name].rotation[:, 2] @ original.rotation[:, 2], -1, 1)))
                    for name, original in self.initial_neighbors.items()), default=0.0)
        self.peak_penetration_m = max(self.peak_penetration_m, penetration)
        self.peak_neighbor_displacement_m = max(self.peak_neighbor_displacement_m, drift)
        self.peak_neighbor_tilt_rad = max(self.peak_neighbor_tilt_rad, tilt)
        self.observations += 1
        previously_free = self.free_space_reached
        if not self.free_space_reached and clearance >= self.policy.free_space_clearance_m:
            self.free_space_reached = True
            self.first_free_space_time_s = float(time_s)
        if penetration > self.policy.maximum_actual_penetration_m:
            reason = "SEVERE_ACTUAL_STACK_PENETRATION"
        elif previously_free and clearance < self.policy.free_space_clearance_m - 1e-6:
            reason = "ACTUAL_FREE_TRANSIT_STACK_CLEARANCE_LOST"
        elif drift > self.policy.maximum_neighbor_displacement_m:
            reason = "EXCESSIVE_ACTUAL_NEIGHBOR_DISPLACEMENT"
        else:
            reason = None
        if tilt > self.policy.maximum_neighbor_tilt_rad:
            if self.instability_clock is None:
                self.instability_clock = float(time_s)
            if time_s - self.instability_clock >= getattr(self.policy, "neighbor_instability_duration_s", 0.5):
                reason = reason or "PERSISTENT_ACTUAL_STACK_INSTABILITY"
        else:
            self.instability_clock = None
        progress = float(np.linalg.norm(target.center - self.progress_reference))
        if not commanded_motion or self.progress_clock is None or progress >= self.policy.minimum_progress_m:
            self.progress_clock = float(time_s)
            self.progress_reference = np.asarray(target.center).copy()
        elif not self.free_space_reached and time_s - self.progress_clock > self.policy.progress_timeout_s:
            reason = reason or "ACTUAL_STACK_EXTRACTION_STALLED"
        return {"accepted": reason is None, "reason": reason,
                "minimum_stack_clearance_m": None if not np.isfinite(clearance) else float(clearance),
                "actual_penetration_m": penetration, "neighbor_displacement_m": drift,
                "neighbor_tilt_rad": tilt, "free_space_reached": self.free_space_reached,
                "first_free_space_time_s": self.first_free_space_time_s}

    def summary(self):
        return {"observations": self.observations, "box_box_physics_enabled": True,
                "free_space_reached": self.free_space_reached,
                "first_free_space_time_s": self.first_free_space_time_s,
                "peak_actual_penetration_m": self.peak_penetration_m,
                "peak_neighbor_displacement_m": self.peak_neighbor_displacement_m,
                "peak_neighbor_tilt_rad": self.peak_neighbor_tilt_rad,
                "contact_force_channel": "NOT_USED_BY_GEOMETRIC_DISTURBANCE_MONITOR"}


@dataclass(frozen=True)
class AttachmentContactAudit:
    """Result of checking every active cup ray against the target carton."""

    accepted: bool
    reason: str | None
    signed_gaps_m: tuple[float | None, ...]
    normal_alignments: tuple[float | None, ...]
    hit_count: int
    within_gap_count: int
    normal_aligned_count: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "reason": self.reason,
            "signed_gaps_m": list(self.signed_gaps_m),
            "normal_alignments": list(self.normal_alignments),
            "hit_count": self.hit_count,
            "within_gap_count": self.within_gap_count,
            "normal_aligned_count": self.normal_aligned_count,
        }


@dataclass(frozen=True)
class PayloadSupportAudit:
    """Actual-state gate for releasing a payload onto one declared support.

    The audit is intentionally geometric and kinematic.  It does not infer a
    support from the planned release time: the payload bottom plane must be at
    the real support top plane, their horizontal footprints must overlap, and
    the payload must have settled below the configured velocity thresholds.
    """

    accepted: bool
    reason: str | None
    support_name: str
    signed_normal_gap_m: float
    footprint_overlap_ratio: float
    support_normal_alignment: float
    linear_speed_m_s: float
    angular_speed_rad_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "reason": self.reason,
            "support_name": self.support_name,
            "signed_normal_gap_m": self.signed_normal_gap_m,
            "footprint_overlap_ratio": self.footprint_overlap_ratio,
            "support_normal_alignment": self.support_normal_alignment,
            "linear_speed_m_s": self.linear_speed_m_s,
            "angular_speed_rad_s": self.angular_speed_rad_s,
        }


def _rotation(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if (
        result.shape != (3, 3)
        or not np.all(np.isfinite(result))
        or not np.allclose(result.T @ result, np.eye(3), atol=1e-9, rtol=0.0)
        or not np.isclose(np.linalg.det(result), 1.0, atol=1e-9, rtol=0.0)
    ):
        raise ValueError(f"{name} must be a finite proper rotation matrix")
    return result


def audit_surface_attachment_contact(
    *,
    grasp_body_position_m: Sequence[float],
    grasp_body_rotation: Sequence[Sequence[float]],
    contact_plane_from_grasp_body_m: float,
    active_cup_offsets_yz_m: Sequence[Sequence[float]],
    target_center_m: Sequence[float],
    target_rotation: Sequence[Sequence[float]],
    target_size_m: Sequence[float],
    max_attachment_gap_m: float,
    max_normal_misalignment_rad: float,
    maximum_penetration_m: float = 1e-5,
) -> AttachmentContactAudit:
    """Audit physical cup-plane contact without moving either rigid body.

    Cup rays start on the *nominal compressed* physical contact plane and point
    along grasp-body +X.  A negative signed gap means that the nominal contact
    point has already crossed the carton face; only the small explicit
    numerical penetration tolerance is accepted.
    """

    body_position = np.asarray(grasp_body_position_m, dtype=float)
    carton_center = np.asarray(target_center_m, dtype=float)
    carton_size = np.asarray(target_size_m, dtype=float)
    cup_offsets = np.asarray(active_cup_offsets_yz_m, dtype=float)
    body_rotation = _rotation(grasp_body_rotation, "grasp_body_rotation")
    carton_rotation = _rotation(target_rotation, "target_rotation")
    scalars = (
        float(contact_plane_from_grasp_body_m),
        float(max_attachment_gap_m),
        float(max_normal_misalignment_rad),
        float(maximum_penetration_m),
    )
    if body_position.shape != (3,) or carton_center.shape != (3,):
        raise ValueError("body and target centers must contain three values")
    if carton_size.shape != (3,) or np.any(carton_size <= 0.0):
        raise ValueError("target_size_m must contain three positive values")
    if cup_offsets.ndim != 2 or cup_offsets.shape[1] != 2 or len(cup_offsets) == 0:
        raise ValueError("active_cup_offsets_yz_m must have shape (N, 2)")
    if not all(np.all(np.isfinite(value)) for value in (body_position, carton_center, carton_size, cup_offsets)):
        raise ValueError("attachment geometry must be finite")
    if not all(np.isfinite(value) for value in scalars):
        raise ValueError("attachment tolerances and contact-plane offset must be finite")
    if max_attachment_gap_m < 0.0 or maximum_penetration_m < 0.0:
        raise ValueError("attachment gap and penetration tolerance must be non-negative")
    if not 0.0 <= max_normal_misalignment_rad < 0.5 * np.pi:
        raise ValueError("normal misalignment must be in [0, pi/2)")

    half_extents = 0.5 * carton_size
    ray_world = body_rotation[:, 0]
    ray_local = carton_rotation.T @ ray_world
    minimum_alignment = cos(max_normal_misalignment_rad)
    signed_gaps: list[float | None] = []
    alignments: list[float | None] = []

    for offset_y, offset_z in cup_offsets:
        point_body = np.asarray(
            [contact_plane_from_grasp_body_m, float(offset_y), float(offset_z)],
            dtype=float,
        )
        point_world = body_position + body_rotation @ point_body
        point_local = carton_rotation.T @ (point_world - carton_center)
        ray_min = -float("inf")
        ray_max = float("inf")
        entry_axis: int | None = None
        entry_outward_sign = 0.0
        intersects = True
        for axis in range(3):
            direction = float(ray_local[axis])
            if abs(direction) <= 1e-12:
                if abs(float(point_local[axis])) > half_extents[axis] + 1e-12:
                    intersects = False
                    break
                continue
            lower = (-half_extents[axis] - point_local[axis]) / direction
            upper = (half_extents[axis] - point_local[axis]) / direction
            if lower <= upper:
                near, far, outward_sign = lower, upper, -1.0
            else:
                near, far, outward_sign = upper, lower, 1.0
            if near > ray_min:
                ray_min = near
                entry_axis = axis
                entry_outward_sign = outward_sign
            ray_max = min(ray_max, far)
            if ray_min > ray_max + 1e-12:
                intersects = False
                break
        if not intersects or entry_axis is None or ray_max < max(ray_min, 0.0) - 1e-12:
            signed_gaps.append(None)
            alignments.append(None)
            continue
        outward_local = np.zeros(3, dtype=float)
        outward_local[entry_axis] = entry_outward_sign
        outward_world = carton_rotation @ outward_local
        alignment = float(np.clip((-ray_world) @ outward_world, -1.0, 1.0))
        signed_gaps.append(float(ray_min))
        alignments.append(alignment)

    hit_count = sum(value is not None for value in signed_gaps)
    comparison_epsilon_m = 1e-12
    within_gap_count = sum(
        value is not None
        and -maximum_penetration_m - comparison_epsilon_m
        <= value
        <= max_attachment_gap_m + comparison_epsilon_m
        for value in signed_gaps
    )
    normal_aligned_count = sum(
        value is not None and value >= minimum_alignment
        for value in alignments
    )
    cup_count = len(cup_offsets)
    if hit_count != cup_count:
        reason = "ACTIVE_CUP_RAY_MISSES_TARGET"
    elif within_gap_count != cup_count:
        if any(
            value is not None
            and value < -maximum_penetration_m - comparison_epsilon_m
            for value in signed_gaps
        ):
            reason = "PHYSICAL_CONTACT_PLANE_PENETRATES_TARGET"
        else:
            reason = "PHYSICAL_CONTACT_GAP_EXCEEDS_LIMIT"
    elif normal_aligned_count != cup_count:
        reason = "PHYSICAL_CONTACT_NORMAL_MISALIGNED"
    else:
        reason = None
    return AttachmentContactAudit(
        accepted=reason is None,
        reason=reason,
        signed_gaps_m=tuple(signed_gaps),
        normal_alignments=tuple(alignments),
        hit_count=hit_count,
        within_gap_count=within_gap_count,
        normal_aligned_count=normal_aligned_count,
    )


def audit_payload_support_contact(
    *,
    payload_center_m: Sequence[float],
    payload_rotation: Sequence[Sequence[float]],
    payload_size_m: Sequence[float],
    payload_linear_velocity_m_s: Sequence[float],
    payload_angular_velocity_rad_s: Sequence[float],
    support: Mapping[str, Any],
    supports: Sequence[Mapping[str, Any]] | None = None,
    max_support_gap_m: float = 0.003,
    maximum_penetration_m: float = 0.001,
    minimum_footprint_overlap_ratio: float = 0.90,
    max_support_tilt_rad: float = np.deg2rad(5.0),
    max_linear_speed_m_s: float = 0.03,
    max_angular_speed_rad_s: float = 0.08,
) -> PayloadSupportAudit:
    """Require real, non-penetrating receiver support before vacuum release.

    ``support.rotation_matrix[:, 2]`` is the support normal.  This keeps the
    calculation valid for a known tilted receiver while still rejecting a
    side-wall as a support because its local-X/Y footprint and top plane would
    not match the payload bottom plane.
    """

    payload_center = np.asarray(payload_center_m, dtype=float)
    payload_size = np.asarray(payload_size_m, dtype=float)
    payload_linear_velocity = np.asarray(payload_linear_velocity_m_s, dtype=float)
    payload_angular_velocity = np.asarray(payload_angular_velocity_rad_s, dtype=float)
    payload_rotation_matrix = _rotation(payload_rotation, "payload_rotation")
    support_center = np.asarray(support.get("center_m", []), dtype=float)
    support_size = np.asarray(support.get("size_m", []), dtype=float)
    support_rotation = _rotation(
        support.get("rotation_matrix", []), "support rotation"
    )
    support_name = str(support.get("name", "")).strip()
    scalars = np.asarray(
        [
            max_support_gap_m,
            maximum_penetration_m,
            minimum_footprint_overlap_ratio,
            max_support_tilt_rad,
            max_linear_speed_m_s,
            max_angular_speed_rad_s,
        ],
        dtype=float,
    )
    if (
        payload_center.shape != (3,)
        or payload_size.shape != (3,)
        or payload_linear_velocity.shape != (3,)
        or payload_angular_velocity.shape != (3,)
        or support_center.shape != (3,)
        or support_size.shape != (3,)
        or not support_name
        or not all(
            np.all(np.isfinite(value))
            for value in (
                payload_center,
                payload_size,
                payload_linear_velocity,
                payload_angular_velocity,
                support_center,
                support_size,
            )
        )
        or np.any(payload_size <= 0.0)
        or np.any(support_size <= 0.0)
        or not np.all(np.isfinite(scalars))
        or max_support_gap_m < 0.0
        or maximum_penetration_m < 0.0
        or not 0.0 < minimum_footprint_overlap_ratio <= 1.0
        or not 0.0 <= max_support_tilt_rad < 0.5 * np.pi
        or max_linear_speed_m_s < 0.0
        or max_angular_speed_rad_s < 0.0
    ):
        raise ValueError("payload support audit inputs must be finite and physical")

    support_from_payload = support_rotation.T @ payload_rotation_matrix
    payload_half = 0.5 * payload_size
    support_half = 0.5 * support_size
    payload_center_support = support_rotation.T @ (payload_center - support_center)
    projected_payload_half = np.abs(support_from_payload) @ payload_half
    signed_gap = float(
        payload_center_support[2]
        - projected_payload_half[2]
        - support_half[2]
    )

    overlaps = []
    payload_widths = []
    for axis in (0, 1):
        payload_min = float(payload_center_support[axis] - projected_payload_half[axis])
        payload_max = float(payload_center_support[axis] + projected_payload_half[axis])
        support_min = -float(support_half[axis])
        support_max = float(support_half[axis])
        overlaps.append(max(0.0, min(payload_max, support_max) - max(payload_min, support_min)))
        payload_widths.append(2.0 * float(projected_payload_half[axis]))
    payload_area = payload_widths[0] * payload_widths[1]
    overlap_ratio = float(overlaps[0] * overlaps[1] / payload_area)
    union_audit = None
    if supports is not None:
        from .conveyor_placement import support_union_audit
        from .geometry import OBB
        actual_box = OBB(payload_center, payload_half, payload_rotation_matrix, "actual_payload", "carton")
        support_boxes = [OBB(
            np.asarray(item["center_m"], dtype=float),
            0.5 * np.asarray(item["size_m"], dtype=float),
            _rotation(item["rotation_matrix"], "support rotation"),
            str(item["name"]), "conveyor",
        ) for item in supports]
        union_audit = support_union_audit(
            actual_box, support_boxes,
            contact_tolerance_m=max(max_support_gap_m, maximum_penetration_m),
        )
        overlap_ratio = (1.0 - union_audit["unsupported_area_m2"] / union_audit["footprint_area_m2"]
                         if union_audit["footprint_area_m2"] > 0 else 0.0)
        support_name = "+".join(union_audit["receiver_names"]) or support_name
    support_normal_alignment = float(
        np.clip(payload_rotation_matrix[:, 2] @ support_rotation[:, 2], -1.0, 1.0)
    )
    linear_speed = float(np.linalg.norm(payload_linear_velocity))
    angular_speed = float(np.linalg.norm(payload_angular_velocity))
    epsilon = 1e-12
    if support_normal_alignment < cos(max_support_tilt_rad) - epsilon:
        reason = "PAYLOAD_SUPPORT_NORMAL_MISALIGNED"
    elif signed_gap > max_support_gap_m + epsilon:
        reason = "PAYLOAD_NOT_IN_SUPPORT_CONTACT"
    elif signed_gap < -maximum_penetration_m - epsilon:
        reason = "PAYLOAD_PENETRATES_SUPPORT"
    elif union_audit is not None and not union_audit["supported"]:
        reason = "PAYLOAD_SUPPORT_FOOTPRINT_INSUFFICIENT" if union_audit["reason"] == "UNSUPPORTED_FOOTPRINT" else "PAYLOAD_NOT_IN_SUPPORT_CONTACT"
    elif overlap_ratio + epsilon < minimum_footprint_overlap_ratio:
        reason = "PAYLOAD_SUPPORT_FOOTPRINT_INSUFFICIENT"
    elif linear_speed > max_linear_speed_m_s + epsilon:
        reason = "PAYLOAD_LINEAR_SPEED_TOO_HIGH_FOR_RELEASE"
    elif angular_speed > max_angular_speed_rad_s + epsilon:
        reason = "PAYLOAD_ANGULAR_SPEED_TOO_HIGH_FOR_RELEASE"
    else:
        reason = None
    return PayloadSupportAudit(
        accepted=reason is None,
        reason=reason,
        support_name=support_name,
        signed_normal_gap_m=signed_gap,
        footprint_overlap_ratio=overlap_ratio,
        support_normal_alignment=support_normal_alignment,
        linear_speed_m_s=linear_speed,
        angular_speed_rad_s=angular_speed,
    )


def _horizontal_footprint_contains(
    payload_center_m: np.ndarray,
    primitive: Mapping[str, Any],
    tolerance_m: float,
) -> bool:
    center = np.asarray(primitive.get("center_m", []), dtype=float)
    size = np.asarray(primitive.get("size_m", []), dtype=float)
    rotation = _rotation(primitive.get("rotation_matrix", []), "conveyor rotation")
    if center.shape != (3,) or size.shape != (3,) or np.any(size <= 0.0):
        raise ValueError("conveyor primitives require finite positive OBB geometry")
    local = rotation.T @ (payload_center_m - center)
    # Only the OBB's horizontal footprint decides transfer ownership.  Vertical
    # support/contact remains PhysX's responsibility and is audited separately.
    world_axes = rotation[:, :2]
    if np.any(np.abs(world_axes[2, :]) > 1e-9):
        raise ValueError("conveyor support surfaces must be horizontal")
    return bool(
        abs(float(local[0])) <= 0.5 * float(size[0]) + tolerance_m
        and abs(float(local[1])) <= 0.5 * float(size[1]) + tolerance_m
    )


def select_active_conveyor_surfaces(
    *,
    payload_center_m: Sequence[float] | None,
    conveyor_primitives: Mapping[str, Mapping[str, Any]],
    started: bool,
    exclusive: bool,
    current_surface: str | None = None,
    preferred_initial_surface: str | None = None,
    footprint_tolerance_m: float = 1e-6,
) -> tuple[str, ...]:
    """Select driven belt surfaces from the payload center deterministically.

    In exclusive mode an overlap retains the already active surface.  Without
    that deterministic ownership, all drives are disabled.  This prevents two
    orthogonal surface velocities from pulling the same carton at a transfer.
    """

    if not started:
        return ()
    if footprint_tolerance_m < 0.0 or not np.isfinite(footprint_tolerance_m):
        raise ValueError("footprint_tolerance_m must be finite and non-negative")
    names = tuple(sorted(str(name) for name in conveyor_primitives))
    if not exclusive:
        return names
    if payload_center_m is None:
        return ()
    center = np.asarray(payload_center_m, dtype=float)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("payload_center_m must contain three finite values")
    matching = tuple(
        name
        for name in names
        if _horizontal_footprint_contains(center, conveyor_primitives[name], footprint_tolerance_m)
    )
    if len(matching) == 1:
        return matching
    if len(matching) > 1:
        if current_surface in matching:
            return (str(current_surface),)
        if preferred_initial_surface in matching:
            return (str(preferred_initial_surface),)
    return ()


__all__ = [
    "AttachmentContactAudit",
    "PayloadSupportAudit",
    "audit_payload_support_contact",
    "audit_surface_attachment_contact",
    "select_active_conveyor_surfaces",
]
