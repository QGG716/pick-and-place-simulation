"""Bounded release predictions shared by CPU planning and actual-state gates.

Predictions are filters, never evidence of physical landing. No function here
changes a rigid body or assigns the conveyor velocity to a flying carton.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Sequence

import numpy as np

from .conveyor_placement import support_union_audit
from .geometry import OBB, rotation_matrix_from_rotation_vector

SUPPORTED_RELEASE = "SUPPORTED_RELEASE"
SHORT_DROP_RELEASE = "SHORT_DROP_RELEASE"
IDEAL_RECEPTION_RELEASE = "IDEAL_RECEPTION_RELEASE"
MOTION_SEMANTICS = "m710_adaptive_contact_release_v2"


@dataclass(frozen=True)
class ReleasePolicy:
    maximum_drop_m: float = 0.05
    ideal_release_min_height_m: float = 0.020
    ideal_release_max_height_m: float = 0.050
    ideal_release_height_reserve_m: float = 0.005
    maximum_flight_s: float = 0.25
    maximum_angular_speed_rad_s: float = 0.08
    maximum_landing_speed_m_s: float = 1.1
    position_uncertainty_m: float = 0.002
    velocity_uncertainty_m_s: float = 0.01
    prediction_step_s: float = 0.005
    gravity_m_s2: float = 9.81

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"invalid release policy: {name}")
        if min(self.maximum_flight_s, self.prediction_step_s, self.gravity_m_s2) <= 0:
            raise ValueError("flight horizon, step and gravity must be positive")

    def to_mapping(self):
        return asdict(self)

    def ideal_heights(self):
        """Actual bounds are distinct from interior nominal engineering targets."""
        lo, hi, reserve = (self.ideal_release_min_height_m,
            self.ideal_release_max_height_m, self.ideal_release_height_reserve_m)
        if (lo != .020 or hi != .050 or self.maximum_drop_m != hi
                or reserve <= 0 or 2 * reserve >= hi - lo):
            raise ValueError("POC requires consistent 20-50 mm bounds and an interior height reserve")
        return tuple(dict.fromkeys((lo + reserve, (lo + hi) / 2, hi - reserve)))


def rigid_com_velocity(tcp_velocity, angular_velocity, tcp_position, com_position):
    vectors = [np.asarray(v, dtype=float) for v in
               (tcp_velocity, angular_velocity, tcp_position, com_position)]
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in vectors):
        raise ValueError("rigid velocity inputs must be finite three-vectors")
    linear, omega, tcp, com = vectors
    return linear + np.cross(omega, com - tcp)


def receiver_footprint_reserve(box: OBB, supports, reserve_m, *, contact_tolerance_m=.002,
                               edge_tolerance_m=1e-6, ideal_region=False):
    """Enlarge payload XY coverage, preserving adjoining support seams.

    This conservative coverage proxy does not alter the actual body's shape.
    Shrinking each conveyor separately would invent gaps inside their union.
    """
    if not np.isfinite(reserve_m) or reserve_m < 0:
        raise ValueError("footprint reserve must be finite and nonnegative")
    padding = reserve_m * (np.abs(box.rotation[0]) + np.abs(box.rotation[1]))
    proxy = OBB(box.center, box.half_extents + padding, box.rotation, box.name, box.category)
    if ideal_region:
        return reception_footprint_audit(proxy, supports, edge_tolerance_m=edge_tolerance_m)
    return support_union_audit(proxy, supports, contact_tolerance_m=contact_tolerance_m,
                               edge_tolerance_m=edge_tolerance_m)


def flight_box(box: OBB, linear_velocity, angular_velocity, time_s: float,
               gravity_m_s2: float = 9.81) -> OBB:
    velocity, omega = np.asarray(linear_velocity, float), np.asarray(angular_velocity, float)
    if (velocity.shape != (3,) or omega.shape != (3,) or time_s < 0
            or not np.all(np.isfinite([*velocity, *omega, time_s, gravity_m_s2]))):
        raise ValueError("invalid flight state")
    center = box.center + velocity * time_s
    center[2] -= 0.5 * gravity_m_s2 * time_s ** 2
    rotation = rotation_matrix_from_rotation_vector(omega * time_s) @ box.rotation
    return OBB(center, box.half_extents, rotation, box.name, box.category)


def predict_release(box: OBB, supports: Sequence[OBB], *, mode: str,
                    linear_velocity=(0., 0., 0.), angular_velocity=(0., 0., 0.),
                    obstacles: Sequence[OBB] = (), policy: ReleasePolicy | None = None,
                    contact_tolerance_m=0.002, edge_tolerance_m=1e-6):
    """Propagate orientation and the full footprint to first receiver contact.

    Significant spin is rejected by this bounded model. Within its angular
    budget the orientation is still propagated, rather than treating the old
    local Z face as the bottom. Flight obstacles use a conservative expansion
    for the distance traversed between samples.
    """
    policy = policy or ReleasePolicy()
    if mode not in {SUPPORTED_RELEASE, SHORT_DROP_RELEASE, IDEAL_RECEPTION_RELEASE}:
        raise ValueError("unknown release mode")
    velocity, omega = np.asarray(linear_velocity, float), np.asarray(angular_velocity, float)
    flight_box(box, velocity, omega, 0., policy.gravity_m_s2)  # validate inputs
    result = {"mode": mode, "accepted": False, "reason": None,
              "release_pose_world": box.world_from_local.tolist(),
              "linear_velocity_m_s": velocity.tolist(), "angular_velocity_rad_s": omega.tolist(),
              "policy": policy.to_mapping(), "actual_landing_state": None,
              "predicted_landing_pose_world": None, "landing_support": None,
              "actual_support": support_union_audit(box, supports,
                  contact_tolerance_m=contact_tolerance_m, edge_tolerance_m=edge_tolerance_m)}
    if not supports:
        result["reason"] = "NO_RECEIVER"
        return result
    heights = [float(np.max(s.corners()[:, 2])) for s in supports]
    if max(heights) - min(heights) > 1e-8:
        result["reason"] = "NONCOPLANAR_RELEASE_RECEIVERS"
        return result
    height = float(np.min(box.corners()[:, 2]) - heights[0])
    result["height_m"] = height
    if mode == IDEAL_RECEPTION_RELEASE:
        result["actual_support"] = None  # Ideal-region geometry is not physical support evidence.
        region = ideal_reception_region(box, supports, policy=policy,
                                        edge_tolerance_m=edge_tolerance_m)
        result.update(accepted=region["accepted"], reason=region["reason"],
                      reception_region=region, landing_support=region["support"],
                      flight_time_s=0., predicted_landing_pose_world=region["reception_pose_world"],
                      prediction_source="BOUNDED_IDEAL_RECEPTION_REGION_NOT_PHYSICAL_LANDING")
        support_names = {s.name for s in supports}
        # Check the continuous vertical handoff envelope against all other bodies.
        end = np.asarray(region["reception_pose_world"])
        delta = end[:3, 3] - box.center
        envelope = OBB(box.center + delta / 2,
                       box.half_extents + np.abs(box.rotation.T @ delta) / 2,
                       box.rotation, box.name, box.category)
        for obstacle in obstacles:
            if obstacle.name not in support_names | {box.name} and envelope.intersects_obb(obstacle):
                result.update(accepted=False, reason="IDEAL_RECEPTION_ENVELOPE_COLLISION",
                              pair=[box.name, obstacle.name])
                break
        return result
    if mode == SUPPORTED_RELEASE:
        result.update(accepted=result["actual_support"]["supported"],
                      reason=result["actual_support"]["reason"], flight_time_s=0.,
                      predicted_landing_pose_world=box.world_from_local.tolist(),
                      landing_support=result["actual_support"])
        return result
    if not 0 < height <= policy.maximum_drop_m + 1e-12:
        result["reason"] = "RELEASE_HEIGHT_OUT_OF_BOUNDS"
        return result
    if np.linalg.norm(omega) > policy.maximum_angular_speed_rad_s:
        result["reason"] = "RELEASE_ANGULAR_SPEED_OUT_OF_MODEL"
        return result
    count = max(1, int(np.ceil(policy.maximum_flight_s / policy.prediction_step_s)))
    previous_t = 0.
    landing_t = None
    support_names = {s.name for s in supports}
    for time_s in np.linspace(0., policy.maximum_flight_s, count + 1):
        current = flight_box(box, velocity, omega, time_s, policy.gravity_m_s2)
        if np.min(current.corners()[:, 2]) <= heights[0]:
            lo, hi = previous_t, time_s
            for _ in range(32):
                mid = (lo + hi) / 2
                if np.min(flight_box(box, velocity, omega, mid, policy.gravity_m_s2).corners()[:, 2]) > heights[0]:
                    lo = mid
                else:
                    hi = mid
            landing_t = hi
            break
        speed_bound = np.linalg.norm(velocity) + policy.gravity_m_s2 * time_s
        expansion = (policy.position_uncertainty_m + policy.velocity_uncertainty_m_s * time_s
                     + policy.prediction_step_s * (speed_bound + np.linalg.norm(omega) * np.linalg.norm(box.half_extents)))
        swept = OBB(current.center, current.half_extents + expansion, current.rotation, box.name, box.category)
        for obstacle in obstacles:
            if obstacle.name not in support_names | {box.name} and swept.intersects_obb(obstacle):
                result["reason"] = "PREDICTED_FLIGHT_COLLISION"
                result["pair"] = [box.name, obstacle.name]
                return result
        previous_t = time_s
    if landing_t is None:
        result["reason"] = "LANDING_OUTSIDE_FLIGHT_HORIZON"
        return result
    landed = flight_box(box, velocity, omega, landing_t, policy.gravity_m_s2)
    uncertainty = policy.position_uncertainty_m + policy.velocity_uncertainty_m_s * landing_t
    landing_support = support_union_audit(landed, supports,
        contact_tolerance_m=contact_tolerance_m, edge_tolerance_m=edge_tolerance_m)
    uncertain_support = receiver_footprint_reserve(landed, supports, uncertainty,
        contact_tolerance_m=contact_tolerance_m, edge_tolerance_m=edge_tolerance_m)
    landing_velocity = velocity - np.array([0., 0., policy.gravity_m_s2 * landing_t])
    result.update(flight_time_s=float(landing_t), predicted_landing_pose_world=landed.world_from_local.tolist(),
                  landing_support=landing_support, landing_velocity_m_s=landing_velocity.tolist(),
                  landing_speed_m_s=float(np.linalg.norm(landing_velocity)), uncertainty_m=uncertainty,
                  uncertain_landing_support=uncertain_support)
    if not landing_support["supported"] or not uncertain_support["supported"]:
        result["reason"] = "PREDICTED_LANDING_FOOTPRINT_UNSUPPORTED"
    elif np.linalg.norm(landing_velocity) > policy.maximum_landing_speed_m_s:
        result["reason"] = "PREDICTED_LANDING_SPEED_EXCEEDED"
    else:
        result.update(accepted=True, reason="QUALIFIED_PREDICTED_LANDING")
    return result


def reception_footprint_audit(box, supports, *, edge_tolerance_m=.002):
    """POC projection only; never proof of physical contact or coplanar bottom corners."""
    from .conveyor_placement import _horizontal_support_face, _face_corners, _area, _intersection, _subtract
    if (not supports or len({s.name for s in supports}) != len(supports)
            or not np.isfinite(edge_tolerance_m) or edge_tolerance_m < 0
            or not np.all(np.isfinite(box.world_from_local))):
        raise ValueError("invalid ideal receiver projection inputs")
    if any(not np.allclose(s.rotation[:, 2], [0, 0, 1], atol=1e-9, rtol=0) for s in supports):
        raise ValueError("ideal receiver top must be horizontal")
    heights = [float(np.max(s.corners()[:, 2])) for s in supports]
    if max(heights) - min(heights) > 1e-8:
        raise ValueError("ideal reception cannot join different receiver heights")
    bottom, axis, sign = _horizontal_support_face(box)  # Existing five-degree limit.
    footprint = bottom[:, :2]
    area = _area(footprint)
    remaining, names = [footprint], []
    for support in supports:
        polygon = _face_corners(support, 1., -edge_tolerance_m)[:, :2]
        if _area(_intersection(footprint, polygon)) > 1e-14:
            names.append(support.name)
        remaining = [piece for region in remaining for piece in _subtract(region, polygon)]
    uncovered = sum(_area(region) for region in remaining)
    accepted = area > 1e-14 and uncovered <= max(1e-12, area * 1e-10)
    return dict(schema="ideal_receiver_projection_v1", supported=bool(accepted),
        reason="IDEAL_FOOTPRINT_COVERED" if accepted else "IDEAL_POSE_OR_FOOTPRINT_REJECTED",
        receiver_names=names, support_z_m=heights[0], footprint_area_m2=area,
        unsupported_area_m2=uncovered, support_face_local_axis=axis, support_face_local_sign=sign,
        payload_half_extents_m=box.half_extents.tolist(), actual_box_pose=box.world_from_local.tolist(),
        support_obbs=[dict(name=s.name, category=s.category, pose_world=s.world_from_local.tolist(),
                           half_extents_m=s.half_extents.tolist()) for s in supports],
        tolerance_m=0., edge_tolerance_m=edge_tolerance_m, physical_support_observed=False)


def ideal_reception_region(box, supports, *, maximum_drop_m=None, policy=None, edge_tolerance_m=.002):
    """Check actual lowest-corner height separately from pose and XY coverage."""
    policy = policy or ReleasePolicy()
    policy.ideal_heights()
    if maximum_drop_m is not None and maximum_drop_m != policy.ideal_release_max_height_m:
        raise ValueError("maximum_drop_m conflicts with ideal release height policy")
    region = reception_footprint_audit(box, supports, edge_tolerance_m=edge_tolerance_m)
    heights = [region["support_z_m"]]
    gap = float(np.min(box.corners()[:, 2]) - heights[0])
    correction = max(0., gap)
    pose = box.world_from_local.copy()
    pose[2, 3] -= correction
    # This is explicitly a region audit, never an actual support observation.
    height_ok = policy.ideal_release_min_height_m - 1e-12 <= gap <= policy.ideal_release_max_height_m + 1e-12
    accepted = region["supported"] and height_ok
    return {"accepted": bool(accepted), "reason": "IDEAL_RECEPTION_REGION_ACCEPTED" if accepted else
            ("IDEAL_RELEASE_HEIGHT_OUT_OF_BOUNDS" if not height_ok else "IDEAL_RECEPTION_REGION_REJECTED"), "support": region,
            "actual_release_pose_world": box.world_from_local.tolist(),
            "reception_pose_world": pose.tolist(), "vertical_correction_m": -correction,
            "height_policy": policy.to_mapping(), "gap_m": gap,
            "actual_top_contact_observed": False, "physical_landing_qualified": False}


def departure_sweep(box: OBB, direction, *, distance_m: float, resolution_m: float):
    """Conservative swept receiver occupancy, including the zero-speed case.

    Samples are expanded by half their spacing, so checking all samples does
    not assume instantaneous belt entrainment or skip between obstacles.
    """
    direction = np.asarray(direction, float)
    if (direction.shape != (3,) or not np.all(np.isfinite(direction))
            or not np.isclose(np.linalg.norm(direction), 1.) or abs(direction[2]) > 1e-12
            or not np.isfinite(distance_m) or distance_m < 0 or resolution_m <= 0):
        raise ValueError("invalid departure sweep")
    count = max(1, int(np.ceil(distance_m / resolution_m)))
    padding = distance_m / count / 2
    local_padding = np.abs(box.rotation.T @ direction) * padding
    return tuple(OBB(box.center + direction * d, box.half_extents + local_padding,
                     box.rotation, box.name, box.category)
                 for d in np.linspace(0, distance_m, count + 1))


def receiver_outlet_clearance(box: OBB, receiver: OBB, direction) -> float:
    direction = np.asarray(direction, float)
    if direction.shape != (3,) or not np.all(np.isfinite(direction)) or not np.isclose(np.linalg.norm(direction), 1.):
        raise ValueError("receiver direction must be a unit vector")
    return float(np.max(receiver.corners() @ direction) - np.max(box.corners() @ direction))


def retained_receiver_envelope(box: OBB, receiver: OBB, direction, *, held=False):
    """Conservative occupancy until the recorded outlet hold, including rest.

    This does not move the carton or assume immediate belt entrainment. The
    executor monitors continued support and stops that surface before the end.
    """
    direction = np.asarray(direction, float)
    distance = 0. if held else max(0., receiver_outlet_clearance(box, receiver, direction) - .10)
    displacement = direction * distance
    # Reserve 10 mm along travel for belt braking and contact response.
    # Normal robot/tool pair margins are applied on top of this occupancy.
    return OBB(box.center + displacement / 2,
               box.half_extents + np.abs(box.rotation.T @ direction) * (distance / 2 + .01),
               box.rotation, box.name, box.category)


def receiver_transport_support(box: OBB, receiver: OBB, supports, direction, *,
                               contact_tolerance_m=.002, edge_tolerance_m=1e-6):
    """Check full support along the selected belt until the bounded outlet hold."""
    direction = np.asarray(direction, float)
    distance = max(0., receiver_outlet_clearance(box, receiver, direction) - .10)
    samples = max(1, int(np.ceil(distance / .02)))
    for progress in np.linspace(0., distance, samples + 1):
        moved = OBB(box.center + progress * direction, box.half_extents, box.rotation, box.name, box.category)
        audit = support_union_audit(moved, supports, contact_tolerance_m=contact_tolerance_m,
                                    edge_tolerance_m=edge_tolerance_m)
        if not audit["supported"]:
            return {"accepted": False, "reason": "RECEIVER_TRANSPORT_FOOTPRINT_LOST",
                    "progress_m": float(progress), "support": audit}
    return {"accepted": True, "distance_to_outlet_hold_m": distance,
            "samples": samples + 1, "requires_actual_support_monitoring": True}


def verify_release_prediction(place, target_name, *, obstacles=None, supports=None, policy=None,
                              require_current_environment=False):
    """Recompute release; omitted environment proves recorded geometry only."""
    if require_current_environment and (obstacles is None or supports is None or policy is None):
        raise ValueError("current obstacles, supports and release policy are required")
    prediction = place["release_prediction"]
    if prediction.get("actual_landing_state") is not None:
        raise ValueError("a CPU prediction cannot claim actual landing")
    support = prediction["landing_support"]
    pose = np.asarray(place["actual_box_pose_world"], float)
    box = OBB(pose[:3, 3], support["payload_half_extents_m"], pose[:3, :3], target_name, "carton")
    recorded_supports = [OBB(np.asarray(b["pose_world"])[:3, 3], b["half_extents_m"],
                    np.asarray(b["pose_world"])[:3, :3], b["name"], "conveyor")
                for b in support["support_obbs"]]
    actual = predict_release(box, recorded_supports if supports is None else supports,
        mode=prediction.get("mode", SHORT_DROP_RELEASE), obstacles=() if obstacles is None else obstacles,
        linear_velocity=prediction["linear_velocity_m_s"], angular_velocity=prediction["angular_velocity_rad_s"],
        policy=ReleasePolicy(**prediction["policy"]) if policy is None else policy, contact_tolerance_m=support["tolerance_m"],
        edge_tolerance_m=support["edge_tolerance_m"])
    if (not actual["accepted"] or not np.allclose(pose, prediction["release_pose_world"], atol=1e-12, rtol=0)
            or not np.allclose(actual["predicted_landing_pose_world"], prediction["predicted_landing_pose_world"], atol=1e-10, rtol=0)):
        raise ValueError("recorded release prediction disagrees with actual release geometry")
    actual["verification_scope"] = ("CURRENT_ENVIRONMENT" if require_current_environment
                                    else "RECORDED_GEOMETRY_ONLY" if obstacles is None else "SUPPLIED_ENVIRONMENT")
    return actual


def release_flight_envelope(box, prediction):
    """Conservative world AABB of bounded rotating flight, including between samples."""
    duration = float(prediction["flight_time_s"])
    if prediction.get("mode") == IDEAL_RECEPTION_RELEASE:
        delta = np.asarray(prediction["predicted_landing_pose_world"])[:3, 3] - box.center
        return OBB(box.center + delta / 2, box.half_extents + np.abs(box.rotation.T @ delta) / 2,
                   box.rotation, box.name, box.category)
    if duration == 0:
        return box
    policy = ReleasePolicy(**prediction["policy"])
    velocity = np.asarray(prediction["linear_velocity_m_s"])
    omega = np.asarray(prediction["angular_velocity_rad_s"])
    count = max(1, int(np.ceil(duration/policy.prediction_step_s)))
    times = list(np.linspace(0, duration, count+1))
    apex = velocity[2]/policy.gravity_m_s2
    if 0 < apex < duration:
        times.append(apex)
    samples = [flight_box(box, velocity, omega, t, policy.gravity_m_s2).corners() for t in times]
    # Translation extrema are included exactly (endpoints plus vertical apex).
    # Bound unsampled rotation by its angular speed and the unchanged radius.
    # predict_release separately checks uncertainty-expanded environment flight.
    padding = duration/count*np.linalg.norm(omega)*np.linalg.norm(box.half_extents) + 1e-12
    vertices = np.concatenate(samples)
    lower, upper = vertices.min(axis=0)-padding, vertices.max(axis=0)+padding
    return OBB((lower+upper)/2, (upper-lower)/2, np.eye(3), box.name, box.category)
