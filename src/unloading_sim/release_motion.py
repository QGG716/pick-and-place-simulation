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
MOTION_SEMANTICS = "m710_adaptive_contact_release_v2"


@dataclass(frozen=True)
class ReleasePolicy:
    maximum_drop_m: float = 0.05
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


def rigid_com_velocity(tcp_velocity, angular_velocity, tcp_position, com_position):
    vectors = [np.asarray(v, dtype=float) for v in
               (tcp_velocity, angular_velocity, tcp_position, com_position)]
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in vectors):
        raise ValueError("rigid velocity inputs must be finite three-vectors")
    linear, omega, tcp, com = vectors
    return linear + np.cross(omega, com - tcp)


def receiver_footprint_reserve(box: OBB, supports, reserve_m, *, contact_tolerance_m=.002,
                               edge_tolerance_m=1e-6):
    """Enlarge payload XY coverage, preserving adjoining support seams.

    This conservative coverage proxy does not alter the actual body's shape.
    Shrinking each conveyor separately would invent gaps inside their union.
    """
    if not np.isfinite(reserve_m) or reserve_m < 0:
        raise ValueError("footprint reserve must be finite and nonnegative")
    padding = reserve_m * (np.abs(box.rotation[0]) + np.abs(box.rotation[1]))
    proxy = OBB(box.center, box.half_extents + padding, box.rotation, box.name, box.category)
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
    if mode not in {SUPPORTED_RELEASE, SHORT_DROP_RELEASE}:
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


def verify_release_prediction(place, target_name):
    """Recompute recorded drop geometry before accepting a stage contract."""
    prediction = place["release_prediction"]
    if prediction.get("actual_landing_state") is not None:
        raise ValueError("a CPU prediction cannot claim actual landing")
    support = prediction["landing_support"]
    pose = np.asarray(place["actual_box_pose_world"], float)
    box = OBB(pose[:3, 3], support["payload_half_extents_m"], pose[:3, :3], target_name, "carton")
    supports = [OBB(np.asarray(b["pose_world"])[:3, 3], b["half_extents_m"],
                    np.asarray(b["pose_world"])[:3, :3], b["name"], "conveyor")
                for b in support["support_obbs"]]
    actual = predict_release(box, supports, mode=SHORT_DROP_RELEASE,
        linear_velocity=prediction["linear_velocity_m_s"], angular_velocity=prediction["angular_velocity_rad_s"],
        policy=ReleasePolicy(**prediction["policy"]), contact_tolerance_m=support["tolerance_m"],
        edge_tolerance_m=support["edge_tolerance_m"])
    if (not actual["accepted"] or not np.allclose(pose, prediction["release_pose_world"], atol=1e-12, rtol=0)
            or not np.allclose(actual["predicted_landing_pose_world"], prediction["predicted_landing_pose_world"], atol=1e-10, rtol=0)):
        raise ValueError("recorded release prediction disagrees with actual release geometry")
    return actual
