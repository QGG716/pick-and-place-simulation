"""Explicit downstream assumption, entered only after an observed real landing.

This module contains no simulator dependencies. The runtime owns the contact
proof and switches the same body; planning consumes the resulting ID registry.
"""
from __future__ import annotations

import copy
import numpy as np

from .geometry import OBB

LANDED = "LANDED_IDEAL_TRANSPORT"
OUTFED = "OUTFED_ASSUMED"
LANDING_SOURCE = "ACTUAL_RELEASE_AND_RECEIVER_TOP_CONTACT"
RECEPTION_SOURCE = "RECEPTION_ASSUMED"
IDEAL_RECEPTION_ACCEPTED = "IDEAL_RECEPTION_ACCEPTED"
HANDOFF_SOURCE = "IDEAL_DOWNSTREAM_FULL_ENVELOPE_CROSSING"


def transport_policy(value=None):
    policy = dict(value or {"mode": "strict_physics"})
    allowed = {"mode", "reception_mode", "tail_protection_enabled", "evaluate_tipping",
               "evaluate_post_landing_environment_collisions", "downstream_capacity", "output_plane"}
    if set(policy) - allowed:
        raise ValueError("unknown transport policy fields")
    if policy.get("reception_mode", "physical") not in {"physical", "ideal"}:
        raise ValueError("unknown reception mode")
    if policy.get("reception_mode") == "ideal" and policy.get("mode") != "ideal_outfeed":
        raise ValueError("ideal reception requires ideal outfeed")
    if policy.get("mode") not in {"strict_physics", "ideal_outfeed"}:
        raise ValueError("unsupported post-landing transport mode")
    if policy["mode"] == "ideal_outfeed":
        for key in ("tail_protection_enabled", "evaluate_tipping",
                    "evaluate_post_landing_environment_collisions"):
            if policy.get(key) is not False:
                raise ValueError(f"ideal_outfeed requires explicit {key}=false")
        if policy.get("downstream_capacity") != "assumed_sufficient":
            raise ValueError("ideal outfeed requires the explicit downstream assumption")
        plane = policy.get("output_plane", {})
        if not plane.get("name") or plane.get("direction") != "-X":
            raise ValueError("ideal outfeed requires a named -X output plane")
        if not np.isfinite(float(plane.get("x_m", float("nan")))):
            raise ValueError("output plane must be finite")
    return policy


def ideal_transport_ids(policy, records):
    """Fail closed on a forged/incomplete state, never exempt a carton category."""
    enabled = transport_policy(policy)["mode"] == "ideal_outfeed"
    result = set()
    for name, record in records.items():
        if record.get("state") not in {LANDED, OUTFED}:
            continue
        if not enabled or record.get("completion_source") not in {LANDING_SOURCE, RECEPTION_SOURCE}:
            raise ValueError("ideal state lacks its policy or actual reception source")
        if record.get("carton_id") != name or not record.get("attachment_removed"):
            raise ValueError("ideal transport requires the same released carton identity")
        assumed = record.get("completion_source") == RECEPTION_SOURCE
        if assumed:
            if (policy.get("reception_mode") != "ideal" or not record.get("actual_attachment_observed")
                    or not record.get("reception_region", {}).get("accepted")):
                raise ValueError("assumed reception cannot satisfy physical reception policy")
        elif not record.get("actual_top_contact_observed"):
            raise ValueError("physical reception requires observed receiver contact")
        if "takeover_pose_world" not in record:
            raise ValueError("takeover pose is missing")
        if record["state"] == OUTFED and record.get("handoff_source") != HANDOFF_SOURCE:
            raise ValueError("outfed state lacks explicit ideal handoff evidence")
        result.add(name)
    return result


def begin_ideal_transport(box: OBB, *, receiver_name, receivers, directions,
                          time_s, policy, attachment_removed, top_contact_observed,
                          support_geometry_accepted, expected_target=None, actual_attachment_observed=False,
                          maximum_drop_m=.05, reception_supports=None, release_policy=None,
                          released_handoff=None):
    policy = transport_policy(policy)
    assumed = policy.get("reception_mode") == "ideal"
    region = None
    if assumed:
        if box.name != expected_target or not actual_attachment_observed:
            raise ValueError("ideal reception requires the actually attached task target")
        if released_handoff is None:
            raise ValueError("ideal takeover requires measured legal release evidence")
        region = released_handoff.takeover_region(box, time_s=time_s, receiver=receiver_name, policy=policy)
        if (maximum_drop_m != region["height_policy"]["maximum_drop_m"]
                or release_policy is not None and release_policy.to_mapping() != region["height_policy"]):
            raise ValueError("ideal takeover release policy differs from the receipt")
        supports = reception_supports or [receivers[receiver_name]]
        bound_supports = region["support"]["support_obbs"]
        if len(supports) != len(bound_supports) or any(
                not any(s.name == r["name"] and np.array_equal(s.world_from_local, r["pose_world"])
                        and np.array_equal(s.half_extents, r["half_extents_m"]) for s in supports)
                for r in bound_supports):
            raise ValueError("ideal takeover receiver geometry differs from the receipt")
        if not region["accepted"]:
            raise ValueError("ideal reception region rejected")
    if (policy["mode"] != "ideal_outfeed" or not attachment_removed
            or (not assumed and (not top_contact_observed or not support_geometry_accepted))):
        raise ValueError("ideal takeover requires actual release and qualified first top contact")
    if receiver_name not in receivers:
        raise ValueError("unknown actual receiving conveyor")
    if released_handoff is not None and released_handoff._record is not None:
        return released_handoff._record
    longitudinal = next((b for name, b in receivers.items()
                         if np.allclose(directions[name], [-1., 0., 0.])), None)
    if longitudinal is None:
        raise ValueError("fixed longitudinal conveyor is missing")
    direction = np.asarray(directions[receiver_name], float)
    route = []
    release_box = box
    if assumed:
        pose = np.asarray(region["reception_pose_world"])
        box = OBB(pose[:3, 3], box.half_extents, pose[:3, :3], box.name, box.category)
        route.append(box.center.tolist())
    if np.allclose(direction, [0., -1., 0.]):
        # The connector is an explicit ideal transfer at the longitudinal
        # centreline. Preserve the measured height and rotation throughout.
        if box.center[1] < longitudinal.center[1] - 1e-6:
            raise ValueError("transverse takeover lies downstream of its -Y connector")
        route.append([float(box.center[0]), float(longitudinal.center[1]), float(box.center[2])])
    elif not np.allclose(direction, [-1., 0., 0.]):
        raise ValueError("unexpected fixed conveyor direction")
    half_x = float((np.abs(box.rotation) @ box.half_extents)[0])
    exit_x = float(policy["output_plane"]["x_m"]) - half_x - 0.001
    if exit_x >= box.center[0]:
        raise ValueError("output plane must be downstream of actual landing")
    route.append([exit_x, route[-1][1] if route else float(box.center[1]), float(box.center[2])])
    record = {"carton_id": box.name, "state": LANDED,
            "completion_source": RECEPTION_SOURCE if assumed else LANDING_SOURCE, "attachment_removed": True,
            "actual_attachment_observed": bool(actual_attachment_observed), "reception_region": region,
            "actual_top_contact_observed": bool(top_contact_observed), "receiver": receiver_name,
            "direction_world": direction.tolist(), "held": False,
            "takeover_time_s": float(time_s), "time_s": float(time_s),
            "takeover_pose_world": release_box.world_from_local.tolist(),
            "pose_world": release_box.world_from_local.tolist(),
            "half_extents_m": box.half_extents.tolist(), "route_world_m": route,
            "route_index": 0, "output_plane": copy.deepcopy(policy["output_plane"]),
            "model": ("SAME_BODY_BOUNDED_IDEAL_RECEPTION" if assumed else
                      "SAME_BODY_KINEMATIC_COLLISION_DISABLED_AFTER_ACTUAL_LANDING"),
            "post_landing_physics_qualified": False}
    if released_handoff is not None:
        record["release_handoff_evidence"] = released_handoff.evidence()
        released_handoff._record = record
    return record


def advance_ideal_transport(record, *, dt_s, speed_m_s):
    """Continuous distance in simulation time, with a full-envelope exit test."""
    if dt_s < 0 or speed_m_s <= 0 or not np.isfinite(dt_s + speed_m_s):
        raise ValueError("invalid ideal transport step")
    if record["state"] == OUTFED:
        return None
    pose = np.asarray(record["pose_world"], float)
    remaining = float(dt_s * speed_m_s)
    route = record["route_world_m"]
    index = record["route_index"]
    while remaining > 0 and index < len(route):
        delta = np.asarray(route[index]) - pose[:3, 3]
        distance = float(np.linalg.norm(delta))
        if distance <= remaining:
            pose[:3, 3] = route[index]
            remaining -= distance
            index += 1
        else:
            pose[:3, 3] += delta * (remaining / distance)
            remaining = 0.
    record.update(pose_world=pose.tolist(), route_index=index,
                  time_s=record["time_s"] + dt_s)
    max_x = float(pose[0, 3] + np.abs(pose[0, :3]) @ record["half_extents_m"])
    if max_x < record["output_plane"]["x_m"]:
        record.update(state=OUTFED, handoff_source=HANDOFF_SOURCE,
                      outfeed_time_s=record["time_s"], full_envelope_max_x_m=max_x)
        return {"event": OUTFED, "carton_id": record["carton_id"],
                "time_s": record["time_s"], "output_plane": record["output_plane"],
                "full_envelope_max_x_m": max_x, "source": HANDOFF_SOURCE}
    return None
