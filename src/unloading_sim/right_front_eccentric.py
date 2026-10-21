"""Explicit original-layer right/front +world-Y freedom; no carton motion."""
from dataclasses import replace
import json
import numpy as np
from .geometry import OBB
from .moveit2_backend import box_message, digest
from .independent_cups import (m710_cup_array_from_mapping,
    evaluate_independent_cup_geometry, select_ideal_independent_cups)


def authorize(target, frozen):
    if frozen is None:
        raise ValueError("ORIGINAL_RIGHT_AUTHORITY_REQUIRED")
    original = json.loads(frozen.original_cartons_json)
    boxes = []
    for b in original:
        pose = np.asarray(b["pose"])
        boxes.append(OBB(pose[:3,3], np.asarray(b["size"])/2, pose[:3,:3], b["id"], "carton"))
    # Recompute the original role from ALL frozen cartons, not remaining Y order
    # or an incoming RIGHT_END label. The connector separately binds this capture.
    recaptured = type(frozen).capture(boxes)
    if recaptured.evidence() != frozen.evidence():
        raise ValueError("ORIGINAL_RIGHT_FROZEN_CAPTURE_CHANGED")
    matches = [b for b in boxes if b.name == target.target_id]
    if (len(matches) != 1 or box_message(matches[0]) != box_message(target.box)
            or target.target_id not in frozen.target_ids
            or frozen.evidence()["roles"][target.target_id]["role"] != "RIGHT_END"):
        raise ValueError("ORIGINAL_RIGHT_ONLY")
    if frozen.evidence().get('selection')=='ALL_ORIGINAL_CARTONS_HIGHEST_REMOVABLE_FIRST':
        # Extending scheduling to 40 cartons cannot extend the original top-row
        # eccentric permission to the right ends of newly exposed lower rows.
        from .lifted_reception import bounds
        if abs(bounds(target.box)[1][2]-max(bounds(b)[1][2] for b in boxes))>1e-8:
            raise ValueError('ECCENTRIC_ORIGINAL_TOP_LAYER_ONLY')
    top = [b for b in boxes if b.name in frozen.target_ids]
    if any(b.center[1] < target.box.center[1]-1e-12 for b in top):
        raise ValueError("ORIGINAL_RIGHT_ONLY")
    if target.face_role != "FRONT" or target.geometric_face_id != "x-":
        raise ValueError("ECCENTRIC_FRONT_ONLY")
    if (abs(float(target.outward_normal[1])) > 1e-12
            or not np.allclose(target.box.rotation, np.eye(3), atol=1e-12, rtol=0)):
        raise ValueError("WORLD_Y_TANGENCY_OR_AXIS_ALIGNED_SCOPE_UNSUPPORTED")
    return dict(mode="ORIGINAL_RIGHT_FRONT_WORLD_POSITIVE_Y",
        original_cartons_sha256=digest(original), original_role="RIGHT_END",
        original_target=box_message(target.box), world_direction=[0.,1.,0.])


def geometry(target, suction):
    return evaluate_independent_cup_geometry(target.contact_pose, target.box,
        target.legacy_face, m710_cup_array_from_mapping(suction),
        max_attachment_gap_m=.002, maximum_penetration_m=.002,
        max_normal_misalignment_rad=np.deg2rad(5.),
        suction_edge_margin_m=float(suction.get("suction_edge_margin_m", 0.)))


def validate_eccentric(target):
    authorize(target, target.eccentric_authority)
    delta = target.world_y_offset_m
    if not np.isfinite(delta) or delta < 0:
        raise ValueError("ECCENTRIC_NONNEGATIVE_FINITE_DELTA_REQUIRED")
    expected_rotation = np.array([[0.,0.,1.],[0.,-1.,0.],[1.,0.,0.]])
    if (not np.allclose(target.contact_pose[:3,3], target.face_center+[0.,delta,0.], atol=1e-12, rtol=0)
            or not np.allclose(target.contact_pose[:3,:3], expected_rotation, atol=1e-12, rtol=0)):
        raise ValueError("ECCENTRIC_ONLY_WORLD_POSITIVE_Y_NO_ROTATION")
    selection = select_ideal_independent_cups(geometry(target, target.eccentric_suction),
        pose_source="REQUESTED_ECCENTRIC_FULL_RING_GEOMETRY")
    if tuple(selection.geometrically_eligible_mask) != target.predicted_eligible_mask:
        raise ValueError("ECCENTRIC_CUP_GEOMETRY_CHANGED")


def eccentric_target(centered, frozen, delta, suction):
    centered.validate()
    authorize(centered, frozen)
    if not np.isfinite(delta) or delta < 0:
        raise ValueError("ECCENTRIC_NONNEGATIVE_FINITE_DELTA_REQUIRED")
    if not np.array_equal(centered.contact_pose[:3,3], centered.face_center):
        raise ValueError("ECCENTRIC_GENERATION_REQUIRES_REAL_CENTER_BASELINE")
    contact = centered.contact_pose.copy(); contact[:3,3] += [0.,delta,0.]
    approach = centered.approach_pose.copy(); approach[:3,3] += [0.,delta,0.]
    target = replace(centered, contact_pose=contact, approach_pose=approach,
        world_y_offset_m=float(delta), eccentric_authority=frozen, eccentric_suction=dict(suction))
    selection = select_ideal_independent_cups(geometry(target, suction),
        pose_source="REQUESTED_ECCENTRIC_FULL_RING_GEOMETRY")
    return replace(target, predicted_eligible_mask=tuple(selection.geometrically_eligible_mask)).validate()


def offset_intervals(centered, frozen, suction):
    """Union of exact full-ring admissible delta intervals, one interval/cup."""
    authorize(centered, frozen)
    array = m710_cup_array_from_mapping(suction)
    edge = float(suction.get("suction_edge_margin_m", 0.))
    intervals = []
    for cup in array.cups:
        world = centered.contact_pose[:3,:3] @ np.asarray(cup.center_contact_frame_m)
        # Axis-aligned front only. A complete circle has radius in both tangents.
        if abs(world[2])+cup.seal_radius_m+edge > centered.box.half_extents[2]+1e-12:
            continue
        limit = centered.box.half_extents[1]-cup.seal_radius_m-edge
        lo, hi = max(0., -limit-world[1]), limit-world[1]
        if hi >= lo: intervals.append((float(lo),float(hi)))
    merged = []
    for lo, hi in sorted(intervals):
        if merged and lo <= merged[-1][1]+1e-12:
            merged[-1] = (merged[-1][0], max(hi,merged[-1][1]))
        else: merged.append((lo,hi))
    if not merged: raise ValueError("ECCENTRIC_NO_FULL_RING_DOMAIN")
    return tuple(merged)


def candidate_deltas(intervals):
    """Finite incremental midpoint refinement at cup-pitch/2 resolution."""
    yield 0.
    queue = list(intervals)
    while queue:
        lo, hi = queue.pop(0)
        mid = (lo+hi)/2
        if mid > 1e-12: yield mid
        if hi-lo > .024:
            queue.extend(((lo,mid),(mid,hi)))


def candidate_visits(options, intervals):
    """Diagonal fair visits, never materialize the delta x height product.

    Same-height sync alternatives are consecutive within a concrete target.
    Caller may reuse kinematic seeds only; a new native worker rechecks geometry.
    """
    deltas = candidate_deltas(intervals)
    known = []
    exhausted = False
    diagonal = 0
    while not exhausted or diagonal < len(known)+len(options)-1:
        if not exhausted:
            try: known.append(next(deltas))
            except StopIteration: exhausted = True
        for i in range(min(diagonal+1,len(options))):
            j = diagonal-i
            if j < len(known):
                yield dict(options[i], world_y_offset_m=known[j])
        diagonal += 1


def validate_accepted_target(plan, frozen, cartons):
    """Independent commit boundary: rebuild target from unchanged box geometry."""
    from .contact_target import ContactTarget
    e = plan["target"]
    box = next(b for b in cartons if b.name == e["target_id"])
    target = ContactTarget(e["target_id"],e["frame_id"],e["scene_revision"],e["source"],
        e["face_role"],e["geometric_face_id"],box,0,-1,np.asarray(e["face_center_m"]),
        np.asarray(e["face_boundary_m"]),np.asarray(e["outward_normal"]),np.asarray(e["in_plane_x"]),
        np.asarray(e["contact_pose"]),np.asarray(e["approach_pose"]),e["standoff_m"],
        world_y_offset_m=e["world_y_offset_m"],eccentric_authority=frozen,
        eccentric_suction=plan["eccentric_contact_validation"]["suction"],
        predicted_eligible_mask=tuple(e["predicted_eligible_mask"]))
    target.validate()
    if target.evidence() != e:
        raise ValueError("ECCENTRIC_ACCEPTANCE_IDENTITY_CHANGED")
    masks = plan["eccentric_contact_validation"]
    if (masks["expected"]["geometrically_eligible_mask"] != list(target.predicted_eligible_mask)
            or masks["expected"]["commanded_active_mask"] != masks["actual"]["commanded_active_mask"]
            or masks["actual"]["actual_contact_mask"] != masks["actual"]["commanded_active_mask"]
            or not any(masks["actual"]["actual_contact_mask"])):
        raise ValueError("ECCENTRIC_ACCEPTANCE_MASK_CHANGED")
    if plan["option"]["segment_directions_world"] != [[-1.,0.,0.]]:
        raise ValueError("ECCENTRIC_ACCEPTANCE_EXTRACTION_CHANGED")
