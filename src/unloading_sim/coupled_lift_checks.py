"""Fail-closed, synchronized checks of the project-owned coupled conveyor solids.

These are discrete geometric checks, not a continuous sweep certificate.  Robot
mesh checks are a separate native gate.  AABB overlap is only a broad phase;
every rejected pair is refined against the actual OBBs returned by the model.
"""
from itertools import combinations
import math

import numpy as np

from .conveyor_lift import coupled_conveyor_state
from .lifted_reception import bounds
from .moveit2_backend import box_message, digest
from .pair_clearance import obb_surface_distance


def assembly_mates(state):
    """Enumerate exact pairs from the model's pin/slider/support topology.

    Never exempt a name prefix, a whole mechanism, a robot link or a carton.
    Rear stands mate to the bearing, NOT to the rotating conveyor body.  A
    support on the floor does not acquire permission to touch the chassis.
    """
    pairs = {}

    def mate(a, b, reason):
        pairs[tuple(sorted((a, b)))] = reason

    for stack in state.mechanism["stacks"]:
        key, n = stack["id"], stack["stages"]
        for level in range(n):
            for side in (0, 1):
                arm = f"{key}_arm_{level}_{side}"
                mate(arm, f"{key}_pin_{level}", "DECLARED_CROSSING_PIN")
                mate(arm, f"{key}_end_pin_{level}_{side}", "DECLARED_END_PIN")
                mate(arm, f"{key}_end_pin_{level+1}_{1-side}", "DECLARED_END_PIN")
                if level == 0:
                    mate(arm, f"{key}_base_rail", "DECLARED_BASE_PIN_SLIDER_RAIL")
                if level == n-1:
                    mate(arm, f"{key}_head_rail", "DECLARED_HEAD_PIN_SLIDER_RAIL")
        for side in (0, 1):
            mate(f"{key}_end_pin_0_{side}", f"{key}_base_rail", "DECLARED_BASE_PIN_SLIDER_RAIL")
            mate(f"{key}_end_pin_{n}_{side}", f"{key}_head_rail", "DECLARED_HEAD_PIN_SLIDER_RAIL")
            mate(f"{key}_foot_{side}", f"{key}_base_rail", "DECLARED_FIXED_RAIL_SUPPORT")
            mate(f"{key}_foot_{side}", "trailer_floor", "DECLARED_FOOT_ON_FLOOR")
        if key in {"lift_transverse_left", "lift_transverse_right"}:
            mate(f"{key}_head_rail", f"{key}_mount", "DECLARED_HEAD_MOUNT")
            mate(f"{key}_mount", "conveyor_transverse", "DECLARED_TRANSVERSE_MOUNT")
        elif key in {"lift_longitudinal_near", "lift_longitudinal_rear"}:
            mate(f"{key}_head_rail", f"{key}_swivel_head", "DECLARED_SWIVEL_HEAD")
            mate(f"{key}_swivel_head", "conveyor_longitudinal", "DECLARED_SWIVEL_CONVEYOR_MOUNT")
        else:
            raise ValueError("UNKNOWN_COUPLED_STACK_TOPOLOGY")
    for stand in ("lift_rear_stand_0", "lift_rear_stand_2"):
        mate(stand, "trailer_floor", "DECLARED_STAND_ON_FLOOR")
        mate(stand, "lift_rear_bearing", "DECLARED_REAR_BEARING_SUPPORT")
    mate("lift_rear_bearing", "conveyor_longitudinal", "DECLARED_REAR_REVOLUTE_BEARING")
    return pairs


def _pair_geometry(a, b):
    gap = obb_surface_distance(a, b)
    signed = float(a.signed_distance_obb(b))
    if not np.isfinite([gap, signed]).all():
        raise ValueError("COUPLED_PAIR_GEOMETRY_UNKNOWN")
    result = dict(pair=[a.name, b.name], surface_distance_m=gap,
                  signed_sat_separation_m=signed,
                  sat_value_is_not_general_penetration_depth=True,
                  classification="SOLID_OBB_INTERSECTION" if signed < -1e-9 else
                  "SOLID_OBB_CONTACT" if gap <= 1e-9 else "CLEARANCE_INSUFFICIENT",
                  geometry_scope="PROJECT_CONCEPT_SOLID_OBBS_NOT_ORIGINAL_CAD",
                  original_cad_penetration_proven=False,
                  query_method="OBB_SAT_AND_VERTEX_FACE_EDGE_EDGE_EUCLIDEAN",
                  bodies=[box_message(a), box_message(b)])
    # Only aligned boxes have an exact box intersection. Never label a rotated
    # bounding-box overlap as penetration volume or as a narrow-phase verdict.
    if all(np.allclose(x.rotation, np.eye(3), atol=1e-12, rtol=0) for x in (a, b)):
        al, ah = bounds(a)
        bl, bh = bounds(b)
        lo, hi = np.maximum(al, bl), np.minimum(ah, bh)
        if np.all(hi-lo > 1e-9):
            result["exact_axis_aligned_intersection"] = dict(
                min_world_m=lo.tolist(), max_world_m=hi.tolist(),
                size_m=(hi-lo).tolist(), volume_m3=float(np.prod(hi-lo)))
    return result


def check_coupled_state(layout, height, cartons=None, *, clearance_m=.005, designed_contacts=()):
    if not np.isfinite(clearance_m) or clearance_m < 0:
        raise ValueError("INVALID_COUPLED_CLEARANCE")
    state = coupled_conveyor_state(layout, height, cartons)
    mates = assembly_mates(state)
    boxes = state.boxes
    # Vectorized broad phase; candidate pairs still require an exact OBB query.
    centers = np.asarray([b.center for b in boxes])
    extents = np.asarray([np.abs(b.rotation) @ b.half_extents for b in boxes])
    lower = np.maximum(np.abs(centers[:, None]-centers[None, :])-
                       extents[:, None]-extents[None, :]-1e-12, 0.)
    nearby = np.linalg.norm(lower, axis=2) <= clearance_m+1e-9
    failures, allowed = [], []
    refined = 0
    # Existing fixed assembly/stack pairs keep their established policy. This
    # additional gate covers every new mechanism pair, plus moving conveyors
    # versus cartons and the unchanged fixed world.
    conveyors = {"conveyor_transverse", "conveyor_longitudinal"}
    original_contacts = {tuple(sorted(p)) for p in designed_contacts}
    for i, j in combinations(range(len(boxes)), 2):
        a, b = boxes[i], boxes[j]
        mechanism = a.category == "lift_mechanism" or b.category == "lift_mechanism"
        moving = a.name in conveyors or b.name in conveyors
        if not (mechanism or moving) or not nearby[i, j]:
            continue
        key = tuple(sorted((a.name, b.name)))
        refined += 1
        detail = _pair_geometry(a, b)
        if detail["surface_distance_m"]+1e-9 >= clearance_m and detail["signed_sat_separation_m"] > 0:
            continue
        rule = mates.get(key)
        # Preserve existing plane contacts only with a zero-volume separating
        # world-Y plane. A tilted longitudinal belt cannot reuse a pure-Z union.
        if rule is None and key in original_contacts:
            al, ah = bounds(a)
            bl, bh = bounds(b)
            y_plane = min(abs(ah[1]-bl[1]), abs(bh[1]-al[1])) <= 1e-9
            x_plane = min(abs(ah[0]-bl[0]), abs(bh[0]-al[0])) <= 1e-9
            if (y_plane or x_plane) and detail["signed_sat_separation_m"] >= -1e-9:
                rule = "FROZEN_ASSEMBLY_NONPENETRATING_SEPARATING_PLANE"
        if rule:
            allowed.append(dict(pair=list(key), rule=rule))
        else:
            detail["required_clearance_m"] = clearance_m
            failures.append(detail)
    return dict(status="PASS_DISCRETE_GEOMETRY" if not failures else "COUPLED_WORLD_GEOMETRY_REJECTED",
                passed=not failures, height_m=float(height), scene_fingerprint=state.scene_fingerprint,
                world_sha256=digest(sorted([box_message(b) for b in boxes], key=lambda b: b["id"])),
                object_count=len(boxes), carton_ids=[b.name for b in boxes if b.category == "carton"],
                failures=failures, allowed_assembly_pairs=allowed, refined_pairs=refined,
                robot_mesh_check="SEPARATE_GATE", continuous_swept_proof=False,
                physical_execution="NOT_RUN", qualification_status="NOT_EVALUATED", execution_ready=False)


def check_coupled_interval(layout, start_height, end_height, cartons=None, *, height_step_m=.01,
                           designed_contacts=()):
    """Both directions and hold; simultaneous states, never independent unions.

    Diagnostic coverage of the entire requested height interval is retained even
    when its start fails. This does not authorize traversal of a rejected start.
    """
    if not np.isfinite([start_height, end_height, height_step_m]).all() or height_step_m <= 0:
        raise ValueError("INVALID_COUPLED_INTERVAL")
    count = max(1, math.ceil(abs(end_height-start_height)/height_step_m))
    if count > 4096:
        raise ValueError("COUPLED_INTERVAL_RESOURCE_LIMIT")
    heights = [start_height] if start_height == end_height else np.linspace(start_height, end_height, count+1)
    samples = [check_coupled_state(layout, h, cartons, designed_contacts=designed_contacts) for h in heights]
    return dict(status="PASS_DISCRETE_GEOMETRY" if all(s["passed"] for s in samples)
                else "COUPLED_INTERVAL_GEOMETRY_REJECTED",
                start_height_m=float(start_height), end_height_m=float(end_height),
                direction="HOLD" if start_height == end_height else "UP" if end_height > start_height else "DOWN",
                requested_height_step_m=height_step_m, sample_count=len(samples),
                maximum_height_step_m=0. if len(samples) == 1 else float(abs(end_height-start_height)/count),
                sample_parameter="HEIGHT_NOT_TIME", samples=samples,
                limitations="DISCRETE_SYNCHRONIZED_OBB_CHECKS; BETWEEN_SAMPLE_CONTACT_NOT_EXCLUDED; ROBOT_SEPARATE",
                execution_duration_s=None, continuous_swept_proof=False,
                physical_execution="NOT_RUN", execution_ready=False)
