"""Local historical path adaptation using the current connector's exact gates."""
from time import perf_counter

import numpy as np

from .conveyor_placement import PlacementCandidate, support_union_audit
from .geometry import OBB
from .ik import solve_ik
from .release_motion import (ReleasePolicy, verify_release_prediction, release_flight_envelope,
                            departure_sweep, reception_footprint_audit)
from .validation_physics import RigidAttachment


def solve_local(c, pose, seed):
    """Tighter numerical target, unchanged acceptance/clearance/contact rules."""
    started = perf_counter()
    result = solve_ik(
        c.robot, pose, seed, max_iterations=min(100, int(c.ik["max_iterations"])),
        damping=min(.005, float(c.ik["damping"])), max_step=float(c.ik["max_step_rad"]),
        position_tolerance=min(1e-8, float(c.ik["position_tolerance_m"])),
        orientation_tolerance=min(1e-8, float(c.ik["orientation_tolerance_rad"])),
        orientation_weight=float(c.ik["orientation_weight"]),
        deadline_monotonic=c._deadline_monotonic,
        deadline_provider=lambda: c._limit(c._deadline_monotonic, c._request_deadline_monotonic))
    c._statistics["ik_calls"] += 1
    c._statistics["ik_seeds_attempted"] += int(result.iterations > 0)
    c._statistics["ik_iterations_consumed"] += result.iterations
    c._statistics["trajectory_ik_wall_seconds"] += perf_counter() - started
    return result


def _slice(hint, stage):
    segment = hint["segment"]
    first, last = segment["stage_ranges"][stage]
    return [np.asarray(q, float).copy() for q in segment["path"][first:last+1]]


def adapt_branch(c, *, hint, target, face, requested_virtual_contact, grasp_q, home_q,
                 all_obstacles, receiver, support_names, suction, seed):
    from .layout_trajectory import PhysicalContactAttachment
    trace = {"seed": seed, "stages": {}, "history": dict(hint["attempt_provenance"])}
    old = hint["segment"]
    history = trace["history"]
    def rejected(failure):
        return None, failure, trace
    solution = solve_local(c, requested_virtual_contact, grasp_q)
    if not solution.success:
        return rejected({"reason": solution.message, "stage": "history_contact_ik"})
    q = solution.q
    if np.max(np.abs(q - grasp_q)) > hint["policy"].maximum_joint_adaptation_rad:
        return rejected({"reason": "HISTORY_JOINT_ADAPTATION_BOUND", "stage": "contact"})
    try:
        selection = c._contact_selection(q, target, face, suction)
    except ValueError as exc:
        return rejected({"reason": "CONTACT_CUP_GEOMETRY_INVALID", "stage": "contact", "detail": str(exc)})
    failure = c._state_failure(q, all_obstacles, target_contact=target, stage="contact_endpoint")
    history["new_commanded_mask"] = selection["commanded_active_mask"]
    history["new_contact_pose_world"] = c.physical_from_virtual(c.robot.fk(q)).tolist()
    if failure:
        return rejected(failure)
    history["new_contact_q_rad"] = q.tolist()
    # Retain every historical free node. The actual start has a new, checked edge
    # to the first node; no colliding intermediate node is silently skipped.
    prefix = [np.asarray(node, float).copy() for node in old["path"][:hint["gate"]+1]]
    if np.max(np.abs(home_q - prefix[0])) > hint["policy"].maximum_joint_adaptation_rad:
        return rejected({"reason": "HISTORY_START_ADAPTATION_BOUND", "stage": "pregrasp"})
    prefix.insert(0, home_q.copy())
    failure = c._path_failure(prefix, all_obstacles, stage="pregrasp")
    if failure:
        return rejected(failure)
    failure = c._approach_reserve_failure(prefix[-1], target)
    if failure:
        return rejected(failure)
    c._remember_path(prefix, c._context_identity(all_obstacles, stage="pregrasp"),
                     "pregrasp", "B_STRICT_LOCAL_CONNECTION")
    prefix, improvement = c._improve_free_path(prefix, all_obstacles)
    if improvement.get("adopted"):
        c._remember_path(prefix, c._context_identity(all_obstacles, stage="pregrasp"),
                         "pregrasp", "B_STRICT_LOCAL_CONNECTION", improvement["after"])
    contact, failure, terminal = c._cartesian(prefix[-1], requested_virtual_contact,
        all_obstacles, seed=seed, target_contact=target, stage="contact")
    trace["stages"]["approach"] = {"selected_mode": "history_free_prefix_current_terminal",
        "free_connection_end_index": len(prefix)-1, "terminal_contact_start_index": len(prefix)-1,
        "terminal": terminal, "actual_start_connected": True,
        "simplification": improvement}
    if failure:
        return rejected(failure)
    # Use the precisely solved endpoint, then recheck the complete rebuilt arc.
    if np.max(np.abs(contact[-1] - q)) > c.budget.cartesian_max_branch_step_rad:
        return rejected({"reason": "IK_BRANCH_JUMP", "stage": "contact"})
    contact[-1] = q.copy()
    selection = c._contact_selection(q, target, face, suction)
    failure = c._path_failure(contact, all_obstacles, target_contact=target, stage="contact")
    if failure:
        return rejected(failure)
    c._remember_path([*prefix, *contact[1:]],
        c._context_identity(all_obstacles, target_contact=target, stage="contact"), "contact", "C_COMPLETE_APPROACH")
    physical = c.physical_from_virtual(c.robot.fk(q))
    rigid = RigidAttachment.capture(physical, target)
    attachment = PhysicalContactAttachment(c.robot, rigid, c.flange_from_virtual_task_tcp,
                                           c.flange_from_physical_contact)
    if not np.allclose(attachment.box_at(q).world_from_local, target.world_from_local, atol=1e-10, rtol=0):
        return rejected({"reason": "HISTORY_ATTACHMENT_DISCONTINUITY", "stage": "contact"})
    history["new_physical_contact_from_box"] = rigid.tcp_from_box.tolist()
    payload_obstacles = [b for b in all_obstacles if b.name != target.name]
    tracker, failure = c._initial_proximity(target, payload_obstacles, support_names)
    if failure:
        return rejected(failure)
    stages = {}
    previous = q.copy()
    for stage in ("support-release", "extraction"):
        points = _slice(hint, stage) if stage in old["stage_ranges"] else [previous.copy()]
        points[0] = previous.copy()
        failure = c._path_failure(points, payload_obstacles, attachment=attachment,
            support_names=support_names, initial_proximity=tracker, stage=stage)
        trace["stages"][stage] = {"source": "HISTORICAL_NODES_CURRENT_ATTACHMENT_FULL_EDGE_RECHECK", "failure": failure}
        if failure:
            return rejected(failure)
        stages[stage] = points
        previous = points[-1]
    if not tracker.fully_released:
        return rejected({"reason": "ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED", "stage": "extraction"})
    failure = c._extraction_reserve_failure(previous, attachment, payload_obstacles)
    if failure:
        return rejected(failure)
    # Preserve only receiver pose intent, not old support/prediction/occupancy.
    desired = c._se3(np.asarray(old["place"]["actual_box_pose_world"]), "history release intent").copy()
    names = tuple(old["place"]["support_names"])
    supports = [b for b in payload_obstacles if b.name in names and b.category == "conveyor"]
    if len(supports) != len(names):
        return rejected({"reason": "HISTORY_RECEIVER_MISSING", "stage": "place"})
    box = OBB(desired[:3, 3], target.half_extents, desired[:3, :3], target.name, target.category)
    support_z = max(float(np.max(b.corners()[:, 2])) for b in supports)
    drop = max(0., float(np.min(box.corners()[:, 2])) - support_z)
    if not c.budget.proof_of_concept and drop > c.budget.maximum_drop_m:
        return rejected({"reason": "HISTORY_RELEASE_HEIGHT_BOUND", "stage": "place"})
    landing = desired.copy()
    landing[2, 3] -= drop
    payload = OBB(landing[:3, 3], target.half_extents, landing[:3, :3], target.name, target.category)
    support = (reception_footprint_audit(payload, supports, edge_tolerance_m=c.placement_policy.edge_tolerance_m)
               if c.budget.proof_of_concept else support_union_audit(payload, supports,
                   contact_tolerance_m=c.contact_tolerance_m, edge_tolerance_m=c.placement_policy.edge_tolerance_m))
    if not support["supported"]:
        return rejected({"reason": "HISTORY_RECEIVER_SUPPORT_INVALID", "stage": "place", "support": support})
    placement = PlacementCandidate(payload, tuple(support["receiver_names"]), support,
        ("HISTORICAL_RECEIVER_INTENT_CURRENT_REVALIDATION",),
        float(np.arctan2(payload.rotation[1, 0], payload.rotation[0, 0])), 0.,
        {name: c.surface_directions_world.get(name) for name in names},
        placement_family=old["place"]["placement_family"])
    heights = c.budget.release_policy().ideal_heights() if c.budget.proof_of_concept else (drop,)
    history["release_reconstruction"] = {"historical_height_m": drop, "current_nominal_heights_m": list(heights),
        "old_release_proof_inherited": False, "source": "RECEIVER_XY_ORIENTATION_INTENT_ONLY"}
    for height in heights:
        segment, failure, trace = c._finish_place_branch(target=target, face=face,
            requested_virtual_contact=requested_virtual_contact, home_q=home_q, contact_q=q,
            physical_contact=physical, rigid=rigid, attachment=attachment, selection=selection,
            pregrasp=prefix, contact=contact, support_release=stages["support-release"],
            extraction=stages["extraction"], released_tracker=tracker, payload_obstacles=payload_obstacles,
            placement=placement, selected_supports=supports, trace=trace, seed=seed,
            release_height=height, history_hint=hint)
        if segment is not None:
            break
    return segment, failure, trace


def loaded_suffix(c, hint, extraction, preplace_virtual, place_virtual, obstacles, attachment, support_names):
    """Re-solve both receiver endpoints under the new attachment; check every edge."""
    if c.budget.proof_of_concept:
        # Retain only the early loaded prefix. Rebuild well before the old low
        # receiver approach; never edit the old release nodes in place.
        old = _slice(hint, "transit")
        supports = [b for b in obstacles if b.name in support_names]
        top = max(float(b.corners()[:, 2].max()) for b in supports)
        safe_height = c.budget.ideal_release_max_height_m + c.budget.receiver_runtime_clearance_reserve_m
        cuts = [i for i, q in enumerate(old) if attachment.box_at(q).corners()[:, 2].min() >= top + safe_height]
        cut = cuts[-1] if cuts else 0
        prefix = [extraction[-1].copy(), *old[1:cut+1]]
        failure = c._path_failure(prefix, obstacles, attachment=attachment, stage="transit")
        if failure:
            return [], [], failure
        suffix, failure, evidence = c._cartesian(prefix[-1], preplace_virtual, obstacles,
            seed=int(hint["attempt_provenance"]["candidate_id"][:8], 16), attachment=attachment, stage="transit")
        hint["attempt_provenance"]["loaded_prefix_reconstruction"] = dict(
            retained_nodes=len(prefix), old_nodes=len(old), safe_height_m=safe_height, suffix=evidence)
        if failure:
            return [], [], failure
        transit = [*prefix, *suffix[1:]]
        place, failure, _ = c._cartesian(transit[-1], place_virtual, obstacles,
            seed=0, attachment=attachment, support_names=support_names, stage="place")
        return transit, place, failure
    paths = []
    previous = extraction[-1]
    for stage, goal in (("transit", preplace_virtual), ("place", place_virtual)):
        points = _slice(hint, stage)
        result = solve_local(c, goal, points[-1])
        if not result.success:
            return [], [], {"reason": result.message, "stage": stage}
        if np.max(np.abs(result.q - points[-1])) > hint["policy"].maximum_joint_adaptation_rad:
            return [], [], {"reason": "HISTORY_RELEASE_ADAPTATION_BOUND", "stage": stage}
        points[0] = previous.copy()
        if len(points) == 1:
            points.append(result.q.copy())
        else:
            points[-1] = result.q.copy()
        failure = c._path_failure(points, obstacles, attachment=attachment, stage=stage,
                                 support_names=support_names if stage == "place" else ())
        if failure:
            return [], [], failure
        paths.append(points)
        previous = points[-1]
    return *paths, None


def current_release_sweep(c, placed, prediction, start, direction):
    flight = release_flight_envelope(placed, prediction)
    if c.post_landing_transport["mode"] == "ideal_outfeed":
        return [flight]
    tools = c.tool_collision_obbs_provider(start)
    if not tools or direction is None:
        raise ValueError("current departure geometry/direction unavailable")
    distance = max(float(np.max(b.corners() @ direction)) for b in tools) - float(np.min(placed.corners() @ direction))
    return departure_sweep(flight, direction, distance_m=max(.01, distance + c.collision_policy.pair_clearance("external", c.collision_margin_m) + c.contact_tolerance_m),
                           resolution_m=c.budget.cartesian_step_m)


def recheck_current_release(segment, connector, payload_obstacles, placed):
    """Public compatibility helper, also used by the legacy CLI import surface."""
    recorded = segment["place"]["release_prediction"]["landing_support"]["support_obbs"]
    supports = [b for b in payload_obstacles if b.name in {r["name"] for r in recorded}]
    if len(supports) != len(recorded):
        raise ValueError("current release support is missing")
    for body in supports:
        prior = next(r for r in recorded if r["name"] == body.name)
        if not np.allclose(body.world_from_local, prior["pose_world"], atol=1e-12, rtol=0) or not np.allclose(body.half_extents, prior["half_extents_m"], atol=1e-12, rtol=0):
            raise ValueError("cached receiving geometry changed; placement must be replanned")
    prediction = verify_release_prediction(segment["place"], segment["target"],
        obstacles=payload_obstacles, supports=supports,
        policy=connector.budget.release_policy(), require_current_environment=True)
    direction = getattr(connector, "surface_directions_world", {}).get(prediction["landing_support"]["receiver_names"][0])
    return prediction, current_release_sweep(connector, placed, prediction,
        np.asarray(segment["path"])[segment["release_index"]], direction)


def checked_departure(c, hint, start, placed, obstacles, direction, prediction):
    if c.budget.proof_of_concept:
        return c._departure(start, placed, obstacles, direction,
            seed=int(hint["attempt_provenance"]["candidate_id"][:8], 16),
            working_normal=c.physical_from_virtual(c.robot.fk(start))[:3, 2], release_prediction=prediction)
    path = _slice(hint, "withdrawal")
    path[0] = start.copy()
    sweep = current_release_sweep(c, placed, prediction, start, direction)
    for body in sweep:
        failure = c._path_failure(path, [*obstacles, body], target_contact=body, stage="withdrawal")
        if failure is None:
            failure = c._state_failure(path[-1], [*obstacles, body], stage="residence")
        if failure:
            return [], failure, {"current_flight_and_residence_checked": False}
    evidence = {"model": "history_current_flight_and_residence_recheck_v1",
        "sweep_samples": len(sweep), "stationary_carton_included": True,
        "next_approach_start_q_rad": path[-1].tolist(), "current_flight_and_residence_checked": True,
        "next_contact": {"status": "NOT_EVALUATED_HISTORY_HINT", "joint_path_length_rad": None}}
    if c.budget.proof_of_concept:
        return path, None, evidence
    # The old departure is already safe. Optional lookahead shares the same
    # production provider, 4 s call / 12 s request caps and parent's history slice.
    try:
        improved, failure, audit = c._departure(start, placed, obstacles, direction,
            seed=int(hint["attempt_provenance"]["candidate_id"][:8], 16),
            working_normal=c.physical_from_virtual(c.robot.fk(start))[:3, 2],
            release_prediction=prediction, verified_baseline=(path, evidence))
        if failure is None:
            return improved, None, audit
        evidence["optional_failure"] = failure
    except (ValueError, RuntimeError) as exc:
        evidence["optional_failure"] = {"reason": type(exc).__name__, "detail": str(exc)}
    return path, None, evidence
