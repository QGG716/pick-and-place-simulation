"""Optional complete-task alternatives. Raw bounded joint states remain distinct."""
from copy import deepcopy
from time import perf_counter
import hashlib

import numpy as np

from .history_adaptation import solve_local
from .motion_quality import quality_improves


def periodic_j6_candidates(robot, q):
    """Only a named, one-DOF revolute model with verified full FK periodicity."""
    names = list(getattr(robot, "active_joint_names", ()))
    model = getattr(robot, "model", None)
    if "J6" not in names or model is None:
        return []
    index = names.index("J6")
    joint = model.joints[index + 1]
    if joint.nq != 1 or not joint.shortname().startswith("JointModelR"):
        return []
    lower, upper = np.asarray(robot.joint_limits)[index]
    if not np.isfinite([lower, upper]).all():
        return []
    candidates = []
    for turns in (-1, 1):
        candidate = np.asarray(q).copy()
        candidate[index] += turns * 2*np.pi
        if not lower <= candidate[index] <= upper:
            continue
        # Check the actual tool FK at both the goal and an interior perturbation.
        equivalent = True
        for shift in (0., .013):
            first, second = np.asarray(q).copy(), candidate.copy()
            first[index] += shift
            second[index] += shift
            equivalent &= np.allclose(robot.fk(first), robot.fk(second), atol=1e-9, rtol=0)
        if equivalent:
            candidates.append(candidate)
    return candidates


def endpoint_cost(q, start, velocities):
    delta = np.abs(np.asarray(q)-start)
    return float(np.max(delta/velocities)), float(delta.sum())


def improve_complete_task(c, scene, target, baseline, scheduled_poses, *, deadline, attempt_limit):
    """At most two ordinary full-task connections after a retained D-level result.

    Same-contact seed/periodic candidates precede at most one configured roll.
    History adaptation limits are untouched: these are new ordinary plans.
    No baseline segment mutation or completion rebinding is needed unless a whole
    independently completed candidate replaces it.
    """
    started = perf_counter()
    report = dict(status="NOT_EVALUATED", candidates=[], selected="BASELINE",
        baseline_content_sha256=baseline.get("validation", {}).get("completion", {}).get("content_sha256"),
        maximum_connections=2, maximum_seed_solves=3, wall_budget_s=None if c.budget.proof_of_concept else 12.,
        generation_statistics={}, attempts_count=0)
    attempts = []
    selected = deepcopy(baseline)
    names = list(getattr(c.robot, "active_joint_names", ()))
    if "J6" not in names or not report["baseline_content_sha256"] or attempt_limit <= 0:
        report["reason"] = "NO_NAMED_BOUND_BASELINE_OR_ATTEMPT_ALLOWANCE"
        return selected, attempts, report
    stop = c._limit(deadline, c._optional_deadline(comparison=True), None if c.budget.proof_of_concept else started + 12.)
    report["deadline_monotonic"] = stop
    if stop is not None and stop-started < 1.:
        report["reason"] = "INSUFFICIENT_REMAINING_TIME"
        return selected, attempts, report
    saved_proofs = deepcopy(c._completed_tasks)
    saved_stats = dict(c._statistics)
    saved_slice = getattr(c, "candidate_slice_s", None)
    start = np.asarray(baseline["path"][0])
    old_q = np.asarray(baseline["contact"]["actual_q_rad"])
    virtual = np.asarray(baseline["contact"]["requested_virtual_task_tcp_pose_world"])
    velocity = np.asarray(getattr(c.robot.model, "velocityLimit"))
    limit = min(2, attempt_limit)
    seen = [old_q.copy()]
    try:
        with c._budget_scope(stop), c._contact_context():
            before = c._optional_quality(baseline["path"], stop)
            report["before"] = before
            if before is None:
                report["reason"] = "BASELINE_QUALITY_UNKNOWN"
                return selected, attempts, report
            pools = [(virtual, baseline["face"], "SAME_CONTACT", [start, old_q])]
            # Reuse an already allowed face/roll orientation, keeping current
            # contact position. Full rings and rigid geometry are checked anew.
            rolls = {}
            for face, (roll, pose, variant) in scheduled_poses:
                if face == baseline["face"]:
                    rolls.setdefault(int(roll), np.asarray(pose).copy())
            current_rotation = c.physical_from_virtual(virtual)[:3, :3]
            nearest_roll = min(rolls, key=lambda roll: np.linalg.norm(rolls[roll][:3, :3]-current_rotation)) if rolls else None
            for roll, physical in rolls.items():
                if roll == nearest_roll:
                    continue
                physical[:3, 3] = c.physical_from_virtual(virtual)[:3, 3]
                pools.append((c.virtual_from_physical(physical), baseline["face"],
                              f"CONFIGURED_ROLL_{roll}", [start]))
                break
            generation = dict(c._statistics)
            for requested, face, family, seeds in pools:
                if len(attempts) >= limit or c._deadline_reached() or report["selected"] != "BASELINE":
                    break
                proposals = []
                if family == "SAME_CONTACT":
                    proposals.extend((q, "VERIFIED_J6_PERIODIC_ENDPOINT") for q in periodic_j6_candidates(c.robot, old_q))
                for seed in seeds:
                    if c._deadline_reached():
                        break
                    # Small deterministic seed set; never call random multistart here.
                    with c._budget_scope(c._limit(stop, perf_counter()+1.)):
                        solved = solve_local(c, requested, seed)
                    report["candidates"].append(dict(kind="IK_SEED", family=family,
                        seed_q_rad=np.asarray(seed).tolist(), success=solved.success, reason=solved.message))
                    if solved.success:
                        proposals.append((solved.q, "CURRENT_SEED_IK"))
                proposals.sort(key=lambda item: endpoint_cost(item[0], start, velocity))
                for q, origin in proposals:
                    if len(attempts) >= limit or c._deadline_reached():
                        break
                    record = dict(kind=origin, family=family, q_rad=np.asarray(q).tolist(),
                        endpoint_cost=endpoint_cost(q, start, velocity), status="SCREENING")
                    report["candidates"].append(record)
                    if any(np.max(np.abs(q-prior)) < 1e-5 for prior in seen):
                        record["status"] = "SAME_RAW_JOINT_STATE"
                        continue
                    seen.append(q.copy())
                    if family == "SAME_CONTACT" and endpoint_cost(q, start, velocity) >= endpoint_cost(old_q, start, velocity):
                        record["status"] = "ENDPOINT_HEURISTIC_NOT_BETTER"
                        continue
                    try:
                        c._contact_selection(q, target, face, scene.policy.data["suction"])
                        failure = c._state_failure(q, scene.all_obstacles, target_contact=target, stage="contact_endpoint")
                    except ValueError as exc:
                        failure = {"reason": "CONTACT_CUP_GEOMETRY_INVALID", "detail": str(exc)}
                    if failure:
                        record.update(status="ENDPOINT_REJECTED", failure=failure)
                        continue
                    # Separate generation accounting before plan() resets per-plan counters.
                    for key in ("ik_calls", "ik_seeds_attempted", "ik_iterations_consumed", "trajectory_ik_wall_seconds"):
                        report["generation_statistics"][key] = report["generation_statistics"].get(key, 0) + c._statistics[key]-generation[key]
                    identity = hashlib.sha256(repr((target.name, family, requested.tolist(), q.tolist())).encode()).hexdigest()
                    c.candidate_slice_s = min(8., max(0., stop-perf_counter()))
                    attempt_start = perf_counter()
                    # No history_hint: all approach, extraction, loaded, receiver,
                    # release and departure stages use the ordinary connector.
                    result = c.plan(target=target, face=face, requested_virtual_contact=requested,
                        grasp_candidates=[{"q_rad": q, "candidate_id": identity}], home_q=start,
                        all_obstacles=scene.all_obstacles, receiver=scene.receiver,
                        support_names=tuple(scene.support_graph.supported_by[target.name]),
                        suction=scene.policy.data["suction"], seed=int(identity[:8], 16),
                        optional_quality=True)
                    generation = dict(c._statistics)
                    item = dict(source="WRIST_QUALITY", face=face, roll_deg=None, candidate_id=identity,
                        strict_grasp_candidates=[{"q_rad": q.tolist(), "candidate_id": identity}],
                        ik_stream=None, failure_reason=None if result.success else (result.failure or {}).get("reason"),
                        complete_trajectory=result.success, path_connection_attempts=result.statistics.get("connection_attempts", 0),
                        trajectory_search={"statistics": result.statistics, "attempts": list(result.attempts), "failure": result.failure},
                        elapsed_s=perf_counter()-attempt_start)
                    attempts.append(item)
                    record.update(status="COMPLETE" if result.success else "TASK_FAILED", failure=result.failure)
                    if result.success:
                        after = c._optional_quality(result.segment["path"], stop)
                        record["complete_quality"] = after
                        # Whole-task guards prevent merely moving wrist travel to payload transit.
                        if quality_improves(before, after):
                            selected = deepcopy(result.segment)
                            report.update(selected=identity, after=after)
                            break
                        record["status"] = "COMPLETE_NOT_BETTER_OR_SCORE_UNKNOWN"
            for key in ("ik_calls", "ik_seeds_attempted", "ik_iterations_consumed", "trajectory_ik_wall_seconds"):
                report["generation_statistics"][key] = report["generation_statistics"].get(key, 0) + c._statistics[key]-generation[key]
            report["status"] = "ADOPTED" if report["selected"] != "BASELINE" else "RETAINED_COMPLETE_BASELINE"
    except (ValueError, RuntimeError) as exc:
        report.update(status="RETAINED_COMPLETE_BASELINE", selected="BASELINE",
                      optional_exception=type(exc).__name__, detail=str(exc))
        selected = deepcopy(baseline)
    finally:
        # A failed optional plan must not erase the earlier request's D record.
        c._completed_tasks.update(saved_proofs)
        c._statistics = saved_stats
        if saved_slice is None:
            if hasattr(c, "candidate_slice_s"):
                del c.candidate_slice_s
        else:
            c.candidate_slice_s = saved_slice
        report.update(elapsed_s=perf_counter()-started, attempts_count=len(attempts))
    return selected, attempts, report
