"""Find exact-valid unloaded working homes near the highest-row centre carton.

This CPU diagnostic changes no layout or active configuration.  It emits
deterministic candidates for explicit adoption by a subsequent run, together
with checked local approaches from each candidate's own joint configuration.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.ik import iter_ik_solutions
from unloading_sim.layout_single_carton import (
    _build_automatic_trajectory_connector,
    _independent_cup_selection_at_pose,
    _scheduled_contact_poses,
    build_verified_motion_input,
    load_layout_motion_policy,
)
from unloading_sim.unloading_sequence import RowUnloadingState, RowSequencePolicy
from unloading_sim.validation_motion import grasp_seed_configurations


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def find_contact_homes(config: Path, *, project_root: Path, output: Path,
                       standoffs=(0.25, 0.20), task_limit=1, poses_per_face=2,
                       seeds_per_pose=16, solutions_per_pose=2, seed=7107025):
    if min(task_limit, poses_per_face, seeds_per_pose, solutions_per_pose) <= 0:
        raise ValueError("home search budgets must be positive")
    if not standoffs or any(not np.isfinite(d) or d <= 0 for d in standoffs):
        raise ValueError("home standoffs must be finite positive distances")
    started = time.perf_counter()
    policy = load_layout_motion_policy(config)
    scene = build_verified_motion_input(policy, project_root)
    layout = policy.layout_validation.layout
    built = _build_automatic_trajectory_connector(scene, layout.robot())
    if built.connector is None:
        raise RuntimeError(f"exact motion backend unavailable: {built.evidence}")
    connector = built.connector
    robot = connector.robot
    selection = RowUnloadingState(RowSequencePolicy(row_height_fraction=float(
        policy.data.get("search_strategy", {}).get("row_height_fraction", 0.05)
    ))).rank(scene.cartons, support_graph=scene.support_graph)
    if not selection.candidates:
        raise RuntimeError(f"no currently removable highest-row carton: {selection.as_dict()}")
    ik = policy.data["ik"]
    statistics = Counter()
    attempts = []
    candidates = []
    source_files = [Path(__file__).resolve(),
                    *(Path(sys.modules[name].__file__).resolve() for name in (
                        "unloading_sim.pinocchio_backend", "unloading_sim.layout_trajectory",
                        "unloading_sim.layout_single_carton", "unloading_sim.tool_geometry",
                        "unloading_sim.unloading_sequence", "unloading_sim.ik")),
                    project_root / "assets/grippers/shanghai_wantai_three_zone/geometry_coverage.json",
                    config.resolve()]
    identity = {str(path): _hash(path) for path in source_files}

    def write_status(status):
        result = {
            "schema": "m710_contact_home_search_v1", "status": status,
            "layout_fingerprint": layout.layout_fingerprint,
            "snapshot_fingerprint": scene.snapshot["scene_fingerprint"],
            "snapshot_verification": dict(scene.snapshot_verification),
            "snapshot_consistency": dict(scene.snapshot_consistency),
            "scene_carton_names": [box.name for box in scene.cartons],
            "row_selection": selection.as_dict(),
            "effective_collision_policy": policy.layout_validation.data.get("collision_policy"),
            "source_sha256": identity,
            "runtime": {"python": platform.python_version(), "numpy": np.__version__},
            "rng_seed": seed,
            "scope": "unloaded_initialization_home_candidates_no_layout_or_configuration_mutation",
            "budget": {"tasks": task_limit, "poses_per_face": poses_per_face,
                       "seeds_per_pose": seeds_per_pose, "solutions_per_pose": solutions_per_pose,
                       "standoffs_m": list(standoffs)},
            "rank_policy": "local_pregrasp_and_contact_feasible_first_then_joint_margin_and_normalized_local_joint_travel",
            "candidates": sorted(candidates, key=lambda item: (not item["local_contact_path_valid"],
                               not item["local_pregrasp_path_valid"], item["score"])),
            "attempts": attempts, "statistics": dict(statistics),
            "elapsed_seconds": time.perf_counter() - started,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result

    for task_index, ranked in enumerate(selection.candidates[:task_limit]):
        target = ranked.carton
        faces = tuple(face for face in ranked.available_faces if face in {"front", "top"})
        counts = Counter()
        for face, (roll, physical, variant) in _scheduled_contact_poses(scene, target, faces):
            if counts[face] >= poses_per_face:
                continue
            counts[face] += 1
            prospective_selection, _ = _independent_cup_selection_at_pose(
                physical, target, face, policy.data["suction"],
                float(policy.data["state_validity"]["contact_tolerance_m"]),
                pose_source="prospective_contact_for_working_home_search",
            )
            if prospective_selection is None:
                attempts.append({"target": target.name, "face": face, "roll_deg": roll,
                                 "reason": "NO_PROSPECTIVE_FULL_RING_CONTACT"})
                continue
            for standoff_index, standoff in enumerate(standoffs):
                home_physical = physical.copy()
                home_physical[:3, 3] -= physical[:3, 2] * standoff
                home_virtual = connector.virtual_from_physical(home_physical)
                attempt_seed = seed + task_index * 100003 + len(attempts) * 1009
                failures = Counter()

                def valid(q):
                    statistics["home_exact_state_validations"] += 1
                    failure = connector.validate_unloaded_state(q, scene.all_obstacles, stage="working_home_search")
                    if failure is not None:
                        failures[str(failure.get("reason", "UNKNOWN"))] += 1
                    return failure is None

                initial = np.asarray(policy.layout_validation.initial_q)
                middle = np.mean(robot.joint_limits, axis=1)
                explicit = grasp_seed_configurations(robot, [initial, middle, robot.clamp(np.zeros(robot.dof))])
                stream = iter_ik_solutions(robot, home_virtual, explicit[:seeds_per_pose],
                    random_restarts=max(0, seeds_per_pose - len(explicit)),
                    rng=np.random.default_rng(attempt_seed), candidate_limit=solutions_per_pose,
                    dedup_tolerance_rad=float(ik["candidate_dedup_tolerance_rad"]),
                    dedup_tolerance_m=float(ik["candidate_dedup_tolerance_m"]),
                    max_iterations=int(ik["max_iterations"]), damping=float(ik["damping"]),
                    max_step=float(ik["max_step_rad"]), position_tolerance=float(ik["position_tolerance_m"]),
                    orientation_tolerance=float(ik["orientation_tolerance_rad"]),
                    orientation_weight=float(ik["orientation_weight"]),
                    collision_check_stride=int(ik["max_iterations"]) + 1, extra_state_valid=valid)
                attempt = {"target": target.name, "face": face, "roll_deg": roll,
                           "standoff_m": standoff, "rng_seed": attempt_seed, "variant": variant,
                           "candidate_indices": []}
                for solution in stream:
                    q = solution.q
                    # Re-evaluate the real endpoint independently of the IK
                    # callback before building any local approach evidence.
                    endpoint_failure = connector.validate_unloaded_state(q, scene.all_obstacles, stage="working_home_final")
                    if endpoint_failure is not None:
                        continue
                    pregrasp = physical.copy()
                    pregrasp[:3, 3] -= physical[:3, 2] * connector.budget.pregrasp_standoff_m
                    pre_path, pre_failure, pre_evidence = connector._cartesian(
                        q, connector.virtual_from_physical(pregrasp), scene.all_obstacles,
                        seed=attempt_seed + 101, stage="working_home_to_pregrasp")
                    contact_path, contact_failure, contact_evidence = [], None, None
                    actual_cups = None
                    if pre_failure is None:
                        contact_path, contact_failure, contact_evidence = connector._cartesian(
                            pre_path[-1], connector.virtual_from_physical(physical), scene.all_obstacles,
                            seed=attempt_seed + 211, target_contact=target, stage="working_home_contact_preview")
                        if contact_failure is None:
                            actual_cups, _ = _independent_cup_selection_at_pose(
                                connector.physical_from_virtual(robot.fk(contact_path[-1])),
                                target, face, policy.data["suction"],
                                float(policy.data["state_validity"]["contact_tolerance_m"]),
                                pose_source="actual_fk_end_of_working_home_contact_preview",
                            )
                            if actual_cups is None:
                                contact_failure = {"reason": "ACTUAL_ENDPOINT_FULL_RING_CONTACT_FAILED"}
                    else:
                        contact_failure = {"reason": "PREGRASP_PATH_FAILED"}
                    joint_margin = np.minimum(q - robot.joint_limits[:, 0], robot.joint_limits[:, 1] - q)
                    normalized_margin = joint_margin / np.diff(robot.joint_limits, axis=1)[:, 0]
                    local_travel = float(np.linalg.norm((pre_path[-1] - q) / np.diff(robot.joint_limits, axis=1)[:, 0]))
                    candidate = {
                        "index": len(candidates), "target": target.name, "face": face, "roll_deg": roll,
                        "row_reason": ranked.as_dict(), "standoff_m": standoff,
                        "home_q_rad": q.tolist(), "requested_virtual_tcp": home_virtual.tolist(),
                        "actual_virtual_tcp": robot.fk(q).tolist(),
                        "strict_fk_position_error_m": float(solution.position_error),
                        "strict_fk_orientation_error_rad": float(solution.orientation_error),
                        "joint_margin_per_joint_rad": joint_margin.tolist(),
                        "minimum_joint_margin_rad": float(np.min(joint_margin)),
                        "jacobian_condition": float(np.linalg.cond(robot.geometric_jacobian(q))),
                        "score": float(1.0 - 2.0 * np.min(normalized_margin) + local_travel),
                        "normalized_local_joint_travel": local_travel,
                        "state_validation": "PASS_UNLOADED_OFFICIAL_MESH_ALL_TOOL_RIGID_SOLIDS_FULL_SCENE",
                        "tool_rigid_box_count": len(layout.robot().tool_collision_obbs(q)),
                        "target_remains_collision_obstacle": True, "attached": False,
                        "local_pregrasp_path_valid": pre_failure is None,
                        "local_pregrasp_path_q_rad": [item.tolist() for item in pre_path],
                        "local_pregrasp_evidence": pre_evidence, "local_pregrasp_failure": pre_failure,
                        "local_contact_path_valid": contact_failure is None,
                        "local_contact_path_q_rad": [item.tolist() for item in contact_path],
                        "local_contact_evidence": contact_evidence, "local_contact_failure": contact_failure,
                        "actual_contact_preview_cups": None if actual_cups is None else actual_cups.to_dict(),
                        "physical_pick_claimed": False,
                    }
                    candidates.append(candidate)
                    attempt["candidate_indices"].append(candidate["index"])
                    write_status("SEARCHING")
                    print(json.dumps({"candidate": candidate["index"], "face": face, "standoff_m": standoff,
                                      "q_rad": q.tolist(), "joint_margin_rad": candidate["minimum_joint_margin_rad"],
                                      "pregrasp_valid": pre_failure is None, "contact_valid": contact_failure is None}), flush=True)
                attempt["ik_stream"] = stream.evidence()
                attempt["state_failures"] = dict(failures)
                attempts.append(attempt)
                statistics["ik_seeds_attempted"] += stream.seed_index
                statistics["ik_iterations_consumed"] += stream.iterations_consumed
                write_status("SEARCHING")
            if all(counts[face] >= poses_per_face for face in faces):
                break
    return write_status("CANDIDATES_FOUND" if candidates else "NO_VALID_HOME_WITHIN_BUDGET")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--standoffs", type=float, nargs="+", default=[0.25, 0.20])
    parser.add_argument("--task-limit", type=int, default=1)
    parser.add_argument("--poses-per-face", type=int, default=2)
    parser.add_argument("--seeds-per-pose", type=int, default=16)
    parser.add_argument("--solutions-per-pose", type=int, default=2)
    parser.add_argument("--seed", type=int, default=7107025)
    args = parser.parse_args()
    root = args.project_root.resolve()
    result = find_contact_homes(args.config or root / "configs/validation/m710id70_layout_v1_single_carton.yaml",
        project_root=root, output=args.output, standoffs=args.standoffs, task_limit=args.task_limit,
        poses_per_face=args.poses_per_face, seeds_per_pose=args.seeds_per_pose,
        solutions_per_pose=args.solutions_per_pose, seed=args.seed)
    print(json.dumps({"status": result["status"], "candidates": len(result["candidates"]),
                      "elapsed_seconds": result["elapsed_seconds"], "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
