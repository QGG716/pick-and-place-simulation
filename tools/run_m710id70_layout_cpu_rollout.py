"""Plan every carton sequentially on CPU using the current layout policy.

The rollout advances only from validated planned terminal states.  It is not a
replacement for Isaac rigid-body execution: every derived state carries a
distinct CPU-planned schema/source and cannot be presented as measured state.
Successful cartons are assumed handed off before the next planning request so
receiver occupancy is not fabricated from an unmodelled belt simulation.
"""

from __future__ import annotations

import argparse
import copy
import json
import platform
from pathlib import Path
import sys
from time import perf_counter
from typing import Any, Mapping

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (  # noqa: E402
    _build_automatic_trajectory_connector,
    build_verified_motion_input,
    load_layout_motion_policy,
    motion_implementation_identity,
    run_layout_single_carton_audit,
    write_layout_single_carton_audit,
)
from unloading_sim.serial_unloading import (  # noqa: E402
    CPU_PLANNED_ROLLOUT_SOURCE,
    PLANNED_MOTION_STATE_SCHEMA,
    apply_planned_motion_state,
)
from unloading_sim.unloading_sequence import RowSequencePolicy, RowUnloadingState  # noqa: E402
from unloading_sim.workcell_layout import canonical_digest  # noqa: E402


SUMMARY_SCHEMA = "m710id70_layout_cpu_planned_rollout_v1"


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def _quaternion_wxyz(rotation: np.ndarray) -> list[float]:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("rotation must be 3x3")
    # The eigensystem form is stable for both identity and near-180-degree
    # rotations and avoids branch-specific cancellation.
    k = np.array([
        [matrix[0, 0] - matrix[1, 1] - matrix[2, 2], matrix[1, 0] + matrix[0, 1],
         matrix[2, 0] + matrix[0, 2], matrix[2, 1] - matrix[1, 2]],
        [matrix[1, 0] + matrix[0, 1], matrix[1, 1] - matrix[0, 0] - matrix[2, 2],
         matrix[2, 1] + matrix[1, 2], matrix[0, 2] - matrix[2, 0]],
        [matrix[2, 0] + matrix[0, 2], matrix[2, 1] + matrix[1, 2],
         matrix[2, 2] - matrix[0, 0] - matrix[1, 1], matrix[1, 0] - matrix[0, 1]],
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0],
         matrix[1, 0] - matrix[0, 1], matrix.trace()],
    ]) / 3.0
    values, vectors = np.linalg.eigh(k)
    quaternion_xyzw = vectors[:, int(np.argmax(values))]
    if quaternion_xyzw[3] < 0.0:
        quaternion_xyzw *= -1.0
    return [float(quaternion_xyzw[3]), *(float(item) for item in quaternion_xyzw[:3])]


def _planned_state(scene, target: str, q_final: np.ndarray,
                   completed: list[str], run_id: str) -> dict[str, Any]:
    if target not in {box.name for box in scene.cartons}:
        raise ValueError("selected target is absent from the active CPU scene")
    completed_next = [*completed, target]
    return {
        "schema": PLANNED_MOTION_STATE_SCHEMA,
        "source": CPU_PLANNED_ROLLOUT_SOURCE,
        "world_session_id": run_id,
        "q_rad": np.asarray(q_final, dtype=float).tolist(),
        "joint_names": list(scene.snapshot["robot"]["joint_names"]),
        "attached": False,
        "cartons": [
            {
                "name": box.name,
                "position_m": box.center.tolist(),
                "orientation_wxyz": _quaternion_wxyz(box.rotation),
                "linear_velocity_m_s": [0.0, 0.0, 0.0],
                "angular_velocity_rad_s": [0.0, 0.0, 0.0],
            }
            for box in scene.cartons
            if box.name != target
        ],
        "completed_carton_ids": sorted(completed_next),
        "handed_off_ids": sorted(completed_next),
        "handoff_assumption": "PLANNED_RECEIVER_CLEAR_BEFORE_NEXT_REQUEST_NOT_PHYSICS_VERIFIED",
    }


def _verified_result(path: Path, scene, policy) -> dict[str, Any]:
    result = json.loads(path.read_text(encoding="utf-8"))
    identity = copy.deepcopy(result)
    recorded = identity.pop("evidence_fingerprint", None)
    if recorded != canonical_digest(identity):
        raise ValueError(f"cached result fingerprint mismatch: {path}")
    if result.get("policy_fingerprint") != policy.policy_fingerprint:
        raise ValueError(f"cached result policy mismatch: {path}")
    if result.get("scene_fingerprint") != scene.snapshot["scene_fingerprint"]:
        raise ValueError(f"cached result scene mismatch: {path}")
    if result.get("implementation_identity") != motion_implementation_identity(ROOT):
        raise ValueError(f"cached result implementation mismatch: {path}")
    return result


def _compact(index: int, result: Mapping[str, Any], *, reused: bool) -> dict[str, Any]:
    segment = result.get("selected_trajectory_segment")
    statistics = result.get("statistics", {})
    performance = result.get("planning_performance", {})
    tasks = result.get("tasks", [])
    attempted_targets = [
        str(task.get("task_id"))
        for task in tasks
        if isinstance(task, Mapping) and task.get("attempts")
    ]
    return {
        "segment_index": index,
        "status": result.get("complete_trajectory_status"),
        "failure_reason": result.get("complete_trajectory_failure_reason"),
        "target": (
            segment.get("target")
            if isinstance(segment, Mapping)
            else attempted_targets[-1] if attempted_targets else None
        ),
        "attempted_targets": attempted_targets,
        "face": None if not isinstance(segment, Mapping) else segment.get("face"),
        "receiver": None if not isinstance(segment, Mapping) else segment.get("place", {}).get("receiver"),
        "placement_family": None if not isinstance(segment, Mapping) else segment.get("place", {}).get("placement_family"),
        "path_state_count": 0 if not isinstance(segment, Mapping) else len(segment.get("path", [])),
        "planning_wall_seconds": performance.get("planning_total_wall_seconds"),
        "candidate_pose_attempts": statistics.get("candidate_pose_attempts"),
        "ik_seeds_attempted": statistics.get("ik_seeds_attempted"),
        "path_connection_attempts": statistics.get("path_connection_attempts"),
        "collision_state_validations": statistics.get("trajectory_state_validations"),
        "collision_edge_validation_calls": statistics.get("trajectory_edge_validation_calls"),
        "task_failure_counts": statistics.get("task_failure_counts", {}),
        "candidate_failure_counts": statistics.get("candidate_failure_counts", {}),
        "scene_fingerprint": result.get("scene_fingerprint"),
        "evidence_fingerprint": result.get("evidence_fingerprint"),
        "reused": reused,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=ROOT / "configs/validation/m710id70_layout_v1_single_carton.yaml",
    )
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "outputs/m710id70_layout_cpu_rollout",
    )
    parser.add_argument("--maximum-cartons", type=int, default=40)
    args = parser.parse_args(argv)
    if args.maximum_cartons < 1:
        raise ValueError("--maximum-cartons must be positive")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    policy = load_layout_motion_policy(args.config)
    scene = build_verified_motion_input(policy)
    strategy = policy.data.get("search_strategy", {})
    row_state = RowUnloadingState(RowSequencePolicy(
        row_height_fraction=float(strategy.get("row_height_fraction", 0.05)),
        actual_cost_weight=float(strategy.get("actual_cost_weight", 0.15)),
    ))
    row_state.rank(scene.cartons, support_graph=scene.support_graph)
    connector_build = _build_automatic_trajectory_connector(
        scene, policy.layout_validation.layout.robot()
    )
    if connector_build.connector is None:
        raise RuntimeError(f"exact CPU connector unavailable: {connector_build.failure_reason}")
    connector = connector_build.connector
    started = perf_counter()
    run_id = "cpu-planned-rollout:" + policy.policy_fingerprint[:16]
    completed: list[str] = []
    records: list[dict[str, Any]] = []

    for index in range(args.maximum_cartons):
        remaining = (
            len(scene.cartons) if scene.remaining_stack_names is None
            else len(scene.remaining_stack_names)
        )
        if remaining == 0:
            break
        segment_dir = output / "segments" / f"segment_{index:03d}"
        motion_path = segment_dir / "motion.json"
        reused = motion_path.is_file()
        print(json.dumps({
            "event": "segment_start", "segment": index, "completed": len(completed),
            "remaining": remaining, "reused": reused,
        }), flush=True)
        if reused:
            result = _verified_result(motion_path, scene, policy)
        else:
            segment_dir.mkdir(parents=True, exist_ok=False)
            segment_started = perf_counter()
            with (segment_dir / "planning_progress.jsonl").open("w", encoding="utf-8") as stream:
                def progress(item: Mapping[str, Any]) -> None:
                    record = {"elapsed_s": perf_counter() - segment_started, **dict(item)}
                    stream.write(json.dumps(record, default=str) + "\n")
                    stream.flush()
                result = run_layout_single_carton_audit(
                    policy,
                    project_root=ROOT,
                    trajectory_connector=connector,
                    progress_callback=progress,
                    motion_input=scene,
                    row_state=row_state,
                )
            write_layout_single_carton_audit(result, motion_path)

        record = _compact(index, result, reused=reused)
        records.append(record)
        if result.get("complete_trajectory_status") != "PASS":
            timings = [float(item["planning_wall_seconds"]) for item in records
                       if item.get("planning_wall_seconds") is not None]
            summary = {
                "schema": SUMMARY_SCHEMA,
                "status": "BLOCKED",
                "state_source": CPU_PLANNED_ROLLOUT_SOURCE,
                "physical_execution_performed": False,
                "gpu_or_isaac_used": False,
                "platform": platform.platform(),
                "python": sys.version,
                "layout_id": scene.snapshot["layout_id"],
                "layout_fingerprint": scene.snapshot["layout_fingerprint"],
                "policy_fingerprint": policy.policy_fingerprint,
                "initial_carton_count": 40,
                "planned_carton_count": len(completed),
                "remaining_carton_count": remaining,
                "completed_carton_ids": completed,
                "blocking_segment": record,
                "segments": records,
                "planning_wall_seconds_sum": sum(timings),
                "planning_wall_seconds_mean": sum(timings) / len(timings),
                "current_invocation_wall_seconds": perf_counter() - started,
                "handoff_assumption": "PLANNED_RECEIVER_CLEAR_BEFORE_NEXT_REQUEST_NOT_PHYSICS_VERIFIED",
                "claims": {
                    "complete_cpu_geometric_grasp_place_paths": False,
                    "physical_dynamics_verified": False,
                    "safe_to_promote_directly_to_machine": False,
                    "next_step": "RESOLVE_BLOCKING_CPU_PATH_BEFORE_OPTIONAL_ISAAC_BATCH",
                },
            }
            _write_json(output / "summary.json", summary)
            print(json.dumps({"event": "rollout_blocked", **record}), flush=True)
            return 2

        segment = result["selected_trajectory_segment"]
        target = str(segment["target"])
        q_final = np.asarray(segment["path"][-1], dtype=float)
        state = _planned_state(scene, target, q_final, completed, run_id)
        _write_json(segment_dir / "planned_terminal_state.json", state)
        scene = apply_planned_motion_state(scene, state, row_state=row_state)
        completed.append(target)
        print(json.dumps({
            "event": "segment_complete", "segment": index, "target": target,
            "completed": len(completed), "remaining": len(scene.remaining_stack_names or ()),
            "planning_wall_seconds": record["planning_wall_seconds"],
        }), flush=True)

        partial = {
            "schema": SUMMARY_SCHEMA,
            "status": "RUNNING",
            "state_source": CPU_PLANNED_ROLLOUT_SOURCE,
            "physical_execution_performed": False,
            "initial_carton_count": 40,
            "planned_carton_count": len(completed),
            "remaining_carton_count": len(scene.remaining_stack_names or ()),
            "completed_carton_ids": completed,
            "segments": records,
            "current_invocation_wall_seconds": perf_counter() - started,
        }
        _write_json(output / "summary.json", partial)

    remaining = len(scene.remaining_stack_names or ())
    complete = remaining == 0
    timings = [float(item["planning_wall_seconds"]) for item in records
               if item.get("planning_wall_seconds") is not None]
    summary = {
        "schema": SUMMARY_SCHEMA,
        "status": "PASS" if complete else "BUDGET_EXHAUSTED",
        "state_source": CPU_PLANNED_ROLLOUT_SOURCE,
        "physical_execution_performed": False,
        "gpu_or_isaac_used": False,
        "platform": platform.platform(),
        "python": sys.version,
        "layout_id": scene.snapshot["layout_id"],
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "policy_fingerprint": policy.policy_fingerprint,
        "initial_carton_count": 40,
        "planned_carton_count": len(completed),
        "remaining_carton_count": remaining,
        "completed_carton_ids": completed,
        "segments": records,
        "planning_wall_seconds_sum": sum(timings),
        "planning_wall_seconds_mean": None if not timings else sum(timings) / len(timings),
        "current_invocation_wall_seconds": perf_counter() - started,
        "handoff_assumption": "PLANNED_RECEIVER_CLEAR_BEFORE_NEXT_REQUEST_NOT_PHYSICS_VERIFIED",
        "claims": {
            "complete_cpu_geometric_grasp_place_paths": complete,
            "physical_dynamics_verified": False,
            "safe_to_promote_directly_to_machine": False,
            "next_step_if_pass": "OPTIONAL_ISAAC_REPLAY_FROM_EACH_SAVED_MOTION_SEGMENT",
        },
    }
    _write_json(output / "summary.json", summary)
    print(json.dumps({
        "event": "rollout_finished", "status": summary["status"],
        "planned": len(completed), "remaining": remaining,
        "current_invocation_wall_seconds": summary["current_invocation_wall_seconds"],
        "output": str(output),
    }), flush=True)
    return 0 if complete else 3


if __name__ == "__main__":
    raise SystemExit(main())
