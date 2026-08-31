"""Replace unsafe cached release reversals with task-space disengage/lift paths.

The operation preserves every segment start and end configuration, so a cached
continuous unload plan remains continuous.  Only the path after ``release_index``
is replaced.  The short normal disengagement is derived from cup compression and
solver capture tolerance; the following lift uses the configured post-release
clearance and keeps the release wrist orientation fixed.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from unloading_sim.grasp import (
    adaptive_post_release_normal_disengage_distance,
    post_release_escape_target,
)
from unloading_sim.ik import solve_ik_multistart
from unloading_sim.online_unload import docked_robot_and_scene
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.scene import load_scene_config
from unloading_sim.timing import time_parameterize_segments


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", default=73, type=int)
    return parser


def _task_space_translation(
    robot,
    start_q: np.ndarray,
    target_position: np.ndarray,
    obstacles,
    rng: np.random.Generator,
    *,
    position_step_m: float,
) -> list[np.ndarray]:
    start_pose = robot.fk(start_q)
    distance = float(np.linalg.norm(target_position - start_pose[:3, 3]))
    steps = max(1, int(np.ceil(distance / position_step_m)))
    path = [np.asarray(start_q, dtype=float).copy()]
    for index in range(1, steps + 1):
        target = start_pose.copy()
        fraction = index / steps
        target[:3, 3] = (
            (1.0 - fraction) * start_pose[:3, 3]
            + fraction * np.asarray(target_position, dtype=float)
        )
        result = solve_ik_multistart(
            robot,
            target,
            seeds=[path[-1]],
            obstacles=obstacles,
            random_restarts=2,
            rng=rng,
            max_iterations=320,
            position_tolerance=0.008,
            orientation_tolerance=0.04,
        )
        if not result.success:
            raise RuntimeError(
                f"task-space release retreat IK failed at step {index}/{steps}: "
                f"position_error={result.position_error:.6f}, "
                f"orientation_error={result.orientation_error:.6f}"
            )
        path.append(np.asarray(result.q, dtype=float).copy())
    return path


def repair_plan(plan: dict, *, seed: int) -> tuple[dict, list[dict]]:
    repaired = copy.deepcopy(plan)
    scene, cfg = load_scene_config(repaired["config"])
    planning = dict(cfg.get("planning", {}))
    validation = cfg.get("simulation_validation", {})
    conveyor_cfg = validation.get("conveyor", {})
    planning.setdefault(
        "post_release_surface_directions_world",
        conveyor_cfg.get("surface_directions_world", {}),
    )
    cup_compression = float(validation.get("vacuum_cup_compression_m", 0.0))
    capture_tolerance = float(validation.get("vacuum_surface_gripper_capture_tolerance_m", 0.0))
    collision_clearance = float(planning.get("carried_box_clearance", 0.01))
    configured_normal_distance = cup_compression + capture_tolerance + collision_clearance
    lift_distance = float(planning.get("post_release_retreat_distance_m", 0.30))
    if configured_normal_distance <= 0.0 or lift_distance <= 0.0:
        raise ValueError("release disengagement and lift distances must be positive")

    audit: list[dict] = []
    for segment_index, segment in enumerate(repaired.get("segments", [])):
        path = [np.asarray(q, dtype=float) for q in segment["path"]]
        release_index = int(segment["release_index"])
        if not (0 <= release_index < len(path) - 1):
            raise ValueError(f"segment {segment_index} has no post-release path to repair")
        original_endpoint = path[-1].copy()
        robot, docked_scene, _ = docked_robot_and_scene(
            scene,
            cfg,
            np.asarray(segment["amr_dock_position"], dtype=float),
        )
        obstacles = docked_scene.obstacles_without({str(segment["target"])})
        rng = np.random.default_rng(int(seed) + segment_index)
        release_q = path[release_index]
        normal_distance, normal_metadata = (
            adaptive_post_release_normal_disengage_distance(
                robot,
                release_q,
                planning,
                configured_normal_distance,
            )
        )
        release_pose = robot.fk(release_q)
        normal_target = (
            release_pose[:3, 3] - normal_distance * release_pose[:3, 2]
        )
        normal_path = _task_space_translation(
            robot,
            release_q,
            normal_target,
            obstacles,
            rng,
            position_step_m=float(
                planning.get("post_release_normal_position_step_m", 0.01)
            ),
        )
        escape_target, escape_metadata = post_release_escape_target(
            robot,
            normal_path[-1],
            str(segment.get("place_surface")),
            planning,
            normal_disengage_distance_m=0.0,
            vertical_lift_distance_m=lift_distance,
        )
        escape_path = _task_space_translation(
            robot,
            normal_path[-1],
            escape_target,
            obstacles,
            rng,
            position_step_m=0.03,
        )
        escape_path = normal_path + escape_path[1:]

        connector_planner = RRTConnectPlanner(
            robot.joint_limits[:, 0],
            robot.joint_limits[:, 1],
            lambda q: robot.is_collision_free(q, obstacles),
            step_size=float(planning.get("rrt_step_size", 0.22)),
            edge_resolution=float(planning.get("rrt_edge_resolution", 0.065)),
            max_iterations=int(planning.get("rrt_max_iterations", 4500)),
            goal_bias=float(planning.get("rrt_goal_bias", 0.15)),
            rng=rng,
        )
        if connector_planner.edge_valid(escape_path[-1], original_endpoint):
            connector_nodes = [escape_path[-1], original_endpoint]
            connector_kind = "direct"
        else:
            connector = connector_planner.plan(
                escape_path[-1],
                original_endpoint,
                time_limit_seconds=float(
                    planning.get("trajectory_repair_time_limit_seconds", 60.0)
                ),
            )
            if not connector.success:
                raise RuntimeError(
                    f"segment {segment_index} release-lift connector failed: {connector.message}"
                )
            connector_nodes = connector.path
            connector_kind = "rrt_connect"
        connector_path = connector_planner.densify(
            connector_nodes,
            resolution=float(planning.get("output_resolution", 0.035)),
        )

        replacement = escape_path
        release_retreat_index = release_index + len(replacement) - 1
        new_path = path[:release_index] + replacement + connector_path[1:]
        segment["path"] = [q.tolist() for q in new_path]
        base = list(segment["base_path"][0])
        segment["base_path"] = [base for _ in new_path]
        segment["release_retreat_index"] = release_retreat_index
        segment["post_release_retreat"] = {
            **escape_metadata,
            "model": "normal_disengage_then_vertical_and_opposed_conveyor_v3",
            "normal_disengage_distance_m": normal_distance,
            "normal_disengage_derivation": normal_metadata,
            "normal_disengage_first": True,
            "fixed_wrist_orientation": True,
            "connector": connector_kind,
            "preserved_segment_endpoint": True,
        }
        audit.append(
            {
                "segment_index": segment_index,
                "target": segment["target"],
                "source_waypoint_count": len(path),
                "repaired_waypoint_count": len(new_path),
                **segment["post_release_retreat"],
            }
        )

    repaired["post_release_retreat_repair"] = {
        "model": "normal_vertical_and_opposed_conveyor_diagonal_v2",
        "seed": int(seed),
        "segments": audit,
    }
    repaired["execution_summary"] = time_parameterize_segments(
        repaired.get("segments", []), cfg
    )
    return repaired, audit


def main() -> None:
    args = _parser().parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    repaired, audit = repair_plan(plan, seed=args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(repaired, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {"output": str(args.output), "segments": len(audit)},
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
