"""Collision-checked optimization for reusable online trajectory libraries."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from time import perf_counter
from typing import Sequence

import numpy as np

from .grasp import _carried_box_state_valid, _in_trailer_orientation_valid
from .online_unload import docked_robot_and_scene, remove_carton
from .planner import RRTConnectPlanner
from .scene import load_scene_config


def joint_path_length(path: Sequence[np.ndarray]) -> float:
    if len(path) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(np.asarray(path, dtype=float), axis=0), axis=1).sum())


def optimize_cached_plan(
    manifest: dict,
    attempts: int = 1200,
    seed: int = 11,
    output_resolution: float = 0.04,
) -> tuple[dict, list[dict]]:
    """Shortcut carried paths while rechecking robot, carton, and tilt safety."""
    scene, cfg = load_scene_config(manifest["config"])
    planning = cfg.get("planning", {})
    optimized = copy.deepcopy(manifest)
    statistics: list[dict] = []

    for segment_index, segment in enumerate(optimized.get("segments", [])):
        dock = np.asarray(segment["amr_dock_position"], dtype=float)
        robot, docked_scene, _ = docked_robot_and_scene(scene, cfg, dock)
        carton = docked_scene.carton(segment["target"])
        path = [np.asarray(q, dtype=float) for q in segment["path"]]
        grasp_index = int(segment["grasp_index"])
        carried_path = path[grasp_index:]
        grasp_q = carried_path[0]
        carton_from_tool = np.linalg.inv(robot.fk(grasp_q)) @ carton.world_from_local
        place_surface = segment.get("place_surface")
        carried_obstacles = docked_scene.obstacles_without({carton.name, place_surface})
        robot_obstacles = docked_scene.obstacles_without({carton.name})
        max_tilt = planning.get("max_in_trailer_carton_tilt_deg")

        def orientation_valid(box) -> bool:
            clearance_zone_max_x = planning.get("clearance_zone_max_x")
            if clearance_zone_max_x is not None and box.center[0] <= float(clearance_zone_max_x):
                return True
            return _in_trailer_orientation_valid(carton, box, docked_scene, max_tilt)

        def valid(q: np.ndarray) -> bool:
            return robot.is_collision_free(q, robot_obstacles) and _carried_box_state_valid(
                robot,
                carton,
                grasp_q,
                q,
                carried_obstacles,
                box_margin=float(planning.get("carried_box_clearance", 0.01)),
                carton_from_tool=carton_from_tool,
                carton_contact_tolerance=float(planning.get("carton_contact_tolerance_m", 0.005)),
                orientation_valid=orientation_valid,
            )

        planner = RRTConnectPlanner(
            robot.joint_limits[:, 0],
            robot.joint_limits[:, 1],
            valid,
            edge_resolution=float(planning.get("rrt_edge_resolution", 0.04)),
            rng=np.random.default_rng(int(seed) + segment_index),
        )
        before_length = joint_path_length(carried_path)
        shortcut_nodes = planner.shortcut(carried_path, attempts=int(attempts))
        dense_carried = planner.densify(shortcut_nodes, resolution=float(output_resolution))
        new_path = path[: grasp_index + 1] + dense_carried[1:]
        after_length = joint_path_length(dense_carried)
        segment["path"] = [q.tolist() for q in new_path]
        segment["base_path"] = [robot.base_transform[:3, 3].tolist() for _ in new_path]
        segment["release_index"] = len(new_path) - 1
        segment["release_retreat_index"] = len(new_path) - 1
        statistics.append(
            {
                "pick_index": int(segment["pick_index"]),
                "target": segment["target"],
                "before_joint_length_rad": before_length,
                "after_joint_length_rad": after_length,
                "reduction_fraction": 0.0 if before_length == 0.0 else 1.0 - after_length / before_length,
                "shortcut_nodes": len(shortcut_nodes),
            }
        )
        remove_carton(scene, carton.name)

    optimized["trajectory_cache"] = {
        "collision_checked": True,
        "shortcut_attempts": int(attempts),
        "seed": int(seed),
        "statistics": statistics,
    }
    return optimized, statistics


def validate_cached_plan(manifest: dict) -> list[dict]:
    """Validate cached segments in scene order and report online latency."""
    scene, cfg = load_scene_config(manifest["config"])
    planning = cfg.get("planning", {})
    statistics: list[dict] = []
    for segment in manifest.get("segments", []):
        started = perf_counter()
        robot, docked_scene, _ = docked_robot_and_scene(
            scene,
            cfg,
            np.asarray(segment["amr_dock_position"], dtype=float),
        )
        carton = docked_scene.carton(segment["target"])
        path = [np.asarray(q, dtype=float) for q in segment["path"]]
        grasp_index = int(segment["grasp_index"])
        grasp_q = path[grasp_index]
        carton_from_tool = np.linalg.inv(robot.fk(grasp_q)) @ carton.world_from_local
        robot_obstacles = docked_scene.obstacles_without({carton.name})
        carried_obstacles = docked_scene.obstacles_without({carton.name, segment.get("place_surface")})
        max_tilt = planning.get("max_in_trailer_carton_tilt_deg")

        def orientation_valid(box) -> bool:
            clearance_zone_max_x = planning.get("clearance_zone_max_x")
            if clearance_zone_max_x is not None and box.center[0] <= float(clearance_zone_max_x):
                return True
            return _in_trailer_orientation_valid(carton, box, docked_scene, max_tilt)
        valid = True
        invalid_index = None
        for index, q in enumerate(path):
            if not robot.is_collision_free(q, robot_obstacles):
                valid, invalid_index = False, index
                break
            if index >= grasp_index and not _carried_box_state_valid(
                robot,
                carton,
                grasp_q,
                q,
                carried_obstacles,
                box_margin=float(planning.get("carried_box_clearance", 0.01)),
                carton_from_tool=carton_from_tool,
                carton_contact_tolerance=float(planning.get("carton_contact_tolerance_m", 0.005)),
                orientation_valid=orientation_valid,
            ):
                valid, invalid_index = False, index
                break
        statistics.append(
            {
                "pick_index": int(segment["pick_index"]),
                "target": segment["target"],
                "valid": valid,
                "invalid_index": invalid_index,
                "validation_seconds": perf_counter() - started,
                "waypoints": len(path),
                "estimated_execution_seconds": (
                    max(0, len(path) - 1) * float(planning.get("trajectory_waypoint_period_seconds", 0.02))
                    + np.sqrt(2.0 * float(segment.get("free_fall_height_m", 0.0)) / 9.81)
                ),
            }
        )
        if not valid:
            break
        remove_carton(scene, carton.name)
    return statistics


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Optimize a planned trajectory for low-latency online reuse")
    parser.add_argument("--plan", required=True)
    parser.add_argument("--output")
    parser.add_argument("--attempts", type=int, default=1200)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)

    start = perf_counter()
    manifest = json.loads(Path(args.plan).read_text(encoding="utf-8"))
    if args.validate_only:
        statistics = validate_cached_plan(manifest)
        print(json.dumps({"valid": bool(statistics) and all(item["valid"] for item in statistics), "seconds": perf_counter() - start, "segments": statistics}, indent=2))
        return
    if not args.output:
        parser.error("--output is required unless --validate-only is used")
    optimized, statistics = optimize_cached_plan(manifest, attempts=args.attempts)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(optimized, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output), "seconds": perf_counter() - start, "segments": statistics}, indent=2))


if __name__ == "__main__":
    main()
