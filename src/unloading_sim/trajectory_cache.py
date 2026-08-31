"""Collision-checked optimization for reusable online trajectory libraries."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from time import perf_counter
from typing import Sequence

import numpy as np

from .grasp import _carried_box_state_valid, _carried_orientation_policy_valid
from .online_unload import docked_robot_and_scene, remove_carton
from .planner import RRTConnectPlanner
from .scene import load_scene_config
from .timing import time_parameterize_segments
from .trajectory import blend_joint_path, simplify_collinear_joint_path


def joint_path_length(path: Sequence[np.ndarray]) -> float:
    if len(path) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(np.asarray(path, dtype=float), axis=0), axis=1).sum())


def repair_invalid_path(
    path: Sequence[np.ndarray],
    planner: RRTConnectPlanner,
    *,
    time_limit_seconds_per_repair: float = 60.0,
) -> tuple[list[np.ndarray], list[dict]]:
    """Replace only invalid waypoint/edge spans with deterministic RRT detours.

    Valid portions of the source trajectory remain untouched.  Each repair is
    anchored by the last accepted state and the first later valid source
    waypoint, so this is not a mandatory spatial channel or phase template.
    """
    points = [np.asarray(q, dtype=float).copy() for q in path]
    if len(points) < 2:
        raise ValueError("repair requires at least two waypoints")
    if not planner.is_state_valid(points[0]) or not planner.is_state_valid(points[-1]):
        raise ValueError("repair path endpoints must be valid")
    if not np.isfinite(time_limit_seconds_per_repair) or time_limit_seconds_per_repair <= 0.0:
        raise ValueError("repair time limit must be finite and positive")

    output = [points[0]]
    repairs: list[dict] = []
    index = 1
    while index < len(points):
        if planner.edge_valid(output[-1], points[index]):
            output.append(points[index])
            index += 1
            continue

        first_replaced = index
        anchor = index
        while anchor < len(points) and not planner.is_state_valid(points[anchor]):
            anchor += 1
        if anchor >= len(points):
            raise RuntimeError("invalid path span has no later valid anchor")
        detour = planner.plan(
            output[-1], points[anchor], time_limit_seconds=time_limit_seconds_per_repair
        )
        if not detour.success:
            raise RuntimeError(
                f"could not repair source path span {first_replaced}:{anchor}: {detour.message}"
            )
        output.extend(np.asarray(q, dtype=float).copy() for q in detour.path[1:])
        repairs.append(
            {
                "first_replaced_index": first_replaced,
                "next_valid_anchor_index": anchor,
                "detour_nodes": len(detour.path),
                "iterations": detour.iterations,
                "message": detour.message,
            }
        )
        index = anchor + 1
    return output, repairs


def optimize_cached_plan(
    manifest: dict,
    attempts: int = 1200,
    seed: int = 11,
    output_resolution: float = 0.04,
    corner_fraction: float = 0.22,
    corner_samples: int = 7,
) -> tuple[dict, list[dict]]:
    """Shortcut and blend paths while rechecking robot, carton, and tilt safety."""
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
        release_index = int(segment.get("release_index", len(path) - 1))
        release_retreat_index = int(segment.get("release_retreat_index", release_index))
        if not (grasp_index <= release_index <= release_retreat_index < len(path)):
            raise ValueError("cached grasp/release indices are inconsistent with the path")
        carried_path = path[grasp_index : release_index + 1]
        post_release_path = path[release_index:]
        grasp_q = carried_path[0]
        carton_from_tool = np.linalg.inv(robot.fk(grasp_q)) @ carton.world_from_local
        place_surface = segment.get("place_surface")
        carried_obstacles = docked_scene.obstacles_without({carton.name, place_surface})
        robot_obstacles = docked_scene.obstacles_without({carton.name})
        def orientation_valid(box) -> bool:
            return _carried_orientation_policy_valid(
                carton,
                box,
                docked_scene,
                planning,
            )

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
        repaired_nodes, repair_audit = repair_invalid_path(
            carried_path,
            planner,
            time_limit_seconds_per_repair=float(
                planning.get("trajectory_repair_time_limit_seconds", 60.0)
            ),
        )
        shortcut_nodes = planner.shortcut(repaired_nodes, attempts=int(attempts))
        pregrasp_nodes = simplify_collinear_joint_path(path[: grasp_index + 1])
        pregrasp_blend = blend_joint_path(
            pregrasp_nodes,
            lambda q: robot.is_collision_free(q, robot_obstacles),
            corner_fraction=corner_fraction,
            samples_per_corner=corner_samples,
            edge_resolution=float(planning.get("rrt_edge_resolution", 0.04)),
        )
        carried_blend = blend_joint_path(
            shortcut_nodes,
            valid,
            corner_fraction=corner_fraction,
            samples_per_corner=corner_samples,
            edge_resolution=float(planning.get("rrt_edge_resolution", 0.04)),
        )
        pregrasp_path = planner.densify(pregrasp_blend.path, resolution=float(output_resolution))
        carried_path = planner.densify(carried_blend.path, resolution=float(output_resolution))
        new_release_index = len(pregrasp_path) + len(carried_path) - 2
        new_path = pregrasp_path + carried_path[1:] + post_release_path[1:]
        after_length = joint_path_length(carried_path)
        segment["path"] = [q.tolist() for q in new_path]
        segment["base_path"] = [robot.base_transform[:3, 3].tolist() for _ in new_path]
        segment["grasp_index"] = len(pregrasp_path) - 1
        segment["release_index"] = new_release_index
        segment["release_retreat_index"] = (
            new_release_index + release_retreat_index - release_index
        )
        segment["corner_blending"] = {
            "pregrasp": pregrasp_blend.audit(),
            "carried": carried_blend.audit(),
        }
        statistics.append(
            {
                "pick_index": int(segment["pick_index"]),
                "target": segment["target"],
                "before_joint_length_rad": before_length,
                "after_joint_length_rad": after_length,
                "reduction_fraction": 0.0 if before_length == 0.0 else 1.0 - after_length / before_length,
                "shortcut_nodes": len(shortcut_nodes),
                "invalid_span_repairs": repair_audit,
                "accepted_blend_corners": pregrasp_blend.accepted_corners + carried_blend.accepted_corners,
                "rejected_blend_corners": pregrasp_blend.rejected_corners + carried_blend.rejected_corners,
            }
        )
        remove_carton(scene, carton.name)

    optimized["trajectory_cache"] = {
        "collision_checked": True,
        "shortcut_attempts": int(attempts),
        "seed": int(seed),
        "statistics": statistics,
    }
    optimized["execution_summary"] = time_parameterize_segments(optimized.get("segments", []), cfg)
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
        release_index = int(segment.get("release_index", len(path) - 1))
        if not (grasp_index <= release_index < len(path)):
            raise ValueError("cached grasp/release indices are inconsistent with the path")
        grasp_q = path[grasp_index]
        carton_from_tool = np.linalg.inv(robot.fk(grasp_q)) @ carton.world_from_local
        robot_obstacles = docked_scene.obstacles_without({carton.name})
        # The release pose remains above the support plane. Keep every
        # conveyor section collision-active so cached validation cannot hide
        # a payload/belt contact that occurs before the release index.
        carried_obstacles = docked_scene.obstacles_without({carton.name})
        def orientation_valid(box) -> bool:
            return _carried_orientation_policy_valid(
                carton,
                box,
                docked_scene,
                planning,
            )
        def state_valid(q: np.ndarray, carried: bool) -> bool:
            if not robot.is_collision_free(q, robot_obstacles):
                return False
            if carried and not _carried_box_state_valid(
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
                return False
            return True

        # Revalidate every interpolated controller edge. A waypoint-only pass
        # is not a continuous collision certificate.
        edge_resolution = float(planning.get("rrt_edge_resolution", 0.04))
        if not np.isfinite(edge_resolution) or edge_resolution <= 0.0:
            raise ValueError("rrt_edge_resolution must be finite and positive")
        valid = state_valid(path[0], carried=grasp_index == 0)
        invalid_index = None if valid else 0
        invalid_edge = None
        invalid_edge_fraction = None
        if valid:
            for edge_index, (start, end) in enumerate(zip(path[:-1], path[1:])):
                sample_count = max(
                    1, int(np.ceil(np.max(np.abs(end - start)) / edge_resolution))
                )
                for sample_index in range(1, sample_count + 1):
                    fraction = sample_index / sample_count
                    q = start + fraction * (end - start)
                    # The payload is attached through the edge arriving at
                    # release_index, then becomes an independently simulated
                    # dynamic body for the release-retreat path.
                    carried = grasp_index <= edge_index < release_index
                    if not state_valid(q, carried=carried):
                        valid = False
                        invalid_index = edge_index + 1 if sample_index == sample_count else None
                        invalid_edge = [edge_index, edge_index + 1]
                        invalid_edge_fraction = fraction
                        break
                if not valid:
                    break
        timed_segment = copy.deepcopy(segment)
        time_parameterize_segments([timed_segment], cfg)
        statistics.append(
            {
                "pick_index": int(segment["pick_index"]),
                "target": segment["target"],
                "valid": valid,
                "invalid_index": invalid_index,
                "invalid_edge": invalid_edge,
                "invalid_edge_fraction": invalid_edge_fraction,
                "edge_resolution_rad": edge_resolution,
                "continuous_edges_validated": True,
                "validation_seconds": perf_counter() - started,
                "waypoints": len(path),
                "execution_timing": timed_segment["timing"],
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
    parser.add_argument("--corner-fraction", type=float, default=0.22)
    parser.add_argument("--corner-samples", type=int, default=7)
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
    optimized, statistics = optimize_cached_plan(
        manifest,
        attempts=args.attempts,
        corner_fraction=args.corner_fraction,
        corner_samples=args.corner_samples,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(optimized, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output), "seconds": perf_counter() - start, "segments": statistics}, indent=2))


if __name__ == "__main__":
    main()
