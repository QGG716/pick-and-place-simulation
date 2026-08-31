"""Merge one regenerated segment into a full plan through an empty-arm connector."""

from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path

import numpy as np

from unloading_sim.online_unload import docked_robot_and_scene, remove_carton
from unloading_sim.geometry import OBB
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.scene import load_scene_config
from unloading_sim.timing import time_parameterize_segments


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--full-plan", required=True)
    parser.add_argument("--repair-plan", required=True)
    parser.add_argument("--segment", type=int, required=True)
    parser.add_argument(
        "--replace-prefix",
        action="store_true",
        help="Replace repair segments 0..segment, then connect the repaired prefix to the full-plan tail.",
    )
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    config_path = Path(args.config)
    full = json.loads(Path(args.full_plan).read_text(encoding="utf-8"))
    repair = json.loads(Path(args.repair_plan).read_text(encoding="utf-8"))
    index = args.segment
    if not (0 <= index < len(full["segments"])):
        raise ValueError("segment index is outside the full plan")
    if len(repair["segments"]) <= index:
        raise ValueError("repair plan does not contain the requested segment")
    for prefix_index in range(index):
        if repair["segments"][prefix_index]["target"] != full["segments"][prefix_index]["target"]:
            raise ValueError("repair plan prefix does not match the full plan")

    repaired = copy.deepcopy(repair["segments"][index])
    original = full["segments"][index]
    if repaired["target"] != original["target"]:
        raise ValueError("repair target does not match the full plan segment")
    if repaired["amr_dock_position"] != original["amr_dock_position"]:
        raise ValueError("repair segment changed the AMR dock")

    scene, cfg = load_scene_config(config_path)
    dock = np.asarray(repaired["amr_dock_position"], dtype=float)
    robot, docked_scene, _ = docked_robot_and_scene(scene, cfg, dock)
    repaired_carton = docked_scene.carton(repaired["target"])
    for segment in full["segments"][: index + 1]:
        remove_carton(docked_scene, segment["target"])

    # The repaired segment has released its carton, but that carton has not
    # disappeared from the physical cell. Keep its placed pose as a static
    # obstacle while connecting to the next pick. Omitting it allowed an
    # otherwise valid empty-arm RRT to sweep J6 through the settled payload.
    place_rotation = np.asarray(repaired["place_rotation"], dtype=float)
    place_center = np.asarray(repaired["place_center"], dtype=float)
    if place_rotation.shape != (3, 3) or place_center.shape != (3,):
        raise ValueError("repaired placement pose is malformed")
    docked_scene.obstacles.append(
        OBB(
            center=place_center,
            half_extents=repaired_carton.half_extents.copy(),
            rotation=place_rotation,
            name=f"placed_{repaired['target']}",
            category="carton",
        )
    )

    start_q = np.asarray(repaired["path"][-1], dtype=float)
    goal_q = np.asarray(full["segments"][index + 1]["path"][0], dtype=float)
    planning = cfg.get("planning", {})
    max_link3_elevation = planning.get("max_link3_elevation_deg")

    def state_valid(q: np.ndarray) -> bool:
        posture_ok = (
            max_link3_elevation is None
            or robot.link_elevation_degrees(q, 3) <= float(max_link3_elevation)
        )
        return posture_ok and robot.is_collision_free(q, docked_scene.all_obstacles)

    if not state_valid(start_q) or not state_valid(goal_q):
        raise RuntimeError("empty-arm connector endpoint is invalid")
    planner = RRTConnectPlanner(
        robot.joint_limits[:, 0],
        robot.joint_limits[:, 1],
        state_valid,
        step_size=float(planning.get("rrt_step_size", 0.22)),
        edge_resolution=float(planning.get("rrt_edge_resolution", 0.065)),
        max_iterations=int(planning.get("rrt_max_iterations", 4500)),
        goal_bias=float(planning.get("rrt_goal_bias", 0.15)),
        rng=np.random.default_rng(int(planning.get("seed", 11)) + 9000 + index),
    )
    result = planner.plan(start_q, goal_q, time_limit_seconds=180.0)
    if not result.success:
        raise RuntimeError(f"empty-arm connector failed: {result.message}")
    connector = planner.densify(
        planner.shortcut(
            result.path,
            attempts=int(planning.get("shortcut_attempts", 180)),
        ),
        resolution=float(planning.get("output_resolution", 0.035)),
    )
    if not np.allclose(connector[0], start_q) or not np.allclose(connector[-1], goal_q):
        raise RuntimeError("connector endpoints changed during processing")

    repaired["path"].extend(q.tolist() for q in connector[1:])
    repaired["base_path"].extend(
        [copy.deepcopy(repaired["base_path"][-1]) for _ in connector[1:]]
    )
    segments = copy.deepcopy(full["segments"])
    if args.replace_prefix:
        segments[: index + 1] = copy.deepcopy(repair["segments"][: index + 1])
        segments[index] = repaired
    else:
        segments[index] = repaired
    for boundary in range(len(segments) - 1):
        end = np.asarray(segments[boundary]["path"][-1], dtype=float)
        start = np.asarray(segments[boundary + 1]["path"][0], dtype=float)
        if not np.allclose(end, start, atol=1e-9, rtol=0.0):
            raise RuntimeError(f"segment boundary {boundary}/{boundary + 1} is discontinuous")

    merged = copy.deepcopy(full)
    merged["config"] = str(config_path)
    merged["segments"] = segments
    merged["execution_summary"] = time_parameterize_segments(segments, cfg)
    merged["repair_audit"] = {
        "replaced_segment_index": index,
        "replaced_prefix": bool(args.replace_prefix),
        "target": repaired["target"],
        "empty_arm_connector_waypoints": len(connector),
        "source_full_plan": str(Path(args.full_plan)),
        "source_repair_plan": str(Path(args.repair_plan)),
    }

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory_path = output_dir / "online_trajectory.csv"
    with trajectory_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["pick_index", "target", "local_index", "q1", "q2", "q3", "q4", "q5", "q6"])
        for segment in segments:
            for local_index, q in enumerate(segment["path"]):
                writer.writerow([segment["pick_index"], segment["target"], local_index, *q])
    merged["trajectory_csv"] = str(trajectory_path)
    merged["timed_trajectory_csv"] = None

    output_path = output_dir / "online_plan.json"
    output_path.write_text(json.dumps(merged, ensure_ascii=False, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(output_path),
                "segments": len(segments),
                "connector_waypoints": len(connector),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
