"""Replace one plan segment and collision-check empty-arm connectors on both sides."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from unloading_sim.geometry import OBB
from unloading_sim.online_unload import docked_robot_and_scene, remove_carton
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.scene import load_scene_config
from unloading_sim.timing import time_parameterize_segments


def _connector(robot, scene, cfg, start, goal, seed):
    planning = cfg.get("planning", {})
    max_link3_elevation = planning.get("max_link3_elevation_deg")

    def state_valid(q):
        posture_ok = (
            max_link3_elevation is None
            or robot.link_elevation_degrees(q, 3) <= float(max_link3_elevation)
        )
        return posture_ok and robot.is_collision_free(q, scene.all_obstacles)

    if not state_valid(start) or not state_valid(goal):
        raise RuntimeError("empty-arm connector endpoint is invalid")
    planner = RRTConnectPlanner(
        robot.joint_limits[:, 0],
        robot.joint_limits[:, 1],
        state_valid,
        step_size=float(planning.get("rrt_step_size", 0.22)),
        edge_resolution=float(planning.get("rrt_edge_resolution", 0.065)),
        max_iterations=int(planning.get("rrt_max_iterations", 4500)),
        goal_bias=float(planning.get("rrt_goal_bias", 0.15)),
        rng=np.random.default_rng(seed),
    )
    result = planner.plan(start, goal, time_limit_seconds=180.0)
    if not result.success:
        raise RuntimeError(f"empty-arm connector failed: {result.message}")
    return planner.densify(
        planner.shortcut(result.path, attempts=int(planning.get("shortcut_attempts", 180))),
        resolution=float(planning.get("output_resolution", 0.035)),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--full-plan", required=True)
    parser.add_argument("--repair-plan", required=True)
    parser.add_argument("--segment", type=int, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    full = json.loads(Path(args.full_plan).read_text(encoding="utf-8"))
    repair = json.loads(Path(args.repair_plan).read_text(encoding="utf-8"))
    index = args.segment
    if not 0 < index < len(full["segments"]) - 1:
        raise ValueError("two-sided repair requires an interior segment")
    for prefix_index in range(index):
        if repair["segments"][prefix_index]["target"] != full["segments"][prefix_index]["target"]:
            raise ValueError("repair plan prefix does not match the full plan")
    repaired = copy.deepcopy(repair["segments"][index])
    if repaired["target"] != full["segments"][index]["target"]:
        raise ValueError("repair target does not match")
    if repaired["amr_dock_position"] != full["segments"][index]["amr_dock_position"]:
        raise ValueError("repair segment changed the AMR dock")

    scene, cfg = load_scene_config(Path(args.config))
    dock = np.asarray(repaired["amr_dock_position"], dtype=float)

    pre_robot, pre_scene, _ = docked_robot_and_scene(scene, cfg, dock)
    for segment in full["segments"][:index]:
        remove_carton(pre_scene, segment["target"])
    before = _connector(
        pre_robot,
        pre_scene,
        cfg,
        np.asarray(full["segments"][index - 1]["path"][-1], dtype=float),
        np.asarray(repaired["path"][0], dtype=float),
        int(cfg.get("planning", {}).get("seed", 11)) + 8000 + index,
    )

    post_robot, post_scene, _ = docked_robot_and_scene(scene, cfg, dock)
    repaired_carton = post_scene.carton(repaired["target"])
    for segment in full["segments"][: index + 1]:
        remove_carton(post_scene, segment["target"])
    post_scene.obstacles.append(
        OBB(
            center=np.asarray(repaired["place_center"], dtype=float),
            half_extents=repaired_carton.half_extents.copy(),
            rotation=np.asarray(repaired["place_rotation"], dtype=float),
            name=f"placed_{repaired['target']}",
            category="carton",
        )
    )
    after = _connector(
        post_robot,
        post_scene,
        cfg,
        np.asarray(repaired["path"][-1], dtype=float),
        np.asarray(full["segments"][index + 1]["path"][0], dtype=float),
        int(cfg.get("planning", {}).get("seed", 11)) + 9000 + index,
    )

    prefix_count = len(before) - 1
    repaired["path"] = (
        [q.tolist() for q in before[:-1]]
        + repaired["path"]
        + [q.tolist() for q in after[1:]]
    )
    repaired["base_path"] = (
        [copy.deepcopy(repaired["base_path"][0]) for _ in before[:-1]]
        + repaired["base_path"]
        + [copy.deepcopy(repaired["base_path"][-1]) for _ in after[1:]]
    )
    for key in ("grasp_index", "release_index", "release_retreat_index"):
        if repaired.get(key) is not None:
            repaired[key] = int(repaired[key]) + prefix_count

    segments = copy.deepcopy(full["segments"])
    segments[index] = repaired
    for boundary in range(len(segments) - 1):
        if not np.allclose(
            segments[boundary]["path"][-1],
            segments[boundary + 1]["path"][0],
            atol=1e-9,
            rtol=0.0,
        ):
            raise RuntimeError(f"segment boundary {boundary}/{boundary + 1} is discontinuous")

    merged = copy.deepcopy(full)
    merged["config"] = str(Path(args.config))
    merged["segments"] = segments
    merged["execution_summary"] = time_parameterize_segments(segments, cfg)
    merged["repair_audit"] = {
        "replaced_segment_index": index,
        "source_full_plan": str(Path(args.full_plan)),
        "source_repair_plan": str(Path(args.repair_plan)),
        "connector_before_waypoints": len(before),
        "connector_after_waypoints": len(after),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(merged, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output), **merged["repair_audit"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
