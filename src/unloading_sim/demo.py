"""Command-line demo for the first-layer unloading simulator."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from time import perf_counter

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.grasp import plan_pick
    from unloading_sim.robot import DHRobot6, RobotKinematics6, URDFRobot6
    from unloading_sim.scene import load_scene_config
    from unloading_sim.visualization import save_plan_figure
else:
    from .grasp import plan_pick
    from .robot import DHRobot6, RobotKinematics6, URDFRobot6
    from .scene import load_scene_config
    from .visualization import save_plan_figure


def build_robot(cfg: dict) -> RobotKinematics6:
    robot_cfg = cfg["robot"]
    model = robot_cfg.get("model", "ur5e_like")
    if model == "ur5e_like":
        return DHRobot6.ur5e_like(
            base_position=robot_cfg["base_position"],
            base_rpy=robot_cfg.get("base_rpy", [0.0, 0.0, 0.0]),
            tool_length=float(robot_cfg.get("tool_length", 0.18)),
        )
    if model == "kuka_kr50_r2500":
        return URDFRobot6.kuka_kr50_r2500(
            urdf_path=robot_cfg.get("urdf_path", "assets/robots/kuka_kr50_r2500/kr_50_r2500.urdf"),
            base_position=robot_cfg.get("base_position", [-1.15, 0.0, 0.0]),
            base_rpy=robot_cfg.get("base_rpy", [0.0, 0.0, 0.0]),
            tool_length=float(robot_cfg.get("tool_length", 0.20)),
        )
    raise ValueError(f"Unsupported built-in model: {model}")


def run(config_path: str | Path) -> dict:
    print(f"[demo] loading config: {config_path}", flush=True)
    scene, cfg = load_scene_config(config_path)
    robot = build_robot(cfg)
    start_q = np.asarray(cfg["robot"]["home_joints"], dtype=float)
    planning_cfg = cfg.get("planning", {})
    target_name = planning_cfg["target_carton"]
    rng = np.random.default_rng(int(planning_cfg.get("seed", 7)))
    print(
        f"[demo] robot={robot.name}, cartons={len(scene.cartons)}, obstacles={len(scene.obstacles)}, target={target_name}",
        flush=True,
    )

    collision = robot.collision_result(start_q, scene.all_obstacles)
    if collision.in_collision:
        raise RuntimeError(
            f"Configured home pose is in collision: {collision.reason}, "
            f"{collision.first_link}, {collision.first_obstacle}"
        )

    t0 = perf_counter()
    print("[demo] planning pick/place path...", flush=True)
    result = plan_pick(
        robot,
        scene,
        start_q,
        target_name,
        planner_options=planning_cfg,
        rng=rng,
    )
    elapsed = perf_counter() - t0

    outputs = cfg.get("outputs", {})
    out_dir = Path(outputs.get("directory", "outputs/demo"))
    if not out_dir.is_absolute():
        out_dir = Path(config_path).resolve().parent.parent / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    metrics = {
        "success": result.success,
        "message": result.message,
        "target_carton": target_name,
        "planning_time_seconds": elapsed,
        "trajectory_waypoints": len(result.full_path),
        "rrt_iterations": result.transit_plan.iterations if result.transit_plan else None,
        "place_rrt_iterations": result.place_plan.iterations if result.place_plan else None,
        "pregrasp_position_error_m": result.pregrasp_ik.position_error if result.pregrasp_ik else None,
        "pregrasp_orientation_error_rad": result.pregrasp_ik.orientation_error if result.pregrasp_ik else None,
        "grasp_position_error_m": result.grasp_ik.position_error if result.grasp_ik else None,
        "grasp_orientation_error_rad": result.grasp_ik.orientation_error if result.grasp_ik else None,
        "place_position_error_m": result.place_ik.position_error if result.place_ik else None,
        "place_orientation_error_rad": result.place_ik.orientation_error if result.place_ik else None,
        "grasp_face_mode": result.candidate.face_mode if result.candidate else None,
        "contact_point_m": result.candidate.contact_point.tolist() if result.candidate else None,
        "outward_normal": result.candidate.outward_normal.tolist() if result.candidate else None,
        "place_tool_position_m": robot.fk(result.place_ik.q)[:3, 3].tolist() if result.place_ik else None,
    }

    if outputs.get("save_trajectory_csv", True) and result.full_path:
        csv_path = out_dir / "trajectory.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "q1", "q2", "q3", "q4", "q5", "q6", "tool_x", "tool_y", "tool_z"])
            for idx, q in enumerate(result.full_path):
                tool = robot.fk(np.asarray(q))[:3, 3]
                writer.writerow([idx, *[float(v) for v in q], *[float(v) for v in tool]])

    if outputs.get("save_metrics_json", True):
        with (out_dir / "metrics.json").open("w", encoding="utf-8") as f:
            json.dump(metrics, f, ensure_ascii=False, indent=2)

    if outputs.get("save_plot", True):
        save_plan_figure(out_dir / "plan.png", robot, scene, result.full_path or [start_q], target_name)

    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the layer-1 trailer unloading motion-planning demo")
    parser.add_argument("--config", default="config/demo.yaml", help="Path to YAML configuration")
    args = parser.parse_args()
    run(args.config)


if __name__ == "__main__":
    main()
