"""Command-line demo for the first-layer unloading simulator."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import sys
from pathlib import Path
from time import perf_counter

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.grasp import plan_pick
    from unloading_sim.robot import DHRobot6, RobotBackend, URDFRobot6
    from unloading_sim.scene import load_scene_config
    from unloading_sim.timing import motion_limits_from_config, time_parameterize_joint_path
    from unloading_sim.visualization import save_plan_figure
else:
    from .grasp import plan_pick
    from .robot import DHRobot6, RobotBackend, URDFRobot6
    from .scene import load_scene_config
    from .timing import motion_limits_from_config, time_parameterize_joint_path
    from .visualization import save_plan_figure


def build_robot(cfg: dict) -> RobotBackend:
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
    if model == "fanuc_m20id35":
        validation_cfg = cfg.get("simulation_validation", {})
        tool_collision_size = validation_cfg.get("vacuum_rigid_plate_collision_size_m")
        tool_collision_center_offset = validation_cfg.get(
            "vacuum_rigid_plate_center_behind_working_plane_m"
        )
        tool_collision_local_boxes = None
        mass_properties_path = validation_cfg.get("vacuum_mass_properties_path")
        if mass_properties_path:
            mass_properties = json.loads(
                Path(mass_properties_path).read_text(encoding="utf-8")
            )
            rigid_bounds = mass_properties.get(
                "rigid_collision_bounding_boxes_step_mm"
            )
            if rigid_bounds:
                flange_step = np.asarray(
                    validation_cfg["vacuum_flange_origin_step_mm"], dtype=float
                )
                step_from_tool = np.asarray(
                    validation_cfg["vacuum_step_from_tool_rotation_matrix"],
                    dtype=float,
                )
                tool_length = float(robot_cfg.get("tool_length", 0.20))
                tool_collision_local_boxes = []
                for bounds in rigid_bounds:
                    lower = np.asarray(bounds[:3], dtype=float)
                    upper = np.asarray(bounds[3:], dtype=float)
                    step_corners = np.asarray(
                        list(itertools.product(*zip(lower, upper))), dtype=float
                    )
                    isaac_tool = (
                        (step_corners - flange_step) @ step_from_tool * 1e-3
                    )
                    # STEP/Isaac tool axes are [normal, length, width]; the
                    # geometric planner uses [width, length, normal] with FK
                    # located at the suction working plane.
                    planner_tool = isaac_tool[:, [2, 1, 0]]
                    planner_tool[:, 2] -= tool_length
                    box_min = np.min(planner_tool, axis=0)
                    box_max = np.max(planner_tool, axis=0)
                    tool_collision_local_boxes.append(
                        np.concatenate((0.5 * (box_min + box_max), box_max - box_min))
                    )
        return URDFRobot6.fanuc_m20id35(
            urdf_path=robot_cfg.get(
                "urdf_path", "assets/robots/fanuc_m20id35/m20_35_18d.urdf"
            ),
            base_position=robot_cfg.get("base_position", [-0.70, 0.0, 0.30]),
            base_rpy=robot_cfg.get("base_rpy", [0.0, 0.0, 0.0]),
            tool_length=float(robot_cfg.get("tool_length", 0.20)),
            tool_collision_size=tool_collision_size,
            tool_collision_center_offset=tool_collision_center_offset,
            tool_collision_local_boxes=tool_collision_local_boxes,
        )
    raise ValueError(f"Unsupported built-in model: {model}")


def validate_start_configuration(
    robot: RobotBackend,
    q: np.ndarray,
    obstacles,
    context: str = "configured home pose",
) -> None:
    q = np.asarray(q, dtype=float)
    if q.shape != (robot.dof,):
        raise ValueError(f"{context} must contain exactly {robot.dof} joint values")
    collision = robot.collision_result(q, obstacles)
    if collision.in_collision:
        raise RuntimeError(
            f"{context} is invalid: {collision.reason}, "
            f"{collision.first_link}, {collision.first_obstacle}"
        )


def run(config_path: str | Path, output_dir: str | Path | None = None) -> dict:
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

    validate_start_configuration(robot, start_q, scene.all_obstacles)

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
    timed = None
    timing_audit = None
    if result.full_path:
        motion_limits = motion_limits_from_config(cfg, len(start_q))
        timed = time_parameterize_joint_path(result.full_path, motion_limits)
        timing_audit = timed.audit(motion_limits)

    outputs = cfg.get("outputs", {})
    out_dir = Path(output_dir) if output_dir is not None else Path(outputs.get("directory", "outputs/demo"))
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
        "execution_timing": timing_audit,
    }

    if outputs.get("save_trajectory_csv", True) and result.full_path:
        csv_path = out_dir / "trajectory.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["index", *[f"q{i + 1}" for i in range(robot.dof)], "tool_x", "tool_y", "tool_z"])
            for idx, q in enumerate(result.full_path):
                tool = robot.fk(np.asarray(q))[:3, 3]
                writer.writerow([idx, *[float(v) for v in q], *[float(v) for v in tool]])
        timed_csv_path = out_dir / "trajectory_timed.csv"
        with timed_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "time_from_start_s", *[f"q{i + 1}" for i in range(robot.dof)]])
            for idx, q in enumerate(result.full_path):
                timestamp = timed.time_from_start[idx] if timed is not None else 0.0
                writer.writerow([idx, float(timestamp), *[float(v) for v in q]])

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
