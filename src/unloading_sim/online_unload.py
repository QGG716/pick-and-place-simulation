"""Online top-down unloading loop for the geometric simulator."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import sys
from pathlib import Path
from time import perf_counter
from typing import Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.demo import build_robot
    from unloading_sim.geometry import OBB
    from unloading_sim.grasp import generate_suction_candidates, plan_pick, select_fast_suction_candidates
    from unloading_sim.ik import solve_ik_multistart
    from unloading_sim.perception import detect_carton_obbs, fixed_conveyor_place_pose
    from unloading_sim.scene import TrailerScene, load_scene_config
else:
    from .demo import build_robot
    from .geometry import OBB
    from .grasp import generate_suction_candidates, plan_pick, select_fast_suction_candidates
    from .ik import solve_ik_multistart
    from .perception import detect_carton_obbs, fixed_conveyor_place_pose
    from .scene import TrailerScene, load_scene_config


def top_down_carton_order(scene: TrailerScene) -> list[str]:
    detections = detect_carton_obbs(scene)
    ordered = sorted(
        detections,
        key=lambda detection: (
            -float(detection.obb.center[2] + detection.obb.half_extents[2]),
            float(detection.obb.center[0]),
            abs(float(detection.obb.center[1])),
            detection.name,
        ),
    )
    return [detection.name for detection in ordered]


def _overlaps_interval(a_min: float, a_max: float, b_min: float, b_max: float, clearance: float = 1e-6) -> bool:
    return min(a_max, b_max) > max(a_min, b_min) + clearance


def exposed_carton_order(scene: TrailerScene) -> list[str]:
    """Return cartons exposed from above and from the trailer door.

    Planning a box that has another box directly above it, or directly between
    it and the door, cannot produce a valid suction withdrawal.  This cheap
    O(n^2) OBB-axis check avoids expensive IK/RRT calls for those boxes.
    """
    cartons = scene.cartons
    exposed: list[str] = []
    for carton in cartons:
        x_min, y_min, z_min = carton.center - carton.half_extents
        x_max, y_max, z_max = carton.center + carton.half_extents
        blocked_above = False
        blocked_at_door = False
        for other in cartons:
            if other.name == carton.name:
                continue
            other_x_min, other_y_min, other_z_min = other.center - other.half_extents
            other_x_max, other_y_max, other_z_max = other.center + other.half_extents
            if (
                other_z_min >= z_max - 1e-6
                and _overlaps_interval(x_min, x_max, other_x_min, other_x_max)
                and _overlaps_interval(y_min, y_max, other_y_min, other_y_max)
            ):
                blocked_above = True
            if (
                other_x_max <= x_min + 1e-6
                and _overlaps_interval(y_min, y_max, other_y_min, other_y_max)
                and _overlaps_interval(z_min, z_max, other_z_min, other_z_max)
            ):
                blocked_at_door = True
        if not blocked_above and not blocked_at_door:
            exposed.append(carton.name)

    rank = {name: index for index, name in enumerate(top_down_carton_order(scene))}
    return sorted(exposed, key=rank.__getitem__)


def remove_carton(scene: TrailerScene, name: str) -> None:
    scene.cartons = [carton for carton in scene.cartons if carton.name != name]


def carton_is_outside_fixed_base_reach(robot, carton, tolerance: float = 0.03) -> bool:
    """Conservatively reject cartons whose nearest point exceeds arm reach."""
    base_position = robot.base_transform[:3, 3]
    nearest_local = np.clip(
        carton.to_local(base_position),
        -carton.half_extents,
        carton.half_extents,
    )
    nearest_point = carton.to_world(nearest_local)
    return float(np.linalg.norm(nearest_point - base_position)) > robot.max_reach + tolerance


def carton_blocked_by_scene(scene: TrailerScene, carton_name: str) -> bool:
    return carton_name not in exposed_carton_order(scene)


def amr_dock_positions(cfg: dict) -> list[np.ndarray]:
    amr_cfg = cfg.get("amr", {})
    positions = amr_cfg.get("dock_positions")
    if positions is None:
        return [np.asarray(cfg["robot"]["base_position"], dtype=float)]
    return [np.asarray(position, dtype=float) for position in positions]


def prioritized_amr_dock_positions(cfg: dict, current_dock: np.ndarray | None) -> list[np.ndarray]:
    docks = amr_dock_positions(cfg)
    if current_dock is None:
        return docks
    return sorted(docks, key=lambda dock: (not np.allclose(dock, current_dock), float(np.linalg.norm(dock - current_dock))))


def validate_amr_conveyor_alignment(cfg: dict) -> None:
    amr_cfg = cfg.get("amr", {})
    conveyor_center = np.asarray(amr_cfg.get("conveyor_mount_center"), dtype=float)
    conveyor = next(
        obstacle for obstacle in cfg["scene"].get("static_obstacles", []) if obstacle["name"] == amr_cfg.get("conveyor_name", "conveyor_deck")
    )
    conveyor_front_x = conveyor_center[0] + 0.5 * float(conveyor["size"][0])
    robot_front_x = float(np.asarray(amr_cfg.get("robot_mount_position", [0.0, 0.0, 0.0]))[0])
    overhang = float(amr_cfg.get("conveyor_front_overhang", 0.20))
    size = np.asarray(amr_cfg.get("footprint_size"), dtype=float)
    platform_offset = np.asarray(amr_cfg.get("platform_center_offset"), dtype=float)
    amr_front_x = float(platform_offset[0] + 0.5 * size[0])
    if not np.isclose(conveyor_front_x, robot_front_x + overhang, atol=1e-9):
        raise ValueError("AMR conveyor front must extend 0.20 m beyond the robot base front")
    if conveyor_front_x < amr_front_x - 1e-9:
        raise ValueError("AMR conveyor front must not sit behind the AMR platform front")


def amr_dock_is_clear(scene: TrailerScene, cfg: dict, dock_position: np.ndarray) -> bool:
    amr_cfg = cfg.get("amr", {})
    size = np.asarray(amr_cfg.get("footprint_size", [1.60, 1.20, 0.30]), dtype=float)
    platform_offset = np.asarray(amr_cfg.get("platform_center_offset", [0.0, 0.0, size[2] / 2.0]), dtype=float)
    base = OBB(
        center=np.asarray(dock_position, dtype=float) + platform_offset,
        half_extents=size / 2.0,
        rotation=np.eye(3),
        name="amr_base",
        category="amr",
    )
    for obstacle in scene.obstacles:
        if obstacle.name == "trailer_floor" or obstacle.name == amr_cfg.get("conveyor_name", "conveyor_deck"):
            continue
        if base.intersects_obb(obstacle, margin=0.02):
            return False
    return all(not base.intersects_obb(carton, margin=0.02) for carton in scene.cartons)


def docked_robot_and_scene(scene: TrailerScene, cfg: dict, dock_position: np.ndarray):
    """Build a fixed-base planning scene for one AMR dock position."""
    local_cfg = copy.deepcopy(cfg)
    amr_cfg = local_cfg.get("amr", {})
    robot_mount = np.asarray(amr_cfg.get("robot_mount_position", [0.0, 0.0, 0.0]), dtype=float)
    local_cfg["robot"]["base_position"] = (np.asarray(dock_position, dtype=float) + robot_mount).tolist()
    conveyor_name = amr_cfg.get("conveyor_name", "conveyor_deck")
    conveyor_center = np.asarray(amr_cfg.get("conveyor_mount_center", [-1.45, -1.70, 0.37]), dtype=float)
    obstacles: list[OBB] = []
    for obstacle in scene.obstacles:
        if obstacle.name == conveyor_name:
            obstacles.append(
                OBB(
                    center=np.asarray(dock_position, dtype=float) + conveyor_center,
                    half_extents=obstacle.half_extents,
                    rotation=obstacle.rotation,
                    name=obstacle.name,
                    category=obstacle.category,
                )
            )
        else:
            obstacles.append(obstacle)
    return build_robot(local_cfg), TrailerScene(obstacles=obstacles, cartons=list(scene.cartons), metadata=dict(scene.metadata)), local_cfg


def score_target_dock_ik(
    robot,
    scene: TrailerScene,
    start_q: np.ndarray,
    target_name: str,
    planning_cfg: dict,
    rng: np.random.Generator,
) -> dict:
    """Measure grasp IK yield and the shortest reachable end-effector motion."""
    carton = scene.carton(target_name)
    candidates = select_fast_suction_candidates(
        generate_suction_candidates(carton, robot.base_transform[:3, 3], face_modes=planning_cfg.get("grasp_face_modes", ["front", "side", "top"])),
        planning_cfg.get("grasp_face_modes", ["front", "side", "top"]),
        int(planning_cfg.get("grasp_candidates_per_face", 1)),
    )
    start_position = robot.fk(start_q)[:3, 3]
    successful = 0
    shortest_motion = float("inf")
    for candidate in candidates:
        pre = solve_ik_multistart(
            robot,
            candidate.pregrasp_pose,
            seeds=[start_q],
            obstacles=scene.all_obstacles,
            random_restarts=int(planning_cfg.get("dock_ik_probe_random_restarts", 1)),
            rng=rng,
            max_iterations=int(planning_cfg.get("ik_max_iterations", 140)),
            position_tolerance=float(planning_cfg.get("ik_position_tolerance", 0.018)),
            orientation_tolerance=float(planning_cfg.get("ik_orientation_tolerance", 0.16)),
        )
        if not pre.success:
            continue
        grasp = solve_ik_multistart(
            robot,
            candidate.grasp_pose,
            seeds=[pre.q, start_q],
            obstacles=scene.all_obstacles,
            ignored_obstacle_names={carton.name},
            random_restarts=0,
            rng=rng,
            max_iterations=int(planning_cfg.get("ik_max_iterations", 140)),
            position_tolerance=float(planning_cfg.get("ik_position_tolerance", 0.018)),
            orientation_tolerance=float(planning_cfg.get("ik_orientation_tolerance", 0.16)),
        )
        if not grasp.success:
            continue
        successful += 1
        shortest_motion = min(
            shortest_motion,
            float(np.linalg.norm(candidate.pregrasp_pose[:3, 3] - start_position))
            + float(np.linalg.norm(candidate.grasp_pose[:3, 3] - candidate.pregrasp_pose[:3, 3])),
        )
    total = len(candidates)
    return {
        "ik_success_count": successful,
        "ik_candidate_count": total,
        "ik_success_rate": successful / total if total else 0.0,
        "estimated_ee_motion_m": shortest_motion if successful else None,
    }


def run_online(config_path: str | Path, max_picks: int | None, output_dir: str | Path | None = None) -> dict:
    config_path = Path(config_path)
    scene, cfg = load_scene_config(config_path)
    validate_amr_conveyor_alignment(cfg)
    robot = build_robot(cfg)
    planning_cfg = dict(cfg.get("planning", {}))
    planning_cfg["verbose"] = True
    planning_seed = int(planning_cfg.get("seed", 11))
    scoring_rng = np.random.default_rng(planning_seed)
    current_q = np.asarray(cfg["robot"]["home_joints"], dtype=float)

    outputs = cfg.get("outputs", {})
    out_dir = Path(output_dir or outputs.get("directory", "outputs/online_unload"))
    if not out_dir.is_absolute():
        out_dir = config_path.resolve().parent.parent / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    conveyor_name = planning_cfg.get("place_obstacle", "conveyor_deck")
    conveyor = next((obstacle for obstacle in scene.obstacles if obstacle.name == conveyor_name), None)

    segments: list[dict] = []
    trajectory_rows: list[tuple[int, str, int, np.ndarray]] = []
    unmoved_boxes: dict[str, str] = {}
    start_time = perf_counter()
    max_pick_count = len(scene.cartons) if max_picks is None or max_picks <= 0 else max_picks
    print(f"[online] start: robot={robot.name}, cartons={len(scene.cartons)}, max_picks={max_pick_count}", flush=True)

    pick_index = 0
    current_dock: np.ndarray | None = None
    while scene.cartons and len(segments) < max_pick_count:
        ordered_names = exposed_carton_order(scene)
        detections = detect_carton_obbs(scene)
        reachable_names = ordered_names
        print(
            f"[online] pick {pick_index}: detected {len(detections)} OBBs; "
            f"exposed reachable candidates={reachable_names[:8]}",
            flush=True,
        )

        chosen_result = None
        chosen_name = None
        chosen_dock = None
        chosen_scene = None
        chosen_cfg = None
        failed_this_round: dict[str, str] = {}
        scored_pairs: list[tuple[tuple[float, float, float], np.ndarray, str, dict]] = []
        for dock_position in prioritized_amr_dock_positions(cfg, current_dock):
            if not amr_dock_is_clear(scene, cfg, dock_position):
                continue
            dock_robot, dock_scene, _ = docked_robot_and_scene(scene, cfg, dock_position)
            for target_name in reachable_names:
                if carton_is_outside_fixed_base_reach(dock_robot, dock_scene.carton(target_name)):
                    continue
                score = score_target_dock_ik(dock_robot, dock_scene, current_q, target_name, planning_cfg, scoring_rng)
                ee_motion = score["estimated_ee_motion_m"]
                dock_motion = 0.0 if current_dock is None else float(np.linalg.norm(dock_position - current_dock))
                rank = (-score["ik_success_rate"], float("inf") if ee_motion is None else ee_motion, dock_motion)
                scored_pairs.append((rank, dock_position, target_name, score))

        for _, dock_position, target_name, dock_score in sorted(scored_pairs, key=lambda item: item[0]):
                dock_robot, dock_scene, dock_cfg = docked_robot_and_scene(scene, cfg, dock_position)
                dock_start = current_q.copy()
                print(
                    f"[online] pick {pick_index}: planning target={target_name}; "
                    f"amr_dock={np.round(dock_position, 3).tolist()}; "
                    f"ik={dock_score['ik_success_count']}/{dock_score['ik_candidate_count']}; "
                    f"ee_motion={dock_score['estimated_ee_motion_m']}",
                    flush=True,
                )
                target_rng = np.random.default_rng(planning_seed + pick_index)
                result = plan_pick(dock_robot, dock_scene, dock_start, target_name, planner_options=planning_cfg, rng=target_rng)
                if result.success:
                    chosen_result = result
                    chosen_name = target_name
                    chosen_dock = dock_position
                    chosen_scene = dock_scene
                    chosen_cfg = dock_cfg
                    chosen_dock_score = dock_score
                    robot = dock_robot
                    break
                failed_this_round[target_name] = result.message

        if chosen_result is None or chosen_name is None:
            for target_name, message in failed_this_round.items():
                unmoved_boxes[target_name] = f"no collision-free fixed-base pick/place plan: {message}"
            print(f"[online] pick {pick_index}: no feasible fixed-base target remains", flush=True)
            break

        assert chosen_scene is not None and chosen_cfg is not None and chosen_dock is not None
        target_carton = scene.carton(chosen_name)
        place_center = None
        place_rotation = None
        max_carried_tilt = None
        if chosen_result.grasp_ik is not None and chosen_result.place_ik is not None:
            carton_from_tool = np.linalg.inv(robot.fk(chosen_result.grasp_ik.q)) @ target_carton.world_from_local
            placed_carton = robot.fk(chosen_result.place_ik.q) @ carton_from_tool
            place_center = placed_carton[:3, 3].copy()
            place_rotation = placed_carton[:3, :3].copy()
            carried_tilts = []
            for q in chosen_result.place_path:
                carried_rotation = (robot.fk(q) @ carton_from_tool)[:3, :3]
                alignment = float(np.clip(carried_rotation[:, 2] @ target_carton.rotation[:, 2], -1.0, 1.0))
                carried_tilts.append(float(np.degrees(np.arccos(alignment))))
            max_carried_tilt = max(carried_tilts, default=0.0)

        path = [np.asarray(q, dtype=float) for q in chosen_result.full_path]
        for local_index, q in enumerate(path):
            trajectory_rows.append((pick_index, chosen_name, local_index, q))
        segments.append(
            {
                "pick_index": pick_index,
                "target": chosen_name,
                "face_mode": chosen_result.candidate.face_mode if chosen_result.candidate else None,
                "contact_point": chosen_result.candidate.contact_point.tolist() if chosen_result.candidate else None,
                "place_center": place_center.tolist() if place_center is not None else None,
                "place_rotation": place_rotation.tolist() if place_rotation is not None else None,
                "max_carried_carton_tilt_deg": max_carried_tilt,
                "grasp_index": chosen_result.grasp_index,
                "release_index": chosen_result.release_index,
                "release_retreat_index": chosen_result.release_retreat_index,
                "path": [q.tolist() for q in path],
                "base_path": [base_position.tolist() for base_position in chosen_result.base_path],
                "amr_dock_position": chosen_dock.tolist(),
                "dock_ik_score": chosen_dock_score,
            }
        )
        current_q = path[-1].copy()
        current_dock = chosen_dock.copy()
        remove_carton(scene, chosen_name)
        unmoved_boxes.pop(chosen_name, None)
        print(
            f"[online] pick {pick_index}: moved {chosen_name}; remaining={len(scene.cartons)}; "
            f"segment_waypoints={len(path)}",
            flush=True,
        )
        pick_index += 1

    trajectory_csv = out_dir / "online_trajectory.csv"
    with trajectory_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["pick_index", "target", "local_index", "q1", "q2", "q3", "q4", "q5", "q6"])
        for pick_index, target, local_index, q in trajectory_rows:
            writer.writerow([pick_index, target, local_index, *[float(v) for v in q]])

    manifest = {
        "config": str(config_path),
        "robot": cfg["robot"],
        "planning_time_seconds": perf_counter() - start_time,
        "segments": segments,
        "moved_boxes": [segment["target"] for segment in segments],
        "unmoved_boxes": [
            {
                "name": carton.name,
                "reason": unmoved_boxes.get(
                    carton.name,
                    "blocked by an unresolved carton" if carton_blocked_by_scene(scene, carton.name) else "max_picks limit reached",
                ),
            }
            for carton in scene.cartons
        ],
        "trajectory_csv": str(trajectory_csv),
    }
    manifest_path = out_dir / "online_plan.json"
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"[online] wrote {manifest_path}", flush=True)
    print(json.dumps({"success": bool(segments), "moved_boxes": len(segments), "manifest": str(manifest_path)}, ensure_ascii=False, indent=2))
    return manifest


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Online top-down carton unloading planner")
    parser.add_argument("--config", default="config/kuka_kr50.yaml")
    parser.add_argument("--max-picks", type=int, default=0, help="0 means unload all detected cartons")
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args(argv)
    run_online(args.config, args.max_picks, args.output_dir)


if __name__ == "__main__":
    main()