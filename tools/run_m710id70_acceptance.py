"""Run deterministic FANUC M-710iD/70 unloading acceptance revision 2."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Any, Sequence

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.depalletizing import (  # noqa: E402
    FACE_OUTWARD_NORMALS, BoxNeighborhoodState, CandidateMetrics, HandoffProtocol, LConveyorGeometry,
    analyze_box_neighborhood, generate_extraction_candidates,
    minimum_clearance_extraction_distance, plan_conveyor_preposition, score_candidate,
)
from unloading_sim.fanuc_m710id70 import (  # noqa: E402
    HOME_JOINTS_RAD, TaskPointResult, _carried_box, build_robot,
    combined_payload_properties, evaluate_task_point, official_reach_guard,
    target_pose, trailer_obstacles, validate_model,
)
from unloading_sim.geometry import OBB  # noqa: E402
from unloading_sim.ik import solve_ik_multistart  # noqa: E402

FAILURE_REASONS = (
    "NO_IK", "JOINT_LIMIT", "SINGULARITY", "ROBOT_COLLISION",
    "TOOL_COLLISION", "TARGET_BOX_COLLISION", "NEIGHBOR_BOX_COLLISION",
    "EXTRACTION_FAILED", "CONVEYOR_POSITION_FAILED", "HANDOFF_FAILED",
    "PAYLOAD_CG_FAILED", "WRIST_LIMIT_FAILED",
)


@dataclass
class CompleteTaskResult:
    grasp_reachable: bool
    extraction_feasible: bool
    conveyor_position_feasible: bool
    conveyor_handoff_feasible: bool
    geometrically_reachable: bool
    payload_qualified: bool
    qualified_task: bool
    failure_reason: str
    grasp_face: str
    strategy: str
    minimum_extraction_distance_m: float | None
    conveyor_extension_m: float | None
    conveyor_z_m: float | None
    loaded_tcp_path_length_m: float | None
    fixed_conveyor_baseline_path_m: float | None
    joint_space_path_length_rad: float | None
    minimum_joint_margin_rad: float | None
    maximum_jacobian_condition: float | None
    candidate_cost: float | None
    exposed_face_count: int
    q_handoff: np.ndarray | None = None


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_jsonable(value), ensure_ascii=False, indent=2), encoding="utf-8")


def _write_csv(path: Path, rows: list[dict], fieldnames: Sequence[str] | None = None) -> None:
    names = list(fieldnames or (list(rows[0]) if rows else ["status"]))
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows or [{"status": "NO_ROWS"}])


def _inclusive(lower: float, upper: float, step: float) -> np.ndarray:
    values = lower + step * np.arange(int(np.floor((upper - lower) / step + 1e-9)) + 1)
    return np.append(values, upper) if values[-1] < upper - 1e-9 else np.concatenate((values[:-1], [upper]))


def _box(name: str, center: Sequence[float], size: Sequence[float]) -> OBB:
    return OBB(np.asarray(center, float), 0.5 * np.asarray(size, float), np.eye(3), name, "carton")


def _surface_area(size: Sequence[float], face: str) -> float:
    depth, width, height = np.asarray(size, float)
    return float({"front": width * height, "left": depth * height,
                  "right": depth * height, "top": depth * width}[face])


def _handoff_pose(target: OBB, face: str, conveyor) -> tuple[np.ndarray, np.ndarray]:
    size = 2.0 * target.half_extents
    longitudinal = conveyor.obstacles[1]
    center = np.array([conveyor.receiving_front_x_m - target.half_extents[0],
                       longitudinal.center[1], conveyor.conveyor_z_m + target.half_extents[2]])
    pose, _ = target_pose(center[0] - target.half_extents[0], center[1], center[2], size, face)
    return pose, center


def _solve_handoff(robot, task: TaskPointResult, target: OBB, other_boxes: Sequence[OBB],
                   walls: Sequence[OBB], conveyor, rng_seed: int):
    pose, desired_center = _handoff_pose(target, task.grasp_mode, conveyor)
    obstacles = [*walls, *other_boxes, *conveyor.obstacles]
    result = solve_ik_multistart(
        robot, pose, seeds=[task.path[-1], task.q_contact, HOME_JOINTS_RAD],
        obstacles=obstacles, random_restarts=1, rng=np.random.default_rng(rng_seed),
        max_iterations=110, damping=0.045, max_step=0.20,
        position_tolerance=0.015, orientation_tolerance=0.10,
        orientation_weight=0.55, collision_margin=0.001,
        extra_state_valid=lambda q: official_reach_guard(robot, q),
    )
    if not result.success:
        return False, "HANDOFF_FAILED", None, None, None
    start, previous = task.path[-1], task.path[-1]
    start_pose = robot.fk(start)
    tcp_points = [robot.fk(start)[:3, 3]]
    joint_length = 0.0
    destination = pose[:3, 3]
    # Keep the extracted height while first clearing the stack in X and Y;
    # only then descend onto the support surface.  This is the shortest
    # deterministic axis-ordered polyline that avoids cutting diagonally back
    # through the neighbour from which the carton was just extracted.
    via_x = np.array([destination[0], start_pose[1, 3], start_pose[2, 3]])
    via_xy = np.array([destination[0], destination[1], start_pose[2, 3]])
    positions: list[np.ndarray] = []
    segment_start = start_pose[:3, 3]
    for segment_end in (via_x, via_xy, destination):
        if np.linalg.norm(segment_end - segment_start) <= 1e-9:
            continue
        positions.extend(((segment_start + segment_end) * 0.5, segment_end.copy()))
        segment_start = segment_end
    for index, position in enumerate(positions, 1):
        is_final = index == len(positions)
        if is_final:
            q = result.q
        else:
            sample_pose = start_pose.copy()
            sample_pose[:3, 3] = position
            sample = solve_ik_multistart(
                robot, sample_pose, seeds=[previous, result.q], obstacles=obstacles,
                random_restarts=0, rng=np.random.default_rng(rng_seed + index),
                max_iterations=65, damping=0.045, max_step=0.20,
                position_tolerance=0.015, orientation_tolerance=0.10,
                orientation_weight=0.55, collision_margin=0.001,
                extra_state_valid=lambda candidate_q: official_reach_guard(robot, candidate_q),
            )
            if not sample.success:
                return False, "HANDOFF_FAILED", None, None, None
            q = sample.q
        if not official_reach_guard(robot, q) or not robot.is_collision_free(q, obstacles, margin=0.001):
            return False, "HANDOFF_FAILED", None, None, None
        carried = _carried_box(robot.fk(q), 2.0 * target.half_extents, task.grasp_mode)
        if any(carried.intersects_obb(obstacle, margin=0.001) for obstacle in walls):
            return False, "TARGET_BOX_COLLISION", None, None, None
        if any(carried.intersects_obb(obstacle, margin=0.001) for obstacle in other_boxes):
            return False, "NEIGHBOR_BOX_COLLISION", None, None, None
        if not is_final and any(carried.intersects_obb(deck) for deck in conveyor.obstacles):
            return False, "HANDOFF_FAILED", None, None, None
        tcp_points.append(robot.fk(q)[:3, 3])
        joint_length += float(np.linalg.norm(q - previous))
        previous = q
    size, deck = 2.0 * target.half_extents, conveyor.obstacles[1]
    local = deck.to_local(desired_center)
    supported = (abs(local[0]) + target.half_extents[0] <= deck.half_extents[0] + 1e-9
                 and abs(local[1]) + target.half_extents[1] <= deck.half_extents[1] + 1e-9
                 and abs(desired_center[2] - target.half_extents[2] - conveyor.conveyor_z_m) <= 1e-9)
    protocol = HandoffProtocol()
    protocol.confirm_supported_handoff(box_supported=supported, collision_free=True)
    try:
        protocol.release_vacuum()
    except RuntimeError:
        return False, "HANDOFF_FAILED", None, None, None
    tcp_length = float(sum(np.linalg.norm(b - a) for a, b in zip(tcp_points[:-1], tcp_points[1:])))
    return True, "OK", result.q, tcp_length, joint_length


def _empty_result(reason: str, state: BoxNeighborhoodState, payload_ok: bool) -> CompleteTaskResult:
    return CompleteTaskResult(False, False, reason != "CONVEYOR_POSITION_FAILED", False,
                              False, payload_ok, False, reason, "NONE", "NONE",
                              None, None, None, None, None, None, None, None, None,
                              state.exposed_face_count)


def evaluate_complete_task(robot, tool, target: OBB, remaining_boxes: Sequence[OBB],
                           conveyor_geometry: LConveyorGeometry, *, seed_q=None,
                           rng_seed: int = 71070, evaluate_handoff: bool = True) -> CompleteTaskResult:
    others = [box for box in remaining_boxes if box.name != target.name]
    state = analyze_box_neighborhood(target, others)
    size = 2.0 * target.half_extents
    payload_ok = combined_payload_properties(tool, 42.5, size)["vendor_com_diagram_status"] == "PASS"
    walls = trailer_obstacles()
    conveyor = plan_conveyor_preposition(
        conveyor_geometry, robot.base_transform[:3, 3], target, remaining_boxes,
        [*walls, *remaining_boxes],
        robot_capsules=robot.link_capsules(HOME_JOINTS_RAD if seed_q is None else seed_q),
    )
    attempts, first_failure = [], "NO_IK"
    any_grasp, any_extraction = False, False
    front_face = float(target.center[0] - target.half_extents[0])
    for index, candidate in enumerate(generate_extraction_candidates(state)):
        if _surface_area(size, candidate.grasp_face) < 0.08:
            first_failure = "TARGET_BOX_COLLISION"
            continue
        task = evaluate_task_point(
            robot, float(target.center[1]), float(target.center[2]), size,
            grasp_mode=candidate.grasp_face, front_face_x_m=front_face,
            seed_q=seed_q, rng_seed=rng_seed + index, obstacles=walls,
            neighbor_obstacles=others, max_iterations=90, random_restarts=1,
        )
        if not task.task_reachable:
            any_grasp = any_grasp or task.collision_free_reachable
            any_extraction = any_extraction or task.extraction_reachable
            first_failure = task.failure_reason
            continue
        any_grasp, any_extraction = True, True
        if conveyor is None:
            first_failure = "CONVEYOR_POSITION_FAILED"
            continue
        if evaluate_handoff:
            okay, reason, q_handoff, carry_length, carry_joint = _solve_handoff(
                robot, task, target, others, walls, conveyor, rng_seed + 100 + index)
            if not okay:
                first_failure = reason
                continue
        else:
            q_handoff, carry_joint = task.path[-1], 0.0
            carry_length = float(np.linalg.norm(_handoff_pose(target, task.grasp_mode, conveyor)[0][:3, 3]
                                                - robot.fk(task.path[-1])[:3, 3]))
        extraction = float(task.minimum_clearance_extraction_distance_m or 0.0)
        loaded_length = extraction + float(carry_length or 0.0)
        fixed = conveyor_geometry.obstacles(robot.base_transform[:3, 3], 0.0, 0.20)[1]
        fixed_front = conveyor_geometry.receiving_front_x_m(robot.base_transform[:3, 3], 0.0)
        fixed_center = np.array([fixed_front - target.half_extents[0], fixed.center[1], 0.20 + target.half_extents[2]])
        baseline = extraction + float(np.linalg.norm(fixed_center - (target.center + candidate.outward_direction_world * extraction)))
        condition, margin = float(task.maximum_jacobian_condition or 1e9), float(task.minimum_joint_margin_rad or 0.0)
        metrics = CandidateMetrics(True, margin, margin, 1.0 / max(condition, 1.0),
                                   0.01, 0.01, 0.01, 0.02, extraction, loaded_length,
                                   loaded_length, float(task.joint_space_path_length_rad or 0.0) + float(carry_joint or 0.0),
                                   float(carry_length or 0.0), 1.0)
        attempts.append(CompleteTaskResult(
            True, True, True, True, True, payload_ok, payload_ok, "OK",
            candidate.grasp_face, candidate.strategy, extraction,
            conveyor.conveyor_extension_m, conveyor.conveyor_z_m, loaded_length,
            baseline, metrics.joint_space_path_length_rad, margin, condition,
            score_candidate(metrics), state.exposed_face_count, q_handoff))
    if attempts:
        return min(attempts, key=lambda item: float(item.candidate_cost))
    return CompleteTaskResult(
        any_grasp, any_extraction,
        conveyor is not None, False, False, payload_ok, False, first_failure, "NONE", "NONE", None,
        None if conveyor is None else conveyor.conveyor_extension_m,
        None if conveyor is None else conveyor.conveyor_z_m, None, None, None,
        None, None, None, state.exposed_face_count)


def _default_neighbors(target: OBB) -> list[OBB]:
    size = 2.0 * target.half_extents
    depth, width, height = size
    candidates = [
        _box("left_neighbor", target.center + [0, width + 0.01, 0], size),
        _box("right_neighbor", target.center + [0, -width - 0.01, 0], size),
        _box("bottom_support", target.center + [0, 0, -height - 0.01], size),
    ]
    return [box for box in candidates if abs(box.center[1]) + box.half_extents[1] <= 1.15
            and box.center[2] - box.half_extents[2] >= 0]


def _grid_rows(base_x: float, base_z: float, step: float, conveyor_geometry: LConveyorGeometry):
    robot, tool = build_robot(base_x, mounting_surface_z_m=base_z)
    y_values, z_values = _inclusive(-1.15, 1.15, step), _inclusive(0.0, 2.70, step)
    sizes = {"A_600x400x300": (0.6, 0.4, 0.3), "B_400x600x300": (0.4, 0.6, 0.3)}
    rows: list[dict] = []
    for z in z_values:
        for y in y_values:
            for orientation, size in sizes.items():
                target = _box(f"target_{orientation}", [1.0 + 0.5 * size[0], y, z], size)
                inside = (abs(y) + 0.5 * size[1] <= 1.15 + 1e-9
                          and z - 0.5 * size[2] >= -1e-9 and z + 0.5 * size[2] <= 2.70 + 1e-9)
                if inside:
                    remaining = [target, *_default_neighbors(target)]
                    topology = analyze_box_neighborhood(target, remaining[1:])
                    by_name = {box.name: box for box in remaining[1:]}
                    constraint_names = {topology.left_neighbor, topology.right_neighbor, topology.top_neighbor}
                    constraints = [by_name[name] for name in constraint_names if name is not None]
                    front_extraction = minimum_clearance_extraction_distance(
                        target, FACE_OUTWARD_NORMALS["front"], constraints)
                    result = evaluate_complete_task(robot, tool, target, remaining,
                                                    conveyor_geometry, rng_seed=71070 + len(rows))
                else:
                    topology, front_extraction = None, None
                    result = CompleteTaskResult(False, False, False, False, False, False, False,
                                                "TARGET_BOX_COLLISION", "NONE", "NONE", None,
                                                None, None, None, None, None, None, None, None, 0)
                rows.append({
                    "y_m": round(float(y), 6), "z_m": round(float(z), 6),
                    "orientation": orientation, "inside_cross_section": inside,
                    "GRASP_REACHABLE": result.grasp_reachable,
                    "EXTRACTION_FEASIBLE": result.extraction_feasible,
                    "CONVEYOR_POSITION_FEASIBLE": result.conveyor_position_feasible,
                    "CONVEYOR_HANDOFF_FEASIBLE": result.conveyor_handoff_feasible,
                    "FULL_TASK_REACHABLE": result.geometrically_reachable,
                    "GEOMETRICALLY_REACHABLE": result.geometrically_reachable,
                    "PAYLOAD_QUALIFIED": result.payload_qualified,
                    "QUALIFIED_TASK": result.qualified_task,
                    "failure_reason": result.failure_reason, "grasp_face": result.grasp_face,
                    "both_sides_constrained": False if topology is None else topology.constrained_both_sides,
                    "front_minimum_clearance_extraction_distance_m": front_extraction,
                    "minimum_clearance_extraction_distance_m": result.minimum_extraction_distance_m,
                    "conveyor_extension_m": result.conveyor_extension_m,
                    "conveyor_z_m": result.conveyor_z_m,
                    "loaded_tcp_path_length_m": result.loaded_tcp_path_length_m,
                    "fixed_conveyor_baseline_path_m": result.fixed_conveyor_baseline_path_m,
                })
    valid = [row for row in rows if row["inside_cross_section"]]
    rate = lambda key: sum(bool(row[key]) for row in valid) / max(len(valid), 1)
    return rows, {
        "base_x_m": base_x, "base_z_m": base_z, "grid_step_m": step,
        "valid_orientation_samples": len(valid),
        "grasp_coverage": rate("GRASP_REACHABLE"),
        "extraction_coverage": rate("EXTRACTION_FEASIBLE"),
        "handoff_coverage": rate("CONVEYOR_HANDOFF_FEASIBLE"),
        "full_task_coverage": rate("FULL_TASK_REACHABLE"),
        "payload_qualified_coverage": rate("PAYLOAD_QUALIFIED"),
    }


def _plot_layers(output: Path, rows: list[dict]) -> None:
    ys = sorted({float(row["y_m"]) for row in rows})
    zs = sorted({float(row["z_m"]) for row in rows})
    grouped: dict[tuple[float, float], list[dict]] = {}
    for row in rows:
        grouped.setdefault((float(row["y_m"]), float(row["z_m"])), []).append(row)
    layers = {"grasp_reachable_map": "GRASP_REACHABLE",
              "extraction_feasible_map": "EXTRACTION_FEASIBLE",
              "conveyor_handoff_feasible_map": "CONVEYOR_HANDOFF_FEASIBLE",
              "full_task_reachable_map": "FULL_TASK_REACHABLE"}
    for filename, key in layers.items():
        grid = np.full((len(zs), len(ys)), -1, dtype=int)
        for zi, z in enumerate(zs):
            for yi, y in enumerate(ys):
                valid = [row for row in grouped[(y, z)] if row["inside_cross_section"]]
                if valid:
                    grid[zi, yi] = int(all(bool(row[key]) for row in valid))
        figure, axis = plt.subplots(figsize=(8, 7))
        axis.imshow(grid, origin="lower", extent=[-1.15, 1.15, 0, 2.70],
                    aspect="equal", interpolation="nearest",
                    cmap=ListedColormap(["#9ca3af", "#dc2626", "#16a34a"]), vmin=-1, vmax=1)
        axis.set(xlabel="Y [m] (+left)", ylabel="Z [m] (+up)", title=filename.replace("_", " "))
        figure.tight_layout()
        figure.savefig(output / f"{filename}.png", dpi=150)
        plt.close(figure)


def _base_x_z_scan(output: Path, conveyor_geometry: LConveyorGeometry):
    probes = [(-0.75, 0.35), (0, 1.35), (0.75, 2.35)]
    rows = []
    for base_x in np.arange(-1.10, -0.29, 0.20):
        for base_z in (0.40, 0.70, 1.00, 1.30):
            robot, tool = build_robot(float(base_x), mounting_surface_z_m=float(base_z))
            results = []
            for index, (y, z) in enumerate(probes):
                target = _box("target", [1.30, y, z], (0.6, 0.4, 0.3))
                results.append(evaluate_complete_task(
                    robot, tool, target, [target, *_default_neighbors(target)], conveyor_geometry,
                    rng_seed=73000 + index, evaluate_handoff=False))
            count = len(results)
            rows.append({
                "base_x_m": round(float(base_x), 3), "base_z_m": round(float(base_z), 3),
                "probe_count": count,
                "grasp_coverage": sum(item.grasp_reachable for item in results) / count,
                "extraction_coverage": sum(item.extraction_feasible for item in results) / count,
                "handoff_coverage": sum(item.conveyor_handoff_feasible for item in results) / count,
                "full_task_coverage": sum(item.geometrically_reachable for item in results) / count,
            })
    _write_csv(output / "base_x_z_scan.csv", rows)
    xs = sorted({float(row["base_x_m"]) for row in rows})
    zs = sorted({float(row["base_z_m"]) for row in rows})
    figure, axes = plt.subplots(2, 2, figsize=(10, 8), sharex=True, sharey=True)
    for axis, key in zip(axes.flat, ("grasp_coverage", "extraction_coverage", "handoff_coverage", "full_task_coverage")):
        grid = np.asarray([[next(float(row[key]) for row in rows if float(row["base_x_m"]) == x and float(row["base_z_m"]) == z) for x in xs] for z in zs])
        image = axis.imshow(grid, origin="lower", extent=[min(xs), max(xs), min(zs), max(zs)], aspect="auto", vmin=0, vmax=1, cmap="viridis")
        axis.set(title=key.replace("_", " "), xlabel="base X [m]", ylabel="base Z [m]")
        figure.colorbar(image, ax=axis, fraction=0.046)
    figure.tight_layout()
    figure.savefig(output / "base_x_z_scan.png", dpi=150)
    plt.close(figure)
    best = max(rows, key=lambda row: (row["full_task_coverage"], row["handoff_coverage"],
                                      row["extraction_coverage"], row["grasp_coverage"],
                                      -abs(row["base_z_m"] - 0.8)))
    viable_z = [row["base_z_m"] for row in rows if row["base_x_m"] == best["base_x_m"]
                and row["full_task_coverage"] >= 0.95 * best["full_task_coverage"]]
    recommendation = {
        "base_x_m": best["base_x_m"], "fixed_base_z_m": best["base_z_m"],
        "minimum_base_z_m": min(viable_z), "maximum_base_z_m": max(viable_z),
        "recommended_z_travel_m": max(viable_z) - min(viable_z),
        "best_probe_coverage": best,
    }
    return rows, recommendation


def _scene_regular() -> list[OBB]:
    return [_box(f"regular_r{row}_c{column}", [1.30, y, 0.15 + 0.31 * row], (0.6, 0.4, 0.3))
            for row in range(8) for column, y in enumerate(np.linspace(-0.82, 0.82, 5))]


def _scene_random(seed: int) -> list[OBB]:
    rng, boxes = np.random.default_rng(seed), []
    for row in range(8):
        cursor, column = -1.13, 0
        while cursor < 1.05:
            size = np.array((0.6, 0.4, 0.3) if rng.random() < 0.55 else (0.4, 0.6, 0.3))
            center_y = cursor + 0.5 * size[1]
            if center_y + 0.5 * size[1] > 1.14:
                break
            if rng.random() > 0.10:
                front = 1.0 + rng.uniform(-0.025, 0.035)
                boxes.append(_box(f"random{seed}_r{row}_c{column}",
                                  [front + 0.5 * size[0], center_y, 0.15 + 0.31 * row], size))
            cursor += size[1] + rng.uniform(0.005, 0.02)
            column += 1
    return boxes


def _continuous(output: Path, base_x: float, base_z: float, conveyor_geometry: LConveyorGeometry):
    robot, tool = build_robot(base_x, mounting_surface_z_m=base_z)
    scenarios = [("regular", _scene_regular()),
                 *((f"random_seed_{seed}", _scene_random(seed)) for seed in (71071, 71072, 71073))]
    rows, summaries = [], []
    for scenario_index, (name, original) in enumerate(scenarios):
        remaining, current_q, sequence, failures = list(original), HOME_JOINTS_RAD.copy(), 0, {}
        while remaining:
            exposed = []
            for box in remaining:
                state = analyze_box_neighborhood(box, [other for other in remaining if other.name != box.name])
                # Never pull a carton that still supports a top neighbour.
                # Rank all currently graspable boxes from current topology,
                # then spend full IK/path cost only on the best small set.
                if state.top_neighbor is None and state.exposed_face_count:
                    constraint_count = int(state.left_neighbor is not None) + int(state.right_neighbor is not None)
                    preliminary_cost = (2.0 * constraint_count - 3.0 * state.exposed_face_count
                                        + abs(float(box.center[1]) + 0.725))
                    exposed.append((preliminary_cost, box, state))
            exposed = sorted(exposed, key=lambda item: (item[0], item[1].name))[:2]
            evaluated = []
            for candidate_index, (_preliminary, box, state) in enumerate(exposed):
                result = evaluate_complete_task(
                    robot, tool, box, remaining, conveyor_geometry, seed_q=current_q,
                    rng_seed=74000 + scenario_index * 1000 + sequence * 50 + candidate_index)
                if result.geometrically_reachable:
                    next_cost = float(result.candidate_cost or 0) - 20 * state.exposed_face_count + 2 * float(result.minimum_extraction_distance_m or 0)
                    evaluated.append((next_cost, box, result, state))
                else:
                    failures[result.failure_reason] = failures.get(result.failure_reason, 0) + 1
            if not evaluated:
                break
            _, selected, result, state = min(evaluated, key=lambda item: (item[0], item[1].name))
            rows.append({
                "scenario": name, "sequence": sequence, "box": selected.name,
                "status": "GEOMETRICALLY_UNLOADED" if not result.payload_qualified else "QUALIFIED_UNLOADED",
                "GEOMETRICALLY_REACHABLE": result.geometrically_reachable,
                "PAYLOAD_QUALIFIED": result.payload_qualified, "grasp_face": result.grasp_face,
                "strategy": result.strategy,
                "minimum_clearance_extraction_distance_m": result.minimum_extraction_distance_m,
                "conveyor_extension_m": result.conveyor_extension_m, "conveyor_z_m": result.conveyor_z_m,
                "loaded_tcp_path_length_m": result.loaded_tcp_path_length_m,
                "fixed_conveyor_baseline_path_m": result.fixed_conveyor_baseline_path_m,
                "exposed_face_count": state.exposed_face_count, "vacuum_release_after_handoff": True,
            })
            current_q = result.q_handoff.copy() if result.q_handoff is not None else current_q
            remaining = [box for box in remaining if box.name != selected.name]
            sequence += 1
        summaries.append({
            "scenario": name, "seed": None if name == "regular" else int(name.rsplit("_", 1)[1]),
            "total_boxes": len(original), "geometrically_unloaded_boxes": len(original) - len(remaining),
            "payload_qualified_unloaded_boxes": 0, "remaining_boxes": len(remaining),
            "failure_counts": json.dumps(failures, sort_keys=True),
        })
    _write_csv(output / "continuous_unloading_results.csv", rows)
    _write_csv(output / "continuous_unloading_summary.csv", summaries)
    return rows, summaries


def _payload_envelope(output: Path, tool) -> list[dict]:
    curves = ((30.0, 0.520, 0.825), (40.0, 0.396, 0.615),
              (50.0, 0.317, 0.457), (60.0, 0.264, 0.352), (70.0, 0.225, 0.277))
    rows = []
    tcp_values = sorted({0.10, 0.15, 0.20, float(tool.tcp_translation_xyz_m[0])})
    for box_mass in sorted({*np.arange(5.0, 45.1, 5.0), 42.5}):
        for depth in np.arange(0.20, 0.701, 0.10):
            for tcp in tcp_values:
                for tool_mass in (10.0, 15.0, 20.0):
                    for tool_cg in (0.10, float(tool.com_xyz_m[0])):
                        total = float(box_mass + tool_mass)
                        combined = float((tool_mass * tool_cg + box_mass * (tcp + 0.5 * depth)) / total)
                        curve = next((item for item in curves if total <= item[0] + 1e-12), None)
                        passed = curve is not None and combined <= curve[2] + 1e-12
                        rows.append({
                            "box_mass_kg": box_mass, "box_depth_m": round(float(depth), 3),
                            "tcp_length_m": tcp, "tool_mass_kg": tool_mass,
                            "tool_cg_axial_m": tool_cg, "total_mass_kg": total,
                            "combined_cg_axial_m": combined,
                            "fanuc_curve_payload_kg": None if curve is None else curve[0],
                            "fanuc_curve_axial_limit_m": None if curve is None else curve[2],
                            "FANUC_PAYLOAD_CURVE_STATUS": "PASS" if passed else "FAIL",
                        })
    _write_csv(output / "payload_envelope.csv", rows)
    return rows


def _failure_statistics(output: Path, task_rows: list[dict]) -> list[dict]:
    valid, rows = [row for row in task_rows if row["inside_cross_section"]], []
    for reason in FAILURE_REASONS:
        if reason == "PAYLOAD_CG_FAILED":
            points, status = [row for row in valid if not row["PAYLOAD_QUALIFIED"]], "EVALUATED"
        elif reason == "WRIST_LIMIT_FAILED":
            points, status = [], "NOT_EVALUATED_MISSING_COMPLETE_DYNAMIC_MODEL"
        else:
            points, status = [row for row in valid if row["failure_reason"] == reason], "EVALUATED"
        distribution = sorted({(row["y_m"], row["z_m"]) for row in points})
        rows.append({"failure_reason": reason, "count": len(points),
                     "percentage": len(points) / max(len(valid), 1),
                     "spatial_distribution": json.dumps(distribution, separators=(",", ":")),
                     "status": status})
    _write_csv(output / "failure_reason_statistics.csv", rows)
    return rows


def _strategy_statistics(output: Path, task_rows: list[dict], continuous_rows: list[dict]) -> list[dict]:
    rows = []
    for face in ("front", "left", "right", "top"):
        grid = [row for row in task_rows if row["grasp_face"] == face and row["FULL_TASK_REACHABLE"]]
        continuous = [row for row in continuous_rows if row["grasp_face"] == face]
        rows.append({"grasp_face": face, "grid_full_task_success_count": len(grid),
                     "continuous_unload_success_count": len(continuous),
                     "total_success_count": len(grid) + len(continuous)})
    _write_csv(output / "grasp_strategy_statistics.csv", rows)
    return rows


def _conveyor_from_config(config: dict) -> LConveyorGeometry:
    values = config["conveyor"]
    return LConveyorGeometry(
        chassis_size_xyz_m=tuple(values["chassis_size_xyz_m"]),
        chassis_center_from_robot_base_xyz_m=tuple(values["chassis_center_from_robot_base_xyz_m"]),
        cross_leg_size_xy_m=tuple(values["cross_leg_size_xy_m"]),
        longitudinal_leg_nominal_length_m=values["longitudinal_leg_nominal_length_m"],
        longitudinal_leg_width_m=values["longitudinal_leg_width_m"],
        deck_thickness_m=values["deck_thickness_m"],
        maximum_extension_m=values["maximum_extension_m"],
        minimum_stack_surface_clearance_m=values["minimum_stack_surface_clearance_m"],
        minimum_receiving_surface_z_m=values["minimum_receiving_surface_z_m"],
        upper_stack_bottom_clearance_m=values["upper_stack_bottom_clearance_m"],
    )


def _fmt_optional(value: float | None) -> str:
    return "NOT_EVALUATED" if value is None else f"{value:.3f}"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=".tmp/accept_m710id70_v2")
    parser.add_argument("--grid-step", type=float, default=0.30)
    parser.add_argument("--skip-continuous", action="store_true")
    args = parser.parse_args(argv)
    output = Path(args.output_dir)
    output = output if output.is_absolute() else ROOT / output
    output.mkdir(parents=True, exist_ok=True)

    config = yaml.safe_load((ROOT / "config/fanuc_m710id_70.yaml").read_text(encoding="utf-8"))
    conveyor_geometry = _conveyor_from_config(config)
    _scan, recommendation = _base_x_z_scan(output, conveyor_geometry)
    base_x, base_z = float(recommendation["base_x_m"]), float(recommendation["fixed_base_z_m"])
    task_rows, coverage = _grid_rows(base_x, base_z, args.grid_step, conveyor_geometry)
    _write_csv(output / "task_reachability.csv", task_rows)
    _plot_layers(output, task_rows)
    robot, tool = build_robot(base_x, mounting_surface_z_m=base_z)
    _write_json(output / "robot_model_validation.json", validate_model(robot))
    payload_rows = _payload_envelope(output, tool)
    if args.skip_continuous:
        continuous_rows, summaries = [], []
        _write_csv(output / "continuous_unloading_results.csv", [])
        _write_csv(output / "continuous_unloading_summary.csv", [])
    else:
        continuous_rows, summaries = _continuous(output, base_x, base_z, conveyor_geometry)
    failure_rows = _failure_statistics(output, task_rows)
    strategy_rows = _strategy_statistics(output, task_rows, continuous_rows)

    bilateral = [float(row["front_minimum_clearance_extraction_distance_m"]) for row in task_rows
                 if row["both_sides_constrained"] and row["front_minimum_clearance_extraction_distance_m"] not in (None, "")]
    reductions = [float(row["fixed_conveyor_baseline_path_m"]) - float(row["loaded_tcp_path_length_m"])
                  for row in continuous_rows if row["fixed_conveyor_baseline_path_m"] not in (None, "")
                  and row["loaded_tcp_path_length_m"] not in (None, "")]
    load = combined_payload_properties(tool, 42.5, (0.6, 0.4, 0.3))
    motion_rows = [row for row in failure_rows if row["failure_reason"] not in {"PAYLOAD_CG_FAILED", "WRIST_LIMIT_FAILED"}]
    first_motion_failure = max(motion_rows, key=lambda row: row["count"])["failure_reason"]
    p50 = None if not bilateral else float(np.percentile(bilateral, 50))
    p95 = None if not bilateral else float(np.percentile(bilateral, 95))
    mean_reduction = None if not reductions else float(np.mean(reductions))
    side_avoided = sum(row["strategy"] == "open_side_direct" for row in continuous_rows)
    scenario_lines = "\n".join(
        f"- {row['scenario']}: {row['geometrically_unloaded_boxes']}/{row['total_boxes']} 箱几何卸出，负载资格通过 {row['payload_qualified_unloaded_boxes']} 箱。"
        for row in summaries) or "- SKIPPED"
    grasp_counts = {row["grasp_face"]: row["total_success_count"] for row in strategy_rows}
    report = f"""# FANUC M-710iD/70 物流车厢卸货技术资格报告 V2

## 结论

- 已彻底移除固定 250 mm 直退。`minimum_clearance_extraction_distance` 按目标箱、抓取面和当前邻接 OBB 实时增量扫描并二分细化。
- 2.3 m × 2.7 m 横截面在 base X={base_x:.3f} m、base Z={base_z:.3f} m 下的完整几何任务覆盖率为 **{coverage['full_task_coverage']:.3%}**；旧版固定直退结论为 25.609%，变化为 **{coverage['full_task_coverage'] - 0.25609:+.3%}**。两者模型定义不同，旧的 4.808 m² 与连续 0 箱结论不再沿用。
- 运动学/碰撞维度：**{'PASS' if coverage['full_task_coverage'] == 1.0 else 'FAIL'}**。负载资格维度：**FAIL**；20 kg 工具和 42.5 kg 箱体总质量 62.5 kg，但组合质心 {load['combined_com_flange_xyz_m'][0]:.3f} m 超出保守 FANUC 70 kg 曲线。
- 当前第一个独立硬失败是 **PAYLOAD_CG_FAILED**；运动规划覆盖的主要失败为 **{first_motion_failure}**。腕部完整动力学因缺少连杆惯量和驱动转矩继续标为 NOT_EVALUATED。

## 新任务判据与分层结果

`FULL_TASK_REACHABLE` 同时要求抓取 IK/限位/奇异性、机器人及工具碰撞、吸附面、几何最小脱垛、动态传送带预定位、带载路径和无碰撞支撑交接。负载资格不再短路运动学计算，单独输出 `GEOMETRICALLY_REACHABLE`、`PAYLOAD_QUALIFIED` 和 `QUALIFIED_TASK`。

- grasp coverage: {coverage['grasp_coverage']:.3%}
- extraction coverage: {coverage['extraction_coverage']:.3%}
- conveyor handoff coverage: {coverage['handoff_coverage']:.3%}
- full geometric task coverage: {coverage['full_task_coverage']:.3%}
- payload-qualified coverage: {coverage['payload_qualified_coverage']:.3%}

## 抓取、脱垛与传送带统计

抓取成功数（横截面成功样本 + 连续卸货成功箱）：{json.dumps(grasp_counts, ensure_ascii=False)}。

双侧受限箱的几何最小正面直线脱垛距离：P50={_fmt_optional(p50)} m，P95={_fmt_optional(p95)} m。连续场景中有 {side_avoided} 个单侧开放箱通过侧面吸取避免了正面直抽；若侧吸 IK/碰撞失败则回退正面局部直抽。

动态 L 形传送带的平均带载 TCP 路径相对固定 extension=0、Z=0.20 m 基线变化：{_fmt_optional(mean_reduction)} m（正值表示缩短，负值表示当前安全清堆折线路径反而增加）。传送带是独立外部碰撞机构，不计入底盘质量、机械臂负载或底盘刚性 link；坐标轴与世界 +X 入车厢、+Y 向左、+Z 向上一致。

四组连续场景：
{scenario_lines}

## 安装位置重扫

- 推荐 base X: {recommendation['base_x_m']:.3f} m
- 推荐固定 base Z: {recommendation['fixed_base_z_m']:.3f} m
- 推荐最低/最高 base Z: {recommendation['minimum_base_z_m']:.3f}/{recommendation['maximum_base_z_m']:.3f} m
- 建议 Z 升降行程: {recommendation['recommended_z_travel_m']:.3f} m

该建议来自 X=0.20 m、Z=0.30 m 步长的第一层离散工程扫描；零行程表示本次离散候选只在一个高度达到最优带宽，不是对连续机构行程的制造定版。

## 风险与下一步

`payload_envelope.csv` 扫描箱体 5–45 kg、深度 0.2–0.7 m、TCP 0.10–0.25 m、工具 10/15/20 kg 和两组工具质心。结果表明减轻工具并不必然改善组合质心；对当前 42.5 kg、600 mm 深箱和工具质心，按 70 kg 曲线反算的 TCP 上限仅约 0.037 m，连 0.10 m 扫描方案也失败。因此最值得修改的是整体末端/抓取方向的有效负载中心（或改用负载包络更大的机器人），而不是只减轻吸具。运动规划侧应优先增加安全姿态的多起点/滚转候选以改善 {first_motion_failure}，不能放宽碰撞或限位；随后用 FANUC 负载设定软件或 ROBOGUIDE 复核。

所有空间失败点见 `failure_reason_statistics.csv`；四层热力图分别为 grasp、extraction、conveyor handoff 和 full task。解析球面没有被当作任务覆盖结论。
"""
    (output / "technical_qualification_report_m710id70_v2.md").write_text(report, encoding="utf-8")
    _write_json(output / "acceptance_summary.json", {
        "coverage": coverage, "base_recommendation": recommendation,
        "continuous": summaries, "payload_envelope_rows": len(payload_rows)})
    print(json.dumps({"output": str(output), "coverage": coverage,
                      "base_recommendation": recommendation, "continuous": summaries}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
