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
    from unloading_sim.grasp import (
        _adaptive_destack_clearance,
        _carried_box_state_valid,
        _edge_collision_free,
        _in_trailer_orientation_valid,
        generate_suction_candidates,
        filter_suction_candidates_by_seal,
        filter_suction_candidates_by_tool_clearance,
        plan_pick,
        select_fast_suction_candidates,
    )
    from unloading_sim.ik import solve_ik_multistart
    from unloading_sim.identity import build_scene_cache_identity, normalize_robot_model_id, validate_cache_identity
    from unloading_sim.perception import detect_carton_obbs, fixed_conveyor_place_pose
    from unloading_sim.scene import TrailerScene, load_scene_config
    from unloading_sim.support import SupportRelationGraph
    from unloading_sim.timing import time_parameterize_segments
else:
    from .demo import build_robot
    from .geometry import OBB
    from .grasp import (
        _adaptive_destack_clearance,
        _carried_box_state_valid,
        _edge_collision_free,
        _in_trailer_orientation_valid,
        generate_suction_candidates,
        filter_suction_candidates_by_seal,
        filter_suction_candidates_by_tool_clearance,
        plan_pick,
        select_fast_suction_candidates,
    )
    from .ik import solve_ik_multistart
    from .identity import build_scene_cache_identity, normalize_robot_model_id, validate_cache_identity
    from .perception import detect_carton_obbs, fixed_conveyor_place_pose
    from .scene import TrailerScene, load_scene_config
    from .support import SupportRelationGraph
    from .timing import time_parameterize_segments


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


def exposed_carton_order(
    scene: TrailerScene,
    face_modes: Sequence[str] = ("front", "side", "top"),
) -> list[str]:
    """Return stable cartons with at least one requested face exposed.

    Cartons supporting another carton are deferred. Among the remaining top
    layer, a carton may be reached from above, the trailer door, or either
    side. This cheap O(n^2) OBB-axis check avoids discarding valid top and side
    grasps before collision-aware planning evaluates them.
    """
    graph = SupportRelationGraph.build(scene.cartons)
    return graph.removable_cartons(face_modes=face_modes)


def remove_carton(scene: TrailerScene, name: str) -> None:
    scene.cartons = [carton for carton in scene.cartons if carton.name != name]


def restrict_to_highest_active_layer(
    scene: TrailerScene,
    names: Sequence[str],
    *,
    tolerance_m: float = 0.008,
) -> list[str]:
    """Keep the highest currently removable layer without changing its order.

    A cleared column can expose a lower carton while other cartons remain on
    the active layer. Allowing that lower carton to compete on IK yield can
    create a vertical trench instead of unloading the row from its centre
    toward both sides. The tolerance only groups small detection/packing
    height variation; it is not a prescribed motion waypoint.
    """
    if not np.isfinite(tolerance_m) or tolerance_m < 0.0:
        raise ValueError("active-layer tolerance must be finite and non-negative")
    ordered = list(names)
    if not ordered:
        return []
    top_heights = {
        name: float(scene.carton(name).center[2] + scene.carton(name).half_extents[2])
        for name in ordered
    }
    highest = max(top_heights.values())
    return [name for name in ordered if top_heights[name] >= highest - tolerance_m]


def restrict_to_nearest_active_depth(
    scene: TrailerScene,
    names: Sequence[str],
    *,
    tolerance_m: float = 0.008,
) -> list[str]:
    """Keep the removable work face nearest the open trailer door (+X inward)."""
    if not np.isfinite(tolerance_m) or tolerance_m < 0.0:
        raise ValueError("active-depth tolerance must be finite and non-negative")
    ordered = list(names)
    if not ordered:
        return []
    front_faces = {
        name: float(np.min(scene.carton(name).corners()[:, 0])) for name in ordered
    }
    nearest = min(front_faces.values())
    return [name for name in ordered if front_faces[name] <= nearest + tolerance_m]


def restore_plan_prefix(
    scene: TrailerScene,
    robot_cfg: dict,
    resume_plan_path: str | Path,
    current_cache_identity: dict | None = None,
) -> tuple[list[dict], list[tuple[int, str, int, np.ndarray]], np.ndarray, np.ndarray | None]:
    """Restore a collision-validated prefix so planning can continue online.

    A prefix is accepted only for the same robot model, with unique carton
    targets that still exist in the initial scene and non-empty six-axis
    paths.  Applying it removes those cartons from the planning scene and
    restores the exact terminal joint and AMR dock state.
    """
    resume_plan_path = Path(resume_plan_path)
    with resume_plan_path.open("r", encoding="utf-8") as f:
        manifest = json.load(f)

    resume_robot = manifest.get("robot", {})
    if normalize_robot_model_id(resume_robot.get("model")) != normalize_robot_model_id(robot_cfg.get("model")):
        raise ValueError("resume plan robot model does not match current config")
    if current_cache_identity is not None and manifest.get("cache_identity") is not None:
        validate_cache_identity(manifest["cache_identity"], current_cache_identity)

    available_names = {carton.name for carton in scene.cartons}
    restored_segments: list[dict] = []
    trajectory_rows: list[tuple[int, str, int, np.ndarray]] = []
    restored_names: set[str] = set()
    current_q: np.ndarray | None = None
    current_dock: np.ndarray | None = None
    for pick_index, source_segment in enumerate(manifest.get("segments", [])):
        segment = copy.deepcopy(source_segment)
        target = str(segment.get("target", ""))
        if target not in available_names:
            raise ValueError(f"resume plan contains unknown carton: {target!r}")
        if target in restored_names:
            raise ValueError(f"resume plan contains duplicate carton: {target!r}")
        path = [np.asarray(q, dtype=float) for q in segment.get("path", [])]
        expected_dof = len(robot_cfg.get("home_joints", []))
        if not path or expected_dof <= 0 or any(q.shape != (expected_dof,) for q in path):
            raise ValueError(f"resume plan contains an invalid path for {target!r}")

        segment["pick_index"] = pick_index
        restored_segments.append(segment)
        trajectory_rows.extend((pick_index, target, local_index, q) for local_index, q in enumerate(path))
        restored_names.add(target)
        remove_carton(scene, target)
        current_q = path[-1].copy()
        dock_position = segment.get("amr_dock_position")
        if dock_position is not None:
            current_dock = np.asarray(dock_position, dtype=float)

    if current_q is None:
        raise ValueError("resume plan has no trajectory segments")
    return restored_segments, trajectory_rows, current_q, current_dock


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
    obstacle_configs = {
        str(obstacle["name"]): obstacle
        for obstacle in cfg["scene"].get("static_obstacles", [])
    }
    conveyor_name = str(amr_cfg.get("conveyor_name", "conveyor_deck"))
    conveyor = obstacle_configs[conveyor_name]
    front_reference_name = str(
        amr_cfg.get("conveyor_front_reference", conveyor_name)
    )
    mounted_centers = dict(amr_cfg.get("mounted_surface_centers", {}))
    if front_reference_name == conveyor_name:
        front_reference_center = conveyor_center
    else:
        if front_reference_name not in mounted_centers:
            raise ValueError("conveyor front reference must be fixed to the AMR")
        front_reference_center = np.asarray(
            mounted_centers[front_reference_name], dtype=float
        )
    front_reference = obstacle_configs[front_reference_name]
    conveyor_front_x = front_reference_center[0] + 0.5 * float(
        front_reference["size"][0]
    )
    robot_front_x = float(np.asarray(amr_cfg.get("robot_mount_position", [0.0, 0.0, 0.0]))[0])
    overhang = float(amr_cfg.get("conveyor_front_overhang", 0.20))
    size = np.asarray(amr_cfg.get("footprint_size"), dtype=float)
    platform_offset = np.asarray(amr_cfg.get("platform_center_offset"), dtype=float)
    amr_front_x = float(platform_offset[0] + 0.5 * size[0])
    if bool(amr_cfg.get("validate_conveyor_front_overhang", True)) and not np.isclose(
        conveyor_front_x, robot_front_x + overhang, atol=1e-9
    ):
        raise ValueError(
            f"AMR conveyor front must extend {overhang:.3f} m beyond the robot base front"
        )
    if bool(amr_cfg.get("require_conveyor_beyond_platform_front", True)) and (
        conveyor_front_x < amr_front_x - 1e-9
    ):
        raise ValueError("AMR conveyor front must not sit behind the AMR platform front")

    cross_name = amr_cfg.get("cross_conveyor_name")
    if cross_name is None:
        return
    cross_name = str(cross_name)
    if cross_name not in mounted_centers or conveyor_name not in mounted_centers:
        raise ValueError("both L-shaped conveyor centres must be fixed to the AMR")
    cross = obstacle_configs.get(cross_name)
    if cross is None:
        raise ValueError(f"cross conveyor obstacle is missing: {cross_name}")

    main_center = np.asarray(mounted_centers[conveyor_name], dtype=float)
    cross_center = np.asarray(mounted_centers[cross_name], dtype=float)
    main_size = np.asarray(conveyor["size"], dtype=float)
    cross_size = np.asarray(cross["size"], dtype=float)
    overlap = np.minimum(
        main_center[:2] + 0.5 * main_size[:2],
        cross_center[:2] + 0.5 * cross_size[:2],
    ) - np.maximum(
        main_center[:2] - 0.5 * main_size[:2],
        cross_center[:2] - 0.5 * cross_size[:2],
    )
    minimum_intersection = np.asarray(
        amr_cfg.get("minimum_conveyor_intersection_size_m", [0.0, 0.0]),
        dtype=float,
    )
    if minimum_intersection.shape != (2,) or np.any(minimum_intersection < 0.0):
        raise ValueError("minimum conveyor intersection size must contain two non-negative values")
    if np.any(overlap + 1e-9 < minimum_intersection):
        raise ValueError(
            "L-shaped conveyor intersection is too small: "
            f"measured={overlap.tolist()} required={minimum_intersection.tolist()}"
        )

    minimum_carton_gap = float(
        amr_cfg.get("minimum_conveyor_to_carton_clearance_m", 0.0)
    )
    if not np.isfinite(minimum_carton_gap) or minimum_carton_gap < 0.0:
        raise ValueError("minimum conveyor-to-carton clearance must be finite and non-negative")
    cartons = cfg["scene"].get("cartons", [])
    docks = amr_dock_positions(cfg)
    if cartons and docks:
        nearest_carton_x = min(
            float(item["center"][0]) - 0.5 * float(item["size"][0])
            for item in cartons
        )
        nearest_conveyor_x = max(
            float(dock[0] + center[0] + 0.5 * obstacle_configs[name]["size"][0])
            for dock in docks
            for name, center in (
                (conveyor_name, main_center),
                (cross_name, cross_center),
            )
        )
        measured_gap = nearest_carton_x - nearest_conveyor_x
        if measured_gap + 1e-9 < minimum_carton_gap:
            raise ValueError(
                "conveyor is too close to the carton work face: "
                f"measured={measured_gap:.3f}m required={minimum_carton_gap:.3f}m"
            )


def amr_dock_is_clear(scene: TrailerScene, cfg: dict, dock_position: np.ndarray) -> bool:
    amr_cfg = cfg.get("amr", {})
    # OBB.intersects_obb inflates both bodies by ``margin``. Split the desired
    # pairwise clearance across them so dock_clearance_m remains the physical
    # gap required between the platform and its surroundings.
    pairwise_clearance = float(amr_cfg.get("dock_clearance_m", 0.02))
    if not np.isfinite(pairwise_clearance) or pairwise_clearance < 0.0:
        raise ValueError("amr.dock_clearance_m must be finite and non-negative")
    obb_margin = 0.5 * pairwise_clearance
    size = np.asarray(amr_cfg.get("footprint_size", [1.60, 1.20, 0.30]), dtype=float)
    platform_offset = np.asarray(amr_cfg.get("platform_center_offset", [0.0, 0.0, size[2] / 2.0]), dtype=float)
    base = OBB(
        center=np.asarray(dock_position, dtype=float) + platform_offset,
        half_extents=size / 2.0,
        rotation=np.eye(3),
        name="amr_base",
        category="amr",
    )
    mounted_names = set(amr_cfg.get("mounted_surface_centers", {}))
    mounted_names.add(amr_cfg.get("conveyor_name", "conveyor_deck"))
    for obstacle in scene.obstacles:
        if obstacle.name == "trailer_floor" or obstacle.name in mounted_names:
            continue
        if base.intersects_obb(obstacle, margin=obb_margin):
            return False
    return all(not base.intersects_obb(carton, margin=obb_margin) for carton in scene.cartons)


def docked_robot_and_scene(scene: TrailerScene, cfg: dict, dock_position: np.ndarray):
    """Build a fixed-base planning scene for one AMR dock position."""
    local_cfg = copy.deepcopy(cfg)
    amr_cfg = local_cfg.get("amr", {})
    robot_mount = np.asarray(amr_cfg.get("robot_mount_position", [0.0, 0.0, 0.0]), dtype=float)
    local_cfg["robot"]["base_position"] = (np.asarray(dock_position, dtype=float) + robot_mount).tolist()
    conveyor_name = amr_cfg.get("conveyor_name", "conveyor_deck")
    mounted_centers = dict(amr_cfg.get("mounted_surface_centers", {}))
    mounted_centers.setdefault(conveyor_name, amr_cfg.get("conveyor_mount_center", [-1.45, -1.70, 0.37]))
    obstacles: list[OBB] = []
    for obstacle in scene.obstacles:
        if obstacle.name in mounted_centers:
            obstacles.append(
                OBB(
                    center=np.asarray(dock_position, dtype=float) + np.asarray(mounted_centers[obstacle.name], dtype=float),
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
    candidates = generate_suction_candidates(
        carton,
        robot.base_transform[:3, 3],
        cup_clearance=float(planning_cfg.get("vacuum_grasp_clearance_m", 0.015)),
        contact_grid_fractions=planning_cfg.get("suction_contact_grid_fractions"),
        face_modes=planning_cfg.get("grasp_face_modes", ["front", "side", "top"]),
    )
    candidates = filter_suction_candidates_by_seal(candidates, carton, planning_cfg)
    candidates = filter_suction_candidates_by_tool_clearance(
        candidates,
        scene.all_obstacles,
        getattr(robot, "tool_collision_size", None),
        target_name=carton.name,
        margin_m=float(planning_cfg.get("tool_collision_margin_m", 0.001)),
    )
    candidates = select_fast_suction_candidates(
        candidates,
        planning_cfg.get("grasp_face_modes", ["front", "side", "top"]),
        int(planning_cfg.get("dock_grasp_candidates_per_face", 1)),
    )
    start_position = robot.fk(start_q)[:3, 3]
    successful = 0
    shortest_motion = float("inf")
    for candidate in candidates:
        seeds = [start_q]
        pregrasp_cache = planning_cfg.get("pregrasp_seed_cache")
        if pregrasp_cache is not None:
            cached_seed = pregrasp_cache.nearest_seed(
                candidate.pregrasp_pose[:3, 3],
                robot.base_transform[:3, 3],
            )
            if cached_seed is not None:
                seeds.append(cached_seed)
        pre = solve_ik_multistart(
            robot,
            candidate.pregrasp_pose,
            seeds=seeds,
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
        carton_from_tool = np.linalg.inv(robot.fk(grasp.q)) @ carton.world_from_local
        carried_obstacles = scene.obstacles_without({carton.name})

        def retreat_state_valid(q: np.ndarray) -> bool:
            return _carried_box_state_valid(
                robot,
                carton,
                grasp.q,
                q,
                carried_obstacles,
                box_margin=float(planning_cfg.get("carried_box_clearance", 0.01)),
                carton_from_tool=carton_from_tool,
                carton_contact_tolerance=float(planning_cfg.get("carton_contact_tolerance_m", 0.001)),
                orientation_valid=lambda box: _in_trailer_orientation_valid(
                    carton,
                    box,
                    scene,
                    planning_cfg.get("max_in_trailer_carton_tilt_deg"),
                ),
            )

        retreat_ok, _ = _edge_collision_free(
            robot,
            grasp.q,
            pre.q,
            carried_obstacles,
            resolution=float(planning_cfg.get("dock_retreat_probe_resolution", 0.08)),
            extra_state_valid=retreat_state_valid,
        )
        if not retreat_ok:
            continue
        successful += 1
        shortest_motion = min(
            shortest_motion,
            float(np.linalg.norm(candidate.pregrasp_pose[:3, 3] - start_position))
            + float(np.linalg.norm(candidate.grasp_pose[:3, 3] - candidate.pregrasp_pose[:3, 3])),
        )
    total = len(candidates)
    place_names = planning_cfg.get(
        "place_obstacles", [planning_cfg.get("place_obstacle", "conveyor_deck")]
    )
    place_surfaces = [
        obstacle for obstacle in scene.obstacles if obstacle.name in place_names
    ]
    estimated_transfer = min(
        (
            float(
                np.linalg.norm(
                    carton.center
                    - surface.to_world(
                        np.array(
                            [
                                np.clip(
                                    surface.to_local(carton.center)[0],
                                    -surface.half_extents[0],
                                    surface.half_extents[0],
                                ),
                                np.clip(
                                    surface.to_local(carton.center)[1],
                                    -surface.half_extents[1],
                                    surface.half_extents[1],
                                ),
                                surface.half_extents[2] + float(np.min(carton.half_extents)),
                            ]
                        )
                    )
                )
            )
            for surface in place_surfaces
        ),
        default=float("inf"),
    )
    return {
        "ik_success_count": successful,
        "ik_candidate_count": total,
        "ik_success_rate": successful / total if total else 0.0,
        "estimated_ee_motion_m": shortest_motion if successful else None,
        "estimated_place_transfer_m": estimated_transfer if np.isfinite(estimated_transfer) else None,
        "estimated_cycle_task_motion_m": (
            shortest_motion + estimated_transfer
            if successful and np.isfinite(estimated_transfer)
            else None
        ),
    }


def run_online(
    config_path: str | Path,
    max_picks: int | None,
    output_dir: str | Path | None = None,
    top_layer_only: bool = False,
    resume_plan: str | Path | None = None,
    runtime_library: str | Path | None = None,
) -> dict:
    config_path = Path(config_path)
    scene, cfg = load_scene_config(config_path)
    cache_identity = build_scene_cache_identity(cfg)
    validate_amr_conveyor_alignment(cfg)
    robot = build_robot(cfg)
    planning_cfg = dict(cfg.get("planning", {}))
    planning_cfg.setdefault(
        "carton_mass_kg",
        float(cfg.get("simulation_validation", {}).get("carton_mass_kg", 0.0)),
    )
    planning_cfg.setdefault(
        "joint_velocity_limits_rad_s",
        cfg.get("execution", {}).get("joint_velocity_limits_rad_s"),
    )
    conveyor_cfg = cfg.get("simulation_validation", {}).get("conveyor", {})
    planning_cfg.setdefault(
        "post_release_surface_directions_world",
        conveyor_cfg.get("surface_directions_world", {}),
    )
    planning_cfg["verbose"] = True
    planning_seed = int(planning_cfg.get("seed", 11))
    scoring_rng = np.random.default_rng(planning_seed)
    current_q = np.asarray(cfg["robot"]["home_joints"], dtype=float)

    outputs = cfg.get("outputs", {})
    out_dir = Path(output_dir or outputs.get("directory", "outputs/online_unload"))
    if not out_dir.is_absolute():
        out_dir = config_path.resolve().parent.parent / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    cache_path = planning_cfg.get("pregrasp_heatmap_cache")
    if cache_path:
        from .reachability import load_pregrasp_heatmap

        resolved_cache = Path(cache_path)
        if not resolved_cache.is_absolute():
            resolved_cache = config_path.resolve().parent.parent / resolved_cache
        if resolved_cache.exists():
            planning_cfg["pregrasp_seed_cache"] = load_pregrasp_heatmap(resolved_cache)
            print(f"[online] loaded pregrasp seed cache: {resolved_cache}", flush=True)

    conveyor_name = planning_cfg.get("place_obstacle", "conveyor_deck")
    conveyor = next((obstacle for obstacle in scene.obstacles if obstacle.name == conveyor_name), None)

    segments: list[dict] = []
    trajectory_rows: list[tuple[int, str, int, np.ndarray]] = []
    unmoved_boxes: dict[str, str] = {}
    initial_top_height = max(carton.center[2] + carton.half_extents[2] for carton in scene.cartons)
    initial_support_graph = SupportRelationGraph.build(scene.cartons)
    support_graph_audit = {
        **initial_support_graph.audit(),
        "proposed_removal_order": initial_support_graph.removal_order(
            planning_cfg.get("grasp_face_modes", ["front", "side", "top"])
        ),
    }
    start_time = perf_counter()
    initial_carton_count = len(scene.cartons)
    max_pick_count = initial_carton_count if max_picks is None or max_picks <= 0 else max_picks

    configured_initial_dock = planning_cfg.get("initial_dock_position")
    current_dock: np.ndarray | None = (
        None
        if configured_initial_dock is None
        else np.asarray(configured_initial_dock, dtype=float)
    )
    certified_planner = None
    runtime_lookup_statistics: list[dict] = []
    if runtime_library is not None:
        from .realtime_planner import CertifiedRuntimePlanner

        library_path = Path(runtime_library)
        if not library_path.is_absolute():
            library_path = config_path.resolve().parent.parent / library_path
        library_data = json.loads(library_path.read_text(encoding="utf-8"))
        certified_planner = CertifiedRuntimePlanner(
            library_data,
            deadline_seconds=float(planning_cfg.get("online_lookup_deadline_seconds", 0.05)),
            expected_identity=cache_identity,
        )
        print(f"[online] certified FANUC runtime library: {library_path}", flush=True)
    resume_source = None
    if resume_plan is not None:
        resume_source = Path(resume_plan)
        if not resume_source.is_absolute():
            resume_source = config_path.resolve().parent.parent / resume_source
        segments, trajectory_rows, current_q, restored_dock = restore_plan_prefix(
            scene,
            cfg["robot"],
            resume_source,
            cache_identity,
        )
        if restored_dock is not None:
            current_dock = restored_dock
        print(
            f"[online] restored {len(segments)} picks from {resume_source}; remaining={len(scene.cartons)}",
            flush=True,
        )

    print(
        f"[online] start: robot={robot.name}, cartons={initial_carton_count}, "
        f"restored={len(segments)}, max_picks={max_pick_count}",
        flush=True,
    )
    pick_index = len(segments)
    while scene.cartons and len(segments) < max_pick_count:
        ordered_names = exposed_carton_order(scene, planning_cfg.get("grasp_face_modes", ["front", "side", "top"]))
        if bool(planning_cfg.get("enforce_nearest_active_depth", False)):
            ordered_names = restrict_to_nearest_active_depth(
                scene,
                ordered_names,
                tolerance_m=float(
                    planning_cfg.get("active_depth_tolerance_m", 0.008)
                ),
            )
        if bool(planning_cfg.get("enforce_highest_active_layer", False)):
            ordered_names = restrict_to_highest_active_layer(
                scene,
                ordered_names,
                tolerance_m=float(
                    planning_cfg.get("active_layer_height_tolerance_m", 0.008)
                ),
            )
        if top_layer_only:
            ordered_names = [
                name
                for name in ordered_names
                if np.isclose(scene.carton(name).center[2] + scene.carton(name).half_extents[2], initial_top_height)
            ]
            if not ordered_names:
                break
        detections = detect_carton_obbs(scene)
        reachable_names = ordered_names
        removal_priority = {name: index for index, name in enumerate(reachable_names)}
        print(
            f"[online] pick {pick_index}: detected {len(detections)} OBBs; "
            f"exposed reachable candidates={reachable_names[:8]}",
            flush=True,
        )

        if certified_planner is not None:
            cached = certified_planner.plan_next(scene, current_q, current_dock)
            runtime_lookup_statistics.append(
                {
                    "pick_index": pick_index,
                    "success": cached.success,
                    "source": cached.source,
                    "latency_seconds": cached.latency_seconds,
                    "message": cached.message,
                }
            )
            if not cached.success or cached.segment is None or cached.target is None:
                for carton in scene.cartons:
                    unmoved_boxes[carton.name] = f"certified runtime cache miss: {cached.message}"
                print(
                    f"[online] pick {pick_index}: certified lookup failed in "
                    f"{cached.latency_seconds * 1000.0:.3f} ms; cold RRT is disabled",
                    flush=True,
                )
                break
            cached_segment = copy.deepcopy(cached.segment)
            cached_segment["pick_index"] = pick_index
            cached_segment["online_planning"] = {
                "source": cached.source,
                "latency_seconds": cached.latency_seconds,
                "hard_deadline_seconds": certified_planner.deadline_seconds,
            }
            path = [np.asarray(q, dtype=float) for q in cached_segment["path"]]
            for local_index, q in enumerate(path):
                trajectory_rows.append((pick_index, cached.target, local_index, q))
            segments.append(cached_segment)
            current_q = path[-1].copy()
            current_dock = np.asarray(cached_segment["amr_dock_position"], dtype=float)
            remove_carton(scene, cached.target)
            unmoved_boxes.pop(cached.target, None)
            print(
                f"[online] pick {pick_index}: certified target={cached.target}; "
                f"lookup={cached.latency_seconds * 1000.0:.3f} ms",
                flush=True,
            )
            pick_index += 1
            continue

        chosen_result = None
        chosen_name = None
        chosen_dock = None
        chosen_scene = None
        chosen_cfg = None
        failed_this_round: dict[str, str] = {}
        scored_pairs: list[tuple[tuple[float, ...], np.ndarray, str, dict]] = []
        dock_candidates = prioritized_amr_dock_positions(cfg, current_dock)
        if current_dock is not None and bool(planning_cfg.get("hold_current_dock", False)):
            dock_candidates = [current_dock.copy()]
        for dock_position in dock_candidates:
            if not amr_dock_is_clear(scene, cfg, dock_position):
                continue
            dock_robot, dock_scene, _ = docked_robot_and_scene(scene, cfg, dock_position)
            for target_name in reachable_names:
                if carton_is_outside_fixed_base_reach(dock_robot, dock_scene.carton(target_name)):
                    continue
                score = score_target_dock_ik(dock_robot, dock_scene, current_q, target_name, planning_cfg, scoring_rng)
                ee_motion = score["estimated_ee_motion_m"]
                dock_motion = 0.0 if current_dock is None else float(np.linalg.norm(dock_position - current_dock))
                # Once docked, exhaust useful work at that pose before moving
                # the AMR.  For an online cell, a reachable target's current
                # end-effector distance is a better sequencing signal than a
                # tiny difference in sampled IK yield: it avoids skipping the
                # adjacent carton merely to reuse an easier IK branch.
                dock_switch = 0 if current_dock is None or np.allclose(dock_position, current_dock) else 1
                motion = float("inf") if ee_motion is None else ee_motion
                cycle_motion = score.get("estimated_cycle_task_motion_m")
                cycle_motion = float("inf") if cycle_motion is None else float(cycle_motion)
                front_clearance = _adaptive_destack_clearance(
                    dock_scene.carton(target_name),
                    dock_scene.all_obstacles,
                    np.array([-1.0, 0.0, 0.0]),
                    clearance_m=float(
                        planning_cfg.get("carton_contact_tolerance_m", 0.001)
                    ),
                )
                enclosed_destack = int(front_clearance.requires_straight_withdrawal)
                if bool(planning_cfg.get("target_motion_first", False)):
                    rank = (
                        removal_priority[target_name],
                        dock_switch,
                        dock_motion,
                        enclosed_destack,
                        0 if score["ik_success_count"] else 1,
                        cycle_motion
                        if bool(planning_cfg.get("target_cycle_motion_first", False))
                        else motion,
                        -score["ik_success_rate"],
                        motion,
                    )
                else:
                    rank = (
                        removal_priority[target_name],
                        dock_switch,
                        dock_motion,
                        enclosed_destack,
                        0 if score["ik_success_count"] else 1,
                        -score["ik_success_rate"],
                        motion,
                    )
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
                target_planning_cfg = dict(planning_cfg)
                balance_weight = float(planning_cfg.get("conveyor_load_balance_weight", 0.0))
                if balance_weight > 0.0:
                    target_planning_cfg["place_surface_penalties"] = {
                        surface_name: balance_weight
                        * sum(segment.get("place_surface") == surface_name for segment in segments)
                        for surface_name in planning_cfg.get(
                            "place_obstacles",
                            [planning_cfg.get("place_obstacle", "conveyor_deck")],
                        )
                    }
                result = plan_pick(
                    dock_robot,
                    dock_scene,
                    dock_start,
                    target_name,
                    planner_options=target_planning_cfg,
                    rng=target_rng,
                )
                if (
                    not result.success
                    and bool(planning_cfg.get("retry_alternate_place_surfaces", True))
                ):
                    primary_surface = str(
                        planning_cfg.get("place_obstacle", "conveyor_deck")
                    )
                    alternate_surfaces = [
                        str(surface_name)
                        for surface_name in planning_cfg.get(
                            "place_obstacles", [primary_surface]
                        )
                        if str(surface_name) != primary_surface
                    ]
                    for fallback_index, surface_name in enumerate(alternate_surfaces):
                        print(
                            f"[online] pick {pick_index}: target={target_name}; "
                            f"retrying independently on place_surface={surface_name}",
                            flush=True,
                        )
                        fallback_cfg = dict(target_planning_cfg)
                        fallback_cfg["place_obstacle"] = surface_name
                        fallback_cfg["place_obstacles"] = [surface_name]
                        fallback_cfg["place_surface_penalties"] = {surface_name: 0.0}
                        result = plan_pick(
                            dock_robot,
                            dock_scene,
                            dock_start,
                            target_name,
                            planner_options=fallback_cfg,
                            rng=np.random.default_rng(
                                planning_seed
                                + pick_index
                                + 1000 * (fallback_index + 1)
                            ),
                        )
                        if result.success:
                            break
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
        release_center = None
        place_rotation = None
        max_carried_tilt = None
        if chosen_result.grasp_ik is not None and chosen_result.place_ik is not None:
            carton_from_tool = np.linalg.inv(robot.fk(chosen_result.grasp_ik.q)) @ target_carton.world_from_local
            placed_carton = robot.fk(chosen_result.place_ik.q) @ carton_from_tool
            release_center = placed_carton[:3, 3].copy()
            from .perception import controlled_release_height

            release_height = controlled_release_height(planning_cfg)
            place_surface = next(
                (obstacle for obstacle in chosen_scene.obstacles if obstacle.name == chosen_result.place_surface_name),
                None,
            )
            drop_axis = np.array([0.0, 0.0, 1.0]) if place_surface is None else place_surface.rotation[:, 2]
            place_center = release_center - release_height * drop_axis
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
                "sealed_cup_indices": (
                    list(chosen_result.candidate.sealed_cup_indices)
                    if chosen_result.candidate
                    else []
                ),
                "sealed_cup_count": (
                    len(chosen_result.candidate.sealed_cup_indices)
                    if chosen_result.candidate
                    else 0
                ),
                "sealed_cups_per_zone": (
                    list(chosen_result.candidate.sealed_cups_per_zone)
                    if chosen_result.candidate
                    else []
                ),
                "place_center": place_center.tolist() if place_center is not None else None,
                "release_center": release_center.tolist() if release_center is not None else None,
                "place_surface": chosen_result.place_surface_name,
                "free_fall_height_m": release_height if release_center is not None else 0.0,
                "free_fall_impact_velocity_m_s": (
                    float(np.sqrt(2.0 * 9.81 * release_height))
                    if release_center is not None
                    else 0.0
                ),
                "free_fall_energy_j": (
                    float(planning_cfg.get("carton_mass_kg", 0.0) * 9.81 * release_height)
                    if release_center is not None
                    else 0.0
                ),
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
        checkpoint_path = out_dir / "online_checkpoint.json"
        with checkpoint_path.open("w", encoding="utf-8") as f:
            json.dump(
                {
                    "config": str(config_path),
                    "robot": cfg["robot"],
                    "cache_identity": cache_identity,
                    "segments": segments,
                    "moved_boxes": [segment["target"] for segment in segments],
                },
                f,
                ensure_ascii=False,
                indent=2,
            )
        print(
            f"[online] pick {pick_index}: moved {chosen_name}; remaining={len(scene.cartons)}; "
            f"segment_waypoints={len(path)}",
            flush=True,
        )
        pick_index += 1

    execution_summary = time_parameterize_segments(segments, cfg)

    trajectory_csv = out_dir / "online_trajectory.csv"
    with trajectory_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["pick_index", "target", "local_index", *[f"q{i + 1}" for i in range(robot.dof)]])
        for pick_index, target, local_index, q in trajectory_rows:
            writer.writerow([pick_index, target, local_index, *[float(v) for v in q]])

    timed_trajectory_csv = out_dir / "online_timed_trajectory.csv"
    with timed_trajectory_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["pick_index", "target", "local_index", "time_from_pick_start_s", *[f"q{i + 1}" for i in range(robot.dof)]]
        )
        for segment in segments:
            timestamps = segment["time_from_start_seconds"]
            for local_index, (timestamp, q) in enumerate(zip(timestamps, segment["path"])):
                writer.writerow(
                    [int(segment["pick_index"]), segment["target"], local_index, float(timestamp), *[float(v) for v in q]]
                )

    manifest = {
        "config": str(config_path),
        "robot": cfg["robot"],
        "cache_identity": cache_identity,
        "planning_time_seconds": perf_counter() - start_time,
        "execution_summary": execution_summary,
        "support_graph": support_graph_audit,
        "runtime_lookup_statistics": runtime_lookup_statistics,
        "resume_plan": str(resume_source) if resume_source is not None else None,
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
        "timed_trajectory_csv": str(timed_trajectory_csv),
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
    parser.add_argument("--top-layer-only", action="store_true", help="Only unload cartons from the initial highest layer")
    parser.add_argument("--resume-plan", default=None, help="Continue from a validated plan prefix")
    parser.add_argument("--runtime-library", default=None, help="Use certified FANUC lookup and disable blocking cold RRT")
    args = parser.parse_args(argv)
    run_online(
        args.config,
        args.max_picks,
        args.output_dir,
        args.top_layer_only,
        args.resume_plan,
        args.runtime_library,
    )


if __name__ == "__main__":
    main()
