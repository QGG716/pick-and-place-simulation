from pathlib import Path

import numpy as np

from unloading_sim.demo import run
from unloading_sim.geometry import OBB, make_transform
from unloading_sim.grasp import _carried_box_at, _in_trailer_orientation_valid, generate_suction_candidates, plan_pick, select_fast_suction_candidates
from unloading_sim.online_unload import (
    amr_dock_is_clear,
    amr_dock_positions,
    carton_blocked_by_scene,
    carton_is_outside_fixed_base_reach,
    docked_robot_and_scene,
    exposed_carton_order,
    prioritized_amr_dock_positions,
    score_target_dock_ik,
    validate_amr_conveyor_alignment,
)
from unloading_sim.perception import conveyor_place_pose_candidates, detect_carton_obbs, fixed_conveyor_place_pose, select_detection, surface_place_pose_candidates
from unloading_sim.robot import DHRobot6
from unloading_sim.scene import TrailerScene, load_scene_config


def test_default_demo_succeeds(tmp_path):
    root = Path(__file__).resolve().parents[1]
    metrics = run(root / "config" / "demo.yaml")
    assert metrics["success"] is True
    assert metrics["trajectory_waypoints"] > 2


def test_kuka_scene_layout_detection_and_grasp_modes():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    detections = detect_carton_obbs(scene)
    assert len(detections) == 24

    trailer = cfg["scene"]["trailer"]
    carton_width_sum = sum(
        2.0 * carton.half_extents[1]
        for carton in scene.cartons
        if np.isclose(carton.center[0], 1.21) and np.isclose(carton.center[2], 1.05)
    )
    assert carton_width_sum > 0.93 * float(trailer["width"])

    conveyor = next(obstacle for obstacle in scene.obstacles if obstacle.name == "conveyor_deck")
    base_x = float(cfg["robot"]["base_position"][0])
    conveyor_front_x = conveyor.center[0] + conveyor.half_extents[0]
    assert conveyor_front_x < base_x

    for carton in scene.cartons:
        assert not conveyor.contains(carton.center)

    target = select_detection(detections, cfg["planning"]["target_carton"]).obb
    candidates = generate_suction_candidates(target, np.asarray(cfg["robot"]["base_position"], dtype=float))
    assert {candidate.face_mode for candidate in candidates} == {"front", "side", "top"}

    place_pose = fixed_conveyor_place_pose(conveyor, target)
    expected_x = conveyor.center[0] + conveyor.half_extents[0] - 0.10
    assert np.isclose(place_pose[0, 3], expected_x)


def test_fast_grasp_selection_keeps_each_face_mode():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    carton = scene.carton(cfg["planning"]["target_carton"])
    candidates = generate_suction_candidates(carton, np.asarray(cfg["robot"]["base_position"], dtype=float))

    selected = select_fast_suction_candidates(candidates, ("top", "side", "front"), candidates_per_face=1)

    assert [candidate.face_mode for candidate in selected] == ["top", "side", "front"]


def test_fixed_base_reach_filter_skips_only_distant_cartons():
    root = Path(__file__).resolve().parents[1]
    scene, _ = load_scene_config(root / "config" / "demo.yaml")
    robot = DHRobot6.ur5e_like(base_rpy=[0.0, 0.0, np.pi])

    assert not carton_is_outside_fixed_base_reach(robot, scene.carton("carton_target"))
    far_carton = scene.carton("carton_target").transformed(
        np.array([[1.0, 0.0, 0.0, 20.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    )
    assert carton_is_outside_fixed_base_reach(robot, far_carton)


def test_exposed_carton_order_excludes_covered_and_rear_cartons():
    root = Path(__file__).resolve().parents[1]
    scene, _ = load_scene_config(root / "config" / "kuka_kr50.yaml")

    exposed = exposed_carton_order(scene)

    assert set(exposed) == {
        "carton_front_l2_c0",
        "carton_front_l2_c1",
        "carton_front_l2_c2",
        "carton_front_l2_c3",
    }
    assert carton_blocked_by_scene(scene, "carton_target")
    assert not carton_blocked_by_scene(scene, "carton_front_l2_c1")


def test_pick_plan_honors_zero_time_candidate_budget():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "demo.yaml")
    robot = DHRobot6.ur5e_like(base_rpy=[0.0, 0.0, np.pi])
    result = plan_pick(
        robot,
        scene,
        np.asarray(cfg["robot"]["home_joints"], dtype=float),
        "carton_target",
        planner_options={"target_planning_time_limit_seconds": 1e-12},
    )

    assert not result.success
    assert "time budget" in result.message


def test_amr_docking_moves_robot_and_conveyor_together():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    dock = amr_dock_positions(cfg)[0]

    assert amr_dock_is_clear(scene, cfg, dock)
    robot, docked_scene, docked_cfg = docked_robot_and_scene(scene, cfg, dock)
    conveyor = next(obstacle for obstacle in docked_scene.obstacles if obstacle.name == "conveyor_deck")
    expected_conveyor_center = dock + np.asarray(cfg["amr"]["conveyor_mount_center"], dtype=float)
    expected_robot_base = dock + np.asarray(cfg["amr"]["robot_mount_position"], dtype=float)

    assert np.allclose(robot.base_transform[:3, 3], expected_robot_base)
    assert np.allclose(conveyor.center, expected_conveyor_center)
    assert docked_cfg["robot"]["base_position"] == expected_robot_base.tolist()


def test_amr_and_conveyor_front_alignment():
    root = Path(__file__).resolve().parents[1]
    _, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")

    validate_amr_conveyor_alignment(cfg)
    amr = cfg["amr"]
    conveyor_front = amr["conveyor_mount_center"][0] + 0.5 * cfg["scene"]["static_obstacles"][0]["size"][0]
    robot_front = amr["robot_mount_position"][0]
    amr_front = amr["platform_center_offset"][0] + 0.5 * amr["footprint_size"][0]

    assert np.isclose(conveyor_front - robot_front, 0.20)
    assert conveyor_front > amr_front


def test_carton_stack_and_amr_front_are_inside_the_requested_door_clearances():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    front_row = [carton for carton in scene.cartons if carton.name.startswith("carton_front_")]
    dock = amr_dock_positions(cfg)[0]
    amr = cfg["amr"]

    assert np.isclose(min(carton.center[0] - carton.half_extents[0] for carton in front_row), 1.0)
    amr_front = dock[0] + amr["platform_center_offset"][0] + 0.5 * amr["footprint_size"][0]
    assert np.isclose(amr_front, 0.5)
    assert np.isclose(min(carton.center[0] - carton.half_extents[0] for carton in front_row) - amr_front, 0.5)


def test_conveyor_place_candidates_use_release_zone_and_full_support():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    dock = amr_dock_positions(cfg)[0]
    robot, docked_scene, _ = docked_robot_and_scene(scene, cfg, dock)
    carton = scene.carton("carton_front_l2_c2")
    conveyor = next(obstacle for obstacle in docked_scene.obstacles if obstacle.name == "conveyor_deck")
    grasp = next(
        candidate
        for candidate in generate_suction_candidates(carton, robot.base_transform[:3, 3])
        if conveyor_place_pose_candidates(conveyor, carton, candidate.grasp_pose, robot.base_transform[:3, 3])
    )
    poses = conveyor_place_pose_candidates(
        conveyor,
        carton,
        grasp.grasp_pose,
        robot.base_transform[:3, 3],
        allow_all_carton_faces=True,
        require_front_release_zone=False,
    )
    carton_from_tool = np.linalg.inv(grasp.grasp_pose) @ carton.world_from_local
    local_signs = np.array(
        [[x, y, z] for x in (-1.0, 1.0) for y in (-1.0, 1.0) for z in (-1.0, 1.0)]
    )

    assert poses
    for pose in poses:
        placed_carton = pose @ carton_from_tool
        corners = (placed_carton[:3, :3] @ (local_signs * carton.half_extents).T).T + placed_carton[:3, 3]
        local_corners = np.array([conveyor.to_local(corner) for corner in corners])
        assert np.max(np.abs(local_corners[:, 0])) <= conveyor.half_extents[0] - 0.01 + 1e-9
        assert np.max(np.abs(local_corners[:, 1])) <= conveyor.half_extents[1] - 0.01 + 1e-9
        placed_up = placed_carton[:3, :3][:, 2]
        local_up = carton.rotation.T @ placed_up
        assert np.isclose(np.max(np.abs(local_up)), 1.0)

    supported_face_axes = {
        int(np.argmax(np.abs(carton.rotation.T @ (pose @ carton_from_tool)[:3, :3][:, 2])))
        for pose in poses
    }
    assert supported_face_axes == {0, 1, 2}


def test_carried_carton_matches_scene_pose_at_grasp():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    robot, _, _ = docked_robot_and_scene(scene, cfg, amr_dock_positions(cfg)[0])
    carton = scene.carton("carton_front_l2_c2")
    grasp_q = np.asarray(cfg["robot"]["home_joints"], dtype=float)
    carton_from_tool = np.linalg.inv(robot.fk(grasp_q)) @ carton.world_from_local

    carried = _carried_box_at(robot, carton, carton_from_tool, grasp_q)

    assert np.allclose(carried.center, carton.center)
    assert np.allclose(carried.rotation, carton.rotation)


def test_carton_tilt_is_limited_only_inside_trailer():
    root = Path(__file__).resolve().parents[1]
    scene, _ = load_scene_config(root / "config" / "kuka_kr50.yaml")
    carton = scene.carton("carton_front_l2_c2")
    tipped_rotation = carton.rotation @ np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    inside = OBB(carton.center, carton.half_extents, tipped_rotation, name="inside", category="carton")
    outside = OBB(np.array([-1.0, 0.0, 1.0]), carton.half_extents, tipped_rotation, name="outside", category="carton")

    assert not _in_trailer_orientation_valid(carton, inside, scene, 25.0)
    assert _in_trailer_orientation_valid(carton, outside, scene, 25.0)


def test_internal_placement_surface_has_no_conveyor_release_zone():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    carton = scene.carton("carton_front_l2_c2")
    robot, _, _ = docked_robot_and_scene(scene, cfg, amr_dock_positions(cfg)[0])
    grasp = generate_suction_candidates(carton, robot.base_transform[:3, 3])[0]
    staging_surface = OBB(
        center=np.array([1.20, 0.0, 0.60]),
        half_extents=np.array([0.60, 0.60, 0.08]),
        rotation=np.eye(3),
        name="trailer_staging_deck",
        category="static",
    )

    poses = surface_place_pose_candidates(staging_surface, carton, grasp.grasp_pose)

    assert poses


def test_compact_amr_fits_inside_empty_trailer():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    empty_scene = TrailerScene(obstacles=scene.obstacles, cartons=[], metadata=scene.metadata)

    assert float(cfg["amr"]["footprint_size"][1]) < float(cfg["scene"]["trailer"]["width"])
    assert amr_dock_is_clear(empty_scene, cfg, np.array([1.0, 0.0, 0.0]))


def test_current_amr_dock_is_tried_first():
    root = Path(__file__).resolve().parents[1]
    _, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    current = np.array([0.60, 0.0, 0.0])

    ordered = prioritized_amr_dock_positions(cfg, current)

    assert np.allclose(ordered[0], current)


def test_dock_ik_score_reports_candidate_yield_and_motion():
    root = Path(__file__).resolve().parents[1]
    scene, cfg = load_scene_config(root / "config" / "kuka_kr50.yaml")
    robot, docked_scene, docked_cfg = docked_robot_and_scene(scene, cfg, amr_dock_positions(cfg)[0])

    score = score_target_dock_ik(
        robot,
        docked_scene,
        np.asarray(docked_cfg["robot"]["home_joints"], dtype=float),
        "carton_front_l2_c0",
        cfg["planning"],
        np.random.default_rng(11),
    )

    assert 0 <= score["ik_success_count"] <= score["ik_candidate_count"]
    assert 0.0 <= score["ik_success_rate"] <= 1.0
    if score["ik_success_count"]:
        assert score["estimated_ee_motion_m"] > 0.0
