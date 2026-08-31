from types import SimpleNamespace

import numpy as np

from unloading_sim.geometry import make_transform
from unloading_sim.grasp import (
    _adaptive_destack_clearance,
    _carried_task_envelope_valid,
    _carried_orientation_policy_valid,
    _carried_obstacle_margin,
    _continuous_equivalent_joint_path,
    _plan_cartesian_carry,
    _reorientation_clearance_standoff,
    _reorientation_wall_clearance_offset,
    SuctionGraspCandidate,
    _expand_candidate_wrist_rolls,
    filter_suction_candidates_by_seal,
    filter_suction_candidates_by_tool_clearance,
    _nearest_equivalent_joint_vector,
    generate_suction_candidates,
    post_release_escape_target,
    suction_footprint_fits_face,
)
from unloading_sim.geometry import OBB
from unloading_sim.online_unload import (
    restrict_to_highest_active_layer,
    restrict_to_nearest_active_depth,
)
from unloading_sim.perception import controlled_release_height
from unloading_sim.scene import TrailerScene, load_scene_config


def test_active_layer_filter_preserves_center_out_order_and_blocks_lower_fallback():
    ident = np.eye(3)
    cartons = [
        OBB(np.array([1.0, 0.0, 0.5]), np.full(3, 0.2), ident, "lower_center", "carton"),
        OBB(np.array([1.0, -0.3, 1.0]), np.full(3, 0.2), ident, "top_center", "carton"),
        OBB(np.array([1.0, -0.9, 1.0]), np.full(3, 0.2), ident, "top_edge", "carton"),
    ]
    scene = TrailerScene(obstacles=[], cartons=cartons, metadata={})

    filtered = restrict_to_highest_active_layer(
        scene,
        ["top_center", "lower_center", "top_edge"],
    )

    assert filtered == ["top_center", "top_edge"]


def test_active_depth_filter_keeps_near_work_face_before_deep_top_cartons():
    ident = np.eye(3)
    cartons = [
        OBB(np.array([1.0, 0.0, 0.5]), np.full(3, 0.2), ident, "near_lower", "carton"),
        OBB(np.array([1.5, 0.0, 1.0]), np.full(3, 0.2), ident, "deep_top", "carton"),
        OBB(np.array([1.0, 0.6, 0.5]), np.full(3, 0.2), ident, "near_edge", "carton"),
    ]
    scene = TrailerScene(obstacles=[], cartons=cartons, metadata={})

    filtered = restrict_to_nearest_active_depth(
        scene,
        ["deep_top", "near_lower", "near_edge"],
    )

    assert filtered == ["near_lower", "near_edge"]


def test_cartesian_carry_uses_the_callers_collision_margin(monkeypatch):
    observed_margins = []

    class RobotStub:
        def fk(self, _q):
            return np.eye(4)

        def is_collision_free(self, _q, _obstacles, ignored_obstacle_names=None):
            return True

    def fake_state_valid(*_args, box_margin, **_kwargs):
        observed_margins.append(box_margin)
        return True

    monkeypatch.setattr("unloading_sim.grasp._carried_box_state_valid", fake_state_valid)
    monkeypatch.setattr(
        "unloading_sim.grasp.solve_ik_multistart",
        lambda *_args, **_kwargs: SimpleNamespace(success=True, q=np.zeros(6)),
    )
    target = np.eye(4)
    target[0, 3] = 0.02
    carton = OBB(np.zeros(3), np.full(3, 0.1), np.eye(3), "box", "carton")

    result = _plan_cartesian_carry(
        RobotStub(),
        carton,
        np.zeros(6),
        np.zeros(6),
        target,
        [],
        [],
        np.eye(4),
        None,
        np.random.default_rng(1),
        box_margin=0.04,
    )

    assert result.success
    assert observed_margins
    assert set(observed_margins) == {0.04}


def test_wrist_roll_candidates_keep_suction_normal_and_contact():
    pose = make_transform(np.eye(3), [1.0, 2.0, 3.0])
    candidate = SuctionGraspCandidate(
        carton_name="box",
        contact_point=np.array([1.0, 2.0, 3.0]),
        outward_normal=np.array([1.0, 0.0, 0.0]),
        face_mode="front",
        pregrasp_pose=pose.copy(),
        grasp_pose=pose.copy(),
        score=1.0,
    )

    variants = _expand_candidate_wrist_rolls([candidate], [0.0, 90.0, -90.0, 180.0])

    assert len(variants) == 4
    for variant in variants:
        assert np.allclose(variant.grasp_pose[:3, 2], candidate.grasp_pose[:3, 2])
        assert np.allclose(variant.grasp_pose[:3, 3], candidate.grasp_pose[:3, 3])


def test_rectangular_suction_footprint_rejects_edge_overhang():
    carton = OBB(
        center=np.array([1.0, 0.0, 0.25]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="box",
        category="carton",
    )
    candidates = generate_suction_candidates(
        carton,
        np.array([0.0, 0.0, 0.3]),
        face_modes=["front"],
    )
    central = next(
        candidate
        for candidate in candidates
        if np.allclose(carton.to_local(candidate.contact_point)[1:], [0.0, 0.0])
    )
    edge = next(
        candidate
        for candidate in candidates
        if not np.allclose(carton.to_local(candidate.contact_point)[1:], [0.0, 0.0])
    )

    assert suction_footprint_fits_face(central, carton, [0.30, 0.40], edge_margin_m=0.01)
    assert not suction_footprint_fits_face(edge, carton, [0.30, 0.40], edge_margin_m=0.01)


def test_fg42_grid_keeps_sixty_fully_sealed_cups_on_centered_carton_face():
    carton = OBB(
        center=np.array([1.0, 0.0, 0.25]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="box",
        category="carton",
    )
    central = next(
        candidate
        for candidate in generate_suction_candidates(
            carton, np.array([0.0, 0.0, 0.3]), face_modes=["front"]
        )
        if np.allclose(carton.to_local(candidate.contact_point)[1:], [0.0, 0.0])
    )
    variants = _expand_candidate_wrist_rolls([central], [0.0, 90.0])
    accepted = filter_suction_candidates_by_seal(
        variants,
        carton,
        {
            "suction_cup_layout": {
                "rows": 6,
                "columns": 12,
                "pitch_m": [0.048, 0.048],
                "cup_radius_m": 0.0215,
                "zone_count": 3,
                "minimum_sealed_cups": 60,
            }
        },
    )

    assert len(accepted) == 2
    assert all(len(candidate.sealed_cup_indices) == 60 for candidate in accepted)
    assert all(candidate.sealed_cups_per_zone == (18, 24, 18) for candidate in accepted)


def test_tool_envelope_filter_rejects_a_rigid_head_blocker():
    target = OBB(
        center=np.array([1.21, 0.825, 1.25]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="outer",
        category="carton",
    )
    candidate = next(candidate for candidate in generate_suction_candidates(
        target,
        np.array([0.0, 0.0, 0.3]),
        face_modes=["front"],
    ) if np.allclose(target.to_local(candidate.contact_point)[1:], [0.0, 0.0]))
    size = np.array([0.576, 0.288, 0.2275])
    pose = candidate.grasp_pose
    blocker = OBB(
        center=pose[:3, :3] @ np.array([0.0, 0.0, -0.5 * size[2]]) + pose[:3, 3],
        half_extents=np.full(3, 0.01),
        rotation=np.eye(3),
        name="blocker",
    )

    assert filter_suction_candidates_by_tool_clearance(
        [candidate],
        [target],
        size,
        target_name=target.name,
    ) == [candidate]
    accepted = filter_suction_candidates_by_tool_clearance(
        [candidate],
        [target, blocker],
        size,
        target_name=target.name,
    )

    assert accepted == []


def test_nearest_equivalent_joint_vector_avoids_full_wrist_turn():
    robot = SimpleNamespace(
        joint_limits=np.array([[-np.pi, np.pi]] * 5 + [[-2.5 * np.pi, 2.5 * np.pi]])
    )
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.1 * np.pi])
    reference = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.05 * np.pi])

    normalized = _nearest_equivalent_joint_vector(robot, q, reference)

    assert np.isclose(normalized[5], 0.1 * np.pi)
    assert robot.joint_limits[5, 0] <= normalized[5] <= robot.joint_limits[5, 1]


def test_scene_config_supports_recursive_inheritance(tmp_path):
    base = tmp_path / "base.yaml"
    middle = tmp_path / "middle.yaml"
    leaf = tmp_path / "leaf.yaml"
    base.write_text(
        """
scene:
  trailer: {length: 4.0, width: 2.0, height: 3.0}
  cartons: []
robot: {model: base}
planning: {seed: 1, nested: {a: 1}}
""",
        encoding="utf-8",
    )
    middle.write_text(
        "extends: base.yaml\nplanning: {nested: {b: 2}}\n",
        encoding="utf-8",
    )
    leaf.write_text(
        "extends: middle.yaml\nrobot: {model: fanuc}\n",
        encoding="utf-8",
    )

    _, config = load_scene_config(leaf)

    assert config["robot"]["model"] == "fanuc"
    assert config["planning"]["nested"] == {"a": 1, "b": 2}


def test_floor_support_contact_uses_tolerance_but_walls_keep_clearance():
    floor = SimpleNamespace(name="trailer_floor", category="trailer")
    wall = SimpleNamespace(name="trailer_left_wall", category="trailer")
    carton = SimpleNamespace(name="neighbour", category="carton")

    assert np.isclose(_carried_obstacle_margin(floor, 0.01, 0.005), -0.005)
    assert np.isclose(_carried_obstacle_margin(carton, 0.01, 0.005), -0.005)
    assert np.isclose(_carried_obstacle_margin(wall, 0.01, 0.005), 0.01)


def test_carried_task_envelope_limits_rotation_and_upward_lift():
    reference = OBB(
        center=np.array([1.0, 0.0, 1.0]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="reference",
    )
    angle = np.radians(6.0)
    rotated = OBB(
        center=reference.center.copy(),
        half_extents=reference.half_extents.copy(),
        rotation=np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        name="rotated",
    )
    lifted = OBB(
        center=reference.center + np.array([0.0, 0.0, 0.09]),
        half_extents=reference.half_extents.copy(),
        rotation=reference.rotation.copy(),
        name="lifted",
    )

    assert _carried_task_envelope_valid(
        reference,
        reference,
        maximum_rotation_degrees=5.0,
        maximum_lift_m=0.08,
    )
    assert not _carried_task_envelope_valid(
        reference,
        rotated,
        maximum_rotation_degrees=5.0,
        maximum_lift_m=0.08,
    )
    assert not _carried_task_envelope_valid(
        reference,
        lifted,
        maximum_rotation_degrees=5.0,
        maximum_lift_m=0.08,
    )


def test_free_space_reorientation_policy_is_shared_with_cache_validation():
    reference = OBB(
        center=np.array([1.0, 0.0, 1.0]),
        half_extents=np.full(3, 0.2),
        rotation=np.eye(3),
        name="reference",
        category="carton",
    )
    angle = np.radians(30.0)
    rotated = OBB(
        center=reference.center.copy(),
        half_extents=reference.half_extents.copy(),
        rotation=np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(angle), -np.sin(angle)],
                [0.0, np.sin(angle), np.cos(angle)],
            ]
        ),
        name="rotated",
        category="carton",
    )
    scene = TrailerScene(
        obstacles=[],
        cartons=[reference],
        metadata={
            "trailer_interior": {
                "length": 4.0,
                "width": 2.4,
                "height": 3.0,
            }
        },
    )

    assert not _carried_orientation_policy_valid(
        reference,
        rotated,
        scene,
        {"max_in_trailer_carton_tilt_deg": 25.0},
    )
    assert _carried_orientation_policy_valid(
        reference,
        rotated,
        scene,
        {
            "max_in_trailer_carton_tilt_deg": 25.0,
            "allow_free_space_reorientation": True,
        },
    )


def test_adaptive_destack_requires_straight_motion_only_when_both_sides_are_blocked():
    target = OBB(
        center=np.zeros(3),
        half_extents=np.array([0.20, 0.25, 0.20]),
        rotation=np.eye(3),
        name="target",
        category="carton",
    )
    left = OBB(
        center=np.array([0.0, 0.50, 0.0]),
        half_extents=target.half_extents.copy(),
        rotation=np.eye(3),
        name="left",
        category="carton",
    )
    right = OBB(
        center=np.array([0.0, -0.50, 0.0]),
        half_extents=target.half_extents.copy(),
        rotation=np.eye(3),
        name="right",
        category="carton",
    )

    enclosed = _adaptive_destack_clearance(
        target, [target, left, right], [-1.0, 0.0, 0.0], clearance_m=0.01
    )
    open_side = _adaptive_destack_clearance(
        target, [target, left], [-1.0, 0.0, 0.0], clearance_m=0.01
    )

    assert enclosed.requires_straight_withdrawal
    assert np.isclose(enclosed.withdrawal_distance_m, 0.41)
    assert enclosed.blocking_carton_names == ("left", "right")
    assert not open_side.requires_straight_withdrawal
    assert np.isclose(open_side.withdrawal_distance_m, 0.0)


def test_adaptive_destack_treats_trailer_wall_as_a_side_blocker_without_using_its_length():
    target = OBB(
        center=np.array([1.21, 0.825, 1.25]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="target",
        category="carton",
    )
    inner = OBB(
        center=np.array([1.21, 0.275, 1.25]),
        half_extents=target.half_extents.copy(),
        rotation=np.eye(3),
        name="inner",
        category="carton",
    )
    wall = OBB(
        center=np.array([2.0, 1.20, 1.5]),
        half_extents=np.array([2.0, 0.025, 1.5]),
        rotation=np.eye(3),
        name="trailer_left_wall",
        category="trailer",
    )

    clearance = _adaptive_destack_clearance(
        target, [target, inner, wall], [-1.0, 0.0, 0.0], clearance_m=0.01
    )

    assert clearance.requires_straight_withdrawal
    assert np.isclose(clearance.withdrawal_distance_m, 0.43)
    assert clearance.blocking_carton_names == ("inner", "trailer_left_wall")


def test_reorientation_standoff_scales_with_carton_geometry_and_grasp_face():
    carton = OBB(
        center=np.array([1.21, 0.0, 1.0]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="carton",
        category="carton",
    )

    standoff = _reorientation_clearance_standoff(
        carton,
        np.array([-1.0, 0.0, 0.0]),
        0.43,
        clearance_m=0.01,
    )

    expected = 0.43 + np.linalg.norm(carton.half_extents) - 0.21 + 0.01
    assert np.isclose(standoff, expected)


def test_supported_edge_carton_withdraws_before_reorientation():
    upper = OBB(
        center=np.array([1.21, -0.825, 1.25]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="upper",
        category="carton",
    )
    lower = OBB(
        center=np.array([1.21, -0.825, 0.75]),
        half_extents=upper.half_extents.copy(),
        rotation=np.eye(3),
        name="lower_support",
        category="carton",
    )

    clearance = _adaptive_destack_clearance(
        upper,
        [upper, lower],
        np.array([-1.0, 0.0, 0.0]),
        clearance_m=0.01,
    )

    assert clearance.requires_straight_withdrawal
    assert np.isclose(clearance.withdrawal_distance_m, 0.43)
    assert clearance.blocking_carton_names == ("lower_support",)


def test_joint_path_unwraps_equivalent_wrist_angles_within_limits():
    robot = SimpleNamespace(
        joint_limits=np.array(
            [
                [-np.pi, np.pi],
                [-np.pi, np.pi],
                [-np.pi, np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
                [-np.pi, np.pi],
                [-2.0 * np.pi, 2.0 * np.pi],
            ]
        )
    )
    path = [
        np.array([0.0, 0.1, -0.2, 0.0, 0.3, 3.13]),
        np.array([0.0, 0.1, -0.2, 0.0, 0.3, -3.13]),
    ]

    continuous = _continuous_equivalent_joint_path(robot, path)

    assert abs(continuous[1][5] - continuous[0][5]) < 0.03
    assert robot.joint_limits[5, 0] <= continuous[1][5] <= robot.joint_limits[5, 1]


def test_post_release_escape_opposes_belt_and_uses_projected_tool_extent():
    tool_obb = OBB(
        center=np.zeros(3),
        half_extents=np.array([0.10, 0.30, 0.05]),
        rotation=np.eye(3),
        name="tool",
        category="robot",
    )

    class RobotStub:
        def fk(self, _q):
            return np.eye(4)

        def tool_collision_obbs(self, _q):
            return [tool_obb]

    target, audit = post_release_escape_target(
        RobotStub(),
        np.zeros(6),
        "cross_belt",
        {
            "post_release_surface_directions_world": {
                "cross_belt": [0.0, -1.0, 0.0]
            },
            "carried_box_clearance": 0.02,
        },
        normal_disengage_distance_m=0.05,
        vertical_lift_distance_m=0.25,
    )

    assert np.allclose(target, [0.0, 0.32, 0.20])
    assert audit["conveyor_escape_enabled"] is True
    assert np.isclose(audit["projected_tool_extent_m"], 0.30)
    assert np.isclose(audit["conveyor_escape_distance_m"], 0.32)


def test_post_release_escape_preserves_normal_lift_without_surface_direction():
    class RobotStub:
        def fk(self, _q):
            return np.eye(4)

    target, audit = post_release_escape_target(
        RobotStub(),
        np.zeros(6),
        "static_table",
        {},
        normal_disengage_distance_m=0.05,
        vertical_lift_distance_m=0.25,
    )

    assert np.allclose(target, [0.0, 0.0, 0.20])
    assert audit["conveyor_escape_enabled"] is False


def test_reorientation_wall_offset_recenters_only_a_wall_limited_edge_carton():
    carton = OBB(
        center=np.array([0.55, -0.825, 1.25]),
        half_extents=np.array([0.21, 0.275, 0.25]),
        rotation=np.eye(3),
        name="edge_carton",
        category="carton",
    )
    walls = [
        OBB(
            center=np.array([2.0, sign * 1.20, 1.5]),
            half_extents=np.array([2.0, 0.025, 1.5]),
            rotation=np.eye(3),
            name="trailer_left_wall" if sign > 0 else "trailer_right_wall",
            category="trailer",
        )
        for sign in (-1.0, 1.0)
    ]

    offset = _reorientation_wall_clearance_offset(
        carton,
        walls,
        np.array([-1.0, 0.0, 0.0]),
        clearance_m=0.01,
    )

    sphere_radius = np.linalg.norm(carton.half_extents)
    expected_y = -1.175 + sphere_radius + 0.01
    assert np.allclose(offset, [0.0, expected_y - carton.center[1], 0.0])

    centered = OBB(
        center=np.array([0.55, 0.0, 1.25]),
        half_extents=carton.half_extents,
        rotation=np.eye(3),
        name="centered_carton",
        category="carton",
    )
    assert np.allclose(
        _reorientation_wall_clearance_offset(
            centered,
            walls,
            np.array([-1.0, 0.0, 0.0]),
            clearance_m=0.01,
        ),
        np.zeros(3),
    )


def test_controlled_release_height_uses_strictest_physical_limit():
    height = controlled_release_height(
        {
            "carton_mass_kg": 7.0,
            "conveyor_release_height": 0.12,
            "conveyor_release_policy": {
                "desired_height_m": 0.05,
                "maximum_height_m": 0.04,
                "maximum_impact_velocity_m_s": 0.80,
                "maximum_drop_energy_j": 2.50,
            },
        }
    )

    assert np.isclose(height, 0.80**2 / (2.0 * 9.81))
    assert height < 0.04
