import numpy as np

from studies.fanuc_m20id35_cross_section.model import (
    STATE_A,
    STATE_B,
    STATE_C,
    aggregate_cell_states,
    carton_inside_trailer,
    inclusive_grid,
    load_config,
    load_gripper_model,
    target_pose,
    target_rotations,
    wrist_load_check,
)
from unloading_sim.robot import URDFRobot6


CONFIG = "studies/fanuc_m20id35_cross_section/study_config.json"


def test_required_grid_is_endpoint_inclusive_and_50mm():
    cfg, _ = load_config(CONFIG)
    x = inclusive_grid(*cfg["grid"]["cross_section_x_range_m"], cfg["grid"]["spacing_m"])
    z = inclusive_grid(*cfg["grid"]["z_range_m"], cfg["grid"]["spacing_m"])
    assert x.shape == (47,)
    assert z.shape == (55,)
    assert np.allclose(np.diff(x), 0.05)
    assert np.allclose(np.diff(z), 0.05)
    assert (x[0], x[-1], z[0], z[-1]) == (-1.15, 1.15, 0.0, 2.7)


def test_front_and_side_target_normals_follow_world_convention():
    cfg, _ = load_config(CONFIG)
    front, centre, outward = target_pose(cfg, 0.4, 1.2, [0.5, 0.4, 0.35], "front")
    assert np.allclose(front, [1.0, 0.4, 1.2])
    assert np.allclose(centre, [1.175, 0.4, 1.2])
    assert np.allclose(outward, [-1.0, 0.0, 0.0])
    side, _, side_outward = target_pose(cfg, 0.4, 1.2, [0.5, 0.4, 0.35], "side")
    assert np.allclose(side, [1.175, 0.15, 1.2])
    assert np.allclose(side_outward, [0.0, -1.0, 0.0])
    for rotation in target_rotations("front", 0.4):
        assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        assert np.allclose(rotation[:, 2], [1.0, 0.0, 0.0])


def test_carton_cells_outside_physical_trailer_are_rejected():
    cfg, _ = load_config(CONFIG)
    size = [0.7, 0.5, 0.5]
    assert carton_inside_trailer(cfg, 0.0, 1.2, size)
    assert not carton_inside_trailer(cfg, -1.15, 1.2, size)
    assert not carton_inside_trailer(cfg, 0.0, 0.0, size)
    assert not carton_inside_trailer(cfg, 0.0, 2.7, size)


def test_step_audit_supplies_rigid_subcomponents_and_requested_tool_length():
    cfg, path = load_config(CONFIG)
    boxes, audit = load_gripper_model(cfg, path)
    assert len(boxes) == len(audit["rigid_collision_bounding_boxes_step_mm"])
    assert 3 < len(boxes) < audit["solid_occurrence_count"]
    assert all(np.asarray(box).shape == (6,) and np.all(np.asarray(box)[3:] > 0) for box in boxes)
    assert cfg["gripper"]["requested_axial_length_m"] == 0.2


def test_aggregate_state_is_conservative_across_required_sizes():
    assert aggregate_cell_states([STATE_A, STATE_A, STATE_A]) == STATE_A
    assert aggregate_cell_states([STATE_C, STATE_C, STATE_C]) == STATE_C
    assert aggregate_cell_states([STATE_C, STATE_B, STATE_A]) == STATE_B


def test_wrist_load_is_invariant_to_rigid_base_translation():
    cfg, path = load_config(CONFIG)
    _boxes, audit = load_gripper_model(cfg, path)
    robot = URDFRobot6.fanuc_m20id35(
        urdf_path="assets/robots/fanuc_m20id35/m20_35_18d.urdf",
        base_position=[0.0, 0.0, 0.0],
        tool_length=cfg["gripper"]["requested_axial_length_m"],
    )
    q = np.array([0.1, 0.3, -0.7, 2.8, -1.0, 0.2])
    pose = robot.fk(q)
    shifted = pose.copy()
    shifted[:3, 3] += [-0.25, 0.12, 0.8]
    first = wrist_load_check(cfg, robot, q, pose, audit, 15.0, [0.5, 0.4, 0.35], "front")
    second = wrist_load_check(cfg, robot, q, shifted, audit, 15.0, [0.5, 0.4, 0.35], "front")
    assert np.allclose(first.moments_nm, second.moments_nm)
    assert np.allclose(first.inertias_kg_m2, second.inertias_kg_m2)
