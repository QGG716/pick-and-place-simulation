from dataclasses import replace

import numpy as np
import pytest

from unloading_sim.robot_load import (
    BoxSpatialLoad,
    BoxLoad,
    LoadCase,
    MotionLoad,
    PoseSpecificLoadCase,
    ToolLoad,
    box_spatial_load,
    build_kinematics_from_limits,
    gravity_moment_about_axis,
    inertia_about_axis,
    load_robot_limits,
    load_qualification_case,
    load_tool,
    qualify_load,
    qualify_load_v2,
    rigid_transform,
    rotate_inertia,
    spatial_inertia_at_point,
)
from unloading_sim.robot_load.model import cuboid_inertia_at_com, parallel_axis


def _root():
    from pathlib import Path
    return Path(__file__).resolve().parents[1]


def _case(box_mass=5.0, depth=0.3, safety=1.0):
    robot = load_robot_limits(_root() / "configs/robots/fanuc_m20id_35.yaml")
    tool = load_tool(_root() / "configs/tools/unloading_gripper.yaml")
    box = BoxLoad(
        size_xyz_m=np.array([depth, 0.3, 0.3]),
        mass_kg=box_mass,
        grasp_face="front_center",
        grasp_point_xyz_m=np.zeros(3),
        estimated_com_xyz_m=np.array([0.5 * depth, 0.0, 0.0]),
    )
    return LoadCase(robot, tool, box, MotionLoad(0.0, 0.0, safety))


def test_cuboid_inertia_and_parallel_axis_are_physical():
    inertia = cuboid_inertia_at_com(12.0, [1.0, 2.0, 3.0])
    assert np.diag(inertia) == pytest.approx([13.0, 10.0, 5.0])
    translated = parallel_axis(inertia, 12.0, [1.0, 0.0, 0.0])
    assert np.diag(translated) == pytest.approx([13.0, 22.0, 17.0])


def test_known_failure_precedes_unknown_com_curve():
    result = qualify_load(_case(box_mass=25.0, safety=1.05))
    assert result.qualification == "FAIL_PAYLOAD"
    assert result.criteria["com"] == "NOT_EVALUATED"
    assert result.intermediate["safety_factored_external_mass_kg"] == pytest.approx(36.75)


def test_otherwise_passing_known_limits_remain_not_evaluated_for_com():
    result = qualify_load(_case(box_mass=1.0, depth=0.05))
    assert result.qualification == "NOT_EVALUATED"
    assert result.criteria["payload"] == "PASS"
    assert result.criteria["com"] == "NOT_EVALUATED"


def test_box_inertia_and_parallel_axis_increase_flange_exposure():
    shallow = qualify_load(_case(box_mass=8.0, depth=0.1))
    deep = qualify_load(_case(box_mass=8.0, depth=0.8))
    shallow_i = np.asarray(shallow.intermediate["safety_factored_inertia_kg_m2_J4_J5_J6"])
    deep_i = np.asarray(deep.intermediate["safety_factored_inertia_kg_m2_J4_J5_J6"])
    assert deep_i[1] > shallow_i[1]
    assert deep_i[2] > shallow_i[2]


def test_dynamic_acceleration_adds_moment_and_is_audited():
    static = qualify_load(_case(box_mass=5.0, depth=0.3))
    dynamic_case = replace(_case(box_mass=5.0, depth=0.3), motion=MotionLoad(2.0, 3.0, 1.1))
    dynamic = qualify_load(dynamic_case)
    assert max(dynamic.intermediate["safety_factored_moment_nm_J4_J5_J6"]) > max(static.intermediate["safety_factored_moment_nm_J4_J5_J6"])
    assert dynamic.intermediate["method"] == "flange_axis_engineering_estimate_v1"


def test_invalid_inertia_tensor_is_rejected():
    with pytest.raises(ValueError, match="symmetric"):
        ToolLoad(1.0, np.zeros(3), np.array([[1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), np.zeros(3), {})


def test_complete_si_case_config_is_loadable_and_fail_closed():
    case = load_qualification_case(_root() / "configs/qualification/fanuc_m20id35_25kg_front.yaml")
    result = qualify_load(case)
    assert case.robot.model == "fanuc_m20id_35"
    assert result.qualification in {"FAIL_WRIST_MOMENT", "FAIL_WRIST_INERTIA"}


def test_v2_j6_axial_point_mass_adds_zero_parallel_axis_inertia():
    # Test 1: a point on the J6 X axis has no perpendicular radius.
    tensor = spatial_inertia_at_point(np.zeros((3, 3)), 10.0, [0.5, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert inertia_about_axis(tensor, [1.0, 0.0, 0.0]) == pytest.approx(0.0, abs=1e-12)


def test_v2_j6_axial_translation_does_not_change_axis_inertia():
    # Test 2: translation parallel to the axis cannot create m*lambda^2.
    values = []
    intrinsic = np.diag([0.04, 0.05, 0.06])
    for x in (0.1, 0.6):
        tensor = spatial_inertia_at_point(intrinsic, 8.0, [x, 0.0, 0.0], np.zeros(3))
        values.append(inertia_about_axis(tensor, [1.0, 0.0, 0.0]))
    assert values[0] == pytest.approx(values[1], abs=1e-12)


def test_v2_j5_orthogonal_axis_inertia_increases_with_tool_forward_distance():
    # Test 3: the same X translation is perpendicular to a Y-axis J5.
    def projected(x):
        tensor = spatial_inertia_at_point(np.zeros((3, 3)), 8.0, [x, 0.0, 0.0], np.zeros(3))
        return inertia_about_axis(tensor, [0.0, 1.0, 0.0])

    assert projected(0.6) - projected(0.1) == pytest.approx(8.0 * (0.6**2 - 0.1**2))


def test_v2_gravity_moment_uses_cross_product_and_axis_projection():
    # Test 4: r=X and gravity=-Z produces +Y moment only.
    j6 = gravity_moment_about_axis(5.0, [0.5, 0.0, 0.0], np.zeros(3), [1.0, 0.0, 0.0])
    j5 = gravity_moment_about_axis(5.0, [0.5, 0.0, 0.0], np.zeros(3), [0.0, 1.0, 0.0])
    assert j6 == pytest.approx(0.0, abs=1e-12)
    assert j5 == pytest.approx(5.0 * 9.80665 * 0.5)


def test_v2_rotating_tool_transforms_com_tensor_and_gravity_projection():
    # Test 5: a +90 degree Z rotation maps +X CoM to +Y and swaps Ixx/Iyy.
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    com = rotation @ np.array([0.5, 0.0, 0.0])
    inertia = rotate_inertia(np.diag([1.0, 2.0, 3.0]), rotation)
    assert com == pytest.approx([0.0, 0.5, 0.0])
    assert np.diag(inertia) == pytest.approx([2.0, 1.0, 3.0])
    assert gravity_moment_about_axis(5.0, com, np.zeros(3), [1.0, 0.0, 0.0]) == pytest.approx(-5.0 * 9.80665 * 0.5)
    assert gravity_moment_about_axis(5.0, com, np.zeros(3), [0.0, 1.0, 0.0]) == pytest.approx(0.0, abs=1e-12)


def test_v2_tcp_label_does_not_change_unchanged_physical_mass_properties():
    # Test 6: move T while compensating T<-box so C_tool and C_box stay fixed.
    limits = load_robot_limits(_root() / "configs/robots/fanuc_m20id_35.yaml")
    kinematics = build_kinematics_from_limits(limits)
    tool_a = load_tool(_root() / "configs/tools/unloading_gripper_8kg.yaml")
    tool_b = replace(tool_a, tcp_xyz_m=np.array([0.45, 0.0, 0.0]))
    box_a = box_spatial_load([0.3, 0.3, 0.3], 10.0, "front_center")
    box_b = BoxSpatialLoad(
        box_a.mass_kg,
        box_a.size_xyz_m,
        box_a.com_in_box_frame_m,
        rigid_transform(np.eye(3), [-0.20, 0.0, 0.0]) @ box_a.pose_box_to_tcp,
        box_a.grasp,
    )
    q = np.asarray(limits.qualification_joint_pose_q)
    a = qualify_load_v2(PoseSpecificLoadCase(limits, kinematics, tool_a, box_a, q))
    b = qualify_load_v2(PoseSpecificLoadCase(limits, kinematics, tool_b, box_b, q))
    for key in (
        "tool_com_xyz",
        "box_com_xyz",
        "combined_com_xyz",
        "gravity_moment_j4",
        "gravity_moment_j5",
        "gravity_moment_j6",
        "inertia_j4",
        "inertia_j5",
        "inertia_j6",
    ):
        assert a.intermediate[key] == pytest.approx(b.intermediate[key], abs=1e-10)


def test_v2_wrist_axes_are_urdf_fk_outputs_and_change_with_pose():
    limits = load_robot_limits(_root() / "configs/robots/fanuc_m20id_35.yaml")
    kinematics = build_kinematics_from_limits(limits)
    q1 = np.asarray(limits.qualification_joint_pose_q)
    q2 = q1.copy()
    q2[4] += 0.4
    axes1 = kinematics.joint_axis_frames(q1)
    axes2 = kinematics.joint_axis_frames(q2)
    assert not np.allclose(axes1["J6"][1], axes2["J6"][1])
    assert np.linalg.norm(axes1["J6"][1]) == pytest.approx(1.0)


def test_v2_known_limit_statuses_are_separate_from_vendor_load_diagram():
    limits = load_robot_limits(_root() / "configs/robots/fanuc_m20id_35.yaml")
    kinematics = build_kinematics_from_limits(limits)
    tool = load_tool(_root() / "configs/tools/unloading_gripper_8kg.yaml")
    q = np.asarray(limits.qualification_joint_pose_q)

    def status(size, mass, grasp="front_center"):
        result = qualify_load_v2(
            PoseSpecificLoadCase(limits, kinematics, tool, box_spatial_load(size, mass, grasp), q)
        )
        assert result.criteria["vendor_load_diagram"] == "NOT_EVALUATED_VENDOR_LOAD_DIAGRAM"
        assert result.qualification != "PASS"
        return result.qualification

    assert status([0.15, 0.15, 0.15], 1.0) == "ENGINEERING_REFERENCES_WITHIN_PUBLIC_VALUES"
    assert status([0.15, 0.15, 0.15], 25.0) == "ENGINEERING_AXIS_INERTIA_REFERENCE_EXCEEDED"
    assert status([0.8, 0.8, 0.8], 25.0, "front_offset") == "POSE_SPECIFIC_GRAVITY_MOMENT_REFERENCE_EXCEEDED"
    assert status([0.15, 0.15, 0.15], 28.0) == "FAIL_KNOWN_PAYLOAD"


def test_exact_500x400x350_15kg_cuboid_inertia():
    inertia = cuboid_inertia_at_com(15.0, [0.5, 0.4, 0.35])
    assert inertia == pytest.approx(np.diag([0.353125, 0.465625, 0.5125]), abs=1e-12)


@pytest.mark.parametrize(
    ("rotation", "expected_diagonal"),
    [
        (np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float), [0.353125, 0.5125, 0.465625]),
        (np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float), [0.5125, 0.465625, 0.353125]),
        (np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float), [0.465625, 0.353125, 0.5125]),
    ],
)
def test_exact_cuboid_inertia_rotates_as_tensor(rotation, expected_diagonal):
    local = cuboid_inertia_at_com(15.0, [0.5, 0.4, 0.35])
    world = rotate_inertia(local, rotation)
    assert np.diag(world) == pytest.approx(expected_diagonal, abs=1e-12)
    assert np.linalg.eigvalsh(world) == pytest.approx(np.linalg.eigvalsh(local), abs=1e-12)
