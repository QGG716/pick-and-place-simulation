from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from unloading_sim.robot_load import (
    LoadAwareIKQualifier,
    TaskLoadRequest,
    TaskLoadSearchOptions,
    VendorLoadEvaluation,
    VendorLoadStatus,
    box_spatial_load,
    build_kinematics_from_limits,
    load_robot_limits,
    load_tool,
    rigid_transform,
    scale_engineering_tool_model,
)


ROOT = Path(__file__).resolve().parents[1]


def _physical_target(robot, kinematics, tool, q):
    flange = kinematics.named_link_frames(q)[robot.flange_link]
    return flange @ rigid_transform(np.eye(3), tool.tcp_xyz_m)


def _request(box_mass=1.0):
    robot = load_robot_limits(ROOT / "configs/robots/fanuc_m20id_35.yaml")
    kinematics = build_kinematics_from_limits(robot)
    tool = load_tool(ROOT / "configs/tools/unloading_gripper_8kg.yaml")
    q = np.asarray(robot.qualification_joint_pose_q)
    target = _physical_target(robot, kinematics, tool, q)
    return TaskLoadRequest(
        robot,
        kinematics,
        tool,
        box_spatial_load([0.15, 0.15, 0.15], box_mass, "front_center"),
        target,
        target[:3, 0],
        current_q=q,
    )


def _small_search():
    return TaskLoadSearchOptions(
        minimum_candidate_count=1,
        random_seeds_per_orientation=1,
        solutions_per_orientation=1,
        normal_tolerance_rad=0.0,
        roll_samples_rad=(0.0,),
        max_iterations=40,
    )


def test_engineering_tool_scaling_changes_mass_and_full_tensor_together():
    reference = load_tool(ROOT / "configs/tools/unloading_gripper_8kg.yaml")
    model = scale_engineering_tool_model(reference, 5.0)
    assert model.mass_kg == 5.0
    assert model.com_xyz_m == pytest.approx(reference.com_xyz_m)
    assert model.tcp_xyz_m == pytest.approx(reference.tcp_xyz_m)
    assert model.inertia_at_com_kg_m2 == pytest.approx(0.625 * reference.inertia_at_com_kg_m2)
    assert model.source["type"] == "ENGINEERING_TOOL_MODEL"


def test_task_load_known_pass_remains_vendor_unknown_by_default():
    result = LoadAwareIKQualifier(_small_search()).evaluate(_request(1.0))
    assert result.known_limits_status == "ENGINEERING_REFERENCES_WITHIN_PUBLIC_VALUES"
    assert result.qualification == "ENGINEERING_REFERENCES_PASS_VENDOR_QUALIFICATION_PENDING"
    assert result.vendor_load_status == "NOT_EVALUATED"
    assert result.primary_failure_reason == "VENDOR_QUALIFICATION_PENDING"
    assert result.best_q is not None
    assert result.second_best_q is not None  # legal J6 wraps are retained


def test_task_load_can_only_be_fully_qualified_by_vendor_pass():
    class PassingVendor:
        def evaluate(self, **_):
            return VendorLoadEvaluation(VendorLoadStatus.PASS, "traceable test fixture")

    result = LoadAwareIKQualifier(_small_search(), vendor_evaluator=PassingVendor()).evaluate(_request(1.0))
    assert result.qualification == "TASK_LOAD_QUALIFIED"
    assert result.vendor_load_status == "PASS"


def test_collision_is_a_hard_failure_even_when_known_load_limits_pass():
    request = _request(1.0)
    request = TaskLoadRequest(
        request.robot,
        request.kinematics,
        request.tool,
        request.box,
        request.tcp_pose_world,
        request.grasp_normal_world,
        current_q=request.current_q,
        collision_evaluator=lambda _q: (False, 0.0),
    )
    result = LoadAwareIKQualifier(_small_search()).evaluate(request)
    assert result.known_limits_status == "ENGINEERING_REFERENCES_WITHIN_PUBLIC_VALUES"
    assert result.qualification == "TASK_GEOMETRIC_CONSTRAINT_FAIL"
    assert "COLLISION_FAIL" in result.all_failure_reasons


def test_task_load_reports_axis_specific_hard_failures_and_minimum_inertia():
    result = LoadAwareIKQualifier(_small_search()).evaluate(_request(25.0))
    assert result.qualification == "ENGINEERING_LOAD_RISK_VENDOR_QUALIFICATION_PENDING"
    assert result.minimum_j4_j5_inertia_utilization is not None
    assert result.minimum_j4_j5_inertia_utilization > 1.0
    assert result.second_best_q is not None
    assert any(
        reason in result.all_failure_reasons
        for reason in (
            "ENGINEERING_J4_AXIS_INERTIA_PUBLIC_REFERENCE_EXCEEDED",
            "ENGINEERING_J5_AXIS_INERTIA_PUBLIC_REFERENCE_EXCEEDED",
        )
    )


def test_task_request_rejects_grasp_normal_not_aligned_with_tcp_axis():
    request = _request(1.0)
    with pytest.raises(ValueError, match="parallel"):
        TaskLoadRequest(
            request.robot,
            request.kinematics,
            request.tool,
            request.box,
            request.tcp_pose_world,
            [0.0, 1.0, 0.0],
        )
