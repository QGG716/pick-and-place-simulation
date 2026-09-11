from __future__ import annotations

from unloading_sim.qualification import (
    BLOCKED_BY,
    FAIL,
    NOT_EVALUATED,
    PASS,
    ReplayQualificationPolicy,
    evaluate_replay_qualification,
)


def _passing_inputs() -> dict:
    return {
        "policy": ReplayQualificationPolicy(),
        "full_schedule_replayed": True,
        "collision_scope_complete": True,
        "unexpected_contact_count": 0,
        "premature_payload_conveyor_contact_count": 0,
        "peak_joint_error_rad": 0.02,
        "effort_limit_ratios": [0.2, 0.8],
        "grasp_expected": True,
        "grasp_enabled": True,
        "attachment_lost": False,
        "peak_attachment_position_error_m": 0.004,
        "peak_attachment_rotation_error_rad": 0.04,
        "payload_displacement_m": 0.9,
        "release_expected": True,
        "release_executed": True,
        "placement_expected": True,
        "placement_center_error_m": 0.012,
        "gripper_wrench_envelope_complete": True,
        "gripper_limits_from_configuration": True,
        "gripper_limits_calibrated": True,
        "production_release_adapter": True,
    }


def test_complete_production_replay_passes_all_checks():
    result = evaluate_replay_qualification(**_passing_inputs())

    assert result["qualification_passed"]
    assert not result["qualification_failures"]
    assert all(result["qualification_checks"].values())


def test_attachment_requires_both_position_and_rotation_evidence():
    inputs = _passing_inputs()
    inputs["peak_attachment_rotation_error_rad"] = 0.11

    result = evaluate_replay_qualification(**inputs)

    assert not result["qualification_passed"]
    assert result["qualification_check_details"]["payload_attachment_intact"]["status"] == FAIL
    assert result["qualification_check_details"]["placement_within_tolerance"]["status"] == BLOCKED_BY


def test_missing_placement_measurement_never_passes():
    inputs = _passing_inputs()
    inputs["placement_center_error_m"] = None

    result = evaluate_replay_qualification(**inputs)

    assert result["qualification_check_details"]["placement_within_tolerance"]["status"] == NOT_EVALUATED
    assert "placement_within_tolerance" in result["qualification_failures"]


def test_payload_touching_conveyor_before_release_is_a_hard_failure():
    inputs = _passing_inputs()
    inputs["premature_payload_conveyor_contact_count"] = 1

    result = evaluate_replay_qualification(**inputs)

    assert not result["qualification_passed"]
    assert (
        result["qualification_check_details"][
            "premature_payload_conveyor_contacts_clear"
        ]["status"]
        == FAIL
    )


def test_expected_conveyor_transport_requires_displacement_and_speed_evidence():
    inputs = _passing_inputs()
    inputs.update(
        conveyor_transport_expected=True,
        conveyor_transport_engaged=True,
        conveyor_transport_speed_within_tolerance=True,
    )

    result = evaluate_replay_qualification(**inputs)

    assert result["qualification_check_details"]["conveyor_transport_engaged"]["status"] == PASS
    assert (
        result["qualification_check_details"]["conveyor_transport_speed_within_tolerance"][
            "status"
        ]
        == PASS
    )


def test_missing_expected_conveyor_speed_evidence_never_passes():
    inputs = _passing_inputs()
    inputs.update(
        conveyor_transport_expected=True,
        conveyor_transport_engaged=True,
        conveyor_transport_speed_within_tolerance=None,
    )

    result = evaluate_replay_qualification(**inputs)

    assert not result["qualification_passed"]
    assert (
        result["qualification_check_details"]["conveyor_transport_speed_within_tolerance"][
            "status"
        ]
        == NOT_EVALUATED
    )


def test_truncated_replay_never_qualifies_payload_transport():
    inputs = _passing_inputs()
    inputs["full_schedule_replayed"] = False

    result = evaluate_replay_qualification(**inputs)

    assert result["qualification_check_details"]["full_schedule_replayed"]["status"] == FAIL
    assert result["qualification_check_details"]["payload_attachment_intact"]["status"] == NOT_EVALUATED
    assert not result["qualification_passed"]


def test_uncalibrated_1800n_hardware_maximum_is_not_a_working_certificate():
    inputs = _passing_inputs()
    inputs["gripper_limits_calibrated"] = False

    result = evaluate_replay_qualification(**inputs)

    assert result["qualification_check_details"]["gripper_limits_calibrated"]["status"] == FAIL
    assert not result["qualification_passed"]


def test_disabled_self_collision_cannot_produce_a_collision_certificate():
    inputs = _passing_inputs()
    inputs["collision_scope_complete"] = False

    result = evaluate_replay_qualification(**inputs)

    assert result["qualification_check_details"]["collision_scope_complete"]["status"] == FAIL
    assert not result["qualification_passed"]


def test_missing_shear_or_moment_blocks_attachment_qualification():
    inputs = _passing_inputs()
    inputs["gripper_wrench_envelope_complete"] = False

    result = evaluate_replay_qualification(**inputs)

    assert result["qualification_check_details"]["gripper_wrench_envelope_complete"]["status"] == FAIL
    assert result["qualification_check_details"]["payload_attachment_intact"]["status"] == NOT_EVALUATED
    assert result["qualification_check_details"]["placement_within_tolerance"]["status"] == BLOCKED_BY


def test_explicit_ideal_cups_simulation_is_independent_from_machine_certification():
    inputs = _passing_inputs()
    inputs.update(
        ideal_holding_capacity_assumption=True,
        gripper_wrench_envelope_complete=False,
        gripper_limits_from_configuration=False,
        gripper_limits_calibrated=False,
        machine_certification_status="NOT_EVALUATED",
    )
    result = evaluate_replay_qualification(**inputs)
    assert result["simulation_qualification_passed"]
    assert not result["machine_certification_gates_simulation"]
    assert result["machine_certification_status"] == "NOT_EVALUATED"
    inputs["unexpected_contact_count"] = 1
    result = evaluate_replay_qualification(**inputs)
    assert not result["simulation_qualification_passed"]
    assert result["qualification_check_details"]["unexpected_contacts_clear"]["status"] == FAIL
