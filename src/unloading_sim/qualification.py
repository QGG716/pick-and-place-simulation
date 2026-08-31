"""Backend-neutral, fail-closed digital-twin qualification rules.

Simulation adapters collect measurements.  This module decides whether those
measurements constitute evidence, so a missing sensor value or a truncated
replay can never silently become a passing production check.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


PASS = "PASS"
FAIL = "FAIL"
NOT_EVALUATED = "NOT_EVALUATED"
BLOCKED_BY = "BLOCKED_BY"


@dataclass(frozen=True)
class ReplayQualificationPolicy:
    tracking_error_limit_rad: float = 0.05
    attachment_position_tolerance_m: float = 0.02
    attachment_rotation_tolerance_rad: float = 0.10
    placement_tolerance_m: float = 0.08
    minimum_payload_displacement_m: float = 0.02

    def __post_init__(self) -> None:
        values = {
            "tracking_error_limit_rad": self.tracking_error_limit_rad,
            "attachment_position_tolerance_m": self.attachment_position_tolerance_m,
            "attachment_rotation_tolerance_rad": self.attachment_rotation_tolerance_rad,
            "placement_tolerance_m": self.placement_tolerance_m,
            "minimum_payload_displacement_m": self.minimum_payload_displacement_m,
        }
        invalid = [name for name, value in values.items() if not np.isfinite(value) or value <= 0.0]
        if invalid:
            raise ValueError(f"qualification tolerances must be finite and positive: {invalid}")


def _detail(status: str, reason: str, **evidence: Any) -> dict[str, Any]:
    result = {"status": status, "reason": reason}
    result.update(evidence)
    return result


def evaluate_replay_qualification(
    *,
    policy: ReplayQualificationPolicy,
    full_schedule_replayed: bool,
    collision_scope_complete: bool,
    unexpected_contact_count: int | None,
    premature_payload_conveyor_contact_count: int | None,
    peak_joint_error_rad: float | None,
    effort_limit_ratios: Sequence[float] | np.ndarray | None,
    grasp_expected: bool,
    grasp_enabled: bool,
    attachment_lost: bool,
    peak_attachment_position_error_m: float | None,
    peak_attachment_rotation_error_rad: float | None,
    payload_displacement_m: float | None,
    release_expected: bool,
    release_executed: bool,
    placement_expected: bool,
    placement_center_error_m: float | None,
    gripper_wrench_envelope_complete: bool,
    gripper_limits_from_configuration: bool,
    gripper_limits_calibrated: bool,
    production_release_adapter: bool,
    conveyor_transport_expected: bool = False,
    conveyor_transport_engaged: bool | None = None,
    conveyor_transport_speed_within_tolerance: bool | None = None,
) -> dict[str, Any]:
    """Evaluate a payload replay and retain causal dependencies in the result."""
    checks: dict[str, dict[str, Any]] = {}

    checks["full_schedule_replayed"] = _detail(
        PASS if full_schedule_replayed else FAIL,
        "complete command schedule replayed" if full_schedule_replayed else "replay was truncated",
    )

    checks["collision_scope_complete"] = _detail(
        PASS if collision_scope_complete else FAIL,
        "robot self-collision and robot/payload scene contacts were monitored"
        if collision_scope_complete
        else "collision reporting did not cover the complete qualification scope",
    )

    if unexpected_contact_count is None:
        checks["unexpected_contacts_clear"] = _detail(
            NOT_EVALUATED, "contact reporting is unavailable"
        )
    else:
        checks["unexpected_contacts_clear"] = _detail(
            PASS if unexpected_contact_count == 0 else FAIL,
            "no unexpected contacts" if unexpected_contact_count == 0 else "unexpected contacts occurred",
            unexpected_contact_count=int(unexpected_contact_count),
        )

    if premature_payload_conveyor_contact_count is None:
        checks["premature_payload_conveyor_contacts_clear"] = _detail(
            NOT_EVALUATED, "release-relative payload/conveyor contact timing is unavailable"
        )
    else:
        checks["premature_payload_conveyor_contacts_clear"] = _detail(
            PASS if premature_payload_conveyor_contact_count == 0 else FAIL,
            "payload did not touch a conveyor before release"
            if premature_payload_conveyor_contact_count == 0
            else "payload touched a conveyor while still attached",
            premature_payload_conveyor_contact_count=int(
                premature_payload_conveyor_contact_count
            ),
        )

    if peak_joint_error_rad is None or not np.isfinite(peak_joint_error_rad):
        checks["joint_tracking_within_limit"] = _detail(
            NOT_EVALUATED, "peak joint tracking error is unavailable"
        )
    else:
        checks["joint_tracking_within_limit"] = _detail(
            PASS if peak_joint_error_rad <= policy.tracking_error_limit_rad else FAIL,
            "joint tracking is within limit"
            if peak_joint_error_rad <= policy.tracking_error_limit_rad
            else "joint tracking exceeds limit",
            measured=float(peak_joint_error_rad),
            limit=float(policy.tracking_error_limit_rad),
            unit="rad",
        )

    ratios = None if effort_limit_ratios is None else np.asarray(effort_limit_ratios, dtype=float)
    if ratios is None or ratios.size == 0 or not np.all(np.isfinite(ratios)):
        checks["joint_efforts_within_limit"] = _detail(
            NOT_EVALUATED, "joint effort ratios are unavailable"
        )
    else:
        maximum_ratio = float(np.max(ratios))
        checks["joint_efforts_within_limit"] = _detail(
            PASS if maximum_ratio <= 1.0 else FAIL,
            "joint efforts are within configured limits"
            if maximum_ratio <= 1.0
            else "joint effort exceeds a configured limit",
            maximum_ratio=maximum_ratio,
        )

    if not grasp_expected:
        attachment = _detail(NOT_EVALUATED, "payload qualification requires a grasp event")
    elif not full_schedule_replayed:
        attachment = _detail(NOT_EVALUATED, "full transport was not replayed")
    elif not gripper_wrench_envelope_complete:
        attachment = _detail(
            NOT_EVALUATED,
            "attachment simulation lacks a complete axial, shear, and moment envelope",
        )
    elif not grasp_enabled:
        attachment = _detail(FAIL, "gripper did not close on the target payload")
    elif attachment_lost:
        attachment = _detail(FAIL, "gripper released the target before the release event")
    elif any(
        value is None or not np.isfinite(value)
        for value in (
            peak_attachment_position_error_m,
            peak_attachment_rotation_error_rad,
            payload_displacement_m,
        )
    ):
        attachment = _detail(NOT_EVALUATED, "attachment pose or payload-motion evidence is missing")
    else:
        position_ok = bool(
            peak_attachment_position_error_m <= policy.attachment_position_tolerance_m
        )
        rotation_ok = bool(
            peak_attachment_rotation_error_rad <= policy.attachment_rotation_tolerance_rad
        )
        motion_ok = bool(payload_displacement_m > policy.minimum_payload_displacement_m)
        attachment = _detail(
            PASS if position_ok and rotation_ok and motion_ok else FAIL,
            "payload remained attached through transport"
            if position_ok and rotation_ok and motion_ok
            else "payload attachment or transport evidence exceeded its limit",
            peak_position_error_m=float(peak_attachment_position_error_m),
            position_tolerance_m=float(policy.attachment_position_tolerance_m),
            peak_rotation_error_rad=float(peak_attachment_rotation_error_rad),
            rotation_tolerance_rad=float(policy.attachment_rotation_tolerance_rad),
            payload_displacement_m=float(payload_displacement_m),
            minimum_payload_displacement_m=float(policy.minimum_payload_displacement_m),
        )
    checks["payload_attachment_intact"] = attachment

    if not release_expected:
        release = _detail(NOT_EVALUATED, "payload qualification requires a release event")
    elif not full_schedule_replayed:
        release = _detail(NOT_EVALUATED, "release event was outside the replayed interval")
    else:
        release = _detail(
            PASS if release_executed else FAIL,
            "release event executed" if release_executed else "release event did not execute",
        )
    checks["release_event_executed"] = release

    if attachment["status"] != PASS:
        placement = _detail(
            BLOCKED_BY,
            "placement cannot be evaluated because payload transport did not pass",
            blocked_by="payload_attachment_intact",
        )
    elif release["status"] != PASS:
        placement = _detail(
            BLOCKED_BY,
            "placement cannot be evaluated because release did not pass",
            blocked_by="release_event_executed",
        )
    elif not placement_expected:
        placement = _detail(NOT_EVALUATED, "expected placement centre is unavailable")
    elif placement_center_error_m is None or not np.isfinite(placement_center_error_m):
        placement = _detail(NOT_EVALUATED, "settled placement measurement is unavailable")
    else:
        placement = _detail(
            PASS if placement_center_error_m <= policy.placement_tolerance_m else FAIL,
            "settled placement is within tolerance"
            if placement_center_error_m <= policy.placement_tolerance_m
            else "settled placement exceeds tolerance",
            measured=float(placement_center_error_m),
            limit=float(policy.placement_tolerance_m),
            unit="m",
        )
    checks["placement_within_tolerance"] = placement

    checks["gripper_wrench_envelope_complete"] = _detail(
        PASS if gripper_wrench_envelope_complete else FAIL,
        "axial, shear, and moment limits are all available"
        if gripper_wrench_envelope_complete
        else "one or more axial, shear, or moment limits are unavailable",
    )
    checks["gripper_limits_from_configuration"] = _detail(
        PASS if gripper_limits_from_configuration else FAIL,
        "gripper limits came from the replay bundle"
        if gripper_limits_from_configuration
        else "diagnostic gripper overrides were used",
    )
    checks["gripper_limits_calibrated"] = _detail(
        PASS if gripper_limits_calibrated else FAIL,
        "gripper working envelope is calibrated"
        if gripper_limits_calibrated
        else "gripper values are unqualified hardware or simulation assumptions",
    )
    checks["production_release_adapter"] = _detail(
        PASS if production_release_adapter else FAIL,
        "the original payload was opened by the production release adapter"
        if production_release_adapter
        else "release used a diagnostic state-handoff adapter",
    )

    if conveyor_transport_expected:
        if conveyor_transport_engaged is None:
            checks["conveyor_transport_engaged"] = _detail(
                NOT_EVALUATED, "post-landing conveyor displacement is unavailable"
            )
        else:
            checks["conveyor_transport_engaged"] = _detail(
                PASS if conveyor_transport_engaged else FAIL,
                "released payload was transported by the conveyor"
                if conveyor_transport_engaged
                else "released payload did not reach the minimum conveyor displacement",
            )
        if conveyor_transport_speed_within_tolerance is None:
            checks["conveyor_transport_speed_within_tolerance"] = _detail(
                NOT_EVALUATED, "post-landing conveyor speed evidence is unavailable"
            )
        else:
            checks["conveyor_transport_speed_within_tolerance"] = _detail(
                PASS if conveyor_transport_speed_within_tolerance else FAIL,
                "payload conveyor speed is within tolerance"
                if conveyor_transport_speed_within_tolerance
                else "payload conveyor speed is outside tolerance",
            )

    boolean_checks = {name: detail["status"] == PASS for name, detail in checks.items()}
    failures = [name for name, passed in boolean_checks.items() if not passed]
    return {
        "model": "fail_closed_payload_qualification_v2",
        "qualification_checks": boolean_checks,
        "qualification_check_details": checks,
        "qualification_failures": failures,
        "qualification_passed": not failures,
    }
