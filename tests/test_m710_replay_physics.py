from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.m710_replay_physics import (
    audit_payload_support_contact,
    audit_surface_attachment_contact,
    select_active_conveyor_surfaces,
)


IDENTITY = np.eye(3).tolist()


def _contact_audit(*, body_x: float = -0.2125, rotation=None, max_gap: float = 0.002):
    return audit_surface_attachment_contact(
        grasp_body_position_m=[body_x, 0.0, 0.0],
        grasp_body_rotation=IDENTITY if rotation is None else rotation,
        contact_plane_from_grasp_body_m=0.2125,
        active_cup_offsets_yz_m=[[-0.10, -0.20], [0.10, 0.20]],
        target_center_m=[0.3, 0.0, 0.0],
        target_rotation=IDENTITY,
        target_size_m=[0.6, 0.4, 0.6],
        max_attachment_gap_m=max_gap,
        max_normal_misalignment_rad=np.deg2rad(5.0),
    )


def test_surface_attachment_uses_nominal_compressed_plane_and_two_mm_gap():
    exact = _contact_audit()
    at_limit = _contact_audit(body_x=-0.2145)
    too_far = _contact_audit(body_x=-0.2146)

    assert exact.accepted and exact.signed_gaps_m == pytest.approx([0.0, 0.0])
    assert at_limit.accepted and at_limit.signed_gaps_m == pytest.approx([0.002, 0.002])
    assert not too_far.accepted
    assert too_far.reason == "PHYSICAL_CONTACT_GAP_EXCEEDS_LIMIT"


def test_surface_attachment_rejects_penetration_miss_and_wrong_normal():
    penetrating = _contact_audit(body_x=-0.2120)
    missed = audit_surface_attachment_contact(
        grasp_body_position_m=[-0.2125, 0.5, 0.0],
        grasp_body_rotation=IDENTITY,
        contact_plane_from_grasp_body_m=0.2125,
        active_cup_offsets_yz_m=[[0.0, 0.0]],
        target_center_m=[0.3, 0.0, 0.0],
        target_rotation=IDENTITY,
        target_size_m=[0.6, 0.4, 0.6],
        max_attachment_gap_m=0.002,
        max_normal_misalignment_rad=np.deg2rad(5.0),
    )
    yaw = np.deg2rad(10.0)
    wrong_normal_rotation = np.asarray(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )
    contact_offset = wrong_normal_rotation @ np.asarray([0.2125, 0.0, 0.0])
    wrong_normal = audit_surface_attachment_contact(
        grasp_body_position_m=(np.asarray([0.0, 0.0, 0.0]) - contact_offset).tolist(),
        grasp_body_rotation=wrong_normal_rotation.tolist(),
        contact_plane_from_grasp_body_m=0.2125,
        active_cup_offsets_yz_m=[[0.0, 0.0]],
        target_center_m=[0.3, 0.0, 0.0],
        target_rotation=IDENTITY,
        target_size_m=[0.6, 0.4, 0.6],
        max_attachment_gap_m=0.002,
        max_normal_misalignment_rad=np.deg2rad(5.0),
    )

    assert not penetrating.accepted
    assert penetrating.reason == "PHYSICAL_CONTACT_PLANE_PENETRATES_TARGET"
    assert not missed.accepted and missed.reason == "ACTIVE_CUP_RAY_MISSES_TARGET"
    assert not wrong_normal.accepted
    assert wrong_normal.reason == "PHYSICAL_CONTACT_NORMAL_MISALIGNED"


def _surface(name: str, center, size):
    return {
        "name": name,
        "center_m": center,
        "size_m": size,
        "rotation_matrix": IDENTITY,
    }


def _support_audit(
    *,
    center=(0.0, 0.0, 0.25),
    rotation=None,
    linear_velocity=(0.0, 0.0, 0.0),
    angular_velocity=(0.0, 0.0, 0.0),
    support_velocity=None,
):
    return audit_payload_support_contact(
        payload_center_m=center,
        payload_rotation=IDENTITY if rotation is None else rotation,
        payload_size_m=[0.6, 0.4, 0.3],
        payload_linear_velocity_m_s=linear_velocity,
        payload_angular_velocity_rad_s=angular_velocity,
        support=_surface("receiver", [0.0, 0.0, 0.05], [1.0, 1.0, 0.1]),
        max_support_gap_m=0.003,
        maximum_penetration_m=0.001,
        minimum_footprint_overlap_ratio=0.90,
        max_support_tilt_rad=np.deg2rad(5.0),
        max_linear_speed_m_s=0.03,
        max_angular_speed_rad_s=0.08,
        support_surface_velocity_world_m_s=support_velocity,
    )


def test_payload_release_support_accepts_only_settled_actual_contact_geometry():
    exact = _support_audit()
    at_gap_limit = _support_audit(center=(0.0, 0.0, 0.253))

    assert exact.accepted
    assert exact.signed_normal_gap_m == pytest.approx(0.0)
    assert exact.footprint_overlap_ratio == pytest.approx(1.0)
    assert exact.support_normal_alignment == pytest.approx(1.0)
    assert at_gap_limit.accepted


def test_payload_release_support_uses_actual_bottom_face_after_reorientation():
    # Local +X is the physical bottom face after this +90-degree Y rotation.
    rotation = np.asarray(
        [
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]
    )
    support = _surface("receiver", [0.0, 0.0, 0.05], [1.0, 1.0, 0.1])
    audit = audit_payload_support_contact(
        payload_center_m=[0.0, 0.0, 0.4],
        payload_rotation=rotation,
        payload_size_m=[0.6, 0.4, 0.3],
        payload_linear_velocity_m_s=[0.0, 0.0, 0.0],
        payload_angular_velocity_rad_s=[0.0, 0.0, 0.0],
        support=support,
        supports=[support],
        max_support_gap_m=0.003,
        maximum_penetration_m=0.001,
        minimum_footprint_overlap_ratio=0.90,
        max_support_tilt_rad=np.deg2rad(5.0),
        max_linear_speed_m_s=0.03,
        max_angular_speed_rad_s=0.08,
    )

    assert audit.accepted
    assert audit.support_normal_alignment == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("support_width_m", "accepted"),
    [(0.60, True), (0.58, False), (0.50, False)],
)
def test_payload_release_support_union_requires_full_footprint_even_with_legacy_ratio(
    support_width_m, accepted
):
    support = _surface(
        "receiver", [0.0, 0.0, 0.05], [support_width_m, 1.0, 0.1]
    )
    audit = audit_payload_support_contact(
        payload_center_m=[0.0, 0.0, 0.25],
        payload_rotation=IDENTITY,
        payload_size_m=[0.6, 0.4, 0.3],
        payload_linear_velocity_m_s=[0.0, 0.0, 0.0],
        payload_angular_velocity_rad_s=[0.0, 0.0, 0.0],
        support=support,
        supports=[support],
        minimum_footprint_overlap_ratio=0.90,
    )

    assert audit.accepted is accepted
    assert audit.footprint_overlap_ratio == pytest.approx(
        support_width_m / 0.6, abs=5e-6
    )
    assert audit.reason == (None if accepted else "PAYLOAD_SUPPORT_FOOTPRINT_INSUFFICIENT")


@pytest.mark.parametrize(
    ("audit", "reason"),
    [
        (_support_audit(center=(0.0, 0.0, 0.2531)), "PAYLOAD_NOT_IN_SUPPORT_CONTACT"),
        (_support_audit(center=(0.0, 0.0, 0.2489)), "PAYLOAD_PENETRATES_SUPPORT"),
        (_support_audit(center=(0.4, 0.0, 0.25)), "PAYLOAD_SUPPORT_FOOTPRINT_INSUFFICIENT"),
        (
            _support_audit(linear_velocity=(0.031, 0.0, 0.0)),
            "PAYLOAD_LINEAR_SPEED_TOO_HIGH_FOR_RELEASE",
        ),
        (
            _support_audit(angular_velocity=(0.0, 0.0, 0.081)),
            "PAYLOAD_ANGULAR_SPEED_TOO_HIGH_FOR_RELEASE",
        ),
    ],
)
def test_payload_release_support_rejects_invalid_actual_state(audit, reason):
    assert not audit.accepted
    assert audit.reason == reason


def test_payload_release_support_rejects_edge_or_side_contact_as_support():
    angle = np.deg2rad(6.0)
    tilted = np.asarray(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), -np.sin(angle)],
            [0.0, np.sin(angle), np.cos(angle)],
        ]
    )
    audit = _support_audit(rotation=tilted)

    assert not audit.accepted
    assert audit.reason == "PAYLOAD_SUPPORT_NORMAL_MISALIGNED"


def test_transfer_ownership_keeps_noncontacting_belts_running_and_overlap_longitudinal():
    surfaces = {
        "cross": _surface("cross", [0.0, 0.0, 0.55], [1.0, 1.0, 0.1]),
        "long": _surface("long", [0.75, 0.0, 0.55], [1.0, 1.0, 0.1]),
    }
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.0, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=False,
        exclusive=True,
    ) == ()
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.0, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
    ) == ("cross", "long")
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.4, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        preferred_overlap_surface="long",
    ) == ("long",)
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.4, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        current_surface="cross",
        preferred_overlap_surface="long",
    ) == ("long",)
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.4, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        preferred_initial_surface="long",
        preferred_overlap_surface="long",
    ) == ("long",)
    assert select_active_conveyor_surfaces(
        payload_center_m=[2.0, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        current_surface="long",
    ) == ("long", "cross")

    # The center is on the cross belt, but the real 0.6 m footprint also
    # overlaps long. The declared longitudinal owner wins at the seam.
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.2, 0.0, 0.8],
        payload_size_m=[0.6, 0.4, 0.3],
        payload_rotation=IDENTITY,
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        preferred_overlap_surface="long",
    ) == ("long",)


def test_moving_support_release_accepts_rest_to_belt_speed_envelope_not_world_rest_only():
    stopped_at_contact = _support_audit(support_velocity=(-0.30, 0.0, 0.0))
    matching = _support_audit(
        linear_velocity=(-0.30, 0.0, 0.0),
        support_velocity=(-0.30, 0.0, 0.0),
    )
    wrong_way = _support_audit(
        linear_velocity=(0.04, 0.0, 0.0),
        support_velocity=(-0.30, 0.0, 0.0),
    )
    assert stopped_at_contact.accepted
    assert matching.accepted
    assert not wrong_way.accepted
    assert wrong_way.reason == "PAYLOAD_VELOCITY_INCOMPATIBLE_WITH_MOVING_SUPPORT"


def test_nonexclusive_legacy_policy_returns_all_declared_surfaces():
    surfaces = {
        "cross": _surface("cross", [0.0, 0.0, 0.55], [1.0, 1.0, 0.1]),
        "long": _surface("long", [0.75, 0.0, 0.55], [1.0, 1.0, 0.1]),
    }
    assert select_active_conveyor_surfaces(
        payload_center_m=None,
        conveyor_primitives=surfaces,
        started=True,
        exclusive=False,
    ) == ("cross", "long")
