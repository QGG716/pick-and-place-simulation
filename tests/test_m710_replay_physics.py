from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.m710_replay_physics import (
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


def test_exclusive_conveyor_selection_never_selects_two_surfaces():
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
    ) == ("cross",)
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.4, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
    ) == ()
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.4, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        current_surface="cross",
    ) == ("cross",)
    assert select_active_conveyor_surfaces(
        payload_center_m=[0.4, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        preferred_initial_surface="long",
    ) == ("long",)
    assert select_active_conveyor_surfaces(
        payload_center_m=[2.0, 0.0, 0.8],
        conveyor_primitives=surfaces,
        started=True,
        exclusive=True,
        current_surface="long",
    ) == ()


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
