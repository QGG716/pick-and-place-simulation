from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.geometry import OBB, make_transform, rotation_matrix_from_rpy
from unloading_sim.independent_cups import (
    HOLDING_CAPACITY_ASSUMPTION,
    IDEAL_INDEPENDENT_CUPS_MODE,
    M710_CUP_COUNT,
    audit_actual_independent_cup_contacts,
    build_m710_independent_cup_array,
    evaluate_independent_cup_geometry,
    m710_cup_array_from_mapping,
    select_ideal_independent_cups,
    select_ideal_independent_cups_from_actual_fk,
)


def _target() -> OBB:
    return OBB(
        center=np.zeros(3),
        half_extents=np.array([0.30, 0.20, 0.15]),
        rotation=np.eye(3),
        name="carton_l07_c02",
        category="carton",
    )


def _top_contact(*, shift=(0.0, 0.0, 0.0)) -> np.ndarray:
    # Contact-frame +Z is the top face's inward direction (-world Z).
    pose = make_transform(np.diag([1.0, -1.0, -1.0]), [0.0, 0.0, 0.15])
    pose[:3, 3] += np.asarray(shift, dtype=float)
    return pose


def _frame_contract() -> tuple[np.ndarray, np.ndarray]:
    rotation = np.asarray(
        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
    )
    flange_from_virtual = make_transform(rotation, [0.2500, 0.0, 0.0])
    flange_from_contact = make_transform(rotation, [0.2125, 0.0, 0.0])
    return flange_from_virtual, flange_from_contact


def test_m710_72_cup_ids_positions_and_zones_are_stable():
    first = build_m710_independent_cup_array()
    second = build_m710_independent_cup_array()

    assert len(first.cups) == M710_CUP_COUNT == 72
    assert first.cup_ids == second.cup_ids
    assert len(set(first.cup_ids)) == 72
    assert first.cups[0].cup_id == "cup_r00_c00"
    assert first.cups[1].cup_id == "cup_r01_c00"
    assert first.cups[6].cup_id == "cup_r00_c01"
    assert first.cups[-1].cup_id == "cup_r05_c11"
    assert first.cups[0].center_contact_frame_m == pytest.approx(
        (-0.120, -0.264, 0.0)
    )
    assert first.cups[-1].center_contact_frame_m == pytest.approx(
        (0.120, 0.264, 0.0)
    )
    assert [first.cups[index].zone for index in (0, 23, 24, 47, 48, 71)] == [
        0,
        0,
        1,
        1,
        2,
        2,
    ]
    assert [cup.index for cup in first.cups] == list(range(72))
    assert np.allclose(first.cups[0].contact_frame_from_cup[:3, :3], np.eye(3))
    assert first.cups[0].to_dict()["working_surface"] == {
        "frame": "nominal_compressed_physical_contact",
        "cup_local_plane": "z=0",
        "cup_local_inward_normal": [0.0, 0.0, 1.0],
        "shape": "complete_circular_seal_ring",
    }


@pytest.mark.parametrize(
    "face,pose,expected_count",
    [
        (
            "front",
            make_transform(
                np.column_stack(([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])),
                [-0.30, 0.0, 0.0],
            ),
            36,
        ),
        (
            "front",
            make_transform(
                np.column_stack(([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]))
                @ rotation_matrix_from_rpy(0.0, 0.0, 0.5 * np.pi),
                [-0.30, 0.0, 0.0],
            ),
            48,
        ),
        ("top", _top_contact(), 48),
        (
            "left",
            make_transform(
                np.column_stack(([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])),
                [0.0, 0.20, 0.0],
            ),
            36,
        ),
        (
            "right",
            make_transform(
                np.column_stack(([1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])),
                [0.0, -0.20, 0.0],
            ),
            36,
        ),
    ],
)
def test_named_target_faces_use_their_actual_plane_and_inward_normal(
    face, pose, expected_count
):
    geometry = evaluate_independent_cup_geometry(
        pose, _target(), face, build_m710_independent_cup_array()
    )

    assert sum(geometry.geometrically_eligible_mask) == expected_count
    assert {
        contact.target_id
        for contact in geometry.contacts
        if contact.geometrically_eligible
    } == {_target().name}


def test_centered_top_face_selects_48_complete_rings_without_legacy_60_cup_gate():
    geometry = evaluate_independent_cup_geometry(
        _top_contact(), _target(), "top", build_m710_independent_cup_array()
    )
    selection = select_ideal_independent_cups(geometry)
    evidence = selection.to_dict()

    assert sum(geometry.geometrically_eligible_mask) == 48
    assert sum(selection.commanded_active_mask) == 48
    assert selection.actual_contact_mask == selection.commanded_active_mask
    assert selection.commanded_active_indices == tuple(range(12, 60))
    assert selection.actual_contact_indices == tuple(range(12, 60))
    assert selection.actual_contacts_per_zone == (12, 24, 12)
    assert all(type(value) is bool for value in selection.geometrically_eligible_mask)
    assert all(type(value) is bool for value in selection.commanded_active_mask)
    assert all(type(value) is bool for value in selection.actual_contact_mask)
    assert len(selection.geometrically_eligible_mask) == 72
    assert len(selection.commanded_active_mask) == 72
    assert len(selection.actual_contact_mask) == 72
    assert evidence["mask_bit_order_cup_ids"] == list(
        build_m710_independent_cup_array().cup_ids
    )
    assert evidence["suction_mode"] == IDEAL_INDEPENDENT_CUPS_MODE
    assert evidence["holding_capacity_assumption"] == HOLDING_CAPACITY_ASSUMPTION
    assert evidence["load_bearing_minimum_cup_count"] is None
    assert evidence["enforce_vacuum_force_capacity"] is False
    assert evidence["enforce_vacuum_break_force"] is False
    assert evidence["enforce_vacuum_break_torque"] is False


def test_one_eligible_cup_can_be_commanded_and_other_71_cups_remain_in_geometry():
    array = build_m710_independent_cup_array()
    geometry = evaluate_independent_cup_geometry(
        _top_contact(), _target(), "top", array
    )
    cup_id = geometry.geometrically_eligible_ids[0]
    selection = select_ideal_independent_cups(
        geometry, commanded_cup_ids=[cup_id]
    )

    assert sum(selection.commanded_active_mask) == 1
    assert sum(selection.actual_contact_mask) == 1
    assert selection.commanded_active_ids == (cup_id,)
    assert selection.actual_contact_ids == (cup_id,)
    assert len(selection.geometry.cups) == 72
    assert len(selection.geometry.contacts) == 72


def test_no_geometrically_valid_cup_fails_closed_and_no_threshold_can_be_configured():
    geometry = evaluate_independent_cup_geometry(
        _top_contact(shift=(0.0, 0.0, 0.01)),
        _target(),
        "top",
        build_m710_independent_cup_array(),
    )
    assert not any(geometry.geometrically_eligible_mask)
    assert {contact.reason for contact in geometry.contacts} == {
        "CUP_RING_GAP_EXCEEDS_LIMIT"
    }
    with pytest.raises(ValueError, match="non-empty geometrically eligible"):
        select_ideal_independent_cups(geometry)
    with pytest.raises(ValueError, match="does not accept.*threshold"):
        m710_cup_array_from_mapping(
            {
                "cup_rows": 6,
                "cup_columns": 12,
                "cup_pitch_m": [0.048, 0.048],
                "cup_radius_m": 0.0215,
                "minimum_sealed_cups": 1,
            }
        )


def test_center_inside_target_is_not_enough_when_complete_seal_ring_overhangs():
    array = build_m710_independent_cup_array()
    geometry = evaluate_independent_cup_geometry(
        _top_contact(shift=(0.0, 0.011, 0.0)), _target(), "top", array
    )
    cup = next(item for item in array.cups if item.cup_id == "cup_r00_c02")
    contact = geometry.contacts[cup.index]
    center_local = _target().to_local(
        np.asarray(contact.face_contact_center_world_m)
    )

    assert abs(center_local[1]) < _target().half_extents[1]
    assert contact.geometrically_eligible is False
    assert contact.reason == "CUP_SEAL_RING_NOT_FULLY_ON_TARGET_FACE"
    assert (
        abs(center_local[1])
        + contact.projected_ring_half_extents_on_face_m[1]
        > _target().half_extents[1]
    )


def test_full_ring_gap_range_rejects_tilt_even_when_center_and_normal_alone_pass():
    # Four degrees is within the configured five-degree normal tolerance.  A
    # center-only ray would pass at zero gap, but one side of the 43 mm ring is
    # more than 0.2 mm through the target plane.
    tilted = _top_contact()
    tilted[:3, :3] = tilted[:3, :3] @ rotation_matrix_from_rpy(
        np.deg2rad(4.0), 0.0, 0.0
    )
    selected_index = 3 + 5 * 6
    selected_center = np.asarray(
        build_m710_independent_cup_array().cups[selected_index].center_contact_frame_m
    )
    # Keep this cup's centre exactly on the face so only an all-ring check can
    # see the opposite penetration/gap extrema caused by tilt.
    tilted[2, 3] = 0.15 - float((tilted[:3, :3] @ selected_center)[2])
    geometry = evaluate_independent_cup_geometry(
        tilted, _target(), "top", build_m710_independent_cup_array()
    )
    center_cup = geometry.contacts[selected_index]

    assert center_cup.normal_alignment > np.cos(np.deg2rad(5.0))
    assert center_cup.center_signed_gap_m == pytest.approx(0.0, abs=1e-12)
    assert center_cup.ring_signed_gap_range_m[0] < -0.0002
    assert center_cup.geometrically_eligible is False
    assert center_cup.reason == "CUP_RING_PENETRATES_TARGET"


class _RecordingRobot:
    def __init__(self, fk_pose: np.ndarray):
        self.fk_pose = fk_pose
        self.received_q: np.ndarray | None = None

    def fk(self, q: np.ndarray) -> np.ndarray:
        self.received_q = np.asarray(q, dtype=float).copy()
        return self.fk_pose.copy()


def test_actual_fk_pose_not_requested_pose_drives_final_masks_and_contact_evidence():
    array = build_m710_independent_cup_array()
    requested_geometry = evaluate_independent_cup_geometry(
        _top_contact(), _target(), "top", array
    )
    actual_physical = _top_contact(shift=(0.0, 0.060, 0.0))
    flange_from_virtual, flange_from_contact = _frame_contract()
    actual_virtual = (
        actual_physical
        @ np.linalg.inv(flange_from_contact)
        @ flange_from_virtual
    )
    robot = _RecordingRobot(actual_virtual)
    q_actual = np.asarray([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])

    selection = select_ideal_independent_cups_from_actual_fk(
        robot,
        q_actual,
        _target(),
        "top",
        array,
        flange_from_virtual,
        flange_from_contact,
    )

    assert np.array_equal(robot.received_q, q_actual)
    assert np.allclose(
        selection.geometry.physical_contact_pose_world, actual_physical, atol=1e-12
    )
    assert np.allclose(selection.actual_virtual_task_tcp_pose_world, actual_virtual)
    assert selection.actual_q_rad == pytest.approx(q_actual)
    assert selection.pose_source == (
        "actual_fk_virtual_tcp_converted_to_physical_contact"
    )
    assert selection.geometrically_eligible_mask != (
        requested_geometry.geometrically_eligible_mask
    )
    assert sum(selection.geometrically_eligible_mask) == 42
    for cup_id in selection.actual_contact_ids:
        contact = selection.geometry.contacts[array.cup_ids.index(cup_id)]
        assert contact.target_id == _target().name
        assert contact.face_contact_center_world_m is not None


def test_actual_contact_mask_preserves_command_and_reports_lost_contacts():
    array = build_m710_independent_cup_array()
    nominal = evaluate_independent_cup_geometry(
        _top_contact(), _target(), "top", array
    )
    commanded = nominal.geometrically_eligible_ids[:2]
    displaced = evaluate_independent_cup_geometry(
        _top_contact(shift=(0.0, 0.0, 0.01)), _target(), "top", array
    )

    audit = audit_actual_independent_cup_contacts(displaced, commanded)

    assert len(audit.commanded_active_mask) == 72
    assert sum(audit.commanded_active_mask) == 2
    assert audit.commanded_active_ids == commanded
    assert len(audit.actual_contact_mask) == 72
    assert not any(audit.actual_contact_mask)
    assert audit.actual_contact_ids == ()


def test_cup_selection_does_not_waive_or_modify_rigid_head_collision():
    target = _target()
    geometry = evaluate_independent_cup_geometry(
        _top_contact(), target, "top", build_m710_independent_cup_array()
    )
    selection = select_ideal_independent_cups(
        geometry, commanded_cup_ids=[geometry.geometrically_eligible_ids[0]]
    )
    rigid_head = OBB(
        center=np.array([0.0, 0.0, 0.20]),
        half_extents=np.array([0.30, 0.20, 0.05]),
        rotation=np.eye(3),
        name="rigid_tool",
        category="robot",
    )
    neighbor = OBB(
        center=np.array([0.0, 0.0, 0.24]),
        half_extents=np.array([0.10, 0.10, 0.05]),
        rotation=np.eye(3),
        name="neighbor",
        category="carton",
    )

    assert sum(selection.commanded_active_mask) == 1
    assert rigid_head.intersects_obb(neighbor, margin=0.01)
    assert selection.to_dict()["rigid_collision_semantics"].startswith(
        "NOT_EVALUATED_HERE"
    )


@pytest.mark.parametrize(
    "commanded_ids,match",
    [
        (["missing"], "unknown commanded"),
        (["cup_r00_c00", "cup_r00_c00"], "must be unique"),
        (["cup_r00_c00"], "not geometrically eligible"),
        ([], "non-empty geometrically eligible"),
    ],
)
def test_invalid_or_empty_cup_commands_fail_closed(commanded_ids, match):
    geometry = evaluate_independent_cup_geometry(
        _top_contact(), _target(), "top", build_m710_independent_cup_array()
    )
    with pytest.raises(ValueError, match=match):
        select_ideal_independent_cups(
            geometry, commanded_cup_ids=commanded_ids
        )
