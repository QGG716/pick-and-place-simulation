from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.fanuc_m710id70 import target_pose
from unloading_sim.geometry import OBB
from unloading_sim.layout_single_carton import (
    EXECUTION_GATE_REASON,
    EXPECTED_TOP_CARTONS,
    NO_IK_SEARCH_STATUS,
    audit_execution_collision_geometry,
    build_verified_motion_input,
    load_layout_motion_policy,
    physical_contact_from_virtual_tcp,
    run_layout_single_carton_audit,
    virtual_tcp_from_physical_contact,
)
from unloading_sim.validation_physics import suction_coverage


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/validation/m710id70_layout_v1_single_carton.yaml"


def _quick_policy():
    policy = load_layout_motion_policy(CONFIG)
    data = dict(policy.data)
    data["ik"] = dict(data["ik"])
    data["ik"].update(random_restarts=0, candidate_limit=1, max_iterations=1)
    return replace(policy, data=data)


def test_motion_input_is_verified_frozen_layout_and_literal_top_population():
    scene = build_verified_motion_input(load_layout_motion_policy(CONFIG), ROOT)
    assert scene.snapshot_verification["status"] == "PASS"
    assert scene.snapshot_consistency["status"] == "PASS"
    assert len(scene.cartons) == 40
    assert len(scene.all_obstacles) == 43
    assert tuple(sorted(scene.removable_cartons)) == EXPECTED_TOP_CARTONS
    assert all(name.startswith("carton_l07_") for name in scene.removable_cartons)
    assert scene.receiver.name == "conveyor_transverse"
    assert {box.name for box in scene.cartons} == {
        f"carton_l{layer:02d}_c{column:02d}" for layer in range(8) for column in range(5)
    }


def test_tool_frame_contract_retains_literal_physical_dimensions():
    frames = load_layout_motion_policy(CONFIG).tool_frames
    np.testing.assert_allclose(
        frames.flange_from_virtual_task_tcp,
        [[0.0, 0.0, 1.0, 0.2500],
         [0.0, 1.0, 0.0, 0.0],
         [-1.0, 0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 1.0]],
        atol=0.0,
        rtol=0.0,
    )
    assert frames.flange_from_uncompressed_cup_plane[0, 3] == 0.2275
    assert frames.flange_from_physical_contact[0, 3] == 0.2125
    assert frames.virtual_to_physical_contact_offset_m == pytest.approx(0.0375, abs=1e-15)
    assert frames.tool0_clocking_status == "PROVISIONAL_180_DEGREE_DISCREPANCY_UNRESOLVED"


@pytest.mark.parametrize(
    ("face", "roll"),
    [("front", 0), ("top", 90), ("left", 180), ("right", 270)],
)
def test_physical_contact_and_virtual_tcp_round_trip_for_every_face_and_roll(face, roll):
    frames = load_layout_motion_policy(CONFIG).tool_frames
    physical, _ = target_pose(0.0, 0.0, 1.2, [0.6, 0.4, 0.3], face, roll)
    virtual = virtual_tcp_from_physical_contact(
        physical,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    np.testing.assert_allclose(virtual[:3, :3], physical[:3, :3], atol=1e-15, rtol=0.0)
    np.testing.assert_allclose(
        virtual[:3, 3],
        physical[:3, 3] + 0.0375 * physical[:3, 2],
        atol=1e-15,
        rtol=0.0,
    )
    recovered = physical_contact_from_virtual_tcp(
        virtual,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    np.testing.assert_allclose(recovered, physical, atol=1e-15, rtol=0.0)


def test_coverage_is_checked_at_transformed_physical_plane_not_virtual_tcp():
    policy = load_layout_motion_policy(CONFIG)
    frames = policy.tool_frames
    box = OBB([0.3, 0.0, 1.2], [0.3, 0.2, 0.15], np.eye(3), "target", "carton")
    # Roll 90 degrees places the 12-cup dimension along the 0.6 m box depth.
    surface, _ = target_pose(0.0, 0.0, 1.2, [0.6, 0.4, 0.3], "top", 90)
    desired_virtual = virtual_tcp_from_physical_contact(
        surface,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    transformed_contact = physical_contact_from_virtual_tcp(
        desired_virtual,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    correct = suction_coverage(
        transformed_contact, box, "top", policy.data["suction"], 0.0002
    )
    assert correct["geometric_coverage"]
    assert correct["sealed_cups"] == 72

    # The historical misuse put the virtual frame directly on the surface.
    # Recovering its real cup plane exposes a 37.5 mm air gap, so no cup seals.
    old_physical = physical_contact_from_virtual_tcp(
        surface,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    old = suction_coverage(old_physical, box, "top", policy.data["suction"], 0.0002)
    assert not old["geometric_coverage"]
    assert old["sealed_cups"] == 0
    assert np.linalg.norm(old_physical[:3, 3] - surface[:3, 3]) == pytest.approx(0.0375)

    wrong_sign_virtual = surface.copy()
    wrong_sign_virtual[:3, 3] -= 0.0375 * surface[:3, 2]
    wrong_sign_contact = physical_contact_from_virtual_tcp(
        wrong_sign_virtual,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    wrong = suction_coverage(
        wrong_sign_contact, box, "top", policy.data["suction"], 0.0002
    )
    assert not wrong["geometric_coverage"]
    assert wrong["sealed_cups"] == 0


def test_execution_collision_geometry_is_explicitly_fail_closed():
    scene = build_verified_motion_input(load_layout_motion_policy(CONFIG), ROOT)
    audit = audit_execution_collision_geometry(scene, ROOT)
    assert not audit["qualified"]
    assert audit["failure_reason"] == EXECUTION_GATE_REASON
    assert not audit["checks"]["qualification_manifest_declared"]
    assert not audit["checks"]["robot_links_use_cad_collision_meshes"]
    assert audit["proxy_geometry_use"] == "KINEMATIC_AUDIT_ONLY"


def test_quick_full_population_audit_is_deterministic_and_preserves_failure_taxonomy():
    # Keep the five-task denominator and every candidate pose, but use one IK
    # iteration here; the production CLI exercises the configured full budget.
    policy = _quick_policy()
    first = run_layout_single_carton_audit(policy, project_root=ROOT)
    second = run_layout_single_carton_audit(policy, project_root=ROOT)
    scene = build_verified_motion_input(policy, ROOT)
    assert first["evidence_fingerprint"] == second["evidence_fingerprint"]
    assert first["task_population"]["carton_ids"] == list(scene.removable_cartons)
    assert first["scene"]["carton_count"] == 40
    assert first["fixed_cell_contract"] == {
        "receiver": "conveyor_transverse",
        "lift_enabled": False,
        "conveyor_extension_enabled": False,
        "conveyor_z_optimization_enabled": False,
        "base_scan_enabled": False,
    }
    front = [
        attempt
        for task in first["tasks"]
        for attempt in task["attempts"]
        if attempt["face"] == "front"
    ]
    assert front
    assert all(attempt["failure_reason"] == "INSUFFICIENT_SEALED_CUPS" for attempt in front)
    assert max(attempt["coverage"]["sealed_cups"] for attempt in front) == 48
    no_ik = [
        attempt
        for task in first["tasks"]
        for attempt in task["attempts"]
        if attempt["failure_reason"] == "NO_IK"
    ]
    assert no_ik
    assert all(attempt["search_status"] == NO_IK_SEARCH_STATUS for attempt in no_ik)
    assert first["statistics"]["task_count"] == 5
    assert first["statistics"]["complete_trajectory_success_count"] == 0
    assert first["complete_trajectory_status"] == "FAIL_CLOSED"
    assert first["complete_trajectory_failure_reason"] == EXECUTION_GATE_REASON
