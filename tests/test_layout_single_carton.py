from __future__ import annotations

from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.fanuc_m710id70 import target_pose
from unloading_sim.geometry import OBB
from unloading_sim.independent_cups import (
    evaluate_independent_cup_geometry,
    m710_cup_array_from_mapping,
    select_ideal_independent_cups,
)
from unloading_sim.layout_single_carton import (
    EXPECTED_TOP_CARTONS,
    audit_execution_collision_geometry,
    build_verified_motion_input,
    load_layout_motion_policy,
    physical_contact_from_virtual_tcp,
    run_layout_single_carton_audit,
    virtual_tcp_from_physical_contact,
)
from unloading_sim.layout_trajectory import LayoutTrajectoryConnectorBuildResult
from unloading_sim.asset_audit import audit_m710id70_official_model
from unloading_sim.support import SupportRelationGraph
from unloading_sim.workcell_layout import audit_initial_state, canonical_digest


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/validation/m710id70_layout_v1_single_carton.yaml"


def _quick_policy():
    policy = load_layout_motion_policy(CONFIG)
    data = dict(policy.data)
    data["ik"] = dict(data["ik"])
    data["ik"].update(random_restarts=0, candidate_limit=1, max_iterations=1)
    return replace(policy, data=data)


def test_production_invalid_home_refuses_snapshot_and_preserves_literal_population():
    policy = load_layout_motion_policy(CONFIG)
    initial = audit_initial_state(policy.layout_validation)
    assert initial["status"] == "FAIL"
    assert any(item["reason"] == "TOOL_SELF_COLLISION" for item in initial["failures"])
    with pytest.raises(ValueError, match="refusing to snapshot invalid initial state"):
        build_verified_motion_input(policy, ROOT)
    layout = policy.layout_validation.layout
    cartons = layout.cartons()
    assert len(cartons) == 40
    assert len(layout.fixed_components()) + len(cartons) == 43
    graph = SupportRelationGraph.build(cartons)
    assert tuple(sorted(graph.removable_cartons(face_modes=("top",)))) == EXPECTED_TOP_CARTONS
    assert {box.name for box in cartons} == {
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
    assert frames.tool0_clocking_status == "OFFICIAL_FLANGE_TO_PROJECT_TOOL0_ADAPTER_RESOLVED"
    assert frames.execution_qualified is True


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
    array = m710_cup_array_from_mapping(policy.data["suction"])
    correct_geometry = evaluate_independent_cup_geometry(
        transformed_contact,
        box,
        "top",
        array,
        max_attachment_gap_m=0.002,
        maximum_penetration_m=0.0002,
    )
    correct = select_ideal_independent_cups(correct_geometry)
    assert len(correct.actual_contact_ids) == 72
    assert correct.to_dict()["load_bearing_minimum_cup_count"] is None

    # The historical misuse put the virtual frame directly on the surface.
    # Recovering its real cup plane exposes a 37.5 mm air gap, so no cup seals.
    old_physical = physical_contact_from_virtual_tcp(
        surface,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    old = evaluate_independent_cup_geometry(
        old_physical,
        box,
        "top",
        array,
        max_attachment_gap_m=0.002,
        maximum_penetration_m=0.0002,
    )
    assert not any(old.geometrically_eligible_mask)
    assert np.linalg.norm(old_physical[:3, 3] - surface[:3, 3]) == pytest.approx(0.0375)

    wrong_sign_virtual = surface.copy()
    wrong_sign_virtual[:3, 3] -= 0.0375 * surface[:3, 2]
    wrong_sign_contact = physical_contact_from_virtual_tcp(
        wrong_sign_virtual,
        frames.flange_from_virtual_task_tcp,
        frames.flange_from_physical_contact,
    )
    wrong = evaluate_independent_cup_geometry(
        wrong_sign_contact,
        box,
        "top",
        array,
        max_attachment_gap_m=0.002,
        maximum_penetration_m=0.0002,
    )
    assert not any(wrong.geometrically_eligible_mask)


def test_execution_collision_geometry_requires_the_official_qualified_assets():
    # Asset readiness and home validity are separate predicates. Valid official
    # assets must never manufacture a valid frozen scene for a colliding home.
    audit = audit_m710id70_official_model(ROOT).to_mapping()
    assert audit["execution_qualified"]
    assert audit["source_integrity"]
    assert audit["static_urdf_integrity"]
    assert audit["model_semantics"]
    with pytest.raises(ValueError, match="refusing to snapshot invalid initial state"):
        build_verified_motion_input(load_layout_motion_policy(CONFIG), ROOT)


def test_unverified_tool_compound_cannot_open_the_execution_geometry_gate():
    policy = load_layout_motion_policy(CONFIG)
    layout = policy.layout_validation.layout
    assert (
        layout.data["tool"]["geometry_status"]
        == "CAD_DERIVED_COMPOUND_OBB_COVERAGE_UNVERIFIED"
    )
    scene = SimpleNamespace(
        policy=policy,
        snapshot={
            "robot": {
                "urdf": layout.assets["robot_urdf"],
                "mounting_reference": {
                    "status": layout.data["robot"]["positioning_status"]
                },
            },
            "tool": {"geometry_status": layout.data["tool"]["geometry_status"]},
        },
    )
    audit = audit_execution_collision_geometry(scene, ROOT)
    assert audit["qualified"] is False
    assert (
        audit["checks"]["tool_rigid_solid_no_false_negative_coverage_proven"]
        is False
    )
    assert audit["failure_reason"] == "EXECUTION_COLLISION_GEOMETRY_NOT_QUALIFIED"


def test_invalid_home_audit_is_deterministic_and_never_searches(
    monkeypatch, tmp_path,
):
    # Keep all five tasks and 40 cartons; none is mislabeled as an IK failure.
    policy = _quick_policy()
    unavailable = LayoutTrajectoryConnectorBuildResult(
        None,
        "UNAVAILABLE",
        "TEST_BACKEND_UNAVAILABLE",
        {"status": "UNAVAILABLE", "failure_reason": "TEST_BACKEND_UNAVAILABLE"},
    )
    monkeypatch.setattr(
        "unloading_sim.layout_single_carton._build_automatic_trajectory_connector",
        lambda scene, robot: unavailable,
    )
    def forbidden(*args, **kwargs):
        pytest.fail("invalid initial state must not enter snapshot, IK, or path search")
    monkeypatch.setattr("unloading_sim.layout_single_carton.build_verified_motion_input", forbidden)
    monkeypatch.setattr("unloading_sim.layout_single_carton.iter_ik_solutions", forbidden)
    first = run_layout_single_carton_audit(policy, project_root=ROOT)
    second = run_layout_single_carton_audit(policy, project_root=ROOT)
    assert first["evidence_fingerprint"] == second["evidence_fingerprint"]
    assert sorted(first["task_population"]["carton_ids"]) == list(EXPECTED_TOP_CARTONS)
    assert first["scene"]["carton_count"] == 40
    candidate_attempts = [
        attempt
        for task in first["tasks"]
        for attempt in task["attempts"]
    ]
    assert candidate_attempts == []
    assert first["run_status"] == "BLOCKED"
    assert first["scene_fingerprint"] is None
    assert first["initial_state_audit"]["status"] == "FAIL"
    assert first["initial_state_exact_diagnostic"]["failure_reason"] == "TEST_BACKEND_UNAVAILABLE"
    assert first["statistics"]["ik_calls"] == 0
    assert first["statistics"]["path_connection_attempts"] == 0
    assert first["statistics"]["tasks_searched"] == 0
    assert first["statistics"]["task_count"] == 5
    assert first["statistics"]["complete_trajectory_success_count"] == 0
    assert first["complete_trajectory_status"] == "FAIL_CLOSED"
    assert first["complete_trajectory_failure_reason"] == "INITIAL_STATE_INVALID"
    assert first["trajectory_backend"]["status"] == "UNAVAILABLE"
    assert first["selected_trajectory_segment"] is None
    assert all(task["failure_reason"] == "INITIAL_STATE_INVALID" for task in first["tasks"])
    # Exercise the real CLI writer and its nonzero process status on blockage.
    spec = importlib.util.spec_from_file_location("layout_audit_cli", ROOT / "tools/run_m710id70_layout_single_carton.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    monkeypatch.setattr(cli, "run_layout_single_carton_audit", lambda *args, **kwargs: first)
    output = tmp_path / "blocked_motion.json"
    assert cli.main(["--output", str(output)]) == 1
    saved = json.loads(output.read_text(encoding="utf-8"))
    recorded = saved.pop("evidence_fingerprint")
    assert canonical_digest(saved) == recorded
    assert saved["initial_state_audit"]["failures"] == first["initial_state_audit"]["failures"]


def test_exact_diagnostic_cannot_override_rejected_production_home(monkeypatch):
    calls = []

    class DiagnosticConnector:
        def validate_unloaded_state(self, q, obstacles, *, stage):
            calls.append((np.asarray(q).tolist(), [box.name for box in obstacles], stage))
            return None

    monkeypatch.setattr(
        "unloading_sim.layout_single_carton._build_automatic_trajectory_connector",
        lambda policy, robot: LayoutTrajectoryConnectorBuildResult(
            DiagnosticConnector(), "AVAILABLE", None, {"status": "AVAILABLE"}
        ),
    )
    def forbidden(*args, **kwargs):
        pytest.fail("diagnostic PASS must not override the production home gate")
    monkeypatch.setattr("unloading_sim.layout_single_carton.build_verified_motion_input", forbidden)
    monkeypatch.setattr("unloading_sim.layout_single_carton.iter_ik_solutions", forbidden)
    result = run_layout_single_carton_audit(_quick_policy(), project_root=ROOT)
    assert result["run_status"] == "BLOCKED"
    assert result["initial_state_exact_diagnostic"]["status"] == "PASS"
    assert result["initial_state_exact_diagnostic"]["can_override_initial_state_gate"] is False
    assert result["statistics"]["initial_state_diagnostic_validations"] == 1
    assert result["statistics"]["trajectory_state_validations"] == 0
    assert result["statistics"]["ik_calls"] == 0
    assert len(calls) == 1
    assert len(calls[0][1]) == 43
    assert calls[0][2] == "initial_state_diagnostic"
