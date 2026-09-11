from __future__ import annotations

import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
from unloading_sim.layout_single_carton import (
    build_verified_motion_input,
    load_layout_motion_policy,
    run_layout_single_carton_audit,
)
from unloading_sim.m710_dynamics import load_m710id70_dynamics
from unloading_sim.m710_execution import (
    DEFAULT_CONFIG_PATH,
    build_m710_execution_preflight,
    load_m710_execution_config,
    verify_m710_execution_preflight,
    _bridge_configuration,
    _audited_execution_assets,
    _scene_primitives,
)
from unloading_sim.workcell_layout import canonical_digest


ROOT = Path(__file__).resolve().parents[1]


def _compact_motion_result() -> dict:
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    return run_layout_single_carton_audit(execution.motion_policy_path, project_root=ROOT)


@pytest.fixture(scope="module")
def geometry_adapter_input():
    """Use the current verified snapshot for pure adapter-format tests."""
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    policy = load_layout_motion_policy(execution.motion_policy_path)
    dynamics = load_m710id70_dynamics(execution.dynamics_path)
    scene = build_verified_motion_input(policy, ROOT)
    return scene, dynamics, execution


@pytest.fixture(scope="module")
def preflight() -> dict:
    return build_m710_execution_preflight(
        DEFAULT_CONFIG_PATH,
        motion_result=_compact_motion_result(),
        backend_execution_status="NOT_RUN_PER_USER_REQUEST",
    )


def test_adapter_preserves_carton_mass_inertia_and_ready_preflight_scene(preflight, geometry_adapter_input):
    primitives = _scene_primitives(*geometry_adapter_input)
    cartons = [item for item in primitives if item["category"] == "carton"]
    fixed = [
        item
        for item in primitives
        if item["name"] in {"chassis", "conveyor_transverse", "conveyor_longitudinal"}
    ]
    assert len(primitives) == 46  # 3 fixed + 40 cartons + 3 finite boundary patches
    assert preflight["scene"]["carton_count"] == 40
    assert preflight["scene"]["dynamic_carton_count"] == 40
    assert len(preflight["scene"]["primitives"]) == 46
    assert len(cartons) == 40
    assert {item["name"] for item in cartons} == {
        f"carton_l{layer:02d}_c{column:02d}" for layer in range(8) for column in range(5)
    }
    assert all(item["dynamic"] is True and item["mass_kg"] == 42.5 for item in cartons)
    expected_inertia = np.diag([0.885416666666667, 1.59375, 1.841666666666667])
    for carton in cartons:
        np.testing.assert_allclose(carton["size_m"], [0.6, 0.4, 0.3], atol=1e-12, rtol=0)
        np.testing.assert_allclose(carton["inertia_at_com_kg_m2"], expected_inertia, atol=1e-12, rtol=0)
    assert len(fixed) == 3
    assert {item["name"] for item in fixed} == {
        "chassis",
        "conveyor_transverse",
        "conveyor_longitudinal",
    }
    assert all(item["dynamic"] is False for item in fixed)
    population = preflight["motion"]["task_population"]
    row_selection = preflight["replay_adapter_inputs"]["plan_common"]["row_selection"]
    assert row_selection["status"] == "READY"
    assert row_selection["row_id"] is not None
    assert population == [
        item["carton_name"] for item in row_selection["candidates"]
    ]


def test_boundary_patches_are_finite_solver_extents_not_trailer_dimension_claims(geometry_adapter_input):
    scene, _, _ = geometry_adapter_input
    assert scene.policy.layout_validation.layout.data["trailer"]["length_m"] is None
    assert scene.policy.layout_validation.layout.data["trailer"]["height_m"] is None
    patches = [item for item in _scene_primitives(*geometry_adapter_input) if "boundary" in item]
    assert {item["name"] for item in patches} == {
        "physical_floor_reachable_patch",
        "physical_right_sidewall_reachable_patch",
        "physical_left_sidewall_reachable_patch",
    }
    assert len(patches) == 3
    for patch in patches:
        assert patch["dynamic"] is False
        assert np.all(np.isfinite(patch["center_m"]))
        assert np.all(np.isfinite(patch["size_m"]))
        assert np.all(np.asarray(patch["size_m"]) > 0.0)
        assert patch["boundary"]["extent_status"] == (
            "FINITE_SOLVER_PATCH_COVERS_ROBOT_REACHABLE_ENVELOPE_NOT_A_TRAILER_DIMENSION"
        )
    by_name = {item["name"]: item for item in patches}
    assert by_name["physical_floor_reachable_patch"]["boundary"] == {
        "axis": "z",
        "value_m": 0.0,
        "inside": "+",
        "extent_status": "FINITE_SOLVER_PATCH_COVERS_ROBOT_REACHABLE_ENVELOPE_NOT_A_TRAILER_DIMENSION",
    }
    assert by_name["physical_right_sidewall_reachable_patch"]["boundary"]["value_m"] == -1.15
    assert by_name["physical_left_sidewall_reachable_patch"]["boundary"]["value_m"] == 1.15


def test_conveyor_transport_and_ready_but_not_yet_executed_contract(preflight, geometry_adapter_input):
    configuration = _bridge_configuration(*geometry_adapter_input)
    simulation = configuration["simulation_validation"]
    conveyor = simulation["conveyor"]
    assert preflight["asset_audit"]["robot"]["manifest_path"] == (
        "assets/robots/fanuc_m710id_70/official/provenance.yaml"
    )
    assert configuration["execution"]["limits_source"] == (
        "configs/robots/fanuc_m710id_70.yaml"
    )
    # Raw backend diagnostics may include local paths; content identities use
    # repository-relative sources and the separately archived summary is portable.
    assert str(ROOT) not in json.dumps(preflight["input_identity"], ensure_ascii=False)
    assert simulation["vacuum_cup_count"] == 72
    assert conveyor["surface_directions_world"] == {
        "conveyor_transverse": [0.0, -1.0, 0.0],
        "conveyor_longitudinal": [-1.0, 0.0, 0.0],
    }
    assert conveyor["speed_m_s"] == 0.30
    assert conveyor["start_policy"] == "after_release_retreat"
    assert conveyor["exclusive_surface_drive_at_transfer"] is True
    assert simulation["rendering"]["required_output"] == {
        "width_px": 1920,
        "height_px": 1080,
        "fps": 30,
        "camera_mode": "fixed_overview_with_contact_and_place_keyframes",
    }
    assert simulation["rendering"]["material_palette"] == {
        "chassis_rgb": [0.10, 0.12, 0.16],
        "conveyor_rgb": [0.035, 0.22, 0.62],
        "conveyor_motion_marker_rgb": [1.0, 0.58, 0.03],
    }
    assert simulation["rendering"]["conveyor_visual_motion"] == {
        "model": "collision_free_wrapped_surface_markers_v1",
        "markers_have_collision": False,
        "markers_follow_active_physx_surface_velocity": True,
    }
    assert configuration["execution"][
        "post_release_settle_seconds"
    ] == pytest.approx(1.0)
    assert simulation["physics"]["contact_offset_m"] == pytest.approx(0.010)
    assert simulation["physics"]["rest_offset_m"] == pytest.approx(0.0)
    assert configuration["execution"]["joint_velocity_feedforward_enabled"] is True
    assert configuration["execution"]["attached_payload_gravity_feedforward_enabled"] is True
    assert simulation["actual_state_gates"]["maximum_free_transit_wait_s"] == pytest.approx(1.0)
    assert simulation["actual_state_gates"]["maximum_release_clearance_wait_s"] == pytest.approx(1.0)
    assert simulation["camera"]["eye_m"] == pytest.approx([-3.8, 0.0, 2.7])
    assert simulation["camera"]["target_m"] == pytest.approx([-0.2, 0.0, 1.25])
    assert preflight["status"] == "READY"
    assert preflight["simulation_execution_ready"] is True
    assert preflight["execution_qualified"] is True
    assert preflight["machine_qualified"] is False
    assert preflight["simulation_readiness_blockers"] == []
    assert preflight["motion"]["statistics"]["ik_calls"] > 0
    assert preflight["backend_execution_status"] == "NOT_RUN_PER_USER_REQUEST"
    assert preflight["isaac_validation_performed"] is False
    assert preflight["replay_adapter_inputs"] is not None
    assert preflight["claims"]["physical_simulation_execution"] == "NOT_RUN_PER_USER_REQUEST"
    assert preflight["claims"]["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"


def test_tool_source_and_coverage_certificate_promote_only_the_scoped_execution_geometry(geometry_adapter_input):
    scene, dynamics, execution = geometry_adapter_input
    assets, _, _ = _audited_execution_assets(execution, scene, dynamics)
    tool = assets["tool"]
    geometry = tool["execution_geometry_qualification"]
    assert geometry["structural_integration_checks_passed"] is True
    assert geometry["rigid_solid_compound_count"] == 130
    assert geometry["rigid_solid_semantic_classification_verified"] is True
    assert geometry["outward_containment_certificate_present"] is True
    assert geometry["no_false_negative_rigid_solid_coverage_proven"] is True
    assert geometry["robot_tool_mount_contact_scope_verified"] is True
    assert geometry["planning_collision_acceptance_qualified"] is True
    assert geometry["dynamic_collision_qualified"] is True
    assert tool["source_execution_qualified"] is True
    assert tool["execution_qualified"] is True
    assert assets["execution_qualified"] is True
    assert tool["integration_blocking_issues"] == {}


def test_motion_and_preflight_content_fingerprints_reject_tampering(preflight):
    assert verify_m710_execution_preflight(preflight)["status"] == "PASS"
    changed_preflight = copy.deepcopy(preflight)
    changed_preflight["dynamics"]["carton_mass_kg_each"] = 1.0
    with pytest.raises(ValueError, match="preflight fingerprint mismatch"):
        verify_m710_execution_preflight(changed_preflight)

    changed_motion = _compact_motion_result()
    changed_motion["run_status"] = "TAMPERED"
    with pytest.raises(ValueError, match="motion result evidence fingerprint mismatch"):
        build_m710_execution_preflight(
            DEFAULT_CONFIG_PATH,
            motion_result=changed_motion,
            backend_execution_status="NOT_RUN_PER_USER_REQUEST",
        )

    stale_motion = _compact_motion_result()
    stale_motion["implementation_identity"]["source_sha256"][
        "src/unloading_sim/layout_single_carton.py"
    ] = "0" * 64
    stale_motion.pop("evidence_fingerprint")
    stale_motion["evidence_fingerprint"] = canonical_digest(stale_motion)
    with pytest.raises(ValueError, match="different implementation or runtime"):
        build_m710_execution_preflight(
            DEFAULT_CONFIG_PATH,
            motion_result=stale_motion,
            backend_execution_status="NOT_RUN_PER_USER_REQUEST",
        )


def test_prepare_cli_writes_verified_ready_preflight_without_claiming_isaac(tmp_path):
    motion_path = tmp_path / "motion.json"
    output_path = tmp_path / "preflight.json"
    motion_path.write_text(json.dumps(_compact_motion_result(), indent=2), encoding="utf-8")
    completed = subprocess.run(
        [
            sys.executable,
            "tools/prepare_m710id70_dynamic_execution.py",
            "--config",
            str(DEFAULT_CONFIG_PATH),
            "--motion-result",
            str(motion_path),
            "--output",
            str(output_path),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    summary = json.loads(completed.stdout)
    result = json.loads(output_path.read_text(encoding="utf-8"))
    assert summary["status"] == result["status"] == "READY"
    assert summary["backend_execution_status"] == "NOT_RUN"
    assert summary["isaac_validation_performed"] is False
    assert summary["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"
    assert summary["dynamic_carton_count"] == 40
    assert verify_m710_execution_preflight(result)["status"] == "PASS"
    assert not list(tmp_path.glob("*.mp4"))


def test_ready_preflight_exports_and_tampering_still_fails_closed(preflight):
    inputs = preflight["replay_adapter_inputs"]
    plan = copy.deepcopy(inputs["plan_common"])
    plan["segments"] = [copy.deepcopy(inputs["trajectory_segment"])]
    bundle = build_fanuc_isaac_replay_bundle(
        plan, inputs["configuration"], preflight=copy.deepcopy(preflight)
    )
    assert bundle.metadata["simulation_execution_ready"] is True
    changed = copy.deepcopy(preflight)
    changed["simulation_execution_ready"] = False
    with pytest.raises(ValueError, match="preflight fingerprint mismatch"):
        build_fanuc_isaac_replay_bundle(
            plan, inputs["configuration"], preflight=changed
        )
