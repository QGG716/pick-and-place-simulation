from __future__ import annotations

import copy
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
from unloading_sim.layout_single_carton import (
    EXPECTED_TOP_CARTONS,
    load_layout_motion_policy,
    run_layout_single_carton_audit,
)
from unloading_sim.identity import load_tool_config
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
    """Unverified records for pure adapter tests; never a planning snapshot.

    No validity booleans or fingerprint are fabricated. These records are
    passed only to geometry/configuration formatters, not the execution gate.
    """
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    policy = load_layout_motion_policy(execution.motion_policy_path)
    layout = policy.layout_validation.layout
    dynamics = load_m710id70_dynamics(execution.dynamics_path)
    def record(box):
        return {"name": box.name, "category": box.category,
                "pose_world": box.world_from_local.tolist(), "half_extents_m": box.half_extents.tolist()}
    geometry = load_tool_config((layout.config_path.parent / layout.data["tool"]["geometry_config"]).resolve())
    scene = SimpleNamespace(
        policy=policy, fixed_components=layout.fixed_components(), cartons=layout.cartons(),
        snapshot={
            "assembly": {"fixed_components": [record(box) for box in layout.fixed_components()]},
            "cartons": [record(box) for box in layout.cartons()],
            "robot": {"world_from_mount": layout.robot_base_transform().tolist()},
            "tool": {"geometry": geometry.data["geometry"]},
        },
    )
    return scene, dynamics, execution


@pytest.fixture(scope="module")
def preflight() -> dict:
    return build_m710_execution_preflight(
        DEFAULT_CONFIG_PATH,
        motion_result=_compact_motion_result(),
        backend_execution_status="NOT_RUN_PER_USER_REQUEST",
    )


def test_adapter_preserves_carton_mass_inertia_and_blocked_preflight_has_no_scene(preflight, geometry_adapter_input):
    primitives = _scene_primitives(*geometry_adapter_input)
    cartons = [item for item in primitives if item["category"] == "carton"]
    fixed = [
        item
        for item in primitives
        if item["name"] in {"chassis", "conveyor_transverse", "conveyor_longitudinal"}
    ]
    assert len(primitives) == 46  # 3 fixed + 40 cartons + 3 finite boundary patches
    assert preflight["scene"]["carton_count"] == 40
    assert preflight["scene"]["dynamic_carton_count"] == 0
    assert preflight["scene"]["configured_dynamic_carton_count"] == 40
    assert preflight["scene"]["primitives"] == []
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
    assert preflight["motion"]["task_population"]["carton_ids"] == [
        "carton_l07_c02",
        "carton_l07_c01",
        "carton_l07_c03",
        "carton_l07_c00",
        "carton_l07_c04",
    ]
    assert sorted(preflight["motion"]["task_population"]["carton_ids"]) == list(EXPECTED_TOP_CARTONS)


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


def test_conveyor_transport_and_fail_closed_no_isaac_no_video_contract(preflight, geometry_adapter_input):
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
    assert configuration["execution"][
        "post_release_settle_seconds"
    ] == pytest.approx(1.0)
    assert preflight["status"] == "BLOCKED"
    assert preflight["simulation_execution_ready"] is False
    assert preflight["execution_qualified"] is False
    assert preflight["machine_qualified"] is False
    assert preflight["simulation_readiness_blockers"] == [
        "INITIAL_STATE_INVALID",
        "ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED",
        "TOOL_RIGID_COLLISION_COVERAGE_NOT_PROVEN",
    ]
    assert preflight["model_initialization"]["deferred_tool_collision_qualification"] == {
        "layout_geometry_status": "CAD_DERIVED_COMPOUND_OBB_COVERAGE_UNVERIFIED",
        "required_coverage_status": "CONSERVATIVE_RIGID_SOLID_COVERAGE_PROVEN",
        "robot_tool_mount_contact_scope_qualified": False,
    }
    assert preflight["model_initialization"]["model_load_ready"] is True
    assert preflight["motion"]["statistics"]["ik_calls"] == 0
    assert preflight["backend_execution_status"] == "NOT_RUN_PER_USER_REQUEST"
    assert preflight["isaac_validation_performed"] is False
    assert preflight["replay_adapter_inputs"] is None
    assert preflight["claims"]["physical_simulation_execution"] == "NOT_RUN_PER_USER_REQUEST"
    assert preflight["claims"]["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"


def test_tool_source_integrity_does_not_promote_unproven_obb_coverage(geometry_adapter_input):
    scene, dynamics, execution = geometry_adapter_input
    audit_scene = SimpleNamespace(
        policy=scene.policy,
        snapshot={
            "tool": {
                "rigid_collision_obbs": [{} for _ in range(58)],
                "execution_collision_representation": (
                    "58_CAD_DERIVED_RIGID_SOLID_COMPOUND_OBBS_PLUS_SEPARATE_FLEXIBLE_CUP_CONTACTS"
                ),
            }
        },
    )
    assets, _, _ = _audited_execution_assets(execution, audit_scene, dynamics)
    tool = assets["tool"]
    geometry = tool["execution_geometry_qualification"]
    assert geometry["structural_integration_checks_passed"] is True
    assert geometry["rigid_solid_semantic_classification_verified"] is False
    assert geometry["outward_containment_certificate_present"] is False
    assert geometry["no_false_negative_rigid_solid_coverage_proven"] is False
    assert geometry["robot_tool_mount_contact_scope_verified"] is False
    assert geometry["broad_j6_tool_collision_exception_present"] is True
    assert geometry["planning_collision_acceptance_qualified"] is False
    assert geometry["dynamic_collision_qualified"] is False
    assert tool["source_execution_qualified"] is False
    assert tool["execution_qualified"] is False
    assert assets["execution_qualified"] is False
    assert "DYNAMIC_COLLISION_REPRESENTATION_NOT_QUALIFIED" in tool["unresolved"]
    assert "DYNAMIC_COLLISION_REPRESENTATION_NOT_QUALIFIED" not in tool[
        "resolved_by_execution_integration"
    ]
    mount_reason = "ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED"
    assert mount_reason in tool["unresolved"]
    assert mount_reason in tool["integration_blocking_issues"]
    assert "every tool rigid box" in tool["integration_blocking_issues"][mount_reason]
    assert "TOOL_RIGID_COLLISION_COVERAGE_NOT_PROVEN" in tool[
        "integration_blocking_issues"
    ]


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


def test_prepare_cli_writes_verified_blocked_preflight_without_running_isaac(tmp_path):
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
    assert summary["status"] == result["status"] == "BLOCKED"
    assert summary["backend_execution_status"] == "NOT_RUN"
    assert summary["isaac_validation_performed"] is False
    assert summary["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"
    assert summary["dynamic_carton_count"] == 0
    assert verify_m710_execution_preflight(result)["status"] == "PASS"
    assert not list(tmp_path.glob("*.mp4"))


def test_real_blocked_preflight_cannot_export_an_m710_bundle(preflight):
    assert preflight["replay_adapter_inputs"] is None
    plan = {"robot": {"model": "fanuc_m710id_70"}}
    plan["segments"] = [
        {
            "pick_index": 0,
            "target": preflight["motion"]["task_population"]["carton_ids"][0],
            "path": [
                [0.0] * 6,
                [0.01] * 6,
                [0.0] * 6,
            ],
                "grasp_index": 1,
                "release_index": 2,
                "release_retreat_index": 2,
            "sealed_cup_indices": list(range(60)),
            "sealed_cup_count": 60,
            "sealed_cups_per_zone": [20, 20, 20],
        }
    ]
    with pytest.raises(ValueError, match="schema"):
        build_fanuc_isaac_replay_bundle(
            plan,
            {},
            preflight=copy.deepcopy(preflight),
        )
