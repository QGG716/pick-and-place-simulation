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
    EXPECTED_TOP_CARTONS,
    RESULT_SCHEMA as MOTION_RESULT_SCHEMA,
    build_verified_motion_input,
    load_layout_motion_policy,
    motion_implementation_identity,
)
from unloading_sim.m710_execution import (
    DEFAULT_CONFIG_PATH,
    build_m710_execution_preflight,
    load_m710_execution_config,
    verify_m710_execution_preflight,
)
from unloading_sim.workcell_layout import canonical_digest


ROOT = Path(__file__).resolve().parents[1]


def _compact_motion_result() -> dict:
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    policy = load_layout_motion_policy(execution.motion_policy_path)
    scene = build_verified_motion_input(policy, ROOT)
    result = {
        "schema": MOTION_RESULT_SCHEMA,
        "run_status": "COMPLETED",
        "layout_id": scene.snapshot["layout_id"],
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "policy_fingerprint": policy.policy_fingerprint,
        "implementation_identity": motion_implementation_identity(ROOT),
        "task_population": {"carton_ids": list(scene.removable_cartons)},
        "statistics": {
            "task_count": 5,
            "task_success_count": 0,
            "candidate_pose_attempts": 48,
            "coverage_rejected": 34,
            "ik_calls": 14,
            "complete_trajectory_success_count": 0,
        },
        "complete_trajectory_status": "FAIL_CLOSED",
        "complete_trajectory_failure_reason": "EXECUTION_COLLISION_GEOMETRY_NOT_QUALIFIED",
    }
    result["evidence_fingerprint"] = canonical_digest(result)
    return result


@pytest.fixture(scope="module")
def preflight() -> dict:
    return build_m710_execution_preflight(
        DEFAULT_CONFIG_PATH,
        motion_result=_compact_motion_result(),
        backend_execution_status="NOT_RUN_PER_USER_REQUEST",
    )


def test_preflight_contains_exact_dynamic_cartons_mass_inertia_and_fixed_assembly(preflight):
    primitives = preflight["scene"]["primitives"]
    cartons = [item for item in primitives if item["category"] == "carton"]
    fixed = [
        item
        for item in primitives
        if item["name"] in {"chassis", "conveyor_transverse", "conveyor_longitudinal"}
    ]
    assert len(primitives) == 46  # 3 fixed + 40 cartons + 3 finite boundary patches
    assert preflight["scene"]["carton_count"] == 40
    assert preflight["scene"]["dynamic_carton_count"] == 40
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
    assert preflight["motion"]["task_population"] == [
        "carton_l07_c02",
        "carton_l07_c01",
        "carton_l07_c03",
        "carton_l07_c00",
        "carton_l07_c04",
    ]
    assert sorted(preflight["motion"]["task_population"]) == list(EXPECTED_TOP_CARTONS)


def test_boundary_patches_are_finite_solver_extents_not_trailer_dimension_claims(preflight):
    assert preflight["scene"]["trailer_length_m"] is None
    assert preflight["scene"]["trailer_height_m"] is None
    assert preflight["scene"]["trailer_extent_claim"] == "NOT_DEFINED"
    patches = [item for item in preflight["scene"]["primitives"] if "boundary" in item]
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


def test_conveyor_transport_and_fail_closed_no_isaac_no_video_contract(preflight):
    simulation = preflight["replay_adapter_inputs"]["configuration"]["simulation_validation"]
    conveyor = simulation["conveyor"]
    assert preflight["asset_audit"]["robot"]["manifest_path"] == (
        "assets/robots/fanuc_m710id_70/cad_asset_manifest.yaml"
    )
    assert preflight["replay_adapter_inputs"]["configuration"]["execution"]["limits_source"] == (
        "configs/robots/fanuc_m710id_70.yaml"
    )
    assert str(ROOT) not in json.dumps(preflight, ensure_ascii=False)
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
    assert preflight["replay_adapter_inputs"]["configuration"]["execution"][
        "post_release_settle_seconds"
    ] == pytest.approx(1.0)
    assert preflight["status"] == "BLOCKED"
    assert preflight["simulation_execution_ready"] is False
    assert preflight["execution_qualified"] is False
    assert preflight["machine_qualified"] is False
    assert "PER_LINK_COLLISION_MESHES_MISSING" in preflight["simulation_readiness_blockers"]
    assert "NO_STRICT_GRASP_IK" in preflight["simulation_readiness_blockers"]
    assert "CERTIFIED_JOINT_DRIVE_TORQUE_LIMITS_NOT_SUPPLIED" in (
        preflight["machine_qualification_warnings"]
    )
    assert "CERTIFIED_JOINT_DRIVE_TORQUE_LIMITS_NOT_SUPPLIED" not in preflight["blockers"]
    assert preflight["backend_execution_status"] == "NOT_RUN_PER_USER_REQUEST"
    assert preflight["isaac_validation_performed"] is False
    assert preflight["replay_adapter_inputs"]["trajectory_segment"] is None
    assert preflight["replay_adapter_inputs"]["trajectory_segment_status"] == "NOT_AVAILABLE"
    assert preflight["claims"]["physical_simulation_execution"] == "NOT_RUN_PER_USER_REQUEST"
    assert preflight["claims"]["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"


def test_motion_and_preflight_content_fingerprints_reject_tampering(preflight):
    assert verify_m710_execution_preflight(preflight)["status"] == "PASS"
    changed_preflight = copy.deepcopy(preflight)
    first_carton = next(
        item for item in changed_preflight["scene"]["primitives"] if item["category"] == "carton"
    )
    first_carton["mass_kg"] = 1.0
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
    assert summary["backend_execution_status"] == "NOT_RUN_PER_USER_REQUEST"
    assert summary["isaac_validation_performed"] is False
    assert summary["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"
    assert summary["dynamic_carton_count"] == 40
    assert verify_m710_execution_preflight(result)["status"] == "PASS"
    assert not list(tmp_path.glob("*.mp4"))


def test_real_blocked_preflight_cannot_export_an_m710_bundle(preflight):
    plan = copy.deepcopy(preflight["replay_adapter_inputs"]["plan_common"])
    plan["segments"] = [
        {
            "pick_index": 0,
            "target": preflight["motion"]["task_population"][0],
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
    with pytest.raises(ValueError, match="fail-closed"):
        build_fanuc_isaac_replay_bundle(
            plan,
            copy.deepcopy(preflight["replay_adapter_inputs"]["configuration"]),
            preflight=copy.deepcopy(preflight),
        )
