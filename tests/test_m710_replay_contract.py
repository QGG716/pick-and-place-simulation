from __future__ import annotations

import ast
import copy
from pathlib import Path

import pytest

from unloading_sim.m710_replay_contract import (
    INTEGRITY_SCOPE,
    M710ReplayContractError,
    add_bundle_payload_sha256,
    build_m710_replay_contract,
    build_replay_input_binding,
    canonical_sha256,
    sha256_file,
    validate_trajectory_segment,
    verify_bundle_payload_sha256,
    verify_m710_preflight_contract,
    verify_m710_replay_bundle,
    verify_m710_replay_contract,
    verify_workspace_preflight_identity,
)


ROOT = Path(__file__).resolve().parents[1]


def _segment() -> dict:
    return {
        "target": "carton_07",
        "path": [[0.0] * 6, [0.1] * 6, [0.2] * 6, [0.0] * 6],
        "grasp_index": 1,
        "release_index": 2,
        "release_retreat_index": 3,
        "sealed_cup_indices": list(range(60)),
        "sealed_cup_count": 60,
        "sealed_cups_per_zone": [20, 20, 20],
    }


def _ready_preflight() -> dict:
    primitives = [
        {
            "name": f"carton_{index:02d}",
            "category": "carton",
            "dynamic": True,
            "center_m": [float(index), 0.0, 0.0],
            "size_m": [0.6, 0.4, 0.3],
        }
        for index in range(40)
    ]
    identity = {
        "execution_config_fingerprint": "1" * 64,
        "layout_fingerprint": "2" * 64,
        "scene_fingerprint": "3" * 64,
        "motion_evidence_fingerprint": "4" * 64,
        "execution_implementation_source_sha256": {"source.py": "5" * 64},
        "dynamics_fingerprint": "6" * 64,
        "robot_manifest_sha256": "7" * 64,
        "tool_manifest_sha256": "8" * 64,
        "robot_missing_mesh_outputs": [],
    }
    execution_fingerprint = canonical_sha256(identity)
    plan = {
        "robot": {"model": "fanuc_m710id_70", "urdf_path": "robot.urdf"},
        "scene_primitives": primitives,
        "layout_fingerprint": identity["layout_fingerprint"],
        "scene_fingerprint": identity["scene_fingerprint"],
        "motion_evidence_fingerprint": identity["motion_evidence_fingerprint"],
        "execution_asset_fingerprint_sha256": execution_fingerprint,
        "simulation_execution_ready": True,
        "execution_qualified": False,
        "execution_blockers": [],
        "machine_qualified": False,
        "machine_qualification_warnings": ["ENGINEERING_INPUTS_NOT_MACHINE_QUALIFIED"],
        "collision_geometry": "real_cad_per_link_visual_and_compound_collision_required",
    }
    cfg = {"execution": {"joint_effort_limits_nm": [1.0] * 6}}
    segment = _segment()
    binding = build_replay_input_binding(
        plan_common=plan,
        configuration=cfg,
        scene_primitives=primitives,
        trajectory_segment=segment,
        trajectory_segment_status="VERIFIED",
        input_identity=identity,
        execution_asset_fingerprint_sha256=execution_fingerprint,
    )
    preflight = {
        "schema": "m710id70_dynamic_execution_preflight_v1",
        "status": "READY",
        "simulation_execution_ready": True,
        "execution_qualified": False,
        "simulation_readiness_blockers": [],
        "blockers": [],
        "machine_qualified": False,
        "machine_qualification_warnings": ["ENGINEERING_INPUTS_NOT_MACHINE_QUALIFIED"],
        "input_identity": identity,
        "execution_asset_fingerprint_sha256": execution_fingerprint,
        "asset_audit": {
            "robot": {
                "manifest_path": "robot_manifest.yaml",
                "manifest_sha256": identity["robot_manifest_sha256"],
                "source_integrity": True,
                "execution_qualified": True,
            },
            "tool": {
                "manifest_path": "tool_manifest.yaml",
                "manifest_sha256": identity["tool_manifest_sha256"],
                "source_integrity": True,
                "execution_qualified": True,
            },
        },
        "motion": {"complete_trajectory_status": "PASS"},
        "scene": {"primitives": primitives},
        "replay_adapter_inputs": {
            "plan_common": plan,
            "configuration": cfg,
            "trajectory_segment": segment,
            "trajectory_segment_status": "VERIFIED",
            "input_binding": binding,
        },
    }
    preflight["preflight_fingerprint"] = canonical_sha256(preflight)
    return preflight


def _bundle(preflight: dict) -> dict:
    adapter = preflight["replay_adapter_inputs"]
    plan = copy.deepcopy(adapter["plan_common"])
    segment = copy.deepcopy(adapter["trajectory_segment"])
    plan["segments"] = [segment]
    contract = build_m710_replay_contract(
        preflight, plan, copy.deepcopy(adapter["configuration"]), segment
    )
    payload = {
        "format": "isaacsim_fanuc_replay_v1",
        "timestamps_seconds": [0.0, 1.0],
        "positions_rad": [[0.0] * 6, [0.1] * 6],
        "metadata": {
            "robot_model": "fanuc_m710id_70",
            "execution_asset_fingerprint_sha256": preflight[
                "execution_asset_fingerprint_sha256"
            ],
            "simulation_execution_ready": True,
            "execution_blockers": [],
            "scene_primitives": copy.deepcopy(preflight["scene"]["primitives"]),
            "m710_execution_preflight": copy.deepcopy(preflight),
            "m710_replay_contract": contract,
        },
    }
    return add_bundle_payload_sha256(payload)


def _refingerprint(preflight: dict) -> None:
    preflight.pop("preflight_fingerprint", None)
    preflight["preflight_fingerprint"] = canonical_sha256(preflight)


def test_contract_module_is_standard_library_only():
    source = (ROOT / "src/unloading_sim/m710_replay_contract.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = {
        alias.name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    } | {
        node.module.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    assert imports <= {
        "__future__",
        "copy",
        "hashlib",
        "json",
        "math",
        "pathlib",
        "typing",
    }


def test_ready_preflight_contract_and_bundle_round_trip():
    preflight = _ready_preflight()
    verification = verify_m710_preflight_contract(preflight, require_ready=True)
    bundle = _bundle(preflight)

    assert verification["status"] == "PASS"
    assert verify_bundle_payload_sha256(bundle) == bundle["bundle_payload_sha256"]
    assert verify_m710_replay_contract(
        bundle["metadata"]["m710_replay_contract"], preflight
    )["status"] == "PASS"
    result = verify_m710_replay_bundle(bundle)
    assert result["status"] == "PASS"
    assert result["integrity_scope"] == INTEGRITY_SCOPE


@pytest.mark.parametrize(
    "mutation",
    [
        lambda p: p["replay_adapter_inputs"]["plan_common"].update(
            scene_fingerprint="9" * 64
        ),
        lambda p: p["replay_adapter_inputs"]["configuration"]["execution"].update(
            joint_effort_limits_nm=[2.0] * 6
        ),
        lambda p: p["scene"]["primitives"][0].update(center_m=[99.0, 0.0, 0.0]),
        lambda p: p["replay_adapter_inputs"]["trajectory_segment"]["path"][1].__setitem__(
            0, 0.3
        ),
    ],
)
def test_preflight_rejects_plan_config_scene_or_trajectory_tampering_even_if_refingerprinted(
    mutation,
):
    preflight = _ready_preflight()
    mutation(preflight)
    _refingerprint(preflight)
    with pytest.raises(M710ReplayContractError):
        verify_m710_preflight_contract(preflight)


def test_random_hash_and_self_reported_ready_cannot_forge_preflight():
    preflight = _ready_preflight()
    preflight["execution_asset_fingerprint_sha256"] = "a" * 64
    preflight["replay_adapter_inputs"]["plan_common"][
        "execution_asset_fingerprint_sha256"
    ] = "a" * 64
    _refingerprint(preflight)
    with pytest.raises(M710ReplayContractError, match="does not bind input_identity"):
        verify_m710_preflight_contract(preflight, require_ready=True)


def test_blocked_preflight_has_no_exportable_trajectory():
    preflight = _ready_preflight()
    preflight.update(
        status="BLOCKED",
        simulation_execution_ready=False,
        execution_qualified=False,
        simulation_readiness_blockers=["PER_LINK_COLLISION_MESHES_MISSING"],
        blockers=["PER_LINK_COLLISION_MESHES_MISSING"],
    )
    plan = preflight["replay_adapter_inputs"]["plan_common"]
    plan.update(
        simulation_execution_ready=False,
        execution_qualified=False,
        execution_blockers=["PER_LINK_COLLISION_MESHES_MISSING"],
    )
    adapter = preflight["replay_adapter_inputs"]
    adapter["trajectory_segment"] = None
    adapter["trajectory_segment_status"] = "NOT_AVAILABLE"
    adapter["input_binding"] = build_replay_input_binding(
        plan_common=plan,
        configuration=adapter["configuration"],
        scene_primitives=preflight["scene"]["primitives"],
        trajectory_segment=None,
        trajectory_segment_status="NOT_AVAILABLE",
        input_identity=preflight["input_identity"],
        execution_asset_fingerprint_sha256=preflight[
            "execution_asset_fingerprint_sha256"
        ],
    )
    _refingerprint(preflight)

    assert verify_m710_preflight_contract(preflight)["simulation_execution_ready"] is False
    with pytest.raises(M710ReplayContractError, match="fail-closed"):
        verify_m710_preflight_contract(preflight, require_ready=True)


def test_bundle_payload_hash_detects_waypoint_and_metadata_tampering():
    bundle = _bundle(_ready_preflight())
    waypoint_changed = copy.deepcopy(bundle)
    waypoint_changed["positions_rad"][1][0] = 0.2
    with pytest.raises(M710ReplayContractError, match="payload fingerprint"):
        verify_m710_replay_bundle(waypoint_changed)

    metadata_changed = copy.deepcopy(bundle)
    metadata_changed["metadata"]["scene_primitives"][0]["center_m"][0] = -1.0
    with pytest.raises(M710ReplayContractError, match="payload fingerprint"):
        verify_m710_replay_bundle(metadata_changed)


def test_workspace_verifier_checks_sources_manifests_and_optional_asset_audit(tmp_path):
    preflight = _ready_preflight()
    source = tmp_path / "source.py"
    robot_manifest = tmp_path / "robot_manifest.yaml"
    tool_manifest = tmp_path / "tool_manifest.yaml"
    source.write_text("pass\n", encoding="utf-8")
    robot_manifest.write_text("robot: real\n", encoding="utf-8")
    tool_manifest.write_text("tool: real\n", encoding="utf-8")
    identity = preflight["input_identity"]
    identity["execution_implementation_source_sha256"] = {
        "source.py": sha256_file(source)
    }
    identity["robot_manifest_sha256"] = sha256_file(robot_manifest)
    identity["tool_manifest_sha256"] = sha256_file(tool_manifest)
    preflight["asset_audit"]["robot"]["manifest_sha256"] = sha256_file(robot_manifest)
    preflight["asset_audit"]["tool"]["manifest_sha256"] = sha256_file(tool_manifest)
    execution_fingerprint = canonical_sha256(identity)
    preflight["execution_asset_fingerprint_sha256"] = execution_fingerprint
    plan = preflight["replay_adapter_inputs"]["plan_common"]
    plan["execution_asset_fingerprint_sha256"] = execution_fingerprint
    adapter = preflight["replay_adapter_inputs"]
    adapter["input_binding"] = build_replay_input_binding(
        plan_common=plan,
        configuration=adapter["configuration"],
        scene_primitives=preflight["scene"]["primitives"],
        trajectory_segment=adapter["trajectory_segment"],
        trajectory_segment_status="VERIFIED",
        input_identity=identity,
        execution_asset_fingerprint_sha256=execution_fingerprint,
    )
    _refingerprint(preflight)

    result = verify_workspace_preflight_identity(
        preflight, tmp_path, current_asset_audit=preflight["asset_audit"]
    )
    assert result["status"] == "PASS"
    source.write_text("changed\n", encoding="utf-8")
    with pytest.raises(M710ReplayContractError, match="source mismatch"):
        verify_workspace_preflight_identity(preflight, tmp_path)


@pytest.mark.parametrize(
    "indices",
    [
        (2, 1, 3),
        (1, 3, 2),
    ],
)
def test_trajectory_contract_rejects_out_of_order_events(indices):
    segment = _segment()
    segment["grasp_index"], segment["release_index"], segment["release_retreat_index"] = indices
    with pytest.raises(M710ReplayContractError, match="event order"):
        validate_trajectory_segment(segment)
