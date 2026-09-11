from __future__ import annotations

from dataclasses import replace

import pytest

from unloading_contracts import EvidenceKind, RobotStateRevision
from unloading_perception.isaac_validation import (
    AXIS_CONVENTION,
    HistoricalResultGate,
    IsaacCaptureBinding,
    build_feasibility_handoff,
    build_scene_manifest,
    canonical_digest,
    check_feasibility_handoff,
    ground_truth_observation,
    load_validation_config,
    SimulationClockGuard,
    verify_text_asset_identity,
)
from unloading_perception.scene import SnapshotAssembler, build_scene_update


IDENTITY = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _snapshot_and_contract():
    snapshot = {
        "schema": "m710id70_workcell_snapshot_v1",
        "layout_id": "m710id70_unloading_layout_v1",
        "layout_fingerprint": "a" * 64,
        "units": {"length": "metre", "angle": "radian", "mass": "kilogram"},
        "world": {"origin": "stack", "axes": {"x": "into_trailer", "y": "left", "z": "up"}},
        "assembly": {
            "pose_world": IDENTITY,
            "fixed_components": [{"name": "chassis", "category": "chassis", "pose_world": IDENTITY, "half_extents_m": [1.0, 0.7, 0.3]}],
        },
        "cartons": [{
            "name": "carton_l07_c02", "category": "carton",
            "pose_world": [[1.0, 0.0, 0.0, 0.3], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 2.25], [0.0, 0.0, 0.0, 1.0]],
            "half_extents_m": [0.3, 0.2, 0.15],
        }],
        "robot": {
            "model": "fanuc_m710id_70", "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
            "q_rad": [0.0] * 6, "world_from_mount": IDENTITY,
            "urdf": {"repository_path": "robot.urdf", "sha256": "b" * 64},
            "model_config": {"repository_path": "robot.yaml", "sha256": "c" * 64},
        },
        "tool": {"assets": {}, "name": "tool"},
        "receiver": {"state": "EMPTY", "transport_capability": "NOT_IMPLEMENTED_FOR_LAYOUT_V1"},
        "attachments": [],
    }
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    contract = {
        "schema": "m710id70_isaac_layout_contract_v1",
        "layout_id": snapshot["layout_id"],
        "layout_fingerprint": snapshot["layout_fingerprint"],
        "scene_fingerprint": snapshot["scene_fingerprint"],
        "primitives": [],
    }
    contract["contract_fingerprint"] = canonical_digest(contract)
    return snapshot, contract


def _manifest(**kwargs):
    snapshot, contract = _snapshot_and_contract()
    return build_scene_manifest(
        snapshot, contract, load_validation_config("configs/isaac/perception_validation.yaml"),
        run_id="test-run", perception_commit="perception-sha", feasibility_reference_commit="feasibility-sha",
        isaac_version="6.0.1.0", simulation_epoch="epoch-a", **kwargs,
    ), contract


def _gt(manifest):
    return ground_truth_observation(manifest, [{
        "simulation_object_id": "carton_l07_c02", "bbox_xyxy": [100, 100, 300, 300],
        "prim_path": "/World/Cartons/carton_l07_c02", "visible": True, "occluded": False,
    }])


def test_manifest_separates_layout_from_dynamic_scene_identity():
    initial, _ = _manifest()
    moved_pose = [[1.0, 0.0, 0.0, 0.35], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 2.25], [0.0, 0.0, 0.0, 1.0]]
    moved, _ = _manifest(object_overrides={"carton_l07_c02": {"T_W_object": moved_pose, "state": "KNOWN_TRANSLATION"}})
    assert moved.layout["layout_fingerprint"] == initial.layout["layout_fingerprint"]
    assert moved.dynamic_scene_fingerprint != initial.dynamic_scene_fingerprint
    assert moved.world_fingerprint != initial.world_fingerprint
    assert moved.layout["axis_convention"] == AXIS_CONVENTION


def test_manifest_rejects_tampering():
    manifest, _ = _manifest()
    payload = manifest.to_dict()
    payload["objects"][0]["full_dimensions_m"][0] = 0.7
    with pytest.raises(ValueError, match="dynamic scene fingerprint"):
        type(manifest).from_dict(payload)


def test_ground_truth_is_explicitly_synthetic_and_metric():
    manifest, _ = _manifest(simulation_frame=50, simulation_time=5.0)
    observation = _gt(manifest)
    cargo = observation.cargo[0]
    assert observation.synthetic is True
    assert observation.provider == "isaac-sim-ground-truth"
    assert cargo.pose_evidence is EvidenceKind.SYNTHETIC
    assert cargo.full_dimensions_m == (0.6, 0.4, 0.3)
    assert cargo.candidate_eligible is True
    assert cargo.pose.covariance is None


def test_occluded_ground_truth_fails_closed():
    manifest, _ = _manifest()
    observation = ground_truth_observation(manifest, [{
        "simulation_object_id": "carton_l07_c02", "bbox_xyxy": [100, 100, 300, 300],
        "visible": True, "occluded": True,
    }])
    assert observation.status.value == "PARTIAL"
    assert not observation.cargo[0].candidate_eligible
    assert observation.unknown_regions


def test_late_gpu_result_remains_evaluable_but_cannot_replace_live_world():
    first = IsaacCaptureBinding("epoch-a", 0, 0.0, "1" * 64, "2" * 64, "3" * 64)
    newer = IsaacCaptureBinding("epoch-a", 50, 5.0, "4" * 64, "2" * 64, "5" * 64)
    gate = HistoricalResultGate()
    gate.register_capture(first)
    gate.register_capture(newer)
    result = gate.classify_result(first)
    assert result["evaluation_eligible"] is True
    assert result["live_world_eligible"] is False


def test_feasibility_handoff_contains_existing_planning_world_snapshot():
    manifest, contract = _manifest()
    observation = _gt(manifest)
    assembler = SnapshotAssembler()
    assembler.update = build_scene_update(observation)
    assembler.robot_state = RobotStateRevision(0, (0.0,) * 6, {"joint_names": ("J1", "J2", "J3", "J4", "J5", "J6")})
    assembler.tool_attachment = {"identity": "tool", "confirmed": True}
    assembler.payload_attachment = {"identity": "payload", "object_id": None}
    assembler.base_state = {"identity": "base", "T_W_A": IDENTITY}
    assembler.conveyor_state = {"identity": "conveyor", "running": False}
    assembler.config_identity = {"identity": "config", "robot_model_fingerprint": "robot", "world_model_fingerprint": "world"}
    snapshot = assembler.assemble().snapshot
    assert snapshot is not None
    handoff = build_feasibility_handoff(snapshot, manifest, evidence_mode="ISAAC_GT")
    report = check_feasibility_handoff(handoff, manifest, contract)
    assert report["status"] == "PASS"
    assert handoff["planning_world_snapshot"]["__type__"] == "PlanningWorldSnapshot"


def test_capture_binding_rejects_hash_mismatch_shape():
    with pytest.raises(ValueError, match="RGB hash"):
        IsaacCaptureBinding("epoch-a", 0, 0.0, "short", "2" * 64, "3" * 64)


def test_simulation_clock_pause_resume_and_restart_are_explicit():
    guard = SimulationClockGuard()
    guard.observe("epoch-a", 0, 0.0)
    guard.observe("epoch-a", 1, 0.1)
    guard.observe("epoch-a", 2, 0.1, paused=True)
    guard.observe("epoch-a", 3, 0.2)
    with pytest.raises(ValueError, match="new simulation epoch"):
        guard.observe("epoch-b", 0, 0.0)
    guard.observe("epoch-b", 0, 0.0, restart=True)
    with pytest.raises(ValueError, match="restart must create"):
        guard.observe("epoch-b", 1, 0.1, restart=True)


def test_text_asset_identity_allows_only_git_newline_conversion(tmp_path):
    crlf = b"<robot>\r\n  <link name=\"base\"/>\r\n</robot>\r\n"
    expected = __import__("hashlib").sha256(crlf).hexdigest()
    asset = tmp_path / "robot.urdf"
    asset.write_bytes(crlf.replace(b"\r\n", b"\n"))
    identity = verify_text_asset_identity(asset, expected)
    assert identity["normalization"] == "GIT_TEXT_LF_CHECKOUT_OF_CRLF_CONTRACT"
    asset.write_bytes(asset.read_bytes().replace(b"base", b"changed"))
    with pytest.raises(ValueError, match="beyond newline normalization"):
        verify_text_asset_identity(asset, expected)
