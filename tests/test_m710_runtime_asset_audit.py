from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from unloading_sim.asset_audit import AssetAuditError
from unloading_sim.layout_single_carton import build_verified_motion_input, load_layout_motion_policy
from unloading_sim.m710_dynamics import load_m710id70_dynamics
from unloading_sim.m710_execution import (
    DEFAULT_CONFIG_PATH, _audited_execution_assets, _bridge_configuration,
    audit_m710_replay_assets, load_m710_execution_config,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def real_asset_inputs():
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    scene = build_verified_motion_input(load_layout_motion_policy(execution.motion_policy_path), ROOT)
    dynamics = load_m710id70_dynamics(execution.dynamics_path)
    cpu_assets, _, _ = _audited_execution_assets(execution, scene, dynamics)
    bridge = _bridge_configuration(scene, dynamics, execution)
    vacuum = dynamics.vacuum_attachment
    metadata = {
        "m710_execution_preflight": {"asset_audit": cpu_assets},
        "collision_policy": scene.policy.layout_validation.data["collision_policy"],
        "gripper": {
            "qualified_rigid_collision_boxes_tool_frame": bridge["tool"]["qualified_rigid_collision_boxes_tool_frame"],
            "frame_contract": scene.policy.tool_frames.evidence(),
            "suction_mode": vacuum.suction_mode, "physical_cup_count": vacuum.physical_cup_count,
            "cup_rows": vacuum.cup_rows, "cup_columns": vacuum.cup_columns,
        },
    }
    return cpu_assets, json.loads(json.dumps(metadata))


def test_cpu_and_replay_reaudit_real_official_and_cad_inputs_identically(real_asset_inputs):
    cpu, metadata = real_asset_inputs
    replay = audit_m710_replay_assets(ROOT, metadata)
    assert replay == cpu == json.loads(json.dumps(cpu))
    assert replay["execution_qualified"]
    assert replay["robot"]["upstream_commit"] == "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"
    assert replay["tool"]["execution_geometry_qualification"]["rigid_solid_compound_count"] == 130


def test_runtime_reaudit_does_not_trust_expected_pass_or_box_count(real_asset_inputs):
    cpu, original = real_asset_inputs
    changed = copy.deepcopy(original)
    changed["gripper"]["qualified_rigid_collision_boxes_tool_frame"].pop()
    result = audit_m710_replay_assets(ROOT, changed)
    assert result != cpu
    assert not result["execution_qualified"]
    assert not result["tool"]["execution_geometry_qualification"]["structural_integration_checks_passed"]


def test_runtime_reaudit_rejects_changed_official_source_bytes(real_asset_inputs, monkeypatch):
    _, metadata = real_asset_inputs
    original_read = Path.read_bytes

    def changed_read(path):
        data = original_read(path)
        if path.as_posix().endswith("official/fanuc_m710_description/meshes/m710id_70/collision/base.stl"):
            return data[:-1] + bytes([data[-1] ^ 1])
        return data

    monkeypatch.setattr(Path, "read_bytes", changed_read)
    with pytest.raises(AssetAuditError):
        audit_m710_replay_assets(ROOT, metadata)
