"""An invalid production home must still produce inspectable input evidence."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path

import pytest

from unloading_sim.layout_single_carton import run_layout_single_carton_audit
from unloading_sim.layout_trajectory import LayoutTrajectoryConnectorBuildResult
from unloading_sim.isaac_collision_policy import expand_robot_only_srdf_pairs
from unloading_sim.m710_initialization_diagnostic import (
    SCHEMA as INITIALIZATION_CONTRACT_SCHEMA,
    SCOPE as INITIALIZATION_SCOPE,
    build_initialization_diagnostic_contract,
    verify_initialization_diagnostic_contract,
)
from unloading_sim.m710_execution import (
    INITIALIZATION_DIAGNOSTIC_SCHEMA,
    build_m710_execution_preflight,
    verify_m710_execution_preflight,
)
from unloading_sim.m710_replay_contract import verify_m710_preflight_contract
from unloading_sim.workcell_layout import canonical_digest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def blocked_preflight(monkeypatch):
    monkeypatch.setattr(
        "unloading_sim.layout_single_carton._build_automatic_trajectory_connector",
        lambda *args: LayoutTrajectoryConnectorBuildResult(
            None, "UNAVAILABLE", "TEST_DIAGNOSTIC_ONLY", {"status": "UNAVAILABLE"}
        ),
    )
    def no_snapshot(*args, **kwargs):
        pytest.fail("invalid home must not be converted into a scene snapshot")
    monkeypatch.setattr("unloading_sim.m710_execution.build_verified_motion_input", no_snapshot)
    motion = run_layout_single_carton_audit(
        ROOT / "configs/validation/m710id70_layout_v1_single_carton.yaml"
    )
    assert motion["run_status"] == "BLOCKED"
    return build_m710_execution_preflight(motion_result=motion)


def test_invalid_home_preflight_preserves_model_inputs_without_execution(blocked_preflight):
    result = blocked_preflight
    assert result["schema"] == INITIALIZATION_DIAGNOSTIC_SCHEMA
    assert verify_m710_execution_preflight(result)["status"] == "PASS"
    assert result["status"] == "BLOCKED"
    assert result["simulation_execution_ready"] is False
    assert result["model_initialization"]["model_load_ready"] is True
    assert result["simulation_readiness_blockers"] == [
        "INITIAL_STATE_INVALID",
        "ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED",
        "TOOL_RIGID_COLLISION_COVERAGE_NOT_PROVEN",
    ]
    assert result["asset_audit"]["robot"]["execution_qualified"] is True
    assert result["asset_audit"]["tool"]["source_integrity"] is True
    assert result["dynamics"]["robot_mass_kg"] == pytest.approx(580.347)
    assert result["dynamics"]["tool_mass_kg"] == 20.0
    assert result["dynamics"]["carton_mass_kg_each"] == 42.5
    assert result["dynamics"]["carton_count"] == 40
    assert len(result["scene"]["carton_ids"]) == 40
    assert result["scene"]["dynamic_carton_count"] == 0
    assert result["scene"]["configured_dynamic_carton_count"] == 40
    assert result["scene"]["primitives"] == []
    assert result["replay_adapter_inputs"] is None
    assert result["motion"]["statistics"]["ik_calls"] == 0
    assert result["initial_state_audit"]["status"] == "FAIL"
    assert result["isaac_validation_performed"] is False


def test_initialization_diagnostic_cannot_be_promoted_or_used_as_replay(blocked_preflight):
    with pytest.raises(ValueError):
        verify_m710_preflight_contract(blocked_preflight, require_ready=True)
    forged = copy.deepcopy(blocked_preflight)
    forged["simulation_execution_ready"] = True
    forged.pop("preflight_fingerprint")
    forged["preflight_fingerprint"] = canonical_digest(forged)
    with pytest.raises(ValueError, match="fail-closed"):
        verify_m710_execution_preflight(forged)
    forged = copy.deepcopy(blocked_preflight)
    forged["initial_state_audit"]["failures"] = []
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_m710_execution_preflight(forged)


def test_prepare_cli_writes_initialization_diagnostic(blocked_preflight, monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        "prepare_diagnostic_cli", ROOT / "tools/prepare_m710id70_dynamic_execution.py"
    )
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    monkeypatch.setattr(cli, "build_m710_execution_preflight", lambda *args, **kwargs: blocked_preflight)
    motion_file = tmp_path / "motion.json"
    motion_file.write_text(json.dumps(blocked_preflight["motion"]), encoding="utf-8")
    output = tmp_path / "preflight.json"
    assert cli.main(["--motion-result", str(motion_file), "--output", str(output)]) == 0
    saved = json.loads(output.read_text(encoding="utf-8"))
    assert verify_m710_execution_preflight(saved)["status"] == "PASS"
    assert saved["replay_adapter_inputs"] is None


@pytest.fixture(scope="module")
def isaac_init_smoke_contract():
    return build_initialization_diagnostic_contract(
        ROOT / "configs/validation/m710id70_layout_v1.yaml",
        ROOT / "configs/simulation/m710id70_official_dynamics_v2.yaml",
        ROOT,
    )


def test_isaac_init_smoke_contract_retains_invalid_pose_and_full_inventory(isaac_init_smoke_contract):
    contract = isaac_init_smoke_contract
    verify_initialization_diagnostic_contract(contract)
    assert contract["schema"] == INITIALIZATION_CONTRACT_SCHEMA
    assert contract["scope"] == INITIALIZATION_SCOPE == "INITIALIZATION_ONLY_NOT_PICK_SUCCESS"
    assert contract["motion_execution_permitted"] is False
    assert contract["attachment_permitted"] is False
    assert contract["claims"]["physical_pick_success"] is False
    assert contract["robot_pose_role"] == "diagnostic_pose_not_motion_qualified_home"
    assert contract["initial_state_audit"]["status"] == "FAIL"
    assert contract["initial_state_audit"]["failures"]
    cartons = [item for item in contract["primitives"] if item["role"] == "dynamic_carton"]
    assert {item["name"] for item in cartons} == {
        f"carton_l{layer:02d}_c{column:02d}" for layer in range(8) for column in range(5)
    }
    assert sum(item["role"] == "rigid_tool" for item in contract["primitives"]) == 58
    assert sum(item["role"] == "fixed_assembly" for item in contract["primitives"]) == 3
    assert contract["dynamics"]["tool"]["mass_kg"] == 20.0
    assert contract["dynamics"]["cartons"]["mass_kg_each"] == 42.5
    assert contract["official_model_audit"]["execution_qualified"] is True
    for asset_name in ("robot_srdf", "validation_config", "layout_config", "dynamics_config"):
        assert len(contract["assets"][asset_name]["sha256"]) == 64


def test_isaac_init_smoke_contract_rejects_content_tampering(isaac_init_smoke_contract):
    changed = copy.deepcopy(isaac_init_smoke_contract)
    changed["robot"]["q_rad"][0] += 0.01
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_initialization_diagnostic_contract(changed)


@pytest.mark.parametrize("field", ["motion_execution_permitted", "attachment_permitted"])
def test_isaac_init_smoke_rehash_cannot_authorize_motion(isaac_init_smoke_contract, field):
    changed = copy.deepcopy(isaac_init_smoke_contract)
    changed[field] = True
    changed.pop("contract_fingerprint")
    changed["contract_fingerprint"] = canonical_digest(changed)
    with pytest.raises(ValueError, match="never authorize motion or attachment"):
        verify_initialization_diagnostic_contract(changed)


@pytest.mark.parametrize("role,expected", [("dynamic_carton", "40 cartons"), ("rigid_tool", "58 rigid tool solids")])
def test_isaac_init_smoke_rehash_cannot_remove_scene_bodies(isaac_init_smoke_contract, role, expected):
    changed = copy.deepcopy(isaac_init_smoke_contract)
    index = next(index for index, item in enumerate(changed["primitives"]) if item["role"] == role)
    changed["primitives"].pop(index)
    changed.pop("contract_fingerprint")
    changed["contract_fingerprint"] = canonical_digest(changed)
    with pytest.raises(ValueError, match=expected):
        verify_initialization_diagnostic_contract(changed)


def test_robot_only_srdf_filters_expand_mesh_pairs_not_whole_bodies():
    links = {"J5_link": "/robot/J5", "J6_link": "/robot/J5/J6"}
    colliders = {
        "J5_link": ["/robot/J5/collision/mesh0", "/robot/J5/collision/mesh1"],
        "J6_link": ["/robot/J5/J6/collision/mesh0"],
    }
    assert expand_robot_only_srdf_pairs(links, colliders, [("J5_link", "J6_link")]) == [
        ("/robot/J5/collision/mesh0", "/robot/J5/J6/collision/mesh0"),
        ("/robot/J5/collision/mesh1", "/robot/J5/J6/collision/mesh0"),
    ]


@pytest.mark.parametrize(
    "collider,reason",
    [
        ("/robot/J5/J6", "not whole bodies"),
        ("/robot/J5/collision/mesh0", "not whole bodies"),
        ("/robot/J5/J6/tool/rigid_13", "tool collision shapes"),
        ("/robot/J5/J6/gripper/mesh", "tool collision shapes"),
    ],
)
def test_robot_only_srdf_filters_reject_tool_body_and_outside_link_shapes(collider, reason):
    links = {"J5_link": "/robot/J5", "J6_link": "/robot/J5/J6"}
    colliders = {"J5_link": ["/robot/J5/collision/mesh0"], "J6_link": [collider]}
    with pytest.raises(ValueError, match=reason):
        expand_robot_only_srdf_pairs(links, colliders, [("J5_link", "J6_link")])


def test_robot_only_srdf_filters_reject_missing_official_colliders():
    links = {"J5_link": "/robot/J5", "J6_link": "/robot/J5/J6"}
    with pytest.raises(ValueError, match="no imported robot collision shapes"):
        expand_robot_only_srdf_pairs(
            links,
            {"J5_link": ["/robot/J5/collision/mesh0"], "J6_link": []},
            [("J5_link", "J6_link")],
        )
