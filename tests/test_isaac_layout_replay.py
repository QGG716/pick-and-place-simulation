from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from unloading_sim.isaac_layout_replay import (
    ISAAC_LAYOUT_DUMP_SCHEMA,
    audit_isaac_layout_backend_dump,
    build_isaac_layout_contract,
    usd_safe_prim_segment,
    verify_isaac_layout_contract,
)
from unloading_sim.m710_initialization_diagnostic import (
    SCOPE as INITIALIZATION_SCOPE,
    build_initialization_diagnostic_contract,
    verify_initialization_diagnostic_contract,
)
from unloading_sim.workcell_layout import canonical_digest


ROOT = Path(__file__).resolve().parents[1]
HISTORICAL_CONTRACT = ROOT / "docs/validation/evidence/m710id70_layout_v1/isaac_layout_contract.json"


def _historical_contract():
    # Replay-format regression only: these immutable bytes describe the old
    # model and must not be regenerated as a PASS for today's failed home.
    return json.loads(HISTORICAL_CONTRACT.read_text(encoding="utf-8"))


def _dump(contract):
    return {
        "schema": ISAAC_LAYOUT_DUMP_SCHEMA,
        "contract_fingerprint": contract["contract_fingerprint"],
        "primitives": [
            {
                "name": item["name"],
                "pose_world": copy.deepcopy(item["pose_world"]),
                "size_xyz_m": list(item["size_xyz_m"]),
            }
            for item in contract["primitives"]
        ],
        "robot": {
            "q_rad": list(contract["robot"]["q_rad"]),
            "world_from_mount": copy.deepcopy(contract["robot"]["world_from_mount"]),
        },
    }


def test_archived_contract_keeps_full_stack_and_highlights_one_existing_target():
    contract = _historical_contract()
    assert verify_isaac_layout_contract(contract)["status"] == "PASS"
    roles = [item["role"] for item in contract["primitives"]]
    assert roles.count("fixed_assembly") == 3
    assert roles.count("selected_carton") == 1
    assert roles.count("supporting_carton") == 39
    assert roles.count("tool_equal_scale_collision_proxy") == 1
    assert contract["claims"]["carton_count"] == 40
    assert contract["claims"]["physical_grasp"] == "NOT_EVALUATED"
    assert contract["claims"]["receiver_transport"] == "NOT_IMPLEMENTED_FOR_LAYOUT_V1"


def test_contract_rejects_deleted_target_duplicate_names_and_tampering():
    contract = _historical_contract()
    tampered = copy.deepcopy(contract)
    tampered["primitives"][0]["size_xyz_m"][0] += 0.001
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_isaac_layout_contract(tampered)
    missing = copy.deepcopy(contract)
    missing["target"] = "carton_missing"
    missing.pop("contract_fingerprint")
    missing["contract_fingerprint"] = canonical_digest(missing)
    with pytest.raises(ValueError, match="target is missing"):
        verify_isaac_layout_contract(missing)
    duplicate = copy.deepcopy(contract)
    duplicate["primitives"].append(copy.deepcopy(duplicate["primitives"][0]))
    duplicate.pop("contract_fingerprint")
    duplicate["contract_fingerprint"] = canonical_digest(duplicate)
    with pytest.raises(ValueError, match="duplicate"):
        verify_isaac_layout_contract(duplicate)


def test_historical_backend_dump_audit_detects_geometry_or_joint_drift():
    contract = _historical_contract()
    assert audit_isaac_layout_backend_dump(contract, _dump(contract))["status"] == "PASS"
    moved = _dump(contract)
    moved["primitives"][4]["pose_world"][0][3] += 0.001
    audit = audit_isaac_layout_backend_dump(contract, moved)
    assert audit["status"] == "FAIL"
    assert audit["primitive_pose_max_abs_m"] == pytest.approx(0.001)
    wrong = _dump(contract)
    wrong["robot"]["q_rad"][0] += 1e-3
    audit = audit_isaac_layout_backend_dump(contract, wrong)
    assert audit["status"] == "FAIL"
    assert audit["robot_q_max_abs_rad"] == pytest.approx(1e-3)


def test_current_official_layout_diagnostic_cannot_be_promoted_to_static_replay():
    contract = build_initialization_diagnostic_contract(
        ROOT / "configs/validation/m710id70_layout_v1.yaml",
        ROOT / "configs/simulation/m710id70_official_dynamics_v2.yaml", ROOT,
    )
    verify_initialization_diagnostic_contract(contract)
    assert contract["scope"] == INITIALIZATION_SCOPE
    assert contract["initial_state_audit"]["status"] == "FAIL"
    assert not contract["motion_execution_permitted"]
    assert not contract["attachment_permitted"]
    assert sum(item["role"] == "dynamic_carton" for item in contract["primitives"]) == 40
    assert sum(item["role"] == "rigid_tool" for item in contract["primitives"]) == 58
    with pytest.raises(ValueError, match="snapshot schema"):
        build_isaac_layout_contract(contract, ROOT)
    with pytest.raises(ValueError, match="contract schema"):
        verify_isaac_layout_contract(contract)


def test_usd_prim_segments_never_start_with_numeric_source_index():
    assert usd_safe_prim_segment(0, "chassis") == "p_00_chassis"
    unicode_name = usd_safe_prim_segment(40, "箱体 40")
    assert unicode_name.startswith("p_40_")
    assert all(char.isascii() and (char.isalnum() or char == "_") for char in unicode_name)
    with pytest.raises(ValueError):
        usd_safe_prim_segment(-1, "bad")
