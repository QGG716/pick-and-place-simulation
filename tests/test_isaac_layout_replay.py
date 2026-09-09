from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from unloading_sim.isaac_layout_replay import (
    ISAAC_LAYOUT_DUMP_SCHEMA,
    audit_isaac_layout_backend_dump,
    build_isaac_layout_contract,
    verify_isaac_layout_contract,
)


ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT = ROOT / "outputs/m710id70_layout_v1/scene_snapshot.json"


def _snapshot():
    if SNAPSHOT.is_file():
        return json.loads(SNAPSHOT.read_text(encoding="utf-8"))
    from unloading_sim.workcell_layout import build_scene_snapshot, load_layout_validation_config

    config = load_layout_validation_config(ROOT / "configs/validation/m710id70_layout_v1.yaml")
    return build_scene_snapshot(config)


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


def test_contract_keeps_full_stack_and_highlights_one_existing_target():
    contract = build_isaac_layout_contract(_snapshot(), ROOT)
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
    snapshot = _snapshot()
    with pytest.raises(ValueError, match="absent"):
        build_isaac_layout_contract(snapshot, ROOT, target="carton_missing")
    contract = build_isaac_layout_contract(snapshot, ROOT)
    tampered = copy.deepcopy(contract)
    tampered["primitives"][0]["size_xyz_m"][0] += 0.001
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_isaac_layout_contract(tampered)


def test_backend_dump_audit_detects_geometry_or_joint_drift():
    contract = build_isaac_layout_contract(_snapshot(), ROOT)
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
