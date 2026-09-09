"""Backend-neutral contract for the M-710 layout initialization replay.

The core package never imports Isaac Sim.  The heavyweight adapter consumes
this content-addressed contract and returns a measured scene dump that can be
audited here on an ordinary CPU-only test runner.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .workcell_layout import canonical_digest, verify_scene_snapshot


ISAAC_LAYOUT_CONTRACT_SCHEMA = "m710id70_isaac_layout_contract_v1"
ISAAC_LAYOUT_DUMP_SCHEMA = "m710id70_isaac_layout_backend_dump_v1"


def _primitive(record: Mapping[str, Any], role: str) -> dict[str, Any]:
    pose = np.asarray(record["pose_world"], dtype=float)
    half = np.asarray(record["half_extents_m"], dtype=float)
    if pose.shape != (4, 4) or half.shape != (3,) or np.any(half <= 0.0):
        raise ValueError(f"invalid frozen primitive: {record.get('name')}")
    return {
        "name": str(record["name"]),
        "category": str(record["category"]),
        "role": role,
        "pose_world": pose.tolist(),
        "size_xyz_m": (2.0 * half).tolist(),
        "dynamic": False,
    }


def build_isaac_layout_contract(
    snapshot: Mapping[str, Any],
    project_root: str | Path,
    *,
    target: str = "carton_l07_c02",
) -> dict[str, Any]:
    """Translate one verified snapshot without changing its geometry."""
    verification = verify_scene_snapshot(snapshot, project_root)
    cartons = {str(item["name"]): item for item in snapshot["cartons"]}
    if target not in cartons:
        raise ValueError(f"selected carton is absent from frozen snapshot: {target}")
    primitives = [
        *(_primitive(item, "fixed_assembly") for item in snapshot["assembly"]["fixed_components"]),
        *(
            _primitive(item, "selected_carton" if item["name"] == target else "supporting_carton")
            for item in snapshot["cartons"]
        ),
        _primitive(snapshot["tool"]["collision_obb"], "tool_equal_scale_collision_proxy"),
    ]
    contract = {
        "schema": ISAAC_LAYOUT_CONTRACT_SCHEMA,
        "scope": "static_layout_initialization_replay_with_one_carton_focus",
        "snapshot_schema": snapshot["schema"],
        "scene_fingerprint": snapshot["scene_fingerprint"],
        "layout_id": snapshot["layout_id"],
        "layout_fingerprint": snapshot["layout_fingerprint"],
        "asset_verification": verification,
        "target": target,
        "world": snapshot["world"],
        "trailer": snapshot["trailer"],
        "primitives": primitives,
        "robot": {
            "model": snapshot["robot"]["model"],
            "joint_names": snapshot["robot"]["joint_names"],
            "q_rad": snapshot["robot"]["q_rad"],
            "world_from_mount": snapshot["robot"]["world_from_mount"],
            "urdf": snapshot["robot"]["urdf"],
        },
        "claims": {
            "geometry_source": "same_frozen_snapshot_as_cpu_audit_and_figures",
            "carton_count": len(snapshot["cartons"]),
            "physical_grasp": "NOT_EVALUATED",
            "payload_dynamics": "NOT_EVALUATED",
            "complete_workcell_clearance": snapshot["initial_state_audit"]["complete_workcell_clearance"],
            "receiver_transport": snapshot["receiver"]["transport_capability"],
        },
    }
    contract["contract_fingerprint"] = canonical_digest(contract)
    return contract


def verify_isaac_layout_contract(contract: Mapping[str, Any]) -> dict[str, Any]:
    value = dict(contract)
    if value.get("schema") != ISAAC_LAYOUT_CONTRACT_SCHEMA:
        raise ValueError("unsupported Isaac layout contract schema")
    recorded = value.pop("contract_fingerprint", None)
    actual = canonical_digest(value)
    if recorded != actual:
        raise ValueError("Isaac layout contract fingerprint mismatch")
    names = [item["name"] for item in value["primitives"]]
    if len(names) != len(set(names)):
        raise ValueError("Isaac layout contract contains duplicate primitive names")
    if value["target"] not in names:
        raise ValueError("Isaac layout target is missing from primitives")
    return {"status": "PASS", "contract_fingerprint": recorded, "primitive_count": len(names)}


def audit_isaac_layout_backend_dump(
    contract: Mapping[str, Any],
    backend_dump: Mapping[str, Any],
    *,
    pose_tolerance: float = 1e-9,
    joint_tolerance: float = 1e-6,
) -> dict[str, Any]:
    """Require the backend dump to reproduce every authored primitive and q."""
    verification = verify_isaac_layout_contract(contract)
    if backend_dump.get("schema") != ISAAC_LAYOUT_DUMP_SCHEMA:
        raise ValueError("unsupported Isaac layout backend dump schema")
    if backend_dump.get("contract_fingerprint") != contract["contract_fingerprint"]:
        raise ValueError("backend dump belongs to a different replay contract")
    expected = {item["name"]: item for item in contract["primitives"]}
    actual = {item["name"]: item for item in backend_dump.get("primitives", [])}
    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    pose_errors: dict[str, float] = {}
    size_errors: dict[str, float] = {}
    for name in sorted(set(expected) & set(actual)):
        pose_errors[name] = float(
            np.max(
                np.abs(
                    np.asarray(actual[name]["pose_world"], dtype=float)
                    - np.asarray(expected[name]["pose_world"], dtype=float)
                )
            )
        )
        size_errors[name] = float(
            np.max(
                np.abs(
                    np.asarray(actual[name]["size_xyz_m"], dtype=float)
                    - np.asarray(expected[name]["size_xyz_m"], dtype=float)
                )
            )
        )
    q_error = float(
        np.max(
            np.abs(
                np.asarray(backend_dump["robot"]["q_rad"], dtype=float)
                - np.asarray(contract["robot"]["q_rad"], dtype=float)
            )
        )
    )
    mount_error = float(
        np.max(
            np.abs(
                np.asarray(backend_dump["robot"]["world_from_mount"], dtype=float)
                - np.asarray(contract["robot"]["world_from_mount"], dtype=float)
            )
        )
    )
    pose_max = max(pose_errors.values(), default=float("inf"))
    size_max = max(size_errors.values(), default=float("inf"))
    passed = (
        not missing
        and not extra
        and pose_max <= pose_tolerance
        and size_max <= pose_tolerance
        and mount_error <= pose_tolerance
        and q_error <= joint_tolerance
    )
    return {
        "schema": "m710id70_isaac_layout_replay_audit_v1",
        "status": "PASS" if passed else "FAIL",
        "contract_verification": verification,
        "missing_primitives": missing,
        "extra_primitives": extra,
        "primitive_pose_max_abs_m": pose_max,
        "primitive_size_max_abs_m": size_max,
        "robot_mount_pose_max_abs_m": mount_error,
        "robot_q_max_abs_rad": q_error,
        "pose_tolerance": pose_tolerance,
        "joint_tolerance": joint_tolerance,
        "scope": contract["scope"],
        "physical_grasp": "NOT_EVALUATED",
        "complete_workcell_clearance": contract["claims"]["complete_workcell_clearance"],
    }


def write_isaac_layout_contract(
    snapshot_path: str | Path,
    project_root: str | Path,
    output_path: str | Path,
    *,
    target: str = "carton_l07_c02",
) -> dict[str, Any]:
    snapshot = json.loads(Path(snapshot_path).read_text(encoding="utf-8"))
    contract = build_isaac_layout_contract(snapshot, project_root, target=target)
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(contract, ensure_ascii=False, indent=2), encoding="utf-8")
    return contract
