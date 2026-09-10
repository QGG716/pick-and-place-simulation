"""Content-addressed initialization diagnostics, never executable pick plans.

An explicitly failed clearance audit may be visualized here. It is preserved
as evidence and cannot become a qualified home or a motion/replay bundle.
"""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .asset_audit import audit_m710id70_official_model
from .m710_official_dynamics import EXPECTED_EFFORT, EXPECTED_VELOCITY, load_m710id70_official_dynamics
from .workcell_layout import audit_initial_state, canonical_digest, load_layout_validation_config


SCHEMA = "m710id70_initialization_diagnostic_contract_v1"
SCOPE = "INITIALIZATION_ONLY_NOT_PICK_SUCCESS"


def generated_usd_tree_identity(
    entrypoint: str | Path, usd_directory: str | Path,
) -> dict[str, Any]:
    """Return a deterministic content identity for one generated USD package.

    The importer may create sibling packages in ``usd_directory`` on repeated
    runs.  Only the tree containing the entrypoint used by this run belongs in
    its identity.
    """
    root = Path(usd_directory).resolve()
    entry = Path(entrypoint).resolve()
    try:
        entry_relative = entry.relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError("generated USD entrypoint is outside --usd-directory") from exc
    if not entry.is_file():
        raise ValueError("generated USD entrypoint does not exist")
    tree_root = entry.parent
    files = []
    for path in sorted((item for item in tree_root.rglob("*") if item.is_file()),
                       key=lambda item: item.relative_to(root).as_posix()):
        resolved = path.resolve()
        try:
            relative = resolved.relative_to(root).as_posix()
            resolved.relative_to(tree_root)
        except ValueError as exc:
            raise ValueError("generated USD tree contains a file outside its package") from exc
        data = resolved.read_bytes()
        files.append({"path": relative, "size_bytes": len(data),
                      "sha256": hashlib.sha256(data).hexdigest()})
    identity = {
        "schema": "generated_usd_tree_identity_v1",
        "entrypoint": entry_relative,
        "files": files,
    }
    identity["aggregate_sha256"] = canonical_digest(identity)
    return identity


def build_initialization_diagnostic_contract(
    validation_config: str | Path, dynamics_config: str | Path, project_root: str | Path,
) -> dict[str, Any]:
    root = Path(project_root).resolve()
    config = load_layout_validation_config(validation_config)
    layout = config.layout
    official = audit_m710id70_official_model(root)
    dynamics = load_m710id70_official_dynamics(dynamics_config)
    robot, q = layout.robot(), config.initial_q
    frames = robot.named_link_frames(q)

    def record(box, role):
        return {"name": box.name, "role": role,
                "pose_world": box.world_from_local.tolist(),
                "size_xyz_m": (box.half_extents * 2.0).tolist()}

    analysis_ref = layout.assets["tool_step_analysis"]
    analysis = json.loads((root / analysis_ref["repository_path"]).read_text(encoding="utf-8"))
    model_path = root / layout.assets["robot_model_config"]["repository_path"]
    model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    srdf_path = (model_path.parent / model["kinematics"]["srdf_path"]).resolve()
    assets = dict(layout.assets)
    for name, path in (("validation_config", Path(validation_config).resolve()),
                       ("layout_config", layout.config_path),
                       ("dynamics_config", Path(dynamics_config).resolve())):
        assets[name] = {"repository_path": path.relative_to(root).as_posix(),
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    assets["robot_srdf"] = {"repository_path": srdf_path.relative_to(root).as_posix(),
                            "sha256": hashlib.sha256(srdf_path.read_bytes()).hexdigest()}
    contract = {
        "schema": SCHEMA, "scope": SCOPE,
        "motion_execution_permitted": False, "attachment_permitted": False,
        "robot_pose_role": "diagnostic_pose_not_motion_qualified_home",
        "layout_id": layout.data["layout_id"], "layout_fingerprint": layout.layout_fingerprint,
        "assets": assets, "official_model_audit": asdict(official),
        "initial_state_audit": audit_initial_state(config),
        "world": layout.data["world"], "trailer": layout.data["trailer"],
        "robot": {"joint_names": list(robot.active_joint_names), "q_rad": q.tolist(),
                  "world_from_mount": layout.robot_base_transform().tolist(),
                  "world_from_J6": frames["J6_link"].tolist(),
                  "world_from_flange": frames["flange"].tolist(),
                  "urdf": layout.assets["robot_urdf"]},
        "primitives": [*[record(b, "fixed_assembly") for b in layout.fixed_components()],
                       *[record(b, "dynamic_carton") for b in layout.cartons()],
                       *[record(b, "rigid_tool") for b in robot.tool_collision_obbs(q)]],
        "tool_visual": {"mesh": layout.assets["tool_collision_mesh"],
                        "flange_origin_step_mm": analysis["flange_origin_step_mm"],
                        "rotation_step_from_tool": analysis["rotation_step_from_tool"]},
        "dynamics": {"fingerprint": dynamics.fingerprint,
                     "source_file_hashes": dict(dynamics.source_file_hashes),
                     "robot_links": {k: asdict(v) for k, v in dynamics.robot_links.items()},
                     "joint_drives": {k: asdict(v) for k, v in dynamics.joint_drives.items()},
                     "tool": {k: v for k, v in asdict(dynamics.tool).items() if k != "config_path"},
                     "cartons": asdict(dynamics.cartons), "simulation": asdict(dynamics.simulation),
                     "contacts": {k: asdict(v) for k, v in dynamics.contacts.items()},
                     "settling": asdict(dynamics.settling)},
        "claims": {"physical_pick_success": False, "machine_qualified": dynamics.machine_qualified,
                   "simulation_inputs_accepted": True, "carton_count": 40,
                   "full_motion_gate": "NOT_BYPASSED_NOT_REQUESTED_BY_THIS_DIAGNOSTIC"},
    }
    contract["contract_fingerprint"] = canonical_digest(contract)
    verify_initialization_diagnostic_contract(contract)
    return contract


def verify_initialization_diagnostic_contract(contract: Mapping[str, Any]) -> None:
    value = dict(contract)
    digest = value.pop("contract_fingerprint", None)
    if value.get("schema") != SCHEMA or digest != canonical_digest(value):
        raise ValueError("initialization diagnostic schema/fingerprint mismatch")
    if (value.get("scope") != SCOPE or value.get("motion_execution_permitted") is not False
            or value.get("attachment_permitted") is not False
            or value["claims"].get("physical_pick_success") is not False):
        raise ValueError("initialization diagnostics must never authorize motion or attachment")
    if value.get("robot_pose_role") != "diagnostic_pose_not_motion_qualified_home":
        raise ValueError("diagnostic robot pose must not be labelled a qualified home")
    records = value["primitives"]
    names = [item["name"] for item in records]
    if len(names) != len(set(names)):
        raise ValueError("duplicate diagnostic geometry identity")
    if sum(item["role"] == "dynamic_carton" for item in records) != 40:
        raise ValueError("initialization diagnostic requires all 40 cartons")
    if sum(item["role"] == "rigid_tool" for item in records) != 58:
        raise ValueError("initialization diagnostic requires all 58 rigid tool solids")
    if value["dynamics"]["cartons"]["mass_kg_each"] != 42.5 or value["dynamics"]["tool"]["mass_kg"] != 20.0:
        raise ValueError("initialization diagnostic mass inventory mismatch")
    if (value["robot"]["urdf"] != value["assets"]["robot_urdf"]
            or value["tool_visual"]["mesh"] != value["assets"]["tool_collision_mesh"]):
        raise ValueError("diagnostic geometry references differ from the hashed assets")
    expected_names = [f"J{i}" for i in range(1, 7)]
    drives = value["dynamics"]["joint_drives"]
    if value["robot"]["joint_names"] != expected_names or set(drives) != set(expected_names):
        raise ValueError("diagnostic requires exactly the six official independent joints")
    efforts = np.asarray([drives[name]["effort_limit_nm"] for name in expected_names], dtype=float)
    velocities = np.asarray([drives[name]["velocity_limit_rad_s"] for name in expected_names], dtype=float)
    if (not np.array_equal(efforts, EXPECTED_EFFORT)
            or not np.allclose(velocities, EXPECTED_VELOCITY, atol=1e-12, rtol=0.0)):
        raise ValueError("diagnostic requires unchanged finite official joint limits")
    if value["initial_state_audit"].get("status") not in {"PASS", "FAIL"}:
        raise ValueError("initialization requires the actual initial-state audit result")


def combined_j6_inertial(contract: Mapping[str, Any]) -> tuple[float, np.ndarray, np.ndarray]:
    """Preserve the official J6 tensor while adding the fixed tool exactly once."""
    robot = contract["robot"]
    transform = np.linalg.inv(np.asarray(robot["world_from_J6"])) @ np.asarray(robot["world_from_flange"])
    link = contract["dynamics"]["robot_links"]["J6_link"]
    tool = contract["dynamics"]["tool"]
    m1, m2 = float(link["mass_kg"]), float(tool["mass_kg"])
    c1 = np.asarray(link["com_xyz_m"])
    c2 = transform[:3, :3] @ np.asarray(tool["com_xyz_m"]) + transform[:3, 3]
    centre = (m1 * c1 + m2 * c2) / (m1 + m2)
    tensor = np.asarray(link["inertia_tensor_com_kg_m2"]) + transform[:3, :3] @ np.asarray(tool["inertia_tensor_com_kg_m2"]) @ transform[:3, :3].T
    for mass, com in ((m1, c1), (m2, c2)):
        delta = com - centre
        tensor += mass * (np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))
    return m1 + m2, centre, tensor


def summarize_initialization_motion(
    states: list[Mapping[str, Any]],
    initial_carton_positions_m: np.ndarray,
    commanded_q_rad: np.ndarray,
    settling: Mapping[str, Any],
) -> dict[str, Any]:
    """Separate final-window rest from retention of the authored initial state."""
    if not states:
        raise ValueError("initialization motion summary requires recorded states")
    times = np.asarray([state["time_s"] for state in states], dtype=float)
    positions = np.asarray([state["carton_positions_m"] for state in states], dtype=float)
    linear = np.asarray([state["carton_linear_velocities_m_s"] for state in states], dtype=float)
    angular = np.asarray([state["carton_angular_velocities_rad_s"] for state in states], dtype=float)
    joints = np.asarray([state["q_rad"] for state in states], dtype=float)
    initial = np.asarray(initial_carton_positions_m, dtype=float)
    commanded = np.asarray(commanded_q_rad, dtype=float)
    if positions.ndim != 3 or positions.shape[-1] != 3 or initial.shape != positions.shape[1:]:
        raise ValueError("initial carton positions do not match recorded state shape")
    if linear.shape != positions.shape or angular.shape != positions.shape:
        raise ValueError("recorded carton velocity arrays do not match positions")
    if joints.ndim != 2 or commanded.shape != joints.shape[1:]:
        raise ValueError("commanded joints do not match recorded state shape")
    if not all(np.all(np.isfinite(value)) for value in (times, positions, linear, angular, joints, initial, commanded)):
        raise ValueError("initialization motion summary requires finite values")
    if times[0] <= 0.0 or (len(times) > 1 and np.any(np.diff(times) <= 0.0)):
        raise ValueError("recorded initialization timestamps must be positive and increasing")
    names = list(states[0]["carton_names"])
    if len(names) != positions.shape[1] or any(list(state["carton_names"]) != names for state in states):
        raise ValueError("recorded carton identities changed during initialization")

    sample_period = float(np.median(np.diff(times))) if len(times) > 1 else float(times[0])
    required_duration = float(settling["required_stable_duration_s"])
    window_count = min(len(states), max(1, int(np.ceil(required_duration / sample_period))))
    window_positions = positions[-window_count:]
    linear_speed = np.linalg.norm(linear, axis=2)
    angular_speed = np.linalg.norm(angular, axis=2)
    displacement = np.linalg.norm(positions - initial[None, :, :], axis=2)
    window_drift = np.linalg.norm(window_positions - window_positions[0:1], axis=2)
    tracking_error = np.abs(joints - commanded[None, :])

    worst_frame, worst_carton = np.unravel_index(int(np.argmax(displacement)), displacement.shape)
    final_window_linear = float(np.max(linear_speed[-window_count:]))
    final_window_angular = float(np.max(angular_speed[-window_count:]))
    final_window_drift = float(np.max(window_drift))
    final_window_stable = (
        final_window_linear <= float(settling["max_linear_speed_m_s"])
        and final_window_angular <= float(settling["max_angular_speed_rad_s"])
        and final_window_drift <= float(settling["max_position_drift_m"])
    )
    retained = float(np.max(displacement)) <= float(settling["max_position_drift_m"])
    if not retained:
        classification = "INITIAL_CONFIGURATION_DISPERSED"
    elif not final_window_stable:
        classification = "MOTION_NOT_SETTLED"
    else:
        classification = "RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY"
    return {
        "classification": classification,
        "sample_period_s": sample_period,
        "final_window_s": window_count * sample_period,
        "final_window_peak_linear_speed_m_s": final_window_linear,
        "final_window_peak_angular_speed_rad_s": final_window_angular,
        "final_window_max_position_drift_m": final_window_drift,
        "final_window_motion_stable": bool(final_window_stable),
        "initial_configuration_retained_throughout": bool(retained),
        "full_run_peak_linear_speed_m_s": float(np.max(linear_speed)),
        "full_run_peak_angular_speed_rad_s": float(np.max(angular_speed)),
        "max_authored_position_displacement_m": float(np.max(displacement)),
        "max_final_position_displacement_m": float(np.max(displacement[-1])),
        "worst_displaced_carton": names[worst_carton],
        "worst_displacement_time_s": float(times[worst_frame]),
        "full_run_max_joint_tracking_error_rad": float(np.max(tracking_error)),
        "final_max_joint_tracking_error_rad": float(np.max(tracking_error[-1])),
        "penetration_gate": "NOT_EVALUATED_DIAGNOSTIC_ONLY",
    }


def trailer_side_wall_transform(y_m: float, inward_normal_y: float) -> np.ndarray:
    """Return a side-wall pose whose local +Y plane normal points inward."""
    if not np.isfinite(y_m) or inward_normal_y not in {-1.0, 1.0}:
        raise ValueError("side-wall position must be finite and inward normal must be +/-Y")
    transform = np.eye(4)
    transform[0, 0] = inward_normal_y
    transform[1, 1] = inward_normal_y
    transform[1, 3] = y_m
    return transform
