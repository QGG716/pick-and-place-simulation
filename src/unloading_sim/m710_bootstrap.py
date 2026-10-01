"""Non-executable, source-bound initialization of one retained M-710 world.

Import remains standard-library only for the pre-Kit gate. No saved motion,
grasp, cup selection or execution-preflight artifact is accepted here.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

SCHEMA = "m710_native_cold_bootstrap_v1"
READY = "INITIAL_WORLD_RETAINED_AWAITING_NATIVE_PLAN"
TARGET = "carton_l07_c02"


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def _file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_bootstrap_contract(contract, *, project_root=None):
    payload = copy.deepcopy(contract)
    expected = payload.pop("contract_sha256", None)
    if payload.get("schema") != SCHEMA or expected != _digest(payload):
        raise ValueError("BOOTSTRAP_CONTRACT_IDENTITY_MISMATCH")
    if payload.get("motion_execution_permitted") is not False or payload.get("attachment_permitted") is not False:
        raise ValueError("BOOTSTRAP_MUST_NOT_AUTHORIZE_MOTION")
    metadata = payload["metadata"]
    if (metadata.get("robot_model") != "fanuc_m710id_70" or metadata.get("target") != TARGET
            or metadata.get("initialization_only") is not True
            or metadata.get("motion_execution_permitted") is not False
            or metadata.get("attachment_permitted") is not False
            or metadata.get("simulation_execution_ready") is not False
            or metadata.get("simulation_execution_qualified") is not False
            or metadata.get("execution_qualified") is not False):
        raise ValueError("BOOTSTRAP_INVALID_SCOPE")
    if any(key in metadata for key in ("m710_execution_preflight", "m710_replay_contract", "native_backend",
                                      "joint_reference", "trajectory_stage_ranges")):
        raise ValueError("BOOTSTRAP_FORBIDDEN_MOTION_INPUT")
    if any(key in payload for key in ("path", "segments", "timestamps_seconds", "positions_rad")):
        raise ValueError("BOOTSTRAP_FORBIDDEN_MOTION_INPUT")
    q = metadata.get("initial_q_rad")
    if (not isinstance(q, list) or len(q) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in q)):
        raise ValueError("BOOTSTRAP_INVALID_INITIAL_Q")
    if any(v < lo or v > hi for v, lo, hi in zip(q, metadata["joint_position_lower_limits_rad"],
                                               metadata["joint_position_upper_limits_rad"])):
        raise ValueError("BOOTSTRAP_INITIAL_Q_LIMIT")
    cartons = [p for p in metadata["scene_primitives"] if p.get("category") == "carton"]
    if (len(cartons) != 40 or len({p["name"] for p in cartons}) != 40
            or TARGET not in {p["name"] for p in cartons}
            or any(p.get("dynamic") is not True or p.get("mass_kg") != 42.5 for p in cartons)):
        raise ValueError("BOOTSTRAP_REQUIRES_40_UNCHANGED_DYNAMIC_CARTONS")
    if any(metadata.get(key) for key in ("initial_actual_state_context", "completed_carton_ids",
            "processed_carton_ids", "handed_off_ids", "receiver_transport_state")):
        raise ValueError("BOOTSTRAP_FORBIDDEN_PRIOR_WORLD_STATE")
    gripper = metadata["gripper"]
    if gripper.get("gripper_mass_kg") != 20.0 or gripper.get("physical_cup_count") != 72:
        raise ValueError("BOOTSTRAP_TOOL_INVENTORY_MISMATCH")
    for field in ("geometrically_eligible_mask", "commanded_active_mask", "planned_fk_contact_mask", "actual_contact_mask"):
        if gripper.get(field) != [False] * 72 or any(type(v) is not bool for v in gripper[field]):
            raise ValueError("BOOTSTRAP_CUPS_MUST_BE_UNCOMMANDED")
    if any(metadata.get(key) is not None for key in ("grasp_time_seconds", "release_time_seconds", "release_retreat_time_seconds")):
        raise ValueError("BOOTSTRAP_FORBIDDEN_EXECUTION_EVENT")
    output = metadata["rendering"]["required_output"]
    if (output["width_px"], output["height_px"], output["fps"]) != (640, 360, 5):
        raise ValueError("BOOTSTRAP_RECORDING_PROFILE_MISMATCH")
    if payload.get("initial_state_audit", {}).get("status") != "PASS":
        raise ValueError("BOOTSTRAP_INITIAL_STATE_INVALID")
    if payload.get("asset_audit", {}).get("execution_qualified") is not True:
        raise ValueError("BOOTSTRAP_ASSETS_NOT_READY")
    if project_root is not None:
        root = Path(project_root).resolve()
        for relative, expected_hash in payload["source_sha256"].items():
            path = (root / relative).resolve()
            if not path.is_relative_to(root) or not path.is_file() or _file_hash(path) != expected_hash:
                raise ValueError("BOOTSTRAP_WORKSPACE_SOURCE_CHANGED: " + relative)
    return {"status": "INITIALIZATION_ONLY", "contract_sha256": expected,
            "motion_execution_permitted": False, "attachment_permitted": False}


def build_bootstrap_contract(execution_config, *, project_root=None):
    from .m710_execution import (load_m710_execution_config, _audited_execution_assets,
        _scene_primitives, _bridge_configuration, _execution_implementation_identity)
    from .layout_single_carton import load_layout_motion_policy, build_verified_motion_input
    from .m710_dynamics import load_m710id70_dynamics
    from .isaac_bridge import build_m710_initialization_metadata
    from .collision_policy import SimulationCollisionPolicy
    from .planning_profile import profile_evidence
    import yaml

    execution = load_m710_execution_config(execution_config)
    root = execution.project_root if project_root is None else Path(project_root).resolve()
    if root != execution.project_root:
        raise ValueError("BOOTSTRAP_EXECUTION_CONFIG_ROOT_MISMATCH")
    policy = load_layout_motion_policy(execution.motion_policy_path)
    scene = build_verified_motion_input(policy, root)
    dynamics = load_m710id70_dynamics(execution.dynamics_path)
    assets, manifest, official_audit = _audited_execution_assets(execution, scene, dynamics)
    if not assets["execution_qualified"]:
        raise ValueError("BOOTSTRAP_ASSETS_NOT_READY")
    model_path = root / scene.snapshot["robot"]["model_config"]["repository_path"]
    model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    srdf_path = (model_path.parent / model["kinematics"]["srdf_path"]).resolve()
    source_hashes = _execution_implementation_identity(root)
    for path in (execution.config_path, execution.motion_policy_path, execution.layout_validation_path,
                 execution.dynamics_path, execution.robot_manifest_path, execution.tool_manifest_path,
                 model_path, srdf_path):
        source_hashes[path.relative_to(root).as_posix()] = _file_hash(path)
    for record in scene.snapshot.get("assets", {}).values():
        if isinstance(record, dict) and record.get("repository_path"):
            source_hashes[record["repository_path"]] = _file_hash(root / record["repository_path"])
    plan = dict(
        robot=dict(model="fanuc_m710id_70", urdf_path=manifest["integration"]["expanded_urdf"]["path"],
            srdf_path=srdf_path.relative_to(root).as_posix(), srdf_sha256=_file_hash(srdf_path),
            base_position=[row[3] for row in scene.snapshot["robot"]["world_from_mount"][:3]],
            base_rpy=[0., 0., 0.],
            isaac_grasp_body_path_suffix="Geometry/base_link/J1_link/J2_link/J3_link/J4_link/J5_link/J6_link",
            flange_offset_from_grasp_body_m=[.175, 0., 0.], official_model_manifest=manifest),
        official_model_manifest=manifest, official_model_manifest_sha256=assets["robot"]["manifest_sha256"],
        official_model_audit=official_audit, official_model_required=True,
        collision_policy=SimulationCollisionPolicy.from_mapping(policy.layout_validation.data.get("collision_policy")).to_mapping(),
        stack_carton_names=[box.name for box in scene.cartons],
        scene_primitives=_scene_primitives(scene, dynamics, execution),
        simulation_profile=profile_evidence(policy.data),
        post_landing_transport=dict(policy.data["search_strategy"]["post_landing_transport"]),
        simulation_execution_ready=False, simulation_execution_qualified=False, execution_qualified=False,
        execution_blockers=["NATIVE_COLD_MOTION_NOT_YET_PLANNED"],
        machine_qualified=dynamics.machine_qualified, machine_qualification_warnings=[],
        execution_asset_fingerprint_sha256=_digest(source_hashes),
        collision_geometry="real_cad_per_link_visual_and_compound_collision_required",
    )
    metadata = build_m710_initialization_metadata(plan, _bridge_configuration(scene, dynamics, execution),
        initial_q=scene.snapshot["robot"]["q_rad"], target=TARGET)
    contract = dict(schema=SCHEMA, motion_execution_permitted=False, attachment_permitted=False,
        scope="NEW_WORLD_INITIALIZATION_AND_SETTLING_ONLY", metadata=metadata, source_sha256=source_hashes,
        initial_state_audit=scene.snapshot["initial_state_audit"], asset_audit=assets,
        layout_fingerprint=scene.snapshot["layout_fingerprint"], scene_fingerprint=scene.snapshot["scene_fingerprint"])
    contract["contract_sha256"] = _digest(contract)
    verify_bootstrap_contract(contract, project_root=root)
    return contract


def audit_bootstrap_assets(contract, *, project_root):
    from .m710_execution import audit_m710_execution_asset_inputs
    root = Path(project_root).resolve()
    metadata, assets = contract["metadata"], contract["asset_audit"]
    gripper = metadata["gripper"]
    current, _, _ = audit_m710_execution_asset_inputs(project_root=root,
        robot_manifest_path=root / assets["robot"]["manifest_path"],
        tool_manifest_path=root / assets["tool"]["manifest_path"],
        frame_contract=gripper["frame_contract"],
        rigid_collision_obb_count=len(gripper["qualified_rigid_collision_boxes_tool_frame"]),
        collision_representation="CAD_RIGID_STRUCTURES_AND_INSERTS_WITH_SEPARATE_FLEXIBLE_BELLOWS",
        vacuum={key: gripper[key] for key in ("suction_mode", "physical_cup_count", "cup_rows", "cup_columns")},
        collision_policy=metadata["collision_policy"])
    if current != assets:
        raise ValueError("BOOTSTRAP_ASSET_REAUDIT_MISMATCH")
    return current


def measured_initial_state(metadata, *, q_rad, qd_rad_s, joint_names, cartons, world_session_id, time_s,
                           native_rest_start_evidence=None):
    if len(cartons) != 40 or {c["name"] for c in cartons} != set(metadata["stack_carton_names"]):
        raise ValueError("BOOTSTRAP_READBACK_CARTON_IDENTITY_MISMATCH")
    if list(joint_names) != metadata["joint_names"]:
        raise ValueError("BOOTSTRAP_READBACK_JOINT_ORDER_MISMATCH")
    return dict(schema="m710id70_actual_motion_state_v1", q_rad=list(q_rad), qd_rad_s=list(qd_rad_s),
        joint_names=list(joint_names), world_session_id=world_session_id, time_s=float(time_s),
        attached=False, attachment_target=None,
        cartons=[dict(name=c["name"], position_m=c["center_m"], orientation_wxyz=c["quaternion_wxyz"],
            linear_velocity_m_s=c["linear_velocity_m_s"], angular_velocity_rad_s=c["angular_velocity_rad_s"])
            for c in cartons], completed_carton_ids=[], processed_carton_ids=[], ideal_received_ids=[],
        handed_off_ids=[], inactive_carton_ids=[], receiver_transport_state={},
        simulation_profile=copy.deepcopy(metadata["simulation_profile"]),
        post_landing_transport=copy.deepcopy(metadata["post_landing_transport"]),
        native_rest_start_evidence=copy.deepcopy(native_rest_start_evidence),
        initialization_provenance=dict(world_scope="NEW_WORLD_NATIVE_COLD", historical_motion_inputs_read=0,
            historical_events_counted_as_new_task_completion=False))


def verify_initial_plan_request(request, *, world_session_id, actual_state_sha256):
    for name, expected in (("world_session_id", world_session_id), ("actual_state_sha256", actual_state_sha256)):
        if request.get(name) != expected:
            raise ValueError("BOOTSTRAP_STALE_OR_MISSING_REQUEST_IDENTITY: " + name)
    if request.get("stop") is True:
        return
    if not isinstance(request.get("bundle_path"), str) or not request["bundle_path"]:
        raise ValueError("BOOTSTRAP_REQUEST_MISSING_BUNDLE")


def verify_initial_bundle_scope(bundle, state):
    metadata = bundle["metadata"]
    if (bundle.get("format") != "isaacsim_fanuc_replay_v1" or metadata.get("target") != TARGET
            or metadata.get("native_cold") is not True or metadata.get("require_native_motion") is not True):
        raise ValueError("BOOTSTRAP_REQUIRES_NATIVE_COLD_TARGET_BUNDLE")
    context = metadata.get("initial_actual_state_context") or {}
    if context.get("world_session_id") != state["world_session_id"] or context.get("actual_state_fingerprint") != _digest(state):
        raise ValueError("BOOTSTRAP_PLAN_NOT_BOUND_TO_MEASURED_INITIAL_STATE")
    if any(metadata.get(key) for key in ("completed_carton_ids", "processed_carton_ids", "handed_off_ids", "receiver_transport_state")):
        raise ValueError("BOOTSTRAP_PLAN_IMPORTS_COMPLETION_EVENTS")
