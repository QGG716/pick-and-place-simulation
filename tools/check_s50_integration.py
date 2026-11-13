"""Reproducible S50 model/layout admission audit; never runs a substitute arm.

Exit 0 means the model definition is admissible, not a planned or executed pick.
Exit 2 retains MODEL_NOT_READY evidence. No search worker is launched by this audit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
import yaml
from unloading_sim.robot_model_binding import ModelNotReady, RobotModelBinding, rigid_transform
from unloading_sim.robot_native_binding import configured_wrist_tool_links
from unloading_sim.s50_qualification import (
    controller_mapping_audit, geometric_definition_errors, timing_audit,
)
from unloading_sim.robot_replacement_layout import (
    BASELINE_COMMIT, build_replacement_layout, compare_non_robot_scenes,
    freeze_non_robot_scene, frozen_baseline_layout,
)


def _evidence_present(value):
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, dict):
        return any(_evidence_present(item) for item in value.values())
    if isinstance(value, list):
        return any(_evidence_present(item) for item in value)
    return False


def qualification_errors(data: dict, binding: RobotModelBinding | None) -> list[str]:
    """Validate supplied evidence structure; populated fields alone are not proof."""
    missing = []
    evidence_fields = {
        'kinematics': ('zero_pose_reference', 'axes_zero_offsets_signs_source'),
        'collision': ('per_link_visual_collision_sources',),
        'tool_interface': ('mounting_geometry_source',),
    }
    for section, fields in evidence_fields.items():
        values = data.get(section) or {}
        for field in fields:
            value = values.get(field)
            if not isinstance(value, (str, dict)) or not _evidence_present(value):
                missing.append(section+'.'+field)
    tool = data.get('tool_interface') or {}
    if tool.get('flange_from_project_tool_mount') is None:
        missing.append('tool_interface.flange_from_project_tool_mount')
    else:
        try:
            rigid_transform(tool['flange_from_project_tool_mount'], 'flange_from_project_tool_mount')
        except (TypeError, ValueError):
            missing.append('tool_interface.INVALID_FLANGE_FROM_PROJECT_TOOL_MOUNT')
    if tool.get('additional_physical_adapter') is not False:
        missing.append('tool_interface.UNAPPROVED_ADDITIONAL_PHYSICAL_ADAPTER')
    collision = data.get('collision') or {}
    for field in ('reviewed_self_collision_pairs', 'owned_tool_exempt_robot_links'):
        if not isinstance(collision.get(field), list):
            missing.append('collision.'+field)
    # Velocity revisions belong to timing_audit, not static/geometric admission.
    if binding is None:
        return missing
    try:
        configured_wrist_tool_links(binding)
    except ModelNotReady as exc:
        missing.extend(exc.missing)
    xml = ET.parse(binding.urdf_path).getroot()
    links = {link.get('name'): link for link in xml.findall('link')}
    # Collisionless fixed frame aliases are allowed; the physical base and six
    # moving bodies each require visual and collision geometry with provenance.
    physical_links = {binding.base_link, *binding.chain_child_links}
    sources = collision.get('per_link_visual_collision_sources')
    for name in sorted(physical_links):
        if not isinstance(sources, dict) or not _evidence_present(sources.get(name)):
            missing.append('collision.LINK_SOURCE_MISSING:'+name)
        for role in ('visual', 'collision'):
            if not any(list(shape.find('geometry')) if shape.find('geometry') is not None else []
                       for shape in links[name].findall(role)):
                missing.append('collision.LINK_GEOMETRY_MISSING:'+name+':'+role)
    reviewed = collision.get('reviewed_self_collision_pairs')
    if isinstance(reviewed, list):
        pairs = []
        for entry in reviewed:
            pair = [entry.get('link1'), entry.get('link2')] if isinstance(entry, dict) else entry
            if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                    or any(not isinstance(link, str) or link not in links for link in pair) or pair[0] == pair[1]):
                missing.append('collision.INVALID_REVIEWED_SELF_COLLISION_PAIR')
                continue
            pairs.append(tuple(sorted(pair)))
        actual = {tuple(sorted((pair.get('link1'), pair.get('link2'))))
                  for pair in ET.parse(binding.srdf_path).getroot().findall('disable_collisions')}
        if len(pairs) != len(set(pairs)) or set(pairs) != actual:
            missing.append('collision.REVIEWED_SRDF_PAIR_MISMATCH')
    return missing


def cad_preview_audit(root=ROOT):
    """Load the real local model and evaluate FK, independent of task limits."""
    try:
        from unloading_sim.s50_cad_model import ensure_preview_assets, load_definition, load_preview_robot
        materialized = ensure_preview_assets(root)
        definition = load_definition(root)
        robot = load_preview_robot(root)
        frames = robot.named_link_frames(np.zeros(6))
        if len(robot.active_joint_names) != 6 or not all(np.isfinite(frame).all() for frame in frames.values()):
            raise ValueError('CAD_PREVIEW_FK_INVALID')
        return dict(ready=True, status='READY_CAD_KINEMATIC_PREVIEW_ONLY',
            model_definition_sha256=definition['definition_sha256'],
            active_joint_names=list(robot.active_joint_names),
            zero='USER_AUTHORIZED_CAD_ASSEMBLY_SIMULATION_ZERO_NOT_CONTROLLER_ZERO',
            positive='RIGHT_HAND_RULE_ABOUT_MEASURED_AXES',
            zero_flange_in_sim_mount=frames[definition['flange_link']].tolist(),
            assets=materialized, controller_mapping_required_for_preview=False,
            velocity_revision_required_for_static_fk=False,
            display_ranges_are_mechanical_limits=False, formal_initial_state_solved=False,
            collision_qualified=False, missing=[])
    except (OSError, ValueError, TypeError, KeyError, ImportError) as exc:
        return dict(ready=False, status='CAD_PREVIEW_NOT_READY',
            missing=[type(exc).__name__ + ':' + str(exc)],
            controller_mapping_required_for_preview=False, velocity_revision_required_for_static_fk=False)


def audit(config: Path | None = None, root: Path = ROOT) -> dict:
    from unloading_sim.s50_task_model import select_task_configuration
    selection = select_task_configuration(config, root=root)
    config = Path(selection['audit_config'])
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    if data.get("model_id") != "huayan_s50_2000":
        raise ValueError("S50_ENTRY_REQUIRES_EXACT_MODEL_ID")
    reference = frozen_baseline_layout(root)
    replacement = build_replacement_layout(root=root)
    frozen = freeze_non_robot_scene(replacement)
    difference = compare_non_robot_scenes(freeze_non_robot_scene(reference), frozen)
    missing = list(selection['missing'])
    # Geometry is compared first; the S50 environment then carries the S50
    # carton mass (FANUC keeps 42.5 kg). A missing value fails closed.
    from unloading_sim.s50_payload import apply_s50_carton_mass
    inherited_carton_mass = frozen["tool_intrinsic"]["carton_mass_kg"]
    try:
        frozen = apply_s50_carton_mass(frozen, root)
        carton_mass_status = "CONFIGURED"
    except ValueError as exc:
        carton_mass_status = str(exc)
        missing.append("payload." + str(exc))
    binding = None
    try:
        if selection['selected_task_config'] is not None:
            binding = RobotModelBinding.from_config(config)
    except ModelNotReady as exc:
        missing.extend(exc.missing)
    except (OSError, ValueError, TypeError, ET.ParseError) as exc:
        missing.append('model.INVALID_DEFINITION:'+type(exc).__name__+':'+str(exc))
    # Source qualification is separate from syntactically valid URDF files.
    missing.extend(qualification_errors(data, binding))
    missing.extend(geometric_definition_errors(data))
    from unloading_sim.s50_task_model import task_identity_audit
    task_identity = task_identity_audit(config)
    missing.extend(task_identity['missing'])
    mapped_limits = controller_mapping_audit(data)
    if binding is not None and mapped_limits['ready']:
        expected = np.asarray([mapped_limits['converted_limits_rad'][f'J{i}'] for i in range(1,7)])
        if not np.allclose(binding.joint_limits, expected, atol=1e-10, rtol=0):
            missing.append('controller_mapping.SIMULATION_URDF_LIMITS_DO_NOT_MATCH_VERIFIED_MAPPING')
    source_path = root / "assets/robots/huayan_s50_2000/sources.json"
    sources = json.loads(source_path.read_text(encoding="utf-8")) if source_path.is_file() else {}
    source = sources.get("user_supplied_cad", {})
    cad_path = root / source.get("copied_path", "__missing_s50_step__")
    actual_sha = hashlib.sha256(cad_path.read_bytes()).hexdigest() if cad_path.is_file() else None
    cad_verified = actual_sha is not None and actual_sha == source.get("sha256")
    if not cad_verified:
        missing.append("source.STEP_SHA256_NOT_VERIFIED")
    if data.get("source", {}).get("model_file_sha256") != actual_sha:
        missing.append("source.CONFIG_STEP_HASH_MISMATCH")
    cartons = list(reference.cartons())
    # Select the actual highest surface, then a central representative; no ID parsing.
    highest = max(float(b.corners()[:, 2].max()) for b in cartons)
    top = [b for b in cartons if abs(float(b.corners()[:, 2].max()) - highest) < 1e-9]
    representative = min(top, key=lambda b: (abs(float(b.center[1])), b.name))
    missing = sorted(set(missing))
    ready = not missing and difference["passed"]
    dynamic_missing = [k for k in ("link_inertials", "joint_effort_limits_nm", "joint_acceleration_limits_rad_s2") if data.get(k) is None]
    tool_mass = frozen["tool_intrinsic"]["tool_mass_kg"]
    carton_mass = frozen["tool_intrinsic"]["carton_mass_kg"] if carton_mass_status == "CONFIGURED" else None
    combined_mass = None if carton_mass is None else tool_mass + carton_mass
    return dict(schema="s50_model_integration_admission_v1", baseline_commit=BASELINE_COMMIT,
        status="MODEL_DEFINITION_ADMISSIBLE_NOT_MOTION_VALIDATED" if ready else "MODEL_NOT_READY",
        replacement_complete=False, missing_critical_data=missing,
        effective_config=str(config), task_config_selection=selection,
        config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
        controller_limit_mapping=controller_mapping_audit(data),
        task_model_identity=task_identity,
        cad_joint_preview=cad_preview_audit(root),
        time_parameterization=timing_audit(data),
        geometric_planning=dict(ready=ready, status='DEFINITION_ADMISSIBLE_NOT_MOTION_VALIDATED' if ready else 'NOT_READY',
            missing=missing, speed_revision_conflict_is_not_a_geometry_blocker=True),
        model_binding=None if binding is None else binding.identity(),
        cad=dict(path=source.get("copied_path"), sha256=actual_sha, source_hash_verified=cad_verified,
                 official_download_hash_equivalence_verified=False),
        layout_difference=difference, parameters=frozen["parameters"],
        environment_fingerprint=frozen["environment_fingerprint"],
        full_scene_fingerprint=None, initial_carton_ids=frozen["initial_carton_ids"],
        imported_completed_count=0, assumed_removed=[], initial_q_rad=None,
        requested_contact_pose_world=frozen["initial_physical_contact_target"],
        initial_ik=dict(status="NOT_RUN", search_attempts=0, reason="MODEL_NOT_READY" if not ready else "NOT_REQUESTED_BY_ADMISSION_AUDIT"),
        initial_mesh_collision=dict(status="NOT_RUN"),
        native_single_carton=dict(status="NOT_RUN", target=representative.name,
            start_carton_count=40, planning_attempts=0, global_unreachable=False,
            reason="MODEL_NOT_READY" if not ready else "ADMISSION_AUDIT_DOES_NOT_LAUNCH_PLANNER"),
        native_worker_launched=False, dynamics=dict(status="NOT_RUN", missing_data=dynamic_missing,
            missing_dynamic_data_blocks_geometric_search=False),
        payload=dict(rated_robot_payload_kg=float(data["rated_payload_kg"]), tool_mass_kg=tool_mass,
            carton_mass_kg=carton_mass, carton_mass_status=carton_mass_status,
            carton_mass_source=frozen["tool_intrinsic"].get("carton_mass_source"),
            inherited_environment_carton_mass_kg=inherited_carton_mass,
            combined_mass_kg=combined_mass,
            nominal_mass_exceeds_rating=None if combined_mass is None else combined_mass > float(data["rated_payload_kg"]),
            qualification="NOT_QUALIFIED", blocks_geometric_search=False),
        measured_capability=dict(robot_belt_minimum_gap_m=None, elbow_envelope_m=None,
            wrist_envelope_m=None, target_reachable=None, reason="NO_QUALIFIED_S50_MOTION_MODEL"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
        help="Generated task/model.yaml, or its generator input which resolves to the hash-bound generated model")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/s50_mount_entry_20261111/admission.json")
    args = parser.parse_args(argv)
    report = audit(args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("status", "missing_critical_data", "native_worker_launched")}, ensure_ascii=False))
    return 2 if report["status"] == "MODEL_NOT_READY" else 0


if __name__ == "__main__":
    raise SystemExit(main())
