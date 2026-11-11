"""Export the real six-axis CAD preview; never relabel preview motion as planning."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
from export_s50_model_viewer import build_data as static_scene_data, encode_payload, serialize_payload
from unloading_sim.s50_cad_model import ensure_preview_assets, load_definition, load_preview_robot, read_obj
from unloading_sim.robot_model_binding import RobotModelBinding, rigid_transform
from unloading_sim.robot import URDFRobot
from s50_viewer_diagnostics import diagnostic_data


def viewer_scene_context(definition, root=ROOT, *, environment=None, mounting_from_base=None):
    """Bind the FK root to this model's measured CAD frames and frozen world."""
    if environment is None:
        from unloading_sim.robot_replacement_layout import frozen_baseline_layout, freeze_non_robot_scene
        environment = freeze_non_robot_scene(frozen_baseline_layout(root))
    cad_from_mount = rigid_transform(definition['frames']['mount']['CAD_from_frame_zero'], 'CAD_from_mount')
    base = next(link for link in definition['links'] if link['name'] == definition['base_link'])
    cad_from_base = rigid_transform(base['CAD_from_link_zero'], 'CAD_from_model_base')
    mount_from_base = np.linalg.inv(cad_from_mount) @ cad_from_base
    # The independent viewer FK explicitly starts its root at the mount frame.
    # Another base convention needs an adapted FK, never a silent placement fix.
    if not np.allclose(mount_from_base, np.eye(4), atol=1e-12, rtol=0):
        raise ValueError('VIEWER_FK_ROOT_REQUIRES_MODEL_BASE_AT_SIMULATION_MOUNT')
    mount = rigid_transform(environment['parameters']['world_from_physical_mount'], 'world_from_physical_mount')
    actual_alignment = mount_from_base if mounting_from_base is None else rigid_transform(mounting_from_base, 'mounting_from_base')
    return dict(environment_fingerprint=environment['environment_fingerprint'],
        world_from_physical_mount=mount.tolist(), world_from_model_base=(mount @ actual_alignment).tolist(),
        initial_carton_ids=environment['initial_carton_ids'])


def validate_scene_context(actual, expected):
    """Same model is insufficient: replay also requires this exact frozen world."""
    if not isinstance(actual, dict):
        raise ValueError('TRAJECTORY_SCENE_CONTEXT_REQUIRED')
    if (not expected.get('environment_fingerprint')
            or actual.get('environment_fingerprint') != expected['environment_fingerprint']):
        raise ValueError('TRAJECTORY_ENVIRONMENT_FINGERPRINT_MISMATCH')
    for field in ('world_from_physical_mount', 'world_from_model_base'):
        try:
            observed = rigid_transform(actual.get(field), field)
            reference = rigid_transform(expected[field], field)
        except (TypeError, ValueError) as exc:
            raise ValueError('TRAJECTORY_INVALID_'+field.upper()) from exc
        if not np.allclose(observed, reference, atol=1e-12, rtol=0):
            raise ValueError('TRAJECTORY_'+field.upper()+'_MISMATCH')
    ids, expected_ids = actual.get('initial_carton_ids'), expected.get('initial_carton_ids')
    if (not isinstance(ids, list) or len(ids) != 40 or not all(isinstance(item, str) for item in ids)
            or len(set(ids)) != 40 or sorted(ids) != sorted(expected_ids or [])):
        raise ValueError('TRAJECTORY_FRESH40_CARTON_IDENTITIES_MISMATCH')


def trajectory_data(path, definition, root=ROOT, *, expected_scene_context=None):
    if path is None:
        return dict(status='NOT_RUN', points=[], reason='NO_ACTUAL_NATIVE_TRAJECTORY', failure_state=None,
            playback_scope='robot_joint_only', environment_playback='initial_scene_static_reference',
            scene_context_verified=False)
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('schema') != 's50_native_joint_trajectory_v1':
        raise ValueError('TRAJECTORY_REQUIRES_EXPLICIT_S50_NATIVE_SCHEMA')
    if data.get('model_definition_sha256') != definition['definition_sha256']:
        raise ValueError('TRAJECTORY_MODEL_IDENTITY_MISMATCH')
    names = [joint['name'] for joint in definition['joints']]
    if data.get('joint_names') != names or data.get('source') != 'native_worker_current_run':
        raise ValueError('TRAJECTORY_JOINT_ORDER_OR_NATIVE_SOURCE_MISMATCH')
    if (data.get('native_provenance') or {}).get('current_run') is not True:
        raise ValueError('TRAJECTORY_CURRENT_RUN_NATIVE_PROVENANCE_REQUIRED')
    if (data.get('playback_scope') != 'robot_joint_only'
            or data.get('environment_playback') != 'initial_scene_static_reference'):
        raise ValueError('TRAJECTORY_REQUIRES_ROBOT_JOINT_ONLY_STATIC_ENVIRONMENT_SCOPE')
    expected = expected_scene_context if expected_scene_context is not None else viewer_scene_context(definition, root)
    validate_scene_context(data.get('scene_context'), expected)
    points = data.get('points') or []
    previous = -float('inf')
    for point in points:
        q = np.asarray(point.get('q_rad'), dtype=float)
        timestamp = point.get('time_s')
        if (q.shape != (6,) or not np.isfinite(q).all() or isinstance(timestamp, bool)
                or not isinstance(timestamp, (int, float)) or not np.isfinite(timestamp)
                or timestamp < 0 or timestamp < previous):
            raise ValueError('INVALID_NATIVE_TRAJECTORY_POINT')
        previous = timestamp
        if definition.get('schema') == 's50_task_model_v1':
            limits = np.asarray([joint['position_limits_rad'] for joint in definition['joints']], dtype=float)
            if np.any(q < limits[:,0]-1e-10) or np.any(q > limits[:,1]+1e-10):
                raise ValueError('NATIVE_TRAJECTORY_OUTSIDE_VERIFIED_TASK_LIMITS')
    failure = data.get('failure_state')
    if failure is not None:
        q = np.asarray(failure.get('q_rad'), dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or not failure.get('stage'):
            raise ValueError('INVALID_FAILURE_CONFIGURATION')
    if not points and not failure:
        raise ValueError('TRAJECTORY_HAS_NO_ACTUAL_STATE')
    data['scene_context_verified'] = True
    return data


def load_viewer_model(root, task_config=None):
    ensure_preview_assets(root)
    if task_config is None:
        return load_definition(root), load_preview_robot(root), None
    from unloading_sim.s50_task_model import select_task_configuration
    selection = select_task_configuration(task_config, root=root)
    if not selection['selected_task_config']:
        raise ValueError('VIEWER_TASK_MODEL_NOT_READY:' + json.dumps(selection, ensure_ascii=False))
    binding = RobotModelBinding.from_config(selection['selected_task_config'])
    source = (binding.config_path.parent / binding.data['generated_definition']).resolve()
    model = json.loads(source.read_text(encoding='utf-8'))
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if model.get('schema') != 's50_task_model_v1' or digest != binding.identity().get('model_definition_sha256'):
        raise ValueError('VIEWER_REQUIRES_VERIFIED_GENERATED_TASK_MODEL')
    model['definition_sha256'] = digest
    model['task_config_selection'] = selection
    # The view's FK root is the model base. Physical placement is applied once
    # by viewer_scene_context, rather than baked into both FK and mesh group.
    robot = URDFRobot.from_config(binding.config_path, mounting_world_transform=np.linalg.inv(binding.mounting_from_base))
    return model, robot, binding


def build_data(root=ROOT, trajectory=None, diagnostics=None, task_config=None):
    root = Path(root).resolve()
    model, robot, binding = load_viewer_model(root, task_config)
    data = static_scene_data(root=root)
    context = viewer_scene_context(model, root, environment=data['environment'],
        mounting_from_base=None if binding is None else binding.mounting_from_base)
    # Display exactly the model's link-local visual meshes, not a substitute hull.
    parts = []
    for link in model['links']:
        source = link['visual_mesh']
        path = (root / source['path']).resolve()
        raw = path.read_bytes()
        if not path.is_relative_to(root) or hashlib.sha256(raw).hexdigest() != source['sha256']:
            raise ValueError('LINK_LOCAL_MESH_IDENTITY_MISMATCH:' + link['name'])
        xyz64, triangles = read_obj(path, expected_sha256=source['sha256'])
        xyz, indices = xyz64.astype('<f4'), triangles.astype('<u4')
        parts.append(dict(name=link['name'], cad_body=link['cad_body'], path=source['path'],
            sha256=source['sha256'], vertices=len(xyz), triangles=len(indices),
            float32_display_max_coordinate_error_m=float(np.max(np.abs(xyz.astype(float)-xyz64))),
            positions_b64=base64.b64encode(xyz.tobytes()).decode('ascii'),
            indices_b64=base64.b64encode(indices.tobytes()).decode('ascii')))
    ranges = np.asarray([[-np.pi, np.pi]]*6) if binding is None else binding.joint_limits
    q = np.clip(np.zeros(6), ranges[:,0], ranges[:,1])
    status = ('CAD_PREVIEW_READY_FORMAL_PLANNING_NOT_VERIFIED' if binding is None
        else 'TASK_LIMIT_MODEL_LOADED_TASK_EXECUTION_NOT_QUALIFIED')
    data.update(schema='s50_articulated_cad_viewer_v1', model_status=status,
        model_definition=model, articulated_model=model, local_parts=parts, joint_controls_enabled=True,
        joint_controls_disabled_reason=None, preview_q0_rad=q.tolist(),
        preview_slider_ranges_rad=ranges.tolist(),
        preview_range_semantics='DISPLAY_ONLY_NOT_MECHANICAL_LIMITS' if binding is None else 'VERIFIED_CONTROLLER_TO_SIMULATION_LIMIT_MAPPING',
        task_model_binding=None if binding is None else binding.identity(),
        joint_ranges_verified=binding is not None,
        zero_core_frames={name: frame.tolist() for name, frame in robot.named_link_frames(q).items()},
        viewer_scene_context=context,
        playback=trajectory_data(trajectory, model, root, expected_scene_context=context),
        diagnostics=diagnostic_data(diagnostics, model, robot, context, validate_scene_context,
            expected_obstacles=next(row['entities'] for row in data['environment']['height_samples']
                if abs(row['height_m']-data['environment']['parameters']['initial_belt_height_m'])<1e-12),
            initial_height_m=data['environment']['parameters']['initial_belt_height_m']),
        frame_semantics=dict(zero='CAD_ASSEMBLY_REFERENCE_NOT_CONTROLLER_ZERO',
            positive='EXPLICIT_RIGHT_HAND_RULE_ABOUT_MEASURED_AXES',
            world_overlay='FIXED_MOUNT_POINT_WITH_SIMULATION_CLOCKING_CONVENTION_NOT_VERIFIED_INSTALLATION',
            tool='FROZEN_TARGET_REFERENCE_ONLY_PHYSICAL_FLANGE_CONNECTION_UNVERIFIED'))
    # Avoid embedding the historical world-coordinate meshes a second time.
    for part in data['cad']['parts']:
        part.pop('positions_b64', None); part.pop('indices_b64', None)
    data['cad']['articulated_model_ready'] = True
    data['cad']['formal_motion_qualified'] = False
    data['trajectory'] = data['playback'].get('points', [])
    data['trajectory_status'] = data['playback']['status']
    return data


def write_viewer(output, data, root=ROOT):
    root = Path(root)
    vendor = root / 'tools/search_replay/vendor'
    license_text = (vendor / 'LICENSE.three').read_text(encoding='utf-8').replace('*/', '* /')
    scripts = '/*\n' + license_text + '\n*/\n' + '\n'.join(
        (vendor / name).read_text(encoding='utf-8') for name in ('three.min.js', 'OrbitControls.js'))
    scripts += '\n' + (root / 'tools/s50_kinematics.js').read_text(encoding='utf-8')
    template = (root / 'tools/s50_cad_motion_viewer.html').read_text(encoding='utf-8')
    html = template.replace('/*__VENDOR__*/', scripts).replace('/*__DATA__*/', json.dumps(encode_payload(data)))
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding='utf-8')
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'docs/experiments/s50_task_collision_20261111/viewer.html')
    parser.add_argument('--trajectory-report', type=Path)
    parser.add_argument('--diagnostics-report', type=Path)
    parser.add_argument('--task-config', type=Path, help='Generated task/model.yaml; never the ungenerated input config')
    args = parser.parse_args(argv)
    data = build_data(trajectory=args.trajectory_report, diagnostics=args.diagnostics_report, task_config=args.task_config)
    write_viewer(args.output, data)
    report = dict(schema=data['schema'], html=str(args.output), model_definition_sha256=data['model_definition']['definition_sha256'],
        model_status=data['model_status'], joint_controls_enabled=True, zero_is_controller_zero=False,
        slider_ranges_are_mechanical_limits=data['joint_ranges_verified'], actual_trajectory_status=data['playback']['status'],
        playback_scope=data['playback']['playback_scope'], environment_playback=data['playback']['environment_playback'],
        trajectory_scene_context_verified=data['playback']['scene_context_verified'],
        collision_diagnostic_pose_count=len(data['diagnostics']['poses']),
        collision_diagnostic_report_sha256=data['diagnostics'].get('source_report_sha256'),
        diagnostic_fk_crosscheck=data['diagnostics'].get('viewer_fk_crosscheck'),
        parts=[{k:v for k,v in p.items() if not k.endswith('_b64')} for p in data['local_parts']],
        float32_display_max_coordinate_error_m=max(p['float32_display_max_coordinate_error_m'] for p in data['local_parts']),
        layout_difference=data['layout_difference'], payload_sha256=hashlib.sha256(serialize_payload(data)).hexdigest(),
        html_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(), browser_visual_validation='PENDING')
    args.output.with_name('viewer-checks.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('html','model_status','joint_controls_enabled','actual_trajectory_status','html_sha256')},ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
