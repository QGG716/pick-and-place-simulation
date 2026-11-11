"""S50 preview FK and separately gated single-carton planning entry.

CAD preview uses only its declared simulation frame convention. It never uses
preview angle ranges for a production planning request or installs a tool.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from unloading_sim.robot import URDFRobot
from unloading_sim.robot_model_binding import RobotModelBinding, rigid_transform
from unloading_sim.robot_native_binding import native_fk_model_request


def save(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def preview_probes(binding):
    zero = np.zeros(6)
    if not np.all((zero >= binding.joint_limits[:, 0]) & (zero <= binding.joint_limits[:, 1])):
        raise ValueError('CAD_PREVIEW_ZERO_OUTSIDE_DECLARED_DISPLAY_RANGE')
    values = [zero]
    for index in range(6):
        for sign in (1., -1.):
            q = zero.copy(); q[index] = sign*.1
            if not binding.joint_limits[index, 0] <= q[index] <= binding.joint_limits[index, 1]:
                raise ValueError('CAD_PREVIEW_DISPLAY_RANGE_TOO_SMALL_FOR_DIRECTION_PROBE')
            values.append(q)
    values.extend(binding.fk_probe_configurations()[1:])
    return values


def run_preview_fk(config, output, *, worker_command=None, seed=50, root=ROOT):
    from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
    from unloading_sim.moveit2_backend import ResidentMoveItClient, MoveItUnavailable
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    binding = RobotModelBinding.from_config(config)
    if binding.data.get('model_usage') != 'cad_preview_fk_only':
        raise ValueError('PREVIEW_FK_REQUIRES_EXPLICIT_CAD_PREVIEW_CONFIG')
    robot = URDFRobot.from_config(config, mounting_world_transform=np.eye(4))
    mount = robot.base_transform
    init = native_fk_model_request(binding, world_from_base=mount, project_root=root, seed=seed)
    save(output/'native-fk-init.json', init)
    links = list(dict.fromkeys([binding.base_link, *binding.chain_child_links, binding.flange_link, binding.tip_link]))
    result = dict(schema='s50_cad_native_fk_validation_v1', model_usage='cad_preview_fk_only',
        model_binding=binding.identity(), coordinate_scope='DECLARED_CAD_SIMULATION_FRAME_ONLY',
        controller_zero_or_limits_verified=False, installation_verified=False,
        tool_installed=False, collision_acceptance_evaluated=False, planning_attempts=0,
        native_trajectory={'status': 'NOT_RUN', 'points': []}, dynamics_qualified=False,
        native_worker_started=False, pinocchio={'status': 'NOT_RUN'}, native={'status': 'NOT_RUN'}, probes=[])
    definition = root/'assets/robots/huayan_s50_2000/generated/cad_kinematic_model.json'
    result['model_definition_sha256'] = hashlib.sha256(definition.read_bytes()).hexdigest() if definition.is_file() else None
    mesh = client = None
    try:
        try:
            mesh = PinocchioHppFclBackend.from_binding(binding, output_dir=output/'pinocchio-model',
                project_root=root, world_from_base=mount)
            result['pinocchio'] = dict(status='LOADED', collision_geometry_count=len(mesh.geometry_model.geometryObjects),
                joint_names=list(mesh.active_joint_names), parser_compatibility=mesh.native_parser_compatibility)
        except (OSError, RuntimeError, ValueError) as exc:
            result['pinocchio'] = dict(status='UNAVAILABLE', reason=type(exc).__name__, detail=str(exc))
        if worker_command:
            client = ResidentMoveItClient(worker_command, log_path=str(output/'native-worker.log'))
            result['native_worker_started'] = True
            ready = client.request(init)
            if (ready.get('status') != 'READY' or ready.get('model_usage') != 'cad_preview_fk_only'
                    or ready.get('capabilities', {}).get('motion_generation') is not False):
                raise ValueError('NATIVE_WORKER_DOES_NOT_ENFORCE_FK_ONLY_PROTOCOL')
            result['native'] = dict(status='LOADED', init_response=ready)
        for q in preview_probes(binding):
            reference = {name:rigid_transform(value,'core.'+name) for name,value in robot.named_link_frames(q).items()}
            row = dict(q_rad=q.tolist(), core_frames={name: reference[name].tolist() for name in links})
            if mesh is not None:
                frames = {name:rigid_transform(value,'pinocchio.'+name) for name,value in mesh.named_link_frames(q).items()}
                row['pinocchio_frames'] = {name: frames[name].tolist() for name in links}
                row['pinocchio_max_matrix_error'] = max(float(np.max(np.abs(frames[name]-reference[name]))) for name in links)
                if row['pinocchio_max_matrix_error'] > 1e-8: raise ValueError('PREVIEW_PINOCCHIO_FK_MISMATCH')
            if client is not None:
                reply = client.request(dict(op='fk', identity=init['identity'], q_start=q.tolist(), links=links))
                for name in links:rigid_transform(reply['frames'][name],'native.'+name)
                row['native_frames'] = reply['frames']
                row['native_max_matrix_error'] = max(float(np.max(np.abs(np.asarray(reply['frames'][name])-reference[name]))) for name in links)
                if row['native_max_matrix_error'] > 1e-8: raise ValueError('PREVIEW_NATIVE_FK_MISMATCH')
            result['probes'].append(row)
        if client is not None:
            # A real protocol rejection proves display ranges cannot be used
            # to generate motion through this resident model session.
            try:
                client.request(dict(op='ik', identity=init['identity'], q_start=[0.]*6))
                rejected = dict(status='UNEXPECTEDLY_ACCEPTED')
            except MoveItUnavailable as exc:
                rejected = dict(status='ERROR', detail=str(exc))
            result['native']['motion_rejection_probe'] = rejected
            if rejected.get('status') != 'ERROR' or 'CAD_PREVIEW_FK_ONLY_NO_MOTION' not in str(rejected):
                raise ValueError('NATIVE_PREVIEW_MOTION_NOT_REJECTED')
        if mesh is not None: result['pinocchio']['status'] = 'PASS_FK_ONLY'
        if client is not None: result['native']['status'] = 'PASS_FK_ONLY'
        result['status'] = 'PASS_PREVIEW_CORE_PINOCCHIO_NATIVE_FK_ONLY' if mesh is not None and client is not None else 'PARTIAL_PREVIEW_FK_CHECKS'
    except Exception as exc:
        result.update(status='PREVIEW_VALIDATION_FAILED', failure=dict(reason=type(exc).__name__, detail=str(exc)))
    finally:
        if client is not None: client.close()
        save(output/'preview-fk-report.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['preview-fk', 'single-carton'], default='single-carton')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/s50_native_single_carton')
    parser.add_argument('--worker-command')
    parser.add_argument('--seed', type=int, default=50)
    parser.add_argument('--initial-draws', type=int, default=64)
    parser.add_argument('--task-resource-seconds', type=float,
        help='Explicit resource ceiling; omitted uses the frozen launch input, shared across candidates')
    args = parser.parse_args(argv)
    if args.mode == 'preview-fk':
        from unloading_sim.s50_cad_model import ensure_preview_assets
        ensure_preview_assets(ROOT)
        config = args.config or ROOT/'configs/robots/huayan_s50_2000_cad_preview.yaml'
        report = run_preview_fk(config, args.output, worker_command=args.worker_command, seed=args.seed)
        print(json.dumps({'status': report['status'], 'probes': len(report['probes'])}))
        return 0 if report['status'] == 'PASS_PREVIEW_CORE_PINOCCHIO_NATIVE_FK_ONLY' else 2
    from check_s50_integration import audit
    config = args.config or ROOT/'configs/robots/huayan_s50_2000.yaml'
    admission = audit(config)
    from unloading_sim.configured_layout_trajectory import plan_single_carton_from_admission
    report = plan_single_carton_from_admission(admission, config=config, output=args.output,
        worker_command=args.worker_command, seed=args.seed, root=ROOT,
        initial_draws=args.initial_draws, task_seconds=args.task_resource_seconds)
    save(args.output/'single-carton-report.json', report)
    print(json.dumps({'status': report['status'], 'missing': report.get('missing', [])}, ensure_ascii=False))
    return 0 if report.get('complete') else 2


if __name__ == '__main__':
    raise SystemExit(main())
