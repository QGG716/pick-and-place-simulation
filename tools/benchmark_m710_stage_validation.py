"""Offline same-input validator benchmark; never a native-cold planning entry.

Saved paths select the motion being tested, never supply a validation result.
Each stage gets a newly built checker and initial contact history. No worker or
Isaac world is launched. The formal supervisor does not import this module.
"""
from __future__ import annotations

import argparse
import cProfile
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import pstats
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from unloading_sim.geometry import OBB
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.validation_physics import RigidAttachment


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def box(record):
    pose = np.asarray(record['pose'], float)
    return OBB(pose[:3, 3], np.asarray(record['size']) / 2, pose[:3, :3],
               record['id'], record['category'])


def prepare(directory, motion, stage):
    policy = load_layout_motion_policy(ROOT / 'configs/validation/m710id70_proof_of_concept.yaml')
    actual = json.loads((directory / 'physics/initialized_actual_state.json').read_text())
    scene = apply_actual_motion_state(build_verified_motion_input(policy, ROOT), actual)
    saved = json.loads((directory/'plan/actual_scene_snapshot.json').read_text())
    # Planning saved its configured row-selection metadata after applying the
    # measured state. Restore that original snapshot only after checking all
    # measured geometry and robot/tool input sections are exactly identical.
    normalized = json.loads(json.dumps(scene.snapshot))
    normalized.pop('scene_fingerprint')
    normalized['actual_state_context']['row_selection'] = saved['actual_state_context']['row_selection']
    assert normalized == {k: v for k, v in saved.items() if k != 'scene_fingerprint'}
    from unloading_sim.workcell_layout import canonical_digest
    assert canonical_digest(normalized) == saved['scene_fingerprint'] == motion['scene_fingerprint']
    scene = replace(scene, snapshot=saved)
    built = _build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot())
    if built.connector is None:
        raise RuntimeError(built.failure_reason)
    c = built.connector
    c.budget = replace(c.budget, planning_wall_time_s=None, candidate_wall_time_s=None,
                       stage_wall_time_s=None)
    c.start_planning_request()
    # Select the final stage by its stored source identity. Stored PASS is never
    # read as a fresh verdict. No motion generator is called by this script.
    selected = motion['selected_trajectory_segment']['native_backend']['stages']
    stage_ids = [r['stage_id'] for r in selected if r['stage'] == stage]
    assert len(stage_ids) == 1
    record = next(r for r in motion['native_backend_evidence'] if r.get('stage_id') == stage_ids[0])
    request = record['submitted_request']
    process = request['process_policy']
    path = [np.asarray(p['q'], float) for p in record['points']]
    assert np.array_equal(path[0], request['q_start'])
    obstacles = tuple(box(b) for b in request['world'])
    assert len(scene.cartons) == 40 and len([b for b in obstacles if b.category == 'carton']) == 39
    target = next(b for b in scene.cartons if b.name == process['target_id'])
    assert json.loads(json.dumps(c.collision_policy.to_mapping())) == process['collision_policy']
    assert np.array_equal(c.flange_from_physical_contact, process['flange_from_physical_contact'])
    assert np.array_equal(c.flange_from_virtual_task_tcp, request['flange_from_task_tcp'])
    # Preserve the original Python attachment bytes rather than recomputing it
    # from a rounded native flange transform or an extraction endpoint.
    contact = motion['selected_trajectory_segment']['contact']
    rigid = RigidAttachment(np.asarray(contact['physical_contact_from_box']), target.half_extents, target.name)
    attachment = PhysicalContactAttachment(c.robot, rigid, c.flange_from_virtual_task_tcp, c.flange_from_physical_contact)
    expected = box(process['target'])
    assert np.max(np.abs(attachment.box_at(path[0]).world_from_local - expected.world_from_local)) < 1e-12
    c.stack_carton_names = set(process['stack_carton_ids'])
    c.robot_state_validator.stack_carton_names = set(c.stack_carton_names)
    c.robot_state_validator.contact_target_name = target.name
    c.robot_state_validator.commanded_cup_mask = tuple(process['commanded_active_mask'])
    tracker = None
    if process['initial_proximity'] is not None:
        tracker, failure = c._initial_proximity(target, obstacles, process['support_names'])
        assert failure is None and tracker.evidence() == process['initial_proximity']
        assert not tracker.fully_released  # This saved extraction starts at the stack.
    options = dict(attachment=attachment, support_names=tuple(process['support_names']),
                   initial_proximity=tracker, stage=stage)
    return c, path, obstacles, options, dict(stage_id=record['stage_id'],
        task_id=record['task_id'], parent_stage_id=record['parent_stage_id'],
        request_fingerprint=record['request_fingerprint'], path_sha256=record['path_sha256'],
        scene_fingerprint=scene.snapshot['scene_fingerprint'], validator_identity=c.validator_identity,
        tracker_initial=None if tracker is None else tracker.evidence(),
        tracker_initial_pose=None if tracker is None else tracker.last_box.world_from_local.tolist(),
        original_source_commit=json.loads((directory/'run.json').read_text())['source_commit'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--formal-directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('reference', 'optimized'), default='reference')
    parser.add_argument('--stages', nargs='+', choices=('transit', 'extraction'), default=['transit', 'extraction'])
    parser.add_argument('--edge-count', type=int, default=0, help='0 means complete; positive means prefix only')
    parser.add_argument('--profile', action='store_true', help='cProfile development fragments only, not timing comparison')
    parser.add_argument('--counterexample', action='store_true', help='Recheck the recorded insufficient-gap state, separately from accepted paths')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve prior benchmark evidence')
    os.environ['M710_STAGE_VALIDATION_MODE'] = args.mode
    args.output.parent.mkdir(parents=True, exist_ok=True)
    motion_path = args.formal_directory / 'plan/motion.json'
    motion = json.loads(motion_path.read_text())
    from unloading_sim.moveit2_native_cold import audit_native_motion_coverage
    segment = motion['selected_trajectory_segment']
    coverage = audit_native_motion_coverage(segment['path'], segment['native_backend']['stages'],
                                            task_id=segment['native_backend']['task_id'])
    assert coverage['passed'] and coverage['nonzero_edge_count'] == 436
    report = dict(schema='m710_same_input_stage_validation_v1', scope='OFFLINE_SAVED_MOTION_VALIDATOR_BENCHMARK_NOT_NATIVE_COLD',
        mode=args.mode, sampled_prefix_edges=args.edge_count, profile_enabled=args.profile,
        source_motion_sha256=sha(motion_path), stages=[], isaac='NOT_RUN', planning_requests=0,
        baseline_source_audit_recomputed=coverage,
        input_files={name: sha(args.formal_directory/name) for name in (
            'plan/motion.json', 'plan/actual_scene_snapshot.json', 'physics/initialized_actual_state.json',
            'bootstrap.json', 'run.json')},
        environment={name: os.environ.get(name) for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'M710_STAGE_VALIDATION_MODE')})
    for stage in args.stages:
        preparing = perf_counter()
        c, path, obstacles, options, identity = prepare(args.formal_directory, motion, stage)
        preparation = perf_counter() - preparing
        if args.edge_count:
            path = path[:args.edge_count + 1]
        profiler = cProfile.Profile() if args.profile else None
        if profiler: profiler.enable()
        began = perf_counter()
        failure = c._path_failure(path, obstacles, **options)
        wall = perf_counter() - began
        if profiler:
            profiler.disable()
            profiler.dump_stats(str(args.output.with_suffix('.' + stage + '.pstats')))
            with args.output.with_suffix('.' + stage + '.profile.txt').open('w') as stream:
                pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(45)
        row = dict(stage=stage, identity=identity, model_preparation_seconds=preparation,
            cache_state='COLD_FRESH_CHECKER_AND_TRACKER_MODEL_BUILT_BEFORE_TIMER', path_edges=len(path)-1,
            wall_seconds=wall, result='ACCEPT' if failure is None else 'REJECT_OR_INCOMPLETE', failure=failure,
            profile=getattr(c, 'last_stage_validation_profile', None),
            validation=c.last_motion_validation, context=dict(c._context_statistics()),
            connector=dict(c._statistics), kernel=dict(c.validation_kernel.statistics),
            state_validator=dict(c.robot_state_validator.performance_counters),
            mesh=dict(c.robot_state_validator.mesh_robot.performance_counters),
            tracker_final=None if options['initial_proximity'] is None else options['initial_proximity'].evidence())
        report['stages'].append(row)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(stage, row['result'], round(wall, 6), flush=True)
    if args.counterexample:
        c, _, obstacles, options, identity = prepare(args.formal_directory, motion, 'transit')
        recorded = next(r['failure'] for r in motion['native_backend_evidence']
            if r.get('status') == 'NATIVE_PATH_REJECTED' and (r.get('failure') or {}).get('reason') == 'CLEARANCE_INSUFFICIENT')
        failure = c._path_failure([np.asarray(recorded['q_rad'])], obstacles, **options)
        report['counterexample'] = dict(scope='OFFLINE_RECHECK_ONLY_NOT_A_PLANNING_INPUT', identity=identity,
            recorded_constraint=recorded, observed=failure, profile=c.last_stage_validation_profile)
        assert failure is not None and failure['classification'] == 'CLEARANCE_INSUFFICIENT'
        assert failure['pair'] == recorded['pair']
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print('recorded_insufficient_gap', 'REJECT', failure['surface_distance_m'], flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
