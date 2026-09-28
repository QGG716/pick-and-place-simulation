"""Focused original-scene approach probe; no full-cycle search or physics.

Freeze stops the ordinary entry at its first free-connection call. Rebuild uses
the recorded actual scene and exact contact candidate, never an archived path.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from unloading_sim.layout_single_carton import (load_layout_motion_policy, build_verified_motion_input,
    _build_automatic_trajectory_connector, motion_implementation_identity)
from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState
from unloading_sim.planning_profile import DEFAULT_MOTION, DEFAULT_EXECUTION


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, default=lambda v: v.tolist()
        if isinstance(v, np.ndarray) else str(v)), encoding='utf-8')


def freeze(args):
    from tools.run_m710_contact_unloading import main as ordinary_entry
    class Captured(BaseException): pass
    data = dict(source=motion_implementation_identity(ROOT), actual_state=str(args.actual_state.resolve()),
        actual_state_sha256=hashlib.sha256(args.actual_state.read_bytes()).hexdigest(),
        scope='EXACT_FIRST_APPROACH_RECONSTRUCTED_BY_ORDINARY_ENTRY', historical_path_supplied=False)
    plan = LayoutTrajectoryConnector.plan
    approach = LayoutTrajectoryConnector._approach
    connect = LayoutTrajectoryConnector._connect_pose
    def plan_capture(self, **kwargs):
        data.update(target=kwargs['target'].name, face=kwargs['face'],
            grasp_candidates=kwargs['grasp_candidates'], plan_seed=kwargs['seed'])
        return plan(self, **kwargs)
    def approach_capture(self, start, grasp_q, requested, obstacles, target, *, seed):
        data.update(start_q=start, grasp_q=grasp_q, requested_virtual_contact=requested,
            approach_seed=seed, branch_identity=self._candidate_identity,
            commanded_mask=self.robot_state_validator.commanded_cup_mask,
            contact_target=self.robot_state_validator.contact_target_name,
            stack_carton_names=sorted(self.stack_carton_names or ()),
            budget=asdict(self.budget), ik=dict(self.ik), joint_limits=self.robot.joint_limits,
            obstacles=[dict(name=b.name, category=b.category, pose=b.world_from_local,
                            half_extents=b.half_extents) for b in obstacles])
        return approach(self, start, grasp_q, requested, obstacles, target, seed=seed)
    def connection_capture(self, pose, seeds, start, obstacles, **kwargs):
        validator = self._motion_validator(obstacles, stage='pregrasp')
        data.update(first_gate_virtual=pose, first_gate_seeds=seeds,
            first_connection_options=kwargs, validation_context=validator.context.context_id,
            validation_binding=json.loads(validator.context.binding_json))
        write(args.output, data)
        raise Captured()
    LayoutTrajectoryConnector.plan = plan_capture
    LayoutTrajectoryConnector._approach = approach_capture
    LayoutTrajectoryConnector._connect_pose = connection_capture
    try:
        ordinary_entry(['--config', DEFAULT_MOTION, '--execution-config', DEFAULT_EXECUTION,
            '--actual-state', str(args.actual_state), '--target-id', args.target,
            '--output', str(args.output.parent/'freeze_entry')])
    except Captured:
        print('CAPTURED', args.output, flush=True)
    finally:
        LayoutTrajectoryConnector.plan = plan
        LayoutTrajectoryConnector._approach = approach
        LayoutTrajectoryConnector._connect_pose = connect


def rebuild(case):
    state_path = Path(case['actual_state'])
    if hashlib.sha256(state_path.read_bytes()).hexdigest() != case['actual_state_sha256']:
        raise ValueError('frozen actual-state identity changed')
    policy = load_layout_motion_policy(DEFAULT_MOTION)
    scene = build_verified_motion_input(policy)
    rows = RowUnloadingState(); rows.rank(scene.cartons, support_graph=scene.support_graph)
    scene = apply_actual_motion_state(scene, json.loads(state_path.read_text()), row_state=rows)
    c = _build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot()).connector
    if c is None: raise RuntimeError('official backend unavailable')
    c.start_planning_request()
    c.stack_carton_names = set(case['stack_carton_names'])
    target = next(b for b in scene.cartons if b.name == case['target'])
    c._contact_selection(np.asarray(case['grasp_q']), target, case['face'], scene.policy.data['suction'])
    if list(c.robot_state_validator.commanded_cup_mask) != case['commanded_mask']:
        raise ValueError('contact mask differs from frozen candidate')
    actual = [dict(name=b.name, category=b.category, pose=b.world_from_local.tolist(),
                   half_extents=b.half_extents.tolist()) for b in scene.all_obstacles]
    if actual != case['obstacles']: raise ValueError('frozen geometry differs')
    c._candidate_identity = case['branch_identity']
    validator = c._motion_validator(scene.all_obstacles, stage='pregrasp')
    if json.loads(validator.context.binding_json) != case['validation_binding']:
        raise ValueError('rebuilt validation binding differs')
    if asdict(c.budget) != case['budget'] or dict(c.ik) != case['ik']:
        raise ValueError('frozen solver configuration differs')
    return c, scene, target


def main(args):
    if args.mode == 'freeze': return freeze(args)
    if args.output.exists(): raise FileExistsError('use a fresh evidence path')
    source = motion_implementation_identity(ROOT)
    probe_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    case_hash = hashlib.sha256(args.case.read_bytes()).hexdigest()
    case = json.loads(args.case.read_text()); c, scene, target = rebuild(case)
    probes = []
    if args.mode == 'direct':
        # First recorded gate only; real IK and production direct validation.
        # This diagnostic deliberately does not attempt RRT or claim no route.
        def direct_gate(pose, seeds, start, obstacles, **kwargs):
            import cProfile
            import pstats
            checks = []
            stream = c._ik_stream(pose, seeds, obstacles, seed=kwargs['ik_seed'],
                                  stage='pregrasp', endpoint_checks=checks,
                                  attachment=None, support_names=(), target_contact=None)
            attempts = []
            for index, candidate in zip(range(c.budget.stage_connection_attempts), stream):
                before = perf_counter(); profile = cProfile.Profile()
                profile.enable()
                path, failure, evidence = c._transit(start, candidate.q, obstacles,
                    seed=kwargs['connection_seed']+index*1009, stage='pregrasp',
                    purpose=kwargs['purpose'], allow_rrt=False,
                    iteration_budget=c.budget.stage_connection_iterations)
                profile.disable()
                timing = sorted((dict(file=k[0], line=k[1], function=k[2], calls=v[1],
                    exclusive_s=v[2], inclusive_s=v[3]) for k,v in pstats.Stats(profile).stats.items()),
                    key=lambda x:x['exclusive_s'], reverse=True)[:35]
                item = dict(gate=pose, index=index, q=candidate.q,
                    delta=candidate.q-start, norm=float(np.linalg.norm(candidate.q-start)),
                    seconds=perf_counter()-before, failure=failure, evidence=evidence, profile=timing)
                attempts.append(item); probes.append(item)
                write(args.output.with_suffix('.partial.json'), probes)
                print('GATE', len(probes), item['norm'], item['seconds'],
                      None if failure is None else failure['reason'], flush=True)
                if failure is None:
                    return candidate.q, path, None, dict(attempts=attempts)
                from unloading_sim.stage_motion_policy import interrupts_generation
                if interrupts_generation(failure): break
            return None, [], failure if attempts else {'reason':'NO_VALID_IK'}, dict(attempts=attempts, endpoints=checks)
    started = perf_counter()
    progress_path = args.output.with_suffix('.progress.jsonl')
    with progress_path.open('x', encoding='utf-8') as progress:
        def record(event):
            progress.write(json.dumps(dict(elapsed_s=perf_counter()-started, **event), default=str)+'\n'); progress.flush()
        c.progress_callback = record
        record(dict(event='FIXED_INPUT', source=source, case_sha256=case_hash))
        a, b, failure, evidence = [], [], None, {}
        status = 'INDETERMINATE'; endpoint = None
        try:
            if args.mode == 'direct':
                _, a, failure, evidence = direct_gate(np.asarray(case['first_gate_virtual']),
                    [np.asarray(q) for q in case['first_gate_seeds']], np.asarray(case['start_q']),
                    scene.all_obstacles, **case['first_connection_options'])
            else:
                a, b, failure, evidence = c._approach(np.asarray(case['start_q']), np.asarray(case['grasp_q']),
                    np.asarray(case['requested_virtual_contact']), scene.all_obstacles, target, seed=case['approach_seed'])
                if failure is None:
                    from unloading_sim.ik import pose_error
                    _, position, angle = pose_error(c.robot.fk(b[-1]), np.asarray(case['requested_virtual_contact']))
                    selection = c._contact_selection(b[-1], target, case['face'], scene.policy.data['suction'])
                    failure = c._state_failure(b[-1], scene.all_obstacles, target_contact=target, stage='contact_endpoint')
                    endpoint = dict(actual_fk=c.robot.fk(b[-1]), position_error_m=position, orientation_error_rad=angle,
                        selected_target=target.name, commanded_mask=selection['commanded_active_mask'],
                        selection=selection, mask_matches_frozen=selection['commanded_active_mask']==case['commanded_mask'])
                    if (position > c.ik['position_tolerance_m'] or angle > c.ik['orientation_tolerance_rad']
                            or not endpoint['mask_matches_frozen']):
                        failure = dict(reason='FINAL_CONTACT_CONTRACT_MISMATCH')
            from unloading_sim.stage_motion_policy import failure_status
            status = failure_status(failure)
        except KeyboardInterrupt:
            status = 'CANCELLED'; failure = dict(reason='DIAGNOSTIC_MANUAL_CANCELLED')
        except Exception as exc:
            failure = dict(reason='DIAGNOSTIC_ERROR', exception=type(exc).__name__, detail=str(exc))
            raise
        finally:
            source_unchanged = source == motion_implementation_identity(ROOT)
            input_unchanged = case_hash == hashlib.sha256(args.case.read_bytes()).hexdigest()
            if not source_unchanged or not input_unchanged:
                status = 'INDETERMINATE'; failure = dict(reason='SOURCE_OR_INPUT_CHANGED')
            write(args.output, dict(source=source, source_unchanged=source_unchanged, input_unchanged=input_unchanged,
                probe_sha256=probe_hash, case_sha256=case_hash, status=status, scope=args.mode, endpoint=endpoint,
                elapsed_seconds=perf_counter()-started, prefix=a, contact=b, failure=failure,
                evidence=evidence, probes=probes, statistics=c._statistics,
                kernel_statistics=c.validation_kernel.statistics, context_statistics=c._context_statistics(),
                isaac_executed=False))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['freeze','approach','direct'], required=True)
    p.add_argument('--actual-state', type=Path)
    p.add_argument('--target', default='carton_l07_c04')
    p.add_argument('--case', type=Path)
    p.add_argument('--output', type=Path, required=True)
    main(p.parse_args())
