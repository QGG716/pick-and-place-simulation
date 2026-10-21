"""Two independent complete-cycle trials after the authorized extra -X 200 mm.

Keep each original task's initial q, carton world, receiver height and ledger.
No previous candidate, trajectory, attachment or native parent is imported.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

from plan_m710_mount_comparison import make_context, make_state, FLAGS
from plan_m710_top_layer_lifted import ROOT, run_seed, save, source_identity, box_message
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.workcell_layout import load_workcell_layout

LAYOUT = 'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100.yaml'
TARGETS = ('carton_l05_c03', 'carton_l04_c02')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--previous-run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--source-commit', required=True)
    p.add_argument('--source-manifest', type=Path, required=True)
    p.add_argument('--worker-command', default='/bin/bash /worker.sh')
    p.add_argument('--worker-binary', type=Path, default=Path('/mount-build/m710_moveit_worker'))
    args = p.parse_args()
    args.development = False
    identity = source_identity(args)
    old_path = args.previous_run/'summary.json'
    old = json.loads(old_path.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    args.task_budget_s, args.stage_budget_s, args.ipc_timeout_s = 300., 90., 100.
    args.direct_first, args.upper_arm_preference = True, True
    args.receive_height_range, args.low_top_minimum_m = (.60, 2.10), .60
    args.right_front_eccentric = args.junction_only = False
    original, built, audit = make_context('x_minus_300_y_minus_100', layout_path=ROOT/LAYOUT)
    assert type(built.robot).__name__ == 'PinocchioHppFclBackend'
    layout = original.policy.layout_validation.layout
    np.testing.assert_allclose(built.robot.base_transform[:3, 3], [-1.625, .250, .600], atol=1e-12)
    previous_layout = load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1_mount_xy_minus_100.yaml')
    for key in ('assembly', 'chassis', 'conveyors', 'tool', 'trailer', 'carton_stack'):
        assert layout.data[key] == previous_layout.data[key], key
    worker_sha = hashlib.sha256(args.worker_binary.read_bytes()).hexdigest()
    protocol = dict(**identity, **FLAGS, active_layout_file=LAYOUT, targets=TARGETS,
        additional_mount_translation_world_m=[-.2, 0., 0.], base_world_m=[-1.625, .250, .600],
        worker_sha256=worker_sha, seed=71070, task_budget_s=300., stage_budget_s=90.,
        scope='TWO_INDEPENDENT_EXTRA_X_200_TRIALS_NOT_CONTINUOUS_BATCH',
        previous_summary_sha256=hashlib.sha256(old_path.read_bytes()).hexdigest(),
        command=[sys.executable, *sys.argv], layout_audit=audit)
    save(args.output/'protocol.json', protocol)
    checks, cases = [], []
    for target in TARGETS:
        previous = next(o for o in old['outcomes'] if o['target_id'] == target)
        root = deepcopy(previous['root'])
        receipt = root['predecessor_final_state_receipt']
        root['predecessor_final_state_receipt'] = None
        state = make_state(dict(root=root, target_id=target, right_front_eccentric=False), original)
        scene = batch_scene(original, state, built.robot, layout)
        failure = built.validate_unloaded_state(state.q, scene.all_obstacles, stage='unloaded')
        check = dict(target_id=target, initial=root, original_root=previous['root'],
            old_receipt_not_injected=receipt, previous_archive=previous['evidence'],
            initial_collision_failure=failure, passed=failure is None,
            backend=type(built.robot).__name__, root_q_geometry_height_unchanged=True,
            old_contact_q_path_attachment_injected=False, new_native_root=True,
            scene_fingerprint=scene.snapshot['scene_fingerprint'])
        checks.append(check); cases.append((target, state, check))
    save(args.output/'initial-state-checks.json', dict(checks=checks, all_passed=all(c['passed'] for c in checks)))
    if not all(c['passed'] for c in checks):
        save(args.output/'summary.json', dict(**protocol, status='INITIAL_STATE_INVALID_NO_FORMAL_SEARCH', outcomes=[]))
        raise ValueError('SHIFTED_MOUNT_INITIAL_STATE_INVALID')
    outcomes = []
    began = perf_counter()
    for target, state, check in cases:
        run = args.output/target; folder = run/'seed-71070'
        folder.mkdir(parents=True)
        save(run/'input.json', check)
        # Existing lineage verifier and replay consume this established format.
        save(run/'protocol.json', dict(**identity, **FLAGS, seeds=[71070],
            scope='PAIRED_MOUNT_INDEPENDENT_SINGLE_CARTON', variant='x_minus_300_y_minus_100',
            active_layout_file=LAYOUT, frozen=state.frozen.evidence(), scoped_target_ids=[target],
            original_cartons=[box_message(b) for b in original.cartons],
            configured_initial_q=state.q.tolist(), initial_root=state.root(),
            historical_completed_count=len(state.ledger), right_front_eccentric=False,
            third_sync_disabled=True, worker_sha256=worker_sha, task_budget_s=300., stage_budget_s=90.,
            upper_arm_preference='BOUNDED_SOFT_FK_WITH_GENERAL_SEEDS'))
        print(json.dumps(dict(event='RETREAT_TRIAL_START', target=target)), flush=True)
        started = perf_counter()
        try:
            result = run_seed(args, original, built, state.frozen, 71070, folder,
                              initial_state=state, target_ids=[target])
        except Exception:
            import traceback
            (run/'integration-exception.txt').write_text(traceback.format_exc(), encoding='utf-8')
            save(args.output/'summary.json', dict(**protocol, status='INTEGRATION_ERROR_RETAINED',
                failed_target=target, outcomes=outcomes, experiment_wall_s=perf_counter()-began))
            raise
        outcomes.append(dict(target_id=target, complete=result['completed_count'] == 1,
            status=result['status'], planning_wall_s=perf_counter()-started,
            initial=result['initial'], terminal=result['terminal'], attempts=result['attempts']))
        save(args.output/'summary.json', dict(**protocol, status='RUNNING' if len(outcomes)<2 else 'TRIALS_FINISHED',
            outcomes=outcomes, experiment_wall_s=perf_counter()-began))
        print(json.dumps(dict(event='RETREAT_TRIAL_RECORDED', target=target,
            complete=outcomes[-1]['complete'], wall_s=outcomes[-1]['planning_wall_s'])), flush=True)


if __name__ == '__main__':
    main()
