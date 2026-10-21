"""Re-run only l05_c03/l04_c02 from their preserved per-task initial worlds.

The existing complete-cycle entry generates all new IK, paths and attachments.
Old contact solutions are not imported. Each case is a separate new native Task.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--core-root', type=Path, required=True)
    p.add_argument('--previous-run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--wrapper-commit', required=True)
    args = p.parse_args()
    sys.path[:0] = [str(args.core_root / 'tools'), str(args.core_root / 'src')]
    from plan_m710_mount_comparison import make_context, make_state, FLAGS
    from plan_m710_top_layer_lifted import run_seed, save
    from unloading_sim.coupled_batch import batch_scene
    old = json.loads((args.previous_run / 'summary.json').read_text())
    manifest = json.loads((args.core_root / 'source-manifest.json').read_text())
    for name, sha in manifest['sha256'].items():
        assert hashlib.sha256((args.core_root / name).read_bytes()).hexdigest() == sha, name
    args.output.mkdir(parents=True, exist_ok=False)
    args.task_budget_s, args.stage_budget_s, args.ipc_timeout_s = 300., 90., 100.
    args.direct_first, args.upper_arm_preference = True, True
    args.receive_height_range, args.low_top_minimum_m = (.60, 2.10), .60
    args.right_front_eccentric = args.junction_only = False
    args.worker_command = '/bin/bash /worker.sh'
    worker_sha = hashlib.sha256(Path('/mount-build/m710_moveit_worker').read_bytes()).hexdigest()
    original, built, audit = make_context('xy_minus_100')
    identity = dict(core_source_commit=manifest['source_commit'], wrapper_commit=args.wrapper_commit,
                    wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    worker_sha256=worker_sha, seed=71070, task_budget_s=300., stage_budget_s=90.,
                    scope='TWO_INDEPENDENT_HISTORICAL_ROOT_RESTARTS_NOT_CONTINUOUS_BATCH', **FLAGS)
    save(args.output / 'protocol.json', identity)
    outcomes = []
    for target_id in ('carton_l05_c03', 'carton_l04_c02'):
        previous = next(o for o in old['outcomes'] if o['target_id'] == target_id)
        root = deepcopy(previous['root'])
        old_receipt = root['predecessor_final_state_receipt']
        root['predecessor_final_state_receipt'] = None
        state = make_state(dict(root=root, target_id=target_id, right_front_eccentric=False), original)
        folder = args.output / target_id
        folder.mkdir()
        scene = batch_scene(original, state, built.robot, original.policy.layout_validation.layout)
        failure = built.validate_unloaded_state(state.q, scene.all_obstacles, stage='unloaded')
        save(folder / 'input.json', dict(target_id=target_id, initial=root, original_root=previous['root'],
             old_receipt_not_injected=old_receipt, previous_archive=previous['evidence'],
             initial_collision_failure=failure, root_q_geometry_height_unchanged=True,
             old_contact_q_path_attachment_injected=False, new_native_root=True))
        assert failure is None, failure
        started = perf_counter()
        result = run_seed(args, original, built, state.frozen, 71070, folder,
                          initial_state=state, target_ids=[target_id])
        outcome = dict(target_id=target_id, complete=result['completed_count'] == 1,
                       status=result['status'], planning_wall_s=perf_counter()-started,
                       attempts=result['attempts'])
        outcomes.append(outcome)
        save(args.output / 'summary.json', dict(**identity, outcomes=outcomes))
        print(json.dumps(dict(target_id=target_id, status=outcome['status'], complete=outcome['complete'])), flush=True)


if __name__ == '__main__':
    main()
