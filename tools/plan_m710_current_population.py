"""Fresh 40-box run using the last confirmed equipment and scoped grasp recipes."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from time import perf_counter
import traceback
import numpy as np

from plan_m710_mount_comparison import make_context, FLAGS
from plan_m710_top_layer_lifted import ROOT, save, source_identity, run_seed, FrozenTopLayer, BatchState, box_message
from plan_m710_all_remaining import archive_task
from unloading_sim.carton_population import FrozenCartonPopulation, rank_population, blocked_population
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.current_population_recipe import SCOPE, LAYOUT, recipe_permission
from l01_tool_chassis import frozen_l01_scope


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--worker-command',required=True)
    p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-commit',required=True)
    p.add_argument('--source-manifest',type=Path)
    p.add_argument('--development',action='store_true')
    p.add_argument('--check-only',action='store_true')
    p.add_argument('--assume-l05-removed-from',type=Path,
        help='Restore the accepted 24-box terminal scene, then explicitly assume l05_c03 removed.')
    args=p.parse_args();identity=source_identity(args)
    if args.source_manifest:
        manifest=json.loads(args.source_manifest.read_text())
        identity.update(source_state=manifest.get('source_state','COMMIT_SNAPSHOT'),
                        runtime_manifest_sha256=hashlib.sha256(args.source_manifest.read_bytes()).hexdigest(),
                        changed_source_files=manifest.get('changed_source_files',[]))
    args.output.mkdir(parents=True,exist_ok=False)
    original,built,audit=make_context('x_minus_300_y_minus_100',
        layout_path=ROOT/'configs/workcells'/LAYOUT,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    layout=original.policy.layout_validation.layout
    frozen=FrozenCartonPopulation.capture(original.cartons)
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),.6)
    assert len(state.cartons)==40 and not state.ledger
    assumption=None;predecessor=None;run_scope=SCOPE
    if args.assume_l05_removed_from:
        from plan_m710_l05_faces import restore_root
        from unloading_sim.assumed_removal import assume_l05_removed,SCOPE as ASSUMED_SCOPE
        predecessor=restore_root(args.assume_l05_removed_from,state,original,built,terminal=True)
        assumption=assume_l05_removed(state);run_scope=ASSUMED_SCOPE
        save(args.output/'predecessor.json',predecessor)
        save(args.output/'assumed-removal.json',assumption)
    historical_count=len(state.ledger);historical_ledger=deepcopy(state.ledger)
    scene=batch_scene(original,state,built.robot,layout)
    failure=built.validate_unloaded_state(state.q,scene.all_obstacles,stage='unloaded')
    permission=recipe_permission(frozen)
    save(args.output/'initial-state-check.json',dict(**identity,passed=failure is None,failure=failure,
        backend=type(built.robot).__name__,root=state.root(),layout_audit=audit,
        base_transform=built.robot.base_transform.tolist(),recipe_permission=permission))
    if failure is not None:raise ValueError('CURRENT_POPULATION_INITIAL_STATE_REJECTED')
    if args.check_only:return
    args.task_budget_s,args.stage_budget_s,args.ipc_timeout_s=300.,90.,100.
    args.direct_first,args.upper_arm_preference=True,True
    args.low_top_minimum_m=.25;args.receive_height_range=(.25,2.10)
    args.low_definition=permission['low_definition'];args.include_receive_before_approach=True
    args.current_population_recipe=permission
    initial=deepcopy(state.root());scope=[n for n in frozen.target_ids if n in {b.name for b in state.cartons}]
    eccentric_ids={n for n,r in FrozenTopLayer.capture(original.cartons).evidence()['roles'].items() if r['role']=='RIGHT_END'}
    protocol=dict(**identity,**FLAGS,scope=run_scope,seeds=[71070],original_cartons=[box_message(b) for b in original.cartons],
        scoped_target_ids=scope,frozen=frozen.evidence(),initial_root=initial,configured_initial_q=state.q.tolist(),
        historical_completed_count=historical_count,integration_fixture=False,current_population_recipe=permission,
        active_layout_file='configs/workcells/'+LAYOUT,mounting_variant='x_minus_300_y_minus_100',
        worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),task_budget_s=300.,stage_budget_s=90.,
        low_top_minimum_m=.25,low_definition=args.low_definition,include_receive_before_approach=True,
        eccentric_authorized_ids=sorted(eccentric_ids),third_sync_disabled=True,no_historical_solution_hints=True,
        all_joint_rate_limits_retained=True,tool_connector_extension_m=.2,scissor_profile='low_profile_250_v1',
        collision_policy=built.collision_policy.to_mapping(),robot_mount_contact_exemption=True,
        robot_mount_allowed_pairs=[list(p) for p in sorted(built.collision_policy.robot_mount_pairs())],
        tool_chassis_permission=frozen_l01_scope(original.cartons))
    if assumption:
        protocol.update(assumed_removed_target_ids=list(state.assumed_removed),assumed_removal=assumption,
            predecessor_summary_sha256=predecessor['summary_sha256'],
            search_operation='RESTART_AFTER_EXPLICIT_ASSUMED_REMOVAL',historical_solver_state_loaded=False)
    save(args.output/'protocol.json',protocol)
    outcomes=[];attempted=set();started=perf_counter()
    def checkpoint(status):
        n=len(state.ledger)-historical_count
        row=dict(**identity,**FLAGS,status=status,seed=71070,initial=initial,terminal=state.root(),target_count=len(scope),
            completed_count=n,success_rate=n/len(scope),historical_completed_count=historical_count,historical_ledger=historical_ledger,ledger=state.ledger,
            outcomes=outcomes,attempted_target_ids=sorted(attempted),remaining_target_ids=[b.name for b in state.cartons],
            blocked=blocked_population(state.cartons,attempted),planning_wall_s=perf_counter()-started,
            native_task_wall_s=sum(o.get('planning_wall_s',0) for o in outcomes),
            release_count=n,nominal_first_contact_count=n,separation_count=n,ideal_drain_count=n)
        if assumption:row.update(assumed_removed_target_ids=list(state.assumed_removed),assumed_removed_count=1,
            total_planned_completed_count=len(state.ledger),original_population_count=40)
        save(args.output/'summary.json',row);return row
    while state.cartons:
        scene=batch_scene(original,state,built.robot,layout)
        ranked=rank_population(frozen,state.cartons,state.accounted_ids(),attempted,state.q,
            built.robot,scene.receiver,longitudinal=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'))
        if not ranked:break
        selected=ranked[0];name=selected['target_id'];attempted.add(name)
        args.right_front_eccentric=name in eccentric_ids;args.junction_only=args.right_front_eccentric
        folder=args.output/f'task-{len(outcomes)+1:03d}';folder.mkdir()
        save(folder/'dispatch.json',dict(selection=selected,root=state.root(),right_front_eccentric=args.right_front_eccentric))
        checkpoint('RUNNING')
        print(json.dumps(dict(event='CARTON_TASK_START',target=name,remaining=len(state.cartons))),flush=True)
        try:
            result=run_seed(args,original,built,frozen,71070,folder,initial_state=state,target_ids=[name])
            for cycle in result['cycles']:state.commit(cycle)
            if state.root()!=result['terminal']:raise ValueError('PER_CARTON_TERMINAL_CHANGED')
            state.cycles=state.cycles[-1:]
        except Exception:
            (folder/'integration-exception.txt').write_text(traceback.format_exc())
            outcomes.append(dict(target_id=name,status='INTEGRATION_ERROR_RETAINED',folder=folder.name))
            checkpoint('INTEGRATION_ERROR_RETAINED');raise
        outcome=dict(target_id=name,status=result['status'],completed_count=result['completed_count'],
            planning_wall_s=result['planning_wall_s'],attempts=result['attempts'],root=result['initial'],terminal=result['terminal'])
        outcome['evidence']=archive_task(folder,args.output);outcomes.append(outcome)
        row=checkpoint('RUNNING')
        print(json.dumps(dict(event='CARTON_TASK_RECORDED',target=name,completed=row['completed_count'],
            remaining=len(state.cartons),attempted=len(attempted),task_s=result['planning_wall_s'])),flush=True)
    final=checkpoint('COMPLETE_PLANNING_ONLY' if not state.cartons else 'INCOMPLETE_RETAINED')
    if len(outcomes)+len(final['blocked'])!=len(scope):raise ValueError('POPULATION_ACCOUNTING_GAP')
    print(json.dumps(dict(status=final['status'],completed=final['completed_count'],targets=len(scope))),flush=True)


if __name__=='__main__':main()
