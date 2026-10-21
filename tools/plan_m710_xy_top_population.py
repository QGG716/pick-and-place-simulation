"""One new 40-carton world at an explicit approved mount, with low top suction."""
import argparse
from copy import deepcopy
import hashlib
import json
from time import perf_counter
from pathlib import Path
import numpy as np

from plan_m710_mount_comparison import make_context, FLAGS
from plan_m710_top_layer_lifted import save, source_identity, run_seed, FrozenTopLayer, BatchState, box_message
from plan_m710_all_remaining import archive_task
from unloading_sim.carton_population import FrozenCartonPopulation, rank_population, blocked_population
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.lifted_reception import bounds

POPULATION_LAYOUTS = {
    'xy_minus_100': 'm710id70_unloading_layout_v1_mount_xy_minus_100.yaml',
    'x_minus_300_y_minus_100': 'm710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100.yaml',
    'x_minus_400_y_minus_100': 'm710id70_unloading_layout_v1_mount_x_minus_400_y_minus_100.yaml',
}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--worker-command',required=True)
    p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-commit',required=True)
    p.add_argument('--source-manifest',type=Path)
    p.add_argument('--development',action='store_true')
    p.add_argument('--check-only',action='store_true')
    p.add_argument('--mounting-variant',choices=tuple(POPULATION_LAYOUTS),default='xy_minus_100')
    p.add_argument('--integration-top-fixture',action='store_true',
        help='Separate synthetic exposed low-row fixture, never part of formal 40-box counts')
    args=p.parse_args();identity=source_identity(args)
    args.output.mkdir(parents=True,exist_ok=False)
    layout_path=Path(__file__).resolve().parents[1]/'configs/workcells'/POPULATION_LAYOUTS[args.mounting_variant]
    original,built,audit=make_context(args.mounting_variant,layout_path=layout_path)
    layout=original.policy.layout_validation.layout
    frozen=FrozenCartonPopulation.capture(original.cartons)
    minimum=float(layout.data['assembly']['world_xyz_m'][2]+layout.data['conveyors']['surface_z_a_m'])
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),minimum)
    if args.integration_top_fixture:
        # Valid exposed low-row geometry for production-interface regression.
        # No removed fixture carton is counted as a successful task.
        low_ids={n for n,h in frozen.evidence()['heights'].items() if h['Hmin_world_m']<minimum-1e-9}
        state.cartons=tuple(b for b in state.cartons if b.name in low_ids)
        state.ledger=[dict(target_id=b.name,source='SYNTHETIC_INTEGRATION_FIXTURE_REMOVAL_NOT_COMPLETION')
                      for b in original.cartons if b.name not in low_ids]
    scene=batch_scene(original,state,built.robot,layout)
    failure=built.validate_unloaded_state(state.q,scene.all_obstacles,stage='unloaded')
    save(args.output/'initial-state-check.json',dict(**identity,backend=type(built.robot).__name__,
        passed=failure is None,failure=failure,root=state.root(),layout_audit=audit,
        base_transform=built.robot.base_transform.tolist(),fixture=args.integration_top_fixture))
    if failure is not None:raise ValueError('NEW_WORLD_INITIAL_STATE_REJECTED')
    if args.check_only:return
    args.task_budget_s,args.stage_budget_s,args.ipc_timeout_s=300.,90.,100.
    args.direct_first,args.upper_arm_preference=True,True
    args.receive_height_range=(minimum,2.10);args.low_top_minimum_m=minimum
    initial=state.root();history=deepcopy(state.ledger);scope=[b.name for b in state.cartons]
    top=FrozenTopLayer.capture(original.cartons)
    eccentric_ids={n for n,r in top.evidence()['roles'].items() if r['role']=='RIGHT_END'}
    worker_sha=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest()
    protocol=dict(**identity,**FLAGS,seeds=[71070],scope='XY_TOP_NEW_WORLD_40',
        integration_fixture=args.integration_top_fixture,original_cartons=[box_message(b) for b in original.cartons],
        scoped_target_ids=scope,frozen=frozen.evidence(),initial_root=initial,
        configured_initial_q=state.q.tolist(),historical_completed_count=len(history),
        active_layout_file=str(layout.config_path.relative_to(Path(__file__).resolve().parents[1])).replace('\\','/'),
        mounting_variant=args.mounting_variant,worker_sha256=worker_sha,task_budget_s=300.,stage_budget_s=90.,
        low_top_minimum_m=minimum,low_definition='FROZEN_ORIGINAL_HMIN_BELOW_MINIMUM',
        low_receiver_rule='LOW_TOP_ONLY_FIXED_AT_MINIMUM_AFTER_CHECKED_UNLOADED_LOWERING',
        upper_receiver_rule='EXISTING_FRONT_SIDE_HMIN_HMAX_SYNCHRONIZATION',
        eccentric_authorized_ids=sorted(eccentric_ids),third_sync_disabled=True,
        no_historical_solution_hints=True,attempted_target_restarts=False)
    save(args.output/'protocol.json',protocol)
    outcomes=[];attempted=set();started=perf_counter()
    def checkpoint(status):
        n=len(state.ledger)-len(history)
        row=dict(**identity,**FLAGS,status=status,seed=71070,initial=initial,terminal=state.root(),
            target_count=len(scope),completed_count=n,success_rate=n/len(scope),
            integration_fixture=args.integration_top_fixture,historical_completed_count=len(history),
            historical_ledger=history,ledger=state.ledger,outcomes=outcomes,
            blocked=blocked_population(state.cartons,attempted),attempted_target_ids=sorted(attempted),
            remaining_target_ids=[b.name for b in state.cartons],planning_wall_s=perf_counter()-started,
            native_task_wall_s=sum(o.get('planning_wall_s',0.) for o in outcomes),
            release_count=n,nominal_first_contact_count=n,separation_count=n,ideal_drain_count=n)
        save(args.output/'summary.json',row);return row
    while state.cartons:
        scene=batch_scene(original,state,built.robot,layout)
        ranked=rank_population(frozen,state.cartons,[e['target_id'] for e in state.ledger],attempted,state.q,
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
            import traceback
            (folder/'integration-exception.txt').write_text(traceback.format_exc(),encoding='utf-8')
            outcomes.append(dict(target_id=name,status='INTEGRATION_ERROR_RETAINED',folder=folder.name))
            checkpoint('INTEGRATION_ERROR_RETAINED');raise
        outcome=dict(target_id=name,status=result['status'],completed_count=result['completed_count'],
            planning_wall_s=result['planning_wall_s'],attempts=result['attempts'],root=result['initial'],terminal=result['terminal'])
        outcome['evidence']=archive_task(folder,args.output);outcomes.append(outcome)
        current=checkpoint('RUNNING')
        print(json.dumps(dict(event='CARTON_TASK_RECORDED',target=name,completed=current['completed_count'],
            remaining=len(state.cartons),attempted=len(attempted),task_s=result['planning_wall_s'])),flush=True)
        if args.integration_top_fixture:break
    final=checkpoint('INTEGRATION_FIXTURE_FINISHED' if args.integration_top_fixture else
        ('COMPLETE_PLANNING_ONLY' if not state.cartons else 'INCOMPLETE_RETAINED'))
    if not args.integration_top_fixture and len(outcomes)+len(final['blocked'])!=40:
        raise ValueError('POPULATION_ACCOUNTING_GAP')
    print(json.dumps(dict(status=final['status'],completed=final['completed_count'],targets=len(scope))),flush=True)


if __name__=='__main__':main()
