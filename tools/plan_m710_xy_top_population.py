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
    'x_minus_500_y_minus_100': 'm710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100.yaml',
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
    p.add_argument('--continue-min250-from',type=Path,
        help='Verified x500 terminal, 250 mm requested minimum and strict box-top suction rule')
    p.add_argument('--continue-min250-repair-from',type=Path,
        help='Retain three checked completions before the receive-world synchronization integration error')
    p.add_argument('--continue-layer-top-from',type=Path,
        help='Restore the checked min250 terminal; allow top suction from the selected frozen layer down')
    p.add_argument('--continue-lowprofile-from',type=Path,
        help='Restore checked 28-box terminal; 250 mm scissor, explicit robot/mount contacts, strict top threshold')
    p.add_argument('--top-layer-anchor',default='carton_l02_c01')
    p.add_argument('--mounting-variant',choices=tuple(POPULATION_LAYOUTS),default='xy_minus_100')
    p.add_argument('--integration-top-fixture',action='store_true',
        help='Separate synthetic exposed low-row fixture, never part of formal 40-box counts')
    p.add_argument('--l05-limit-restart','--l05-j4-velocity-restart',dest='l05_j4_velocity_restart',type=Path,
        help='Separate user-approved diagnostic, restoring the accepted prefix before failed l05_c03')
    p.add_argument('--l05-limit-worker-command','--l05-j4-velocity-worker-command',dest='l05_j4_velocity_worker_command')
    p.add_argument('--l05-limit-library','--l05-j4-velocity-library',dest='l05_j4_velocity_library',type=Path)
    p.add_argument('--l05-motion-limit-policy',choices=('j4_velocity_only','all_joint_rates'),default='j4_velocity_only')
    args=p.parse_args();identity=source_identity(args)
    if args.continue_lowprofile_from and (args.continue_min250_from or args.continue_min250_repair_from
            or args.continue_layer_top_from or args.mounting_variant!='x_minus_500_y_minus_100'
            or args.integration_top_fixture or args.l05_j4_velocity_restart):
        raise ValueError('LOWPROFILE_CONTINUATION_SCOPE_CHANGED')
    if args.continue_layer_top_from and (args.continue_min250_from or args.continue_min250_repair_from
            or args.mounting_variant!='x_minus_500_y_minus_100' or args.integration_top_fixture
            or args.l05_j4_velocity_restart):
        raise ValueError('LAYER_TOP_CONTINUATION_SCOPE_CHANGED')
    if args.continue_min250_repair_from and not args.continue_min250_from:
        raise ValueError('MIN250_REPAIR_REQUIRES_ORIGINAL_PREDECESSOR')
    if args.continue_min250_from and (args.mounting_variant!='x_minus_500_y_minus_100'
            or args.integration_top_fixture or args.l05_j4_velocity_restart):
        raise ValueError('MIN250_CONTINUATION_SCOPE_CHANGED')
    args.output.mkdir(parents=True,exist_ok=False)
    layout_path=Path(__file__).resolve().parents[1]/'configs/workcells'/POPULATION_LAYOUTS[args.mounting_variant]
    if args.continue_min250_from or args.continue_layer_top_from:layout_path=layout_path.with_stem(layout_path.stem+'_min250')
    if args.continue_lowprofile_from:layout_path=layout_path.with_stem(layout_path.stem+'_min250_lowprofile')
    original,built,audit=make_context(args.mounting_variant,layout_path=layout_path,
        robot_mount_contact_exemption=bool(args.continue_lowprofile_from))
    layout=original.policy.layout_validation.layout
    frozen=FrozenCartonPopulation.capture(original.cartons)
    nominal=float(layout.data['assembly']['world_xyz_m'][2]+layout.data['conveyors']['surface_z_a_m'])
    minimum=float(layout.data['conveyors'].get('coupling',{}).get('minimum_surface_height_world_m',nominal))
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),nominal)
    from unloading_sim.low_top_receiver import BOX_TOP_LOW_DEFINITION,LEGACY_LOW_DEFINITION
    args.low_definition=BOX_TOP_LOW_DEFINITION if args.continue_min250_from or args.continue_layer_top_from or args.continue_lowprofile_from else LEGACY_LOW_DEFINITION
    args.include_receive_before_approach=bool(args.continue_min250_from or args.continue_layer_top_from or args.continue_lowprofile_from)
    args.layer_top_permission=None
    if args.continue_lowprofile_from:
        from layer_top_continuation import restore_layer_top_terminal
        lowprofile_predecessor=restore_layer_top_terminal(args.continue_lowprofile_from,state)
        save(args.output/'predecessor.json',lowprofile_predecessor)
    if args.continue_layer_top_from:
        from layer_top_continuation import restore_layer_top_terminal,SCOPE as LAYER_TOP_SCOPE
        from unloading_sim.low_top_receiver import selected_layer_top_permission
        layer_predecessor=restore_layer_top_terminal(args.continue_layer_top_from,state)
        save(args.output/'predecessor.json',layer_predecessor)
        args.layer_top_permission=selected_layer_top_permission(frozen,args.top_layer_anchor)
    if args.continue_min250_from:
        from min250_continuation import restore_population_terminal,SCOPE as MIN250_SCOPE
        predecessor=restore_population_terminal(args.continue_min250_from,state,args.mounting_variant)
        save(args.output/'predecessor.json',predecessor)
        if args.continue_min250_repair_from:
            from min250_continuation import restore_before_world_sync_repair
            repair_predecessor=restore_before_world_sync_repair(args.continue_min250_repair_from,state)
            save(args.output/'restart-predecessor.json',repair_predecessor)
    diagnostic=None
    normal_worker_command=args.worker_command
    if args.l05_j4_velocity_restart:
        from l05_velocity_diagnostic import restore_before_l05, TARGET, SCOPE, RATES_SCOPE
        if args.integration_top_fixture or not args.l05_j4_velocity_worker_command or not args.l05_j4_velocity_library:
            raise ValueError('L05_DIAGNOSTIC_CONFIGURATION_INCOMPLETE')
        predecessor=restore_before_l05(args.l05_j4_velocity_restart,state,args.mounting_variant)
        save(args.output/'predecessor.json',predecessor)
        all_rates=args.l05_motion_limit_policy=='all_joint_rates'
        diagnostic=dict(target_id=TARGET,joint='ALL_SIX' if all_rates else 'J4',
            ignored_check='PILZ_ALL_JOINT_VELOCITY_ACCELERATION_DECELERATION' if all_rates else 'PILZ_JOINT_VELOCITY_ONLY',
            acceleration_deceleration_retained=not all_rates,other_joints_retained=not all_rates,
            ik_and_position_limits_retained=True,
            position_collision_and_process_checks_retained=True,physical_execution='NOT_RUN',
            library_sha256=hashlib.sha256(args.l05_j4_velocity_library.read_bytes()).hexdigest(),
            library_path=str(args.l05_j4_velocity_library),
            worker_command=args.l05_j4_velocity_worker_command,
            baseline_summary_sha256=predecessor['summary_sha256'])
    elif args.l05_j4_velocity_worker_command or args.l05_j4_velocity_library or args.l05_motion_limit_policy!='j4_velocity_only':
        raise ValueError('L05_DIAGNOSTIC_RESTART_REQUIRED')
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
    protocol=dict(**identity,**FLAGS,seeds=[71070],scope=(RATES_SCOPE if all_rates else SCOPE) if diagnostic else
        MIN250_SCOPE if args.continue_min250_from else LAYER_TOP_SCOPE if args.continue_layer_top_from else
        'XY_TOP_LOWPROFILE250_CONTINUATION' if args.continue_lowprofile_from else 'XY_TOP_NEW_WORLD_40',
        integration_fixture=args.integration_top_fixture,original_cartons=[box_message(b) for b in original.cartons],
        scoped_target_ids=scope,frozen=frozen.evidence(),initial_root=initial,
        configured_initial_q=state.q.tolist(),historical_completed_count=len(history),
        active_layout_file=str(layout.config_path.relative_to(Path(__file__).resolve().parents[1])).replace('\\','/'),
        mounting_variant=args.mounting_variant,worker_sha256=worker_sha,task_budget_s=300.,stage_budget_s=90.,
        low_top_minimum_m=minimum,low_definition=args.low_definition,
        low_receiver_rule='LOW_TOP_ONLY_FIXED_AT_MINIMUM_AFTER_CHECKED_UNLOADED_LOWERING',
        upper_receiver_rule='EXISTING_FRONT_SIDE_HMIN_HMAX_SYNCHRONIZATION',
        eccentric_authorized_ids=sorted(eccentric_ids),third_sync_disabled=True,
        no_historical_solution_hints=True,attempted_target_restarts=False)
    if diagnostic:protocol.update(planning_diagnostic=diagnostic,search_operation='NEW_NATIVE_TASK_FROM_VERIFIED_PREFIX',tree_retained=False)
    if args.continue_lowprofile_from:
        protocol.update(search_operation=lowprofile_predecessor['search_operation'],tree_retained=False,
            all_joint_rate_limits_retained=True,include_receive_before_approach=True,
            scissor_profile='low_profile_250_v1',robot_mount_contact_exemption=True,
            collision_policy=built.collision_policy.to_mapping(),
            robot_mount_allowed_pairs=[list(p) for p in sorted(built.collision_policy.robot_mount_pairs())],
            top_permission='FROZEN_BOX_TOP_STRICTLY_BELOW_250MM_ONLY',top_route=[[0.,0.,1.],[-1.,0.,0.]])
    if args.continue_layer_top_from:
        protocol.update(layer_top_permission=args.layer_top_permission,
            top_receiver_rule='CURRENT_COMMITTED_HEIGHT_FIXED_DURING_TOP_CYCLE',
            low_receiver_rule='SELECTED_LAYER_TOP_PLUS_EXISTING_FRONT_SIDE_OPTIONS',
            top_route=[[0.,0.,1.],[-1.,0.,0.]],
            search_operation=layer_predecessor['search_operation'],tree_retained=False,
            all_joint_rate_limits_retained=True,include_receive_before_approach=True)
    if args.continue_min250_from:
        protocol.update(search_operation=predecessor['search_operation'],tree_retained=False,
            predecessor_summary_sha256=predecessor['summary_sha256'],
            include_receive_before_approach=True,
            requested_minimum_geometry_unqualified=True,all_joint_rate_limits_retained=True)
        if args.continue_min250_repair_from:
            protocol.update(integration_repair_restart=True,this_round_predecessor_completed_count=3,
                search_operation=repair_predecessor['search_operation'],attempted_target_restarts=True)
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
        if diagnostic:row['planning_diagnostic']=diagnostic
        save(args.output/'summary.json',row);return row
    while state.cartons:
        scene=batch_scene(original,state,built.robot,layout)
        ranked=rank_population(frozen,state.cartons,[e['target_id'] for e in state.ledger],attempted,state.q,
            built.robot,scene.receiver,longitudinal=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'))
        if not ranked:break
        selected=ranked[0];name=selected['target_id'];attempted.add(name)
        if args.continue_min250_from and not outcomes and name!='carton_l03_c02':
            raise ValueError('MIN250_CONTINUATION_FIRST_TARGET_CHANGED')
        if args.continue_layer_top_from and not outcomes and name!='carton_l02_c01':
            raise ValueError('LAYER_TOP_CONTINUATION_FIRST_TARGET_CHANGED')
        if args.continue_lowprofile_from and not outcomes and name!='carton_l02_c01':
            raise ValueError('LOWPROFILE_CONTINUATION_FIRST_TARGET_CHANGED')
        diagnostic_enabled=bool(diagnostic and name==diagnostic['target_id'])
        args.worker_command=args.l05_j4_velocity_worker_command if diagnostic_enabled else normal_worker_command
        if diagnostic and not outcomes and name!=diagnostic['target_id']:
            raise ValueError('L05_DIAGNOSTIC_FIRST_TARGET_CHANGED')
        args.right_front_eccentric=name in eccentric_ids;args.junction_only=args.right_front_eccentric
        folder=args.output/f'task-{len(outcomes)+1:03d}';folder.mkdir()
        save(folder/'dispatch.json',dict(selection=selected,root=state.root(),right_front_eccentric=args.right_front_eccentric,
            motion_limit_diagnostic_enabled=diagnostic_enabled))
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
        if diagnostic:outcome['motion_limit_diagnostic_enabled']=diagnostic_enabled
        outcome['evidence']=archive_task(folder,args.output);outcomes.append(outcome)
        current=checkpoint('RUNNING')
        print(json.dumps(dict(event='CARTON_TASK_RECORDED',target=name,completed=current['completed_count'],
            remaining=len(state.cartons),attempted=len(attempted),task_s=result['planning_wall_s'])),flush=True)
        if args.integration_top_fixture:break
    final=checkpoint('INTEGRATION_FIXTURE_FINISHED' if args.integration_top_fixture else
        ('COMPLETE_PLANNING_ONLY' if not state.cartons else 'INCOMPLETE_RETAINED'))
    if not args.integration_top_fixture and len(outcomes)+len(final['blocked'])!=len(scope):
        raise ValueError('POPULATION_ACCOUNTING_GAP')
    print(json.dumps(dict(status=final['status'],completed=final['completed_count'],targets=len(scope))),flush=True)


if __name__=='__main__':main()
