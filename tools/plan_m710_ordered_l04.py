"""Continue the accepted changed-mount l05 cycle, strictly c02 then c03."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
from plan_m710_mount_comparison import make_context, FLAGS
from plan_m710_top_layer_lifted import ROOT, save, source_identity, run_seed, BatchState, box_message
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import batch_scene, message_box
from unloading_sim.y_shift_retry import LAYOUT
from verify_m710_top_layer_batch import verify

SCOPE='ORDERED_L04_C02_THEN_C03_AFTER_ACCEPTED_L05'
ORDER=['carton_l04_c02','carton_l04_c03']


def restore_accepted(run, frozen, original, built):
    protocol=json.loads((run/'protocol.json').read_text())
    result=json.loads((run/'seed-71070/result.json').read_text())
    checked=verify(run)
    assert checked['runs'][0]['completed']==1
    assert protocol['active_layout_file']=='configs/workcells/'+LAYOUT
    assert protocol['scoped_target_ids']==['carton_l05_c03']
    assert protocol['collision_policy']==json.loads(json.dumps(built.collision_policy.to_mapping()))
    assert protocol['original_cartons']==[box_message(b) for b in original.cartons]
    assert protocol['frozen']==frozen.evidence()
    assert result['completed_count']==1 and len(result['cycles'])==1
    assert result['cycles'][0]['target_id']=='carton_l05_c03'
    root=result['initial']
    state=BatchState(frozen,tuple(message_box(b) for b in root['world_cartons']),
        np.asarray(root['q_rad']),root['height_m'],deepcopy(result['historical_ledger']))
    assert state.root()==root and not root.get('assumed_removed')
    state.commit(result['cycles'][0])
    assert state.root()==result['terminal'] and len(state.cartons)==15 and len(state.ledger)==25
    receipt=dict(source_run=str(run),sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest()
        for name in ('protocol.json','seed-71070/result.json','verification.json')},
        terminal=state.root(),ledger=state.ledger,checked=checked,
        search_operation='NEW_NATIVE_TASK_AT_ACCEPTED_TERMINAL',historical_solver_state_loaded=False,
        historical_trajectories_used_as_solver_input=False,physical_execution='NOT_RUN')
    return state,protocol,receipt


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--predecessor',type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-commit',required=True);p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--development',action='store_true');p.add_argument('--check-only',action='store_true')
    args=p.parse_args();identity=source_identity(args)
    identity.update(source_state='BASE_COMMIT_WITH_RECORDED_PATCH',
        source_manifest_sha256=hashlib.sha256(args.source_manifest.read_bytes()).hexdigest())
    args.output.mkdir(parents=True,exist_ok=False)
    original,built,audit=make_context('ordered_l04',layout_path=ROOT/'configs/workcells'/LAYOUT,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    frozen=FrozenCartonPopulation.capture(original.cartons)
    state,predecessor_protocol,receipt=restore_accepted(args.predecessor,frozen,original,built)
    initial=deepcopy(state.root());save(args.output/'predecessor.json',receipt)
    scene=batch_scene(original,state,built.robot,original.policy.layout_validation.layout)
    failure=built.validate_unloaded_state(state.q,scene.all_obstacles,stage='unloaded')
    save(args.output/'initial-state-check.json',dict(passed=failure is None,failure=failure,
        root=state.root(),base_transform=built.robot.base_transform.tolist(),layout_audit=audit))
    if failure is not None:raise ValueError('ORDERED_L04_INITIAL_STATE_REJECTED')
    if args.check_only:return
    args.task_budget_s,args.stage_budget_s,args.ipc_timeout_s=300.,90.,100.
    args.direct_first,args.upper_arm_preference=True,True
    args.low_top_minimum_m=.25;args.receive_height_range=(.25,2.10)
    args.low_definition='FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'
    args.include_receive_before_approach=True;args.right_front_eccentric=False;args.junction_only=False
    outcomes=[];unsearched=[]
    for index,target in enumerate(ORDER):
        folder=args.output/target;folder.mkdir();(folder/'seed-71070').mkdir()
        protocol=dict(**identity,**FLAGS,scope=SCOPE,seeds=[71070],scoped_target_ids=[target],
            ordered_target_ids=ORDER,sequence_index=index,accepted_predecessor=receipt,
            frozen=frozen.evidence(),original_cartons=[box_message(b) for b in original.cartons],
            configured_initial_q=state.q.tolist(),initial_root=state.root(),right_front_eccentric=False,
            active_layout_file='configs/workcells/'+LAYOUT,collision_policy=built.collision_policy.to_mapping(),
            third_sync_disabled=True,all_joint_rate_limits_retained=True,task_budget_s=300.,stage_budget_s=90.,
            authorized_mount_change=predecessor_protocol['authorized_mount_change'],
            historical_completed_count=len(state.ledger),worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
            no_historical_solution_hints=True)
        save(folder/'protocol.json',protocol)
        print(json.dumps(dict(event='ORDERED_TARGET_START',target=target,initial=state.root())),flush=True)
        result=run_seed(args,original,built,frozen,71070,folder/'seed-71070',initial_state=deepcopy(state),target_ids=[target])
        for cycle in result['cycles']:state.commit(cycle)
        assert state.root()==result['terminal']
        report=verify(folder);save(folder/'verification.json',report)
        outcomes.append(dict(target_id=target,folder=target,completed_count=result['completed_count'],
            status=result['status'],planning_wall_s=result['planning_wall_s'],attempts=result['attempts']))
        if not result['completed_count']:
            unsearched=[dict(target_id=n,reason='ORDERED_PREDECESSOR_NOT_COMPLETED') for n in ORDER[index+1:]]
        save(args.output/'summary.json',dict(**identity,**FLAGS,scope=SCOPE,ordered_target_ids=ORDER,
            initial=initial,terminal=state.root(),outcomes=outcomes,unsearched=unsearched,
            completed_count=len(state.ledger)-25,historical_completed_count=25,independent_trials=False))
        print(json.dumps(dict(event='ORDERED_TARGET_RECORDED',target=target,completed=result['completed_count'])),flush=True)
        if not result['completed_count']:break


if __name__=='__main__':main()
