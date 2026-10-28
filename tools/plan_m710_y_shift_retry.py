"""Retry two saved carton scenes after the explicit 100 mm -Y base move."""
import argparse,hashlib,json
from copy import deepcopy
from pathlib import Path
import numpy as np
from plan_m710_mount_comparison import make_context,FLAGS
from plan_m710_top_layer_lifted import ROOT,save,source_identity,run_seed,BatchState,box_message
from plan_m710_l05_faces import restore_root
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.assumed_removal import assume_l05_removed
from unloading_sim.y_shift_retry import BASE_LAYOUT,LAYOUT,TARGETS,mount_evidence

CASES={TARGETS[0]:['side_negative_y_hmin','front'],TARGETS[1]:['front','top','side_positive_y','side_negative_y']}


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
    old,old_built,_=make_context('previous',layout_path=ROOT/'configs/workcells'/BASE_LAYOUT,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    frozen=FrozenCartonPopulation.capture(old.cartons)
    state=BatchState(frozen,tuple(old.cartons),np.asarray(old.snapshot['robot']['q_rad']),.6)
    predecessor=restore_root(args.predecessor,state,old,old_built,terminal=True)
    save(args.output/'predecessor.json',predecessor)
    original,built,audit=make_context('y_minus_100_retry',layout_path=ROOT/'configs/workcells'/LAYOUT,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    change=mount_evidence(old.policy.layout_validation.layout,original.policy.layout_validation.layout)
    assert old_built.collision_policy.to_mapping()==built.collision_policy.to_mapping()
    assert FrozenCartonPopulation.capture(original.cartons).evidence()==frozen.evidence()
    save(args.output/'mount-change.json',change)
    checks=[];states={};assumptions={}
    # The saved q is the first choice. If the changed mount makes it illegal,
    # only the existing configured initial q may define this new planning world.
    # Neither selection claims an executed relocation or an inter-world path.
    saved=deepcopy(state.root())
    choices=[('SAVED_TERMINAL_JOINTS',state.q.copy()),('CONFIGURED_INITIAL_JOINTS',np.asarray(original.snapshot['robot']['q_rad']))]
    chosen=None
    for label,q in choices:
        state.q=q
        scene=batch_scene(original,state,built.robot,original.policy.layout_validation.layout)
        failure=built.validate_unloaded_state(q,scene.all_obstacles,stage='unloaded')
        checks.append(dict(selection=label,root=state.root(),passed=failure is None,failure=failure))
        if failure is None:chosen=label;break
    selection=dict(saved_root=saved,checks=checks,selected=chosen,new_root=state.root() if chosen else None,
        no_interworld_motion_claimed=True,layout_audit=audit)
    save(args.output/'initial-state-selection.json',selection)
    if chosen is None:raise ValueError('Y_SHIFT_NO_CHECKED_INITIAL_STATE_NO_SEARCH')
    for target in TARGETS:
        root_state=deepcopy(state);assumption=None
        if target==TARGETS[1]:assumption=assume_l05_removed(root_state)
        scene=batch_scene(original,root_state,built.robot,original.policy.layout_validation.layout)
        failure=built.validate_unloaded_state(root_state.q,scene.all_obstacles,stage='unloaded')
        if failure is not None:raise ValueError('Y_SHIFT_TARGET_ROOT_REJECTED:'+target)
        states[target]=root_state;assumptions[target]=assumption
    save(args.output/'initial-state-checks.json',dict(passed=True,selected=chosen,
        base_transform=built.robot.base_transform.tolist(),roots={n:s.root() for n,s in states.items()}))
    if args.check_only:return
    args.task_budget_s,args.stage_budget_s,args.ipc_timeout_s=300.,90.,100.
    args.direct_first,args.upper_arm_preference=True,True
    args.low_top_minimum_m=.25;args.receive_height_range=(.25,2.10)
    args.low_definition='FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'
    args.include_receive_before_approach=True;args.right_front_eccentric=False;args.junction_only=False
    outcomes=[];unsearched=[]
    for target in TARGETS:
        for index,case in enumerate(CASES[target]):
            state=states[target];assumption=assumptions[target]
            args.l05_face_permission=None;args.l04_face_permission=None
            scope='PAIRED_MOUNT_INDEPENDENT_SINGLE_CARTON';markers={}
            if case=='side_negative_y_hmin':
                from unloading_sim.l05_face_trial import permission,SCOPE
                scope=SCOPE;args.l05_face_permission=permission(frozen,'side_negative_y',state.height,'Hmin')
                markers=dict(l05_face_permission=args.l05_face_permission,receive_mode='Hmin',root_stage='population_terminal')
            elif case!='front':
                from unloading_sim.l04_face_trial import permission,SCOPE
                scope=SCOPE;args.l04_face_permission=permission(frozen,case,state.height)
                markers=dict(l04_face_permission=args.l04_face_permission,receive_mode='Hmax',root_stage='assumed_population_initial')
            folder=args.output/target/case;folder.mkdir(parents=True);(folder/'seed-71070').mkdir()
            protocol=dict(**identity,**FLAGS,**markers,scope=scope,seeds=[71070],scoped_target_ids=[target],
                frozen=frozen.evidence(),original_cartons=[box_message(b) for b in original.cartons],
                configured_initial_q=state.q.tolist(),initial_root=state.root(),right_front_eccentric=False,
                active_layout_file='configs/workcells/'+LAYOUT,collision_policy=built.collision_policy.to_mapping(),
                third_sync_disabled=True,all_joint_rate_limits_retained=True,task_budget_s=300.,stage_budget_s=90.,
                authorized_mount_change=change,initial_state_selection=selection,case=case,
                comparison_mode='INDEPENDENT_CHANGED_MOUNT_SAVED_SCENE',historical_completed_count=24,
                worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),no_historical_solution_hints=True,
                original_population_predecessor_summary_sha256=predecessor['summary_sha256'])
            if assumption:protocol.update(assumed_removal=assumption,assumed_removed_target_ids=['carton_l05_c03'])
            save(folder/'protocol.json',protocol)
            print(json.dumps(dict(event='CASE_START',target=target,case=case)),flush=True)
            result=run_seed(args,original,built,frozen,71070,folder/'seed-71070',initial_state=deepcopy(state),target_ids=[target])
            assert state.root()==protocol['initial_root']
            outcomes.append(dict(target_id=target,case=case,folder=str(folder.relative_to(args.output)),
                completed_count=result['completed_count'],status=result['status'],planning_wall_s=result['planning_wall_s'],attempts=result['attempts']))
            if result['completed_count']:
                unsearched.extend(dict(target_id=target,case=n,reason='FIRST_QUALIFIED_COMPLETE_CYCLE_FOUND') for n in CASES[target][index+1:])
            save(args.output/'summary.json',dict(**identity,**FLAGS,authorized_mount_change=change,independent_trials=True,
                outcomes=outcomes,unsearched=unsearched,previous_population_count_unchanged=24))
            print(json.dumps(dict(event='CASE_RECORDED',target=target,case=case,completed=result['completed_count'])),flush=True)
            if result['completed_count']:break


if __name__=='__main__':main()
