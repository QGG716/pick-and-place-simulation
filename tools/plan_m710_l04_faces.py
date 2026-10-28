"""Independent requested l04_c03 faces at the assumed-l05-removal root."""
import argparse,hashlib,json
from copy import deepcopy
from pathlib import Path
import numpy as np
from plan_m710_mount_comparison import make_context,FLAGS
from plan_m710_top_layer_lifted import ROOT,save,source_identity,run_seed,BatchState,box_message
from plan_m710_l05_faces import restore_root
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.current_population_recipe import LAYOUT
from unloading_sim.assumed_removal import assume_l05_removed
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.l04_face_trial import TARGET,SCOPE,FACES,permission


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--predecessor',type=Path,required=True)
    p.add_argument('--assumed-population',type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-commit',required=True);p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--development',action='store_true');p.add_argument('--check-only',action='store_true')
    args=p.parse_args();identity=source_identity(args)
    identity.update(source_state='BASE_COMMIT_WITH_RECORDED_PATCH',
        source_manifest_sha256=hashlib.sha256(args.source_manifest.read_bytes()).hexdigest())
    args.output.mkdir(parents=True,exist_ok=False)
    original,built,audit=make_context('x_minus_300_y_minus_100',layout_path=ROOT/'configs/workcells'/LAYOUT,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    frozen=FrozenCartonPopulation.capture(original.cartons)
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),.6)
    predecessor=restore_root(args.predecessor,state,original,built,terminal=True)
    assumption=assume_l05_removed(state)
    prior=json.loads((args.assumed_population/'summary.json').read_text())
    verified=json.loads((args.assumed_population/'verification.json').read_text())
    prior_protocol=json.loads((args.assumed_population/'protocol.json').read_text())
    assert verified['status']=='PASS_SAVED_NATIVE_LINEAGE_POPULATION_AND_FAILURE_RETENTION'
    assert prior['initial']==state.root() and prior_protocol['assumed_removal']==assumption
    assert prior_protocol['collision_policy']==json.loads(json.dumps(built.collision_policy.to_mapping()))
    front=next(o for o in prior['outcomes'] if o['target_id']==TARGET)
    assert front['root']==state.root() and front['completed_count']==0
    save(args.output/'predecessor.json',predecessor);save(args.output/'assumed-removal.json',assumption)
    save(args.output/'retained-front-failure.json',dict(outcome=front,
        summary_sha256=hashlib.sha256((args.assumed_population/'summary.json').read_bytes()).hexdigest(),
        verification_sha256=hashlib.sha256((args.assumed_population/'verification.json').read_bytes()).hexdigest(),
        comparison='INDEPENDENT_SAME_15_BOX_INITIAL_ROOT',historical_solver_state_loaded=False))
    scene=batch_scene(original,state,built.robot,original.policy.layout_validation.layout)
    failure=built.validate_unloaded_state(state.q,scene.all_obstacles,stage='unloaded')
    save(args.output/'initial-state-check.json',dict(passed=failure is None,failure=failure,root=state.root(),layout_audit=audit))
    if failure is not None:raise ValueError('L04_FACE_INITIAL_STATE_REJECTED')
    from unloading_sim.contact_target import simulation_contact_targets
    from unloading_sim.l01_selected_routes import finite_side_exposure
    from types import SimpleNamespace
    target=next(b for b in state.cartons if b.name==TARGET)
    geometry=SimpleNamespace(tool_collision_obbs_provider=built.tool_collision_obbs_provider,
        robot_state_validator=built.robot_state_validator,budget=built.budget,collision_policy=built.collision_policy,
        physical_contact_pose=lambda q:built.physical_from_virtual(built.robot.fk(q)))
    exposure=finite_side_exposure(geometry,state.q)
    _,screen=simulation_contact_targets(scene,target,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m,
        side_exposure_depth_m=exposure['depth_m'])
    save(args.output/'face-screening.json',dict(screening=screen,finite_side_exposure=exposure))
    if args.check_only:return
    args.task_budget_s,args.stage_budget_s,args.ipc_timeout_s=300.,90.,100.
    args.direct_first,args.upper_arm_preference=True,True
    args.low_top_minimum_m=.25;args.receive_height_range=(.25,2.10)
    args.low_definition='FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'
    args.include_receive_before_approach=True;args.right_front_eccentric=False;args.junction_only=False
    reports=[]
    for face in FACES:
        args.l04_face_permission=permission(frozen,face,state.height)
        folder=args.output/face;folder.mkdir();(folder/'seed-71070').mkdir()
        protocol=dict(**identity,**FLAGS,scope=SCOPE,seeds=[71070],scoped_target_ids=[TARGET],
            frozen=frozen.evidence(),original_cartons=[box_message(b) for b in original.cartons],
            configured_initial_q=state.q.tolist(),initial_root=state.root(),right_front_eccentric=False,
            active_layout_file='configs/workcells/'+LAYOUT,collision_policy=built.collision_policy.to_mapping(),
            third_sync_disabled=True,all_joint_rate_limits_retained=True,task_budget_s=300.,stage_budget_s=90.,
            comparison_mode='INDEPENDENT_SAME_15_BOX_INITIAL_ROOT',root_stage='assumed_population_initial',
            l04_face_permission=args.l04_face_permission,receive_mode='Hmax',assumed_removal=assumption,
            assumed_removed_target_ids=['carton_l05_c03'],
            worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
            historical_completed_count=24,no_historical_solution_hints=True)
        save(folder/'protocol.json',protocol)
        print(json.dumps(dict(event='FACE_START',face=face)),flush=True)
        result=run_seed(args,original,built,frozen,71070,folder/'seed-71070',initial_state=deepcopy(state),target_ids=[TARGET])
        assert state.root()==assumption['after']
        reports.append(dict(face=face,status=result['status'],completed_count=result['completed_count'],
            planning_wall_s=result['planning_wall_s'],attempts=result['attempts']))
        save(args.output/'summary.json',dict(**identity,**FLAGS,independent_trials=True,target_id=TARGET,
            initial_root=state.root(),outcomes=reports,assumed_removed_target_ids=['carton_l05_c03'],
            previous_population_count_unchanged=24))
        print(json.dumps(dict(event='FACE_RECORDED',face=face,completed=result['completed_count'])),flush=True)


if __name__=='__main__':main()
