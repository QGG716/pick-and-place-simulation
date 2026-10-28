"""Independent top and both side-face trials at the retained l05_c03 failure root."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tarfile
import numpy as np
from plan_m710_mount_comparison import make_context, FLAGS
from plan_m710_top_layer_lifted import ROOT, save, source_identity, run_seed, BatchState, box_message
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.current_population_recipe import LAYOUT
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.l05_face_trial import TARGET,SCOPE,FACES,permission


def restore_root(run,state,original,built,*,terminal=False):
    read=lambda n:json.loads((run/n).read_text())
    protocol,summary,verification=(read(n) for n in ('protocol.json','summary.json','verification.json'))
    if (protocol['scope']!='CURRENT_RECIPES_NEW_WORLD_40' or not verification['status'].startswith('PASS_')
            or protocol['active_layout_file']!='configs/workcells/'+LAYOUT
            or protocol['collision_policy']!=json.loads(json.dumps(built.collision_policy.to_mapping()))
            or protocol['original_cartons']!=[box_message(b) for b in original.cartons]
            or summary['initial']!=state.root()):
        raise ValueError('L05_FACE_PREDECESSOR_CHANGED')
    receipts=[];retained_failure=None
    for outcome in summary['outcomes']:
        if outcome['target_id']==TARGET:
            assert outcome['completed_count']==0 and outcome['root']==state.root()
            retained_failure=outcome['attempts']
            if not terminal:break
        archive=run/outcome['evidence']['archive']
        assert hashlib.sha256(archive.read_bytes()).hexdigest()==outcome['evidence']['sha256']
        with tarfile.open(archive) as tar:
            raw=tar.extractfile(archive.name.removesuffix('.tar.gz')+'/result.json').read()
        assert hashlib.sha256(raw).hexdigest()==outcome['evidence']['file_sha256']['result.json']
        result=json.loads(raw)
        assert result['initial']==state.root() and result['completed_count']==len(result['cycles'])
        assert result['completed_count'] in (0,1)
        for cycle in result['cycles']:state.commit(cycle)
        assert state.root()==result['terminal']==outcome['terminal']
        receipts.append(dict(target_id=outcome['target_id'],archive_sha256=outcome['evidence']['sha256'],
            result_sha256=outcome['evidence']['file_sha256']['result.json']))
        state.cycles=state.cycles[-1:]
    if retained_failure is None:raise ValueError('L05_FACE_FAILURE_NOT_FOUND')
    assert len(state.cartons)==(16 if terminal else 29) and len(state.ledger)==(24 if terminal else 11)
    if terminal:assert state.root()==summary['terminal']
    archived=state.root();state.cycles=[]
    return dict(prefix=receipts,archived_root=archived,new_native_root=state.root(),
        summary_sha256=hashlib.sha256((run/'summary.json').read_bytes()).hexdigest(),
        protocol_sha256=hashlib.sha256((run/'protocol.json').read_bytes()).hexdigest(),
        verification_sha256=hashlib.sha256((run/'verification.json').read_bytes()).hexdigest(),
        retained_original_failure=retained_failure,search_operation='RESTART_AT_SAVED_SCENE_AND_Q',
        historical_solver_state_loaded=False,physical_execution='NOT_RUN')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--predecessor',type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-commit',required=True);p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--development',action='store_true');p.add_argument('--check-only',action='store_true')
    p.add_argument('--root-stage',choices=['original_failure','population_terminal'],default='original_failure')
    p.add_argument('--face',choices=list(FACES))
    p.add_argument('--receive-mode',choices=['Hmax','Hmin'],default='Hmax')
    args=p.parse_args()
    if args.receive_mode=='Hmin' and (args.face!='side_negative_y' or args.root_stage!='population_terminal'):
        p.error('Hmin requires --face side_negative_y --root-stage population_terminal')
    identity=source_identity(args)
    identity.update(source_state='BASE_COMMIT_WITH_RECORDED_PATCH',
        source_manifest_sha256=hashlib.sha256(args.source_manifest.read_bytes()).hexdigest())
    args.output.mkdir(parents=True,exist_ok=False)
    original,built,audit=make_context('x_minus_300_y_minus_100',layout_path=ROOT/'configs/workcells'/LAYOUT,
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    frozen=FrozenCartonPopulation.capture(original.cartons)
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),.6)
    terminal=args.root_stage=='population_terminal'
    predecessor=restore_root(args.predecessor,state,original,built,terminal=terminal)
    save(args.output/'predecessor.json',predecessor)
    scene=batch_scene(original,state,built.robot,original.policy.layout_validation.layout)
    failure=built.validate_unloaded_state(state.q,scene.all_obstacles,stage='unloaded')
    save(args.output/'initial-state-check.json',dict(passed=failure is None,failure=failure,root=state.root(),layout_audit=audit))
    if failure is not None:raise ValueError('L05_FACE_INITIAL_STATE_REJECTED')
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
    for face in ([args.face] if args.face else list(FACES)[1:] if terminal else FACES):
        args.l05_face_permission=permission(frozen,face,state.height,args.receive_mode)
        folder=args.output/face;folder.mkdir();(folder/'seed-71070').mkdir()
        protocol=dict(**identity,**FLAGS,scope=SCOPE,seeds=[71070],scoped_target_ids=[TARGET],
            frozen=frozen.evidence(),original_cartons=[box_message(b) for b in original.cartons],
            configured_initial_q=state.q.tolist(),initial_root=state.root(),right_front_eccentric=False,
            active_layout_file='configs/workcells/'+LAYOUT,collision_policy=built.collision_policy.to_mapping(),
            third_sync_disabled=True,all_joint_rate_limits_retained=True,task_budget_s=300.,stage_budget_s=90.,
            comparison_mode=f'INDEPENDENT_SAME_ACCEPTED_{len(state.ledger)}_BOX_ROOT',root_stage=args.root_stage,
            l05_face_permission=args.l05_face_permission,receive_mode=args.receive_mode,
            worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
            historical_completed_count=len(state.ledger),no_historical_solution_hints=True)
        save(folder/'protocol.json',protocol)
        print(json.dumps(dict(event='FACE_START',face=face)),flush=True)
        result=run_seed(args,original,built,frozen,71070,folder/'seed-71070',initial_state=deepcopy(state),target_ids=[TARGET])
        assert state.root()==predecessor['new_native_root']
        reports.append(dict(face=face,status=result['status'],completed_count=result['completed_count'],
            planning_wall_s=result['planning_wall_s'],attempts=result['attempts']))
        save(args.output/'summary.json',dict(**identity,**FLAGS,independent_trials=True,target_id=TARGET,
            initial_root=state.root(),outcomes=reports,previous_population_count_unchanged=24))
        print(json.dumps(dict(event='FACE_RECORDED',face=face,completed=result['completed_count'])),flush=True)


if __name__=='__main__':main()
