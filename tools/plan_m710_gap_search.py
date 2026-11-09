"""Fresh40 fixed-mount native gap search; saved failures are never completions."""
import argparse,hashlib,json,os,sys,time,traceback
from copy import deepcopy
from pathlib import Path
from dataclasses import replace
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from unloading_sim.gap_search_layout import GapCandidate,SCHEMA,BASELINE,candidate_data,floor_support_evidence,initial_stack_facing_evidence
from unloading_sim.gap_search_recipes import candidate_options
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import check_coupled_state
from unloading_sim.coupled_batch import BatchState,batch_scene,message_box
from unloading_sim.carton_population import FrozenCartonPopulation,rank_population
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.l01_selected_routes import finite_side_exposure
from unloading_sim.moveit2_backend import digest
from plan_m710_mount_comparison import make_context
from plan_m710_top_layer_lifted import save
from plan_m710_joint_search import run_trial,node_for,restore_state
from plan_m710_all_remaining import archive_task

def emit(**data):print(json.dumps(data,ensure_ascii=False,allow_nan=False),flush=True)

def frozen_layout(g,mount_height=1.):
    candidate=GapCandidate(g,mount_height);path=ROOT/'configs/workcells'/('gap-search-'+candidate.key+'.yaml')
    base=yaml.safe_load((path.parent/BASELINE).read_text(encoding='utf-8'))
    data=candidate_data(base,candidate)
    if path.exists():
        if yaml.safe_load(path.read_text())!=data:raise ValueError('FROZEN_GAP_CONFIG_CHANGED')
    else:path.write_text(yaml.safe_dump(data,sort_keys=False))
    return load_workcell_layout(path)

SUPPORT_MODES=('require-modeled','record-unknown')


def admission(layout,external_support_mode='require-modeled'):
    if external_support_mode not in SUPPORT_MODES:raise ValueError('UNKNOWN_EXTERNAL_SUPPORT_MODE')
    numeric=audit_layout_constraints(layout);checks=[]
    if numeric['overall_status']!='PASS':raise ValueError('GAP_NUMERIC_AUDIT_FAILED')
    for height in (.55,.6,1.,1.5):
        state=coupled_conveyor_state(layout,height,layout.cartons())
        checked=check_coupled_state(layout,height,layout.cartons(),designed_contacts=[['chassis','conveyor_transverse']])
        support=floor_support_evidence(layout,state)
        checks.append(dict(height_m=height,geometry=checked,floor_support=support))
    collision=any(not c['geometry']['passed'] for c in checks)
    unknown=any(c['floor_support']['status']!='VERIFIED_ON_MODELED_FLOOR' for c in checks)
    status=('SAMPLED_WORLD_COLLISION' if collision else
        'ADMITTED_EXTERNAL_SUPPORT_UNVERIFIED' if unknown and external_support_mode=='record-unknown' else
        'UNKNOWN_EXTERNAL_SUPPORT' if unknown else 'ADMITTED')
    return dict(status=status,external_support_mode=external_support_mode,support_verified=not unknown,
        layout_data=layout.data,layout_fingerprint=layout.layout_fingerprint,numeric=numeric,checks=checks,
        full_footprint_support_required_for_verified_result=True,global_infeasibility_proven=False,
        mechanism_design='REFERENCE_G0200_RAIL_AND_RELIEF_DIMENSIONS_FROZEN_NO_LENGTH_CHANGE')

def prepare(args,layout,record):
    initial=record.get('initial')
    requested_initial_q=getattr(args,'initial_q_rad',None)
    specified_q=initial['q_rad'] if initial else requested_initial_q
    original,built,audit=make_context('gap',layout_path=layout.config_path,
        initial_q=specified_q,initial_seed=args.seed if specified_q is None else None,initial_draws=args.initial_draws)
    frozen=FrozenCartonPopulation.capture(original.cartons)
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),.6)
    scene=batch_scene(original,state,built.robot,layout)
    attempts=[];rng=np.random.default_rng(args.seed)
    orientation=getattr(args,'initial_orientation','stack-facing')
    if orientation not in ('stack-facing','unconstrained'):raise ValueError('UNKNOWN_INITIAL_ORIENTATION')
    proposals=[state.q] if specified_q is not None else [state.q,*rng.uniform(built.robot.joint_limits[:,0]+.05,built.robot.joint_limits[:,1]-.05,size=(args.initial_draws,6))]
    for i,q in enumerate(proposals):
        facing=initial_stack_facing_evidence(built.physical_from_virtual(built.robot.fk(q)),original.cartons)
        failure=built.validate_unloaded_state(q,scene.all_obstacles,stage='unloaded')
        if failure is None and orientation=='stack-facing' and not facing['passed']:
            failure='INITIAL_CUPS_NOT_FACING_STACK'
        from unloading_sim.front_arm_posture import front_arm_posture
        posture=front_arm_posture(built.robot,q,initial=True) if getattr(args,'front_arm_posture','legacy-soft')=='upright' else None
        if failure is None and posture is not None and not posture['passed']:failure='INITIAL_ARM_POSTURE_REJECTED'
        attempts.append(dict(index=i,q_rad=q.tolist(),failure=failure,stack_facing=facing,arm_posture=posture))
        if failure is None:state.q=q;break
    else:raise ValueError('NO_INITIAL_WITNESS_IN_FINITE_DRAWS:'+json.dumps(attempts))
    built.joint_search_initial_q=state.q.tolist()
    record['initial_orientation_evidence']=dict(mode=orientation,**facing)
    record['initial_arm_posture_evidence']=posture
    if initial:
        assert state.root()==initial
    else:
        record['initial']=deepcopy(state.root());node=node_for(state,layout)
        record.update(nodes={node['id']:node},best_node=node['id'],active_node=node['id'],initial_audit=audit,initial_candidates=attempts,best_count=0)
    return original,built,frozen

def offered(original,built,state,frozen):
    layout=original.policy.layout_validation.layout;scene=batch_scene(original,state,built.robot,layout)
    rows=rank_population(frozen,state.cartons,state.accounted_ids(),set(),state.q,built.robot,scene.receiver,
        longitudinal=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'))
    if rows:
        top=max(min(b.corners()[:,2]) for b in state.cartons if b.name in {r['target_id'] for r in rows})
        rows=[r for r in rows if abs(min(next(b for b in state.cartons if b.name==r['target_id']).corners()[:,2])-top)<1e-9]
    depth=finite_side_exposure(built,state.q)['depth_m'];groups=[];screens={}
    for row in rows:
        box=next(b for b in state.cartons if b.name==row['target_id'])
        faces,screen=simulation_contact_targets(scene,box,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m,side_exposure_depth_m=depth)
        options=list(candidate_options(faces,scene,frozen,state.height,layout,initial_q=built.joint_search_initial_q))
        screens[box.name]=dict(screening=screen,candidates=len(options),upper_original_layer=bool(min(box.corners()[:,2])>=1.5-1e-9))
        groups.append(options)
    # Each same-layer carton gets one fair visit before returning to its next
    # process option. Within a carton the face order is TOP, FRONT, SIDE.
    return [group[i] for i in range(max(map(len,groups),default=0)) for group in groups if i<len(group)],screens

def failure_category(result):
    failure=result.get('failure') or {}
    # Never search nested request/world dumps: every request contains the word
    # "clearance", including ordinary numerical timeouts.
    value=' '.join(str(failure.get(k,'')) for k in ('reason','termination')).upper()
    if failure.get('reason')=='MoveItUnavailable':value+=' '+str(failure.get('detail','')).upper()
    if result.get('implementation_failure'):return 'SOFTWARE_ERROR'
    if any(x in value for x in ('DEADLINE','TIMEOUT','BUDGET_EXHAUSTED')):return 'NUMERICAL_SEARCH_DEADLINE_NOT_UNREACHABLE'
    if any(x in value for x in ('COLLISION','CLEARANCE','PENETRATION')):return 'COLLISION_OR_CLEARANCE'
    if 'IK' in value:return 'IK_SEARCH_INCOMPLETE'
    return 'FINITE_NATIVE_SEARCH_INCOMPLETE'

def queue_refinements(db,key):
    record=db['layouts'][key]
    if record.get('refinements_queued'):return
    record['refinements_queued']=True;g=record['gap_m']
    # An actually accepted cycle is the promising-neighborhood criterion.
    # Explicit 10 mm neighbors and 5 mm midpoints; never a feasibility proof.
    for delta in (-.02,-.01,.01,.02,-.005,.005):
        value=round(g+delta,6)
        if value<.005 or value in db['requested_gaps_m']:continue
        db['requested_gaps_m'].append(value)
        db['refinement_queue'].append(dict(gap_m=value,parent_gap_m=g,step_m=abs(delta),basis='ACCEPTED_NATIVE_COMPLETE_CYCLE'))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--resume',action='store_true')
    p.add_argument('--worker-command',required=True);p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--gaps',nargs='*',type=float,default=[.2,.005,*[round(i*.05,3) for i in range(1,25)]])
    p.add_argument('--max-trials',type=int,default=120,help='Finite invocation resource limit; resume retains all costs and cursors')
    p.add_argument('--trial-seconds',type=float,default=120.);p.add_argument('--prefix-seconds',type=float,default=45.)
    p.add_argument('--stage-seconds',type=float,default=60.);p.add_argument('--initial-draws',type=int,default=64)
    p.add_argument('--connection-seconds',type=float,default=6.,help='Numerical visit to one retained IK configuration, bounded by the existing shared deadline')
    p.add_argument('--ompl-seconds',type=float,default=2.,help='One native RRT restart allowance, bounded by connection, stage and task deadlines')
    p.add_argument('--initial-q-rad',nargs=6,type=float,
        help='Explicit fresh-world initial joints, independently checked without random fallback; never changes an existing world')
    p.add_argument('--initial-orientation',choices=('stack-facing','unconstrained'),default='stack-facing',
        help='Physical cup normal points into the stack at the initial state; unconstrained is for explicit historical reproduction only')
    p.add_argument('--seed',type=int,default=71107);p.add_argument('--refine',action='store_true')
    p.add_argument('--front-arm-posture',choices=('legacy-soft','upright'),default='legacy-soft')
    p.add_argument('--mount-height',type=float,choices=(.9,1.),default=1.,help='Fixed for the entire fresh world; 900 mm is the user-authorized ceiling fallback')
    p.add_argument('--admission-only',action='store_true');p.add_argument('--max-rounds',type=int,default=2)
    p.add_argument('--external-support-mode',choices=SUPPORT_MODES,default='require-modeled',
        help='record-unknown permits native planning only; preserves the finite floor and explicitly withholds support qualification')
    p.add_argument('--focus-gap',type=float,help='Continue only this already-admitted g; other worlds and all costs remain in the checkpoint')
    p.add_argument('--stop-file',type=Path,help='Finish the current trial, then checkpoint if this file exists')
    args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if min(args.max_trials,args.trial_seconds,args.prefix_seconds,args.stage_seconds,args.initial_draws,args.max_rounds)<=0:
        raise ValueError('NONPOSITIVE_SEARCH_RESOURCE_LIMIT')
    from unloading_sim.approach_recovery_policy import bounded_numeric_slice
    for value in (args.connection_seconds,args.ompl_seconds):bounded_numeric_slice(value,args.trial_seconds)
    identity=dict(source_manifest_sha256=hashlib.sha256(args.source_manifest.read_bytes()).hexdigest(),
        native_binary_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest())
    manifest=json.loads(args.source_manifest.read_text())
    for name,sha in manifest['files'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=sha:raise ValueError('SOURCE_MANIFEST_CHANGED:'+name)
    path=out/'checkpoint.json'
    if args.resume:
        db=json.loads(path.read_text());assert db['identity']==identity and db['contract']==SCHEMA
        if db.get('options',{}).get('external_support_mode','require-modeled')!=args.external_support_mode:
            raise ValueError('RESUME_EXTERNAL_SUPPORT_MODE_CHANGED')
        if args.initial_q_rad is not None and db.get('options',{}).get('requested_initial_q_rad')!=args.initial_q_rad:
            raise ValueError('RESUME_INITIAL_STATE_CHANGED')
        if db.get('options',{}).get('initial_orientation','unconstrained')!=args.initial_orientation:
            raise ValueError('RESUME_INITIAL_ORIENTATION_CHANGED')
        if db.get('options',{}).get('front_arm_posture','legacy-soft')!=args.front_arm_posture:
            raise ValueError('RESUME_FRONT_ARM_POSTURE_CHANGED')
        if db.get('options',{}).get('mount_height_m',1.)!=args.mount_height:
            raise ValueError('RESUME_MOUNT_HEIGHT_CHANGED')
        if db.get('pending'):
            pending=db.pop('pending');folder=out/pending['folder']
            if folder.exists():pending['archive']=archive_task(folder,out)
            pending.update(category='INTERRUPTED_RESTART_NOT_CONTINUATION',complete=False,
                wall_s=max(0.,time.time()-pending['started_unix']))
            db['interrupted'].append(pending);db['elapsed_search_s']+=pending['wall_s'];save(path,db)
    else:
        if path.exists():raise ValueError('EXISTING_SEARCH_REQUIRES_RESUME')
        db=dict(contract=SCHEMA,identity=identity,source=manifest,created_unix=time.time(),layouts={},trials=[],interrupted=[],
            elapsed_search_s=0.,best=None,physical_execution='NOT_RUN',global_infeasibility_proven=False,
            requested_gaps_m=list(dict.fromkeys(args.gaps)),refinement_queue=[],status='ADMISSION',
            options=dict(seed=args.seed,max_rounds=args.max_rounds,external_support_mode=args.external_support_mode,
                requested_initial_q_rad=args.initial_q_rad,initial_orientation=args.initial_orientation,
                front_arm_posture=args.front_arm_posture,mount_height_m=args.mount_height),
            source_commit=manifest.get('base_commit','WORKTREE'))
        save(path,db)
    for g in db['requested_gaps_m']:
        key=GapCandidate(g,args.mount_height).key
        if key in db['layouts']:continue
        layout=frozen_layout(g,args.mount_height);start=time.monotonic();record=admission(layout,args.external_support_mode)
        record.update(gap_m=g,layout_path=layout.config_path.name,admission_wall_s=time.monotonic()-start,trials=0,candidate=layout.data['gap_search'])
        db['layouts'][key]=record;save(path,db);emit(event='admission',gap_m=g,status=record['status'])
    if args.admission_only:return
    db.setdefault('invocations',[]).append(dict(started_unix=time.time(),source_identity=identity,
        max_trials=args.max_trials,trial_seconds=args.trial_seconds,prefix_seconds=args.prefix_seconds,
        stage_seconds=args.stage_seconds,max_rounds=args.max_rounds,focus_gap_m=args.focus_gap,
        external_support_mode=args.external_support_mode,
        connection_seconds=args.connection_seconds,ompl_seconds=args.ompl_seconds,
        starting_trial_count=len(db['trials']),semantics='NEW_NATIVE_REQUESTS_WITH_RETAINED_SCENE_AND_CUMULATIVE_COST'))
    save(path,db)
    cache={};invocation=0
    while invocation<args.max_trials:
        if args.stop_file is not None and args.stop_file.exists():
            db['status']='CHECKPOINTED_BETWEEN_TRIALS';save(path,db);break
        available=[(k,r) for k,r in db['layouts'].items() if r['status'] in ('ADMITTED','ADMITTED_EXTERNAL_SUPPORT_UNVERIFIED','ACTIVE')]
        if args.focus_gap is not None:
            focus=GapCandidate(args.focus_gap,args.mount_height).key
            if focus not in db['layouts']:raise ValueError('FOCUSED_GAP_NOT_ADMITTED')
            available=[(k,r) for k,r in available if k==focus]
        if not available:db['status']='FINITE_ADMITTED_SEARCH_EXHAUSTED';break
        # Highest accepted progress gets one extra visit, while every admitted
        # g receives fair rounds. No accepted edges ever cross layout identities.
        # Honor the explicitly requested distance order, without a hidden 200 mm preference.
        available.sort(key=lambda kr:(kr[1]['trials']//5,-kr[1].get('best_count',0),db['requested_gaps_m'].index(kr[1]['gap_m'])))
        key,record=available[0];layout=frozen_layout(record['gap_m'],args.mount_height)
        if key not in cache:
            start=time.monotonic()
            try:cache[key]=prepare(args,layout,record)
            except Exception as exc:
                record.update(status='INITIALIZATION_FAILED',initialization_error=str(exc),initialization_traceback=traceback.format_exc(),global_unreachable=False)
                db['elapsed_search_s']+=time.monotonic()-start;save(path,db);emit(event='initialization_failed',gap_m=record['gap_m'],error=str(exc)[:500]);continue
            db['elapsed_search_s']+=time.monotonic()-start
        original,built,frozen=cache[key];node=record['nodes'][record['active_node']]
        state=restore_state(node,frozen);visits,screens=offered(original,built,state,frozen)
        node['screens']=screens;node['candidate_count']=len(visits)
        node['unsearched_candidate_count']=max(0,len(visits)-node['cursor'])
        if not visits or node['cursor']>=len(visits)*args.max_rounds:
            node['closed']=True
            parents=[n for n in record['nodes'].values() if not n['closed']]
            if not parents:record['status']='FINITE_CANDIDATES_EXHAUSTED'
            else:record['active_node']=max(parents,key=lambda n:len(n['root']['completed']))['id']
            save(path,db);continue
        index=node['cursor'];target,option=visits[index%len(visits)];node['cursor']+=1
        node['unsearched_candidate_count']=max(0,len(visits)-node['cursor'])
        node['unsearched_numerical_round_visits']=max(0,len(visits)*args.max_rounds-node['cursor'])
        number=len(db['trials'])+len(db['interrupted']);folder=out/f'task-{number:06d}'
        seed=args.seed+number;budget=args.trial_seconds*(1+(index//len(visits)))
        db['pending']=dict(layout=key,node=node['id'],folder=folder.name,target_id=target.target_id,option_key=option['candidate_key'],
            started_unix=time.time(),allocated_budget_s=budget,seed=seed)
        save(path,db);emit(event='trial_start',number=number,gap_m=record['gap_m'],completed=len(state.ledger),target=target.target_id,mode=option['template'],height=option['height_m'])
        plan,result=run_trial(args,original,built,state,target,option,folder,budget,seed)
        row=dict(result,layout=key,node=node['id'],target_id=target.target_id,diagnostic=False,
            category='COMPLETE_CYCLE' if plan else failure_category(result),trial_number=number)
        if plan is not None:
            state.commit(plan);child=node_for(state,layout,parent=node['id'],incoming=len(db['trials']))
            record['nodes'][child['id']]=child;node['children'].append(child['id']);record['active_node']=child['id']
            if len(state.ledger)>len(record['nodes'][record['best_node']]['root']['completed']):record['best_node']=child['id'];record['best_count']=len(state.ledger)
            if db['best'] is None or len(state.ledger)>len(db['layouts'][db['best']]['nodes'][db['layouts'][db['best']]['best_node']]['root']['completed']):db['best']=key
            save(out/f'accepted-{key}-{len(state.ledger):02d}.json',plan)
            if args.refine:queue_refinements(db,key)
        row['evidence']=archive_task(folder,out);db['trials'].append(row);db['pending']=None
        record['trials']+=1;record['status']='ACTIVE';db['elapsed_search_s']+=result['wall_s'];invocation+=1
        db['status']=('QUALIFIED_40_OF_40' if record.get('support_verified',True) else
            'NATIVE_40_OF_40_EXTERNAL_SUPPORT_UNVERIFIED') if len(state.ledger)==40 else 'SEARCHING'
        save(path,db)
        failure=result.get('failure') or {}
        emit(event='trial_end',number=number,gap_m=record['gap_m'],completed=len(state.ledger),category=row['category'],wall_s=result['wall_s'],
            failure={k:v for k,v in failure.items() if isinstance(v,(str,int,float,bool))})
        if len(state.ledger)==40:break
        if row['category']=='SOFTWARE_ERROR':db['status']='SOFTWARE_ERROR_RETAINED';save(path,db);break
    if db['status']=='SEARCHING':db['status']='INVOCATION_RESOURCE_LIMIT_CHECKPOINTED'
    # Exhausted/empty admissions can leave the loop without completing a
    # trial. Persist that terminal status as well as ordinary resource stops.
    save(path,db)
    emit(event='checkpoint',status=db['status'],trials=len(db['trials']),best=db['best'],elapsed_search_s=db['elapsed_search_s'])
if __name__=='__main__':main()
