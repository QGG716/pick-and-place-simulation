"""Resumable fixed-installation fresh-40 native joint search (planning only)."""
import argparse
from copy import deepcopy
from dataclasses import replace
import gzip,hashlib,json,os,sys,tarfile,time,traceback
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from plan_m710_mount_comparison import make_context,FLAGS
from plan_m710_top_layer_lifted import save,source_identity,box_message
from plan_m710_all_remaining import archive_task
from unloading_sim.joint_search_layout import SCHEMA,BASELINE,LayoutCandidate,candidate_data,initial_candidates
from unloading_sim.joint_search_recipes import candidate_options,SearchCycleConnector,SearchTopConnector
from unloading_sim.coupled_batch import BatchState,batch_scene,coupled_binding,CoupledBatchClient,message_box
from unloading_sim.carton_population import FrozenCartonPopulation,rank_population
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import check_coupled_state
from unloading_sim.moveit2_backend import ResidentMoveItClient,digest,MoveItUnavailable
from unloading_sim.l01_selected_routes import finite_side_exposure

DIAGNOSTICS=['carton_l07_c00','carton_l07_c04','carton_l05_c00','carton_l05_c04','carton_l05_c03',
    'carton_l04_c00','carton_l04_c04','carton_l04_c03','carton_l04_c02','carton_l01_c00','carton_l01_c04',
    'carton_l00_c00','carton_l00_c04','carton_l00_c03']

def emit(**data):print(json.dumps(data,ensure_ascii=False,allow_nan=False),flush=True)

def compact_failure(failure):
    if not isinstance(failure,dict):return failure
    return {k:v for k,v in failure.items() if k in ('reason','scope','blocking_stage','global_unreachable') or k=='detail' and isinstance(v,str)}


def interrupted_evidence(folder,output):
    """Recover the archive-before-checkpoint crash window without committing it."""
    if folder.exists():
        calls=json.loads((folder/'requests.json').read_text()) if (folder/'requests.json').exists() else []
        return archive_task(folder,output),calls
    metadata=output/(folder.name+'-archive.json')
    if not metadata.exists():return None,[]
    evidence=json.loads(metadata.read_text())
    assert evidence['archive']==folder.name+'.tar.gz'
    archive=output/evidence['archive'];assert hashlib.sha256(archive.read_bytes()).hexdigest()==evidence['sha256']
    calls=[]
    with tarfile.open(archive) as tar:
        for name,sha in evidence['file_sha256'].items():
            raw=tar.extractfile(folder.name+'/'+name).read();assert hashlib.sha256(raw).hexdigest()==sha
            if name=='requests.json':calls=json.loads(raw)
    return evidence,calls

def admit_candidate(db):
    """Two coarse admissions per refinement; fine samples never spawn refinements."""
    fine=db['refinement_queue']
    if fine and db['admissions']%3==2:
        item=fine.pop(0);e=item['candidate'];kind='REFINEMENT';parent=item['parent']
    elif db['next_candidate']<len(db['candidates']):
        e=db['candidates'][db['next_candidate']];db['next_candidate']+=1;kind='COARSE';parent=None
    elif fine:
        item=fine.pop(0);e=item['candidate'];kind='REFINEMENT';parent=item['parent']
    else:return None
    db['admissions']+=1
    return e,kind,parent

def restore_state(node,frozen):
    root=node['root']
    return BatchState(frozen,tuple(message_box(b) for b in root['world_cartons']),np.asarray(root['q_rad']),root['height_m'],
        deepcopy(node['ledger']),[] if root['predecessor_final_state_receipt'] is None else
        [dict(final_state_receipt=deepcopy(root['predecessor_final_state_receipt']))])

def node_for(state,layout,parent=None,incoming=None):
    root=deepcopy(state.root())
    return dict(id=digest(dict(layout=layout.layout_fingerprint,root=root)),root=root,ledger=deepcopy(state.ledger),
        parent=parent,incoming=incoming,cursor=0,children=[],attempted=[],closed=False)

def bind_best_continuation(db,enabled=False):
    """Freeze the accepted prefix for a continuation of the remaining cartons."""
    scope=db.get('continuation_scope')
    if scope is None and enabled:
        if db.get('pending'):raise ValueError('CONTINUATION_REQUIRES_CLOSED_PREVIOUS_ATTEMPT')
        key=db.get('best')
        if key not in db['layouts']:raise ValueError('CONTINUATION_REQUIRES_ACCEPTED_BEST')
        record=db['layouts'][key];node=record['nodes'][record['best_node']];root=node['root']
        if (not root['completed'] or root.get('assumed_removed')
                or len(root['completed'])+len(root['world_cartons'])!=40):
            raise ValueError('CONTINUATION_REQUIRES_UNALTERED_FRESH40_PREFIX')
        scope=dict(layout=key,anchor_node=node['id'],anchor_root_sha256=digest(root),
            accepted_prefix=list(root['completed']),prior_trials=len(db['trials']),
            prior_elapsed_search_s=db['elapsed_search_s'],
            scope='ACCEPTED_BEST_PREFIX_AND_DESCENDANTS_ONLY',
            failed_cartons_remain_in_world=True,physical_execution='NOT_RUN')
        db['continuation_scope']=scope
    if scope is not None:
        record=db['layouts'][scope['layout']];root=record['nodes'][scope['anchor_node']]['root']
        if digest(root)!=scope['anchor_root_sha256'] or root['completed']!=scope['accepted_prefix']:
            raise ValueError('CONTINUATION_ANCHOR_CHANGED')
    return scope


def continuation_active_nodes(record,anchor=None):
    """Keep earlier and sibling worlds as evidence, without rewinding to them."""
    def descended(node):
        seen=set()
        while node is not None:
            key=node['id']
            if key in seen:raise ValueError('CYCLIC_SEARCH_PARENT_CHAIN')
            seen.add(key)
            if key==anchor:return True
            parent=node['parent']
            node=None if parent is None else record['nodes'][parent]
        return False
    return [n for n in record['nodes'].values()
        if not n['closed'] and len(n['children'])<3 and (anchor is None or descended(n))]


def option_visit_priority(option):
    pose,sync,mode,face,*timing=option['visit_order']
    # Diagonal rounds prevent every departure/sync combination at pose zero
    # from preceding the first alternate pose. Candidate identities stay fixed.
    departure=0 if not timing else 1 if timing[0]==.5 else int(timing[0])+1
    return pose+sync+departure,sync,pose,departure,mode,face

def coverage_order(options):
    """Visit existing business-choice representatives before their products.

    Every item is an unchanged, already screened candidate. A short node slice
    must not spend all visits on centered face/mode/departure combinations.
    """
    ordered=sorted(options,key=lambda item:option_visit_priority(item[1]))
    if not ordered:return []
    def dimensions(option):
        c=option['joint_search_contract']
        return (c['roll_rad'],tuple(c['offset_face_m']),c['face'],
            (c['mode'],option['height_m']),
            (option.get('outfeed_timing','WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW'),
                option.get('withdraw_extra_m',0.)),option['synchronization'])
    base=dimensions(ordered[0][1]);families=[{} for _ in base]
    for item in ordered:
        for index,value in enumerate(dimensions(item[1])):
            if value!=base[index]:
                previous=families[index].setdefault(value,item)
                # The accepted low-front suffix can fail only because the cup
                # still touches after a short drop. Exercise existing departure
                # alternatives on that face before spending them on top IK.
                if (index==4 and item[1]['joint_search_contract']['face']=='x-'
                        and previous[1]['joint_search_contract']['face']!='x-'):
                    families[index][value]=item
    queues=[list(family.values()) for family in families]
    queues[2].sort(key=lambda item:item[1]['joint_search_contract']['face']!='z+')
    queues[4].sort(key=lambda item:(item[1].get('outfeed_timing')!='WITHDRAW_THEN_WAIT_FULL_OUTFEED',
        item[1].get('withdraw_extra_m',0.)))
    result=[];seen=set()
    def append(item):
        key=item[1]['candidate_key']
        if key not in seen:seen.add(key);result.append(item)
    append(ordered[0])
    for index in range(max(map(len,queues),default=0)):
        for queue in queues:
            if index<len(queue):append(queue[index])
    for item in ordered:append(item)
    assert len(result)==len(ordered),'DUPLICATE_SCREENED_BUSINESS_CANDIDATE'
    return result


def continuation_coverage_order(options):
    """Try one short-drop front park with extra clearance, then the whole set."""
    ordered=coverage_order(options)
    preferred=[]
    for item in ordered:
        option=item[1];c=option['joint_search_contract']
        if (c['face']=='x-' and option.get('outfeed_timing')=='WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED'
                and option.get('withdraw_extra_m',0.) in (0.,.10)
                and 0.<=c.get('release_bottom_m',float('inf'))-option['height_m']<=.050000001):
            preferred.append(item)
    early=[]
    if preferred:early.append(min(preferred,key=lambda item:(item[1].get('withdraw_extra_m',0.)!=.10,option_visit_priority(item[1]))))
    for roll in (0.,np.pi):
        short_axis=[item for item in ordered if
            item[1]['joint_search_contract'].get('contact_pose_family')=='TOP_FRONT_EDGE_240_MM_SHORT_AXIS_V1'
            and item[1]['joint_search_contract']['roll_rad']==roll]
        if short_axis:early.append(min(short_axis,key=lambda item:option_visit_priority(item[1])))
    for roll in (np.pi/2,-np.pi/2):
        midpoint=[item for item in ordered if
            item[1]['joint_search_contract'].get('contact_pose_family')=='TOP_FRONT_EDGE_180_MM_QUARTER_TURN_V1'
            and item[1]['joint_search_contract']['roll_rad']==roll
            and item[1]['joint_search_contract']['mode']=='LIFTED_TOP_EXPLICIT' and item[1]['height_m']==.65]
        if midpoint:early.append(min(midpoint,key=lambda item:option_visit_priority(item[1])))
    raised=[item for item in ordered if
        item[1]['joint_search_contract'].get('contact_pose_family')=='TOP_FRONT_EDGE_120_240_MM_QUARTER_TURN_V1'
        and item[1]['joint_search_contract']['offset_face_m']==[-.12,0.]
        and item[1]['joint_search_contract']['roll_rad']==np.pi/2
        and item[1]['joint_search_contract']['mode']=='LIFTED_TOP_EXPLICIT'
        and item[1]['height_m']==.65]
    if raised:early.append(min(raised,key=lambda item:option_visit_priority(item[1])))
    # Four explicit upstream top poses; the existing native and geometry gates
    # still evaluate every selected pose and its complete cycle.
    top={}
    for item in ordered:
        c=item[1]['joint_search_contract']
        if c.get('contact_pose_family') in {'TOP_FRONT_EDGE_120_240_MM_QUARTER_TURN_V1','TOP_FRONT_EDGE_180_MM_QUARTER_TURN_V1'}:
            top.setdefault((tuple(c['offset_face_m']),c['roll_rad']),item)
    early.extend(top[key] for key in sorted(top,key=lambda k:(k[0][0],-k[1])))
    early=list({item[1]['candidate_key']:item for item in early}.values())
    keys={item[1]['candidate_key'] for item in early}
    return [*early,*[item for item in ordered if item[1]['candidate_key'] not in keys]]


def offered(original,built,state,frozen):
    layout=original.policy.layout_validation.layout;scene=batch_scene(original,state,built.robot,layout)
    ranks=rank_population(frozen,state.cartons,state.accounted_ids(),set(),state.q,built.robot,scene.receiver,
        longitudinal=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'))
    depth=finite_side_exposure(built,state.q)['depth_m'];groups=[];screens={}
    for row in ranks:
        box=next(b for b in state.cartons if b.name==row['target_id'])
        faces,screen=simulation_contact_targets(scene,box,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m,side_exposure_depth_m=depth)
        screens[box.name]=screen
        order=continuation_coverage_order if getattr(built,'joint_search_short_drop_departure_first',False) else coverage_order
        options=order(list(candidate_options(faces,scene,frozen,state.height,layout,initial_q=built.joint_search_initial_q)))
        groups.append(options)
    # All removable boxes and each face receive visits; no permanent attempted set.
    visits=[g[i] for i in range(max(map(len,groups),default=0)) for g in groups if i<len(g)]
    return scene,visits,screens

def run_trial(args,original,built,state,target,option,folder,budget,seed):
    folder.mkdir();began=time.monotonic();layout=original.policy.layout_validation.layout;fatal=False
    scene=batch_scene(original,state,built.robot,layout);client=None;c=None;plan=None;detail={}
    row=dict(initial=state.root(),target=target.evidence(),option=option,seed=seed,
        solver_seed_scope='EXPLICIT_MOVEIT_OMPL_SEED_NOT_KDL_INTERNAL_RANDOMNESS',planning_mode='cold_from_scratch',
        restart=True,historical_solver_state_loaded=False,layout_fingerprint=layout.layout_fingerprint)
    save(folder/'dispatch.json',row)
    try:
        preheight=option['pre_approach_height_m']
        pre=coupled_binding(layout,state.cartons,state.height,preheight,target.target_id,scene.snapshot['assembly']['designed_contacts'])
        binding=coupled_binding(layout,state.cartons,preheight,option['height_m'],target.target_id,scene.snapshot['assembly']['designed_contacts'])
        binding['pre_approach_transition']=pre
        binding['joint_search_layout']=layout.data['joint_search']
        if option.get('park_goal_source'):
            assert option['park_goal_source']=='FRESH40_FROZEN_INITIAL_LEGAL_STATE'
            assert option['park_q_rad']==built.joint_search_initial_q
            binding['outfeed_park_q_rad']=option['park_q_rad']
        binding['receive_contract']=dict(height_m=option['height_m'],initial_target=box_message(target.box),
            junction_front=bool(option['reception']['junction'] and target.geometric_face_id=='x-'),
            third_sync_disabled=True,joint_search_contract=option['joint_search_contract'])
        os.environ['M710_MOVEIT_SEED']=str(seed)
        client=CoupledBatchClient(ResidentMoveItClient(args.worker_command,log_path=str(folder/'worker.log')),binding=binding)
        cls=SearchTopConnector if target.face_role=='TOP' else SearchCycleConnector
        c=cls.from_existing(built,scene,native_client=client,planning_mode='cold_from_scratch',
            native_budget=dict(task_wall_time_s=budget,stage_wall_time_s=min(args.stage_seconds,budget),ipc_timeout_s=min(args.stage_seconds,budget)+10))
        c.option=option;c.batch_root=state.root();c.template_provenance=dict(role=dict(role=option['original_role']),template=option['template'])
        c.upper_arm_preference_enabled=True;c.empty_approach_direct_first=True
        c.height_stage_deadlines={'prefix':began+min(args.prefix_seconds,budget)};c.height_prefix_active=True
        c.height_visit_deadline=c.height_stage_deadlines['prefix']
        c.budget=replace(c.budget,planning_wall_time_s=budget,candidate_wall_time_s=None,
            stage_wall_time_s=min(args.stage_seconds,budget),postprocess_wall_time_s=min(args.stage_seconds,budget))
        c.stack_carton_names={b.name for b in state.cartons};c.robot_state_validator.stack_carton_names=set(c.stack_carton_names)
        c.start_planning_request(began);c._native_task_started=began
        c._request_deadline_monotonic=began+budget;c._deadline_monotonic=began+budget;c._final_export_reserve_s=0.;c.boundary_costs={}
        plan,detail=c.plan_target(target,state.q,scene.all_obstacles,scene.policy.data['suction'],seed)
    except Exception as exc:
        fatal=isinstance(exc,(TypeError,AttributeError,KeyError,AssertionError)) or isinstance(exc,MoveItUnavailable) and any(
            s in str(exc) for s in ('MISMATCH','SCOPE_CHANGED','REQUEST_STOP:ERROR','FORBIDDEN_REQUEST'))
        detail=deepcopy(getattr(c,'_single_active_row',None) or {})
        detail.update(complete=False,failure=dict(reason=type(exc).__name__,detail=str(exc),global_unreachable=False))
        (folder/'exception.txt').write_text(traceback.format_exc())
    finally:
        save(folder/'attempt.json',detail)
        if client:
            save(folder/'requests.json',client.records)
            save(folder/'search-trace.json',dict(model=client.trace_inputs.get('model'),attempts=client.traces))
            save(folder/'connection-attempts.json',getattr(c,'connection_attempts',[]))
            client.close()
    if plan is not None:
        from verify_m710_joint_search import verify_cycle
        save(folder/'cycle-verification.json',verify_cycle(plan,folder,state.root(),layout,built))
        save(folder/'plan.json',plan)
    wall=time.monotonic()-began
    costs={}
    for call in getattr(client,'records',[]):
        label=str(call.get('stage') or 'startup')+'/'+str(call.get('op'))
        costs[label]=costs.get(label,0.)+call.get('round_trip_s',0.)
    result=dict(**row,complete=plan is not None,wall_s=wall,failure=detail.get('failure'),
        contact_complete=detail.get('contact_complete',False),region_samples=len(detail.get('region_attempts',[])),
        physical_execution='NOT_RUN',implementation_failure=fatal,native_call_cost_s=costs)
    save(folder/'result.json',result)
    return plan,result

def prepare_layout(args,candidate):
    baseline=yaml.safe_load((ROOT/'configs/workcells'/BASELINE).read_text())
    path=ROOT/'configs/workcells'/('joint-search-'+candidate.key+'.yaml')
    data=candidate_data(baseline,candidate)
    if path.exists():assert yaml.safe_load(path.read_text())==data
    else:path.write_text(yaml.safe_dump(data,sort_keys=False))
    original,built,audit=make_context(candidate.key,layout_path=path,robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True,
        initial_seed=args.seed,initial_draws=args.initial_draws)
    layout=original.policy.layout_validation.layout;frozen=FrozenCartonPopulation.capture(original.cartons)
    checks=[]
    for height in sorted(set([candidate.minimum,.6,1.15,candidate.maximum])):
        check=check_coupled_state(layout,height,(),designed_contacts=original.snapshot['assembly']['designed_contacts'])
        checks.append(check)
        if not check['passed']:return None,dict(status='SAMPLED_MECHANISM_COLLISION',height=height,checks=checks,global_unreachable=False)
    q=np.asarray(original.snapshot['robot']['q_rad']);state=BatchState(frozen,tuple(original.cartons),q,.6)
    scene=batch_scene(original,state,built.robot,layout)
    initial=[]
    rng=np.random.default_rng(args.seed)
    proposals=[q,*[rng.uniform(built.robot.joint_limits[:,0]+.05,built.robot.joint_limits[:,1]-.05) for _ in range(args.initial_draws)]]
    chosen=None
    for index,q in enumerate(proposals):
        failure=built.validate_unloaded_state(q,scene.all_obstacles,stage='unloaded')
        initial.append(dict(index=index,q_rad=q.tolist(),failure=failure))
        if failure is None:chosen=q;break
    if chosen is None:return None,dict(status='NO_INITIAL_WITNESS_IN_FINITE_DRAWS',checks=checks,initial=initial,global_unreachable=False)
    state.q=chosen
    built.joint_search_initial_q=chosen.tolist()
    return (original,built,frozen,state),dict(status='PASS_GEOMETRY_SAMPLES_AND_INITIAL',checks=checks,initial=initial,
        layout_audit=audit,layout_data=data,layout_fingerprint=layout.layout_fingerprint,assets=layout.assets)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--resume',action='store_true')
    p.add_argument('--continue-from-best',action='store_true',help='On resume, freeze the accepted best prefix and search only its descendants; retain failed cartons and historical costs')
    p.add_argument('--budget-hours',type=float,default=8.);p.add_argument('--layout-slice-seconds',type=float,default=900.)
    p.add_argument('--trial-seconds',type=float,default=65.);p.add_argument('--prefix-seconds',type=float,default=20.)
    p.add_argument('--stage-seconds',type=float,default=40.);p.add_argument('--initial-draws',type=int,default=64)
    p.add_argument('--seed',type=int,default=71070);p.add_argument('--diagnostic-seconds',type=float,default=12.)
    p.add_argument('--diagnostic-targets',nargs='*',default=DIAGNOSTICS)
    p.add_argument('--worker-command',required=True);p.add_argument('--worker-binary',type=Path,required=True)
    p.add_argument('--source-manifest',type=Path,required=True);p.add_argument('--source-commit',required=True)
    p.add_argument('--development',action='store_true');p.add_argument('--max-layouts',type=int,default=0)
    p.add_argument('--single-candidate',nargs=4,type=float,metavar=('X_A','Y_A','Z_A','MIN_HEIGHT'))
    p.add_argument('--mechanism-profile',choices=['low_profile_250_v1','low_profile_200_v1','low_profile_200_v2','low_profile_200_outfeed20_v3'],default='low_profile_250_v1')
    p.add_argument('--prior-checkpoint',type=Path,help='Retain prior failed revision cost; never import its plans or ledger')
    p.add_argument('--scheduler-revision-from',type=Path,help='Allow resume only if the prior manifest differs solely in this scheduler entry')
    p.add_argument('--numeric-height-revision-from',type=Path,help='Exact recorded float-equality correction, with accepted-branch reverification')
    p.add_argument('--geometry-revision-from',type=Path,help='Checked v5 construction/serialization revision; keep every prior ledger and search cost')
    p.add_argument('--initial-witness-revision-from',type=Path,help='Checked v6 fresh-world initialization recovery; no accepted state changes')
    p.add_argument('--mount-metadata-revision-from',type=Path,help='Checked v7 propagation of existing support-plane metadata into mesh/native backend')
    p.add_argument('--outfeed-timing-revision-from',type=Path,help='Checked finite stationary-box withdrawal alternative; reverify all accepted branches')
    p.add_argument('--withdraw-distance-revision-from',type=Path,help='Checked finite additional normal withdrawals; preserve previous candidate identities and cost')
    p.add_argument('--outfeed-park-revision-from',type=Path,help='Checked native detached parking stage with full-tool path checks')
    p.add_argument('--candidate-fairness-revision-from',type=Path,help='Representative coverage of existing pose, face, receive, synchronization and departure choices; same candidate keys and checks')
    p.add_argument('--park-clearance-revision-from',type=Path,help='Add one explicit 100 mm normal withdrawal before native parking; retain old candidates, parent chains and costs')
    p.add_argument('--top-front-pose-revision-from',type=Path,help='Add explicit top-face offsets -120/-240 mm with rolls +/-90 degrees; retain old candidates, checks and costs')
    p.add_argument('--top-midpoint-pose-revision-from',type=Path,help='Add explicit top-face offset -180 mm with rolls +/-90 degrees; retain old candidates, checks and costs')
    p.add_argument('--top-short-axis-pose-revision-from',type=Path,help='Add explicit top-face offset -240 mm with rolls 0/180 degrees; retain old candidates, checks and costs')
    p.add_argument('--raised-top-revision-from',type=Path,help='Add named 650/850 mm top-lift reception candidates; preserve Hmin/Hmax and all prior cost')
    p.add_argument('--previous-worker-binary',type=Path,help='Required old-binary hash anchor for checked native revisions')
    args=p.parse_args()
    if args.continue_from_best and not args.resume:raise ValueError('CONTINUATION_REQUIRES_RESUME')
    manifest=json.loads(args.source_manifest.read_text())
    assert manifest['source_commit']==args.source_commit
    actual={str(p.relative_to(ROOT)).replace('\\','/') for d in ('src','tools','ros2','configs') for p in (ROOT/d).rglob('*')
        if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and not p.name.startswith('joint-search-')}
    assert actual==set(manifest['sha256'])
    for name,sha in manifest['sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha,name
    identity=dict(source_commit=args.source_commit,development=False)
    if min(args.budget_hours,args.trial_seconds,args.prefix_seconds,args.stage_seconds,args.layout_slice_seconds)<=0:raise ValueError('NONPOSITIVE_SEARCH_BUDGET')
    manifest=json.loads(args.source_manifest.read_text());source_hash=digest(manifest)
    worker_hash=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest();cp=args.output/'checkpoint.json'
    if args.resume:
        db=json.loads(cp.read_text())
        if db.get('pending'):
            db['pending'].setdefault('source_hash',db['source_hash'])
            db['pending'].setdefault('worker_hash',db['worker_hash'])
        if db['worker_hash']!=worker_hash:
            assert (args.mount_metadata_revision_from or args.outfeed_timing_revision_from or args.outfeed_park_revision_from or args.raised_top_revision_from) and args.previous_worker_binary
            assert hashlib.sha256(args.previous_worker_binary.read_bytes()).hexdigest()==db['worker_hash']
            db.setdefault('native_binary_revisions',[]).append(dict(previous=db['worker_hash'],current=worker_hash,
                scope='EXPLICIT_650_850_MM_TOP_LIFT_RECEPTION_NO_GEOMETRY_OR_PERMISSION_CHANGE' if args.raised_top_revision_from else 'DETACHED_OUTFEED_PARK_WITH_NATIVE_PARENT_AND_FROZEN_GOAL_CHECKS' if args.outfeed_park_revision_from else 'FINITE_OUTFEED_ORDER_WITH_NATIVE_PARENT_CHECKS' if args.outfeed_timing_revision_from else 'QUALIFIED_FIXED_BASE_SUPPORT_PLANE_ONLY_WITH_NATIVE_NEGATIVE_GEOMETRY_TESTS'))
            for trial in db['trials']:trial.setdefault('worker_hash',db['worker_hash'])
            db['worker_hash']=worker_hash
        if db['source_hash']!=source_hash:
            previous_path=args.top_short_axis_pose_revision_from or args.top_midpoint_pose_revision_from or args.top_front_pose_revision_from or args.park_clearance_revision_from or args.raised_top_revision_from or args.candidate_fairness_revision_from or args.outfeed_park_revision_from or args.withdraw_distance_revision_from or args.outfeed_timing_revision_from or args.mount_metadata_revision_from or args.initial_witness_revision_from or args.geometry_revision_from or args.numeric_height_revision_from or args.scheduler_revision_from
            assert previous_path
            previous=json.loads(previous_path.read_text());assert digest(previous)==db['source_hash']
            changed={k for k in set(previous['sha256'])|set(manifest['sha256']) if previous['sha256'].get(k)!=manifest['sha256'].get(k)}
            scope='SCHEDULER_ONLY_SAME_NATIVE_BINARY_MODEL_POLICY_AND_ACCEPTED_STATE'
            if args.raised_top_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','src/unloading_sim/joint_search_recipes.py',
                    'ros2/m710_moveit_backend/src/coupled_receiver.h','ros2/m710_moveit_backend/src/joint_search_test.cpp',
                    'tools/lifted_reception/batch_viewer.html'},changed
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'raised-top-revision-reverification.json',reports)
                db['timing_revision_visits']=sorted(reports,key=lambda k:db['layouts'][k]['best_count'],reverse=True)
                for record in db['layouts'].values():
                    if not record.get('nodes'):continue
                    record['closed']=False
                    for node in record['nodes'].values():
                        node.setdefault('prior_visit_cursors',[]).append(dict(source_hash=db['source_hash'],cursor=node['cursor']))
                        node['cursor']=0;node['closed']=False
                scope='EXPLICIT_650_850_MM_TOP_LIFT_RECEPTION_AND_ACCURATE_REPLAY_LABELS_SAME_HMIN_HMAX_GEOMETRY_COLLISION_RULES_ACCEPTED_PARENT_CHAINS_AND_CUMULATIVE_COST'
            elif args.candidate_fairness_revision_from or args.park_clearance_revision_from or args.top_front_pose_revision_from or args.top_midpoint_pose_revision_from or args.top_short_axis_pose_revision_from:
                expected={'tools/plan_m710_joint_search.py'}
                if args.park_clearance_revision_from:expected|={'src/unloading_sim/joint_search_recipes.py','src/unloading_sim/joint_search_outfeed.py'}
                if args.top_front_pose_revision_from or args.top_midpoint_pose_revision_from or args.top_short_axis_pose_revision_from:expected.add('src/unloading_sim/joint_search_recipes.py')
                assert changed==expected,changed
                db['timing_revision_visits']=sorted((key for key,r in db['layouts'].items() if r.get('nodes')),
                    key=lambda key:db['layouts'][key]['best_count'],reverse=True)
                for record in db['layouts'].values():
                    if not record.get('nodes'):continue
                    record['closed']=False
                    for node in record['nodes'].values():
                        node.setdefault('prior_visit_cursors',[]).append(dict(source_hash=db['source_hash'],cursor=node['cursor']))
                        node['cursor']=0;node['closed']=False
                scope='DIAGONAL_POSE_SYNC_DEPARTURE_VISITS_SAME_CANDIDATE_KEYS_NATIVE_BINARY_MODEL_POLICY_ACCEPTED_STATES_AND_CUMULATIVE_COST'
                if args.park_clearance_revision_from:
                    scope='EXPLICIT_100_MM_NORMAL_WITHDRAWAL_BEFORE_NATIVE_PARK_SAME_POSES_GEOMETRY_RULES_GOAL_PARENT_CHAINS_AND_CUMULATIVE_COST'
                if args.top_front_pose_revision_from:
                    scope='FOUR_EXPLICIT_TOP_FRONT_EDGE_POSES_OFFSET_120_240_MM_ROLL_PLUS_MINUS_90_DEG_SAME_NATIVE_GEOMETRY_RULES_PARENT_CHAINS_AND_CUMULATIVE_COST'
                if args.top_midpoint_pose_revision_from:
                    scope='TWO_EXPLICIT_TOP_FRONT_MIDPOINT_POSES_OFFSET_180_MM_ROLL_PLUS_MINUS_90_DEG_SAME_NATIVE_GEOMETRY_RULES_PARENT_CHAINS_AND_CUMULATIVE_COST'
                if args.top_short_axis_pose_revision_from:
                    scope='TWO_EXPLICIT_TOP_FRONT_SHORT_AXIS_POSES_OFFSET_240_MM_ROLL_0_180_DEG_SAME_NATIVE_GEOMETRY_RULES_PARENT_CHAINS_AND_CUMULATIVE_COST'
            elif args.outfeed_park_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','tools/verify_m710_joint_search.py','tools/export_m710_joint_search.py',
                    'src/unloading_sim/joint_search_recipes.py','src/unloading_sim/joint_search_outfeed.py','src/unloading_sim/coupled_batch.py',
                    'src/unloading_sim/stage_motion_policy.py','src/unloading_sim/moveit2_backend.py','src/unloading_sim/single_contact.py',
                    'ros2/m710_moveit_backend/src/ideal_outfeed.h','ros2/m710_moveit_backend/src/worker.cpp',
                    'ros2/m710_moveit_backend/src/joint_search_test.cpp'},changed
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'outfeed-park-revision-reverification.json',reports)
                db['timing_revision_visits']=sorted(reports,key=lambda k:db['layouts'][k]['best_count'],reverse=True)
                for key in reports:
                    record=db['layouts'][key];record['closed']=False
                    for node in record['nodes'].values():
                        node.setdefault('prior_visit_cursors',[]).append(dict(source_hash=db['source_hash'],cursor=node['cursor']))
                        node['cursor']=0;node['closed']=False
                scope='OPTIONAL_DETACHED_NATIVE_PARK_AT_FROZEN_INITIAL_Q_STATIONARY_CARTON_FULL_TOOL_PATH_AND_OUTFEED_CHECKS_NO_COLLISION_PERMISSION_CHANGE'
            elif args.withdraw_distance_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','tools/verify_m710_joint_search.py',
                    'src/unloading_sim/joint_search_recipes.py','src/unloading_sim/joint_search_outfeed.py',
                    'src/unloading_sim/coupled_batch.py'},changed
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'withdraw-distance-revision-reverification.json',reports)
                db['timing_revision_visits']=sorted(reports,key=lambda k:db['layouts'][k]['best_count'],reverse=True)
                for key in reports:
                    record=db['layouts'][key];record['closed']=False
                    for node in record['nodes'].values():
                        node.setdefault('prior_visit_cursors',[]).append(dict(source_hash=db['source_hash'],cursor=node['cursor']))
                        node['cursor']=0;node['closed']=False
                scope='FINITE_ADDITIONAL_100_300_500_MM_NORMAL_WITHDRAWAL_NATIVE_LIN_SAME_WORLD_NO_PERMISSION_OR_NATIVE_BINARY_CHANGE'
            elif args.outfeed_timing_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','tools/verify_m710_joint_search.py',
                    'tools/export_m710_joint_search.py','src/unloading_sim/joint_search_recipes.py',
                    'src/unloading_sim/joint_search_outfeed.py','src/unloading_sim/coupled_batch.py',
                    'ros2/m710_moveit_backend/src/ideal_outfeed.h','ros2/m710_moveit_backend/src/worker.cpp',
                    'ros2/m710_moveit_backend/src/joint_search_test.cpp'},changed
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'outfeed-timing-revision-reverification.json',reports)
                db['timing_revision_visits']=sorted(reports,key=lambda k:db['layouts'][k]['best_count'],reverse=True)
                for key in reports:
                    record=db['layouts'][key];record['closed']=False
                    for node in record['nodes'].values():
                        node.setdefault('prior_visit_cursors',[]).append(dict(source_hash=db['source_hash'],cursor=node['cursor']))
                        node['cursor']=0;node['closed']=False
                scope='FINITE_STATIONARY_CARTON_NATIVE_LIN_WITHDRAWAL_THEN_FULL_CHECKED_OUTFEED_NO_COLLISION_PERMISSION_CHANGE_ALL_ACCEPTED_BRANCHES_REVERIFIED'
            elif args.mount_metadata_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','src/unloading_sim/layout_trajectory.py',
                    'ros2/m710_moveit_backend/src/process_policy.h','ros2/m710_moveit_backend/src/worker.cpp',
                    'ros2/m710_moveit_backend/src/joint_search_test.cpp'},changed
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'mount-metadata-revision-reverification.json',reports)
                for key,record in db['layouts'].items():
                    if record['candidate']['base_xyz_a_m'][2]>.6:
                        record['diagnostics_done']=[]
                        for trial in db['trials']:
                            if trial['layout']==key and not trial['diagnostic'] and 'INVALID_START_CLEARANCE' in str(trial.get('failure')):
                                retries=record['nodes'][trial['node']].setdefault('revision_retries',[])
                                if trial['candidate']['candidate_key'] not in retries:retries.append(trial['candidate']['candidate_key'])
                scope='COPY_EXISTING_QUALIFIED_FIXED_BASE_PLANE_METADATA_TO_MESH_BACKEND_NO_NEW_PERMISSION_OR_GEOMETRY_ACCEPTED_BRANCHES_REVERIFIED'
            elif args.initial_witness_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','tools/plan_m710_mount_comparison.py','tools/verify_m710_joint_search.py'},changed
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'initial-witness-revision-reverification.json',reports)
                retries=db.setdefault('preparation_retries',[])
                for key,record in db['layouts'].items():
                    evidence_path=args.output/key/'geometry.json'
                    if not record.get('nodes') and evidence_path.exists():
                        evidence=json.loads(evidence_path.read_text())
                        if 'refusing to snapshot invalid initial state' in str(evidence.get('detail','')):
                            record['closed']=False
                            if key not in retries:retries.append(key)
                            save(args.output/key/'geometry-before-v6.json',evidence)
                scope='FRESH40_INITIAL_WITNESS_BEFORE_SNAPSHOT_ALL_ACCEPTED_BRANCHES_REVERIFIED_NO_GEOMETRY_POLICY_OR_NATIVE_CHANGE'
            elif args.geometry_revision_from:
                assert changed=={'tools/plan_m710_joint_search.py','src/unloading_sim/joint_search_layout.py',
                    'src/unloading_sim/workcell_layout.py','src/unloading_sim/conveyor_lift.py'},changed
                # Historical accepted worlds use the unchanged original deck and
                # low250 solids. No accepted recessed/compact world is migrated.
                for record in db['layouts'].values():
                    if record.get('nodes'):
                        assert record['candidate']['base_xyz_a_m'][2]==.6
                        assert record['candidate']['mechanism_profile']=='low_profile_250_v1'
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'geometry-revision-reverification.json',reports)
                db['preparation_retries']=[]
                for key,record in db['layouts'].items():
                    evidence_path=args.output/key/'geometry.json'
                    if not record.get('nodes') and evidence_path.exists():
                        evidence=json.loads(evidence_path.read_text())
                        serialization='cannot represent an object' in str(evidence.get('detail',''))
                        seam=any('mount_recess_xplus' in f['pair'] for c in evidence.get('checks',[]) for f in c.get('failures',[]))
                        if serialization or seam:
                            record['closed']=False;db['preparation_retries'].append(key)
                            save(args.output/key/'geometry-before-v5.json',evidence)
                baseline=yaml.safe_load((ROOT/'configs/workcells'/BASELINE).read_text())
                done={LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile']).key
                    for e in db['candidates'][:db['next_candidate']]}
                db['candidates']=db['candidates'][:db['next_candidate']]+[c.evidence() for c in initial_candidates(baseline,args.seed) if c.key not in done]
                scope='SERIALIZATION_FIX_RETAINED_EXTERIOR_SEAM_IDENTITY_NEW_COMPACT_V2_ONLY_ACCEPTED_ORIGINAL_WORLDS_REVERIFIED'
            elif args.numeric_height_revision_from:
                recipe='src/unloading_sim/joint_search_recipes.py'
                assert changed=={'tools/plan_m710_joint_search.py',recipe},changed
                assert previous['sha256'][recipe]=='c6c49566ff24c1b6958f7ea7be8edb4d253f1fdab8b60c3175f1cadb8b4c4ee5'
                assert manifest['sha256'][recipe]=='a19d193c6bcf8586ccd1cdf91ef69c2e6ff03f6b669cd3931dc5ce9d70c22524'
                from verify_m710_joint_search import verify_search
                reports={key:verify_search(args.output,key) for key,r in db['layouts'].items() if r.get('nodes')}
                save(args.output/'numeric-revision-reverification.json',reports)
                for trial in db['trials']:
                    if (trial.get('failure') or {}).get('detail')=='SEARCH_RECEIVE_CONTRACT' and not trial['diagnostic']:
                        node=db['layouts'][trial['layout']]['nodes'][trial['node']]
                        retries=node.setdefault('revision_retries',[])
                        if trial['candidate']['candidate_key'] not in retries:retries.append(trial['candidate']['candidate_key'])
                scope='ONE_NANOMETRE_FLOAT_EQUALITY_TOLERANCE_NO_FORMULA_OR_GEOMETRY_CHANGE_ACCEPTED_BRANCHES_REVERIFIED'
            else:assert changed=={'tools/plan_m710_joint_search.py'},changed
            db.setdefault('source_revisions',[]).append(dict(from_source_hash=db['source_hash'],to_source_hash=source_hash,
                changed_files=sorted(changed),scope=scope))
            for trial in db['trials']:trial.setdefault('source_hash',db['source_hash'])
            db['source_hash']=source_hash
        assert db['contract']==SCHEMA
        db['status']='RUNNING'
    else:
        args.output.mkdir(parents=True,exist_ok=False)
        baseline=yaml.safe_load((ROOT/'configs/workcells'/BASELINE).read_text())
        candidates=[LayoutCandidate(*args.single_candidate[:3],args.single_candidate[3],profile=args.mechanism_profile).evidence()] if args.single_candidate else [c.evidence() for c in initial_candidates(baseline,args.seed)]
        db=dict(contract=SCHEMA,**identity,**FLAGS,source_hash=source_hash,worker_hash=worker_hash,
            options={k:v for k,v in vars(args).items() if isinstance(v,(str,int,float,bool,list))},
            elapsed_search_s=0.,candidates=candidates,next_candidate=0,layouts={},trials=[],pending=None,best=None,
            representative_diagnostics=args.diagnostic_targets,status='RUNNING',preparation_and_compilation_in_search_budget=False,native_call_cost_s={})
        if args.prior_checkpoint:
            previous=json.loads(args.prior_checkpoint.read_text())
            assert previous['contract']==SCHEMA
            db['elapsed_search_s']=previous['elapsed_search_s']
            db['native_call_cost_s']=deepcopy(previous.get('native_call_cost_s',{}))
            db['prior_runs']=[*previous.get('prior_runs',[]),dict(checkpoint=str(args.prior_checkpoint),
                sha256=hashlib.sha256(args.prior_checkpoint.read_bytes()).hexdigest(),source_hash=previous['source_hash'],
                retained_search_cost_s=previous['elapsed_search_s'],ledger_imported=False,reason='CORRECTED_IMPLEMENTATION_RESTART_FRESH40')]
    db.setdefault('refinement_queue',[]);db.setdefault('admissions',db['next_candidate'])
    for record in db['layouts'].values():record.setdefault('admission_kind','COARSE')
    continuation=bind_best_continuation(db,args.continue_from_best)
    db.setdefault('search_sessions',[]).append(dict(started_epoch=time.time(),resume=args.resume,
        prior_elapsed_search_s=db['elapsed_search_s'],budget_hours_cumulative=args.budget_hours,
        trial_seconds=args.trial_seconds,prefix_seconds=args.prefix_seconds,stage_seconds=args.stage_seconds,
        seed=args.seed,source_hash=source_hash,worker_hash=worker_hash,continuation_scope=deepcopy(continuation)))
    session=time.monotonic();prior=db['elapsed_search_s'];deadline=session+max(0,args.budget_hours*3600-prior)
    def checkpoint():
        db['elapsed_search_s']=prior+time.monotonic()-session
        save(cp,db)
    def charge(result):
        for k,v in result['native_call_cost_s'].items():db['native_call_cost_s'][k]=db['native_call_cost_s'].get(k,0.)+v
        if result['implementation_failure']:
            db['status']='STOPPED_IMPLEMENTATION_FAILURE_EVIDENCE_RETAINED';checkpoint()
            raise RuntimeError(str(result['failure']))
    if db.get('pending'):
        pending=db['pending'];pending['status']='INTERRUPTED_RETAINED';pending['charged_s']=max(0,time.time()-pending['started_epoch'])
        pending.update(complete=False,wall_s=pending['charged_s'],failure=dict(reason='INTERRUPTED_RETAINED',global_unreachable=False))
        interrupted=args.output/f"task-{pending['id']:06d}"
        evidence,calls=interrupted_evidence(interrupted,args.output)
        for call in calls:
            label=str(call.get('stage') or 'startup')+'/'+str(call.get('op'))
            db['native_call_cost_s'][label]=db['native_call_cost_s'].get(label,0.)+call.get('round_trip_s',0.)
        if evidence is not None:pending['evidence']=evidence
        db['trials'].append(pending);db['elapsed_search_s']+=pending['charged_s'];prior+=pending['charged_s']
        deadline-=pending['charged_s'];db['pending']=None;checkpoint()
    contexts={};new_since_resume=0
    while time.monotonic()<deadline:
        # Admit a new combined XYZ candidate, then revisit the best retained
        # partial branches. Finite local refinements are appended explicitly.
        resume_rows=[(k,v) for k,v in db['layouts'].items() if v.get('nodes') and not v.get('closed')
            and (continuation is None or k==continuation['layout'])]
        choose_resume=bool(resume_rows) and db['admissions']%3==2 and new_since_resume>0
        if args.max_layouts and db['admissions']>=args.max_layouts:choose_resume=bool(resume_rows)
        if continuation is not None:
            if not resume_rows:break
            choose_resume=True
        if db.get('preparation_retries'):
            key=db['preparation_retries'].pop(0);record=db['layouts'][key];e=record['candidate']
            candidate=LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile'])
        elif db.get('timing_revision_visits'):
            key=db['timing_revision_visits'].pop(0);record=db['layouts'][key];e=record['candidate']
            candidate=LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile'])
        elif choose_resume:
            key,record=max(resume_rows,key=lambda kv:(kv[1]['best_count']-kv[1]['visits']*.1,-kv[1]['cost_s']))
            e=record['candidate'];candidate=LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile']);new_since_resume=0
        else:
            if args.max_layouts and db['admissions']>=args.max_layouts:break
            admitted=admit_candidate(db)
            if admitted is None:break
            e,kind,parent=admitted;new_since_resume+=1
            candidate=LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile']);key=candidate.key
            if key in db['layouts']:continue
            record=dict(candidate=e,nodes={},best_count=0,cost_s=0.,visits=0,diagnostics_done=[],closed=False,
                admission_kind=kind,refinement_parent=parent)
            db['layouts'][key]=record
        if continuation is not None:assert key==continuation['layout'],'CONTINUATION_LAYOUT_CHANGED'
        emit(event='LAYOUT_VISIT',key=key,candidate=e,best=record['best_count']);started=time.monotonic()
        if key not in contexts:
            try:context,evidence=prepare_layout(args,candidate)
            except Exception as exc:
                context=None;evidence=dict(status='PREPARATION_REJECTED',detail=str(exc),exception=traceback.format_exc(),global_unreachable=False)
            folder=args.output/key;folder.mkdir(exist_ok=True);save(folder/'geometry.json',evidence)
            if context is None:
                record.update(closed=True,geometry=evidence['status'],cost_s=record['cost_s']+time.monotonic()-started);checkpoint();continue
            record['geometry']=evidence['status']
            contexts[key]=context
        original,built,frozen,initial=contexts[key];layout=original.policy.layout_validation.layout
        built.joint_search_short_drop_departure_first=continuation is not None
        if not record['nodes']:
            first=node_for(initial,layout);record['nodes'][first['id']]=first;record['initial']=first['root'];record['best_node']=first['id']
            if db['best'] is None:db['best']=key
        record['visits']+=1;until=min(deadline,started+args.layout_slice_seconds)
        # Explicit diagnostic worlds never contribute to any formal ledger.
        if args.diagnostic_seconds>0:
            for name in args.diagnostic_targets:
                if name in record['diagnostics_done'] or time.monotonic()>=until:continue
                target=next(b for b in initial.cartons if b.name==name)
                cartons=tuple(b for b in initial.cartons if b.center[2]<=target.center[2]+1e-9)
                diagnostic=BatchState(frozen,cartons,initial.q.copy(),initial.height,
                    assumed_removed=tuple(sorted(set(frozen.target_ids)-{b.name for b in cartons})))
                scene,visits,screens=offered(original,built,diagnostic,frozen)
                options=[v for v in visits if v[0].target_id==name]
                if options and until-time.monotonic()>=2:
                    t,o=options[0];number=len(db['trials']);folder=args.output/f'task-{number:06d}'
                    db['pending']=dict(id=number,layout=key,target_id=name,started_epoch=time.time(),diagnostic=True,
                        source_hash=source_hash,worker_hash=worker_hash)
                    checkpoint()
                    plan,result=run_trial(args,original,built,diagnostic,t,o,folder,min(args.diagnostic_seconds,until-time.monotonic()),args.seed)
                    archive=archive_task(folder,args.output)
                    db['trials'].append(dict(id=number,layout=key,diagnostic=True,target_id=name,complete=result['complete'],
                        wall_s=result['wall_s'],failure=compact_failure(result['failure']),evidence=archive,cleared_for_diagnosis=sorted(set(frozen.target_ids)-{b.name for b in cartons})))
                    db['pending']=None;charge(result)
                record['diagnostics_done'].append(name);checkpoint()
        while time.monotonic()<until:
            active=continuation_active_nodes(record,None if continuation is None else continuation['anchor_node'])
            if not active:record['closed']=True;break
            # After twelve unsuccessful visits, an adjacent order branch gets
            # a turn. Full-parent-chain alternatives cannot be starved by the
            # thousands of numerical/pose options of a single deepest leaf.
            node=max(active,key=lambda n:(len(n['root']['completed'])-n['cursor']/12.,
                -len(n['children']),len(n['root']['completed'])))
            state=restore_state(node,frozen);scene,visits,screens=offered(original,built,state,frozen)
            if until-time.monotonic()<2:break
            retry=bool(node.get('revision_retries'))
            if retry:
                candidate_key=node['revision_retries'].pop(0)
                target,option=next((t,o) for t,o in visits if o['candidate_key']==candidate_key)
            else:
                if node['cursor']>=len(visits):node['closed']=True;node['unsearched_count']=0;checkpoint();continue
                target,option=visits[node['cursor']];node['cursor']+=1;node['unsearched_count']=len(visits)-node['cursor']
            token=digest(dict(layout=key,root=node['root'],candidate=option['candidate_key'],seed=args.seed,
                **({'numeric_revision':source_hash} if retry else {})))
            if token in node['attempted']:
                assert not retry
                continue
            node['attempted'].append(token)
            number=len(db['trials']);folder=args.output/f'task-{number:06d}'
            db['pending']=dict(id=number,layout=key,node=node['id'],target_id=target.target_id,token=token,
                source_hash=source_hash,worker_hash=worker_hash,
                started_epoch=time.time(),diagnostic=False,reason='CHECKED_IMPLEMENTATION_REVISION_RETRY_COST_RETAINED' if retry else 'NEW_SCENE_OR_Q_OR_HEIGHT' if node['parent'] else 'FRESH40_ROOT')
            checkpoint();emit(event='FORMAL_ATTEMPT',layout=key,completed=len(state.ledger),remaining=len(state.cartons),target=target.target_id,
                face=target.geometric_face_id,mode=option['joint_search_contract']['mode'],pose=option['joint_search_contract']['offset_face_m'],roll=option['joint_search_contract']['roll_rad'])
            plan,result=run_trial(args,original,built,state,target,option,folder,min(args.trial_seconds,until-time.monotonic()),args.seed)
            trial=dict(db['pending'],wall_s=result['wall_s'],complete=result['complete'],failure=compact_failure(result['failure']),
                contact_complete=result['contact_complete'],region_samples=result['region_samples'],candidate=option)
            if plan is not None:
                state.commit(plan);child=node_for(state,layout,node['id'],number)
                record['nodes'][child['id']]=child;node['children'].append(child['id']);trial['child']=child['id']
                if len(state.ledger)>record['best_count']:
                    record['best_count']=len(state.ledger);record['best_node']=child['id']
                    if db['best'] is None or record['best_count']>db['layouts'][db['best']]['best_count']:db['best']=key
            trial['evidence']=archive_task(folder,args.output);db['trials'].append(trial);db['pending']=None
            charge(result)
            emit(event='FORMAL_RESULT',layout=key,complete=trial['complete'],best=record['best_count'],remaining=40-record['best_count'],failure=trial['failure'],wall_s=result['wall_s'])
            checkpoint()
            if record['best_count']==40:
                from verify_m710_joint_search import verify_search
                report=verify_search(args.output,key,require_complete=True);save(args.output/'verification.json',report)
                db['status']='FOUND_VERIFIED_40_OF_40_UNDER_LISTED_ASSUMPTIONS';checkpoint();return
        record['cost_s']+=time.monotonic()-started;checkpoint()
        if record['best_count']>=10 and record['admission_kind']=='COARSE' and not record.get('refinements_added') and not args.single_candidate:
            record['refinements_added']=True
            for resolution in (.025,.010):
                for delta in ((resolution,0,0),(-resolution,0,0),(0,resolution,0),(0,-resolution,0),(0,0,resolution),(0,0,-resolution),
                              (resolution,-resolution,resolution),(-resolution,resolution,-resolution)):
                    xyz=np.asarray(e['base_xyz_a_m'])+delta
                    try:new=LayoutCandidate(*xyz,candidate.minimum,candidate.maximum,candidate.profile);candidate_data(yaml.safe_load((ROOT/'configs/workcells'/BASELINE).read_text()),new)
                    except ValueError:continue
                    db['refinement_queue'].append(dict(candidate=new.evidence(),parent=key,resolution_m=resolution))
            checkpoint()
    db['status']='BUDGET_OR_FINITE_ADMISSION_LIMIT_REACHED_NOT_GLOBAL_INFEASIBILITY';checkpoint()
    from verify_m710_joint_search import verify_search
    if db['best']:save(args.output/'verification.json',verify_search(args.output,db['best']))
    emit(event='SEARCH_STOPPED',status=db['status'],best_layout=db['best'],elapsed_search_s=db['elapsed_search_s'])

if __name__=='__main__':main()
