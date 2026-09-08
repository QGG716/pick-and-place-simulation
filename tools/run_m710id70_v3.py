"""CPU-only, seeded M-710iD/70 V3 acceptance with resumable task evidence."""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import csv
import hashlib
from io import TextIOWrapper
import json
from pathlib import Path
import sys
from zipfile import ZipFile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT))
import numpy as np

from unloading_sim.depalletizing import analyze_box_neighborhood
from unloading_sim.geometry import OBB
from unloading_sim.validation_config import load_validation_config, DEFAULT
from unloading_sim.validation_motion import Cell, evaluate_task
from unloading_sim.validation_physics import RigidAttachment, external_load, aggregate_status
from unloading_sim.validation_receiver import clear_receiving_area
from unloading_sim.validation_scenes import grid_tasks, regular_scene, random_scene, box
from unloading_sim.timing import time_parameterize_joint_path
from tools.capture_m710_evidence import capture

TASK_CACHE_SCHEMA = "m710_task_cache_v2_model_assets"
VALIDATION_STRATEGY_VERSION = "strict_contact_escape_scheduler_v2"


def canonical_digest(value):
    encoded=json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=True)
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


def write_csv(path,rows):
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys or ['status']);writer.writeheader()
        for row in rows:
            writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items()})


def sample_loads(cell,result):
    best=result['selected']
    if best is None:return
    trajectory=time_parameterize_joint_path(best['trajectory']['q_knots'],cell.config.motion_limits())
    stages=best['trajectory']['stages']
    attach_stage='support_release' if 'support_release' in stages else 'extraction'
    attach_t=trajectory.time_from_start[stages[attach_stage][0]]
    release_t=trajectory.time_from_start[stages['carry'][1]]
    attachment=RigidAttachment(np.asarray(best['tcp_from_box']),
        np.asarray(next(a for a in result['attempts'] if a['face']==best['face'] and a['roll_deg']==best['roll_deg'])['extraction']['box_size_xyz_m'])/2,result['box'])
    times=sorted({*np.arange(0,trajectory.duration_seconds,cell.d['execution']['sample_period_s']),*trajectory.time_from_start,
                  *[a+u*(b-a) for a,b in zip(trajectory.time_from_start[:-1],trajectory.time_from_start[1:]) for u in [.5,(3-np.sqrt(3))/6,(3+np.sqrt(3))/6]]})
    rows=[]
    for t in times:
        q,qd,qdd,jerk=trajectory.sample(t)
        attached=attachment if attach_t<=t<=release_t else None
        load=external_load(cell.robot,cell.config.tool,q,attached,cell.d['scene']['box_mass_kg'],cell.d['scene']['box_com_fraction'],cell.config.model,qd,qdd)
        rows.append({'t_s':float(t),'q_rad':q.tolist(),'qd_rad_s':qd.tolist(),'qdd_rad_s2':qdd.tolist(),'jerk_rad_s3':jerk.tolist(),
                     'attached':attached is not None,'load':load})
    best['trajectory']['samples']=rows
    result['load_status']=aggregate_status(row['load']['qualification'] for row in rows)
    best['load_status']=result['load_status']


def compact(task_id,result,**extra):
    best=result['selected'] or {}
    reasons=Counter(a['reason'] for a in result['attempts'])
    details=Counter(a['failure_taxonomy']['detail'] for a in result['attempts'] if 'failure_taxonomy' in a)
    return {'task_id':task_id,**extra,'box':result['box'],'seed':result['seed'],'mode':result['mode'],
            'GRASP_REACHABLE':result['grasp_reachable'],'EXTRACTION_FEASIBLE':result['extraction_feasible'],
            'GEOMETRICALLY_REACHABLE':result['geometric_feasible'],'PAYLOAD_QUALIFIED':result['payload_qualified'],
            'DYNAMICS_VERIFIED':result['dynamics_verified'],'load_status':result['load_status'],
            'failure_stage':result['failure_stage'],'failure_reason':result['failure_reason'],
            'candidate_failure_counts':dict(reasons),'grasp_failure_detail_counts':dict(details),
            'face':best.get('face'),'roll_deg':best.get('roll_deg'),
            'escape_search_termination':best.get('escape_search',{}).get('termination'),
            'escape_robot_validations':len(best.get('escape_attempts',[])),
            'loaded_tcp_path_m':best.get('loaded_tcp_path_m'),'cycle_s':best.get('cycle_s'),
            'conveyor_action_s':best.get('conveyor_action_s')}


def initial_proximity_recovery_row(task_id,result):
    """Summarize the task-level outcome without counting gate passage as success."""
    attempts=[a for a in result['attempts'] if a.get('initial_proximity',{}).get('pairs')]
    pair_names=sorted({pair['obstacle'] for attempt in attempts
                       for pair in attempt['initial_proximity']['pairs']})
    path_evidence=[sub['initial_proximity'] for attempt in attempts
                   for sub in attempt.get('conveyor_attempts',[])
                   if 'initial_proximity' in sub]
    gate_failures={'PAYLOAD_INITIAL_CLEARANCE_FAILED','PAYLOAD_INITIAL_PENETRATION'}
    passed_initial_gate=any(attempt.get('reason') not in gate_failures for attempt in attempts)
    fully_released=any(evidence['fully_released'] for evidence in path_evidence)
    if result['geometric_feasible']:
        classification='COMPLETE_GEOMETRIC_SUCCESS'
    elif fully_released:
        classification='FULL_MARGIN_RESTORED_DOWNSTREAM_FAILED'
    elif path_evidence:
        classification='SEPARATION_ATTEMPTED_NOT_RELEASED'
    elif passed_initial_gate:
        classification='INITIAL_GATE_PASSED_FAILED_BEFORE_SEPARATION'
    elif attempts:
        classification='INITIAL_PROXIMITY_REJECTED'
    else:
        classification='NO_REGISTERED_INITIAL_PROXIMITY'
    return {'task_id':task_id,'registered_initial_proximity':bool(attempts),
            'registered_neighbors':pair_names,'passed_initial_gate':passed_initial_gate,
            'separation_attempted':bool(path_evidence),'normal_margin_restored':fully_released,
            'complete_geometric_success':result['geometric_feasible'],
            'classification':classification,'final_failure_stage':result['failure_stage'],
            'final_failure_reason':result['failure_reason']}


def grasp_task_set_recovery_row(task_id,result):
    attempts=result['attempts']
    nominal=[a for a in attempts if a.get('task_set',{}).get('variant')=='nominal']
    expanded=[a for a in attempts if a.get('task_set',{}).get('variant')!='nominal']
    # The marker is written only after strict IK/FK, actual-FK suction
    # coverage, robot/tool collision and initial attachment clearance pass.
    nominal_valid=any(a.get('strict_grasp_valid',False) for a in nominal)
    non_nominal_valid=any(a.get('strict_grasp_valid',False) for a in expanded)
    task_set_valid=nominal_valid or non_nominal_valid
    details=Counter(a['failure_taxonomy']['detail'] for a in attempts if 'failure_taxonomy' in a)
    return {'task_id':task_id,'nominal_strict_grasp_valid':nominal_valid,
            'non_nominal_strict_grasp_valid':non_nominal_valid,
            'expanded_task_set_strict_grasp_valid':task_set_valid,
            'task_set_recovered_grasp':task_set_valid and not nominal_valid,
            'grasp_reachable':result['grasp_reachable'],
            'complete_geometric_success':result['geometric_feasible'],
            'final_failure_stage':result['failure_stage'],'final_failure_reason':result['failure_reason'],
            'failure_detail_counts':dict(details)}


def frozen_v3_task_failures():
    """Load task-level V3 classifications from the immutable evidence bundle."""
    archive=ROOT/'docs/validation/evidence/m710id70_v3_evidence.zip'
    with ZipFile(archive) as bundle, bundle.open('v3/task_reachability.csv') as raw:
        rows=csv.DictReader(TextIOWrapper(raw,encoding='utf-8-sig',newline=''))
        return {row['task_id'].removesuffix('_dynamic'):row['failure_reason']
                for row in rows if row['task_id'].endswith('_dynamic')}


def frozen_recovery_summary(rows):
    """Compare task-set results to task identities in the frozen V3 run."""
    baseline=frozen_v3_task_failures()
    groups={}
    for reason in ('GRASP_CONSTRAINT_FAILED','NO_IK','PAYLOAD_INITIAL_CLEARANCE_FAILED'):
        selected=[row for row in rows if baseline.get(row['task_id'])==reason]
        recovered=[row for row in selected if row['expanded_task_set_strict_grasp_valid']]
        task_details=Counter(detail for row in selected for detail in row['failure_detail_counts'])
        candidate_details=sum((Counter(row['failure_detail_counts']) for row in selected),Counter())
        groups[reason]={
            'baseline_tasks':len(selected),
            'task_set_strict_grasp_valid_tasks':sum(row['expanded_task_set_strict_grasp_valid'] for row in selected),
            'task_set_recovered_tasks':len(recovered),
            'recovered_task_ids':[row['task_id'] for row in recovered],
            'final_task_status_counts':dict(Counter(row['final_failure_reason'] for row in selected)),
            'task_failure_detail_presence_counts':dict(task_details),
            'candidate_failure_detail_counts':dict(candidate_details),
        }
    return groups


class Run:
    def __init__(self,cfg,output,workers=1):
        self.cfg=cfg;self.output=output;self.count=0;self.workers=workers
        self.cache_identity=cfg.cache_identity()
        self.configuration_digest=canonical_digest(self.cache_identity)
        # Untracked implementation files must also invalidate cached evidence.
        source_paths=sorted([*ROOT.joinpath('src/unloading_sim').rglob('*.py'),Path(__file__)])
        self.code_manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in source_paths}
        self.code_digest=canonical_digest(self.code_manifest)

    def task_identity(self,cell,target,remaining,q,belt,seed,mode,only_face,grasp_only):
        key={'cache_schema_version':TASK_CACHE_SCHEMA,
             'validation_strategy_version':VALIDATION_STRATEGY_VERSION,
             'code':self.code_digest,'config':self.configuration_digest,
             'model_asset_fingerprint_sha256':self.cfg.asset_manifest['semantic_fingerprint_sha256'],
             'cache_validity_runtime':self.cache_identity['cache_validity_runtime'],
             'height':cell.robot.base_transform.tolist(),
             'target':target.name,'boxes':[{'name':b.name,'pose':b.world_from_local.tolist(),'half':b.half_extents.tolist()} for b in remaining],
             'q':np.asarray(q).tolist(),'belt':list(belt),'seed':int(seed),'mode':mode,
             'only_face':only_face,'grasp_only':grasp_only}
        return key,canonical_digest(key)

    @staticmethod
    def cached_result(path,digest,asset_fingerprint):
        if not path.exists():return None
        saved=json.loads(path.read_text(encoding='utf-8'))
        result=saved.get('result',{})
        evidence=result.get('validation_evidence',{}) if isinstance(result,dict) else {}
        if (saved.get('cache_schema_version')!=TASK_CACHE_SCHEMA
                or saved.get('input_digest')!=digest
                or evidence.get('model_asset_fingerprint_sha256')!=asset_fingerprint):
            return None
        return result

    def task(self,task_id,cell,target,remaining,q,belt,seed,mode='dynamic',only_face=None,grasp_only=False):
        key,digest=self.task_identity(cell,target,remaining,q,belt,seed,mode,only_face,grasp_only)
        path=self.output/'tasks'/f'{task_id}.json'
        cached=self.cached_result(path,digest,self.cfg.asset_manifest['semantic_fingerprint_sha256'])
        if cached is not None:return cached
        result=evaluate_task(cell,target,remaining,q,belt,seed=seed,mode=mode,only_face=only_face,
                             grasp_only=grasp_only)
        if not grasp_only:sample_loads(cell,result)
        result['validation_evidence']={
            'task_cache_schema_version':TASK_CACHE_SCHEMA,
            'validation_strategy_version':VALIDATION_STRATEGY_VERSION,
            'model_asset_fingerprint_sha256':self.cfg.asset_manifest['semantic_fingerprint_sha256'],
            'configuration_digest':self.configuration_digest,'code_digest':self.code_digest,
            'cache_validity_runtime':self.cache_identity['cache_validity_runtime']}
        write_json(path,{'cache_schema_version':TASK_CACHE_SCHEMA,'input_digest':digest,
                         'inputs':key,'result':result})
        self.count+=1
        if self.count%5==0:
            print(json.dumps({'completed_tasks':self.count,'last':task_id,'success':result['geometric_feasible'],'failure':result['failure_reason']}),flush=True)
        return result

    def batch(self,jobs):
        if self.workers==1:
            return [_task_job((self.cfg,self.output,job)) for job in jobs]
        with ProcessPoolExecutor(max_workers=self.workers) as pool:
            results=[]
            for index,result in enumerate(pool.map(_task_job,[(self.cfg,self.output,job) for job in jobs])):
                results.append(result)
                if index%5==0:print(json.dumps({'batch_completed':index+1,'batch_total':len(jobs),'task':jobs[index][0]}),flush=True)
            return results


def _task_job(payload):
    cfg,output,job=payload
    identifier,height,target,remaining,q,belt,seed,mode,*extra=job
    return Run(cfg,output).task(identifier,Cell(cfg,height),target,remaining,q,belt,seed,mode,
                                grasp_only=bool(extra and extra[0]))


def small_scenes(cfg):
    s=cfg.data['scene'];size=list(s['box_sizes_m'].values())[0]
    return [('last_bottom',[box('last_bottom',[s['front_x_m']+size[0]/2,0,size[2]/2],size)]),
            ('side_control',[box('side_control',[s['front_x_m']+size[0]/2,0,.9],size)]),
            ('small_stack',[box('small_bottom',[s['front_x_m']+size[0]/2,0,size[2]/2],size),
                            box('small_top',[s['front_x_m']+size[0]/2,0,size[2]/2+s['row_pitch_m']],size)]),
            ('side_edge',[box('side_edge',[s['front_x_m']+size[0]/2,.82,.9],size)])]


def continuous(run,name,original,scenario_index):
    cell=Cell(run.cfg);q=np.asarray(cell.d['robot']['home_joints']);belt=(cell.d['conveyor']['fixed_extension_m'],cell.d['conveyor']['fixed_z_m'])
    remaining=list(original);received=[];events=[];sequence=0;rows=[]
    while remaining:
        exposed=[]
        for target in remaining:
            topology=analyze_box_neighborhood(target,[b for b in remaining if b.name!=target.name])
            if topology.top_neighbor is None and topology.exposed_face_count:
                exposed.append((-(topology.exposed_face_count),target.name,target))
        successes=[]
        # Exhaust every exposed target, not just the two best ranked boxes.
        jobs=[];targets=[]
        for index,(_,_,target) in enumerate(sorted(exposed,key=lambda t:t[:2])):
            seed=cell.p['continuous_seed']+scenario_index*1000+sequence*50+index
            identifier=f'{name}_{sequence:03}_{target.name}'
            jobs.append((identifier,None,target,[*remaining,*received],q,belt,seed,'dynamic'));targets.append(target)
        for target,job,result in zip(targets,jobs,run.batch(jobs)):
            identifier=job[0]
            rows.append(compact(identifier,result,scenario=name,sequence=sequence))
            if result['geometric_feasible']:
                successes.append((result['selected']['cycle_s'],target.name,target,result))
        if not successes:break
        _,_,target,result=min(successes,key=lambda t:t[:2]);best=result['selected']
        pose=np.asarray(best['placed_box_pose'])
        placed=OBB(pose[:3,3],target.half_extents,pose[:3,:3],f'received_{target.name}','received')
        q=np.asarray(best['final_q']);belt=tuple(best['state'])
        remaining=[b for b in remaining if b.name!=target.name]
        moved,transfer=clear_receiving_area(cell,placed,belt,q,remaining,received)
        received.append(placed if moved is None else moved)
        events.append({'sequence':sequence,'box':target.name,'event':'SUPPORTED_RELEASE_THEN_EMPTY_WITHDRAWAL',
                       'receiver_state':'OCCUPIED','received_boxes':[b.name for b in received],
                       'belt_transport':transfer,
                       'placed_box_pose':best['placed_box_pose'],'q_after_withdrawal':q.tolist()})
        sequence+=1
        # The current L-corner has no verified outgoing support path for these
        # boxes. Keep its occupant and stop, not delete it to fabricate cycles.
        if moved is None:
            events.append({'event':'RECEIVER_BLOCKED','reason':transfer['reason'],'retained_box':received[-1].name})
            break
    for target in remaining:
        topology=analyze_box_neighborhood(target,[b for b in remaining if b.name!=target.name])
        rows.append({'task_id':f'{name}_remaining_{target.name}','scenario':name,'box':target.name,'status':'REMAINING',
                     'failure_stage':'support_order' if topology.top_neighbor else 'planning',
                     'failure_reason':f'SUPPORTS_{topology.top_neighbor}' if topology.top_neighbor else 'ALL_RECORDED_CANDIDATES_FAILED_OR_RECEIVER_BLOCKED'})
    summary={'scenario':name,'total_boxes':len(original),'geometric_cycles':sequence,'geometrically_unloaded_boxes':sequence,
             'payload_qualified_unloaded_boxes':0,'dynamics_verified_boxes':0,'remaining_boxes':len(remaining),
             'received_boxes_retained':len(received),'events':events,
             'initial_home_failure':cell.state_failure(np.asarray(cell.d['robot']['home_joints']),[*cell.fixtures(),*original,*cell.decks((0,.2))])}
    write_json(run.output/f'{name}_continuous.json',{'summary':summary,'rows':rows})
    return summary,rows


def grid(run):
    cell=Cell(run.cfg);s=cell.d['scene'];rows=[];paired=[];proximity=[];grasp_recovery=[];recovery=[];extraction_metrics=[];q=np.asarray(cell.d['robot']['home_joints']);c=cell.d['conveyor'];belt=(c['fixed_extension_m'],c['fixed_z_m'])
    tasks=list(grid_tasks(s))
    jobs=[(f'grid_{index:03}_{mode}',None,target,[target,*neighbors],q,belt,cell.p['seed']+index,mode)
          for index,(_,target,neighbors,valid) in enumerate(tasks) if valid for mode in ('dynamic','fixed')]
    completed=dict(zip((job[0] for job in jobs),run.batch(jobs)))
    for index,(orientation,target,neighbors,valid) in enumerate(tasks):
        extra={'y_m':float(target.center[1]),'z_m':float(target.center[2]),'orientation':orientation,'inside_cross_section':valid}
        if not valid:
            rows.append({'task_id':f'grid_{index:03}',**extra,'GEOMETRICALLY_REACHABLE':False,'GRASP_REACHABLE':False,'EXTRACTION_FEASIBLE':False,'PAYLOAD_QUALIFIED':False,'DYNAMICS_VERIFIED':False,'failure_reason':'BOX_OUTSIDE_TRAILER'})
            continue
        results={}
        # Identical targets, neighbors, home, tolerances, RNG stream and planner.
        for mode in ('dynamic','fixed'):
            result=completed[f'grid_{index:03}_{mode}']
            results[mode]=result
        rows.append(compact(f'grid_{index:03}_dynamic',results['dynamic'],**extra))
        proximity.append(initial_proximity_recovery_row(f'grid_{index:03}',results['dynamic']))
        grasp_recovery.append(grasp_task_set_recovery_row(f'grid_{index:03}',results['dynamic']))
        recovery.append({'task_id':f'grid_{index:03}','fixed_feasible':results['fixed']['geometric_feasible'],
                         'dynamic_feasible':results['dynamic']['geometric_feasible'],
                         'any_mode_feasible':results['fixed']['geometric_feasible'] or results['dynamic']['geometric_feasible']})
        for mode,result in results.items():
            for attempt_index,attempt in enumerate(result['attempts']):
                for option_index,option in enumerate(attempt.get('conveyor_attempts',[])):
                    if 'extraction_metrics' not in option:continue
                    extraction_metrics.append({'task_id':f'grid_{index:03}','mode':mode,
                        'attempt_index':attempt_index,'option_index':option_index,'face':attempt['face'],
                        'roll_deg':attempt['roll_deg'],'task_set_variant':attempt['task_set']['variant'],
                        'result':option['reason'],
                        'escape_search_termination':option.get('escape_search',{}).get('termination'),
                        'escape_robot_validations':len(option.get('escape_attempts',[])),
                        'escape_robot_validation_budget':option.get('escape_search',{}).get('robot_validation_budget'),
                        **option['extraction_metrics']})
        a,b=results['fixed'],results['dynamic'];common=a['geometric_feasible'] and b['geometric_feasible']
        paired.append({'task_id':f'grid_{index:03}','fixed_feasible':a['geometric_feasible'],'dynamic_feasible':b['geometric_feasible'],
                       'new_feasible':b['geometric_feasible'] and not a['geometric_feasible'],'lost_feasible':a['geometric_feasible'] and not b['geometric_feasible'],
                       'common_feasible':common,'loaded_path_delta_m':None if not common else b['selected']['loaded_tcp_path_m']-a['selected']['loaded_tcp_path_m'],
                       'cycle_delta_s':None if not common else b['selected']['cycle_s']-a['selected']['cycle_s'],
                       'dynamic_action_s':None if not b['selected'] else b['selected']['conveyor_action_s'],
                       'fixed_action_s':None if not a['selected'] else a['selected']['conveyor_action_s']})
    affected=[row for row in proximity if row['registered_initial_proximity']]
    proximity_summary={
        'denominator':len(proximity),
        'frozen_v3_reference':{'failure_reason':'PAYLOAD_INITIAL_CLEARANCE_FAILED','task_count':24,
            'source':'docs/validation/technical_qualification_report_m710id70_v3.md'},
        'registered_initial_proximity_tasks':len(affected),
        'passed_initial_gate_tasks':sum(row['passed_initial_gate'] for row in affected),
        'separation_attempted_tasks':sum(row['separation_attempted'] for row in affected),
        'normal_margin_restored_tasks':sum(row['normal_margin_restored'] for row in affected),
        'complete_geometric_success_tasks':sum(row['complete_geometric_success'] for row in affected),
        'classification_counts':dict(Counter(row['classification'] for row in affected)),
        'downstream_failure_counts':dict(Counter(row['final_failure_reason'] for row in affected
                                                  if not row['complete_geometric_success'])),
        'interpretation':'Passing the initial-proximity gate is not a complete geometric success.',
    }
    grasp_summary={
        'denominator':len(grasp_recovery),
        'frozen_v3_reference':{'GRASP_CONSTRAINT_FAILED':44,'NO_IK':36,
            'source':'docs/validation/technical_qualification_report_m710id70_v3.md'},
        'nominal_strict_grasp_valid_tasks':sum(row['nominal_strict_grasp_valid'] for row in grasp_recovery),
        'expanded_task_set_strict_grasp_valid_tasks':sum(row['expanded_task_set_strict_grasp_valid'] for row in grasp_recovery),
        'task_set_recovered_grasp_tasks':sum(row['task_set_recovered_grasp'] for row in grasp_recovery),
        'grasp_reachable_tasks':sum(row['grasp_reachable'] for row in grasp_recovery),
        'complete_geometric_success_tasks':sum(row['complete_geometric_success'] for row in grasp_recovery),
        'candidate_failure_detail_counts':dict(sum((Counter(row['failure_detail_counts']) for row in grasp_recovery),Counter())),
        'final_task_failure_counts':dict(Counter(row['final_failure_reason'] for row in grasp_recovery)),
        'interpretation':'Search exhaustion is not proof of task-space infeasibility; every recovered grasp passed strict FK and constraints.',
    }
    recovery_summary={'denominator':len(recovery),
        'fixed_complete_geometric_successes':sum(row['fixed_feasible'] for row in recovery),
        'dynamic_complete_geometric_successes':sum(row['dynamic_feasible'] for row in recovery),
        'any_mode_complete_geometric_successes':sum(row['any_mode_feasible'] for row in recovery),
        'fixed_success_task_ids':[row['task_id'] for row in recovery if row['fixed_feasible']],
        'dynamic_success_task_ids':[row['task_id'] for row in recovery if row['dynamic_feasible']],
        'denominator_and_scenes_unchanged':True,
        'performance_optimization_status':'DEFERRED_UNTIL_STABLE_NONZERO_COMMON_SET'}
    write_csv(run.output/'task_reachability.csv',rows);write_csv(run.output/'conveyor_ab.csv',paired)
    write_csv(run.output/'initial_proximity_recovery.csv',proximity)
    write_json(run.output/'initial_proximity_recovery.json',proximity_summary)
    write_csv(run.output/'grasp_task_set_recovery.csv',grasp_recovery)
    write_json(run.output/'grasp_task_set_recovery.json',grasp_summary)
    write_csv(run.output/'extraction_path_metrics.csv',extraction_metrics)
    write_csv(run.output/'geometric_recovery.csv',recovery)
    write_json(run.output/'geometric_recovery.json',recovery_summary)
    return rows,paired


def grasp_scan(run):
    """Evaluate each original task's complete grasp task set exactly once."""
    cell=Cell(run.cfg);s=cell.d['scene'];q=np.asarray(cell.d['robot']['home_joints'])
    c=cell.d['conveyor'];belt=(c['fixed_extension_m'],c['fixed_z_m']);tasks=list(grid_tasks(s))
    jobs=[(f'grasp_{index:03}',None,target,[target,*neighbors],q,belt,cell.p['seed']+index,
           'grasp_only',True)
          for index,(_,target,neighbors,valid) in enumerate(tasks) if valid]
    completed=dict(zip((job[0] for job in jobs),run.batch(jobs)))
    rows=[];baseline=frozen_v3_task_failures()
    for index,(_,target,neighbors,valid) in enumerate(tasks):
        if not valid:continue
        row=grasp_task_set_recovery_row(f'grid_{index:03}',completed[f'grasp_{index:03}'])
        row['frozen_v3_failure_reason']=baseline.get(row['task_id'])
        row['recovered_from_frozen_v3']=row['expanded_task_set_strict_grasp_valid'] and \
            row['frozen_v3_failure_reason'] in {'GRASP_CONSTRAINT_FAILED','NO_IK'}
        rows.append(row)
    summary={
        'denominator':len(rows),
        'frozen_v3_reference':{'strict_grasp_valid':24,'GRASP_CONSTRAINT_FAILED':44,'NO_IK':36,
            'source':'docs/validation/technical_qualification_report_m710id70_v3.md'},
        'nominal_strict_grasp_valid_tasks':sum(row['nominal_strict_grasp_valid'] for row in rows),
        'expanded_task_set_strict_grasp_valid_tasks':sum(row['expanded_task_set_strict_grasp_valid'] for row in rows),
        'task_set_recovered_grasp_tasks':sum(row['task_set_recovered_grasp'] for row in rows),
        'grasp_reachable_tasks':sum(row['grasp_reachable'] for row in rows),
        'final_task_status_counts':dict(Counter(row['final_failure_reason'] for row in rows)),
        'candidate_failure_detail_counts':dict(sum((Counter(row['failure_detail_counts']) for row in rows),Counter())),
        'recovery_by_frozen_v3_failure':frozen_recovery_summary(rows),
        'search_scope':'strict grasp qualification only; no extraction/transit/place success is inferred',
    }
    write_csv(run.output/'grasp_task_set_recovery.csv',rows)
    write_json(run.output/'grasp_task_set_recovery.json',summary)
    return rows,summary


def fixed_geometry_scan(run):
    """Run the frozen 104-task population without reactivating P1 A/B work."""
    cell=Cell(run.cfg);s=cell.d['scene'];q=np.asarray(cell.d['robot']['home_joints'])
    c=cell.d['conveyor'];belt=(c['fixed_extension_m'],c['fixed_z_m']);tasks=list(grid_tasks(s))
    jobs=[(f'grid_{index:03}_fixed',None,target,[target,*neighbors],q,belt,cell.p['seed']+index,'fixed')
          for index,(_,target,neighbors,valid) in enumerate(tasks) if valid]
    completed=dict(zip((job[0] for job in jobs),run.batch(jobs)))
    rows=[];proximity=[];grasp=[];metrics=[]
    for index,(orientation,target,neighbors,valid) in enumerate(tasks):
        if not valid:continue
        task_id=f'grid_{index:03}';result=completed[f'{task_id}_fixed']
        rows.append(compact(task_id,result,y_m=float(target.center[1]),z_m=float(target.center[2]),
                            orientation=orientation,inside_cross_section=True))
        proximity.append(initial_proximity_recovery_row(task_id,result))
        grasp.append(grasp_task_set_recovery_row(task_id,result))
        for attempt_index,attempt in enumerate(result['attempts']):
            for option_index,option in enumerate(attempt.get('conveyor_attempts',[])):
                if 'extraction_metrics' in option:
                    metrics.append({'task_id':task_id,'mode':'fixed','attempt_index':attempt_index,
                        'option_index':option_index,'face':attempt['face'],'roll_deg':attempt['roll_deg'],
                        'task_set_variant':attempt['task_set']['variant'],'result':option['reason'],
                        'escape_search_termination':option.get('escape_search',{}).get('termination'),
                        'escape_robot_validations':len(option.get('escape_attempts',[])),
                        'escape_robot_validation_budget':option.get('escape_search',{}).get('robot_validation_budget'),
                        **option['extraction_metrics']})
    successful=[row['task_id'] for row in rows if row['GEOMETRICALLY_REACHABLE']]
    frozen=frozen_v3_task_failures()
    frozen_clearance=[row for row in proximity
                      if frozen.get(row['task_id'])=='PAYLOAD_INITIAL_CLEARANCE_FAILED']
    selected=[completed[f'{task_id}_fixed']['selected'] for task_id in successful]
    summary={'denominator':len(rows),'complete_geometric_successes':len(successful),
        'complete_geometric_success_task_ids':successful,
        'failure_reason_counts':dict(Counter(row['failure_reason'] for row in rows)),
        'grasp_reachable_tasks':sum(row['GRASP_REACHABLE'] for row in rows),
        'extraction_feasible_tasks':sum(row['EXTRACTION_FEASIBLE'] for row in rows),
        'success_grasp_face_counts':dict(Counter(option['face'] for option in selected)),
        'successes_requiring_support_release':sum(option['support_release']['required'] for option in selected),
        'p1_dynamic_conveyor_optimization':'NOT_RUN_DEFERRED',
        'denominator_and_scenes_unchanged':True}
    proximity_summary={'frozen_v3_baseline_tasks':len(frozen_clearance),
        'passed_initial_gate_tasks':sum(row['passed_initial_gate'] for row in frozen_clearance),
        'separation_attempted_tasks':sum(row['separation_attempted'] for row in frozen_clearance),
        'normal_margin_restored_tasks':sum(row['normal_margin_restored'] for row in frozen_clearance),
        'complete_geometric_success_tasks':sum(row['complete_geometric_success'] for row in frozen_clearance),
        'classification_counts':dict(Counter(row['classification'] for row in frozen_clearance)),
        'downstream_failure_counts':dict(Counter(row['final_failure_reason'] for row in frozen_clearance
                                                  if not row['complete_geometric_success'])),
        'interpretation':'Initial gate passage and margin restoration are not counted as complete success.'}
    grasp_summary={'denominator':len(grasp),
        'nominal_strict_grasp_valid_tasks':sum(row['nominal_strict_grasp_valid'] for row in grasp),
        'expanded_task_set_strict_grasp_valid_tasks':sum(row['expanded_task_set_strict_grasp_valid'] for row in grasp),
        'complete_geometric_success_tasks':sum(row['complete_geometric_success'] for row in grasp),
        'recovery_by_frozen_v3_failure':frozen_recovery_summary(grasp)}
    write_csv(run.output/'fixed_task_reachability.csv',rows)
    write_csv(run.output/'fixed_initial_proximity_recovery.csv',proximity)
    write_json(run.output/'fixed_initial_proximity_recovery.json',proximity_summary)
    write_csv(run.output/'fixed_grasp_task_set_recovery.csv',grasp)
    write_json(run.output/'fixed_grasp_task_set_recovery.json',grasp_summary)
    write_csv(run.output/'fixed_extraction_path_metrics.csv',metrics)
    write_json(run.output/'fixed_geometric_recovery.json',summary)
    return rows,summary


def lift_scan(run):
    cfg=run.cfg;d=cfg.data;home=np.asarray(d['robot']['home_joints']);heights=d['lift']['scan_heights_m'];rows=[];motion=[]
    tasks=list(grid_tasks(d['scene']));jobs=[];job_rows=[]
    # Static robot/lift states are reused across target cases. The obstacle
    # checks still run for each unchanged task population.
    sweep_cells={}
    for height in heights:
        z0=d['lift']['height_m'];steps=max(1,int(np.ceil(abs(height-z0)/d['lift']['step_m'])))
        sweep_cells[height]=[(float(z),Cell(cfg,float(z))) for z in np.linspace(z0,height,2*steps+1)]
    for index,(orientation,target,neighbors,valid) in enumerate(tasks):
        if not valid:continue
        remaining=[target,*neighbors]
        for height in heights:
            cell=Cell(cfg,height);c=d['conveyor'];belt=(c['fixed_extension_m'],c['fixed_z_m'])
            failure=None;z0=d['lift']['height_m']
            steps=max(1,int(np.ceil(abs(height-z0)/d['lift']['step_m'])))
            for z,moving in sweep_cells[height]:
                failure=moving.state_failure(home,[*moving.fixtures(),*remaining,*moving.decks(belt)])
                if failure:
                    failure={**failure,'lift_height_m':float(z)};break
            motion.append({'task_id':f'grid_{index:03}','height_m':height,'lift_path_valid':failure is None,'failure':failure,
                           'lift_action_s':abs(height-z0)/d['lift']['speed_m_s']})
            if failure:
                rows.append({'task_id':f'grid_{index:03}','height_m':height,'full_task_feasible':False,'failure':failure})
                continue
            identifier=f'grid_{index:03}_dynamic' if height==z0 else f'lift_{height:.2f}_{index:03}'
            jobs.append((identifier,height,target,remaining,home,belt,d['planning']['seed']+index,'dynamic'))
            job_rows.append({'task_id':f'grid_{index:03}','height_m':height})
    for row,result in zip(job_rows,run.batch(jobs)):
        rows.append({**row,'full_task_feasible':result['geometric_feasible'],
                     'failure':None if result['geometric_feasible'] else result['failure_reason']})
    rows.sort(key=lambda row:(row['task_id'],row['height_m']))
    coverage={str(h):sum(r['full_task_feasible'] for r in rows if r['height_m']==h) for h in heights}
    union=sorted({r['task_id'] for r in rows if r['full_task_feasible']})
    result={'coverage_by_height':coverage,'coverage_union_task_ids':union,'union_count':len(union),
            'denominator':sum(valid for *_,valid in tasks),'travel_recommendation':'NOT_EVALUATED_FROM_DISCRETE_COVERAGE_ALONE',
            'mechanical_validation':'NOT_EVALUATED_STIFFNESS_DRIVE_BRAKE_AND_STABILITY_DATA_MISSING'}
    write_csv(run.output/'lift_full_task_feasibility.csv',rows);write_csv(run.output/'lift_motion_validation.csv',motion)
    write_json(run.output/'lift_coverage_union.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs/m710id70_v3/v3')
    parser.add_argument('--phase',choices=['small','grasp','grid-fixed','grid','continuous','lift','all'],default='all')
    parser.add_argument('--workers',type=int,default=4,help='Independent CPU processes; task seeds and outputs are unchanged')
    args=parser.parse_args();cfg=load_validation_config(args.config);out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    capture(out,'python tools/run_m710id70_v3.py '+' '.join(sys.argv[1:]));write_json(out/'effective_config.json',cfg.evidence())
    run=Run(cfg,out,args.workers)
    if args.phase=='small':
        summaries=[];rows=[]
        for i,(name,scene) in enumerate(small_scenes(cfg)):
            summary,log=continuous(run,name,scene,10+i);summaries.append(summary);rows+=log
        # Explicitly force side candidates in the controlled witness search.
        cell=Cell(cfg);target=small_scenes(cfg)[1][1][0];c=cfg.data['conveyor']
        for face in ('left','right'):
            result=run.task(f'controlled_{face}',cell,target,[target],np.asarray(cfg.data['robot']['home_joints']),
                            (c['fixed_extension_m'],c['fixed_z_m']),cfg.data['planning']['seed'],only_face=face)
            rows.append(compact(f'controlled_{face}',result,scenario='side_control_forced'))
        bottom=small_scenes(cfg)[0][1][0]
        result=run.task('controlled_bottom_fixed',cell,bottom,[bottom],np.asarray(cfg.data['robot']['home_joints']),
                        (c['fixed_extension_m'],c['fixed_z_m']),cfg.data['planning']['seed'],mode='fixed')
        rows.append(compact('controlled_bottom_fixed',result,scenario='bottom_fixed_belt_alternative'))
        write_csv(out/'small_scenes.csv',rows);write_json(out/'small_summary.json',summaries)
    if args.phase in ('grasp','all'):grasp_scan(run)
    if args.phase in ('grid-fixed','all'):fixed_geometry_scan(run)
    if args.phase=='grid':grid(run)
    if args.phase=='continuous':
        summaries=[];rows=[];s=cfg.data['scene']
        scenes=[('regular',regular_scene(s)),*[(f'random_seed_{seed}',random_scene(s,seed)) for seed in s['random_seeds']]]
        for i,(name,scene) in enumerate(scenes):
            summary,log=continuous(run,name,scene,i);summaries.append(summary);rows+=log
            print(json.dumps(summary),flush=True)
        write_csv(out/'continuous_unloading_results.csv',rows);write_json(out/'continuous_summary.json',summaries)
    if args.phase=='lift':lift_scan(run)
    print(json.dumps({'completed_phase':args.phase,'output':str(out)}),flush=True)


if __name__=='__main__':main()
