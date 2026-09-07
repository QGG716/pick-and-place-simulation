"""CPU-only, seeded M-710iD/70 V3 acceptance with resumable task evidence."""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import csv
import hashlib
import json
from pathlib import Path
import sys

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
    attach_t=trajectory.time_from_start[stages['extraction'][0]]
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
    return {'task_id':task_id,**extra,'box':result['box'],'seed':result['seed'],'mode':result['mode'],
            'GRASP_REACHABLE':result['grasp_reachable'],'EXTRACTION_FEASIBLE':result['extraction_feasible'],
            'GEOMETRICALLY_REACHABLE':result['geometric_feasible'],'PAYLOAD_QUALIFIED':result['payload_qualified'],
            'DYNAMICS_VERIFIED':result['dynamics_verified'],'load_status':result['load_status'],
            'failure_stage':result['failure_stage'],'failure_reason':result['failure_reason'],
            'candidate_failure_counts':dict(reasons),'face':best.get('face'),'roll_deg':best.get('roll_deg'),
            'loaded_tcp_path_m':best.get('loaded_tcp_path_m'),'cycle_s':best.get('cycle_s'),
            'conveyor_action_s':best.get('conveyor_action_s')}


class Run:
    def __init__(self,cfg,output,workers=1):
        self.cfg=cfg;self.output=output;self.count=0;self.workers=workers
        self.configuration_digest=hashlib.sha256(json.dumps(cfg.evidence(),sort_keys=True).encode()).hexdigest()
        # Untracked implementation files must also invalidate cached evidence.
        source_paths=sorted([*ROOT.joinpath('src/unloading_sim').rglob('*.py'),Path(__file__)])
        self.code_digest=hashlib.sha256(b''.join(p.read_bytes() for p in source_paths)).hexdigest()

    def task(self,task_id,cell,target,remaining,q,belt,seed,mode='dynamic',only_face=None):
        key={'code':self.code_digest,'config':self.configuration_digest,'height':cell.robot.base_transform.tolist(),
             'target':target.name,'boxes':[{'name':b.name,'pose':b.world_from_local.tolist(),'half':b.half_extents.tolist()} for b in remaining],
             'q':np.asarray(q).tolist(),'belt':list(belt),'seed':int(seed),'mode':mode,'only_face':only_face}
        digest=hashlib.sha256(json.dumps(key,sort_keys=True).encode()).hexdigest()
        path=self.output/'tasks'/f'{task_id}.json'
        if path.exists():
            saved=json.loads(path.read_text(encoding='utf-8'))
            if saved.get('input_digest')==digest:return saved['result']
        result=evaluate_task(cell,target,remaining,q,belt,seed=seed,mode=mode,only_face=only_face)
        sample_loads(cell,result)
        write_json(path,{'input_digest':digest,'inputs':key,'result':result})
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
    identifier,height,target,remaining,q,belt,seed,mode=job
    return Run(cfg,output).task(identifier,Cell(cfg,height),target,remaining,q,belt,seed,mode)


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
    cell=Cell(run.cfg);s=cell.d['scene'];rows=[];paired=[];q=np.asarray(cell.d['robot']['home_joints']);c=cell.d['conveyor'];belt=(c['fixed_extension_m'],c['fixed_z_m'])
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
        a,b=results['fixed'],results['dynamic'];common=a['geometric_feasible'] and b['geometric_feasible']
        paired.append({'task_id':f'grid_{index:03}','fixed_feasible':a['geometric_feasible'],'dynamic_feasible':b['geometric_feasible'],
                       'new_feasible':b['geometric_feasible'] and not a['geometric_feasible'],'lost_feasible':a['geometric_feasible'] and not b['geometric_feasible'],
                       'common_feasible':common,'loaded_path_delta_m':None if not common else b['selected']['loaded_tcp_path_m']-a['selected']['loaded_tcp_path_m'],
                       'cycle_delta_s':None if not common else b['selected']['cycle_s']-a['selected']['cycle_s'],
                       'dynamic_action_s':None if not b['selected'] else b['selected']['conveyor_action_s'],
                       'fixed_action_s':None if not a['selected'] else a['selected']['conveyor_action_s']})
    write_csv(run.output/'task_reachability.csv',rows);write_csv(run.output/'conveyor_ab.csv',paired)
    return rows,paired


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
    parser.add_argument('--phase',choices=['small','grid','continuous','lift','all'],default='all')
    parser.add_argument('--workers',type=int,default=4,help='Independent CPU processes; task seeds and outputs are unchanged')
    args=parser.parse_args();cfg=load_validation_config(args.config);out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    capture(out,'python tools/run_m710id70_v3.py '+' '.join(sys.argv[1:]));write_json(out/'effective_config.json',cfg.evidence())
    run=Run(cfg,out,args.workers)
    if args.phase in ('small','all'):
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
    if args.phase in ('grid','all'):grid(run)
    if args.phase in ('continuous','all'):
        summaries=[];rows=[];s=cfg.data['scene']
        scenes=[('regular',regular_scene(s)),*[(f'random_seed_{seed}',random_scene(s,seed)) for seed in s['random_seeds']]]
        for i,(name,scene) in enumerate(scenes):
            summary,log=continuous(run,name,scene,i);summaries.append(summary);rows+=log
            print(json.dumps(summary),flush=True)
        write_csv(out/'continuous_unloading_results.csv',rows);write_json(out/'continuous_summary.json',summaries)
    if args.phase in ('lift','all'):lift_scan(run)
    print(json.dumps({'completed_phase':args.phase,'output':str(out)}),flush=True)


if __name__=='__main__':main()
