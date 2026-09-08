"""Run the A/B/C stage-IK ablation on a small set of original grid tasks."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from time import perf_counter

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT))
import numpy as np

from unloading_sim.validation_config import DEFAULT, load_validation_config
from unloading_sim.validation_motion import Cell
from unloading_sim.validation_scenes import grid_tasks
from tools.capture_m710_evidence import capture
from tools.run_m710id70_v3 import Run, compact, write_csv, write_json

MODES=('legacy_single','filtered_single','multi_solution')


def run_ablation(config: Path, output: Path, task_indices: list[int]) -> dict:
    output.mkdir(parents=True,exist_ok=True)
    capture(output,'python tools/run_m710id70_v3_stage_ablation.py '+' '.join(sys.argv[1:]))
    rows=[]
    for mode in MODES:
        cfg=load_validation_config(config)
        cfg.data['planning']['stage_ik_search_mode']=mode
        tasks=list(grid_tasks(cfg.data['scene']))
        home=np.asarray(cfg.data['robot']['home_joints'])
        conveyor=cfg.data['conveyor'];belt=(conveyor['fixed_extension_m'],conveyor['fixed_z_m'])
        run=Run(cfg,output/mode,workers=1)
        for index in task_indices:
            cell=Cell(cfg)
            orientation,target,neighbors,inside=tasks[index]
            if not inside:
                raise ValueError(f'grid_{index:03} is outside the frozen 104-task denominator')
            started=perf_counter()
            result=run.task(f'grid_{index:03}_{mode}',cell,target,[target,*neighbors],home,belt,
                            cell.p['seed']+index,mode='fixed')
            elapsed=perf_counter()-started
            summary=compact(f'grid_{index:03}',result,orientation=orientation)
            totals=result['search_statistics']['totals']
            rows.append({**summary,'ablation_mode':mode,'wall_time_s':elapsed,
                'stage_connection_searches':totals['stage_connection_searches']})
    by_task={f'grid_{index:03}':{} for index in task_indices}
    for row in rows:
        by_task[row['task_id']][row['ablation_mode']]=row
    attribution=[]
    for task_id,variants in by_task.items():
        a,b,c=(variants[mode] for mode in MODES)
        attribution.append({'task_id':task_id,
            'a_to_b_outcome_changed':(a['failure_stage'],a['failure_reason'],a['GEOMETRICALLY_REACHABLE']) !=
                                     (b['failure_stage'],b['failure_reason'],b['GEOMETRICALLY_REACHABLE']),
            'a_to_b_attribution':'stage endpoint validity filtering' if
                (a['failure_stage'],a['failure_reason'],a['GEOMETRICALLY_REACHABLE']) !=
                (b['failure_stage'],b['failure_reason'],b['GEOMETRICALLY_REACHABLE']) else 'no task-level outcome change',
            'b_to_c_outcome_changed':(b['failure_stage'],b['failure_reason'],b['GEOMETRICALLY_REACHABLE']) !=
                                     (c['failure_stage'],c['failure_reason'],c['GEOMETRICALLY_REACHABLE']),
            'b_to_c_attribution':'bounded alternate-configuration connection search' if
                (b['failure_stage'],b['failure_reason'],b['GEOMETRICALLY_REACHABLE']) !=
                (c['failure_stage'],c['failure_reason'],c['GEOMETRICALLY_REACHABLE']) else 'no task-level outcome change',
            'multi_solution_used_nonfirst_candidate':bool(c['stage_connections_using_nonfirst_candidate_task_total']),
            'rrt_iterations_a_b_c':[a['rrt_iterations_consumed_task_total'],b['rrt_iterations_consumed_task_total'],
                                    c['rrt_iterations_consumed_task_total']],
            'ik_seeds_attempted_a_b_c':[a['ik_seeds_attempted_task_total'],b['ik_seeds_attempted_task_total'],
                                        c['ik_seeds_attempted_task_total']],
            'wall_time_s_a_b_c':[a['wall_time_s'],b['wall_time_s'],c['wall_time_s']]})
    result={'schema_version':'m710_stage_ik_ablation_v1','task_indices':task_indices,
        'modes':{'A':'legacy_single','B':'filtered_single','C':'multi_solution'},
        'budget_interpretation':{
            'A_and_B':'one stage connection receives the legacy rrt_iterations per-call budget',
            'C':'all alternate candidates within one stage connection share stage_connection_iteration_budget',
            'comparison':'configuration numbers alone are not treated as equal total compute; observed work is recorded per task'},
        'outcome_counts':{mode:dict(Counter(
            f"{'success' if row['GEOMETRICALLY_REACHABLE'] else 'failure'}:{row['failure_reason']}"
            for row in rows if row['ablation_mode']==mode)) for mode in MODES},
        'attribution':attribution,'rows':rows}
    write_csv(output/'stage_ik_ablation.csv',rows)
    write_json(output/'stage_ik_ablation.json',result)
    return result


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--task-indices',type=int,nargs='+',default=[22,45])
    args=parser.parse_args()
    result=run_ablation(args.config,args.output_dir,args.task_indices)
    print(json.dumps({'output':str(args.output_dir),'tasks':result['task_indices']},ensure_ascii=False))


if __name__=='__main__':
    main()
