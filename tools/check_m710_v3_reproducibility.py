"""Recompute representative tasks serially and compare with four-worker run."""
from pathlib import Path
import json
import re
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT))
import numpy as np
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell,evaluate_task
from unloading_sim.validation_scenes import grid_tasks
from tools.run_m710id70_v3 import write_json,sample_loads


def main():
    cfg=load_validation_config();cell=Cell(cfg);out=ROOT/'outputs/m710id70_v3/v3';tasks=list(grid_tasks(cfg.data['scene']));rows=[]
    for index,mode in [(20,'fixed'),(20,'dynamic'),(85,'dynamic')]:
        _,target,neighbors,valid=tasks[index]
        assert valid
        result=evaluate_task(cell,target,[target,*neighbors],np.asarray(cfg.data['robot']['home_joints']),
            (cfg.data['conveyor']['fixed_extension_m'],cfg.data['conveyor']['fixed_z_m']),seed=cfg.data['planning']['seed']+index,mode=mode)
        sample_loads(cell,result)
        original=json.loads((out/f'tasks/grid_{index:03}_{mode}.json').read_text(encoding='utf-8'))['result']
        rows.append({'task':f'grid_{index:03}_{mode}','exact_result_equality':result==original})
    command=json.loads((out/'source_manifest.json').read_text(encoding='utf-8'))['command']
    match=re.search(r'--workers\s+(\d+)',command)
    write_json(out/'determinism_audit.json',{'parallel_workers':int(match.group(1)) if match else None,'replay_workers':1,'tasks':rows,'pass':all(row['exact_result_equality'] for row in rows)})
    if not all(row['exact_result_equality'] for row in rows):raise RuntimeError('serial/parallel task mismatch')
    print(rows)


if __name__=='__main__':main()
