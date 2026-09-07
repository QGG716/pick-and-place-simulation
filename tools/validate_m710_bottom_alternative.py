"""Additional bottom-box lift-clearance action with unchanged collision rules."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT))
import numpy as np
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell,evaluate_task
from unloading_sim.validation_scenes import box
from tools.run_m710id70_v3 import write_json,sample_loads


def main():
    cfg=load_validation_config();cell=Cell(cfg);s=cfg.data['scene'];size=next(iter(s['box_sizes_m'].values()))
    target=box('last_bottom',[s['front_x_m']+size[0]/2,0,size[2]/2],size);c=cfg.data['conveyor']
    result=evaluate_task(cell,target,[target],np.asarray(cfg.data['robot']['home_joints']),
        (c['fixed_extension_m'],c['fixed_z_m']),seed=cfg.data['planning']['seed'],mode='fixed')
    sample_loads(cell,result)
    write_json(ROOT/'outputs/m710id70_v3/v3/bottom_support_release_alternative.json',{
        'scope':'additional action comparison, original task grid and statistics unchanged',
        'configuration':cfg.evidence(),'support_lift_attempts':cell.support_release_events,'result':result})
    print({'full_geometry':result['geometric_feasible'],'failure_stage':result['failure_stage'],'failure_reason':result['failure_reason'],
           'support_lift_attempts':len(cell.support_release_events)})


if __name__=='__main__':main()
