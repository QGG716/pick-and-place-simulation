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


class SupportReleaseCell(Cell):
    def __init__(self,config):
        super().__init__(config);self.support_lift_attempts=[]

    def transit(self,start,goal,obstacles,seed,attachment=None,support_names=(),target_contact=None):
        prefix=[]
        if attachment is not None:
            current_box=attachment.box_at(self.robot.fk(start))
            floor=next(b for b in self.walls if b.name=='floor')
            safe_bottom=floor.center[2]+floor.half_extents[2]+2*self.p['collision_margin_m']+2*self.p['support_tolerance_m']
            lift=safe_bottom-float(current_box.corners()[:,2].min())
            if lift>0:
                destination=self.robot.fk(start).copy();destination[2,3]+=lift
                prefix,failure=self.cartesian(start,destination,obstacles,seed,attachment,[*support_names,'floor'])
                self.support_lift_attempts.append({'lift_m':lift,'q_path':[q.tolist() for q in prefix],'failure':failure,
                    'derivation':'floor top + 2*OBB margin + 2*support numerical tolerance minus actual box bottom'})
                if failure:return prefix,failure
                start=prefix[-1]
        path,failure=super().transit(start,goal,obstacles,seed,attachment,support_names,target_contact)
        return ([*prefix,*path[1:]] if prefix and path else path or prefix),failure


def main():
    cfg=load_validation_config();cell=SupportReleaseCell(cfg);s=cfg.data['scene'];size=next(iter(s['box_sizes_m'].values()))
    target=box('last_bottom',[s['front_x_m']+size[0]/2,0,size[2]/2],size);c=cfg.data['conveyor']
    result=evaluate_task(cell,target,[target],np.asarray(cfg.data['robot']['home_joints']),
        (c['fixed_extension_m'],c['fixed_z_m']),seed=cfg.data['planning']['seed'],mode='fixed')
    sample_loads(cell,result)
    write_json(ROOT/'outputs/m710id70_v3/v3/bottom_support_release_alternative.json',{
        'scope':'additional action comparison, original task grid and statistics unchanged',
        'configuration':cfg.evidence(),'support_lift_attempts':cell.support_lift_attempts,'result':result})
    print({'full_geometry':result['geometric_feasible'],'failure_stage':result['failure_stage'],'failure_reason':result['failure_reason'],
           'support_lift_attempts':len(cell.support_lift_attempts)})


if __name__=='__main__':main()
