"""Small CPU/native Hmax contract fixture; no planning or completion claims."""
import argparse
import json
from pathlib import Path
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.moveit2_backend import box_message
from unloading_sim.l01_hmax import layer_permission


def fixture():
    root=Path(__file__).resolve().parents[1]
    layout=load_workcell_layout(root/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile_tool_extension100.yaml')
    cartons=layout.cartons();frozen=FrozenCartonPopulation.capture(cartons);p=layer_permission(frozen)
    cases=[]
    for b in cartons:
        if b.name not in p['allowed_target_ids']:continue
        height=frozen.evidence()['heights'][b.name]['Hmax_world_m']
        binding=coupled_binding(layout,cartons,height,height,b.name,[])
        binding['receive_contract']=dict(height_m=height,initial_target=box_message(b),
            junction_front=True,third_sync_disabled=True,l01_hmax_permission=p)
        req=dict(planning_only=True,planning_mode='cold_from_scratch',stage='extraction',
            world=binding['samples'][-1]['world'],process_policy=dict(target_id=b.name,target=box_message(b)))
        cases.append(dict(binding=binding,request=req))
    return dict(l01_hmax=cases,physical_execution='NOT_RUN',planning_performed=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.write_text(json.dumps(fixture(),separators=(',',':'))+'\n',encoding='utf-8')
