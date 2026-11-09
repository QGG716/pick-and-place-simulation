"""Native contract inputs only; no synthetic planning success."""
import json,sys
from pathlib import Path
import numpy as np
from test_gap_search import layout_for,options_for,ROOT,BASELINE
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.moveit2_backend import box_message
from unloading_sim.joint_search_outfeed import outfeed_path
from unloading_sim.geometry import OBB

def generate(mount_height=1.,gap=.2):
    l=layout_for(load_workcell_layout(ROOT/'configs/workcells'/BASELINE),gap,mount_height)
    outfeeds=[];receives=[]
    for h in (.55,.6,1.,1.5):
        world=coupled_conveyor_state(l,h,()).boxes
        box=OBB([-.55,.4,h+.15],[.3,.2,.15],np.eye(3),'target','carton')
        outfeeds.append(dict(outfeed=outfeed_path(box,world,finite_support=True),q_start=[0.]*6,q_goal=[0.]*6,
            attachment=None,world=[box_message(b) for b in world],process_policy=dict(target=box_message(box))))
    for box in (min(l.cartons(),key=lambda b:b.center[2]),max(l.cartons(),key=lambda b:b.center[2])):
        seen=set()
        for target,option in options_for(l,box):
            c=option['joint_search_contract'];key=(c['mode'],c['face'],c['height_m'])
            if key in seen:continue
            seen.add(key);h=c['height_m'];binding=coupled_binding(l,l.cartons(),h,h,box.name,[['chassis','conveyor_transverse']])
            binding.update(joint_search_layout=l.data['gap_search'],receive_contract=dict(height_m=h,initial_target=box_message(box),joint_search_contract=c))
            req=dict(stage='extraction',world=binding['samples'][-1]['world'],planning_only=True,process_policy=dict(target=box_message(box)))
            receives.append(dict(request=req,binding=binding))
    return dict(layout=l.data['gap_search'],receive=receives,outfeeds=outfeeds,
        plinth=box_message(next(b for b in l.fixed_components() if b.name=='mount_plinth')))
if __name__=='__main__':
    data=generate();data['lower_mount']=generate(.9,.2)
    Path(sys.argv[1]).write_text(json.dumps(data))
