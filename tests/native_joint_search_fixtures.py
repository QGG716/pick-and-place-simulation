"""Produce same-source geometry inputs for m710_joint_search_test."""
import json,sys
from dataclasses import replace
from pathlib import Path
import numpy as np
from unloading_sim.joint_search_layout import LayoutCandidate,candidate_data,BASELINE,SCHEMA
from unloading_sim.workcell_layout import load_workcell_layout,compute_layout_fingerprint
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.joint_search_outfeed import outfeed_path
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.geometry import OBB
from unloading_sim.moveit2_backend import box_message
from unloading_sim.lifted_reception import StartHeight

def generate():
    root=Path(__file__).resolve().parents[1];old=load_workcell_layout(root/'configs/workcells'/BASELINE)
    d=candidate_data(old.data,LayoutCandidate(.325,-.2,.6));layout=replace(old,data=d,layout_fingerprint=compute_layout_fingerprint(d,old.assets))
    outfeeds=[];receives=[]
    for height in (.25,.6,1.15,2.1):
        world=coupled_conveyor_state(layout,height,()).boxes
        box=OBB(np.array([-.65,.45,height+.15]),np.array([.2,.2,.15]),np.eye(3),'target','carton')
        outfeeds.append(dict(outfeed=outfeed_path(box,world),q_start=[0.]*6,q_goal=[0.]*6,attachment=None,
            world=[box_message(b) for b in world],process_policy=dict(target=box_message(box))))
    high=max(layout.cartons(),key=lambda b:b.center[2]);low=min(layout.cartons(),key=lambda b:b.center[2])
    for box,mode,face,explicit in [(high,'Hmin','x-',None),(high,'Hmax','x-',None),(low,'LIFTED_TOP_FIXED','z+',None),
                                 (low,'LIFTED_TOP_EXPLICIT','z+',.65),(low,'LIFTED_TOP_EXPLICIT','z+',.85)]:
        h=StartHeight.capture(box);height=h.hmin if mode=='Hmin' else h.hmax if mode=='Hmax' else explicit if explicit else .25
        contract=dict(schema=SCHEMA,target_id=box.name,target_identity=h.box_identity,d_m=.05,mode=mode,face=face,
            height_m=height,minimum_m=.25,maximum_m=2.1,release_bottom_m=max(h.z_b+.05,height+.05) if face=='z+' else h.z_b)
        if explicit:contract['explicit_receive_height_m']=explicit
        binding=coupled_binding(layout,layout.cartons(),height,height,box.name,[])
        binding.update(joint_search_layout=d['joint_search'],receive_contract=dict(height_m=height,initial_target=box_message(box),
            junction_front=face=='x-',third_sync_disabled=True,joint_search_contract=contract))
        request=dict(stage='extraction',world=binding['samples'][-1]['world'],planning_only=True,process_policy=dict(target=box_message(box)))
        receives.append(dict(request=request,binding=binding))
    return dict(outfeeds=outfeeds,receive=receives)

if __name__=='__main__':Path(sys.argv[1]).write_text(json.dumps(generate()))
