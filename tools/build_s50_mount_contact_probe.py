"""Prepare source-bound native/Coal mounting diagnostics without planning."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_cad_model import load_preview_robot
from unloading_sim.robot_replacement_layout import frozen_baseline_layout
from unloading_sim.mount_contact import bind_mount_contact,mount_contact_evidence,mount_contact_contract,use_actual_mesh_backend
from unloading_sim.robot_native_binding import native_model_from_binding

def build(root=ROOT,*,with_coal=False,output=None):
    layout=frozen_baseline_layout(root);mount=layout.robot_base_transform()
    robot=load_preview_robot(root,world_from_mount=mount);binding=robot.model_binding
    support=next(box for box in layout.fixed_components() if box.name=='mount_plinth')
    bound=bind_mount_contact(robot,root=root,world_from_physical_mount=mount,expected_plinth=support)
    contract=mount_contact_contract(robot);cases=[];cpu=[]
    # q values are declared diagnostic display poses, not task limits or IK.
    for case_id,dz,q in [('normal_contact',0,[0.]*6),('real_base_penetration',-.001,[0.]*6),
            # Selected by the 2026-11-11 actual Coal finite diagnostic: J2=2.0
            # did not intersect; J2=2.4 produced an active-link/plinth contact.
            # This is a collision negative fixture, not a mechanical limit.
            ('moving_link_plinth_probe',0,[0.,2.4,0.,0.,0.,0.])]:
        actual=mount.copy();actual[2,3]+=dz;robot.base_transform=actual
        urdf,srdf,_=native_model_from_binding(binding,actual,project_root=root)
        request=dict(identity=dict(robot_model_binding=binding.identity(),fixed_base_contact_contract=contract),
            world=[contract['expected_plinth']],planning_only=True,planning_mode='cold_from_scratch')
        row=dict(case_id=case_id,urdf=ET.tostring(urdf,encoding='unicode'),srdf=ET.tostring(srdf,encoding='unicode'),request=request,q_rad=q)
        cases.append(row)
        if with_coal:
            from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
            from unloading_sim.s50_collision_diagnostics import diagnose_pose
            backend=PinocchioHppFclBackend.from_binding(binding,project_root=root,output_dir=Path(output)/case_id,world_from_base=actual)
            use_actual_mesh_backend(robot,backend)
            diagnostic=diagnose_pose(backend,q,[support],mount,pose_id=case_id)
        else:diagnostic=None
        cpu.append(dict(case_id=case_id,mount_contact=mount_contact_evidence(robot,[support]),coal=diagnostic))
    return dict(schema='s50_mount_contact_probe_input_v1',scope='CAD_SIMULATION_CONVENTION_ONLY_NOT_TASK_READY',cases=cases),cpu

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=ROOT/'outputs/s50_task_collision_20261111/mount')
    parser.add_argument('--coal',action='store_true');args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    payload,cpu=build(with_coal=args.coal,output=args.output)
    for name,value in [('native-input.json',payload),('python-coal-report.json',cpu)]:
        (args.output/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({row['case_id']:row['mount_contact']['passed'] for row in cpu}))
if __name__=='__main__':main()
