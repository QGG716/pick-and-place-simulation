"""One-box S50 kinematic simulation under explicit temporary user conventions.

Independent of the strict manufacturer/task admission entry. Never changes its
qualification flags. Uses the actual CAD serial chain and original tool frames.
"""
from pathlib import Path
import json
import hashlib
import xml.etree.ElementTree as ET
from types import SimpleNamespace
import numpy as np
import yaml
from .robot import URDFRobot
from .robot_replacement_layout import frozen_baseline_layout, freeze_non_robot_scene
from .s50_cad_model import ensure_preview_assets, load_definition
from .ik import solve_ik


def save(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')


def make_context(root, output, config=None):
    root=Path(root).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    config=Path(config or root/'configs/robots/huayan_s50_2000_kinematic_sim.yaml')
    settings=yaml.safe_load(config.read_text(encoding='utf8'))
    ensure_preview_assets(root)
    definition=load_definition(root)
    layout=frozen_baseline_layout(root);frozen=freeze_non_robot_scene(layout)
    intrinsic=frozen['tool_intrinsic'];adapter=np.asarray(settings['flange_from_original_tool_mount'],float)
    frames={k.removeprefix('T_tool_mount_reference_'):adapter@np.asarray(v,float) for k,v in intrinsic['intrinsic_frames'].items()}
    world=layout.robot_base_transform()@np.asarray(settings['installation_reference_from_model_base'])
    source=root/settings['source_urdf'];tree=ET.parse(source)
    tree.getroot().set('name','huayan_s50_2000_kinematic_sim')
    limits=np.deg2rad(settings['joint_ranges_deg']);joints=definition['joints']
    for i,j in enumerate(joints):
        limit=tree.find(f"joint[@name='{j['name']}']/limit")
        limit.set('lower',str(limits[i,0]));limit.set('upper',str(limits[i,1]))
        limit.set('velocity',str(settings['timing']['joint_velocity_rad_s']))
    for mesh in tree.iter('mesh'):mesh.set('filename',str((source.parent/mesh.get('filename')).resolve()))
    urdf=output/'s50_kinematic_sim.urdf';tree.write(urdf,encoding='utf8',xml_declaration=True)
    robot=URDFRobot.from_urdf(urdf,active_joint_names=[j['name'] for j in joints],base_link=definition['base_link'],tip_link=definition['flange_link'],tool_length=0)
    robot.base_transform=world;robot.tip_from_tcp=frames['nominal_compressed_contact']
    robot.joint_limits=limits
    return SimpleNamespace(root=root,output=output,settings=settings,definition=definition,layout=layout,frozen=frozen,
        robot=robot,frames=frames,world=world,urdf=urdf,intrinsic=intrinsic,rng=np.random.default_rng(settings['seed']),attempts=[])


def tool_virtual(context,q):
    return context.robot.fk(q)@np.linalg.inv(context.frames['nominal_compressed_contact'])@context.frames['virtual_task_tcp']


def ik_candidates(context,target,*,seeds=(),draws=80,stage='ik',valid=None,limit=8):
    robot=context.robot;lo,hi=robot.joint_limits.T
    explicit=list(seeds);reference=np.asarray(explicit[0]) if explicit else np.zeros(6)
    seeds=explicit+[np.zeros(6)]+[context.rng.uniform(np.maximum(lo,-np.pi),np.minimum(hi,np.pi)) for _ in range(draws)]
    answers=[]
    for index,seed in enumerate(seeds):
        r=solve_ik(robot,target,np.asarray(seed),max_iterations=220,damping=.015,max_step=.25,
            position_tolerance=.0003,orientation_tolerance=.001)
        row=dict(stage=stage,seed_index=index,iterations=r.iterations,ik_success=r.success,
            position_error_m=r.position_error,orientation_error_rad=r.orientation_error,q_rad=r.q.tolist())
        if r.success:
            q=r.q.copy()
            # Equivalent revolute turns only, inside the explicit temporary limits.
            for j in range(6):
                equivalent=[q[j]+2*np.pi*k for k in range(-2,3) if lo[j]<=q[j]+2*np.pi*k<=hi[j]]
                q[j]=min(equivalent,key=lambda a:abs(a-reference[j]))
            good=True if valid is None else valid(q)
            row['basic_geometry_passed']=bool(good)
            if good and not any(np.max(np.abs(q-a))<.05 for a in answers):answers.append(q)
        context.attempts.append(row)
        if len(answers)>=limit:break
    return answers


def target_contact(context):
    target=next(b for b in context.layout.cartons() if b.name==context.settings['target'])
    pose=np.asarray(context.frozen['initial_physical_contact_target']).copy()
    pose[:3,3]=target.center-[target.half_extents[0],0,0]
    return target,pose
