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
from .s50_cad_model import ensure_preview_assets, load_definition, read_mesh_bytes
from .ik import solve_ik
from .geometry import rotation_matrix_from_rotation_vector


def save(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')


def simulation_layout(root, settings):
    """One fixed station per run; the explicit gap moves the whole assembly."""
    baseline=frozen_baseline_layout(root)
    if 'layout_gap_m' not in settings:return baseline
    from dataclasses import replace
    from .gap_search_layout import BASELINE,GapCandidate,candidate_data
    from .workcell_layout import load_workcell_layout,compute_layout_fingerprint
    original=load_workcell_layout(Path(root)/'configs/workcells'/BASELINE)
    data=candidate_data(original.data,GapCandidate(float(settings['layout_gap_m']),.9))
    return replace(original,data=data,layout_fingerprint=compute_layout_fingerprint(data,original.assets))


def simulation_scene(root,settings):
    from .s50_tool_mount import bind_s50_tool_mount
    layout=simulation_layout(root,settings)
    return layout,bind_s50_tool_mount(freeze_non_robot_scene(layout),settings,root)


def cad_coordinates(settings,q):
    convention=settings.get('joint_coordinate_convention',{})
    return np.asarray(convention.get('sign',[1]*6))*np.asarray(q)+np.deg2rad(convention.get('cad_offset_deg',[0]*6))


def simulation_coordinates(settings,q_cad):
    convention=settings.get('joint_coordinate_convention',{})
    return (np.asarray(q_cad)-np.deg2rad(convention.get('cad_offset_deg',[0]*6)))/np.asarray(convention.get('sign',[1]*6))


def _joint_rpy(r):
    pitch=np.arctan2(-r[2,0],np.hypot(r[0,0],r[1,0]))
    if abs(np.cos(pitch))<1e-10:return [0.,float(pitch),float(np.arctan2(-r[0,1],r[1,1]))]
    return [float(np.arctan2(r[2,1],r[2,2])),float(pitch),float(np.arctan2(r[1,0],r[0,0]))]


def load_simulation_model(root, settings):
    """Load the selected CAD revision without borrowing manufacturer readiness."""
    root=Path(root)
    if not settings.get('model_definition'):
        ensure_preview_assets(root)
        return load_definition(root)
    path=root/settings['model_definition'];raw=path.read_bytes()
    definition=json.loads(raw);definition['definition_sha256']=hashlib.sha256(raw).hexdigest()
    if len(definition['joints'])!=6 or len(definition['links'])!=7:
        raise ValueError('SIM_REQUIRES_SIX_AXIS_SEVEN_BODY_MODEL')
    source=root/definition['source_step_path']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=definition['source_step_sha256']:
        raise ValueError('SIM_SELECTED_STEP_CHANGED')
    if hashlib.sha256((root/settings['source_urdf']).read_bytes()).hexdigest()!=definition['urdf_sha256']:
        raise ValueError('SIM_SELECTED_CHAIN_CHANGED')
    for link in definition['links']:
        for mesh in [link['visual_mesh'],*link['collision_meshes']]:
            destination=root/mesh['path'];content=read_mesh_bytes(destination)
            if hashlib.sha256(content).hexdigest()!=mesh['sha256']:
                raise ValueError('SIM_SELECTED_MESH_CHANGED:'+mesh['path'])
            if not destination.is_file():destination.write_bytes(content)
    return definition


def make_context(root, output, config=None):
    root=Path(root).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    settings=(dict(config) if isinstance(config,dict) else
        yaml.safe_load(Path(config or root/'configs/robots/huayan_s50_2000_kinematic_sim.yaml').read_text(encoding='utf8')))
    definition=load_simulation_model(root,settings)
    layout,frozen=simulation_scene(root,settings)
    save(output/'frozen-scene.json',frozen)
    intrinsic=frozen['tool_intrinsic'];adapter=np.asarray(settings['flange_from_original_tool_mount'],float)
    frames={k.removeprefix('T_tool_mount_reference_'):adapter@np.asarray(v,float) for k,v in intrinsic['intrinsic_frames'].items()}
    world=layout.robot_base_transform()@np.asarray(settings['installation_reference_from_model_base'])
    source=root/settings['source_urdf'];tree=ET.parse(source)
    tree.getroot().set('name','huayan_s50_2000_kinematic_sim')
    limits=np.deg2rad(settings['joint_ranges_deg']);joints=definition['joints']
    for i,j in enumerate(joints):
        node=tree.find(f"joint[@name='{j['name']}']");limit=node.find('limit')
        limit.set('lower',str(limits[i,0]));limit.set('upper',str(limits[i,1]))
        limit.set('velocity',str(settings['timing']['joint_velocity_rad_s']))
        convention=settings.get('joint_coordinate_convention')
        if convention:
            axis=np.asarray(j['axis']);origin=np.asarray(j['origin']).copy()
            origin[:3,:3]=origin[:3,:3]@rotation_matrix_from_rotation_vector(axis*np.deg2rad(convention['cad_offset_deg'][i]))
            node.find('origin').set('rpy',' '.join(map(str,_joint_rpy(origin[:3,:3]))))
            node.find('axis').set('xyz',' '.join(map(str,axis*convention['sign'][i])))
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
    periodic=(hi-lo)>=2*np.pi
    seed_lo=np.where(periodic,np.maximum(lo,-np.pi),lo);seed_hi=np.where(periodic,np.minimum(hi,np.pi),hi)
    seeds=explicit+[np.zeros(6)]+[context.rng.uniform(seed_lo,seed_hi) for _ in range(draws)]
    answers=[]
    for index,seed in enumerate(seeds):
        r=solve_ik(robot,target,np.asarray(seed),max_iterations=220,damping=.015,max_step=.25,
            position_tolerance=.0003,orientation_tolerance=.001)
        row=dict(target=context.settings['target'],stage=stage,seed_index=index,iterations=r.iterations,ik_success=r.success,
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
