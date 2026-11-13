"""Prepare the newly supplied STEP for kinematic simulation; keep historical CAD intact.

Reuses the existing converter, measured-axis construction and mesh exporter.
No manufacturer, BRep validity or exact tessellation qualification is required.
"""
from pathlib import Path
import argparse, copy, json, sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'assets/robots/huayan_s50_2000')]
from audit_step import convert
from build_cad_kinematic_model import _export_local_mesh, write_urdf
from unloading_sim.s50_cad_model import build_definition, load_definition, sha256
from unloading_sim.s50_kinematic_sim import save

DEST='assets/robots/huayan/s50_1_kinematic'


def prepare(prepared=False):
    source=ROOT/'assets/robots/huayan/S50_1.STEP'
    intermediate=ROOT/'assets/robots/huayan_s50_2000/derived/s50_1'
    if not prepared:convert(source,intermediate)
    audit=json.loads((intermediate/'cad_audit.json').read_text(encoding='utf8'))
    if sha256(source.read_bytes())!=audit['source_sha256']:raise ValueError('CONVERSION_SOURCE_CHANGED')
    parts={p['id']:p for p in audit['parts']};old=load_definition(ROOT)
    measurements=json.loads((ROOT/'assets/robots/huayan_s50_2000/derived/interface_measurements.json').read_text(encoding='utf8'))
    measurements=copy.deepcopy(measurements);measurements['source_sha256']=audit['source_sha256']
    comparisons=[]
    # Locate the shared cylinder axes in this STEP using the existing convention.
    for number,record in enumerate(measurements['axes'],1):
        matches=[];threshold=.009 if number==3 else .02
        for a in parts[record['parent_cad_body']]['cylindrical_faces']:
            if a['radius_m']<threshold:continue
            p=np.asarray(a['origin_m']);u=np.asarray(a['direction'])
            for b in parts[record['child_cad_body']]['cylindrical_faces']:
                if b['radius_m']<threshold:continue
                v=np.asarray(b['direction']);q=np.asarray(b['origin_m'])
                if np.linalg.norm(np.cross(u,v))>1e-5 or np.linalg.norm(np.cross(q-p,u))>1e-5:continue
                canonical=u if u[np.argmax(abs(u))]>=0 else -u
                matches.append((p-u*np.dot(p,u),canonical,dict(parent_face_index=a['face_index'],child_face_index=b['face_index'])))
        if not matches:raise ValueError(f'NO_SHARED_J{number}_AXIS')
        point,axis,_=matches[0]
        if any(np.linalg.norm(p-point)>1e-5 or np.linalg.norm(a-axis)>1e-5 for p,a,_ in matches):
            raise ValueError(f'AMBIGUOUS_J{number}_AXIS')
        comparisons.append(dict(joint=f'J{number}',line_point_difference_m=float(np.linalg.norm(point-record['point_CAD_m'])),
            axis_difference=float(np.linalg.norm(axis-record['unoriented_axis_CAD']))))
        record.update(point_CAD_m=point.tolist(),unoriented_axis_CAD=axis.tolist(),evidence=[m[2] for m in matches])
    planes=[f for f in parts['base_s50']['planar_faces'] if f['normal'][1]<-.999999]
    support=min(f['origin_m'][1] for f in planes)
    measurements['support_plane'].update(y_CAD_m=support,axis_intersection_CAD_m=[0,support,0],
        support_faces=[f for f in planes if abs(f['origin_m'][1]-support)<1e-6])
    planes=[f for f in parts['j6_s50']['planar_faces'] if f['normal'][0]<-.999999 and f['area_m2']>.001]
    measurements['flange_distal_face']=min(planes,key=lambda f:f['origin_m'][0])
    definition=build_definition(measurements,audit)
    definition.update(source_step_path=source.relative_to(ROOT).as_posix(),
        source_model_usage='USER_AUTHORIZED_KINEMATIC_SIMULATION',
        historical_model_sha256=old['definition_sha256'])
    oldlinks={l['name']:l for l in old['links']}
    changes=[]
    for link in definition['links']:
        part=parts[link['cad_body']]
        visual=_export_local_mesh(ROOT,dict(path=part['mesh'],sha256=part['mesh_sha256']),
            link['name'],np.asarray(link['CAD_from_link_zero']),DEST)
        link['visual_mesh']=visual;link['collision_meshes']=[{**visual,'role':'SELECTED_STEP_SURFACE'}]
        # Retain the two existing small local face patches as conservative basic
        # collision geometry, only where the new STEP has the same missing face.
        # They are explicitly historical-source patches, not new exact CAD proof.
        for i,patch in enumerate(oldlinks[link['name']]['collision_meshes'][1:]):
            missing=part['missing_tessellation_faces']
            if not any(f['face_index']==patch['source_patch']['source_face_index'] for f in missing):continue
            local=_export_local_mesh(ROOT,dict(path=patch['source_patch']['mesh'],sha256=patch['source_patch']['mesh_sha256']),
                link['name']+f'_local_patch_{i}',np.asarray(link['CAD_from_link_zero']),DEST)
            link['collision_meshes'].append({**local,'role':'RETAINED_LOCAL_CONSERVATIVE_PATCH',
                'source_step_sha256':old['source_step_sha256'],'source_face_index':patch['source_patch']['source_face_index']})
        changes.append(dict(link=link['name'],vertices=visual['vertices'],triangles=visual['triangles'],
            missing_tessellation_faces=part['missing_tessellation_faces'],source_topology_valid=part['topology_valid']))
    urdf,srdf=write_urdf(ROOT,definition,DEST)
    definition.update(urdf_path=urdf.relative_to(ROOT).as_posix(),urdf_sha256=sha256(urdf.read_bytes()),
        srdf_path=srdf.relative_to(ROOT).as_posix(),srdf_sha256=sha256(srdf.read_bytes()))
    save(ROOT/DEST/'model.json',definition)
    save(ROOT/DEST/'source-summary.json',dict(source=definition['source_step_path'],sha256=audit['source_sha256'],
        previous_source_sha256=old['source_step_sha256'],axis_comparison=comparisons,parts=changes,
        flange_origin_difference_m=(np.array(definition['frames']['flange']['CAD_from_frame_zero'])[:3,3]-np.array(old['frames']['flange']['CAD_from_frame_zero'])[:3,3]).tolist(),
        support_plane_y_m=support,units='metres',scope='BASIC_KINEMATIC_MODEL_PREPARATION_NOT_CERTIFICATION'))
    print(json.dumps(dict(model=DEST+'/model.json',source_sha256=audit['source_sha256'],axis_comparison=comparisons)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared',action='store_true',help='Reuse a conversion of the same source bytes.')
    prepare(p.parse_args().prepared)
