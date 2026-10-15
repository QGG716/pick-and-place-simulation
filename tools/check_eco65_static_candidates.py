"""Four static joint candidates, exact bound robot/tool convexes; no path search."""
import argparse
import copy
import hashlib
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco as mj
import trimesh
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom
from unloading_sim.eco65.model import ROOT, load_json, save_json, robot_for, mounting_pairs
from build_eco65_full_face_layout import verify_static


def run(folder):
    start=time.perf_counter();snap=verify_static(load_json(folder/'snapshot.json'))
    manifest=load_json(folder/'usd/workstation_manifest.json');pose=load_json(folder/'static_pose.json')
    if manifest['source_snapshot_fingerprint']!=snap['fingerprint'] or pose['source_snapshot_fingerprint']!=snap['fingerprint']:
        raise ValueError('STATIC_CANDIDATE_SOURCE_MISMATCH')
    if hashlib.sha256((folder/'usd/workstation_D.usdc').read_bytes()).hexdigest()!=manifest['workstation_sha256']:
        raise ValueError('STATIC_CANDIDATE_USD_CHANGED')
    if manifest['home_q_rad']!=pose['home_q']:raise ValueError('STATIC_CANDIDATE_POSE_CHANGED')
    scene=snap['scene'];robot=robot_for(snap['tool'],scene);q0=np.array(pose['home_q'])
    xml=ET.Element('mujoco');ET.SubElement(xml,'compiler',angle='radian')
    assets=ET.SubElement(xml,'asset');world=ET.SubElement(xml,'worldbody');entries=[]
    def mesh(name,v,kind,owner,T=None):
        ET.SubElement(assets,'mesh',name=name,vertex=' '.join(map(str,np.asarray(v).reshape(-1))))
        body=ET.SubElement(world,'body',name=name,mocap='true')
        if T is not None:
            body.set('pos',' '.join(map(str,T[:3,3])));body.set('quat',' '.join(map(str,Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]])))
        ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,contype='0',conaffinity='0')
        entries.append(dict(name=name,kind=kind,owner=owner))
    old_frames=robot.named_link_frames(q0)
    moving=[]
    for i,row in enumerate(manifest['collision_records']):
        if '/Mast/' in row['path']:
            mesh('mast_'+str(i),row['points'],'environment',row['path'].rsplit('/',1)[1],np.array(row['T_D']));continue
        path=row['path'];isrobot='/Robot/' in path
        owner=path.split('/')[3] if isrobot else path.rsplit('/',1)[1].removeprefix('collision_')
        mesh('moving_'+str(i),row['points'],'robot' if isrobot else 'tool',owner)
        frame=old_frames[owner] if isrobot else old_frames['link_6']@np.array(snap['tool']['T_flange_tool_cad'])
        moving.append(dict(entry=entries[-1],local=np.linalg.inv(frame)@np.array(row['T_D'])))
    for item in scene['obstacles']+[dict(id=b['id'],size_m=b['size_m'],position_m=b['pose']) for b in scene['boxes']]:
        ET.SubElement(world,'geom',name=item['id'],type='box',size=' '.join(map(str,np.array(item['size_m'])/2)),pos=' '.join(map(str,item['position_m'])),contype='0',conaffinity='0')
        entries.append(dict(name=item['id'],kind='environment',owner=item['id']))
    component_count=0
    for item in scene['conveyors']['items']:
        stage=Usd.Stage.Open(str(ROOT/item['usd_path']))
        for row in item['meshes']:
            if not row['collision']:continue
            m=UsdGeom.Mesh(stage.GetPrimAtPath(row['prim']));v=np.array(m.GetPointsAttr().Get());idx=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
            for n in m.GetFaceVertexCountsAttr().Get():
                ids=idx[offset:offset+n];offset+=n;faces.extend([[ids[0],ids[k],ids[k+1]] for k in range(1,n-1)])
            for part in trimesh.Trimesh(v,np.array(faces),process=True).split(only_watertight=False):
                if len(part.vertices)<4 or np.linalg.matrix_rank(part.vertices-part.vertices.mean(0))<3:continue
                hull=part.convex_hull
                mesh('conveyor_component_'+str(component_count),hull.vertices,'environment',item['id']);component_count+=1
    model=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));data=mj.MjData(model)
    for e in entries:e['gid']=model.geom(e['name']).id
    adjacent={frozenset([j.parent,j.child]) for j in robot.joints};mount=mounting_pairs(snap['tool']);pairs=[]
    for i,a in enumerate(entries):
        for b in entries[i+1:]:
            if a['kind']==b['kind']=='environment' or a['kind']==b['kind']=='tool':continue
            if a['kind']==b['kind']=='robot' and frozenset([a['owner'],b['owner']]) in adjacent:continue
            names=frozenset([('robot_'+e['owner']) if e['kind']=='robot' else e['owner'] for e in [a,b]])
            allowance=names in mount or names==frozenset(['robot_base_link','robot_mount_plate'])
            pairs.append((a,b,-.0004 if allowance else .0024,allowance))
    checks=[];selected=None
    for delta in [0.,-np.pi,-np.pi/2,np.pi/2]:
        q=q0.copy();q[0]+=delta;record=dict(q_rad=q.tolist(),joint_limits=bool(robot.within_limits(q)),failures=[],queried_pairs=0)
        if not record['joint_limits']:checks.append(record);continue
        frames=robot.named_link_frames(q);cad=frames['link_6']@np.array(snap['tool']['T_flange_tool_cad'])
        record['TCP_D']=robot.fk(q).tolist();record['flange_D']=frames['link_6'].tolist()
        for row in moving:
            e=row['entry'];T=(frames[e['owner']] if e['kind']=='robot' else cad)@row['local'];bid=model.body(e['name']).mocapid[0]
            data.mocap_pos[bid]=T[:3,3];data.mocap_quat[bid]=Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]
        mj.mj_forward(model,data)
        for a,b,threshold,allowance in pairs:
            ga,gb=a['gid'],b['gid']
            if np.linalg.norm(data.geom_xpos[ga]-data.geom_xpos[gb])-model.geom_rbound[ga]-model.geom_rbound[gb]>.0024:continue
            distance=float(mj.mj_geomDistance(model,data,ga,gb,.0024,None));record['queried_pairs']+=1
            if distance+1e-9<threshold:record['failures'].append(dict(pair=[a['owner'],b['owner']],distance_m=distance,threshold_m=threshold,mount_allowance=allowance))
        record['available_geometry_clear']=not record['failures'];checks.append(record)
        if delta!=0 and not record['failures']:selected=record;break
    status='STATIC_CANDIDATE_CHECKED' if scene.get('mast',{}).get('parts') else 'STATIC_DIAGNOSTIC_MAST_UNAVAILABLE'
    report=dict(status=status,checks=checks,selected=selected,conveyor_convex_components=component_count,
        conveyor_representation='per_connected_component_conservative_convex_hulls; Isaac retains exact triangles',
        environment_obstacles=len(scene['obstacles']),mast=scene.get('mast'),seconds=time.perf_counter()-start,
        motion='NO_MOTION_EXECUTED',dynamics='NOT_EVALUATED',trajectory_from_old_HOME='NOT_EVALUATED',complete_scene_acceptance=False)
    save_json(folder/'static_candidate_checks.json',report)
    if selected:
        proposed=copy.deepcopy(pose);proposed.update(home_q=selected['q_rad'],source='bounded static candidate check; original retained in static_pose.json',new_scene_status=status,candidate_report=str(folder/'static_candidate_checks.json'))
        save_json(folder/'display_pose.json',proposed)
    print({k:report[k] for k in ['status','seconds','conveyor_convex_components']},'selected',selected is not None)
    print([(r['q_rad'][0],len(r['failures'])) for r in checks])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True);run(p.parse_args().folder)
