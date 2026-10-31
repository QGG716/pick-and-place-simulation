"""Measure actual collision-mesh extents at accepted native waypoints.

This explains finite enclosure choices. It is neither another collision audit
nor a continuous swept-volume proof; no synthetic trajectories are generated.
"""
import argparse
import base64
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from export_eco65_search_replay import export_model
from unloading_sim.eco65.model import load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot


def measure(snapshot_path, plan_path, output):
    snapshot=verify_snapshot(load_json(snapshot_path));plan=load_json(plan_path)
    if plan['snapshot_fingerprint']!=snapshot['fingerprint']:
        raise ValueError('ENVELOPE_PLAN_SCENE_MISMATCH')
    model=export_model(snapshot);solids=[]
    for row in model['links']:
        if row['layer']!='collision':continue
        if row.get('asset'):
            vertices=np.frombuffer(base64.b64decode(model['meshes'][row['asset']]['positions_b64']),dtype=np.float32).reshape(-1,3)
        elif row.get('size'):vertices=trimesh.creation.box(row['size']).vertices
        elif row.get('kind')=='cylinder':
            vertices=trimesh.creation.cylinder(radius=row['radius'],height=row['length']).vertices
        else:raise ValueError('UNSUPPORTED_ENVELOPE_SOLID:'+row['id'])
        t=np.asarray(row['origin']);solids.append((row['id'],row['link'],vertices@t[:3,:3].T+t[:3,3]))
    axes={'left':(1,1.),'right':(1,-1.),'roof':(2,1.)};maxima={k:None for k in axes};count=0
    joints=model['joints'];indices={n:i for i,n in enumerate(model['joint_names'])}
    for record in plan['records']:
        if record['kind']!='robot_motion':continue
        for index,q in enumerate(record['q']):
            count+=1;frames={'world':np.eye(4)};pending=list(joints)
            while pending:
                later=[]
                for joint in pending:
                    if joint['parent'] not in frames:later.append(joint);continue
                    t=frames[joint['parent']]@np.asarray(joint['origin'])
                    if joint['name'] in indices:
                        value=q[indices[joint['name']]];motion=np.eye(4)
                        if joint['type']=='prismatic':motion[:3,3]=np.asarray(joint['axis'])*value
                        else:motion[:3,:3]=Rotation.from_rotvec(np.asarray(joint['axis'])*value).as_matrix()
                        t=t@motion
                    frames[joint['child']]=t
                if len(later)==len(pending):raise ValueError('FRAME_TREE_UNRESOLVED')
                pending=later
            for name,parent,vertices in solids:
                t=frames[parent];world=vertices@t[:3,:3].T+t[:3,3]
                for label,(axis,sign) in axes.items():
                    i=int(np.argmax(sign*world[:,axis]));extent=float(sign*world[i,axis])
                    if maxima[label] is None or extent>maxima[label]['extent_m']:
                        maxima[label]=dict(extent_m=extent,point_D_m=world[i].tolist(),solid_id=name,
                            record_id=record['record_id'],waypoint_index=index,q_rad=q)
    margin=snapshot['native_model']['init']['collision_policy']['required_pair_clearance_m']
    result=dict(schema='eco65_recorded_waypoint_mesh_envelope_v1',snapshot_fingerprint=snapshot['fingerprint'],
        plan_fingerprint=plan['fingerprint'],completed_ids=plan['terminal_state']['completed_ids'],
        waypoint_count=count,extrema=maxima,margin_m=margin,
        symmetric_inner_width_for_this_sampled_envelope_m=2*(max(maxima[k]['extent_m'] for k in ('left','right'))+margin),
        inner_height_for_this_sampled_envelope_m=maxima['roof']['extent_m']+margin,
        scope='ACCEPTED_NATIVE_WAYPOINTS_ONLY_ALL_ACTUAL_ROBOT_TOOL_MAST_COLLISION_MESHES',
        continuous_swept_proof=False,global_minimum_enclosure_proof=False,physical_execution='NOT_RUN')
    save_json(output,result);print(result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','plan','output'):p.add_argument('--'+name,required=True)
    a=p.parse_args();measure(a.snapshot,a.plan,a.output)
