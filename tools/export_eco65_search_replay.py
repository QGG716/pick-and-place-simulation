"""Self-contained offline ECO65 diagnostic and complete-plan viewer; no physics."""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
from time import perf_counter
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from unloading_sim.eco65.model import ROOT, load_json, robot_for
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.native_replay import replay_inputs, state_at
from export_m710_search_replay import origin, validate_attempt


def export_model(snapshot):
    native=snapshot['native_model']['init'];scene=snapshot['scene'];tool=snapshot['tool']
    urdf=ET.fromstring(native['parameters']['robot_description'])
    joints=[];links=[];assets={}
    def mesh_data(mesh):
        positions=np.asarray(mesh.vertices,dtype='<f4');faces=np.asarray(mesh.faces,dtype='<u4')
        key=hashlib.sha256(positions.tobytes()+faces.tobytes()).hexdigest()
        if key not in assets:assets[key]=dict(positions_b64=base64.b64encode(positions.tobytes()).decode(),
             indices_b64=base64.b64encode(faces.tobytes()).decode(),triangles=len(faces))
        return key
    def load_mesh(name):
        name=str(name).removeprefix('file://')
        if name.startswith('/work-eco65-20261015/'):name=name.split('/work-eco65-20261015/',1)[1]
        p=Path(name);p=p if p.is_absolute() else ROOT/p
        return trimesh.load(p,force='mesh',process=False)
    def mesh_item(link,name,m,T,layer,rgb=None):
        links.append(dict(link=link,id=name,kind='mesh',asset=mesh_data(m),origin=np.asarray(T).tolist(),
                          layer=layer,rgb=rgb,tool=name.startswith('part_')))
    for j in urdf.findall('joint'):
        joints.append(dict(name=j.get('name'),type=j.get('type'),parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),origin=origin(j.find('origin')).tolist(),
            axis=list(map(float,j.find('axis').get('xyz').split())) if j.find('axis') is not None else [0,0,1]))
    for link in urdf.findall('link'):
        name=link.get('name')
        for layer in ('visual','collision'):
            for i,c in enumerate(link.findall(layer)):
                g=c.find('geometry');item=dict(link=name,id=name,origin=origin(c.find('origin')).tolist(),
                    tool=name in native['tool_links'],layer=layer)
                if g.find('mesh') is not None:
                    m=g.find('mesh');mesh=load_mesh(m.get('filename'))
                    mesh.vertices=np.asarray(mesh.vertices)*np.fromstring(m.get('scale','1 1 1'),sep=' ')
                    item.update(kind='mesh',asset=mesh_data(mesh))
                elif g.find('box') is not None:item.update(kind='box',size=np.fromstring(g.find('box').get('size'),sep=' ').tolist())
                elif g.find('cylinder') is not None:
                    e=g.find('cylinder');item.update(kind='cylinder',radius=float(e.get('radius')),length=float(e.get('length')))
                else:raise ValueError('UNSUPPORTED_NATIVE_GEOMETRY:'+ET.tostring(g).decode())
                links.append(item)
                if layer=='collision' and name in {r['id'] for r in tool['adapter']}:
                    links.append(dict(item,layer='visual'))
    for part in tool['visual_parts']:
        mesh_item('link_6',part['id'],load_mesh(part['visual']),tool['T_flange_tool_cad'],'visual',part.get('rgb'))
    mast=scene.get('mast',{});camera=scene.get('permanent_camera')
    if mast.get('parts'):
        if mast['attachment']!='link_1':raise ValueError('J1_MAST_ATTACHMENT_REQUIRED')
        T=np.asarray(mast['T_link1_mast'])
        for part in mast['parts']:
            if camera and part['id'] in camera['replaced_mast_parts']:continue
            mesh_item('link_1','eco65_mast_'+part['id'],load_mesh(part['visual']),
                      T@np.asarray(part.get('T_mast',np.eye(4))),'visual',part.get('rgb'))
        seat=mast.get('mount_interface',{}).get('seat')
        if seat:mesh_item('link_1','eco65_mast_mounting_seat',load_mesh(seat['mesh']),np.eye(4),'visual')
        if camera:
            for part in camera['parts']:
                mesh_item('link_1','camera_'+part['id'],load_mesh(part['visual']),T@np.asarray(camera['T_mast_CAD']),'visual',part.get('rgb'))
            for part in camera.get('pitch_adapter',[]):
                mesh_item('link_1',part['name'],trimesh.creation.box(part['size_m']),T@np.asarray(part['T_mast']),'visual')
    static_visual=[]
    # Same USD references as the workstation exporter, authored in D, no T_W_D application.
    if scene.get('mesh_obstacles'):
        from pxr import Usd, UsdGeom
        for row in scene['mesh_obstacles']:
            path=ROOT/row['usd_path']
            if hashlib.sha256(path.read_bytes()).hexdigest()!=row['usd_sha256']:raise ValueError('VISUAL_SOURCE_CHANGED')
            usd=Usd.Stage.Open(str(path));cache=UsdGeom.XformCache()
            for prim in usd.Traverse():
                if not prim.IsA(UsdGeom.Mesh):continue
                m=UsdGeom.Mesh(prim);points=np.asarray(m.GetPointsAttr().Get(),float)
                indices=list(m.GetFaceVertexIndicesAttr().Get());counts=list(m.GetFaceVertexCountsAttr().Get())
                triangles=[];offset=0
                for count in counts:
                    face=indices[offset:offset+count];offset+=count
                    triangles.extend([face[0],face[j],face[j+1]] for j in range(1,count-1))
                colors=m.GetDisplayColorAttr().Get()
                static_visual.append(dict(id=row['id']+str(prim.GetPath()),kind='mesh',
                    asset=mesh_data(trimesh.Trimesh(vertices=points,faces=triangles,process=False)),
                    pose=np.asarray(cache.GetLocalToWorldTransform(prim)).T.tolist(),rgb=list(colors[0]) if colors else [.35,.4,.45]))
    return dict(joints=joints,links=links,meshes=assets,joint_names=native['joint_names'],identity=native['identity'],
                joint_limits_rad=[[float(snapshot['native_model']['official_limits'][n][k])
                    for k in ('lower','upper')] for n in native['joint_names']],
                layout_installation=scene.get('layout_installation'),
                prototype=scene.get('prototype'),
                base_translation=scene.get('base_translation'),
                camera_definition=None if not camera else dict(
                    T_link1_mast=mast['T_link1_mast'],T_mast_optical=camera['T_mast_optical'],
                    T_mast_CAD=camera['T_mast_CAD'],
                    axis_rotation=camera.get('axis_rotation'),mount=camera.get('mount'),
                    calibration=camera.get('calibration'),layout_remount=camera.get('layout_remount')),
                flange_from_contact_frame=native['process_geometry']['flange_from_contact_frame'],
                edge_alignment=native['process_geometry'].get('edge_alignment'),
                tcp_link=native['identity']['task_tcp_link'],flange_link='link_6',static_visual=static_visual,
                cup_ids=native['process_geometry']['expected_contour_ids'])


def diagnostic_attempt(folder,snapshot):
    run=load_json(folder/'run.json');req=load_json(folder/'request.json')
    response=load_json(folder/'response.json') if (folder/'response.json').exists() else dict(status=run['status'])
    trace=None
    if (folder/'search_trace.json.gz').exists():
        with gzip.open(folder/'search_trace.json.gz','rt') as f:trace=json.load(f)
        ready=load_json(folder/'initialization.json')
        trace.setdefault('resident_seed',ready.get('seed'))
        trace.setdefault('resident_seed_source','SAME_RUN_ARCHIVED_READY_RESPONSE')
        trace.setdefault('rng_semantics','OMPL_RNG_SEEDED_ONCE_AT_RESIDENT_INIT_REQUEST_SEED_IS_A_REQUEST_LABEL')
        req=trace['input']
    if req['identity']!=snapshot['native_model']['init']['identity']:raise ValueError('DIAGNOSTIC_MODEL_MISMATCH')
    if req['attachment'] and req['attachment']['id'] in {o['id'] for o in req['world']}:raise ValueError('DUPLICATE_TARGET')
    a=dict(run_id=run['run_id'],request=req,response_identity=response,trace=trace,
           native_path=response.get('points',[]),final_timed_path=[],adopted=False,
           archive=str(folder),trace_status=run.get('trace_status','NOT_RECORDED'))
    if trace:validate_attempt(a)
    return a


def task_diagnostic_attempts(folder,snapshot):
    """Independent partial-run adapter; never passes partial records to replay_inputs."""
    attempts=[];native_ready={}
    with gzip.open(folder/'native_calls.jsonl.gz','rt') as stream:
        for line in stream:
            row=json.loads(line);req=row['request'];response=row.get('result',{})
            if req['op']=='init':native_ready=response
            if req['op'] not in ('plan','diagnostic_connection'):continue
            if req['identity']!=snapshot['native_model']['init']['identity']:raise ValueError('DIAGNOSTIC_MODEL_MISMATCH')
            trace=None;reference=response.get('search_trace_ref')
            if reference:
                path=ROOT/reference['path']
                if hashlib.sha256(path.read_bytes()).hexdigest()!=reference['sha256']:raise ValueError('TRACE_ARCHIVE_CHANGED')
                with gzip.open(path,'rt') as f:trace=json.load(f)
                trace.setdefault('resident_seed',native_ready.get('seed'))
                trace.setdefault('resident_seed_source','SAME_RUN_ARCHIVED_READY_RESPONSE' if 'seed' in native_ready else 'NOT_CAPTURED')
                trace.setdefault('rng_semantics','OMPL_RNG_SEEDED_ONCE_AT_RESIDENT_INIT_REQUEST_SEED_IS_A_REQUEST_LABEL')
            attempt=dict(run_id=req['task_id'],request=trace['input'] if trace else req,response_identity=response,
                trace=trace,native_path=response.get('points',[]),final_timed_path=[],adopted=False,archive=str(folder),
                trace_status=trace['status'] if trace else response.get('search_trace_status',
                    'INTERRUPTED_NOT_CAPTURED' if 'error' in row or response.get('status')=='ERROR' else 'NOT_RECORDED' if req['pipeline_id']=='ompl' else 'NOT_APPLICABLE_PILZ'))
            if trace:validate_attempt(attempt)
            attempts.append(attempt)
    return attempts


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--diagnostic',type=Path,action='append',default=[]);p.add_argument('--plan',type=Path)
    p.add_argument('--task-diagnostic',type=Path,action='append',default=[])
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();start=perf_counter()
    a.output.resolve().relative_to(ROOT/'outputs')
    if a.output.exists():raise FileExistsError('Use fresh HTML path')
    snapshot=verify_snapshot(load_json(a.snapshot));model=export_model(snapshot)
    attempts=[diagnostic_attempt(folder,snapshot) for folder in a.diagnostic]
    for folder in a.task_diagnostic:attempts.extend(task_diagnostic_attempts(folder,snapshot))
    plan=load_json(a.plan) if a.plan else None
    plan_fixtures=[]
    if plan:
        robot=replay_inputs(snapshot,plan)
        offset=0.
        for stage in plan['stages']:
            duration=stage['times'][-1]
            for time in (offset,offset+duration*.37,offset+duration):
                state=state_at(snapshot,plan,robot,time)
                plan_fixtures.append(dict(time_s=time,q=state['q'].tolist(),phase=state['phase'],
                    attached=state['attached'],target_pose=state['cartons'][snapshot['scene']['target_box_id']].tolist()))
            offset+=duration
        final=[]
        for s in plan['stages']:
            req=s['native_request'];response=s['native_result'];trace=None
            reference=response.get('search_trace_ref')
            if reference:
                path=ROOT/reference['path']
                if hashlib.sha256(path.read_bytes()).hexdigest()!=reference['sha256']:raise ValueError('TRACE_ARCHIVE_CHANGED')
                with gzip.open(path,'rt') as f:trace=json.load(f)
            attempts.append(dict(run_id=plan['task_id'],request=trace['input'] if trace else req,response_identity=response,trace=trace,
                native_path=response.get('points',[]),final_timed_path=[dict(q=q,t=t) for q,t in zip(s['q'],s['times'])],
                adopted=True,archive=str(a.plan),trace_status='CAPTURED' if trace else 'NOT_RECORDED' if req['pipeline_id']=='ompl' else 'NOT_APPLICABLE_PILZ'))
            final.append({k:s[k] for k in ('stage','stage_id','q','times','attachment','target_pose_start')})
        plan={k:plan[k] for k in ('fingerprint','task_id','events','home_q','active_ids','target_face','simulated_duration_s')}|dict(stages=final)
    if not attempts:raise ValueError('REQUEST_OR_COMPLETE_PLAN_REQUIRED')
    attempts.sort(key=lambda row:not(row['request']['stage']=='transit' and row.get('trace',{} ) and row['trace'].get('planners')))
    robot=robot_for(snapshot['tool'],snapshot['scene']);fixtures=[]
    for attempt in attempts[:3]:
        q=attempt['request']['q_start'];frames=robot.named_link_frames(q)
        fixtures.append(dict(q=q,frames={k:np.asarray(v).tolist() for k,v in frames.items()},tcp=robot.fk(q).tolist()))
    # Share exact immutable world geometry across requests, including trace.input.
    # The browser restores the original world array from this content key.
    worlds={}
    for attempt in attempts:
        req=dict(attempt['request']);world=req.pop('world')
        key=hashlib.sha256(json.dumps(world,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        worlds.setdefault(key,world);req['world_ref']=key;attempt['request']=req
        if attempt.get('trace'):
            trace_input=dict(attempt['trace']['input']);trace_input.pop('world');trace_input['world_ref']=key
            attempt['trace']['input']=trace_input
    data=dict(schema='eco65_search_replay_v1',model=model,attempts=attempts,plan=plan,
        shared_worlds=worlds,
        snapshot_fingerprint=snapshot['fingerprint'],scene_target=snapshot['scene']['target_box_id'],
        receiving_pose=snapshot['scene']['native_task']['receiving']['box_pose_D'],fk_fixtures=fixtures,plan_fixtures=plan_fixtures,
        coordinate_frame='D: +X into trailer, +Y left, +Z up; metres/radians/seconds; T_W_D not applied',
        status='KINEMATIC_REPLAY',planning_status='PLANNING_ONLY_NOT_EXECUTABLE')
    vendor=ROOT/'tools/search_replay/vendor';scripts=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    template=(ROOT/'tools/search_replay/eco65_viewer.html').read_text(encoding='utf-8')
    html=template.replace('/*__VENDOR__*/',scripts).replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(html,encoding='utf-8')
    report=dict(html=str(a.output),sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),bytes=a.output.stat().st_size,
        export_wall_s=perf_counter()-start,requests=len(attempts),complete=bool(plan),cups=len(model['cup_ids']),
        visual_meshes=sum(r['layer']=='visual' for r in model['links']),collision_shapes=sum(r['layer']=='collision' for r in model['links']),
        snapshot_fingerprint=snapshot['fingerprint'],coordinate_frame=data['coordinate_frame'],fk_fixtures=fixtures)
    (a.output.with_suffix('.json')).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='fk_fixtures'}))


if __name__=='__main__':main()
