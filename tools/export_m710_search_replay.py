"""Export same-run native search history and exact model as offline display-only HTML."""
from pathlib import Path
import argparse,base64,gzip,hashlib,json,struct,xml.etree.ElementTree as ET
import numpy as np
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.geometry import make_transform,rotation_matrix_from_rpy

def origin(element):
    if element is None:return np.eye(4)
    return make_transform(rotation_matrix_from_rpy(*map(float,element.get('rpy','0 0 0').split())),list(map(float,element.get('xyz','0 0 0').split())))

def mesh_vertices(path):
    data=path.read_bytes()
    n=struct.unpack_from('<I',data,80)[0] if len(data)>=84 else 0
    if len(data)==84+50*n:
        dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
        return np.frombuffer(data,dtype=dtype,count=n,offset=84)['vertices'].reshape(-1,3)
    rows=[list(map(float,l.split()[1:])) for l in data.decode().splitlines() if l.strip().startswith('vertex ')]
    return np.asarray(rows,dtype=np.float32)

def export_model(model):
    urdf=ET.fromstring(model['parameters']['robot_description'])
    joints=[];links=[];assets={}
    for j in urdf.findall('joint'):
        joints.append(dict(name=j.get('name'),type=j.get('type'),parent=j.find('parent').get('link'),child=j.find('child').get('link'),
            origin=origin(j.find('origin')).tolist(),axis=list(map(float,j.find('axis').get('xyz').split())) if j.find('axis') is not None else [0,0,1]))
    for link in urdf.findall('link'):
        name=link.get('name')
        # Official collision meshes, complete tool colliders: exact geometry used by native planning.
        for c in link.findall('collision'):
            g=c.find('geometry');item=dict(link=name,origin=origin(c.find('origin')).tolist(),tool=name in model['tool_links'])
            if g.find('box') is not None:item.update(kind='box',size=list(map(float,g.find('box').get('size').split())))
            elif g.find('mesh') is not None:
                m=g.find('mesh');path=Path(m.get('filename').removeprefix('file://'));key=hashlib.sha256(path.read_bytes()).hexdigest()
                if key not in assets:
                    vertices=mesh_vertices(path)*np.array(list(map(float,m.get('scale','1 1 1').split())))
                    assets[key]=dict(positions_b64=base64.b64encode(np.asarray(vertices,dtype='<f4').tobytes()).decode(),triangles=len(vertices)//3,source=str(path))
                item.update(kind='mesh',asset=key)
            elif g.find('cylinder') is not None:
                e=g.find('cylinder');item.update(kind='cylinder',radius=float(e.get('radius')),length=float(e.get('length')))
            else:raise ValueError('Unsupported geometry: '+ET.tostring(g).decode())
            links.append(item)
    return dict(joints=joints,links=links,meshes=assets,joint_names=model['joint_names'],identity=model['identity'],
        flange_from_contact=model['process_geometry']['flange_from_physical_contact'])

def validate_attempt(a):
    trace=a.get('trace')
    if not trace:return dict(status='SEARCH_HISTORY_NOT_RECORDED')
    req=trace['input'];response=a['response_identity']
    for k in ('task_id','stage_id','parent_stage_id','request_id'):
        assert req[k]==response[k],(k,req.get(k),response.get(k))
    if req['attachment']:
        assert req['attachment']['id'] not in [b['id'] for b in req['world']]
    for p in trace['planners']:
        nodes=p['nodes'];assert len(nodes)==p['planner_data_vertices'];assert len(p['edges'])==p['planner_data_edges']
        assert [n['id'] for n in nodes]==list(range(len(nodes)))
        join=p['join'];parents={}
        for first,second in p['edges']:
            assert 0<=first<len(nodes) and 0<=second<len(nodes)
            if [first,second]==join:continue
            assert nodes[first]['tree']==nodes[second]['tree']
            parent,child=(first,second) if nodes[first]['tree']=='START' else (second,first)
            assert child not in parents;parents[child]=parent
        for child in parents:
            seen=set();cur=child
            while cur in parents:assert cur not in seen;seen.add(cur);cur=parents[cur]
            if not nodes[cur]['root']:
                assert nodes[cur]['tree']=='GOAL' and np.max(np.abs(np.asarray(nodes[cur]['q'])-req['q_goal']))<1e-10
        for e in p['events']:
            assert e[2] == (e[7]=='ACCEPTED')
            assert 0<=e[5]<=e[6]<=len(p['checked_samples'])
        assert p['total_extensions']>=len(p['events'])
    points=a.get('final_timed_path') or []
    if points:
        assert np.allclose(points[0]['q'],req['q_start'],atol=1e-12,rtol=0)
        assert all(points[i]['t']<=points[i+1]['t'] for i in range(len(points)-1))
    return dict(status='PASS_TRACE_STRUCTURE_ONLY',planners=len(trace['planners']),qualification_status='NOT_EVALUATED')

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--label',required=True);args=p.parse_args()
    raw=json.loads((args.run/'search-trace.json').read_text());result=json.loads((args.run/'result.json').read_text())
    assert raw['run_id']==result['run_id'] and raw['source_commit']==result['source_commit']
    checks=[validate_attempt(a) for a in raw['attempts']]
    data=dict(schema='m710_replay_display_v1',label=args.label,raw=raw,result=result,model=export_model(raw['model']),checks=checks,
        status='PLANNING_REPLAY_NOT_PHYSICAL_EXECUTION',qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN')
    template=(ROOT/'tools/search_replay/viewer.html').read_text()
    vendor=ROOT/'tools/search_replay/vendor'
    scripts=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    html=template.replace('/*__VENDOR__*/',scripts).replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(html,encoding='utf-8')
    (args.output.parent/(args.output.stem+'-checks.json')).write_text(json.dumps(checks,indent=2))
    print(json.dumps(dict(html=str(args.output),bytes=args.output.stat().st_size,run_id=raw['run_id'],checks=checks)))

if __name__=='__main__':main()
