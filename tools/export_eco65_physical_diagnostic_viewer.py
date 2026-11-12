"""Display saved physical-task inputs and native diagnostic samples; no hardware IO."""
import argparse
from copy import deepcopy
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from export_eco65_search_replay import export_model
from unloading_sim.workbench.scene import load_snapshot
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA


def original_ptp_summary(record):
    """Report the archived log without inventing unsaved joint/contact samples."""
    log_path = record / 'moveit.log'
    log = log_path.read_text(errors='replace') if log_path.exists() else ''
    invalid = re.search(r'Invalid states at index locations: \[([^]]+)\] out of (\d+)', log)
    contacts = re.search(r"Found a contact between '([^']+)'[^\n]+ and '([^']+)'", log)
    pair = list(contacts.groups()) if contacts else []
    return dict(
        pair=pair,
        invalid_indices=[int(x) for x in invalid[1].split()] if invalid else [],
        state_count=int(invalid[2]) if invalid else None,
        sample_q_status='NOT_RECORDED_DO_NOT_FABRICATE',
        note=(f"原日志报告 {' 与 '.join(pair)} 碰撞，未保存对应关节样本。"
              if pair else '原记录没有可显示的直连碰撞关节样本。')
             + '红色姿态来自后续有完整关节记录的间隙检查。',
    )


def export(record, diagnostic, output):
    config=load(ROOT/'configs/workbench/single_carton.example.json')
    snapshot=load_snapshot(config['execution']['snapshot'],geometry_only=True)
    observation=json.loads((record/'observation.json').read_text())
    geometry=json.loads((record/'geometry.json').read_text())
    init=json.loads((diagnostic/'init.json').read_text())
    request=json.loads((diagnostic/'request.json').read_text())
    result=json.loads((diagnostic/'result.json').read_text())
    if result['original_task']!=record.name or request['identity']!=init['identity']:
        raise ValueError('DIAGNOSTIC_TASK_OR_MODEL_MISMATCH')
    if not np.allclose(request['q_start'],observation['q_start'],atol=1e-12,rtol=0):
        raise ValueError('DIAGNOSTIC_START_CHANGED')
    target_id=request['single_carton_contact']['target_id']
    box=next(b for b in geometry['boxes'] if b['id']==target_id)
    native_box=next(b for b in request['world'] if b['id']==target_id)
    if box['pose']!=native_box['pose'] or box['size_m']!=native_box['size']:
        raise ValueError('DIAGNOSTIC_BOX_CHANGED')
    # The existing exporter supplies the CAD visuals and original native URDF.
    # Static geometry comes exclusively from this physical request in world=base,
    # so no historical D-frame visuals or old simulation cartons are duplicated.
    display=deepcopy(snapshot);display['native_model']['init']=deepcopy(init)
    display['native_model']['init']['process_geometry']['expected_contour_ids']=init['process_geometry']['connected_cup_ids']
    display['scene']['mesh_obstacles']=[]
    model=export_model(display)
    if len(model['cup_ids'])!=12:raise ValueError('TWELVE_PHYSICAL_CUPS_REQUIRED')
    installation=Installation.load(config['execution']['installation'])
    coordinates=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
    if not np.allclose(installation.world_from_base,np.eye(4),atol=1e-12):raise ValueError('VIEWER_REQUIRES_CONFIRMED_WORLD_EQUALS_BASE')
    candidate=contact_candidates(box,snapshot['tool'],request['single_carton_contact']['geometry']['topology'])[0]
    if not np.allclose(candidate['precontact_pose'],request['goal_pose'],atol=1e-9):raise ValueError('CANDIDATE_RECONSTRUCTION_CHANGED')
    states=[dict(id='start',label='实测起点',q=observation['q_start'],note='本次任务锁定的 q_start；保存的实测记录，不是当前在线反馈。',pair=[])]
    examples=result['result']['clearance']['rejected_examples']
    for state_id,link,label in [('tool','part_028','吸具 / 肘部间隙不足'),('mast','eco65_mast_mast_part_019','支架 / 连杆间隙不足')]:
        failure=next((x for x in examples if link in x.get('pair',[])),None)
        if failure:
            states.append(dict(id=state_id,label=label,q=failure['q_rad'],pair=failure['pair'],failure=failure,
                note='后续离线复核保存的拒绝姿态；红色部件间隙不足，不代表已经发生实机碰撞。'))
    trace=result['result']['search_trace']['planners'][0]
    for node in trace['nodes']:
        if node['root'] and node['tree']=='GOAL':
            states.append(dict(id='goal',label='搜索目标姿态',q=node['q'],pair=[],note='实际搜索目标树根，仅为逆解终态；没有完整可执行路径。'))
            break
    q_capture=observation['q_capture']
    link1_camera=np.linalg.inv(coordinates.frames(q_capture)['link_1'])@np.asarray(observation['T_world_camera'])
    fixtures=[]
    for state in states:
        fixtures.append(dict(q=state['q'],frames={k:v.tolist() for k,v in coordinates.frames(state['q']).items()},tcp=coordinates.tcp(state['q']).tolist()))
    data=dict(schema='eco65_physical_diagnostic_viewer_v1',task_id=record.name,source_observation_id=observation['observation_id'],
        status='DIAGNOSTIC_REPLAY_NOT_EXECUTABLE',physical_execution='NOT_RUN',model=model,world=request['world'],
        box=box,candidate=candidate,states=states,trace=trace,fk_fixtures=fixtures,T_link1_camera=link1_camera.tolist(),
        captured_world_camera=observation['T_world_camera'],q_capture=q_capture,
        flange_from_tcp=init['identity']['flange_from_task_tcp'],
        original_ptp=original_ptp_summary(record),
        sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [record/'observation.json',record/'geometry.json',diagnostic/'init.json',diagnostic/'request.json',diagnostic/'result.json',Path(config['execution']['snapshot'])]
            + ([record/'moveit.log'] if (record/'moveit.log').exists() else [])})
    vendor=ROOT/'tools/search_replay/vendor'
    template=(ROOT/'tools/search_replay/eco65_physical_viewer.html').read_text()
    scripts=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    html=template.replace('/*__VENDOR__*/',scripts).replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(html)
    report=dict(html=str(output),bytes=output.stat().st_size,task_id=record.name,physical_execution='NOT_RUN',execution_interface=False,
        visual_shapes=sum(x['layer']=='visual' for x in model['links']),collision_shapes=sum(x['layer']=='collision' for x in model['links']),
        world_objects=len(request['world']),cups=len(model['cup_ids']),states=[x['id'] for x in states],sources=data['sources'])
    output.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record',type=Path,required=True);parser.add_argument('--diagnostic',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();export(args.record.resolve(),args.diagnostic.resolve(),args.output.resolve())
