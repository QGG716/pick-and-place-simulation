"""Additional +180 degree J6 preview from saved poses. Offline, no path or permit.

Keep J1--J5 and the box world pose fixed. Recompute CAD seal coverage and the
attachment for the alternative tool orientation, and query native static FK /
collision checks. The rotation between these endpoints is NOT a checked path.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
from visualize_field_planning_failures import read_case, export_model
from unloading_sim.moveit2_backend import ResidentMoveItClient
from unloading_sim.workbench.single_carton.config import load, wrist_branch_violation
from unloading_sim.workbench.single_carton.geometry import transform, seal_check
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.scene import load_snapshot
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA, load_mapping


TURN=np.diag([-1.,-1.,1.,1.])


def rotate_q(q):
    result=np.array(q,float,copy=True);result[5]+=np.pi
    return result.tolist()  # No angle wrapping or change to J1--J5.


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=load(ROOT/'configs/workbench/single_carton.example.json')
    ins=Installation.load(config['execution']['installation'])
    model=CoordinateModel(URDF,URDF_SHA,ins.world_from_base,ins.tool)
    authority=Authority('eco65_single_carton_segment_v1')
    planner=SinglePlanner(config,load_mapping(config['execution']['mapping']),authority,model);planner.build()
    snapshot=deepcopy(load_snapshot(config['execution']['snapshot'],geometry_only=True))
    # The visual exporter adds contour presentation fields. Never add those to
    # the independent native request identity / topology binding.
    snapshot['native_model']['init']=deepcopy(planner.init);snapshot['scene']['mesh_obstacles']=[]
    snapshot['native_model']['init']['process_geometry']['expected_contour_ids']=planner.init['process_geometry']['connected_cup_ids']
    visual=export_model(snapshot);cases=[];evidence=[]
    tool=planner.source['tool'];flange_tcp=np.asarray(tool['T_flange_tcp'])
    cad=np.linalg.inv(flange_tcp)@tool['T_flange_tool_cad']
    rings={s['id']:transform(cad,s['ring_cad_m']) for s in tool['seals']}
    normal=cad[:3,:3]@tool['seals'][0]['normal_cad']
    client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(out),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=out/'native.log')
    try:
        client.request(planner.init)
        for number,path in enumerate(args.records,1):
            case=read_case(path.resolve(),model)
            selected=next(c for c in case['candidates'] if c['index']==case['default_candidate'])
            folder=path/(case['box']['id']+'-'+str(selected['index']))
            stages=[]
            for stage in ('approach','contact'):
                source=folder/(stage+'-endpoint-native.json')
                saved=json.loads(source.read_text());stages.append((stage,saved['request']['q_start'],saved['request'],source))
            source=folder/'detach-native.json'
            if source.exists():
                saved=json.loads(source.read_text());points=saved.get('result',{}).get('points',[])
                if points:stages.append(('detach',points[-1]['q'],saved['request'],source))
            contact_request=stages[1][2];face=contact_request['single_carton_contact']['face']
            contact_q=stages[1][1];contact_after=model.tcp(rotate_q(contact_q))
            after_ids=[k for k,r in rings.items() if seal_check(contact_after,face,{k:r},normal)['valid']]
            variants=[]
            for stage,q,original,source in stages:
                case['sources'][str(source.resolve().relative_to(ROOT))]=hashlib.sha256(source.read_bytes()).hexdigest()
                assert original['identity']==planner.init['identity'],'MODEL_IDENTITY_CHANGED'
                for after in (False,True):
                    state=rotate_q(q) if after else list(q)
                    actual=model.tcp(state);before=model.tcp(q)
                    expected=before@np.linalg.inv(flange_tcp)@TURN@flange_tcp if after else before
                    assert np.max(abs(actual-expected))<1e-12,'J6_AXIS_TRANSFORM_MISMATCH'
                    assert np.array_equal(np.asarray(state)[:5],np.asarray(q)[:5]),'ARM_POSTURE_CHANGED'
                    query=deepcopy(original)
                    for key in ('absolute_deadline_monotonic_s','deadline_clock_reference_monotonic_s','goal_pose',
                                'pipeline_id','planner_id','velocity_scale','acceleration_scale','allowed_planning_time_s',
                                'defer_output_check_to_sent_path','flange_from_task_tcp'):
                        query.pop(key,None)
                    ids=after_ids if after else original['single_carton_contact']['active_ids']
                    query.update(op='validate',q_start=state,request_budget_s=110.)
                    query['single_carton_contact']['active_ids']=ids
                    if after and query.get('attachment'):
                        # Alternative grasp of the same box, not a loaded spin.
                        query['attachment']['pose']=(TURN@np.asarray(query['attachment']['pose'])).tolist()
                    attachment=query.get('attachment')
                    if attachment:
                        old_world=model.frames(q)['link_6']@np.asarray(original['attachment']['pose'])
                        new_world=model.frames(state)['link_6']@np.asarray(attachment['pose'])
                        assert np.max(abs(old_world-new_world))<1e-12,'BOX_WORLD_POSE_CHANGED'
                    ik=dict(op='ik',identity=planner.init['identity'],q_start=state,goal_pose=actual.tolist(),seed=6518,
                            planning_only=False,position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4,request_budget_s=110.)
                    target=out/f'case-{number}-{stage}-{"after" if after else "before"}.json'
                    write(target,dict(status='PENDING',ik_request=ik,collision_request=query))
                    ik_reply=client.request(ik,timeout=120)
                    assert ik_reply.get('status')=='SUCCESS' and np.max(abs(np.asarray(ik_reply['q'])-state))<1e-6,'NATIVE_IK_CHANGED_BRANCH'
                    reply=client.request(query,timeout=120)
                    write(target,dict(status='REPLIED',ik_request=ik,ik_result=ik_reply,collision_request=query,collision_result=reply))
                    violation=wrist_branch_violation(config['wrist_branch_limits'],case['q_start'],[state])
                    valid=bool(reply.get('native_valid') and reply.get('legacy_intersection_valid'))
                    variant=dict(stage=stage,after=after,q=state,original_q=list(q),tcp_pose=actual.tolist(),
                        attachment=attachment,sealing_ids=ids,anticipated_seals=len(ids),contact_scope='RECHECKED_AT_CONTACT_NOT_PRESSURE_VERIFIED',
                        wrist_rejection=violation,static_valid=valid,failure=reply.get('failure'),source=str(source.resolve().relative_to(ROOT)),
                        native_evidence=str(target.relative_to(ROOT)),native_position_error_m=ik_reply.get('position_error_m'),
                        native_orientation_error_rad=ik_reply.get('orientation_error_rad'),
                        j6_delta_deg=180. if after else 0.,path_validated=False,executable=False)
                    variants.append(variant)
            case['candidates']=[dict(selected,rotation_variants=variants,paths=[])]
            case['rotation_preview']=True;cases.append(case)
            evidence.append(dict(task_id=case['task_id'],candidate=selected['index'],variants=variants,sources=case['sources']))
    finally:client.close()
    assert not authority._plans,'PREVIEW_MUST_NOT_REGISTER_PERMITS'
    for case in cases:
        for name,digest in case['sources'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'ARCHIVE_CHANGED'
    data=dict(model=visual,cases=cases,retry=True,rotation_preview=True,physical_execution='NOT_RUN',executable=False)
    template=(ROOT/'tools/search_replay/field_failures.html').read_text()
    extension=(ROOT/'tools/search_replay/tool_rotation.js').read_text()
    vendor=ROOT/'tools/search_replay/vendor';js=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    html=template.replace('/*__VENDOR__*/',js).replace('/*__DATA__*/',json.dumps(data,separators=(',',':')).replace('</','<\\/'))
    (out/'index.html').write_text(html.replace('</html>','<script>'+extension+'</script></html>'))
    report=dict(scope='ADDITIONAL_J6_PLUS_180_STATIC_PREVIEW_ONLY',source_viewer='outputs/field-posture-retry-20261118/j4-free-viewer/index.html',
        rotation_convention='RIGHT_HAND_POSITIVE_J6; CCW_LOOKING_FROM_POSITIVE_AXIS_TOWARD_ORIGIN',
        cases=evidence,source_unchanged=True,collision_shapes=sum(x['layer']=='collision' for x in visual['links']),cups=len(visual['cup_ids']),
        native_queries=sum(len(c['variants']) for c in evidence),hardware_execution='NOT_RUN',paths_checked=False,permits_issued=0,
        production_config_changed=False,calibration_changed=False)
    write(out/'report.json',report)
    print(json.dumps(dict(output=str(out),cases=len(cases),native_queries=report['native_queries'],hardware_execution='NOT_RUN')))


if __name__=='__main__':main()
