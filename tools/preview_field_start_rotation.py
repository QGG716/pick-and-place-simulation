"""Show a saved measured start and a clockwise J6 half-turn with its new plan.

Offline static queries only. The measured q_start/q_capture and plan are never
changed. No physical transports, path interpolation or execution permits.
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
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.scene import load_snapshot
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA, load_mapping


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record',type=Path)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--j6-deg',type=float,choices=(-180.,),default=-180.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();record=args.record.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=load(args.config);ins=Installation.load(config['execution']['installation'])
    model=CoordinateModel(URDF,URDF_SHA,ins.world_from_base,ins.tool)
    authority=Authority('eco65_single_carton_segment_v1')
    planner=SinglePlanner(config,load_mapping(config['execution']['mapping']),authority,model);planner.build()
    geometry=json.loads((record/'geometry.json').read_text())
    cases=[read_case(record,model,b['id']) for b in geometry['boxes']]
    if not cases:raise ValueError('NO_RECOVERED_TARGET_FOR_VIEWER')
    q0=np.asarray(cases[0]['q_start']);q1=q0.copy();q1[5]+=np.deg2rad(args.j6_deg)
    flange_tcp=np.asarray(planner.source['tool']['T_flange_tcp'])
    expected=model.tcp(q0)@np.linalg.inv(flange_tcp)@np.diag([-1.,-1.,1.,1.])@flange_tcp
    assert np.max(abs(model.tcp(q1)-expected))<1e-12,'WRONG_ROTATION_AXIS'
    assert np.array_equal(q0[:5],q1[:5]),'ARM_POSTURE_CHANGED'
    # Use a real generated contact only to bind the unchanged strict native
    # approach policy. This does not turn a target-level rejection into a plan.
    static_candidates=contact_candidates(geometry['boxes'][0],planner.source['tool'],config['topology'],config.get('contact_search'))
    if not static_candidates:raise ValueError('NO_CONTACT_CONTEXT_FOR_STRICT_STATIC_QUERY')
    world=cases[0]['world']+[dict(id=cases[0]['box']['id'],size=cases[0]['box']['size_m'],pose=cases[0]['box']['pose'],category='carton')]
    variants=[]
    client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(out),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=out/'native.log')
    try:
        write(out/'native-init.json',planner.init);client.request(planner.init)
        for name,q in (('measured',q0),('rotated-preview',q1)):
            tcp=model.tcp(q)
            ik=dict(op='ik',identity=planner.init['identity'],q_start=q.tolist(),goal_pose=tcp.tolist(),seed=6518,
                    planning_only=False,position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4,request_budget_s=110.)
            check=dict(planner.context(q,world,cases[0]['box'],static_candidates[0],'approach'),op='validate',request_budget_s=110.)
            path=out/(name+'.json');write(path,dict(status='PENDING',ik_request=ik,collision_request=check))
            ik_result=client.request(ik,timeout=120);result=client.request(check,timeout=120)
            write(path,dict(status='REPLIED',ik_request=ik,ik_result=ik_result,collision_request=check,collision_result=result))
            assert ik_result.get('status')=='SUCCESS' and np.max(abs(np.asarray(ik_result['q'])-q))<1e-6,'NATIVE_IK_CHANGED_BRANCH'
            variants.append(dict(name=name,q=q.tolist(),tcp_pose=tcp.tolist(),j6_delta_deg=0. if name=='measured' else args.j6_deg,
                static_valid=bool(result.get('native_valid') and result.get('legacy_intersection_valid')),failure=result.get('failure'),
                native_position_error_m=ik_result.get('position_error_m'),native_orientation_error_rad=ik_result.get('orientation_error_rad'),
                wrist_rejection=wrist_branch_violation(config['wrist_branch_limits'],q0,[q]),
                native_evidence=str(path.relative_to(ROOT)),path_checked=False,executable=False))
    finally:client.close()
    assert not authority._plans,'PREVIEW_MUST_NOT_REGISTER_PERMITS'
    for case in cases:
        case['start_rotation']=variants
        for name,digest in case['sources'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'SOURCE_RECORD_CHANGED'
    snapshot=deepcopy(load_snapshot(config['execution']['snapshot'],geometry_only=True))
    snapshot['native_model']['init']=deepcopy(planner.init);snapshot['scene']['mesh_obstacles']=[]
    snapshot['native_model']['init']['process_geometry']['expected_contour_ids']=planner.init['process_geometry']['connected_cup_ids']
    visual=export_model(snapshot)
    data=dict(model=visual,cases=cases,retry=True,start_rotation_preview=True,physical_execution='NOT_RUN',executable=False)
    template=(ROOT/'tools/search_replay/field_failures.html').read_text()
    vendor=ROOT/'tools/search_replay/vendor';js=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    html=template.replace('/*__VENDOR__*/',js).replace('/*__DATA__*/',json.dumps(data,separators=(',',':')).replace('</','<\\/'))
    extension=(ROOT/'tools/search_replay/start_rotation.js').read_text()
    (out/'index.html').write_text(html.replace('</html>','<script>'+extension+'</script></html>'))
    report=dict(scope='MEASURED_START_WITH_SEPARATE_J6_ROTATION_PREVIEW',record=str(record.relative_to(ROOT)),
        rotation_convention='CLOCKWISE_IS_NEGATIVE_LOOKING_FROM_POSITIVE_J6_AXIS_TOWARD_ORIGIN',
        q_start=q0.tolist(),q_capture=cases[0]['q_capture'],preview_delta_deg=args.j6_deg,variants=variants,
        source_hashes=cases[0]['sources'],source_record_unchanged=True,targets=[dict(id=c['box']['id'],failures=c['target_failures']) for c in cases],
        collision_shapes=sum(x['layer']=='collision' for x in visual['links']),cups=len(visual['cup_ids']),world_objects=len(world),
        permits_issued=0,hardware_execution='NOT_RUN',rotation_path_checked=False,production_config_changed=False)
    write(out/'report.json',report)
    print(json.dumps(dict(output=str(out),targets=len(cases),variants=variants),ensure_ascii=False))


if __name__=='__main__':main()
