"""Export actual rejected centre-carton attempts as an offline 3D diagnostic."""
from pathlib import Path
import sys,argparse,json,hashlib
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'))
from unloading_sim.s50_kinematic_sim import make_context,save
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.robot_replacement_layout import _box_record
from export_s50_kinematic_viewer import simulation_viewer_data,write_html_payload
from export_s50_model_viewer import encode_payload


def export(report,output,check=False):
    report=Path(report);output=Path(output);cycle=json.loads(report.read_text(encoding='utf8'))
    c=make_context(ROOT,output.parent/'diagnostic-model',cycle['settings'])
    data=simulation_viewer_data(cycle)
    if data['environment']['environment_fingerprint']!=cycle['environment_fingerprint']:
        raise ValueError('DIAGNOSTIC_ENVIRONMENT_CHANGED')
    target=next(b for b in c.layout.cartons() if b.name==cycle['target'])
    groups=[];current=[]
    for row in cycle['ik_attempts']:
        if row['stage']=='extract':current.append(row)
        elif current:groups.append(current);current=[]
    if current:groups.append(current)
    failures=[r for r in cycle['failures'] if r.get('kind')=='EXTRACTION_CANDIDATE_FAILED']
    if len(groups)!=len(failures):raise ValueError('EXTRACTION_TRACE_GROUP_MISMATCH')
    grasp_attempts=[r for r in cycle['ik_attempts'] if r['stage']==target.name+'_front'
                    and r['ik_success'] and r.get('basic_geometry_passed')]
    checker=None
    if check:
        from unloading_sim.s50_sim_collision import SimCollision
        checker=SimCollision(c)
    branches=[]
    for group,failure in zip(groups,failures):
        detail=failure['detail'];grasp=np.asarray(group[0]['desired_physical_contact']).copy()
        lift=max(.02,1.5+.1-(target.center[2]-target.half_extents[2]));grasp[2,3]-=lift
        source=min(grasp_attempts,key=lambda r:np.max(np.abs(c.robot.fk(r['q_rad'])-grasp)))
        q=np.asarray(source['q_rad']).copy();reference=np.asarray(group[0]['q_rad'])
        for j,(lo,hi) in enumerate(c.robot.joint_limits):
            turns=[q[j]+2*np.pi*k for k in range(-2,3) if lo<=q[j]+2*np.pi*k<=hi]
            q[j]=min(turns,key=lambda v:abs(v-reference[j]))
        if np.max(np.abs(c.robot.fk(q)-grasp))>1e-9:raise ValueError('ACTUAL_GRASP_TRACE_NOT_FOUND')
        relative=np.linalg.inv(grasp)@target.world_from_local
        def state(q,phase,point,desired=None):
            q=np.asarray(q);contact=c.robot.fk(q);body=contact@relative
            return dict(q_rad=q.tolist(),phase=phase,point_index=point,
                world_from_physical_contact=contact.tolist(),world_from_carton=body.tolist(),
                desired_physical_contact=contact.tolist() if desired is None else desired)
        knots=[state(q,'grasp',0)]
        for row in group:
            if row['ik_success'] and row['point_index']<detail['point_index']:
                knots.append(state(row['q_rad'],'lift' if row['point_index']==1 else 'withdraw',row['point_index'],row['desired_physical_contact']))
        if not np.allclose(knots[-1]['q_rad'],detail['last_valid_q_rad'],atol=1e-12,rtol=0):
            raise ValueError('LAST_VALID_TRACE_MISMATCH')
        rejected=state(detail['q_rad'],'rejected',detail['point_index'],detail['desired_physical_contact'])
        desired=np.asarray(detail['desired_physical_contact']);axes=c.robot.joint_axis_frames(detail['q_rad'])
        actual=np.asarray(rejected['world_from_physical_contact']);j1=axes['s50_joint_1'][0];j5=axes['s50_joint_5'][0]
        tcp_from_j5=actual[:3,:3].T@(actual[:3,3]-j5)
        d=abs(float((j5-j1)@axes['s50_joint_2'][1]));desired_j5=desired[:3,3]-desired[:3,:3]@tcp_from_j5
        fully_out=grasp.copy();fully_out[0,3]-=2*target.half_extents[0];fully_out[2,3]+=lift
        branch=dict(id=failure['branch'],reason=detail['reason'],knots=knots,rejected=rejected,
            detail=detail,physical_contact_from_attached_box=relative.tolist(),
            initial_contact=grasp.tolist(),j1_axis_origin_m=j1.tolist(),minimum_j5_radius_m=d,
            desired_j5_xyz_m=desired_j5.tolist(),desired_j5_radius_m=float(np.linalg.norm(desired_j5[:2]-j1[:2])),
            tcp_from_j5_local_m=tcp_from_j5.tolist(),full_clearance_contact_target=fully_out.tolist(),
            full_clearance_carton_target=(fully_out@relative).tolist(),
            full_clearance_j5_target_m=(fully_out[:3,3]-fully_out[:3,:3]@tcp_from_j5).tolist())
        if checker:
            for row in knots:
                if not checker.check(row['q_rad'],1.5,np.asarray(row['world_from_carton']),True,'diagnostic_valid_prefix'):
                    raise ValueError('RETAINED_PREFIX_COLLISION:'+str(checker.last))
            ok=checker.check(rejected['q_rad'],1.5,np.asarray(rejected['world_from_carton']),True,'diagnostic_rejected_candidate')
            if detail['reason']=='POINTWISE_COLLISION' and (ok or checker.last['pair']!=detail['collision']['pair']):
                raise ValueError('RETAINED_COLLISION_NOT_REPRODUCED')
            branch['geometry_recheck']=dict(valid_prefix_states=len(knots),rejected_candidate_collision_free=ok,
                rejected_pair=None if ok else checker.last['pair'])
        branches.append(branch)
    data['diagnostic']=dict(target=target.name,layout_gap_m=cycle['layout_gap_m'],belt_height_m=1.5,
        world_from_model_base=cycle['world_from_model_base'],settings=cycle['settings'],
        flange_from_physical_contact=cycle['flange_from_physical_contact'],
        flange_from_virtual_task_tcp=cycle['flange_from_virtual_task_tcp'],branches=branches,
        scene=[_box_record(b) for b in coupled_conveyor_state(c.layout,1.5,c.layout.cartons()).boxes],
        source_report_sha256=hashlib.sha256(report.read_bytes()).hexdigest(),
        status='REJECTED_PLANNING_CANDIDATES_NOT_EXECUTED_TASK',interpolation='DISPLAY_ONLY_BETWEEN_ACTUAL_IK_KNOTS',
        completed_cartons=0,initial_cartons=40)
    vendor=ROOT/'tools/search_replay/vendor'
    scripts='\n'.join((vendor/f).read_text(encoding='utf8') for f in ['three.min.js','OrbitControls.js'])
    scripts+='\n'+(ROOT/'tools/s50_kinematics.js').read_text(encoding='utf8')
    html=(ROOT/'tools/s50_withdrawal_diagnostic_viewer.html').read_text(encoding='utf8').replace('/*__VENDOR__*/',scripts)
    write_html_payload(output,html,encode_payload(data))
    evidence=dict(status='EXPORTED',html_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        source_report_sha256=data['diagnostic']['source_report_sha256'],branches=len(branches),
        valid_ik_knots=sum(len(b['knots']) for b in branches),rejected_candidates=len(branches),
        geometry_recheck=[b.get('geometry_recheck') for b in branches],browser_check='PENDING')
    save(output.with_suffix('.json'),evidence);print(json.dumps(evidence),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--check',action='store_true');a=p.parse_args();export(a.report,a.output,a.check)
