"""Numerical branch recovery on the same retained target and unchanged face pose."""
from pathlib import Path
import sys,argparse,json,copy,hashlib,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,ik_candidates,save
from unloading_sim.s50_face_kinematic import FaceRun,pose_line
from unloading_sim.independent_cups import evaluate_independent_cup_geometry


def main():
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--draws',type=int,default=512);p.add_argument('--path-recovery',action='store_true');p.add_argument('--deferred-lift',action='store_true')
    p.add_argument('--standoff',type=float);p.add_argument('--face',choices=['front','top','left','right'],default='front')
    p.add_argument('--flange-spacer',type=float,default=0.)
    p.add_argument('--test-transfer',action='store_true');p.add_argument('--release-y',type=float,default=-.2)
    a=p.parse_args();source=json.loads(a.report.read_text(encoding='utf8'))
    settings=copy.deepcopy(source['settings'])
    settings['posture_preferences'].update(upper_arm_forward_free_deg=30.,upper_arm_large_forward_weight=32.)
    if a.standoff is not None:settings['tool_mount_standoff_m']=a.standoff
    if a.flange_spacer:settings['tool_flange_spacer_m']=a.flange_spacer
    c=make_context(ROOT,a.output,settings);changed=c.frozen['environment_fingerprint']!=source['environment_fingerprint']
    if changed:
        # A changed physical tool is a diagnostic scene clone, not continuation
        # or completion credit in the original execution world.
        r=FaceRun(c);r.completed=list(source['completed_carton_ids'])
        r.active={n:b for n,b in r.active.items() if n in source['remaining_carton_ids']}
        c.active_cartons=tuple(r.active.values());c.outfed_carton_ids=set(r.completed)
        r.q=np.asarray(source['samples'][-1]['q_rad']);r.home=np.asarray(source['independent_home_q_rad'])
        r.height=source['samples'][-1]['belt_height_m'];c.rng.bit_generator.state=source['numerical_rng_state']
    else:r=FaceRun.resume_empty_boundary(c,a.report)
    name=source.get('failure',{}).get('target',source.get('current_target'));r.target=r.active[name];r.box=r.target.world_from_local.copy();c.settings['target']=name
    first_ik=len(c.attempts);first_fail=len(r.failures)
    seeds=[r.q];unique=[]
    for row in source['ik_attempts']:
        if row.get('target')!=name or row['stage']!=name+'_front' or not row['ik_success']:continue
        q=np.asarray(row['q_rad']);wrapped=(q+np.pi)%(2*np.pi)-np.pi
        if any(np.max(abs(wrapped-u))<.04 for u in unique):continue
        unique.append(wrapped);seeds.append(q)
        for turn in [-1,1]:
            alternate=q.copy();alternate[3]+=turn*np.pi;alternate[4]*=-1;alternate[5]+=turn*np.pi
            for j in [0,3,4,5]:alternate[j]=(alternate[j]+np.pi)%(2*np.pi)-np.pi
            seeds.append(alternate)
    T=r.grasp_pose(a.face)
    found=ik_candidates(c,T,seeds=seeds,draws=a.draws,stage=name+'_'+a.face+'_recovery',limit=16,
        valid=lambda q:r.valid(q,'recovery_contact'))
    results=[]
    for q in found:
        actual=c.robot.fk(q);relative=np.linalg.inv(actual)@r.box
        for distance in ([.12,.06,.02] if a.path_recovery else [.12]):
            pre=actual.copy();pre[:3,3]-=distance*pre[:3,2]
            for extra in ([.05,.1] if a.deferred_lift else [0.,.05,.1] if a.path_recovery else [0.]):
                delay=.25 if a.deferred_lift else None
                result=dict(q_rad=q.tolist(),pregrasp_distance_m=distance,extra_lift_m=extra,lift_after_withdraw_m=delay)
                try:
                    geometry=evaluate_independent_cup_geometry(actual,r.target,a.face,r.cup_array)
                    result['eligible_cups']=sum(geometry.geometrically_eligible_mask)
                    if not result['eligible_cups']:raise RuntimeError('NO_FULL_TARGET_SEAL_RING')
                    approach=r.pointwise(pose_line(actual,pre,step=.01),q,'recovery_pregrasp_reverse')
                    extraction=r.extraction(q,relative,extra,delay)
                    result.update(status='APPROACH_AND_STRAIGHT_EXTRACTION_PASS',pregrasp_q_rad=approach[-1].tolist(),
                        path_rad=[x.tolist() for x in extraction['path']],preference=r.withdrawal_score(extraction['path']))
                    if a.test_transfer:
                        loaded=r.loaded_candidate(extraction,relative,a.face,a.release_y,0.,'level_then_lower')
                        r.plan_departure(loaded)
                        result.update(status='APPROACH_EXTRACTION_TRANSFER_AND_RETURN_PASS',loaded_ik_knots=loaded['ik_knots'],
                            release_q_rad=loaded['rows'][-1]['q'].tolist(),release_y_m=a.release_y,route='level_then_lower',
                            release_drop_m=loaded['release_drop_m'])
                except RuntimeError as e:result.update(status='FAILED',reason=str(e),detail=copy.deepcopy(getattr(r,'last_pointwise_failure',None)))
                results.append(result);print(name,distance,extra,result['status'],result.get('reason'),flush=True)
    result=dict(scope='CHANGED_MOUNT_DIAGNOSTIC_SCENE_NOT_EXECUTED' if changed else 'SAME_REMAINING_TARGET_NUMERICAL_RECOVERY_NOT_EXECUTED',
        source_report_sha256=hashlib.sha256(a.report.read_bytes()).hexdigest(),tool_mount_standoff_m=c.intrinsic['mount_standoff_m'],face=a.face,
        new_executed_cartons=0,target=name,remaining_carton_ids=list(r.active),
        completed_carton_ids=r.completed,environment_fingerprint=c.frozen['environment_fingerprint'],grasp_pose=T.tolist(),
        contact_solutions=len(found),branches=results,ik_attempts=c.attempts[first_ik:],failures=r.failures[first_fail:])
    save(a.output/'recovery.json',result);print('CONTACT_SOLUTIONS',len(found),flush=True)


if __name__=='__main__':main()
