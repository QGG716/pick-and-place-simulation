"""Bounded native pose/short-segment coverage; never a complete unloading task."""
import argparse
import gzip
import hashlib
import json
from copy import deepcopy
from pathlib import Path
from time import perf_counter
import uuid
import numpy as np
from unloading_sim.eco65.model import ROOT, load_json, save_json, transform
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.native_contact import bind_task_contract, check_contours, face_outward
from unloading_sim.eco65.native_cups import grasp_candidates
from unloading_sim.eco65.native_task import NativeTask, extraction_candidates
from unloading_sim.eco65.home_candidate import make_candidate
from unloading_sim.moveit2_backend import digest, MoveItUnavailable


def read_coverage_report(path):
    """Read the exact report or its lossless archive; retain original-byte identity."""
    path=Path(path)
    stored=path if path.is_file() else path.with_suffix(path.suffix+'.gz')
    data=gzip.decompress(stored.read_bytes()) if stored.suffix=='.gz' else stored.read_bytes()
    return json.loads(data),hashlib.sha256(data).hexdigest()


def exposure_snapshot(source, target_id):
    """Keep the original equipment/model, explicitly remove only upper layers.

    Robot/tool URDF and their asset hashes do not change. The world, process
    contract, contact identity and snapshot identity are rebound together.
    """
    result=deepcopy(source);scene=result['scene'];task=scene['native_task']
    target=next(b for b in scene['boxes'] if b['id']==target_id)
    removed=sorted(b['id'] for b in scene['boxes'] if b['layer']>target['layer'])
    scene['boxes']=[b for b in scene['boxes'] if b['id'] not in removed]
    for b in scene['boxes']:b['supports']=[i for i in b['supports'] if i not in removed]
    target=next(b for b in scene['boxes'] if b['id']==target_id)
    scene.update(box=deepcopy(target),target_box_id=target_id)
    remaining=sorted(b['id'] for b in scene['boxes'])
    task.update(target_id=target_id,stack_carton_ids=remaining,
                original_support_names=list(target['supported_by']),
                coverage_context=dict(scope='CONDITIONAL_COVERAGE_DIAGNOSTIC_NOT_COMPLETE_TASK',
                    source_snapshot_fingerprint=source['fingerprint'],removed_carton_ids=removed,
                    remaining_carton_ids=remaining))
    scene['scene_fingerprint']=digest({k:v for k,v in scene.items() if k!='scene_fingerprint'})
    init=result['native_model']['init']
    init['process_geometry']=bind_task_contract(init['process_geometry'],scene)
    init['identity'].update(scene_fingerprint=scene['scene_fingerprint'],
                            contact_geometry_fingerprint=digest(init['process_geometry']))
    result['fingerprint']=digest({k:v for k,v in result.items() if k!='fingerprint'})
    return verify_snapshot(result)


def inspect_face(snapshot, home_q, face, command, out, candidate_limit, ik_seeds, seconds, short_segments,
                 candidate_start_index=0):
    box=snapshot['scene']['box'];init=snapshot['native_model']['init'];started=perf_counter()
    proposal=grasp_candidates(init['process_geometry'],transform(box['pose']),box['size_m'],
        init['identity']['flange_from_task_tcp'],snapshot['scene']['base_position_m'],
        target_id=box['id'],faces=[face],neighbor_boxes=snapshot['scene']['boxes'])
    result=dict(target_id=box['id'],face=face,scope='POSE_AND_SHORT_SEGMENTS_NOT_COMPLETE_TASK',
        snapshot_fingerprint=snapshot['fingerprint'],
        remaining_carton_ids=snapshot['scene']['native_task']['stack_carton_ids'],
        removed_carton_ids=snapshot['scene']['native_task']['coverage_context']['removed_carton_ids'],
        initial_full_stack_visibility=('TOP_OCCLUDED_BY_UPPER_LAYER' if box['layer']<3 and face=='top' else 'FACE_EXPOSED'),
        initial_full_stack_extraction=('SUPPORTS_UPPER_CARTONS' if box['layer']<3 else 'NO_UPPER_CARTONS'),
        geometric_candidate_count=len(proposal['candidates']),attempts=[],status='NOT_RUN',
        candidate_index_range=[candidate_start_index,min(candidate_start_index+candidate_limit,len(proposal['candidates']))],
        short_segments='NOT_RUN',full_task='NOT_RUN',dynamics='NOT_EVALUATED')
    out.mkdir(parents=True,exist_ok=False);save_json(out/'snapshot.json',snapshot)
    save_json(out/'candidates.json',proposal)
    if not command:
        save_json(out/'coverage.json',result);return result
    candidate=make_candidate(home_q,snapshot['tool'],snapshot['scene'],
        dict(source='COVERAGE_INITIAL_STATE_PROPOSAL_REQUIRES_NATIVE_RECHECK'))
    task=None
    try:
        task=NativeTask(snapshot,command,task_seconds=seconds,stage_seconds=20,
            home_candidate=candidate,progress_path=out/'native_progress.jsonl')
        home=None;short_branches=0
        for index,grasp in enumerate(proposal['candidates'][candidate_start_index:candidate_start_index+candidate_limit],candidate_start_index):
            task.select_grasp(grasp);task.parent='';task.attachment=None;task.stages=[];task.events=[]
            top=np.array(grasp['tcp_pose_D']);pre=top.copy()
            pre[:3,3]+=face_outward(transform(box['pose']),face)*snapshot['scene']['native_task']['preapproach_m']
            row=dict(candidate_index=index,candidate_id=grasp['candidate_id'],active_ids=grasp['active_ids'],
                nominal_seal=grasp['nominal_contact_geometry']['status'],status='SEARCHING')
            result['attempts'].append(row)
            if home is None:home,_=task.select_home(pre)
            # Contact itself must be natively reachable, collision checked and
            # actually sealed; a nominal polygon placement is not an IK pass.
            contact_q=next(task.ik(top,home,'contact',ik_seeds),None)
            if contact_q is None:row['status']='CONTACT_IK_OR_STATE_REJECTED';continue
            seal=check_contours(init['process_geometry'],task.robot.named_link_frames(contact_q)['link_6'],
                transform(box['pose']),box['size_m'],gap=.0004,penetration=.0004,face=face,active_ids=task.active_ids)
            row.update(actual_seal=seal,contact_q_rad=contact_q.tolist())
            if seal['status']!='PASS':row['status']='CONTACT_SEAL_REJECTED';continue
            row['status']='CONTACT_POSE_PASS';result['status']='CONTACT_POSE_PASS'
            if not short_segments:break
            try:
                distance=snapshot['scene']['native_task']['preapproach_m'];normal=face_outward(transform(box['pose']),face)
                short_initial_attempts=[];row['short_initial_attempts']=short_initial_attempts
                offsets=[(2*distance,distance),(distance+.005,distance),(distance,distance-.005)]
                # Actual top and front contacts can be reachable while a
                # 25 mm normal offset is rejected. Check two shorter, nonzero
                # diagnostic approaches; neither substitutes a HOME path.
                offsets += [(.015,.010),(.010,.005)]
                for start_offset,pre_offset in offsets:
                    if pre_offset<=0:continue
                    pre=top.copy();pre[:3,3]+=normal*pre_offset
                    pre_q=next(task.ik(pre,contact_q,'pregrasp',ik_seeds),None)
                    probe=dict(start_offset_m=start_offset,pre_offset_m=pre_offset,status='PREGRASP_REJECTED')
                    short_initial_attempts.append(probe)
                    if pre_q is None:continue
                    start=top.copy();start[:3,3]+=normal*start_offset
                    start_q=next(task.ik(start,pre_q,'pregrasp',ik_seeds),None)
                    probe['status']='START_REJECTED' if start_q is None else 'NATIVE_STATES_PASS'
                    if start_q is not None:break
                else:raise ValueError('SHORT_SEGMENT_INITIAL_STATE_REJECTED')
                if short_branches>=8:raise MoveItUnavailable('BOUNDED_SHORT_BRANCHES_EXHAUSTED')
                short_branches+=1;task.task_id='eco65-coverage-'+uuid.uuid4().hex
                row.update(short_initial_q_rad=start_q.tolist(),short_initial_state_semantics='STATIC_NATIVE_CHECKED_STATE_NOT_A_HOME_CONNECTION')
                q=task.motion(start_q,'pregrasp',goal=pre_q)
                q=task.motion(q,'contact',pose=top)
                seal=check_contours(init['process_geometry'],task.robot.named_link_frames(q)['link_6'],
                    transform(box['pose']),box['size_m'],gap=.0004,penetration=.0004,face=face,active_ids=task.active_ids)
                if seal['status']!='PASS':raise ValueError('ACTUAL_CONTACT_ENDPOINT_SEAL_REJECTED')
                task.attachment=np.linalg.inv(task.robot.named_link_frames(q)['link_6'])@transform(box['pose'])
                for lift,evidence in extraction_candidates(task.scene,task.target_pose,box['size_m'],task.robot.fk(q),face):
                    try:task.motion(q,'extraction',pose=lift);break
                    except ValueError as error:
                        if str(error)!='NATIVE_STAGE_CANDIDATES_EXHAUSTED:extraction':raise
                else:raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:extraction')
                row.update(status='NATIVE_SHORT_SEGMENTS_PASS',extraction=evidence,actual_seal=seal)
                result['short_segments']='NATIVE_PASS_INDEPENDENT_PATH_REVIEW_NOT_RUN'
                break
            except ValueError as error:row['short_segment_failure']=str(error)
        if result['status']=='NOT_RUN':result['status']='BOUNDED_CANDIDATES_REJECTED'
    except (ValueError,MoveItUnavailable) as error:
        result.update(status='INTERRUPTED',error=str(error))
    finally:
        if task:
            result.update(native_calls_archive=task.call_archive_info(),native_calls=task.native_calls,
                initial_state_checks=task.home_checks,native_ik_records=task.ik_records,native_records=task.records)
            task.close()
        result.update(wall_s=perf_counter()-started,unsearched_candidates=max(0,len(proposal['candidates'])-len(result['attempts'])))
        save_json(out/'coverage.json',result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','home','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--worker-command');p.add_argument('--target',action='append')
    p.add_argument('--resume-from',type=Path,help='Reuse completed reports only when their exact conditional snapshot identity matches.')
    p.add_argument('--retry-incomplete',action='store_true',help='Recheck prior pose/short-segment failures; preserve successful short chains.')
    p.add_argument('--candidate-limit',type=int,default=12);p.add_argument('--ik-seeds',type=int,default=8)
    p.add_argument('--candidate-start-index',type=int,default=0,
                   help='Explicit bounded continuation in the deterministic geometric candidate order; never imports a path.')
    p.add_argument('--seconds-per-face',type=float,default=120);p.add_argument('--short-segments',action='store_true')
    a=p.parse_args();source=verify_snapshot(load_json(a.snapshot));out=a.output.resolve();out.relative_to(ROOT/'outputs')
    if a.candidate_start_index<0 or min(a.candidate_limit,a.ik_seeds)<1:p.error('Invalid candidate/seed bounds')
    out.mkdir(parents=True,exist_ok=False);home=load_json(a.home)['q_rad'];rows=[];prior_rows={}
    if a.resume_from:
        table=load_json(a.resume_from/'coverage_table.json')
        if table['source_snapshot_fingerprint']!=source['fingerprint']:raise ValueError('COVERAGE_RESUME_SOURCE_CHANGED')
        prior_rows={(r['target_id'],r['face']):r for r in table['rows']}
    for box in source['scene']['boxes']:
        snapshot=exposure_snapshot(source,box['id'])
        for face in ('top','front'):
            command=a.worker_command if not a.target or box['id'] in a.target else None
            previous=a.resume_from/(box['id']+'_'+face)/'coverage.json' if a.resume_from else None
            prior=prior_rows.get((box['id'],face))
            if prior and prior.get('resumed_from'):
                binding=prior['resumed_from'];previous=ROOT/binding['path'];previous.resolve().relative_to(ROOT)
                if not previous.is_file() and not previous.with_suffix(previous.suffix+'.gz').is_file():
                    raise FileNotFoundError('COVERAGE_RESUME_REPORT_MISSING:'+str(previous))
            if previous and (previous.is_file() or previous.with_suffix(previous.suffix+'.gz').is_file()):
                row,report_sha256=read_coverage_report(previous)
                if prior and prior.get('resumed_from') and report_sha256!=prior['resumed_from']['sha256']:
                    raise ValueError('COVERAGE_RESUME_REPORT_CHANGED')
                if (row['snapshot_fingerprint']!=snapshot['fingerprint'] or row['target_id']!=box['id'] or row['face']!=face):
                    raise ValueError('COVERAGE_RESUME_IDENTITY_MISMATCH')
                row['resumed_from']=dict(path=previous.resolve().relative_to(ROOT).as_posix(),
                    sha256=report_sha256,sha256_scope='UNCOMPRESSED_JSON_BYTES',
                    semantics='PRESERVED_PRIOR_RESULT_NOT_RERUN')
                if a.retry_incomplete and row['short_segments']!='NATIVE_PASS_INDEPENDENT_PATH_REVIEW_NOT_RUN':
                    history=row['resumed_from']
                    row=inspect_face(snapshot,home,face,command,out/(box['id']+'_'+face),a.candidate_limit,
                                     a.ik_seeds,a.seconds_per_face,a.short_segments,a.candidate_start_index)
                    row['prior_incomplete_result']=history
                    save_json(out/(box['id']+'_'+face)/'coverage.json',row)
            else:
                row=inspect_face(snapshot,home,face,command,out/(box['id']+'_'+face),a.candidate_limit,
                                 a.ik_seeds,a.seconds_per_face,a.short_segments,a.candidate_start_index)
            rows.append({k:v for k,v in row.items() if k not in ('native_calls','native_records','native_ik_records','initial_state_checks')})
            save_json(out/'coverage_table.json',dict(source_snapshot_fingerprint=source['fingerprint'],rows=rows))


if __name__=='__main__':main()
