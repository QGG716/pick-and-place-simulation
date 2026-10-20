"""Summarize actual run receipts, including failed contact IK and total wall time."""
import argparse
from collections import Counter, defaultdict
import gzip
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import load_json,save_json,robot_for
from unloading_sim.eco65.native_contact import check_contours


def summarize(run,snapshot_path):
    snapshot=load_json(snapshot_path)
    plan_path=run/('plan.json' if (run/'plan.json').exists() else 'partial_plan.json')
    plan=load_json(plan_path)
    robot=robot_for(snapshot['tool'],snapshot['scene'])
    statuses=Counter();wall=defaultdict(float);failures=[];contacts=[];calls=0
    with gzip.open(run/'native_calls.jsonl.gz','rt',encoding='utf8') as stream:
        for line in stream:
            row=__import__('json').loads(line);q=row['request'];r=row.get('result',{})
            calls+=1;statuses[q['op']+':'+r.get('status','MISSING')]+=1
            wall[q.get('business_interval','INITIALIZATION')+'/'+q['op']]+=row['wall_s']
            if q['op']=='check_state_offline' and q.get('stage')=='contact':
                target_id=q['process_policy']['target_id']
                target=next((b for b in q['world'] if b['id']==target_id),None)
                if target:
                    actual=check_contours(snapshot['native_model']['init']['process_geometry'],
                        robot.named_link_frames(q['q_start'])['link_6'],target['pose'],target['size'],
                        gap=.0004,penetration=.0004,face=q['process_policy']['target_face'],
                        active_ids=q['process_policy']['active_ids'])
                    contacts.append(dict(q_rad=q['q_start'],target_id=target_id,
                        target_face=q['process_policy']['target_face'],
                        active_ids=q['process_policy']['active_ids'],native_status=r['status'],
                        geometric_seal_status=actual['status'],edge_alignment=actual['edge_alignment']))
            if r.get('status') in ('STATE_REJECTED','INVALID_START_STATE','INVALID_GOAL_STATE'):
                witness=r.get('failure') or (r.get('process') or {}).get('last_failure')
                if not witness:
                    examples=(r.get('clearance') or {}).get('rejected_examples',[])
                    witness=examples[0] if examples else None
                failures.append(dict(op=q['op'],stage=q.get('stage'),q_rad=q.get('q_start'),
                    status=r.get('status'),witness=witness))
    records=plan['records']
    first_attach=next((r for r in records if r.get('event_type')=='ATTACH'),None)
    phases=[]
    for r in records:
        item=dict(id=r['record_id'],kind=r['kind'],stage=r['stage'],
            native_status=r['native_result']['status'],height_m=r['state_after']['height_m'])
        if r['kind']=='robot_motion':
            d=np.diff(np.array(r['q'])[:,0])
            item.update(robot_motion_s=r['times'][-1],q1_start=r['q'][0][0],q1_end=r['q'][-1][0],
                        clockwise_rad=float(np.maximum(-d,0).sum()),counterclockwise_rad=float(np.maximum(d,0).sum()))
        phases.append(item)
    candidates=load_json(run/'first_grasp_candidates.json')
    summary=dict(schema='eco65_camera_edge_clockwise_run_summary_v1',
        run_report=load_json(run/'run_report.json'),
        complete=plan['complete'],planning_wall_s=plan['planning_wall_s'],
        robot_motion_s=plan['robot_motion_s'],mechanism_display_s=plan['mechanism_display_s'],
        native_calls=calls,statuses=dict(statuses),native_round_trip_wall_by_phase_s=dict(wall),
        first_target_id=plan['first_target_id'],next_target_id=plan['next_target_id'],
        initial_q=plan['initial_state']['q'],initial_selection=plan.get('initial_selection'),
        first_actual_grasp_q=None if first_attach is None else first_attach['state_after']['q'],
        accepted_records=phases,contact_candidates=contacts,state_failures=failures,
        candidate_geometry=dict(count=len(candidates['candidates']),
            minimum_neighbor_contours_hint=min(g['neighbor_contours_in_clearance_band'] for g in candidates['candidates']),
            hint_scope='GEOMETRY_RANKING_ONLY_NOT_GLOBAL_INFEASIBILITY_PROOF'),
        camera=snapshot['scene']['permanent_camera']['axis_rotation'],
        nominal_extrinsics=snapshot['scene']['camera_extrinsics'],
        scene_fingerprint=snapshot['fingerprint'],physical_execution='NOT_RUN',execution_ready=False,
        observation_source='FROZEN_SCENE_NOT_VISION_CLOSED_LOOP')
    save_json(run/'summary.json',summary)
    print({k:summary[k] for k in ('complete','planning_wall_s','native_calls','statuses','first_actual_grasp_q')})
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--snapshot',type=Path,required=True)
    a=p.parse_args();summarize(a.run,a.snapshot)
