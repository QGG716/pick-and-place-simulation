"""Read-only checks and metrics for the two authorized extra-X mounting trials."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--runtime-root',type=Path,default=Path(__file__).resolve().parents[1])
    a=p.parse_args()
    sys.path[:0]=[str(a.runtime_root/'tools'),str(a.runtime_root/'src')]
    import numpy as np
    from plan_m710_mount_comparison import make_context, FLAGS
    from plan_m710_top_layer_lifted import save
    from verify_m710_top_layer_batch import verify
    from unloading_sim.upper_arm_preference import upper_arm
    protocol=json.loads((a.run/'protocol.json').read_text())
    summary=json.loads((a.run/'summary.json').read_text())
    assert summary['status']=='TRIALS_FINISHED' and len(summary['outcomes'])==2
    _,connector,_=make_context('x_minus_300_y_minus_100',layout_path=a.runtime_root/protocol['active_layout_file'])
    robot=connector.robot
    assert type(robot).__name__=='PinocchioHppFclBackend'
    np.testing.assert_allclose(robot.base_transform[:3,3],[-1.625,.25,.6],atol=1e-12)
    rows=[]
    for outcome in summary['outcomes']:
        target=outcome['target_id'];run=a.run/target;folder=run/'seed-71070'
        checked=verify(run);result=json.loads((folder/'result.json').read_text())
        inp=json.loads((run/'input.json').read_text())
        assert inp['passed'] and inp['initial_collision_failure'] is None
        assert result['initial']==inp['initial'] and result['initial']['predecessor_final_state_receipt'] is None
        for k,v in inp['initial'].items():
            if k!='predecessor_final_state_receipt':assert v==inp['original_root'][k],k
        counts=Counter();attempts=[];q_samples=[];native_ids=[]
        for attempt in result['attempts']:
            if 'folder' not in attempt:continue
            d=folder/attempt['folder'];calls=json.loads((d/'requests.json').read_text())
            trace=json.loads((d/'search-trace.json').read_text());detail=json.loads((d/'attempt.json').read_text())
            urdf=ET.fromstring(trace['model']['parameters']['robot_description'])
            joint=next(j for j in urdf.findall('joint') if j.find('parent').get('link')=='world' and j.find('child').get('link')=='base_link')
            np.testing.assert_allclose(list(map(float,joint.find('origin').get('xyz').split())),[-1.625,.25,.6],atol=1e-12)
            witnesses=[];local=Counter()
            for c in calls:
                local['native_requests']+=1
                if c['op']=='ik' and c['status']=='SUCCESS':local['numerical_ik_success']+=1
                if c['op']!='plan':continue
                resp=c['response'];end=resp.get('clearance',{}).get('counts',{}).get('endpoint',{})
                if c.get('q_goal') is not None and c.get('pipeline_id')!='external_axis' and end.get('queries',0)>=2 and end.get('rejected',1)==0:
                    local['explicit_q_goal_check_passed']+=1
                if c['status']=='SUCCESS':local[c['stage']+':'+c['planner_id']]+=1
                for w in resp.get('clearance',{}).get('rejected_examples',[]):witnesses.append(dict(stage=c['stage'],witness=w))
            counts.update(local)
            native_ids.extend(sorted({t['request']['task_id'] for t in trace['attempts']}))
            contact=detail.get('terminal_contact') or {}
            contact_q=detail.get('contact_end_q_rad')
            attempts.append(dict(folder=attempt['folder'],complete=attempt['complete'],failure=attempt.get('failure'),
                counts=dict(local),delta_m=attempt['option'].get('world_y_offset_m',0.),height_m=attempt['option']['height_m'],
                sync=detail.get('receive_sync_location'),contact_q_rad=contact_q,
                upper_arm=None if contact_q is None else upper_arm(robot,np.asarray(contact_q)),
                cups={k:sum(v) for k,v in contact.items() if k.endswith('_mask')},
                region_attempts=len(detail.get('region_attempts',[])),collision_witnesses=witnesses,
                native_round_trip_s=sum(c['round_trip_s'] for c in calls),times=detail['times']))
        accepted=None
        if result['cycles']:
            assert len(result['cycles'])==1
            cycle=result['cycles'][0]
            extraction=next(s for s in cycle['stages'] if s['stage']=='extraction' and s['generation_source']!='SEMANTIC_EVENT')
            end_q=np.asarray(extraction['points'][-1]['q'])
            for s in cycle['stages']:
                q_samples.extend(p['q'] for p in s.get('points',[]))
            q_samples=np.asarray(q_samples)
            assert all(robot.within_limits(q) for q in q_samples)
            margins=np.minimum(q_samples-robot.joint_limits[:,0],robot.joint_limits[:,1]-q_samples)
            accepted=dict(delta_m=cycle['target'].get('world_y_offset_m',0.),height_m=cycle['height_m'],
                synchronization=cycle['receive_sync_location'],template=cycle['option']['template'],
                extraction_end_q_rad=end_q.tolist(),extraction_end_q_deg=np.rad2deg(end_q).tolist(),
                extraction_end_fk=robot.fk(end_q).tolist(),all_native_joint_sample_count=len(q_samples),
                per_joint_minimum_limit_margin_deg=np.rad2deg(margins.min(axis=0)).tolist(),
                release_box=cycle['nominal_release']['samples'][0]['box'],
                nominal_drop_m=cycle['nominal_release']['drop_m'],
                events=[e['event_type'] for e in cycle['events']],
                attachment_pose_continuity_error=cycle['attachment']['pose_continuity_max_abs'],
                stages=[dict(stage=s['stage'],planner=s['planner_id'],source=s['generation_source'],
                    points=len(s.get('points',[]))) for s in cycle['stages']])
        rows.append(dict(target_id=target,complete=result['completed_count']==1,planning_wall_s=outcome['planning_wall_s'],
            initial_cartons=len(result['initial']['world_cartons']),remaining_cartons=len(result['terminal']['world_cartons']),
            historical_completed=result['historical_completed_count'],counts=dict(counts),attempts=attempts,
            accepted=accepted,verification=checked,native_task_ids=native_ids))
    report=dict(**FLAGS,source_commit=protocol['source_commit'],worker_sha256=protocol['worker_sha256'],
        verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),rows=rows,
        completed=sum(r['complete'] for r in rows),targets=2,experiment_wall_s=summary['experiment_wall_s'],
        scope='TWO_INDEPENDENT_PLANNING_TRIALS_NOT_WHOLE_STACK_RATE')
    save(a.run/'verification.json',report)
    print(json.dumps([{k:r[k] for k in ('target_id','complete','planning_wall_s','counts')} for r in rows]))


if __name__=='__main__':main()
