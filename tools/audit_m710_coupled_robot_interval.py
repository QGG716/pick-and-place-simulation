"""Read-only native full-robot/tool/payload interval diagnostics at recorded q.

This does not generate motion, feed a solver, commit a batch state, or qualify
execution. The existing standalone native validate interface imports each
synchronized world. Its diagnostic session is separate from planning sessions.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_batch import message_box
from unloading_sim.moveit2_backend import ResidentMoveItClient,box_message
from unloading_sim.workcell_layout import load_workcell_layout
import numpy as np


def audit(run,seed,worker,output):
    protocol=json.loads((run/'protocol.json').read_text())
    result=json.loads((run/f'seed-{seed}'/'result.json').read_text())
    accepted=next(a for a in result['attempts'] if a.get('complete'))
    cycle=result['cycles'][0]
    trace=json.loads((run/f'seed-{seed}'/accepted['folder']/'search-trace.json').read_text())
    requests={t['request']['stage_id']:t['request'] for t in trace['attempts']}
    chosen=[next(s for s in cycle['stages'] if s['generation_source']=='NATIVE_EXTERNAL_AXIS'),
            next(s for s in cycle['stages'] if s['generation_source']=='SEMANTIC_EVENT')]
    model=deepcopy(trace['model'])
    # Standalone validate is explicitly not an executable planning request.
    model.update(planning_only=False,search_trace_enabled=False,
                 single_target_recovery_enabled=False,bounded_preplace_ik_enabled=False)
    model.pop('lifted_receiver_binding',None)
    layout=load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1.yaml')
    cartons=tuple(message_box(b) for b in cycle['batch_root']['world_cartons'])
    low=float(result['initial']['height_m'])
    target=next(b for b in cartons if b.name==cycle['target_id'])
    bottom=float(target.corners()[:,2].min());high=bottom-.05;middle=high-2*target.half_extents[2]
    intervals=[(low,high),(high,low),(low,middle),(middle,low),(middle,high),(high,middle),
               (low,low),(middle,middle),(high,high)]
    output.mkdir(parents=True,exist_ok=False)
    client=ResidentMoveItClient(worker,log_path=str(output/'worker.log'))
    records=[];summaries=[]
    try:
        ready=client.request(model);assert ready['status']=='READY'
        for stage in chosen:
            original=requests[stage['stage_id']]
            held=original['attachment'] is not None
            template=deepcopy(original)
            template.update(op='validate',planning_only=False,require_native_motion=False)
            # No trajectory or ownership event is executed by this query.
            for key in ('nominal_release','receiver_end_world','ownership_transition'):
                template.pop(key,None)
            for start,end in intervals:
                count=max(1,int(np.ceil(abs(end-start)/.01))) if start!=end else 0
                rows=[]
                for h in np.linspace(start,end,count+1):
                    state=coupled_conveyor_state(layout,float(h),cartons)
                    request=dict(template,world=[box_message(b) for b in state.boxes
                        if not (held and b.name==cycle['target_id'])])
                    reply=client.request(request)
                    assert reply['status']=='SUCCESS',reply
                    assert not reply['pipeline_calls'],reply['pipeline_calls']
                    row=dict(height_m=float(h),scene_fingerprint=state.scene_fingerprint,response=reply)
                    rows.append(row)
                passed=sum(r['response']['native_valid'] for r in rows)
                summaries.append(dict(attached=held,from_height_m=start,to_height_m=end,
                    samples=len(rows),passed=passed,failed=len(rows)-passed,source_stage_id=stage['stage_id'],
                    q_rad=original['q_start'],maximum_height_step_m=abs(end-start)/max(count,1)))
                records.append(dict(summary=summaries[-1],samples=rows))
                print(json.dumps(summaries[-1]),flush=True)
    finally:client.close()
    report=dict(scope='POSTHOC_RECORDED_Q_DIAGNOSTIC_ONLY_NO_MOTION_OR_BATCH_CREDIT',
        source_commit=protocol['source_commit'],seed=seed,target_id=cycle['target_id'],
        input_model=model,source_requests=[requests[s['stage_id']] for s in chosen],intervals=records,
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    (output/'raw.json').write_text(json.dumps(report,separators=(',',':')))
    (output/'summary.json').write_text(json.dumps(dict(intervals=summaries,scope=report['scope']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    p.add_argument('--seed',type=int,default=71070);p.add_argument('--worker-command',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    audit(a.run,a.seed,a.worker_command,a.output)
