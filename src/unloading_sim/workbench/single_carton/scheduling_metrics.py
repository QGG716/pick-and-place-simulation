"""Four handoff intervals from original motion/pressure clocks, without overlap."""
import json
from pathlib import Path


def union_seconds(intervals):
    total=0.;end=float('-inf')
    for lo,hi in sorted(intervals):
        total+=max(0.,hi-max(lo,end));end=max(end,hi)
    return total


def handoffs(cycle_events,motion_events):
    motions={}
    for rows in motion_events:
        sends=[r['sent_s'] for r in rows if r.get('kind')=='send']
        result=next((r for r in rows if r.get('kind')=='result' and r.get('state')=='SUCCEEDED'),None)
        if sends and result:motions[result['task_id']]=(min(sends),max(sends),result)
    ordered={r['segment']:motions[r['motion_task_id']] for r in cycle_events
             if r.get('kind')=='motion_task' and r['motion_task_id'] in motions}
    pressure={r['phase']:r for r in cycle_events if r.get('kind')=='pressure_rule_satisfied'}
    complete=next((r for r in cycle_events if r.get('kind')=='stage' and r.get('state')=='COMPLETE'),None)
    boundaries=[]
    for i in range(4):
        if i not in ordered:continue
        _,start,result=ordered[i];rule=result.get('arrival_rule_satisfied_s')
        end=ordered.get(i+1,(None,))[0] if i<3 else (complete or {}).get('host_monotonic_s')
        waits=[] if rule is None else [(start,rule)]
        p=pressure.get('grip' if i==1 else 'release') if i in (1,3) else None
        missing=rule is None or end is None or (i in (1,3) and p is None)
        if p:waits.append((p['valve_completed_s'],p['pressure_received_s']))
        required=None if missing else union_seconds([(max(start,a),min(end,b)) for a,b in waits if min(end,b)>max(start,a)])
        boundaries.append(dict(segment=i,start_s=start,arrival_rule_satisfied_s=rule,
                               pressure_rule_satisfied_s=None if p is None else p['pressure_received_s'],
                               continued_s=end,required_wait_intervals=waits,required_wait_s=required,
                               excess_s=None if missing else max(0.,end-start-required)))
    return boundaries


def read_handoffs(cycle_path,motion_root):
    path=Path(cycle_path)
    if not path.exists():return []
    rows=[json.loads(x) for x in path.read_text().splitlines()]
    motions=[[json.loads(x) for x in p.read_text().splitlines()] for p in Path(motion_root).glob('*/events.jsonl')]
    return handoffs(rows,motions)
