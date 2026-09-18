"""Offline timing/count report for the finite runner's actual DDS receipts (no inference)."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import read_json,atomic_json


def statistics(values):
    values=sorted(values)
    return {'count':len(values),**({
        'p50':values[round((len(values)-1)*.5)],'p95':values[round((len(values)-1)*.95)],
        'max':values[-1]} if values else {})}


def summarize(directory):
    progress=read_json(directory/'progress.json')
    results=[]
    for group in progress['groups']:
        result={k:group.get(k) for k in ('task_id','capture','action','status','result_origin','algorithm_attempts',
            'sam_attempts','metric_attempts','algorithm_seconds','run_id','artifact','counts','failure_stage','error',
            'artifact_completed_monotonic','switch_submitted_monotonic')}
        receipt_path=directory/'receipts'/(group['task_id']+'.json')
        if receipt_path.exists():
            receipt=read_json(receipt_path)
            result['ros']={k:v for k,v in receipt.items() if k!='world_events'}
            rows=receipt.get('world_events',[])
            if receipt['status']=='ROS_ACCEPTED':
                first=receipt['first_world_monotonic']
                current=[r for r in rows if r['source_epoch']==receipt['session']]
                result['windows']={}
                for name,selected in (('switch',[r for r in rows if r['monotonic']<first+5.]),
                                      ('steady',[r for r in current if r['monotonic']>=first+5.])):
                    result['windows'][name]={'world_count':len(selected),
                        'receive_interval_s':statistics(b['monotonic']-a['monotonic'] for a,b in zip(selected,selected[1:])),
                        'evaluation_interval_s':statistics(b['evaluated']-a['evaluated'] for a,b in zip(selected,selected[1:])),
                        'blocking_counts':{reason:sum(reason in r['reasons'] for r in selected)
                                           for reason in sorted({s for r in selected for s in r['reasons']})},
                        **{f'{source}_{where}_age_s':statistics(r[clock]-r[source+'_sample'] for r in selected)
                           for source in ('robot','mechanism') for where,clock in (('evaluation','evaluated'),('receive','ros_time'))}}
                steady=[r for r in current if r['monotonic']>=first+5.]
                result['steady_ages_within_original_thresholds']=bool(steady) and all(
                    0<=r[clock]-r[source+'_sample']<=threshold for r in steady
                    for source,threshold in (('robot',.5),('mechanism',2.)) for clock in ('evaluated','ros_time'))
        results.append(result)
    return {'batch_id':progress['batch_id'],'exit_code':progress['exit_code'],'status':progress['status'],
        'sequence_kind':progress['sequence_kind'],'groups':results}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists(): parser.error('refuse to replace an existing timing report')
    result=summarize(args.directory)
    atomic_json(args.output,result)
    print(json.dumps(result,indent=2))
