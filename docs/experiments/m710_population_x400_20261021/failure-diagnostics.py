"""Condense current-run failures without asking any solver for another attempt."""
from pathlib import Path
from collections import Counter
import json,tarfile,re

run=Path('/population-x400-20261021/formal-v1')
out=Path('/population-x400-20261021/analysis/failure-diagnostics.json')
summary=json.loads((run/'summary.json').read_text())
rows=[]
for outcome in summary['outcomes']:
    if outcome['completed_count']:continue
    prefix=outcome['evidence']['archive'].removesuffix('.tar.gz')
    attempts=[]
    with tarfile.open(run/outcome['evidence']['archive']) as tar:
        for a in outcome['attempts']:
            if 'folder' not in a:continue
            base=prefix+'/'+a['folder']+'/'
            calls=json.load(tar.extractfile(base+'requests.json'))
            detail=json.load(tar.extractfile(base+'attempt.json'))
            worker_lines=tar.extractfile(base+'worker.log').read().decode(errors='replace').splitlines()
            pilz_errors=[line for line in worker_lines if ('[ERROR]' in line or '[WARN]' in line)
                         and 'pilz_industrial_motion_planner' in line]
            normalized_errors=Counter(re.sub(r'^\[.*?\] \[.*?\] ','',line) for line in pilz_errors)
            velocity_limits=[dict(joint=m.group(1),actual_rad_s=float(m.group(2).rstrip('.')),limit_rad_s=float(m.group(3).rstrip('.')))
                for line in pilz_errors
                for m in [re.search(r'Joint velocity limit of (\w+) violated.*Actual joint velocity is ([\d.eE+-]+), while the limit is ([\d.eE+-]+)',line)] if m]
            regions=detail.get('region_attempts',[])
            op_counts=Counter((c.get('op'),c.get('stage'),c.get('planner_id'),c.get('status')) for c in calls)
            errors=Counter((c.get('stage'),c.get('response',{}).get('solver_message'),
                            c.get('response',{}).get('moveit_error_code'))
                           for c in calls if c.get('status') not in ('READY','SUCCESS'))
            region_counts=Counter((r.get('failure') or {}).get('blocking_stage',
                                  (r.get('failure') or {}).get('reason','UNKNOWN')) for r in regions)
            attempts.append(dict(folder=a['folder'],failure=a.get('failure'),option=a['option'],
                contact_complete=detail.get('contact_complete',False),
                contact_end_q_rad=detail.get('contact_end_q_rad'),
                native_log_errors=dict(normalized_errors),native_velocity_limit_events=velocity_limits,
                native_log_first_errors=pilz_errors[:12],
                operation_counts=[dict(op=k[0],stage=k[1],planner=k[2],status=k[3],count=v) for k,v in op_counts.items()],
                native_failures=[dict(stage=k[0],solver_message=k[1],moveit_error_code=k[2],count=v) for k,v in errors.items()],
                region_failure_counts=dict(region_counts),region_samples=len(regions),
                non_extraction_failures=[dict(center_world_m=r['center_world_m'],
                    blocking_stage=(r.get('failure') or {}).get('blocking_stage'),
                    reason=(r.get('failure') or {}).get('reason')) for r in regions
                    if (r.get('failure') or {}).get('blocking_stage')],
                collision_witnesses=[dict(stage=c.get('stage'),stage_id=c.get('stage_id'),witness=w)
                    for c in calls for w in c.get('response',{}).get('clearance',{}).get('rejected_examples',[])]))
    rows.append(dict(target_id=outcome['target_id'],planning_wall_s=outcome['planning_wall_s'],
                     attempts=attempts,unsearched=[a for a in outcome['attempts'] if 'folder' not in a]))
out.write_text(json.dumps(dict(source_commit=summary['source_commit'],solver_calls_added=0,
    rows=rows),indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(failed_cartons=len(rows),targets=[r['target_id'] for r in rows])))
