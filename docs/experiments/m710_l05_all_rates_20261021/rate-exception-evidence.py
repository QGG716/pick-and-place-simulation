"""Read measured native rate exceptions and accepted trajectories; no solver calls."""
from pathlib import Path
from collections import Counter
import json,tarfile,re,math,xml.etree.ElementTree as ET
p=Path('/l05-all-rates-20261021');run=p/'formal-v3'
s=json.loads((run/'summary.json').read_text())
o=next(o for o in s['outcomes'] if o['target_id']=='carton_l05_c03')
other_target_logs=0
for other in s['outcomes']:
    if other['target_id']=='carton_l05_c03':continue
    with tarfile.open(run/other['evidence']['archive']) as t:
        for m in t:
            if m.name.endswith('/worker.log'):
                data=t.extractfile(m).read()
                assert b'M710_DIAGNOSTIC_STATE_VELOCITY_IGNORED' not in data
                assert b'M710_DIAGNOSTIC_JOINT_RATE_IGNORED' not in data
                other_target_logs+=1
evidence=[]
with tarfile.open(run/o['evidence']['archive']) as t:
    prefix=o['evidence']['archive'].removesuffix('.tar.gz')
    result=json.load(t.extractfile(prefix+'/result.json'))
    for attempt in o['attempts']:
        if 'folder' not in attempt:continue
        b=prefix+'/'+attempt['folder']+'/'
        calls=json.load(t.extractfile(b+'requests.json'))
        detail=json.load(t.extractfile(b+'attempt.json'))
        lines=t.extractfile(b+'worker.log').read().decode(errors='replace').splitlines()
        events=[]
        for line in lines:
            if 'M710_DIAGNOSTIC_JOINT_RATE_IGNORED ' not in line:continue
            fields=dict(word.split('=',1) for word in line.split() if '=' in word)
            for key in fields:
                if key not in ('target','joint','physical_execution'):fields[key]=float(fields[key])
            assert fields['target']=='carton_l05_c03' and fields['physical_execution']=='NOT_RUN'
            events.append(fields)
        peaks={}
        for joint in sorted({e['joint'] for e in events}):
            values=[e for e in events if e['joint']==joint]
            peaks[joint]=dict(count=len(values),peak_velocity=max(values,key=lambda e:abs(e['velocity_rad_s'])),
                peak_acceleration=max(values,key=lambda e:abs(e['acceleration_rad_s2'])),
                peak_delta_q=max(values,key=lambda e:abs(e['delta_q_rad'])))
        evidence.append(dict(folder=attempt['folder'],complete=attempt['complete'],
            contact_q_rad=detail.get('contact_end_q_rad'),marker_count=len(events),peaks_by_joint=peaks,
            state_velocity_bypasses=sum('M710_DIAGNOSTIC_STATE_VELOCITY_IGNORED ' in line for line in lines),
            native_plan_statuses=[dict(stage=k[0],status=k[1],count=n) for k,n in
                Counter((c.get('stage'),c['status']) for c in calls if c['op']=='plan').items()],
            native_log_errors=dict(Counter(re.sub(r'^\[.*?\] \[.*?\] ','',line) for line in lines
                if '[ERROR]' in line and 'pilz_industrial_motion_planner' in line)),
            full_event_log=b+'worker.log'))
    trace=json.load(t.extractfile(prefix+'/attempt-001/search-trace.json'))
    urdf=ET.fromstring(trace['model']['parameters']['robot_description'])
    bounds={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper')))
        for j in urdf.findall('joint') if j.get('name') in ('J1','J2','J3','J4','J5','J6')}
    trajectory_peaks=[];checked_positions=[]
    for cycle in result['cycles']:
        for stage in cycle['stages']:
            points=stage.get('points',[])
            if not points:continue
            for point in points:
                assert len(point['q'])==6
                for j,q in enumerate(point['q']):
                    lower,upper=bounds['J'+str(j+1)]
                    assert math.isfinite(q) and lower-1e-9<=q<=upper+1e-9
                checked_positions.append(point['q'])
            trajectory_peaks.append(dict(stage=stage['stage'],stage_id=stage['stage_id'],
                samples=len(points),peak_abs_velocity_rad_s=[max(abs(p['v'][j]) for p in points) for j in range(6)],
                peak_abs_delta_q_rad=[max((abs(b['q'][j]-a['q'][j]) for a,b in zip(points,points[1:])),default=0.) for j in range(6)]))
out=dict(source_commit=s['source_commit'],target_id='carton_l05_c03',complete=bool(o['completed_count']),
    planning_diagnostic=s['planning_diagnostic'],solver_calls_added=0,attempts=evidence,
    other_target_logs_checked_without_rate_exemption=other_target_logs,
    accepted_trajectory_peaks=trajectory_peaks,physical_execution='NOT_RUN',execution_ready=False)
out['accepted_sample_position_limits']=dict(samples_checked=len(checked_positions),all_pass=True,
    joints={name:dict(lower_rad=bounds[name][0],upper_rad=bounds[name][1],
        observed_min_rad=min(q[i] for q in checked_positions),observed_max_rad=max(q[i] for q in checked_positions))
        for i,name in enumerate(('J1','J2','J3','J4','J5','J6'))})
(p/'analysis/rate-exception-evidence.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(dict(complete=out['complete'],markers=sum(a['marker_count'] for a in evidence),
    joints=sorted({j for a in evidence for j in a['peaks_by_joint']}))))
