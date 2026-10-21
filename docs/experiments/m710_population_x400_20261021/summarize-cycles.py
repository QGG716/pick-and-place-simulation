"""Read this run's accepted contact q through the same production robot backend."""
from pathlib import Path
from collections import Counter
import json,sys,tarfile
import numpy as np

work=Path('/work-population-x400-90c8304')
sys.path[:0]=[str(work/'tools'),str(work/'src')]
from plan_m710_mount_comparison import make_context
from unloading_sim.upper_arm_preference import upper_arm

run=Path('/population-x400-20261021/formal-v1');out=Path('/population-x400-20261021/analysis');out.mkdir(exist_ok=False)
read=lambda p:json.loads(p.read_text())
save=lambda name,data:(out/name).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
protocol=read(run/'protocol.json');summary=read(run/'summary.json');verified=read(run/'verification.json')
original,connector,audit=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'])
assert type(connector.robot).__name__=='PinocchioHppFclBackend'
rows=[]
by_verify={r['target_id']:r for r in verified['outcomes']}
for outcome in summary['outcomes']:
    name=outcome['target_id'];v=by_verify[name]
    with tarfile.open(run/outcome['evidence']['archive']) as archive:
        result=json.load(archive.extractfile(outcome['evidence']['archive'].removesuffix('.tar.gz')+'/result.json'))
    row=dict(target_id=name,complete=bool(result['completed_count']),
        low_top=protocol['frozen']['heights'][name]['Hmin_world_m']<.6-1e-9,
        layer_index=protocol['frozen']['roles'][name]['original_layer_index'],
        status=result['status'],wall_s=result['planning_wall_s'],counts=v['counts'],
        dispatched_candidates=sum('folder' in a for a in result['attempts']),
        unsearched_options=sum(a.get('unsearched_option_count',0) for a in result['attempts']))
    if result['cycles']:
        cycle=result['cycles'][0];q=np.array(cycle['attachment']['contact_q_rad'])
        row['accepted']=dict(delta_m=cycle['target'].get('world_y_offset_m',0.),
            temporary_height_m=cycle['option'].get('pre_approach_height_m',result['initial']['height_m']),
            receive_height_m=cycle['height_m'],synchronization=cycle['receive_sync_location'],
            face=cycle['target']['face_role'],option=cycle['option'],
            actual_contact_q_rad=q.tolist(),actual_contact_upper_arm=upper_arm(connector.robot,q),
            cups={k:sum(mask) for k,mask in cycle['attachment']['selection'].items() if k.endswith('_mask')},
            nominal_drop_m=cycle['nominal_release']['drop_m'])
    else:
        row['failed_candidates']=[a for a in v['attempts'] if 'folder' in a]
    rows.append(row)
for blocked in summary['blocked']:
    name=blocked['target_id']
    rows.append(dict(target_id=name,complete=False,low_top=protocol['frozen']['heights'][name]['Hmin_world_m']<.6-1e-9,
        layer_index=protocol['frozen']['roles'][name]['original_layer_index'],status=blocked['status'],
        supported_cartons=blocked['supported_cartons'],native_search_performed=False,dispatched_candidates=0))
assert len(rows)==40 and len({r['target_id'] for r in rows})==40
save('all-targets.json',dict(source_commit=protocol['source_commit'],backend=type(connector.robot).__name__,rows=rows))
visits=[a for o in summary['outcomes'] for a in o['attempts'] if 'folder' in a]
save('costs.json',dict(total_dispatch_and_archive_wall_s=summary['planning_wall_s'],
    native_task_wall_s=summary['native_task_wall_s'],candidate_visit_wall_s=sum(a['wall_s'] for a in visits),
    costs_are_nested_not_additive=True,dispatched_candidates=len(visits),
    accepted_candidates=sum(a['complete'] for a in visits),failed_candidates=sum(not a['complete'] for a in visits),
    native_requests=verified['counts']['native_requests'],request_counts=verified['counts'],
    tasks=[{k:r.get(k) for k in ('target_id','status','wall_s','dispatched_candidates','unsearched_options')} for r in rows]))
print(json.dumps(dict(completed=sum(r['complete'] for r in rows),targets=len(rows),
    low_completed=sum(r['complete'] for r in rows if r['low_top']),
    high_completed=sum(r['complete'] for r in rows if not r['low_top']),
    failures=[r['target_id'] for r in rows if not r['complete']]),ensure_ascii=False))
