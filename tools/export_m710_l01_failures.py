"""Read-only inspection of the retained low-profile front/side l01 failures.

Only recorded q, worlds, attachments and gap witnesses are shown. This exporter
does not call IK, plan motion, recompute gaps, or invent nearest contact points.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOURCE='f17d636ed547a86389150f268445d44bae6f58c4'
RAW_SHA='88638eb2ebf8e8f3cac7067f2a1b8a72af9d755af778680f4b6e57d7793b9d20'


def export(raw,replay_path,output,source_root,overlay):
    assert hashlib.sha256(raw.read_bytes()).hexdigest()==RAW_SHA
    sys.path[:0]=[str(source_root/'src'),str(source_root/'tools')]
    from plan_m710_mount_comparison import make_context
    from export_m710_search_replay import export_model
    replay=json.JSONDecoder().raw_decode(replay_path.read_text(encoding='utf-8').rsplit('const D=',1)[1])[0]
    with tarfile.open(raw) as archive:
        manifest=json.load(archive.extractfile('file-sha256.json'))
        blobs={n:archive.extractfile(n).read() for n in manifest}
    for n,sha in manifest.items():assert hashlib.sha256(blobs[n]).hexdigest()==sha,n
    read=lambda n:json.loads(blobs['formal-v1/'+n])
    summary,protocol=read('summary.json'),read('protocol.json')
    assert summary['source_commit']==replay['source_commit']==SOURCE
    assert protocol['scope']=='XY_TOP_LOWPROFILE250_CONTINUATION'
    assert summary['completed_count']==2 and summary['historical_completed_count']==28
    assert protocol['robot_mount_contact_exemption']
    context,built,audit=make_context(protocol['mounting_variant'],layout_path=source_root/protocol['active_layout_file'],
        robot_mount_contact_exemption=True)
    assert type(built.robot).__name__=='PinocchioHppFclBackend'
    def fk(q):return built.robot.named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist()
    model=replay['model'];cases=[];displays=[];total_witnesses=0
    reference=read('task-001/attempt-001/search-trace.json')['model']
    assert export_model(reference)==model
    def snapshot(request,q,label,*,witness=None,response=None):
        req=copy.deepcopy(request)
        index=len(displays)
        displays.append(dict(request=req,response_identity=response or dict(status='SAVED_INSPECTION_STATE'),
            final_timed_path=[],trace=None,external_axis=None))
        return dict(display_index=index,q_rad=q,state_label=label,witness=witness,production_flange=fk(q))
    for outcome in summary['outcomes']:
        if outcome['completed_count']:continue
        name=outcome['target_id'];task=outcome['evidence']['archive'].removesuffix('.tar.gz')
        result=read(task+'/result.json');root=result['initial']
        assert result['terminal']==root and root['height_m']==.25
        assert all(b['id'] in {x['id'] for x in summary['terminal']['world_cartons']} for b in root['world_cartons'])
        for n,sha in outcome['evidence']['file_sha256'].items():
            assert hashlib.sha256(blobs['formal-v1/'+task+'/'+n]).hexdigest()==sha
        options=[dict(template=a['option']['template'],height_m=a['option']['height_m'],
            failure=a.get('failure'),screening=a.get('screening')) for a in result['attempts'] if 'option' in a]
        assert any(o['template']=='FRONT_DIRECT_HMIN' and abs(o['height_m']+.05)<1e-12 for o in options)
        case=dict(target_id=name,root=root,target_box=next(b for b in root['world_cartons'] if b['id']==name),
            height_m=.25,options=options,task_wall_s=outcome['planning_wall_s'],snapshots=[],
            region_samples=[],blockers=[],actual_contact_cups=None,delta_m=0.,
            source_files=outcome['evidence']['file_sha256'],task=task)
        active=next((a for a in result['attempts'] if a.get('folder')),None)
        detail=None
        if active:
            prefix=task+'/'+active['folder'];calls=read(prefix+'/requests.json');trace=read(prefix+'/search-trace.json')
            # Cartons were committed before these failures, so scene identities
            # differ. Bind the exact robot/tool model, retain each request world.
            identity_without_scene=lambda m:{k:v for k,v in m['identity'].items() if k!='scene_fingerprint'}
            assert identity_without_scene(trace['model'])==identity_without_scene(reference)
            assert trace['model']['parameters']['robot_description']==reference['parameters']['robot_description']
            for key in ('joint_names','tool_links','process_geometry'):
                assert trace['model'][key]==reference[key]
            if 'formal-v1/'+prefix+'/attempt.json' in blobs:detail=read(prefix+'/attempt.json')
        if detail and detail.get('contact_complete'):
            case.update(kind='clearance',failure=detail['failure'],actual_contact_cups=sum(detail['terminal_contact']['actual_contact_mask']),
                commanded_cups=sum(detail['expected_contact']['commanded_active_mask']))
            assert case['actual_contact_cups']==case['commanded_cups']==48
            requests={a['request']['stage_id']:a['request'] for a in trace['attempts']}
            extraction=[c for c in calls if c['op']=='plan' and c['stage']=='extraction']
            first=requests[extraction[0]['stage_id']]
            assert first['q_start']==detail['contact_end_q_rad'] and first['attachment']['id']==name
            assert name not in {b['id'] for b in first['world']}
            contact=[a for a in trace['attempts'] if a['request']['stage']=='contact' and a['final_timed_path']]
            assert contact[-1]['response_identity']['status']=='SUCCESS'
            assert contact[-1]['final_timed_path'][-1]['q']==first['q_start']
            case['snapshots'].append(snapshot(first,first['q_start'],'已通过contact末态 / 抽离起点'))
            case['region_samples']=[dict(center_world_m=r['center_world_m'],distance_m=r.get('distance_m')) for r in detail['region_attempts']]
            assert len(case['region_samples'])==detail['failure']['samples_attempted']
            unique={};pair_counts={}
            for call in extraction:
                req=requests[call['stage_id']]
                for witness in call['response'].get('clearance',{}).get('rejected_examples',[]):
                    total_witnesses+=1;pair=' / '.join(witness['pair']);pair_counts[pair]=pair_counts.get(pair,0)+1
                    assert witness['phase']=='search' and req['attachment']['id']==name
                    assert witness['reason'] in ('CLEARANCE_INSUFFICIENT','INTERSECTION_OR_CONTACT')
                    assert witness['required_pair_clearance_m']==.005
                    if witness['reason']=='CLEARANCE_INSUFFICIENT':assert 0<witness['surface_distance_m']<.005
                    else:assert witness['surface_distance_m']<=0  # Not a penetration-depth measurement.
                    # Equal q/pair in repeated LIN requests is one display state,
                    # while every original occurrence remains counted and archived.
                    key=(pair,tuple(witness['q_rad']))
                    if key not in unique:unique[key]=(call,req,witness)
            for key,(call,req,witness) in sorted(unique.items(),key=lambda item:(item[0][0],item[1][2]['surface_distance_m'])):
                response={k:call['response'][k] for k in ('request_id','stage_id','status','reason','solver_message') if k in call['response']}
                case['snapshots'].append(snapshot(req,witness['q_rad'],'被拒绝的LIN搜索构型 / 非执行位置',
                    witness=witness,response=response))
            case['pair_counts']=pair_counts
            case['counts']=next(o['counts'] for o in read('verification.json')['outcomes'] if o['target_id']==name)
        else:
            if active:
                assert not any(c['op'] in ('ik','plan') for c in calls)
                case['blockers']=next(s['blockers'] for s in active['screening'] if s['geometric_face_id']=='y+')
            case.update(kind='face' if case['blockers'] else 'height',failure=options,counts={})
            world=copy.deepcopy(replay['worlds'][float(root['height_m']).hex()]+root['world_cartons'])
            req=dict(stage='inspection',q_start=root['q_rad'],q_goal=None,world=world,attachment=None,process_policy=dict(target_id=name))
            case['snapshots'].append(snapshot(req,root['q_rad'],'业务筛选时的实际根状态 / 未派发IK'))
        cases.append(case)
    assert len(cases)==5 and sum(c['kind']=='clearance' for c in cases)==3
    cases.sort(key=lambda c:c['target_id'])
    data=dict(label='l01正吸/侧吸失败原因',model=model,inspection_cases=cases,
        raw=dict(source_commit=SOURCE,run_id='lowprofile250-formal-v1-l01-inspection',attempts=displays),
        result=dict(complete=False,target_id='l01五箱'),request_log=[],source_archive_sha256=RAW_SHA,
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    template=(source_root/'tools/search_replay/viewer.html').read_text(encoding='utf-8')
    vendor=source_root/'tools/search_replay/vendor'
    template=template.replace('/*__VENDOR__*/','\n'.join((vendor/n).read_text(encoding='utf-8') for n in ('three.min.js','OrbitControls.js')))
    template=template.replace('/*__DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    template=template.replace('</html>','<script>\n'+overlay.read_text(encoding='utf-8')+'\n</script></html>')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(template,encoding='utf-8')
    save=lambda path,obj:path.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    save(output.with_name('cases.json'),cases)
    checks=dict(status='PASS',source_commit=SOURCE,source_archive_sha256=RAW_SHA,verified_raw_files=len(manifest),
        accepted_model_replay_sha256=hashlib.sha256(replay_path.read_bytes()).hexdigest(),
        exporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),overlay_sha256=hashlib.sha256(overlay.read_bytes()).hexdigest(),
        cases=len(cases),snapshots=len(displays),recorded_witness_occurrences=total_witnesses,
        unique_recorded_witnesses=sum(len(c['snapshots'])-1 for c in cases),
        all_display_states_production_fk_checked=True,solver_calls_added=0,generated_robot_motion_points=0,
        physical_execution='NOT_RUN')
    save(output.with_name('source-checks.json'),checks);print(json.dumps(checks))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('raw',type=Path);p.add_argument('replay',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--source-root',type=Path,default=ROOT)
    p.add_argument('--overlay',type=Path,default=ROOT/'tools/search_replay/l01_failures.js')
    a=p.parse_args();export(a.raw,a.replay,a.output,a.source_root,a.overlay)
