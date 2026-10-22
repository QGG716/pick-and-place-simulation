"""Read one retained low-top IK failure; reuse the production model and viewer."""
from pathlib import Path
import argparse,copy,hashlib,json,sys,tarfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('--target',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-root',type=Path,default=ROOT)
    p.add_argument('--overlay-dir',type=Path,default=ROOT/'tools/search_replay')
    a=p.parse_args();work=a.source_root
    sys.path[:0]=[str(work/'tools'),str(work/'src')]
    from export_m710_search_replay import export_model
    from plan_m710_mount_comparison import make_context
    from unloading_sim.conveyor_lift import coupled_conveyor_state
    from unloading_sim.moveit2_backend import box_message
    import numpy as np
    read=lambda f:json.loads(f.read_text())
    summary=read(a.run/'summary.json');protocol=read(a.run/'protocol.json')
    outcome=next(o for o in summary['outcomes'] if o['target_id']==a.target)
    assert outcome['completed_count']==0
    receipt=outcome['evidence'];archive=a.run/receipt['archive'];prefix=archive.name.removesuffix('.tar.gz')
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==receipt['sha256']
    with tarfile.open(archive) as t:
        def raw(name):
            b=t.extractfile(prefix+'/'+name).read()
            assert hashlib.sha256(b).hexdigest()==receipt['file_sha256'][name],name
            return b
        for name in receipt['file_sha256']:raw(name)
        result=json.loads(raw('result.json'))
        rows=[r for r in result['attempts'] if 'folder' in r]
        assert len(rows)==1
        folder=rows[0]['folder'];trace=json.loads(raw(folder+'/search-trace.json'))
        calls=json.loads(raw(folder+'/requests.json'));detail=json.loads(raw(folder+'/attempt.json'))
    ik=[r for r in calls if r['op']=='ik']
    assert ik and all(r['status']=='NATIVE_IK_FAILED' for r in ik)
    assert not any(r.get('response',{}).get('clearance',{}).get('rejected_examples') for r in calls)
    assert len(trace['attempts'])==1
    lift=trace['attempts'][0];axis=lift['external_axis']
    assert lift['response_identity']['status']=='SUCCESS' and all(c['passed'] for c in axis['checks'])
    call=ik[-1];q=call['q_start'];end_world=lift['request']['receiver_end_world']
    assert q==axis['robot_q'] and call['q_goal'] is None
    assert all(r['q_start']==q and r['goal_pose']==call['goal_pose'] and r['scene_fingerprint']==lift['request']['receiver_end_fingerprint'] for r in ik)
    assert not detail.get('contact_complete') and detail['target']['face_role']=='TOP'
    context,built,audit=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'])
    assert type(built.robot).__name__=='PinocchioHppFclBackend'
    layout=context.policy.layout_validation.layout
    model=export_model(trace['model']);frames=[]
    cartons=[b for b in end_world if b['id'] in {c['id'] for c in outcome['root']['world_cartons']}]
    for checked in axis['checks']:
        world=[box_message(b) for b in coupled_conveyor_state(layout,checked['height_m'],()).boxes]+cartons
        frames.append(dict(height_m=checked['height_m'],world=world))
    key=lambda b:b['id']
    assert sorted(frames[0]['world'],key=key)==sorted(lift['request']['world'],key=key)
    assert sorted(frames[-1]['world'],key=key)==sorted(end_world,key=key)
    req={k:copy.deepcopy(v) for k,v in call.items() if k not in ('response','status','round_trip_s')}
    req.update(world=end_world,process_policy=dict(target_id=a.target))
    expected=detail['expected_contact'];target=detail['target']
    case=dict(target_id=a.target,kind='ik',title='顶吸接近 IK 未求解，预算耗尽',
        detail=f"受检降带 {axis['from_height_m']:.2f}→{axis['to_height_m']:.2f} m 通过；{len(ik)} 次原生IK请求均未得到解。尚未进行目标构型碰撞检查，未生成接近轨迹，也未到contact。尚不能定位具体关节或几何限制，也不能证明全局不可达。",
        state_label='真实起点 q · 未生成目标关节解',display_q=q,
        target_box=next(b for b in end_world if b['id']==a.target),target=target,collision=None,
        counts=dict(numerical_ik_success=0,pregrasp=0,contact=0,checked_lift=1),
        task_wall_s=outcome['planning_wall_s'],failure=detail['failure'],height_m=axis['to_height_m'],delta_m=0.,
        actual_contact_cups=None,commanded_cups=sum(expected['commanded_active_mask']),region_samples=[],
        native_response={k:call['response'][k] for k in ['request_id','status','error_code','solver','solver_api','actual_ik_s']},
        production_flange_column_major=built.robot.named_link_frames(np.array(q))['flange'].T.reshape(-1).tolist())
    data=dict(label=a.target+' 本轮失败',model=model,inspection_cases=[case],
        inspection_title=a.target.removeprefix('carton_')+' · 顶吸接近 IK 失败',
        inspection_subtitle='本轮规划记录 · 非物理执行<br>累计 −X500 mm / −Y100 mm',
        inspection_summary='现场为本箱派发时的20箱。此前20箱来自本轮完整抓放。未附着、未抽离、未释放；没有本轮碰撞见证。',
        raw=dict(run_id='population-x500-20261022-'+a.target,source_commit=summary['source_commit'],
            attempts=[dict(request=req,response_identity=case['native_response'],final_timed_path=[],trace=None,external_axis=None,
                history_status='IK_FAILED_NO_NATIVE_MOTION')]),
        result=dict(complete=False,target_id=a.target),request_log=[],
        ik_prefix=dict(frames=frames,ik_calls=len(ik),source_archive_sha256=receipt['sha256'],
            source_files=receipt['file_sha256'],contact_pose=target['contact_pose'],approach_pose=target['approach_pose'],
            solver_goal_pose=call['goal_pose'],first_request_id=ik[0]['response']['request_id'],last_request_id=call['response']['request_id']),
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    template=(work/'tools/search_replay/viewer.html').read_text()
    vendor=work/'tools/search_replay/vendor'
    template=template.replace('/*__VENDOR__*/','\n'.join((vendor/n).read_text() for n in ['three.min.js','OrbitControls.js']))
    template=template.replace('/*__DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    overlay='\n'.join((a.overlay_dir/n).read_text(encoding='utf-8') for n in ['five_failures.js','ik_prefix.js'])
    template=template.replace('</html>','<script>\n'+overlay+'\n</script></html>')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(template,encoding='utf-8')
    checks=dict(status='PASS',source_commit=summary['source_commit'],source_archive_sha256=receipt['sha256'],
        all_source_files_verified=True,production_backend=type(built.robot).__name__,ik_calls=len(ik),ik_success=0,
        collision_witnesses=0,generated_robot_motion_points=0,lift_checked_samples=len(frames),
        solver_calls_added=0,physical_execution='NOT_RUN')
    a.output.with_name(a.output.stem+'-checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks))


if __name__=='__main__':main()
