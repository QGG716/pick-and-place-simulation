"""Show retained top-approach IK failures without inventing q goals or motion."""
import argparse,hashlib,json,sys,tarfile
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--source-root',type=Path,default=ROOT)
    args=p.parse_args();work=args.source_root
    sys.path[:0]=[str(work/'tools'),str(work/'src')]
    from export_m710_search_replay import export_model
    from plan_m710_mount_comparison import make_context
    from unloading_sim.conveyor_lift import coupled_conveyor_state
    from unloading_sim.moveit2_backend import box_message
    read=lambda f:json.loads(f.read_text())
    summary=read(args.run/'summary.json');protocol=read(args.run/'protocol.json')
    context,built,audit=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'])
    layout=context.policy.layout_validation.layout
    assert type(built.robot).__name__=='PinocchioHppFclBackend'
    cases=[];attempts=[];model=None
    for o in summary['outcomes']:
        if o['completed_count']:continue
        receipt=o['evidence'];archive=args.run/receipt['archive'];prefix=archive.name.removesuffix('.tar.gz')
        assert hashlib.sha256(archive.read_bytes()).hexdigest()==receipt['sha256']
        with tarfile.open(archive) as t:
            def get(n):
                raw=t.extractfile(prefix+'/'+n).read()
                assert hashlib.sha256(raw).hexdigest()==receipt['file_sha256'][n]
                return json.loads(raw)
            result=get('result.json')
            for row in result['attempts']:
                if row.get('option',{}).get('template')!='TOP_FIXED_CURRENT_RECEIVER' or not row.get('folder'):continue
                folder=row['folder'];calls=get(folder+'/requests.json');trace=get(folder+'/search-trace.json');detail=get(folder+'/attempt.json')
                ik=[c for c in calls if c['op']=='ik'];assert ik and not any(c['status']=='SUCCESS' for c in ik)
                assert not any(c['op']=='plan' for c in calls) and not trace['attempts']
                assert not detail.get('contact_complete') and detail['target']['face_role']=='TOP'
                assert not any(c.get('response',{}).get('clearance',{}).get('rejected_examples') for c in calls)
                root=row['root'];q=root['q_rad'];goal=ik[0]['goal_pose']
                assert all(c['q_start']==q and c['q_goal'] is None and c['goal_pose']==goal for c in ik)
                if model is None:model=export_model(trace['model'])
                world=[box_message(b) for b in coupled_conveyor_state(layout,root['height_m'],()).boxes]+root['world_cartons']
                request=dict(stage='pregrasp',q_start=q,q_goal=None,goal_pose=goal,world=world,
                    attachment=None,process_policy=dict(target_id=o['target_id']))
                response={k:v for k,v in ik[-1]['response'].items() if k in ('request_id','status','error_code','solver','solver_api')}
                attempts.append(dict(request=request,response_identity=response,final_timed_path=[],trace=None,external_axis=None))
                cases.append(dict(target_id=o['target_id'],kind='ik',title='顶吸接近目标 IK 未求解',
                    detail=f"实际派发 {len(ik)} 次原生IK请求，90秒共享前缀预算内没有得到解。未进入目标碰撞、接近、contact或先+Z再−X抽离。图中青色坐标轴仅为请求TCP，机械臂保持本次真实起点。",
                    state_label='实际起点 q · 没有目标关节解',display_q=q,
                    target_box=next(b for b in world if b['id']==o['target_id']),target=detail['target'],
                    collision=None,counts=dict(numerical_ik_success=0,pregrasp=0,contact=0),
                    task_wall_s=result['planning_wall_s'],failure=detail['failure'],height_m=root['height_m'],delta_m=0.,
                    actual_contact_cups=None,commanded_cups=sum(detail['expected_contact']['commanded_active_mask']),
                    region_samples=[],native_response=response,
                    production_flange_column_major=built.robot.named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist(),
                    original_archive_sha256=receipt['sha256'],source_files=receipt['file_sha256'],ik_calls=len(ik)))
    assert cases
    data=dict(label='本轮顶吸IK失败',model=model,inspection_cases=cases,
        inspection_title=f'{len(cases)}箱顶吸接近IK失败 · 本轮记录',
        inspection_subtitle='先+Z再−X路线尚未到达<br>横带保持550 mm · 物理执行NOT_RUN',
        inspection_summary=f"本轮新完成{summary['completed_count']}/{summary['target_count']}，历史28箱单列。每箱显示实际派发时的世界与原生起点；未搜索的支撑阻挡箱见结果报告。",
        raw=dict(source_commit=summary['source_commit'],attempts=attempts),result=dict(complete=False,target_id='顶吸失败箱'),request_log=[],
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    template=(work/'tools/search_replay/viewer.html').read_text(encoding='utf-8');vendor=work/'tools/search_replay/vendor'
    template=template.replace('/*__VENDOR__*/','\n'.join((vendor/n).read_text(encoding='utf-8') for n in ('three.min.js','OrbitControls.js')))
    template=template.replace('/*__DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    template=template.replace('</html>','<script>\n'+(work/'tools/search_replay/five_failures.js').read_text(encoding='utf-8')+'\n</script></html>')
    args.output.parent.mkdir(exist_ok=True,parents=True);args.output.write_text(template,encoding='utf-8')
    checks=dict(status='PASS',source_commit=summary['source_commit'],cases=len(cases),all_source_hashes_verified=True,
        ik_calls=sum(c['ik_calls'] for c in cases),ik_success=0,generated_robot_motion_points=0,collision_witnesses=0,
        backend=type(built.robot).__name__,solver_calls_added=0,physical_execution='NOT_RUN')
    args.output.with_name(args.output.stem+'-checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks))


if __name__=='__main__':main()
