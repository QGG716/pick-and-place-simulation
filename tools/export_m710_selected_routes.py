"""Read-only inspection of retained, user-directed l01 route attempts."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--source-root',type=Path,default=ROOT)
    p.add_argument('--overlay',type=Path,default=ROOT/'tools/search_replay/selected_routes.js')
    p.add_argument('--geometry-preview',type=Path,help='Saved same-source static c01 envelope, never a robot path')
    args=p.parse_args();work=args.source_root
    sys.path[:0]=[str(work/'src'),str(work/'tools')]
    from plan_m710_mount_comparison import make_context
    from export_m710_search_replay import export_model
    from unloading_sim.conveyor_lift import coupled_conveyor_state
    from unloading_sim.moveit2_backend import box_message
    from unloading_sim.upper_arm_preference import upper_arm
    read=lambda f:json.loads(f.read_text())
    summary=read(args.run/'summary.json');protocol=read(args.run/'protocol.json')
    finite=protocol['scope']=='XY_TOP_L01_C01_FINITE_SIDE_CHANNEL_HMAX'
    c04_side=protocol['scope']=='XY_TOP_L01_C04_SIDE_NEG_Y_HMAX'
    c04_face=protocol['scope']=='XY_TOP_L01_C04_FACE_EXTENSION200_HMAX'
    c04_trial=c04_side or c04_face
    assert c04_trial or finite or protocol['scope']=='XY_TOP_L01_SELECTED_SIDE_TOP_HMAX'
    assert summary['completed_count']==0 and summary['historical_completed_count']==(34 if c04_trial else 32)
    assert [o['target_id'] for o in summary['outcomes']]==(['carton_l01_c04'] if c04_trial else ['carton_l01_c01'] if finite else ['carton_l01_c01','carton_l01_c00','carton_l01_c04'])
    context,built,_=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'],
        robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
    assert type(built.robot).__name__=='PinocchioHppFclBackend'
    layout=context.policy.layout_validation.layout
    cases=[];displays=[];model=None;total_ik=0;solver_s=0.
    for outcome in summary['outcomes']:
        receipt=outcome['evidence'];archive=args.run/receipt['archive'];prefix=archive.name.removesuffix('.tar.gz')
        assert hashlib.sha256(archive.read_bytes()).hexdigest()==receipt['sha256']
        with tarfile.open(archive) as t:
            def get(n):
                raw=t.extractfile(prefix+'/'+n).read()
                assert hashlib.sha256(raw).hexdigest()==receipt['file_sha256'][n]
                return json.loads(raw)
            result=get('result.json');assert result['initial']==result['terminal']
            assert len(result['attempts'])==1
            row=result['attempts'][0];root=row['root'];folder=row['folder']
            calls=get(folder+'/requests.json');trace=get(folder+'/search-trace.json')
            assert not trace['attempts'] and not any(c['op']=='plan' for c in calls)
            assert not any(c.get('response',{}).get('clearance',{}).get('rejected_examples') for c in calls)
            exported=export_model(trace['model'])
            if model is None:model=exported
            else:assert exported==model
            q=root['q_rad'];option=row['option'];name=outcome['target_id']
            ik=[c for c in calls if c['op']=='ik'];total_ik+=len(ik)
            solver_s+=sum(c['response'].get('actual_ik_s',0.) for c in ik)
            world=[box_message(b) for b in coupled_conveyor_state(layout,root['height_m'],()).boxes]+root['world_cartons']
            case=dict(target_id=name,display_q=q,target_box=next(b for b in world if b['id']==name),
                height_m=root['height_m'],option=option,screening=row['screening'],failure=row['failure'],
                task_wall_s=result['planning_wall_s'],ik_calls=len(ik),actual_contact_cups=None,
                commanded_cups=None,blockers=[],collision_witnesses=[],
                q2_rad=q[1],upper_arm=upper_arm(built.robot,np.asarray(q)),
                production_flange_column_major=built.robot.named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist(),
                source_files=receipt['file_sha256'],original_archive_sha256=receipt['sha256'])
            if ik:
                detail=get(folder+'/attempt.json');goal=ik[0]['goal_pose']
                assert name==('carton_l01_c01' if finite else 'carton_l01_c04') and not detail.get('contact_complete')
                assert all(c['status'] in ('NATIVE_IK_FAILED','NATIVE_IK_REQUEST_DEADLINE')
                    and c['q_start']==q and c['q_goal'] is None and c['goal_pose']==goal for c in ik)
                case.update(kind='ik',goal_pose=goal,commanded_cups=sum(detail['expected_contact']['commanded_active_mask']),
                    predicted_cups=sum(detail['expected_contact']['geometrically_eligible_mask']),
                    title=({'top':'顶吸','side':'−Y侧吸','front':'正吸'}[protocol['c04_face_permission']['requested_face']] if c04_face else '−Y侧吸' if c04_side else '侧吸' if finite else '顶吸')+'接近 IK 未求解',
                    detail=f'90 秒共享前缀预算内实际派发 {len(ik)} 次 IK，均未求解。尚未检查目标碰撞、接近轨迹、contact 或两段抽离。没有本次碰撞见证，也不是全局不可达证明。')
                if finite or c04_side or (c04_face and option['face']['geometric_face_id']=='y-'):
                    case['finite_side_exposure']=row['finite_side_exposure']
                    assert next(f for f in row['screening'] if f['geometric_face_id']==('y-' if c04_trial else 'y+'))['available']
            else:
                assert row['failure']=='BUSINESS_FACE_NOT_CURRENTLY_EXPOSED'
                face=next(f for f in row['screening'] if f['geometric_face_id']=='y+')
                assert not face['available'] and face['reason']=='FACE_OCCLUDED'
                case.update(kind='face',goal_pose=None,blockers=face['blockers'],title='+Y 侧吸被面可见性规则拦截',
                    detail='保留的 +Y 射线遮挡规则判定该面不开放；遮挡箱：'+', '.join(n.replace('carton_','') for n in face['blockers'])+'。尚未派发 IK 或轨迹规划。这是几何筛选结果，不是实测路径碰撞；图中的 +Y 箭头只表示指定方向。')
            displays.append(dict(request=dict(stage='pregrasp',q_start=q,q_goal=None,goal_pose=case['goal_pose'],world=world,
                attachment=None,process_policy=dict(target_id=name)),response_identity=dict(status='SAVED_INSPECTION_STATE'),
                final_timed_path=[],trace=None,external_axis=None))
            cases.append(case)
    data=dict(label='l01 指定三箱本轮记录',model=model,inspection_cases=cases,
        raw=dict(source_commit=summary['source_commit'],attempts=displays),result=dict(complete=False,target_id='l01 指定三箱'),request_log=[],
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    if c04_trial:
        route={'z+':'先+Z再−X','y-':'先−Y再−X','x-':'直接−X'}[cases[0]['option']['face']['geometric_face_id']]
        label='c04 '+cases[0]['title'].removesuffix('接近 IK 未求解')
        data.update(label=label,inspection_title=label+' · 本次 IK 失败记录',
            inspection_subtitle=f'本轮 0/1 · 请求{route}，尚未到达抽离<br>Hmax 250 mm · 物理执行 NOT_RUN',
            historical_completed=34,remaining_cartons=6,scope_note='仅重试 c04；l00 五箱未搜索。',
            inspection_camera=dict(position=[3.7,-4.1,3.6],target=[-.8,.35,.6]),
            tool_connector_extension=protocol['tool_connector_extension'])
    if finite:
        data.update(label='c01 短侧移',inspection_title='c01 短侧移 · 几何与本次 IK 记录',
            inspection_subtitle='c01 单箱 · 本轮 0/1<br>Hmax 固定 250 mm · 物理执行 NOT_RUN')
        if protocol.get('tool_connector_extension_m'):
            data.update(tool_connector_extension=protocol['tool_connector_extension'],
                inspection_title='c01 连接件加长100 mm · 本次实测',
                inspection_subtitle='c01 单箱 · 本轮 0/1 · 连接板140→240 mm<br>Hmax 固定250 mm · 物理执行 NOT_RUN')
    if args.geometry_preview:
        preview=read(args.geometry_preview)
        assert finite and preview['source_commit']==summary['source_commit']
        assert preview['scope']=='STATIC_FIXED_TOOL_POSE_ENVELOPE_NOT_ROBOT_PATH'
        assert preview['target']==cases[0]['target_id'] and preview['tool_body_count']==cases[0]['finite_side_exposure']['tool_bodies']
        data['geometry_preview']=preview
    template=(work/'tools/search_replay/viewer.html').read_text(encoding='utf-8');vendor=work/'tools/search_replay/vendor'
    template=template.replace('/*__VENDOR__*/','\n'.join((vendor/n).read_text(encoding='utf-8') for n in ('three.min.js','OrbitControls.js')))
    template=template.replace('/*__DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    template=template.replace('</html>','<script>\n'+args.overlay.read_text(encoding='utf-8')+'\n</script></html>')
    args.output.parent.mkdir(exist_ok=True,parents=True);args.output.write_text(template,encoding='utf-8')
    checks=dict(status='PASS',source_commit=summary['source_commit'],cases=len(cases),all_source_hashes_verified=True,
        exporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),overlay_sha256=hashlib.sha256(args.overlay.read_bytes()).hexdigest(),
        ik_calls=total_ik,native_solver_s=solver_s,ik_success=0,generated_robot_motion_points=0,collision_witnesses=0,
        backend=type(built.robot).__name__,solver_calls_added=0,physical_execution='NOT_RUN')
    args.output.with_name(args.output.stem+'-checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks))


if __name__=='__main__':main()
