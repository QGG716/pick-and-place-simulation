"""Offline TCP calibration, RGB-D and native-cycle evidence in one delivery."""
import argparse
import gzip
import hashlib
import html
import json
from pathlib import Path
import shutil
import zipfile
from unloading_sim.eco65.model import ROOT,load_json,save_json
from export_eco65_coupled_replay import export


def deliver(a):
    out=a.output.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    snapshot=load_json(a.snapshot);binding=snapshot['scene']['perception_binding'];cal=binding['calibration']
    run=load_json(a.run/'run_report.json');plan_path=a.run/run.get('plan_file','plan.json' if run.get('complete') else 'partial_plan.json')
    replay=export(a.snapshot,out/'replay.html',plan_path=plan_path,run_folder=a.run)
    plan=load_json(plan_path);stages=[]
    for r in plan['records']:
        stages.append(dict(kind=r['kind'],stage=r['stage'],native_status=r['native_result']['status'],
            target_before=r['state_before']['target_id'],target_after=r['state_after']['target_id'],
            q_start=r['state_before']['q'],q_end=r['state_after']['q'],
            height_start_m=r['state_before']['height_m'],height_end_m=r['state_after']['height_m']))
    reasons={};successes={};native_wall=0.
    with gzip.open(a.run/'native_calls.jsonl.gz','rt',encoding='utf8') as stream:
        for line in stream:
            row=json.loads(line);req=row['request'];reply=row.get('result',{});key=req.get('stage',req['op'])+':'+reply.get('status','ERROR')
            successes[key]=successes.get(key,0)+1
            native_wall+=float(row.get('wall_s',0.))
            for f in (reply.get('clearance') or {}).get('rejected_examples',[]):
                key=' / '.join(f.get('pair',[]))+':'+f.get('reason','UNKNOWN');reasons[key]=reasons.get(key,0)+1
    validation=cal['tcp_heldout_validation'];det=binding['detection']
    report=dict(schema='eco65_tcp_perception_pipeline_delivery_v1',complete=run.get('complete',False),
        completion_goal=plan.get('completion_goal'),inventory_ids=[b['id'] for b in snapshot['scene']['boxes']],
        process_history=plan.get('process_history'),
        target_id=plan['first_target_id'],next_target_id=plan.get('next_target_id'),
        source_layout_snapshot_fingerprint=binding['source_layout_snapshot_fingerprint'],
        planning_snapshot_fingerprint=snapshot['fingerprint'],calibration=dict(method=cal['calibration_method'],status=cal['status'],
            known_probe_tip_F_m=cal['tcp_observations']['probe']['p_F_tip_m'],touches=len(cal['tcp_observations']['touches']),
            views=len(cal['views']),heldout_max_point_error_m=validation['held_out_max_3d_m'],
            truth_error=validation['extrinsic_truth_error'],intrinsics_source=cal['intrinsics_source']),
        vision=dict(status=det['status'],selected=det['detections'][det['selected_column_rank']],assumptions=det['assumptions']),
        planning=run,stages=stages,native_outcomes=successes,collision_failure_counts=reasons,
        native_requests_including_ipc_wall_s=native_wall,
        robot_motion_s=plan['robot_motion_s'],mechanism_display_s=plan['mechanism_display_s'],
        event_display_s=replay['times']['event_display_s'],nominal_drop_s=replay['times']['nominal_drop_s'],
        task_policy=plan['j1_direction_policy'],adopted_process=plan.get('adopted_process'),
        completed_cartons=plan['terminal_state']['completed_ids'] if plan['terminal_state'] else [],
        calibration_motion='DISCRETE_STATIC_NATIVE_IK_FK_AND_GEOMETRIC_TOUCH; NO_CONTINUOUS_TOUCH_PATH_CLAIM',
        physical_execution='NOT_RUN',execution_ready=False,planning_only=True)
    terminal=plan['terminal_state'] or {}
    report['terminal_inventory']=dict(world_cartons=[x['id'] for x in terminal.get('world',[]) if x.get('category')=='carton'],
        attached=terminal.get('attachment'),completed_ids=terminal.get('completed_ids',[]))
    wall=run.get('planning_wall_s')
    wall_scope='FULL_RUN'
    if wall is None:
        wall=run.get('prefix_planning_wall_s',plan.get('planning_wall_s'))
        wall_scope='ACCEPTED_PREFIX_CHECKPOINT'
    report['displayed_planning_wall_s']=wall
    report['displayed_planning_wall_scope']=wall_scope
    wall_text='未记录' if wall is None else f'{wall:.3f} s'
    wall_label='截至已完成前缀的规划墙钟' if wall_scope=='ACCEPTED_PREFIX_CHECKPOINT' else '规划墙钟'
    save_json(out/'report.json',report)
    evidence=out/'evidence';evidence.mkdir()
    for source in a.evidence:
        dest=evidence/source.name
        shutil.copytree(source,dest,ignore=shutil.ignore_patterns('*.usdc','*.usda','*.obj','geometry','*.mp4','*.tar.gz'))
    # Real input imagery is retained alongside the embedded offline viewer.
    image=next(p for p in evidence.rglob('detections.png'))
    task_label=('全部 '+str(len(snapshot['scene']['boxes']))+' 箱连续卸垛'
                if plan.get('completion_goal')=='ALL_CARTONS_DRAINED' else '首箱抓放与下一箱接触')
    body=f'''<!doctype html><meta charset="utf-8"><title>ECO65 TCP 标定与流程结果</title>
<style>body{{font:16px system-ui;max-width:1050px;margin:30px auto;padding:0 22px;background:#101820;color:#e7f0f5}}a{{color:#80ddd1}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#18232f;padding:18px}}img{{max-width:640px;width:100%}}</style>
<h1>ECO65 · 探针尖端 TCP 标定 → 视觉 → 原生规划</h1>
<p>{task_label}。已完成 {len(report['completed_cartons'])} 箱。{'完整任务已完成。' if report['complete'] else '完整任务尚未跑通；以下保留本次成功步骤与失败证据。'}</p>
<p><a href="replay.html">打开离线三维回放与真实搜索诊断</a> · <a href="report.json">完整数值报告</a></p>
<p>新标定：9 个探针触点、6 张图像；留出触点最大误差 {validation['held_out_max_3d_m']*1000:.3f} mm。探针尖端沿用已知法兰偏置，相机外参重新求解。内参为经渲染射线核对的仿真投影参数。</p>
<p>标定触点是原生 IK/FK 加静态几何接触观测，未执行连续探针运动。箱体检测来自新的 RGB-D；不可见部分由已知尺寸和堆叠先验补全。</p>
<img src="{image.relative_to(out).as_posix()}" alt="本次 RGB-D 检测">
<p>{wall_label} {wall_text}；接受轨迹机器人时间 {plan['robot_motion_s']:.3f} s；机构展示时间 {plan['mechanism_display_s']:.3f} s。</p>
<p>planning_only=True · physical_execution=NOT_RUN · execution_ready=false</p>
<pre>{html.escape(json.dumps(report,ensure_ascii=False,indent=2))}</pre>'''
    (out/'report.html').write_text(body,encoding='utf8')
    with zipfile.ZipFile(out/'delivery.zip','w',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(out.rglob('*')):
            if path.is_file() and path.name not in ('delivery.zip','replay.zip'):
                z.write(path,path.relative_to(out).as_posix())
        # The existing native bundle holds exact requests, returns and traces.
        z.write(out/'replay.zip','replay.zip')
    save_json(out/'delivery_manifest.json',dict(files=[dict(path=p.relative_to(out).as_posix(),bytes=p.stat().st_size,
        sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in out.iterdir() if p.is_file()],physical_execution='NOT_RUN',execution_ready=False))
    print('DELIVERY',out,report['complete'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('snapshot','run','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--evidence',type=Path,nargs='+',required=True);deliver(p.parse_args())
