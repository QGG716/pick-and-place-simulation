"""Read-only five-case inspection from the preserved 2026-10-21 native run.

No solver/backend is invoked. Reuses the already verified viewer model and FK.
Run from the repository root: python tools/export_m710_five_failures.py
"""
from pathlib import Path
import copy
import hashlib
import json
import lzma
import shutil
import tarfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/experiments/m710_xy_top_population_20261021'
OUT = ROOT / 'docs/experiments/m710_five_failures_20261021'
CACHE = ROOT / 'artifacts/five-failures-source'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def compact_response(r):
    return {k: r[k] for k in ('request_id', 'task_id', 'stage_id', 'parent_stage_id',
            'status', 'reason', 'solver_message', 'moveit_error_code', 'error_code') if k in r}


def main():
    summary, verification = read(SOURCE / 'summary.json'), read(SOURCE / 'verification.json')
    failed = [o for o in summary['outcomes'] if not o['completed_count']]
    assert len(failed) == 5
    tasks = {o['evidence']['archive'].removesuffix('.tar.gz') for o in failed}
    if not all((CACHE / task / 'attempt-001/search-trace.json').exists() for task in tasks):
        CACHE.mkdir(parents=True, exist_ok=True)
        # Decompress once; repeated seeking in xz is expensive.
        archive = ROOT / 'artifacts/five-failures-raw.tar'
        with lzma.open(SOURCE / 'formal-raw.tar.xz', 'rb') as src, archive.open('wb') as dst:
            shutil.copyfileobj(src, dst, 8 * 1024 * 1024)
        with tarfile.open(archive) as tar:
            for member in tar:
                parts = Path(member.name).parts
                if len(parts) > 2 and parts[1] in tasks and member.isfile() and member.name.endswith('.json'):
                    dest = CACHE.joinpath(*parts[1:])
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_bytes(tar.extractfile(member).read())

    old_html = (SOURCE / 'failed-goals.html').read_text(encoding='utf-8')
    old = json.JSONDecoder().raw_decode(old_html.split('const DATA=', 1)[1])[0]
    attempts, cases, checks = [], [], []
    for outcome in failed:
        task = outcome['evidence']['archive'].removesuffix('.tar.gz')
        for name, expected in outcome['evidence']['file_sha256'].items():
            if name.endswith('.json'):
                assert hashlib.sha256((CACHE / task / name).read_bytes()).hexdigest() == expected, name
        folder = CACHE / task / 'attempt-001'
        trace, calls, report = (read(folder / n) for n in ('search-trace.json', 'requests.json', 'attempt.json'))
        verified = next(v for v in verification['outcomes'] if v['target_id'] == outcome['target_id'])
        actual_cups = verified['attempts'][0]['actual_contact_cups']
        if report.get('contact_complete'):
            assert actual_cups == sum(report['terminal_contact']['actual_contact_mask']) == 48
        requests = {a['request']['stage_id']: a['request'] for a in trace['attempts']}
        collision_calls = [c for c in calls if c.get('response', {}).get('clearance', {}).get('rejected_examples')]
        evidence = None
        if report.get('contact_complete'):
            kind = 'region'
            call = next(c for c in reversed(calls) if c['op'] == 'plan' and c['stage'] == 'extraction')
            req = copy.deepcopy(requests[call['stage_id']])
            q = req['q_start']
            assert q == report['contact_end_q_rad']
            assert req['attachment'] and req['q_start'] == trace['attempts'][1]['final_timed_path'][-1]['q']
            assert not collision_calls
            title = '接触通过，抽离搜索未完成'
            detail = '接近和 contact 已通过，实际几何有效 48 杯。抽离／接收区域搜索耗尽阶段预算；最后原生返回 NO_IK_SOLUTION。没有本轮碰撞见证，不能指定碰撞对象。'
            state_label = '已接受 contact 末态 · 抽离起点'
        elif collision_calls:
            kind = 'collision'
            call = collision_calls[0]
            req = copy.deepcopy(requests[call['stage_id']])
            evidence = call['response']['clearance']['rejected_examples'][0]
            q = evidence['q_rad']
            assert q == req['q_goal'] and evidence['phase'] == 'endpoint' and req['attachment'] is None
            title = '顶吸接近目标被碰撞检查拒绝'
            blocker = next(n for n in evidence['pair'] if n.startswith('carton_'))
            detail = f'数值 IK 得到过解，但刚性工具 tool_rigid_0 与 {blocker.removeprefix("carton_")} 相交／接触。该目标没有合法接近轨迹；后续恢复仍未完成，最终阶段预算耗尽。'
            state_label = '本次被拒绝的目标 q · 静态构型'
        else:
            kind = 'ik'
            call = next(c for c in reversed(calls) if c['op'] == 'ik')
            lift = trace['attempts'][0]
            assert lift['response_identity']['status'] == 'SUCCESS'
            assert call['scene_fingerprint'] == lift['request']['receiver_end_fingerprint']
            req = copy.deepcopy({k: v for k, v in call.items() if k not in ('response', 'status', 'round_trip_s')})
            req['world'] = copy.deepcopy(lift['request']['receiver_end_world'])
            q = req['q_start']
            assert q == lift['external_axis']['robot_q'] and req['q_goal'] is None
            assert verified['counts']['numerical_ik_success'] == 0
            title = '顶吸 IK 在预算内未求解'
            detail = '横带受检降到 0.60 m 后，顶吸目标没有得到有效 IK 解。机器人显示记录中的实际起点，青色坐标轴是未求解的请求 TCP；没有目标 q、接近轨迹或碰撞见证。'
            state_label = '最后 IK 请求的真实起点 · 不是解'
        req['process_policy'] = {'target_id': outcome['target_id']}
        keep = ('op', 'task_id', 'stage_id', 'parent_stage_id', 'stage', 'candidate_id', 'scene_fingerprint',
                'q_start', 'q_goal', 'goal_pose', 'world', 'attachment', 'pipeline_id', 'planner_id', 'purpose', 'process_policy')
        req = {k: v for k, v in req.items() if k in keep}
        response = compact_response(call['response'])
        target_box = next(b for b in trace['attempts'][0]['request']['world'] if b['id'] == outcome['target_id'])
        region_samples = [{k: r[k] for k in ('center_world_m', 'stage_id', 'native_result', 'acceptance_status') if k in r}
                          for r in report.get('region_attempts', [])]
        if kind == 'region':
            assert len(region_samples) == report['failure']['samples_attempted'] == 346
            assert all(r['native_result']['status'] != 'SUCCESS' for r in region_samples)
        attempt = dict(request=req, response_identity=response, final_timed_path=[], trace=None,
                       external_axis=None, display_label=state_label,
                       history_status='RECORDED_IN_RAW_ARCHIVE_NOT_EMBEDDED')
        attempts.append(attempt)
        case = dict(target_id=outcome['target_id'], kind=kind, title=title, detail=detail,
                    state_label=state_label, display_q=q, target_box=target_box,
                    target=report['target'], collision=evidence, counts=verified['counts'],
                    task_wall_s=outcome['planning_wall_s'], failure=report['failure'],
                    height_m=verified['attempts'][0]['temporary_height_m'],
                    delta_m=verified['attempts'][0]['delta_m'],
                    actual_contact_cups=actual_cups,
                    commanded_cups=sum(report['expected_contact']['commanded_active_mask']),
                    region_samples=region_samples, receiving_region=report.get('receiving_region'),
                    native_response=response, remaining_process_options=3 if kind == 'region' else 0,
                    source_files={f'{task}/{n}': sha for n, sha in outcome['evidence']['file_sha256'].items() if n.endswith('.json')})
        old_case = next((c for c in old['inspection_cases'] if c['target_id'] == outcome['target_id']), None)
        if old_case:
            assert q == old_case['q_rad']
            case['production_flange_column_major'] = old_case['flange_column_major']
        cases.append(case)
        checks.append(dict(target_id=outcome['target_id'], kind=kind, source_file_hashes_verified=True,
                           native_q_and_world_preserved=True, generated_trajectory_points=0,
                           collision_pair=evidence['pair'] if evidence else None,
                           region_sample_count=len(region_samples)))
    data = dict(label='本轮 5 箱失败原因', model=old['model'], inspection_cases=cases,
                raw=dict(run_id='xy-top-formal-v1-five-failure-inspection', source_commit=summary['source_commit'], attempts=attempts),
                result=dict(complete=False, target_id='5 个失败箱'), request_log=[],
                qualification_status='NOT_EVALUATED', continuous_swept_proof=False,
                physical_execution='NOT_RUN', execution_ready=False)
    template = (ROOT / 'tools/search_replay/viewer.html').read_text(encoding='utf-8')
    vendor = ROOT / 'tools/search_replay/vendor'
    template = template.replace('/*__VENDOR__*/', '\n'.join((vendor / f).read_text(encoding='utf-8') for f in ('three.min.js', 'OrbitControls.js')))
    template = template.replace('/*__DATA__*/', json.dumps(data, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/'))
    overlay = (ROOT / 'tools/search_replay/five_failures.js').read_text(encoding='utf-8')
    template = template.replace('</html>', '<script>\n' + overlay + '\n</script></html>')
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'index.html').write_text(template, encoding='utf-8', newline='\n')
    (OUT / 'source-checks.json').write_text(json.dumps(dict(status='PASS', source_commit=summary['source_commit'], checks=checks), indent=2) + '\n', newline='\n')
    print(json.dumps(dict(cases=len(cases), output=str(OUT / 'index.html'))))


if __name__ == '__main__':
    main()
