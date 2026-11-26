"""Offline 3D viewer of saved field planning failures (current record format).

Shows, per record: the frozen start pose (and optional reference start poses
from other records), every planned native path (approach / contact / detach /
return), each endpoint precheck state and each sent-path audit rejection with
its colliding pair in red. Reads saved records only: never connects devices,
never plans, never marks anything executable.
"""
import argparse
import base64
from copy import deepcopy
import glob
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
from export_eco65_search_replay import export_model
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.scene import load_snapshot
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA, load_mapping

STAGES = ('approach', 'contact', 'detach', 'return', 'return_empty', 'withdraw')
STAGE_CN = dict(approach='接近', contact='接触', detach='脱离', ret='带箱返回')


def kind(reason):
    if reason.startswith('ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED'):
        return 'ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED'
    if 'mast' in reason and reason.startswith('detach'):
        return 'RETURN_ROBOT_VS_MAST'
    return reason.split(':')[0] + (':' + reason.split(':')[1] if reason.count(':') > 1 else '')


def short(reason):
    return reason if len(reason) < 1200 else reason[:1200] + ' …'


def path_entry(f, coordinates, label):
    d = json.loads(f.read_text()); r = d.get('result') or {}
    pts = r.get('points') or []
    if not isinstance(r, dict) or len(pts) < 2:
        return None
    step = max(1, len(pts) // 150); shown = pts[::step]
    if shown[-1] is not pts[-1]:
        shown.append(pts[-1])
    return dict(name=f.name, stage=label, status=r.get('status'), q=[p['q'] for p in shown],
                tcp=[coordinates.tcp(p['q'])[:3, 3].tolist() for p in shown], samples=len(pts),
                displayed_samples=len(shown), max_velocity_rad_s=max(abs(v) for p in pts for v in p.get('v', [0])),
                max_acceleration_rad_s2=max(abs(v) for p in pts for v in p.get('a', [0])),
                source=f.name, attachment=d.get('request', {}).get('attachment'), audited=False)


STAGE_LABEL = dict(approach='接近', contact='接触', detach='脱离', approach_line='接近关节直线预检', ret='带箱返回',
                   return_empty='空载返回', withdraw='退出', start='起点')
STAGE_LABEL['return'] = '带箱返回'
PLANNER_LABEL = (('after-pilz-audit-ompl', 'OMPL 避让（Pilz 直连被审计拒绝后）'), ('after-pilz-audit', 'Pilz 被审计拒绝后的回退请求'),
                 ('pilz-ladder', 'Pilz 直线（降速重试）'), ('pilz', 'Pilz 直连'), ('ompl', 'OMPL 避让'))


def pair_text(failure, target):
    pair = [('目标箱' if x == target else x) for x in (failure.get('pair') or [])]
    gap = failure.get('surface_distance_m', -1)
    need = failure.get('required_pair_clearance_m')
    dist = '相交' if gap is None or gap < 0 else f"{gap*1000:.4f} mm" + (f"（要求 {need*1000:.1f} mm）" if need else '')
    return ' ↔ '.join(pair) + ' ' + dist if pair else (failure.get('reason') or '拒绝')


def posture_ranking(folder):
    f = folder / 'approach-endpoint-ik-native.json'
    if not f.exists():
        return []
    def find(x):
        if isinstance(x, dict):
            if 'posture_ranking' in x:
                return x['posture_ranking']
            x = list(x.values())
        if isinstance(x, list):
            for v in x:
                r = find(v)
                if r:
                    return r
        return None
    return find(json.loads(f.read_text())) or []


def near(a, b, tol=2e-3):
    return a is not None and b is not None and np.max(np.abs(np.asarray(a, float) - np.asarray(b, float))) < tol


def branch_states(folder, target, paths, planned, q_start=None):
    """Endpoint prechecks grouped per IK branch (deduplicated over the
    endpoint-precheck*.json files), each group titled with its posture score
    and outcome; planned paths and audit points get their branch number."""
    branches = []; tried = []
    # The screen first, then the -altN rechecks in their numeric order.
    rank = lambda f: (0, 0) if f.name == 'endpoint-precheck.json' else (1, int(''.join(c for c in f.stem.split('-alt')[-1] if c.isdigit()) or 0))
    for pre in sorted(folder.glob('endpoint-precheck*.json'), key=rank):
        current = None
        for check in json.loads(pre.read_text()):
            # front_branch_first: the blocked front branch is planned; a branch
            # rechecked as -altN was planned after an earlier branch failed.
            if check.get('stage') == 'front_branch_first':
                tried.append(check['front_q'])
            if check.get('stage') == 'approach' and check.get('q') is not None and '-alt' in pre.name:
                tried.append(check['q'])
            if check.get('q') is None:
                continue
            if check['stage'] == 'approach':
                current = next((b for b in branches if near(b['q'], check['q'], 1e-4)), None)
                if current is None:
                    current = dict(q=check['q'], rows={}); branches.append(current)
            if current is not None:
                current['rows'].setdefault(check['stage'], check)
    ranking = posture_ranking(folder)
    final = next((p for p in paths if p['name'] == 'approach-native.json'), None) if planned else None
    for n, b in enumerate(branches, 1):
        b['n'] = n
        b['score'] = next((r for r in ranking if near(r['q'], b['q'], 1e-4)), None)
        b['planned'] = [p for p in paths if p['stage'] == 'approach' and near(p['q'][-1], b['q'])]
        for p in paths:
            contact = b['rows'].get('contact', {}).get('q')
            if near(p['q'][-1], b['q']) or near(p['q'][0], b['q']) or near(p['q'][0], contact) or near(p['q'][-1], contact):
                p.setdefault('branch', n)
    out = []
    for b in branches:
        rows = b['rows']
        endpoint_fail = next((rows[k]['failure'] for k in ('approach', 'contact', 'detach') if k in rows and rows[k].get('failure')), None)
        line = rows.get('approach_line', {})
        if final is not None and near(final['q'][-1], b['q']):
            verdict = '最终计划采用'
        elif endpoint_fail:
            verdict = '终态被挡'
        elif b['planned'] or any(near(q, b['q'], 1e-4) for q in tried):
            verdict = '已规划（Pilz 直连 + OMPL 避让），接近段未通过' if line.get('failure') else '已规划，接近段未通过'
        elif line.get('failure'):
            verdict = '直线预检被挡，未规划'
        else:
            verdict = '未规划'
        sc = b['score']
        posture = (f" · 姿态分 {sc['score']:.2f}（大臂偏竖直 {sc['upper_from_up_deg']:.0f}°，小臂偏水平 {sc['forearm_from_horizontal_deg']:.0f}°，"
                   f"J4→J5 偏 −Z {sc['wrist_from_down_deg']:.0f}°）") if sc else ''
        group = f"② 逆解分支 {b['n']} · {verdict}{posture}"
        for stage in ('approach', 'contact', 'detach', 'approach_line'):
            if stage not in rows:
                continue
            f = rows[stage].get('failure')
            mark = '✗ ' + pair_text(f, target) if f else '✓'
            name = STAGE_LABEL[stage] + ('' if stage == 'approach_line' else '位姿')
            out.append(dict(q=rows[stage]['q'], stage='approach' if stage == 'approach_line' else stage, failure=f,
                            group=group, label=f"{name} {mark}"))
            # The blocked state ON the joint line (q_start -> branch endpoint,
            # edge 0 at the recorded fraction), not only the line's endpoint.
            if stage == 'approach_line' and f and f.get('fraction') is not None and q_start is not None and f.get('edge', 0) == 0:
                a = np.asarray(q_start, float); z = np.asarray(rows[stage]['q'], float)
                out.append(dict(q=(a + f['fraction'] * (z - a)).tolist(), stage='approach', failure=f, group=group,
                                label=f"接近关节直线预检 被挡点（直线 {f['fraction']*100:.0f}% 处） ✗ {pair_text(f, target)}"))
    for p in paths:
        rest = p['name'][len(p['stage'])+1:-len('-native.json')] if p['name'] != f"{p['stage']}-native.json" else ''
        planner = next((v for k, v in PLANNER_LABEL if f"{p['stage']}-{k}" in p['name']),
                       f"方案 {rest.upper()}" if rest else '本段最后一次规划输出')
        p['group'] = '④ 规划路径'
        p['label'] = f"{STAGE_LABEL.get(p['stage'], p['stage'])} · {planner}" + (f" · 分支 {p['branch']}" if p.get('branch') else '')
    return out


def read_case(record, coordinates, references):
    obs = json.loads((record / 'observation.json').read_text())
    geo = json.loads((record / 'geometry.json').read_text())
    scene = json.loads((record / 'site-scene.json').read_text())
    pf = json.loads((record / 'planning-failures.json').read_text())
    # Field records: {failures, planning_budget}; offline replays: a plain list.
    budget = pf.get('planning_budget', {}) if isinstance(pf, dict) else {}
    failures = pf['failures'] if isinstance(pf, dict) else pf
    manifests = sorted(record.glob('*-candidates.json'))
    target = (budget['targets'][0]['target'] if budget.get('targets') else
              failures[0]['target'] if failures else manifests[0].name[:-len('-candidates.json')])
    box = next(b for b in geo['boxes'] if b['id'] == target)
    # Searched candidates without a failure (an accepted plan) are shown too.
    known = {x['candidate'] for x in failures if 'candidate' in x}
    for folder in sorted(record.glob(f'{target}-[0-9]*')):
        index = int(folder.name.rsplit('-', 1)[1])
        if folder.is_dir() and index not in known:
            failures.append(dict(target=target, candidate=index, reason='NO_FAILURE_RECORDED: 已规划（无失败记录）'))
    arr = np.load(record / 'observation.npz'); masks = np.load(record / 'segmentation.npz')
    rgb = arr['rgb'].copy(); mask = masks[f"mask_{box.get('detection_index', 0)}"].astype(bool)
    rgb[mask] = (rgb[mask] * .6 + np.array([30, 220, 160]) * .4).astype(np.uint8)
    image = io.BytesIO(); Image.fromarray(rgb).save(image, format='PNG')
    attachment = None
    candidates = []
    planned = any(record.glob('complete-plan*'))
    for failure in sorted((x for x in failures if 'candidate' in x), key=lambda x: x['candidate']):
        folder = record / f"{target}-{failure['candidate']}"
        states = [dict(q=r['q'], stage='start', failure=None, label=r['label'], group='① 起点') for r in references]
        paths = []
        for stage in STAGES:
            for f in sorted(folder.glob(f'{stage}*-native.json')):
                if any(t in f.name for t in ('endpoint', 'sent-p', '-ik', '-line')):
                    continue
                p = path_entry(f, coordinates, stage)
                if p:
                    paths.append(p)
                    attachment = attachment or p['attachment']
        states += branch_states(folder, target, paths, planned, obs['q_start'])
        # Diagnostic replays may keep one numbered copy per attempt
        # (name.nNN.json); each is matched to its IK branch by the goal.
        branch_of = {}
        for s in states:
            if s['label'].startswith('接近位姿') and '逆解分支 ' in s.get('group', ''):
                branch_of[tuple(np.round(s['q'], 4))] = s['group'].split('逆解分支 ')[1].split(' ')[0]
        def branch_name(goal):
            return next((f'分支 {n}' for q, n in branch_of.items() if goal is not None and near(q, goal, 1e-3)), None)
        # Offline via-point experiment (via-probe.json, 2026-11-26 "先试试中间点"):
        # start -> via -> branch endpoint, both legs checked like the joint-line screen.
        probe = folder / 'via-probe.json'
        VIA_LABEL = dict(A_J1_only='只先转 J1', B_J1_and_wrist='先转 J1 和手腕',
                         C_J1_wrist_arm_back_15='先转 J1 和手腕、大臂后倒 15°', C_J1_wrist_arm_back_30='先转 J1 和手腕、大臂后倒 30°')
        shown_ends = []
        for row in (json.loads(probe.read_text()) if probe.exists() else []):
            if any(near(row['endpoint'], e, 1e-4) for e in shown_ends):
                continue
            shown_ends.append(row['endpoint']); b = branch_name(row['endpoint']) or '分支 ?'
            for key, v in row['variants'].items():
                group = f"⑥ 中间点试验 · {b} · {VIA_LABEL.get(key, key)} · {'两段都通' if v['clear'] else '被挡'}"
                a, m, z = (np.asarray(x, float) for x in (row['start'], v['via'], row['endpoint']))
                for leg, (u, w, chk) in enumerate(((a, m, v['first']), (m, z, v['second'])), 1):
                    f = chk.get('failure')
                    if f and f.get('fraction') is not None:
                        states.append(dict(q=(u + f['fraction'] * (w - u)).tolist(), stage='approach', failure=f, group=group,
                                           label=f"第 {leg} 段被挡点（{f['fraction']*100:.0f}% 处） ✗ {pair_text(f, target)}"))
                states.append(dict(q=v['via'], stage='approach', failure=None, group=group,
                                   label='中间点 ' + '  '.join(f'{x:.1f}' for x in np.degrees(m)) + '°'))
                qs = [(u + t * (w - u)).tolist() for u, w in ((a, m), (m, z)) for t in np.linspace(0, 1, 60)]
                paths.append(dict(name=f'via-{key}', stage='approach', status='VIA_CLEAR' if v['clear'] else 'VIA_BLOCKED', q=qs,
                                  tcp=[coordinates.tcp(q)[:3, 3].tolist() for q in qs], samples=len(qs), displayed_samples=len(qs),
                                  max_velocity_rad_s=0., max_acceleration_rad_s2=0., source='via-probe.json', attachment=None,
                                  audited=False, group=f'⑥ 中间点路径（关节直线两段）', branch=b.split(' ')[-1],
                                  label=f"中间点 · {b} · {VIA_LABEL.get(key, key)} · {'两段都通' if v['clear'] else '被挡'}"))
        attempts = {}
        for f in folder.glob('*-native.n*.json'):
            d = json.loads(f.read_text()); attempts[int(f.name.rsplit('.n', 1)[1][:-5])] = (d.get('request') or {}).get('q_goal')
        detach_attempts = {}
        for f in folder.glob('detach-*-sent-check.n*.json'):
            detach_attempts[int(f.name.rsplit('.n', 1)[1][:-5])] = f.name.split('-sent-check')[0][len('detach-'):].upper()
        def attempt_of(f):
            if '.n' not in f.name:
                return ''
            n = int(f.name.rsplit('.n', 1)[1][:-5]); later = [k for k in sorted(attempts) if k >= n]
            b = branch_name(attempts[later[0]]) if later else None
            if b:
                return f'（{b}）'
            # A loaded return follows the detach profile audited just before it.
            if f.name.startswith('return-'):
                earlier = [k for k in sorted(detach_attempts) if k < n]
                if earlier:
                    return f'（脱离方案 {detach_attempts[earlier[-1]]} 之后）'
            return f'（第 {n} 次记录）'
        # Searches that returned no path (e.g. OMPL budget exhausted): the
        # state the search's clearance check rejected (its gap witness).
        seen = []
        for f in sorted(folder.glob('*-native.json')) + sorted(folder.glob('*-native.n*.json')):
            if any(t in f.name for t in ('endpoint', 'sent-p', '-ik', '-line')):
                continue
            d = json.loads(f.read_text()); r = d.get('result') or {}
            w = (r.get('clearance') or {}).get('search_gap_witness') if isinstance(r, dict) else None
            if r.get('status') == 'SUCCESS' or not w or not w.get('q_rad') or any(near(w['q_rad'], q, 1e-6) for q in seen):
                continue
            seen.append(w['q_rad'])
            stage = f.name.split('-')[0]
            planner = next((v for k, v in PLANNER_LABEL if f"{stage}-{k}" in f.name), f.name)
            solver_budget = ((r.get('absolute_solver_deadline') or {}).get('budget_s'))
            states.append(dict(q=w['q_rad'], stage=stage, failure=w, group='⑤ 未找到路径的搜索',
                               label=f"{STAGE_LABEL.get(stage, stage)}{attempt_of(f)} · {planner} · {r.get('status')}"
                                     + (f"（预算 {solver_budget:g} s）" if solver_budget else '') + f" · 搜索中被拒状态 ✗ {pair_text(w, target)}"))
        for chk in sorted(folder.glob('*-sent-check.json')) + sorted(folder.glob('*-sent-check.n*.json')):
            c = json.loads(chk.read_text()); fail = c.get('path_failure') or c.get('failure')
            if fail and fail.get('q_rad'):
                stage = chk.name.split('-sent-check')[0]
                sd = fail.get('surface_distance_m', -1)
                states.append(dict(q=fail['q_rad'], stage=stage.split('-')[0], group='③ 发送轨迹审计拒绝点',
                                   label=f"{STAGE_LABEL.get(stage.split('-')[0], stage)}发送轨迹{attempt_of(chk)} ✗ {pair_text(fail, target)}",
                                   failure=dict(fail, surface_distance_m=sd)))
        candidates.append(dict(index=failure['candidate'], reason=short(failure['reason']), kind=kind(failure['reason']),
                               states=states, paths=paths, contact=None, preferred_ik=None))
    world = scene['world'] + [dict(id=b['id'], size=b['size_m'], pose=b['pose'], category='carton')
                              for b in geo['boxes'] if b['id'] != box['id'] and not any(w['id'] == b['id'] for w in scene['world'])]
    life = record / 'planning-task-lifecycle.json'
    lifecycle = json.loads(life.read_text()) if life.exists() else {}
    deg = np.degrees(obs['q_start'])
    case_label = f"{record.name[:24]} · 起点 J1 {deg[0]:.1f}° J4 {deg[3]:.0f}° J5 {deg[4]:.0f}° · {'有完整计划' if planned else '无计划'}"
    return dict(task_id=record.name, case_label=case_label, observation_id=obs['observation_id'], q_start=obs['q_start'], q_capture=obs['q_capture'],
                stream=obs['stream'], world=world, box=box, candidates=candidates, target_failures=[], record=str(record),
                duration_s=float(budget.get('classified_monotonic_s', 0) - budget.get('deadline_monotonic_s', 0)) + 90.0 if budget else 0.,
                retry=None, default_candidate=None, image='data:image/png;base64,' + base64.b64encode(image.getvalue()).decode(),
                sources={}, camera=obs['T_world_camera'], intrinsics=obs['intrinsics'],
                fk_fixture={k: v.tolist() for k, v in coordinates.frames(obs['q_start']).items()},
                lifecycle_status=lifecycle.get('status'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', nargs='+', type=Path)
    parser.add_argument('--reference-start', action='append', default=[], type=Path,
                        help='Record whose frozen q_start is shown for comparison')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/workbench/single_carton.example.json',
                        help='Single-carton config whose execution snapshot supplies the robot/tool/mast model')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    config = load(args.config)
    installation = Installation.load(config['execution']['installation'])
    coordinates = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), coordinates)
    planner.build()
    snapshot = deepcopy(load_snapshot(config['execution']['snapshot'], geometry_only=True))
    snapshot['native_model']['init'] = planner.init; snapshot['scene']['mesh_obstacles'] = []
    snapshot['native_model']['init']['process_geometry']['expected_contour_ids'] = planner.init['process_geometry']['connected_cup_ids']
    model = export_model(snapshot)
    references = []
    for r in args.reference_start:
        obs = json.loads((r / 'observation.json').read_text())
        deg = '  '.join(f'{v:.1f}' for v in np.degrees(obs['q_start']))
        references.append(dict(q=obs['q_start'], label=f'参考起点 {r.name[:8]}（{deg}°）'))
    cases = [read_case(p.resolve(), coordinates, references) for p in args.records]
    data = dict(model=model, cases=cases, physical_execution='NOT_RUN', executable=False, posture_comparison=False, retry=False)
    template = (ROOT / 'tools/search_replay/field_failures.html').read_text()
    template = template.replace('两次现场规划失败 · 保存证据回看', '现场规划失败 · 接近路线 / 碰撞点 / 起点姿态')
    # Grouped, self-describing pose list (user request 2026-11-25: the labels
    # were many and their relation unclear): optgroups per start / IK branch /
    # planned path / audit point; labels carry the verdict and colliding pair.
    for a, b in (("modes.push({label:s.label+(f?' · 拒绝':' · 已检查'),q:[s.q],", "modes.push({group:s.group,label:s.label,q:[s.q],"),
                 ("for(const p of chosen.paths)modes.push({label:`${p.stage} · ${p.name}`,", "for(const p of chosen.paths)modes.push({group:p.group,label:p.label,"),
                 ("modes=[{label:'实测 q_start',q:[active.q_start]", "modes=[{group:'① 起点',label:'实测 q_start',q:[active.q_start]"),
                 ("$('mode').replaceChildren();modes.forEach((m,i)=>option($('mode'),m.label,i));",
                  "$('mode').replaceChildren();{const groups=new Map();modes.forEach((m,i)=>{const g=m.group||'其他';if(!groups.has(g)){const o=document.createElement('optgroup');o.label=g;groups.set(g,o);$('mode').append(o);}option(groups.get(g),m.label,i);});}"),
                 ("DATA.cases.forEach((c,i)=>option($('case'),`箱位 ${i+1} · ${c.task_id.slice(0,8)}`,i));",
                  "DATA.cases.forEach((c,i)=>option($('case'),c.case_label||c.task_id.slice(0,8),i));"),
                 ("`箱位 ${Number($('case').value)+1} · world", "`${active.case_label||active.task_id.slice(0,8)} · world"),
                 ('<label>现场箱位</label>', '<label>记录（起点 · 结果）</label>'),
                 ('<label>保存的姿态 / 局部原生输出</label>', '<label>姿态 / 路径（① 起点 ② 逆解分支 ③ 审计拒绝点 ④ 规划路径 ⑤ 未找到路径的搜索）</label>')):
        if a not in template:
            raise ValueError('VIEWER_TEMPLATE_ANCHOR_MISSING: ' + a[:60])
        template = template.replace(a, b)
    # Side view (looking along base +Y at the target) and camera access for checks.
    template = template.replace('<button id="focus">目标局部</button>',
                                '<button id="focus">目标局部</button><button id="side">侧视</button>')
    template = template.replace('window.FIELD_VIEWER_READY=true;', (
        "function sideView(){const c=new T.Vector3(...active.box.pose.slice(0,3).map(r=>r[3]));"
        "camera.position.copy(c).add(new T.Vector3(0.08,-0.75,0.05));controls.target.copy(c);controls.update();draw();}"
        "$('side').onclick=sideView;window.FIELD_VIEWER_READY=true;"))
    template = template.replace('window.fieldViewer={fk,', 'window.fieldViewer={sideView,camera,controls,draw,fk,')
    vendor = ROOT / 'tools/search_replay/vendor'
    js = (vendor / 'three.min.js').read_text() + '\n' + (vendor / 'OrbitControls.js').read_text()
    (args.output / 'index.html').write_text(template.replace('/*__VENDOR__*/', js).replace(
        '/*__DATA__*/', json.dumps(data, separators=(',', ':')).replace('</', '<\\/')))
    print(json.dumps(dict(cases=[dict(task=c['task_id'], candidates=len(c['candidates']),
                                      paths=sum(len(x['paths']) for x in c['candidates']),
                                      states=sum(len(x['states']) for x in c['candidates'])) for c in cases],
                          mast_in_model=any('mast_part_019' in x['id'] for x in model['links']),
                          output=str(args.output / 'index.html')), ensure_ascii=False))


if __name__ == '__main__':
    main()
