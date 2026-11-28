"""Offline 3D viewer of the full-face unloading simulation (tools/simulate_eco65_face_unloading.py).

One case per suction attempt (FRONT / TOP) of each simulated carton: the
scene as planned (STEP static layer with the horizontal belt, the remaining
cartons, the target), every contact candidate with its failure, the IK
branches and endpoint prechecks, the loaded release state on the belt, the
sent-path audit rejections, the searches that returned no path and every
planned native path, colliding parts in red. Reuses the field-failure viewer
(tools/search_replay/field_failures.html) and its record readers.

Reads saved simulation folders only: never connects devices, never plans,
never marks anything executable.
"""
import argparse
import base64
from copy import deepcopy
import io
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools')); sys.path.insert(0, str(ROOT / 'src'))
from visualize_field_blockers_3d import STAGES, branch_states, kind, pair_text, path_entry, short, PLANNER_LABEL, STAGE_LABEL  # noqa: E402
from export_eco65_search_replay import export_model  # noqa: E402
from unloading_sim.workbench.single_carton.config import load  # noqa: E402
from unloading_sim.workbench.single_carton.planner import SinglePlanner  # noqa: E402
from unloading_sim.workbench.controlled.trajectory import Authority  # noqa: E402
from unloading_sim.workbench.coordinates import Installation, CoordinateModel  # noqa: E402
from unloading_sim.workbench.scene import load_snapshot  # noqa: E402
from unloading_sim.workbench.ui import URDF, URDF_SHA, load_mapping  # noqa: E402

MODE_CN = dict(FRONT='正吸（吸前面）', TOP='顶吸（吸顶面）')
KIND_CN = (('LOADED_Q_START_COLLISION', '落箱位碰撞'), ('ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED', '接近/接触终态被挡'),
           ('approach:NATIVE_REJECTED:NATIVE_SOLVER_BUDGET_EXHAUSTED', '接近段预算内无路径'),
           ('approach:NATIVE_REJECTED', '接近段被拒'), ('contact:NATIVE_REJECTED', '接触段被拒'),
           ('detach:ALL_PROFILES_REJECTED', '脱离/带箱运送全部方案被拒'), ('TARGET_PAYLOAD_BLOCKED_AT_REST', '箱体原位即被挡'),
           ('TARGET_PAYLOAD_INTERSECTS_AT_REST', '箱体原位即相交'), ('HOLDING_CAPACITY', '吸持力不足'),
           ('UNSEARCHED', '截止前未规划'), ('NO_FAILURE_RECORDED', '已规划（无失败记录）'))


def kind_cn(reason):
    return next((v for k, v in KIND_CN if reason.startswith(k)), reason.split(':')[0][:40])


def loaded_release_state(folder, target):
    """The loaded release state (q_release with the carton attached) and its check."""
    f = folder / 'loaded-start-native.json'
    if not f.exists():
        return None
    d = json.loads(f.read_text()); req = d.get('request') or {}; r = d.get('result') or {}
    if req.get('q_start') is None:
        return None
    fail = r.get('failure') if isinstance(r, dict) else None
    if fail is None and isinstance(r, dict) and r.get('status') not in (None, 'SUCCESS', 'VALID'):
        fail = dict(reason=r.get('status'))
    mark = ('✗ ' + pair_text(fail, target)) if fail else '✓'
    return dict(q=req['q_start'], stage='return', failure=fail, attachment=req.get('attachment'),
                group='② 落箱位（吸具朝下，带箱）', label=f'落箱位（带载释放终态） {mark}')


def stack_image(report, target_id, world_ids):
    """Front view (base Y-Z) of the stack: target yellow, still standing tan,
    blocked by fixed structure dark red, already unloaded faint."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK TC', 'Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, ax = plt.subplots(figsize=(3.6, 4.6), dpi=110)
    fig.patch.set_facecolor('#172435'); ax.set_facecolor('#101a25')
    box = report['box_m']
    for s in report['stack']:
        y, z = s['centre'][1], s['centre'][2]
        if s['id'] == target_id: c, a = '#e8b45b', 1.
        elif s['id'] in report['blocked_by_fixed_structure']: c, a = '#8b2c35', 1.
        elif s['id'] in world_ids: c, a = '#b98d5a', .85
        else: c, a = '#3a4b5e', .5
        ax.add_patch(Rectangle((y-box[1]/2, z-box[2]/2), box[1], box[2], facecolor=c, alpha=a, edgecolor='#101a25', lw=.6))
    ax.axhline(report['belt']['belt_top_z_m'], color='#5b8fb9', lw=1.2, ls='--')
    ax.text(0.205, report['belt']['belt_top_z_m']+.01, '纵带顶面', color='#8ec3ea', fontsize=7, ha='right')
    ax.axhline(0.8, color='#8d99ae', lw=1); ax.text(0.205, 0.81, '上盖板', color='#a8bacd', fontsize=7, ha='right')
    ax.set_xlim(0.22, -0.51); ax.set_ylim(-0.82, 0.86); ax.set_aspect('equal')
    ax.tick_params(colors='#a8bacd', labelsize=6)
    for sp in ax.spines.values(): sp.set_color('#38485a')
    ax.set_xlabel('base y (m)  ← 从机械臂看向箱堆', color='#a8bacd', fontsize=7); ax.set_ylabel('base z (m)', color='#a8bacd', fontsize=7)
    fig.tight_layout(); buf = io.BytesIO(); fig.savefig(buf, format='png', facecolor=fig.get_facecolor()); plt.close(fig)
    return 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode()


def read_attempt(run, folder, report, row, attempt, coordinates):
    meta = json.loads((folder/'sim-attempt.json').read_text(encoding='utf-8'))
    box = meta['target']; target = box['id']; q_start = meta['q_start']
    failures = json.loads((folder/'planning-failures.json').read_text(encoding='utf-8'))
    manifest = folder/f'{target}-candidates.json'
    rows = json.loads(manifest.read_text()) if manifest.exists() else []
    listed = [c['index'] for c in rows]; poses = {c['index']: (c['tcp_pose'], c['precontact_pose'], c['active_ids']) for c in rows}
    known = {x['candidate'] for x in failures if x.get('candidate') is not None}
    for index in listed:
        if index in known: continue
        sub = folder/f'{target}-{index}'
        failures.append(dict(target=target, candidate=index, reason=('NO_FAILURE_RECORDED: 已规划（无失败记录）' if sub.is_dir() else
                             'UNSEARCHED: 本次尝试的规划截止时间前没有轮到这个候选（不代表不可行）')))
    target_level = [x for x in failures if x.get('candidate') is None]
    candidates = []
    for failure in sorted((x for x in failures if x.get('candidate') is not None), key=lambda x: x['candidate']):
        sub = folder/f"{target}-{failure['candidate']}"
        states, paths = [], []
        if sub.is_dir():
            for stage in STAGES:
                for f in sorted(sub.glob(f'{stage}*-native.json')):
                    if any(t in f.name for t in ('endpoint', 'sent-p', '-ik', '-line')):
                        continue
                    p = path_entry(f, coordinates, stage)
                    if p: paths.append(p)
            states += branch_states(sub, target, paths, False, q_start)
            for path in paths:   # loaded_return_retry (2026-11-28) records
                name = path['name']
                if '-retry-via-via' in name:
                    leg = name.split('-retry-via-via')[1].split('-')[0]
                    path['label'] = f"带箱返回 · 回程重试：中间点第 {leg} 段（Pilz）"
                elif '-retry-via' in name:
                    path['label'] = '带箱返回 · 回程重试：经中间点（拼接结果）'
                elif '-retry-ompl' in name:
                    path['label'] = '带箱返回 · 回程重试：OMPL 避让搜索'
            rel = loaded_release_state(sub, target)
            if rel: states.insert(0, rel)
            seen = []
            for f in sorted(sub.glob('*-native.json')):
                if any(t in f.name for t in ('endpoint', 'sent-p', '-ik', '-line', 'loaded-start')):
                    continue
                d = json.loads(f.read_text()); r = d.get('result') or {}
                w = (r.get('clearance') or {}).get('search_gap_witness') if isinstance(r, dict) else None
                if r.get('status') == 'SUCCESS' or not w or not w.get('q_rad') or any(np.allclose(w['q_rad'], q, atol=1e-6) for q in seen):
                    continue
                seen.append(w['q_rad']); stage = f.name.split('-')[0]
                planner = next((v for k, v in PLANNER_LABEL if f"{stage}-{k}" in f.name), f.name)
                states.append(dict(q=w['q_rad'], stage=stage, failure=w, group='⑤ 未找到路径的搜索',
                                   label=f"{STAGE_LABEL.get(stage, stage)} · {planner} · {r.get('status')} · 搜索中被拒状态 ✗ {pair_text(w, target)}"))
            for chk in sorted(sub.glob('*-sent-check.json')) + sorted(sub.glob('*-sent-check.n*.json')):
                c = json.loads(chk.read_text()); fail = c.get('path_failure') or c.get('failure')
                if fail and fail.get('q_rad'):
                    stage = chk.name.split('-sent-check')[0]
                    states.append(dict(q=fail['q_rad'], stage=stage.split('-')[0], group='③ 发送轨迹审计拒绝点',
                                       label=f"{STAGE_LABEL.get(stage.split('-')[0], stage)}发送轨迹（{stage}） ✗ {pair_text(fail, target)}",
                                       failure=dict(fail, surface_distance_m=fail.get('surface_distance_m', -1))))
        reason, label = failure['reason'], kind_cn(failure['reason'])
        pre = sub/'endpoint-precheck.json'
        if pre.exists():
            ik = next((c for c in json.loads(pre.read_text()) if c.get('stage') == 'approach' and c.get('conclusion') == 'IK_NOT_FOUND_NOT_CONCLUSIVE'), None)
            if ik is not None:
                label = '接近位姿无逆解（够不到）'
                reason = (f"接近位姿（预接触位）没有逆解：{ik['ik']['seeds']} 个种子 0 个解，机械臂够不到吸具要停的位置"
                          f"（绿色板 = 吸具目标位姿）；随后的接近规划因此失败。\n原始结论：{reason}")
        tcp, precontact, active = poses.get(failure['candidate'], (None, None, []))
        # Loaded stages carry the carton on the tool: audit / search states of
        # detach and return get the attachment of this candidate's own paths.
        carried = next((p['attachment'] for p in paths if p['stage'] in ('detach', 'return') and p.get('attachment')), None)
        for st in states:
            if st['stage'] in ('detach', 'return') and not st.get('attachment') and carried: st['attachment'] = carried
        candidates.append(dict(index=failure['candidate'], reason=short(reason), kind=kind(failure['reason']),
                               kind_cn=label, seals=len(active), states=states, paths=paths, contact=None, preferred_ik=None,
                               tcp_targets=[x for x in (tcp, precontact) if x is not None]))
    plan_file = folder/'plan.json'
    if attempt.get('success') and plan_file.exists():
        plan = json.loads(plan_file.read_text(encoding='utf-8')); accepted = plan.get('index')
        att = plan.get('attachment'); final = []; total = 0.
        if isinstance(att, list): att = dict(pose=att)   # plan.json keeps the flange->carton matrix itself
        for seg in plan.get('segments', []):
            pts = (seg.get('prepared') or {}).get('points') or []
            if len(pts) < 2: continue
            total += float(pts[-1]['t'])
            step = max(1, len(pts)//200); shown = pts[::step]
            if shown[-1] is not pts[-1]: shown.append(pts[-1])
            stage = seg.get('stage'); loaded = stage in ('detach', 'return')
            final.append(dict(name=f'final-{stage}', stage=stage, status='AUDITED', q=[x['q'] for x in shown],
                              tcp=[coordinates.tcp(x['q'])[:3, 3].tolist() for x in shown], samples=len(pts), displayed_samples=len(shown),
                              max_velocity_rad_s=max(abs(v) for x in pts for v in x.get('v', [0])),
                              max_acceleration_rad_s2=max(abs(v) for x in pts for v in x.get('a', [0])),
                              source='plan.json（最终发送轨迹）', attachment=att if loaded else None, audited=True,
                              group='⓪ 最终计划（四段发送轨迹均通过离线审计，无执行许可）',
                              label=f"{STAGE_LABEL.get(stage, stage)} · 最终发送轨迹 · {pts[-1]['t']:.1f} s"))
        hc = (plan.get('candidate') or {}).get('holding_capacity') or {}
        seal = len((plan.get('candidate') or {}).get('active_ids') or [])
        holding = ('通过' if hc.get('valid') else '不足') + (f"（最小裕度 {hc['minimum_margin']:.2f}）" if isinstance(hc.get('minimum_margin'), (int, float)) else '')
        for c in candidates:
            if c['index'] == accepted:
                c['paths'] = final+c['paths']; c['kind_cn'] = '最终计划采用' + ('' if hc.get('valid') else '（吸持力不足，仅记录）')
                c['reason'] = (f'取得完整计划：四段发送轨迹均通过离线审计（无执行许可），运动总时长 {total:.1f} s。\n'
                               f'预计密封 {seal} 杯；吸持力检查：{holding}。现场配置为"只记录、不否决"（用户 2026-11-24），'
                               f'{"按强制检查此计划不会被采用。" if not hc.get("valid") else "强制检查也通过。"}')
            elif c['reason'].startswith('NO_FAILURE_RECORDED'):
                c['kind_cn'] = '无最终结论（另一候选已成功）'
                c['reason'] = ('另一个候选先取得完整计划后，本次尝试停止，这个候选没有记下最终结论：它可能只做了终态预检，'
                               '也可能已在并行车道里部分规划或审计（下方列出的路径和拒绝点就是它留下的记录）。不代表可行或不可行。')
        summary_extra = dict(holding=holding, seal=seal, holding_valid=bool(hc.get('valid')), motion_s=total)
    else:
        summary_extra = {}
    if not candidates:
        reason = '; '.join(str(x.get('reason'))[:400] for x in target_level) or meta.get('error') or '没有生成候选'
        candidates.append(dict(index=-1, reason=short(reason), kind=kind(reason), kind_cn=kind_cn(reason), states=[], paths=[],
                               contact=None, preferred_ik=None, tcp_targets=[]))
    world = [o for o in meta['world'] if o.get('category') != 'workspace_boundary' and o['id'] != target]
    rp = meta['release_policy']; T = np.eye(4); T[:3, 3] = [*rp['position_xy_m'], rp['start_height_m']]
    world.append(dict(id='落箱点（TCP 目标）', size=[.02, .02, .02], pose=T.tolist(), category='marker'))
    world_ids = {o['id'] for o in meta['world'] if o.get('category') == 'carton'}
    counts = {}
    for c in candidates: counts[c['kind_cn']] = counts.get(c['kind_cn'], 0)+1
    deg = np.degrees(q_start)
    return dict(task_id=f"{target}-{attempt['mode'].lower()}", case_label=f"{target} · {MODE_CN[attempt['mode']]} · "
                f"{'有完整计划' if attempt['success'] else '无计划'} · {len(listed)} 个候选",
                q_start=q_start, world=world, box=box, candidates=candidates, duration_s=float(attempt['seconds']), retry=None,
                default_candidate=None, image=stack_image(report, target, world_ids),
                summary=dict(mode=attempt['mode'], mode_cn=MODE_CN[attempt['mode']], success=attempt['success'], error=attempt.get('error'),
                             counts=counts, target_level=[short(str(x.get('reason'))) for x in target_level],
                             start_deg=[round(float(x), 1) for x in deg], release=rp, layer=row['layer'], column=row['column'],
                             gap_m=report['gap_m'], stack_front_x_m=report['stack_front_x_m'], **summary_extra))


PALETTE = ((0.35, 0.80, 0.88), (0.95, 0.66, 0.31), (0.56, 0.85, 0.45), (0.80, 0.55, 0.95), (0.95, 0.45, 0.55),
           (0.98, 0.86, 0.36), (0.40, 0.60, 0.98), (0.55, 0.95, 0.80))
STAGE_ORDER_CN = dict(approach='接近', contact='接触', detach='脱离（直线拉出）', ret='带箱运送到纵带')


def build_combined(run, report, coordinates):
    """One case with every successful carton's audited final plan in unloading
    order: a continuous path (cartons attach at the start of their detach,
    disappear from the stack after their loaded return ends at the release
    point) and one full-cycle path per carton. Display only."""
    rows = [(n, row) for n, row in enumerate(report['results']) if row.get('result') == 'UNLOADED']
    if not rows:
        return None
    first = None; cartons = []; whole = dict(q=[], tcp=[], t=[], colors=[], events=[], segments=[], vmax=0., amax=0.); cycles = []; offset = 0.
    lines = []
    for k, (n, row) in enumerate(rows):
        attempt = next(a for a in row['attempts'] if a.get('success'))
        folder = run/f"{n:02d}-{row['id']}-{attempt['mode'].lower()}"
        meta = json.loads((folder/'sim-attempt.json').read_text(encoding='utf-8'))
        if first is None: first = meta
        plan = json.loads((folder/'plan.json').read_text(encoding='utf-8'))
        att = plan['attachment']; att = att['pose'] if isinstance(att, dict) else att
        hc = (plan.get('candidate') or {}).get('holding_capacity') or {}
        seal = len((plan.get('candidate') or {}).get('active_ids') or [])
        cartons.append(dict(id=row['id'], size=meta['target']['size_m'], attachment=att))
        colour = list(PALETTE[k % len(PALETTE)])
        cyc = dict(q=[], tcp=[], t=[], colors=[], events=[], segments=[], vmax=0., amax=0.); cyc_t = 0.; total = 0.
        for seg in plan['segments']:
            pts = (seg.get('prepared') or {}).get('points') or []
            if len(pts) < 2: continue
            step = max(1, len(pts)//200); shown = pts[::step]
            if shown[-1] is not pts[-1]: shown.append(pts[-1])
            stage = seg['stage']; key = 'ret' if stage == 'return' else stage
            vmax = max(abs(v) for x in pts for v in x.get('v', [0])); amax = max(abs(v) for x in pts for v in x.get('a', [0]))
            for target, base in ((whole, offset), (cyc, cyc_t)):
                target['vmax'] = max(target['vmax'], vmax); target['amax'] = max(target['amax'], amax)
                a = len(target['q'])
                target['q'] += [x['q'] for x in shown]; target['t'] += [base+float(x['t']) for x in shown]
                target['tcp'] += [coordinates.tcp(x['q'])[:3, 3].tolist() for x in shown]
                shade = colour if stage in ('detach', 'return') else [c*.55 for c in colour]
                target['colors'] += [shade]*len(shown)
                target['segments'].append(dict(k=k, stage=STAGE_ORDER_CN.get(key, stage), a=a, b=len(target['q'])-1))
                if stage == 'detach': target.setdefault('_attach', {})[k] = a
                if stage == 'return': target['events'].append(dict(k=k, attach=target['_attach'][k], release=len(target['q'])-1))
            offset += float(pts[-1]['t']); cyc_t += float(pts[-1]['t']); total += float(pts[-1]['t'])
        cycles.append((k, row, cyc, total, attempt))
        lines.append(f"{k+1:>2}. {row['id']}  {MODE_CN[attempt['mode']][:2]}  密封 {seal} 杯  吸持力 {'通过' if hc.get('valid') else '不足'}"
                     f"  规划 {attempt['seconds']:.1f} s  运动 {total:.1f} s")
    def entry(name, label, data, group, gone0, samples_note):
        return dict(name=name, stage='all', status='AUDITED', q=data['q'], tcp=data['tcp'], t=data['t'], tcp_colors=data['colors'],
                    events=data['events'], segments=data['segments'], gone0=gone0, samples=len(data['q']),
                    displayed_samples=len(data['q']), max_velocity_rad_s=data['vmax'], max_acceleration_rad_s2=data['amax'],
                    source='各箱 plan.json（最终发送轨迹，逐段抽样显示）' + samples_note, attachment=None, audited=True,
                    group=group, label=label)
    paths = [entry('final-all', f'全部 {len(rows)} 箱 · 按卸箱顺序连续 · 运动 {offset:.0f} s', whole,
                   '⓪ 全部成功路径（连续）', [], '')]
    for k, row, cyc, total, attempt in cycles:
        paths.append(entry(f'final-{row["id"]}', f'第 {k+1} 箱 {row["id"]} · 接近→接触→脱离→运送 · {total:.1f} s', cyc,
                           '⓪ 每箱一轮（四段连续）', list(range(k)), ''))
    for path in paths: path.pop('_attach', None)
    initial = {o['id'] for o in first['world'] if o.get('category') == 'carton'} | {first['target']['id']}
    box = np.asarray(report['box_m'], float)
    world = [o for o in first['world'] if o.get('category') not in ('carton', 'workspace_boundary')]
    for spec in report['stack']:
        if spec['id'] not in initial: continue
        T = np.eye(4); T[:3, 3] = spec['centre']
        world.append(dict(id=spec['id'], size=box.tolist(), pose=T.tolist(), category='carton'))
    rp = first['release_policy']; T = np.eye(4); T[:3, 3] = [*rp['position_xy_m'], rp['start_height_m']]
    world.append(dict(id='落箱点（TCP 目标）', size=[.02, .02, .02], pose=T.tolist(), category='marker'))
    centre = np.mean([s['centre'] for s in report['stack'] if s['id'] in initial], axis=0)
    dummy = np.eye(4); dummy[:3, 3] = centre
    remaining = initial - {c['id'] for c in cartons}
    html = (f"<b>全部成功路径</b> · {len(rows)} 箱按卸箱顺序首尾相接 · 运动总时长 {offset:.0f} s · 规划总用时 "
            f"{sum(a['seconds'] for *_, a in cycles):.0f} s<br>箱堆前面 x {report['stack_front_x_m']:.3f} m（距剪叉末端 {report['gap_m']*1000:.0f} mm）"
            f"<br>正在处理的箱子标黄；吸上后随吸具移动，落到纵带上方后从垛上移除（落下后的去向不建模）。"
            f"<br>TCP 线每箱一种颜色，深色为空载（接近、接触），亮色为带箱（脱离、运送）。")
    reason = ('四段发送轨迹都通过离线审计的箱子，按卸箱顺序首尾相接（上一箱的落箱位就是下一箱的起点）。无执行许可。\n\n'
              + '\n'.join(lines) + f'\n\n其余 {len(remaining)} 箱没有卸下（失败或上面压着箱子），留在垛上。')
    return dict(task_id='all-successful', case_label=f'全部成功路径 · {len(rows)} 箱按卸箱顺序 · 运动 {offset:.0f} s',
                q_start=whole['q'][0], world=world, box=dict(id='__combined_dummy__', size_m=[1e-4]*3, pose=dummy.tolist()),
                cartons=cartons, combined=True,
                candidates=[dict(index=0, reason=reason, kind='ALL_SUCCESSFUL', kind_cn=f'{len(rows)} 箱最终计划', states=[], paths=paths,
                                 contact=None, preferred_ik=None, tcp_targets=[])],
                duration_s=float(sum(a['seconds'] for *_, a in cycles)), retry=None, default_candidate=None,
                image=stack_image(report, None, remaining),
                summary=dict(combined=True, html=html))


COMBINED_PATCHES = (
    # Play controls under the sample slider.
    ('<input id="sample" type="range" min="0" value="0">',
     '<input id="sample" type="range" min="0" value="0"><div class="buttons"><button id="play">▶ 播放</button>'
     '<select id="speed"><option value="1">×1 实时</option><option value="5">×5</option><option value="10" selected>×10</option>'
     '<option value="30">×30</option></select></div>'),
    ("attachment:p.attachment,note:`${p.audited?",
     "attachment:p.attachment,events:p.events,gone0:p.gone0,segments:p.segments,t:p.t,tcp_colors:p.tcp_colors,note:`${p.audited?"),
    ("matrix(active.box.pose));}}draw();}", "matrix(active.box.pose));}}timeline(frames);draw();}"),
    ("i=Number($('sample').value);pose(m.q[i],m.pair||[]);", "i=Number($('sample').value);curSample=i;pose(m.q[i],m.pair||[]);"),
    ("(m.note?'\\n'+m.note:'')", "(m.note?'\\n'+m.note:'')+segInfo(m,i)"),
    ("if(m.tcp){line=new T.Line(new T.BufferGeometry().setFromPoints(m.tcp.map(p=>new T.Vector3(...p))),new T.LineBasicMaterial({color:0x5acde0}));scene.add(line);}",
     "if(m.tcp){const g=new T.BufferGeometry().setFromPoints(m.tcp.map(p=>new T.Vector3(...p)));let mat;if(m.tcp_colors){g.setAttribute('color',new T.Float32BufferAttribute(m.tcp_colors.flat(),3));mat=new T.LineBasicMaterial({vertexColors:true});}else mat=new T.LineBasicMaterial({color:0x5acde0});line=new T.Line(g,mat);scene.add(line);}"),
    ("function mode(){", "function mode(){stopPlay();"),
    ("bad>=0?bad:0;mode();}", "bad>=0?bad:0;if(active.combined){const k=modes.findIndex(m=>m.events);if(k>=0)$('mode').value=k;}mode();}"),
    ("{const s=active.summary;$('stats').innerHTML=`<b>${s.mode_cn}</b>",
     "{const s=active.summary;if(s.combined)$('stats').innerHTML=s.html;else $('stats').innerHTML=`<b>${s.mode_cn}</b>"),
    ("function focus(target=false){",
     "let curSample=0,playing=false,playStart=0,playT0=0,playI0=0;const carriedMeshes={};\n"
     "function timeline(frames){for(const k in carriedMeshes)carriedMeshes[k].visible=false;const m=modes[Number($('mode').value)];if(!m||!m.events)return;"
     "const cs=active.cartons||[],gone=new Set(m.gone0||[]);let carried=-1;for(const e of m.events){if(curSample>e.release)gone.add(e.k);else if(curSample>=e.attach)carried=e.k;}"
     "const hide=new Set([...gone].map(k=>cs[k].id));if(carried>=0)hide.add(cs[carried].id);const seg=(m.segments||[]).find(s=>curSample>=s.a&&curSample<=s.b);"
     "for(const r of world){if(hide.has(r.id))r.o.visible=false;else if(seg&&r.id===cs[seg.k].id){r.o.material.color.set(0xe8b45b);r.o.material.opacity=.85;}}"
     "if(carried>=0){const c=cs[carried],key=$('case').value+':'+carried;let o=carriedMeshes[key];if(!o){o=mesh(new T.BoxGeometry(...c.size),0xe8b45b,.85);scene.add(o);carriedMeshes[key]=o;}"
     "o.matrix.copy(frames[model.flange_link].clone().multiply(matrix(c.attachment)));o.visible=true;}}\n"
     "function segInfo(m,i){if(!m.segments)return '';const s=m.segments.find(x=>i>=x.a&&i<=x.b);if(!s)return '';const c=(active.cartons||[])[s.k];"
     "return `\\n第 ${s.k+1} 箱 ${c?c.id:''} · ${s.stage}`+(m.t?` · 累计运动 ${m.t[i].toFixed(1)} s / ${m.t[m.t.length-1].toFixed(1)} s`:'');}\n"
     "function stopPlay(){playing=false;const b=$('play');if(b)b.textContent='▶ 播放';}\n"
     "function playStep(now){if(!playing)return;const m=modes[Number($('mode').value)],n=m.q.length;let i=Number($('sample').value);"
     "if(m.t){const target=playT0+(now-playStart)/1000*Number($('speed').value);while(i<n-1&&m.t[i+1]<=target)i++;}"
     "else i=Math.min(n-1,playI0+Math.floor((now-playStart)/1000*3*Number($('speed').value)));"
     "if(i!==Number($('sample').value)){$('sample').value=i;sample();}if(i>=n-1){stopPlay();return;}requestAnimationFrame(playStep);}\n"
     "$('play').onclick=()=>{if(playing){stopPlay();return;}const m=modes[Number($('mode').value)];if(!m||m.q.length<2)return;"
     "let i=Number($('sample').value);if(i>=m.q.length-1){i=0;$('sample').value=0;sample();}playing=true;$('play').textContent='⏸ 暂停';"
     "playStart=performance.now();playT0=m.t?m.t[i]:0;playI0=i;requestAnimationFrame(playStep);};\n"
     "function focus(target=false){"),
)


PATCHES = (
    ("modes.push({label:s.label+(f?' · 拒绝':' · 已检查'),q:[s.q],", "modes.push({group:s.group,label:s.label,q:[s.q],"),
    ("for(const p of chosen.paths)modes.push({label:`${p.stage} · ${p.name}`,", "for(const p of chosen.paths)modes.push({group:p.group,label:p.label,"),
    ("modes=[{label:'实测 q_start',q:[active.q_start]", "modes=[{group:'① 起点',label:'本轮起点 q_start',q:[active.q_start]"),
    ("$('mode').replaceChildren();modes.forEach((m,i)=>option($('mode'),m.label,i));",
     "$('mode').replaceChildren();{const groups=new Map();modes.forEach((m,i)=>{const g=m.group||'其他';if(!groups.has(g)){const o=document.createElement('optgroup');o.label=g;groups.set(g,o);$('mode').append(o);}option(groups.get(g),m.label,i);});}"),
    ("DATA.cases.forEach((c,i)=>option($('case'),`箱位 ${i+1} · ${c.task_id.slice(0,8)}`,i));",
     "DATA.cases.forEach((c,i)=>option($('case'),c.case_label,i));"),
    ("active.candidates.forEach((c,i)=>option($('candidate'),`#${c.index} · ${c.kind}`,i));",
     "active.candidates.forEach((c,i)=>option($('candidate'),`#${c.index} · ${c.kind_cn}`+(c.seals!==undefined?` · 预计密封 ${c.seals} 杯`:''),i));"),
    ("const selected=active.retry?active.candidates.findIndex(c=>c.index===active.default_candidate):active.candidates.findIndex(c=>c.kind==='ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED');",
     "const accepted=active.candidates.findIndex(c=>c.kind_cn.startsWith('最终计划采用')),selected=accepted>=0?accepted:active.candidates.findIndex(c=>c.paths.length||c.states.length);"),
    ("$('stats').textContent=`任务 ${active.task_id.slice(0,8)} · 失败返回 ${active.duration_s.toFixed(1)} s`;",
     "{const s=active.summary;$('stats').innerHTML=`<b>${s.mode_cn}</b> · ${s.success?'取得完整计划':'没有完整计划'} · 规划用时 ${active.duration_s.toFixed(0)} s`+(s.success?`<br>运动总时长 ${s.motion_s.toFixed(1)} s · 预计密封 ${s.seal} 杯 · 吸持力 ${s.holding}${s.holding_valid?'':'（现场配置只记录）'}`:'')+`<br>第 ${s.layer} 层 第 ${s.column} 列 · 箱堆前面 x ${s.stack_front_x_m.toFixed(3)} m（距剪叉末端 ${(s.gap_m*1000).toFixed(0)} mm）<br>候选结果：${Object.entries(s.counts).map(([k,v])=>k+' '+v).join('，')}`+(s.target_level.length?`<br>目标级：${s.target_level.join('；')}`:'')+(s.error?`<br>中止：${s.error}`:'');}"),
    ("$('hud').textContent=`箱位 ${Number($('case').value)+1} · world = base_link · ${active.world.length} 个现场物体 · ${model.cup_ids.length} 杯`;",
     "$('hud').textContent=`${active.case_label}\\nworld = base_link · ${active.world.length} 个物体（STEP 静态层、纵带已放平、其余纸箱）· ${model.cup_ids.length} 杯`;"),
    ("const target=b.id===active.box.id,o=mesh(g,target?0xe8b45b:0x8298ad,target?.7:.1);o.matrix.copy(matrix(b.pose));scene.add(o);world.push({o,id:b.id,target});",
     "const target=b.id===active.box.id,[color,alpha]=target?[0xe8b45b,.75]:worldColor(b),o=mesh(g,color,alpha);o.matrix.copy(matrix(b.pose));scene.add(o);world.push({o,id:b.id,target,color,alpha});"),
    ("r.o.material.color.set(pair.includes(r.id)?0xff455f:r.target?0xe8b45b:0x8298ad);r.o.material.opacity=pair.includes(r.id)?.8:r.target?.7:.1;",
     "r.o.material.color.set(pair.includes(r.id)?0xff455f:r.color);r.o.material.opacity=pair.includes(r.id)?.85:r.alpha;"),
    ("function candidate(){chosen=active.candidates[Number($('candidate').value)];",
     "let targetMarks=[];function showTargets(list){for(const m of targetMarks)scene.remove(m);targetMarks=(list||[]).map((p,i)=>{const g=new T.Group();g.add(new T.AxesHelper(.07));"
     "const b=new T.Mesh(new T.BoxGeometry(.176,.132,.003),new T.MeshStandardMaterial({color:i?0x9be7c0:0x4fd18b,transparent:true,opacity:i?.18:.4,side:T.DoubleSide,depthWrite:false}));g.add(b);"
     "g.matrixAutoUpdate=false;g.matrix.copy(matrix(p));scene.add(g);return g;});}\n"
     "function candidate(){chosen=active.candidates[Number($('candidate').value)];showTargets(chosen.tcp_targets);"),
    ("function selectCase(){",
     "function worldColor(b){if(b.category==='carton')return [0xb98d5a,.55];if(b.category==='marker')return [0x4fd18b,.95];if(b.role==='BELT_INCLINED')return [0x5b8fb9,.55];"
     "if(b.role==='LIFT_TOP'||b.role==='LIFT_INTERIOR')return [0x6c9a8b,.25];const s=b.size||[1,1,1];if(Math.min(...s)<.012&&Math.max(...s)>.5)return [0x8298ad,.06];return [0x8298ad,.16];}\nfunction selectCase(){"),
    ('<label>现场箱位</label>', '<label>仿真尝试（箱子 · 吸取方式 · 结果）</label>'),
    ("$('hud').textContent=`箱位 ${Number($('case').value)+1} · world = base_link · ${active.world.length} 个现场物体 · ${model.cup_ids.length} 杯\\n${m.label}`;",
     "$('hud').textContent=`${active.case_label}\\n${m.label}`;"),
    ('纯离线显示 · 原始 RGB-D 与对应候选记录 · 没有完整可执行计划 · 没有实际运动',
     '纯离线仿真 · 箱子几何已知（不经相机）· 现场同一套规划与碰撞规则 · 没有完整可执行计划 · 没有实际运动'),
    ('<small>绿色：同帧分割区域；三维黄色箱体包含推断部分。</small>', '<small>箱堆正视图：黄 = 本次目标，棕 = 仍在垛上，暗红 = 与固定结构重叠放不下。</small>'),
    ('<label>候选编号（全部保留）</label>', '<label>吸取候选（全部保留，按规划顺序）</label>'),
    ('<label>保存的姿态 / 局部原生输出</label>', '<label>姿态 / 路径（① 起点 ② 落箱位 / 逆解分支 ③ 审计拒绝点 ④ 规划路径 ⑤ 未找到路径的搜索）</label>'),
    ('显示实测现场障碍', '显示场景物体'),
    ('黄：完整箱体包络　蓝灰：现场障碍　青：原生输出 TCP 采样线（未认证）',
     '黄：目标箱　棕：其余纸箱　蓝：纵带（已放平）　绿方块：落箱点　绿板：本候选吸具目标位姿（深 = 接触位，浅 = 预接触位）　红：碰撞零件　青：TCP 采样线'),
    ('红色：记录所指的碰撞部件 / 障碍；机器人和吸具使用现有 CAD。未导入历史仿真环境。',
     '红色：记录所指的碰撞部件 / 障碍。机器人、吸具、顶端相机桅杆用现有 CAD；场景为 layout 20261127 的 STEP 静态层。纯离线仿真，没有执行许可。'),
)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs', nargs='+', type=Path, help='simulation output folders')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--title', default='整面卸箱仿真 · 失败回看')
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    first = a.runs[0].resolve()
    config = load(first/'sim-config.json')
    installation = Installation.load(config['execution']['installation'])
    coordinates = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), coordinates)
    planner.build()
    snapshot = deepcopy(load_snapshot(config['execution']['snapshot'], geometry_only=True))
    snapshot['native_model']['init'] = planner.init; snapshot['scene']['mesh_obstacles'] = []
    snapshot['native_model']['init']['process_geometry']['expected_contour_ids'] = planner.init['process_geometry']['connected_cup_ids']
    model = export_model(snapshot)
    # The CAD cup colour is red; red means "colliding part" in this viewer.
    for item in model['links']:
        rgb = item.get('rgb')
        if rgb and rgb[0] > .7 and rgb[1] < .4 and rgb[2] < .4: item['rgb'] = [.2, .22, .26]
    cases = []
    for run in a.runs:
        run = run.resolve(); report = json.loads((run/'simulation-report.json').read_text(encoding='utf-8'))
        for n, row in enumerate(report['results']):
            for attempt in row['attempts']:
                folder = run/f"{n:02d}-{row['id']}-{attempt['mode'].lower()}"
                if (folder/'sim-attempt.json').exists():
                    cases.append(read_attempt(run, folder, report, row, attempt, coordinates))
    if not cases: raise SystemExit('NO_SIMULATION_ATTEMPTS_WITH_RECORDS')
    for run in reversed(a.runs):
        run = run.resolve(); combined = build_combined(run, json.loads((run/'simulation-report.json').read_text(encoding='utf-8')), coordinates)
        if combined is not None: cases.insert(0, combined)
    data = dict(model=model, cases=cases, physical_execution='NOT_RUN', executable=False, posture_comparison=False, retry=False)
    template = (ROOT/'tools/search_replay/field_failures.html').read_text()
    template = template.replace('两次现场规划失败 · 保存证据回看', a.title)
    for old, new in PATCHES+COMBINED_PATCHES:
        if template.count(old) != 1: raise ValueError(f'VIEWER_TEMPLATE_ANCHOR_NOT_UNIQUE ({template.count(old)}): '+old[:70])
        template = template.replace(old, new)
    template = template.replace('<title>两次现场规划失败', '<title>整面卸箱仿真')
    vendor = ROOT/'tools/search_replay/vendor'
    js = (vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    (a.output/'index.html').write_text(template.replace('/*__VENDOR__*/', js).replace(
        '/*__DATA__*/', json.dumps(data, separators=(',', ':'), ensure_ascii=False).replace('</', '<\\/')), encoding='utf-8')
    print(json.dumps(dict(cases=[dict(case=c['case_label'], candidates=len(c['candidates']),
                                      paths=sum(len(x['paths']) for x in c['candidates']),
                                      states=sum(len(x['states']) for x in c['candidates'])) for c in cases],
                          output=str(a.output/'index.html')), ensure_ascii=False))


if __name__ == '__main__':
    main()
