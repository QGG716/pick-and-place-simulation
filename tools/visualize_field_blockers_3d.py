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
    for failure in sorted((x for x in failures if 'candidate' in x), key=lambda x: x['candidate']):
        folder = record / f"{target}-{failure['candidate']}"
        states = [dict(q=r['q'], stage='start', failure=None, label=r['label']) for r in references]
        for pre in sorted(folder.glob('endpoint-precheck*.json')):
            for check in json.loads(pre.read_text()):
                if check.get('q') is None:
                    continue
                alt = pre.stem.replace('endpoint-precheck', '').strip('-')
                states.append(dict(q=check['q'], stage=check['stage'], failure=check.get('failure'),
                                   label=f"{check['stage']} 终态{(' ' + alt) if alt else ''}"))
        paths = []
        for stage in STAGES:
            for f in sorted(folder.glob(f'{stage}*-native.json')):
                if any(t in f.name for t in ('endpoint', 'sent-p', '-ik')):
                    continue
                p = path_entry(f, coordinates, stage)
                if p:
                    paths.append(p)
                    attachment = attachment or p['attachment']
        for chk in sorted(folder.glob('*-sent-check.json')):
            c = json.loads(chk.read_text()); fail = c.get('path_failure') or c.get('failure')
            if fail and fail.get('q_rad'):
                stage = chk.name.replace('-sent-check.json', '')
                sd = fail.get('surface_distance_m', -1)
                states.append(dict(q=fail['q_rad'], stage=stage.split('-')[0], label=f'{stage} 路径审计拒绝点',
                                   failure=dict(fail, surface_distance_m=sd)))
        candidates.append(dict(index=failure['candidate'], reason=short(failure['reason']), kind=kind(failure['reason']),
                               states=states, paths=paths, contact=None, preferred_ik=None))
    world = scene['world'] + [dict(id=b['id'], size=b['size_m'], pose=b['pose'], category='carton')
                              for b in geo['boxes'] if b['id'] != box['id'] and not any(w['id'] == b['id'] for w in scene['world'])]
    life = record / 'planning-task-lifecycle.json'
    lifecycle = json.loads(life.read_text()) if life.exists() else {}
    return dict(task_id=record.name, observation_id=obs['observation_id'], q_start=obs['q_start'], q_capture=obs['q_capture'],
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
