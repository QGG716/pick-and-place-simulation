"""Compare archived field IK branches without replanning or hardware access.

The directional score is a diagnostic, not a new planner selection policy.
Optional native checks are static collision queries, never path acceptance.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from visualize_field_planning_failures import plt
from unloading_sim.moveit2_backend import ResidentMoveItClient
from unloading_sim.workbench.single_carton.config import load, wrist_branch_violation
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA, load_mapping


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def posture(coordinates, q):
    frames = coordinates.frames(q)
    points = np.array([frames[name][:3, 3] for name in ['base_link'] + [f'link_{i}' for i in range(1, 7)]])
    upper = points[3] - points[2]
    forearm = points[4] - points[3]
    up = float(np.rad2deg(np.arccos(np.clip(upper[2] / np.linalg.norm(upper), -1, 1))))
    horizontal = float(np.rad2deg(np.arcsin(np.clip(abs(forearm[2]) / np.linalg.norm(forearm), 0, 1))))
    return dict(upper_from_up_deg=up, forearm_from_horizontal_deg=horizontal,
                diagnostic_angle_sum_deg=up + horizontal, joint_origins_m=points.tolist(),
                tcp_pose=coordinates.tcp(q).tolist(), q=list(q), q_deg=np.rad2deg(q).tolist())


def compare(record, coordinates, limits, sources):
    def read(path):
        sources[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
        return json.loads(path.read_text())

    obs = read(record/'observation.json')
    box = read(record/'geometry.json')['boxes'][0]
    failures = read(record/'planning-failures.json')
    q0 = obs['q_start']
    rows = []
    poses = []
    for path in sorted(record.glob('*/approach-endpoint-ik-native.json')):
        ik = read(path)
        pose = np.asarray(ik['request']['goal_pose'])
        poses.append(pose)
        distinct = []
        accepted = []
        for reply in ik['replies']:
            if reply['status'] != 'SUCCESS':
                continue
            q = np.asarray(reply['q'])
            if any(np.max(abs(q-other)) < 1e-3 for other in distinct):
                continue
            distinct.append(q)
            result = posture(coordinates, q)
            actual = np.asarray(result['tcp_pose'])
            position_error = float(np.linalg.norm(actual[:3, 3] - pose[:3, 3]))
            rotation_error = float(np.arccos(np.clip((np.trace(actual[:3, :3].T @ pose[:3, :3])-1)/2, -1, 1)))
            assert position_error <= 2e-5 and rotation_error <= 2e-4, 'ARCHIVED_IK_FK_MISMATCH'
            result.update(candidate=int(path.parent.name.rsplit('-', 1)[1]), source=str(path.relative_to(ROOT)),
                          position_error_m=position_error, orientation_error_rad=rotation_error,
                          wrist_rejection=wrist_branch_violation(limits, q0, [q]))
            if result['wrist_rejection'] is None:
                accepted.append(q)
            rows.append(result)
        saved = ik['distinct_solutions']
        assert len(accepted) == len(saved) and all(any(np.max(abs(q-other)) < 1e-3 for other in saved)
                                                  for q in accepted), 'WRIST_POLICY_DIFFERS_FROM_ARCHIVE'
    # Compare alternatives for the exact candidate shown by the failure viewer.
    shown_index = next(f['candidate'] for f in sorted(failures, key=lambda f: f['candidate'])
                       if f['reason'].startswith('ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED'))
    folder = next(record.glob(f'*-{shown_index}'))
    endpoint = read(folder/'approach-endpoint-native.json')
    shown = posture(coordinates, endpoint['request']['q_start'])
    shown.update(candidate=shown_index, wrist_rejection=wrist_branch_violation(limits, q0, [shown['q']]),
                 archived_collision_failure=endpoint['result'].get('failure'))
    same = [r for r in rows if r['candidate'] == shown_index]
    # Near-identical geometric branches can have different joint windings.
    key = lambda r: (round(r['diagnostic_angle_sum_deg'], 2), float(np.linalg.norm(np.array(r['q'])-q0)))
    preferred = min(same, key=key)
    same_wrist = min((r for r in same if r['q'][4]*q0[4] > 0), key=key)
    faces = [{k: f[k] for k in ('center_m', 'normal', 'size_m', 'observed')} for f in box['faces']]
    allowed = [r for r in rows if r['wrist_rejection'] is None]
    detach_outputs = []
    for path in sorted(record.glob('*/*native.json')):
        saved = json.loads(path.read_text())
        reply = saved.get('result', {})
        if saved.get('request', {}).get('stage') != 'detach' or not isinstance(reply, dict) or not reply.get('points'):
            continue
        read(path)  # Bind the original output used by the directional diagnosis.
        value = posture(coordinates, reply['points'][-1]['q'])
        detach_outputs.append(dict(source=str(path.relative_to(ROOT)),
                                   upper_from_up_deg=value['upper_from_up_deg'],
                                   forearm_from_horizontal_deg=value['forearm_from_horizontal_deg']))
    result = dict(task_id=record.name, q_start=posture(coordinates, q0), faces=faces,
                  candidate_count=len(failures), distinct_ik_per_pose_total=len(rows),
                  wrist_allowed_count=len(allowed),
                  wrist_filter_reproduces_archive=True,
                  wrist_rejection_counts=dict(Counter(r['wrist_rejection']['reason'] for r in rows if r['wrist_rejection'])),
                  precontact_tcp_z_range_m=[float(min(p[2, 3] for p in poses)), float(max(p[2, 3] for p in poses))],
                  best_raw=min(rows, key=key), best_wrist_allowed=min(allowed, key=key) if allowed else None,
                  shown=shown, same_pose_preferred=preferred, same_pose_positive_j5=same_wrist,
                  detach_outputs=detach_outputs,
                  detach_output_note='Includes aliases/retries, not distinct complete paths; final saved samples only; no sent-path audit.',
                  all_ik=rows)
    return result, endpoint['request']


def draw(cases, output):
    fig = plt.figure(figsize=(16, 9), dpi=150, facecolor='#f5f7fa')
    fig.suptitle('同一接近点的不同逆解：高度没变，肘部与腕部分支不同', fontsize=19, y=.98)
    titles = ['实测起始姿态', '查看器显示的被拒绝终态', '更接近偏好，但 J5 翻侧', '保持 J5 同侧，但 J4 超限']
    colors = ['#3a8966', '#db694d', '#4b85c4', '#8c72b5']
    for i, case in enumerate(cases):
        for j, key in enumerate(('q_start', 'shown', 'same_pose_preferred', 'same_pose_positive_j5')):
            value = case[key]
            ax = fig.add_subplot(len(cases), 4, i*4+j+1, projection='3d')
            points = np.array(value['joint_origins_m'])
            tcp = np.asarray(value['tcp_pose'])[:3, 3]
            ax.plot(*points.T, color='#a9b2bc', lw=3, marker='o', markersize=4)
            ax.plot(*points[2:4].T, color=colors[j], lw=7)
            ax.plot(*points[3:5].T, color=colors[j], lw=5)
            ax.plot(*np.array([points[-1], tcp]).T, color='#334455', lw=3)
            ax.scatter(*tcp, color='#dca630', s=38, marker='s')
            # Explicit base vertical and horizontal references at the shoulder.
            ref = points[2]
            ax.plot([ref[0]]*2, [ref[1]]*2, [ref[2], ref[2]+.30], ls='--', color='#768391', lw=1)
            ax.plot([ref[0]-.40, ref[0]], [ref[1]]*2, [ref[2]+.26]*2, ls='--', color='#768391', lw=1)
            ax.set(xlim=(-.65, .10), ylim=(-.55, .18), zlim=(0, .72))
            ax.set_box_aspect((.75, .73, .72)); ax.view_init(18, -115)
            ax.set_axis_off()
            ax.set_title(f'箱位 {i+1} · {titles[j]}', fontsize=11, color=colors[j], pad=0)
            ax.text2D(.01, -.01, f'大臂偏离向上 {value["upper_from_up_deg"]:.1f}°\n小臂偏离水平 {value["forearm_from_horizontal_deg"]:.1f}°',
                      transform=ax.transAxes, fontsize=11)
            if j:
                ax.text2D(.01, -.16, f'同一 TCP：z = {tcp[2]*1000:.1f} mm\nJ4 {value["q_deg"][3]:.1f}°  /  J5 {value["q_deg"][4]:.1f}°',
                          transform=ax.transAxes, fontsize=10, color='#4d5d6d')
    fig.subplots_adjust(left=.01, right=.99, bottom=.16, top=.91, hspace=.42, wspace=.02)
    fig.text(.02, .045, '基于原记录逆解与 URDF 正运动学的关节骨架示意。偏好角度是诊断量，不是关节硬指令。\n'
             '后三列为同一候选、同一 TCP 位置和方向；这些姿态均不是可执行方案。静态碰撞检查结果见 report.json，未验证整条路径。', fontsize=11)
    fig.savefig(output, facecolor=fig.get_facecolor()); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', nargs=2, type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--validate', action='store_true', help='offline exact static collision queries only')
    args = parser.parse_args(); out = args.output.resolve(); out.mkdir(parents=True, exist_ok=True)
    config_path = ROOT/'configs/workbench/single_carton.example.json'
    config = load(config_path)
    installation = Installation.load(config['execution']['installation'])
    coordinates = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    assert np.allclose(installation.world_from_base, np.eye(4)), 'BASE_FRAME_REQUIRED_FOR_DIRECTION_METRICS'
    sources = {}; cases = []; requests = []
    for record in args.records:
        case, request = compare(record.resolve(), coordinates, config['wrist_branch_limits'], sources)
        cases.append(case); requests.append(request)
    report = dict(scope='ARCHIVED_IK_POSTURE_DIAGNOSIS_NOT_A_NEW_PLAN', hardware_execution='NOT_RUN',
                  executable=False, planner_selection_changed=False, native_static_checks='NOT_RUN',
                  source_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                  wrist_limits=config['wrist_branch_limits'],
                  metric_basis={'upper': 'link_2 origin to link_3 origin, angle to base +Z',
                                'forearm': 'link_3 origin to link_4 origin, angle to base XY plane',
                                'scope': 'geometric directional diagnostic, not CAD centreline certification or planner cost'},
                  cases=cases, sources=sources)
    report['fk_validation'] = dict(
        checked=sum(len(c['all_ik']) for c in cases),
        max_position_error_m=max(r['position_error_m'] for c in cases for r in c['all_ik']),
        max_orientation_error_rad=max(r['orientation_error_rad'] for c in cases for r in c['all_ik']))
    if args.validate:
        planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), coordinates)
        planner.build()
        client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=out/'native.log')
        try:
            for i, (case, request) in enumerate(zip(cases, requests), 1):
                assert request['identity'] == planner.init['identity'], 'MODEL_IDENTITY_CHANGED'
                client.request(planner.init)
                for name in ('shown', 'same_pose_preferred', 'same_pose_positive_j5'):
                    query = deepcopy(request); query['q_start'] = case[name]['q']
                    assert query['op'] == 'validate'
                    path = out/f'case-{i}-{name}-static.json'
                    write(path, dict(request=query, status='PENDING', hardware_execution='NOT_RUN'))
                    reply = client.request(query, timeout=120)
                    write(path, dict(request=query, result=reply, status='REPLIED', hardware_execution='NOT_RUN'))
                    case[name]['static_collision'] = {k: reply.get(k) for k in ('status', 'native_valid', 'legacy_intersection_valid', 'failure')}
                    case[name]['static_collision']['evidence'] = str(path.relative_to(ROOT))
            report['native_static_checks'] = 'COMPLETED_NOT_PATH_ACCEPTANCE'
        finally:
            client.close()
    draw(cases, out/'posture-comparison.png')
    for name, digest in sources.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest, 'ARCHIVED_EVIDENCE_CHANGED'
    report['archived_evidence_unchanged'] = True
    write(out/'report.json', report)
    print(json.dumps({'output': str(out), 'cases': len(cases), 'fk_checks': sum(len(c['all_ik']) for c in cases),
                      'static_checks': report['native_static_checks'], 'hardware_execution': 'NOT_RUN'}))


if __name__ == '__main__':
    main()
