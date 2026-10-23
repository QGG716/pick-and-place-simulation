"""Export retained min250 failures using the existing model/renderer, without solving.

The raw archive is verified before reading. Each case keeps its own recorded
world and robot q. Impossible receiver geometry is a labelled height schematic,
never an invented mechanism, collision witness, or robot goal.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def export(raw_path, replay_path, output):
    with tarfile.open(raw_path) as archive:
        manifest = json.load(archive.extractfile('file-sha256.json'))
        for name, expected in manifest.items():
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == expected, name

        def read(name):
            return json.load(archive.extractfile('formal-v2/' + name))

        summary = read('summary.json')
        replay = json.JSONDecoder().raw_decode(replay_path.read_text(encoding='utf-8').rsplit('const D=', 1)[1])[0]
        assert summary['source_commit'] == replay['source_commit']
        assert len(summary['terminal']['world_cartons']) == 12
        assert replay['segments'][-1]['q'] == summary['terminal']['q_rad']
        cases, attempts = [], []

        def add(target, root, kind, options, blockers):
            q = root['q_rad']
            matches = [i for i, s in enumerate(replay['segments']) if s.get('q') == q]
            assert matches, 'RECORDED_Q_HAS_NO_VERIFIED_REPLAY_FK'
            height_key = float(root['height_m']).hex()
            world = copy.deepcopy(replay['worlds'][height_key] + root['world_cartons'])
            case = dict(target_id=target, kind=kind, options=options, blockers=blockers,
                        display_q=q, height_m=root['height_m'], world_carton_count=len(root['world_cartons']),
                        target_box=next(b for b in root['world_cartons'] if b['id'] == target),
                        production_flange_column_major=replay['fk_witnesses'][matches[-1]],
                        state_label='本箱筛选时的实际根状态' if kind == 'geometry' else '本轮结束时的实际剩余状态',
                        native_search_performed=False, collision_witness=None)
            cases.append(case)
            attempts.append(dict(request=dict(stage='inspection', q_start=q, q_goal=None, world=world,
                                 attachment=None, process_policy=dict(target_id=target)),
                                 response_identity=dict(status='INSPECTION_SNAPSHOT', request_id='NOT_A_SOLVER_REQUEST'),
                                 final_timed_path=[], trace=None, external_axis=None))

        for outcome in summary['outcomes']:
            if outcome['completed_count']:
                continue
            task = outcome['evidence']['archive'].removesuffix('.tar.gz')
            result = read(task + '/result.json')
            options = []
            for row in result['attempts']:
                assert not row['complete']
                option = row['option']
                entry = dict(template=option['template'], receive_height_m=option['height_m'],
                             temporary_height_m=option.get('H_temporary_m'),
                             failure=row['failure'], geometry_evidence=row.get('geometry_evidence'),
                             screening=row.get('screening'), wall_s=row['wall_s'],
                             source_file=task + '/' + (row.get('folder') or row.get('evidence_folder', '')) + '/summary.json')
                if row.get('folder'):
                    calls = read(task + '/' + row['folder'] + '/requests.json')
                    assert not any(c['op'] in ('ik', 'plan') for c in calls)
                    entry['worker_calls'] = [c['op'] for c in calls]
                options.append(entry)
            add(outcome['target_id'], result['initial'], 'geometry', options, [])

        for blocked in summary['blocked']:
            assert blocked['native_search_performed'] is False
            add(blocked['target_id'], summary['terminal'], 'support', [], blocked['supported_cartons'])

    assert len(cases) == 12 and len({c['target_id'] for c in cases}) == 12
    data = dict(label='250 mm 续跑失败检查', model=replay['model'], inspection_cases=cases,
                raw=dict(source_commit=summary['source_commit'], attempts=attempts),
                result=dict(complete=False, target_id='12 个剩余箱'), request_log=[],
                qualification_status='NOT_EVALUATED', continuous_swept_proof=False,
                physical_execution='NOT_RUN', execution_ready=False,
                geometry_schematic=dict(requested_surface_m=.25, body_thickness_m=.12,
                    top_pin_offset_m=.07, lower_pin_world_z_m=.10, requested_upper_pin_world_z_m=.06,
                    stages=2, requested_stage_rise_m=-.02, minimum_stage_rise_open_m=.035,
                    necessary_surface_lower_bound_open_m=.36,
                    meaning='DERIVED_HEIGHT_SCHEMATIC_ONLY_NOT_CONSTRUCTED_OR_EXECUTED'),
                source_archive_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
                accepted_replay_sha256=hashlib.sha256(replay_path.read_bytes()).hexdigest())
    template = (ROOT / 'tools/search_replay/viewer.html').read_text(encoding='utf-8')
    vendor = ROOT / 'tools/search_replay/vendor'
    template = template.replace('/*__VENDOR__*/', '\n'.join((vendor / n).read_text(encoding='utf-8') for n in ['three.min.js', 'OrbitControls.js']))
    template = template.replace('/*__DATA__*/', json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False).replace('</', '<\\/'))
    overlay = (ROOT / 'tools/search_replay/min250_failures.js').read_text(encoding='utf-8')
    template = template.replace('</html>', '<script>\n' + overlay + '\n</script></html>')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(template, encoding='utf-8', newline='\n')
    checks = dict(status='PASS', source_commit=summary['source_commit'], source_archive_sha256=data['source_archive_sha256'],
                  accepted_replay_sha256=data['accepted_replay_sha256'], verified_raw_files=len(manifest),
                  cases=len(cases), direct_failures=5, support_blocked=7, solver_calls_added=0,
                  generated_robot_motion_points=0, collision_witnesses=0, physical_execution='NOT_RUN')
    output.with_name(output.stem + '-checks.json').write_text(json.dumps(checks, indent=2) + '\n')
    output.with_name(output.stem + '-cases.json').write_text(json.dumps(cases, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(checks))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw', type=Path)
    parser.add_argument('replay', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    export(args.raw, args.replay, args.output)
