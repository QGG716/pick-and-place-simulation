"""Archive the stage-IK round without committing heavyweight task caches."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from tools.archive_m710id70_v3_baseline import (COMPARISON_FIELDS, compare_rows,
                                                read_csv, read_json,
                                                representative_task, write_csv,
                                                write_json)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _unique_task_rows(path: Path) -> list[dict[str,str]]:
    rows=read_csv(path)
    task_ids=[row['task_id'] for row in rows]
    if len(rows)!=104 or len(set(task_ids))!=104:
        raise ValueError(f'{path}: expected 104 unique task rows, found {len(rows)}/{len(set(task_ids))}')
    return rows


def _copy(source: Path, output: Path, name: str | None=None) -> None:
    shutil.copyfile(source,output/(name or source.name))


def archive(baseline: Path, after: Path, ablation: Path, output: Path) -> dict[str,Any]:
    before_path=baseline/'after_fixed_task_reachability.csv'
    after_path=after/'fixed_task_reachability.csv'
    before=_unique_task_rows(before_path);new=_unique_task_rows(after_path)
    comparison=compare_rows(before,new)
    if not all(row['task_id']==expected for row,expected in zip(
            comparison,sorted(row['task_id'] for row in before))):
        raise ValueError('comparison rows are not a deterministic task-id ordering')
    output.mkdir(parents=True,exist_ok=True)
    _copy(before_path,output,'before_fixed_task_reachability.csv')
    _copy(after_path,output,'after_fixed_task_reachability.csv')
    write_csv(output/'task_comparison.csv',comparison)
    for name in ('effective_config.json','source_manifest.json','fixed_geometric_recovery.json',
                 'fixed_search_statistics.json','fixed_initial_proximity_recovery.json',
                 'fixed_grasp_task_set_recovery.json'):
        _copy(after/name,output,name)
    _copy(baseline/'legacy_search_statistics.json',output,'before_search_statistics.json')
    _copy(ablation/'stage_ik_ablation.csv',output)
    _copy(ablation/'stage_ik_ablation.json',output)
    _copy(ablation/'source_manifest.json',output,'ablation_source_manifest.json')

    transitions=Counter(row['success_transition'] for row in comparison)
    changed=[row['task_id'] for row in comparison if not row['review_fields_equal']]
    failure_transitions=Counter(
        f"{row['before_failure_stage']}:{row['before_failure_reason']} -> "
        f"{row['after_failure_stage']}:{row['after_failure_reason']}"
        for row in comparison if (row['before_failure_stage'],row['before_failure_reason']) !=
                                 (row['after_failure_stage'],row['after_failure_reason']))
    successes=[row['task_id'] for row in comparison if row['after_GEOMETRICALLY_REACHABLE']=='True']
    nonfirst_connections=[]
    for path in sorted((after/'tasks').glob('grid_*_fixed.json')):
        result=read_json(path)['result'];task_id=path.stem.removesuffix('_fixed')
        for attempt_index,attempt in enumerate(result['attempts']):
            for option_index,option in enumerate(attempt.get('conveyor_attempts',[])):
                for name in ('pregrasp_search','handoff_search'):
                    search=option.get(name)
                    if not search:
                        continue
                    for connection in search['connection_attempts']:
                        if connection['connection_success'] and connection['candidate_index']>0:
                            nonfirst_connections.append({'task_id':task_id,'attempt_index':attempt_index,
                                'option_index':option_index,'stage_search':name,
                                'candidate_index':connection['candidate_index'],
                                'candidate_id':connection['candidate_id'],
                                'connection_attempts':len(search['connection_attempts']),
                                'shared_rrt_iterations_consumed':search['shared_rrt_iterations_consumed']})
    summary={'schema_version':'m710_stage_ik_round_comparison_v1','denominator':len(comparison),
        'unique_task_ids':len({row['task_id'] for row in comparison}),
        'success_transition_counts':dict(transitions),'before_success_count':sum(
            row['before_GEOMETRICALLY_REACHABLE']=='True' for row in comparison),
        'after_success_count':len(successes),'after_success_task_ids':successes,
        'new_success_task_ids':[row['task_id'] for row in comparison if row['success_transition']=='NEW_SUCCESS'],
        'lost_success_task_ids':[row['task_id'] for row in comparison if row['success_transition']=='LOST_SUCCESS'],
        'review_fields':list(COMPARISON_FIELDS),'changed_review_field_task_count':len(changed),
        'changed_review_field_task_ids':changed,'failure_stage_reason_transitions':dict(failure_transitions),
        'successful_nonfirst_stage_connections':nonfirst_connections,
        'interpretation':'Task-level counts use unique original task IDs; candidate attempts are reported separately.'}
    write_json(output/'comparison_summary.json',summary)

    if successes:
        witness=successes[0]
        write_json(output/f'representative_success_{witness}.json',
                   representative_task(after/'tasks'/f'{witness}_fixed.json'))
    example=read_json(after/'tasks/grid_020_fixed.json')['result']['validation_evidence']
    ablation_data=read_json(ablation/'stage_ik_ablation.json')
    ablation_code_digests=sorted({read_json(path)['result']['validation_evidence']['code_digest']
        for mode in ('legacy_single','filtered_single','multi_solution')
        for path in (ablation/mode/'tasks').glob('*.json')})
    identity={'post_calculation_identity':example,'ablation_code_digests':ablation_code_digests,
        'post_and_ablation_calculation_code_match':ablation_code_digests==[example['code_digest']],
        'ablation_modes':ablation_data['modes'],
        'note':'Ablation ran before the implementation commit; its captured content digest matches the committed post-scan calculation code.'}
    write_json(output/'calculation_identity_reconciliation.json',identity)

    manifest={'schema_version':'m710_stage_ik_round_archive_v1',
        'commit_at_archive':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT).decode().strip(),
        'branch':subprocess.check_output(['git','branch','--show-current'],cwd=ROOT).decode().strip(),
        'baseline_source':str(baseline),'after_source':str(after),'ablation_source':str(ablation),
        'task_set':'fixed conveyor original 104 valid grid tasks',
        'commands':{
            'post':'.venv\\Scripts\\python.exe tools\\run_m710id70_v3.py --phase grid-fixed --workers 12 --output-dir .tmp\\review_stage_ik_post',
            'ablation':'.venv\\Scripts\\python.exe tools\\run_m710id70_v3_stage_ablation.py --output-dir .tmp\\review_stage_ik_ablation --task-indices 22 45',
            'archive':'.venv\\Scripts\\python.exe tools\\archive_m710id70_v3_stage_ik_round.py --baseline-dir docs\\validation\\evidence\\m710id70_v3_hardening_baseline --after-dir .tmp\\review_stage_ik_post --ablation-dir .tmp\\review_stage_ik_ablation --output-dir docs\\validation\\evidence\\m710id70_v3_stage_ik_round'},
        'checks':{'before_rows':len(before),'after_rows':len(new),'comparison_rows':len(comparison),
                  'unique_task_ids':len({row['task_id'] for row in comparison}),
                  'task_id_sets_equal':{row['task_id'] for row in before}=={row['task_id'] for row in new}},
        'files':{}}
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name!='archive_manifest.json':
            manifest['files'][path.name]={'sha256':sha256_file(path),'bytes':path.stat().st_size}
    write_json(output/'archive_manifest.json',manifest)
    return summary


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-dir',type=Path,required=True)
    parser.add_argument('--after-dir',type=Path,required=True)
    parser.add_argument('--ablation-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    summary=archive(args.baseline_dir.resolve(),args.after_dir.resolve(),args.ablation_dir.resolve(),
                    args.output_dir.resolve())
    print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':
    main()
