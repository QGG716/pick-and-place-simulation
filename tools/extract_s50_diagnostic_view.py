"""Extract actual collision records for the viewer; leave full evidence on server."""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path


def extract(paths):
    result = None
    sources = []
    seen = set()
    fields = ('schema', 'qualification_scope', 'model_definition_sha256',
        'environment_fingerprint', 'world_from_model_base', 'world_from_physical_mount',
        'initial_carton_ids', 'conveyor_height_m', 'obstacles')
    for path in paths:
        path = Path(path)
        raw = path.read_bytes()
        data = json.loads(raw)
        if (data.get('schema') != 's50_runtime_collision_diagnostics_v1'
                or data.get('qualification_scope') != 'CAD_DIAGNOSTIC_NOT_TASK'
                or data.get('status') != 'DIAGNOSTIC_QUERIES_COMPLETED_NOT_TASK_QUALIFIED'):
            raise ValueError('ONLY_COMPLETED_CAD_DIAGNOSTICS_CAN_BE_EXTRACTED')
        if result is None:
            result = deepcopy(data)
            result['poses'] = []
        elif any(data.get(key) != result.get(key) for key in fields):
            raise ValueError('DIAGNOSTIC_SOURCES_HAVE_DIFFERENT_SCENE_OR_MODEL')
        digest = hashlib.sha256(raw).hexdigest()
        sources.append(dict(path=str(path.resolve()), sha256=digest, bytes=len(raw),
            pose_count=len(data['poses']), pair_count=sum(p['queried_pair_count'] for p in data['poses'])))
        for pose in data['poses']:
            if pose['pose_id'] in seen:
                raise ValueError('DUPLICATE_DIAGNOSTIC_POSE')
            seen.add(pose['pose_id'])
            row = deepcopy(pose)
            pairs = pose['pairs']
            closest = sorted(range(len(pairs)), key=lambda i: (
                pairs[i].get('surface_distance_m') is None,
                pairs[i].get('surface_distance_m') or 0))[:40]
            keep = set(closest) | {i for i,pair in enumerate(pairs) if pair.get('intersects')}
            row['pairs'] = [deepcopy(pair) for i,pair in enumerate(pairs) if i in keep]
            row['pair_record_scope'] = 'ALL_INTERSECTIONS_AND_40_LOWEST_RAW_DISTANCES'
            row['retained_pair_count'] = len(row['pairs'])
            row['full_report_sha256'] = digest
            result['poses'].append(row)
    if result is None:
        raise ValueError('DIAGNOSTIC_SOURCE_REQUIRED')
    result['source_full_reports'] = sources
    result['record_scope'] = 'VIEWER_EXTRACT_NOT_COMPLETE_PAIR_EVIDENCE'
    result['total_original_pair_count'] = sum(row['pair_count'] for row in sources)
    result['total_retained_pair_count'] = sum(len(p['pairs']) for p in result['poses'])
    result.pop('elapsed_s', None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = extract(args.reports)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(output=str(args.output), poses=len(result['poses']),
        original_pairs=result['total_original_pair_count'], retained_pairs=result['total_retained_pair_count'],
        sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(), bytes=args.output.stat().st_size)))


if __name__ == '__main__':
    main()
