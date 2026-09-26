"""Read-only, exhaustive cost review; never converts strict failure to success.

Only returned SO(3) solver cost is classified separately. All other fields,
including solver status and boundary_fit, remain protected exact values.
Classification is not a numerical explanation or an execution authorization.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct

from compare_metric_faces_ab import compare, read
from unloading_contracts import loads, to_wire

OBJECTIVE = 'ROBUST_METRIC_POINT_TO_SHARED_SO3_PLANES'
# Defensive manual-review guard, NOT a solver threshold or an allowed difference.
# Values below it still require real solver evidence; no tolerance is accepted.
MANUAL_REVIEW_COST_CEILING = 1e6


def diagnostic_path(path):
    return bool(re.fullmatch(r'[^/]+/(?:final/instances/\d+|observation/cargo/\d+/raw_result/record)/metric_solver/cost', path))


def numeric_difference(a, b):
    result = {'before': a, 'after': b, 'before_type': type(a).__name__, 'after_type': type(b).__name__}
    if type(a) is float and type(b) is float and math.isfinite(a) and math.isfinite(b):
        result.update(absolute_difference=abs(a-b), relative_difference_to_before=abs(a-b)/abs(a) if a else None,
                      before_hex=a.hex(), after_hex=b.hex())
        def ordered(x):
            bits = struct.unpack('>Q', struct.pack('>d', x))[0]
            return (~bits & ((1 << 64)-1)) if bits >> 63 else bits | (1 << 63)
        result['ulp_distance'] = abs(ordered(a)-ordered(b))
    return result


class ExhaustiveDifferences:
    def __init__(self, detail_limit=40):
        self.detail_limit = detail_limit
        self.count = self.protected_count = self.visited = 0
        self.details, self.diagnostics = [], []

    def __call__(self, a, b, path, strict_details):
        self.visited += 1
        if type(a) is not type(b):
            self.add(a, b, path, strict_details); return
        if isinstance(a, dict):
            for key in sorted(a.keys() | b.keys()):
                if key not in a or key not in b:
                    self.add(a.get(key, '<MISSING>'), b.get(key, '<MISSING>'), path+'/'+key, strict_details)
                else:
                    self(a[key], b[key], path+'/'+key, strict_details)
        elif isinstance(a, list):
            for i in range(max(len(a), len(b))):
                if i >= len(a) or i >= len(b):
                    self.add(a[i] if i < len(a) else '<MISSING>', b[i] if i < len(b) else '<MISSING>', path+'/'+str(i), strict_details)
                else:
                    self(a[i], b[i], path+'/'+str(i), strict_details)
        elif a != b or (isinstance(a, float) and not math.isfinite(a)):
            self.add(a, b, path, strict_details)

    def add(self, a, b, path, strict_details):
        self.count += 1
        detail = {'path': path, **numeric_difference(a, b)}
        if diagnostic_path(path):
            self.diagnostics.append(detail)  # ALL diagnostic values, never truncated
        else:
            self.protected_count += 1
        if len(self.details) < self.detail_limit: self.details.append(detail)
        if len(strict_details) < 40: strict_details.append(path + ': exact difference')


def solver_cost(record, path):
    solver = record.get('metric_solver')
    if solver is None:
        if record.get('camera_facing_faces'): raise ValueError('MISSING_SOLVER: '+path)
        return None
    if solver.get('reason') == 'INCOMPATIBLE_INSTANCE_PLANE_NORMALS' and solver.get('success') is False:
        if 'cost' in solver: raise ValueError('UNDECLARED_DIAGNOSTIC_COST: '+path)
        return None
    if record.get('geometry_version') != 'DEPTH_METRIC_PATCHES_V1' or solver.get('objective') != OBJECTIVE:
        raise ValueError('UNDECLARED_SOLVER_SCHEMA: '+path)
    cost = solver.get('cost')
    if type(cost) is not float or not math.isfinite(cost) or not 0 <= cost < MANUAL_REVIEW_COST_CEILING:
        raise ValueError('INVALID_OR_MANUAL_REVIEW_COST: '+path)
    return cost


def check_copies(root, plan):
    for module in plan['modules']:
        name = module['module_id']
        records = read(root/name/'rgbd_cuboids.json')['instances']
        cargo = to_wire(loads((root/name/'mode_b_rgbd_observation.json').read_text(encoding='utf-8')))['cargo']
        by_id = {c['raw_result']['instance_lineage']['mask_id']: c for c in cargo}
        for record in records:
            path = name+'/'+str(record['mask_id'])
            cost = solver_cost(record, path)
            raw = by_id[record['mask_id']]['raw_result']
            copy = raw.get('record')
            if copy is not None:
                if solver_cost(copy, path+'/observation') != cost:
                    raise ValueError('DIAGNOSTIC_COPY_MISMATCH: '+path)
            elif cost is not None and 'hypotheses' not in raw:
                raise ValueError('DIAGNOSTIC_COPY_MISSING: '+path)
            for surface_list in (record['camera_facing_faces'], by_id[record['mask_id']].get('observed_surfaces', [])):
                for identity in ('face_id', 'support_label'):
                    ids = [s[identity] for s in surface_list if identity in s]
                    if len(ids) != len(set(ids)): raise ValueError('DUPLICATE_FACE_ID: '+path)


def review(before, after, plan_path, *, detail_limit=40):
    collector = ExhaustiveDifferences(detail_limit)
    strict = compare(before, after, plan_path, mode='blas-threads', difference_collector=collector)
    for root in (Path(before), Path(after)): check_copies(root, read(plan_path))
    return {'plan_complete': True, 'records_exact_equal': strict['results_equal'],
        'protected_geometry_exact_equal': collector.protected_count == 0,
        'decisions_and_support_exact_equal': collector.protected_count == 0,
        'difference_count': collector.count, 'protected_difference_count': collector.protected_count,
        'visited_value_pairs': collector.visited, 'detail_limit': detail_limit,
        'difference_details': collector.details, 'diagnostic_differences': collector.diagnostics,
        'diagnostic_review_status': 'NO_DIFFERENCES' if not collector.count else 'UNEXPLAINED_REQUIRES_REAL_SOLVER_EVIDENCE',
        'modules': strict['modules'], 'plan_sha256': strict['plan_sha256'],
        'scope_and_limitations': [__doc__, 'Protected equality conservatively includes every unclassified field.',
            'Returned cost classification does not describe the internal optimization objective.',
            'No ULP tolerance; different content keeps different hashes and authorization identities.',
            'Historical ORACLE-PROMPTED input remains display-only; no execution authorization.']}


def verify_evidence(archive_root, manifest_path, plan_path, code_root):
    def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = read(manifest_path)
    if not manifest['files']: raise ValueError('EMPTY_EVIDENCE_MANIFEST')
    for name, ref in manifest['files'].items():
        path = (archive_root/name).resolve()
        if not path.is_relative_to(archive_root.resolve()): raise ValueError('EVIDENCE_OUTSIDE_ARCHIVE')
        if path.stat().st_size != ref['bytes'] or digest(path) != ref['sha256']:
            raise ValueError('EVIDENCE_HASH_MISMATCH: '+name)
    plan = read(plan_path)
    from compare_depth_preprocessing import require_roster
    require_roster(plan['modules'], 'module_id', ['module_0_upper', 'module_1_lower'], 'fixed capture modules')
    for module, count in zip(plan['modules'], (16, 31)):
        if module['mask_ids'] != list(range(1, count+1)) or module['mask_array_shape'] != [count, 1944, 2592]:
            raise ValueError('FIXED_CAPTURE_ROSTER_OR_RESOLUTION_CHANGED')
    if not plan.get('input_sha256'): raise ValueError('MISSING_INPUT_IDENTITIES')
    for name, expected in plan['input_sha256'].items():
        if digest(Path(name)) != expected: raise ValueError('INPUT_HASH_MISMATCH: '+name)
    if digest(Path(plan['vision_root'])/'pipeline/geometry/recover_box_cuboids_3d.py') != plan['extractor_file_sha256']:
        raise ValueError('UPSTREAM_HASH_MISMATCH')
    for run in ('A-inherit', 'B-blas-1'):
        for filename, key, prefix in [('ab_summary.json','code_sha256',''),
            ('function_timings.json','production_sha256',''), ('thread_policy_report.json','entry_code_sha256','tools')]:
            for name, expected in read(archive_root/run/filename)[key].items():
                if digest(code_root/prefix/name) != expected: raise ValueError('CODE_HASH_MISMATCH: '+name)
    return {'manifest_sha256': digest(manifest_path), 'verified_files': len(manifest['files']),
            'verified_inputs': len(plan['input_sha256']), 'code_verified': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('archive-root', 'evidence-manifest', 'plan', 'code-root', 'output'):
        parser.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        identities = verify_evidence(args.archive_root, args.evidence_manifest, args.plan, args.code_root)
        result = review(args.archive_root/'A-inherit', args.archive_root/'B-blas-1', args.plan)
        result['evidence_verification'] = identities
    except Exception as exc:
        result = {'plan_complete': False, 'records_exact_equal': False,
            'protected_geometry_exact_equal': None, 'decisions_and_support_exact_equal': None,
            'diagnostic_review_status': 'FAILED', 'error': str(exc), 'error_type': type(exc).__name__}
    with args.output.open('x', encoding='utf-8') as out:
        json.dump(result, out, indent=2, sort_keys=True, allow_nan=False)
    # This report does not create a relaxed acceptance mode.
    return 0 if result['plan_complete'] and result['records_exact_equal'] else 1


if __name__ == '__main__': raise SystemExit(main())
