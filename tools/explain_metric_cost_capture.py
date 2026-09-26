"""Validate the bounded mask-9 capture pair; explain, never relax strict equality."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metric_thread_policy import policy, verify_blas, verify_non_targets
from reproduce_metric_cost import residual_reference


def explain(review_path, before, after):
    import numpy as np
    def read(p): return json.loads(p.read_text(encoding='utf-8'))
    def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
    review = read(review_path)
    if not (review['plan_complete'] and review['protected_geometry_exact_equal'] and review['decisions_and_support_exact_equal']):
        raise ValueError('PROTECTED_OR_INCOMPLETE_HISTORICAL_RESULT')
    reports, arrays, references = [], [], []
    for root, threads, historical in ((before,None,'A-inherit'), (after,1,'B-blas-1')):
        p = root/'solver_capture.json'; r = read(p)
        if r['status'] != 'COMPLETED' or r['policy'] != policy(threads): raise ValueError('INVALID_CAPTURE_STATUS_OR_POLICY')
        if r['plan_sha256'] != review['plan_sha256'] or r['scope'] != 'module_1_lower/mask_id=9 ONLY':
            raise ValueError('CAPTURE_SCOPE_MISMATCH')
        if not r['historical_record_exact_matches'][historical] or not r['independent_validation_unchanged']:
            raise ValueError('HISTORICAL_RECORD_NOT_REPRODUCED')
        if r['raw_image_automatic'] is not False or r['planning_admissible'] is not False:
            raise ValueError('INVALID_CAPTURE_GATES')
        for phase in ('before_geometry','after_geometry'):
            verify_blas(r[phase]['blas'], threads)
            verify_non_targets(r['before_policy'],r[phase])
            if r[phase]['pid'] != r['before_policy']['pid']: raise ValueError('CAPTURE_PID_MISMATCH')
        if threads is None and (r['applied'] or r['before_policy']['blas'] != r['after_geometry']['blas']):
            raise ValueError('INHERITED_THREADS_CHANGED')
        npz = root/'solver_arrays.npz'
        if sha(npz) != r['solver_arrays_sha256']: raise ValueError('SOLVER_ARRAY_HASH_MISMATCH')
        with np.load(npz, allow_pickle=False) as data: values = {k:data[k] for k in data.files}
        if set(values) != {'initial','x','fun','jac','grad'}: raise ValueError('MISSING_SOLVER_ARRAY')
        if any(not np.isfinite(v).all() for v in values.values()): raise ValueError('NONFINITE_SOLVER_ARRAY')
        s = r['solver']
        if s['loss'] != 'linear' or s['method'] != 'trf' or not s['success']: raise ValueError('UNREVIEWED_SOLVER')
        if not 0 <= s['cost'] < 1e6 or s['cost'] != s['dot_cost']: raise ValueError('COST_NOT_REPRODUCED_BY_DOT')
        if s['residual_reference'] != residual_reference(values['fun']): raise ValueError('RESIDUAL_REFERENCE_MISMATCH')
        reports.append(r); arrays.append(values)
        references.append({'report_sha256':sha(p), 'arrays_sha256':sha(npz)})
    for key in ('inputs','loss','method','explicit_kwargs','status','nfev','njev','optimality','message','residual_reference'):
        if reports[0]['solver'][key] != reports[1]['solver'][key]: raise ValueError('SOLVER_INPUT_OR_RESULT_CHANGED: '+key)
    for key in ('input_binding','capture_id','T_W_C_at_capture','production_sha256','scipy_trf_source'):
        if reports[0][key] != reports[1][key]: raise ValueError('REPRODUCTION_BINDING_CHANGED: '+key)
    for key in ('python','numpy','scipy','opencv','blas','opencv_threads','thread_environment','affinity'):
        if reports[0]['before_policy'][key] != reports[1]['before_policy'][key]: raise ValueError('INHERITED_ENVIRONMENT_CHANGED: '+key)
    comparisons = {}
    for k in arrays[0]:
        a,b = arrays[0][k], arrays[1][k]
        exact = a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()
        comparisons[k] = {'shape':list(a.shape), 'dtype':str(a.dtype), 'bitwise_equal':exact}
        if not exact: raise ValueError('FINAL_ARRAY_CHANGED: '+k)
    module = next(m for m in review['modules'] if m['module_id']=='module_1_lower')
    ordinal = next(i for i,r in enumerate(module['instances']) if r['mask_id']==9)
    expected = {f'module_1_lower/final/instances/{ordinal}/metric_solver/cost',
                f'module_1_lower/observation/cargo/{ordinal}/raw_result/record/metric_solver/cost'}
    if review['difference_count'] != 2 or {d['path'] for d in review['diagnostic_differences']} != expected:
        raise ValueError('NEW_UNEXPLAINED_DIFFERENCES')
    costs = [r['solver']['cost'] for r in reports]
    if any([d['before'],d['after']] != costs for d in review['diagnostic_differences']):
        raise ValueError('HISTORICAL_COST_NOT_REPRODUCED')
    return {**review, 'diagnostic_review_status':'EXPLAINED_SAME_FINAL_RESIDUAL_DOT_REDUCTION',
        'historical_review_sha256':sha(review_path), 'capture_references':references,
        'captured_arrays':comparisons, 'costs':costs,
        'residual_reference':reports[0]['solver']['residual_reference'],
        'scope_and_limitations':review['scope_and_limitations'] + [
            'One real solve per process for fixed module_1_lower mask 9; no full performance rerun.',
            'Same returned robust residuals and parameters; dot reproduces both historical costs.',
            'Exact rational reference concerns supplied binary64 residuals only, not optimization or real-valued residual evaluation.',
            'Internal BLAS reduction instruction/order not traced; no universal bitwise reproducibility claim.',
            'Explicit BLAS 1 may be selected for this validated independent offline input/environment; default still inherit.']}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('review','before','after','output'): p.add_argument('--'+name,type=Path,required=True)
    a = p.parse_args(argv)
    try: result = explain(a.review,a.before,a.after)
    except Exception as exc: result = {'diagnostic_review_status':'FAILED','error':str(exc),'records_exact_equal':False}
    with a.output.open('x',encoding='utf-8') as f: json.dump(result,f,indent=2,sort_keys=True,allow_nan=False)
    # Even explained diagnostic differences still fail strict equality.
    return 0 if result.get('records_exact_equal') else 1


if __name__ == '__main__': raise SystemExit(main())
