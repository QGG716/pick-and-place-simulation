"""One fixed mask-9 solver capture, offline only; no full pipeline or GT input.

Wrap the real least_squares call without changing arguments, residuals or result.
Residual references are computed AFTER the solver has returned. No acceptance
tolerance is introduced. Use separate processes for inherit and explicit BLAS 1.
"""
import argparse
from dataclasses import asdict
from fractions import Fraction
import hashlib
import importlib
import inspect
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src')]
from metric_thread_policy import positive_int, policy, snapshot, configure_blas, verify_blas, verify_non_targets


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def residual_reference(values):
    # Exact rational arithmetic ON THE RETURNED binary64 RESIDUALS ONLY.
    # It proves neither exact residual evaluation nor exact optimization.
    exact = sum((Fraction(float(v))**2 for v in values), Fraction())/2
    return {'exact_binary64_residual_sum_numerator': str(exact.numerator),
        'exact_binary64_residual_sum_denominator': str(exact.denominator),
        'correctly_rounded_cost': float(exact),
        'fsum_of_rounded_squares_cost': .5*math.fsum(float(v)*float(v) for v in values)}


def capture_least_squares(real, captures, arrays):
    def call(fun, x0, *args, **kwargs):
        if fun.__name__ != 'robust_residual': return real(fun, x0, *args, **kwargs)
        import numpy as np
        initial = np.asarray(x0).copy()
        bound = inspect.signature(real).bind(fun, x0, *args, **kwargs); bound.apply_defaults()
        if bound.arguments['loss'] != 'linear': raise ValueError('UNREVIEWED_LOSS')
        closure = inspect.getclosurevars(fun).nonlocals
        def array_id(a):
            a = np.asarray(a)
            return {'shape': list(a.shape), 'dtype': str(a.dtype), 'sha256': hashlib.sha256(a.tobytes()).hexdigest()}
        inputs = {'initial': array_id(initial), 'association': array_id(closure['association']),
            'signs': array_id(closure['signs']), 'config': asdict(closure['config']),
            'groups': [{k: array_id(g[k]) for k in ('normal','offset','fit_points')} for g in closure['groups']]}
        result = real(fun, x0, *args, **kwargs)  # exact original call, same object returned
        if captures: raise ValueError('EXPECTED_EXACTLY_ONE_ORTHOGONAL_SOLVE')
        arrays.update(initial=initial, x=result.x.copy(), fun=result.fun.copy(), jac=result.jac.copy(), grad=result.grad.copy())
        captures.append({'inputs': inputs, 'loss': bound.arguments['loss'], 'method': bound.arguments['method'],
            'explicit_kwargs': kwargs, 'cost': float(result.cost), 'dot_cost': float(.5*np.dot(result.fun, result.fun)),
            'success': bool(result.success), 'status': int(result.status), 'message': str(result.message),
            'nfev': int(result.nfev), 'njev': int(result.njev), 'optimality': float(result.optimality),
            'residual_reference': residual_reference(result.fun)})
        return result
    return call


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--archive-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--blas-threads', type=positive_int)
    a = parser.parse_args(argv)
    a.output.mkdir(parents=True, exist_ok=False)
    report = {'status':'FAILED', 'policy':policy(a.blas_threads), 'plan_sha256':digest(a.plan),
              'scope':'module_1_lower/mask_id=9 ONLY', 'raw_image_automatic':False, 'planning_admissible':False,
              'evidence_label':'ORACLE-PROMPTED', 'gt_geometry_read':False}
    try:
        report['before_policy'] = snapshot()
        report['applied'] = configure_blas(report['before_policy']['blas'], a.blas_threads)
        report['before_geometry'] = snapshot()
        verify_blas(report['before_geometry']['blas'], a.blas_threads)
        verify_non_targets(report['before_policy'], report['before_geometry'])
        import numpy as np
        import scipy.optimize
        from unloading_perception.metric_faces import extract_observation_labels, fit_metric_faces, MetricFitConfig, _binding
        from unloading_perception.final_geometry import validate_final_record
        from diagnose_metric_calibration import load_extractor
        from compare_metric_faces_ab import read
        from review_metric_blas_cost import verify_evidence
        report['evidence_verification'] = verify_evidence(a.archive_root, a.archive_root/'evidence_manifest.json', a.plan, a.archive_root/'code')
        plan = read(a.plan); spec = next(m for m in plan['modules'] if m['module_id'] == 'module_1_lower')
        directory, masks_path = Path(spec['input_directory']), Path(spec['masks_path'])
        camera = read(directory/'camera_info.json'); binding = read(directory/'capture_binding.json')
        if camera['K'] != spec['K'] or digest(directory/'metric_depth_m.npy') != binding['metric_depth_sha256']:
            raise ValueError('CAPTURE_BINDING_MISMATCH')
        depth = np.load(directory/'metric_depth_m.npy', allow_pickle=False)
        with np.load(masks_path, allow_pickle=False) as masks:
            if masks['mask_ids'].tolist() != spec['mask_ids'] or list(masks['masks'].shape) != spec['mask_array_shape']:
                raise ValueError('MASK_ROSTER_MISMATCH')
            mask = masks['masks'][spec['mask_ids'].index(9)].copy()
        K = np.asarray(spec['K']).reshape(3,3)
        entry = next(i for i in read(masks_path.parent/'cargo_instances.json')['instances'] if int(i['instance_id']) == 9)
        historical = {run: next(r for r in read(a.archive_root/run/'module_1_lower/rgbd_cuboids.json')['instances'] if r['mask_id'] == 9)
                      for run in ('A-inherit','B-blas-1')}
        config = MetricFitConfig(positive_infinity_is_no_hit=True)
        if any(asdict(config) != h['config'] or _binding(depth, mask, K) != h['support_capture_binding'] for h in historical.values()):
            raise ValueError('HISTORICAL_SOLVER_INPUT_OR_CONFIG_MISMATCH')
        report['input_binding'] = _binding(depth,mask,K)
        report['capture_id'] = spec['capture_id']; report['T_W_C_at_capture'] = camera['T_W_C']
        report['production_sha256'] = {p: digest(ROOT/p) for p in ('src/unloading_perception/metric_faces.py',
            'src/unloading_perception/metric_support.py', 'src/unloading_perception/final_geometry.py')}
        trf = importlib.import_module('scipy.optimize._lsq.trf')
        report['scipy_trf_source'] = {'path':trf.__file__, 'sha256':digest(trf.__file__),
            'cost_lines': [line.strip() for line in inspect.getsource(trf.trf_no_bounds).splitlines() if 'cost' in line and ('dot' in line or 'rho' in line)]}
        labels,seeds,audit = extract_observation_labels(depth, mask, K, load_extractor(Path(plan['vision_root'])),config)
        captures, arrays = [], {}
        original = scipy.optimize.least_squares
        scipy.optimize.least_squares = capture_least_squares(original, captures, arrays)
        try:
            record = fit_metric_faces(depth,mask,K,labels,seeds,mask_id=9,config=config,
                                      source_ambiguous=bool(entry.get('supporting_proposal_ids',[])))
        finally: scipy.optimize.least_squares = original
        record['observation_segmentation'] = audit
        if len(captures) != 1: raise ValueError('MISSING_REAL_SOLVER_CAPTURE')
        # Independently validate exactly as observation conversion does.
        checked = validate_final_record(record, depth, mask, K)
        report['independent_validation_unchanged'] = checked == record
        report['historical_record_exact_matches'] = {name:record == r for name,r in historical.items()}
        for value in arrays.values():
            if not np.isfinite(value).all(): raise ValueError('NONFINITE_SOLVER_CAPTURE')
        np.savez_compressed(a.output/'solver_arrays.npz', **arrays)
        report['solver_arrays_sha256'] = digest(a.output/'solver_arrays.npz')
        report['solver'] = captures[0]
        report['after_geometry'] = snapshot()
        verify_blas(report['after_geometry']['blas'], a.blas_threads)
        verify_non_targets(report['before_policy'], report['after_geometry'])
        if a.blas_threads is None and report['before_policy']['blas'] != report['after_geometry']['blas']:
            raise ValueError('INHERITED_BLAS_CHANGED')
        report['status'] = 'COMPLETED'
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
    with (a.output/'solver_capture.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
    print(json.dumps({'status':report['status'], 'error':report.get('error')}), flush=True)
    return 0 if report['status']=='COMPLETED' else 1


if __name__ == '__main__': raise SystemExit(main())
