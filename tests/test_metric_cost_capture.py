"""Diagnostic wrapper tests use synthetic solvers, not historical reproduction."""
from dataclasses import dataclass
from fractions import Fraction
from types import SimpleNamespace
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import pytest

from tools.reproduce_metric_cost import capture_least_squares, residual_reference


def test_reference_is_exact_only_for_supplied_binary64_residuals():
    values = [.1, .2, .3]
    exact = sum((Fraction(v)**2 for v in values), Fraction())/2
    ref = residual_reference(values)
    assert Fraction(int(ref['exact_binary64_residual_sum_numerator']), int(ref['exact_binary64_residual_sum_denominator'])) == exact
    assert ref['correctly_rounded_cost'] == float(exact)


def test_wrapper_passes_arguments_and_returns_original_object():
    @dataclass
    class Config: seed: int = 17
    config = Config(); association = np.array([0]); signs = np.array([1.])
    groups = [dict(normal=np.array([0.,0.,1.]), offset=-2., fit_points=np.ones((2,3)))]
    def robust_residual(x):
        assert config and association is not None and signs is not None and groups
        return x
    returned = SimpleNamespace(x=np.array([1.]), fun=np.array([.1,.2]), jac=np.ones((2,1)), grad=np.ones(1),
        cost=.025000000000000005, success=True, status=1, message='synthetic', nfev=1, njev=1, optimality=0.)
    initial = np.array([0.]); seen = []
    def original(fun, x0, *, loss='linear', method='trf', max_nfev=100):
        seen.append((fun, x0, max_nfev)); return returned
    captures, arrays = [], {}
    wrapped = capture_least_squares(original, captures, arrays)
    assert wrapped(robust_residual, initial, max_nfev=9) is returned
    assert seen[0][0] is robust_residual and seen[0][1] is initial and seen[0][2] == 9
    assert captures[0]['loss'] == 'linear' and np.array_equal(arrays['fun'], returned.fun)
    assert np.array_equal(initial, [0.])
    with pytest.raises(ValueError, match='UNREVIEWED_LOSS'):
        wrapped(robust_residual, initial, loss='soft_l1')


def test_non_target_solver_is_not_captured_or_modified():
    calls = []
    def boundary_residual(x): return x
    def real(*args, **kwargs): calls.append((args,kwargs)); return 'original'
    captures, arrays = [], {}
    assert capture_least_squares(real,captures,arrays)(boundary_residual, [0.], loss='soft_l1') == 'original'
    assert not captures and not arrays and calls[0][1] == {'loss':'soft_l1'}


def archived_capture_fixture(tmp_path):
    # Verify real saved data; this test does NOT run a solver or a historical A/B.
    source = Path(__file__).resolve().parents[1]/'docs/validation/evidence/metric-cost-review-20260926'
    shutil.copyfile(source/'historical_review.json',tmp_path/'review.json')
    for label in ('inherit','blas-1'):
        root = tmp_path/label; root.mkdir()
        shutil.copyfile(source/f'mask9-{label}.json',root/'solver_capture.json')
        shutil.copyfile(source/f'mask9-{label}.npz',root/'solver_arrays.npz')
    return tmp_path/'review.json',tmp_path/'inherit',tmp_path/'blas-1'


def test_real_archived_explanation_still_returns_strict_failure(tmp_path):
    from tools.explain_metric_cost_capture import explain, main
    review,before,after = archived_capture_fixture(tmp_path)
    result = explain(review,before,after)
    assert result['diagnostic_review_status'] == 'EXPLAINED_SAME_FINAL_RESIDUAL_DOT_REDUCTION'
    assert result['protected_geometry_exact_equal'] and result['decisions_and_support_exact_equal']
    assert result['records_exact_equal'] is False and result['difference_count'] == 2
    assert all(v['bitwise_equal'] for v in result['captured_arrays'].values())
    assert main(['--review',str(review),'--before',str(before),'--after',str(after),
                 '--output',str(tmp_path/'explained.json')]) == 1


@pytest.mark.parametrize('damage',['x','fun','hash','cost','nonfinite_cost','huge_cost','status','validation','threads','new_difference'])
def test_captured_evidence_cannot_explain_unrelated_or_invalid_changes(tmp_path,damage):
    from tools.explain_metric_cost_capture import explain
    review,before,after = archived_capture_fixture(tmp_path)
    path=after/'solver_capture.json'; report=json.loads(path.read_text())
    if damage in ('x','fun','hash'):
        npz=after/'solver_arrays.npz'
        with np.load(npz) as data: values={k:data[k] for k in data.files}
        values['fun' if damage=='fun' else 'x'][0] += 1e-8
        np.savez_compressed(npz,**values)
        if damage!='hash': report['solver_arrays_sha256']=hashlib.sha256(npz.read_bytes()).hexdigest()
    elif damage in ('cost','nonfinite_cost','huge_cost'):
        report['solver']['cost']={'cost':2.,'nonfinite_cost':float('nan'),'huge_cost':1e200}[damage]
    elif damage=='status': report['solver']['status']=4
    elif damage=='validation': report['independent_validation_unchanged']=False
    elif damage=='threads': report['after_geometry']['blas'][0]['num_threads']=64
    else:
        historical=json.loads(review.read_text());historical['difference_count']+=1
        review.write_text(json.dumps(historical))
    path.write_text(json.dumps(report))
    with pytest.raises(ValueError): explain(review,before,after)
