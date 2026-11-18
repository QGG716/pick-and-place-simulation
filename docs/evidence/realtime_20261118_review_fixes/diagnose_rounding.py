"""Inspect a recorded mismatch without changing either prepare implementation.

Run after tools/check_native_prepare.py; requires its literal per-case scale.
Only compiles a coefficient probe, reads saved points and writes diagnostics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess

import numpy as np
import scipy
from scipy.interpolate import BPoly, PPoly
from scipy.special import comb

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.comparison.read_text())
    row = max((r for r in report['rows'] if r.get('max_abs')), key=lambda r:r['max_abs']['a'])
    source = Path(row['source'])
    points = json.loads(source.read_text())['result']['points']
    scale, period = row['time_scale'], row['period_s']
    t = np.array([p['t'] for p in points])*scale
    q = np.array([p['q'] for p in points]);v = np.array([p['v'] for p in points])/scale
    a = np.array([p['a'] for p in points])/scale**2
    bp = BPoly.from_derivatives(t, [[q[i],v[i],a[i]] for i in range(len(t))])
    reference = PPoly.from_bernstein_basis(bp)
    args.output.mkdir(parents=True, exist_ok=False)
    binary = args.output/'prepare_coefficients'
    command = ['g++', '-std=c++17', '-O3', '-I/usr/include/eigen3',
               '-I'+str(ROOT/'ros2/m710_moveit_backend/src'), str(HERE/'prepare_coefficients.cpp'), '-o', str(binary)]
    subprocess.run(command, check=True)
    native = json.loads(subprocess.check_output([str(binary)], text=True,
        input=json.dumps(dict(points=points, time_scale=scale))+'\n'))
    coefficients = np.array(native['c']).reshape(reference.c.shape)
    dx = np.diff(t)
    scalar_powers = np.array([[float(x)**k for x in dx] for k in range(6)])
    diagnostic = np.zeros_like(bp.c)
    # Diagnostic reconstruction only: substitute scalar powers in the same
    # Bernstein-to-power formula. Production Python stays untouched.
    for i in range(6):
        factor = (-1)**i*comb(5,i)*bp.c[i]
        for k in range(i,6):diagnostic[5-k] += factor*(comb(5-i,k-i)*(-1)**k)/scalar_powers[k,:,None]
    ts = np.minimum(np.arange(int(np.ceil(t[-1]/period))+1)*period, t[-1])
    native_coeff_poly = PPoly(coefficients, t)
    coefficient_diff = abs(reference.c-coefficients)
    index = tuple(int(x) for x in np.unravel_index(coefficient_diff.argmax(), coefficient_diff.shape))
    result = dict(scope='READ_ONLY_COEFFICIENT_DIAGNOSIS_NOT_NUMERIC_ACCEPTANCE',
        source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        comparison_sha256=hashlib.sha256(args.comparison.read_bytes()).hexdigest(),
        time_scale=scale, period_s=period, worker_max_abs=row['max_abs'],
        coefficient_max_abs=float(coefficient_diff.max()), coefficient_max_index=index,
        interval_duration_s=float(dx[index[1]]),
        scalar_power_diagnostic_vs_native_coeff_max_abs=float(abs(diagnostic-coefficients).max()),
        coefficient_only_evaluation_max_abs={name:float(abs(reference(ts,order)-native_coeff_poly(ts,order)).max())
                                           for order,name in enumerate(('q','v','a'))},
        array_vs_scalar_power_differences={str(k):int(np.count_nonzero(scalar_powers[k] != dx**k)) for k in range(6)},
        compiler_command=command, probe_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        native_header_sha256=hashlib.sha256((ROOT/'ros2/m710_moveit_backend/src/native_prepare.h').read_bytes()).hexdigest(),
        environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__),
        python_reference_changed=False, tolerance=1e-12, physical='NOT_RUN')
    (args.output/'report.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':main()
