"""Truth-containment sweep for measured carton envelopes (synthetic, no device IO).

Uses the literal ray-cast fixture of tests/test_single_carton_variable_geometry
and mask defects: one side missing 0-60 px, SAM-like low-resolution decoding at
thresholds 0.35/0.5/0.65, +/-2 px shifts, depth noise 0/0.5/1 mm. A rejection
is safe; an accepted envelope that does not contain the true cuboid is a
violation. Prints JSON; optional --zero-coverage records raw direct-boundary
statistics for threshold calibration.
"""
import argparse
import json
import sys
from itertools import product
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'tests')]
from test_single_carton_variable_geometry import measured_box  # noqa: E402
from field_like_scene import samlike  # noqa: E402
from unloading_sim.workbench.single_carton import measured_envelope  # noqa: E402
from unloading_sim.workbench.single_carton.geometry import recover, corners  # noqa: E402

SIZES = [(.24, .18, .12), (.32, .21, .16), (.12, .08, .06)]
ANGLES = [(10, 30, 15), (-15, 25, 0), (20, -30, 25)]


def defects(mask):
    ys, xs = np.nonzero(mask)
    for side, cols in product(('left', 'right', 'top', 'bottom'), (0, 5, 10, 20, 30, 40, 60)):
        m = mask.copy()
        if side == 'left': m[:, :xs.min()+cols] = False
        if side == 'right': m[:, xs.max()+1-cols:] = False
        if side == 'top': m[:ys.min()+cols] = False
        if side == 'bottom': m[ys.max()+1-cols:] = False
        yield f'missing_{side}_{cols}px', m
    for t in (.35, .5, .65):
        yield f'samlike_{t}', samlike(mask, t)
    for dy, dx in ((0, 2), (0, -2), (2, 0), (-2, 0)):
        yield f'shift_{dy}_{dx}', np.roll(mask, (dy, dx), axis=(0, 1))


def run(noise_levels=(0., .5, 1.)):
    rows = []
    for size, angles, noise in product(SIZES, ANGLES, noise_levels):
        p, m, c, R = measured_box(size, angles)
        rng = np.random.default_rng(6518)
        p = p+p/p[..., 2, None]*rng.uniform(-noise/1000, noise/1000, m.shape)[..., None]
        truth = np.eye(4); truth[:3, :3] = R; truth[:3, 3] = c
        for name, mask in defects(m):
            row = dict(size_m=size, angles_deg=angles, noise_mm=noise, defect=name)
            try:
                b = recover(p, mask, np.eye(4), conservative_measured=True)
            except ValueError as e:
                rows.append(dict(row, status='REJECTED', reason=str(e))); continue
            T = np.asarray(b['pose']); local = (corners(truth, size)-T[:3, 3])@T[:3, :3]
            violation = float(np.max(abs(local)-np.asarray(b['size_m'])/2))
            M = np.asarray(b['measured_box']['pose']); tl = (corners(truth, size)-M[:3, 3])@M[:3, :3]
            edges = []
            for i in b['uncertainty']['intervals']:
                a, s = i['axis'], i['side']
                beyond = float((tl[:, a].min()-i['outer_m'])*-1 if s < 0 else tl[:, a].max()-i['outer_m'])
                cov = i.get('direct_coverage') or {}
                edges.append(dict(axis=a, side=s, method=i['method'], truth_beyond_outer_m=beyond,
                                  strip_pixels=cov.get('strip_pixels'), span_m=cov.get('span_m'),
                                  candidate_pixels=cov.get('candidate_pixels')))
            rows.append(dict(row, status='ACCEPTED', contained=violation <= 1e-7,
                             max_violation_m=violation, envelope_m=b['size_m'], edges=edges))
    return rows


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--zero-coverage', action='store_true')
    ap.add_argument('--output')
    a = ap.parse_args()
    if a.zero_coverage:
        measured_envelope.DIRECT_MIN_PIXELS = 1; measured_envelope.DIRECT_MIN_SPAN_M = 0.
    rows = run()
    accepted = [r for r in rows if r['status'] == 'ACCEPTED']
    summary = dict(cases=len(rows), accepted=len(accepted), rejected=len(rows)-len(accepted),
                   not_contained=sum(not r['contained'] for r in accepted),
                   direct_min_pixels=measured_envelope.DIRECT_MIN_PIXELS,
                   direct_min_span_m=measured_envelope.DIRECT_MIN_SPAN_M)
    text = json.dumps(dict(summary=summary, rows=rows), indent=1)
    if a.output: Path(a.output).write_text(text)
    print(json.dumps(summary))
