"""planning_module PYTHON vs NATIVE on the synthetic plans (no hardware).

Compares two ``tools/check_step4_mechanisms.py --mode off`` outputs (A.json,
B.json), one run with ``--planning-module PYTHON`` and one with ``NATIVE``:
outcome, selected candidate, endpoint screen, total duration, per segment
planner / send point count / send duration / probe states / detach profile /
first and last joint vectors, and every failure reason. Floats agree within
1e-12 (the worker's prepare equals Python's to rounding); everything else must
be identical. Also reports round trips, pipe bytes, composites and plan time.

Usage: python tools/check_native_planning.py --python DIR --native DIR --output OUT.json
"""
import argparse
import json
from pathlib import Path
import sys

TOLERANCE = 1e-12
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))


def differences(a, b, path=''):
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and abs(a-b) <= TOLERANCE:
            return []
        return [(path, a, b)]
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            out += differences(a.get(k), b.get(k), f'{path}/{k}')
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [(path+'#len', len(a), len(b))]
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += differences(x, y, f'{path}[{i}]')
        return out
    return [] if a == b else [(path, a, b)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--python', type=Path, required=True)
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from check_step4_mechanisms import comparable
    rows = []
    for scene in ('A', 'B'):
        a = json.loads((args.python/f'{scene}.json').read_text()); b = json.loads((args.native/f'{scene}.json').read_text())
        diff = differences(comparable(a), comparable(b))
        rows.append(dict(scene=scene, identical_within_tolerance=not diff, differences=diff[:20],
                         python=dict(elapsed_s=a['elapsed_s'], candidate=a.get('candidate'), **(a.get('transport') or {})),
                         native=dict(elapsed_s=b['elapsed_s'], candidate=b.get('candidate'), **(b.get('transport') or {}))))
    report = dict(schema='eco65_native_planning_equivalence_v1', scope='SYNTHETIC_SCENES_NOT_FIELD_DATA', tolerance=TOLERANCE,
                  compared_fields='outcome, candidate, endpoint screen, total duration, per segment planner/points/send duration/'
                                  'probe states/detach profile/first and last q, every failure reason',
                  rows=rows, all_identical=all(r['identical_within_tolerance'] for r in rows))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps(report, indent=1, default=str)[:4000])


if __name__ == '__main__':
    main()
