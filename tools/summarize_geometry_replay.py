"""Print a before/after table from tools/compare_geometry_replay.py outputs (JSON + markdown)."""
import json
import sys
from pathlib import Path


def load(folder):
    out = {}
    for f in sorted(Path(folder).glob('*.json')):
        r = json.loads(f.read_text())
        for d in r['detections']:out[(r['task_id'], d['detection_index'])] = (d, r)
    return out


def cell(d):
    if d is None:return 'n/a'
    if d['status'] != 'RECOVERED':return 'REJECTED: '+d['reason']
    e = ' '.join(f"{k}:{v['method'].split('_')[0][:6]}{'('+v['missing_class']+')' if v.get('missing_class') else ''}"
                 for k, v in d['edges'].items())
    rest = d['rest']; first = rest['first']
    blocked = ('BLOCKED '+(f"{first['id']} {first['overlap_mm']} mm x{rest['intersections']}" if first else 'WORKSPACE')
               if rest['blocked'] else 'CLEAR')
    if (rest.get('coplanar_seam_contact') or {}).get('all_within_seam_band'):blocked = 'COPLANAR_SEAM_CONTACT_NOT_PERMITTED'
    return (f"meas {d['measured_size_mm']} env {d['envelope_size_mm']}; {e}; rest {blocked}; "
            f"search {'YES' if d['enters_candidate_search'] else 'NO'}")


if __name__ == '__main__':
    columns = sys.argv[1:]
    data = [load(c) for c in columns]
    keys = sorted(set().union(*data))
    print('| task/det | '+' | '.join(Path(c).name for c in columns)+' |')
    print('|---|'+'---|'*len(columns))
    for k in keys:
        print(f'| {k[0][:8]}/{k[1]} | '+' | '.join(cell(d.get(k, (None,))[0]) for d in data)+' |')
