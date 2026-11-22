"""Equivalence of sent_path_audit SAMPLED vs ADAPTIVE_CONTACT on saved contact/support segments.

Development diagnostic only: no device import or connection, no permit. For
every saved contact / detach / withdraw sent-path check of the given offline
replay directories, the full probe path (its chunks joined) is validated by
one physical single-carton worker with SAMPLED and with ADAPTIVE_CONTACT, as
saved and in two constructed variants that push into the contact face, the
released carton or the supports: the end continued forward and the start
continued backward by as much as the last / first 5 % of the states move.
Verdicts, legacy results and the first
path failure (reason, edge, fraction, state) must be identical.
"""
import argparse
import glob
import json
import re
import time
from pathlib import Path
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.single_carton.resident import CancellableWorkerClient, TASK_SEED

STAGES = ('contact', 'detach', 'withdraw')


def segments(run):
    scene = json.loads((run/'native-scene.json').read_text())
    groups = {}
    for path in glob.glob(str(run/'*'/'*-sent-p*-native.json'))+glob.glob(str(run/'*'/'*-sent-native.json')):
        m = re.match(r'(.*)-sent(?:-p(\d+))?-native\.json$', Path(path).name)
        record = json.loads(Path(path).read_text()); req = record.get('request') or {}
        if req.get('stage') not in STAGES or 'probe_path' not in req:continue
        groups.setdefault((Path(path).parent.name, m.group(1)), []).append((int(m.group(2) or 0), req))
    for (candidate, record), chunks in sorted(groups.items()):
        chunks.sort(key=lambda c: c[0])
        path = []
        for k, (_, req) in enumerate(chunks):path += req['probe_path'] if k == 0 else req['probe_path'][1:]
        req = {k: v for k, v in chunks[0][1].items() if k not in ('probe_path', 'sent_path_audit')}
        if 'scene_handle' in req:
            if req['scene_fingerprint'] != scene['scene_fingerprint']:raise SystemExit('SCENE_FINGERPRINT_MISMATCH')
            req.pop('scene_handle'); req['world'] = scene['world']
        yield f'{run.name}/{candidate}/{record}', req, path


def variants(path):
    # Continue the path along the mean direction of its last / first 5 % of
    # states (the end samples themselves are nearly at rest), so the variant
    # moves about as far again: into the contact face, the released carton or
    # the supports.
    P = np.asarray(path, float); n = max(2, len(P)//20)
    forward = np.vstack([P, P[-1]+np.outer(np.arange(1, n+1)/n, P[-1]-P[-1-n])])
    backward = np.vstack([P[0]-np.outer(np.arange(n, 0, -1)/n, P[n]-P[0]), P])
    return dict(saved=P.tolist(), extended_forward=forward.tolist(), extended_backward=backward.tolist())


def verdict(reply):
    f = reply.get('path_failure') or {}
    return dict(native_valid=reply.get('native_valid'), legacy=reply.get('legacy_intersection_valid'),
                failure=reply.get('failure'), path_reason=f.get('reason'), edge=f.get('edge'), fraction=f.get('fraction'),
                q=f.get('q_rad'), pair=f.get('pair'))


def rebind(req, identity):
    """Rebind a request recorded under another checkout: only the model fingerprint
    (which also digests launch parameters such as file paths) may differ; the
    worker still checks the policy, tool links and contact geometry byte for byte."""
    other = req.get('identity') or {}
    if other == identity:return req, False
    if {k: v for k, v in other.items() if k != 'model_tool_fingerprint'} != {k: v for k, v in identity.items() if k != 'model_tool_fingerprint'}:
        raise SystemExit('REQUEST_IDENTITY_DIFFERS_BEYOND_MODEL_FINGERPRINT')
    return dict(req, identity=identity), True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('runs', nargs='+', type=Path)
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    config = load(args.config); config['record_root'] = str(out/'workers')
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    worker = CancellableWorkerClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=out/'moveit.log', name='equivalence')
    rows = []
    try:
        ready = worker.request(planner.init, timeout=180)
        if not ready.get('capabilities', {}).get('adaptive_contact_sent_path_audit'):raise SystemExit('WORKER_LACKS_ADAPTIVE_CONTACT')
        for run in args.runs:
            for name, req, path in segments(run.resolve()):
                req, rebound = rebind(req, planner.init['identity'])
                for variant, probe in variants(path).items():
                    result = {}
                    for mode in ('SAMPLED', 'ADAPTIVE_CONTACT'):
                        worker.reset(planner.init['identity'], TASK_SEED)
                        one = dict(req, op='validate', probe_path=probe, request_budget_s=600.)
                        if mode != 'SAMPLED':one['sent_path_audit'] = mode
                        started = time.monotonic(); reply = worker.request(one, timeout=900); wall = time.monotonic()-started
                        audit = (reply.get('clearance') or {}).get('sent_path_audit') or {}
                        result[mode] = dict(verdict=verdict(reply), wall_s=wall, audit={k: audit.get(k) for k in (
                            'sampled_states', 'checked_states', 'skipped_states', 'state_rule_checks', 'state_rule_fallbacks', 'skip_rule')})
                    same = result['SAMPLED']['verdict'] == result['ADAPTIVE_CONTACT']['verdict']
                    rows.append(dict(segment=name, stage=req['stage'], variant=variant, probe_states=len(probe), identical=same, identity_rebound=rebound,
                                     sampled=result['SAMPLED'], adaptive_contact=result['ADAPTIVE_CONTACT']))
                    v = result['SAMPLED']['verdict']
                    print(f"{'SAME' if same else 'DIFF'} {name[-40:]:40s} {variant:18s} valid={v['native_valid']} {v['path_reason']} "
                          f"{result['SAMPLED']['wall_s']:6.2f}s -> {result['ADAPTIVE_CONTACT']['wall_s']:6.2f}s", flush=True)
    except Exception as error:
        failure = repr(error)[:500]
    else:
        failure = None
    finally:
        worker.close()
    summary = dict(schema='eco65_adaptive_contact_equivalence_v1', aborted=failure, scope='OFFLINE_DIAGNOSTIC_NOT_EXECUTABLE', permits_issued=0,
                   hardware_execution='NOT_RUN', cases=len(rows), identical=sum(r['identical'] for r in rows),
                   rejected_cases=sum(r['sampled']['verdict']['native_valid'] is False or r['sampled']['verdict']['path_reason'] is not None for r in rows),
                   different=[r for r in rows if not r['identical']],
                   sampled_wall_s=round(sum(r['sampled']['wall_s'] for r in rows), 2),
                   adaptive_contact_wall_s=round(sum(r['adaptive_contact']['wall_s'] for r in rows), 2), rows=rows,
                   runs=[str(r) for r in args.runs], config=str(args.config))
    (out/'equivalence.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps({k: summary[k] for k in ('cases', 'identical', 'rejected_cases', 'sampled_wall_s', 'adaptive_contact_wall_s')}))
    if summary['different'] or failure:raise SystemExit(1)


if __name__ == '__main__':
    main()
