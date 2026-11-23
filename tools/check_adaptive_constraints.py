"""Equivalence of adaptive_constraints PER_GROUP vs PER_BODY (and SAMPLED) on saved sent-path audits.

Development diagnostic only: no device import or connection, no permit. For
every saved sent-path audit of the accepted candidate in the given offline
replay directories (all six stages; the probe path is read from its
``<record>-probe-path.npy`` and checked against the recorded SHA-256), the
whole path is validated by physical single-carton workers, as saved and in
two constructed variants that continue the end forward and the start
backward (``check_adaptive_contact_audit.variants``), with the segment's
adaptive audit and PER_GROUP constraints, with PER_BODY constraints, and
for the stages given by ``--sampled-stages`` with SAMPLED as well. Verdicts,
legacy results and the first path failure (reason, edge, fraction, state,
pair) must be identical; the exactly checked states are reported.
"""
import argparse
import glob
import hashlib
import io
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.single_carton.resident import CancellableWorkerClient, TASK_SEED
from check_adaptive_contact_audit import variants, verdict, rebind

STAGES = ('approach', 'contact', 'detach', 'transfer', 'withdraw', 'return_empty')


def segments(run):
    scene = json.loads((run/'native-scene.json').read_text())
    report = json.loads((run/'replay-report.json').read_text())
    accepted = (report.get('plan_cycle_entry') or {}).get('candidate_index')
    if accepted is None:accepted = (report.get('complete_plan') or {}).get('candidate_index')
    if accepted is None:return
    for sub in sorted(run.glob(f'*-{accepted}')):
        for stage in STAGES:
            for first in (sub/f'{stage}-sent-p0-native.json', sub/f'{stage}-sent-native.json'):
                if first.is_file():break
            else:continue
            req = json.loads(first.read_text())['request']
            ref = req.get('probe_path_ref') or {}
            data = (sub/ref['file']).read_bytes()
            if hashlib.sha256(data).hexdigest() != ref['sha256']:raise SystemExit('PROBE_PATH_SHA256_MISMATCH:'+str(sub/ref['file']))
            path = np.load(io.BytesIO(data), allow_pickle=False).tolist()
            handle = 'scene_handle' in req
            req = {k: v for k, v in req.items() if k not in ('probe_path_ref', 'sent_path_audit', 'scene_handle', 'request_budget_s')}
            if handle:
                # The task scene by handle; withdraw / return_empty carry their release world inline.
                if req['scene_fingerprint'] != scene['scene_fingerprint']:raise SystemExit('SCENE_FINGERPRINT_MISMATCH')
                req['world'] = scene['world']
            elif 'world' not in req:raise SystemExit('REQUEST_WITHOUT_WORLD:'+str(first))
            audit = json.loads(first.read_text())['request'].get('sent_path_audit', 'SAMPLED')
            yield f'{run.parent.name}/{run.name}/{stage}', req, path, audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--sampled-stages', default='', help='comma-separated stages also validated with SAMPLED')
    parser.add_argument('--variants', default='saved,extended_forward,extended_backward')
    parser.add_argument('runs', nargs='+', type=Path)
    args = parser.parse_args()
    sampled = set(filter(None, args.sampled_stages.split(','))); wanted = args.variants.split(',')
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    config = load(args.config); config['record_root'] = str(out/'workers')
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    cases = []
    for run in args.runs:
        for name, req, path, audit in segments(run.resolve()):
            req, rebound = rebind(req, planner.init['identity'])
            for variant, probe in variants(path).items():
                if variant in wanted:cases.append((name, req, probe, audit, variant, rebound))
    local = threading.local(); clients = []; lock = threading.Lock(); rows = []

    def worker():
        if getattr(local, 'client', None) is None:
            client = CancellableWorkerClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                             log_path=out/f'moveit-{len(clients)}.log', name='equivalence')
            ready = client.request(planner.init, timeout=180)
            if not ready.get('capabilities', {}).get('adaptive_per_body_constraints'):raise SystemExit('WORKER_LACKS_PER_BODY_CONSTRAINTS')
            with lock:clients.append(client)
            local.client = client
        return local.client

    def run_case(case):
        name, req, probe, audit, variant, rebound = case
        client = worker(); result = {}
        modes = [('PER_GROUP', audit, 'PER_GROUP'), ('PER_BODY', audit, 'PER_BODY')]+([('SAMPLED', 'SAMPLED', None)] if req['stage'] in sampled else [])
        for label, method, constraints in modes:
            client.reset(planner.init['identity'], TASK_SEED)
            one = dict(req, op='validate', probe_path=probe, request_budget_s=1200.)
            if method != 'SAMPLED':one['sent_path_audit'] = method
            if constraints == 'PER_BODY':one['adaptive_constraints'] = 'PER_BODY'
            started = time.monotonic(); reply = client.request(one, timeout=1500); wall = time.monotonic()-started
            a = (reply.get('clearance') or {}).get('sent_path_audit') or {}
            result[label] = dict(verdict=verdict(reply), wall_s=round(wall, 3), audit={k: a.get(k) for k in (
                'sampled_states', 'checked_states', 'skipped_states', 'anchors', 'constraint_s', 'proof_s', 'skip_rule', 'adaptive_constraints')})
        reference = result['PER_GROUP']['verdict']
        same = all(r['verdict'] == reference for r in result.values())
        row = dict(segment=name, stage=req['stage'], variant=variant, audit=audit, probe_states=len(probe), identical=same,
                   identity_rebound=rebound, results=result)
        g, b = result['PER_GROUP']['audit'].get('checked_states'), result['PER_BODY']['audit'].get('checked_states')
        print(f"{'SAME' if same else 'DIFF'} {name[-34:]:34s} {variant:18s} {audit:16s} valid={reference['native_valid']} "
              f"{reference['path_reason']} exact {g} -> {b}  {result['PER_GROUP']['wall_s']:.2f}s -> {result['PER_BODY']['wall_s']:.2f}s", flush=True)
        with lock:rows.append(row)

    failure = None
    try:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            for f in [pool.submit(run_case, c) for c in cases]:f.result()
    except BaseException as error:
        failure = repr(error)[:500]
    finally:
        for client in clients:
            try:client.close()
            except Exception:pass
    rows.sort(key=lambda r: (r['segment'], r['variant']))
    tot = lambda label, key: sum((r['results'][label]['audit'].get(key) or 0) for r in rows if label in r['results'])
    summary = dict(schema='eco65_adaptive_constraints_equivalence_v1', aborted=failure, scope='OFFLINE_DIAGNOSTIC_NOT_EXECUTABLE',
                   permits_issued=0, hardware_execution='NOT_RUN', cases=len(rows), identical=sum(r['identical'] for r in rows),
                   rejected_cases=sum(r['results']['PER_GROUP']['verdict']['native_valid'] is False or
                                      r['results']['PER_GROUP']['verdict']['path_reason'] is not None for r in rows),
                   with_sampled=sum('SAMPLED' in r['results'] for r in rows), different=[r for r in rows if not r['identical']],
                   checked_states=dict(PER_GROUP=tot('PER_GROUP', 'checked_states'), PER_BODY=tot('PER_BODY', 'checked_states')),
                   wall_s=dict(PER_GROUP=round(sum(r['results']['PER_GROUP']['wall_s'] for r in rows), 2),
                               PER_BODY=round(sum(r['results']['PER_BODY']['wall_s'] for r in rows), 2)),
                   rows=rows, runs=[str(r) for r in args.runs], config=str(args.config))
    (out/'equivalence.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps({k: summary[k] for k in ('cases', 'identical', 'rejected_cases', 'with_sampled', 'checked_states', 'wall_s', 'aborted')}))
    if summary['different'] or failure:raise SystemExit(1)


if __name__ == '__main__':
    main()
