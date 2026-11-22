"""Equivalence of solution_path_check PIPELINE vs CLEARANCE_PREDICATE on saved plan requests.

Development diagnostic only: no device import or connection, no permit. Every
saved native ``plan`` request of the given offline replay directories is sent
twice to one physical single-carton worker, each time right after a task
reset (fixed seed, so OMPL is reproducible): once with the pipeline's own
solution check (mesh intersection + clearance predicate on every returned
waypoint) and once with the clearance predicate only. The statuses, the
returned points and the native output statuses must be identical.
"""
import argparse
import glob
import json
import time
from pathlib import Path
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.single_carton.resident import TASK_SEED


def requests(run):
    scene = json.loads((run/'native-scene.json').read_text())
    for path in sorted(glob.glob(str(run/'*'/'*-native.json'))):
        record = json.loads(Path(path).read_text())
        req = record.get('request') or {}
        if req.get('op') != 'plan':continue
        req = {k: v for k, v in req.items() if k not in ('solution_path_check',)}
        if 'scene_handle' in req:
            if req['scene_fingerprint'] != scene['scene_fingerprint']:raise SystemExit('SCENE_FINGERPRINT_MISMATCH:'+path)
            req.pop('scene_handle'); req['world'] = scene['world']
        yield path, req, record.get('result') or {}


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
    from unloading_sim.workbench.single_carton.resident import CancellableWorkerClient
    worker = CancellableWorkerClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=out/'moveit.log', name='equivalence')
    rows = []
    try:
        ready = worker.request(planner.init, timeout=180)
        if not ready.get('capabilities', {}).get('clearance_predicate_solution_check'):
            raise SystemExit('WORKER_LACKS_CLEARANCE_PREDICATE_SOLUTION_CHECK')
        for run in args.runs:
            for path, req, saved in requests(run.resolve()):
                req, rebound = rebind(req, planner.init['identity'])
                pair = {}
                for mode in ('PIPELINE', 'CLEARANCE_PREDICATE'):
                    worker.reset(planner.init['identity'], TASK_SEED)
                    one = dict(req, solution_path_check=mode)
                    started = time.monotonic(); reply = worker.request(one, timeout=300); wall = time.monotonic()-started
                    pair[mode] = dict(status=reply.get('status'), native_output_status=reply.get('native_output_status'),
                                      points=reply.get('points'), wall_s=wall, mtc_plan_s=reply.get('mtc_plan_s'),
                                      check=reply.get('solution_path_check'))
                a, b = pair['PIPELINE'], pair['CLEARANCE_PREDICATE']
                same = a['status'] == b['status'] and a['native_output_status'] == b['native_output_status'] and a['points'] == b['points']
                rows.append(dict(record=str(Path(path).relative_to(ROOT)) if str(path).startswith(str(ROOT)) else path,
                                 stage=req.get('stage'), pipeline=req.get('pipeline_id'), planner=req.get('planner_id'),
                                 saved_status=saved.get('status'), pipeline_status=a['status'], predicate_status=b['status'],
                                 identical=same, pipeline_wall_s=round(a['wall_s'], 4), predicate_wall_s=round(b['wall_s'], 4),
                                 points=len(a['points'] or []), predicate_check=b['check'], identity_rebound=rebound))
                print(f"{'SAME' if same else 'DIFF'} {rows[-1]['stage']:13s} {rows[-1]['planner']:24s} {a['status']:28s} "
                      f"{a['wall_s']:6.2f}s -> {b['wall_s']:6.2f}s", flush=True)
    except Exception as error:
        failure = repr(error)[:500]
    else:
        failure = None
    finally:
        worker.close()
    summary = dict(schema='eco65_solution_path_check_equivalence_v1', aborted=failure, scope='OFFLINE_DIAGNOSTIC_NOT_EXECUTABLE', permits_issued=0,
                   hardware_execution='NOT_RUN', requests=len(rows), identical=sum(r['identical'] for r in rows),
                   different=[r for r in rows if not r['identical']],
                   pipeline_wall_s=round(sum(r['pipeline_wall_s'] for r in rows), 2),
                   predicate_wall_s=round(sum(r['predicate_wall_s'] for r in rows), 2), rows=rows,
                   runs=[str(r) for r in args.runs], config=str(args.config))
    (out/'equivalence.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps({k: summary[k] for k in ('requests', 'identical', 'pipeline_wall_s', 'predicate_wall_s')}))
    if summary['different'] or failure:raise SystemExit(1)


if __name__ == '__main__':
    main()
