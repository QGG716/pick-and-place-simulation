"""Real-worker check of the unified planning deadline on a SYNTHETIC scene.

No device, SDK, valve or field record is used. The observation is a synthetic
LIVE-stamped stand-in in this process's session (so plan_cycle accepts it)
and the scene is one synthetic carton; nothing here is field evidence and no
permit can be executed (the authority session ends with this process).

Modes:
  --mode deadline --deadlines 0.5,3,6   plan_cycle(deadline=entry+d) per value;
      reports return time after the deadline, classification and the
      searched / unsearched counts.
  --mode search [--deadline-s D] --candidates 0,1,2   search_box only (as the
      replay tool does) on chosen candidates; used to compare the unset path
      of two code versions record by record.
  --mode clock   a validate request whose clock reference lies in the future
      must be refused with DEADLINE_CLOCK_DOMAIN_MISMATCH.
"""
import argparse
import json
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton import planner as planner_module
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.geometry import corners
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.moveit2_backend import ResidentMoveItClient

Q_START = np.deg2rad([86.447, 16.414, -69.186, -30.712, 86.945, 77.313])


def synthetic_box():
    pose = np.eye(4); pose[:3, 3] = [.4, 0., .4]
    return dict(id='synthetic-target', detection_index=0, pose=pose.tolist(), size_m=[.15, .1, .1],
                corners_m=corners(pose, [.15, .1, .1]).tolist(),
                faces=[dict(observed=True, center_m=[.4, 0., .45], normal=[0, 0, 1], u=[1, 0, 0], v=[0, 1, 0], size_m=[.15, .1])],
                quality={}, scope='SYNTHETIC_DEADLINE_CHECK_NOT_FIELD_DATA')


def observation(model):
    from unloading_sim.workbench.contracts import Stamp
    return SimpleNamespace(q_start=tuple(Q_START), world_tcp_start=model.tcp(Q_START).tolist(),
                           frame=SimpleNamespace(stamp=Stamp.receive(), observation_id='synthetic-deadline-check'),
                           robot_sample={'source': 'LIVE'},
                           metadata=lambda: dict(source='SYNTHETIC_DEADLINE_CHECK_NOT_HARDWARE_EVIDENCE'))


def make_planner(config_path):
    config = load(config_path)
    config.pop('site', None)  # synthetic scene: no frozen RGB-D site scene
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    return planner, model


def deadline_mode(planner, model, values, out):
    rows = []
    for value in values:
        folder = out/f'deadline-{value:g}'; folder.mkdir()
        entered = time.monotonic(); row = dict(deadline_s=value)
        try:
            plan = planner.plan_cycle(observation(model), [synthetic_box()], folder, deadline=entered+value)
            row.update(outcome='PLAN_FOUND', candidate=plan['candidate_index'])
        except planner_module.PlanningBudgetExhausted as error:
            row.update(outcome='PLANNING_BUDGET_EXHAUSTED', where=error.where, evidence=error.evidence)
        except Exception as error:
            row.update(outcome='OTHER', error=str(error)[:600])
        row['returned_after_entry_s'] = time.monotonic()-entered
        row['returned_after_deadline_s'] = row['returned_after_entry_s']-value
        row['within_50_ms'] = row['returned_after_deadline_s'] <= .05 if row['outcome'] != 'PLAN_FOUND' else None
        rows.append(row)
        print(json.dumps({k: v for k, v in row.items() if k != 'evidence'}), flush=True)
    return rows


def search_mode(planner, model, candidates, deadline_s, out):
    box = synthetic_box(); failures = []; found = None
    world = [dict(id=box['id'], size=box['size_m'], pose=box['pose'], category='carton')]
    planner.client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                          log_path=out/'moveit.log')
    entered = time.monotonic()
    try:
        if deadline_s is None:
            planner.worker_ready(planner.client.request(planner.init), [box])
        else:
            planner.deadline, planner.deadline_reference = entered+deadline_s, time.monotonic()
            planner.worker_ready(planner.bounded_request(planner.client, planner.init, 120., None, 'main-init'), [box])
        planner.start_check_pool(out)
        try:
            found = (planner.search_box if deadline_s is None else planner.bounded_search)(
                np.asarray(Q_START), world, box, out, lambda _: None, lambda: False, failures, candidates)
            outcome = 'PLAN_FOUND' if found else 'NO_PLAN'
        except planner_module.PlanningBudgetExhausted as error:
            outcome = 'PLANNING_BUDGET_EXHAUSTED:'+error.where
    finally:
        if getattr(planner, 'deadline', None) is None:
            planner.close_check_pool()
            planner.client.close(); planner.client = None
        else:
            planner.background_close(out)
    result = dict(outcome=outcome, candidate=found['index'] if found else None,
                  segments=[dict(stage=s['stage'], points=len(s['prepared']['points']),
                                 send_duration_s=s['prepared']['send_duration_s'], planner=s['evidence']['planner'])
                            for s in found['segments']] if found else None,
                  failures=failures, elapsed_s=time.monotonic()-entered)
    (out/'search-result.json').write_text(json.dumps(result, indent=1, default=str))
    print(json.dumps({k: v for k, v in result.items() if k != 'failures'}, default=str))
    return result


def clock_mode(planner, model, out):
    box = synthetic_box()
    world = [dict(id=box['id'], size=box['size_m'], pose=box['pose'], category='carton')]
    from unloading_sim.workbench.single_carton.geometry import contact_candidates
    candidate = contact_candidates(box, planner.source['tool'], planner.site['topology'])[0]
    client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                  log_path=out/'moveit.log')
    results = {}
    try:
        client.request(planner.init)
        base = dict(planner.context(Q_START, world, box, candidate, 'approach'), op='validate', request_budget_s=30.)
        now = time.monotonic()
        cases = dict(accepted=dict(absolute_deadline_monotonic_s=now+20., deadline_clock_reference_monotonic_s=now),
                     reference_in_future=dict(absolute_deadline_monotonic_s=now+200., deadline_clock_reference_monotonic_s=now+100.),
                     deadline_long_past=dict(absolute_deadline_monotonic_s=now-120., deadline_clock_reference_monotonic_s=now-130.))
        for name, fields in cases.items():
            try:
                reply = client.request(dict(base, **fields), timeout=60)
                results[name] = dict(status='REPLIED', native_valid=reply.get('native_valid'),
                                     absolute=reply.get('budget', {}).get('absolute_deadline'))
            except Exception as error:
                results[name] = dict(status='REFUSED', detail=str(error))
                if client.process.poll() is not None:  # the client closes itself on errors
                    client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                                  log_path=out/'moveit.log')
                    client.request(planner.init)
    finally:
        client.close()
    print(json.dumps(results, indent=1))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=('deadline', 'search', 'clock'), required=True)
    parser.add_argument('--deadlines', default='0.5,3,6')
    parser.add_argument('--deadline-s', type=float)
    parser.add_argument('--candidates', default='0,1,2')
    parser.add_argument('--config', type=Path, default=ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    planner, model = make_planner(args.config)
    if args.mode == 'deadline':
        result = deadline_mode(planner, model, [float(x) for x in args.deadlines.split(',')], out)
    elif args.mode == 'search':
        result = search_mode(planner, model, {int(x) for x in args.candidates.split(',')}, args.deadline_s, out)
    else:
        result = clock_mode(planner, model, out)
    (out/'check.json').write_text(json.dumps(dict(scope='SYNTHETIC_SCENE_NOT_FIELD_DATA_NO_HARDWARE', mode=args.mode,
                                                  result=result), indent=1, default=str))


if __name__ == '__main__':
    main()
