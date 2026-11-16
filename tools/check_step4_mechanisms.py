"""Step-4 mechanism check on SYNTHETIC reachable scenes (no device, SDK, valve or field record).

Two synthetic scenes (box A, box B) with q_start a small joint offset from a
native IK solution of candidate 0's pre-contact pose, so complete four-segment
plans exist. plan_cycle runs exactly as in the workbench (synthetic LIVE stamp
in this process's session; the permits die with this process).

  --mode off        default config (every step-4 switch off): A and B, each in a fresh planner
  --mode on         every step-4 switch on, one resident planner: A, A, B; then B in a fresh resident planner
  --mode compare    compare an off run with an on run (decisions, candidates, planners, points, durations, failures)

The outcome summary of each plan is written to <output>/<run>.json.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))

#: Synthetic scenes: box centre (m) and q_start (rad). A: q_start is the native
#: IK solution (seed set of the planner) of candidate 0's contact pose raised
#: 0.15 m, found with tools in this step; a complete four-segment plan exists.
#: B: same box, q_start 0.2 m above (all candidates rejected at detach), so A
#: and B differ in both scene history and outcome.
SCENES = {'A': ([.35, 0., .15], [2.9228, 0.0566, -0.9514, -0.676, 1.5708, 4.4958]),
          'B': ([.35, 0., .15], [2.9228, -0.177, -0.3793, -1.0145, 1.5708, 4.4958])}
Q_OFFSET = np.zeros(6)
CLIENTS = []  # every worker client created in this process (round trips and pipe bytes)


def count_clients():
    """Register every ResidentMoveItClient this process creates (measurement only)."""
    import unloading_sim.moveit2_backend as backend
    original = backend.ResidentMoveItClient
    if getattr(original, 'registered', False):
        return

    class Registered(original):
        registered = True

        def __init__(self, *a, **k):
            super().__init__(*a, **k); CLIENTS.append(self)
    backend.ResidentMoveItClient = Registered
SWITCHES = ('resident_workers', 'scene_handles', 'async_evidence', 'path_endpoint_reuse', 'binding_hash_cache')


def box_of(name):
    from unloading_sim.workbench.single_carton.geometry import corners
    xyz = SCENES[name][0]; pose = np.eye(4); pose[:3, 3] = xyz
    return dict(id=f'synthetic-{name}', detection_index=0, pose=pose.tolist(), size_m=[.15, .1, .1],
                corners_m=corners(pose, [.15, .1, .1]).tolist(),
                faces=[dict(observed=True, center_m=[xyz[0], xyz[1], xyz[2]+.05], normal=[0, 0, 1], u=[1, 0, 0], v=[0, 1, 0], size_m=[.15, .1])],
                quality={}, scope='SYNTHETIC_STEP4_CHECK_NOT_FIELD_DATA')


def planner_for(switches, record_root, check_mode=None, audit=None, module=None):
    from unloading_sim.workbench.single_carton.config import load
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    config = load(ROOT/'configs/workbench/single_carton.example.json')
    config.pop('site', None); config['record_root'] = str(record_root)
    if module:
        config['planning_module'] = module
    if audit:
        config['sent_path_audit'] = audit
    if check_mode:
        config['collision_check_mode'] = check_mode
    for key in SWITCHES:
        config[key] = bool(switches)
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    return planner, model


def observation(model, name):
    from unloading_sim.workbench.contracts import Stamp
    q = np.asarray(SCENES[name][1])+Q_OFFSET
    return SimpleNamespace(q_start=tuple(q), world_tcp_start=model.tcp(q).tolist(),
                           frame=SimpleNamespace(stamp=Stamp.receive(), observation_id=f'synthetic-{name}'),
                           robot_sample={'source': 'LIVE'}, metadata=lambda: dict(source='SYNTHETIC_STEP4_CHECK'))


def summary(plan, error, elapsed, folder):
    failures = []
    pf = folder/'planning-failures.json'
    if pf.exists():
        data = json.loads(pf.read_text()); failures = data['failures'] if isinstance(data, dict) else data
    out = dict(elapsed_s=elapsed, outcome='PLAN_READY' if plan else 'NO_PLAN', error=None if error is None else str(error)[:300],
               failures=[dict(candidate=f.get('candidate'), reason=f['reason']) for f in failures])
    if plan:
        out.update(candidate=plan['candidate_index'], endpoint_screen=plan['endpoint_screen'], lanes=plan['candidate_lanes'],
                   duration_s=plan['duration_s'], native_output_check=plan['native_output_check'],
                   segments=[dict(stage=s['stage'], planner=s['package']['evidence']['planner'],
                                  points=len(s['package']['prepared']['points']),
                                  send_duration_s=s['package']['prepared']['send_duration_s'],
                                  probe_states=s['package']['evidence']['collision_result']['probe_states'],
                                  detach_profile=s['package']['evidence'].get('detach_profile'),
                                  first_q=s['package']['prepared']['points'][0]['q'],
                                  last_q=s['package']['prepared']['points'][-1]['q'])
                             for s in plan['segments']],
                   binding_hash=None if 'binding_hash_evidence' not in plan else
                       {k: plan['binding_hash_evidence'][k] for k in ('reused', 'rehashed', 'seconds')},
                   resident_task_start=plan.get('resident_task_start'))
    return out


def run(planner, model, name, out, label):
    from unloading_sim.workbench import timing
    folder = out/label; folder.mkdir(); first_client = len(CLIENTS)
    started = time.monotonic(); plan = error = None
    try:
        with timing.span('prepare_plan.plan_cycle', scope='SYNTHETIC_PLAN_CYCLE_ONLY'):
            plan = planner.plan_cycle(observation(model, name), [box_of(name)], folder)
    except Exception as e:
        error = e
    # ECO65_TIMING=1: this plan's spans for tools/bench_planning_latency.py --timing
    # (plan_cycle only; no observation, scene build or UI of the workbench entry).
    writer = timing.write_later(folder/'timing-prepare_plan.json', 'prepare_plan',
                                dict(task_id=f'synthetic-{label}', final_phase='PLAN_READY' if plan else 'NO_PLAN',
                                     scope='SYNTHETIC_SCENE_NOT_FIELD_RECORD'), since=started)
    if writer is not None:
        writer.join()
    result = summary(plan, error, time.monotonic()-started, folder)
    clients = CLIENTS[first_client:]
    result['transport'] = dict(clients=len(clients), round_trips=sum(c.round_trips for c in clients),
                               sent_bytes=sum(c.transfer_bytes()[0] for c in clients),
                               received_bytes=sum(c.transfer_bytes()[1] for c in clients),
                               binary_frames=sum(c.binary_active for c in clients),
                               composites=len(list(folder.rglob('*-composite.json'))))
    (out/f'{label}.json').write_text(json.dumps(result, indent=1, default=str))
    print(label, json.dumps({k: result.get(k) for k in ('outcome', 'candidate', 'elapsed_s', 'error')}, default=str), flush=True)
    return result


def comparable(result):
    keep = dict(outcome=result['outcome'], candidate=result.get('candidate'), endpoint_screen=result.get('endpoint_screen'),
                duration_s=result.get('duration_s'), segments=result.get('segments'), failures=result['failures'])
    return json.loads(json.dumps(keep, default=str))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=('off', 'on', 'compare'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--off', type=Path); parser.add_argument('--on', type=Path)
    parser.add_argument('--collision-check-mode', choices=('EXACT', 'TWO_LEVEL'))
    parser.add_argument('--sent-path-audit', choices=('SAMPLED', 'ADAPTIVE'))
    parser.add_argument('--planning-module', choices=('PYTHON', 'NATIVE'))
    args = parser.parse_args()
    if args.mode == 'compare':
        pairs = [('off', 'A', args.off/'A.json', args.on/'A-first.json'), ('resident-repeat', 'A', args.on/'A-first.json', args.on/'A-second.json'),
                 ('off', 'B', args.off/'B.json', args.on/'B-after-A.json'), ('resident-history', 'B', args.on/'B-alone.json', args.on/'B-after-A.json')]
        rows = []
        for kind, scene, a, b in pairs:
            x, y = comparable(json.loads(a.read_text())), comparable(json.loads(b.read_text()))
            rows.append(dict(comparison=f'{kind}:{scene}', left=str(a.name), right=str(b.name), identical=x == y,
                             differences=None if x == y else [k for k in x if x[k] != y[k]]))
        args.output.write_text(json.dumps(rows, indent=1)); print(json.dumps(rows, indent=1)); return
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    if args.mode == 'off':
        for name in ('A', 'B'):
            count_clients()
            planner, model = planner_for(False, out/'records', args.collision_check_mode, args.sent_path_audit, args.planning_module)
            try: run(planner, model, name, out, name)
            finally: planner.close()
        return
    planner, model = planner_for(True, out/'records', args.collision_check_mode, args.sent_path_audit)
    try:
        planner.prewarm().join()
        planner.resident.wait_ready(300)
        for name, label in (('A', 'A-first'), ('A', 'A-second'), ('B', 'B-after-A')):
            run(planner, model, name, out, label)
    finally:
        planner.close()
    planner, model = planner_for(True, out/'records-fresh', args.collision_check_mode, args.sent_path_audit)
    try:
        planner.prewarm().join(); planner.resident.wait_ready(300)
        run(planner, model, 'B', out, 'B-alone')
    finally:
        planner.close()


if __name__ == '__main__':
    main()
