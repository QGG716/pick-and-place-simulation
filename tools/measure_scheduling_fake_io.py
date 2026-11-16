"""Scheduling latency baseline: one complete single-carton cycle on fake IO.

Runs the real ``Cycle`` state machine, the real ``ExecutionService`` spawned
execution process with its real ``Supervisor``, ``Authority`` claim and record
writers, and the real ``SinglePlanner.binding()`` file rehash. Only the vendor
SDK transport (``RealmanTransport``) and the serial vacuum owner are replaced
by in-memory fakes. No hardware, SDK, serial port, network or valve is
touched; the result is ``FAKE_DEVICE_CYCLE_COMPLETE`` and never physical
acceptance.

Segment trajectories come from ``measurement-segments-NOT-EXECUTABLE.json``
written by an instrumented offline replay (``ECO65_TIMING=1``) or, when none
is given, a synthetic four-segment trajectory. Either way the timed send
uses the real 5 ms grid, so the fake run takes the real motion duration.
Requires ``ECO65_TIMING=1``.

Usage:
  ECO65_TIMING=1 python tools/measure_scheduling_fake_io.py --output DIR
      [--segments measurement-segments-NOT-EXECUTABLE.json] [--time-scale 10]
"""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--segments', type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--time-scale', type=float, default=1.,
                        help='speed-up of the fake motion only (send grid, monitor and dwell rules unchanged)')
    parser.add_argument('--binding-hash-cache', action='store_true', help='enable the step-4 binding hash cache')
    args = parser.parse_args()
    if os.environ.get('ECO65_TIMING') != '1':
        raise SystemExit('ECO65_TIMING=1 REQUIRED')
    sys.path.insert(0, str(ROOT/'tools'))
    os.environ['ECO65_FAKE_IO_MEASUREMENT'] = '1'
    from unloading_sim.workbench import timing
    from unloading_sim.workbench.controlled import service as service_module
    from unloading_sim.workbench.controlled.trajectory import Authority, prepare, Limits
    from unloading_sim.workbench.single_carton.config import load, start_joint_check
    from unloading_sim.workbench.single_carton.cycle import Cycle
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from unloading_sim.workbench.single_carton.runtime import Journal
    from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.contracts import HOST_SESSION
    import fake_io_transport

    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    config = load(args.config)
    if args.binding_hash_cache:
        config['binding_hash_cache'] = True
    execution = deepcopy(config['execution']); execution['record_root'] = str(out/'records')
    installation = Installation.load(execution['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    authority = Authority('eco65_single_carton_segment_v1')
    planner = SinglePlanner(config, load_mapping(execution['mapping']), authority, model)
    with timing.span('setup.planner_build'):
        planner.build()

    task_id = 'f'*32
    source = json.loads(args.segments.read_text()) if args.segments else None
    q_start = list(map(float, source['q_start'])) if source else [0., .2, -.4, 0., .3, 0.]
    if source:
        segments = []
        for s in source['segments']:
            prepared = s['prepared']
            if args.time_scale != 1.:
                prepared = prepare(prepared['source_points'], Limits(**prepared['limits']),
                                   max(1., prepared['time_scale']/args.time_scale), prepared['period_s'],
                                   prepared['collision_sampling']['reach_bound_m'])
            segments.append(dict(stage=s['stage'], prepared=prepared,
                                 evidence=dict(native_plan='PASS', sent_path_collision='PASS',
                                               source='OFFLINE_REPLAY_SEGMENT_MEASUREMENT_ONLY')))
    else:
        segments = synthetic_segments(planner, q_start)
    # Same binding structure as plan_cycle, stamped LIVE in this process's
    # session so the real submit path accepts it. It is fake-IO only.
    planner.cycle_binding = dict(task_id=task_id, observation=dict(
        stamp=dict(source='LIVE', host_session=HOST_SESSION), robot_sample=dict(source='LIVE')),
        target=dict(id='measurement-target'), candidate=None, attachment=None,
        release=dict(status='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'),
        expected_attachment='PLANNED_NOT_PHYSICALLY_CONFIRMED', scope='FAKE_IO_SCHEDULING_MEASUREMENT')
    start_policy = start_joint_check(config)
    if not start_policy['enabled']:
        segments[0]['prepared'] = dict(segments[0]['prepared'], start_joint_check=start_policy)

    # Fake transport is imported by the spawned child through this module path.
    fake_io_transport.configure(out/'fake-io', q_start, execution)
    service_module.RealmanTransport = fake_io_transport.FakeTransport
    service = fake_io_transport.SpawnedService(execution, load_mapping(execution['mapping']), authority, planner.binding)
    vacuum = fake_io_transport.FakeVacuum(config['vacuum'], out/'fake-io')
    (out/'cycle').mkdir()
    journal = Journal(out/'cycle', task_id)
    try:
        # As in the workbench: connect first (connect revokes older permits), then plan/register.
        service.connect('fake-controller', 8080)
        fake_io_transport.wait_feedback(service)
        binding = planner.binding()
        packages = [dict(stage=s['stage'], package=authority.register(s['prepared'], binding, s['evidence'])) for s in segments]
        plan = dict(binding=binding, segments=packages, q_start=q_start, full_path_validated=True,
                    start_joint_check=start_policy, release=dict(status='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'),
                    candidate=None, target=dict(id='measurement-target'))
        timing.recorder.take()  # setup spans are not scheduling
        cycle = Cycle(service, vacuum, config['vacuum'], journal, lambda q, p: None, lambda q, p: None, physical_io=False)
        cycle.ready(task_id, plan)
        entered = timing.now()
        stop = threading.Event()

        def monitor():
            while not stop.wait(.02):
                cycle.tick()
                if cycle.state in {'COMPLETE', 'FAILED', 'CANCELLED', 'STOP_UNCONFIRMED'}:
                    return
        with timing.span('execute.binding'):
            planner.binding()
        with timing.span('execute.cycle_start'):
            cycle.start()
        ticker = threading.Thread(target=monitor, name='single-carton-supervisor'); ticker.start()
        ticker.join(sum(s['prepared']['send_duration_s'] for s in segments)+120)
        stop.set()
        state = cycle.status()
    finally:
        journal.close()
        vacuum.halt.set(); vacuum.dump()
        try:
            service.close()
        except Exception as error:
            print('close:', error)
    events, overhead = timing.recorder.take()
    handoffs = handoff_analysis(out, execution, config['vacuum'])
    submits = sorted(Path(execution['record_root']).rglob('submit-timing.json'))
    child = [json.loads(p.read_text()) for p in submits]
    first_send = fake_io_transport.first_sends(out/'fake-io')
    (out/'timing-scheduling.json').write_text(json.dumps(dict(
        schema='eco65_timing_v1', phase='scheduling_fake_io', clock='time.monotonic', entered_monotonic_s=entered,
        cycle_status=state, segment_source='OFFLINE_REPLAY_MEASUREMENT_SEGMENTS' if source else 'SYNTHETIC',
        time_scale_override=args.time_scale, measurement_overhead_s=overhead, events=events,
        child_submits=child, fake_io_first_sends=first_send, handoffs=handoffs, hardware_execution='NOT_RUN',
        physical_result='NOT_RUN', result=state.get('state')), default=str))
    print(json.dumps(dict(state=state.get('state'), physical_result=state.get('physical_result'),
                          events=len(events), child_submits=len(child), output=str(out)), indent=1))


def handoff_analysis(out, execution, vacuum_config):
    """Waits between segments, split into what the rules require and the rest.

    Motion end: the supervisor needs ``dwell_s`` (0.4 s) of stationary arrival
    feedback after the last send; anything beyond is feedback/poll latency.
    The cycle then notices SUCCEEDED on its next 20 ms tick and submits the
    next segment, whose first point is due two periods after acceptance.
    Pressure: the rule needs the first qualifying fresh heartbeat after the
    valve command plus ``*_dwell_s`` (1 s) of qualifying heartbeats; time
    before the first qualifying heartbeat depends on the (here fake) cadence.
    """
    from unloading_sim.workbench.controlled.engine import MonitorConfig
    dwell = MonitorConfig(**execution.get('monitor', {})).dwell_s
    stages = [json.loads(x) for x in (out/'cycle'/'events.jsonl').read_text().splitlines()]
    stages = [e for e in stages if e.get('kind') == 'stage']
    when = {}
    for e in stages:
        when.setdefault(e['state'], e['host_monotonic_s'])
    motions = []
    for path in sorted((Path(execution['record_root'])).rglob('events.jsonl')):
        rows = [json.loads(x) for x in path.read_text().splitlines()]
        accepted = next((r['monotonic_s'] for r in rows if r['kind'] == 'accepted'), None)
        sends = [r for r in rows if r['kind'] == 'send']
        result = next((r for r in rows if r['kind'] == 'result'), None)
        if accepted is None or not sends or result is None:
            continue
        motions.append(dict(task=path.parent.name, accepted_s=accepted, first_send_s=sends[0]['sent_s'],
                            last_send_s=sends[-1]['sent_s'], result_s=result['monotonic_s'], state=result['state'],
                            sends=len(sends)))
    motions.sort(key=lambda m: m['accepted_s'])
    names = ('APPROACH', 'CONTACT', 'DETACH', 'RETURN_LOADED')
    after = {'APPROACH': 'CONTACT', 'CONTACT': 'ESTABLISH_VACUUM', 'DETACH': 'RETURN_LOADED', 'RETURN_LOADED': 'RELEASE'}
    segments = []
    for name, m in zip(names, motions):
        entered = when.get(name); noticed = when.get(after[name])
        segments.append(dict(stage=name, cycle_state_entered_s=entered,
            state_entry_to_accept_s=None if entered is None else m['accepted_s']-entered,
            accept_to_first_send_s=m['first_send_s']-m['accepted_s'],
            motion_send_s=m['last_send_s']-m['first_send_s'],
            last_send_to_succeeded_s=m['result_s']-m['last_send_s'], rule_arrival_dwell_s=dwell,
            beyond_arrival_dwell_s=m['result_s']-m['last_send_s']-dwell,
            succeeded_to_cycle_transition_s=None if noticed is None else noticed-m['result_s'],
            sends=m['sends'], engine_result=m['state']))
    vac = json.loads((out/'fake-io'/'vacuum.json').read_text())
    beats = vac['heartbeats']; pressure = []
    for start, end, key, met in (('ESTABLISH_VACUUM', 'ATTACHMENT_CONFIRMED', 'grip', lambda k: k <= vacuum_config['grip_below_kpa']),
                                 ('RELEASE', 'COMPLETE', 'release', lambda k: k >= vacuum_config['release_above_kpa'])):
        t0, t1 = when.get(start), when.get(end)
        if t0 is None or t1 is None:
            continue
        first = next((t for t, k in beats if t > t0 and met(k)), None)
        rule = vacuum_config[key+'_dwell_s']
        pressure.append(dict(stage=start, entered_s=t0, confirmed_s=t1, total_s=t1-t0,
            to_first_qualifying_heartbeat_s=None if first is None else first-t0, rule_dwell_s=rule,
            beyond_rule_after_first_qualifying_s=None if first is None else t1-first-rule,
            heartbeat_period_s=vac['heartbeat_period_s'], heartbeat_basis=vac['basis']))
    detach_start = next((m for n, m in zip(names, motions) if n == 'DETACH'), None)
    if detach_start and when.get('ATTACHMENT_CONFIRMED') is not None:
        pressure.append(dict(stage='ATTACHMENT_CONFIRMED_TO_DETACH_FIRST_SEND',
                             seconds=detach_start['first_send_s']-when['ATTACHMENT_CONFIRMED']))
    return dict(motion_segments=segments, pressure=pressure, monitor_dwell_s=dwell,
                note='Fake ideal follower: physical settling time is zero here; field values add real settling.')


def synthetic_segments(planner, q_start):
    """Four small native-shaped segments; used only when no replay segments exist."""
    from unloading_sim.workbench.controlled.trajectory import prepare
    import numpy as np
    out = []; q = np.asarray(q_start, float)
    for i, stage in enumerate(('approach', 'contact', 'detach', 'return')):
        goal = q_start if stage == 'return' else (np.asarray(q_start)+.02*(i+1)).tolist()
        t = np.linspace(0, 1, 11)*1.5; s = 10*t**3/1.5**3-15*t**4/1.5**4+6*t**5/1.5**5
        ds = (30*t**2/1.5**3-60*t**3/1.5**4+30*t**4/1.5**5); dds = (60*t/1.5**3-180*t**2/1.5**4+120*t**3/1.5**5)
        a, b = q, np.asarray(goal, float)
        points = [dict(t=float(tt), q=(a+(b-a)*ss).tolist(), v=((b-a)*d).tolist(), a=((b-a)*dd).tolist())
                  for tt, ss, d, dd in zip(t, s, ds, dds)]
        out.append(dict(stage=stage, prepared=prepare(points, planner.limits, 1., .005, planner.reach_bound),
                        evidence=dict(native_plan='PASS', sent_path_collision='PASS', source='SYNTHETIC_MEASUREMENT_ONLY')))
        q = b
    return out


if __name__ == '__main__':
    main()
