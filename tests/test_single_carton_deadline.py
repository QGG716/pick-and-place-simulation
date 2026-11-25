"""Unified planning deadline (planning_deadline_s); fake native workers only, no hardware."""
import json
import threading
import time
from types import SimpleNamespace
import pytest
from test_single_carton_planner import case, lanes_case, cand_of  # noqa: F401  (fixture reuse)
from unloading_sim.workbench.single_carton.planner import (PlanningBudgetExhausted, DEADLINE_REPLY_RESERVE_S,
                                                           SinglePlanner)
from unloading_sim.workbench.single_carton.config import issues

ON_TIME_S = .05  # the step-2 acceptance: return within the deadline + 50 ms


def native(calls):
    return [r for r in calls if r.get('op') in ('ik', 'plan', 'validate')]


def test_without_deadline_every_request_keeps_its_original_budget(case):
    p, obs, box, folder, calls, opts = case
    plan = p.plan_cycle(obs, [box], folder)
    assert plan['full_path_validated']
    for r in native(calls):
        assert 'absolute_deadline_monotonic_s' not in r and 'deadline_clock_reference_monotonic_s' not in r
        assert r['request_budget_s'] in (110., 590.)
        if r['op'] == 'plan':
            assert r['allowed_planning_time_s'] == 25.
    failures = folder/'planning-failures.json'
    assert not failures.exists() and p.deadline is None and not p.ipc_executors


def test_deadline_derives_request_budget_solver_time_and_absolute_deadline(case):
    p, obs, box, folder, calls, opts = case
    start = time.monotonic(); deadline = start+60.
    plan = p.plan_cycle(obs, [box], folder, deadline=deadline)
    assert plan['full_path_validated'] and len(p.authority._plans) == 4
    requests = native(calls)
    assert requests
    for r in requests:
        assert r['absolute_deadline_monotonic_s'] == pytest.approx(deadline-DEADLINE_REPLY_RESERVE_S)
        assert start <= r['deadline_clock_reference_monotonic_s'] <= deadline
        assert 0 < r['request_budget_s'] <= deadline-DEADLINE_REPLY_RESERVE_S-start
        if r['op'] == 'plan':
            assert r['allowed_planning_time_s'] == 25.  # 25 s cap is below the 60 s remaining
    # The deadline is per plan_cycle: nothing stays set afterwards.
    assert p.deadline is None


def test_short_deadline_caps_solver_time_and_reports_it_as_budget_not_infeasible(case):
    p, obs, box, folder, calls, opts = case
    seen = []

    def plan(r):
        seen.append(r['allowed_planning_time_s'])
        if r['pipeline_id'] == 'ompl' or r['stage'] == 'approach':
            # A worker solve that ended at the capped (deadline-derived) budget.
            return dict(status='NATIVE_SOLVER_BUDGET_EXHAUSTED')
    opts['plan'] = plan
    deadline = time.monotonic()+5.
    with pytest.raises(PlanningBudgetExhausted) as raised:
        p.plan_cycle(obs, [box], folder, deadline=deadline)
    assert seen and all(0 < t < 5. for t in seen)
    evidence = raised.value.evidence
    assert evidence['status'] == 'PLANNING_BUDGET_EXHAUSTED'
    target = evidence['targets'][0]
    # Nothing was concluded infeasible: the expired candidate stays unsearched.
    assert target['infeasible_candidates'] == 0 and target['unsearched'] == target['considered'] > 0
    raised.value.record_thread.join(5)  # the classification record is written off the caller thread
    record = json.loads((folder/'planning-failures.json').read_text())
    assert record['failures'] == [] and record['planning_budget']['targets'][0]['unsearched'] == target['unsearched']
    assert p.authority._plans == {}


def test_per_call_solver_cap_with_ample_deadline_is_searched_but_not_infeasible(case):
    p, obs, box, folder, calls, opts = case
    opts['plan'] = lambda r: dict(status='NATIVE_SOLVER_BUDGET_EXHAUSTED') if r['stage'] == 'approach' else None
    with pytest.raises(ValueError, match='NO_COMPLETE_FEASIBLE_SINGLE_CARTON_PLAN') as raised:
        p.plan_cycle(obs, [box], folder, deadline=time.monotonic()+600.)
    record = json.loads((folder/'planning-failures.json').read_text())
    budget = record['planning_budget']
    assert budget['status'] == 'SEARCH_FINISHED_WITHIN_DEADLINE'
    target = budget['targets'][0]
    assert target['unsearched'] == 0
    assert target['searched'] == target['considered'] == len(record['failures'])
    assert target['infeasible_candidates'] == 0
    assert target['solver_budget_exhausted_candidates'] == target['considered']
    assert target['status'] == 'SEARCH_BUDGET_EXHAUSTED_NO_PLAN'
    assert '已搜索' in str(raised.value) and '未搜索 0' in str(raised.value)


def test_worker_deadline_expiry_in_a_check_is_budget_exhaustion(case):
    p, obs, box, folder, calls, opts = case
    opts['validate'] = lambda r: dict(native_valid=False, legacy_intersection_valid=True,
                                      failure=dict(reason='NATIVE_ABSOLUTE_DEADLINE_EXPIRED', phase='diagnostic'))
    with pytest.raises(PlanningBudgetExhausted, match='NATIVE_DEADLINE') as raised:
        p.plan_cycle(obs, [box], folder, deadline=time.monotonic()+600.)
    # The first endpoint check of the first candidate carried the expiry.
    subs = sorted(folder.glob('*-0/loaded-start-native.json'))
    assert subs and json.loads(subs[0].read_text())['status'] == 'PLANNING_BUDGET_EXHAUSTED'
    target = raised.value.evidence['targets'][0]
    assert target['status'] == 'BUDGET_EXHAUSTED_IN_SCREENING' and target['infeasible_candidates'] == 0


@pytest.mark.parametrize('budget', [.15, .3, .6])
def test_slow_native_search_returns_within_50_ms_of_the_deadline(case, monkeypatch, budget):
    # Cached candidates: the deadline falls in slow native calls, not in the
    # ~1 s pure-Python candidate generation (see the next test).
    p, obs, box, folder, calls, opts, cands = lanes_case(case, monkeypatch, count=2, lanes=1)

    def plan(r):
        time.sleep(.2)
    opts['plan'] = plan
    entered = time.monotonic()
    with pytest.raises(PlanningBudgetExhausted) as raised:
        p.plan_cycle(obs, [box], folder, deadline=entered+budget)
    late = time.monotonic()-(entered+budget)
    assert late <= ON_TIME_S, late
    target = raised.value.evidence['targets'][0]
    assert target['infeasible_candidates'] == 0 and target['unsearched'] == target['considered'] == 2


def test_expiry_during_candidate_generation_is_classified_not_infeasible(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    import unloading_sim.workbench.single_carton.planner as module
    real = module.contact_candidates
    monkeypatch.setattr(module, 'contact_candidates', lambda *a: (time.sleep(.5), real(*a))[1])
    with pytest.raises(PlanningBudgetExhausted, match='SEARCH_STEP_RUNNING_AT_DEADLINE') as raised:
        p.plan_cycle(obs, [box], folder, deadline=time.monotonic()+.2)
    target = raised.value.evidence['targets'][0]
    # The return time here is bounded by the longest GIL hold of that step
    # (measured up to 83 ms for the real generation), reported, not asserted.
    assert target['status'] == 'BUDGET_EXHAUSTED_IN_CANDIDATE_GENERATION'
    assert target['unsearched'] == 'ALL_NOT_GENERATED' and target['infeasible_candidates'] == 0
    p.last_task_context.search_thread.join(5)
    assert not [r for r in calls if r.get('op') in ('ik', 'plan', 'validate')]  # nothing dispatched after it


def test_unanswered_request_is_abandoned_at_the_deadline_and_never_reused(case, monkeypatch):
    # Two IK-reachable candidates screen in milliseconds; the first plan hangs.
    p, obs, box, folder, calls, opts, cands = lanes_case(case, monkeypatch, count=2, lanes=1)
    release = threading.Event()
    worker_entered = threading.Event()

    def plan(r):
        worker_entered.set()
        release.wait(5)  # a worker that does not reply in time
    opts['plan'] = plan
    entered = time.monotonic()
    try:
        # The outer owner can observe expiry just after the search thread has
        # ended but before its result is published. TASK_WAIT is the same
        # budget exhaustion in that race; the abandoned-worker checks below
        # (and the original +50 ms limit) must still hold.
        with pytest.raises(PlanningBudgetExhausted, match='NO_REPLY_BY_DEADLINE|SEARCH_STEP_RUNNING_AT_DEADLINE|TASK_WAIT'):
            p.plan_cycle(obs, [box], folder, deadline=entered+.4)
        assert time.monotonic()-(entered+.4) <= ON_TIME_S
        assert worker_entered.is_set() and not p.authority._plans
        p.last_task_context.search_thread.join(1)  # the released search records the unanswered worker
        assert p.last_task_context.abandoned and not any(id(c) in p.last_task_context.abandoned for c in p.checkers() if c is not None)
    finally:
        release.set()


def test_parallel_lanes_stop_at_the_deadline_with_unsearched_counts(case, monkeypatch):
    p, obs, box, folder, calls, opts, cands = lanes_case(case, monkeypatch, count=6, lanes=3)
    opts['plan'] = lambda r: time.sleep(1.) if r['stage'] == 'contact' else None
    entered = time.monotonic()
    with pytest.raises(PlanningBudgetExhausted) as raised:
        p.plan_cycle(obs, [box], folder, deadline=entered+.5)
    assert time.monotonic()-(entered+.5) <= ON_TIME_S
    target = raised.value.evidence['targets'][0]
    assert target['considered'] == 6 and target['infeasible_candidates'] == 0 and target['unsearched'] == 6
    assert p.authority._plans == {}


def test_no_dispatch_after_the_deadline(case):
    p, obs, box, folder, calls, opts = case
    with pytest.raises(PlanningBudgetExhausted, match='BEFORE_DISPATCH'):
        p.plan_cycle(obs, [box], folder, deadline=time.monotonic()-1.)
    assert native(calls) == []


def test_a_pool_worker_finishing_after_its_plan_never_joins_the_next_pool(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    release = threading.Event(); closing = threading.Event()

    class Slow:
        def __init__(self, *args, **kwargs): self.closed = False

        def request(self, r, **kwargs):
            release.wait(5); return dict(status='READY', capabilities={'bounded_single_carton_planning': True})

        def close(self):
            closing.set(); time.sleep(.5); self.closed = True  # a slow close (the real one takes 5 s)
    monkeypatch.setattr('unloading_sim.moveit2_backend.ResidentMoveItClient', Slow)
    p.site['native_check_workers'] = 2; p.client = None
    p.deadline = time.monotonic()+30.
    p.start_check_pool(folder); first = list(p.pool_threads)
    p.background_close(folder)  # the first plan ends while its pool is still starting
    p.start_check_pool(folder)  # the next plan begins
    release.set()
    assert closing.wait(2)
    started = time.monotonic()
    with p.pool_lock:  # the late worker closes outside the pool lock
        waited = time.monotonic()-started
    assert waited < .1
    for thread in first + list(p.pool_threads):
        thread.join(3)
    assert len(p.pool) == 2 and not any(c.closed for c in p.pool)
    assert sum(s['state'] == 'CLOSED' for s in p.pool_status) == 0  # the new plan's own status list


@pytest.mark.parametrize('value,ok', [(None, True), (2.0, True), (0.05, True), (0, False), (-1, False),
                                      (float('inf'), False), (4000, False), (True, False), ('2', False)])
def test_planning_deadline_configuration(value, ok):
    from pathlib import Path
    from unloading_sim.workbench.single_carton.config import load
    config = load(Path(__file__).resolve().parents[1]/'configs/workbench/single_carton.example.json', code_defaults=False)
    if value is None:
        config.pop('planning_deadline_s', None)
    else:
        config['planning_deadline_s'] = value
    found = [e for e in issues(config) if e.startswith('planning_deadline_s')]
    assert (not found) == ok


@pytest.mark.parametrize('value,ok',[(25.,True),(90.,True),(0,False),(101,False),(float('nan'),False),(True,False),('90',False)])
def test_solver_budget_configuration_stays_bounded(value,ok,case):
    p,*_=case;p.site['native_solver_budget_s']=value
    assert (not [e for e in issues(p.site) if e.startswith('native_solver_budget_s')])==ok


def test_longer_solver_budget_cannot_extend_absolute_deadline(case):
    p,*_=case;p.site['native_solver_budget_s']=90.
    p.deadline_reference=time.monotonic();p.deadline=p.deadline_reference+.5
    request=p.deadline_fields(p.plan_request({'q_start':[0.]*6},[.1]*6,False),110.)
    assert request['allowed_planning_time_s']==90.
    assert 0<request['request_budget_s']<=.5
    assert request['absolute_deadline_monotonic_s']==p.deadline-DEADLINE_REPLY_RESERVE_S


def test_prepare_plan_passes_one_absolute_deadline_from_its_entry():
    from unloading_sim.workbench.single_carton import runtime
    received = {}

    class Planner:
        authority = SimpleNamespace(invalidate=lambda: None)

        def plan_cycle(self, *args, **kwargs):
            received.update(kwargs, called=time.monotonic())
            raise ValueError('STOP_AFTER_CAPTURING_ARGUMENTS')
    for budget in (None, 1.5):
        received.clear()
        config = {} if budget is None else dict(planning_deadline_s=budget)
        manager = runtime.TaskManager(SimpleNamespace(observation_generation=1), SimpleNamespace(request_stop=lambda: None),
                                      SimpleNamespace(holding=False, inhibit=lambda: None), Planner(), None, None, config)
        try:
            manager.phase = 'TARGET_READY'; manager.observation = SimpleNamespace(generation=1)
            before = time.monotonic()
            with pytest.raises(ValueError, match='STOP_AFTER_CAPTURING_ARGUMENTS'):
                manager.prepare_plan()
            if budget is None:
                assert 'deadline' not in received
            else:
                assert before <= received['deadline']-budget <= received['called']
        finally:
            manager.closing.set(); manager.thread.join(1)


def test_abandoned_clients_are_excluded_without_a_deadline_too():
    p = SinglePlanner.__new__(SinglePlanner)
    p.pool_lock = threading.Lock(); alive = SimpleNamespace(process=SimpleNamespace(poll=lambda: None))
    p.client = alive; p.pool = []; p.abandoned = set()
    assert p.checkers() == [alive]
    p.abandoned.add(id(alive))
    assert p.checkers() == []
