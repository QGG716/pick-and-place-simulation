"""Real RRT exits through production dispatch; only interruption/IK scheduling injected."""
from dataclasses import replace
from types import SimpleNamespace
from time import perf_counter

import numpy as np
import pytest

from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.stage_motion_policy import failure_status, interrupts_generation
from test_stage_motion_policy import connector, obstacle, START, GOAL, detour


def transit(c, obstacles, iterations=1, **kwargs):
    return c._transit(START, GOAL, obstacles, purpose='FREE_APPROACH',
        stage='pregrasp', seed=44, iteration_budget=iterations, **kwargs)


def pose(c, obstacles):
    return c._connect_pose(c.robot.fk(GOAL), [START], START, obstacles,
        purpose='FREE_APPROACH', stage='pregrasp', ik_seed=3, connection_seed=44)


def stream(monkeypatch, c, goals):
    class Stream:
        index = 0
        def __next__(self):
            if self.index == len(goals):
                raise StopIteration
            q = goals[self.index]; self.index += 1
            return SimpleNamespace(q=q, position_error=0., orientation_error=0., search_evidence={})
        def evidence(self):
            return dict(seeds_attempted=self.index)
    monkeypatch.setattr(c, '_ik_stream', lambda *a, **k: Stream())


@pytest.mark.parametrize('entry', ['transit', 'pose'])
def test_real_small_rrt_quota_is_uncertain_not_unreachable(entry, monkeypatch):
    c = connector(); obstacles = [obstacle()]
    c.budget = replace(c.budget, stage_connection_iterations=1)
    if entry == 'pose':
        stream(monkeypatch, c, [GOAL])
        _, path, failure, trace = pose(c, obstacles)
        search = trace['attempts'][-1]['connection']
    else:
        path, failure, trace = transit(c, obstacles)
        search = trace
    assert not path
    assert failure_status(failure) == trace['validation_status'] == 'INDETERMINATE'
    assert failure['reason'] == 'MAXIMUM_ITERATIONS_REACHED'
    assert failure['validation']['termination_scope'] == 'CANDIDATE'
    assert not interrupts_generation(failure)
    assert search['direct_edge_validation']['status'] == 'INVALID'
    assert search['extension_attempts'] > 0


@pytest.mark.parametrize('kind', ['cancel', 'work', 'deadline', 'context'])
@pytest.mark.parametrize('entry', ['transit', 'pose'])
def test_interrupt_inside_real_rrt_reaches_dispatch(kind, entry, monkeypatch):
    c = connector(); obstacles = [obstacle()]; planners = []
    original = RRTConnectPlanner._extend
    def inject(self, *args, **kwargs):
        if not planners:
            planners.append(self)
            # Before the real extension's budget check, with no validation failure.
            assert self._validation_stop_result is None
            if kind == 'cancel': c.cancel_requested = True
            elif kind == 'work': c.validation_budget.max_checks = c.validation_budget.checks
            elif kind == 'deadline': c.validation_budget.deadline = perf_counter()-1
            else: obstacles[0].center[1] += .001
        return original(self, *args, **kwargs)
    monkeypatch.setattr(RRTConnectPlanner, '_extend', inject)
    if entry == 'pose':
        stream(monkeypatch, c, [GOAL, GOAL + np.array([0, 0, 0, 0, 0, .1])])
        _, path, failure, trace = pose(c, obstacles)
        assert len([a for a in trace['attempts'] if a['pass'] == 'CANDIDATES_THEN_RRT']) == 1
    else:
        path, failure, trace = transit(c, obstacles, iterations=200)
    assert planners and not path
    status = 'CANCELLED' if kind == 'cancel' else 'INDETERMINATE'
    assert failure_status(failure) == trace['validation_status'] == status
    assert interrupts_generation(failure)
    assert {'cancel': 'CANCEL', 'work': 'WORK_BUDGET', 'deadline': 'DEADLINE',
            'context': 'CONTEXT_CHANGED'}[kind] in failure['reason']
    assert not planners[0]._rejected_candidates
    if kind == 'cancel': assert planners[0]._validation_stop_result is None


def test_candidate_quota_leaves_second_candidate_opportunity(monkeypatch):
    c = connector(); obstacles = [obstacle()]
    c.budget = replace(c.budget, stage_connection_iterations=2)
    stream(monkeypatch, c, [GOAL, GOAL + np.array([0, 0, 0, 0, 0, .1])])
    _, path, failure, trace = pose(c, obstacles)
    attempts = [a for a in trace['attempts'] if a['pass'] == 'CANDIDATES_THEN_RRT']
    assert len(attempts) == 2
    assert failure_status(attempts[0]['failure']) == 'INDETERMINATE'
    assert sum(a['connection']['planning_iterations_consumed'] for a in attempts) <= 2
    assert trace['validation_status'] == ('VALID' if path else 'INDETERMINATE')


@pytest.mark.parametrize('failure', [dict(status='CANCELLED', reason='producer stopped'),
    dict(validation_status='INDETERMINATE', reason='producer incomplete'),
    dict(validation=dict(status='CANCELLED'), reason='producer stopped')])
def test_legacy_candidate_status_is_preserved(failure, monkeypatch):
    c = connector()
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('request stopped'))
    path, actual, trace = transit(c, [obstacle()], candidates=[
        ('HISTORY_HINT', lambda: ([], failure, {}))])
    assert not path and failure_status(actual) == failure_status(failure)
    assert trace['validation_status'] == failure_status(failure)


def test_geometry_rejection_and_real_rrt_success_remain_distinct():
    c = connector(); blocked = obstacle(); blocked.center[:] = START[:3]
    _, failure, trace = transit(c, [blocked])
    assert failure_status(failure) == 'INVALID' and not trace['rrt_called']
    c = connector(); path, failure, trace = transit(c, [obstacle()], iterations=200)
    assert path and failure is None and trace['validation_status'] == 'VALID'
    assert trace['extension_attempts'] > 0


def test_progress_boundaries_and_checkpoint_rate():
    c = connector(); records = []; c.progress_callback = records.append
    path, failure, _ = transit(c, [])
    assert path and failure is None
    assert records[0]['event'] == 'stage_enter'
    assert records[-1]['event'] == 'stage_exit' and records[-1]['success']
    for _ in range(100): c._progress('work_checkpoint')
    assert sum(r['event'] == 'work_checkpoint' for r in records) <= 1


def test_context_change_at_rrt_publication_rejects_completed_result(monkeypatch):
    c = connector(); obstacles = [obstacle()]
    original = RRTConnectPlanner._accept_candidate
    def mutate_after_validation(self, path):
        valid = original(self, path)
        if valid: obstacles[0].center[1] += .001
        return valid
    monkeypatch.setattr(RRTConnectPlanner, '_accept_candidate', mutate_after_validation)
    path, failure, trace = transit(c, obstacles, iterations=200)
    assert not path and trace['validation_status'] == 'INDETERMINATE'
    assert failure['reason'] == 'VALIDATION_CONTEXT_CHANGED'
    assert interrupts_generation(failure)


def test_legacy_local_sample_quota_does_not_cancel_other_candidates():
    c = connector()
    path, failure, trace = transit(c, [obstacle()], candidates=[
        ('LOCAL_CARTESIAN_CANDIDATE', lambda: ([], dict(reason='CARTESIAN_SAMPLE_BUDGET_EXCEEDED'), {})),
        ('HISTORY_HINT', lambda: (detour(), None, {}))])
    assert path and failure is None and trace['selected_method'] == 'HISTORY_HINT'
    assert trace['attempts'][-2]['status'] == 'INDETERMINATE'
    assert not trace['rrt_called']


def test_final_connector_result_preserves_real_rrt_cancel(monkeypatch):
    # Isolate the outer branch scheduler; its failure comes from a real RRT run.
    c = connector(); obstacles = [obstacle()]; calls = []
    original = RRTConnectPlanner._extend
    def cancel(self, *a, **k):
        c.cancel_requested = True
        return original(self, *a, **k)
    monkeypatch.setattr(RRTConnectPlanner, '_extend', cancel)
    def branch(**kwargs):
        calls.append(1)
        path, failure, trace = transit(c, obstacles, iterations=200)
        assert not path
        return None, failure, trace
    monkeypatch.setattr(c, '_plan_branch', branch)
    result = c.plan(target=obstacle(), face='front', requested_virtual_contact=np.eye(4),
        grasp_candidates=[dict(q_rad=GOAL.tolist())]*2, home_q=START,
        all_obstacles=obstacles, receiver=obstacle(), support_names=(), suction={}, seed=44)
    assert len(calls) == 1 and not result.success
    assert result.failure['validation']['status'] == result.statistics['validation_status'] == 'CANCELLED'
    assert 'CANCEL' in result.statistics['termination']
