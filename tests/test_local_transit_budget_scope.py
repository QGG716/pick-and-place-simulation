"""Real local producer, dispatcher, attachment, validator and RRT on analytic robot.

These deterministic routes are regressions, not original FANUC success evidence.
"""
import numpy as np
import pytest

from test_stage_motion_policy import connector, START, GOAL, obstacle, detour, connect
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.motion_validation import Status
from unloading_sim.stage_motion_policy import GenerationMethod, interrupts_generation, failure_status


def loaded(c):
    return PhysicalContactAttachment(c.robot,
        RigidAttachment(np.eye(4), np.full(3, .01), 'payload'), np.eye(4), np.eye(4))


def local(c, attachment, obstacles, destination=None):
    # Exactly 121 requested samples, with unchanged 80/240 method allocation.
    if destination is None:
        destination = c.robot.fk(START).copy()
        destination[0, 3] += 120.5 * c.budget.cartesian_step_m
    return c._bounded_local_transit(START, destination, obstacles, attachment, seed=44)


def test_real_producer_121_80_240_zero_work_and_scope_through_wrappers():
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()]
    before = dict(c._statistics)
    path, failure, evidence = local(c, a, world)
    assert not path and failure['required'] == 121
    assert {k: failure[k] for k in ('allocated', 'consumed', 'candidate_remaining', 'shared_remaining')} == dict(
        allocated=80, consumed=0, candidate_remaining=80, shared_remaining=240)
    assert failure['termination_scope'] == 'CANDIDATE'
    assert c._statistics == before and c._local_transit_remaining == 240
    v = c._motion_validator(world, attachment=a, stage='transit')
    for original in (failure, dict(reason='SHARED_LOCAL_TRANSIT_SAMPLE_BUDGET', required=121)):
        wrapped = original
        for _ in range(3):
            assert failure_status(wrapped) == 'INDETERMINATE'
            assert not interrupts_generation(wrapped)
            result = v._result(Status.INDETERMINATE, wrapped)
            wrapped = dict(reason=original['reason'], validation=result.evidence())


@pytest.mark.parametrize('pool', [240, 0])
def test_real_shortage_then_real_loaded_rrt(pool):
    c = connector(); c._local_transit_remaining = pool
    a = loaded(c); world = [obstacle()]
    path, failure, trace = connect(c, world, attachment=a, candidates=[
        (GenerationMethod.LOCAL_CARTESIAN_CANDIDATE, lambda: local(c, a, world))])
    assert failure is None, trace
    assert trace['rrt_called'] and trace['extension_attempts'] > 0
    assert trace['selected_method'] == 'RRT_CONNECT'
    assert c._local_transit_remaining == pool
    attempt = next(x for x in trace['attempts'] if x['method'] == 'LOCAL_CARTESIAN_CANDIDATE')
    assert attempt['status'] == 'INDETERMINATE'
    assert attempt['failure']['termination_scope'] == ('METHOD' if pool == 0 else 'CANDIDATE')
    v = c._motion_validator(world, attachment=a, stage='transit')
    assert v.check_path(path).valid
    assert not v.check_motion(START, GOAL).valid


def test_shortage_gives_next_candidate_opportunity_without_rrt():
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()]; calls = []
    def next_candidate():
        calls.append(True)
        return detour(), None, {}
    path, failure, trace = connect(c, world, attachment=a, candidates=[
        (GenerationMethod.LOCAL_CARTESIAN_CANDIDATE, lambda: local(c, a, world)),
        (GenerationMethod.HISTORY_HINT, next_candidate)])
    assert path and failure is None and calls == [True] and not trace['rrt_called']


@pytest.mark.parametrize('stop', ['cancel', 'context', 'work', 'deadline'])
def test_request_stop_during_shortage_wins(stop, monkeypatch):
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()]
    def produce():
        result = local(c, a, world)
        if stop == 'cancel': c.cancel_requested = True
        elif stop == 'context': world[0].center[1] += .1
        elif stop == 'work': c.validation_budget.max_checks = c.validation_budget.checks
        else: c.validation_budget.deadline = 0.
        return result
    monkeypatch.setattr(c, '_rrt_transit', lambda *a, **k: pytest.fail('request stopped'))
    path, failure, trace = connect(c, world, attachment=a, candidates=[
        (GenerationMethod.LOCAL_CARTESIAN_CANDIDATE, produce),
        (GenerationMethod.HISTORY_HINT, lambda: pytest.fail('request stopped'))])
    assert not path and interrupts_generation(failure)
    assert failure_status(failure) == ('CANCELLED' if stop == 'cancel' else 'INDETERMINATE')
    assert c._local_transit_remaining == 240


@pytest.mark.parametrize('mode', ['geometry', 'cancel', 'exception', 'success'])
def test_actual_partial_samples_settled_once(mode, monkeypatch):
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()] if mode == 'geometry' else []
    real = c._cartesian
    def run(*args, **kwargs):
        result = real(*args, **kwargs)
        if mode == 'exception': raise RuntimeError('after real work')
        return result
    monkeypatch.setattr(c, '_cartesian', run)
    if mode == 'cancel':
        real_state = c._state_failure
        def state(*args, **kwargs):
            result = real_state(*args, **kwargs)
            if c._statistics['cartesian_samples'] >= 2: c.cancel_requested = True
            return result
        monkeypatch.setattr(c, '_state_failure', state)
    before = c._statistics['cartesian_samples']
    if mode == 'exception':
        with pytest.raises(RuntimeError, match='after real work'):
            local(c, a, world, c.robot.fk(GOAL))
    else:
        path, failure, evidence = local(c, a, world, c.robot.fk(GOAL))
        if mode == 'geometry': assert failure_status(failure) == 'INVALID'
        if mode == 'cancel': assert interrupts_generation(failure)
        if mode == 'success': assert path and failure is None
        assert evidence['consumed'] == c._statistics['cartesian_samples']-before
    consumed = c._statistics['cartesian_samples']-before
    assert 0 < consumed <= 80
    assert c._local_transit_remaining == 240-consumed
    if mode in ('geometry', 'cancel'): assert consumed < 27


def test_local_shortage_and_rrt_quota_exhaustion_stays_indeterminate():
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()]
    # Use actual RRT with zero candidate iteration quota; no fake planner result.
    path, failure, trace = c._transit(START, GOAL, world, purpose='FREE_LOADED_TRANSFER',
        stage='transit', attachment=a, seed=44, iteration_budget=0, candidates=[
            (GenerationMethod.LOCAL_CARTESIAN_CANDIDATE, lambda: local(c, a, world))])
    assert not path and failure_status(failure) == 'INDETERMINATE'
    assert not interrupts_generation(failure)
    assert trace['rrt_called']


def test_connect_pose_real_producer_continues_to_rrt():
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()]
    q, path, failure, trace = c._connect_pose(c.robot.fk(GOAL), [GOAL], START, world,
        purpose='FREE_LOADED_TRANSFER', stage='transit', attachment=a,
        ik_seed=3, connection_seed=44, candidate_factory=lambda q: [
            (GenerationMethod.LOCAL_CARTESIAN_CANDIDATE, lambda: local(c, a, world))])
    assert failure is None and path and trace['rrt_called']


def test_real_chunk_residual_shortage_settles_only_executed_prefix():
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); goal = c.robot.fk(START)
    goal[0, 3] += 80*c.budget.cartesian_step_m
    path, failure, evidence = local(c, a, [], goal)
    # Real first-chunk FK residual makes the next strict chunk need 41, not 40.
    assert not path and failure['required'] == 41
    assert failure['consumed'] == c._statistics['cartesian_samples'] == 40
    assert failure['candidate_remaining'] == 40
    assert failure['shared_remaining'] == c._local_transit_remaining == 200
    assert not interrupts_generation(failure)


@pytest.mark.parametrize('legacy', [False, True])
def test_actual_shortage_survives_plan_wrapper_and_other_grasp_candidates(legacy, monkeypatch):
    c = connector(); c._local_transit_remaining = 240
    a = loaded(c); world = [obstacle()]; calls = []
    def branch(**kwargs):
        calls.append(True)
        _, failure, evidence = local(c, a, world)
        if legacy:
            failure = {k: failure[k] for k in ('reason', 'required')}
        validator = c._motion_validator(world, attachment=a, stage='transit')
        failure = dict(failure, validation=validator._result(Status.INDETERMINATE, failure).evidence())
        return None, failure, evidence
    monkeypatch.setattr(c, '_plan_branch', branch)
    result = c.plan(target=obstacle(), face='front', requested_virtual_contact=np.eye(4),
        grasp_candidates=[dict(q_rad=GOAL.tolist())]*2, home_q=START,
        all_obstacles=world, receiver=obstacle(), support_names=(), suction={}, seed=44)
    assert len(calls) == 2 and not result.success
    assert result.statistics['validation_status'] == 'INDETERMINATE'
    assert result.failure['validation']['termination_scope'] == 'CANDIDATE'
    assert result.failure['validation']['can_continue_candidates']
