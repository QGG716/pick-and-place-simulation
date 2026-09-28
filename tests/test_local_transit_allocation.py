"""Bounded loans use real interpolation; no analytic route is FANUC evidence."""
import numpy as np
import pytest
from test_stage_motion_policy import connector, START, GOAL, obstacle, connect
from test_local_transit_budget_scope import loaded
from unloading_sim.stage_motion_policy import interrupts_generation, GenerationMethod


@pytest.mark.parametrize('samples', [80., 120.5])
def test_demand_borrows_from_240_and_covers_chunk_residual(samples):
    c=connector(); c._local_transit_remaining=240; a=loaded(c)
    goal=c.robot.fk(START); goal[0,3]+=samples*c.budget.cartesian_step_m
    path,failure,e=c._bounded_local_transit(START,goal,[],a,seed=44)
    assert failure is None and path
    assert 80 < e['allocated'] <= 160
    assert e['reserved_for_later_candidates']==80
    assert e['consumed']==c._statistics['cartesian_samples']
    assert e['consumed']>e['work_estimate']['required']  # strict chunk FK residual
    assert e['consumed']<=e['allocated']
    assert c._local_transit_remaining==240-e['consumed']>=80
    assert c._motion_validator([],attachment=a,stage='transit').check_path(path).valid


def test_legacy_explicit_80_allocation_still_returns_scoped_121_shortage():
    c=connector();c._local_transit_remaining=80;a=loaded(c)
    goal=c.robot.fk(START);goal[0,3]+=120.5*c.budget.cartesian_step_m
    path,f,e=c._local_cartesian_transit(START,goal,[],a,seed=44)
    assert not path and f['required']==121 and f['consumed']==0
    assert f['allocated']==f['candidate_remaining']==80
    assert not interrupts_generation(f)


def test_borrowed_candidate_geometry_failure_preserves_pool_and_real_rrt():
    c=connector();c._local_transit_remaining=240;a=loaded(c);world=[obstacle()]
    destination=c.robot.fk(START);destination[0,3]+=120.5*c.budget.cartesian_step_m
    attempts=[]
    def candidate():
        result=c._bounded_local_transit(START,destination,world,a,seed=44)
        attempts.append(result[2]);return result
    path,f,t=connect(c,world,attachment=a,candidates=[
        (GenerationMethod.LOCAL_CARTESIAN_CANDIDATE,candidate)])
    assert f is None and path and t['rrt_expanded']
    e=attempts[0]
    assert 0<e['consumed']<e['allocated'] and c._local_transit_remaining==240-e['consumed']
    assert t['attempts'][3]['status']=='INVALID'


@pytest.mark.parametrize('pool', [0,40,100])
def test_insufficient_pool_cannot_become_request_stop(pool):
    c=connector();c._local_transit_remaining=pool;a=loaded(c)
    destination=c.robot.fk(START);destination[0,3]+=120.5*c.budget.cartesian_step_m
    p,f,e=c._bounded_local_transit(START,destination,[],a,seed=44)
    assert not p and not interrupts_generation(f)
    assert e['consumed']==0 and c._local_transit_remaining==pool


@pytest.mark.parametrize('stop', ['cancel','context','work'])
def test_request_stop_during_real_borrowed_interpolation(stop,monkeypatch):
    c=connector();c._local_transit_remaining=240;a=loaded(c);world=[]
    destination=c.robot.fk(START);destination[0,3]+=120.5*c.budget.cartesian_step_m
    original=c._state_failure
    injected=[]
    def state(*args,**kwargs):
        result=original(*args,**kwargs)
        if not injected and c._statistics['cartesian_samples']>=2:
            injected.append(True)
            if stop=='cancel': c.cancel_requested=True
            elif stop=='context': world.append(obstacle())
            else: c.validation_budget.max_checks=c.validation_budget.checks
        return result
    monkeypatch.setattr(c,'_state_failure',state)
    path,f,e=c._bounded_local_transit(START,destination,world,a,seed=44)
    assert injected and not path and interrupts_generation(f)
    assert 0<e['consumed']<80<e['allocated']
    assert c._local_transit_remaining==240-e['consumed']
