"""Field 2026-11-26 after the first complete dry run (task 9ec45e42):
release with the tool facing -Z at the same X, Y (height lowered when out of
reach), the round ending at that release pose; loaded-return collisions
recorded, not vetoed; the timed sender asks for real-time priority.
Fake devices only; no hardware."""
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import release_pose as rp
from unloading_sim.workbench.single_carton.planner import loaded_return_policy
from unloading_sim.workbench.controlled.service import sender_realtime
from test_single_carton_cycle import make

BASIS = 'SYNTHETIC'


def test_release_pose_policy_validation():
    assert rp.policy({}) is None and rp.policy(dict(release_pose=dict(mode='DISABLED'))) is None
    assert rp.policy(dict(release_pose=dict(mode=rp.MODE, basis=BASIS)))['mode'] == rp.MODE
    with pytest.raises(ValueError):
        rp.policy(dict(release_pose=dict(mode=rp.MODE)))
    with pytest.raises(ValueError, match='FOUR_SEGMENT'):
        rp.policy(dict(release_pose=dict(mode=rp.MODE, basis=BASIS), cycle_structure=dict(mode='START_RELEASE_SEPARATE')))


def test_goal_turns_the_tool_down_by_the_smallest_rotation_at_the_same_xy():
    T = np.eye(4); T[:3, :3] = np.array([[0, 0, 1.], [0, 1., 0], [-1., 0, 0]]).T   # tool z = +X (horizontal)
    T[:3, 3] = [-.07, -.58, .48]
    G, angle = rp.goal(T, .23)
    assert abs(angle - 90) < 1e-9
    assert np.allclose(G[:3, 2], [0, 0, -1]) and np.allclose(G[:3, :3]@G[:3, :3].T, np.eye(3))
    assert np.allclose(G[:3, 1], T[:3, 1])                       # rotation about the axis normal to z_old, z_new only
    assert np.allclose(G[:3, 3], [-.07, -.58, .25])
    down = T.copy(); down[:3, :3] = np.diag([1., -1., -1.])
    assert rp.goal(down)[0] is None                                  # already facing down: release at q_start


def test_lowering_schedule_and_fk_check():
    assert rp.lowering_schedule(.484, .40) == [0., .02, .04, .06, .08]
    assert rp.lowering_schedule(.1, .2) == []
    T = np.eye(4); T[:3, 3] = [0, -.5, .48]
    B = np.eye(4); B[:3, :3] = np.diag([1., -1., -1.]); B[:3, 3] = [0, -.5, .25]
    assert rp.check(T, B, .23)['valid'] and not rp.check(T, B, .20)['valid']


def plan_with_release_down(plan, q_release):
    plan['segments'][-1]['package']['prepared']['points'][-1]['q'] = list(q_release)
    plan.update(q_release=list(q_release), release_pose=dict(mode=rp.MODE, basis=BASIS, q_release=list(q_release),
                                                            evidence=dict(check=dict(valid=True))))
    return plan


def test_cycle_returns_to_q_release_releases_and_ends_there():
    c, t, r, v, events, plan = make()
    q_release = [.3, -.2, .1, .4, .5, .6]
    c.ready('down', plan_with_release_down(plan, q_release))
    assert np.allclose(c.release_q(), q_release)
    c.start(); r.finish(); c.tick(); r.finish(); c.tick()
    v.kpa = -65; t.step(); v.stamp = t(); c.tick(); t.step(1.1); c.tick(); v.stamp = t(); c.tick()
    assert c.state == 'DETACH'
    r.finish(); c.tick(); assert c.state == 'RETURN_LOADED'
    r.finish(); v.stamp = None; c.tick()
    assert c.state == 'RELEASE' and v.commands == ['grip', 'release'] and np.allclose(r.q, q_release)
    v.kpa = 0; t.step(); c.tick(); t.step(1.01); c.tick()
    assert c.state == 'COMPLETE' and len(r.calls) == 4               # no extra segment back to q_start


def test_cycle_rejects_a_return_not_ending_at_q_release_and_unverified_fk():
    c, t, r, v, events, plan = make()
    bad = plan_with_release_down(plan, [.3]*6); bad['segments'][-1]['package']['prepared']['points'][-1]['q'] = [0.]*6
    with pytest.raises(ValueError, match='RETURN_NOT_Q_RELEASE'):
        c.ready('bad', bad)
    c, t, r, v, events, plan = make()
    unverified = plan_with_release_down(plan, [.3]*6); unverified['release_pose']['evidence'] = {}
    with pytest.raises(ValueError, match='FK_NOT_VERIFIED'):
        c.ready('bad', unverified)


def test_without_release_pose_the_cycle_is_unchanged():
    c, t, r, v, events, plan = make()
    assert np.allclose(c.release_q(), plan['q_start']) and c.legacy_release is None


def test_loaded_return_policy_validation():
    assert loaded_return_policy({}) is None
    assert loaded_return_policy(dict(loaded_return_collision=dict(mode='DISABLED'))) is None
    assert loaded_return_policy(dict(loaded_return_collision=dict(mode='RECORD_ONLY', basis=BASIS)))['mode'] == 'RECORD_ONLY'
    for bad in (dict(mode='RECORD_ONLY'), dict(mode='IGNORE', basis=BASIS), dict(mode='RECORD_ONLY', basis=BASIS, extra=1)):
        with pytest.raises(ValueError):
            loaded_return_policy(dict(loaded_return_collision=bad))


def test_sender_realtime_reports_what_was_obtained(monkeypatch):
    import os
    def denied(*a):
        raise PermissionError(1, 'Operation not permitted')
    monkeypatch.setattr(os, 'sched_setscheduler', denied)
    record = sender_realtime()
    assert record['requested'] == 'SCHED_FIFO' and record['obtained'] == 'SCHED_OTHER' and 'PermissionError' in record['error']
    calls = []
    monkeypatch.setattr(os, 'sched_setscheduler', lambda pid, policy, param: calls.append((pid, policy, param.sched_priority)))
    record = sender_realtime(dict(priority=15))
    assert record['obtained'] == 'SCHED_FIFO' and calls == [(0, os.SCHED_FIFO, 15)]
    assert sender_realtime(dict(mode='DISABLED'))['obtained'] == 'SCHED_OTHER'
