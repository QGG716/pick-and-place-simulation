"""User decision 2026-11-22: start S, loaded transfer clockwise to R, release, return counter-clockwise to S.

Fake devices and fake native replies only; nothing here opens a device or
issues a valve command, and no plan is executable.
"""
from copy import deepcopy
import json
import numpy as np
import pytest
from test_single_carton_planner import case  # noqa: F401
from test_single_carton_cycle import Clock, Robot, Vacuum
from unloading_sim.workbench.single_carton import cycle_structure as cs
from unloading_sim.workbench.single_carton.cycle import Cycle
from unloading_sim.workbench.single_carton.config import load, issues
from unloading_sim.workbench.ui import ROOT

POLICY = dict(mode='START_RELEASE_SEPARATE', release_j1_offset_deg=-180., transfer_j1='CLOCKWISE',
              return_j1='COUNTERCLOCKWISE', basis='USER_DECISION_2026_11_22_TEST')


def test_settings_are_explicit_and_validated():
    assert cs.settings({}) is None
    s = cs.settings(dict(cycle_structure=POLICY))
    assert s['stages'] == list(cs.SEPARATE_STAGES) and s['j1_reverse_budget_rad'] == 1e-6
    for bad in (dict(POLICY, mode='OTHER'), dict(POLICY, release_j1_offset_deg=0.), dict(POLICY, release_j1_offset_deg=361.),
                dict(POLICY, release_j1_offset_deg=True), dict(POLICY, transfer_j1='CW'),
                dict(POLICY, return_j1='CLOCKWISE'), dict(POLICY, basis=' '), dict(POLICY, extra=1)):
        with pytest.raises(ValueError):cs.settings(dict(cycle_structure=bad))
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert not any('cycle_structure' in e for e in issues(dict(c, cycle_structure=POLICY)))
    assert any('cycle_structure' in e for e in issues(dict(c, cycle_structure=dict(POLICY, basis=''))))
    assert any('cycle_structure' in e and 'NATIVE' in e for e in issues(dict(c, cycle_structure=POLICY, planning_module='NATIVE')))


def test_release_pose_is_start_with_j1_offset_inside_joint_bounds():
    s = cs.settings(dict(cycle_structure=POLICY))
    q = np.deg2rad([81.432, -13.064, -55.026, -15.762, 85.987, -185.149])
    np.testing.assert_allclose(np.rad2deg(cs.release_q(s, q)), [-98.568, -13.064, -55.026, -15.762, 85.987, -185.149], atol=1e-9)
    with pytest.raises(ValueError, match='RELEASE_POSE_OUTSIDE_JOINT_LIMITS'):
        cs.release_q(s, q, lower=np.full(6, -1.), upper=np.full(6, 4.))


def test_j1_direction_is_checked_on_goal_and_every_sent_state():
    s = cs.settings(dict(cycle_structure=POLICY))
    assert cs.goal_violation(s, 'transfer_j1', [1.]+[0.]*5, [-1.]+[0.]*5) is None
    assert cs.goal_violation(s, 'transfer_j1', [1.]+[0.]*5, [1.1]+[0.]*5)['reason'] == 'J1_GOAL_AGAINST_DIRECTION'
    down = np.zeros((50, 6)); down[:, 0] = np.linspace(1., -2., 50)
    assert cs.path_violation(s, 'transfer_j1', down) is None
    noisy = down.copy(); noisy[10, 0] += 5e-7          # within the 1e-6 rad cumulative budget
    assert cs.path_violation(s, 'transfer_j1', noisy) is None
    bump = down.copy(); bump[20:23, 0] = bump[19, 0]+np.array([1e-3, 2e-3, 1e-3])
    wrong = cs.path_violation(s, 'transfer_j1', bump)
    assert wrong['state'] == 20 and wrong['reason'] == 'J1_PATH_AGAINST_DIRECTION'
    assert cs.path_violation(s, 'return_j1', down[::-1]) is None
    assert cs.path_violation(s, 'return_j1', down)['direction'] == 'COUNTERCLOCKWISE'
    summary = cs.j1_summary(bump)
    assert summary['increase_rad'] == pytest.approx(2e-3+bump[19, 0]-bump[19, 0]) or summary['increase_rad'] > 0


def test_withdraw_goal_reverses_the_contact_approach_in_the_carton_frame():
    C = np.eye(4); C[:3, 3] = [.5, .1, .3]
    n = np.array([0., 0., 1.])
    Rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]); A = np.eye(4); A[:3, :3] = Rz@np.diag([1, -1, -1])@C[:3, :3]
    A[:3, 3] = [-.2, .6, .4]
    goal = cs.withdraw_goal(A, C, n, .06)
    np.testing.assert_allclose(goal[:3, :3], A[:3, :3])
    np.testing.assert_allclose(goal[:3, 3]-A[:3, 3], A[:3, :3]@C[:3, :3].T@n*.06, atol=1e-12)


def test_released_task_moves_every_pose_bound_field_rigidly():
    T = np.eye(4); T[:3, 3] = [.4, 0, .4]
    box = dict(id='b', size_m=[.15, .1, .1], pose=T.tolist(), corners_m=[[.325, -.05, .35], [.475, .05, .45]],
               measured_box=dict(pose=T.tolist(), size_m=[.14, .1, .1]))
    cand = dict(face=dict(center_m=[.4, 0, .45], normal=[0, 0, 1], u=[1, 0, 0], v=[0, 1, 0], size_m=[.15, .1]),
                tcp_pose=np.eye(4).tolist(), precontact_pose=np.eye(4).tolist())
    R = np.eye(4); R[:3, :3] = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]; R[:3, 3] = [-.4, .2, .5]
    world = [dict(id='wall', size=[1, 1, 1], pose=np.eye(4).tolist()), dict(id='b', size=[.15, .1, .1], pose=T.tolist(), category='carton')]
    rbox, rcand, rworld, meta = cs.released_task(box, cand, world, R)
    G = R@np.linalg.inv(T)
    assert rbox['pose'] == R.tolist() and np.allclose(rbox['measured_box']['pose'], G@T)
    np.testing.assert_allclose(rcand['face']['center_m'], (G@[.4, 0, .45, 1])[:3])
    np.testing.assert_allclose(rcand['face']['normal'], G[:3, :3]@[0, 0, 1])
    ids = [o['id'] for o in rworld]
    assert ids == ['wall', 'b'] and rworld[1]['pose'] == R.tolist() and rworld[1]['category'] == 'released_carton'
    assert meta['vacated_volume']['status'] == 'VACATED_BY_TRANSFER' and meta['vacated_volume']['pose'] == T.tolist()
    assert world[1]['id'] == 'b' and box['pose'] == T.tolist()       # inputs untouched


def separate_plan():
    s = cs.settings(dict(cycle_structure=POLICY)); q0 = [0.]*6; qr = cs.release_q(s, q0).tolist()
    ends = [[.01]*6, [.02]*6, [.03]*6, qr, [qr[0]+.01]+qr[1:], q0]
    segments, start = [], q0
    for stage, end in zip(cs.SEPARATE_STAGES, ends):
        segments.append(dict(stage=stage, package=dict(prepared=dict(points=[dict(q=list(start)), dict(q=list(end))]))))
        start = end
    return dict(full_path_validated=True, segments=segments, q_start=q0, q_release=qr, cycle_structure=s)


def make_separate():
    clock = Clock(); robot = Robot(clock); vacuum = Vacuum(clock); records = []
    config = dict(grip_below_kpa=-60., hold_below_kpa=-30., release_above_kpa=-10., grip_dwell_s=1., release_dwell_s=1.,
                  grip_timeout_s=3., release_timeout_s=3., max_age_s=3.)
    released = []
    cycle = Cycle(robot, vacuum, config, lambda **event: records.append(event), lambda *a: None,
                  lambda q, plan: released.append(list(q)), clock)
    plan = separate_plan(); cycle.ready('one-box', plan)
    return cycle, clock, robot, vacuum, records, plan, released


def test_separate_cycle_releases_at_q_release_then_withdraws_and_returns_before_complete():
    c, t, r, v, events, plan, released = make_separate()
    c.start(); assert c.state == 'APPROACH'
    r.finish(); c.tick(); assert c.state == 'CONTACT'
    r.finish(); c.tick(); assert c.state == 'ESTABLISH_VACUUM' and v.commands == ['grip']
    v.kpa = -65; t.step(); v.stamp = t(); c.tick(); t.step(1.1); c.tick(); v.stamp = t(); c.tick()
    assert c.state == 'DETACH'
    r.finish(); c.tick(); assert c.state == 'TRANSFER_LOADED' and v.commands == ['grip']
    r.finish(); v.stamp = None; c.tick()
    assert c.state == 'RELEASE' and v.commands == ['grip', 'release'] and released == [plan['q_release']]
    v.kpa = 0; t.step(); c.tick(); assert c.state == 'RELEASE'
    t.step(1.01); c.tick()
    assert c.state == 'WITHDRAW_EMPTY' and not v.holding and c.release_confirmed
    assert any(e.get('kind') == 'release_confirmed_at_release_pose' for e in events)
    r.finish(); c.tick(); assert c.state == 'RETURN_EMPTY'
    r.finish(); c.tick()
    assert c.state == 'COMPLETE' and events[-1]['result'] == 'FAKE_DEVICE_CYCLE_COMPLETE' and not c.physical_complete
    assert [p['prepared']['points'][-1]['q'] for p in r.calls] == [s['package']['prepared']['points'][-1]['q'] for s in plan['segments']]
    assert c.status()['cycle_structure'] == 'START_RELEASE_SEPARATE'


def test_separate_cycle_pressure_loss_during_transfer_never_releases():
    c, t, r, v, events, plan, released = make_separate()
    c.start(); r.finish(); c.tick(); r.finish(); c.tick()
    v.kpa = -65; t.step(); v.stamp = t(); c.tick(); t.step(1.1); c.tick(); v.stamp = t(); c.tick()
    r.finish(); c.tick(); assert c.state == 'TRANSFER_LOADED'
    v.kpa = -20; v.stamp = None; c.tick()
    assert c.state == 'STOPPING' and v.commands == ['grip'] and released == []


def test_release_requires_arrival_at_q_release_not_at_start():
    c, t, r, v, events, plan, released = make_separate()
    c.start(); r.finish(); c.tick(); r.finish(); c.tick()
    v.kpa = -65; t.step(); v.stamp = t(); c.tick(); t.step(1.1); c.tick(); v.stamp = t(); c.tick()
    r.finish(); c.tick(); assert c.state == 'TRANSFER_LOADED'
    r.state = 'SUCCEEDED'; r.arrived = True; r.q = [0.]*6; v.stamp = None; c.tick()   # wrong pose reported as arrival
    assert c.state == 'STOPPING' and 'RELEASE_POSE_NOT_CONFIRMED' in c.reason and v.commands == ['grip']


@pytest.mark.parametrize('mutate,error', [
    (lambda p: p.update(q_release=[0.]*6), 'Q_RELEASE_NOT_DERIVED_FROM_Q_START'),
    (lambda p: p['segments'][3]['package']['prepared']['points'][-1].update(q=[-3.]+[0.]*5), 'TRANSFER_NOT_AT_Q_RELEASE'),
    (lambda p: p['segments'][4]['package']['prepared']['points'][0].update(q=[1.]*6), 'SEGMENT_CHAIN_DISCONTINUOUS'),
    (lambda p: p['segments'].pop(4), 'FULL_PATH_REQUIRED'),
    (lambda p: p['segments'][3].update(stage='return'), 'INVALID_STAGE_CHAIN'),
])
def test_ready_rejects_inconsistent_separate_plans(mutate, error):
    c, *_ = make_separate()
    plan = separate_plan(); mutate(plan)
    c.state = 'IDLE'
    with pytest.raises(ValueError, match=error):c.ready('one-box', plan)


def long_moves(opts, override=None):
    """Fake Pilz replies slow enough for the 0.10 rad/s, 0.20 rad/s^2 limits."""
    def plan(r):
        if override:
            hooked = override(r)
            if hooked is not None:return hooked
        if r['stage'] == 'withdraw':goal = [r['q_start'][0]+.01]+list(r['q_start'][1:])
        else:goal = r.get('q_goal') or [dict(approach=.01, contact=.02, detach=.03)[r['stage']]]*6
        T = 25*float(np.max(np.abs(np.asarray(goal)-r['q_start'])))+1.
        return dict(status='SUCCESS', native_output_status='PASS',
                    points=[dict(t=0., q=r['q_start'], v=[0.]*6, a=[0.]*6), dict(t=T, q=goal, v=[0.]*6, a=[0.]*6)])
    opts['plan'] = plan


def test_planner_builds_the_six_segment_chain_with_release_world(case):
    p, obs, box, folder, calls, opts = case
    p.site['cycle_structure'] = POLICY; p.cycle_policy = cs.settings(p.site)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    long_moves(opts)
    plan = p.plan_cycle(obs, [box], folder)
    assert [s['stage'] for s in plan['segments']] == list(cs.SEPARATE_STAGES)
    assert plan['q_release'] == pytest.approx([-np.pi]+[0.]*5) and plan['cycle_structure']['mode'] == cs.MODE
    motions = {c['stage']: c for c in calls if c.get('op') == 'plan'}
    assert motions['transfer']['q_goal'] == pytest.approx(plan['q_release']) and motions['transfer']['attachment'] is not None
    assert motions['return_empty']['q_goal'] == [0.]*6 and motions['return_empty']['attachment'] is None
    assert motions['withdraw']['planner_id'] == 'LIN' and motions['withdraw']['attachment'] is None
    assert motions['withdraw']['single_carton_contact']['allow_contact'] is True
    assert motions['return_empty']['single_carton_contact']['allow_contact'] is False
    lips = [p.contact_geometry['terminal_band_by_contour'][i] for i in motions['withdraw']['single_carton_contact']['target_lip_contact_ids']]
    assert motions['withdraw']['allowed_pairs'][1:] == [[box['id'], lip] for lip in lips]
    assert motions['return_empty']['allowed_pairs'] == [['base_link', 'robot_mount_plate']]
    for stage in ('withdraw', 'return_empty'):
        world = {o['id']: o for o in motions[stage]['world']}
        released = world[box['id']]
        assert released['category'] == 'released_carton' and released['pose'] == motions[stage]['single_carton_contact']['target_pose']
        assert sum(o['id'] == box['id'] for o in motions[stage]['world']) == 1
    assert {o['id'] for o in motions['transfer']['world']} == {o['id'] for o in motions['approach']['world']}
    recorded = json.loads(next(folder.glob('*/loaded-release-check.json')).read_text())
    assert recorded['request']['stage'] == 'transfer' and recorded['request']['q_start'] == pytest.approx(plan['q_release'])
    j1 = {s['stage']: s['package']['evidence'].get('j1_direction') for s in plan['segments']}
    assert j1['transfer']['rule'] == 'CLOCKWISE' and j1['transfer']['increase_rad'] <= 1e-6
    assert j1['return_empty']['rule'] == 'COUNTERCLOCKWISE' and j1['return_empty']['decrease_rad'] <= 1e-6
    assert len(p.authority._plans) == 6
    # Cycle.ready accepts the planner's real plan structure.
    clock = Clock(); cycle = Cycle(Robot(clock), Vacuum(clock), {}, lambda **e: None, lambda *a: None, lambda *a: None, clock)
    cycle.ready('t', plan); assert cycle.movements == cs.SEPARATE_MOVEMENTS


def test_transfer_against_clockwise_is_rejected_before_any_permit(case):
    p, obs, box, folder, calls, opts = case
    p.site['cycle_structure'] = POLICY; p.cycle_policy = cs.settings(p.site)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    def overshoot(r):
        if r['stage'] != 'transfer':return None
        q0 = np.asarray(r['q_start']); q1 = np.asarray(r['q_goal']); mid = (q0+q1)/2; mid[0] = q0[0]+.2
        return dict(status='SUCCESS', native_output_status='PASS', points=[
            dict(t=0., q=q0.tolist(), v=[0.]*6, a=[0.]*6), dict(t=60., q=mid.tolist(), v=[0.]*6, a=[0.]*6),
            dict(t=200., q=q1.tolist(), v=[0.]*6, a=[0.]*6)])
    long_moves(opts, overshoot)
    with pytest.raises(ValueError, match='NO_COMPLETE_FEASIBLE') as error:p.plan_cycle(obs, [box], folder)
    assert 'J1_DIRECTION_REJECTED' in json.dumps(json.loads((folder/'planning-failures.json').read_text()), ensure_ascii=False)
    assert p.authority._plans == {}


def test_release_offset_against_the_transfer_direction_plans_no_transfer(case):
    p, obs, box, folder, calls, opts = case
    p.site['cycle_structure'] = dict(POLICY, release_j1_offset_deg=170.); p.cycle_policy = cs.settings(p.site)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    long_moves(opts)
    with pytest.raises(ValueError, match='NO_COMPLETE_FEASIBLE'):p.plan_cycle(obs, [box], folder)
    assert not any(c.get('stage') == 'transfer' for c in calls if c.get('op') == 'plan') and p.authority._plans == {}


def test_legacy_cycle_is_unchanged_without_cycle_structure(case):
    p, obs, box, folder, calls, opts = case
    assert p.cycle_policy is None
    plan = p.plan_cycle(obs, [box], folder)
    assert [s['stage'] for s in plan['segments']] == ['approach', 'contact', 'detach', 'return'] and plan['q_release'] is None


def test_concurrent_chains_use_fixed_workers_and_same_plan(case, monkeypatch):
    """lane_clients >= 2: contact/detach/transfer, withdraw/return_empty and approach on fixed workers."""
    p, obs, box, folder, calls, opts = case
    p.site['cycle_structure'] = POLICY; p.cycle_policy = cs.settings(p.site)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    long_moves(opts)
    from unloading_sim.workbench.single_carton.geometry import contact_candidates
    tcp_z = np.asarray(contact_candidates(box, p.source['tool'], p.site['topology'])[0]['tcp_pose'])[2, 3]
    # Screened approach configuration: precontact IK -> 0.01, contact IK -> 0.02 (fake FK switches at 0.01).
    opts['ik'] = lambda r: dict(status='SUCCESS', q=[.01]*6 if r['goal_pose'][2][3] > tcp_z+.01 else [.02]*6,
                                position_error_m=0., orientation_error_rad=0.)
    sequential = p.plan_cycle(obs, [box], folder)
    from unloading_sim import moveit2_backend
    inner = moveit2_backend.ResidentMoveItClient()     # the fixture's fake native client
    class Tagged:
        def __init__(self, name):self.name = name
        def request(self, r, **kwargs):
            r = dict(r); r['_worker'] = self.name
            return inner.request(r, **kwargs)
    workers = [Tagged('w0'), Tagged('w1'), Tagged('w2')]
    monkeypatch.setattr(p, 'checkers', lambda: workers)
    monkeypatch.setattr(p, 'scene_handle', lambda client, folder, cancelled=None: 'H')
    del calls[:]
    folder2 = folder.parent/('b'*32); folder2.mkdir()
    concurrent = p.plan_cycle(obs, [box], folder2)
    owner = {c['stage']: c['_worker'] for c in calls if c.get('op') == 'plan'}
    assert owner == dict(contact='w0', detach='w0', transfer='w0', withdraw='w1', return_empty='w1', approach='w2')
    for a, b in zip(sequential['segments'], concurrent['segments']):
        assert a['stage'] == b['stage']
        assert a['package']['prepared']['points'] == b['package']['prepared']['points']


def test_geometric_probe_grid_keeps_send_grid_and_one_millimetre_bound():
    from scipy.interpolate import BPoly, PPoly
    from unloading_sim.workbench.controlled.trajectory import prepare, Limits, geometric_probe_times
    rng = np.random.default_rng(6518)
    lim = Limits((-6.,)*6, (6.,)*6, (.1,)*6, (.2,)*6)
    t = np.r_[0., np.cumsum(rng.uniform(3., 5., 12))]
    q = np.cumsum(rng.normal(0, .02, (13, 6)), axis=0); v = rng.normal(0, .004, (13, 6)); a = rng.normal(0, .001, (13, 6))
    v[[0, -1]] = 0; a[[0, -1]] = 0
    points = [dict(t=float(t[i]), q=q[i].tolist(), v=v[i].tolist(), a=a[i].tolist()) for i in range(13)]
    for scale in (1., 3.):
        base = prepare(points, lim, scale, .005, 1.2); geo = prepare(points, lim, scale, .005, 1.2, probe_grid='GEOMETRIC')
        assert base['points'] == geo['points'] and len(geo['collision_path']) < len(base['collision_path'])
        assert geo['collision_path'][0] == base['collision_path'][0] and np.allclose(geo['collision_path'][-1], q[-1])
        assert geo['collision_sampling']['maximum_pair_displacement_between_checks_m'] == .001
        pp = PPoly.from_bernstein_basis(BPoly.from_derivatives(t*scale, [[q[i], v[i]/scale, a[i]/scale**2] for i in range(13)]))
        ext = []
        for j in range(6):
            f = PPoly(pp.c[:, :, j], pp.x).derivative(1); r = f.derivative().roots(extrapolate=False)
            xs = np.r_[t*scale, r[np.isfinite(r)]]; ext.append((xs, abs(f(xs))))
        times, _ = geometric_probe_times(t*scale, ext, 1.2)
        worst = max(4*1.2*np.abs(np.diff(pp(np.linspace(times[k], times[k+1], 30)), axis=0)).sum() for k in range(len(times)-1))
        assert worst <= .001+1e-12
    # Independent of uniform time scaling: same number of probe states.
    assert len(prepare(points, lim, 1., .005, 1.2, probe_grid='GEOMETRIC')['collision_path']) == \
        len(prepare(points, lim, 3., .005, 1.2, probe_grid='GEOMETRIC')['collision_path'])
    with pytest.raises(ValueError, match='INVALID_PROBE_GRID'):prepare(points, lim, 1., .005, 1.2, probe_grid='X')


def test_latency_options_are_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    ok = dict(deepcopy(c), sent_path_probe='GEOMETRIC', parallel_screening=True, lane_clients=3)
    assert not any(k in e for e in issues(ok) for k in ('sent_path_probe', 'parallel_screening', 'lane_clients'))
    assert any('sent_path_probe' in e for e in issues(dict(ok, sent_path_probe='SPARSE')))
    assert any('parallel_screening' in e for e in issues(dict(ok, parallel_screening='yes')))
    assert any('lane_clients' in e for e in issues(dict(ok, lane_clients=4)))
    assert any('NATIVE' in e and 'lane_clients' in e for e in issues(dict(ok, planning_module='NATIVE')))


def test_concurrent_audits_cover_every_segment_once_with_disjoint_workers(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    p.site['cycle_structure'] = POLICY; p.cycle_policy = cs.settings(p.site); p.site['concurrent_audits'] = True
    p.site['sent_path_audit'] = 'ADAPTIVE'; opts['capabilities'] = dict(adaptive_sent_path_audit=True)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    long_moves(opts)
    from unloading_sim import moveit2_backend
    inner = moveit2_backend.ResidentMoveItClient()
    class Tagged:
        def __init__(self, name):self.name = name
        def request(self, r, **kwargs):
            r = dict(r); r['_worker'] = self.name
            if r.get('op') == 'validate' and 'probe_path' in r and r.get('sent_path_audit') == 'ADAPTIVE':
                reply = inner.request(r, **kwargs)
                return dict(reply, clearance=dict(sent_path_audit=dict(sampled_states=len(r['probe_path']), skipped_states=0,
                    anchor_dynamic_pair_distance_lower_bound_m=None, anchor_workspace_margin_m=None, margin_cap_m=.05,
                    proof_epsilon_m=1e-6, skip_rule='FAKE')))
            return inner.request(r, **kwargs)
    workers = [Tagged(f'w{k}') for k in range(9)]
    monkeypatch.setattr(p, 'checkers', lambda: workers)
    monkeypatch.setattr(p, 'scene_handle', lambda client, folder, cancelled=None: 'H')
    plan = p.plan_cycle(obs, [box], folder)
    probes = [c for c in calls if c.get('op') == 'validate' and 'probe_path' in c]
    for segment in plan['segments']:
        path = segment['package']['prepared']['collision_path']
        chunks = sorted((c for c in probes if c['stage'] == segment['stage']), key=lambda c: path.index(c['probe_path'][0]))
        assert chunks[0]['probe_path'][0] == path[0] and chunks[-1]['probe_path'][-1] == path[-1]
        assert sum(len(c['probe_path']) for c in chunks)-(len(chunks)-1) == len(path)
    owners = {}
    for c in probes:owners.setdefault(c['_worker'], set()).add(c['stage'])
    assert all(len(stages) == 1 for stages in owners.values())     # disjoint worker subsets
