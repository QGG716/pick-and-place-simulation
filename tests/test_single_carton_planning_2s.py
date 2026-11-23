"""2-second planning, step 1 (2026-11-22): pipelined_audits and early_scene_upload change scheduling only.

Fake native replies only; nothing here opens a device or issues a valve
command, and no plan is executable.
"""
from copy import deepcopy
import json
import numpy as np
import pytest
from test_single_carton_planner import case  # noqa: F401
from test_single_carton_start_release_cycle import POLICY, long_moves
from unloading_sim.workbench.single_carton import cycle_structure as cs
from unloading_sim.workbench.single_carton.config import load, issues
from unloading_sim.workbench.ui import ROOT

PLANNING, AUDIT = 3, 4


def six_segment(p, opts, box):
    p.site['cycle_structure'] = POLICY; p.cycle_policy = cs.settings(p.site)
    from unloading_sim.workbench.controlled.trajectory import Limits
    p.limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    long_moves(opts)
    from unloading_sim.workbench.single_carton.geometry import contact_candidates
    tcp_z = np.asarray(contact_candidates(box, p.source['tool'], p.site['topology'])[0]['tcp_pose'])[2, 3]
    opts['ik'] = lambda r: dict(status='SUCCESS', q=[.01]*6 if r['goal_pose'][2][3] > tcp_z+.01 else [.02]*6,
                                position_error_m=0., orientation_error_rad=0.)


def tagged(p, monkeypatch, n):
    from unloading_sim import moveit2_backend
    inner = moveit2_backend.ResidentMoveItClient()

    class Tagged:
        def __init__(self, name):
            self.name = name

        def request(self, r, **kwargs):
            r = dict(r); r['_worker'] = self.name
            return inner.request(r, **kwargs)
    workers = [Tagged(f'w{k}') for k in range(n)]
    monkeypatch.setattr(p, 'checkers', lambda: workers)
    monkeypatch.setattr(p, 'scene_handle', lambda client, folder, cancelled=None: 'H')
    return workers


def pipelined(p):
    p.site.update(pipelined_audits=True, pipelined_audit_workers=AUDIT, lane_clients=3, parallel_candidate_lanes=1)


def outcome(plan, folder):
    failures = json.loads((folder/'planning-failures.json').read_text()) if (folder/'planning-failures.json').is_file() else None
    if plan is None:
        return dict(failures=[(f.get('candidate'), f['reason'].split(':')[0]+':'+f['reason'].split(':')[1] if ':' in f['reason'] else f['reason'])
                              for f in (failures or [])])
    return dict(candidate=plan['candidate_index'],
                segments=[(s['stage'], s['package']['prepared']['points'], s['package']['evidence'].get('detach_profile'))
                          for s in plan['segments']])


def run(p, obs, box, folder):
    try:
        return outcome(p.plan_cycle(obs, [box], folder), folder)
    except ValueError as error:
        assert 'NO_COMPLETE_FEASIBLE' in str(error)
        return outcome(None, folder)


def test_switches_are_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    ok = dict(deepcopy(c), pipelined_audits=True, pipelined_audit_workers=4, scene_handles=True, early_scene_upload=True)
    assert not any(k in e for e in issues(ok) for k in ('pipelined', 'early_scene_upload'))
    assert any('pipelined_audits' in e for e in issues(dict(ok, pipelined_audits='yes')))
    assert any('pipelined_audit_workers' in e for e in issues(dict(ok, pipelined_audit_workers=0)))
    assert any('pipelined_audit_workers' in e for e in issues({k: v for k, v in ok.items() if k != 'pipelined_audit_workers'}))
    assert any('early_scene_upload' in e for e in issues(dict(ok, scene_handles=False)))


def test_pipelined_audits_same_plan_on_disjoint_planning_and_audit_workers(case, monkeypatch, tmp_path):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    sequential = run(p, obs, box, folder)
    pipelined(p); tagged(p, monkeypatch, PLANNING+AUDIT)
    del calls[:]
    folder2 = folder.parent/('b'*32); folder2.mkdir()
    plan = p.plan_cycle(obs, [box], folder2)
    assert outcome(plan, folder2) == sequential
    plans = [c for c in calls if c.get('op') == 'plan']
    probes = [c for c in calls if c.get('op') == 'validate' and 'probe_path' in c]
    assert {c['_worker'] for c in plans} <= {f'w{k}' for k in range(PLANNING)}
    assert {c['_worker'] for c in probes} <= {f'w{k}' for k in range(PLANNING, PLANNING+AUDIT)}
    # Every segment's exact probe path is covered once by contiguous chunks.
    for segment in plan['segments']:
        path = segment['package']['prepared']['collision_path']
        mine = [c for c in probes if c['stage'] == segment['stage'] and c['probe_path'][0] in path and c['probe_path'][-1] in path]
        chunks = sorted(mine, key=lambda c: path.index(c['probe_path'][0]))
        assert chunks[0]['probe_path'][0] == path[0] and chunks[-1]['probe_path'][-1] == path[-1]
        assert sum(len(c['probe_path']) for c in chunks)-(len(chunks)-1) == len(path)
        assert segment['package']['evidence']['sent_path_collision'] == 'PASS'
    assert len(p.authority._plans) == 6


@pytest.mark.parametrize('rejected', [('detach',), ('return_empty', 'approach'), ('transfer-first',), ('contact', 'withdraw')])
def test_pipelined_audits_reject_like_the_cheapest_first_order(case, monkeypatch, rejected):
    """The first rejected audit in the cheapest-first order decides, as in verify_candidate."""
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    seen = {}

    def sent(r):
        stage = r['stage']
        if stage == 'transfer' and 'transfer-first' in rejected:
            first = seen.setdefault('transfer', tuple(r['probe_path'][0]))
            reject = tuple(r['probe_path'][0]) == first
        else:
            reject = stage in rejected
        failure = {'reason': 'FAKE_SENT_PATH_REJECTED', 'stage': stage} if reject else None
        return dict(native_valid=True, legacy_intersection_valid=True, path_failure=failure, failure=None)
    opts['sent'] = sent
    sequential = run(p, obs, box, folder)
    seen.clear(); pipelined(p); tagged(p, monkeypatch, PLANNING+AUDIT)
    folder2 = folder.parent/('b'*32); folder2.mkdir()
    assert run(p, obs, box, folder2) == sequential
    assert p.authority._plans == {} or 'transfer-first' in rejected


def test_early_scene_upload_reaches_every_worker_before_screening(case, monkeypatch):
    import threading
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    p.site.update(scene_handles=True, early_scene_upload=True); opts['capabilities'] = dict(scene_handles=True)
    workers = tagged(p, monkeypatch, 5)
    early = []
    def handle(client, folder, cancelled=None):
        if threading.current_thread().name == 'scene-upload':
            early.append(client.name)
        return 'H'
    monkeypatch.setattr(p, 'scene_handle', handle)
    p.plan_cycle(obs, [box], folder)
    assert sorted(early) == sorted(w.name for w in workers)


def ladder_fake(opts, succeed_at=.25):
    """A worker-side LIN ladder emulation: scales below succeed_at fail."""
    def attempt(r, scale):
        if r['planner_id'] == 'LIN' and scale > succeed_at:
            return dict(status='NATIVE_PLANNING_FAILED', detail='FAKE_JOINT_LIMIT')
        return None
    previous = opts.get('plan')

    def plan(r):
        if 'lin_retry_scales' in r:
            attempts = []; scale = r['velocity_scale']; out = attempt(r, scale)
            attempts.append(dict(velocity_scale=scale, status=(out or {}).get('status', 'SUCCESS')))
            for factor in r['lin_retry_scales']:
                if out is None:
                    break
                scale = factor; out = attempt(r, factor)
                attempts.append(dict(velocity_scale=factor, status=(out or {}).get('status', 'SUCCESS')))
            reply = dict(out) if out is not None else previous(dict(r, velocity_scale=scale))
            reply['lin_retry'] = dict(attempts=attempts, final_velocity_scale=scale, final_acceleration_scale=scale)
            return reply
        out = attempt(r, r['velocity_scale'])
        return out if out is not None else previous(r)
    opts['plan'] = plan


def test_native_lin_ladder_gives_the_same_profiles_and_plan(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box); ladder_fake(opts)
    separate = p.plan_cycle(obs, [box], folder)
    lin_separate = [(c['stage'], c['velocity_scale']) for c in calls if c.get('op') == 'plan' and c['planner_id'] == 'LIN']
    assert ('contact', .25) in lin_separate and ('contact', .5) in lin_separate
    p.site['native_lin_ladder'] = True; opts['capabilities'] = dict(lin_retry_ladder=True)
    del calls[:]
    folder2 = folder.parent/('b'*32); folder2.mkdir()
    merged = p.plan_cycle(obs, [box], folder2)
    lin_merged = [c for c in calls if c.get('op') == 'plan' and c['planner_id'] == 'LIN']
    assert all(c['lin_retry_scales'] == [.5, .25, .1, .05] and c['velocity_scale'] == 1. for c in lin_merged)
    # One request per LIN segment instead of one per profile.
    assert len(lin_merged) == sum(1 for _, v in lin_separate if v == 1.) < len(lin_separate)
    for a, b in zip(separate['segments'], merged['segments']):
        assert a['stage'] == b['stage'] and a['package']['prepared']['points'] == b['package']['prepared']['points']
        assert a['package']['evidence']['planner'] == b['package']['evidence']['planner']
    assert (next(folder2.glob('*/contact-pilz-ladder-native.json'))).is_file()


def test_process_geometry_ref_requests_carry_the_digest_only(case, tmp_path):
    p, obs, box, folder, calls, opts = case
    from unloading_sim.workbench.controlled.trajectory import digest
    p.contact_geometry.update(contours=[dict(id='c', points_m=[[0., 0., 0.]]*3)])
    p.contact_geometry_digest = digest(p.contact_geometry)
    p.site['process_geometry_ref'] = True; opts['capabilities'] = dict(process_geometry_ref=True)
    p.plan_cycle(obs, [box], folder)
    contexts = [c for c in calls if 'single_carton_contact' in c]
    assert contexts and all('geometry' not in c['single_carton_contact'] and
                            c['single_carton_contact']['geometry_ref'] == p.contact_geometry_digest for c in contexts)
    recorded = json.loads((folder/'native-contact-geometry.json').read_text())
    assert recorded['process_geometry_digest'] == digest(recorded['geometry']) == p.contact_geometry_digest


def test_batched_tcp_is_bitwise_identical_to_per_state_fk():
    from unloading_sim.workbench.ui import URDF, URDF_SHA
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.single_carton.holding_capacity import tcp_batch
    config = load(ROOT/'configs/workbench/single_carton.example.json')
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    limits = model.robot.joint_limits
    q = limits[:, 0]+np.random.default_rng(7).random((500, 6))*(limits[:, 1]-limits[:, 0])
    assert np.array_equal(tcp_batch(model, q), np.asarray([model.tcp(x) for x in q]))
    with pytest.raises(ValueError, match='exceed official URDF limits'):
        tcp_batch(model, np.vstack([q[:2], limits[:, 1]+.01]))


def test_tree_digest_detects_every_change_and_record_round_trips(tmp_path):
    from unloading_sim.workbench.plan_digest import tree_digest
    from unloading_sim.workbench import plan_record
    rng = np.random.default_rng(3)
    points = [dict(t=float(k)*.005, q=rng.random(6).tolist(), v=rng.random(6).tolist(), a=rng.random(6).tolist()) for k in range(300)]
    binding = dict(world=[dict(id=f'o{k}', pose=np.eye(4).tolist(), size=[.1, .2, .3]) for k in range(20)], files={'a': 'x'})
    package = dict(prepared=dict(points=points, collision_path=rng.random((900, 6)).tolist(), time_scale=1.), binding=binding,
                   evidence=dict(planner=dict(planner_id='PTP', velocity_scale=1), flags=[True, None, 2]))
    plan = dict(binding=binding, segments=[dict(stage='a', package=package)], q_start=[0.]*6)
    reference = tree_digest(plan)
    assert reference == tree_digest(json.loads(json.dumps(plan)))     # content, not object identity
    mutations = [lambda p: p['segments'][0]['package']['prepared']['points'][7]['q'].__setitem__(2, points[7]['q'][2]+1e-16),
                 lambda p: p['segments'][0]['package']['prepared']['points'][7]['q'].__setitem__(2, 1),
                 lambda p: p['segments'][0]['package']['prepared']['collision_path'][5].__setitem__(0, True),
                 lambda p: p['segments'][0]['package']['evidence']['planner'].__setitem__('velocity_scale', 1.),
                 lambda p: p['segments'][0]['package']['binding']['world'][3]['size'].append(0.),
                 lambda p: p['binding']['files'].__setitem__('b', 'x'),
                 lambda p: p['segments'][0]['package']['prepared']['points'].pop()]
    for mutate in mutations:
        changed = json.loads(json.dumps(plan)); mutate(changed)
        assert tree_digest(changed) != reference
    with pytest.raises(ValueError):
        bad = json.loads(json.dumps(plan)); bad['segments'][0]['package']['prepared']['collision_path'][0][0] = float('nan'); tree_digest(bad)
    path = tmp_path/'complete-plan.json'
    plan_record.write(path, plan)
    restored = plan_record.load(path)
    assert json.dumps(restored) == json.dumps(plan) and tree_digest(restored) == reference
    assert list((tmp_path/'complete-plan-arrays').glob('*.npy'))


def test_cycle_uses_the_plan_digest_method_fixed_at_ready():
    from unloading_sim.workbench.single_carton.cycle import plan_digest_function
    from unloading_sim.workbench.plan_digest import tree_digest
    from unloading_sim.workbench.controlled.trajectory import digest
    assert plan_digest_function(None) is digest and plan_digest_function('CANONICAL_TREE_SHA256') is tree_digest
    with pytest.raises(ValueError, match='UNKNOWN_PLAN_DIGEST_METHOD'):
        plan_digest_function('SHA1')


def test_tree_digest_and_record_treat_numpy_floats_as_json_does(tmp_path):
    # 2026-11-23: a plan field holding numpy.float64 (a float subclass that the
    # canonical JSON digest prints as a float) made tree_digest raise.
    from unloading_sim.workbench.plan_digest import tree_digest
    from unloading_sim.workbench import plan_record
    rng = np.random.default_rng(5)
    points = [dict(t=float(k), q=[np.float64(x) if k % 3 == 0 else float(x) for x in rng.random(6)]) for k in range(40)]
    plan = dict(margin=np.float64(.25), flags=[True, 1, np.float64(2.)], points=points,
                path=[np.float64(x) for x in rng.random(300)], mixed=[np.float64(1.), 2.])
    reference = tree_digest(plan)
    assert reference == tree_digest(json.loads(json.dumps(plan)))
    changed = json.loads(json.dumps(plan)); changed['margin'] = .26
    assert tree_digest(changed) != reference
    changed = json.loads(json.dumps(plan)); changed['flags'][1] = 1.
    assert tree_digest(changed) != reference                     # an int stays distinct from a float
    path = tmp_path/'complete-plan.json'
    plan_record.write(path, plan)
    restored = plan_record.load(path)
    assert restored == json.loads(json.dumps(plan)) and tree_digest(restored) == reference


def test_parallel_screen_reports_the_target_block_when_other_screens_are_cancelled(case, monkeypatch):
    # 2026-11-23: a screen cancelled by another candidate's TargetBlocked raised
    # RuntimeError, ended its lane thread unrecorded, and the caller got KeyError.
    import threading
    from unloading_sim.workbench.single_carton.planner import TargetBlocked
    p, obs, box, folder, calls, opts = case
    monkeypatch.setattr(p, 'checkers', lambda: ['w0', 'w1', 'w2'])
    blocked = threading.Event()

    def screen(q, world, b, candidate, sub, cancelled, solutions, client):
        if candidate['n'] == 1:
            blocked.wait(5); raise TargetBlocked('TARGET_PAYLOAD_BLOCKED_AT_REST: fake')
        if candidate['n'] == 0:
            blocked.set()
            for _ in range(500):
                if cancelled():
                    raise RuntimeError('approach-endpoint-ik:CANCELLED')
                threading.Event().wait(.01)
        return dict(release_supports=[], screen=dict(conclusion='CLEAR'))
    monkeypatch.setattr(p, 'screen_candidate', screen)
    candidates = [dict(n=k) for k in range(5)]
    with pytest.raises(TargetBlocked):
        p.parallel_screen(np.zeros(6), [], box, candidates, None, folder, lambda _: None, lambda: False, [])
    # Any other error of a screen is raised, never lost.
    def broken(q, world, b, candidate, sub, cancelled, solutions, client):
        if candidate['n'] == 2:
            raise RuntimeError('WORKER_CLOSED')
        return dict(release_supports=[], screen=dict(conclusion='CLEAR'))
    monkeypatch.setattr(p, 'screen_candidate', broken)
    folder2 = folder.parent/('c'*32); folder2.mkdir()
    with pytest.raises(RuntimeError, match='WORKER_CLOSED'):
        p.parallel_screen(np.zeros(6), [], box, candidates, None, folder2, lambda _: None, lambda: False, [])


def shared(p, lanes=2):
    p.site.update(pipelined_audits=True, pipelined_audit_mode='IDLE_SPECULATIVE_WORKERS', lane_clients=3,
                  parallel_candidate_lanes=lanes)
    p.site.pop('pipelined_audit_workers', None)


def test_idle_speculative_audit_mode_is_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    ok = dict(deepcopy(c), pipelined_audits=True, pipelined_audit_mode='IDLE_SPECULATIVE_WORKERS')
    assert not any('pipelined' in e for e in issues(ok))
    assert any('pipelined_audit_workers' in e for e in issues(dict(ok, pipelined_audit_workers=4)))
    assert any('pipelined_audit_mode' in e for e in issues(dict(ok, pipelined_audit_mode='ANY')))
    assert any('pipelined_audit_mode' in e for e in issues(dict(ok, pipelined_audits=False)))


@pytest.mark.parametrize('lanes', [1, 2])
def test_idle_speculative_audits_same_plan_and_planner_requests_on_fixed_lanes(case, monkeypatch, lanes):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    sequential = run(p, obs, box, folder)
    shared(p, lanes); workers = tagged(p, monkeypatch, 3*lanes+1)
    del calls[:]
    folder2 = folder.parent/('b'*32); folder2.mkdir()
    plan = p.plan_cycle(obs, [box], folder2)
    assert outcome(plan, folder2) == sequential
    lane_of = {w.name: k//3 for k, w in enumerate(workers[:3*lanes])}
    for c in calls:
        if c.get('op') == 'plan':
            # A planner request goes to the fixed lane of its candidate's rank (as in parallel_candidates).
            assert c['_worker'] in lane_of
    for segment in plan['segments']:
        assert segment['package']['evidence']['sent_path_collision'] == 'PASS'
    assert len(p.authority._plans) == 6


@pytest.mark.parametrize('rejected', [('detach',), ('return_empty', 'approach'), ('transfer-first',), ('contact', 'withdraw')])
def test_idle_speculative_audits_reject_like_the_cheapest_first_order(case, monkeypatch, rejected):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    seen = {}

    def sent(r):
        stage = r['stage']
        if stage == 'transfer' and 'transfer-first' in rejected:
            first = seen.setdefault('transfer', tuple(r['probe_path'][0]))
            reject = tuple(r['probe_path'][0]) == first
        else:
            reject = stage in rejected
        failure = {'reason': 'FAKE_SENT_PATH_REJECTED', 'stage': stage} if reject else None
        return dict(native_valid=True, legacy_intersection_valid=True, path_failure=failure, failure=None)
    opts['sent'] = sent
    sequential = run(p, obs, box, folder)
    seen.clear(); shared(p); tagged(p, monkeypatch, 7)
    folder2 = folder.parent/('b'*32); folder2.mkdir()
    assert run(p, obs, box, folder2) == sequential


def test_shared_slots_let_speculative_lanes_wait_for_audits_only():
    import threading
    from unloading_sim.workbench.single_carton.pipeline import SharedSlots
    a, b, c = object(), object(), object()
    slots = SharedSlots([[a], [b]]); slots.set_head(0, True)
    slots.put((0, 0), 'chunk')
    assert slots.eligible(b) and not slots.eligible(a)
    order = []
    t = threading.Thread(target=lambda: slots.lane_request(b, 1, lambda: order.append('speculative')))
    t.start(); t.join(.2)
    assert order == [] and t.is_alive()                         # waits while a chunk is queued
    slots.lane_request(a, 0, lambda: order.append('head'))      # the head lane never waits for audits
    assert slots.take(b) == 'chunk'; slots.done(b)
    t.join(2); assert order == ['head', 'speculative']
    slots.set_head(0, False); assert slots.eligible(a)          # head plan complete: its lane may audit
    slots.close(); assert slots.take(a) is None


def test_python_switch_interval_is_validated_applied_while_planning_and_restored(case, monkeypatch):
    import sys
    p, obs, box, folder, calls, opts = case
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert not any('python_switch_interval_s' in e for e in issues(dict(deepcopy(c), python_switch_interval_s=5e-4)))
    for bad in (0., 1e-5, .01, '1e-3', True):
        assert any('python_switch_interval_s' in e for e in issues(dict(deepcopy(c), python_switch_interval_s=bad)))
    from unloading_sim.workbench.single_carton import planner as planner_module
    before = sys.getswitchinterval(); seen = []
    original = planner_module.contact_candidates
    def contact_candidates(*a, **k):
        seen.append(sys.getswitchinterval()); return original(*a, **k)
    monkeypatch.setattr(planner_module, 'contact_candidates', contact_candidates)
    p.site['python_switch_interval_s'] = 5e-4
    p.plan_cycle(obs, [box], folder)
    assert seen and all(abs(x-5e-4) < 1e-9 for x in seen) and sys.getswitchinterval() == before
    assert p.last_task['python_switch_interval_s'] == dict(task=5e-4, previous=before)   # also in planning-task-lifecycle.json


@pytest.mark.parametrize('pipelined_mode', [None, 'IDLE_SPECULATIVE_WORKERS'])
@pytest.mark.parametrize('variant', ['pass', 'holding_fails', 'audit_rejects'])
def test_early_holding_postcheck_keeps_the_decision_order(case, monkeypatch, variant, pipelined_mode):
    """Computed during the audits, applied after them: same outcome, nothing recorded for a discarded result."""
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    if pipelined_mode:
        shared(p); tagged(p, monkeypatch, 7)
    evaluated = []
    def holding_evaluate(planned):
        evaluated.append(planned['profile'])
        return dict(valid=variant != 'holding_fails', scope='FAKE_CAPACITY_NOT_EVIDENCE')
    monkeypatch.setattr(p, 'holding_evaluate', holding_evaluate)
    monkeypatch.setattr(p, 'holding_postcheck', lambda planned, sub, started=None:
                        type(p).holding_postcheck(p, planned, sub, started))
    if variant == 'audit_rejects':
        opts['sent'] = lambda r: dict(native_valid=True, legacy_intersection_valid=True, failure=None,
                                      path_failure={'reason': 'FAKE_SENT_PATH_REJECTED'} if r['stage'] == 'withdraw' else None)
    reference = run(p, obs, box, folder)
    p.site['early_holding_postcheck'] = True; evaluated.clear()
    folder2 = folder.parent/('d'*32); folder2.mkdir()
    assert run(p, obs, box, folder2) == reference
    written = list(folder2.glob('*/holding-capacity-final.json'))
    if variant == 'audit_rejects':
        assert evaluated and not written          # evaluated, discarded, never recorded
    else:
        assert written


def test_workspace_pair_cache_switch_reaches_every_context_request(case):
    p, obs, box, folder, calls, opts = case
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert any('workspace_pair_cache' in e for e in issues(dict(deepcopy(c), workspace_pair_cache='yes')))
    p.site['workspace_pair_cache'] = True; opts['capabilities'] = dict(workspace_pair_cache=True)
    p.plan_cycle(obs, [box], folder)
    contexts = [r for r in calls if 'clearance_policy' in r]
    assert contexts and all(r.get('workspace_pair_cache') is True for r in contexts)


def test_adaptive_constraints_switch_reaches_every_adaptive_audit_request(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert any('adaptive_constraints' in e for e in issues(dict(deepcopy(c), adaptive_constraints='PER_PAIR')))
    assert any('adaptive_constraints' in e for e in issues(dict(deepcopy(c), adaptive_constraints='PER_BODY', sent_path_audit='SAMPLED')))
    p.site.update(sent_path_audit='ADAPTIVE_CONTACT', adaptive_constraints='PER_BODY')
    opts['capabilities'] = dict(adaptive_per_body_constraints=True, adaptive_sent_path_audit=True, adaptive_contact_sent_path_audit=True)
    try:
        p.plan_cycle(obs, [box], folder)
    except ValueError as error:   # the fake worker reports no adaptive audit evidence; only the requests matter here
        assert 'NO_COMPLETE_FEASIBLE' in str(error)
    audits = [r for r in calls if r.get('op') == 'validate' and 'probe_path' in r]
    assert audits and all(r.get('adaptive_constraints') == 'PER_BODY' and r['sent_path_audit'] in ('ADAPTIVE', 'ADAPTIVE_CONTACT') for r in audits)


def test_prepare_processes_give_bitwise_identical_results_and_errors():
    from unloading_sim.workbench.single_carton.prepare_pool import PreparePool
    from unloading_sim.workbench.controlled.trajectory import prepare, Limits
    limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    t = np.linspace(0., 6., 40); q = np.outer(np.sin(np.pi*t/12)**2, np.linspace(.05, .3, 6))
    v = np.gradient(q, t, axis=0); a = np.gradient(v, t, axis=0); v[[0, -1]] = 0.; a[[0, -1]] = 0.
    points = [dict(t=float(t[k]), q=q[k].tolist(), v=v[k].tolist(), a=a[k].tolist()) for k in range(len(t))]
    pool = PreparePool(2)
    try:
        for scale, grid in ((1., 'GEOMETRIC'), (2., 'GEOMETRIC'), (1.5, 'SENT_GRID')):
            try:
                local = prepare(points, limits, scale, .005, 1.8, probe_grid=grid)
            except ValueError as error:
                with pytest.raises(ValueError, match=str(error).split(':')[0]):
                    pool.prepare(points, limits, scale, .005, 1.8, grid)
                continue
            assert json.dumps(pool.prepare(points, limits, scale, .005, 1.8, grid)) == json.dumps(local)
        with pytest.raises(ValueError, match='INVALID_TIMING'):
            pool.prepare(points, limits, 1000., .005, 1.8, 'GEOMETRIC')
    finally:
        pool.close()
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert not any('prepare_processes' in e for e in issues(dict(deepcopy(c), prepare_processes=4)))
    for bad in (0, 9, '4', True):
        assert any('prepare_processes' in e for e in issues(dict(deepcopy(c), prepare_processes=bad)))


def test_prepare_processes_same_plan(case):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    reference = run(p, obs, box, folder)
    p.site['prepare_processes'] = 2
    try:
        folder2 = folder.parent/('e'*32); folder2.mkdir()
        assert run(p, obs, box, folder2) == reference
        assert p.prepare_holder.get('pool') is not None
    finally:
        pool = p.prepare_holder.pop('pool', None)
        if pool is not None:
            pool.close()


# -- user decisions B and E (2026-11-23) -------------------------------------------------

def test_decision_switches_are_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    ok = dict(deepcopy(c), defer_pilz_solution_check=True, deadline_ompl_fallback='NOT_STARTED_WHEN_SHORT')
    assert not any(k in e for e in issues(ok) for k in ('defer_pilz_solution_check', 'deadline_ompl_fallback'))
    assert any('defer_pilz_solution_check' in e for e in issues(dict(ok, defer_pilz_solution_check='yes')))
    assert any('deadline_ompl_fallback' in e for e in issues(dict(ok, deadline_ompl_fallback='SKIP')))
    assert any('defer_pilz_solution_check' in e for e in issues(dict(ok, planning_module='NATIVE')))
    assert any('deadline_ompl_fallback' in e for e in issues(dict(ok, planning_module='NATIVE')))


def confirming(opts, confirm=True):
    """Fake worker side of decision B: a deferred Pilz reply confirms the deferral."""
    inner = opts['plan']

    def plan(r):
        out = inner(r)
        if r.get('solution_path_check') == 'DEFERRED_TO_SENT_PATH_AUDIT':
            assert r['pipeline_id'] == 'pilz_industrial_motion_planner' and r['defer_output_check_to_sent_path']
            if confirm and out is not None and out.get('status') == 'SUCCESS':
                out = dict(out, solution_path_check=dict(mode='DEFERRED_TO_SENT_PATH_AUDIT', waypoints=len(out['points'])))
        return out
    opts['plan'] = plan


def test_deferred_pilz_solution_check_same_plan_and_every_pilz_request_defers(case):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    opts['capabilities'] = dict(single_carton_output_check_deferral=True, pilz_solution_check_deferral=True)
    reference = run(p, obs, box, folder)
    p.site['defer_pilz_solution_check'] = True; confirming(opts)
    del calls[:]
    folder2 = folder.parent/('f'*32); folder2.mkdir()
    plan = p.plan_cycle(obs, [box], folder2)
    assert outcome(plan, folder2) == reference
    pilz = [c for c in calls if c.get('op') == 'plan']
    assert pilz and all(c['solution_path_check'] == 'DEFERRED_TO_SENT_PATH_AUDIT' for c in pilz)
    for segment in plan['segments']:
        evidence = segment['package']['evidence']
        assert evidence['planner']['solution_path_check'] == 'DEFERRED_TO_SENT_PATH_AUDIT'
        assert evidence['sent_path_collision'] == 'PASS'


def test_deferred_pilz_solution_check_fails_closed(case):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    p.site['defer_pilz_solution_check'] = True
    # A worker without the capability is refused at task start, not per candidate.
    opts['capabilities'] = dict(single_carton_output_check_deferral=True)
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_PILZ_SOLUTION_CHECK_DEFERRAL'):
        p.plan_cycle(obs, [box], folder)
    # A reply that does not confirm the deferral is never accepted.
    opts['capabilities'] = dict(single_carton_output_check_deferral=True, pilz_solution_check_deferral=True)
    confirming(opts, confirm=False)
    folder2 = folder.parent/('f'*32); folder2.mkdir()
    with pytest.raises(RuntimeError, match='SOLUTION_CHECK_DEFERRAL_NOT_CONFIRMED_BY_WORKER'):
        p.plan_cycle(obs, [box], folder2)


def test_deferred_check_ompl_fallback_keeps_the_configured_check(case):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    opts['capabilities'] = dict(single_carton_output_check_deferral=True, pilz_solution_check_deferral=True,
                                clearance_predicate_solution_check=True)
    p.site.update(defer_pilz_solution_check=True, solution_path_check='CLEARANCE_PREDICATE')
    inner = opts['plan']
    opts['plan'] = lambda r: (dict(status='NATIVE_PLANNING_FAILED', detail='FAKE')
                              if r['stage'] == 'return_empty' and r['pipeline_id'] != 'ompl' else inner(r))
    confirming(opts)
    p.plan_cycle(obs, [box], folder)
    ompl = [c for c in calls if c.get('pipeline_id') == 'ompl']
    assert ompl and all(c['solution_path_check'] == 'CLEARANCE_PREDICATE' for c in ompl)


def test_deferred_check_audit_rejection_of_approach_tries_the_next_ik_branch(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    from unloading_sim.workbench.single_carton.planner import SegmentRejected
    p.site['defer_pilz_solution_check'] = True
    screened = dict(screen=dict(conclusion='CLEAR', approach_q=[.1]*6, contact_q=[.2]*6, pending_branches=[[.3]*6, [.4]*6]))
    planned_with, checked = [], []
    monkeypatch.setattr(p, '_plan_candidate_segments',
                        lambda q, w, b, c, sub, s, *a: planned_with.append(s['screen']['approach_q']) or dict(rejected=[], profile='OUTWARD'))
    monkeypatch.setattr(p, 'endpoint_precheck', lambda q, w, b, c, att, sub, cancelled, x, qs, label: checked.append(qs[0]) or
                        dict(conclusion='CLEAR' if qs[0][0] > .35 else 'NOT_CONCLUSIVE', approach_q=qs[0], contact_q=[.5]*6))
    candidate = dict(tcp_pose=np.eye(4).tolist())
    planned = p.plan_candidate_segments([0.]*6, [], box, candidate, folder, screened)
    assert planned_with == [[.1]*6] and planned['pending_branches'] == [[.3]*6, [.4]*6]
    error = SegmentRejected('approach', 'approach:SENT_PATH_REJECTED:{"reason": "FAKE"}')
    reuse = p.replan_after(planned, error)
    again = p.plan_candidate_segments([0.]*6, [], box, candidate, folder, screened, reuse=reuse)
    # Same order as after a planning rejection: .3 is not clear, .4 is planned.
    assert checked == [[.3]*6, [.4]*6] and planned_with[-1] == [.4]*6
    assert again['ik_branch']['fallback_index'] == 2 and len(again['ik_branch']['earlier_failures']) == 1
    with pytest.raises(SegmentRejected):            # no branch left: the audit rejection is final
        p.replan_after(again, error)
    p.site['defer_pilz_solution_check'] = False     # switch off: unchanged, the candidate ends
    with pytest.raises(SegmentRejected):
        p.replan_after(planned, error)


def failing_approach(opts):
    opts['plan'] = lambda r: (dict(status='NATIVE_PLANNING_FAILED', detail='FAKE_PTP')
                              if r['stage'] == 'approach' and r['pipeline_id'] != 'ompl' else None)


def test_ompl_fallback_not_started_when_the_deadline_is_short(case):
    import time
    from unloading_sim.workbench.single_carton.planner import PlanningBudgetExhausted
    p, obs, box, folder, calls, opts = case
    p.site.update(deadline_ompl_fallback='NOT_STARTED_WHEN_SHORT', native_ompl_budget_s=5., native_ompl_budget_basis='TEST')
    failing_approach(opts)
    with pytest.raises(PlanningBudgetExhausted, match='OMPL_FALLBACK_NOT_STARTED') as raised:
        p.plan_cycle(obs, [box], folder, deadline=time.monotonic()+3.)
    assert not [r for r in calls if r.get('pipeline_id') == 'ompl']
    target = raised.value.evidence['targets'][0]
    assert target['infeasible_candidates'] == 0
    assert target['unsearched'] == target['considered'] == target['ompl_fallback_not_started_candidates'] > 0
    assert target['status'] == 'SEARCH_INCOMPLETE_OMPL_FALLBACK_NOT_STARTED_NO_PLAN'
    record = json.loads(next(folder.glob('*/approach-ompl-native.json')).read_text())
    assert record['status'] == 'NOT_DISPATCHED' and record['reason'] == 'OMPL_FALLBACK_NOT_STARTED_DEADLINE'
    assert record['remaining_native_s'] < record['ompl_budget_s'] == 5.
    raised.value.record_thread.join(5)


@pytest.mark.parametrize('policy, deadline_s', [('NOT_STARTED_WHEN_SHORT', 600.), ('CAPPED_BY_DEADLINE', 3.)])
def test_ompl_fallback_runs_with_enough_time_or_by_default(case, policy, deadline_s):
    import time
    p, obs, box, folder, calls, opts = case
    p.site.update(deadline_ompl_fallback=policy, native_ompl_budget_s=5., native_ompl_budget_basis='TEST')
    failing_approach(opts)
    plan = p.plan_cycle(obs, [box], folder, deadline=time.monotonic()+deadline_s)
    ompl = [r for r in calls if r.get('pipeline_id') == 'ompl']
    assert plan['full_path_validated'] and ompl
    assert all(r['allowed_planning_time_s'] == (5. if deadline_s > 5 else pytest.approx(r['allowed_planning_time_s'])) for r in ompl)
    if deadline_s < 5:
        assert all(r['allowed_planning_time_s'] < 3. for r in ompl)    # unchanged: capped by the deadline
    assert not p.fallback_not_started


# -- user decision C (2026-11-23): Python calls the worker's C++ prepare -------------------

def prepare_library_prefix():
    import os
    from pathlib import Path
    for prefix in (os.environ.get('M710_MOVEIT_INSTALL_PREFIX'), str(ROOT/'install/moveit-dev4'), str(ROOT/'install/moveit')):
        if prefix and (Path(prefix)/'lib/libm710_prepare.so').is_file():
            return prefix
    return None


def test_prepare_implementation_is_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert not any('prepare_implementation' in e for e in issues(dict(deepcopy(c), prepare_implementation='NATIVE_LIBRARY')))
    assert any('prepare_implementation' in e for e in issues(dict(deepcopy(c), prepare_implementation='CPP')))
    assert any('prepare_implementation' in e for e in issues(dict(deepcopy(c), prepare_implementation='NATIVE_LIBRARY', prepare_processes=2)))


def test_native_prepare_library_agrees_with_python_prepare(monkeypatch):
    prefix = prepare_library_prefix()
    if prefix is None:
        pytest.skip('libm710_prepare.so is not installed (development prefix install/moveit-dev4)')
    monkeypatch.setenv('M710_MOVEIT_INSTALL_PREFIX', prefix)
    from unloading_sim.workbench.single_carton import native_prepare
    from unloading_sim.workbench.controlled.trajectory import prepare, Limits
    limits = Limits((-4.,)*6, (4.,)*6, (.1,)*6, (.2,)*6)
    t = np.linspace(0., 6., 40); q = np.outer(np.sin(np.pi*t/12)**2, np.linspace(.05, .3, 6))
    v = np.gradient(q, t, axis=0); a = np.gradient(v, t, axis=0); v[[0, -1]] = 0.; a[[0, -1]] = 0.
    points = [dict(t=float(t[k]), q=q[k].tolist(), v=v[k].tolist(), a=a[k].tolist()) for k in range(len(t))]
    compared = 0
    for scale in (1., 1.5, 2., 4.):
        for grid in ('GEOMETRIC', 'SENT_GRID'):
            try:
                expected = prepare(points, limits, scale, .005, 1.8, probe_grid=grid)
            except ValueError as error:
                with pytest.raises(ValueError, match=str(error)):
                    native_prepare.prepare(points, limits, scale, .005, 1.8, grid)
                continue
            got = native_prepare.prepare(points, limits, scale, .005, 1.8, grid)
            assert got['prepare_implementation']['abi_version'] == 1 and len(got['prepare_implementation']['sha256']) == 64
            assert {k: v for k, v in got.items() if k != 'prepare_implementation'}.keys() == expected.keys()
            for key in ('collision_path',):
                assert np.max(abs(np.asarray(got[key])-np.asarray(expected[key]))) <= 1e-12
            for k in ('q', 'v', 'a', 't'):
                assert np.max(abs(np.asarray([p[k] for p in got['points']])-np.asarray([p[k] for p in expected['points']]))) <= 1e-12
            same = {k for k in expected['collision_sampling'] if k != 'analytic_speed_bounds_rad_s'}
            assert all(got['collision_sampling'][k] == expected['collision_sampling'][k] for k in same)
            assert np.allclose(got['collision_sampling']['analytic_speed_bounds_rad_s'],
                               expected['collision_sampling']['analytic_speed_bounds_rad_s'], rtol=0, atol=1e-12)
            compared += 1
    assert compared >= 4
    for bad, message in ((dict(scale=1000.), 'INVALID_TIMING'), (dict(reach=0.), 'INVALID_REACH_BOUND')):
        with pytest.raises(ValueError, match=message):
            native_prepare.prepare(points, limits, bad.get('scale', 2.), .005, bad.get('reach', 1.8), 'GEOMETRIC')


def test_prepare_implementation_native_library_is_used_and_fails_closed(case, monkeypatch, tmp_path):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    reference = run(p, obs, box, folder)
    from unloading_sim.workbench.single_carton import native_prepare
    from unloading_sim.workbench.controlled import trajectory
    used = []; original_load, original_prepare = native_prepare.load, native_prepare.prepare
    monkeypatch.setattr(native_prepare, 'load', lambda: (None, dict(name='FAKE')))
    monkeypatch.setattr(native_prepare, 'prepare', lambda *a, probe_grid='SENT_GRID': used.append(probe_grid) or trajectory.prepare(*a, probe_grid=probe_grid))
    p.site['prepare_implementation'] = 'NATIVE_LIBRARY'
    folder2 = folder.parent/('g'*32); folder2.mkdir()
    assert run(p, obs, box, folder2) == reference and used
    # A missing library stops the task before any planning.
    monkeypatch.setattr(native_prepare, 'load', original_load); monkeypatch.setattr(native_prepare, 'prepare', original_prepare)
    monkeypatch.setenv('M710_MOVEIT_INSTALL_PREFIX', str(tmp_path/'missing'))
    native_prepare.LIBRARY.clear()
    folder3 = folder.parent/('h'*32); folder3.mkdir()
    with pytest.raises(RuntimeError, match='NATIVE_PREPARE_LIBRARY_MISSING'):
        p.plan_cycle(obs, [box], folder3)


# -- audit_chunking WORK_QUEUE (2026-11-23, scheduling only) ------------------------------

def adaptive_fake(p, opts, monkeypatch, n=9, reject=()):
    from unloading_sim import moveit2_backend
    p.site.update(concurrent_audits=True, sent_path_audit='ADAPTIVE'); opts['capabilities'] = dict(adaptive_sent_path_audit=True)
    inner = moveit2_backend.ResidentMoveItClient()

    class Tagged:
        def __init__(self, name):
            self.name = name

        def request(self, r, **kwargs):
            r = dict(r); r['_worker'] = self.name
            reply = inner.request(r, **kwargs)
            if r.get('op') == 'validate' and 'probe_path' in r:
                if r['stage'] in reject:
                    reply = dict(reply, native_valid=False, path_failure={'reason': 'FAKE_SENT_PATH_REJECTED', 'edge': 1})
                if r.get('sent_path_audit') == 'ADAPTIVE':
                    reply = dict(reply, clearance=dict(sent_path_audit=dict(sampled_states=len(r['probe_path']), skipped_states=0,
                        anchor_dynamic_pair_distance_lower_bound_m=None, anchor_workspace_margin_m=None, margin_cap_m=.05,
                        proof_epsilon_m=1e-6, skip_rule='FAKE')))
            return reply
    workers = [Tagged(f'w{k}') for k in range(n)]
    monkeypatch.setattr(p, 'checkers', lambda: workers)
    monkeypatch.setattr(p, 'scene_handle', lambda client, folder, cancelled=None: 'H')
    return workers


def test_audit_work_queue_is_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert not any('audit_chunking' in e for e in issues(dict(deepcopy(c), concurrent_audits=True, audit_chunking='WORK_QUEUE')))
    assert any('audit_chunking' in e for e in issues(dict(deepcopy(c), concurrent_audits=True, audit_chunking='RANDOM')))
    assert any('audit_chunking' in e for e in issues(dict(deepcopy(c), audit_chunking='WORK_QUEUE')))


def test_audit_work_queue_covers_every_segment_once_and_gives_the_same_plan(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box); adaptive_fake(p, opts, monkeypatch)
    reference = run(p, obs, box, folder)
    p.site['audit_chunking'] = 'WORK_QUEUE'
    del calls[:]
    folder2 = folder.parent/('q'*32); folder2.mkdir()
    plan = p.plan_cycle(obs, [box], folder2)
    assert outcome(plan, folder2) == reference
    probes = [c for c in calls if c.get('op') == 'validate' and 'probe_path' in c]
    stages_per_worker = {}
    for segment in plan['segments']:
        path = segment['package']['prepared']['collision_path']
        chunks = sorted((c for c in probes if c['stage'] == segment['stage']), key=lambda c: path.index(c['probe_path'][0]))
        # Contiguous chunks with shared boundary states: every probe state once.
        assert chunks[0]['probe_path'][0] == path[0] and chunks[-1]['probe_path'][-1] == path[-1]
        assert all(x['probe_path'][-1] == y['probe_path'][0] for x, y in zip(chunks, chunks[1:]))
        assert sum(len(c['probe_path']) for c in chunks)-(len(chunks)-1) == len(path)
        assert segment['package']['evidence']['sent_path_collision'] == 'PASS'
    for c in probes:
        stages_per_worker.setdefault(c['_worker'], set()).add(c['stage'])
    assert len(p.authority._plans) == 6


@pytest.mark.parametrize('rejected', [('detach',), ('return_empty', 'approach'), ('withdraw',)])
def test_audit_work_queue_rejects_like_the_subset_scheme(case, monkeypatch, rejected):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box); adaptive_fake(p, opts, monkeypatch, reject=rejected)
    reference = run(p, obs, box, folder)
    p.site['audit_chunking'] = 'WORK_QUEUE'
    folder2 = folder.parent/('r'*32); folder2.mkdir()
    assert run(p, obs, box, folder2) == reference
    assert p.authority._plans == {}


def test_adaptive_rule_transforms_switch_reaches_every_adaptive_audit_request(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert any('adaptive_rule_transforms' in e for e in issues(dict(deepcopy(c), adaptive_rule_transforms='FAST')))
    assert any('adaptive_rule_transforms' in e for e in issues(dict(deepcopy(c), adaptive_rule_transforms='LINK_TRANSFORMS_ONLY', sent_path_audit='SAMPLED')))
    assert not any('adaptive_rule_transforms' in e for e in issues(dict(deepcopy(c), adaptive_rule_transforms='LINK_TRANSFORMS_ONLY',
                                                                      sent_path_audit='ADAPTIVE_CONTACT')))
    p.site.update(sent_path_audit='ADAPTIVE_CONTACT', adaptive_rule_transforms='LINK_TRANSFORMS_ONLY')
    opts['capabilities'] = dict(adaptive_sent_path_audit=True, adaptive_contact_sent_path_audit=True)
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_ADAPTIVE_RULE_LINK_TRANSFORMS'):
        p.plan_cycle(obs, [box], folder)
    opts['capabilities']['adaptive_rule_link_transforms'] = True
    folder2 = folder.parent/('t'*32); folder2.mkdir()
    try:
        p.plan_cycle(obs, [box], folder2)
    except ValueError as error:   # the fake worker reports no adaptive audit evidence; only the requests matter here
        assert 'NO_COMPLETE_FEASIBLE' in str(error)
    audits = [r for r in calls if r.get('op') == 'validate' and 'probe_path' in r]
    assert audits and all(r.get('adaptive_rule_transforms') == 'LINK_TRANSFORMS_ONLY' for r in audits)


# -- digest_memo (2026-11-23): issue-side subtree memo; checks always re-read ---------------

def test_tree_digest_memo_gives_the_same_digest_and_checks_re_read():
    from unloading_sim.workbench.plan_digest import tree_digest
    from unloading_sim.workbench.controlled.trajectory import Authority
    rows = [dict(t=float(k), q=[float(k)]*6) for k in range(40)]
    value = dict(points=rows, path=[[float(k)]*6 for k in range(50)], name='x')
    memo = {}
    assert tree_digest(value, memo) == tree_digest(value) and memo
    assert tree_digest(value, memo) == tree_digest(value)
    authority = Authority('eco65_single_carton_segment_v1')
    with pytest.raises(ValueError, match='SIGNING_MEMO_REQUIRES_TREE_DIGEST'):
        authority.use_signing_memo()
    authority.use_tree_digest(); authority.use_signing_memo()
    prepared = dict(points=rows, collision_path=value['path'])
    authority.prewarm([prepared['points'], prepared['collision_path']])
    evidence = dict(native_plan='PASS', sent_path_collision='PASS')
    package = authority.register(prepared, dict(b=1), evidence)
    fresh = Authority('eco65_single_carton_segment_v1'); fresh.use_tree_digest()
    assert authority._plans[package['plan_id']] == fresh._digest(package)     # same digest as without a memo
    # A list changed in place after it was memoized: issue reused the old
    # subtree digest, the check re-reads and refuses.
    rows[3]['q'][0] = 99.
    with pytest.raises(ValueError, match='UNTRUSTED_STALE_OR_PREVIEW_TRAJECTORY'):
        authority.claim(package, dict(b=1))
    authority.invalidate()
    assert authority.signing_memo == {}


def test_digest_memo_same_plan_and_ready_hash_equal_to_a_full_re_read(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    six_segment(p, opts, box)
    from unloading_sim.workbench.single_carton.cycle import Cycle
    from unloading_sim.workbench.plan_digest import tree_digest
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert any('digest_memo' in e for e in issues(dict(deepcopy(c), digest_memo=True)))       # needs structured_plan_digest
    assert not any('digest_memo' in e for e in issues(dict(deepcopy(c), digest_memo=True, structured_plan_digest=True)))
    p.site['structured_plan_digest'] = True; p.authority.use_tree_digest()
    reference = run(p, obs, box, folder)
    p.site['digest_memo'] = True; p.authority.use_signing_memo()
    seen = {}

    def finish(plan, guard):
        class Vacuum:
            holding = False
        cycle = Cycle(None, Vacuum(), {}, lambda **event: None, None, None)
        cycle.ready('t', plan, digest_memo=p.digest_memo())
        seen['cycle'] = cycle
    folder2 = folder.parent/('m'*32); folder2.mkdir()
    plan = p.plan_cycle(obs, [box], folder2, finalize=finish)
    assert outcome(plan, folder2) == reference
    cycle = seen['cycle']
    assert p.digest_memo() and cycle.plan_hash == tree_digest(plan)            # ready with memo == full re-read
    for segment in plan['segments']:
        package = segment['package']
        assert p.authority._plans[package['plan_id']] == tree_digest(package)
    plan['segments'][2]['package']['prepared']['points'][1]['q'][0] += 1e-9
    assert cycle.plan_digest(cycle.plan) != cycle.plan_hash                    # start() re-reads: PLAN_CHANGED
