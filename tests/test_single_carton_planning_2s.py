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

