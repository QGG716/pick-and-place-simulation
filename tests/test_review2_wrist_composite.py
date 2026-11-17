"""Actual C++ composite output drives actual Python planning/replay, fake IO only."""
import copy
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest

from test_single_carton_planner import case
from unloading_sim.workbench.single_carton.config import wrist_branch_limits
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.native_flow import NativeFlow, ReplayClient
from unloading_sim.workbench.single_carton.planner import SegmentRejected

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def composite(tmp_path_factory):
    binary = tmp_path_factory.mktemp('composite')/'runner'
    subprocess.run(['g++', '-std=c++17', '-O1', '-I'+str(ROOT/'ros2/m710_moveit_backend/src'),
        '-I/usr/include/eigen3', str(ROOT/'tests/native_review2_composite.cpp'), '-o', str(binary)], check=True)
    def run(request, replies, folder):
        request = dict(request, journal_directory=str(folder), request_id='review2', task_id=folder.name)
        result = subprocess.run([str(binary)], input=json.dumps(dict(request=request, replies=replies)),
            text=True, capture_output=True, check=True, timeout=30)
        return json.loads(result.stdout)
    return run


def setup_case(case):
    p, obs, box, folder, calls, opts = case
    p.wrist_limits = wrist_branch_limits(dict(wrist_branch_limits=dict(keep_j5_sign=True,
        j5_min_abs_deg=5., max_j4_travel_deg=45., max_j6_travel_deg=45., basis='FAKE_IO_REGRESSION')))
    q = np.deg2rad([0., 0., 0., 0., 30., 0.]).tolist()
    p.wrist_reference = q
    candidate = contact_candidates(box, p.source['tool'], p.site['topology'])[0]
    world = [o for o in p.world if o.get('category') != 'carton'] + [dict(
        id=box['id'], pose=box['pose'], size=box['size_m'], category='carton')]
    return p, q, world, box, candidate, folder


@pytest.mark.parametrize('scenario', ['flip', 'band', 'j4', 'j6', 'partial', 'normal', 'contact_flip', 'qstart_band'])
def test_endpoint_composite_replays_same_wrist_decisions(case, composite, scenario):
    p, q, world, box, candidate, folder = setup_case(case)
    good = list(q); good[0] = .01
    bad = list(good)
    if scenario in ('flip', 'partial', 'contact_flip'): bad[4] *= -1
    elif scenario == 'band': bad[4] = np.deg2rad(2.)
    elif scenario == 'j4': bad[3] = np.deg2rad(46.)
    elif scenario == 'j6': bad[5] = np.deg2rad(46.)
    elif scenario == 'qstart_band': q[4] = np.deg2rad(2.)
    replies = []; ik = 0
    class Client:
        def request(self, request, **kwargs):
            nonlocal ik
            if request['op'] == 'ik':
                contact = request['goal_pose'] == candidate['tcp_pose']
                value = good if scenario == 'normal' or (contact and scenario != 'contact_flip') else bad
                if scenario == 'partial': value = good if ik % 2 else bad
                ik += 1
                result = dict(status='SUCCESS', q=value, position_error_m=0., orientation_error_rad=0.)
            else: result = dict(native_valid=True, legacy_intersection_valid=True, failure=None)
            replies.append(copy.deepcopy(result))
            return result
    client = Client();p.client = client
    request = NativeFlow(p).screen_request(q, world, box, [candidate], [0], client, folder)
    expected = p.screen_candidate(q, world, box, candidate, folder)
    result = composite(request, replies, folder)
    replay = ReplayClient(client, None);replay.add(result);p.client = replay
    actual = p.screen_candidate(q, world, box, candidate, folder)
    assert actual == expected
    assert result['outcomes'][0]['outcome'] == expected['screen']['conclusion']
    assert result['stopped'] is None and replay.unused() == 0
    assert replay.served == len(replies) == result['script_consumed']


@pytest.mark.parametrize('stage', ['approach', 'contact', 'detach', 'return', None])
@pytest.mark.parametrize('known', [False, True])
def test_segment_middle_violation_preserves_composite_replay(case, composite, stage, known, monkeypatch):
    p, q, world, box, candidate, folder = setup_case(case)
    replies = []
    class Client:
        def request(self, request, **kwargs):
            if request['op'] == 'plan':
                name = request['stage']
                goal = request.get('q_goal', list(q))[:]
                if 'q_goal' not in request: goal[0] = dict(approach=.01, contact=.02, detach=.03)[name]
                states = [request['q_start']]
                if name == stage:
                    middle = list(q); middle[3] = np.deg2rad(46.)
                    states.append(middle)
                states.append(goal)
                result = dict(status='SUCCESS', native_output_status='PASS', points=[dict(
                    t=float(i*30), q=s, v=[0.]*6, a=[0.]*6) for i,s in enumerate(states)])
            else: result = dict(native_valid=True, legacy_intersection_valid=True, failure=None, path_failure=None)
            replies.append(copy.deepcopy(result))
            return result
    client = Client();p.client = client
    screened = dict(screen=dict(conclusion='NOT_CONCLUSIVE'), release_supports=[])
    if known: screened['screen']['approach_q'] = [.01, *q[1:]]
    def outcome(call):
        try:
            result = call()
            return ('PASS', [s['stage'] for s in result['segments']])
        except ValueError as e:
            return ('REJECTED', str(e).split(':WRIST_BRANCH_REJECTED:')[0])
    expected = outcome(lambda: p.plan_candidate(q, world, box, candidate, folder, screened,
        cancelled=lambda:False, clients=[client], workers=[client]))
    collected = []; offset = 0
    flow = NativeFlow(p)
    def send(native, request, sub, name, cancelled=None):
        nonlocal offset
        result = composite(request, replies[offset:], sub)
        offset += result['script_consumed'];collected.append(result)
        return result
    flow.send = send
    cancelled = lambda:False
    original_handle = flow.handle
    uploads = []
    def checked_handle(request, client, sub, callback=None):
        assert callback is cancelled
        uploads.append(request['stage'])
        return original_handle(request, client, sub, callback)
    flow.handle = checked_handle
    replays = []
    class CapturedReplay(ReplayClient):
        def __init__(self, *args):
            super().__init__(*args);replays.append(self)
    monkeypatch.setattr('unloading_sim.workbench.single_carton.native_flow.ReplayClient', CapturedReplay)
    actual = outcome(lambda: flow.plan_one(q, world, box, candidate, folder, screened,
        lambda _:None, cancelled, client, [client]))
    assert actual == expected
    assert offset == len(replies)
    assert all(r['stopped'] is None for r in collected)
    assert replays and all(r.unused() == 0 for r in replays)
    if stage is None:assert set(uploads) == {'approach', 'contact', 'detach', 'return'}
