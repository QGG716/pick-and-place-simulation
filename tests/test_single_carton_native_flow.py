"""Step-7 Python side: binary frames, canonical digest, planning_module, replay client (no worker)."""
import io
import json
from pathlib import Path
import struct

import numpy as np
import pytest

from unloading_sim.workbench import binary_frames as bf
from unloading_sim.workbench.controlled.trajectory import Authority, Limits, digest, prepare
from unloading_sim.workbench.single_carton.config import issues, load
from unloading_sim.workbench.single_carton.native_flow import NativeFlowDiverged, ReplayClient, prepare_key, replay_key

ROOT = Path(__file__).resolve().parents[1]


def sample(seed=0):
    rng = np.random.default_rng(seed)
    return dict(op='validate', identity=dict(name='x', flange_from_task_tcp=[[1., 0.], [0., 1.]]), seed=6518,
                probe_path=rng.normal(size=(300, 6)).tolist(), q_start=rng.normal(size=6).tolist(),
                ints=list(range(100)), mixed=[0, .5]*40, short=[.1, .2], nested=[dict(t=float(i), q=[.1*i]*6) for i in range(3)],
                big_vector=rng.normal(size=128).tolist(), text='检查', flag=True, none=None, neg_zero=[-0.0]*70)


def test_split_merge_restores_identical_values_and_types():
    value = sample()
    header, blocks = bf.split(value)
    assert set(header['probe_path']) == {'$block'} and set(header['big_vector']) == {'$block'} and len(blocks) == 3
    assert blocks[header['probe_path']['$block']].shape == (300, 6)
    # Integers, mixed lists and short float lists stay JSON.
    assert isinstance(header['ints'], list) and isinstance(header['mixed'], list) and header['short'] == [.1, .2]
    restored = bf.merge(header, blocks)
    assert json.dumps(restored, sort_keys=True) == json.dumps(value, sort_keys=True)
    assert np.signbit(restored['neg_zero'][0])


def test_frame_round_trip_and_wire_reader():
    value = sample(1)
    data = bf.encode(bf.REPLY, 7, value)
    assert data[:4] == bf.MAGIC and struct.unpack('<H', data[4:6])[0] == 2
    stream = io.BytesIO(data+(json.dumps(dict(status='READY'))+'\n').encode()+bf.encode(bf.REPLY, 8, dict(x=[1.5]*64)))
    wire = bf.Wire(type('P', (), dict(stdout=stream, stdin=io.BytesIO()))())
    replies = list(wire.replies())
    assert replies[0] == value and replies[1] == dict(status='READY') and replies[2] == dict(x=[1.5]*64)
    assert wire.received_bytes == len(stream.getvalue())


def test_wire_sends_json_until_binary_is_declared_then_frames():
    sink = io.BytesIO()
    wire = bf.Wire(type('P', (), dict(stdin=sink, stdout=io.BytesIO()))())
    wire.send(dict(op='init'))
    assert sink.getvalue().endswith(b'\n') and not wire.activate(dict(capabilities=dict(binary_protocol=1)))
    assert wire.activate(dict(capabilities=dict(binary_protocol=2)))
    before = len(sink.getvalue()); wire.send(dict(op='validate', probe_path=[[0.]*6]*20)); wire.cancel('r-1')
    frames = io.BytesIO(sink.getvalue()[before:])
    kind, _, value = bf.read_frame(lambda n: frames.read(n))
    assert kind == bf.REQUEST and value['probe_path'] == [[0.]*6]*20
    kind, _, value = bf.read_frame(lambda n: frames.read(n))
    assert kind == bf.CANCEL and value == dict(op='cancel', target_request_id='r-1')


def test_canonical_digest_depends_on_every_value_and_not_on_key_order():
    value = sample(2)
    reordered = {k: value[k] for k in reversed(list(value))}
    assert bf.canonical_digest(value) == bf.canonical_digest(reordered)
    changed = json.loads(json.dumps(value)); changed['probe_path'][150][3] = np.nextafter(changed['probe_path'][150][3], 1.)
    assert bf.canonical_digest(changed) != bf.canonical_digest(value)
    assert bf.canonical_digest(value) != digest(value)


def test_authority_binary_digest_is_used_at_issue_and_claim():
    authority = Authority('eco65_single_carton_segment_v1')
    authority.use_canonical_binary_digest()
    package = authority.register(dict(points=[dict(q=[.1]*6)]*30, collision_path=[[.2]*6]*200), dict(b=1),
                                 dict(native_plan='PASS', sent_path_collision='PASS'))
    assert authority.digest_method == 'CANONICAL_BINARY_SHA256'
    authority.verify(package, dict(b=1))
    tampered = json.loads(json.dumps(package)); tampered['prepared']['collision_path'][100][0] += 1e-15
    with pytest.raises(ValueError):
        authority.verify(tampered, dict(b=1))
    authority.claim(package, dict(b=1))
    with pytest.raises(ValueError, match='AUTHORITY_DIGEST_METHOD_FIXED'):
        other = Authority('eco65_single_carton_segment_v1')
        other.register(dict(), dict(), dict(native_plan='PASS', sent_path_collision='PASS'))
        other.use_canonical_binary_digest()


@pytest.mark.parametrize('value,ok', [(None, True), ('PYTHON', True), ('NATIVE', True), ('native', False), ('CPP', False)])
def test_planning_module_configuration(value, ok):
    config = load(ROOT/'configs/workbench/single_carton.example.json')
    if value is None:
        config.pop('planning_module', None)
    else:
        config['planning_module'] = value
    assert (not [e for e in issues(config) if e.startswith('planning_module')]) == ok


def test_example_configuration_keeps_python_planning():
    data = json.loads((ROOT/'configs/workbench/single_carton.example.json').read_text())
    assert data.get('planning_module', 'PYTHON') == 'PYTHON'


def test_replay_key_ignores_only_volatile_fields():
    base = dict(op='plan', stage='approach', q_start=[0.]*6, scene_fingerprint='f', world=[dict(id='a')])
    assert replay_key(dict(base, request_budget_s=110., request_id='1'), None) == replay_key(dict(base, world=[]), None)
    assert replay_key(dict(base, allowed_planning_time_s=25.), None) != replay_key(dict(base, allowed_planning_time_s=3.), None)
    assert replay_key(dict(base, allowed_planning_time_s=25.), 1.) == replay_key(dict(base, allowed_planning_time_s=3.), 1.)
    assert replay_key(base, None) != replay_key(dict(base, q_start=[0.]*5+[1e-17]), None)
    # A probe path is identified by its length and end states, as the worker echoes it.
    path = [[float(i)]*6 for i in range(10)]
    echoed = dict(base, probe_path=dict(length=10, first=path[0], last=path[-1]))
    assert replay_key(dict(base, probe_path=path), None) == replay_key(echoed, None)
    assert replay_key(dict(base, probe_path=path[:-1]), None) != replay_key(echoed, None)


def test_replay_serves_recorded_results_once_and_reports_divergence():
    request = dict(op='validate', stage='approach', q_start=[0.]*6)
    replay = ReplayClient(target=None, deadline=None)
    replay.add(dict(calls=[dict(index=0, name='x', request=dict(request, request_budget_s=110.), result=dict(native_valid=True),
                                elapsed_s=.01)], prepares=[]))
    assert replay.request(dict(request, request_budget_s=590.))['native_valid'] is True
    assert replay.unused() == 0
    with pytest.raises(NativeFlowDiverged):
        replay.request(request)
    stopped = ReplayClient(None, None); stopped.add(dict(calls=[], prepares=[], stopped='CANCELLED'))
    with pytest.raises(Exception, match='CANCELLED'):
        stopped.request(request)


def test_replayed_prepare_has_the_python_structure():
    points = [dict(t=0., q=[0.]*6, v=[0.]*6, a=[0.]*6), dict(t=8., q=[.3, -.2, .1, 0., .2, -.1], v=[0.]*6, a=[0.]*6)]
    limits = Limits((-3.,)*6, (3.,)*6, (.1,)*6, (.2,)*6)
    python = prepare(points, limits, 1.25, .005, 1.4)
    native = dict(scaled_duration_s=python['scaled_duration_s'], source_duration_s=python['source_duration_s'],
                  send_duration_s=python['send_duration_s'], terminal_hold_s=python['terminal_hold_s'],
                  subdivisions_per_send=python['collision_sampling']['subdivisions_per_send'],
                  analytic_speed_bounds_rad_s=python['collision_sampling']['analytic_speed_bounds_rad_s'],
                  t=[p['t'] for p in python['points']], q=[p['q'] for p in python['points']], v=[p['v'] for p in python['points']],
                  a=[p['a'] for p in python['points']], collision_path=python['collision_path'])
    replay = ReplayClient(None, None)
    replay.add(dict(calls=[], prepares=[dict(points=points, time_scale=1.25, period_s=.005, reach_bound_m=1.4, status='PREPARED',
                                              prepared=native),
                                         dict(points=points, time_scale=1., period_s=.005, reach_bound_m=1.4,
                                              status='PREPARE_REJECTED', error='INTERPOLATION_LIMIT:1:1')]))
    assert replay.prepare_segment(points, limits, 1.25, .005, 1.4) == python
    with pytest.raises(ValueError, match='INTERPOLATION_LIMIT:1:1'):
        replay.prepare_segment(points, limits, 1., .005, 1.4)
    with pytest.raises(NativeFlowDiverged):
        replay.prepare_segment(points, limits, 1.5, .005, 1.4)
    assert prepare_key(points, 1.25, .005, 1.4) != prepare_key(points, 1.25, .005, 1.4000000000000001+1e-15)
