"""Step-6 Python side of sent_path_audit; fake native workers only."""
from pathlib import Path
import numpy as np
import pytest
from test_single_carton_planner import case  # noqa: F401  (fixture reuse)
from unloading_sim.workbench.controlled.trajectory import Limits, prepare
from unloading_sim.workbench.single_carton.config import issues, load
from unloading_sim.workbench.single_carton.planner import SinglePlanner

ROOT = Path(__file__).resolve().parents[1]


def adaptive_reply(r):
    n = len(r['probe_path'])
    audit = dict(method='ADAPTIVE', sampled_states=2*n, checked_states=n//3+1, skipped_states=2*n-n//3-1, anchors=n//3+1,
                 minimum_distance_lower_bound_m=.0031, minimum_workspace_margin_m=None, margin_cap_m=.05, proof_epsilon_m=1e-6,
                 constraint_s=0., proof_s=0., skip_rule='PER_GROUP_PAIR_LEVER_BOUND_FROM_LAST_EXACT_SAMPLE', samples='IDENTICAL_TO_SAMPLED')
    clearance = dict(counts=dict(output=dict(queries=n//3+1, seconds=.01)))
    if r.get('sent_path_audit') == 'ADAPTIVE':
        clearance['sent_path_audit'] = audit
    return dict(native_valid=True, legacy_intersection_valid=True, path_failure=None, failure=None, clearance=clearance)


@pytest.mark.parametrize('value,ok', [(None, True), ('SAMPLED', True), ('ADAPTIVE', True), ('adaptive', False), ('DENSE', False)])
def test_sent_path_audit_configuration(value, ok):
    config = load(ROOT/'configs/workbench/single_carton.example.json')
    if value is None:
        config.pop('sent_path_audit', None)
    else:
        config['sent_path_audit'] = value
    assert (not [e for e in issues(config) if e.startswith('sent_path_audit')]) == ok


def test_example_configuration_keeps_sampled():
    import json
    data = json.loads((ROOT/'configs/workbench/single_carton.example.json').read_text())
    assert data.get('sent_path_audit', 'SAMPLED') == 'SAMPLED'


def test_sampled_default_sends_no_audit_field_and_records_the_method(case):
    p, obs, box, folder, calls, opts = case
    plan = p.plan_cycle(obs, [box], folder)
    assert not any('sent_path_audit' in r for r in calls)
    for segment in plan['segments']:
        audit = segment['package']['evidence']['collision_result']['sent_path_audit']
        assert audit['audit_method'] == 'SAMPLED' and audit['skipped_states'] == 0
        assert audit['continuous_swept_proof'] == 'BOUNDED_DISPLACEMENT_1_4_MM'


def test_adaptive_only_on_segments_without_contact_or_support(case):
    p, obs, box, folder, calls, opts = case
    p.site['sent_path_audit'] = 'ADAPTIVE'
    opts['capabilities'] = dict(adaptive_sent_path_audit=True)
    opts['sent'] = adaptive_reply
    plan = p.plan_cycle(obs, [box], folder)
    sent = [r for r in calls if r.get('op') == 'validate' and 'probe_path' in r]
    assert sent
    for r in sent:
        spec = r['single_carton_contact']
        assert (r.get('sent_path_audit') == 'ADAPTIVE') == (not spec['allow_contact'] and not spec['supports'])
    assert not [r for r in sent if r['stage'] == 'contact' and 'sent_path_audit' in r]
    assert [r for r in sent if r['stage'] == 'approach' and r.get('sent_path_audit') == 'ADAPTIVE']
    # Never on endpoint checks, IK or planning requests.
    assert not [r for r in calls if 'sent_path_audit' in r and not (r.get('op') == 'validate' and 'probe_path' in r)]
    methods = {s['stage']: s['package']['evidence']['collision_result']['sent_path_audit'] for s in plan['segments']}
    assert methods['approach']['audit_method'] == 'ADAPTIVE' and methods['approach']['skipped_states'] > 0
    assert methods['approach']['minimum_distance_lower_bound_m'] == pytest.approx(.0031)
    assert methods['contact']['audit_method'] == 'SAMPLED'


def test_adaptive_without_worker_capability_is_refused_before_any_check(case):
    p, obs, box, folder, calls, opts = case
    p.site['sent_path_audit'] = 'ADAPTIVE'
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_ADAPTIVE_SENT_PATH_AUDIT'):
        p.plan_cycle(obs, [box], folder)
    assert not [r for r in calls if r.get('op') in ('validate', 'plan', 'ik')]


def test_adaptive_reply_without_audit_evidence_is_rejected():
    prepared = dict(collision_path=[[0.]*6]*3, collision_sampling=dict(maximum_pair_displacement_between_checks_m=.001))
    with pytest.raises(ValueError, match='NATIVE_WORKER_DID_NOT_REPORT_ADAPTIVE_AUDIT'):
        SinglePlanner.audit_summary('ADAPTIVE', [dict(clearance=dict(counts={}))], prepared, True)
    rejected = SinglePlanner.audit_summary('SAMPLED', [dict(clearance=dict(counts=dict(output=dict(queries=7))))], prepared, False)
    assert rejected['checked_states'] == 7 and rejected['continuous_swept_proof'] is None


@pytest.mark.parametrize('seed', range(6))
def test_probe_states_bound_any_pair_displacement_by_1_mm(seed):
    """The step bound that the continuous 1.4 mm guarantee uses.

    Between consecutive probe states of the prepared sent path, any body pair
    moves at most 4 R sum_j |dq_j| (lever 2R per moving body, two bodies);
    prepare spaces the probe states so that this is <= 1 mm, and the exact
    displacement integral sum_j |dq_j| never exceeds sum_j V_j dt.
    """
    rng = np.random.default_rng(seed)
    q0 = rng.uniform(-1, 1, 6); q1 = q0+rng.uniform(-.4, .4, 6); q2 = q1+rng.uniform(-.4, .4, 6)
    points = [dict(t=0., q=q0.tolist(), v=[0.]*6, a=[0.]*6), dict(t=6., q=q1.tolist(), v=(rng.uniform(-.02, .02, 6)).tolist(), a=[0.]*6),
              dict(t=12., q=q2.tolist(), v=[0.]*6, a=[0.]*6)]
    limits = Limits((-3.,)*6, (3.,)*6, (.1,)*6, (.2,)*6)
    reach = float(rng.uniform(.8, 2.))
    prepared = prepare(points, limits, 4., .005, reach)
    path = np.asarray(prepared['collision_path'])
    steps = np.abs(np.diff(path, axis=0)).sum(axis=1)
    assert np.all(4*reach*steps <= .001+1e-12)
    assert prepared['collision_sampling']['maximum_pair_displacement_between_checks_m'] == .001
    # Uniform time scaling keeps the geometric path: the same probe-state
    # density bound holds, so a slowed segment needs no new audit.
    slower = np.asarray(prepare(points, limits, 8., .005, reach)['collision_path'])
    assert np.all(4*reach*np.abs(np.diff(slower, axis=0)).sum(axis=1) <= .001+1e-12)
    assert np.allclose(slower[0], path[0]) and np.allclose(slower[-1], path[-1])
