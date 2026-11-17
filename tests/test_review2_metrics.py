"""Full-sample coverage and scope; no hardware or replay required."""
from test_single_carton_bench import bench


def test_untimed_partial_failure_prevents_full_group_acceptance():
    samples = [dict(events=[], seconds=1., success=True, scope='FULL_PREPARE_PLAN'),
        dict(events=[], seconds=None, success=False, scope='PARTIAL_REPLAY_NOT_PREPARE_PLAN')]
    result = bench.summarize('planning', samples, dict(metric='planning', stages=[], total_s=2.), 1.5)
    assert result['meets_total_p95'] is None
    assert result['status'] == 'PARTIAL_SCOPE'
    assert result['coverage'] == dict(full=1, partial=1, missing=0, untimed=1,
        full_timed=1, full_timed_fraction=.5)
    assert result['success_rate'] == .5
    assert result['successful_full_samples']['p95_s'] == 1.


def test_untimed_full_failure_is_missing_coverage_and_outcomes_are_kept():
    samples = [dict(events=[], seconds=1., success=True, scope='FULL_PREPARE_PLAN')]
    samples += [dict(events=[], seconds=None, success=False, scope='FULL_PREPARE_PLAN', outcome=o)
        for o in ['FAILED', 'CANCELLED', 'PLANNING_BUDGET_EXHAUSTED']]
    result = bench.summarize('planning', samples, dict(metric='planning', stages=[], total_s=2.), 1.5)
    assert result['meets_total_p95'] is None and result['status'] == 'INCOMPLETE_COVERAGE'
    assert result['coverage']['missing'] == 3 and result['coverage']['full_timed_fraction'] == .25
    assert result['outcomes'] == dict(SUCCESS=1, FAILED=1, CANCELLED=1, PLANNING_BUDGET_EXHAUSTED=1)


def test_discovery_includes_field_collector_and_explicit_roots(tmp_path, monkeypatch):
    monkeypatch.setattr(bench, 'ROOT', tmp_path)
    historical = tmp_path/'outputs/single_carton/old'
    field = tmp_path/'outputs/realtime-field-20261117/new'
    custom = tmp_path/'custom/task'
    for directory in (historical, field, custom):
        directory.mkdir(parents=True)
        for name in bench.RECORD_FILES:(directory/name).write_text('fake')
    assert bench.discover_roots() == sorted([historical, field])
    assert bench.discover_roots([custom.parent, custom.parent]) == [custom]


def test_success_latency_is_separate_from_failed_return_latency():
    samples = [dict(events=[], seconds=1., success=True, scope='PREPARE_PLAN_ENTRY_TO_PLAN_READY'),
        dict(events=[], seconds=119., success=False, scope='PREPARE_PLAN_ENTRY_TO_FAILED_RETURN')]
    result = bench.summarize('planning', samples, dict(metric='planning', stages=[], total_s=2.), 1.5)
    assert result['coverage']['full'] == 2 and result['coverage']['missing'] == 0
    assert result['p95_s'] == 119. and result['successful_full_samples']['p95_s'] == 1.
    assert result['meets_total_p95'] is False and result['success_rate'] == .5
