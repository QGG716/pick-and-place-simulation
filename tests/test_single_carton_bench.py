"""Latency bench arithmetic on synthetic timing files; no record, device or worker."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bench', ROOT/'tools/bench_planning_latency.py')
bench = importlib.util.module_from_spec(spec); spec.loader.exec_module(bench)
BUDGET = json.loads((ROOT/'configs/workbench/planning_budget.json').read_text())


def span(name, seconds, start=0., **attrs):
    return dict(name=name, start_monotonic_s=start, duration_s=seconds, **attrs)


def planning_doc(total, screening, sent, ok=True):
    events = [span('prepare_plan.plan_cycle', total), span('search.screen_candidate', screening),
              span('search.sequential_plan_candidate', 1.0+sent), span('sent_check', sent),
              span('native.main_worker_process_start', .01), span('native.request', 3.8, record='main-init'),
              span('json.dumps', .02, file='complete-plan.json'), span('disk.write', .01, file='complete-plan.json'),
              span('json.dumps', .5, file='site-scene.json')]
    events.append(span('prepare_plan.entry_to_return',total))
    if ok:
        events.append(span('prepare_plan.entry_to_plan_ready', total))
    return dict(schema='eco65_timing_v1', phase='prepare_plan', task_id='synthetic',
                final_phase='PLAN_READY' if ok else 'FAILED', events=events)


def test_budget_allocation_sums_to_the_two_second_target():
    planning = BUDGET['planning']
    assert planning['total_s'] == 2.0
    assert sum(s['target_s'] for s in planning['stages']) == pytest.approx(2.0)
    assert BUDGET['status'] == 'ALLOCATION_AND_REPORTING_ONLY_NOT_ENFORCED'


def test_nearest_rank_quantiles_never_interpolate():
    values = [1., 2., 3., 4., 10.]
    assert bench.quantile(values, .5) == 3. and bench.quantile(values, .95) == 10.
    assert bench.quantile([], .5) is None


def test_planning_summary_breakdown_success_and_over_budget(tmp_path):
    docs = [planning_doc(2.5, .5, .4), planning_doc(1.5, .3, .3), planning_doc(9.0, 4., .2, ok=False)]
    samples = []
    for i, doc in enumerate(docs):
        path = tmp_path/f'{i}'/'timing-prepare_plan.json'; path.parent.mkdir(); path.write_text(json.dumps(doc))
        samples += [s for kind, s in bench.samples_from_timing(doc, path) if kind == 'planning']
    out = bench.summarize('planning', samples, BUDGET['planning'], 1.2)
    assert out['samples'] == 3 and out['success_rate'] == pytest.approx(2/3)
    assert out['p50_s'] == 2.5 and out['max_s'] == 9.0 and out['meets_total_p95'] is False
    stages = {s['name']: s for s in out['stages']}
    # Sent checks inside the sequential span are not counted twice.
    assert stages['candidate_planning']['p50_s'] == pytest.approx(1.0)
    assert stages['sent_path_check']['max_s'] == pytest.approx(.4)
    assert stages['worker_start']['p50_s'] == pytest.approx(3.8)
    # Only complete-plan.json counts toward finalize; other writes do not.
    assert stages['finalize']['max_s'] == pytest.approx(.02)
    over = {o['name']: o for o in out['over_budget']}
    assert over['worker_start']['above_flag_factor'] and over['endpoint_screening']['above_flag_factor']
    assert 'sent_path_check' not in over


def test_no_input_reports_no_samples_and_invents_nothing(tmp_path):
    out = tmp_path/'bench'
    result = subprocess.run([sys.executable, str(ROOT/'tools/bench_planning_latency.py'), '--output', str(out)],
                            cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads((out/'bench.json').read_text())
    assert report['note'].startswith('NO_INPUT')
    for kind in ('planning', 'vision', 'scheduling'):
        assert report[kind]['status'] == 'NO_SAMPLES' and report[kind]['p95_s'] is None


def test_discover_lists_only_complete_saved_records(tmp_path):
    complete = tmp_path/'complete'; partial = tmp_path/'partial'
    for folder in (complete, partial):folder.mkdir()
    for name in bench.RECORD_FILES:(complete/name).write_text('x')
    (partial/'observation.json').write_text('x')
    assert bench.discover(tmp_path) == [complete] and bench.discover(tmp_path/'missing') == []


def test_fake_io_scheduling_sample_uses_first_send_after_execute_entry(tmp_path):
    doc = dict(phase='scheduling_fake_io', result='COMPLETE', entered_monotonic_s=100.,
               fake_io_first_sends=[dict(monotonic_s=103.5), dict(monotonic_s=110.)],
               events=[span('execute.binding', 1.8), span('execute.cycle_start', 1.7)])
    (kind, sample), = bench.samples_from_timing(doc, tmp_path/'timing-scheduling.json')
    assert kind == 'scheduling' and sample['seconds'] is None and sample['partial_seconds'] == pytest.approx(3.5) and sample['fake_io']
    out = bench.summarize('scheduling', [sample], BUDGET['scheduling'], 1.2)
    assert out['p50_s'] is None and out['status']=='PARTIAL_SCOPE' and out['over_budget'] == []
    assert out['coverage']['partial'] == 1 and out['coverage']['untimed'] == 1
