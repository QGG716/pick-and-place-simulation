"""Real one-shot orchestration with explicitly fake CPU inference boundaries."""
import json
from pathlib import Path
import runpy
import subprocess
import sys

import pytest

from workcell_once_fakes import SCRIPT, capture_fixture, install


def test_partial_technical_failure_returns_nonzero(tmp_path, monkeypatch):
    capture, models, modules = capture_fixture(tmp_path)
    calls = install(monkeypatch, capture)
    subject = runpy.run_path(str(SCRIPT), run_name="workcell_once_subject")
    monkeypatch.setattr(sys, "argv", [str(SCRIPT), "--capture", str(capture), "--vision", str(tmp_path), "--models", str(models)])
    code = subject["main"]()
    summary = json.loads((capture / "perception-once/summary.json").read_text(encoding="utf-8"))
    statuses = [row["status"] for row in summary["runs"]]
    print("actual_exit_code=", code, "module_statuses=", statuses)
    assert statuses == ["TECHNICAL_FAILURE", "COMPLETED_WITH_ALGORITHM_RESULTS"]
    assert calls.count(("sam", modules[0])) == calls.count(("sam", modules[1])) == 1
    assert code == 1


def prepare(tmp_path, monkeypatch, scenarios=('normal', 'normal')):
    capture, models, modules = capture_fixture(tmp_path, scenarios)
    calls = install(monkeypatch, capture)
    subject = runpy.run_path(str(SCRIPT), run_name='workcell_once_subject')
    argv = ['--capture', str(capture), '--vision', str(tmp_path), '--models', str(models)]
    return capture, modules, calls, subject['main'], argv


def report(capture):
    return json.loads((capture / 'perception-once/summary.json').read_text(encoding='utf-8'))


def assert_counts(summary, *, completed, failed, not_run=0):
    assert summary['module_counts'] == {
        'expected': 2, 'completed': completed, 'failed': failed, 'not_run': not_run, 'running': 0,
    }
    assert summary['expected_module_count'] == 2
    assert len(summary['runs']) == 2
    for row in summary['runs']:
        assert row['planning_admissible'] is row['raw_image_automatic'] is False
        assert row['proposal_source'] == 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL'
        assert 0 <= row['sam_attempts'] <= 1 and 0 <= row['metric_attempts'] <= 1


@pytest.mark.parametrize('scenario,metric_calls', [('normal', 1), ('empty', 0)])
def test_zero_complete_cuboids_and_empty_segmentation_are_success(tmp_path, monkeypatch, scenario, metric_calls):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, (scenario, scenario))
    assert main(argv) == 0
    summary = report(capture)
    assert summary['overall_status'] == 'COMPLETED' and summary['exit_code'] == 0
    assert summary['finalized'] and not summary['errors']
    assert summary['model_manifest']['sam']['snapshot_path'] == 'CPU_TEST_SUBSTITUTE'
    assert_counts(summary, completed=2, failed=0)
    for row in summary['runs']:
        assert row['complete_cuboids_accepted'] == row['observed_faces'] == 0
        assert row['sam_attempts'] == 1 and row['metric_attempts'] == metric_calls
        assert calls.count(('sam', row['module'])) == 1
        assert calls.count(('metric', row['module'])) == metric_calls
        assert row['mask_evaluation']['sam_count'] == metric_calls
        assert row['instances_without_accepted_face'] == metric_calls
    progress = [json.loads(line) for line in (capture/'test-progress.jsonl').read_text(encoding='utf-8').splitlines()]
    initial = progress[0]
    assert initial['call'] == 'initialize'
    assert initial['summary']['module_counts']['not_run'] == 2
    assert initial['summary']['exit_code'] == 1 and not initial['summary']['finalized']
    for entry in progress:
        if entry['call'] == 'sam':
            assert any(row['status'] == 'RUNNING' and row['stage'] == 'sam_inference' for row in entry['summary']['runs'])


@pytest.mark.parametrize('scenarios', [('sam_error', 'normal'), ('normal', 'sam_error'), ('sam_error', 'sam_error')])
def test_failure_is_not_reset_by_later_success(tmp_path, monkeypatch, scenarios, capsys):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, scenarios)
    assert main(argv) == 1
    summary = report(capture)
    assert summary['exit_code'] == 1 and summary['overall_status'] == 'TECHNICAL_FAILURE'
    failures = scenarios.count('sam_error')
    assert_counts(summary, completed=2-failures, failed=failures)
    for module, scenario, row in zip(modules, scenarios, summary['runs']):
        assert calls.count(('sam', module)) == 1
        if scenario == 'sam_error':
            assert row['failure_stage'] == 'sam_inference' and row['error_type'] == 'RuntimeError'
            assert row['metric_attempts'] == 0 and ('metric', module) not in calls
            assert 'CPU_TEST_SAM_FAILURE' in (capture/'perception-once'/module/'failure.txt').read_text(encoding='utf-8')
        else:
            assert row['status'] == 'COMPLETED_WITH_ALGORITHM_RESULTS' and row['metric_attempts'] == 1
    assert 'CPU_TEST_SAM_FAILURE' in capsys.readouterr().err


def test_shared_initialization_failure_keeps_full_not_run_roster(tmp_path, monkeypatch):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, ('init_error', 'normal'))
    assert main(argv) == 1
    summary = report(capture)
    assert_counts(summary, completed=0, failed=0, not_run=2)
    assert summary['errors'][0]['stage'] == 'runtime_initialize'
    assert summary['errors'][0]['error_type'] == 'RuntimeError'
    assert calls == [('initialize', 'shared')]
    assert all(row['sam_attempts'] == row['metric_attempts'] == 0 for row in summary['runs'])
    assert all(row['not_run_reason'] == 'RUN_FAILURE:runtime_initialize' for row in summary['runs'])


@pytest.mark.parametrize('scenario,stage,error_type,metric_attempts', [
    ('sam_timeout', 'sam_inference', 'TimeoutExpired', 0),
    ('sam_structured', 'sam_inference', 'RuntimeError', 0),
    ('sam_missing_response', 'worker_artifacts', 'FileNotFoundError', 0),
    ('response_failed', 'worker_artifacts', 'ValueError', 0),
    ('old_request', 'worker_artifacts', 'ValueError', 0),
    ('old_artifacts', 'worker_artifacts', 'ValueError', 0),
    ('corrupt_masks', 'worker_artifacts', 'ValueError', 0),
    ('image_write_failure', 'mask_evaluation', 'OSError', 0),
    ('metric_error', 'metric_geometry', 'RuntimeError', 1),
    ('metric_timeout', 'metric_geometry', 'TimeoutExpired', 1),
    ('metric_structured', 'metric_geometry', 'RuntimeError', 1),
    ('observation_failed', 'geometry_result', 'RuntimeError', 1),
    ('geometry_invalid', 'geometry_result', 'JSONDecodeError', 1),
    ('geometry_structure', 'geometry_result', 'ValueError', 1),
    ('geometry_missing', 'geometry_result', 'ValueError', 1),
    ('missing_face_sets', 'geometry_result', 'KeyError', 1),
])
def test_technical_failure_boundaries(tmp_path, monkeypatch, scenario, stage, error_type, metric_attempts):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, (scenario, 'normal'))
    (capture/'old-metrics.json').write_text('{}', encoding='utf-8')
    assert main(argv) == 1
    summary = report(capture)
    assert_counts(summary, completed=1, failed=1)
    row = summary['runs'][0]
    assert row['failure_stage'] == stage and row['error_type'] == error_type
    assert row['sam_attempts'] == 1 and row['metric_attempts'] == metric_attempts
    assert calls.count(('sam', modules[0])) == 1
    assert calls.count(('metric', modules[0])) == metric_attempts


@pytest.mark.parametrize('missing,stage', [('sensor_rgb.png', 'rgb_hash'), ('metric_depth_m.npy', 'prepare_inputs')])
def test_required_input_failure_is_a_reported_module(tmp_path, monkeypatch, missing, stage):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    (capture/'FULL_STACK_NOMINAL/modules'/modules[0]/missing).unlink()
    assert main(argv) == 1
    summary = report(capture)
    assert_counts(summary, completed=1, failed=1)
    row = summary['runs'][0]
    assert row['failure_stage'] == stage and row['error_type'] == 'FileNotFoundError'
    assert row['sam_attempts'] == row['metric_attempts'] == 0
    assert ('sam', modules[0]) not in calls


@pytest.mark.parametrize('stage', ['output', 'module'])
def test_directory_creation_failure(tmp_path, monkeypatch, stage, capsys):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    original = Path.mkdir
    target = capture/'perception-once'
    if stage == 'module':
        target /= modules[0]

    def deny(path, *args, **kwargs):
        if path == target:
            raise PermissionError('CPU_TEST_DIRECTORY_DENIED')
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'mkdir', deny)
    assert main(argv) == 1
    assert 'CPU_TEST_DIRECTORY_DENIED' in capsys.readouterr().err
    if stage == 'output':
        assert not target.exists() and not calls
    else:
        summary = report(capture)
        assert_counts(summary, completed=1, failed=1)
        assert summary['runs'][0]['failure_stage'] == 'module_directory'
        assert summary['runs'][0]['sam_attempts'] == 0


@pytest.mark.parametrize('filename', ['manifest.json', 'models.json'])
def test_shared_required_json_failure_is_reported(tmp_path, monkeypatch, filename):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    path = capture/'manifest.json' if filename == 'manifest.json' else tmp_path/'models.json'
    path.write_text('{invalid', encoding='utf-8')
    assert main(argv) == 1 and not calls
    summary = report(capture)
    assert summary['errors'][0]['error_type'] == 'JSONDecodeError'
    if filename == 'manifest.json':
        assert summary['expected_module_count'] is None and summary['runs'] == []
    else:
        assert_counts(summary, completed=0, failed=0, not_run=2)


def test_valid_empty_manifest_is_not_success(tmp_path, monkeypatch):
    from unloading_perception.isaac_validation import IsaacSceneManifest, canonical_digest
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    path = capture/'manifest.json'
    payload = json.loads(path.read_text(encoding='utf-8'))
    payload['cameras'] = []
    payload['dynamic_scene_fingerprint'] = canonical_digest({
        'objects': payload['objects'], 'mechanisms': payload['mechanisms'], 'camera_calibration': (),
    })
    payload['world_fingerprint'] = canonical_digest({
        'layout_fingerprint': payload['layout']['layout_fingerprint'],
        'dynamic_scene_fingerprint': payload['dynamic_scene_fingerprint'], 'robot': payload['robot'],
    })
    payload['manifest_fingerprint'] = canonical_digest({k: v for k, v in payload.items() if k != 'manifest_fingerprint'})
    IsaacSceneManifest.from_dict(payload)  # A valid contract, but no one-shot work.
    path.write_text(json.dumps(payload), encoding='utf-8')
    assert main(argv) == 1 and not calls
    summary = report(capture)
    assert summary['expected_module_count'] == 0 and summary['exit_code'] == 1
    assert summary['overall_status'] == 'TECHNICAL_FAILURE'


@pytest.mark.parametrize('scenario', ['sam_error', 'metric_error', 'geometry_missing'])
def test_stale_capture_outputs_are_never_consumed_or_modified(tmp_path, monkeypatch, scenario):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, (scenario, 'normal'))
    source = capture/'FULL_STACK_NOMINAL/modules'/modules[0]
    for name in ('rgbd_cuboids.json', 'mode_b1_worker_response.json', 'oracle_proposals.json'):
        (source/name).write_text('{"status":"COMPLETE","instances":[]}', encoding='utf-8')
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    assert main(argv) == 1
    assert {p.name: p.read_bytes() for p in source.iterdir()} == before
    assert report(capture)['runs'][0]['status'] == 'TECHNICAL_FAILURE'


def test_existing_output_is_refused_without_touching_history(tmp_path, monkeypatch):
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    output = capture/'perception-once'
    output.mkdir()
    (output/'summary.json').write_text('{"historical":true}', encoding='utf-8')
    (output/'marker').write_bytes(b'keep')
    before = {p.name: p.read_bytes() for p in output.iterdir()}
    assert main(argv) == 1 and not calls
    assert {p.name: p.read_bytes() for p in output.iterdir()} == before


@pytest.mark.parametrize('persistent,original_failure', [(False, False), (True, False), (True, True)])
def test_final_summary_write_failure_is_nonzero_and_preserves_original_error(tmp_path, monkeypatch, capsys, persistent, original_failure):
    import os
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, ('sam_error' if original_failure else 'normal', 'normal'))
    replace = os.replace
    failures = []

    def fail_final(source, destination):
        if Path(destination).name == 'summary.json':
            pending = json.loads(Path(source).read_text(encoding='utf-8'))
            if pending['finalized'] and (persistent or not failures):
                # A failed atomic replace leaves only conservative progress.
                assert report(capture)['exit_code'] == 1
                failures.append(pending)
                raise PermissionError('CPU_TEST_FINAL_SUMMARY_FAILURE')
        return replace(source, destination)

    monkeypatch.setattr(os, 'replace', fail_final)
    assert main(argv) == 1
    assert len(failures) == (2 if persistent else 1)
    summary = report(capture)
    assert summary['exit_code'] == 1
    assert not list((capture/'perception-once').glob('*.tmp'))
    if not persistent:
        assert summary['finalized'] and summary['errors'][-1]['stage'] == 'summary_write'
    else:
        assert not summary['finalized']
    errors = capsys.readouterr().err
    assert 'CPU_TEST_FINAL_SUMMARY_FAILURE' in errors
    if original_failure:
        assert 'CPU_TEST_SAM_FAILURE' in errors


def test_progress_write_failure_stops_unstarted_inference(tmp_path, monkeypatch):
    import os
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    replace = os.replace
    failed = False

    def fail_progress(source, destination):
        nonlocal failed
        if Path(destination).name == 'summary.json':
            pending = json.loads(Path(source).read_text(encoding='utf-8'))
            if (not failed and not pending['finalized'] and pending['runs']
                    and pending['runs'][0]['status'] == 'COMPLETED_WITH_ALGORITHM_RESULTS'):
                failed = True
                raise OSError('CPU_TEST_PROGRESS_FAILURE')
        return replace(source, destination)

    monkeypatch.setattr(os, 'replace', fail_progress)
    assert main(argv) == 1
    summary = report(capture)
    assert_counts(summary, completed=1, failed=0, not_run=1)
    assert ('sam', modules[1]) not in calls
    assert summary['runs'][1]['sam_attempts'] == 0


def test_initial_summary_failure_does_not_start_runtime(tmp_path, monkeypatch, capsys):
    import os
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)

    def deny(source, destination):
        raise PermissionError('CPU_TEST_REPORT_STORAGE_UNAVAILABLE')

    monkeypatch.setattr(os, 'replace', deny)
    assert main(argv) == 1 and not calls
    assert not (capture/'perception-once/summary.json').exists()
    assert 'CPU_TEST_REPORT_STORAGE_UNAVAILABLE' in capsys.readouterr().err


def test_run_identity_initialization_failure_still_has_report(tmp_path, monkeypatch):
    import uuid
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)

    def fail():
        raise OSError('CPU_TEST_IDENTITY_INITIALIZATION_FAILURE')

    monkeypatch.setattr(uuid, 'uuid4', fail)
    assert main(argv) == 1 and not calls
    summary = report(capture)
    assert summary['errors'][0]['stage'] == 'run_identity'
    assert summary['exit_code'] == 1 and summary['finalized']


@pytest.mark.parametrize('scenario,exception,code', [('interrupt', KeyboardInterrupt, 130), ('system_exit', SystemExit, 7), ('system_exit_zero', SystemExit, 1)])
@pytest.mark.parametrize('index', [0, 1])
def test_interrupt_preserves_completed_and_not_run_records(tmp_path, monkeypatch, scenario, exception, code, index):
    scenarios = ['normal', 'normal']
    scenarios[index] = scenario
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch, scenarios)
    with pytest.raises(exception) as caught:
        main(argv)
    if exception is SystemExit:
        assert caught.value.code == code
    summary = report(capture)
    assert summary['exit_code'] == code and summary['overall_status'] == 'INTERRUPTED'
    assert_counts(summary, completed=index, failed=1, not_run=1-index)
    assert summary['runs'][index]['error_type'] == exception.__name__
    if index == 0:
        assert ('sam', modules[1]) not in calls
        assert summary['runs'][1]['not_run_reason'] == 'INTERRUPTED'


@pytest.mark.parametrize('scenarios,code', [(('normal', 'normal'), 0), (('sam_error', 'normal'), 1),
                                         (('normal', 'interrupt'), 130), (('system_exit_zero', 'normal'), 1)])
def test_real_cli_process_exit_and_summary(tmp_path, scenarios, code):
    capture, models, modules = capture_fixture(tmp_path, scenarios)
    launcher = Path(__file__).with_name('workcell_once_cli.py')
    process = subprocess.run([sys.executable, str(launcher), '--capture', str(capture),
                              '--vision', str(tmp_path), '--models', str(models)],
                             capture_output=True, text=True, encoding='utf-8', timeout=20)
    assert process.returncode == code, process.stderr
    summary = report(capture)
    assert summary['exit_code'] == process.returncode and summary['finalized']
    assert (summary['overall_status'] == 'COMPLETED') == (code == 0)
    if code == 0:
        assert process.stderr == ''
        assert_counts(summary, completed=2, failed=0)
    else:
        assert process.stderr
        assert summary['module_counts']['failed'] == 1
    calls = [json.loads(line) for line in (capture/'test-calls.jsonl').read_text(encoding='utf-8').splitlines()]
    for module in modules:
        assert calls.count(['sam', module]) <= 1 and calls.count(['metric', module]) <= 1


def test_real_cli_argument_errors_remain_argparse_nonzero():
    process = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True, encoding='utf-8', timeout=20)
    assert process.returncode == 2
    assert 'required' in process.stderr
