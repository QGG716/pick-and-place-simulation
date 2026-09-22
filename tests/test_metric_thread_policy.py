"""Scoped native thread control; CPU substitutes are explicitly synthetic."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest
from tools import metric_thread_policy as policy


def pools(count=64):
    return [dict(path='/test/'+name+'.libs/libopenblas.so', owner=name, num_threads=count,
                 getter='openblas_get_num_threads', setter='openblas_set_num_threads') for name in ('numpy', 'scipy')]


def state(count=64):
    return dict(pid=os.getpid(), blas=pools(count), opencv_threads=22, thread_environment={}, affinity=[0, 1])


def test_inherit_never_invokes_setter_or_changes_environment(monkeypatch):
    original = dict(os.environ)
    monkeypatch.setattr(policy.ctypes, 'CDLL', lambda p: pytest.fail('inherit must not set threads'))
    assert policy.configure_blas(pools(), None) == []
    assert dict(os.environ) == original


def test_native_getter_checks_all_loaded_target_pools(monkeypatch):
    values = {'numpy': 64, 'scipy': 64, 'opencv': 1}
    calls = []
    class Native:
        def __init__(self, name, setter=False): self.name, self.setter = name, setter
        def __call__(self, *args):
            if self.setter: values[self.name] = args[0]; calls.append(self.name)
            else: return values[self.name]
    libraries = {name: SimpleNamespace(openblas_get_num_threads=Native(name), openblas_set_num_threads=Native(name, True)) for name in values}
    monkeypatch.setattr(policy, 'read_optional', lambda p: {'value': '\n'.join('0 r /test/'+n+'.libs/libopenblas.so' for n in values)})
    monkeypatch.setattr(policy.ctypes, 'CDLL', lambda p: libraries[p.split('/')[2].split('.')[0]])
    before = policy.discover_blas()
    policy.configure_blas(before, 1)
    after = policy.discover_blas()
    policy.verify_blas(after, 1)
    assert calls == ['numpy', 'scipy'] and values == {'numpy': 1, 'scipy': 1, 'opencv': 1}


@pytest.mark.parametrize('bad', [[], pools()[:1], [{**p, 'num_threads': None} for p in pools()], pools(2)])
def test_missing_or_ineffective_native_settings_fail(bad):
    with pytest.raises(ValueError, match='THREAD_POLICY_NOT_VERIFIED'):
        policy.verify_blas(bad, 1)


@pytest.mark.parametrize('value', ['0', '-1', '1.5', 'bad'])
def test_invalid_cli_values_fail_before_numeric_initialization(tmp_path, value):
    result = subprocess.run([sys.executable, 'tools/run_metric_geometry_blas.py', '--plan', 'unused',
        '--output', str(tmp_path/'output'), '--blas-threads', value], capture_output=True, text=True)
    assert result.returncode != 0 and 'positive integer' in result.stderr
    assert not (tmp_path/'output').exists() and 'ModuleNotFoundError' not in result.stderr


def test_explicit_unverified_policy_stops_before_geometry(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools'))
    import run_metric_geometry_blas as entry
    plan = tmp_path/'plan.json'; plan.write_text('{}')
    monkeypatch.setattr(entry, 'snapshot', state)  # synthetic getter stays 64 despite request
    monkeypatch.setattr(entry, 'configure_blas', lambda *a: [])
    assert entry.main(['--plan', str(plan), '--output', str(tmp_path/'out'), '--blas-threads', '1']) == 2
    report = json.loads((tmp_path/'out/thread_policy_report.json').read_text())
    assert report['status'] == 'THREAD_POLICY_NOT_VERIFIED' and not (tmp_path/'out/ab_summary.json').exists()


def test_unknown_cgroup_permissions_are_not_infinite(monkeypatch):
    monkeypatch.setattr(policy, 'read_optional', lambda p: {'value': None, 'error': 'Permission denied'})
    value = policy.cgroup_snapshot()
    assert not value['levels'] and value['mapping_error']


def test_non_target_settings_cannot_change():
    before = state(); after = deepcopy(before); after['opencv_threads'] = 1
    with pytest.raises(ValueError, match='NON_TARGET'):
        policy.verify_non_targets(before, after)


@pytest.mark.parametrize('threads', [None, 1])
def test_real_native_libraries_in_child_entry_preserve_parent(tmp_path, threads):
    pytest.importorskip('scipy.linalg'); pytest.importorskip('cv2')
    parent = policy.snapshot()
    plan = tmp_path/'plan.json'; plan.write_text('{}')
    output = tmp_path/'child'
    # Only geometry is replaced here. Entry, numerical library loading, native
    # getters/setters, argument parsing and real child process remain production.
    code = '''
import json,sys,types
from pathlib import Path
sys.path[:0]=['tools','src','packages/unloading_contracts/src']
import run_metric_geometry_blas as entry
stub=types.ModuleType('profile_metric_faces')
def run(argv, *, runtime_policy):
    out=Path(argv[argv.index('--output')+1]); out.mkdir()
    (out/'delegated_policy.json').write_text(json.dumps(runtime_policy))
    (out/'ab_summary.json').write_text(json.dumps({'artifact':{'test':'SYNTHETIC_GEOMETRY_SUBSTITUTE'}}))
    return 0
stub.main=run;sys.modules['profile_metric_faces']=stub
raise SystemExit(entry.main(sys.argv[1:]))
'''
    command = [sys.executable, '-c', code, '--plan', str(plan), '--output', str(output)]
    if threads is not None: command += ['--blas-threads', str(threads)]
    completed = subprocess.run(command, text=True, capture_output=True)
    assert completed.returncode == 0, completed.stdout+completed.stderr
    report = json.loads((output/'thread_policy_report.json').read_text())
    assert report['pid'] != os.getpid() and report['status'] == 'COMPLETED'
    assert json.loads((output/'delegated_policy.json').read_text()) == policy.policy(threads)
    policy.verify_blas(report['before_geometry']['blas'], threads)
    policy.verify_blas(report['after_geometry']['blas'], threads)
    current = policy.snapshot()
    assert parent['blas'] == current['blas'] and parent['thread_environment'] == current['thread_environment']
