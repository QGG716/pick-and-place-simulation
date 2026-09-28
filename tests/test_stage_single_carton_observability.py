"""Harness durability tests, not claimed as planning or physical evidence."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('single_carton_probe', ROOT/'tools/validate_stage_motion_single_carton.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


@pytest.mark.parametrize('stop', ['cancel', 'resource', 'normal'])
def test_entry_identity_and_progress_survive_interruption(tmp_path, monkeypatch, stop):
    import sys
    monkeypatch.syspath_prepend(str(ROOT))
    from tools import run_m710_contact_unloading as entry
    history = tmp_path/'history'
    (history/'planning').mkdir(parents=True); (history/'inputs').mkdir()
    (history/'planning/motion.json').write_text(json.dumps(dict(selected_trajectory_segment=dict(target='box'))))
    (history/'inputs/actual_remaining_state.json').write_text('{}')
    output = tmp_path/'result.json'
    monkeypatch.setattr(probe, 'identity', lambda root: dict(source='fixed-test-source'))
    def normal(argv, *, source_guard):
        assert json.loads(output.read_text())['status'] == 'RUNNING'
        assert '--reuse-motion' not in argv and '--history-source' not in argv
        source_guard()
        folder = Path(argv[argv.index('--output')+1]); folder.mkdir()
        (folder/'planning_progress.jsonl').write_text(json.dumps(dict(event='stage_enter', entry='_cartesian'))+'\n')
        if stop == 'cancel': raise KeyboardInterrupt('operator')
        if stop == 'resource': raise probe.OfflineResourceLimit('test external cap')
        return 2
    monkeypatch.setattr(entry, 'main', normal)
    assert probe.main(SimpleNamespace(history=history, output=output, offline_resource_seconds=None)) == 2
    result = json.loads(output.read_text())
    assert result['source_unchanged_at_exit'] and result['inputs_unchanged_at_exit']
    assert result['last_known_stage']['entry'] == '_cartesian'
    assert result['location_is_last_observed_only']
    assert result['status'] == dict(cancel='MANUAL_CANCELLED', resource='OFFLINE_RESOURCE_LIMIT', normal='NORMAL_EXIT')[stop]
    assert not result['complete_planning'] and not result['independent_preflight_run']
