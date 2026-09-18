"""Synthetic CPU wiring only; no model inference or replacement of production fusion."""
from dataclasses import replace
import json
from pathlib import Path

import pytest
from unloading_contracts import canonical_fingerprint
from unloading_perception.algorithm_artifact import (algorithm_replay_record, load_algorithm_artifact,
    publish_algorithm_run, validate_algorithm_replay)
from unloading_perception.replay import replay_publication
from unloading_perception.scene import build_scene_update
from test_fusion_face_reduction import batch_fixture, module_observations


def artifact_fixture(root, *, mismatch=False, missing=False, empty=False):
    # Explicit synthetic measured patches, not evidence of real model quality.
    batches = batch_fixture(True)
    observations = module_observations(batches)
    results = {}
    root.mkdir(exist_ok=True)
    for batch, observation in zip(batches, observations):
        (root/batch.module_id).mkdir(exist_ok=True)
        observation = replace(observation, synthetic=False,
            coverage={**observation.coverage, 'proposal_source': 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL',
                      'raw_image_automatic': False, 'input_provenance': {'test': 'SYNTHETIC_CPU_FIXTURE'}})
        if mismatch and results:
            observation = replace(observation, source_sequence=observation.source_sequence+1)
        if empty:
            observation = replace(observation, cargo=(), unknown_regions=())
        results[batch.module_id] = {'observation': observation, 'observed_face_sets': () if empty else batch.face_sets}
    if missing: results.pop(batches[-1].module_id)
    def write_json(path, data):
        path.write_text(json.dumps(data), encoding='utf-8')
    return publish_algorithm_run(root, results, expected_modules=[b.module_id for b in batches],
        run_id='synthetic-test-run', models={'test': 'NO_REAL_INFERENCE'}, config={'test': True}, write_json=write_json)


def test_formal_artifact_replay_preserves_original_and_all_evidence(tmp_path):
    ref = artifact_fixture(tmp_path)
    original, index = load_algorithm_artifact(ref['path'], ref['sha256'])
    record = algorithm_replay_record(ref['path'], ref['sha256'], 'display-session')
    assert original.coverage['raw_image_automatic'] is False
    assert all(c.candidate_eligible is False for c in original.cargo)
    assert len(original.coverage['module_coverage']) == 2
    assert original.unknown_regions
    for seq in (0, 10):
        published = replay_publication(record, sequence=seq, elapsed=float(seq), published_time=100.+seq, clock_domain='ros')
        restored = validate_algorithm_replay(published)
        assert canonical_fingerprint(restored) == canonical_fingerprint(original)
        assert published.cargo == original.cargo and published.unknown_regions == original.unknown_regions
        assert published.capture_time == 0. and original.capture_time != published.capture_time
        assert published.source_epoch != original.source_epoch
        assert not build_scene_update(published).planning_admissible
        update = build_scene_update(published, replay_display_only=True)
        assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in update.blocking_reasons
        assert update.unknown_regions
    assert index['run_id'] == 'synthetic-test-run'
    changed = replace(record, cargo=())
    with pytest.raises(ValueError, match='ORIGINAL_CHANGED'):
        validate_algorithm_replay(changed)


@pytest.mark.parametrize('target', ['index', 'observation', 'module', 'fusion'])
def test_corrupted_artifact_never_becomes_replay(tmp_path, target):
    ref = artifact_fixture(tmp_path)
    index = json.loads(Path(ref['path']).read_text(encoding='utf-8'))
    selected = {'index': ref, 'observation': index['observation'],
                'module': next(iter(index['module_observations'].values())), 'fusion': index['fusion_result']}[target]
    Path(selected['path']).write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        algorithm_replay_record(ref['path'], ref['sha256'], 'session')


@pytest.mark.parametrize('option,error', [('mismatch','MIXED_CAPTURE_GROUP'), ('missing','MISSING_CURRENT_RUN_MODULE')])
def test_incompatible_or_missing_module_cannot_publish_index(tmp_path, option, error):
    with pytest.raises(ValueError, match=error): artifact_fixture(tmp_path, **{option: True})
    assert not (tmp_path/'algorithm_artifact.json').exists()


def test_empty_algorithm_is_unknown_not_free_space(tmp_path):
    ref = artifact_fixture(tmp_path, empty=True)
    original, _ = load_algorithm_artifact(ref['path'], ref['sha256'])
    assert not original.cargo and original.unknown_regions
    replay = algorithm_replay_record(ref['path'], ref['sha256'], 'session')
    assert not build_scene_update(replay, replay_display_only=True).planning_admissible


def test_once_new_output_and_partial_failure_do_not_reuse_previous_success(tmp_path, monkeypatch):
    from test_workcell_perception_once import prepare
    capture, modules, calls, main, argv = prepare(tmp_path, monkeypatch)
    first = tmp_path/'run-one'
    assert main(argv+['--output-directory', str(first)]) == 0
    ref = json.loads((first/'summary.json').read_text(encoding='utf-8'))['algorithm_artifact']
    load_algorithm_artifact(ref['path'], ref['sha256'])
    before = (first/'summary.json').read_bytes()
    assert main(argv+['--output-directory', str(first)]) == 1
    assert (first/'summary.json').read_bytes() == before
    # A separate test substitute fails the second side; no fallback to run-one.
    control = json.loads((capture/'test-control.json').read_text(encoding='utf-8'))
    control[modules[1]] = 'sam_error'
    (capture/'test-control.json').write_text(json.dumps(control), encoding='utf-8')
    from workcell_once_fakes import install
    install(monkeypatch, capture)
    second = tmp_path/'run-two'
    assert main(argv+['--output-directory', str(second)]) == 1
    assert not (second/'algorithm_artifact.json').exists()
    summary = json.loads((second/'summary.json').read_text(encoding='utf-8'))
    assert summary['handoff_not_run_reason'] == 'MISSING_CURRENT_RUN_MODULE'
