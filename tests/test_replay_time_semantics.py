"""CPU replay provenance and read-only domain snapshots; no ML/ROS dependency."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest
from unloading_contracts import RobotStateRevision, to_wire
from unloading_perception.backends import CargoJsonReplayBackend
from unloading_perception.demo import _synthetic_observation
from unloading_perception.replay import HISTORICAL_REPLAY, replay_publication, validate_replay_observation
from unloading_perception.scene import SnapshotAssembler, build_scene_update

FIXTURE = Path(__file__).parent / 'fixtures/vision_upstream/cargo7_minimal.json'


@pytest.mark.parametrize('known', [False, True])
def test_file_identity_and_original_time_survive_publications_and_restart(tmp_path, known):
    payload = json.loads(FIXTURE.read_text(encoding='utf-8'))
    if known:
        payload['source_record'] = dict(record_id='synthetic-history', capture_time=42.,
            clock_domain='historical-camera-clock', source_sequence=7)
    path = tmp_path / 'record.json'
    path.write_text(json.dumps(payload), encoding='utf-8')
    backend = CargoJsonReplayBackend()
    record = backend.read(path, replay_epoch='session-one')
    first = replay_publication(record, sequence=0, elapsed=.1, published_time=100., clock_domain='ros')
    later = replay_publication(record, sequence=8, elapsed=50., published_time=150., clock_domain='ros')
    restart = backend.read(path, replay_epoch='session-two')
    for value in (first, later, restart):
        meta = validate_replay_observation(value)
        assert meta['source_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
        assert meta['source_uri'] == path.as_uri()
        assert meta['original_capture_time'] == (42. if known else None)
        assert meta['original_clock_domain'] == ('historical-camera-clock' if known else None)
        assert meta['original_time_status'] == ('PROVIDED' if known else 'NOT_PROVIDED')
        assert value.capture_time == 0. and value.clock_domain == 'replay'
        assert value.observation_id == record.observation_id
        assert meta['record_id'] == record.coverage['replay']['record_id']
    assert first.source_epoch == later.source_epoch != restart.source_epoch
    assert first.source_sequence != later.source_sequence
    assert first.cargo == later.cargo and first.unknown_regions == later.unknown_regions
    assert first.processed_time == .1 and later.processed_time == 50.


def test_complete_metric_geometry_is_still_blocked_in_domain_model():
    record = CargoJsonReplayBackend().read(FIXTURE)
    complete = replace(record, cargo=_synthetic_observation().cargo, unknown_regions=())
    assert complete.cargo[0].candidate_eligible
    before = to_wire(complete)
    update = build_scene_update(complete, replay_display_only=True)
    assert update.blocking_reasons == (HISTORICAL_REPLAY,)
    assert not update.planning_admissible and to_wire(complete) == before
    assert not build_scene_update(complete).planning_admissible
    assembler = SnapshotAssembler()
    assembler.update = update
    assert assembler.assemble().snapshot is None  # never invent missing states
    assembler.robot_state = RobotStateRevision(1, (0.,)*6, {'joint_names': tuple(f'j{i}' for i in range(6))})
    assembler.tool_attachment = {'source': 'synthetic-test', 'verified': True}
    assembler.payload_attachment = {'object_id': None}
    assembler.base_state = {'source': 'synthetic-test'}
    assembler.conveyor_state = {'source': 'synthetic-test'}
    assembler.config_identity = {'robot_model_fingerprint': 'synthetic-robot', 'world_model_fingerprint': 'synthetic-world'}
    result = assembler.assemble()
    assert result.status == 'COMPLETE_BUT_NOT_PLANNABLE'
    assert not result.snapshot.scene_snapshot['planning_admissible']
    assert result.snapshot.scene_snapshot['blocking_reasons'] == (HISTORICAL_REPLAY,)
    unblocked_scene = {**result.snapshot.scene_snapshot, 'planning_admissible': True, 'blocking_reasons': ()}
    assert replace(result.snapshot, scene_snapshot=unblocked_scene).fingerprint != result.snapshot.fingerprint


@pytest.mark.parametrize('field,value', [('session_id', ''), ('source_sha256', 'bad'),
    ('record_id', ''), ('capture_time_semantics', 'sensor_time'), ('publication_sequence', 99),
    ('original_capture_time', 10.), ('original_time_status', 'PROVIDED')])
def test_inconsistent_replay_metadata_is_rejected(field, value):
    record = CargoJsonReplayBackend().read(FIXTURE)
    meta = dict(record.coverage['replay'])
    meta[field] = value
    broken = replace(record, coverage={**record.coverage, 'replay': meta})
    with pytest.raises(ValueError, match='REPLAY_'):
        build_scene_update(broken, replay_display_only=True)
    with pytest.raises(ValueError, match='REPLAY_METADATA_MISSING'):
        build_scene_update(replace(record, coverage={}), replay_display_only=True)


def test_bad_file_fails_without_fabricating_observation(tmp_path):
    backend = CargoJsonReplayBackend()
    with pytest.raises(FileNotFoundError): backend.read(tmp_path / 'absent.json')
    broken = tmp_path / 'broken.json'
    broken.write_text('{broken', encoding='utf-8')
    with pytest.raises(ValueError): backend.read(broken)
