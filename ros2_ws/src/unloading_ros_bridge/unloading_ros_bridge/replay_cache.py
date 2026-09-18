"""Reuse verified static replay content, never source sampling or admission decisions."""
from copy import copy, deepcopy
from dataclasses import replace
import json

from unloading_contracts import to_wire
from unloading_perception.replay import validate_replay_observation
from .common import float_to_time
from .mapping import observation_source_times

PUBLICATION = ('publication_sequence', 'session_elapsed_seconds', 'published_time', 'publication_clock_domain')
TRANSPORT_FIELDS = {'source_sequence', 'processed_time', 'exact_processed_time_s', 'coverage_json'}


def replay_content(message):
    """Compare actual full ROS values, including every cargo JSON byte, not declared IDs."""
    coverage = json.loads(message.coverage_json)
    if not isinstance(coverage, dict) or not isinstance(coverage.get('replay'), dict):
        raise ValueError('invalid replay coverage')
    replay = coverage['replay']
    static = {**coverage, 'replay': {k:v for k,v in replay.items() if k not in PUBLICATION}}
    static.pop('source_restart', None)
    return (tuple((name, getattr(message,name)) for name in message.get_fields_and_field_types()
                  if name not in TRANSPORT_FIELDS), static), coverage


class ReplayRecordCache:
    def __init__(self):
        self.content = self.observation = None

    def matching_publication(self, message):
        content, coverage = replay_content(message)
        if self.content != content:
            return None
        capture, processed = observation_source_times(message)
        publication = replace(self.observation, source_sequence=int(message.source_sequence),
            capture_time=capture, processed_time=processed, coverage=coverage)
        validate_replay_observation(publication, require_publication=True)
        return publication

    def remember(self, message, observation):
        self.content = deepcopy(replay_content(message)[0])
        self.observation = observation


class HeartbeatTemplate:
    def __init__(self, snapshot, message, update):
        self.update = update
        self.snapshot, self.message = snapshot, deepcopy(message)
        # Keep every domain field; only replace the small validated robot revision.
        wire = json.loads(message.domain_payload_json)
        payload = wire['payload']
        payload.pop('robot_state_revision')
        encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(',', ':'))
        self.prefix = '{"schema_version":'+json.dumps(wire['schema_version'])+',"payload":'+encoded[:-1]+',"robot_state_revision":'

    def sample(self, robot_state, *, now, sequence, robot_sample, mechanism_sample, mechanism_sequence):
        snapshot = self.snapshot.with_robot_sampling(robot_state)
        result = copy(self.message)
        result.publisher_sequence = sequence
        result.publisher_restart = sequence == 0
        result.published_time = float_to_time(now)
        result.robot_sample_time = float_to_time(robot_sample)
        result.mechanism_sample_time = float_to_time(mechanism_sample)
        result.mechanism_revision_sequence = mechanism_sequence
        result.domain_payload_json = self.prefix+json.dumps(to_wire(robot_state), ensure_ascii=False,
                                                          sort_keys=True, separators=(',', ':'))+'}}'
        return snapshot, result
