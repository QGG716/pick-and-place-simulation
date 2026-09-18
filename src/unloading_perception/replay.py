"""Single-file replay provenance; never converts history to online sensor time."""
from dataclasses import replace
import math
from typing import Mapping
from urllib.parse import urlparse

HISTORICAL_REPLAY = 'HISTORICAL_REPLAY_DISPLAY_ONLY'
REPLAY_TIME = 'replay_session_static_zero'


def source_metadata(payload, uri, digest, session, sequence, elapsed):
    original = payload.get('source_record', payload)
    if not isinstance(original, Mapping):
        raise ValueError('replay source_record must be an object')
    capture = original.get('capture_time')
    domain = original.get('clock_domain')
    return dict(schema_version='single_file_replay_v1', source_uri=uri, source_sha256=digest,
        session_id=session, record_id=original.get('record_id') or original.get('observation_id') or 'sha256:' + digest,
        original_capture_time=capture, original_clock_domain=domain,
        original_source_sequence=original.get('source_sequence'),
        original_time_status='NOT_PROVIDED' if capture is None else 'PROVIDED',
        capture_time_semantics=REPLAY_TIME, session_elapsed_seconds=elapsed,
        publication_sequence=sequence, published_time=None, publication_clock_domain=None)


def validate_replay_observation(observation, *, require_publication=False):
    meta = observation.coverage.get('replay')
    if not isinstance(meta, Mapping):
        raise ValueError('REPLAY_METADATA_MISSING')
    required = ('schema_version', 'source_uri', 'source_sha256', 'session_id', 'record_id',
        'original_capture_time', 'original_clock_domain', 'original_source_sequence', 'original_time_status',
        'capture_time_semantics', 'session_elapsed_seconds', 'publication_sequence',
        'published_time', 'publication_clock_domain')
    if any(k not in meta for k in required):
        raise ValueError('REPLAY_METADATA_INCOMPLETE')
    digest = meta['source_sha256']
    if (not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest)
            or not isinstance(meta['source_uri'], str) or urlparse(meta['source_uri']).scheme != 'file'
            or not urlparse(meta['source_uri']).path
            or not isinstance(meta['record_id'], str) or not meta['record_id']):
        raise ValueError('REPLAY_SOURCE_INVALID')
    if (meta['schema_version'] != 'single_file_replay_v1' or observation.clock_domain != 'replay'
            or meta['session_id'] != observation.source_epoch
            or observation.observation_id != 'replay-record-' + digest
            or meta['capture_time_semantics'] != REPLAY_TIME or observation.capture_time != 0.
            or type(meta['publication_sequence']) is not int
            or meta['publication_sequence'] != observation.source_sequence
            or isinstance(meta['session_elapsed_seconds'], bool)
            or meta['session_elapsed_seconds'] != observation.processed_time):
        raise ValueError('REPLAY_ENVELOPE_MISMATCH')
    capture, domain = meta['original_capture_time'], meta['original_clock_domain']
    if capture is None:
        if meta['original_time_status'] != 'NOT_PROVIDED':
            raise ValueError('REPLAY_ORIGINAL_TIME_INVALID')
    elif (not isinstance(capture, (int, float)) or isinstance(capture, bool) or not math.isfinite(capture)
          or not isinstance(domain, str) or not domain or meta['original_time_status'] != 'PROVIDED'):
        raise ValueError('REPLAY_ORIGINAL_TIME_INVALID')
    if domain is not None and (not isinstance(domain, str) or not domain):
        raise ValueError('REPLAY_ORIGINAL_CLOCK_INVALID')
    sequence = meta['original_source_sequence']
    if sequence is not None and (type(sequence) is not int or sequence < 0):
        raise ValueError('REPLAY_ORIGINAL_SEQUENCE_INVALID')
    published = meta['published_time']
    if published is None:
        if require_publication or meta['publication_clock_domain'] is not None:
            raise ValueError('REPLAY_PUBLICATION_MISSING')
    elif (not isinstance(published, (int, float)) or isinstance(published, bool)
          or not math.isfinite(published) or published < 0.
          or meta['publication_clock_domain'] not in ('ros', 'ros_sim_time')):
        raise ValueError('REPLAY_PUBLICATION_INVALID')
    return meta


def replay_publication(observation, *, sequence, elapsed, published_time, clock_domain):
    meta = dict(validate_replay_observation(observation))
    meta.update(publication_sequence=sequence, session_elapsed_seconds=elapsed,
                published_time=published_time, publication_clock_domain=clock_domain)
    result = replace(observation, source_sequence=sequence, processed_time=elapsed,
        coverage={**observation.coverage, 'replay': meta, 'source_restart': sequence == 0})
    validate_replay_observation(result, require_publication=True)
    return result
