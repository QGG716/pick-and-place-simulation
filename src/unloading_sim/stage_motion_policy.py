"""Motion intent and generation choices; validity belongs to MotionValidator.

These names do not rename exported stages or grant contact permissions. The
connector supplies the current contract and its existing bounded RRT callback.
"""
from enum import Enum
from dataclasses import dataclass
from time import perf_counter

import numpy as np

from .motion_validation import Status


class MotionPurpose(str, Enum):
    FREE_APPROACH = 'FREE_APPROACH'
    CONTACT_PROCESS = 'CONTACT_PROCESS'
    SUPPORT_RELEASE_PROCESS = 'SUPPORT_RELEASE_PROCESS'
    EXTRACTION_PROCESS = 'EXTRACTION_PROCESS'
    FREE_LOADED_TRANSFER = 'FREE_LOADED_TRANSFER'
    PLACEMENT_PROCESS = 'PLACEMENT_PROCESS'
    DEPARTURE_PROCESS = 'DEPARTURE_PROCESS'


class GenerationMethod(str, Enum):
    JOINT_DIRECT = 'JOINT_DIRECT'
    VERIFIED_TEMPLATE = 'VERIFIED_TEMPLATE'
    HISTORY_HINT = 'HISTORY_HINT'
    LOCAL_CARTESIAN_CANDIDATE = 'LOCAL_CARTESIAN_CANDIDATE'
    PROCESS_WAYPOINT_CANDIDATE = 'PROCESS_WAYPOINT_CANDIDATE'
    RRT_CONNECT = 'RRT_CONNECT'


FREE_PURPOSES = frozenset((MotionPurpose.FREE_APPROACH, MotionPurpose.FREE_LOADED_TRANSFER))


@dataclass(frozen=True)
class VerifiedTemplate:
    """Request-local receipt. Its validator owns strong input references/lease."""
    path: tuple
    validator: object

    @classmethod
    def capture(cls, path, validator, budget):
        checked = validator.check_path(path, budget)
        if not checked.valid:
            return None
        # Immutable tuples, independent of mutable caller arrays.
        return cls(tuple(tuple(float(v) for v in q) for q in path), validator)

    def candidate(self, current_validator):
        current = (current_validator is self.validator and
                   self.validator.context_current() == self.validator.context.context_id)
        method = GenerationMethod.VERIFIED_TEMPLATE if current else GenerationMethod.HISTORY_HINT
        return method, lambda: (self.path, None, dict(
            source='REQUEST_LOCAL_VALIDATED_PATH', evidence_reusable=current,
            validation_context=self.validator.context.context_id))


def require_purpose(purpose, *, free=False):
    purpose = MotionPurpose(purpose)  # Unknown/absent intent is never generic search.
    if free and purpose not in FREE_PURPOSES:
        raise ValueError(f'{purpose.value} forbids unconstrained free connection')
    return purpose


def interrupts_generation(failure):
    if not failure:
        return False
    status = (failure.get('validation') or {}).get('status', failure.get('status'))
    return status in ('INDETERMINATE', 'CANCELLED') or any(word in str(failure.get('reason', ''))
        for word in ('CONTEXT_CHANGED', 'CANCEL', 'DEADLINE', 'UNKNOWN', 'VALIDATION_WORK_BUDGET'))


def free_connection_prefix(start, goal, validator, budget, *, purpose, candidates=()):
    """Check endpoints/direct/candidates in order, without constructing a tree.

    Candidate producers are lazy and finite at the caller. Every hint includes
    checked endpoint bridges under this exact validator; a name is never proof.
    Only a decisive geometric rejection permits the caller to continue to RRT.
    """
    purpose = require_purpose(purpose, free=True)
    started = perf_counter()
    before = dict(validator.statistics)
    trace = dict(purpose=purpose.value, attempts=[], selected_method=None,
                 validation_context=validator.context.context_id,
                 guarantee=validator.context.guarantee, validation_completed=False,
                 rrt_constructed=False, rrt_called=False, rrt_expanded=False,
                 extension_attempts=0, planning_iterations_consumed=0)

    def finish(path, result, continuation=False):
        trace.update(validation_completed=bool(result.valid),
                     validation_level='B_STRICT_LOCAL_CONNECTION' if result.valid else 'A_UNVERIFIED_GEOMETRY',
                     validation_status=result.status.value,
                     failure=result.failure, continue_to_candidates_or_rrt=continuation,
                     elapsed_seconds=perf_counter()-started,
                     state_samples=validator.statistics['state_samples']-before['state_samples'],
                     state_cache_reuses=validator.statistics['state_cache_reuses']-before['state_cache_reuses'])
        return path, result, trace

    for name, q in (('start', start), ('goal', goal)):
        result = validator.check_states([q], budget)[0]
        trace['attempts'].append(dict(method='ENDPOINT_CHECK', endpoint=name,
                                      status=result.status.value, failure=result.failure))
        if not result.valid:
            trace['invalid_endpoint'] = name
            return finish([], result)
    result = validator.check_motion(start, goal, budget)
    trace['attempts'].append(dict(method=GenerationMethod.JOINT_DIRECT.value,
                                  status=result.status.value, failure=result.failure,
                                  cache_hit=result.cache_hit))
    if result.valid:
        trace['selected_method'] = GenerationMethod.JOINT_DIRECT.value
        return finish([np.asarray(start).copy(), np.asarray(goal).copy()], result)
    if result.status != Status.INVALID:
        return finish([], result)
    direct = result
    available = False
    for method, producer in candidates:
        method = GenerationMethod(method)
        if method not in {GenerationMethod.VERIFIED_TEMPLATE, GenerationMethod.HISTORY_HINT,
                          GenerationMethod.LOCAL_CARTESIAN_CANDIDATE,
                          GenerationMethod.PROCESS_WAYPOINT_CANDIDATE}:
            raise ValueError('not a bounded free-connection candidate')
        # Catch cancellation/context changes before generating another candidate.
        guard = validator.check_states([start], budget)[0]
        if not guard.valid:
            return finish([], guard)
        available = True
        points, failure, evidence = producer()
        attempt = dict(method=method.value, generation=evidence, failure=failure)
        trace['attempts'].append(attempt)
        if failure is not None:
            status = (failure.get('validation') or {}).get('status')
            reason = str(failure.get('reason', ''))
            if interrupts_generation(failure):
                result = validator._result(Status.CANCELLED if 'CANCEL' in reason or status == 'CANCELLED'
                                           else Status.INDETERMINATE, failure)
                return finish([], result)
            attempt['status'] = 'GENERATION_REJECTED'
            continue
        if not points:
            attempt['status'] = 'UNAVAILABLE'
            continue
        # Even equal endpoints are harmless cache hits; unequal endpoints must
        # never be silently spliced into a validated path.
        path = [np.asarray(start).copy(), *[np.asarray(q).copy() for q in points], np.asarray(goal).copy()]
        result = validator.check_path(path, budget)
        attempt.update(status=result.status.value, failure=result.failure,
                       endpoint_bridges_checked=True)
        if result.valid:
            trace['selected_method'] = method.value
            return finish(path, result)
        if result.status != Status.INVALID:
            return finish([], result)
    if not available:
        trace['attempts'].append(dict(method='TEMPLATE_OR_LOCAL_CANDIDATE', status='UNAVAILABLE'))
    trace['rrt_reason'] = 'VALID_ENDPOINTS_DIRECT_AND_AVAILABLE_CANDIDATES_REJECTED'
    return finish([], direct, continuation=True)


def motion_subsegments(segment, trace):
    """Add intent boundaries without changing any replay stage or event index."""
    ranges = segment['stage_ranges']
    approach = segment.get('approach', {})
    gate = approach.get('free_connection_end_index')
    if gate is None:
        if 'pregrasp' not in ranges:
            raise ValueError('free/contact motion-purpose boundary is unavailable')
        gate = ranges['pregrasp'][1]
    if not 0 <= gate <= segment['grasp_index']:
        raise ValueError('invalid free/contact motion-purpose boundary')
    stages = trace['stages']
    selected_approach = (approach.get('attempts') or [{}])[-1].get('search', approach)
    records = [
        (MotionPurpose.FREE_APPROACH, 'pregrasp' if 'pregrasp' in ranges else 'contact',
         [0, gate], selected_approach.get('connection', {})),
        (MotionPurpose.CONTACT_PROCESS, 'contact', [gate, segment['grasp_index']],
         selected_approach.get('terminal', {})),
        (MotionPurpose.SUPPORT_RELEASE_PROCESS, 'support-release',
         ranges.get('support-release', [segment['grasp_index']]*2), stages.get('support-release', {})),
        (MotionPurpose.EXTRACTION_PROCESS, 'extraction', ranges['extraction'], stages.get('extraction', {})),
        (MotionPurpose.FREE_LOADED_TRANSFER, 'transit', ranges['transit'], stages.get('transit', {})),
        (MotionPurpose.PLACEMENT_PROCESS, 'place', ranges['place'], stages.get('place', {})),
        (MotionPurpose.DEPARTURE_PROCESS, 'withdrawal', ranges['withdrawal'], stages.get('withdrawal', {})),
    ]
    result=[]
    for purpose, stage, interval, evidence in records:
        result.append(dict(purpose=purpose.value, exported_stage=stage if stage in ranges else None,
            validation_stage='pregrasp' if purpose == MotionPurpose.FREE_APPROACH else stage,
            path_range=list(interval),
            loaded=purpose in {MotionPurpose.SUPPORT_RELEASE_PROCESS, MotionPurpose.EXTRACTION_PROCESS,
                              MotionPurpose.FREE_LOADED_TRANSFER, MotionPurpose.PLACEMENT_PROCESS},
            target=segment['target'], receiver=segment['place']['effective_receiver'],
            generation=generation_summary(evidence), required_segment_checks_completed=True,
            execution_ready=False, guarantee='DISCRETE_WITH_EXISTING_PAIR_PROOFS'))
    return result


def generation_summary(evidence):
    """Reuse counters/context IDs; do not duplicate state samples or hash scenes."""
    fields = ('selected_method', 'endpoint_ik_streams', 'endpoint_ik_seed_attempts',
        'endpoint_ik_calls', 'cartesian_samples', 'along_path_ik_calls', 'rrt_constructed',
        'rrt_called', 'rrt_expanded', 'rrt_reason', 'extension_attempts',
        'planning_iterations_consumed', 'expensive_states', 'state_cache_reuses',
        'validation_context', 'validation_contexts', 'guarantee', 'validation_completed', 'elapsed_seconds')
    summary={key:evidence[key] for key in fields if key in evidence}
    if 'source' in evidence:summary['source']=evidence['source']
    summary['attempts']=[]
    for attempt in evidence.get('attempts', []):
        item={key:attempt[key] for key in ('candidate_index', 'pass', 'method', 'status', 'route') if key in attempt}
        item['failure_reason']=(attempt.get('failure') or {}).get('reason')
        if 'connection' in attempt:item['connection']=generation_summary(attempt['connection'])
        summary['attempts'].append(item)
    if 'search' in evidence:summary['process']=generation_summary(evidence['search'])
    if 'parts' in evidence:summary['parts']=[generation_summary(part) for part in evidence['parts']]
    if 'selected_attempt' in evidence:
        selected=next((a for a in evidence.get('attempts', []) if a.get('index')==evidence['selected_attempt']), {})
        summary['process']=generation_summary(selected.get('search', {}))
    if 'selected_direction_world' in evidence:
        selected=next((a for a in evidence.get('attempts', []) if a.get('failure') is None and
            a.get('direction_world')==evidence['selected_direction_world']), {})
        summary['process']=generation_summary(selected.get('search', {}))
    return summary


def validate_motion_subsegments(segment):
    records = segment.get('motion_subsegments')
    if records is None:
        return  # Historical schema retains its explicit legacy boundaries.
    expected = list(MotionPurpose)
    if len(records) != len(expected):
        raise ValueError('motion-purpose subsegments must cover seven internal uses')
    previous = 0
    ranges = segment['stage_ranges']
    gate = segment.get('approach', {}).get('free_connection_end_index',
                                          ranges.get('pregrasp', [None, None])[1])
    expected_ranges = [[0, gate], [gate, segment['grasp_index']],
        ranges.get('support-release', [segment['grasp_index']]*2), ranges['extraction'],
        ranges['transit'], ranges['place'], ranges['withdrawal']]
    for item, purpose, expected_range in zip(records, expected, expected_ranges):
        if item['purpose'] != purpose.value:
            raise ValueError('motion-purpose ordering changed')
        first, last = item['path_range']
        if [first, last] != list(expected_range):
            raise ValueError('motion-purpose range disagrees with exported process boundary')
        if (type(first) is not int or type(last) is not int or first != previous
                or not first <= last < len(segment['path'])):
            raise ValueError('motion-purpose ranges must be contiguous')
        previous = last
    if (records[1]['path_range'][1] != segment['grasp_index']
            or records[5]['path_range'][1] != segment['release_index']
            or previous != segment['release_retreat_index']):
        raise ValueError('motion-purpose ranges disagree with attachment/release events')
