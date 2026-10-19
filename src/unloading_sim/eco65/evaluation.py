"""Evaluation identity and finite-sampling evidence; no collision bypass."""
import math
import numpy as np

STRICT = 'strict'
CHAIN = 'chain_integration_v1'
PROFILES = (STRICT, CHAIN)
INTERPOLATION = 'COMMON_PROGRESS_STRAIGHT_JOINT_EDGES'


def evaluation_profile(value=STRICT):
    if value not in PROFILES:
        raise ValueError('UNKNOWN_EVALUATION_PROFILE')
    return value


def validate_output_evaluation(profile, request, result):
    evaluation_profile(profile)
    for item in (request, result):
        if evaluation_profile(item.get('evaluation_profile', STRICT)) != profile:
            raise ValueError('EVALUATION_PROFILE_MISMATCH')
    if profile == STRICT:
        if result.get('native_output_status') != 'PASS':
            raise ValueError('STRICT_NATIVE_OUTPUT_REQUIRED')
        return
    q = np.asarray([p['q'] for p in result['points']], float)
    resolution = request['clearance_policy']['edge_resolution_rad']
    if (not np.isfinite(resolution) or resolution <= 0 or q.ndim != 2 or
            len(q) < 2 or q.shape[1] != 6 or not np.isfinite(q).all()):
        raise ValueError('INVALID_BASIC_SAMPLED_PATH')
    count = 1+sum(max(1, math.ceil(float(np.max(np.abs(b-a)))/resolution)) for a,b in zip(q[:-1],q[1:]))
    evidence = result.get('basic_path_check', {})
    if (result.get('native_output_status') != 'BASIC_SAMPLED_PASS' or
            evidence.get('status') != 'BASIC_SAMPLED_PASS' or evidence.get('completed') is not True or
            evidence.get('failure') is not None or evidence.get('raw_nodes') != len(q) or
            evidence.get('sampling_rule') != 'MAX_ABS_JOINT_DELTA_CEIL_V1' or
            evidence.get('geometric_interpolation') != INTERPOLATION or
            evidence.get('resolution_rad') != resolution or
            any(evidence.get(k) != count for k in ('checked_states','clearance_checks','process_checks','ordinary_collision_checks')) or
            evidence.get('endpoint_process_checks') != 1 or evidence.get('actual_endpoint_status') != 'PASS' or
            not np.isfinite(evidence.get('wall_s', float('nan'))) or evidence['wall_s'] < 0 or
            result.get('start_state_status') != 'PASS' or
            result.get('joint_goal_state_status') != ('PASS' if 'q_goal' in request else 'NOT_APPLICABLE_POSE_GOAL') or
            result.get('dense_output_review') != 'NOT_RUN_PROFILE' or
            result.get('planner_internal_process_checks') is not True):
        raise ValueError('BASIC_SAMPLED_EVIDENCE_INCOMPLETE')
    for key in ('clearance', 'process_policy'):
        if result.get(key, {}).get('dense_path_calls') != 0:
            raise ValueError('CHAIN_DENSE_REVIEW_UNEXPECTED')
    internal = result['process_policy'].get('planner_state_checks')
    if type(internal) is not int or internal < 0:
        raise ValueError('PLANNER_STATE_CALL_COUNT_MISSING')
    if 'goal_pose' in request:
        fk = result.get('endpoint_fk', {})
        for key, bound in [('position_error_m', 1e-6), ('orientation_error_rad', 1e-6)]:
            if not np.isfinite(fk.get(key, float('nan'))) or not 0 <= fk[key] <= bound:
                raise ValueError('BASIC_ENDPOINT_FK_MISSING_OR_FAILED')


def replay_review_gate(plan, snapshot, review=None):
    if evaluation_profile(plan.get('evaluation_profile', STRICT)) == CHAIN:
        return  # validate_plan_contract verifies native basic evidence first.
    if (not review or review.get('status') != 'PASS' or
            review.get('plan_fingerprint') != plan['fingerprint'] or
            review.get('snapshot_fingerprint') != snapshot['fingerprint']):
        raise ValueError('OFFLINE_REPLAY_REQUIRES_SAME_REVIEWED_PLAN')


def validate_export_reference(plan, manifest):
    if evaluation_profile(plan.get('evaluation_profile', STRICT)) == CHAIN:
        source = manifest.get('home_source', {})
        if (source.get('fingerprint') != plan['fingerprint'] or
                source.get('evaluation_profile') != CHAIN):
            raise ValueError('WORKSTATION_PLAN_OR_EVALUATION_PROFILE_MISMATCH')
