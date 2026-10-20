"""Versioned stopped-axis cycle state and evidence checks; no motion generator."""
from copy import deepcopy
import numpy as np

from ..moveit2_backend import digest, validate_native_result, native_generation_evidence
from .evaluation import CHAIN, INTERPOLATION, validate_output_evaluation
from .model import robot_for
from .native_contact import check_contours
from .transfer_policy import endpoint_fk

MODE = 'eco65_coupled_cycle_v1'
J1_POLICY = 'RAW_J1_FIRST_CW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V2'
J1_REVERSE_LIMIT = 1e-6
INTERVALS = {'FIRST_APPROACH': -1, 'OUTBOUND': -1, 'RETURN': 1}
EVENTS = ('ATTACH', 'RELEASE', 'NOMINAL_RECEIVE', 'IDEAL_RECEIVER_DRAIN', 'NEXT_TARGET')


def reverse_travel(q, direction):
    values = np.asarray(q, float)
    if values.ndim != 2 or values.shape[1] != 6 or not np.isfinite(values).all():
        raise ValueError('CYCLE_INVALID_JOINT_PATH')
    return float(np.maximum(0., -direction*np.diff(values[:, 0])).sum())


def check_business_direction(q, interval, already_used=0.):
    if interval not in INTERVALS:
        raise ValueError('BUSINESS_INTERVAL_REQUIRED')
    used = already_used+reverse_travel(q, INTERVALS[interval])
    if not np.isfinite(used) or used > J1_REVERSE_LIMIT:
        raise ValueError('BUSINESS_J1_TOTAL_REVERSE_EXCEEDED')
    return used


def sign_state(state):
    state = deepcopy(state)
    state.pop('fingerprint', None)
    state['fingerprint'] = digest(state)
    return state


def check_state(state, original_ids):
    plain = dict(state)
    expected = plain.pop('fingerprint')
    if digest(plain) != expected:
        raise ValueError('CYCLE_STATE_FINGERPRINT_CHANGED')
    if state['business_interval'] not in INTERVALS:
        raise ValueError('BUSINESS_INTERVAL_REQUIRED')
    q = np.asarray(state['q'])
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('CYCLE_STATE_Q_INVALID')
    objects = state['world']
    names = [x['id'] for x in objects]
    if len(names) != len(set(names)):
        raise ValueError('CYCLE_WORLD_DUPLICATE')
    completed = set(state['completed_ids'])
    attached = state['attachment']
    attached_ids = {attached['id']} if attached else set()
    world_cartons = set(names) & set(original_ids)
    if (world_cartons & attached_ids or completed & (world_cartons | attached_ids)
            or world_cartons | attached_ids | completed != set(original_ids)):
        raise ValueError('CYCLE_CARTON_OWNERSHIP_INVALID')
    if set(state['remaining_ids']) != set(original_ids)-completed:
        raise ValueError('CYCLE_REMAINING_IDENTITIES_CHANGED')


def box_pose(state, target_id, robot):
    attached = state['attachment']
    if attached and attached['id'] == target_id:
        return robot.named_link_frames(state['q'])['link_6']@np.asarray(attached['pose'])
    return np.asarray(next(x['pose'] for x in state['world'] if x['id'] == target_id))


def receiving_geometry(pose, size, bounds, *, desired_drop=None, margin=.01):
    """Actual full projection and actual lower face, in D metres."""
    pose, size, bounds = np.asarray(pose), np.asarray(size), np.asarray(bounds)
    corners = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5)
                        for z in (-.5, .5)])*size
    corners = corners@pose[:3, :3].T+pose[:3, 3]
    low, high = corners.min(0), corners.max(0)
    gap = float(low[2]-bounds[1, 2])
    projection = bool(np.all(low[:2] >= bounds[0, :2]+margin-1e-6)
                      and np.all(high[:2] <= bounds[1, :2]-margin+1e-6))
    level = bool(np.max(np.abs(pose[:3, 2]-[0.,0.,1.])) <= 1e-5)
    return dict(passed=projection and level and gap >= -1e-6
                and (desired_drop is None or abs(gap-desired_drop) <= 1e-6),
                full_projection_inside=projection, level=level, actual_gap_m=gap,
                actual_box_bottom_m=float(low[2]), receiver_surface_m=float(bounds[1, 2]),
                margin_m=margin, desired_drop_m=desired_drop)


def frozen_receive_heights(box, low, high, desired_drop=.05):
    bottom = float(box['pose'][2]-box['size_m'][2]/2)
    values = [('HIGH', bottom-desired_drop),
              ('LOW', bottom-box['size_m'][2]-desired_drop)]
    return [dict(name=name, height_m=h, frozen_initial_bottom_m=bottom,
                 desired_drop_m=desired_drop) for name, h in values
            if low-1e-12 <= h <= high+1e-12]


def validate_cycle(plan, snapshot, *, require_complete=True):
    raw = dict(plan)
    signature = raw.pop('fingerprint')
    if digest(raw) != signature or plan['schema'] != MODE:
        raise ValueError('CYCLE_PLAN_IDENTITY_INVALID')
    if (plan['snapshot_fingerprint'] != snapshot['fingerprint']
            or plan['identity'] != snapshot['native_model']['init']['identity']
            or plan['j1_direction_policy'] != J1_POLICY
            or plan['evaluation_profile'] != CHAIN
            or plan['replay_interpolation'] != INTERPOLATION
            or plan['physical_execution'] != 'NOT_RUN' or plan['execution_ready'] is not False):
        raise ValueError('CYCLE_IDENTITY_OR_POLICY_CHANGED')
    ids = [x['id'] for x in snapshot['scene']['boxes']]
    sizes = {x['id']: x['size_m'] for x in snapshot['scene']['boxes']}
    robot = robot_for(snapshot['tool'], snapshot['scene'])
    from .coupled_scene import coupled_conveyor_state, with_coupled_state
    from .native_scene import world_objects
    mechanism_cache={}
    def check_mechanism(state):
        if (state['collision_policy'] != snapshot['native_model']['init']['collision_policy']
                or state['scene_fingerprint'] != snapshot['scene']['scene_fingerprint']):
            raise ValueError('CYCLE_STATE_COLLISION_POLICY_OR_SCENE_CHANGED')
        height=state['height_m']
        if height not in mechanism_cache:
            mechanism_cache[height]=(coupled_conveyor_state(
                snapshot['scene']['coupled_conveyor_spec'],height),
                world_objects(with_coupled_state(snapshot['scene'],height)))
        mechanism,world=mechanism_cache[height]
        if (state['mechanism']!=mechanism or
                [x for x in state['world'] if x['id'] not in sizes]!=world):
            raise ValueError('CYCLE_MECHANISM_NOT_UNIFIED_STATE')
    previous = plan['initial_state']
    check_state(previous, ids)
    check_mechanism(previous)
    parent = previous['parent_stage_id']
    seen = set()
    reversal = {name: 0. for name in INTERVALS}
    events = []
    for record in plan['records']:
        before, after = record['state_before'], record['state_after']
        check_state(before, ids)
        check_state(after, ids)
        check_mechanism(before)
        check_mechanism(after)
        if before != previous or record['parent_stage_id'] != parent:
            raise ValueError('CYCLE_RECORD_DISCONTINUITY')
        if record['record_id'] in seen or after['parent_stage_id'] != record['record_id']:
            raise ValueError('CYCLE_NATIVE_PARENT_CHANGED')
        seen.add(record['record_id'])
        req, result = record['native_request'], record['native_result']
        policy = snapshot['native_model']['init']['collision_policy']
        if (req['clearance_policy']['source_policy'] != policy
                or req['process_policy']['collision_policy'] != policy
                or result['clearance']['policy']['source_policy'] != policy):
            raise ValueError('CYCLE_NATIVE_COLLISION_POLICY_CHANGED')
        if (result['status'] != 'SUCCESS' or result['parent_stage_id'] != parent
                or result['stage_id'] != record['record_id']
                or req['world'] != before['world'] or req['attachment'] != before['attachment']
                or req['q_start'] != before['q'] or req['identity'] != plan['identity']):
            raise ValueError('CYCLE_NATIVE_RECEIPT_MISMATCH')
        if record['kind'] == 'robot_motion':
            q, times = np.asarray(record['q']), np.asarray(record['times'])
            if (q.shape[0] < 2 or not np.array_equal(q[0], before['q'])
                    or not np.array_equal(q[-1], after['q']) or times[0] != 0
                    or np.any(np.diff(times) <= 0) or not np.isfinite(times).all()
                    or not np.any(np.abs(np.diff(q, axis=0)) > 1e-12)):
                raise ValueError('CYCLE_NATIVE_MOTION_INVALID')
            interval = before['business_interval']
            direction = INTERVALS[interval]
            if req.get('business_interval') != interval or req.get('j1_direction') != direction:
                raise ValueError('CYCLE_DIRECTION_MISSING')
            reversal[interval] = check_business_direction(q, interval, reversal[interval])
            if (before['height_m'] != after['height_m']
                    or before['world'] != after['world']
                    or before['attachment'] != after['attachment']):
                raise ValueError('CYCLE_ROBOT_CHANGED_EXTERNAL_STATE')
            validate_output_evaluation(CHAIN, req, result)
            native = validate_native_result(result, q[0], req.get('q_goal'),
                                            snapshot['native_model']['init']['joint_names'])
            if not np.array_equal(native, q) or not native_generation_evidence(result, req):
                raise ValueError('CYCLE_NATIVE_PATH_CHANGED')
            if 'goal_pose' in req:
                error = endpoint_fk(robot, q[-1], req['goal_pose'])
                if error['position_error_m'] > 1e-6 or error['orientation_error_rad'] > 1e-6:
                    raise ValueError('CYCLE_ENDPOINT_FK_FAILED')
        else:
            if before['q'] != after['q'] or result.get('trajectory_present') is not False:
                raise ValueError('CYCLE_NONMOTION_CHANGED_ROBOT')
            if record['kind'] == 'external_axis':
                if before['attachment'] != after['attachment'] or before['active_ids'] != after['active_ids']:
                    raise ValueError('CYCLE_LIFT_ATTACHMENT_CHANGED')
                for sample in record['samples']:
                    check_mechanism(sample['state'])
                    if sample['state']['q'] != before['q']:
                        raise ValueError('CYCLE_LIFT_Q_CHANGED')
                if result.get('generation_source') != 'EXTERNAL_AXIS':
                    raise ValueError('CYCLE_EXTERNAL_AXIS_EVIDENCE_MISSING')
            elif record['kind'] == 'event':
                event = record['event_type']
                if event not in EVENTS or result.get('generation_source') != 'SEMANTIC_EVENT':
                    raise ValueError('CYCLE_EVENT_EVIDENCE_MISSING')
                target = before['target_id']
                events.append((event, target))
                if event in ('ATTACH', 'RELEASE'):
                    if not np.allclose(box_pose(before, target, robot), box_pose(after, target, robot),
                                       atol=1e-8, rtol=0):
                        raise ValueError('CYCLE_OWNERSHIP_TELEPORT')
                if event == 'ATTACH':
                    if before['attachment'] or not after['attachment']:
                        raise ValueError('CYCLE_ATTACH_OWNERSHIP_INVALID')
                    seal = check_contours(snapshot['native_model']['init']['process_geometry'],
                        robot.named_link_frames(after['q'])['link_6'], box_pose(after, target, robot),
                        sizes[target], gap=.0004, penetration=.0004,
                        angle=snapshot['tool']['contact']['normal_tolerance_rad'],
                        face=after['target_face'], active_ids=after['active_ids'])
                    if seal['status'] != 'PASS':
                        raise ValueError('CYCLE_ACTUAL_CONTACT_FAILED')
                if event == 'NOMINAL_RECEIVE':
                    a, b = box_pose(before, target, robot), box_pose(after, target, robot)
                    if (not np.allclose(a[:3, :3], b[:3, :3], atol=1e-12, rtol=0)
                            or not np.allclose(a[:2, 3], b[:2, 3], atol=1e-12, rtol=0)
                            or b[2, 3] >= a[2, 3] or before['attachment'] or after['attachment']):
                        raise ValueError('CYCLE_NOMINAL_DROP_INVALID')
            else:
                raise ValueError('CYCLE_RECORD_KIND_INVALID')
        previous, parent = after, record['record_id']
    if previous != plan['terminal_state']:
        raise ValueError('CYCLE_TERMINAL_STATE_CHANGED')
    if require_complete:
        first, second = plan['first_target_id'], plan['next_target_id']
        if (not plan['complete'] or first == second
                or events != [('ATTACH', first), ('RELEASE', first), ('NOMINAL_RECEIVE', first),
                              ('IDEAL_RECEIVER_DRAIN', first), ('NEXT_TARGET', first), ('ATTACH', second)]
                or previous['completed_ids'] != [first]
                or previous['target_id'] != second or previous['attachment']['id'] != second):
            raise ValueError('CYCLE_INCOMPLETE')
    return dict(status='PASS',complete=plan['complete'],j1_reverse_used_rad=reversal,
                world_cartons=len(set(x['id'] for x in previous['world']) & set(ids)),
                attached_cartons=int(previous['attachment'] is not None),
                completed_cartons=len(previous['completed_ids']))
