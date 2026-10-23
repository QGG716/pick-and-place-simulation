"""Versioned stopped-axis cycle state and evidence checks; no motion generator."""
from copy import deepcopy
from numbers import Real, Integral
import numpy as np

from ..moveit2_backend import digest, validate_native_result, native_generation_evidence
from .evaluation import CHAIN, INTERPOLATION, validate_output_evaluation
from .model import robot_for
from .native_contact import check_contours
from .transfer_policy import endpoint_fk

MODE = 'eco65_coupled_cycle_v1'
J1_POLICY = 'RAW_J1_FIRST_CW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V2'
J1_CAPTURE_POLICY = 'RAW_J1_FIRST_CCW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V3'
J1_FREE_POLICY = 'RAW_FINITE_JOINTS_BIDIRECTIONAL_NO_J1_MONOTONICITY_V4'
J1_REVERSE_LIMIT = 1e-6
INTERVALS = {'FIRST_APPROACH': -1, 'OUTBOUND': -1, 'RETURN': 1}
EVENTS = ('ATTACH', 'RELEASE', 'NOMINAL_RECEIVE', 'IDEAL_RECEIVER_DRAIN', 'NEXT_TARGET')


def reverse_travel(q, direction):
    values = np.asarray(q, float)
    if values.ndim != 2 or values.shape[1] != 6 or not np.isfinite(values).all():
        raise ValueError('CYCLE_INVALID_JOINT_PATH')
    return float(np.maximum(0., -direction*np.diff(values[:, 0])).sum())


def business_direction(interval, policy=J1_POLICY):
    if policy not in (J1_POLICY,J1_CAPTURE_POLICY,J1_FREE_POLICY):raise ValueError('BUSINESS_DIRECTION_POLICY_REQUIRED')
    if interval not in INTERVALS:
        raise ValueError('BUSINESS_INTERVAL_REQUIRED')
    if policy==J1_FREE_POLICY:return 0
    return 1 if interval=='FIRST_APPROACH' and policy==J1_CAPTURE_POLICY else INTERVALS[interval]


def check_business_direction(q, interval, already_used=0., policy=J1_POLICY):
    if policy==J1_FREE_POLICY:
        # Validate raw finite samples; zero means no directional constraint.
        return reverse_travel(q,business_direction(interval,policy))
    used = already_used+reverse_travel(q, business_direction(interval,policy))
    if not np.isfinite(used) or used > J1_REVERSE_LIMIT:
        raise ValueError('BUSINESS_J1_TOTAL_REVERSE_EXCEEDED')
    return used


def sign_state(state):
    state = deepcopy(state)
    state.pop('fingerprint', None)
    state['fingerprint'] = digest(state)
    return state


def same_generated_geometry(a,b):
    """Permit only sub-picometre/radian platform roundoff in generated floats.

    IDs, integer topology and inventory stay exact. Stored record fingerprints
    are checked independently; this never rewrites recorded geometry or q.
    """
    if isinstance(a,dict) and isinstance(b,dict):
        return a.keys()==b.keys() and all(same_generated_geometry(a[k],b[k]) for k in a)
    if isinstance(a,list) and isinstance(b,list):
        return len(a)==len(b) and all(same_generated_geometry(x,y) for x,y in zip(a,b))
    if isinstance(a,Real) and isinstance(b,Real) and not isinstance(a,(bool,np.bool_)) and not isinstance(b,(bool,np.bool_)):
        if isinstance(a,Integral) and isinstance(b,Integral):return a==b
        return bool(np.isfinite([a,b]).all() and abs(a-b)<=1e-12)
    if type(a) is not type(b):return False
    return a==b


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


def frozen_receive_heights(box, low, high, desired_drop=.05, *, hardware_fallback=False):
    bottom = float(box['pose'][2]-box['size_m'][2]/2)
    values = [('HIGH', bottom-desired_drop),
              ('LOW', bottom-box['size_m'][2]-desired_drop)]
    candidates = [dict(name=name, height_m=h, frozen_initial_bottom_m=bottom,
                 desired_drop_m=desired_drop) for name, h in values
            if low-1e-12 <= h <= high+1e-12]
    if not candidates and hardware_fallback:
        candidates.append(dict(name='HARDWARE_LOW_WITH_NATIVE_PAYLOAD_HEIGHT_ADJUSTMENT',
            height_m=low, frozen_initial_bottom_m=bottom, desired_drop_m=desired_drop,
            required_box_bottom_at_release_m=low+desired_drop))
    return candidates


def validate_cycle(plan, snapshot, *, require_complete=True):
    raw = dict(plan)
    signature = raw.pop('fingerprint')
    if digest(raw) != signature or plan['schema'] != MODE:
        raise ValueError('CYCLE_PLAN_IDENTITY_INVALID')
    if (plan['snapshot_fingerprint'] != snapshot['fingerprint']
            or plan['identity'] != snapshot['native_model']['init']['identity']
            or plan['j1_direction_policy'] not in (J1_POLICY,J1_CAPTURE_POLICY,J1_FREE_POLICY)
            or plan['evaluation_profile'] != CHAIN
            or plan['replay_interpolation'] != INTERPOLATION
            or plan['physical_execution'] != 'NOT_RUN' or plan['execution_ready'] is not False):
        raise ValueError('CYCLE_IDENTITY_OR_POLICY_CHANGED')
    ids = [x['id'] for x in snapshot['scene']['boxes']]
    batch = plan.get('completion_goal') == 'ALL_CARTONS_DRAINED'
    if plan.get('completion_goal') not in (None, 'ALL_CARTONS_DRAINED'):
        raise ValueError('CYCLE_COMPLETION_GOAL_INVALID')
    if batch and plan.get('inventory_ids') != ids:
        raise ValueError('BATCH_INVENTORY_CHANGED')
    sizes = {x['id']: x['size_m'] for x in snapshot['scene']['boxes']}
    robot = robot_for(snapshot['tool'], snapshot['scene'])
    free=plan['j1_direction_policy']==J1_FREE_POLICY
    if free:
        from .transverse_process import POLICY, corners, pose_of
        if (plan.get('transverse_process_policy')!=POLICY or
                snapshot['scene'].get('transverse_process_policy')!=POLICY):
            raise ValueError('CYCLE_TRANSVERSE_POLICY_IDENTITY_CHANGED')
    from .coupled_scene import coupled_conveyor_state, native_coupled_world
    mechanism_cache={}
    def check_mechanism(state):
        if (state['collision_policy'] != snapshot['native_model']['init']['collision_policy']
                or state['scene_fingerprint'] != snapshot['scene']['scene_fingerprint']):
            raise ValueError('CYCLE_STATE_COLLISION_POLICY_OR_SCENE_CHANGED')
        height=state['height_m']
        if height not in mechanism_cache:
            mechanism_cache[height]=(coupled_conveyor_state(
                snapshot['scene']['coupled_conveyor_spec'],height),
                native_coupled_world(snapshot['scene'],height))
        mechanism,world=mechanism_cache[height]
        recorded=dict(state['mechanism']);recorded_hash=recorded.pop('scene_fingerprint')
        expected={k:v for k,v in mechanism.items() if k!='scene_fingerprint'}
        if (digest(recorded)!=recorded_hash or not same_generated_geometry(recorded,expected) or
                not same_generated_geometry([x for x in state['world'] if x['id'] not in sizes],world)):
            raise ValueError('CYCLE_MECHANISM_NOT_UNIFIED_STATE')
    previous = plan['initial_state']
    check_state(previous, ids)
    check_mechanism(previous)
    parent = previous['parent_stage_id']
    seen = set()
    reversal = {name: 0. for name in INTERVALS}
    legs = [dict(interval=previous['business_interval'],target_id=previous['target_id'],reverse_rad=0.)]
    events = []
    for record in plan['records']:
        before, after = record['state_before'], record['state_after']
        if free:
            option=before.get('process_option')
            if not option or record['native_request'].get('process_option')!=option:
                raise ValueError('CYCLE_PROCESS_OPTION_MISSING')
            if before['attachment'] and abs(before['height_m']-option['height_m'])>1e-8:
                raise ValueError('EXTRACTION_BEFORE_REQUIRED_RECEIVE_HEIGHT')
            if record['kind']=='external_axis' and before['attachment']:
                raise ValueError('RECEIVER_SYNC_AFTER_ATTACHMENT_DISABLED')
            if record.get('event_type')=='ATTACH' and abs(before['height_m']-option['height_m'])>1e-8:
                raise ValueError('ATTACH_BEFORE_REQUIRED_RECEIVE_HEIGHT')
            if record.get('event_type')=='RELEASE':
                actual=box_pose(before,before['target_id'],robot)
                original=next(b for b in snapshot['scene']['boxes'] if b['id']==before['target_id'])
                region=receiving_geometry(actual,original['size_m'],before['mechanism']['receiving']['bounds_D_m'],
                    desired_drop=option['expected_drop_m'],margin=before['mechanism']['receiving']['margin_m'])
                if (not region['passed'] or
                        abs(region['actual_box_bottom_m']-option['release_bottom_world_m'])>1e-6 or
                        np.max(np.abs(actual[:3,:3]-pose_of(original)[:3,:3]))>1e-6):
                    raise ValueError('CYCLE_TEMPLATE_RELEASE_CHANGED')
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
        if batch and req.get('completion_goal') != 'ALL_CARTONS_DRAINED':
            raise ValueError('BATCH_NATIVE_GOAL_MISSING')
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
            direction = business_direction(interval,plan['j1_direction_policy'])
            if (req.get('business_interval') != interval or req.get('j1_direction') != direction or
                    req.get('j1_direction_policy')!=plan['j1_direction_policy']):
                raise ValueError('CYCLE_DIRECTION_MISSING')
            if batch:
                if abs(req['j1_reverse_used_rad']-legs[-1]['reverse_rad'])>1e-12:
                    raise ValueError('BATCH_DIRECTION_BUDGET_DISCONTINUITY')
                legs[-1]['reverse_rad']=check_business_direction(q,interval,legs[-1]['reverse_rad'],plan['j1_direction_policy'])
                reversal[interval]+=reverse_travel(q,direction)
            else:
                reversal[interval] = check_business_direction(q, interval, reversal[interval],plan['j1_direction_policy'])
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
        if batch:
            expected_interval=('OUTBOUND' if record.get('event_type')=='ATTACH' else
                               'RETURN' if record.get('event_type')=='RELEASE' else before['business_interval'])
            if after['business_interval']!=expected_interval:
                raise ValueError('BATCH_BUSINESS_INTERVAL_CHANGED')
            if after['business_interval']!=before['business_interval']:
                legs.append(dict(interval=after['business_interval'],target_id=after['target_id'],reverse_rad=0.))
        previous, parent = after, record['record_id']
    if previous != plan['terminal_state']:
        raise ValueError('CYCLE_TERMINAL_STATE_CHANGED')
    if require_complete and batch:
        completed=previous['completed_ids']
        expected=[]
        for i,target in enumerate(completed):
            if i:expected.append(('NEXT_TARGET',completed[i-1]))
            expected.extend((event,target) for event in ('ATTACH','RELEASE','NOMINAL_RECEIVE','IDEAL_RECEIVER_DRAIN'))
        if (not plan['complete'] or len(completed)!=len(ids) or set(completed)!=set(ids)
                or previous['attachment'] is not None or previous['remaining_ids']
                or events!=expected or completed[0]!=plan['first_target_id']):
            raise ValueError('BATCH_INCOMPLETE')
    elif require_complete:
        first, second = plan['first_target_id'], plan['next_target_id']
        if (not plan['complete'] or first == second
                or events != [('ATTACH', first), ('RELEASE', first), ('NOMINAL_RECEIVE', first),
                              ('IDEAL_RECEIVER_DRAIN', first), ('NEXT_TARGET', first), ('ATTACH', second)]
                or previous['completed_ids'] != [first]
                or previous['target_id'] != second or previous['attachment']['id'] != second):
            raise ValueError('CYCLE_INCOMPLETE')
    return dict(status='PASS',complete=plan['complete'],j1_reverse_used_rad=reversal,
                j1_business_legs=legs if batch else None,
                world_cartons=len(set(x['id'] for x in previous['world']) & set(ids)),
                attached_cartons=int(previous['attachment'] is not None),
                completed_cartons=len(previous['completed_ids']))
