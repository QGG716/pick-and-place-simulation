"""Sample the saved coupled plan, without changing any planning/safety gate.

The belt height is a state variable: the committed root supplies its first
value, and retained native external-axis checks supply each later value.
No time interpolation, waypoint reduction or default belt height is used.
"""
from __future__ import annotations

from copy import deepcopy
import numpy as np


def _height(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise ValueError('COMPACTNESS_RECORDED_CONVEYOR_HEIGHT_MISSING_OR_INVALID')
    if not .55-1e-9 <= value <= 1.5+1e-9:
        raise ValueError('COMPACTNESS_RECORDED_CONVEYOR_HEIGHT_OUTSIDE_FROZEN_RANGE')
    return float(value)


def _q(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('COMPACTNESS_RECORDED_JOINT_VECTOR_MISSING_OR_INVALID')
    return q.tolist()


def coupled_plan_samples(plan):
    """Keep every saved robot waypoint and every saved external-axis check."""
    root = plan.get('batch_root') or {}
    height = _height(root.get('height_m'))
    q = _q(root.get('q_rad'))
    height_source = 'batch_root.height_m'
    samples = []
    seen = set()
    stages = plan.get('stages') or []
    if not stages:
        raise ValueError('COMPACTNESS_REQUIRES_RECORDED_PLAN_STAGES')
    for stage in stages:
        sid, phase = stage.get('stage_id'), stage.get('stage')
        if not sid or sid in seen or not phase:
            raise ValueError('COMPACTNESS_INVALID_RECORDED_STAGE_ID_OR_PHASE')
        seen.add(sid)
        points = stage.get('points') or []
        source = stage.get('generation_source')
        if source not in ('NATIVE_GENERATED', 'NATIVE_EXTERNAL_AXIS', 'SEMANTIC_EVENT'):
            raise ValueError('COMPACTNESS_UNSUPPORTED_STAGE_GENERATION_SOURCE')
        def add(joints, h, label, origin, **extra):
            samples.append(dict(pose_id=f'{sid}:{label}', phase=phase, native_stage_id=sid,
                q_rad=_q(joints), conveyor_height_m=h, height_source=origin,
                generation_source=source, **extra))
        previous_time = -1.
        for index, point in enumerate(points):
            joints = _q(point.get('q'))
            if index == 0 and not np.allclose(joints, q, atol=1e-10, rtol=0):
                raise ValueError('COMPACTNESS_ROBOT_STAGE_DISCONTINUITY')
            t = float(point.get('t', float('nan')))
            if not np.isfinite(t) or t < 0 or t < previous_time:
                raise ValueError('COMPACTNESS_INVALID_RECORDED_WAYPOINT_TIME')
            previous_time = t
            if source != 'NATIVE_GENERATED' and not np.allclose(joints, q, atol=1e-10, rtol=0):
                raise ValueError('COMPACTNESS_NON_NATIVE_MOVING_STAGE')
            add(joints, height, f'waypoint-{index}', height_source,
                source_kind='ORIGINAL_ROBOT_WAYPOINT', waypoint_index=index, stage_time_s=t)
            if source == 'NATIVE_GENERATED':
                q = joints
        if source == 'NATIVE_EXTERNAL_AXIS':
            axis = stage.get('external_axis') or {}
            before, after = _height(axis.get('from_height_m')), _height(axis.get('to_height_m'))
            if abs(before-height) > 1e-10 or axis.get('robot_q_constant') is not True:
                raise ValueError('COMPACTNESS_EXTERNAL_AXIS_STATE_MISMATCH')
            axis_q = _q(axis.get('robot_q'))
            if not np.allclose(axis_q, q, atol=1e-10, rtol=0):
                raise ValueError('COMPACTNESS_EXTERNAL_AXIS_ROBOT_Q_CHANGED')
            checks = axis.get('checks') or []
            heights = [_height(check.get('height_m')) for check in checks]
            if (not checks or abs(heights[0]-before) > 1e-10 or abs(heights[-1]-after) > 1e-10
                    or any(check.get('passed') is not True for check in checks)):
                raise ValueError('COMPACTNESS_EXTERNAL_AXIS_CHECKS_INCOMPLETE_OR_FAILED')
            for index, h in enumerate(heights):
                add(q, h, f'axis-check-{index}', f'{sid}.external_axis.checks[{index}].height_m',
                    source_kind='ORIGINAL_EXTERNAL_AXIS_CHECK', external_axis_check_index=index)
            height, height_source = after, f'{sid}.external_axis.to_height_m'
        elif not points:
            if source != 'SEMANTIC_EVENT':
                raise ValueError('COMPACTNESS_NATIVE_STAGE_HAS_NO_WAYPOINTS')
            terminal = _q(stage.get('terminal_q'))
            if not np.allclose(terminal, q, atol=1e-10, rtol=0):
                raise ValueError('COMPACTNESS_SEMANTIC_EVENT_MOVES_ROBOT')
            add(q, height, 'event-terminal', height_source, source_kind='RECORDED_SEMANTIC_TERMINAL')
    if abs(_height(plan.get('height_m'))-height) > 1e-10:
        raise ValueError('COMPACTNESS_FINAL_CONVEYOR_HEIGHT_MISMATCH')
    return samples


def measure_task_plan(context, plan, verification):
    """Return measured geometry or an explicit metric gap; preserve verification."""
    result = dict(schema='s50_task_plan_compactness_v1', status='NOT_MEASURED',
        scope='RECORDED_PLAN_SAMPLES_NOT_CONTINUOUS_MINIMUM', samples=[],
        all_robot_cycle_verification=deepcopy(verification),
        replaces_all_robot_safety_checks=False, tool_geometry_included=False,
        physical_execution='NOT_RUN')
    try:
        if verification.get('status') != 'PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED':
            raise ValueError('COMPACTNESS_REQUIRES_EXISTING_FULL_CYCLE_VERIFICATION')
        records = coupled_plan_samples(plan)
        binding = context.binding
        binding.verify_source_unchanged()
        result['model_binding'] = binding.identity()
        backend = context.connector.robot
        if getattr(backend, 'model_binding', None) is None or backend.model_binding.identity() != binding.identity():
            raise ValueError('COMPACTNESS_BACKEND_MODEL_IDENTITY_MISMATCH')
        from .conveyor_lift import coupled_conveyor_state
        from .pinocchio_backend import _transform
        from .s50_collision_diagnostics import _world_geometry, diagnostic_coal_obstacle, coal_pair_query
        from .s50_compactness import compactness_sample, summarize_samples
        coal = backend.coal
        for record in records:
            state = coupled_conveyor_state(context.layout, record['conveyor_height_m'], ())
            belts = [box for box in state.boxes if box.name == 'conveyor_transverse']
            if len(belts) != 1:
                raise ValueError('COMPACTNESS_REQUIRES_EXACTLY_ONE_FROZEN_TRANSVERSE_BELT')
            belt = belts[0]
            shape, evidence = diagnostic_coal_obstacle(coal, belt)
            geometry = _world_geometry(backend, record['q_rad'])
            pairs = []
            for item in geometry:
                world = item['world']
                pair = coal_pair_query(coal, item['shape'], _transform(coal, world[:3,:3], world[:3,3]),
                    shape, _transform(coal, belt.rotation, belt.center))
                pairs.append(dict(kind='robot_environment', first={k:item[k] for k in ('link','geometry')},
                    second=dict(object=belt.name, category=belt.category, **evidence), **pair))
            sample = compactness_sample(geometry, pairs, context.layout.physical_mount_transform(),
                q_rad=record['q_rad'], pose_id=record['pose_id'], conveyor_height_m=record['conveyor_height_m'],
                phase=record['phase'], safety_evidence=dict(
                    full_cycle_verification_status=verification['status'], replaces_all_robot_safety_checks=False))
            sample['recorded_plan_sample'] = record
            result['samples'].append(sample)
        result['summary'] = summarize_samples(result['samples'])
        result['status'] = ('MEASURED_RECORDED_PLAN_SAMPLES' if all(
            group['all_component_pairs_known'] for sample in result['samples'] for group in sample['groups'].values())
            else 'UNKNOWN_RECORDED_COMPONENT_DISTANCE')
        result['original_robot_waypoint_count'] = sum(r['source_kind']=='ORIGINAL_ROBOT_WAYPOINT' for r in records)
        result['external_axis_check_count'] = sum(r['source_kind']=='ORIGINAL_EXTERNAL_AXIS_CHECK' for r in records)
    except Exception as exc:
        result.update(status='COMPACTNESS_NOT_MEASURED', failure=dict(reason=type(exc).__name__, detail=str(exc)),
            partial_sample_count=len(result['samples']))
    return result
