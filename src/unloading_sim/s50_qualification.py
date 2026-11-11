"""Purpose-specific S50 evidence checks; CAD preview is not task qualification."""
from __future__ import annotations

import numpy as np


AXIS_LABELS = tuple(f'J{i}' for i in range(1, 7))


def has_source(value):
    if isinstance(value, str):
        return bool(value.strip())
    if isinstance(value, dict):
        return any(has_source(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return any(has_source(item) for item in value)
    return False


def convert_controller_limits(controller_limits, sign, offset_rad):
    """Convert q_controller = sign*q_sim + offset without expanding the range."""
    limits = np.asarray(controller_limits, dtype=float)
    if limits.shape != (2,) or not np.isfinite(limits).all() or limits[0] >= limits[1]:
        raise ValueError('INVALID_CONTROLLER_POSITION_LIMITS')
    if isinstance(sign, bool) or sign not in (-1, 1):
        raise ValueError('CONTROLLER_SIGN_MUST_BE_PLUS_OR_MINUS_ONE')
    if isinstance(offset_rad, bool) or not isinstance(offset_rad, (int, float)) or not np.isfinite(offset_rad):
        raise ValueError('CONTROLLER_OFFSET_MUST_BE_FINITE_RADIANS')
    return np.sort((limits - offset_rad) / sign)


def controller_mapping_audit(data):
    mapping = data.get('controller_mapping') or {}
    joints = mapping.get('joints') or {}
    limits = data.get('published_controller_position_limits_rad', data.get('joint_position_limits_rad', {}))
    missing, converted, records = [], {}, []
    for label in AXIS_LABELS:
        entry = joints.get(label) or {}
        problems = []
        if entry.get('verified') is not True or not has_source(entry.get('source')):
            problems.append('REFERENCE_NOT_VERIFIED')
        try:
            result = convert_controller_limits(limits.get(label), entry.get('sign'), entry.get('offset_rad'))
        except (TypeError, ValueError) as exc:
            problems.append(str(exc))
            result = None
        if not problems:
            converted[label] = result.tolist()
        missing.extend('controller_mapping.' + label + '.' + item for item in problems)
        records.append(dict(axis=label, verified=not problems, sign=entry.get('sign'),
            offset_rad=entry.get('offset_rad'), source=entry.get('source'),
            controller_limits_rad=limits.get(label), simulation_limits_rad=converted.get(label),
            missing=problems))
    return dict(ready=not missing, equation='q_controller = sign * q_sim + offset_rad',
                axes=records, converted_limits_rad=converted, missing=missing,
                display_ranges_are_not_mechanical_limits=True)


def geometric_definition_errors(data):
    """Physical definitions needed for installed, limited, collision-checked tasks."""
    missing = controller_mapping_audit(data)['missing']
    installation = data.get('installation') or {}
    for field in ('physical_mount_frame', 'mount_clocking'):
        evidence = installation.get(field) or {}
        if evidence.get('verified') is not True or not has_source(evidence.get('source')):
            missing.append('installation.' + field + '.NOT_VERIFIED')
    connection = (data.get('tool_interface') or {}).get('physical_connection') or {}
    if connection.get('verified') is not True or not has_source(connection.get('source')):
        missing.append('tool_interface.physical_connection.NOT_VERIFIED')
    collision = data.get('collision') or {}
    coverage = collision.get('coverage') or {}
    if coverage.get('verified') is not True or not has_source(coverage.get('source')):
        missing.append('collision.coverage.NOT_VERIFIED')
    if data.get('model_usage') == 'cad_preview_fk_only':
        missing.append('CAD_PREVIEW_DISPLAY_RANGES_FORBIDDEN_FOR_TASK_PLANNING')
    return missing


def timing_audit(data):
    """Unresolved velocity revisions affect timing, never static FK admission."""
    conflict = data.get('joint_velocity_version_conflict') or {}
    unresolved = bool(conflict and not has_source(conflict.get('verified_resolution_source')))
    assumption = data.get('time_parameterization_assumption') or {}
    if assumption:
        missing = []
        if assumption.get('kind') != 'ENGINEERING_ASSUMPTION' or assumption.get('manufacturer_confirmed') is not False:
            missing.append('timing.ASSUMPTION_MUST_NOT_CLAIM_MANUFACTURER_CONFIRMATION')
        if not has_source(assumption.get('source_or_rationale')):
            missing.append('timing.ASSUMPTION_SOURCE_MISSING')
        for key in ('velocity_rad_s', 'acceleration_rad_s2'):
            try:
                values = np.asarray(assumption.get(key), dtype=float)
                if values.shape != (6,) or not np.isfinite(values).all() or np.any(values <= 0):
                    missing.append('timing.INVALID_' + key)
            except (TypeError, ValueError):
                missing.append('timing.INVALID_' + key)
        return dict(ready=not missing, status='READY_WITH_ENGINEERING_ASSUMPTION' if not missing else 'NOT_READY',
            manufacturer_confirmed=False, version_conflict_unresolved=unresolved, assumption=assumption,
            missing=missing, blocks_static_fk=False, blocks_geometric_search=False)
    missing = []
    if unresolved:
        missing.append('joint_velocity_version_conflict.verified_resolution_source')
    for key, source_key in (
            ('joint_velocity_limits_rad_s', 'joint_velocity_source'),
            ('joint_acceleration_limits_rad_s2', 'joint_acceleration_source')):
        try:
            values = np.asarray(data.get(key), dtype=float)
            if values.shape != (6,) or not np.isfinite(values).all() or np.any(values <= 0):
                missing.append('timing.INVALID_' + key)
        except (TypeError, ValueError):
            missing.append('timing.INVALID_' + key)
        if not has_source(data.get(source_key)):
            missing.append('timing.' + source_key + '.SOURCE_MISSING')
    return dict(ready=not missing, status='NOT_READY' if missing else 'PUBLISHED_LIMITS_AVAILABLE_NOT_DYNAMICS_QUALIFIED',
        manufacturer_confirmed=not missing, version_conflict_unresolved=unresolved,
        assumption=None, missing=missing, blocks_static_fk=False, blocks_geometric_search=False)
