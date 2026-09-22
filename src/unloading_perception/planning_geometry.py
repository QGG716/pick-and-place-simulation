"""Offline, conditional cuboid envelopes; never an online admission decision.

Only fixed, explicitly configured axes are supported. For each dimension-axis
permutation we solve six face-coordinate intervals, not a fitted box pose.
Patch vertices constrain inclusion, never equality to physical edges.
"""
from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from copy import deepcopy
import itertools
import json
import math
from pathlib import Path

import numpy as np

from unloading_contracts import canonical_fingerprint, to_wire
from unloading_sim.geometry import OBB
from .algorithm_artifact import file_reference, load_algorithm_artifact
from .geometry import validate_transform_parent_child
from .observed_faces import DIRECT_FACE_EVIDENCE


def _array(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name}: invalid shape or nonfinite coordinates')
    return result


def _text(record, *keys):
    if any(not isinstance(record.get(k), str) or not record[k].strip() for k in keys):
        raise ValueError('missing identity/source: ' + ','.join(keys))


def _configuration(config):
    if config.get('schema_version') != 'observed_size_prior_v1':
        raise ValueError('unsupported size prior schema')
    _text(config, 'config_identity')
    size = config['size_prior']
    _text(size, 'specification_id', 'source', 'scope', 'error_basis')
    dims = _array(size['full_dimensions_interval_m'], (3, 2), 'size intervals')
    if (dims <= 0).any() or (dims[:, 0] > dims[:, 1]).any():
        raise ValueError('size intervals must be positive and ordered')
    correspondence = size['axis_correspondence']
    if correspondence == 'unknown':
        mappings = list(itertools.permutations(range(3)))
    elif isinstance(correspondence, (list, tuple)) and sorted(correspondence) == [0, 1, 2]:
        mappings = [tuple(correspondence)]
    else:
        raise ValueError('axis_correspondence must be unknown or a dimension index per box axis')
    env = config.get('environment_prior')
    if env is None:
        return dims, mappings, None, None, None
    _text(env, 'source', 'scope')
    rotation = _array(env['rotation_world_from_box_axes'], (3, 3), 'axis rotation')
    transform = np.eye(4)
    transform[:3, :3] = rotation
    validate_transform_parent_child(transform)
    measurement = config['measurement_assumptions']
    _text(measurement, 'source', 'support_semantics')
    error = float(measurement['coordinate_error_bound_m'])
    angle = float(measurement['normal_error_bound_rad'])
    if not math.isfinite(error) or error < 0 or not math.isfinite(angle) or not 0 <= angle < math.pi / 4:
        raise ValueError('invalid measurement bounds; normal bound must be below pi/4')
    return dims, mappings, rotation, error, angle


def _surface(surface):
    _text(surface, 'face_id', 'module_id', 'capture_id', 'source_instance_id',
          'calibration_identity', 'sensor_epoch', 'clock_domain')
    if surface.get('schema_version') != 'observed_surface_v1' or surface.get('frame_id') != 'world':
        raise ValueError('only calibrated observed_surface_v1 world coordinates are supported')
    if surface.get('evidence') not in DIRECT_FACE_EVIDENCE or surface.get('volume_status') != 'UNKNOWN':
        raise ValueError('direct measured patch evidence with unknown volume is required')
    points = _array(surface['corners_3d_m'], (4, 3), 'patch vertices')
    _array(surface['boundary_2d_px'], (4, 2), 'image support boundary')
    normal = _array(surface['plane_normal'], (3,), 'normal')
    scalars = _array([surface['plane_offset_m'], surface['plane_residual_m'],
                      surface['point_support_count'], surface['point_support_ratio'],
                      surface['capture_time']], (5,), 'measurement metadata')
    offset, residual, count, ratio, capture_time = scalars
    if abs(np.linalg.norm(normal) - 1) > 1e-6 or residual < 0 or count <= 0 or not 0 < ratio <= 1 or capture_time < 0:
        raise ValueError('invalid normal, residual, time or support')
    if np.max(np.abs(points @ normal + offset)) > 1e-4:
        raise ValueError('vertices disagree with declared measured plane')
    # A convex, nondegenerate ordered patch is required; no implicit corner repair.
    edges = np.roll(points, -1, axis=0) - points
    turns = np.cross(edges, np.roll(edges, -1, axis=0)) @ normal
    if not (np.all(turns > 1e-12) or np.all(turns < -1e-12)):
        raise ValueError('degenerate or unordered support polygon')
    transform = np.asarray(validate_transform_parent_child(surface['T_W_C_at_capture']))
    # Existing measured normals point outward towards the capture camera.
    if np.dot(normal, transform[:3, 3] - points.mean(axis=0)) <= 0:
        raise ValueError('normal does not point toward the capture camera')
    return points, normal, transform[:3, 3]


def _intervals(points, assignments, dimensions, error):
    """Exact interval projection of L <= p+e, U >= p-e, dmin<=U-L<=dmax.

    A signed face additionally constrains its L or U to every p_i +/- e.
    This 2-variable difference system is independent on each configured axis.
    Its extrema cover ALL continuous translations and sizes, not pose samples.
    """
    all_points = np.concatenate(points)
    low_min = np.full(3, -np.inf)
    low_max = all_points.min(axis=0) + error
    high_min = all_points.max(axis=0) - error
    high_max = np.full(3, np.inf)
    for patch, (axis, sign) in zip(points, assignments):
        minimum = float(patch[:, axis].max() - error)
        maximum = float(patch[:, axis].min() + error)
        if sign < 0:
            low_min[axis] = max(low_min[axis], minimum)
            low_max[axis] = min(low_max[axis], maximum)
        else:
            high_min[axis] = max(high_min[axis], minimum)
            high_max[axis] = min(high_max[axis], maximum)
    dmin, dmax = dimensions.T
    new_low_min = np.maximum(low_min, high_min - dmax)
    new_low_max = np.minimum(low_max, high_max - dmin)
    new_high_min = np.maximum(high_min, low_min + dmin)
    new_high_max = np.minimum(high_max, low_max + dmax)
    if np.any(new_low_min > new_low_max + 1e-12) or np.any(new_high_min > new_high_max + 1e-12):
        return None
    # Tiny arithmetic discrepancies are widened outwards, never used to shrink evidence.
    low = np.stack((np.minimum(new_low_min, new_low_max), np.maximum(new_low_min, new_low_max)), axis=1)
    high = np.stack((np.minimum(new_high_min, new_high_max), np.maximum(new_high_min, new_high_max)), axis=1)
    return low, high


def envelope_to_obb(envelope: Mapping, *, name: str = 'offline-envelope') -> OBB:
    """Convert to the existing planner's OBB, with stronger boundary validation."""
    if envelope.get('frame_id') != 'world':
        raise ValueError('envelope must use world coordinates')
    center = _array(envelope['center_m'], (3,), 'center')
    size = _array(envelope['full_dimensions_m'], (3,), 'full dimensions')
    rotation = _array(envelope['rotation_world_from_box_axes'], (3, 3), 'rotation')
    transform = np.eye(4)
    transform[:3, :3] = rotation
    validate_transform_parent_child(transform)
    if (size <= 0).any():
        raise ValueError('full dimensions must be positive')
    box = OBB(center=center, half_extents=size / 2, rotation=rotation, name=name, category='carton')
    if not np.isfinite(box.corners()).all():
        raise ValueError('invalid planner OBB corners')
    return box


def _envelope(low, high, rotation):
    # Outward rounding covers floating point evaluation of analytic extrema.
    pad = 1e-12 * max(1., float(np.max(np.abs(np.r_[low, high]))))
    low, high = low - pad, high + pad
    result = {'frame_id': 'world', 'center_m': (rotation @ ((low + high) / 2)).tolist(),
              'full_dimensions_m': (high - low).tolist(),
              'rotation_world_from_box_axes': rotation.tolist()}
    envelope_to_obb(result)
    return result


def derive_geometry(surfaces: Sequence[Mapping], config: Mapping, *, enabled: bool = False) -> dict:
    """GT-free production interface: measured surfaces and explicit configuration only.

    Both the caller and configuration must enable the experiment. Returned OBBs
    bound a conditional cuboid model, not unmodelled dents/flaps or free space.
    """
    result = {'status': 'DISABLED', 'face_count': len(surfaces), 'independent_directions': 0,
              'candidates': [], 'rejected_hypotheses': [], 'reasons': [], 'ambiguities': [],
              'checks': [], 'planning_geometry': None, 'unique_reconstruction': False,
              'conditional_conservative_volume': False, 'planning_admissible': False,
              'candidate_eligible': False}
    if not enabled or config.get('enabled') is not True:
        result['reasons'] = ['SIZE_PRIOR_EXPERIMENT_DISABLED']
        return result
    try:
        dims, mappings, rotation, error, angle = _configuration(config)
        validated = [_surface(s) for s in surfaces]
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        result.update(status='INVALID_INPUT', reasons=[str(exc)])
        return result
    if not validated or rotation is None:
        result.update(status='INSUFFICIENT_INFORMATION', reasons=[
            'NO_MEASURED_FACES' if not validated else 'FIXED_AXIS_ENVIRONMENT_PRIOR_REQUIRED'])
        return result
    points, assignments = [], []
    for surface, (patch, normal, camera) in zip(surfaces, validated):
        local, local_normal = patch @ rotation, normal @ rotation
        axis = int(np.argmax(np.abs(local_normal)))
        sign = 1 if local_normal[axis] > 0 else -1
        normal_angle = math.acos(float(np.clip(abs(local_normal[axis]), 0, 1)))
        result['checks'].append({'face_id': surface['face_id'], 'axis': axis, 'sign': sign,
                                 'normal_angle_rad': normal_angle})
        if normal_angle > angle + 1e-12:
            result.update(status='PRIOR_OBSERVATION_CONFLICT', reasons=['NORMAL_INCOMPATIBLE_WITH_FIXED_AXES'])
            return result
        if sign * ((camera @ rotation)[axis] - local[:, axis].mean()) <= error:
            result.update(status='INSUFFICIENT_INFORMATION', reasons=['OUTWARD_SIDE_NOT_RESOLVED_WITHIN_ERROR_BOUND'])
            return result
        points.append(local)
        assignments.append((axis, sign))
    result['independent_directions'] = len({axis for axis, _ in assignments})
    bounds = []
    seen = set()
    for mapping in mappings:
        dimensions = dims[list(mapping)]
        key = tuple(dimensions.ravel())
        if key in seen:
            continue
        seen.add(key)
        solution = _intervals(points, assignments, dimensions, error)
        if solution is None:
            result['rejected_hypotheses'].append({'axis_dimension_indices': list(mapping),
                'reason': 'NO_FEASIBLE_FACE_POSITIONS_AND_PATCH_INCLUSION'})
            continue
        low, high = solution
        bounds.append((low[:, 0], high[:, 1]))
        result['candidates'].append({'axis_dimension_indices': list(mapping),
            'full_dimensions_interval_m': dimensions.tolist(),
            'negative_face_position_interval_m': low.tolist(),
            'positive_face_position_interval_m': high.tolist(),
            'continuous_position_ambiguity': bool(np.max(np.r_[np.diff(low), np.diff(high)]) > 1e-10),
            'envelope': _envelope(low[:, 0], high[:, 1], rotation)})
    if not bounds:
        result.update(status='PRIOR_OBSERVATION_CONFLICT', reasons=['ALL_DIMENSION_ASSIGNMENTS_CONFLICT'])
        return result
    if len(bounds) > 1:
        result['ambiguities'].append('MULTIPLE_DIMENSION_AXIS_ASSIGNMENTS')
    if any(c['continuous_position_ambiguity'] for c in result['candidates']):
        result['ambiguities'].append('CONTINUOUS_FACE_POSITION_OR_SIZE_UNCERTAINTY')
    if result['independent_directions'] < 3:
        result['ambiguities'].append('UNOBSERVED_INDEPENDENT_FACE_DIRECTIONS')
    result['planning_geometry'] = _envelope(np.min([b[0] for b in bounds], axis=0),
                                          np.max([b[1] for b in bounds], axis=0), rotation)
    result['conditional_conservative_volume'] = True
    result['unique_reconstruction'] = len(bounds) == 1 and not result['candidates'][0]['continuous_position_ambiguity']
    result['status'] = 'CONSERVATIVE_VOLUME' if result['unique_reconstruction'] else 'MULTIPLE_HYPOTHESES'
    result['reasons'] = ['ANALYTIC_ENVELOPE_OF_ALL_FEASIBLE_TRANSLATIONS_SIZES_AND_ENUMERATED_AXIS_ASSIGNMENTS',
                         'CONDITIONAL_ON_FIXED_AXES_CUBOID_SPECIFICATION_AND_DECLARED_MEASUREMENT_BOUNDS',
                         'PATCH_BOUNDARIES_NOT_USED_AS_PHYSICAL_EDGES']
    return result


def adapt_artifact(index_path, expected_sha256, config: Mapping, output_directory, *,
                   enabled=False, selection: Mapping | None = None) -> dict:
    """Validate bindings, freeze selection, write into a new directory, convert OBBs.

    No GT, robot state, tracker, replay publication or execution bridge is loaded.
    """
    observation, index = load_algorithm_artifact(index_path, expected_sha256)
    cargo_by_id = {c.source_instance_id: c for c in observation.cargo}
    if selection is None:
        selection = {'rule': 'all fused source instances, lexicographically sorted',
                     'artifact_sha256': expected_sha256, 'source_instance_ids': sorted(cargo_by_id)}
    ids = list(selection['source_instance_ids'])
    if (selection.get('artifact_sha256') != expected_sha256 or not ids or len(set(ids)) != len(ids)
            or any(i not in cargo_by_id for i in ids) or not selection.get('rule')):
        raise ValueError('invalid or unbound selection')
    root = Path(output_directory).resolve()
    artifact_root = Path(index_path).resolve().parent
    if root == artifact_root or artifact_root in root.parents:
        raise ValueError('derived output must be outside the original artifact directory')
    root.mkdir(parents=True, exist_ok=False)

    def write(name, value):
        (root / name).write_text(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                           indent=2, allow_nan=False) + '\n', encoding='utf-8')

    # Freeze inputs/selection BEFORE evaluating any selected object.
    write('selection.json', selection)
    write('prior.json', config)
    source_bytes = Path(index['observation']['path']).read_bytes()
    (root / 'original_observation.json').write_bytes(source_bytes)
    reports = []
    for instance in ids:
        item = cargo_by_id[instance]
        raw_surfaces = to_wire(item)['observed_surfaces']
        report = derive_geometry(item.observed_surfaces, config, enabled=enabled)
        # Association uncertainty cannot become a single object's volume.
        if report['status'] != 'DISABLED' and ('AMBIGUOUS' in item.association_status or 'CONFLICT' in item.association_status):
            report.update(status='INSUFFICIENT_INFORMATION', planning_geometry=None,
                          conditional_conservative_volume=False, unique_reconstruction=False,
                          reasons=report['reasons'] + ['UNRESOLVED_INPUT_ASSOCIATION'])
        report.update(source_instance_id=instance, association_status=item.association_status,
                      raw_observed_surfaces=raw_surfaces, source_members=to_wire(item.raw_result).get('source_members', []))
        report['planner_obb_validated'] = report['planning_geometry'] is not None
        if report['planner_obb_validated']:
            box = envelope_to_obb(report['planning_geometry'], name=instance)
            report['planner_obb_corners_m'] = box.corners().tolist()
        reports.append(report)
    result = {'schema_version': 'offline_planning_geometry_v1',
        'source_artifact': file_reference(Path(index_path)), 'source_observation': dict(index['observation']),
        'source_observation_fingerprint': canonical_fingerprint(observation),
        'prior_fingerprint': canonical_fingerprint(config), 'experiment_enabled': bool(enabled and config.get('enabled') is True),
        'source_capture_time': observation.capture_time, 'source_epoch': observation.source_epoch,
        'source_sequence': observation.source_sequence, 'source_clock_domain': observation.clock_domain,
        'source_coverage': to_wire(observation.coverage),
        'original_unknown_regions': to_wire(observation.unknown_regions),
        'planning_admissible': False, 'candidate_eligible': False,
        'blocking_reasons': ['OFFLINE_DERIVED_GEOMETRY_ONLY', 'HISTORICAL_REPLAY_DISPLAY_ONLY',
                             'UNKNOWN_OR_UNTRANSFORMED_REGIONS', 'NO_ROBOT_OR_SCENE_EXECUTION_VALIDATION'],
        'raw_image_automatic': False, 'evidence_label': 'ORACLE-PROMPTED',
        'unselected_object_ids': sorted(set(cargo_by_id) - set(ids)), 'objects': reports}
    write('derived_geometry.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', required=True)
    parser.add_argument('--artifact-sha256', required=True)
    parser.add_argument('--prior', required=True)
    parser.add_argument('--selection')
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--enable-size-prior', action='store_true')
    args = parser.parse_args(argv)
    config = json.loads(Path(args.prior).read_text(encoding='utf-8'))
    # Explicit CLI opt-in is recorded in the copied configuration, not the source file.
    config = deepcopy(config)
    if args.enable_size_prior:
        config['enabled'] = True
    selection = None if not args.selection else json.loads(Path(args.selection).read_text(encoding='utf-8'))
    result = adapt_artifact(args.artifact, args.artifact_sha256, config, args.output_directory,
                            enabled=args.enable_size_prior, selection=selection)
    print(json.dumps({'objects': len(result['objects']), 'conditional_envelopes': sum(
        o['planner_obb_validated'] for o in result['objects']), 'planning_admissible': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
