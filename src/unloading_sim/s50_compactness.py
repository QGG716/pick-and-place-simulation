"""Measured S50 component clearances, never a substitute for all-body safety."""
from __future__ import annotations
from copy import deepcopy
import numpy as np
from .robot_model_binding import rigid_transform


GROUP_LINKS = {
    'all_robot': ('s50_base', *(f's50_link_{i}' for i in range(1, 7))),
    'active_links': tuple(f's50_link_{i}' for i in range(1, 7)),
    'elbow': ('s50_link_2', 's50_link_3'),
    'wrist': ('s50_link_4', 's50_link_5', 's50_link_6'),
}


def group_envelopes(geometry, world_from_installation_reference):
    """Bounds of actual loaded vertices, expressed in the fixed reference."""
    inverse = np.linalg.inv(rigid_transform(world_from_installation_reference, 'installation_reference'))
    result = {}
    for name, links in GROUP_LINKS.items():
        values = []
        present = set()
        for item in geometry:
            if item['link'] not in links:
                continue
            present.add(item['link'])
            vertices = np.asarray(item['vertices'], dtype=float)
            if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or not np.isfinite(vertices).all():
                raise ValueError('COMPACTNESS_REQUIRES_ACTUAL_FINITE_MESH_VERTICES')
            transform = inverse @ rigid_transform(item['world'], 'actual_geometry_frame')
            values.append(vertices @ transform[:3, :3].T + transform[:3, 3])
        if present != set(links):
            raise ValueError('COMPACTNESS_COMPONENT_GEOMETRY_MISSING:' + name)
        vertices = np.concatenate(values)
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        result[name] = dict(links=list(links), definition='EXPLICIT_LINK_UNION_NOT_JOINT_CENTER',
            mount_frame_bounds_m=[low.tolist(), high.tolist()], horizontal_bounds_xy_m=[low[:2].tolist(), high[:2].tolist()],
            horizontal_span_xy_m=(high[:2]-low[:2]).tolist(),
            maximum_horizontal_radius_from_installation_axis_m=float(np.linalg.norm(vertices[:, :2], axis=1).max()),
            mesh_vertex_count=len(vertices), source='ACTUAL_COAL_MESH_VERTICES')
    return result


def compactness_sample(geometry, pairs, world_from_installation_reference, *, q_rad,
        pose_id, conveyor_height_m, phase=None, safety_evidence=None):
    """Retain component-specific minima and the q that actually produced them."""
    q = np.asarray(q_rad, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('INVALID_COMPACTNESS_JOINT_VECTOR')
    envelopes = group_envelopes(geometry, world_from_installation_reference)
    belt = [p for p in pairs if p.get('kind') == 'robot_environment'
        and p.get('second', {}).get('object') == 'conveyor_transverse']
    groups = {}
    for name, links in GROUP_LINKS.items():
        selected = [p for p in belt if p['first']['link'] in links]
        expected = {(g['link'], g['geometry']) for g in geometry if g['link'] in links}
        actual = [(p['first']['link'], p['first']['geometry']) for p in selected]
        complete_set = len(actual) == len(set(actual)) and set(actual) == expected
        known = [p for p in selected if p.get('status') == 'QUERIED'
            and p.get('unsigned_separation_m') is not None
            and np.isfinite(p['unsigned_separation_m'])]
        complete = complete_set and len(known) == len(selected)
        nearest = min(known, key=lambda p: p['unsigned_separation_m']) if known else None
        groups[name] = dict(envelope=envelopes[name], queried_shape_pairs=len(selected),
            expected_shape_pairs=len(expected), all_component_pairs_known=complete,
            status='MEASURED_AT_RECORDED_CONFIGURATION' if complete else 'UNKNOWN_COMPONENT_DISTANCE',
            minimum_unsigned_separation_m=nearest['unsigned_separation_m'] if complete else None,
            any_intersection=any(p.get('intersects') is True for p in selected),
            nearest_observed_pair=deepcopy(nearest),
            q_rad=q.tolist(), pose_id=str(pose_id), phase=phase,
            conveyor_height_m=conveyor_height_m)
    return dict(schema='s50_component_compactness_sample_v1', pose_id=str(pose_id), phase=phase,
        q_rad=q.tolist(), conveyor_height_m=conveyor_height_m,
        world_from_installation_reference=np.asarray(world_from_installation_reference).tolist(),
        groups=groups, scope='MEASURED_GEOMETRY_REPORT_ONLY', safety_evidence=deepcopy(safety_evidence),
        replaces_all_robot_safety_checks=False, mechanical_limit_qualification_implied=False,
        tool_geometry_included=False)


def summarize_samples(samples):
    """Sampled phase extrema; this is not continuous trajectory clearance."""
    samples = list(samples)
    result = {}
    phases = list(dict.fromkeys(s.get('phase') or 'CAD_DIAGNOSTIC' for s in samples))
    for phase in phases:
        rows = [s for s in samples if (s.get('phase') or 'CAD_DIAGNOSTIC') == phase]
        groups = {}
        for name in GROUP_LINKS:
            complete = all(row['groups'][name]['all_component_pairs_known'] for row in rows)
            observed = [row for row in rows if row['groups'][name]['minimum_unsigned_separation_m'] is not None]
            nearest = min(observed, key=lambda r:r['groups'][name]['minimum_unsigned_separation_m']) if observed else None
            widest = max(rows, key=lambda r:r['groups'][name]['envelope']['maximum_horizontal_radius_from_installation_axis_m'])
            bounds = np.asarray([r['groups'][name]['envelope']['mount_frame_bounds_m'] for r in rows])
            extrema = {}
            for axis, label in enumerate(('x', 'y')):
                for side, operation in ((0, np.argmin), (1, np.argmax)):
                    index = int(operation(bounds[:,side,axis]));record = rows[index]
                    extrema[label+('_min' if side==0 else '_max')] = dict(value_m=float(bounds[index,side,axis]),
                        q_rad=record['q_rad'], pose_id=record['pose_id'])
            groups[name] = dict(sample_count=len(rows), all_samples_known=complete,
                minimum_sampled_distance_m=nearest['groups'][name]['minimum_unsigned_separation_m'] if complete and nearest else None,
                minimum_configuration=None if nearest is None else dict(pose_id=nearest['pose_id'],q_rad=nearest['q_rad'],
                    conveyor_height_m=nearest['conveyor_height_m'], pair=nearest['groups'][name]['nearest_observed_pair']),
                horizontal_extrema=extrema, maximum_sampled_radius_m=widest['groups'][name]['envelope']['maximum_horizontal_radius_from_installation_axis_m'],
                maximum_radius_configuration=dict(pose_id=widest['pose_id'],q_rad=widest['q_rad']),
                any_intersection=any(row['groups'][name]['any_intersection'] for row in rows))
        result[phase] = groups
    return dict(schema='s50_component_compactness_summary_v1', sample_count=len(samples), phases=result,
        extrema_scope='RECORDED_SAMPLES_ONLY_NOT_CONTINUOUS_PATH_MINIMUM',
        replaces_all_robot_safety_checks=False, structural_unreachability_claimed=False)
