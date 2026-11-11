"""Read actual CAD collision probes without treating them as legal task poses."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from unloading_sim.robot_model_binding import rigid_transform


def diagnostic_data(path, definition, robot, scene_context, validate_context, *, expected_obstacles=None, initial_height_m=.6):
    if path is None:
        return dict(status='NOT_RUN', poses=[], qualification_scope='CAD_DIAGNOSTIC_NOT_TASK')
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw)
    if data.get('schema') != 's50_runtime_collision_diagnostics_v1':
        raise ValueError('UNKNOWN_S50_COLLISION_DIAGNOSTIC_SCHEMA')
    if data.get('qualification_scope') != 'CAD_DIAGNOSTIC_NOT_TASK':
        raise ValueError('DIAGNOSTICS_MUST_NOT_CLAIM_LEGAL_TASK_POSES')
    if data.get('model_definition_sha256') != definition['definition_sha256']:
        raise ValueError('DIAGNOSTIC_MODEL_IDENTITY_MISMATCH')
    context = data.get('scene_context') or {key:data.get(key) for key in scene_context}
    validate_context(context, scene_context)
    if not np.isclose(data.get('conveyor_height_m',float('nan')), initial_height_m, atol=1e-12, rtol=0):
        raise ValueError('DIAGNOSTIC_CONVEYOR_HEIGHT_MISMATCH')
    obstacles=data.get('obstacles')
    if not isinstance(obstacles,list) or any(not isinstance(row,dict) or not row.get('name') for row in obstacles):
        raise ValueError('DIAGNOSTIC_OBSTACLE_IDENTITIES_REQUIRED')
    by_name={row['name']:row for row in obstacles}
    if len(by_name)!=len(obstacles):raise ValueError('DIAGNOSTIC_DUPLICATE_OBSTACLE')
    if expected_obstacles is not None:
        expected={row['id']:row for row in expected_obstacles}
        if by_name.keys()!=expected.keys():raise ValueError('DIAGNOSTIC_OBSTACLE_SET_MISMATCH')
        for name,row in by_name.items():
            if (row.get('solid') != expected[name].get('solid')
                    or not np.allclose(row['size_m'],expected[name]['size_m'],atol=1e-12,rtol=0)
                    or not np.allclose(rigid_transform(row['world_from_entity'],'obstacle'),
                        expected[name]['world_from_entity'],atol=1e-12,rtol=0)):
                raise ValueError('DIAGNOSTIC_OBSTACLE_GEOMETRY_MISMATCH:'+name)
    poses = data.get('poses')
    if not isinstance(poses, list) or not poses:
        raise ValueError('DIAGNOSTICS_HAVE_NO_ACTUAL_POSES')
    base = rigid_transform(context['world_from_model_base'], 'world_from_model_base')
    known_links = {link['name'] for link in definition['links']}
    ids = set()
    for pose in poses:
        q = np.asarray(pose.get('q_rad'), dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('INVALID_DIAGNOSTIC_JOINT_VECTOR')
        pose_id = pose.get('pose_id')
        if not isinstance(pose_id, str) or not pose_id or pose_id in ids:
            raise ValueError('DIAGNOSTIC_POSE_ID_MISSING_OR_DUPLICATED')
        ids.add(pose_id)
        if not np.isclose(pose.get('conveyor_height_m',float('nan')), initial_height_m, atol=1e-12, rtol=0):
            raise ValueError('DIAGNOSTIC_POSE_CONVEYOR_HEIGHT_MISMATCH')
        frames = pose.get('link_frames_world') or {}
        reference = robot.named_link_frames(q)
        for name in known_links:
            actual = rigid_transform(frames.get(name), 'diagnostic.' + name)
            if not np.allclose(actual, base @ reference[name], atol=1e-9, rtol=0):
                raise ValueError('DIAGNOSTIC_CORE_FK_MISMATCH:' + name)
        pairs = pose.get('pairs')
        if not isinstance(pairs, list):
            raise ValueError('DIAGNOSTIC_PAIR_RESULTS_REQUIRED')
        for pair in pairs:
            if not isinstance(pair.get('first'), dict) or pair['first'].get('link') not in known_links:
                raise ValueError('UNKNOWN_DIAGNOSTIC_ROBOT_LINK')
            second = pair.get('second') or {}
            if not (second.get('object') in by_name or second.get('link') in known_links):
                raise ValueError('UNKNOWN_DIAGNOSTIC_SECOND_OBJECT')
            distance = pair.get('surface_distance_m')
            if distance is not None and (isinstance(distance, bool) or not isinstance(distance, (int,float))
                                         or not np.isfinite(distance)):
                raise ValueError('INVALID_DIAGNOSTIC_DISTANCE')
            if pair.get('nearest_points_valid') is True:
                nearest = np.asarray(pair.get('nearest_points_world'), dtype=float)
                if nearest.shape != (2,3) or not np.isfinite(nearest).all():
                    raise ValueError('INVALID_DIAGNOSTIC_NEAREST_POINTS')
    data['source_report_sha256'] = hashlib.sha256(raw).hexdigest()
    data['scene_context_verified'] = True
    data['viewer_fk_crosscheck'] = 'PASS_ALL_RECORDED_LINK_FRAMES'
    return data
