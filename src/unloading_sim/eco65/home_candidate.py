"""Explicit simulation initial state, never evidence of a homing motion."""
from copy import deepcopy
import numpy as np
from ..moveit2_backend import digest


def equipment_identity(scene):
    # Warehouse restoration adds obstacles but does not invalidate the equipment
    # proposal. Acceptance still requires the complete current planning scene.
    identity = dict(config_sha256=scene['config_sha256'], base_position_m=scene['base_position_m'],
        T_W_D=scene['T_W_D'], boxes=scene['boxes'],
        obstacles=[o for o in scene['obstacles'] if o['owner'] != 'warehouse'])
    if scene.get('schema') == 'eco65_native_12carton_scene_v1':
        identity.update(native_world_objects=scene['native_world_objects'],
            moving_attachments=scene['moving_attachments'],
            fixed_attachment_interfaces=scene['fixed_attachment_interfaces'],
            permanent_camera=scene.get('permanent_camera'), native_task=scene['native_task'])
    return digest(identity)


def make_candidate(q, tool, scene, provenance):
    value = dict(schema='eco65_home_candidate_v1', q_rad=np.asarray(q, float).tolist(),
        joint_names=[f'joint_{i}' for i in range(1, 7)], tool_fingerprint=digest(tool),
        equipment_fingerprint=equipment_identity(scene), provenance=deepcopy(provenance),
        semantics='SIMULATION_INITIAL_STATE_NOT_A_VERIFIED_HOMING_MOTION')
    value['fingerprint'] = digest(value)
    return verify_candidate(value, tool, scene)


def verify_candidate(value, tool, scene):
    check = dict(value); expected = check.pop('fingerprint')
    q = np.asarray(value['q_rad'], float)
    if (digest(check) != expected or value['schema'] != 'eco65_home_candidate_v1' or
            value['joint_names'] != [f'joint_{i}' for i in range(1, 7)] or
            q.shape != (6,) or not np.isfinite(q).all() or not value['provenance'] or
            value['semantics'] != 'SIMULATION_INITIAL_STATE_NOT_A_VERIFIED_HOMING_MOTION'):
        raise ValueError('HOME_CANDIDATE_INVALID')
    if value['tool_fingerprint'] != digest(tool) or value['equipment_fingerprint'] != equipment_identity(scene):
        raise ValueError('HOME_CANDIDATE_TOOL_OR_EQUIPMENT_CHANGED')
    return value
