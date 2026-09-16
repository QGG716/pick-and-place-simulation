"""Derive the perception workcell without mutating archived feasibility evidence."""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import json
import math

import yaml


def trailer_solids(trailer):
    """Four finite box solids; thickness is entirely outside the clear interior."""
    lo, hi = map(float, trailer['local_x_range_m'])
    width, height, thickness = (float(trailer[k]) for k in
                                ('inside_width_m', 'inside_height_m', 'wall_thickness_m'))
    if not all(math.isfinite(v) for v in (lo, hi, width, height, thickness)) or hi <= lo or min(width, height, thickness) <= 0:
        raise ValueError('invalid trailer section')
    center, length = (lo + hi) / 2, hi - lo
    records = [
        ('Floor', [center, 0, -thickness / 2], [length, width + 2 * thickness, thickness]),
        ('LeftWall', [center, (width + thickness) / 2, height / 2], [length, thickness, height]),
        ('RightWall', [center, -(width + thickness) / 2, height / 2], [length, thickness, height]),
        ('Roof', [center, 0, height + thickness / 2], [length, width + 2 * thickness, thickness]),
    ]
    return [{'name': name, 'center_m': xyz, 'size_xyz_m': size, 'role': 'static_environment',
             'collision_enabled': True, 'visible': True,
             'pose_world': [[1., 0., 0., xyz[0]], [0., 1., 0., xyz[1]], [0., 0., 1., xyz[2]], [0., 0., 0., 1.]]}
            for name, xyz, size in records]


def derive_effective_scene(snapshot, contract, config, project_root):
    """Validate the parent, then derive and fingerprint the full new payload."""
    from .isaac_validation import feasibility_digest, load_vision_rig_spec
    root = Path(project_root)
    for value, key in ((snapshot, 'scene_fingerprint'), (contract, 'contract_fingerprint')):
        payload = dict(value)
        recorded = payload.pop(key)
        if recorded != feasibility_digest(payload):
            raise ValueError('archived parent fingerprint mismatch')
    if snapshot['scene_fingerprint'] != contract['scene_fingerprint']:
        raise ValueError('archived parent scene mismatch')
    effective_path = root / config['effective_scene_config']
    effective = yaml.safe_load(effective_path.read_text(encoding='utf-8'))
    if effective['schema_version'] != 'perception_workcell_v2':
        raise ValueError('unsupported effective scene')
    assembly_xyz = [row[3] for row in snapshot['assembly']['pose_world'][:3]]
    if assembly_xyz != effective['assembly_translation_world_m']:
        raise ValueError('frozen assembly changed')
    if config['layout_bundle']['robot_base_xyz_A_m'] != effective['robot_base_xyz_A_m']:
        raise ValueError('robot base sources disagree')
    spec = load_vision_rig_spec(root / config['vision_rig_config'])
    assets = {key: json.loads((root / value).read_text(encoding='utf-8')) for key, value in effective['assets'].items()}
    source = {'parent_layout_id': snapshot['layout_id'], 'parent_scene_fingerprint': snapshot['scene_fingerprint'],
              'parent_contract_fingerprint': contract['contract_fingerprint'],
              'effective_config': effective, 'asset_configs': assets,
              'rig_config': yaml.safe_load((root / config['vision_rig_config']).read_text(encoding='utf-8'))}
    derived, result = deepcopy(snapshot), deepcopy(contract)
    layout_id = effective['scene_id']
    layout_fingerprint = feasibility_digest(source)
    solids = trailer_solids(effective['trailer'])
    environment = {'trailer': effective['trailer'], 'solids': solids,
                   'visual_assets': assets, 'qualification': effective['qualification'],
                   'conveyors': effective['conveyors']}
    derived.update(layout_id=layout_id, layout_fingerprint=layout_fingerprint,
                   trailer=effective['trailer'], environment=environment, derivation=source)
    robot = derived['robot']
    xyz = [a + b for a, b in zip(assembly_xyz, effective['robot_base_xyz_A_m'])]
    robot['world_from_mount'] = [[1., 0., 0., xyz[0]], [0., 1., 0., xyz[1]], [0., 0., 1., xyz[2]], [0., 0., 0., 1.]]
    robot['q_rad'][0] = spec.nominal_q1_rad
    robot['urdf'] = deepcopy(config['layout_bundle']['official_robot_asset'])
    # These were evaluated at a different historical q/base; they are not
    # effective collision or FK evidence for the imported official model.
    for key in ('flange_pose_world', 'tcp_pose_world', 'link_collision_obbs', 'mounting_reference'):
        robot.pop(key, None)
    derived['initial_state_audit'] = {'status': 'NOT_EXECUTION_QUALIFIED_REQUIRES_NEW_STATIC_CHECKS'}
    derived['evidence'] = {'parent_scene_fingerprint': snapshot['scene_fingerprint'], 'historical_evidence_inherited': False}
    derived.pop('scene_fingerprint')
    derived['scene_fingerprint'] = feasibility_digest(derived)
    result.update(layout_id=layout_id, layout_fingerprint=layout_fingerprint,
                  scene_fingerprint=derived['scene_fingerprint'], trailer=effective['trailer'],
                  environment=environment, robot=deepcopy(robot), derivation=source,
                  scope='PERCEPTION_DERIVED_STATIC_SCENE')
    result['primitives'].extend(solids)
    result['claims'] = {'geometry_source': config['effective_scene_config'], 'carton_count': len(derived['cartons']),
                        'physical_grasp': 'NOT_EVALUATED', 'payload_dynamics': 'NOT_EVALUATED',
                        'complete_workcell_clearance': 'NOT_EVALUATED', 'receiver_transport': 'STATIC_ONLY',
                        'other_branch_execution_certification': False}
    result.pop('contract_fingerprint')
    result['contract_fingerprint'] = feasibility_digest(result)
    return derived, result
