"""Asset-bound simulation-only robot/vision-mast collision policy.

The exemptions are exact link pairs. Neither a name prefix nor an attachment's
role is sufficient to gain a permission: each item must resolve to the frozen
mast/camera/seat source inventory first.
"""
from __future__ import annotations

from copy import deepcopy
import math
import re

from ..moveit2_backend import digest

MAST_MODE = 'robot_body_vision_mast_exempt_v2'
EXTRACTION_CARTON_MODE = 'attached_target_remaining_original_cartons_during_extraction'
ROBOT_BODY_LINKS = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']
LEGACY_POLICY = dict(schema='eco65_pair_collision_policy_v1', collision_margin_m=.002,
    mesh_error_reserve_m=.0002, required_pair_clearance_m=.0024,
    self_collision_clearance_m=.0024, contact_tolerance_m=.0004,
    seal_edge_margin_m=.003, compliant_cup_neighbor_contact_mode='check',
    stack_contact_mode='named_support_top_only', stack_contact_stages=['extraction'])


def _source_member(row, scene):
    source = row['source']; ident = row['id']; role = row['role']
    mast = scene['mast']; camera = scene.get('permanent_camera') or {}
    if row['parent_link'] != 'link_1':
        raise ValueError('MAST_EXEMPTION_PARENT_CHANGED:'+ident)
    if role == 'permanent_mast':
        part_id = source.get('part_id')
        parts = {p['id']: p for p in mast['parts']}
        if part_id not in parts or part_id in camera.get('replaced_mast_parts', []):
            raise ValueError('MAST_EXEMPTION_SOURCE_NOT_IN_ASSEMBLY:'+ident)
        part = parts[part_id]
        valid = any(source.get('mesh') == part.get(k) and
                    source.get('sha256') == part.get(k+'_sha256') for k in ('visual', 'collision'))
        expected = 'eco65_mast_'+part_id
        if 'component' in source:
            expected += '__convex_'+format(source['component'], '03d')
        if not valid or ident != expected:
            raise ValueError('MAST_EXEMPTION_SOURCE_CHANGED:'+ident)
    elif role == 'permanent_mounting_seat':
        seat = mast['mount_interface']['seat']
        if (source.get('mesh') != seat['mesh'] or source.get('sha256') != seat['sha256']
                or not re.fullmatch(r'eco65_mast_mounting_seat(?:__convex_[0-9]{3})?', ident)):
            raise ValueError('MAST_EXEMPTION_SEAT_CHANGED:'+ident)
    elif role == 'permanent_camera':
        parts = {p['id']: p for p in camera.get('parts', [])}
        part = parts.get(source.get('part_id'))
        if (part is None or ident != 'eco65_camera_'+part['id']
                or source.get('mesh') != part['collision']
                or source.get('sha256') != part['collision_sha256']):
            raise ValueError('MAST_EXEMPTION_CAMERA_CHANGED:'+ident)
    elif role == 'permanent_camera_adapter':
        parts = {p['name']: p for p in camera.get('pitch_adapter', [])}
        definition = source.get('definition', {})
        if (definition.get('name') not in parts or definition != parts[definition['name']]
                or source.get('camera_mount') != camera.get('mount')
                or ident != 'eco65_camera_'+definition['name']):
            raise ValueError('MAST_EXEMPTION_CAMERA_ADAPTER_CHANGED:'+ident)
    else:
        raise ValueError('MAST_EXEMPTION_UNKNOWN_ASSEMBLY_MEMBER:'+ident)
    if not row.get('mesh_sha256') or not row.get('mesh'):
        raise ValueError('MAST_EXEMPTION_COLLIDER_IDENTITY_REQUIRED:'+ident)
    return dict(id=ident, parent_link=row['parent_link'], role=role,
                source_fingerprint=digest(source), mesh_sha256=row['mesh_sha256'])


def mast_exemption(scene):
    rows = scene.get('moving_attachments', [])
    if not rows or scene.get('mast', {}).get('attachment') != 'link_1':
        raise ValueError('MAST_EXEMPTION_FROZEN_ASSEMBLY_REQUIRED')
    members = sorted((_source_member(row, scene) for row in rows), key=lambda r: r['id'])
    names = [r['id'] for r in members]
    if len(set(names)) != len(names) or set(names) & set(ROBOT_BODY_LINKS+['robot_mount_plate']):
        raise ValueError('MAST_EXEMPTION_DUPLICATE_OR_ROBOT_PLATFORM')
    return dict(schema='eco65_robot_vision_mast_exemption_v1',
        scope='ROBOT_BODY_VS_ENUMERATED_VISION_MAST_ASSEMBLY_ONLY',
        authorization='2026-10-20_PLANNING_ONLY_USER_SIMULATION_ASSUMPTION',
        robot_body_links=list(ROBOT_BODY_LINKS), assembly_links=names, assembly_members=members,
        exempt_pairs=[[robot, member] for robot in ROBOT_BODY_LINKS for member in names],
        retained_checks=['robot_mount_plate', 'tool_vs_mast', 'cups_vs_mast', 'payload_vs_mast',
                         'robot_self_nonexempt', 'robot_vs_environment'],
        geometry_retained=True, physical_execution='NOT_RUN', execution_ready=False)


def extraction_carton_exemption(scene):
    """Bind the new assumption to this coupled scene's twelve actual cartons."""
    spec = scene.get('coupled_conveyor_spec', {})
    if (spec.get('schema') != 'eco65_coupled_spec_v1'
            or spec.get('config', {}).get('extraction_carton_contact_mode') != EXTRACTION_CARTON_MODE
            or scene.get('extraction_carton_contact_mode') != EXTRACTION_CARTON_MODE):
        raise ValueError('EXTRACTION_CARTON_CONTACT_REQUIRES_COUPLED_SCENE')
    boxes = scene.get('boxes', [])
    expected = {f'carton_r00_l{layer:02d}_c{column:02d}' for layer in range(4) for column in range(3)}
    names = [box.get('id') for box in boxes]
    if (len(names) != 12 or len(set(names)) != 12 or set(names) != expected
            or scene.get('target_box_id') not in expected
            or any(box.get('size_m') != [.15, .1, .1]
                   or len(box.get('pose', [])) != 3
                   or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in box['pose'])
                   for box in boxes)):
        raise ValueError('EXTRACTION_CARTON_FROZEN_INVENTORY_INVALID')
    frozen = sorted(({key: box[key] for key in ('id', 'pose', 'size_m')} for box in boxes),
                    key=lambda box: box['id'])
    return dict(schema='eco65_attached_target_stack_exemption_v1',
        continuous_task_mode='eco65_coupled_cycle_v1', stage='extraction',
        scope='ATTACHED_TARGET_VS_REMAINING_ORIGINAL_CARTONS_ONLY',
        authorization='2026-10-20_USER_AUTHORIZED_KINEMATIC_DEPALLETIZING_CONTACT',
        original_carton_ids=sorted(expected), original_carton_inventory_fingerprint=digest(frozen),
        allow_intersection=True, geometry_retained=True,
        retained_checks=['robot_vs_cartons', 'tool_and_all_cups_vs_cartons',
                         'payload_vs_mast', 'payload_vs_conveyors_and_other_environment',
                         'non_extraction_carton_pairs', 'released_world_carton_checks'],
        physical_execution='NOT_RUN', execution_ready=False)


def extraction_carton_pairs(policy, *, continuous_task_mode, stage, target_id,
                            attached_target_id, world_carton_ids, completed_ids=()):
    """Exact active pairs for display/diagnostics; native derives the same set."""
    rule = policy.get('extraction_carton_contact')
    if (rule is None or continuous_task_mode != 'eco65_coupled_cycle_v1'
            or stage != 'extraction' or attached_target_id is None):
        return []
    if (policy.get('schema') != 'eco65_pair_collision_policy_v3'
            or rule.get('schema') != 'eco65_attached_target_stack_exemption_v1'
            or rule.get('continuous_task_mode') != continuous_task_mode
            or rule.get('stage') != stage or rule.get('allow_intersection') is not True
            or rule.get('scope') != 'ATTACHED_TARGET_VS_REMAINING_ORIGINAL_CARTONS_ONLY'):
        raise ValueError('EXTRACTION_CARTON_POLICY_BINDING_INVALID')
    frozen = set(rule['original_carton_ids'])
    world, completed = set(world_carton_ids), set(completed_ids)
    if (target_id != attached_target_id or target_id not in frozen or target_id in completed
            or target_id in world or not world <= frozen or not completed <= frozen
            or world & completed or world | completed | {target_id} != frozen
            or len(world) != len(world_carton_ids) or len(completed) != len(completed_ids)):
        raise ValueError('EXTRACTION_CARTON_RUNTIME_OWNERSHIP_INVALID')
    return [[target_id, other] for other in sorted(world)]


def effective_collision_policy(scene):
    policy = deepcopy(LEGACY_POLICY)
    mode = scene.get('collision_policy_mode')
    if mode is None:
        if scene.get('extraction_carton_contact_mode') is not None:
            raise ValueError('EXTRACTION_CARTON_CONTACT_REQUIRES_COUPLED_SCENE')
        return policy
    if mode != MAST_MODE:
        raise ValueError('UNKNOWN_ECO65_COLLISION_POLICY_MODE')
    policy.update(schema='eco65_pair_collision_policy_v2', mode=mode,
                  robot_mast_exemption=mast_exemption(scene))
    extraction_mode = scene.get('extraction_carton_contact_mode')
    if extraction_mode is not None:
        if extraction_mode != EXTRACTION_CARTON_MODE:
            raise ValueError('UNKNOWN_ECO65_EXTRACTION_CARTON_CONTACT_MODE')
        policy.update(schema='eco65_pair_collision_policy_v3',
                      stack_contact_mode=EXTRACTION_CARTON_MODE,
                      extraction_carton_contact=extraction_carton_exemption(scene))
    return policy
