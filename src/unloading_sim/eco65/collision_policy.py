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
CUP_NEIGHBOR_MODE = 'named_terminal_bands_remaining_original_cartons'
ROBOT_TOOL_MODE = 'explicit_link_3_part_026_simulation_exemption_v1'
RIGID_CARTON_MODE = 'enumerated_rigid_tool_original_cartons_v1'
RIGID_TOOL_LINKS = sorted([f'part_{i:03d}'+('_rigid' if i in range(2,25,2) else '')
                          for i in range(43) if i!=41]+
                         [f'part_041_region_{i}' for i in range(3)])
PART_026_SHA256 = 'e40dd92e8d06fba77b65e5b078ad7f0ab21bb9b1ad29e564ab4e3675be18e9f4'
CUP_LIP_LINKS = [f'part_{i:03d}_lip' for i in range(2, 25, 2)]
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
    if (policy.get('schema') not in ('eco65_pair_collision_policy_v3', 'eco65_pair_collision_policy_v4', 'eco65_pair_collision_policy_v5', 'eco65_pair_collision_policy_v6')
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


def cup_neighbor_exemption(scene):
    spec = scene.get('coupled_conveyor_spec', {})
    if (scene.get('cup_neighbor_contact_mode') != CUP_NEIGHBOR_MODE
            or spec.get('config', {}).get('cup_neighbor_contact_mode') != CUP_NEIGHBOR_MODE):
        raise ValueError('CUP_NEIGHBOR_CONTACT_REQUIRES_BOUND_COUPLED_SCENE')
    cartons = extraction_carton_exemption(scene)
    return dict(schema='eco65_cup_neighbor_contact_v1',
        continuous_task_mode='eco65_coupled_cycle_v1',
        scope='ENUMERATED_CUP_TERMINAL_BANDS_VS_REMAINING_NON_TARGET_ORIGINAL_CARTONS',
        authorization='2026-10-20_USER_AUTHORIZED_CUP_NEIGHBOR_CONTACT',
        terminal_band_links=list(CUP_LIP_LINKS),
        original_carton_ids=cartons['original_carton_ids'],
        original_carton_inventory_fingerprint=cartons['original_carton_inventory_fingerprint'],
        enabled_and_disabled_cups=True, geometry_retained=True, allow_intersection=True,
        retained_checks=['actual_target_seal_and_edge_alignment', 'cup_vs_current_target',
                         'rigid_tool_vs_all_cartons', 'robot_vs_cartons', 'cup_vs_other_environment',
                         'released_target_during_withdrawal'],
        physical_execution='NOT_RUN', execution_ready=False)


def cup_neighbor_pairs(policy, *, continuous_task_mode, target_id, world_carton_ids,
                       completed_ids=()):
    rule = policy.get('cup_neighbor_contact')
    if rule is None or continuous_task_mode != 'eco65_coupled_cycle_v1':
        return []
    if (policy.get('schema') not in ('eco65_pair_collision_policy_v4', 'eco65_pair_collision_policy_v5', 'eco65_pair_collision_policy_v6')
            or rule.get('schema') != 'eco65_cup_neighbor_contact_v1'
            or rule.get('terminal_band_links') != CUP_LIP_LINKS):
        raise ValueError('CUP_NEIGHBOR_POLICY_BINDING_INVALID')
    frozen, world, completed = (set(rule['original_carton_ids']),
                                set(world_carton_ids), set(completed_ids))
    if (target_id not in frozen or not world <= frozen or not completed <= frozen
            or world & completed or len(world) != len(world_carton_ids)):
        raise ValueError('CUP_NEIGHBOR_RUNTIME_OWNERSHIP_INVALID')
    return [[cup, box] for cup in rule['terminal_band_links']
            for box in sorted(world-completed-{target_id})]


def robot_tool_exemption():
    """One user-authorized rigid pair; no robot/tool category-wide permission."""
    return dict(schema='eco65_named_robot_tool_exemption_v1',
        scope='EXACT_LINK_3_VS_PART_026_ONLY',
        authorization='2026-10-22_USER_AUTHORIZED_LINK_3_PART_026_SIMULATION_CONTACT',
        exempt_pairs=[['link_3', 'part_026']], tool_semantic='conservative_rigid',
        tool_mesh_sha256=PART_026_SHA256, geometry_retained=True,
        allow_intersection=True, physical_execution='NOT_RUN', execution_ready=False,
        retained_checks=['all_other_robot_tool_pairs', 'robot_self_nonexempt',
                         'tool_vs_environment', 'robot_vs_environment', 'payload_checks'])


def rigid_tool_inventory(tool):
    rows = sorted((dict(id=p['id'], semantic=p['semantic'], mesh_sha256=p['mesh_sha256'])
                   for p in tool['collisions'] if p['semantic']=='conservative_rigid'), key=lambda p:p['id'])
    if [p['id'] for p in rows] != RIGID_TOOL_LINKS:
        raise ValueError('RIGID_TOOL_CARTON_SOURCE_INVENTORY_CHANGED')
    return rows


def rigid_tool_carton_exemption(scene):
    rows = scene.get('rigid_tool_carton_inventory', [])
    if (scene.get('rigid_tool_carton_contact_mode') != RIGID_CARTON_MODE or
            [p.get('id') for p in rows] != RIGID_TOOL_LINKS or
            any(p.get('semantic')!='conservative_rigid' or
                not re.fullmatch('[0-9a-f]{64}',p.get('mesh_sha256','')) for p in rows)):
        raise ValueError('RIGID_TOOL_CARTON_EXACT_INVENTORY_REQUIRED')
    original=extraction_carton_exemption(scene)
    return dict(schema='eco65_rigid_tool_carton_contact_v1',
        scope='ENUMERATED_RIGID_TOOL_VS_ORIGINAL_CARTONS_WORLD_OR_ATTACHED',
        continuous_task_mode='eco65_coupled_cycle_v1',
        authorization='2026-10-22_USER_AUTHORIZED_RIGID_TOOL_TARGET_AND_NEIGHBOR_CARTON_CONTACT',
        rigid_tool_links=list(RIGID_TOOL_LINKS),rigid_tool_members=deepcopy(rows),
        original_carton_ids=original['original_carton_ids'],
        original_carton_inventory_fingerprint=original['original_carton_inventory_fingerprint'],
        include_current_target=True,include_attached_target=True,include_released_target=True,
        allow_intersection=True,geometry_retained=True,physical_execution='NOT_RUN',execution_ready=False,
        retained_checks=['actual_target_seal_and_edge_alignment','terminal_band_vs_current_target',
                         'robot_vs_cartons','tool_vs_non_carton_environment','payload_vs_environment',
                         'robot_tool_nonexempt','joint_limits_and_business_direction'])


def rigid_tool_carton_pairs(policy, *, continuous_task_mode, world_carton_ids,
                           attached_target_id=None, completed_ids=()):
    rule=policy.get('rigid_tool_carton_contact')
    if rule is None or continuous_task_mode!='eco65_coupled_cycle_v1':return []
    if (policy.get('schema')!='eco65_pair_collision_policy_v6' or
            rule.get('rigid_tool_links')!=RIGID_TOOL_LINKS):
        raise ValueError('RIGID_TOOL_CARTON_POLICY_BINDING_INVALID')
    original=set(rule['original_carton_ids']);world=set(world_carton_ids);done=set(completed_ids)
    attached=set() if attached_target_id is None else {attached_target_id}
    if (len(world)!=len(world_carton_ids) or len(done)!=len(completed_ids) or
            world & attached or (world|attached)&done or world|attached|done!=original):
        raise ValueError('RIGID_TOOL_CARTON_OWNERSHIP_INVALID')
    return [[tool,box] for tool in RIGID_TOOL_LINKS for box in sorted(world|attached)]


def effective_collision_policy(scene):
    policy = deepcopy(LEGACY_POLICY)
    mode = scene.get('collision_policy_mode')
    if mode is None:
        if scene.get('rigid_tool_carton_contact_mode') is not None:
            raise ValueError('RIGID_TOOL_CARTON_REQUIRES_BOUND_COUPLED_POLICY')
        if scene.get('robot_tool_exemption_mode') is not None:
            raise ValueError('ROBOT_TOOL_EXEMPTION_REQUIRES_BOUND_COUPLED_POLICY')
        if scene.get('extraction_carton_contact_mode') is not None or scene.get('cup_neighbor_contact_mode') is not None:
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
    if scene.get('cup_neighbor_contact_mode') is not None:
        policy.update(schema='eco65_pair_collision_policy_v4',
                      compliant_cup_neighbor_contact_mode=CUP_NEIGHBOR_MODE,
                      cup_neighbor_contact=cup_neighbor_exemption(scene))
    if scene.get('robot_tool_exemption_mode') is not None:
        if (scene['robot_tool_exemption_mode'] != ROBOT_TOOL_MODE or
                policy['schema'] != 'eco65_pair_collision_policy_v4'):
            raise ValueError('ROBOT_TOOL_EXEMPTION_EXACT_SCOPE_REQUIRED')
        policy.update(schema='eco65_pair_collision_policy_v5',
                      robot_tool_exemption=robot_tool_exemption())
    if scene.get('rigid_tool_carton_contact_mode') is not None:
        if policy['schema']!='eco65_pair_collision_policy_v5':
            raise ValueError('RIGID_TOOL_CARTON_REQUIRES_BOUND_COUPLED_POLICY')
        policy.update(schema='eco65_pair_collision_policy_v6',
                      rigid_tool_carton_contact=rigid_tool_carton_exemption(scene))
        policy['cup_neighbor_contact']['retained_checks'].remove('rigid_tool_vs_all_cartons')
        policy['extraction_carton_contact']['retained_checks'].remove('tool_and_all_cups_vs_cartons')
        policy['robot_tool_exemption']['retained_checks'].remove('tool_vs_environment')
        policy['robot_tool_exemption']['retained_checks'].append('tool_vs_non_carton_environment')
    return policy
