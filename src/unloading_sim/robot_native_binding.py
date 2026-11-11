"""Build a native model from a verified binding, with no robot-name fallback.

The native process adapter currently requires a link named ``flange``. When a
source model uses a different name, this builder adds a collisionless frame at
exactly that physical flange. It does not add a physical adapter or change TCP.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np

from .robot_model_binding import ModelNotReady, rigid_transform


def _rpy(rotation):
    r = np.asarray(rotation)
    pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
    if abs(np.cos(pitch)) < 1e-10:
        return [0., float(pitch), float(np.arctan2(-r[0, 1], r[1, 1]))]
    return [float(np.arctan2(r[2, 1], r[2, 2])), float(pitch), float(np.arctan2(r[1, 0], r[0, 0]))]


def _set_origin(joint, transform):
    origin = joint.find('origin')
    if origin is None:
        origin = ET.SubElement(joint, 'origin')
    origin.set('xyz', ' '.join(map(str, transform[:3, 3])))
    origin.set('rpy', ' '.join(map(str, _rpy(transform[:3, :3]))))


def native_model_from_binding(binding, world_from_base, *, project_root, asset_root=None):
    """Return parsed URDF/SRDF and explicit role mapping for native transport.

    Only a root base or an existing fixed world-to-base joint is supported.
    This avoids guessing placement through an unknown upstream model frame.
    Mesh sources are validated locally before optional server-path relocation.
    """
    binding.verify_source_unchanged()
    mount = rigid_transform(world_from_base, 'world_from_base')
    urdf = ET.parse(binding.urdf_path).getroot()
    srdf = ET.parse(binding.srdf_path).getroot()
    links = {element.get('name') for element in urdf.findall('link')}
    joints = urdf.findall('joint')
    if len(links) != len(urdf.findall('link')):
        raise ModelNotReady(binding.model_id, ['DUPLICATE_URDF_LINK'])
    parent_joints = [joint for joint in joints if joint.find('child').get('link') == binding.base_link]
    if len(parent_joints) > 1:
        raise ModelNotReady(binding.model_id, ['MULTIPLE_BASE_PARENTS'])
    if parent_joints:
        world_joint = parent_joints[0]
        if world_joint.get('type') != 'fixed' or world_joint.find('parent').get('link') != 'world':
            raise ModelNotReady(binding.model_id, ['NATIVE_BASE_PARENT_MUST_BE_FIXED_WORLD'])
    else:
        if 'world' in links:
            raise ModelNotReady(binding.model_id, ['UNCONNECTED_EXISTING_WORLD_LINK'])
        ET.SubElement(urdf, 'link', name='world')
        world_joint = ET.SubElement(urdf, 'joint', name='configured_world_mount', type='fixed')
        ET.SubElement(world_joint, 'parent', link='world')
        ET.SubElement(world_joint, 'child', link=binding.base_link)
    _set_origin(world_joint, mount)
    active_names = {joint.get('name') for joint in joints if joint.get('type') != 'fixed'}
    if active_names != set(binding.active_joint_names):
        raise ModelNotReady(binding.model_id, ['NATIVE_MODEL_CONTAINS_UNBOUND_MOVING_JOINTS'])
    effort_placeholders = []
    for joint in joints:
        if joint.get('name') not in binding.active_joint_names:
            continue
        limit = joint.find('limit')
        if limit is None:
            raise ModelNotReady(binding.model_id, ['NATIVE_JOINT_LIMIT_MISSING:'+joint.get('name')])
        effort = limit.get('effort')
        if effort is None:
            # urdfdom requires this XML attribute even for geometric planning.
            # Zero denotes no qualified actuator capacity; it is NOT a vendor
            # torque limit. Only this transient XML tree is changed.
            limit.set('effort', '0')
            effort_placeholders.append(joint.get('name'))
        else:
            try:
                value = float(effort)
            except ValueError as exc:
                raise ModelNotReady(binding.model_id, ['NATIVE_EFFORT_VALUE_INVALID:'+joint.get('name')]) from exc
            if not np.isfinite(value) or value < 0:
                raise ModelNotReady(binding.model_id, ['NATIVE_EFFORT_VALUE_INVALID:'+joint.get('name')])
    if binding.flange_link != 'flange':
        if 'flange' in links:
            raise ModelNotReady(binding.model_id, ['NATIVE_FLANGE_ALIAS_NAME_CONFLICT'])
        ET.SubElement(urdf, 'link', name='flange')
        alias = ET.SubElement(urdf, 'joint', name='configured_physical_flange_alias', type='fixed')
        ET.SubElement(alias, 'parent', link=binding.flange_link)
        ET.SubElement(alias, 'child', link='flange')
        _set_origin(alias, np.eye(4))
    root = Path(project_root).resolve()
    for mesh in urdf.iter('mesh'):
        path = binding.resolve_mesh(mesh.get('filename', ''))
        if asset_root is not None:
            if not path.is_relative_to(root):
                raise ModelNotReady(binding.model_id, ['MESH_OUTSIDE_RELOCATABLE_PROJECT_ROOT'])
            path = Path(asset_root) / path.relative_to(root)
        mesh.set('filename', 'file://'+str(path).replace('\\', '/'))
    source_placeholders = (list(binding.active_joint_names)
        if (binding.data.get('preview_only') or {}).get('urdf_effort_is_placeholder') is True else [])
    unknown_effort = sorted(set(effort_placeholders) | set(source_placeholders))
    return urdf, srdf, dict(native_flange_link='flange', source_flange_link=binding.flange_link,
        source_flange_from_native_flange=np.eye(4).tolist(), physical_adapter_added=False,
        urdf_parser_compatibility=dict(effort_placeholder_joint_names=effort_placeholders,
            source_declared_effort_placeholder_joint_names=source_placeholders,
            placeholder_effort_nm=0., physical_effort_unknown_joint_names=unknown_effort,
            purpose='URDF_PARSER_ONLY_FOR_KINEMATICS_AND_COLLISION', source_urdf_modified=False,
            known_effort_values_preserved=True, dynamics_qualified=False))


def configured_wrist_tool_links(binding):
    """Validate reviewed physical wrist ownership, excluding the proximal arm."""
    names = (binding.data.get('collision') or {}).get('owned_tool_exempt_robot_links')
    if names is None:
        raise ModelNotReady(binding.model_id, ['collision.owned_tool_exempt_robot_links'])
    if not isinstance(names, list) or len(names) != len(set(names)):
        raise ModelNotReady(binding.model_id, ['INVALID_OWNED_TOOL_EXEMPT_LINKS'])
    links = {link.get('name') for link in ET.parse(binding.urdf_path).getroot().findall('link')}
    if not set(names) <= links:
        raise ModelNotReady(binding.model_id, ['UNKNOWN_OWNED_TOOL_EXEMPT_LINK'])
    if names and not (binding.data.get('collision') or {}).get('owned_tool_exemptions_source'):
        raise ModelNotReady(binding.model_id, ['collision.owned_tool_exemptions_source'])
    from .robot import URDFRobot
    robot = URDFRobot.from_urdf(binding.urdf_path, binding.active_joint_names,
        binding.base_link, binding.tip_link, tool_length=0.)
    distal = set(); active_index = 0
    for joint in robot.joints:
        if joint.joint_type != 'fixed': active_index += 1
        if active_index >= 4: distal.add(joint.child)
    if not set(names) <= distal:
        raise ModelNotReady(binding.model_id, ['OWNED_TOOL_EXEMPTION_MUST_NOT_INCLUDE_PROXIMAL_ARM'])
    return tuple(names)


def configured_wrist_tool_pairs(binding, policy, owned_tool_names):
    """Require explicit physical ownership; never copy FANUC wrist names."""
    names = configured_wrist_tool_links(binding)
    scoped_model = getattr(policy, 'robot_binding_model_id', '')
    if scoped_model or (names and binding.model_id != 'fanuc_m710id_70'):
        if (scoped_model != binding.model_id
                or getattr(policy, 'robot_binding_urdf_sha256', '') != binding.identity()['urdf_sha256']):
            raise ModelNotReady(binding.model_id, ['COLLISION_POLICY_ROBOT_BINDING_MISMATCH'])
    expected = {(link, tool) for link in names for tool in owned_tool_names}
    if set(policy.wrist_tool_pairs(owned_tool_names)) != expected:
        raise ModelNotReady(binding.model_id, ['COLLISION_POLICY_MODEL_WRIST_OWNERSHIP_MISMATCH'])
    return expected


def physical_flange_frame(robot, q):
    binding = getattr(robot, 'model_binding', None)
    return robot.named_link_frames(q)[binding.flange_link if binding is not None else 'flange']


def native_fk_model_request(binding, *, world_from_base=None, project_root, asset_root=None, seed=50):
    """A bare-model native FK protocol; cannot authorize IK or any motion.

    In a CAD preview the caller's base is a declared simulation coordinate
    convention, never a verified installation transform or controller zero.
    """
    world_from_base = np.eye(4) if world_from_base is None else world_from_base
    urdf, srdf, roles = native_model_from_binding(binding, world_from_base,
        project_root=project_root, asset_root=asset_root)
    params = dict(robot_description=ET.tostring(urdf, encoding='unicode'),
                  robot_description_semantic=ET.tostring(srdf, encoding='unicode'))
    identity = dict(schema='configured_robot_fk_only_v1', model_usage='cad_preview_fk_only',
        robot_model_binding=binding.identity(), native_frame_roles=roles,
        world_from_base=rigid_transform(world_from_base, 'world_from_base').tolist(),
        model_tool_fingerprint=hashlib.sha256(json.dumps(params, sort_keys=True).encode()).hexdigest(),
        controller_coordinate_verified=False, installation_alignment_verified=False,
        tool_installed=False, motion_generation_allowed=False, dynamics_qualified=False)
    return dict(op='init', parameters=params, identity=identity,
        joint_names=list(binding.active_joint_names), seed=int(seed), model_usage='cad_preview_fk_only',
        planning_only=True, planning_mode='cold_from_scratch', collision_policy={}, tool_links=[],
        compliant_tool_links=[], process_geometry=None,
        expected_collision_shapes={link.get('name'): len(link.findall('collision'))
            for link in urdf.findall('link') if link.findall('collision')})


def configured_native_timing(binding):
    """Explicit finite timing only; no inherited FANUC acceleration defaults."""
    from .s50_qualification import timing_audit
    audit = timing_audit(binding.data)
    if audit['ready'] is not True:
        raise ModelNotReady(binding.model_id, audit['missing'])
    assumption = binding.data.get('time_parameterization_assumption') or {}
    acceleration = assumption.get('acceleration_rad_s2', binding.data.get('joint_acceleration_limits_rad_s2'))
    velocity = assumption.get('velocity_rad_s', binding.joint_velocity_limits)
    def values(raw, key):
        if isinstance(raw, dict): raw = [raw.get(name) for name in binding.active_joint_names]
        array = np.asarray(raw, dtype=float)
        if array.shape != (6,) or not np.isfinite(array).all() or np.any(array <= 0):
            raise ModelNotReady(binding.model_id, ['timing.INVALID_'+key])
        return array
    velocity = np.minimum(values(velocity,'velocity_rad_s'), binding.joint_velocity_limits)
    acceleration = values(acceleration,'acceleration_rad_s2')
    return dict(audit=audit, velocity_rad_s=velocity.tolist(), acceleration_rad_s2=acceleration.tolist(),
        effective_velocity_rule='MIN_OF_EXPLICIT_TIMING_AND_BOUND_MODEL_VELOCITY', dynamics_qualified=False)
