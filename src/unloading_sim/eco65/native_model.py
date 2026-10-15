"""Pinned ECO65-B derived MoveIt description; no vendor hardware interface."""
from __future__ import annotations

import hashlib
import itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from .model import ROOT, check_transform, load_json, transform, mounting_pairs
from .asset_paths import resolve_asset, resource_uri
from .native_contact import extract_cad_contours
from ..moveit2_backend import digest, _rpy

from .asset_inventory import CONFIG, load_robot_config, asset_inventory
POLICY = dict(schema='eco65_pair_collision_policy_v1', collision_margin_m=.002,
              mesh_error_reserve_m=.0002, required_pair_clearance_m=.0024,
              self_collision_clearance_m=.0024, contact_tolerance_m=.0004,
              seal_edge_margin_m=.003, compliant_cup_neighbor_contact_mode='check',
              stack_contact_mode='named_support_top_only', stack_contact_stages=['extraction'])


def build_native_model(scene, output, *, root=ROOT, config=None, native_asset_root=None):
    import trimesh
    root = Path(root); output = Path(output).resolve(); output.relative_to(root/'outputs')
    c = config or load_robot_config(); audit = asset_inventory(root, c)
    if audit['status'] != 'PASS': raise ValueError('BLOCKED_ASSETS:'+str(audit['failures']))
    tool = load_json(root/c['tool_config'])
    cad_to_tcp = check_transform(tool['T_tool_cad_tcp'])
    flange_to_cad = check_transform(tool['T_flange_tool_cad'])
    tcp = check_transform(tool['T_flange_tcp'])
    if not np.allclose(flange_to_cad@cad_to_tcp, tcp, atol=1e-10, rtol=0):
        raise ValueError('TOOL_TRANSFORM_CHAIN_MISMATCH')
    geometry = extract_cad_contours(tool, root)
    if not mounting_pairs(tool): raise ValueError('ASSET_BOUND_MOUNT_CONTACT_REQUIRED')
    contact_to_tcp = np.linalg.inv(cad_to_tcp)
    for row in geometry['contours']:
        points = np.array(row['points_m'])@contact_to_tcp[:3, :3].T+contact_to_tcp[:3, 3]
        normal = contact_to_tcp[:3, :3]@row['normal']
        if np.max(np.abs(points[:, 2])) > 1e-7 or not np.allclose(normal, [0, 0, 1], atol=1e-8, rtol=0):
            raise ValueError('CAD_CONTACT_PLANE_TCP_MISMATCH')
    urdf = ET.parse(root/c['urdf']).getroot()
    movable = [j for j in urdf.findall('joint') if j.get('type') != 'fixed']
    if [j.get('name') for j in movable] != c['joint_names']:
        raise ValueError('OFFICIAL_JOINT_ORDER_MISMATCH')
    official_limits = {j.get('name'): dict(j.find('limit').attrib) for j in movable}
    for row in official_limits.values():
        values = [float(row[k]) for k in ('lower', 'upper', 'velocity', 'effort')]
        if not np.isfinite(values).all() or values[0] >= values[1] or min(values[2:]) <= 0:
            raise ValueError('OFFICIAL_FINITE_LIMITS_REQUIRED')
    output.mkdir(parents=True, exist_ok=True)
    meshes = output/'collision'; meshes.mkdir(exist_ok=True)
    bindings = []

    def convex(source, name):
        m = trimesh.load(source, force='mesh', process=True)
        if not len(m.faces) or not np.isfinite(m.vertices).all(): raise ValueError('EMPTY_OR_INVALID_MESH')
        hull = m.convex_hull
        # Both consumers now use this explicit convex boundary. MuJoCo's
        # implicit convexification of a concave source is never compared with
        # unmodified triangle-soup FCL solely by filename.
        p = meshes/(name+'.obj'); hull.export(p)
        bindings.append(dict(id=name, source=Path(source).relative_to(root).as_posix(),
            source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),
            convex_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
            convex_path=p.relative_to(root).as_posix(),
            representation='explicit_convex_hull_of_source_triangle_vertices',
            source_vertices=len(m.vertices), hull_vertices=len(hull.vertices)))
        return resource_uri(root, p.relative_to(root).as_posix(), native_asset_root)

    def fixed(name, parent, T):
        ET.SubElement(urdf, 'link', name=name)
        j = ET.SubElement(urdf, 'joint', name='mount_'+name, type='fixed')
        ET.SubElement(j, 'parent', link=parent); ET.SubElement(j, 'child', link=name)
        ET.SubElement(j, 'origin', xyz=' '.join(map(str, T[:3, 3])), rpy=' '.join(map(str, _rpy(T[:3, :3]))))
        return urdf.find(f"link[@name='{name}']")

    if urdf.find("link[@name='world']") is not None: raise ValueError('UNEXPECTED_OFFICIAL_WORLD_FRAME')
    ET.SubElement(urdf, 'link', name='world')
    mount = ET.SubElement(urdf, 'joint', name='eco65_fixed_installation', type='fixed')
    ET.SubElement(mount, 'parent', link='world'); ET.SubElement(mount, 'child', link=c['base_link'])
    ET.SubElement(mount, 'origin', xyz=' '.join(map(str, scene['base_position_m'])), rpy='0 0 0')
    original_links = [l for l in urdf.findall('link') if l.get('name') != 'world']
    for link in original_links:
        # Source independent geometry uses the pinned visual mesh convex hull.
        visual = link.find('visual')
        if visual is None: raise ValueError('OFFICIAL_LINK_VISUAL_MISSING')
        mesh = visual.find('geometry/mesh'); uri = mesh.get('filename')
        if not np.array_equal(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '), [1, 1, 1]):
            raise ValueError('OFFICIAL_MESH_SCALE_CHANGED')
        if not uri.startswith('package://ECO65-B/'): raise ValueError('OFFICIAL_PACKAGE_MISMATCH')
        source = root/c['package_root']/uri.removeprefix('package://ECO65-B/')
        for old in list(link.findall('collision')): link.remove(old)
        collision = ET.SubElement(link, 'collision')
        origin = visual.find('origin')
        if origin is not None: ET.SubElement(collision, 'origin', **origin.attrib)
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh',
                      filename=convex(source, 'robot_'+link.get('name')))
        mesh.set('filename', resource_uri(root, source.relative_to(root).as_posix(), native_asset_root))
    names, bands = [], []
    for row in tool['collisions']:
        name = row['id']; names.append(name)
        if row['semantic'] == 'terminal_contact_band': bands.append(name)
        elif row['semantic'] != 'conservative_rigid': raise ValueError('UNKNOWN_TOOL_COLLISION_SEMANTIC')
        link = fixed(name, c['flange_link'], flange_to_cad)
        ET.SubElement(ET.SubElement(ET.SubElement(link, 'collision'), 'geometry'), 'mesh',
                      filename=convex(resolve_asset(root, row['mesh']), name))
    for row in tool['adapter']:
        name = row['id']; names.append(name)
        link = fixed(name, c['flange_link'], transform(row['position']))
        g = ET.SubElement(ET.SubElement(link, 'collision'), 'geometry')
        if row['kind'] == 'box': ET.SubElement(g, 'box', size=' '.join(map(str, row['size'])))
        elif row['kind'] == 'cylinder': ET.SubElement(g, 'cylinder', radius=str(row['radius']), length=str(2*row['half_length']))
        else: raise ValueError('ADAPTER_GEOMETRY_UNSUPPORTED')
    if len(bands) != 12: raise ValueError('TWELVE_CONTACT_BANDS_REQUIRED')
    fixed(c['task_tcp_link'], c['flange_link'], tcp)
    srdf = ET.Element('robot', name=urdf.get('name'))
    group = ET.SubElement(srdf, 'group', name=c['planning_group'])
    ET.SubElement(group, 'chain', base_link=c['base_link'], tip_link=c['task_tcp_link'])
    for joint in movable:
        ET.SubElement(srdf, 'disable_collisions', link1=joint.find('parent').get('link'),
                      link2=joint.find('child').get('link'), reason='Adjacent_official_chain')
    for a, b in itertools.combinations(names, 2):
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='Owned_tool_assembly')
    # Named flange/mount contact is checked geometrically by Eco65Process.
    params = {'robot_description': ET.tostring(urdf, encoding='unicode'),
              'robot_description_semantic': ET.tostring(srdf, encoding='unicode')}
    pg = c['planning_group']; limits = c['simulation_limits']
    params.update({f'robot_description_kinematics.{pg}.kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        f'robot_description_kinematics.{pg}.kinematics_solver_timeout': .1,
        'pilz_industrial_motion_planner.planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
        'pilz_industrial_motion_planner.request_adapters': '', 'ompl.planning_plugin': 'ompl_interface/OMPLPlanner',
        'ompl.request_adapters': '', 'ompl.planner_configs.RRTConnectkConfigDefault.type': 'geometric::RRTConnect',
        'ompl.planner_configs.RRTConnectkConfigDefault.range': .1,
        f'ompl.{pg}.planner_configs': ['RRTConnectkConfigDefault'], f'ompl.{pg}.longest_valid_segment_fraction': .001,
        'robot_description_planning.cartesian_limits.max_trans_vel': limits['tcp_velocity_m_s'],
        'robot_description_planning.cartesian_limits.max_trans_acc': limits['tcp_acceleration_m_s2'],
        'robot_description_planning.cartesian_limits.max_trans_dec': -limits['tcp_acceleration_m_s2'],
        'robot_description_planning.cartesian_limits.max_rot_vel': limits['tcp_rotation_rad_s']})
    for joint in c['joint_names']:
        prefix = f'robot_description_planning.joint_limits.{joint}.'
        params.update({prefix+'has_velocity_limits': True,
                       prefix+'max_velocity': min(float(official_limits[joint]['velocity']), limits['joint_velocity_rad_s']),
                       prefix+'has_acceleration_limits': True, prefix+'max_acceleration': limits['joint_acceleration_rad_s2']})
    identity = {k: c[k] for k in ('robot_id', 'robot_variant', 'planning_group', 'base_link', 'flange_link', 'task_tcp_link')}
    identity.update(schema='eco65_native_model_v1', model_tool_fingerprint=digest(params),
        scene_fingerprint=scene['scene_fingerprint'], asset_fingerprint=digest(audit['asset_sha256']),
        policy_fingerprint=digest(POLICY), flange_from_task_tcp=tcp.tolist(),
        task_tcp_fingerprint=digest(tcp.tolist()), contact_geometry_fingerprint=digest(geometry),
        policy_scope='PLANNING_ONLY_NOT_EXECUTABLE')
    init = dict(op='init', parameters=params, identity=identity, joint_names=c['joint_names'], seed=6518,
        collision_policy=POLICY, tool_links=sorted(names), compliant_tool_links=sorted(bands),
        process_geometry=geometry, planning_only=True, planning_mode='cold_from_scratch',
        expected_collision_shapes={l.get('name'): len(l.findall('collision')) for l in urdf.findall('link') if l.findall('collision')})
    return dict(init=init, tool=tool, asset_audit=audit, convex_bindings=bindings,
                official_limits=official_limits, simulation_limits=limits,
                hardware_capability='NOT_EVALUATED')
