"""Fixed base/mast remount, shared by native FK, collision, USD and replay."""
from copy import deepcopy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from .model import ROOT, URDF, check_transform, transform
from .camera_installation import pole_axis, installation_fingerprint
from ..moveit2_backend import digest

CONFIG = ROOT/'configs/workcells/eco65_base_mast_remount_v2.yaml'


def remount_transforms(scene, config):
    """World yaw is compared at identical raw J1, not by editing joint angles."""
    if (config['schema'] != 'eco65_base_mast_remount_v1' or config['enable_hardware']
            or scene.get('layout_installation')):
        raise ValueError('LAYOUT_REMOUNT_CONFIG_OR_SOURCE_INVALID')
    old_rpy = np.asarray(scene.get('base_rpy_rad', [0., 0., 0.]), float)
    if not np.allclose(old_rpy[:2], 0.):
        raise ValueError('LAYOUT_REMOUNT_REQUIRES_VERTICAL_J1')
    old_base = Rotation.from_euler('xyz', old_rpy).as_matrix()
    new_base = Rotation.from_euler('z', config['robot_base_yaw_deg'], degrees=True).as_matrix()
    world_delta = Rotation.from_euler('z', config['mast_world_yaw_delta_deg'], degrees=True).as_matrix()
    mast = scene['mast']; M = check_transform(mast['T_link1_mast'])
    # Rotate about the J1 line through the parent origin. Its vertical location
    # is invariant, so the existing installation height remains unchanged.
    L = transform(rotation=new_base.T@world_delta@old_base)
    new_M = L@M
    axis = pole_axis(mast); p = np.asarray(axis['point_mast_m'])
    R = new_M[:3,:3].T@new_base.T@old_base@M[:3,:3]
    A = transform(p-R@p, R)
    switch = new_base@np.asarray(config['switch_face_normal_base'])
    if not np.allclose(switch, config['switch_face_normal_D'], atol=1e-12):
        raise ValueError('SWITCH_FACE_NOT_MINUS_X')
    return dict(old_base=old_base, new_base=new_base, local_mast_delta=L,
                old_mast=M, new_mast=new_M, camera_counterrotation=A, axis=axis)


def apply_layout_installation(scene, config, output, root=ROOT):
    """Reorient unchanged supplier parts and regenerate only the derived seat."""
    from .native_io import file_hash
    import trimesh
    from eco65_mast_seating import conformal_seat, vertical_seating
    from build_eco65_native_12carton import _convex

    root = Path(root); out = Path(output); out.mkdir(parents=True, exist_ok=False)
    t = remount_transforms(scene, config)
    result = deepcopy(scene); mast = result['mast']; camera = result['permanent_camera']
    M, N, A = t['old_mast'], t['new_mast'], t['camera_counterrotation']
    camera_before = t['old_base']@M[:3,:3]@np.asarray(camera['T_mast_optical'])[:3,:3]
    result['base_rpy_rad'] = [0., 0., float(np.radians(config['robot_base_yaw_deg']))]
    mast['T_link1_mast'] = N.tolist()
    # Part 019 is the round pole with integral upper pocket. Turn it in its
    # circular lower clamp; keep the pocket, pitch adapters and camera rigid.
    # This preserves their actual connections without stretching the risers.
    moving = set(config['camera_counterrotation_part_ids'])
    if moving != {'mast_part_019'}:
        raise ValueError('LAYOUT_CAMERA_COUNTERROTATION_INVENTORY_INVALID')
    for part in mast['parts']:
        if part['id'] in moving:
            part['T_mast'] = (A@check_transform(part.get('T_mast', np.eye(4)))).tolist()
    for key in ('T_mast_CAD', 'T_mast_optical'):
        camera[key] = (A@check_transform(camera[key])).tolist()
    for row in camera['pitch_adapter']:
        row['T_mast'] = (A@check_transform(row['T_mast'])).tolist()
    mount = camera['mount']
    mount['pivot_mast_m'] = (A@np.r_[mount['pivot_mast_m'], 1.])[:3].tolist()
    mount['axis_mast'] = (A[:3,:3]@mount['axis_mast']).tolist()
    camera['T_link1_optical'] = (N@camera['T_mast_optical']).tolist()
    f = np.asarray(camera['T_link1_optical'])[:3,2]
    heading = float(np.degrees(np.arctan2(-f[1], -f[0])))
    mount['heading_from_link1_minus_X_deg'] = heading
    mast['T_mast_camera'] = camera['T_mast_optical']
    mast['relative_heading_deg'] = heading
    camera['layout_remount'] = dict(schema=config['schema'],
        reference=config['rotation_reference'], optical_orientation=config['camera_orientation'],
        old_T_link1_optical=scene['permanent_camera']['T_link1_optical'],
        old_base_rpy_rad=scene.get('base_rpy_rad', [0.,0.,0.]),
        new_base_rpy_rad=result['base_rpy_rad'],
        T_camera_counterrotation_mast=A.tolist(), counterrotated_mast_parts=sorted(moving),
        mechanical_relation='ROUND_POLE_REINDEXED_IN_UNCHANGED_LOWER_CLAMP',
        source_camera_installation_fingerprint=installation_fingerprint(scene['permanent_camera']))
    calibration = camera['calibration']
    calibration.update(T_parent_child=camera['T_link1_optical'], prior_calibration_valid=False,
        installation_fingerprint=installation_fingerprint(camera))
    result['camera_extrinsics'] = deepcopy(calibration)

    # The old seat was fitted to the old bracket yaw. Build a fresh seat from
    # the same official link surface and the rotated original bracket, at the
    # unchanged installation height; do not leave old invisible collision cells.
    part = next(p for p in mast['parts'] if p['id']=='mast_part_001')
    robot_path = URDF.parents[1]/'meshes/link_1.STL'
    robot = trimesh.load(robot_path, force='mesh', process=False)
    bracket = trimesh.load(root/part['visual'], force='mesh', process=False)
    upper = bracket.triangles@N[:3,:3].T+N[:3,3]
    seating = vertical_seating(robot.triangles, upper)
    if seating['upper_z_translation_m'] > 1e-7:
        raise ValueError('REMOUNTED_BRACKET_INTERSECTS_OFFICIAL_LINK')
    v, f, seat_report = conformal_seat(robot.triangles, upper,
                                      max_thickness_m=config['seat_max_thickness_m'])
    seat_mesh = trimesh.Trimesh(v, f, process=False)
    seat_path = out/'mounting_seat.obj'; seat_mesh.export(seat_path, digits=17)
    relative = seat_path.relative_to(root).as_posix(); sha = file_hash(seat_path)
    interface = mast['mount_interface']
    interface.update(source='Original rotated bracket and official link_1 surfaces; unchanged Z',
        contact_witness_link1_m=seating['witness']['contact_m'],
        remount_geometry_revision=config['geometry_revision'])
    interface['seat'].update(mesh=relative, sha256=sha)
    assets = {relative: sha}
    rows = []
    parts = {p['id']:p for p in mast['parts']}
    adapters = {'eco65_camera_'+p['name']:p for p in camera['pitch_adapter']}
    removed_seat_ids = []
    for row in result['moving_attachments']:
        role = row['role']
        if role=='permanent_mounting_seat':
            removed_seat_ids.append(row['id']); continue
        if role=='permanent_mast':
            local = parts[row['source']['part_id']].get('T_mast', np.eye(4))
            row['T_parent_local'] = (N@local).tolist()
        elif role=='permanent_camera':
            row['T_parent_local'] = (N@camera['T_mast_CAD']).tolist()
        elif role=='permanent_camera_adapter':
            definition = adapters[row['id']]
            row['T_parent_local'] = (N@definition['T_mast']).tolist()
            row['source'].update(definition=deepcopy(definition),camera_mount=deepcopy(mount))
        rows.append(row)
    # Spatially local hulls retain the established seat approximation.
    cells = seat_mesh.split(only_watertight=False); groups = {}
    for i, cell in enumerate(cells):
        if not cell.is_watertight or not cell.is_convex:
            raise ValueError('REMOUNT_SEAT_CELL_NOT_CLOSED_CONVEX')
        key = tuple(np.floor(cell.vertices.mean(0)[:2]/.005).astype(int))
        groups.setdefault(key, []).append(i)
    seat_ids = []
    for i, key in enumerate(sorted(groups)):
        indices = groups[key]
        solid, _ = _convex(trimesh.util.concatenate([cells[j] for j in indices]))
        ident = f'eco65_mast_mounting_seat__convex_{i:03d}'
        dest = out/(ident+'.obj'); solid.export(dest, digits=17)
        path = dest.relative_to(root).as_posix(); assets[path] = file_hash(dest); seat_ids.append(ident)
        rows.append(dict(id=ident,parent_link='link_1',T_parent_local=np.eye(4).tolist(),
            mesh=path,mesh_sha256=assets[path],role='permanent_mounting_seat',
            source=dict(mesh=relative,sha256=sha,decomposition='source_cells_local_conservative_hulls',
                        cell_indices=indices,source_cell_count=len(cells),grouping_pitch_m=.005)))
    result['moving_attachments'] = rows
    interfaces = [r for r in result['fixed_attachment_interfaces'] if r['attachment_id'] not in removed_seat_ids]
    for row in interfaces: row['source'] = deepcopy(interface)
    interfaces += [dict(parent_link='link_1',attachment_id=ident,maximum_penetration_m=.0004,
                       source=deepcopy(interface)) for ident in seat_ids]
    result['fixed_attachment_interfaces'] = interfaces
    error = float(np.max(abs(t['new_base']@N[:3,:3]@np.asarray(camera['T_mast_optical'])[:3,:3]-camera_before)))
    if error > 1e-10: raise ValueError('REMOUNT_CAMERA_WORLD_ORIENTATION_CHANGED')
    result['layout_installation'] = dict(config=deepcopy(config),
        old_base_rpy_rad=scene.get('base_rpy_rad',[0.,0.,0.]),new_base_rpy_rad=result['base_rpy_rad'],
        old_T_link1_mast=M.tolist(),new_T_link1_mast=N.tolist(),
        camera_world_orientation_max_error=error,
        switch_face_normal_D=(t['new_base']@config['switch_face_normal_base']).tolist(),
        seat_geometry=seat_report,bracket_seating=seating,
        replaced_seat_collision_ids=removed_seat_ids,new_seat_collision_ids=seat_ids,
        source_scene_fingerprint=scene['scene_fingerprint'])
    result['scene_fingerprint'] = digest({k:v for k,v in result.items() if k!='scene_fingerprint'})
    return result, assets
