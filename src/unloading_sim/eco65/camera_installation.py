"""Rigid camera remount about the CAD pole, with unchanged lower mast/seat."""
from copy import deepcopy
import hashlib
import numpy as np
from .model import check_transform, transform
from ..moveit2_backend import digest


def pole_axis(mast):
    part = next(p for p in mast['parts'] if p['id'] == 'mast_part_019')
    # The bottom circular planar face belongs to the vertical pole, whereas
    # the whole part AABB includes the eccentric U-shaped upper pocket.
    face = min((f for f in part['planar_faces']
                if np.allclose(f['normal'], [0, 0, -1], atol=1e-10)),
               key=lambda f: f['point_m'][2])
    return dict(point_mast_m=face['point_m'], axis_mast=[0., 0., 1.],
                source_part_id=part['id'], source_visual_sha256=part['visual_sha256'],
                circular_end_area_m2=face['area_m2'],
                radius_m=float(np.sqrt(face['area_m2']/np.pi)),
                basis='ORIGINAL_CAD_POLE_BOTTOM_CIRCULAR_FACE')


def rotate_camera(camera, mast, config):
    if camera.get('axis_rotation'):
        raise ValueError('CAMERA_AXIS_ROTATION_ALREADY_APPLIED')
    if abs(camera['mount']['downward_pitch_deg']-config['downward_pitch_deg'])>1e-9:
        raise ValueError('CAMERA_PRIOR_PITCH_CHANGED')
    axis = pole_axis(mast)
    p = np.asarray(axis['point_mast_m'])
    if not np.allclose(p, config['expected_axis_point_mast_m'], atol=1e-9, rtol=0):
        raise ValueError('CAMERA_POLE_AXIS_CHANGED')
    yaw = np.radians(config['yaw_deg'])
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                  [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    A = transform(p-R@p, R)
    result = deepcopy(camera)
    old_cad, old_opt = (check_transform(camera[k]) for k in ('T_mast_CAD', 'T_mast_optical'))
    result['T_mast_CAD'] = (A@old_cad).tolist()
    result['T_mast_optical'] = (A@old_opt).tolist()
    result['T_link1_optical'] = (check_transform(mast['T_link1_mast'])@A@old_opt).tolist()
    mount = result['mount']
    mount['prior_pitch_installation'] = deepcopy(camera['mount'])
    mount['schema'] = 'eco65_camera_mechanical_pitch_yaw_v2'
    mount['heading_from_link1_minus_X_deg'] = config['heading_from_link1_minus_X_deg']
    mount['pivot_mast_m'] = (A@np.r_[camera['mount']['pivot_mast_m'], 1.])[:3].tolist()
    mount['axis_mast'] = (R@camera['mount']['axis_mast']).tolist()
    mount['T_new_from_old_mast'] = A.tolist()
    fixed_risers = [r for r in camera['pitch_adapter'] if r['name'].startswith('pitch_riser_')]
    connections = []
    for row in result['pitch_adapter']:
        T = A@check_transform(row['T_mast'])
        if row['name'].startswith('pitch_riser_'):
            # The pocket is eccentric to the pole. The reversed journal no
            # longer sits above its old wall. Widen each existing derived
            # riser to bridge the nearest unchanged wall and new journal.
            fixed = min(fixed_risers, key=lambda r: abs(r['T_mast'][0][3]-T[0, 3]))
            x_fixed = fixed['T_mast'][0][3]
            row['size_m'][0] += abs(x_fixed-T[0, 3])
            T[0, 3] = (x_fixed+T[0, 3])/2
            row['revision'] = dict(kind='PROJECT_DERIVED_OFFSET_RISER',
                fixed_wall_x_m=x_fixed, journal_x_m=float((A@check_transform(
                    next(r for r in camera['pitch_adapter'] if r['name']==row['name'])['T_mast']))[0,3]),
                fixed_wall_top_z_m=.45, original_supplier_CAD_modified=False)
            connections.append([fixed['name'].replace('pitch_riser_', 'mast_pocket_'), row['name']])
        row['T_mast'] = T.tolist()
    mount['connections'] = connections+[c for c in camera['mount']['connections']
                                        if not c[0].startswith('mast_pocket_')]
    result['axis_rotation'] = dict(schema=config['schema'], **axis, yaw_deg=config['yaw_deg'],
        T_new_from_old_mast=A.tolist(), old_T_mast_CAD=old_cad.tolist(),
        old_T_mast_optical=old_opt.tolist(), fixed_parts='ALL_ORIGINAL_MAST_AND_LOWER_SEAT',
        modified_parts=[r['name'] for r in result['pitch_adapter'] if 'revision' in r])
    result['calibration'] = dict(status=config['calibration_status'],
        source='KNOWN_RIGID_INSTALLATION_CHANGE_FROM_PRIOR_NOMINAL_CAD_OPTICAL_RELATION',
        prior_calibration_valid=False, parent_frame='link_1', child_frame='camera_optical',
        T_parent_child=result['T_link1_optical'],
        direction='T_parent_child maps camera_optical coordinates into link_1',translation_unit='metre',
        installation_fingerprint=digest(result['axis_rotation']),
        observation_source=config['observation_source'])
    return result


def installation_fingerprint(camera):
    identity = camera['axis_rotation']
    if camera.get('layout_remount'):
        identity = dict(axis_rotation=identity, layout_remount=camera['layout_remount'])
    if camera.get('pitch_mount_yaw'):
        identity = dict(previous=identity, pitch_mount_yaw=camera['pitch_mount_yaw'])
    if camera.get('pole_to_bar_end'):
        identity = dict(previous=identity, pole_to_bar_end=camera['pole_to_bar_end'])
    return digest(identity)


PITCH_MOUNT_YAW_SCHEMA = 'eco65_pitch_mount_yaw_v1'


def apply_pitch_mount_yaw(scene, config):
    """User decision 2026-11-25: the vertical pole and the horizontal bar on J1
    stay; the pitch mount (risers, journals, saddle) with the camera on it
    turns about Z (the pole axis) by ``yaw_deg``; the camera is not turned
    relative to its mount. Every mast part (including the pole with its upper
    pocket and the lower seat) is unchanged. Collision model only; the field
    perception keeps its own calibrated extrinsics file.
    """
    if (config.get('schema') != PITCH_MOUNT_YAW_SCHEMA or config.get('enable_hardware')
            or type(config.get('yaw_deg')) not in (int, float) or not np.isfinite(config['yaw_deg'])
            or not -360. < config['yaw_deg'] < 360. or not str(config.get('basis', '')).strip()):
        raise ValueError('PITCH_MOUNT_YAW_CONFIG_INVALID')
    if scene['permanent_camera'].get('pitch_mount_yaw'):
        raise ValueError('PITCH_MOUNT_YAW_ALREADY_APPLIED')
    result = deepcopy(scene); mast = result['mast']; camera = result['permanent_camera']
    axis = pole_axis(mast); p = np.asarray(axis['point_mast_m'])
    if not np.allclose(p, config['expected_axis_point_mast_m'], atol=1e-9, rtol=0):
        raise ValueError('CAMERA_POLE_AXIS_CHANGED')
    yaw = np.radians(config['yaw_deg'])
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0.], [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    A = transform(p-R@p, R)
    old = deepcopy(scene['permanent_camera'])
    for key in ('T_mast_CAD', 'T_mast_optical'):
        camera[key] = (A@check_transform(camera[key])).tolist()
    for row in camera['pitch_adapter']:
        row['T_mast'] = (A@check_transform(row['T_mast'])).tolist()
    mount = camera['mount']
    mount['pivot_mast_m'] = (A@np.r_[mount['pivot_mast_m'], 1.])[:3].tolist()
    mount['axis_mast'] = (R@np.asarray(mount['axis_mast'])).tolist()
    M = check_transform(mast['T_link1_mast'])
    camera['T_link1_optical'] = (M@check_transform(camera['T_mast_optical'])).tolist()
    f = np.asarray(camera['T_link1_optical'])[:3, 2]
    heading = float(np.degrees(np.arctan2(-f[1], -f[0])))
    mount['heading_from_link1_minus_X_deg'] = heading
    mast['T_mast_camera'] = camera['T_mast_optical']
    mast['relative_heading_deg'] = heading
    camera['pitch_mount_yaw'] = dict(schema=config['schema'], yaw_deg=float(config['yaw_deg']), basis=config['basis'],
        **{k: axis[k] for k in ('point_mast_m', 'axis_mast', 'source_part_id', 'source_visual_sha256')}, axis_basis=axis['basis'],
        T_new_from_old_mast=A.tolist(), old_T_link1_optical=old['T_link1_optical'],
        moved=['permanent_camera.parts', 'permanent_camera.pitch_adapter'],
        fixed_parts='ALL_MAST_PARTS_INCLUDING_POLE_POCKET_HORIZONTAL_BAR_AND_SEAT',
        source_camera_installation_fingerprint=installation_fingerprint(old))
    camera['calibration'].update(T_parent_child=camera['T_link1_optical'], prior_calibration_valid=False,
                                 installation_fingerprint=installation_fingerprint(camera))
    result['camera_extrinsics'] = deepcopy(camera['calibration'])
    adapters = {'eco65_camera_'+r['name']: r for r in camera['pitch_adapter']}
    moved = 0
    for row in result['moving_attachments']:
        if row['role'] == 'permanent_camera':
            row['T_parent_local'] = (M@check_transform(camera['T_mast_CAD'])).tolist(); moved += 1
        elif row['role'] == 'permanent_camera_adapter':
            definition = adapters[row['id']]
            row['T_parent_local'] = (M@check_transform(definition['T_mast'])).tolist(); moved += 1
            row['source'] = dict(row.get('source', {}), definition=deepcopy(definition), camera_mount=deepcopy(mount))
    if moved != len(camera['parts'])+len(camera['pitch_adapter']):
        raise ValueError('PITCH_MOUNT_YAW_ATTACHMENT_INVENTORY_CHANGED')
    result['scene_fingerprint'] = digest({k: v for k, v in result.items() if k != 'scene_fingerprint'})
    return result


def camera_world_transform(scene, world_from_link1, extrinsics=None):
    """Current nominal optical conversion; stale calibration is not a fallback."""
    expected=scene['permanent_camera']['calibration']
    supplied=scene['camera_extrinsics'] if extrinsics is None else extrinsics
    matrix=supplied.get('T_parent_child',supplied.get('transform'))
    installation=installation_fingerprint(scene['permanent_camera'])
    # The first remount run recorded the same nominal matrix as "transform".
    # Accept that captured descriptor only when it equals this scene's own
    # definition; external descriptors must carry the installation identity.
    captured=supplied==expected
    if ((not captured and supplied.get('installation_fingerprint')!=installation)
            or matrix!=scene['permanent_camera']['T_link1_optical']
            or supplied.get('status')!=expected['status']):
        raise ValueError('CAMERA_EXTRINSICS_DO_NOT_MATCH_CURRENT_INSTALLATION')
    return check_transform(world_from_link1)@check_transform(matrix)


def apply_camera_installation(scene, config, output, root):
    """Update the one scene definition, all existing native links and nominal extrinsics."""
    import trimesh
    from .asset_paths import resolve_asset
    out = output.resolve(); out.mkdir(parents=True, exist_ok=True)
    result = deepcopy(scene)
    camera = rotate_camera(scene['permanent_camera'], scene['mast'], config)
    axis = camera['axis_rotation']
    part = next(p for p in scene['mast']['parts'] if p['id']==axis['source_part_id'])
    mesh_path = resolve_asset(root, part['visual'])
    if hashlib.sha256(mesh_path.read_bytes()).hexdigest()!=part['visual_sha256']:
        raise ValueError('CAMERA_POLE_SOURCE_CHANGED')
    vertices = np.asarray(trimesh.load(mesh_path, process=False).vertices)
    # The cylinder tessellation has end rings, no mid-height vertices.
    pole = vertices[abs(vertices[:,2]-axis['point_mast_m'][2])<1e-8]
    if not len(pole) or np.max(abs(np.linalg.norm(
            pole[:,:2]-np.asarray(axis['point_mast_m'])[:2],axis=1)-axis['radius_m']))>1e-6:
        raise ValueError('CAMERA_AXIS_NOT_ON_ACTUAL_POLE')
    M = check_transform(scene['mast']['T_link1_mast'])
    adapters = {'eco65_camera_'+r['name']:r for r in camera['pitch_adapter']}
    bodies = {'eco65_camera_'+r['id'] for r in camera['parts']}
    assets = {}
    for row in result['moving_attachments']:
        if row['id'] in bodies:
            row['T_parent_local'] = (M@camera['T_mast_CAD']).tolist()
        if row['id'] in adapters:
            definition = adapters[row['id']]
            row['T_parent_local'] = (M@definition['T_mast']).tolist()
            path = out/(row['id']+'.obj')
            trimesh.creation.box(definition['size_m']).export(path)
            row.update(mesh=path.relative_to(root).as_posix(),
                       mesh_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            row['source']=dict(definition=deepcopy(definition),camera_mount=deepcopy(camera['mount']))
            assets[row['mesh']] = row['mesh_sha256']
    result['permanent_camera'] = camera
    result['camera_extrinsics'] = deepcopy(camera['calibration'])
    # Retained compatibility fields are aliases of the actual optical frame.
    result['mast']['T_mast_camera'] = camera['T_mast_optical']
    result['mast']['relative_heading_deg'] = config['heading_from_link1_minus_X_deg']
    result['mast']['camera_definition_source'] = 'permanent_camera.T_mast_optical'
    result['scene_fingerprint'] = digest({k:v for k,v in result.items() if k!='scene_fingerprint'})
    return result, assets
