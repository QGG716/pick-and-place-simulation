"""One frozen compact equipment + final warehouse table geometry in D."""
from __future__ import annotations
from copy import deepcopy
import hashlib
from pathlib import Path
import numpy as np
import yaml

from .model import ROOT, transform, load_json, save_json
from .unloading_layout import build_scene
from ..moveit2_backend import digest

CONFIG = ROOT/'configs/workcells/eco65_native_warehouse_v1.yaml'
TWELVE_CONFIG = ROOT/'configs/workcells/eco65_native_12carton_v1.yaml'
TWELVE_SCHEMA = 'eco65_native_12carton_scene_v1'


def world_objects(scene):
    """Static native objects only; callers manage carton attachment ownership."""
    objects = [dict(id=o['id'], size=o['size_m'],
                    pose=transform(o['position_m']).tolist(), category='environment')
               for o in scene['obstacles']]
    objects.extend(deepcopy(scene.get('native_world_objects', [])))
    ids = [o['id'] for o in objects]
    if len(ids) != len(set(ids)): raise ValueError('NATIVE_WORLD_DUPLICATE_OBJECT')
    return objects


def validate_fixed_interfaces(scene):
    """Only the two source-bound J1 mount pairs may use contact tolerance."""
    rows = scene.get('fixed_attachment_interfaces', [])
    expected = {'eco65_mast_mast_part_001', 'eco65_mast_mounting_seat'}
    permanent = {row['id'] for row in scene.get('moving_attachments', [])}
    seen = set()
    for row in rows:
        name = row['attachment_id']; source = row.get('source')
        if (row['parent_link'] != 'link_1' or name not in expected or name not in permanent
                or name in seen or not source or not source.get('source') or not source.get('seat')
                or source != scene.get('mast', {}).get('mount_interface')
                or row.get('maximum_penetration_m') != .0004):
            raise ValueError('PERMANENT_ATTACHMENT_INTERFACE_UNBOUND')
        seen.add(name)
    return deepcopy(rows)


def adapt_static_scene(snapshot, native_world_objects, moving_attachments, receiving,
                       home_q, *, config=None, config_sha256=None, permanent_camera=None,
                       fixed_attachment_interfaces=None):
    """Bind the existing twelve cartons and permanent equipment to a new task.

    Mesh preparation is supplied by the source-verifying builder. This function
    never changes a carton pose, reconstructs the layout, or accepts a HOME.
    """
    config = deepcopy(config or yaml.safe_load(TWELVE_CONFIG.read_text(encoding='utf-8')))
    check = deepcopy(snapshot); expected = check.pop('fingerprint')
    if (snapshot.get('schema') != 'eco65_full_face_static_snapshot_v1'
            or digest(check) != expected):
        raise ValueError('TWELVE_SOURCE_SNAPSHOT_INVALID')
    source = snapshot['scene']; boxes = deepcopy(source['boxes'])
    expected_ids = {f'carton_r00_l{layer:02d}_c{col:02d}' for layer in range(4) for col in range(3)}
    if (len(boxes) != 12 or {b['id'] for b in boxes} != expected_ids
            or any(b['size_m'] != [.25, .21, .10] for b in boxes)):
        raise ValueError('TWELVE_CARTON_SOURCE_LAYOUT_REQUIRED')
    if (config.get('schema') != 'eco65_native_12carton_config_v1'
            or config.get('target_id') != 'carton_r00_l03_c01'
            or config.get('target_face') != 'top'
            or config.get('enable_hardware') is not False):
        raise ValueError('TWELVE_TASK_CONFIG_INVALID')
    target = next(b for b in boxes if b['id'] == config['target_id'])
    if (target.get('row') != 0 or target.get('layer') != 3 or target.get('column') != 1
            or target.get('supports') or target.get('supported_by') != ['carton_r00_l02_c01']
            or abs(target['pose'][1]) > 1e-10
            or not np.isclose(target['pose'][2], .53, atol=1e-10, rtol=0)):
        raise ValueError('TOP_MIDDLE_CARTON_GEOMETRY_INVALID')
    if (source.get('enable_hardware') is not False
            or source.get('environment_status') != 'ALL_BOUNDS_INCLUDED_CONSERVATIVELY'
            or source.get('T_W_D') != transform([.32, -.1, .8]).tolist()):
        raise ValueError('TWELVE_SOURCE_ENVIRONMENT_INVALID')
    if not source.get('mast', {}).get('parts') or source['mast'].get('attachment') != 'link_1':
        raise ValueError('PERMANENT_J1_MAST_REQUIRED')
    if any(k in source for k in ('probe', 'board', 'calibration_fixture', 'touch_points')):
        raise ValueError('CALIBRATION_FIXTURE_NOT_AN_UNLOADING_SOURCE')
    conveyor_ids = {item['id'] for item in source['conveyors']['items']}
    for obj in source['obstacles']:
        if (obj['id'].startswith(('conveyor_', 'mast_', 'probe_', 'calibration_'))
                or obj.get('owner') in conveyor_ids or obj.get('role') in ('mast', 'calibration_fixture')):
            raise ValueError('SOURCE_HAS_DUPLICATE_OR_TEMPORARY_EQUIPMENT_PROXY:'+obj['id'])
    if not native_world_objects or not moving_attachments:
        raise ValueError('PERMANENT_EQUIPMENT_GEOMETRY_REQUIRED')
    q = np.asarray(home_q, float)
    if q.shape != (6,) or not np.isfinite(q).all(): raise ValueError('HOME_PROPOSAL_INVALID')
    receiver = deepcopy(receiving)
    bounds = np.asarray(receiver['bounds_D_m'], float)
    pose = np.asarray(receiver['box_pose_D'], float)
    if (receiver['region_id'] != config['receiving_region'] or not receiver['support_ids']
            or bounds.shape != (2, 3) or not np.isfinite(bounds).all()
            or np.any(bounds[1]-bounds[0] <= 0) or pose.shape != (4, 4)
            or not np.isfinite(pose).all()):
        raise ValueError('RECEIVING_GEOMETRY_REQUIRED')
    half = np.asarray(target['size_m'])/2
    if (not np.allclose(pose[:3, :3], np.eye(3), atol=1e-12, rtol=0)
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
            or np.any(pose[:2, 3]-half[:2] < bounds[0, :2]-1e-9)
            or np.any(pose[:2, 3]+half[:2] > bounds[1, :2]+1e-9)
            or abs(pose[2, 3]-half[2]-bounds[1, 2]) > 1e-9):
        raise ValueError('TARGET_NOT_FULLY_SUPPORTED_IN_RECEIVING_REGION')
    world_ids = {o['id'] for o in native_world_objects}
    if not set(receiver['support_ids']) <= world_ids:
        raise ValueError('RECEIVING_SUPPORT_MESH_NOT_BOUND')
    scene = deepcopy(source); scene.pop('scene_fingerprint', None)
    scene.update(schema=TWELVE_SCHEMA, layout_id=config['layout_id'],
        source_static_snapshot_fingerprint=expected, source_static_scene_fingerprint=source['scene_fingerprint'],
        source_layout_id=source['layout_id'], box=deepcopy(target), target_box_id=target['id'],
        boxes=boxes, execution_mode='simulation', enable_hardware=False, planning_frame='D',
        status='PLANNING_ONLY_NOT_EXECUTABLE', transport='NATIVE_PLANNING_PREDICTED_EVENTS',
        native_world_objects=deepcopy(native_world_objects), moving_attachments=deepcopy(moving_attachments),
        fixed_attachment_interfaces=deepcopy(fixed_attachment_interfaces or []),
        margin_m=.002, mesh_error_reserve_m=.0002, contact_tolerance_m=.0004,
        permanent_camera=deepcopy(permanent_camera), config_sha256=deepcopy(config_sha256 or snapshot['config_sha256']))
    for obj in scene['obstacles']:
        obj.setdefault('owner', obj['id'])
        obj.setdefault('role', 'static_equipment')
    scene['native_task'] = dict(schema='eco65_single_carton_task_v1', target_id=target['id'],
        target_face=config['target_face'], stack_carton_ids=sorted(expected_ids),
        original_support_names=list(target['supported_by']), receiving=receiver,
        preapproach_m=config['preapproach_m'], preplace_m=config['preplace_m'],
        withdrawal_m=config['withdrawal_m'], minimum_lift_m=config['minimum_lift_m'])
    # Compatibility values are derived from this snapshot, never the old four-
    # carton placement. They do not certify the seed or conveyor transport.
    scene['source_equipment_config'] = dict(task=dict(initial_seed_q_rad=q.tolist()),
        conveyors=dict(surface_z_m=float(bounds[1, 2]),
            longitudinal=dict(receive_center_xy_m=pose[:2, 3].tolist())))
    scene['source_config'] = deepcopy(scene['source_equipment_config'])
    scene['support_surface_ids'] = ['cargo_pedestal', *receiver['support_ids']]
    scene['occupancy'] = {receiver['region_id']: []}
    scene['components_by_id'] = {o['id']: dict(role=o['role'], owner=o['owner']) for o in scene['obstacles']}
    scene['environment_asset_sha256'] = deepcopy(scene['environment']['asset_sha256'])
    world_objects(scene)
    scene['scene_fingerprint'] = digest(scene)
    return verify_scene(scene)


def combined_scene(config=CONFIG, *, root=ROOT, environment=None):
    root = Path(root).resolve(); path = Path(config).resolve()
    c = yaml.safe_load(path.read_text(encoding='utf-8'))
    if (c['schema'] != 'eco65_native_combined_scene_v1' or c['planning_frame'] != 'D'
            or c['target_id'] != 'carton_r00_l01_c00' or c['target_face'] != 'top'
            or c['route'] != 'B' or c['enable_hardware']):
        raise ValueError('NATIVE_SCENE_CONFIG_MISMATCH')
    equipment = yaml.safe_load((root/c['equipment_config']).read_text(encoding='utf-8'))
    warehouse = yaml.safe_load((root/c['warehouse_config']).read_text(encoding='utf-8'))
    placement = warehouse['placement']
    if (placement['workstation_scale'] != 1 or placement['environment_scale'] != 1
            or placement['yaw_rad'] != 0 or placement['translation_W_D_m'] != [.32, -.1, .8]):
        raise ValueError('PINNED_WAREHOUSE_TRANSFORM_CHANGED')
    table = warehouse['table']; merged = deepcopy(equipment)
    merged['table']['bounds_xy_m'] = table['bounds_xy_D_m']
    merged['table']['thickness_m'] = table['thickness_m']
    scene = build_scene(merged)
    scene['source_equipment_config'] = equipment
    scene['source_warehouse_config'] = warehouse
    scene['native_config'] = c
    scene['T_W_D'] = transform(placement['translation_W_D_m']).tolist()
    scene['planning_frame'] = 'D'; scene['layout_id'] = c['layout_id']
    scene['box'] = deepcopy(next(b for b in scene['boxes'] if b['id'] == c['target_id']))
    scene['target_box_id'] = c['target_id']
    x0, x1, y0, y1 = table['bounds_xy_D_m']; w = table['leg_width_m']; th = table['thickness_m']
    inset = table['leg_inset_m']; xs = [x0+inset, x1-inset]; ys = [y0+inset, y1-inset]
    def part(ident, size, position):
        scene['obstacles'].append(dict(id=ident, size_m=size, position_m=position,
            role='warehouse_table_frame', owner='warehouse_table', rgba=[.2, .24, .28, 1]))
    for i, x in enumerate(xs):
        for j, y in enumerate(ys): part(f'Table/Leg_{i}{j}', [w, w, .8-th], [x, y, (-.8-th)/2])
    for z in [-.09, table['lower_support_z_D_m']]:
        for i, y in enumerate(ys): part(f'Table/RailX_{i}_{abs(int(z*100))}', [xs[1]-xs[0], w, w], [(x0+x1)/2, y, z])
        for i, x in enumerate(xs): part(f'Table/RailY_{i}_{abs(int(z*100))}', [w, ys[1]-ys[0], w], [x, (y0+y1)/2, z])
    scene['environment_status'] = 'NOT_EVALUATED'
    if environment is not None:
        check = deepcopy(environment); expected = check.pop('fingerprint')
        if (digest(check) != expected or environment['schema'] != 'eco65_warehouse_bounds_v1'
                or environment['T_W_D'] != scene['T_W_D'] or environment['complete_traversal'] is not True
                or not environment['objects']):
            raise ValueError('ENVIRONMENT_BOUNDS_IDENTITY_INVALID')
        translation = np.asarray(placement['translation_W_D_m'])
        ids = set()
        for row in environment['objects']:
            ident = 'warehouse/'+row['id']
            if ident in ids: raise ValueError('ENVIRONMENT_DUPLICATE_OBJECT')
            ids.add(ident)
            lo, hi = np.asarray(row['bounds_W_m'], float)
            if not np.isfinite([lo, hi]).all() or np.any(hi <= lo): raise ValueError('INVALID_ENVIRONMENT_BOUNDS')
            scene['obstacles'].append(dict(id=ident, size_m=(hi-lo).tolist(),
                position_m=((hi+lo)/2-translation).tolist(), role='warehouse_environment',
                owner='warehouse', rgba=[.5, .5, .5, 1], representation='conservative_world_AABB'))
        scene['environment_status'] = 'ALL_BOUNDS_INCLUDED_CONSERVATIVELY'
        scene['environment_fingerprint'] = environment['fingerprint']
        scene['environment_asset_sha256'] = environment['asset_sha256']
        scene['environment'] = deepcopy(environment)
    scene['components_by_id'] = {o['id']: dict(role=o['role'], owner=o['owner']) for o in scene['obstacles']}
    scene['config_sha256'] = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()}
    for relative in [c['equipment_config'], c['warehouse_config'], c['robot_config']]:
        scene['config_sha256'][relative] = hashlib.sha256((root/relative).read_bytes()).hexdigest()
    scene['scene_fingerprint'] = digest(scene)
    return scene


def verify_scene(scene, *, require_environment=True):
    check = deepcopy(scene); expected = check.pop('scene_fingerprint')
    if digest(check) != expected: raise ValueError('SCENE_FINGERPRINT_MISMATCH')
    if require_environment and scene['environment_status'] != 'ALL_BOUNDS_INCLUDED_CONSERVATIVELY':
        raise ValueError('WAREHOUSE_ENVIRONMENT_NOT_EVALUATED')
    if scene.get('schema') == TWELVE_SCHEMA:
        from .native_contact import validate_task_contract
        expected_ids = {f'carton_r00_l{l:02d}_c{c:02d}' for l in range(4) for c in range(3)}
        task = scene['native_task']
        if (len(scene['boxes']) != 12 or {b['id'] for b in scene['boxes']} != expected_ids
                or set(task['stack_carton_ids']) != expected_ids
                or task['target_id'] != 'carton_r00_l03_c01'
                or scene['target_box_id'] != task['target_id'] or scene['box']['id'] != task['target_id']
                or task['target_face'] != 'top' or not scene['moving_attachments']
                or not scene['native_world_objects'] or scene.get('enable_hardware') is not False
                or scene.get('execution_mode') != 'simulation' or scene.get('planning_frame') != 'D'
                or scene.get('T_W_D') != transform([.32, -.1, .8]).tolist()):
            raise ValueError('TWELVE_TASK_SCENE_BINDING_INVALID')
        validate_task_contract(scene)
        validate_fixed_interfaces(scene)
        world_objects(scene)
        return scene
    expected_ids = {f'carton_r00_l{l:02d}_c{c:02d}' for l in range(2) for c in range(2)}
    if {b['id'] for b in scene['boxes']} != expected_ids or len(scene['boxes']) != 4:
        raise ValueError('FOUR_CARTON_IDENTITY_MISMATCH')
    return scene


def draw_layout(scene, output):
    """Dimension view only; this is not a robot asset/FK or collision pass."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, indices, title in [(axes[0], (0, 1), 'D: top view (m)'), (axes[1], (0, 2), 'D: side view (m)')]:
        for o in scene['obstacles']+scene['boxes']:
            if o.get('role') == 'warehouse_environment': continue
            pos = np.array(o.get('position_m', o.get('pose'))); size = np.array(o['size_m'])
            i, j = indices; lo = pos-size/2
            ax.add_patch(Rectangle((lo[i], lo[j]), size[i], size[j], facecolor=o.get('rgba', [.7,.45,.2,1])[:3],
                                   edgecolor='black', linewidth=.4, alpha=.3))
        base = scene['base_position_m']; ax.plot(base[indices[0]], base[indices[1]], 'r+', label='base frame')
        ax.autoscale_view(); ax.set_aspect('equal'); ax.set_title(title); ax.grid(alpha=.2)
    fig.suptitle('Compact equipment + 2 m warehouse table\nLayout geometry only; robot/tool collision and HOME are not evaluated')
    fig.tight_layout(); fig.savefig(output, dpi=160); plt.close(fig)
