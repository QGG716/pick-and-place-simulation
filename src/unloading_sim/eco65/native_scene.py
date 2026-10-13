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
