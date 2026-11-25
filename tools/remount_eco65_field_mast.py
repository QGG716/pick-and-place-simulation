"""Apply a mast or camera-mount change to the field collision snapshot.

Configs: an incremental mast remount (eco65_incremental_mast_remount_v2) or a
pitch-mount yaw (eco65_pitch_mount_yaw_v1). User decision 2026-11-25: the pole
and the horizontal bar stay, the pitch mount with the camera turns 90 deg
clockwise about Z; "现场用的标定文件先不变，改布局是为了碰撞检测". Only the
collision/model snapshot changes. The field perception keeps its calibrated
extrinsics file; this tool reports the nominal model camera next to it.

Reads the source snapshot, never connects devices, writes a new directory.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import yaml

from unloading_sim.eco65.model import ROOT, load_json, save_json
from unloading_sim.eco65.native_io import file_hash
from unloading_sim.eco65.native_scene import verify_scene
from unloading_sim.eco65.native_model import build_native_model
from unloading_sim.eco65.layout_installation import (apply_layout_installation, apply_bar_yaw, BAR_YAW_SCHEMA,
    apply_pole_to_bar_end, POLE_TO_BAR_END_SCHEMA)
from unloading_sim.eco65.camera_installation import pole_axis, apply_pitch_mount_yaw, PITCH_MOUNT_YAW_SCHEMA
from unloading_sim.moveit2_backend import digest
from unloading_sim.workbench.scene import load_snapshot


def pole_point(mast):
    part = next(p for p in mast['parts'] if p['id'] == 'mast_part_019')
    T = np.asarray(mast['T_link1_mast'])@np.asarray(part.get('T_mast', np.eye(4)))
    return (T@np.r_[pole_axis(mast)['point_mast_m'], 1.])[:3]


def camera_row(T):
    T = np.asarray(T, float); z = T[:3, 2]
    return dict(position_link1_m=T[:3, 3].round(4).tolist(), optical_axis_link1=z.round(4).tolist(),
                heading_deg=float(np.degrees(np.arctan2(z[1], z[0]))), downward_pitch_deg=float(np.degrees(np.arcsin(-z[2]))))


def remount(source, config_path, output, calibrated, native_asset_root):
    # The field loads its snapshot geometry-only (archived capture records are
    # not runtime geometry); the new snapshot is checked the same way.
    old = load_snapshot(source, geometry_only=True)
    out = Path(output).resolve(); out.relative_to(ROOT/'outputs'); out.mkdir(parents=True, exist_ok=False)
    config_path = Path(config_path).resolve(); config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if config['schema'] == PITCH_MOUNT_YAW_SCHEMA:
        scene, layout_assets = apply_pitch_mount_yaw(old['scene'], config), {}
    elif config['schema'] == POLE_TO_BAR_END_SCHEMA:
        scene, layout_assets = apply_pole_to_bar_end(old['scene'], config), {}
    elif config['schema'] == BAR_YAW_SCHEMA:
        scene, layout_assets = apply_bar_yaw(old['scene'], config, out/'layout_attachments', ROOT)
    elif config['schema'] == 'eco65_incremental_mast_remount_v2':
        scene, layout_assets = apply_layout_installation(deepcopy(old['scene']), config, out/'layout_attachments', ROOT)
    else:
        raise ValueError('FIELD_MAST_CHANGE_SCHEMA_UNSUPPORTED')
    hashes = dict(old['config_sha256']); hashes[config_path.relative_to(ROOT).as_posix()] = file_hash(config_path)
    scene['config_sha256'] = hashes
    scene['scene_fingerprint'] = digest({k: v for k, v in scene.items() if k != 'scene_fingerprint'})
    verify_scene(scene)
    model = build_native_model(scene, out/'model', native_asset_root=native_asset_root)
    assets = dict(old['asset_sha256']); assets.update(layout_assets)
    assets.update({row['convex_path']: row['convex_sha256'] for row in model['convex_bindings']})
    snapshot = dict(schema='eco65_native_snapshot_v1', scene=scene, tool=model['tool'], native_model=model,
                    asset_sha256=assets, config_sha256=hashes)
    snapshot['fingerprint'] = digest(snapshot)
    save_json(out/'snapshot.json', snapshot)
    load_snapshot(out/'snapshot.json', geometry_only=True)
    old_ids = [r['id'] for r in old['scene']['moving_attachments'] if r['role'] != 'permanent_mounting_seat']
    new_ids = [r['id'] for r in scene['moving_attachments'] if r['role'] != 'permanent_mounting_seat']
    if old_ids != new_ids:
        raise ValueError('MAST_REMOUNT_CHANGED_NON_SEAT_ATTACHMENTS')
    old_pole = pole_point(old['scene']['mast'])
    new_pole = pole_point(scene['mast'])
    cal = json.loads(Path(calibrated).read_text())
    report = dict(status='BUILT_COLLISION_MODEL_ONLY_NOT_RUNTIME_VALIDATED', physical_execution='NOT_RUN',
                  user_decision=config.get('basis', config['schema']),
                  source_snapshot=str(Path(source)), source_snapshot_fingerprint=old['fingerprint'],
                  snapshot=str((out/'snapshot.json').relative_to(ROOT)), snapshot_fingerprint=snapshot['fingerprint'],
                  config=config_path.relative_to(ROOT).as_posix(), config_sha256=hashes[config_path.relative_to(ROOT).as_posix()],
                  change=config['schema'],geometry_revision=config.get('geometry_revision'),
                  pole_axis_link1_m=dict(before=old_pole.round(5).tolist(), after=new_pole.round(5).tolist()),
                  model_camera=dict(before=camera_row(old['scene']['permanent_camera']['T_link1_optical']),
                                    after=camera_row(scene['permanent_camera']['T_link1_optical'])),
                  field_perception_extrinsics=dict(file=str(calibrated), sha256=file_hash(calibrated), status=cal.get('status'),
                                                   unchanged=True, camera=camera_row(cal['matrix'])),
                  layout_installation=scene['layout_installation'],
                  pitch_mount_yaw=scene['permanent_camera'].get('pitch_mount_yaw'),
                  bar_yaw={k: v for k, v in (scene['mast'].get('bar_yaw') or {}).items() if k != 'seat_geometry'} or None,
                  pole_to_bar_end=scene['mast'].get('pole_to_bar_end'),
                  other_moving_attachment_ids_unchanged=True)
    save_json(out/'remount_report.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'outputs/workbench/scene_snapshot.json')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--calibrated-extrinsics', type=Path, default=ROOT/'configs/workbench/reference/active_extrinsics.json')
    parser.add_argument('--native-asset-root', default='/work-eco65-20261015')
    a = parser.parse_args()
    r = remount(a.source, a.config, a.output, a.calibrated_extrinsics, a.native_asset_root)
    print(json.dumps({k: r[k] for k in ('status', 'snapshot', 'snapshot_fingerprint', 'pole_axis_link1_m', 'model_camera',
                                        'field_perception_extrinsics')}, ensure_ascii=False, indent=1))
