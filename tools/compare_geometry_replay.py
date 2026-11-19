"""Offline geometry comparison on saved field records (no device, no inference, no GPU).

For each saved record: recover boxes from the record's own RGB-D and SAM
masks with the code found on PYTHONPATH, build the frozen real-site scene and
run the candidate-independent payload rest check (SinglePlanner.payload_rest_check,
the same check search_box runs first). No native worker is started, no
candidate or path is searched, no permit exists. Writes one JSON row per
detection: status/reason, measured size, envelope size, the six edge methods,
missing-edge classes, rest-check result and whether candidate search would
start. Source record files are hashed before and after.

  PYTHONPATH=<tree>/src python tools/compare_geometry_replay.py --config CFG \
      --output OUT_DIR TASK_ID [TASK_ID ...]
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np

INPUTS = ('observation.json', 'observation.npz', 'segmentation.npz', 'geometry.json', 'detections.json')


def digest(folder):
    return {n: hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in INPUTS if (folder/n).is_file()}


def edges(box):
    u = box.get('uncertainty') or {}
    out = {}
    for i in u.get('intervals', []):
        name = 'wht'[i['axis']]+('-' if i['side'] < 0 else '+')
        out[name] = dict(method=i['method'], outer_mm=round(1000*(i['outer_m']-i['measured_m'])*i['side'], 1),
                         missing_class=i.get('missing_boundary_class'))
    return out


def run(record, config_path, out, masks_from=None):
    import unloading_sim.workbench.single_carton as package
    from unloading_sim.workbench.single_carton.config import load
    from unloading_sim.workbench.single_carton.perception import recover_boxes
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from unloading_sim.workbench.single_carton.site_scene import build_scene
    from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
    from unloading_sim.workbench.single_carton.size_policy import (size_policy, neighbor_depth_bound,
        mark_incomplete_scene, unmodeled_cartons, NO_PRIOR_POLICIES)
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    before = digest(record)
    config = load(config_path); geometry = config.get('geometry')
    obs = json.loads((record/'observation.json').read_text())
    with np.load(record/'observation.npz', allow_pickle=False) as saved:
        rgb, depth = saved['rgb'], saved['depth']
    source = masks_from or record
    with np.load(source/'segmentation.npz', allow_pickle=False) as saved:
        masks = tuple(saved[f'mask_{i}'].astype(bool) for i in range(len(saved.files)))
    old = json.loads((source/'geometry.json').read_text()) if (source/'geometry.json').is_file() else {}
    detections = old.get('detections') or [dict(label='saved-mask')]*len(masks)
    frame = SimpleNamespace(observation_id=obs['observation_id'], rgb=rgb, depth=depth, intrinsics=obs['intrinsics'],
                            depth_scale_m=obs['depth_scale_m'], stamp=SimpleNamespace(source='SAVED_RECORD_OFFLINE_REPLAY'))
    boxes, rejected = recover_boxes(frame, obs['T_world_camera'], detections, masks, geometry,
                                    (config.get('site') or {}).get('workspace_bounds_m'))
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    self_filter = RobotSelfFilter(planner.init['parameters']['robot_description'], model)
    scene = build_scene(SimpleNamespace(frame=frame, world_camera=obs['T_world_camera'], q_capture=obs['q_capture']),
                        SimpleNamespace(observation_id=obs['observation_id'], masks=masks), boxes, config['site'],
                        self_filter, neighbor_depth_m=neighbor_depth_bound(geometry),
                        unresolved_cartons=rejected if size_policy(geometry) in NO_PRIOR_POLICIES else (), geometry=geometry)
    mark_incomplete_scene(scene, geometry, rejected)
    world = json.loads(json.dumps(scene['world']))+[dict(id=b['id'], size=b['size_m'], pose=b['pose'], category='carton') for b in boxes]
    rows = []
    for b in boxes:
        rest = planner.payload_rest_check(world, b)
        try:
            from unloading_sim.workbench.single_carton.planner import seam_band_contacts
            seam = seam_band_contacts(world, b, rest)
        except ImportError:
            seam = None
        blocked = bool(rest['intersections'] or rest['outside_workspace'])
        first = rest['intersections'][0] if rest['intersections'] else None
        rows.append(dict(detection_index=b['detection_index'], status='RECOVERED',
            measured_size_mm=np.round(np.asarray(b.get('measured_size_m', b['size_m']))*1000, 1).tolist(),
            envelope_size_mm=np.round(np.asarray(b['size_m'])*1000, 1).tolist(), edges=edges(b),
            instance_fit=(b.get('instance_fit') or {}).get('used'),
            stack_support_clip=(b.get('quality') or {}).get('confirmed_stack_support_clip') is not None,
            rest=dict(blocked=blocked, intersections=len(rest['intersections']), outside_workspace=rest['outside_workspace'],
                      first=None if first is None else dict(id=first['id'], category=first.get('category'),
                                                           overlap_mm=round(first['overlap_depth_m']*1000, 3)),
                      supports=rest['supports'], coplanar_seam_contact=seam),
            enters_candidate_search=not blocked and not unmodeled_cartons(scene)))
    for r in rejected:
        rows.append(dict(detection_index=r['detection_index'], status='REJECTED', reason=r['reason']))
    after = digest(record)
    if after != before:raise SystemExit('SOURCE_RECORD_CHANGED')
    result = dict(task_id=record.name, code=str(Path(package.__file__).resolve().parents[3]), config=str(config_path),
                  masks_from=str(source), source_hashes=before, source_unchanged=True,
                  detections=sorted(rows, key=lambda r: r['detection_index']),
                  stack_support=[{k: v for k, v in s.items() if k in ('status', 'reason', 'estimates', 'selected_m', 'spread_m')}
                                 for s in scene.get('confirmed_stack_support', [])],
                  pointcloud_artifacts={k: v for k, v in (scene.get('upper_carton_pointcloud_artifacts') or {}).items()
                                        if k in ('status', 'policy', 'ignored_points')},
                  scope='OFFLINE_SAVED_RECORD_GEOMETRY_AND_REST_CHECK_NO_PATH_SEARCH_NOT_EXECUTABLE',
                  hardware='NOT_RUN', physical_result='NOT_RUN', permits_issued=0)
    out.mkdir(parents=True, exist_ok=True)
    (out/f'{record.name}.json').write_text(json.dumps(result, indent=1, ensure_ascii=False, default=str))
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('records', nargs='+')
    ap.add_argument('--config', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--root', type=Path, default=Path('outputs/single_carton'))
    ap.add_argument('--masks-from', action='append', default=[], help='TASK=DIR: saved masks from an earlier offline replay')
    a = ap.parse_args()
    masks_from = dict(x.split('=', 1) for x in a.masks_from)
    for task in a.records:
        record = a.root/task
        try:
            r = run(record, a.config, a.output, Path(masks_from[task]) if task in masks_from else None)
            print(task, json.dumps([(d['detection_index'], d['status'], d.get('reason') or d['envelope_size_mm'],
                                     d.get('enters_candidate_search')) for d in r['detections']], ensure_ascii=False), flush=True)
        except Exception as error:  # recorded, never hidden
            a.output.mkdir(parents=True, exist_ok=True)
            (a.output/f'{task}.error.txt').write_text(f'{type(error).__name__}: {error}')
            print(task, 'ERROR', type(error).__name__, error, flush=True)
