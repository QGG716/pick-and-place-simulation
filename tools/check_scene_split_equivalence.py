"""Equivalence report for scene_build=SPLIT_PARALLEL on saved field records.

For every saved single-carton record with a frozen RGB-D, SAM masks and
q_capture, the site scene is built twice from the same recovered boxes:
SEQUENTIAL (build_scene as before) and SPLIT_PARALLEL (precompute_scene
followed by build_scene(precomputed=...)). The two scenes are compared as
canonical JSON (every obstacle, voxel run, support, shadow and evidence count)
and the stage times are reported.

Offline development only: no device import or connection, no permit, no motion
or valve command. The source records are read, never modified.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.size_policy import size_policy, neighbor_depth_bound, NO_PRIOR_POLICIES
from unloading_sim.workbench.single_carton.site_scene import build_scene, precompute_scene
from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), default=lambda x: np.asarray(x).tolist())


def first_difference(a, b, path='scene'):
    if type(a) is not type(b): return path
    if isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b: return f'{path}.{k}'
            d = first_difference(a[k], b[k], f'{path}.{k}')
            if d: return d
        return None
    if isinstance(a, list):
        if len(a) != len(b): return path+'[len]'
        for i, (x, y) in enumerate(zip(a, b)):
            d = first_difference(x, y, f'{path}[{i}]')
            if d: return d
        return None
    return None if a == b else path


def records(root, limit):
    found = [d for d in sorted(root.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)
             if (d/'observation.npz').is_file() and (d/'segmentation.npz').is_file() and (d/'observation.json').is_file()]
    return found[:limit] if limit else found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--records', type=Path, default=ROOT/'outputs/single_carton')
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = load(args.config)
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    self_filter = RobotSelfFilter(planner.init['parameters']['robot_description'], model)
    geometry = config.get('geometry')
    rows = []
    for folder in records(args.records, args.limit):
        before = {n: hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in ('observation.json', 'observation.npz', 'segmentation.npz')}
        obs = json.loads((folder/'observation.json').read_text())
        with np.load(folder/'observation.npz', allow_pickle=False) as saved:
            rgb, depth = saved['rgb'], saved['depth']
        with np.load(folder/'segmentation.npz', allow_pickle=False) as saved:
            masks = tuple(saved[f'mask_{i}'].astype(bool) for i in range(len(saved.files)))
        detections_file = folder/'detections.json'
        detections = (json.loads(detections_file.read_text()).get('detections') if detections_file.is_file() else None) or [dict(label='saved-mask')]*len(masks)
        frame = SimpleNamespace(observation_id=obs['observation_id'], rgb=rgb, depth=depth, intrinsics=obs['intrinsics'],
                                depth_scale_m=obs['depth_scale_m'], stamp=SimpleNamespace(source='SAVED_RECORD_OFFLINE_REPLAY'))
        observation = SimpleNamespace(frame=frame, world_camera=obs['T_world_camera'], q_capture=obs['q_capture'])
        result = SimpleNamespace(observation_id=obs['observation_id'], masks=masks)
        row = dict(task_id=folder.name, masks=len(masks))
        try:
            boxes, rejected = recover_boxes(frame, obs['T_world_camera'], detections, masks, geometry,
                                            (config.get('site') or {}).get('workspace_bounds_m'))
            options = dict(neighbor_depth_m=neighbor_depth_bound(geometry),
                           unresolved_cartons=rejected if size_policy(geometry) in NO_PRIOR_POLICIES else (), geometry=geometry)
            t0 = time.perf_counter()
            sequential = build_scene(observation, result, copy.deepcopy(boxes), config['site'], self_filter,
                                     **copy.deepcopy(options))
            t1 = time.perf_counter()
            pre = precompute_scene(frame, obs['T_world_camera'], obs['q_capture'], config['site'], self_filter)
            t2 = time.perf_counter()
            split = build_scene(observation, result, copy.deepcopy(boxes), config['site'], self_filter,
                                precomputed=pre, **copy.deepcopy(options))
            t3 = time.perf_counter()
            a, b = json.loads(canonical(sequential)), json.loads(canonical(split))
            row.update(boxes=len(boxes), identical=a == b, first_difference=first_difference(a, b),
                       sequential_s=round(t1-t0, 3), precompute_s=round(t2-t1, 3), after_masks_s=round(t3-t2, 3),
                       scene_sha256=hashlib.sha256(canonical(a).encode()).hexdigest())
        except Exception as error:
            row.update(error=f'{type(error).__name__}: {error}')
        after = {n: hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in before}
        row['source_unchanged'] = before == after
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    compared = [r for r in rows if 'identical' in r]
    report = dict(schema='eco65_scene_split_equivalence_v1', config=str(args.config), records=len(rows),
                  compared=len(compared), identical=sum(r['identical'] for r in compared),
                  errors=[r for r in rows if 'error' in r], all_sources_unchanged=all(r['source_unchanged'] for r in rows),
                  hardware_execution='NOT_RUN', rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
