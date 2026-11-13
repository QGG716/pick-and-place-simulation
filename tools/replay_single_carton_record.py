"""Offline replay of one saved single-carton task record with the current code.

Re-runs box recovery, the frozen real-site scene and the two-phase native
search (the same SinglePlanner.search_box used by the workbench) on the
record's original RGB-D, SAM masks, q_capture and q_start. With
--detector config, detection and SAM are re-run on the saved RGB with the
configured backend (local or the remote GPU server) and compared with the
record's own masks.

Development diagnostic only: no device import or connection, no permit is
registered, no motion or valve command exists in this path. A complete plan
found here is NOT an executable plan and NOT physical acceptance; the
operator must re-observe, re-plan and click "执行单箱" in the workbench.
The source record is hashed before and after and must stay unchanged.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.site_scene import build_scene
from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.moveit2_backend import ResidentMoveItClient

INPUTS = ('observation.json', 'observation.npz', 'segmentation.npz', 'geometry.json')


def digest(folder):
    return {n: hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in INPUTS if (folder/n).is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', help='task id under outputs/single_carton, or a record folder')
    parser.add_argument('--config', type=Path, default=ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--candidates', help='comma-separated candidate indices (default: all)')
    parser.add_argument('--scene-only', action='store_true', help='rebuild geometry and scene, no native planning')
    parser.add_argument('--detector', choices=('saved', 'config'), default='saved',
                        help="saved: the record's own SAM masks; config: re-run detection + SAM on the saved RGB "
                             "with the configured backend (e.g. the remote GPU server)")
    args = parser.parse_args()
    record = Path(args.record)
    if not record.is_dir(): record = ROOT/'outputs/single_carton'/args.record
    record = record.resolve()
    before = digest(record)
    if not {'observation.json', 'observation.npz', 'segmentation.npz'} <= set(before):
        raise SystemExit('RECORD_HAS_NO_FROZEN_OBSERVATION_AND_MASKS: '+str(record))
    out = (args.output or ROOT/'outputs/single_carton_development'/f'replay-{record.name[:8]}-{datetime.now():%Y%m%dT%H%M%S}').resolve()
    out.mkdir(parents=True, exist_ok=False)
    config = load(args.config)
    obs = json.loads((record/'observation.json').read_text())
    arrays = np.load(record/'observation.npz'); saved = np.load(record/'segmentation.npz')
    masks = tuple(saved[f'mask_{i}'] for i in range(len(saved.files)))
    old = json.loads((record/'geometry.json').read_text()) if (record/'geometry.json').is_file() else {}
    detections = old.get('detections') or [dict(label='saved-mask')]*len(masks)
    frame = SimpleNamespace(observation_id=obs['observation_id'], rgb=arrays['rgb'], depth=arrays['depth'],
                            intrinsics=obs['intrinsics'], depth_scale_m=obs['depth_scale_m'],
                            stamp=SimpleNamespace(source='SAVED_RECORD_OFFLINE_REPLAY'))
    detection_report = None
    if args.detector == 'config':
        from unloading_sim.workbench.single_carton.perception import make_detector
        loading = time.monotonic(); model = make_detector(**config['detector']); load_s = time.monotonic()-loading
        try:
            inferring = time.monotonic(); boxes_only = model.detect_boxes(frame); detect_s = time.monotonic()-inferring
            segmenting = time.monotonic(); result = model.segment(frame, boxes_only); segment_s = time.monotonic()-segmenting
        finally:
            if hasattr(model, 'close'): model.close()
        field = [saved[f'mask_{i}'] for i in range(len(saved.files))]
        def iou(a, b): return float((a & b).sum()/max(1, (a | b).sum()))
        detection_report = dict(backend=config['detector'].get('backend', 'local'), description=getattr(model, 'description', ''),
            model_load_and_verify_s=round(load_s, 2), detect_s=round(detect_s, 3), segment_s=round(segment_s, 3),
            remote=getattr(model, 'info', None), provenance=result.provenance,
            detections=[dict(label=d['label'], score=d['score'], bbox_xyxy=d['bbox_xyxy'], inference=d.get('inference')) for d in result.detections],
            field_detections=[dict(label=d['label'], score=d['score'], bbox_xyxy=d['bbox_xyxy']) for d in old.get('detections', [])],
            mask_iou_vs_field_record=[max((iou(m, f) for f in field), default=0.) for m in result.masks])
        print(json.dumps(dict(detection={k: v for k, v in detection_report.items() if k != 'remote'}), ensure_ascii=False, default=str), flush=True)
        detections, masks = result.detections, tuple(result.masks)
    started = time.monotonic()
    boxes, rejected = recover_boxes(frame, obs['T_world_camera'], detections, masks, config.get('geometry'))
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    authority = Authority('eco65_single_carton_segment_v1')
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), authority, model)
    planner.build()
    scene = build_scene(SimpleNamespace(frame=frame, world_camera=obs['T_world_camera'], q_capture=obs['q_capture']),
                        SimpleNamespace(observation_id=obs['observation_id'], masks=masks), boxes, config['site'],
                        RobotSelfFilter(planner.init['parameters']['robot_description'], model))
    (out/'geometry.json').write_text(json.dumps(dict(boxes=boxes, rejected=rejected), indent=2))
    (out/'site-scene.json').write_text(json.dumps(scene))
    report = dict(scope='OFFLINE_REPLAY_OF_SAVED_RECORD_NOT_EXECUTABLE', record=str(record), task_id=record.name,
                  hardware_execution='NOT_RUN', physical_result='NOT_RUN', permits_issued=0,
                  prior_size_m=config['geometry']['prior_size_m'],
                  boxes=[dict(id=b['id'], size_m=b['size_m'], nominal_size_m=b.get('nominal_size_m'),
                              observed_patch_m=b['quality'].get('observed_patch_size_m'),
                              support_plane=b['quality'].get('observed_support_plane')) for b in boxes],
                  rejected=rejected, scene={k: v for k, v in scene.items() if k != 'world'},
                  scene_objects={c: sum(o['category'] == c for o in scene['world']) for c in {o['category'] for o in scene['world']}},
                  perception_and_scene_s=round(time.monotonic()-started, 2), detector=args.detector, detection=detection_report)
    print(json.dumps(dict(boxes=report['boxes'], rejected=rejected, scene_objects=report['scene_objects']), ensure_ascii=False))
    failures = []; found = None
    try:
        if not args.scene_only and boxes:
            world = json.loads(json.dumps(scene['world']))+[dict(id=b['id'], size=b['size_m'], pose=b['pose'], category='carton') for b in boxes]
            selected = {int(x) for x in args.candidates.split(',')} if args.candidates else None
            planner.client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=out/'moveit.log')
            ready = planner.client.request(planner.init)
            if not ready.get('capabilities', {}).get('bounded_single_carton_planning'):
                raise SystemExit('NATIVE_WORKER_UPDATE_REQUIRED_FOR_BOUNDED_SINGLE_CARTON_PLANNING')
            planner.start_check_pool(out)
            q = np.array(obs['q_start']); planning = time.monotonic()
            progress = lambda text: print(f'[{time.monotonic()-planning:7.1f}s] {text}', flush=True)
            for box in boxes:
                found = planner.search_box(q, world, box, out, progress, lambda: False, failures, selected)
                if found: break
            report['planning_s'] = round(time.monotonic()-planning, 2)
    finally:
        planner.close_check_pool(); report['native_check_pool'] = planner.pool_status
        if planner.client is not None: planner.client.close(); planner.client = None
        if authority._plans: raise SystemExit('OFFLINE_REPLAY_MUST_NOT_REGISTER_PERMITS')
        report['failures'] = failures
        if found:
            report['complete_plan'] = dict(
                status='COMPLETE_FOUR_STAGE_NATIVE_PLAN_OFFLINE_NOT_EXECUTABLE', candidate_index=found['index'],
                endpoint_screen=found['screen']['conclusion'], active_seal_ids=found['candidate']['active_ids'],
                detach_profile=found['segments'][2]['evidence'].get('detach_profile'),
                seal_check=found['seal'], release=found['release'],
                segments=[dict(stage=s['stage'], points=len(s['prepared']['points']),
                               send_duration_s=s['prepared']['send_duration_s'],
                               native_plan=s['evidence']['native_plan'], sent_path_collision=s['evidence']['sent_path_collision'],
                               planner=s['evidence']['planner'],
                               parallel_checkers=s['evidence']['collision_result'].get('parallel_checkers'),
                               sent_check_wall_s=s['evidence']['collision_result'].get('wall_s'))
                          for s in found['segments']])
        after = digest(record)
        report['source_record_unchanged'] = before == after
        (out/'replay-report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str))
        print('report', out/'replay-report.json')
        if before != after: raise SystemExit('SOURCE_RECORD_CHANGED')


if __name__ == '__main__':
    main()
