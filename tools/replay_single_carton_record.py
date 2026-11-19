"""Offline replay of one saved single-carton task record with the current code.

Re-runs box recovery, the frozen real-site scene and the two-phase native
search (the same SinglePlanner.search_box used by the workbench) on the
record's original RGB-D, SAM masks, q_capture and q_start. With
--detector config, detection and SAM are re-run on the saved RGB with the
configured backend (local or the remote GPU server) and compared with the
record's own masks when available. A capture whose inference failed can be
retried explicitly with --detector config; no mask or box is invented.
--frozen-scene reuses the saved geometry and obstacles
instead, for a planning-policy comparison with unchanged field input.

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
import shutil
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner, PlanningBudgetExhausted
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.size_policy import size_policy,neighbor_depth_bound,mark_incomplete_scene,unmodeled_cartons,NO_PRIOR_POLICIES
from unloading_sim.workbench.single_carton.site_scene import build_scene
from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.moveit2_backend import ResidentMoveItClient
from unloading_sim.workbench import timing

INPUTS = ('observation.json', 'observation.npz', 'segmentation.npz', 'geometry.json', 'detections.json')


def digest(folder):
    return {n: hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in INPUTS if (folder/n).is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', help='task id under outputs/single_carton, or a record folder')
    parser.add_argument('--config', type=Path, default=ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--candidates', help='comma-separated candidate indices (default: all)')
    parser.add_argument('--scene-only', action='store_true', help='rebuild geometry and scene, no native planning')
    parser.add_argument('--frozen-scene', action='store_true', help='reuse saved geometry and site-scene for a controlled planning-policy comparison')
    parser.add_argument('--detector', choices=('saved', 'config'), default='saved',
                        help="saved: the record's own SAM masks; config: re-run detection + SAM on the saved RGB "
                             "with the configured backend (e.g. the remote GPU server)")
    parser.add_argument('--planning-deadline-s', type=float,
                        help='unified planning deadline from the planning entry (default: the config value; absent: none)')
    args = parser.parse_args()
    record = Path(args.record)
    if not record.is_dir(): record = ROOT/'outputs/single_carton'/args.record
    record = record.resolve()
    before = digest(record)
    if args.frozen_scene:
        before['site-scene.json']=hashlib.sha256((record/'site-scene.json').read_bytes()).hexdigest()
        if args.detector!='saved':raise SystemExit('FROZEN_SCENE_REQUIRES_SAVED_DETECTOR')
    required = {'observation.json', 'observation.npz'}
    if args.detector == 'saved': required.add('segmentation.npz')
    if not required <= set(before):
        raise SystemExit('RECORD_HAS_NO_FROZEN_OBSERVATION_AND_MASKS: '+str(record))
    out = (args.output or ROOT/'outputs/single_carton_development'/f'replay-{record.name[:8]}-{datetime.now():%Y%m%dT%H%M%S}').resolve()
    out.mkdir(parents=True, exist_ok=False)
    for name in ('observation.json','observation.npz','segmentation.npz','detections.json'):
        if (record/name).is_file(): shutil.copyfile(record/name,out/name)
    replay_started = timing.now()
    config = load(args.config)
    obs = json.loads((record/'observation.json').read_text())
    with np.load(record/'observation.npz', allow_pickle=False) as saved_rgbd:
        arrays = {k: saved_rgbd[k] for k in ('rgb', 'depth')}
    masks = ()
    if (record/'segmentation.npz').is_file():
        with np.load(record/'segmentation.npz', allow_pickle=False) as saved:
            masks = tuple(saved[f'mask_{i}'].astype(bool) for i in range(len(saved.files)))
    old = json.loads((record/'geometry.json').read_text()) if (record/'geometry.json').is_file() else {}
    if not old and (record/'detections.json').is_file():
        old=json.loads((record/'detections.json').read_text())
        if old.get('observation_id')!=obs['observation_id']:
            raise ValueError('SAVED_DETECTIONS_FRAME_MISMATCH')
    detections = old.get('detections') or [dict(label='saved-mask')]*len(masks)
    frame = SimpleNamespace(observation_id=obs['observation_id'], rgb=arrays['rgb'], depth=arrays['depth'],
                            intrinsics=obs['intrinsics'], depth_scale_m=obs['depth_scale_m'],
                            stamp=SimpleNamespace(source='SAVED_RECORD_OFFLINE_REPLAY'))
    detection_report = None
    if args.detector == 'config':
        from unloading_sim.workbench.single_carton.perception import make_detector
        loading = time.monotonic()
        with timing.span('vision.model_load'): model = make_detector(**config['detector'])
        load_s = time.monotonic()-loading
        try:
            inferring = time.monotonic()
            with timing.span('vision.detect'): boxes_only = model.detect_boxes(frame)
            detect_s = time.monotonic()-inferring
            segmenting = time.monotonic()
            with timing.span('vision.segment'): result = model.segment(frame, boxes_only)
            segment_s = time.monotonic()-segmenting
        finally:
            if hasattr(model, 'close'): model.close()
        field = masks
        def iou(a, b): return float((a & b).sum()/max(1, (a | b).sum()))
        detection_report = dict(backend=config['detector'].get('backend', 'local'), description=getattr(model, 'description', ''),
            model_load_and_verify_s=round(load_s, 2), detect_s=round(detect_s, 3), segment_s=round(segment_s, 3),
            remote=getattr(model, 'info', None), provenance=result.provenance,
            detections=[dict(label=d['label'], score=d['score'], bbox_xyxy=d['bbox_xyxy'], inference=d.get('inference')) for d in result.detections],
            field_detections=[dict(label=d['label'], score=d['score'], bbox_xyxy=d['bbox_xyxy']) for d in old.get('detections', [])],
            field_mask_comparison='AVAILABLE' if field else 'NOT_AVAILABLE',
            mask_iou_vs_field_record=[max(iou(m, f) for f in field) for m in result.masks] if field else None)
        print(json.dumps(dict(detection={k: v for k, v in detection_report.items() if k != 'remote'}), ensure_ascii=False, default=str), flush=True)
        detections, masks = result.detections, tuple(result.masks)
        # The output scene and viewer must use the newly inferred masks, not a
        # copy of older masks used only for the optional comparison above.
        np.savez_compressed(out/'segmentation.npz', **{f'mask_{i}': m for i, m in enumerate(masks)})
        timing.write_json(out/'inference.json', detection_report, 'inference.json', indent=2)
    started = time.monotonic()
    with timing.span('observe.reconstruct'):
        if args.frozen_scene:boxes,rejected=old['boxes'],old.get('rejected',[])
        else:boxes, rejected = recover_boxes(frame, obs['T_world_camera'], detections, masks, config.get('geometry'))
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    authority = Authority('eco65_single_carton_segment_v1')
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), authority, model)
    with timing.span('observe.planner_build'): planner.build()
    if args.frozen_scene:scene=json.loads((record/'site-scene.json').read_text())
    else:
        with timing.span('observe.self_filter_construct'):
            self_filter = RobotSelfFilter(planner.init['parameters']['robot_description'], model)
        with timing.span('observe.build_scene'):
            scene = build_scene(SimpleNamespace(frame=frame, world_camera=obs['T_world_camera'], q_capture=obs['q_capture']),
                                SimpleNamespace(observation_id=obs['observation_id'], masks=masks), boxes, config['site'],
                                self_filter,neighbor_depth_m=neighbor_depth_bound(config.get('geometry')),
                                unresolved_cartons=rejected if size_policy(config.get('geometry')) in NO_PRIOR_POLICIES else ())
        mark_incomplete_scene(scene,config.get('geometry'),rejected)
    timing.write_json(out/'geometry.json', dict(boxes=boxes, rejected=rejected, detections=detections), 'geometry.json', indent=2)
    timing.write_json(out/'site-scene.json', scene, 'site-scene.json')
    report = dict(scope='OFFLINE_REPLAY_OF_SAVED_RECORD_NOT_EXECUTABLE', record=str(record), task_id=record.name,
                  hardware_execution='NOT_RUN', physical_result='NOT_RUN', permits_issued=0,
                  size_policy=size_policy(config.get('geometry')),
                  prior_size_m=(config['geometry'].get('prior_size_m') if size_policy(config.get('geometry'))=='CONFIGURED_SINGLE_TYPE' else None),
                  boxes=[dict(id=b['id'], size_m=b['size_m'], nominal_size_m=b.get('nominal_size_m'),
                              observed_patch_m=b['quality'].get('observed_patch_size_m'),
                              support_plane=b['quality'].get('observed_support_plane')) for b in boxes],
                  rejected=rejected, scene={k: v for k, v in scene.items() if k != 'world'},
                  scene_objects={c: sum(o['category'] == c for o in scene['world']) for c in {o['category'] for o in scene['world']}},
                  perception_and_scene_s=round(time.monotonic()-started, 2), detector=args.detector, detection=detection_report)
    report.update(frozen_scene=args.frozen_scene,source_hashes=before,q_start=obs['q_start'],q_capture=obs['q_capture'],
                  planning_policy={k:config.get(k) for k in ('wrist_branch_limits','posture_preference','contact_search','tool_robot_clearance','execution_lock')})
    print(json.dumps(dict(boxes=report['boxes'], rejected=[dict(detection_index=r['detection_index'],reason=r['reason'],
        visibility_bounds=(r.get('evidence') or {}).get('visibility_bounds')) for r in rejected],
        scene_objects=report['scene_objects']), ensure_ascii=False))
    failures = []; found = None
    try:
        if unmodeled_cartons(scene):
            report['geometry_status']='GEOMETRY_UNRESOLVED'
            failures.append(dict(reason='UNBOUNDED_INSTANCE_GEOMETRY',searched=0,unsearched='ALL_NOT_GENERATED'))
        elif not args.scene_only and boxes:
            world = json.loads(json.dumps(scene['world']))+[dict(id=b['id'], size=b['size_m'], pose=b['pose'], category='carton') for b in boxes]
            selected = {int(x) for x in args.candidates.split(',')} if args.candidates else None
            planning_entered = timing.now()
            budget = args.planning_deadline_s if args.planning_deadline_s is not None else config.get('planning_deadline_s')
            deadline = None if budget is None else planning_entered+float(budget)
            with timing.span('native.main_worker_process_start'):
                planner.client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=out/'moveit.log')
            planning = time.monotonic()  # reset below exactly where it was measured before
            progress = lambda text: print(f'[{time.monotonic()-planning:7.1f}s] {text}', flush=True)
            try:
                if deadline is None:
                    planner.worker_ready(timing.native_request(planner.client, planner.init, record='main-init'), boxes)
                else:
                    # Same unified deadline as plan_cycle(deadline=...), from the planning entry.
                    planner.deadline, planner.deadline_reference = deadline, time.monotonic()
                    planner.worker_ready(planner.bounded_request(planner.client, planner.init, 120., None, 'main-init'), boxes)
                report['native_output_check_deferral'] = planner.defer_output_check
                planner.start_check_pool(out)
                q = np.array(obs['q_start']); planning = time.monotonic()
                for box in boxes:
                    with timing.span('plan_cycle.search_box', target=box['id']):
                        if deadline is None: found = planner.search_box(q, world, box, out, progress, lambda: False, failures, selected)
                        else: found = planner.bounded_search(q, world, box, out, progress, lambda: False, failures, selected)
                    if box['id'] in planner.search_counts:
                        planner.search_counts[box['id']].update(phase='DONE' if found is None else 'SEARCH_SUCCEEDED',
                                                              accepted=None if found is None else found['index'])
                    if found: break
                if deadline is not None:
                    report['planning_budget'] = planner.budget_summary(boxes, failures, None, None)
            except PlanningBudgetExhausted as error:
                report['planning_budget'] = planner.budget_summary(boxes, failures, None, error)
                report['planning_budget']['returned_after_deadline_s'] = time.monotonic()-deadline
                print('PLANNING_BUDGET_EXHAUSTED', json.dumps(report['planning_budget']['targets'], ensure_ascii=False), flush=True)
            report['planning_s'] = round(time.monotonic()-planning, 2)
            if found and timing.ENABLED:
                measure_plan_tail(planner, found, box, world, scene, obs, out)
            # The enclosing interval ends after blocking worker cleanup below.
    finally:
        with timing.span('plan_cycle.close_workers'):
            if planner.deadline is None:
                planner.close_check_pool(); report['native_check_pool'] = planner.pool_status
                if planner.client is not None: planner.client.close(); planner.client = None
            else:
                report['native_check_pool'] = planner.pool_status
                planner.background_close(out); planner.deadline = None
        if 'planning_entered' in locals():
            timing.add('replay.planning_entry_to_search_end',planning_entered,timing.now()-planning_entered,scope='PARTIAL_REPLAY_NOT_PREPARE_PLAN')
        if authority._plans: raise SystemExit('OFFLINE_REPLAY_MUST_NOT_REGISTER_PERMITS')
        report['failures'] = failures
        (out/'planning-failures.json').write_text(json.dumps(failures,indent=2,ensure_ascii=False,default=str))
        if found:
            report['complete_plan'] = dict(
                status='COMPLETE_FOUR_STAGE_NATIVE_PLAN_OFFLINE_NOT_EXECUTABLE', target_id=box['id'], candidate_index=found['index'],
                endpoint_screen=found['screen']['conclusion'], active_seal_ids=found['candidate']['active_ids'],
                candidate_lanes=found.get('lanes', 1),
                detach_profile=found['segments'][2]['evidence'].get('detach_profile'),
                seal_check=found['seal'], release=found['release'],
                segments=[dict(stage=s['stage'], points=len(s['prepared']['points']),
                               send_duration_s=s['prepared']['send_duration_s'],
                               native_plan=s['evidence']['native_plan'], sent_path_collision=s['evidence']['sent_path_collision'],
                               planner=s['evidence']['planner'],
                               native_output_check=s['evidence']['planner'].get('native_output_check'),
                               parallel_checkers=s['evidence']['collision_result'].get('parallel_checkers'),
                               sent_check_wall_s=s['evidence']['collision_result'].get('wall_s'))
                          for s in found['segments']])
        after = digest(record)
        if args.frozen_scene:after['site-scene.json']=hashlib.sha256((record/'site-scene.json').read_bytes()).hexdigest()
        report['source_record_unchanged'] = before == after
        (out/'replay-report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str))
        print('report', out/'replay-report.json')
        if timing.ENABLED:
            # Measurement only, after the replay: all spans of this run.
            events, overhead = timing.recorder.take(replay_started)
            (out/'timing-replay.json').write_text(json.dumps(dict(
                schema='eco65_timing_v1', phase='offline_replay', clock='time.monotonic', task_id=record.name,
                measurement_overhead_s=overhead, events=events), default=str))
        if before != after: raise SystemExit('SOURCE_RECORD_CHANGED')


def measure_plan_tail(planner, found, box, world, scene, obs, out):
    """Time what ``plan_cycle`` does after the search, without minting any permit.

    The same binding rehash, the digest that ``Authority.register`` takes of each
    segment package, the ``complete-plan.json`` serialization/write and the
    ``Cycle.ready`` plan digest, on an equivalently shaped plan. Nothing is
    registered; the written measurement file is removed at once. The segments
    are kept for the fake-IO scheduling measurement as NOT_EXECUTABLE data.
    """
    from unloading_sim.workbench.controlled.trajectory import digest as content_digest
    from unloading_sim.workbench.single_carton.config import start_joint_check
    candidate = found['candidate']; attachment = found['attachment']; release = found['release']
    size = np.array(box['size_m']); mass = planner.site['carton_mass_kg']
    planner.cycle_binding = dict(task_id=Path(out).name, observation=obs, target=box, world=content_digest(world),
        candidate=candidate, attachment=np.asarray(attachment).tolist(), release=release,
        expected_attachment='PLANNED_NOT_PHYSICALLY_CONFIRMED', site_scene=scene, release_world=world,
        payload_mass_properties=dict(mass_kg=mass, com_box_m=[0., 0., 0.],
            inertia_box_kg_m2=(mass/12*np.diag([size[1]**2+size[2]**2, size[0]**2+size[2]**2, size[0]**2+size[1]**2])).tolist(),
            basis='USER_CONFIRMED_2_KG_UNIFORM_CUBOID_INERTIA_ESTIMATE'))
    try:
        with timing.span('plan_cycle.binding'): binding = planner.binding()
        segments = []
        for s in found['segments']:
            package = dict(schema=planner.authority.schema, session='MEASUREMENT_ONLY_NO_SESSION', plan_id='0'*32,
                           prepared=s['prepared'], binding=binding, evidence=s['evidence'])
            with timing.span('authority.register_digest', stage=s['stage'], equivalent=True): content_digest(package)
            segments.append(dict(stage=s['stage'], package=package))
        start_policy = start_joint_check(planner.site)
        plan = dict(binding=binding, segments=segments, target=box, candidate=candidate, release=release,
                    start_joint_check=start_policy, q_start=list(obs['q_start']), T_world_tcp_start=obs.get('T_world_tcp_start'),
                    full_path_validated=True, candidate_index=found['index'], endpoint_screen=found['screen']['conclusion'],
                    candidate_lanes=found.get('lanes', 1), hardware_execution='NOT_RUN',
                    duration_s=sum(s['prepared']['send_duration_s'] for s in found['segments']))
        probe = Path(out)/'tail-measurement-NOT-A-PLAN.json'
        timing.write_json(probe, plan, 'complete-plan.json'); probe.unlink()
        with timing.span('prepare_plan.cycle_ready', equivalent=True): content_digest(plan)
        for s in segments:
            with timing.span('child.submit.trajectory_package_json_dumps', stage=s['stage'], equivalent=True):
                text = json.dumps(s['package'], indent=2, allow_nan=False)
            timing.add('child.submit.trajectory_package_bytes', timing.now(), 0., stage=s['stage'], bytes=len(text), equivalent=True)
        (Path(out)/'measurement-segments-NOT-EXECUTABLE.json').write_text(json.dumps(dict(
            status='MEASUREMENT_INPUT_NOT_EXECUTABLE_NO_PERMIT', binding=binding, candidate=candidate, target=box,
            release=release, q_start=list(obs['q_start']), attachment=np.asarray(attachment).tolist(),
            segments=[dict(stage=s['stage'], prepared=s['prepared'], evidence=s['evidence']) for s in found['segments']]),
            default=str))
    finally:
        planner.cycle_binding = None


if __name__ == '__main__':
    main()
