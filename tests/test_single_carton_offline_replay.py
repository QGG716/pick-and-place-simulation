"""A failed field inference can be retried without borrowing an older mask."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.mark.parametrize('record_kind', ['no_mask','old_mask','detections_only','wrong_detection_frame'])
def test_explicit_reinference_persists_the_masks_used_for_geometry(tmp_path, monkeypatch, record_kind):
    saved_only=record_kind in ('detections_only','wrong_detection_frame')
    has_old_mask=record_kind=='old_mask'
    spec = importlib.util.spec_from_file_location('offline_replay', Path(__file__).resolve().parents[1]/'tools/replay_single_carton_record.py')
    replay = importlib.util.module_from_spec(spec); spec.loader.exec_module(replay)
    record = tmp_path/'field'; record.mkdir(); out = tmp_path/'retry'
    observed = dict(observation_id='new-field', q_start=[.1]*6, q_capture=[.1001]*6,
                    T_world_camera=np.eye(4).tolist(), intrinsics={}, depth_scale_m=.001)
    (record/'observation.json').write_text(json.dumps(observed))
    rgb = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)
    np.savez(record/'observation.npz', rgb=rgb, depth=np.ones((3, 3), np.uint16))
    if has_old_mask: np.savez(record/'segmentation.npz', mask_0=np.zeros((3, 3), bool))
    mask = np.eye(3, dtype=bool)
    detections = [dict(label='box', score=.8, bbox_xyxy=[0, 0, 3, 3])]
    if saved_only:
        np.savez(record/'segmentation.npz',mask_0=mask)
        (record/'detections.json').write_text(json.dumps(dict(
            observation_id='wrong' if record_kind=='wrong_detection_frame' else 'new-field',detections=detections)))
    source_hashes = replay.digest(record)
    seen = []
    class Detector:
        def detect_boxes(self, frame):
            assert not saved_only,'A saved inference must not load or run a model'
            np.testing.assert_array_equal(frame.rgb, rgb)
            return SimpleNamespace(observation_id=frame.observation_id, detections=detections)
        def segment(self, frame, boxes):
            assert frame.observation_id == boxes.observation_id == 'new-field'
            return SimpleNamespace(masks=[mask], detections=detections, provenance='FAKE_IO')
    from unloading_sim.workbench.single_carton import perception
    monkeypatch.setattr(perception, 'make_detector', lambda **kw: Detector())
    config = dict(detector=dict(backend='local'), geometry=dict(prior_size_m=[.15, .1, .1]),
                  execution=dict(installation='fake', mapping='fake'),
                  site=dict(workspace_bounds_m=[[-.8,-.8,-.15],[.8,.8,1.2]]))
    planner = SimpleNamespace(build=lambda: None, init=dict(parameters=dict(robot_description='fake')),
                              deadline=None, close=lambda: None, resident=None)
    monkeypatch.setattr(replay, 'load', lambda _: config)
    monkeypatch.setattr(replay.Installation, 'load', lambda _: SimpleNamespace(world_from_base=np.eye(4), tool=None))
    monkeypatch.setattr(replay, 'CoordinateModel', lambda *a: None)
    monkeypatch.setattr(replay, 'load_mapping', lambda _: None)
    monkeypatch.setattr(replay, 'SinglePlanner', lambda *a: planner)
    monkeypatch.setattr(replay, 'RobotSelfFilter', lambda *a: None)
    def recover(frame, camera, boxes, masks, policy, workspace_bounds):
        assert boxes==detections
        assert workspace_bounds==config['site']['workspace_bounds_m']
        np.testing.assert_array_equal(masks[0], mask); seen.append('recover')
        return [], []
    def scene(observation, result, *args, **kwargs):
        np.testing.assert_array_equal(result.masks[0], mask)
        assert observation.q_capture == observed['q_capture']; seen.append('scene')
        return dict(world=[])
    monkeypatch.setattr(replay, 'recover_boxes', recover)
    monkeypatch.setattr(replay, 'build_scene', scene)
    monkeypatch.setattr(sys, 'argv', ['replay', str(record), '--output', str(out), '--detector', 'saved' if saved_only else 'config', '--scene-only'])
    if record_kind=='wrong_detection_frame':
        with pytest.raises(ValueError,match='SAVED_DETECTIONS_FRAME_MISMATCH'):replay.main()
        assert not seen and source_hashes==replay.digest(record)
        return
    replay.main()
    with np.load(out/'segmentation.npz') as saved: np.testing.assert_array_equal(saved['mask_0'], mask)
    assert seen == ['recover', 'scene'] and source_hashes == replay.digest(record)
    report = json.loads((out/'replay-report.json').read_text())
    assert report['q_start'] == observed['q_start'] and report['q_capture'] == observed['q_capture']
    assert report['physical_result'] == 'NOT_RUN' and report['permits_issued'] == 0
    if saved_only:assert report['detection'] is None
    else:assert report['detection']['mask_iou_vs_field_record'] == ([0.] if has_old_mask else None)
    assert json.loads((out/'geometry.json').read_text())['detections'] == detections


@pytest.mark.parametrize('resident,prewarm,acquire_error', [
    (False, False, False), (True, False, False), (True, True, False), (True, True, True)])
def test_replay_uses_production_worker_lifecycle(tmp_path, monkeypatch, resident, prewarm, acquire_error):
    """A configured resident run must acquire/reset/release the real task pool.

    The old replay created its own client, silently ignoring this switch and
    scene/evidence services; a cold replay also must not invent a warm result.
    """
    spec = importlib.util.spec_from_file_location('offline_lifecycle', Path(__file__).resolve().parents[1]/'tools/replay_single_carton_record.py')
    replay = importlib.util.module_from_spec(spec); spec.loader.exec_module(replay)
    record = tmp_path/'field'; record.mkdir(); out = tmp_path/'retry'
    obs = dict(observation_id='field', q_start=[.1]*6, q_capture=[.1]*6,
               T_world_camera=np.eye(4).tolist(), intrinsics={}, depth_scale_m=.001)
    box = dict(id='box', size_m=[.1]*3, pose=np.eye(4).tolist(), quality={})
    for name, value in [('observation.json', obs), ('geometry.json', dict(boxes=[box])),
                        ('site-scene.json', dict(world=[]))]:
        (record/name).write_text(json.dumps(value))
    np.savez(record/'observation.npz', rgb=np.zeros((3,3,3),np.uint8), depth=np.ones((3,3),np.uint16))
    np.savez(record/'segmentation.npz', mask_0=np.ones((3,3),bool))
    config = dict(resident_workers=resident, scene_handles=True, planning_deadline_s=20.,
                  detector={}, geometry={}, execution=dict(installation='fake',mapping='fake'),
                  site={}, collision_check_mode='TWO_LEVEL', sent_path_audit='ADAPTIVE')
    calls = []
    pool = SimpleNamespace(wait_ready=lambda timeout: calls.append('warm'),
                           wait_closed=lambda timeout: calls.append('joined') or True)
    class Planner:
        deadline = None; evidence = None; task_start = None; pool_status = []; search_counts = {}
        defer_output_check = True
        def __init__(self): self.resident = None
        def build(self): pass
        def resident_pool(self): self.resident = pool; return pool
        def start_task_services(self, world, folder):
            assert world[0]['id']=='box' and self.deadline > replay.time.monotonic()
            calls.append('services')
        def start_main_worker(self, folder): calls.append('start')
        def acquire_workers(self, folder, boxes):
            calls.append('acquire')
            assert boxes==[box]
            if resident:
                self.resident = pool; self.task_start = dict(already_reset=['main-0'])
            if acquire_error: raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_TWO_LEVEL_COLLISION')
        def bounded_search(self, *args): calls.append('search'); return None
        def budget_summary(self, *args): return dict(status='COMPLETE',targets=[])
        def release_workers(self, folder): calls.append('release')
        def close(self): calls.append('close')
    planner = Planner()
    monkeypatch.setattr(replay, 'load', lambda _: config)
    monkeypatch.setattr(replay.Installation,'load',lambda _:SimpleNamespace(world_from_base=np.eye(4),tool=None))
    monkeypatch.setattr(replay,'CoordinateModel',lambda *a:None)
    monkeypatch.setattr(replay,'load_mapping',lambda _:None)
    monkeypatch.setattr(replay,'SinglePlanner',lambda *a:planner)
    monkeypatch.setattr(replay.timing,'ENABLED',False)
    argv = ['replay',str(record),'--output',str(out),'--frozen-scene']
    if prewarm: argv.append('--prewarm-workers')
    monkeypatch.setattr(sys,'argv',argv)
    before = replay.digest(record)
    if acquire_error:
        with pytest.raises(ValueError,match='NATIVE_WORKER_UPDATE_REQUIRED'): replay.main()
    else: replay.main()
    assert calls == (['warm'] if prewarm else []) + ['services','start','acquire'] + (
        [] if acquire_error else ['search']) + ['release','close'] + (['joined'] if resident else [])
    report = json.loads((out/'replay-report.json').read_text())
    assert report['planning_mechanisms']['resident_workers'] is resident
    assert report['planning_mechanisms']['worker_start']==('PREWARMED' if prewarm else 'COLD')
    assert ('worker_prewarm_s' in report) is prewarm
    assert report['resident_task_start']==(dict(already_reset=['main-0']) if resident else None)
    assert report['permits_issued']==0 and report['physical_result']=='NOT_RUN'
    assert replay.digest(record)==before and report['source_record_unchanged']
    assert Path(config['record_root']).is_relative_to(out)


def test_target_level_view_keeps_neighbor_and_its_own_mask(tmp_path, monkeypatch):
    tools=Path(__file__).resolve().parents[1]/'tools';monkeypatch.syspath_prepend(str(tools))
    import visualize_field_planning_failures as viewer
    monkeypatch.setattr(viewer,'ROOT',tmp_path)
    record=tmp_path/'record';record.mkdir()
    obs=dict(observation_id='fresh',q_start=[.1]*6,q_capture=[.1001]*6,stream={},intrinsics={},T_world_camera=np.eye(4).tolist())
    boxes=[dict(id=f'box-{i}',pose=np.eye(4).tolist(),size_m=[.1]*3,detection_index=i) for i in range(2)]
    failure=dict(target='box-1',reason='TARGET_PAYLOAD_INTERSECTS_AT_REST',evidence=dict(intersections=[dict(id='box-0',overlap_depth_m=.02)]))
    values={'observation.json':obs,'geometry.json':dict(boxes=boxes),'site-scene.json':dict(world=[]),
            'planning-failures.json':[failure],'replay-report.json':dict(task_id='fresh',planning_s=.2,
                complete_plan=dict(target_id='box-0',candidate_index=0))}
    for name,value in values.items():(record/name).write_text(json.dumps(value))
    np.savez(record/'observation.npz',rgb=np.zeros((2,2,3),np.uint8))
    np.savez(record/'segmentation.npz',mask_0=np.zeros((2,2),bool),mask_1=np.ones((2,2),bool))
    case=viewer.read_case(record,SimpleNamespace(frames=lambda q:dict(base_link=np.eye(4))),'box-1')
    assert case['box']['id']=='box-1' and [b['id'] for b in case['world']]==['box-0']
    assert case['target_failures']==[failure] and case['q_start']==obs['q_start']
    assert case['default_candidate'] is None  # A different target's success is not this target's plan.
    assert case['candidates'][0]['index'] is None and case['candidates'][0]['states']==case['candidates'][0]['paths']==[]
    from PIL import Image
    import io,base64
    shown=np.array(Image.open(io.BytesIO(base64.b64decode(case['image'].split(',')[1]))))
    np.testing.assert_array_equal(shown[0,0],[12,88,64])
