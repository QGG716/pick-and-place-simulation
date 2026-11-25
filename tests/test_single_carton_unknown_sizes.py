"""Unknown mixed cartons preserve contact evidence, never inherit a stale size."""
from types import SimpleNamespace
import json
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.observed_surface import observed_surface
from unloading_sim.workbench.single_carton.size_policy import size_policy,neighbor_depth_bound,mark_incomplete_scene
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.config import load,issues
from test_single_carton_geometry import plane
from test_single_carton_variable_geometry import measured_box
from test_single_carton_planner import case

UNKNOWN=dict(size_policy='UNKNOWN_PER_INSTANCE',size_policy_basis='SYNTHETIC_MIXED_UNKNOWN_CARTONS',
             prior_size_m=[.17,.128,.101],dimension_tolerance_m=.008)


def test_two_different_surface_sizes_do_not_inherit_common_prior(monkeypatch):
    a,ma=plane(.24,.18);b,mb=plane(.15,.10);b[...,0]+=.35
    points=np.concatenate([a,b],axis=1)
    masks=[np.concatenate([ma,np.zeros_like(mb)],axis=1),np.concatenate([np.zeros_like(ma),mb],axis=1)]
    monkeypatch.setattr('unloading_sim.workbench.single_carton.perception.metric_points',lambda _:points)
    monkeypatch.setattr('unloading_sim.workbench.single_carton.partial_geometry.recover_partial',
                        lambda *a,**k:pytest.fail('Unknown carton inherited a size prior'))
    f=SimpleNamespace(rgb=np.zeros(points.shape,dtype=np.uint8),observation_id='two-different-cartons')
    boxes,rejected=recover_boxes(f,np.eye(4),[{},{}],masks,UNKNOWN)
    assert not boxes and len(rejected)==2
    for r,expected in zip(rejected,([.24,.18],[.15,.1])):
        e=r['evidence'];assert r['reason']=='INSTANCE_FULL_EXTENT_UNBOUNDED'
        assert not e['prior_applied'] and not e['full_extent_bounded'] and e['full_dimensions_m'] is None
        assert not e['observed_extent_is_carton_size']
        np.testing.assert_allclose(sorted(e['observed_patch_size_m']),sorted(expected),atol=.006)
    assert neighbor_depth_bound(UNKNOWN)==.4
    legacy=dict(UNKNOWN,size_policy='CONFIGURED_SINGLE_TYPE')
    assert neighbor_depth_bound(legacy)==pytest.approx(np.linalg.norm(legacy['prior_size_m'])+.008)


@pytest.mark.parametrize('occlusion',['none','image_clip','neighbor_occlusion'])
def test_unknown_size_keeps_real_seal_candidates_on_partial_observed_face(occlusion):
    p,m,_,_=measured_box(angles=(0,0,0))
    if occlusion=='image_clip':p=p[:,175:];m=m[:,175:]
    if occlusion=='neighbor_occlusion':m[:, :175]=False
    e=observed_surface(p,m,np.eye(4));assert not e['full_extent_bounded']
    tool=json.loads(Path('configs/local/eco65_desktop/tool.json').read_text())
    c=load(Path('configs/workbench/single_carton.example.json'), code_defaults=False)
    candidates=contact_candidates(dict(faces=[e['observed_face']]),tool,c['topology'])
    assert candidates and all(x['seal_check']['observed_surface_covered'] for x in candidates)
    assert not any(k in e for k in ('pose','corners_m','size_m'))


def test_unknown_policy_requires_its_own_basis_and_rejects_unknown_enum():
    c=load(Path('configs/workbench/single_carton.example.json'), code_defaults=False)
    for value in (None,'',12):
        broken=dict(c,geometry=dict(UNKNOWN,size_policy_basis=value))
        assert any(x.startswith('geometry.size_policy') for x in issues(broken))
    with pytest.raises(ValueError,match='POLICY_INVALID'):size_policy(dict(size_policy='GUESS'))


def test_incomplete_neighbor_volume_cannot_be_used_as_a_clear_planning_scene(case):
    p,obs,box,folder,calls,opts=case
    obs.frame.observation_id='synthetic-unbounded-neighbor'
    scene=dict(world=[],observation_id=obs.frame.observation_id)
    mark_incomplete_scene(scene,UNKNOWN,[dict(detection_index=1)])
    assert not scene['hidden_carton_volume_assumed_clear']
    with pytest.raises(ValueError,match='UNBOUNDED_INSTANCE_GEOMETRY'):p.plan_cycle(obs,[box],folder,scene=scene)
    assert not calls and not p.authority._plans


@pytest.mark.parametrize('has_measured_neighbor,scene_failure,async_evidence,modeled',[
    (False,False,False,False),(True,False,False,False),(True,True,False,False),
    (True,True,True,False),(True,False,False,True)])
def test_prepare_retains_surface_candidates_and_reports_unresolved_without_planning(monkeypatch,tmp_path,has_measured_neighbor,scene_failure,async_evidence,modeled):
    from unloading_sim.workbench.single_carton import runtime,site_scene,self_filter
    from unloading_sim.workbench.controlled.trajectory import Authority
    from test_single_carton_observation import observation
    from eco65_test_inputs import use_synthetic_remote_detector
    obs=observation();p,m=plane(.24,.18);e=observed_surface(p,m,np.eye(4))
    rejected=[dict(detection_index=0,reason='INSTANCE_FULL_EXTENT_UNBOUNDED',evidence=e)]
    c=use_synthetic_remote_detector(load(Path('configs/workbench/single_carton.example.json'), code_defaults=False),tmp_path)
    if not has_measured_neighbor:c['geometry']=dict(UNKNOWN)
    c.update(record_root=str(tmp_path/'records'),async_evidence=async_evidence)
    result=SimpleNamespace(detections=[dict(label='box')],masks=[np.ones(obs.frame.rgb.shape[:2],bool)])
    vision=SimpleNamespace(infer_frozen=lambda *a,**k:result)
    backend=SimpleNamespace(vision=vision,observation_generation=obs.generation)
    planner=SimpleNamespace(authority=Authority('eco65_single_carton_segment_v1'),build=lambda:None,
        init=dict(parameters=dict(robot_description='fake')),source=dict(tool=json.loads(Path('configs/local/eco65_desktop/tool.json').read_text())))
    seen=[]
    def scene(*args,**kwargs):
        if scene_failure:raise TypeError('SYNTHETIC_SCENE_SERIALIZATION_FAILURE')
        seen.append(kwargs['neighbor_depth_m'])
        records=([dict(detection_index=0,
            scope='STATIC_OBSTACLE_INTERSECTION_WITH_ENFORCED_WORKSPACE_NOT_PAYLOAD',
            sensor_free_cells_excluded=True,user_clearance_applied=False)] if modeled else [])
        assert kwargs['unresolved_cartons']==rejected
        return dict(world=[dict(id='observed-obstacle')],unresolved_carton_obstacles=records)
    monkeypatch.setattr(runtime,'freeze',lambda *a,**k:obs)
    measured=[dict(id='measured-other',detection_index=1)] if has_measured_neighbor else []
    monkeypatch.setattr(runtime,'reconstruct',lambda *a,**k:(measured,rejected))
    monkeypatch.setattr(site_scene,'build_scene',scene)
    monkeypatch.setattr(self_filter,'RobotSelfFilter',lambda *a:None)
    manager=runtime.TaskManager(backend,None,SimpleNamespace(holding=False),planner,None,None,c)
    try:
        if scene_failure:
            with pytest.raises(TypeError,match='SYNTHETIC_SCENE_SERIALIZATION_FAILURE'):manager.observe()
            manager.evidence_flush()
            with np.load(manager.folder/'segmentation.npz',allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved['mask_0'],result.masks[0])
            saved=json.loads((manager.folder/'detections.json').read_text())
            assert saved['observation_id']==obs.frame.observation_id and saved['detections']==result.detections
            assert not planner.authority._plans and manager.plan is None and manager.phase=='FAILED'
            return
        manager.observe();manager.journal.flush()
        assert manager.phase==('TARGET_READY' if has_measured_neighbor else 'GEOMETRY_UNRESOLVED') and not manager.busy and seen==[.4]
        assert manager.scene['world']==[dict(id='observed-obstacle')]
        assert manager.scene['unbounded_carton_indices']==[0] and not manager.plan
        assert manager.boxes==measured
        assert f'可规划目标 {len(measured)} 箱' in manager.reason
        if not has_measured_neighbor:
            with pytest.raises(ValueError,match='请先完成有效观测'):manager.prepare_plan()
        stored=json.loads((manager.folder/'geometry.json').read_text())['rejected'][0]['evidence']
        assert stored['contact_candidate_check']['count']>0 and stored['contact_candidate_check']['all_observed_coverage']
        assert not planner.authority._plans
    finally:
        manager.closing.set();manager.thread.join(1)
        if manager.evidence is not None:manager.evidence.close()
        if manager.journal:manager.journal.close()
