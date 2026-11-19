"""The live prior-free route measures each instance or keeps explicit gaps."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.config import load,issues
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.size_policy import neighbor_depth_bound,mark_incomplete_scene
from test_single_carton_variable_geometry import measured_box


MEASURED=dict(size_policy='MEASURED_PER_INSTANCE',size_policy_basis='SYNTHETIC_PRIOR_FREE_MEASUREMENT')


def recover_instances(monkeypatch,points,masks,geometry):
    monkeypatch.setattr('unloading_sim.workbench.single_carton.perception.metric_points',lambda _:points)
    def no_prior(*a,**kw):pytest.fail('Prior-free mode called prior-based recovery')
    monkeypatch.setattr('unloading_sim.workbench.single_carton.partial_geometry.recover_partial',no_prior)
    frame=SimpleNamespace(rgb=np.zeros(points.shape,dtype=np.uint8),observation_id='prior-free-fixture')
    return recover_boxes(frame,np.eye(4),[{} for _ in masks],masks,geometry)


def test_two_different_complete_boxes_are_measured_independently_of_historical_prior(monkeypatch):
    sizes=[(.24,.18,.12),(.32,.21,.16)]
    a,ma,_,_=measured_box(sizes[0]);b,mb,_,_=measured_box(sizes[1])
    # Disjoint pixel instances; metric rays remain their literal fixture rays.
    points=np.concatenate([a,b],axis=1)
    masks=[np.concatenate([ma,np.zeros_like(mb)],axis=1),np.concatenate([np.zeros_like(ma),mb],axis=1)]
    results=[]
    for stale in ([.17,.128,.101],[.01,1.8,.6],None):
        geometry=dict(MEASURED,prior_size_m=stale,dimension_tolerance_m=-9,prior_measurement_basis=None)
        boxes,rejected=recover_instances(monkeypatch,points,masks,geometry)
        assert not rejected and len(boxes)==2
        for box,size in zip(boxes,sizes):
            np.testing.assert_allclose(sorted(box['size_m']),sorted(size),atol=.006)
            assert not box['prior_applied'] and box['size_policy']=='MEASURED_PER_INSTANCE'
            assert box['dimension_source']=='RGBD_VISIBLE_FACE_AND_SIDE_BOUNDARIES_NO_SIZE_PRIOR'
        results.append(boxes)
        assert neighbor_depth_bound(geometry) is None
    assert results[0]==results[1]==results[2]


@pytest.mark.parametrize('case',['single_face','image_clip','occlusion','depth_hole'])
def test_measured_route_preserves_partial_contact_without_inventing_hidden_size(monkeypatch,case):
    p,m,_,_=measured_box(angles=(0,0,0))
    if case=='image_clip':p=p[:,175:];m=m[:,175:]
    elif case=='occlusion':m[:,:175]=False;p[:,:175,2]=.4
    elif case=='depth_hole':p[160:170,180:190]=np.nan
    boxes,rejected=recover_instances(monkeypatch,p,[m],MEASURED)
    assert not boxes and len(rejected)==1
    e=rejected[0]['evidence']
    assert rejected[0]['reason']=='BOX_THICKNESS_UNOBSERVED_SHOW_SIDE'
    assert e['geometry_state']=='OBSERVED_SURFACE_ONLY' and not e['prior_applied']
    assert not e['full_extent_bounded'] and e['full_dimensions_m'] is None
    c=load(Path('configs/workbench/single_carton.example.json'))
    tool=json.loads(Path('configs/local/eco65_desktop/tool.json').read_text())
    candidates=contact_candidates(dict(faces=[e['observed_face']]),tool,c['topology'])
    assert candidates and all(x['seal_check']['observed_surface_covered'] for x in candidates)


def test_measured_configuration_requires_basis_but_not_a_size_prior():
    c=load(Path('configs/workbench/single_carton.example.json'))
    c['geometry']=deepcopy(MEASURED)
    assert not any(x.startswith('geometry.') for x in issues(c))
    for value in ('',None,12):
        c['geometry']['size_policy_basis']=value
        assert any(x.startswith('geometry.size_policy') for x in issues(c))


def test_measured_scene_keeps_unbounded_neighbors_and_marks_complete_scene_separately():
    scene=dict(world=[dict(id='observed-neighbor')])
    mark_incomplete_scene(scene,MEASURED,[dict(detection_index=1)])
    assert scene['unbounded_carton_indices']==[1] and scene['geometry_qualification']=='INCOMPLETE_INSTANCE_EXTENTS'
    assert scene['world']==[dict(id='observed-neighbor')] and not scene['hidden_carton_volume_assumed_clear']
    mark_incomplete_scene(scene,MEASURED,[])
    assert scene['unbounded_carton_indices']==[] and scene['geometry_qualification']=='MEASURED_INSTANCE_EXTENTS'
