"""Missing boundary depth and uncertain geometry are not physical penetration."""
import json
import cv2
import numpy as np
import pytest

from unloading_sim.workbench.single_carton.geometry import corners,transform
from unloading_sim.workbench.single_carton.partial_geometry import recover_partial
from test_single_carton_variable_geometry import measured_box
from test_single_carton_planner import case,wall_near


@pytest.mark.parametrize('angles',[(0,0,0),(10,30,17),(0,0,42)])
def test_missing_boundary_depth_keeps_full_true_cuboid_and_records_intervals(angles):
    size=[.24,.18,.12];points,mask,center,rotation=measured_box(size,angles)
    # SAM remains exact, while the stereo depth is absent on both sides of
    # the image silhouette. No depth is filled in by the recovery algorithm.
    band=cv2.dilate(mask.astype('uint8'),np.ones((7,7),'uint8')).astype(bool)&~cv2.erode(mask.astype('uint8'),np.ones((7,7),'uint8')).astype(bool)
    points[band]=np.nan
    box=recover_partial(points,mask,np.eye(4),prior=size)
    actual=np.eye(4);actual[:3,:3]=rotation;actual[:3,3]=center
    local=transform(np.linalg.inv(box['pose']),corners(actual,size))
    assert np.all(abs(local)<=np.asarray(box['size_m'])/2+.003)
    intervals=[e for e in box['quality']['credible_visible_edges'] if 'DEPTH_GAP' in e['basis']]
    assert intervals and all(e['position_uncertainty_m']>0 for e in intervals)
    assert not box['uncertainty']['unique_pose'] and not box['quality']['hidden_extent_measured']


def test_nearer_occluder_is_not_a_background_supported_edge():
    from unloading_sim.workbench.single_carton.partial_geometry import credible_edges
    y,x=np.indices((120,220));rays=np.stack(((x-110)/500,(y-60)/500,np.ones_like(x)),-1)
    mask=(x>=108)&(x<=200)&(y>=10)&(y<=109)
    depth=np.where(mask,.7,1.4);depth[:,102:108]=.5
    points=rays*depth[...,None]
    plane=dict(normal=np.array([0.,0.,-1.]),center=np.array([0.,0.,.7]))
    R=np.diag([1.,-1.,-1.]);origin=np.array([0.,0.,.7])
    edges=credible_edges(points,mask,plane,R,origin,6518)
    # True right/top/bottom edges are permitted. The left line has a closer
    # six-pixel occluder before the background and must supply no edge bound.
    for e in edges:
        if abs(e['direction_uv'][1])>.98:
            assert e['center_uv_m'][0]>.05


def test_uncertainty_envelope_overlap_does_not_claim_all_hypotheses_infeasible(case):
    p,obs,box,folder,calls,opts=case
    opts['capabilities']=dict(partial_observed_contact_envelope=True)
    box['uncertainty']=dict(kind='BOUNDED_CUBOID_UNION',hypotheses=[dict(diagnostic_fixture=True)])
    p.world=[wall_near(box,.005)]
    with pytest.raises(ValueError,match='GEOMETRY_UNCERTAINTY_OVERLAP_AT_REST') as error:
        p.plan_cycle(obs,[box],folder)
    assert '所有候选都无法脱离' not in str(error.value)
    assert not [c for c in calls if c.get('op') in {'ik','plan','validate'}] and not p.authority._plans
    rest=json.loads((folder/(box['id']+'-payload-rest-check.json')).read_text())
    assert rest['intersections'] and not rest['measured_volume_overlap_proven']
    assert not rest['all_hypotheses_infeasible_proven']
    failures=json.loads((folder/'planning-failures.json').read_text())
    budget=p.budget_summary([box],failures,None,None)['targets'][0]
    assert budget['status']=='GEOMETRY_UNRESOLVED' and budget['searched']==budget['infeasible_candidates']==0
    assert budget['unsearched']=='ALL_NOT_GENERATED'
