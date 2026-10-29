from dataclasses import replace
import numpy as np
import pytest
from unloading_sim.geometry import OBB
from unloading_sim.concept_solid import relieved_transverse,ConceptSolid
from unloading_sim.xy_base_continuation import xy_path,relocation_world,check_plinth_world,relocation_receipt
from unloading_sim.joint_search_layout import LayoutCandidate
from unloading_sim.moveit2_backend import box_message,digest


def world():
    return [OBB(np.array([-1.95,.35,.3]),np.array([1.05,.75,.3]),np.eye(3),'chassis','chassis'),
        OBB(np.array([-2.00675,.15,.75]),np.array([.28175,.275,.15]),np.eye(3),'mount_plinth','mount_structure'),
        OBB(np.array([.3,0.,.15]),np.array([.3,.2,.15]),np.eye(3),'remaining','carton')]


def test_xy_path_visits_literal_endpoints_both_axes_and_never_diagonal():
    p=xy_path([0.,-.2],[.012,-.207])
    np.testing.assert_allclose(p,[[0.,-.2],[.004,-.2],[.008,-.2],[.012,-.2],[.012,-.2035],[.012,-.207]],atol=1e-12,rtol=0)
    assert np.all(np.count_nonzero(np.abs(np.diff(p,axis=0))>1e-12,axis=1)==1)
    assert max(np.linalg.norm(np.diff(p,axis=0),axis=1))<=.005
    np.testing.assert_allclose(xy_path([.012,-.207],[0.,-.2],order=(1,0)),p[::-1],atol=1e-12,rtol=0)
    np.testing.assert_array_equal(xy_path([0,0],[0,0]),[[0,0]])


@pytest.mark.parametrize('kwargs',[{'step':.006},{'step':0},{'order':(0,0)}])
def test_xy_path_rejects_relaxed_sampling_and_wrong_axes(kwargs):
    with pytest.raises(ValueError):xy_path([0,0],[.1,.1],**kwargs)


def test_canonical_world_moves_only_plinth_and_native_frame_is_equivalent():
    initial=world();before=[box_message(b) for b in initial]
    canonical=relocation_world(initial,[.2,.1,0]);evaluation=relocation_world(initial,[.2,.1,0],evaluation_frame=True)
    np.testing.assert_allclose(canonical[1].center,[-1.80675,.25,.75],atol=1e-12)
    assert box_message(canonical[0])==before[0] and box_message(canonical[2])==before[2]
    np.testing.assert_allclose(evaluation[0].center,[-2.15,.25,.3],atol=1e-12)
    np.testing.assert_allclose(evaluation[1].center,initial[1].center,atol=1e-12)
    for a,b in zip(canonical,evaluation):np.testing.assert_allclose(a.corners(),b.corners()+[.2,.1,0],atol=1e-12)
    assert [box_message(b) for b in initial]==before
    with pytest.raises(ValueError):relocation_world(initial,[0,0,.1])


def test_translation_preserves_actual_concept_solid_vertices():
    boxes=world();solid=relieved_transverse(OBB(np.array([-.55,0,.2]),np.array([.35,1.1,.06]),np.eye(3),'belt','conveyor'),.2)
    moved=relocation_world([*boxes,solid],[.1,0,0],evaluation_frame=True)[-1]
    assert isinstance(moved,ConceptSolid) and moved.solid==solid.solid
    np.testing.assert_allclose(moved.center,[-.65,0,.2],atol=1e-12)


def test_support_and_new_plinth_collisions_are_not_chassis_exemptions():
    boxes=world();assert check_plinth_world(boxes)['passed']
    assert check_plinth_world(relocation_world(boxes,[.825,0,0]))['passed']
    assert not check_plinth_world(relocation_world(boxes,[.826,0,0]))['passed']
    obstacle=OBB(np.array([-2.00675,.15,.75]),np.array([.01,.01,.01]),np.eye(3),'unknown','unknown')
    assert not check_plinth_world([*boxes,obstacle])['passed']
    shifted=[boxes[0],replace(boxes[1],center=boxes[1].center-[0,0,.001]),boxes[2]]
    assert not check_plinth_world(shifted)['passed']


def test_move_receipt_preserves_q_cartons_completed_and_parent_receipt():
    a=LayoutCandidate(0,-.2,.9,.2,profile='low_profile_200_outfeed20_v3');b=replace(a,x=.4)
    root=dict(q_rad=[.1]*6,velocity_rad_s=[0.]*6,height_m=.25,world_cartons=[box_message(world()[-1])],
        completed=['accepted'],predecessor_final_state_receipt={'id':'actual-native-end'})
    r=relocation_receipt(root,a,b,'a'*64)
    assert r['q_rad']==root['q_rad'] and r['completed']==['accepted']
    assert r['parent_final_state_receipt']=={'id':'actual-native-end'} and r['world_cartons_sha256']==digest(root['world_cartons'])
    for change in ({'assumed_removed':['failed']},{'velocity_rad_s':[.1]*6}):
        with pytest.raises(ValueError):relocation_receipt(dict(root,**change),a,b,'a'*64)
    with pytest.raises(ValueError):relocation_receipt(root,a,replace(b,z=1.),'a'*64)
