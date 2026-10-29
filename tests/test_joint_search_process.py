"""Boundary and tamper tests; synthetic states are not native run evidence."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.geometry import OBB
from unloading_sim.coupled_batch import BatchState,message_box
from unloading_sim.carton_population import FrozenCartonPopulation,rank_population
from unloading_sim.joint_search_outfeed import outfeed_path,check_full_tool_wait,check_full_tool_park,WITHDRAW_FIRST,PARK_FIRST,withdrawal_extra
from unloading_sim.joint_search_recipes import validate_receive_contract
from unloading_sim.joint_search_layout import SCHEMA,BASELINE
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.lifted_reception import StartHeight
from unloading_sim.moveit2_backend import box_message

@pytest.fixture(scope='module')
def layout():return load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells'/BASELINE)

@pytest.mark.parametrize('height',[.25,.6,1.15,2.1])
def test_outfeed_actual_full_envelope_and_wait_timing(layout,height):
    world=coupled_conveyor_state(layout,height,()).boxes
    box=OBB(np.array([-.65,.45,height+.15]),np.array([.20,.20,.15]),np.eye(3),'target','carton')
    path=outfeed_path(box,world)
    poses=[message_box(x['box']) for x in path['samples']]
    points=np.array([b.center for b in poses]);d=np.diff(points,axis=0)
    assert max(np.linalg.norm(d,axis=1))<=.005000001
    assert np.all(d[:,:2]<=1e-10) and max(poses[-1].corners()[:,0])< -3.2
    assert all(np.array_equal(b.rotation,box.rotation) and np.array_equal(b.half_extents,box.half_extents) for b in poses)
    assert path['tool_timing']=='WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW'
    assert not path['continuous_swept_proof']
    np.testing.assert_allclose(np.diff([x['t'] for x in path['samples']])*.2,np.linalg.norm(d,axis=1),atol=1e-12)

def test_outfeed_tests_middle_of_route_and_uncompressed_cups(layout):
    world=coupled_conveyor_state(layout,.6,()).boxes
    box=OBB(np.array([-.65,.45,.75]),np.array([.2,.2,.15]),np.eye(3),'target','carton')
    path=outfeed_path(box,world);middle=message_box(path['samples'][len(path['samples'])//2]['box'])
    cup=OBB(middle.center,np.full(3,.01),np.eye(3),'tool_compliant_bellows_0','tool')
    proof=check_full_tool_wait(SimpleNamespace(tool_collision_obbs_provider=lambda q:[cup]),np.zeros(6),path)
    assert not proof['passed'] and proof['pair'][1]=='tool_compliant_bellows_0' and 0<proof['sample_index']<len(path['samples'])-1

def test_second_finite_timing_keeps_entire_box_route_and_rejects_simultaneous(layout):
    world=coupled_conveyor_state(layout,.6,()).boxes
    box=OBB(np.array([-.65,.45,.75]),np.array([.2,.2,.15]),np.eye(3),'target','carton')
    original=outfeed_path(box,world)
    alternative=outfeed_path(box,world,tool_timing=WITHDRAW_FIRST)
    assert alternative==dict(original,tool_timing=WITHDRAW_FIRST)
    assert outfeed_path(box,world,tool_timing=PARK_FIRST)==dict(original,tool_timing=PARK_FIRST)
    with pytest.raises(ValueError,match='TIMING_UNSUPPORTED'):
        outfeed_path(box,world,tool_timing='UNVERIFIED_SIMULTANEOUS')

def test_extra_withdrawal_has_finite_domain_and_requires_stationary_box_first():
    assert withdrawal_extra({})==0.
    for extra in (.10,.30,.50):
        assert withdrawal_extra(dict(outfeed_timing=WITHDRAW_FIRST,withdraw_extra_m=extra))==extra
    assert withdrawal_extra(dict(outfeed_timing=PARK_FIRST,withdraw_extra_m=.10))==.10
    for option in ({'withdraw_extra_m':.10},{'withdraw_extra_m':-.10},
            dict(outfeed_timing=WITHDRAW_FIRST,withdraw_extra_m=1.),
            dict(outfeed_timing=PARK_FIRST,withdraw_extra_m=.30)):
        with pytest.raises(ValueError,match='FINITE_WITHDRAWAL_DISTANCE'):withdrawal_extra(option)

def test_parking_checks_intermediate_uncompressed_tool_not_just_endpoints():
    box=OBB(np.zeros(3),np.array([.2,.2,.15]),np.eye(3),'target','carton')
    provider=lambda q:[OBB(np.array([q[0],0.,0.]),np.full(3,.01),np.eye(3),'tool_compliant_bellows_0','tool')]
    connector=SimpleNamespace(tool_collision_obbs_provider=provider)
    a=np.array([-1.,0,0,0,0,0]);b=-a
    for q in (a,b):assert check_full_tool_wait(connector,q,dict(samples=[dict(box=box_message(box))]))['passed']
    proof=check_full_tool_park(connector,[a,b],box)
    assert not proof['passed'] and 0<proof['joint_sample_index']<200
    clear=check_full_tool_park(connector,[a,a-.1],box)
    assert clear['passed'] and clear['maximum_joint_step_rad']==.01 and not clear['continuous_swept_proof']

def test_receive_cannot_clamp_or_rename_low_front(layout):
    box=min(layout.cartons(),key=lambda b:b.center[2]);h=StartHeight.capture(box)
    c=dict(schema=SCHEMA,target_identity=h.box_identity,d_m=.05,height_m=.25,minimum_m=.25,maximum_m=2.1,
        mode='Hmax',face='x-',release_bottom_m=h.z_b)
    with pytest.raises(ValueError,match='CLAMPED'):validate_receive_contract(c,box,.25)
    c['mode']='LIFTED_TOP_FIXED'
    with pytest.raises(ValueError,match='TOP_LIFT'):validate_receive_contract(c,box,.25)
    c.update(face='z+',release_bottom_m=.3)
    validate_receive_contract(c,box,.25)
    bad=deepcopy(c);bad['height_m']=.24
    with pytest.raises(ValueError):validate_receive_contract(bad,box,.24)

def test_height_roundoff_is_not_clamping(layout):
    box=next(b for b in layout.cartons() if b.name=='carton_l01_c00');h=StartHeight.capture(box)
    assert h.hmax==pytest.approx(.25)
    c=dict(schema=SCHEMA,target_identity=h.box_identity,d_m=.05,height_m=h.hmax,minimum_m=.25,maximum_m=2.1,
        mode='Hmax',face='x-',release_bottom_m=h.z_b)
    validate_receive_contract(c,box,h.hmax)
    validate_receive_contract(c,box,np.nextafter(h.hmax,np.inf))
    with pytest.raises(ValueError):validate_receive_contract(c,box,h.hmax+1e-5)
    assert c['height_m']==h.hmax


@pytest.mark.parametrize('height',[.65,.85])
def test_explicit_top_raise_has_named_height_and_does_not_relabel_hmax(layout,height):
    box=min(layout.cartons(),key=lambda b:b.center[2]);h=StartHeight.capture(box)
    c=dict(schema=SCHEMA,target_identity=h.box_identity,d_m=.05,height_m=height,minimum_m=.25,maximum_m=2.1,
        mode='LIFTED_TOP_EXPLICIT',face='z+',explicit_receive_height_m=height,release_bottom_m=height+.05)
    validate_receive_contract(c,box,height)
    for patch in ({'face':'x-'},{'explicit_receive_height_m':.7},{'mode':'Hmax'},
                  {'release_bottom_m':h.z_b},{'maximum_m':.6}):
        with pytest.raises(ValueError):validate_receive_contract(dict(c,**patch),box,height)
    bad=dict(c);bad.pop('explicit_receive_height_m')
    with pytest.raises(ValueError):validate_receive_contract(bad,box,height)
    high=max(layout.cartons(),key=lambda b:b.center[2]);high_h=StartHeight.capture(high)
    with pytest.raises(ValueError,match='TOP_HEIGHT_SCOPE'):
        validate_receive_contract(dict(c,target_identity=high_h.box_identity),high,height)


def test_low_carton_offers_both_explicit_top_heights_and_original_minimum(layout):
    from unloading_sim.contact_target import simulation_contact_targets
    from unloading_sim.support import SupportRelationGraph
    from unloading_sim.joint_search_recipes import candidate_options
    box=min(layout.cartons(),key=lambda b:b.center[2]);fixed=layout.fixed_components()
    scene=SimpleNamespace(support_graph=SupportRelationGraph.build([box]),snapshot={'scene_fingerprint':'test-only'},
        policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}),
        receiver=next(b for b in fixed if b.name=='conveyor_transverse'),fixed_components=fixed)
    faces=simulation_contact_targets(scene,box,[-1.625,.15,.6],.1)[0]
    frozen=FrozenCartonPopulation.capture(layout.cartons())
    options=[o for _,o in candidate_options(faces,scene,frozen,.6,layout,initial_q=np.zeros(6))]
    old_parks={o['candidate_key']:o for o in options if o.get('outfeed_timing')==PARK_FIRST and not o.get('withdraw_extra_m')}
    extended=[o for o in options if o.get('outfeed_timing')==PARK_FIRST and o.get('withdraw_extra_m')]
    assert len(extended)==len(old_parks)>0
    from unloading_sim.moveit2_backend import digest
    for extra in extended:
        assert extra['withdraw_extra_m']==.10
        predecessor=dict(extra);predecessor.pop('candidate_key');predecessor.pop('withdraw_extra_m')
        predecessor['visit_order']=[*predecessor['visit_order'][:4],.5]
        assert digest(predecessor) in old_parks
    raised=[o for o in options if o['joint_search_contract']['mode']=='LIFTED_TOP_EXPLICIT']
    assert {o['height_m'] for o in raised}=={.65,.85}
    assert all(o['face']['geometric_face_id']=='z+' for o in raised)
    assert all(o['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]] for o in raised)
    assert any(o['height_m']==.25 and o['joint_search_contract']['mode']=='LIFTED_TOP_FIXED' for o in options)

def test_failed_target_reenters_when_support_changes_without_world_mutation(layout):
    cartons=tuple(layout.cartons());frozen=FrozenCartonPopulation.capture(cartons);robot=layout.robot()
    belt=next(b for b in layout.fixed_components() if b.name=='conveyor_transverse')
    longitudinal=next(b for b in layout.fixed_components() if b.name=='conveyor_longitudinal')
    q=np.asarray(layout.data['robot']['initial_q_rad']) if 'initial_q_rad' in layout.data['robot'] else np.zeros(6)
    first=rank_population(frozen,cartons,[],set(),q,robot,belt,longitudinal=longitudinal)
    failed=first[0]['target_id'];removed=first[1]['target_id']
    next_rows=rank_population(frozen,[b for b in cartons if b.name!=removed],[removed],set(),q,robot,belt,longitudinal=longitudinal)
    assert failed in [r['target_id'] for r in next_rows] and len(cartons)==40


def test_explicit_top_front_edge_positions_have_literal_seal_geometry(layout):
    from unloading_sim.contact_target import simulation_contact_targets
    from unloading_sim.support import SupportRelationGraph
    from unloading_sim.joint_search_recipes import candidate_options,TOP_FRONT_EDGE_POSES
    from unloading_sim.independent_cups import build_m710_independent_cup_array,evaluate_independent_cup_geometry
    box=next(b for b in layout.cartons() if b.name=='carton_l00_c02');fixed=layout.fixed_components()
    scene=SimpleNamespace(support_graph=SupportRelationGraph.build([box]),snapshot={'scene_fingerprint':'test-only'},
        policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}),
        receiver=next(b for b in fixed if b.name=='conveyor_transverse'),fixed_components=fixed)
    faces=simulation_contact_targets(scene,box,[-1.625,.15,.6],.1)[0]
    options=list(candidate_options(faces,scene,FrozenCartonPopulation.capture(layout.cartons()),.6,layout,initial_q=np.zeros(6)))
    new=[(t,o) for t,o in options if o['joint_search_contract'].get('contact_pose_family') in
        {'TOP_FRONT_EDGE_120_240_MM_QUARTER_TURN_V1','TOP_FRONT_EDGE_180_MM_QUARTER_TURN_V1'}]
    assert {(o['joint_search_contract']['offset_face_m'][0],o['joint_search_contract']['roll_rad']) for _,o in new}=={
        (-.12,np.pi/2),(-.12,-np.pi/2),(-.24,np.pi/2),(-.24,-np.pi/2),(-.18,np.pi/2),(-.18,-np.pi/2)}
    assert all(t.face_role=='TOP' and o['joint_search_contract']['offset_face_m'][1]==0. for t,o in new)
    representative={tuple(o['joint_search_contract']['offset_face_m']):t for t,o in new}
    for offset,world_x,eligible in [(-.12,.18,54),(-.24,.06,42),(-.18,.12,48)]:
        target=representative[(offset,0.)]
        np.testing.assert_allclose(target.contact_pose[:3,3],[world_x,0.,.3],atol=1e-12,rtol=0)
        geometry=evaluate_independent_cup_geometry(target.contact_pose,target.box,target.legacy_face,
            build_m710_independent_cup_array(),max_attachment_gap_m=.002,maximum_penetration_m=.0002,
            max_normal_misalignment_rad=np.deg2rad(5.))
        assert sum(geometry.geometrically_eligible_mask)==eligible
    short=[(t,o) for t,o in options if o['joint_search_contract'].get('contact_pose_family')=='TOP_FRONT_EDGE_240_MM_SHORT_AXIS_V1']
    assert {o['joint_search_contract']['roll_rad'] for _,o in short}=={0.,np.pi}
    for roll in (0.,np.pi):
        target=next(t for t,o in short if o['joint_search_contract']['roll_rad']==roll)
        np.testing.assert_allclose(target.contact_pose[:3,3],[.06,0.,.3],atol=1e-12,rtol=0)
        geometry=evaluate_independent_cup_geometry(target.contact_pose,target.box,target.legacy_face,
            build_m710_independent_cup_array(),max_attachment_gap_m=.002,maximum_penetration_m=.0002,
            max_normal_misalignment_rad=np.deg2rad(5.))
        assert sum(geometry.geometrically_eligible_mask)==32

def test_joint_search_commit_requires_outfeed_before_any_mutation(layout):
    frozen=FrozenCartonPopulation.capture(layout.cartons());state=BatchState(frozen,tuple(layout.cartons()),np.zeros(6),.6)
    original=deepcopy(state.root());target=frozen.target_ids[0]
    bad=dict(batch_root=original,target_id=target,events=[],option={'joint_search_contract':{'schema':SCHEMA}})
    with pytest.raises(ValueError,match='FULL_OUTFEED_CHECK'):state.commit(bad)
    assert state.root()==original and not state.ledger
