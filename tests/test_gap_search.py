"""Literal fixed-layout and process boundaries; not native success evidence."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.gap_search_layout import GapCandidate,candidate_data,BASELINE,floor_support_evidence,validate_search_layout
from unloading_sim.gap_search_recipes import candidate_options,validate_receive_contract
from unloading_sim.workcell_layout import load_workcell_layout,compute_layout_fingerprint,audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import check_coupled_state
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import BatchState
from unloading_sim.joint_search_outfeed import outfeed_path
from unloading_sim.joint_search_layout import qualified_mount_pairs
from unloading_sim.geometry import OBB
from unloading_sim.lifted_reception import StartHeight
from unloading_sim.moveit2_backend import box_message

ROOT=Path(__file__).resolve().parents[1]
@pytest.fixture(scope='module')
def base():return load_workcell_layout(ROOT/'configs/workcells'/BASELINE)
def layout_for(base,g):
    d=candidate_data(base.data,GapCandidate(g))
    return replace(base,data=d,layout_fingerprint=compute_layout_fingerprint(d,base.assets))

@pytest.mark.parametrize('g,assembly,base_x',[(.2,-1.95,-1.35),(.4,-2.15,-1.55),(.6,-2.35,-1.75)])
def test_literal_mount_world_and_only_gap(base,g,assembly,base_x):
    l=layout_for(base,g);assert audit_layout_constraints(l)['overall_status']=='PASS'
    np.testing.assert_allclose(l.robot_base_transform()[:3,3],[base_x,.15,1.],atol=1e-12,rtol=0)
    np.testing.assert_allclose(l.world_from_assembly[:3,3],[assembly,.35,0.],atol=1e-12,rtol=0)
    assert l.data['robot']['base_origin_xy_a_m']==[.6,-.2]
    assert 1.05-l.data['robot']['base_origin_xy_a_m'][0]==pytest.approx(.45)
    parts={b.name:b for b in l.fixed_components()}
    assert 2*parts['mount_plinth'].half_extents[2]==pytest.approx(.4)
    np.testing.assert_allclose(2*parts['chassis'].half_extents,[2.1,1.5,.6])
    assert len(l.cartons())==len({b.name for b in l.cartons()})==40
    for key in ('tool','trailer','carton_stack','world'):assert l.data[key]==base.data[key]
    tampered=deepcopy(l.data);tampered['robot']['base_origin_xy_a_m'][1]=-.19
    with pytest.raises(ValueError,match='FROZEN_FIELDS'):validate_search_layout(replace(l,data=tampered))

def test_bounds_reconstruction_fixed_rail_and_floor_evidence(base):
    a=layout_for(base,.2);b=layout_for(base,.6)
    for h in (.55,.6,1.,1.5):
        sa=coupled_conveyor_state(a,h,());sb=coupled_conveyor_state(b,h,())
        named={x.name:x for x in sb.boxes}
        for x in sa.boxes:
            if x.name.startswith('trailer'):continue
            y=named[x.name];np.testing.assert_allclose(y.half_extents,x.half_extents,atol=1e-12,rtol=0)
            np.testing.assert_allclose(y.rotation,x.rotation,atol=1e-12,rtol=0)
            np.testing.assert_allclose(y.center-x.center,[-.4,0,0],atol=1e-12,rtol=0)
        rail=next(x for x in sa.boxes if x.name=='lift_lower_rail_0')
        assert 2*rail.half_extents[0]==pytest.approx(1.05)
        assert floor_support_evidence(a,sa)['status']=='VERIFIED_ON_MODELED_FLOOR'
        assert floor_support_evidence(b,sb)['status']=='UNKNOWN_EXTERNAL_SUPPORT'
        assert check_coupled_state(a,h,(),designed_contacts=[['chassis','conveyor_transverse']])['passed']
    for h in (.549999,1.500001):
        with pytest.raises(ValueError):coupled_conveyor_state(a,h,())

def test_mount_policy_rejects_broad_pairs_and_bad_plinth(base):
    policy=SimulationCollisionPolicy(fixed_mount_contact_mode='qualified_planes_only')
    assert not policy.robot_mount_pairs() and not policy.tool_chassis_pairs('carton_l01_c00',['tool'])
    for kwargs in ({'robot_mount_contact_exemption':True},{'tool_chassis_contact_target_ids':('x',)}):
        with pytest.raises(ValueError):replace(policy,**kwargs)
    l=layout_for(base,.2);robot=SimpleNamespace(joint_search_mount=l.data['gap_search'],base_transform=l.robot_base_transform())
    parts=l.fixed_components();assert qualified_mount_pairs(robot,parts)=={('base_link','mount_plinth')}
    bad=[OBB(x.center+[0,0,.001],x.half_extents,x.rotation,x.name,x.category) if x.name=='mount_plinth' else x for x in parts]
    with pytest.raises(ValueError):qualified_mount_pairs(robot,bad)

def options_for(l,box):
    # Removing obstacles here exercises candidate generation only, never credit.
    scene=SimpleNamespace(cartons=(box,),fixed_components=l.fixed_components(),
        receiver=next(b for b in l.fixed_components() if b.name=='conveyor_transverse'),
        snapshot={'scene_fingerprint':'test'},scene_revision='test')
    scene.all_obstacles=(*scene.fixed_components,box)
    from unloading_sim.support import SupportRelationGraph
    scene.support_graph=SupportRelationGraph.build((box,))
    scene.policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}})
    faces,_=simulation_contact_targets(scene,box,l.robot_base_transform()[:3,3],.1,side_exposure_depth_m=.01)
    return list(candidate_options(faces,scene,FrozenCartonPopulation.capture(l.cartons()),.6,l,initial_q=[0.]*6))

def test_original_height_front_only_and_real_drop(base):
    l=layout_for(base,.2);top=max(l.cartons(),key=lambda b:b.center[2]);options=options_for(l,top)
    assert options and {t.face_role for t,o in options}=={'FRONT'}
    t,o=options[0];assert o['template']=='FRONT_DIRECT_HIGH_DROP' and o['height_m']==1.5
    assert o['release_bottom_world_m']==pytest.approx(2.1)
    assert o['joint_search_contract']['expected_drop_m']==pytest.approx(.6)
    assert o['segment_directions_world']==[[-1.,0.,0.]]
    c=o['joint_search_contract'];validate_receive_contract(c,top,1.5)
    with pytest.raises(ValueError):validate_receive_contract(dict(c,face='z+'),top,1.5)
    with pytest.raises(ValueError):validate_receive_contract(dict(c,release_bottom_m=1.55),top,1.5)

def test_lower_fair_faces_and_post_extraction_raise(base):
    l=layout_for(base,.2);low=min(l.cartons(),key=lambda b:b.center[2]);options=options_for(l,low)
    assert [t.face_role for t,o in options[:3]]==['TOP','FRONT','SIDE']
    for target,o in options:
        assert .55<=o['height_m']<=1.5 and o['release_bottom_world_m']>=.6-1e-9
        assert [0.,0.,1.] in o['segment_directions_world']
        validate_receive_contract(o['joint_search_contract'],low,o['height_m'])

def test_finite_outfeed_support_and_failed_box_preserved(base):
    l=layout_for(base,.2);world=coupled_conveyor_state(l,1.5,()).boxes
    box=OBB([-.55,.4,1.65],[.3,.2,.15],np.eye(3),'target','carton')
    path=outfeed_path(box,world,finite_support=True)
    assert path['finite_support'] and path['samples'][-1]['box']['pose'][0][3]+.3 < -3.2
    short=[OBB(b.center+[1.,0,0],b.half_extents,b.rotation,b.name,b.category) if b.name=='conveyor_longitudinal' else b for b in world]
    with pytest.raises(ValueError,match='FINITE_CONVEYOR_SUPPORT'):outfeed_path(box,short,finite_support=True)
    state=BatchState(FrozenCartonPopulation.capture(l.cartons()),l.cartons(),np.zeros(6),.6)
    before=deepcopy(state.root())
    with pytest.raises((KeyError,ValueError)):state.commit({'status':'FAIL'})
    assert state.root()==before and len(state.cartons)==40

def test_dispatch_screening_checkpoint_serializes_numpy_height_comparison(base,monkeypatch):
    import sys,json
    sys.path.insert(0,str(ROOT/'tools'))
    import plan_m710_gap_search as entry
    from unloading_sim.support import SupportRelationGraph
    l=layout_for(base,.2);box=max(l.cartons(),key=lambda b:b.center[2])
    scene=SimpleNamespace(cartons=(box,),fixed_components=l.fixed_components(),
        receiver=next(b for b in l.fixed_components() if b.name=='conveyor_transverse'),
        snapshot={'scene_fingerprint':'test'},support_graph=SupportRelationGraph.build((box,)),
        policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
    monkeypatch.setattr(entry,'batch_scene',lambda *a:scene)
    monkeypatch.setattr(entry,'rank_population',lambda *a,**k:[dict(target_id=box.name)])
    monkeypatch.setattr(entry,'finite_side_exposure',lambda *a:dict(depth_m=.01))
    original=SimpleNamespace(policy=SimpleNamespace(layout_validation=SimpleNamespace(layout=l)))
    built=SimpleNamespace(robot=SimpleNamespace(base_transform=l.robot_base_transform()),
        budget=SimpleNamespace(pregrasp_standoff_m=.1),joint_search_initial_q=[0.]*6)
    frozen=FrozenCartonPopulation.capture(l.cartons());state=BatchState(frozen,(box,),np.zeros(6),.6)
    visits,screens=entry.offered(original,built,state,frozen)
    assert visits and json.loads(json.dumps(screens))[box.name]['upper_original_layer'] is True

def test_gap_native_client_allows_only_bound_detached_outfeed_and_park():
    from unloading_sim.coupled_batch import CoupledBatchClient
    from unloading_sim.moveit2_backend import MoveItUnavailable
    from unloading_sim.gap_search_layout import SCHEMA
    client=CoupledBatchClient(SimpleNamespace(request=lambda *a,**k:dict(status='SUCCESS')),
        binding=dict(joint_search_layout=dict(schema=SCHEMA),outfeed_park_q_rad=[0.]*6))
    req=dict(op='plan',planning_mode='cold_from_scratch',planning_only=True,stage='withdrawal',
        pipeline_id='semantic_event',planner_id='WAIT_IDEAL_OUTFEED',attachment=None,parent_stage_id='checked-drop',
        q_start=[0.]*6,q_goal=[0.]*6,world=[dict(id='box')],process_policy=dict(target_id='box',attached=False),
        outfeed=dict(schema='fixed40_wait_outfeed_v1',finite_support=dict(schema='finite_belt_contact_envelope_v1')))
    assert client.request(req)['status']=='SUCCESS'
    with pytest.raises(MoveItUnavailable):client.request(dict(req,outfeed=dict(schema='fixed40_wait_outfeed_v1')))
    with pytest.raises(MoveItUnavailable):client.request(dict(req,attachment=dict(id='box')))
    park=dict(req,stage='residence',purpose='FREE_UNLOADED_REPOSITION',pipeline_id='ompl',planner_id='RRTConnectkConfigDefault',ownership_transition=None)
    assert client.request(park)['status']=='SUCCESS'
    with pytest.raises(MoveItUnavailable):client.request(dict(park,q_goal=[1.]*6))

def test_failure_classification_uses_outcome_not_nested_request_words():
    import sys
    sys.path.insert(0,str(ROOT/'tools'))
    from plan_m710_gap_search import failure_category
    failed=dict(failure=dict(reason='SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED',last_failure=dict(request=dict(clearance_policy={}))))
    assert failure_category(failed)=='NUMERICAL_SEARCH_DEADLINE_NOT_UNREACHABLE'
    assert failure_category(dict(failure=dict(reason='RECEIVER_REGION_SEARCH_INCOMPLETE',termination='STAGE_BUDGET_EXHAUSTED')))=='NUMERICAL_SEARCH_DEADLINE_NOT_UNREACHABLE'
    assert failure_category(dict(failure=dict(reason='ROBOT_COLLISION')))=='COLLISION_OR_CLEARANCE'


def test_gap_regional_numerical_slice_keeps_shared_deadlines_and_failure(monkeypatch):
    from unloading_sim.gap_search_recipes import GapCycleConnector
    from unloading_sim.coupled_batch import CoupledCycleConnector
    from time import perf_counter
    c=object.__new__(GapCycleConnector)
    original=perf_counter()+100
    c._single_connection_deadline=original
    c._request_deadline_monotonic=original+100
    c._lift_approach_deadline=original-10
    monkeypatch.setattr(GapCycleConnector,'_native_limits',lambda self:(60.,70.))
    def failed(self,plan,row,*args):
        assert 0<self._single_connection_deadline-perf_counter()<=12
        assert self._request_deadline_monotonic==original+100
        assert self._lift_approach_deadline==original-10
        return None,dict(failure=dict(reason='RECEIVER_REGION_SEARCH_INCOMPLETE',termination='STAGE_BUDGET_EXHAUSTED'),region_attempts=[{}])
    monkeypatch.setattr(CoupledCycleConnector,'_plan_extraction_suffix',failed)
    plan,row=c._plan_extraction_suffix({}, {})
    assert plan is None and c._single_connection_deadline==original
    assert row['failure']['termination']=='LOCAL_NUMERICAL_VISIT_DEADLINE_NOT_UNREACHABLE'
    assert row['regional_numeric_visit']['native_attempts']==1


def test_gap_connected_native_path_keeps_original_completion_budget(monkeypatch):
    from unloading_sim.gap_search_recipes import GapCycleConnector
    from unloading_sim.coupled_batch import CoupledCycleConnector
    from time import perf_counter
    c=object.__new__(GapCycleConnector)
    c._gap_numeric_deadline=perf_counter()+12
    c._gap_full_suffix_deadline=perf_counter()+120
    c._single_connection_deadline=c._gap_full_suffix_deadline
    def connected(self,*args):
        assert self._single_connection_deadline==self._gap_numeric_deadline
        return ([1],None,{}, {},None,None)
    monkeypatch.setattr(CoupledCycleConnector,'region_motion',connected)
    result=c.region_motion()
    assert result[1] is None
    assert c._single_connection_deadline==c._gap_full_suffix_deadline


def test_gap_empty_admitted_search_persists_terminal_checkpoint(tmp_path,monkeypatch):
    import hashlib,json,sys
    sys.path.insert(0,str(ROOT/'tools'))
    import plan_m710_gap_search as entry
    manifest=tmp_path/'source.json';manifest.write_text('{"files":{}}')
    worker=tmp_path/'worker';worker.write_bytes(b'not executed by this empty search')
    identity=dict(source_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        native_binary_sha256=hashlib.sha256(worker.read_bytes()).hexdigest())
    checkpoint=tmp_path/'checkpoint.json'
    checkpoint.write_text(json.dumps(dict(contract=entry.SCHEMA,identity=identity,requested_gaps_m=[],
        layouts={'unsupported':dict(status='UNKNOWN_EXTERNAL_SUPPORT')},trials=[],best=None,
        elapsed_search_s=17.,status='SEARCHING',pending=None)))
    monkeypatch.setattr(sys,'argv',['gap-search','--output',str(tmp_path),'--resume',
        '--worker-command','unused','--worker-binary',str(worker),'--source-manifest',str(manifest)])
    entry.main()
    saved=json.loads(checkpoint.read_text())
    assert saved['status']=='FINITE_ADMITTED_SEARCH_EXHAUSTED'
    assert saved['elapsed_search_s']==17. and saved['trials']==[]
