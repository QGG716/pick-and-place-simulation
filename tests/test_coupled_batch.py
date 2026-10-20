"""Pure state/geometry regressions. Synthetic commits are not run evidence."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.coupled_batch import BatchState,coupled_binding,nominal_drop,message_box
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.top_layer_lifted import FrozenTopLayer,process_options
from unloading_sim.lifted_reception import translated
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.moveit2_backend import box_message

@pytest.fixture(scope='module')
def layout():return load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells/m710id70_unloading_layout_v1.yaml')

@pytest.mark.parametrize('a,b',[(.6,2.05),(2.05,1.75),(1.75,2.05),(2.05,2.05)])
def test_native_binding_is_same_source_all_objects_bidirectional(layout,a,b):
    cartons=layout.cartons();target=FrozenTopLayer.capture(cartons).target_ids[0]
    binding=coupled_binding(layout,cartons,a,b,target,[])
    assert binding['samples'][0]['height_m']==a and binding['samples'][-1]['height_m']==b
    for sample in binding['samples']:
        state=coupled_conveyor_state(layout,sample['height_m'],cartons)
        assert sorted(sample['world'],key=lambda x:x['id'])==sorted([box_message(x) for x in state.boxes],key=lambda x:x['id'])
    assert not binding['continuous_swept_proof']

@pytest.mark.parametrize('height,drop_m',[(2.05,.05),(1.75,.35)])
def test_nominal_first_touch_constant_orientation_and_initial_rest(layout,height,drop_m):
    target=next(b for b in layout.cartons() if b.name==FrozenTopLayer.capture(layout.cartons()).target_ids[2])
    box=translated(target,[-.85,0,0]);belt=next(b for b in coupled_conveyor_state(layout,height).boxes if b.name=='conveyor_transverse')
    drop=nominal_drop(box,belt)
    assert drop['drop_m']==pytest.approx(drop_m)
    assert drop['samples'][0]['box']==box_message(box)
    final=message_box(drop['samples'][-1]['box'])
    np.testing.assert_array_equal(final.rotation,box.rotation)
    assert final.center[2]-final.half_extents[2]==pytest.approx(height)
    assert drop['samples'][-1]['t']==pytest.approx(np.sqrt(2*drop_m/9.81))
    assert drop['maximum_displacement_step_m']<=.002
    with pytest.raises(ValueError,match='REGION'):nominal_drop(translated(box,[0,3,0]),belt)

def test_commit_cannot_drain_early_or_change_root_and_keeps_lower_35(layout):
    frozen=FrozenTopLayer.capture(layout.cartons());state=BatchState(frozen,tuple(layout.cartons()),np.zeros(6),.6)
    lower=[box_message(b) for b in state.cartons if b.name not in frozen.target_ids]
    for i,target in enumerate(frozen.target_ids):
        plan=dict(batch_root=state.root(),target_id=target,events=[dict(event_type=t,target_id=target) for t in
            ('RELEASE','NOMINAL_FIRST_CONTACT','SAFE_TOOL_SEPARATION','IDEAL_RECEIVER_DRAIN')],
            separation=dict(passed=True),nominal_release=dict(passed=True),end_q_rad=[float(i)]*6,
            height_m=1.75 if i%2 else 2.05,landed_box={'test_injection':True},release_stage_id='TEST_ONLY',final_state_receipt={'test_injection':i})
        bad=deepcopy(plan);bad['events'].pop(1)
        with pytest.raises(ValueError,match='PREMATURE'):state.commit(bad)
        bad=deepcopy(plan);bad['batch_root']['height_m']=99
        with pytest.raises(ValueError,match='ROOT_CHANGED'):state.commit(bad)
        state.commit(plan)
        assert state.q.tolist()==plan['end_q_rad'] and state.height==plan['height_m']
    assert len(state.cartons)==35
    assert [box_message(b) for b in state.cartons]==lower

def test_both_right_of_region_front_options_are_lateral_hmin(layout):
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells/m710id70_unloading_layout_v1_rear_fixed_20261019.yaml')
    frozen=FrozenTopLayer.capture(layout.cartons());belt=next(b for b in layout.fixed_components() if b.name=='conveyor_transverse')
    for name in frozen.target_ids[:2]:
        target=next(b for b in layout.cartons() if b.name==name)
        options=process_options(target,belt,frozen,layout.cartons())
        assert options[0]['template']=='FRONT_LATERAL_HMIN'
        assert options[0]['segment_directions_world']==[[0.,1.,0.],[-1.,0.,0.]]
        assert options[0]['height_m']==pytest.approx(1.75)


def test_native_plain_first_sample_retains_only_exact_actual_parent():
    from unloading_sim.single_right_receiver import retain_exact_native_start
    from unloading_sim.moveit2_backend import MoveItUnavailable
    class State(np.ndarray):pass
    q=np.zeros(6).view(State);q.native_receipt={'stage_id':'ACTUAL_PARENT'}
    path=[np.asarray(q).copy(),np.ones(6)]
    assert retain_exact_native_start(path,q)[0] is q
    with pytest.raises(MoveItUnavailable,match='START_CHANGED'):
        retain_exact_native_start([np.full(6,1e-12)],q)


def test_open_lateral_channel_ranks_before_blocked_neighbor(layout):
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells/m710id70_unloading_layout_v1_rear_fixed_20261019.yaml')
    from unloading_sim.top_layer_lifted import rank_remaining
    from types import SimpleNamespace
    cartons=layout.cartons();f=FrozenTopLayer.capture(cartons)
    remaining=[b for b in cartons if b.name not in f.target_ids[2:]]
    robot=SimpleNamespace(within_limits=lambda q:True,fk=lambda q:np.eye(4))
    belt=next(b for b in layout.fixed_components() if b.name=='conveyor_transverse')
    ranked=rank_remaining(f,remaining,f.target_ids[2:],np.zeros(6),robot,belt)
    assert ranked[0]['target_id']==f.target_ids[1]
    assert ranked[0]['lateral_channel_blockers']==[]
    assert ranked[1]['lateral_channel_blockers']==[f.target_ids[1]]
