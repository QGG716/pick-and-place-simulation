"""Strict box-top selection and retained-geometry rejection at 250 mm."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state,ScissorGeometryRangeError
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.geometry import OBB
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.lifted_reception import StartHeight
from unloading_sim.low_top_receiver import low_top_option,uses_low_top,BOX_TOP_LOW_DEFINITION
from unloading_sim.moveit2_backend import box_message

ROOT=Path(__file__).resolve().parents[1]
NAME='m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100'


@pytest.fixture(scope='module')
def layout():return load_workcell_layout(ROOT/f'configs/workcells/{NAME}_min250.yaml')


def test_only_minimum_changes_no_chassis_rear_height_or_dimensions_move(layout):
    old=load_workcell_layout(ROOT/f'configs/workcells/{NAME}.yaml')
    np.testing.assert_allclose(layout.robot_base_transform()[:3,3],[-1.825,.25,.6],atol=1e-12)
    assert audit_layout_constraints(layout)['overall_status']=='PASS'
    data=deepcopy(layout.data)
    assert data['conveyors']['coupling'].pop('minimum_surface_height_world_m')==.25
    data['evidence']=old.data['evidence']
    assert data==old.data
    before=coupled_conveyor_state(old,1.15)
    after=coupled_conveyor_state(layout,1.15)
    assert [box_message(b) for b in before.boxes]==[box_message(b) for b in after.boxes]
    changed=deepcopy(layout.data);changed['conveyors']['coupling']['minimum_surface_height_world_m']=.24
    assert audit_layout_constraints(replace(layout,data=changed))['overall_status']=='FAIL'


@pytest.mark.parametrize('top,expected',[(.249999,True),(.25,False),(.250001,False)])
def test_strict_top_boundary_and_opaque_identity(layout,top,expected):
    box=OBB([.3,0,top-.1],[.3,.2,.1],np.eye(3),'opaque-id','carton')
    frozen=FrozenCartonPopulation.capture((box,))
    fixed={b.name:b for b in layout.fixed_components()}
    option=low_top_option(box,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.25,
        definition=BOX_TOP_LOW_DEFINITION)
    assert (option is not None)==expected
    if expected:
        assert option['height_m']==.25 and option['stationary_receiver_during_target']
        assert option['face']['geometric_face_id']=='z+'


def test_actual_300mm_boxes_never_trigger_top_at_250mm(layout):
    heights=[StartHeight.capture(b) for b in layout.cartons()]
    assert min(h.z_b+h.h for h in heights)==pytest.approx(.3)
    assert not any(uses_low_top(h,.25,BOX_TOP_LOW_DEFINITION) for h in heights)
    l03=next(h for h in heights if h.target_id=='carton_l03_c02')
    assert l03.z_b+l03.h==pytest.approx(1.2)
    assert l03.hmin==pytest.approx(.55)
    assert not uses_low_top(l03,.25,BOX_TOP_LOW_DEFINITION)


def test_250mm_impossible_pin_order_is_not_an_ik_failure(layout):
    with pytest.raises(ScissorGeometryRangeError) as error:
        coupled_conveyor_state(layout,.25,())
    evidence=error.value.evidence
    assert evidence['mechanism_id']=='lift_transverse_left'
    assert evidence['lower_pin_world_z_m']==pytest.approx(.10)
    assert evidence['upper_pin_world_z_m']==pytest.approx(.06)
    assert evidence['stage_rise_m']==pytest.approx(-.02)
    assert evidence['required_stage_rise_open_interval_m']==[.035,1.215]
    assert not evidence['native_search_performed']
    # Hmin=0.6-0.3-0.05 may round just below .25. It must retain the
    # same physical rejection, never turn into a generic input exception.
    with pytest.raises(ScissorGeometryRangeError):coupled_conveyor_state(layout,.6-.3-.05,())
    with pytest.raises(ScissorGeometryRangeError) as transition:
        coupled_binding(layout,(),1.15,.25,'opaque',())
    assert transition.value.evidence['checked_surface_height_world_m']<=.36+1e-12


def test_negative_longitudinal_pitch_keeps_rear_station_and_rigid_length(layout):
    state=coupled_conveyor_state(layout,.55,())
    assert state.mechanism['tilt_rad']<0
    assert state.mechanism['rear_hinge_world'][2]==.6
    assert state.mechanism['near_tip_world'][2]==.55
    assert np.linalg.norm(np.subtract(state.mechanism['near_tip_world'],state.mechanism['rear_hinge_world']))==pytest.approx(2.8)
    assert len(state.boxes)==len(coupled_conveyor_state(layout,.6,()).boxes)


def test_saved_mount_binding_for_continuation():
    from verify_m710_population import expected_population_mount
    protocol=dict(scope='XY_TOP_MIN250_CONTINUATION',mounting_variant='x_minus_500_y_minus_100',
        active_layout_file=f'configs/workcells/{NAME}_min250.yaml')
    assert expected_population_mount(protocol)==[-1.825,.25,.6]
    protocol['active_layout_file']=f'configs/workcells/{NAME}.yaml'
    with pytest.raises(ValueError,match='LAYOUT_VARIANT_MISMATCH'):expected_population_mount(protocol)


@pytest.mark.parametrize('receive',[.55,.85])
def test_lower_receive_height_is_an_actual_pre_approach_candidate(receive):
    from unloading_sim.junction_reception import synchronization_options
    option=dict(template='FRONT_DIRECT_HMIN' if receive==.55 else 'FRONT_DIRECT_HMAX',
        height_m=receive,reception=dict(junction=False),face=dict(geometric_face_id='x-'))
    legacy=list(synchronization_options(option,1.15,.25))
    routes=list(synchronization_options(option,1.15,.25,include_receive_before_approach=True))
    assert [r.get('pre_approach_height_m') for r in routes]==[None,receive,.25]
    assert routes[0]==legacy[0] and routes[-1]==legacy[-1]
    assert routes[1]['synchronization']=='ALREADY_AT_RECEIVE'
    assert all(r['face']==option['face'] and r['height_m']==receive for r in routes)
    assert len(list(synchronization_options(option,receive,.25,include_receive_before_approach=True)))==2
