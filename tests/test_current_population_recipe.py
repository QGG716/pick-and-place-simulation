from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.current_population_recipe import (
    LAYOUT, recipe_permission, current_options, fixed_cycle_height, validate_preparation_binding)
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.low_top_receiver import uses_low_top, LowTopReceiverConnector
from unloading_sim.lifted_reception import StartHeight
from unloading_sim.l01_hmax import validate_option


def setup():
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells'/LAYOUT)
    boxes={b.name:b for b in layout.cartons()}
    frozen=FrozenCartonPopulation.capture(tuple(boxes.values()))
    return layout,boxes,frozen,recipe_permission(frozen)


def test_current_rules_keep_upper_layers_and_exact_bottom_grants():
    layout,boxes,frozen,permission=setup();upper=0
    expected={'carton_l01_c00':['x-','y+'],'carton_l01_c01':['x-'],
              'carton_l01_c04':['z+','y-','x-']}
    for name,target in boxes.items():
        choices,top,hmax,rotation=current_options(permission,target,frozen,layout,tuple(boxes.values()),.55)
        if name not in permission['bottom_top_permission']['allowed_target_ids']:
            assert choices is top is hmax is rotation is None;upper+=1;continue
        assert [o['face']['geometric_face_id'] for o in choices]==expected.get(name,['z+'])
        assert all(o['height_m']==o['pre_approach_height_m']==.25 for o in choices)
        assert all(o['fixed_receiver_preparation']['initial_height_m']==.55 for o in choices)
        assert bool(rotation)==(name=='carton_l00_c03')
        if rotation:assert rotation['world_z_degrees']==[90,-90]
    assert upper==30
    assert not any(uses_low_top(StartHeight.capture(b),.25,permission['low_definition']) for b in boxes.values())
    bad=deepcopy(permission);bad['bottom_top_permission']['allowed_target_ids'].append('carton_l02_c02')
    with pytest.raises(ValueError,match='PERMISSION_CHANGED'):
        current_options(bad,boxes['carton_l02_c02'],frozen,layout,tuple(boxes.values()),.55)


def test_hmax_preparation_does_not_change_face_height_or_target_grant():
    layout,boxes,frozen,permission=setup();box=boxes['carton_l01_c01']
    choices,_,grant,_=current_options(permission,box,frozen,layout,tuple(boxes.values()),.55)
    option,=choices
    target=SimpleNamespace(target_id=box.name,box=box,geometric_face_id='x-',world_y_offset_m=0.,eccentric_authority=None)
    validate_option(target,option,grant,.55)
    without=deepcopy(option);without.pop('fixed_receiver_preparation')
    with pytest.raises(ValueError):validate_option(target,without,grant,.55)
    for field,value in [('cycle_height_m',.35),('initial_height_m',.6),('target_id','carton_l00_c01'),('native_check_required',False)]:
        bad=deepcopy(option);bad['fixed_receiver_preparation'][field]=value
        with pytest.raises(ValueError):validate_option(target,bad,grant,.55)
    bad=deepcopy(option);bad['segment_directions_world']=[[0.,1.,0.],[-1.,0.,0.]]
    with pytest.raises(ValueError):validate_option(target,bad,grant,.55)


def test_preparation_requires_the_complete_native_transition_binding():
    layout,boxes,frozen,permission=setup();target=boxes['carton_l01_c01'];cartons=tuple(boxes.values())
    choices,_,_,_=current_options(permission,target,frozen,layout,cartons,.55);option=choices[0]
    binding=coupled_binding(layout,cartons,.25,.25,target.name,[])
    with pytest.raises(ValueError,match='BINDING_MISSING'):
        validate_preparation_binding(option,{'height_m':.55},binding)
    binding['pre_approach_transition']=coupled_binding(layout,cartons,.55,.25,target.name,[])
    validate_preparation_binding(option,{'height_m':.55},binding)
    for mutate in (
        lambda b:b['pre_approach_transition']['samples'][0].update(height_m=.25),
        lambda b:b['pre_approach_transition'].update(maximum_height_step_m=.02),
        lambda b:b['pre_approach_transition'].update(target_id='carton_l01_c00'),
        lambda b:b['samples'][0]['world'].pop(),
    ):
        bad=deepcopy(binding);mutate(bad)
        with pytest.raises(ValueError):validate_preparation_binding(option,{'height_m':.55},bad)


def test_bottom_top_keeps_explicit_grant_with_checked_preparation():
    layout,boxes,frozen,permission=setup();box=boxes['carton_l01_c02']
    choices,grant,_,_=current_options(permission,box,frozen,layout,tuple(boxes.values()),.55)
    c=object.__new__(LowTopReceiverConnector);c.option=choices[0];c.layer_top_permission=grant;c.batch_root={'height_m':.55}
    target=SimpleNamespace(validate=lambda:None,target_id=box.name,box=box,geometric_face_id='z+',
        face_role='TOP',outward_normal=np.array([0.,0.,1.]),world_y_offset_m=0.,eccentric_authority=None)
    c.validate_template(target)
    c.option=deepcopy(c.option);c.option.pop('fixed_receiver_preparation')
    with pytest.raises(Exception,match='LOW_TOP_PROCESS_SCOPE_CHANGED'):c.validate_template(target)


def test_fresh_driver_dispatches_40_without_historical_roots(tmp_path,monkeypatch):
    pytest.importorskip('pinocchio')
    import sys
    import plan_m710_current_population as entry
    def dispatch(args,original,built,frozen,seed,folder,*,initial_state,target_ids):
        assert len(initial_state.cartons)==40 and not initial_state.ledger and not initial_state.cycles
        assert initial_state.height==.6 and seed==71070
        assert built.robot.base_transform[:3,3]==pytest.approx([-1.625,.25,.6])
        assert args.include_receive_before_approach and args.current_population_recipe==recipe_permission(frozen)
        assert args.low_definition=='FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'
        assert target_ids==['carton_l07_c02']
        raise RuntimeError('CHECKED_FRESH_DISPATCH')
    monkeypatch.setattr(entry,'run_seed',dispatch)
    monkeypatch.setattr(sys,'argv',['plan','--output',str(tmp_path/'run'),'--worker-command','UNUSED',
        '--worker-binary',str(Path(entry.__file__)),'--source-commit','TEST','--development'])
    with pytest.raises(RuntimeError,match='CHECKED_FRESH_DISPATCH'):entry.main()


def test_current_verifier_requires_exact_mount_and_extension():
    from verify_m710_population import expected_population_mount
    from unloading_sim.current_population_recipe import SCOPE
    protocol=dict(scope=SCOPE,mounting_variant='x_minus_300_y_minus_100',
                  tool_connector_extension_m=.2,active_layout_file='configs/workcells/'+LAYOUT)
    assert expected_population_mount(protocol)==[-1.625,.25,.6]
    for field,value in [('tool_connector_extension_m',.1),('mounting_variant','x_minus_500_y_minus_100'),
                        ('active_layout_file','configs/workcells/m710id70_unloading_layout_v1.yaml')]:
        with pytest.raises(ValueError,match='CURRENT_POPULATION_LAYOUT_CHANGED'):
            expected_population_mount(dict(protocol,**{field:value}))
