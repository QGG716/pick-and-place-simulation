from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import tarfile
import numpy as np
import pytest
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import BatchState
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.l01_selected_routes import selected_routes_permission,selected_route_option,validate_selected_route,ORDER,TOP_TARGET

ROOT=Path(__file__).resolve().parents[1]

def setup():
    layout=load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile.yaml')
    boxes={b.name:b for b in layout.cartons()};frozen=FrozenCartonPopulation.capture(tuple(boxes.values()))
    from unloading_sim.conveyor_lift import coupled_conveyor_state
    height=frozen.evidence()['heights'][ORDER[0]]['Hmax_world_m']
    world={b.name:b for b in coupled_conveyor_state(layout,height).boxes}
    return boxes,frozen,world,height

def test_exact_three_routes_fixed_hmax_and_original_roles():
    boxes,frozen,world,height=setup();permission=selected_routes_permission(frozen)
    assert height==pytest.approx(.25) and permission['dispatch_order']==list(ORDER)
    for name in ORDER:
        option=selected_route_option(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,permission)
        top=name==TOP_TARGET
        assert option['face']['geometric_face_id']==('z+' if top else 'y+')
        assert option['segment_directions_world']==([[0.,0.,1.],[-1.,0.,0.]] if top else [[0.,1.,0.],[-1.,0.,0.]])
        assert option['height_m']==option['pre_approach_height_m']==height
        if top:assert option['release_bottom_world_m']==pytest.approx(.35)
        if name=='carton_l01_c01':assert option['original_role']=='MIDDLE'
        target=SimpleNamespace(target_id=name,box=boxes[name],geometric_face_id=option['face']['geometric_face_id'],world_y_offset_m=0,eccentric_authority=None)
        validate_selected_route(target,option,permission,height)
        changed=deepcopy(option);changed['height_m']+=.01
        with pytest.raises(ValueError):validate_selected_route(target,changed,permission,height)
    for name in ('carton_l01_c02','carton_l00_c04'):
        with pytest.raises(ValueError):selected_route_option(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,permission)

def test_restore_32_terminal_without_old_native_tree(tmp_path):
    from layer_top_continuation import restore_l01_tool_terminal,L01_TOOL_RECORDS
    boxes,frozen,_,_=setup();source=ROOT/'docs/experiments/m710_l01_tool_chassis_20261023'
    for name in L01_TOOL_RECORDS:(tmp_path/name).write_bytes((source/'records'/name).read_bytes())
    with tarfile.open(source/'formal-v1-raw.tar.xz') as t:
        for m in t:
            if m.isfile() and m.name.startswith('formal-v1/task-') and m.name.endswith('/result.json'):
                p=tmp_path/Path(m.name).relative_to('formal-v1');p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(t.extractfile(m).read())
    state=BatchState(frozen,tuple(boxes.values()),np.zeros(6),2.05)
    receipt=restore_l01_tool_terminal(tmp_path,state)
    assert len(state.cartons)==8 and len(state.ledger)==32 and not state.cycles
    assert state.root()['predecessor_final_state_receipt'] is None and receipt['first_target_id']==ORDER[0]
    assert set(ORDER)<={b.name for b in state.cartons}
    (tmp_path/'summary.json').write_text('{}')
    with pytest.raises(ValueError,match='RECORD_CHANGED'):restore_l01_tool_terminal(tmp_path,state)


def test_c01_finite_side_channel_keeps_c04_and_rejects_near_neighbors():
    from unloading_sim.contact_target import simulation_contact_targets
    from unloading_sim.support import SupportRelationGraph
    boxes,frozen,world,height=setup();target=boxes['carton_l01_c01']
    remaining=[b for n,b in boxes.items() if n.startswith('carton_l00_') or n in ORDER]
    scene=SimpleNamespace(support_graph=SupportRelationGraph(remaining,[]),snapshot={'scene_fingerprint':'test'},
        policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
    faces,_=simulation_contact_targets(scene,target,[-1.825,.25,.6],.1)
    assert not any(f.geometric_face_id=='y+' for f in faces)
    faces,screen=simulation_contact_targets(scene,target,[-1.825,.25,.6],.1,side_exposure_depth_m=.322700003)
    assert any(f.geometric_face_id=='y+' for f in faces)
    assert next(f for f in screen if f['geometric_face_id']=='y+')['blockers']==[]
    faces,screen=simulation_contact_targets(scene,boxes['carton_l01_c00'],[-1.825,.25,.6],.1,side_exposure_depth_m=.322700003)
    assert not any(f.geometric_face_id=='y+' for f in faces)
    assert 'carton_l01_c01' in next(f for f in screen if f['geometric_face_id']=='y+')['blockers']
    permission=selected_routes_permission(frozen,finite_c01_channel=True)
    assert permission['dispatch_order']==['carton_l01_c01'] and set(permission['routes'])=={'carton_l01_c01'}
    option=selected_route_option(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,permission)
    assert option['finite_side_channel'] and option['original_role']=='MIDDLE'
    for name in ('carton_l01_c00','carton_l01_c04'):
        with pytest.raises(ValueError,match='TARGET_OUTSIDE'):
            selected_route_option(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,permission)


def test_c01_turn_uses_full_projection_not_center_and_needs_only_120mm():
    from unloading_sim.l01_selected_routes import nonjunction_side_turn
    from unloading_sim.lifted_reception import translated
    boxes,_,world,_=setup();target=boxes['carton_l01_c01']
    check=nonjunction_side_turn(target,world.values())
    assert check['minimum_center_y_world_m']==pytest.approx(-.3)
    assert check['minimum_center_y_world_m']-target.center[1]==pytest.approx(.12)
    assert not check['passed']
    assert not nonjunction_side_turn(translated(target,[0,.12,0]),world.values())['passed']
    assert nonjunction_side_turn(translated(target,[0,.1202,0]),world.values())['passed']
