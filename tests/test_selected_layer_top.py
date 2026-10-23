"""Selected-layer authority, fixed receiver, and the actual retained root."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import tarfile
import numpy as np
import pytest
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.coupled_batch import BatchState
from unloading_sim.geometry import OBB
from unloading_sim.low_top_receiver import (selected_layer_top_permission,layer_top_option,
    LowTopReceiverConnector,LAYER_TEMPLATE)
from unloading_sim.moveit2_backend import MoveItUnavailable
from unloading_sim.support import SupportRelationGraph
from unloading_sim.workcell_layout import load_workcell_layout

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def layout():
    return load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250.yaml')


def test_layer_permission_uses_geometry_with_opaque_ids(layout):
    boxes=tuple(OBB(b.center,b.half_extents,b.rotation,f'opaque-{i}',b.category) for i,b in enumerate(layout.cartons()))
    anchor=next(b for b in boxes if abs(b.center[2]-.75)<1e-9)
    frozen=FrozenCartonPopulation.capture(boxes)
    permission=selected_layer_top_permission(frozen,anchor.name)
    assert permission['ceiling_world_m']==pytest.approx(.9)
    assert len(permission['allowed_target_ids'])==15
    assert {b.name for b in boxes if b.center[2]<.8}==set(permission['allowed_target_ids'])
    assert not any(b.name in permission['allowed_target_ids'] for b in boxes if b.center[2]>1.)


@pytest.mark.parametrize('name,bottom,lift',[
    ('carton_l02_c01',.65,.05),('carton_l01_c02',.60,.30),('carton_l00_c02',.60,.60)])
def test_plus_z_then_minus_x_at_actual_receiver_height(layout,name,bottom,lift):
    frozen=FrozenCartonPopulation.capture(layout.cartons());fixed={b.name:b for b in layout.fixed_components()}
    permission=selected_layer_top_permission(frozen,'carton_l02_c01')
    target=next(b for b in layout.cartons() if b.name==name)
    option=layer_top_option(target,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.55,permission)
    assert option['template']==LAYER_TEMPLATE
    assert option['height_m']==option['pre_approach_height_m']==.55
    assert option['low_layer_threshold_m']==pytest.approx(.9)
    assert option['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
    assert option['release_bottom_world_m']==pytest.approx(bottom)
    assert bottom-(target.center[2]-target.half_extents[2])==pytest.approx(lift)
    assert option['stationary_receiver_during_target']
    assert option['face']['working_axis_into_box']==[0.,0.,-1.]


def test_no_upper_permission_or_changed_original_geometry(layout):
    frozen=FrozenCartonPopulation.capture(layout.cartons());fixed={b.name:b for b in layout.fixed_components()}
    permission=selected_layer_top_permission(frozen,'carton_l02_c01')
    target=next(b for b in layout.cartons() if b.name=='carton_l03_c02')
    assert layer_top_option(target,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.55,permission) is None
    altered=deepcopy(permission);altered['allowed_target_ids'].append(target.name)
    with pytest.raises(ValueError,match='PERMISSION_CHANGED'):
        layer_top_option(target,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.55,altered)
    moved=OBB(target.center+[0,0,-.3],target.half_extents,target.rotation,target.name,target.category)
    with pytest.raises(ValueError,match='FROZEN_TARGET_CHANGED'):
        layer_top_option(moved,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.55,permission)


def test_connector_rejects_changed_receiver_or_route(layout):
    boxes=layout.cartons();frozen=FrozenCartonPopulation.capture(boxes);fixed={b.name:b for b in layout.fixed_components()}
    target=next(b for b in boxes if b.name=='carton_l02_c01')
    active=tuple(b for b in boxes if b.center[2]<=target.center[2])
    scene=SimpleNamespace(support_graph=SupportRelationGraph.build(active),snapshot={'scene_fingerprint':'fixture'},
        policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
    face=next(t for t in simulation_contact_targets(scene,target,[-1.825,.25,.6],.1)[0] if t.face_role=='TOP')
    permission=selected_layer_top_permission(frozen,target.name)
    option=layer_top_option(target,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.55,permission)
    c=LowTopReceiverConnector.__new__(LowTopReceiverConnector)
    c.option=option;c.layer_top_permission=permission;c.batch_root={'height_m':.55}
    c.validate_template(face)
    c.batch_root={'height_m':.6}
    with pytest.raises(MoveItUnavailable,match='PROCESS_SCOPE_CHANGED'):c.validate_template(face)
    c.batch_root={'height_m':.55};c.option=deepcopy(option)
    c.option['segment_directions_world']=[[-1.,0.,0.],[0.,0.,1.]]
    with pytest.raises(MoveItUnavailable,match='PROCESS_SCOPE_CHANGED'):c.validate_template(face)


def test_restore_actual_28_box_prefix_without_native_parent(layout,tmp_path):
    from layer_top_continuation import restore_layer_top_terminal,RECORDS
    source=ROOT/'docs/experiments/m710_min250_20261023'
    for name in RECORDS:(tmp_path/name).write_bytes((source/'records/v2'/name).read_bytes())
    with tarfile.open(source/'raw/formal-v2.tar.xz') as archive:
        for m in archive:
            if m.isfile() and m.name.startswith('formal-v2/task-') and m.name.endswith('/result.json'):
                dest=tmp_path/Path(m.name).relative_to('formal-v2');dest.parent.mkdir(parents=True,exist_ok=True)
                dest.write_bytes(archive.extractfile(m).read())
    frozen=FrozenCartonPopulation.capture(layout.cartons())
    state=BatchState(frozen,tuple(layout.cartons()),np.zeros(6),2.05)
    receipt=restore_layer_top_terminal(tmp_path,state)
    assert len(state.cartons)==12 and len(state.ledger)==28 and not state.cycles
    assert state.root()['predecessor_final_state_receipt'] is None
    assert state.height==.5499999999999999
    assert receipt['archived_root']['predecessor_final_state_receipt'] is not None
    assert receipt['first_target_id']=='carton_l02_c01' and not receipt['tree_retained']
