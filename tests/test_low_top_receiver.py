"""Literal low-row boundaries, named floor support and unchanged permissions."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.low_top_receiver import low_top_option,top_floor_support
from unloading_sim.geometry import OBB
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.support import SupportRelationGraph
from unloading_sim.right_front_eccentric import authorize


@pytest.fixture
def layout():
    return load_workcell_layout(Path(__file__).resolve().parents[1]/
        'configs/workcells/m710id70_unloading_layout_v1_mount_xy_minus_100.yaml')


def test_low_scope_is_four_original_layers_without_id_parsing(layout):
    boxes=tuple(OBB(b.center,b.half_extents,b.rotation,f'opaque-{i}',b.category)
                for i,b in enumerate(layout.cartons()))
    frozen=FrozenCartonPopulation.capture(boxes)
    fixed={b.name:b for b in layout.fixed_components()}
    low=[]
    for b in boxes:
        option=low_top_option(b,fixed['conveyor_transverse'],fixed['conveyor_longitudinal'],frozen,.6)
        if b.center[2] < 1.2:
            low.append(b.name)
            assert option['height_m']==.6 and option['pre_approach_height_m']==.6
            assert option['face']['geometric_face_id']=='z+'
            assert option['face']['working_axis_into_box']==[0.,0.,-1.]
            assert np.array_equal(np.asarray(option['face']['contact_pose'])[:3,:3],np.diag([1.,-1.,-1.]))
            assert option['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
            expected=.95 if b.center[2]>.9 else .65
            assert option['release_bottom_world_m']==pytest.approx(expected,abs=1e-12)
        else:assert option is None
    assert len(low)==20


def test_only_actual_named_floor_contact_gets_initial_support(layout):
    bottom=next(b for b in layout.cartons() if b.name=='carton_l00_c02')
    obstacles=layout.fixed_components()
    assert top_floor_support(bottom,obstacles,.0002)==['trailer_floor']
    raised=OBB(bottom.center+[0,0,.001],bottom.half_extents,bottom.rotation,'raised','carton')
    assert top_floor_support(raised,obstacles,.0002)==[]
    penetrated=OBB(bottom.center-[0,0,.001],bottom.half_extents,bottom.rotation,'bad','carton')
    assert top_floor_support(penetrated,obstacles,.0002)==[]
    renamed=[b for b in obstacles if b.name!='trailer_floor']
    with pytest.raises(ValueError,match='NAMED_FLOOR_MISSING'):top_floor_support(bottom,renamed,.0002)


def test_population_scope_does_not_authorize_lower_right_eccentric(layout):
    boxes=layout.cartons();frozen=FrozenCartonPopulation.capture(boxes)
    for name,allowed in [('carton_l07_c00',True),('carton_l06_c00',False)]:
        target=next(b for b in boxes if b.name==name)
        active=tuple(b for b in boxes if b.center[2]<=target.center[2]+1e-8)
        scene=SimpleNamespace(support_graph=SupportRelationGraph.build(active),snapshot={'scene_fingerprint':'fixture'},
            policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
        face=next(t for t in simulation_contact_targets(scene,target,[-1.425,.25,.6],.1)[0] if t.face_role=='FRONT')
        if allowed:assert authorize(face,frozen)['original_role']=='RIGHT_END'
        else:
            with pytest.raises(ValueError,match='ORIGINAL_TOP_LAYER_ONLY'):authorize(face,frozen)
