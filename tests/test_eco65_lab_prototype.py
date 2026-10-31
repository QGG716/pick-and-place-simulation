"""Directed lab geometry/protocol checks; not native trajectory evidence."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml
from unloading_sim.eco65.quarter_prototype import carton_grid,mechanism_spec
from unloading_sim.eco65.coupled_scene import coupled_conveyor_state
from unloading_sim.eco65.transverse_process import PROTOTYPE_POLICY,freeze_box,process_options,corners
from unloading_sim.eco65.outfeed import outfeed_path,support_height


@pytest.fixture
def config():
    return yaml.safe_load(Path('configs/workcells/eco65_lab_prototype_v2.yaml').read_text(encoding='utf-8'))


@pytest.fixture
def spec(config):return mechanism_spec(config,'directed-test',{})


@pytest.mark.parametrize('count,centres,width',[(3,[-.105,0,.105],.310),(4,[-.1575,-.0525,.0525,.1575],.415),(5,[-.21,-.105,0,.105,.21],.520)])
def test_ground_grid_literal_dimensions(config,count,centres,width):
    config['cartons']['columns']=count;boxes=carton_grid(config['cartons'])
    assert len(boxes)==4*count
    assert [b['pose'][1] for b in boxes[:count]]==pytest.approx(centres)
    v=np.vstack([corners(b) for b in boxes])
    assert v.min(0)==pytest.approx([0,-width/2,0])
    assert v.max(0)==pytest.approx([.15,width/2,.4])
    assert all(b['supported_by']==['trailer_floor'] for b in boxes[:count])
    assert all(b['size_m']==[.15,.1,.1] for b in boxes)


def test_no_hidden_pedestal_or_layer_gap(config):
    for key,value in [('bottom_z_m',.18),('layer_gap_m',.005)]:
        cfg=deepcopy(config['cartons']);cfg[key]=value
        with pytest.raises(ValueError,match='COMMON_GROUND'):carton_grid(cfg)


def test_explicit_even_column_target_does_not_require_unique_middle(config,spec):
    from test_eco65_native import synthetic_twelve_carton_scene
    from unloading_sim.eco65.native_contact import validate_task_contract
    from unloading_sim.eco65.quarter_prototype import SCHEMA
    scene=synthetic_twelve_carton_scene();config['cartons']['columns']=4
    scene['schema']=SCHEMA;scene['prototype']=dict(configuration=config)
    scene['boxes']=carton_grid(config['cartons']);target=scene['boxes'][12]
    scene['box']=target;scene['target_box_id']=target['id']
    state=coupled_conveyor_state(spec,.3);scene['native_world_objects']=state['world_objects']
    scene['native_task'].update(target_id=target['id'],inventory_count=16,
        stack_carton_ids=sorted(b['id'] for b in scene['boxes']),original_support_names=target['supported_by'],
        receiving=state['receiving'],target_selection=dict(schema='eco65_explicit_exposed_target_v1',
            target_id=target['id'],inventory_scope='FROZEN_CONFIGURED_CARTONS',source='USER_REQUEST'))
    assert validate_task_contract(scene)['target_id']=='carton_r00_l03_c00'


@pytest.mark.parametrize('bottom',[0.,.099,.100,.101,.3,.5])
def test_top_has_no_height_gate_but_occlusion_still_rejects(config,spec,bottom):
    b=dict(id='target',pose=[.075,0,bottom+.05],size_m=[.15,.1,.1])
    above=dict(id='above',pose=[.075,0,bottom+.15],size_m=[.15,.1,.1])
    receiver=coupled_conveyor_state(spec,.1)['receiving']
    assert freeze_box(b,[b],policy=PROTOTYPE_POLICY)['top_allowed']
    options=process_options(b,[b],[b],spec,receiver)
    top=next(o for o in options['options'] if o['face']=='top')
    assert top['template']=='TOP_LIFTED_TRANSVERSE_RECEIVER'
    assert .1<=top['height_m']<=.525
    blocked=process_options(b,[b,above],[b,above],spec,receiver)
    assert not any(o['face']=='top' for o in blocked['options'])
    assert any(r['face']=='top' and r['reason']=='FACE_NOT_EXPOSED' for r in blocked['rejected'])
    if bottom<=.101:assert not any(o['face']=='front' for o in options['options'])


def test_frozen_high_low_are_not_clamped(spec):
    b=dict(id='target',pose=[.075,0,.35],size_m=[.15,.1,.1])
    rows=process_options(b,[b],[b],spec,coupled_conveyor_state(spec,.1)['receiving'])['options']
    front=[r for r in rows if r['face']=='front']
    assert [r['height_m'] for r in front]==pytest.approx([.2875,.1875])
    assert [r['release_bottom_world_m'] for r in front]==pytest.approx([.3,.3])
    assert [r['expected_drop_m'] for r in front]==pytest.approx([.0125,.1125])


def test_full_rigid_stroke_and_fixed_feet(spec):
    fixed=None
    for h in [.1,.15,.2,.3,.4,.5,.525]:
        state=coupled_conveyor_state(spec,h);m=state['mechanism']
        a,b=np.array(m['upper_endpoint_D_m']),np.array(m['lower_endpoint_D_m'])
        assert a==pytest.approx([-.225,-.2,h]);assert b[2]==pytest.approx(.15)
        assert np.linalg.norm(a-b)==pytest.approx(.700)
        assert state['receiving']['bounds_D_m'][0][:2]==pytest.approx([-.225,-.275])
        assert state['receiving']['bounds_D_m'][1][:2]==pytest.approx([-.05,.275])
        assert spec['longitudinal_width_m']==.15 and spec['transverse_width_m']==pytest.approx(.175)
        now=[r for r in state['world_objects'] if r['id'] in m['fixed_structure_ids']]
        if fixed is None:fixed=now
        assert now==fixed
        for stack in m['stacks']:
            for bar in stack['bars']:assert np.linalg.norm(np.diff(bar['endpoints_m'],axis=0))==pytest.approx(bar['length_m'])


@pytest.mark.parametrize('height',[.1,.15,.3,.525])
def test_ideal_outfeed_uses_finite_real_surfaces_and_full_exit(config,spec,height):
    state=coupled_conveyor_state(spec,height);p=np.eye(4);p[:3,3]=[-.1375,0,height+.05]
    path=outfeed_path(p,[.15,.1,.1],spec,state,config['outfeed'])
    positions=np.array([s['pose'] for s in path['samples']])[:,:3,3]
    assert np.max(np.linalg.norm(np.diff(positions,axis=0),axis=1))<=.010+1e-9
    assert positions[-1,0]+.075<-.8
    assert path['physical_outfeed'] is False
    for row in path['samples']:
        h,_=support_height(row['pose'],[.15,.1,.1],spec,state)
        assert row['pose'][2][3]-.05==pytest.approx(h)
    p[0,3]=-2
    with pytest.raises(ValueError,match='NO_REAL_SUPPORT'):support_height(p,[.15,.1,.1],spec,state)
