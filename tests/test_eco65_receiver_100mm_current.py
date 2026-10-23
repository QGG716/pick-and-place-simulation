"""100 mm travel is distinct from frozen-carton process receive heights."""
import numpy as np
import pytest
import yaml
from unloading_sim.eco65.model import ROOT
from unloading_sim.eco65.coupled_scene import make_coupled_spec, coupled_conveyor_state, build_coupled_scene
from unloading_sim.eco65.transverse_process import process_options
from test_eco65_coupled_scene import source_scene, vertices
from test_eco65_transverse_process import box


def config():
    return yaml.safe_load((ROOT/'configs/workcells/eco65_coupled_carton_v5.yaml').read_text())


def test_100mm_negative_tilt_fixed_feet_and_inherited_actual_height():
    source=source_scene();spec=make_coupled_spec(source,config())
    assert spec['low_height_m']==.100 and spec['high_height_m']==.450
    fixed=None
    for height in (.100,.150,.180,.23130922456,.33130922456,.43130922456,.450):
        state=coupled_conveyor_state(spec,height);m=state['mechanism']
        assert state['tilt_rad']==pytest.approx(np.arcsin((height-.180)/.4443197215708027))
        assert m['lower_endpoint_D_m'][2]==.180
        assert np.linalg.norm(np.array(m['upper_endpoint_D_m'])-m['lower_endpoint_D_m'])==pytest.approx(.4443197215708027)
        now={r['id']:r for r in state['world_objects'] if r['id'] in m['fixed_structure_ids']}
        if fixed is None:fixed=now
        assert fixed==now
        assert all(vertices(r)[:,2].min()>=-1e-9 for r in state['world_objects'])
        for stack in m['stacks']:
            for bar in stack['bars']:
                assert np.linalg.norm(np.diff(bar['endpoints_m'],axis=0))==pytest.approx(bar['length_m'])
    with pytest.raises(ValueError):coupled_conveyor_state(spec,.099)
    source['coupled_state']={'height_m':.3311827942858692}
    assert build_coupled_scene(source,config())['coupled_state']['height_m']==.3311827942858692


@pytest.mark.parametrize('bottom,heights',[(.18130922456,[.13130922456]),(.28130922456,[.23130922456,.13130922456]),(.38130922456,[.33130922456,.23130922456]),(.48130922456,[.43130922456,.33130922456])])
def test_100mm_does_not_clamp_receive_height_or_enable_unapproved_top(bottom,heights):
    spec=make_coupled_spec(source_scene(),config());b=box(bottom)
    result=process_options(b,[b],[b],spec,coupled_conveyor_state(spec,.331182794)['receiving'])
    assert not result['frozen']['top_allowed']
    assert [o['height_m'] for o in result['options']]==pytest.approx(heights)
    assert all(o['face']=='front' for o in result['options'])
    if not heights:
        assert all(r['reason']=='PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE' for r in result['rejected'])
