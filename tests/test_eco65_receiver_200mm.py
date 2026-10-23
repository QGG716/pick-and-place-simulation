"""Directed checks for the 200 mm travel limit; no native execution evidence."""
import numpy as np
import pytest
import yaml
from unloading_sim.eco65.model import ROOT
from unloading_sim.eco65.coupled_scene import make_coupled_spec, coupled_conveyor_state, build_coupled_scene
from unloading_sim.eco65.transverse_process import process_options
from test_eco65_coupled_scene import source_scene, vertices
from test_eco65_transverse_process import box


def config():
    return yaml.safe_load((ROOT/'configs/workcells/eco65_coupled_carton_v3.yaml').read_text())


def test_200mm_limit_keeps_rigid_geometry_fixed_feet_and_actual_height():
    source=source_scene();spec=make_coupled_spec(source,config())
    assert spec['low_height_m']==.200 and spec['high_height_m']==.450
    states=[coupled_conveyor_state(spec,h) for h in (.2,.231182794,.331182794,.431182794,.45)]
    fixed=None
    for state in states:
        m=state['mechanism'];rows=state['world_objects']
        assert state['tilt_rad']>0
        assert m['lower_endpoint_D_m'][2]==.180
        assert np.linalg.norm(np.array(m['upper_endpoint_D_m'])-m['lower_endpoint_D_m'])==pytest.approx(.4443197215708027)
        now={r['id']:r for r in rows if r['id'] in m['fixed_structure_ids']}
        if fixed is None:fixed=now
        assert fixed==now
        assert all(vertices(r)[:,2].min()>=-1e-9 for r in rows)
        for stack in m['stacks']:
            assert all(np.linalg.norm(np.diff(bar['endpoints_m'],axis=0))==pytest.approx(bar['length_m']) for bar in stack['bars'])
    with pytest.raises(ValueError):coupled_conveyor_state(spec,.199)
    source['coupled_state']={'height_m':.3311827942858692}
    scene=build_coupled_scene(source,config())
    assert scene['coupled_state']['height_m']==.3311827942858692
    source['coupled_state']['height_m']=.18
    with pytest.raises(ValueError):build_coupled_scene(source,config())
    assert build_coupled_scene(source,config(),initial_height_m=.3311827942858692)['coupled_state']['height_m']==.3311827942858692


@pytest.mark.parametrize('bottom,heights',[(.181182794,[]),(.281182794,[.231182794]),(.381182794,[.331182794,.231182794]),(.481182794,[.431182794,.331182794])])
def test_actual_layers_do_not_clamp_or_fall_back_to_top(bottom,heights):
    spec=make_coupled_spec(source_scene(),config());b=box(bottom)
    result=process_options(b,[b],[b],spec,coupled_conveyor_state(spec,.331182794)['receiving'])
    assert not result['frozen']['top_allowed']
    assert [o['height_m'] for o in result['options']]==pytest.approx(heights)
    assert all(o['face']=='front' for o in result['options'])
    if not heights:assert all(r['reason']=='PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE' for r in result['rejected'])
