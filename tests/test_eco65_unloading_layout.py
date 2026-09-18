"""Bounded layout/geometry regressions; no complete trajectory or unloading run."""
import copy
import numpy as np
import pytest
import yaml
from unloading_sim.eco65.model import ROOT,LOCAL,OUTPUT,World,load_json,transform
from unloading_sim.eco65.unloading_layout import CONFIG,build_scene,audit_layout,UnloadingWorld,bounds,face_tcp,seal_check,support_at

RUN=ROOT/'outputs/eco65_desktop_layout/layout_v1_20260917_candidate02'
@pytest.fixture(scope='module')
def data():
    if not (LOCAL/'tool.json').exists():pytest.skip('Private actual CAD is required; no generic substitute')
    tool=load_json(LOCAL/'tool.json');cfg=yaml.safe_load((ROOT/'configs/workcells/eco65_desktop_unloading_layout_v1.yaml').read_text(encoding='utf-8'));scene=build_scene(cfg)
    return cfg,scene,tool

def test_layout_topology_literal_dimensions_and_no_chassis(data):
    c,s,t=data
    assert c['robot']['geometry_scale']==1 and c['robot']['mobility']=='fixed'
    assert s['base_position_m']==[-.445,.145,.024]
    assert c['conveyors']['transverse']['bounds_xy_m']==[-.36,-.08,-.125,.415]
    assert c['conveyors']['longitudinal']['bounds_xy_m']==[-1.05,-.08,-.415,-.135]
    assert all(s['wall_configuration'].values())
    assert not any('chassis' in o['id'] or 'wheel' in o['id'] for o in s['obstacles'])
    assert np.allclose(s['footprint']['minimum_static_size_m'],[1.782,.924])
    assert s['footprint']['desk_fit']=='PENDING_MEASURED_DESK_DIMENSIONS'

def test_nine_cartons_and_real_layer_support(data):
    c,s,t=data;a=audit_layout(s,t)
    assert a['status']=='PASS' and len(a['stack'])==9
    assert len({b['id'] for b in s['boxes']})==9
    assert sorted({round(b['pose'][2],6) for b in s['boxes']})==[.092,.252,.412]
    assert sum(b['currently_removable'] for b in a['stack'])==3
    assert all(b['support_valid'] for b in a['stack'])

def test_actual_all_seal_top_and_front_fit_and_small_face_fails(data):
    c,s,t=data;b=s['boxes'][0];T=transform(b['pose'])
    for face in ('top','front','right_facing'):
        result=seal_check(t,face_tcp(T,b['size_m'],face),T,b['size_m'],face)
        assert result['fits'] and result['rings']==12 and not result['independent_control_assumed']
    small=[.24,.20,.10]
    assert not seal_check(t,face_tcp(T,small,'front'),T,small,'front')['fits']

def test_transfer_owner_does_not_replace_actual_bridge_support(data):
    c,s,t=data;size=c['carton_stack']['carton_size_xyz_m']
    assert support_at(s,[-.22,-.13],size,.13)['coverage']==pytest.approx(1.)
    changed=copy.deepcopy(s);changed['obstacles']=[o for o in changed['obstacles'] if o['id']!='transfer_bridge']
    assert changed['regions']['transfer']['process_owner']=='longitudinal'
    assert support_at(changed,[-.22,-.13],size,.13)['coverage']==pytest.approx(.95)

def test_outlet_has_real_support_beyond_opening(data):
    c,s,t=data;size=c['carton_stack']['carton_size_xyz_m'];xy=c['conveyors']['outlet']['final_center_xy_m']
    assert xy[0]+size[0]/2<s['opening_x_m']
    assert support_at(s,xy,size,.13)['coverage']==pytest.approx(1.)
    changed=copy.deepcopy(s);changed['obstacles']=[o for o in changed['obstacles'] if o['id']!='outlet_bridge']
    assert support_at(changed,[-1.10,-.275],size,.13)['coverage']<.6

def test_roof_removal_and_hardware_cannot_silently_enter_design(data):
    c,s,t=data;c=copy.deepcopy(c);c['trailer']['panels']['roof']=False
    with pytest.raises(AssertionError):build_scene(c)
    c['trailer']['panels']['roof']=True;c['validation']['enable_hardware']=True
    with pytest.raises(PermissionError):build_scene(c)

def test_pose_evidence_has_all_geometry_and_separate_support_status(data):
    if not (RUN/'reports/poses.json').exists():pytest.skip('Generate the limited pose report first')
    c,s,t=data;report=load_json(RUN/'reports/poses.json');w=UnloadingWorld(t,s)
    try:
        assert len(w.boxes)==9 and len(report['candidates'])==9
        assert w.valid(report['home']['q'])
        for panel in ('trailer_left','trailer_right','trailer_roof'):
            assert panel in w.geoms
            assert any(panel in {a['name'],b['name']} and ('robot' in {a['kind'],b['kind']} or 'conservative_rigid' in {a['kind'],b['kind']}) for a,b in w.pairs)
        for r in report['candidates']:
            assert r['full_path_status']=='NOT_EVALUATED'
            if r['status']=='POSE_VALID':
                w.select_target(r['box_id'],np.array(r['box_pose']),r['face'])
                assert w.valid(r['q'])
                assert seal_check(t,w.robot.fk(r['q']),np.array(r['box_pose']),w.boxes[r['box_id']]['size_m'],r['face'])['fits']
        middle=next(r for r in report['candidates'] if r['id']=='pick_front_supported_middle')
        assert middle['removal_status']=='SUPPORT_DEPENDENCY_BLOCKED' and middle['supports_boxes']
        failed=next(r for r in report['candidates'] if r['id']=='place_transverse_front')
        assert failed['status']=='IK_FOUND_COLLISION' and failed['collision_solutions']
    finally:w.close()

def test_shared_model_old_single_box_entry_still_matches(data):
    tool=load_json(LOCAL/'tool.json');scene=load_json(LOCAL/'scene.json');w=World(tool,scene)
    try:
        q=load_json(OUTPUT/'reports/home.json')['q']
        assert len(w.boxes)==1 and w.valid(q,'initial')
        plan=load_json(OUTPUT/'trajectory/plan.json');contact=next(p for p in plan['segments'] if p['phase']=='contact')['q'][-1]
        assert w.valid(contact,'contact')
        assert w.seal_fit(contact,transform(scene['box']['pose']))[0]
    finally:w.close()
def test_functional_regions_and_local_support_ids(data):
    from unloading_sim.eco65.unloading_layout import functional_regions
    c,s,t=data;r=functional_regions(s)
    assert np.allclose(r['conveyor_transverse']['receiving_bounds_xy_m'],[-.36,-.08,-.105,.395])
    assert np.allclose(r['conveyor_longitudinal']['receiving_bounds_xy_m'],[-1.03,-.10,-.415,-.135])
    assert r['transfer']['process_owner']=='longitudinal'
    assert not r['outlet']['disappearance_allowed']
    assert support_at(s,[-.52,-.275],[.24,.20,.16],.13)['surface_ids']==['conveyor_longitudinal_belt']