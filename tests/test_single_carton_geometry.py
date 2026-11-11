"""Numeric partial-face reconstruction and real CAD passive seal topology."""
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unloading_sim.workbench.single_carton.geometry import recover,corners,contact_candidates,seal_check,transform
from unloading_sim.workbench.single_carton.config import load,issues

ROOT=Path(__file__).resolve().parents[1]

def plane(width=.15,height=.1,rotation=None):
    x,y=np.meshgrid(np.linspace(-width/2,width/2,151),np.linspace(-height/2,height/2,101))
    p=np.stack((x,y,np.zeros_like(x)),axis=-1)
    R=np.eye(3) if rotation is None else rotation;p=p@R.T+[.02,-.01,.6]
    mask=np.zeros((131,181),bool);mask[15:-15,15:-15]=True
    padded=np.zeros((131,181,3));padded[15:-15,15:-15]=p
    return padded,mask

def test_one_face_center_is_half_thickness_behind_observed_plane():
    p,m=plane();b=recover(p,m,np.eye(4),prior=(.15,.1,.1))
    np.testing.assert_allclose(np.array(b['pose'])[:3,3],[.02,-.01,.65],atol=.001)
    np.testing.assert_allclose(b['faces'][0]['normal'],[0,0,-1],atol=1e-6)
    assert len(b['corners_m'])==8 and len(b['observed_faces'])==1 and len(b['inferred_faces'])==5
    assert not b['quality']['hidden_extent_measured']
    assert b['dimension_source'].startswith('EXPLICIT_DIMENSIONS')

@pytest.mark.parametrize('euler',[(15,20,30),(-20,0,60),(0,-35,-10)])
def test_oriented_visible_face_and_world_chain(euler):
    R=Rotation.from_euler('xyz',euler,degrees=True).as_matrix();p,m=plane(rotation=R)
    W=np.eye(4);W[:3,:3]=Rotation.from_euler('z',70,degrees=True).as_matrix();W[:3,3]=[.3,-.2,.1]
    b=recover(p,m,W,prior=(.15,.1,.1))
    expected=transform(W,[[.02,-.01,.6]+R[:,2]*.05])[0]
    np.testing.assert_allclose(np.array(b['pose'])[:3,3],expected,atol=.002)
    np.testing.assert_allclose(b['faces'][0]['normal'],-W[:3,:3]@R[:,2],atol=.002)
    assert b['quality']['surface_consistency']>.98

@pytest.mark.parametrize('case',['merged','insufficient','background','missing_boundary'])
def test_invalid_geometry_rejected_instead_of_fabricating_cuboid(case):
    p,m=plane(width=.3 if case=='merged' else .15)
    if case=='insufficient':p[:]=np.nan
    if case=='background':p[15:50,15:-15,2]+=.18
    if case=='missing_boundary':m[:,:60]=False
    with pytest.raises(ValueError):recover(p,m,np.eye(4),prior=(.15,.1,.1))

@pytest.fixture(scope='module')
def cad():
    return json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())

def test_all_cups_cannot_fit_but_confirmed_passive_valves_enable_six_cup_contact(cad):
    p,m=plane();box=recover(p,m,np.eye(4),prior=(.15,.1,.1));config=load(ROOT/'configs/workbench/single_carton.example.json')
    topology=dict(config['topology'],uncovered_cups_self_close=False)
    with pytest.raises(ValueError,match='DO_NOT_FIT'):contact_candidates(box,cad,topology)
    candidates=contact_candidates(box,cad,config['topology']);c=candidates[0]
    assert len(c['active_ids'])==6
    assert len(c['commanded_cup_ids'])==12 and len(c['expected_passively_closed_ids'])==6
    assert c['target_lip_contact_ids']==c['commanded_cup_ids']
    assert set(c['passive_lip_contact_ids'])==set(c['target_lip_contact_ids'])-set(c['active_ids'])
    assert set(c['rings_tcp'])==set(c['active_ids'])  # passive contact never becomes a seal
    assert c['pneumatic_mode']=='COMMON_VALVE_PASSIVE_SELF_CLOSING'
    assert c['seal_check']['minimum_edge_margin_m']>=.003
    assert seal_check(c['tcp_pose'],c['face'],c['rings_tcp'],c['suction_normal_tcp'])['valid']
    shifted=np.array(c['tcp_pose']);shifted[:3,3]+=.01*np.asarray(c['face']['normal'])
    assert not seal_check(shifted,c['face'],c['rings_tcp'],c['suction_normal_tcp'])['valid']
    tilted=np.array(c['tcp_pose']);tilted[:3,:3]=tilted[:3,:3]@Rotation.from_euler('x',5,degrees=True).as_matrix()
    assert not seal_check(tilted,c['face'],c['rings_tcp'],c['suction_normal_tcp'])['valid']

def test_current_field_parameters_and_source_bytes():
    c=load(ROOT/'configs/workbench/single_carton.example.json');assert issues(c)==[]
    assert c['carton_mass_kg']==2 and c['execution']['tool_mass_kg']==1.5
    assert c['execution']['controller_payload_kg']==3.5
    assert c['vacuum']['establish_commands']==[0x12] and c['vacuum']['release_commands']==[0x13]
    assert c['vacuum']['grip_below_kpa']==-60 and c['vacuum']['hold_below_kpa']==-30 and c['vacuum']['release_above_kpa']==-10
    assert c['vacuum']['grip_dwell_s']==c['vacuum']['release_dwell_s']==1
    assert c['vacuum']['grip_timeout_s']==c['vacuum']['release_timeout_s']==3
    import hashlib
    assert hashlib.sha256(Path(c['camera_extrinsics']).read_bytes()).hexdigest()=='73946bc457de625353fa65ad181a1de0ffdba9ba0b64ca0ae1ff3d52e7b325c7'
    raw=json.loads(Path(c['camera_extrinsics']).read_bytes());assert raw['status']=='SOLVED_REQUIRES_VALIDATION' # retained historical evidence
    c['vacuum']['release_dwell_s']=None;assert 'vacuum.release_dwell_s' in issues(c)


def test_actual_feedback_rejects_passive_edge_penetration_even_when_active_seal_is_valid():
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton.geometry import candidate_contact_check
    from unloading_sim.workbench.single_carton.runtime import TaskManager
    face=dict(center_m=[0.,0.,0.],normal=[0.,0.,1.],u=[1.,0.,0.],v=[0.,1.,0.],size_m=[.15,.1])
    seal=[[-.01,-.01,0.],[.01,-.01,0.],[.01,.01,0.],[-.01,.01,0.]]
    edge=[[.06,-.01,0.],[.09,-.01,0.],[.09,.01,0.],[.06,.01,0.]]
    candidate=dict(face=face,rings_tcp={'seal':seal},suction_normal_tcp=[0.,0.,-1.],
                   target_lip_rings_tcp={'seal':seal,'passive':edge},target_lip_contact_ids=['seal','passive'])
    T=np.eye(4)
    assert candidate_contact_check(T,candidate)['valid']
    T[:3,:3]=Rotation.from_euler('y',.009).as_matrix()
    assert seal_check(T,face,{'seal':seal},[0.,0.,-1.])['valid']
    actual=candidate_contact_check(T,candidate)
    assert not actual['valid'] and actual['terminal_lip_contact']['minimum_plane_gap_m']<-.0008
    manager=TaskManager.__new__(TaskManager);manager.model=SimpleNamespace(tcp=lambda q:T)
    events=[];manager.journal=lambda **event:events.append(event)
    with pytest.raises(ValueError,match='ACTUAL_CONTACT_POSE_NOT_SEALED'):
        manager.check_contact(np.zeros(6),{'candidate':candidate})
    assert not events[0]['seal']['terminal_lip_contact']['valid']

@pytest.mark.parametrize('value',[None,0.,3.,6.,float('nan')])
def test_loaded_payload_missing_or_empty_test_setting_rejected(value):
    c=load(ROOT/'configs/workbench/single_carton.example.json');c['controller_payload_kg']=value
    assert any(x.startswith('controller_payload_kg') for x in issues(c))
