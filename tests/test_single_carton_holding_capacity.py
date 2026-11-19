"""Source CAD areas, gravity, peel and finite-difference FK; no hardware."""
from copy import deepcopy
from pathlib import Path
import json
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.geometry import corners,contact_candidates
from unloading_sim.workbench.single_carton.holding_capacity import evaluate,settings,tier,GRAVITY_M_S2


def fixture(vertical=True,depth=.1):
    tool=json.loads(Path('configs/local/eco65_desktop/tool.json').read_text(encoding='utf-8'))
    config=json.loads(Path('configs/workbench/single_carton.example.json').read_text(encoding='utf-8'))
    T=np.eye(4);size=[.15,.1,depth]
    box=dict(pose=T.tolist(),size_m=size,corners_m=corners(T,size).tolist(),
        faces=[dict(center_m=[0,0,depth/2],normal=[0,0,1],u=[1,0,0],v=[0,1,0],size_m=[.15,.1],observed=True)])
    candidate=contact_candidates(box,tool,config['topology'])[0]
    if vertical:
        W=np.eye(4);W[:3,:3]=np.array([[1,0,0],[0,0,-1],[0,1,0]])
        box['pose']=(W@T).tolist();box['corners_m']=corners(box['pose'],size).tolist()
        candidate['tcp_pose']=(W@np.asarray(candidate['tcp_pose'])).tolist()
        for k in ('u','v','normal','center_m'):candidate['face'][k]=(W[:3,:3]@candidate['face'][k]).tolist()
    return box,candidate,tool,config


def test_one_cup_two_kg_vertical_side_is_rejected_six_pass():
    box,candidate,tool,config=fixture();six=evaluate(box,candidate,tool,config)
    assert len(candidate['active_ids'])==6 and six['valid']
    assert six['total_suction_n']==pytest.approx(191.1588019486)
    assert six['worst']['tangential_load_n']==pytest.approx(19.6133)
    one=deepcopy(candidate);one['active_ids']=one['active_ids'][:1]
    rejected=evaluate(box,one,tool,config)
    assert not rejected['valid'] and rejected['worst']['tangential_margin']<1
    assert rejected['total_suction_n']==pytest.approx(31.8598003248)


def test_vertical_translation_parallel_to_gravity_does_not_invent_peel():
    box,candidate,tool,config=fixture();before=evaluate(box,candidate,tool,config)
    candidate['tcp_pose'][2][3]+=.06
    after=evaluate(box,candidate,tool,config)
    np.testing.assert_allclose(after['worst']['peel_moment_tcp_nm'],before['worst']['peel_moment_tcp_nm'],atol=1e-12)


def test_actual_sent_fk_acceleration_and_60mm_offset_exceed_peel_limit():
    # A deep carton close to its peel limit, with a small 0.2 m/s² normal
    # acceleration. Pure static vertical translation cannot change peel.
    box,candidate,tool,config=fixture(depth=.217)
    t=np.linspace(0,1,201);poses=np.repeat(np.asarray(candidate['tcp_pose'])[None],len(t),axis=0)
    poses[:,1,3]+=.1*t*t
    before=evaluate(box,candidate,tool,config,poses,t)
    shifted=deepcopy(candidate);shifted['tcp_pose'][2][3]+=.06;poses[:,2,3]+=.06
    after=evaluate(box,shifted,tool,config,poses,t)
    assert before['valid'] and not after['valid']
    assert after['maximum_linear_acceleration_m_s2']==pytest.approx(.2,abs=1e-8)
    assert after['worst']['peel_margin']<1


def test_both_com_estimates_are_evaluated():
    box,candidate,tool,config=fixture();good=evaluate(box,candidate,tool,config);assert good['valid']
    box['measured_box']=deepcopy(box);box['measured_box']['pose'][1][3]+=.2
    result=evaluate(box,candidate,tool,config)
    assert not result['valid'] and result['worst']['com_source']=='measured_box'


@pytest.mark.parametrize('margin,expected',[(.99,-1),(1,0),(1.49,0),(1.5,1),(1.999,1),(2,2),(4,2)])
def test_capacity_tiers(margin,expected):assert tier(margin)==expected


def test_capacity_tier_precedes_posture_and_posture_breaks_ties():
    from unloading_sim.workbench.single_carton.holding_capacity import priority
    rows=[('not_clear',False,4,(0,)),('adequate',True,1.2,(0,)),('medium',True,1.7,(0,)),
          ('strong_bad_posture',True,2.1,(9,)),('strong_good_posture',True,2.01,(1,))]
    assert [r[0] for r in sorted(rows,key=lambda r:priority(*r[1:]))]==[
        'strong_good_posture','strong_bad_posture','medium','adequate','not_clear']


@pytest.mark.parametrize('key,value',[('friction_coefficient',.51),('safety_factor',1.9),('effective_area_factor',.81),('basis','')])
def test_field_capacity_settings_only_tighten(key,value):
    c=dict(holding_capacity=dict(friction_coefficient=.5,safety_factor=2,effective_area_factor=.8,basis='synthetic'))
    c['holding_capacity'][key]=value
    with pytest.raises(ValueError):settings(c)


def test_cpp_and_python_capacity_tiers_match(tmp_path):
    import shutil,subprocess
    compiler=shutil.which('g++')
    if not compiler:pytest.skip('Existing C++ compiler unavailable; run in resident Ubuntu environment')
    root=Path(__file__).resolve().parents[1]
    source=tmp_path/'capacity.cpp';binary=tmp_path/'capacity'
    source.write_text('#include "sc_capacity.h"\n#include <iostream>\nint main(){double x;while(std::cin>>x)std::cout<<m710::singleCartonCapacityTier(x)<<"\\n";}\n')
    subprocess.run([compiler,'-std=c++17','-I'+str(root/'ros2/m710_moveit_backend/src'),str(source),'-o',str(binary)],check=True)
    values=[.99,1,1.49,1.5,1.999,2,4]
    result=subprocess.run([str(binary)],input='\n'.join(map(str,values)),text=True,capture_output=True,check=True)
    assert list(map(int,result.stdout.split()))==[tier(x) for x in values]
