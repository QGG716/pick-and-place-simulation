"""Requested J5/roll/lift changes: geometry and source-bound soft ranking only."""
import json
from pathlib import Path
import subprocess
import numpy as np
import pytest
from copy import deepcopy
from test_single_carton_planner import case  # noqa: F401
from test_review2_wrist_composite import composite  # noqa: F401
from unloading_sim.workbench.single_carton import posture
from unloading_sim.workbench.single_carton.config import load,wrist_branch_limits,wrist_branch_violation
from unloading_sim.workbench.single_carton.geometry import contact_candidates,contact_search_settings,seal_check
from unloading_sim.workbench.coordinates import Installation,CoordinateModel
from unloading_sim.workbench.ui import ROOT,URDF,URDF_SHA


@pytest.fixture(scope='module')
def policy():
    c=load(ROOT/'configs/workbench/single_carton.example.json');i=Installation.load(c['execution']['installation'])
    model=CoordinateModel(URDF,URDF_SHA,i.world_from_base,i.tool)
    return c,model,posture.bind(c,model)


def test_requested_j5_crossing_keeps_other_limits_and_stop_lock(policy):
    c,_,_=policy;limits=wrist_branch_limits(c);start=np.deg2rad([81.,-12.,-54.,-16.,86.,-5.])
    states=[start.copy() for _ in range(3)]
    for q,v in zip(states,[20.,0.,-60.]):q[4]=np.deg2rad(v)
    assert wrist_branch_violation(limits,start,states) is None
    states[-1][3]=start[3]+np.deg2rad(91.)
    assert wrist_branch_violation(limits,start,states) is None
    assert wrist_branch_violation(dict(limits,max_j4_travel_deg=90.),start,states)['reason']=='J4_TRAVEL_EXCEEDED'
    states[-1][5]=start[5]+np.deg2rad(181.)
    assert wrist_branch_violation(limits,start,states)['reason']=='J6_TRAVEL_EXCEEDED'
    with pytest.raises(ValueError):wrist_branch_limits(dict(wrist_branch_limits=dict(limits,max_j6_travel_deg=None)))
    assert c['execution_lock']['locked'] and c['tool_robot_clearance']['clearance_m']==.04


def test_soft_preference_is_geometry_based_not_a_joint_lock(policy):
    _,model,p=policy;start=np.deg2rad([81.237,-11.987,-53.993,-15.809,85.781,-4.639])
    down=np.deg2rad([15.912,-121.822,121.227,2.708,59.876,-1.310])
    upright=np.deg2rad([15.912,3.575,-78.295,-103.168,-59.876,-1.310])
    assert posture.key(p,upright,start)<posture.key(p,down,start)
    relaxed=upright.copy();relaxed[1]+=.5
    assert np.isfinite(posture.metrics(p,relaxed,start)['score'])
    wound=upright.copy();wound[5]+=2*np.pi
    assert posture.key(p,upright,start)<posture.key(p,wound,start)
    assert wound[5]==upright[5]+2*np.pi
    frames=model.frames(upright);v=frames['link_3'][:3,3]-frames['link_2'][:3,3]
    assert posture.metrics(p,upright,start)['upper_from_up_deg']==pytest.approx(np.rad2deg(np.arccos(v[2]/np.linalg.norm(v))))
    # An external world frame must not turn the robot-base vertical preference.
    rotated=deepcopy(model);rotated.robot.base_transform[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    assert posture.metrics(posture.bind(policy[0],rotated),upright,start)==posture.metrics(p,upright,start)


def test_rotated_and_raised_candidates_keep_actual_seal_contours(policy):
    c,_,_=policy;tool=json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())
    face=dict(observed=True,center_m=[-.52,0,.4],normal=[1.,0.,0.],u=[0.,0.,-1.],v=[0.,1.,0.],size_m=[.128,.17])
    box=dict(faces=[face]);cs=contact_candidates(box,tool,c['topology'],c['contact_search'])
    assert len(cs)==32 and {x['contact_search']['lift_along_face_m'] for x in cs}==set(c['contact_search']['lift_offsets_m'])
    assert any(0<len(x['active_ids'])<6 and x['contact_search']['lift_along_face_m']>0 for x in cs)
    flange_tcp=np.asarray(tool['T_flange_tcp']);spin=np.diag([-1.,-1.,1.,1.])
    for x in cs:
        cfg=x['contact_search'];before=np.asarray(cfg['reference_tcp_pose']);after=np.asarray(x['tcp_pose'])
        expected=before@np.linalg.inv(flange_tcp)@spin@flange_tcp;expected[2,3]+=cfg['lift_along_face_m']
        np.testing.assert_allclose(after,expected,atol=1e-12)
        assert seal_check(after,face,x['rings_tcp'],x['suction_normal_tcp'])['valid']
        assert len(x['commanded_cup_ids'])==len(x['target_lip_contact_ids'])==12
        assert set(x['expected_passively_closed_ids'])==set(x['commanded_cup_ids'])-set(x['active_ids'])
    # Raising is not permission to seal a cup that lies outside the observed face.
    x=cs[0];bad=np.asarray(x['tcp_pose']).copy();bad[2,3]+=.2
    assert not seal_check(bad,face,x['rings_tcp'],x['suction_normal_tcp'])['valid']
    with pytest.raises(ValueError,match='DO_NOT_FIT'):
        contact_candidates(box,tool,dict(c['topology'],uncovered_cups_self_close=False),c['contact_search'])


def test_invalid_preferences_are_not_silently_ignored(policy):
    c,_,_=policy
    with pytest.raises(ValueError):contact_search_settings(dict(c['contact_search'],lift_offsets_m=[0.,float('nan')]))
    with pytest.raises(ValueError):posture.settings(dict(posture_preference=dict(c['posture_preference'],upper_up_weight=-1.)))


def test_actual_cpp_posture_order_matches_python(policy,tmp_path):
    _,model,p=policy
    cpp=tmp_path/'score.cpp';exe=tmp_path/'score'
    cpp.write_text('#include "sc_posture.h"\n#include <iostream>\nint main(){nlohmann::json j;std::cin>>j;auto r=j.at("reference").get<std::vector<double>>();nlohmann::json out=nlohmann::json::array();for(auto q:j.at("samples"))out.push_back(m710::singleCartonPostureKey(j.at("policy"),q.get<std::vector<double>>(),r));std::cout<<out.dump();}\n')
    subprocess.run(['g++','-std=c++17','-O1','-I/usr/include/eigen3','-I'+str(ROOT/'ros2/m710_moveit_backend/src'),str(cpp),'-o',str(exe)],check=True)
    q0=np.deg2rad([81.,-12.,-54.,-16.,86.,-5.]);lo,hi=model.robot.joint_limits.T
    samples=np.random.default_rng(6518).uniform(lo,hi,(128,6)).tolist()
    result=subprocess.run([str(exe)],input=json.dumps(dict(policy=p,reference=q0.tolist(),samples=samples)),text=True,capture_output=True,check=True)
    cpp_keys=json.loads(result.stdout);py_keys=[posture.key(p,q,q0) for q in samples]
    assert [x[0] for x in cpp_keys]==[x[0] for x in py_keys]
    assert sorted(range(len(samples)),key=lambda i:cpp_keys[i])==sorted(range(len(samples)),key=lambda i:py_keys[i])


def test_composite_and_python_choose_same_preferred_branch(policy,case,composite):
    from unloading_sim.workbench.single_carton.native_flow import NativeFlow,ReplayClient
    c,_,preference=policy;p,obs,box,folder,_,_=case
    q0=np.deg2rad([81.237,-11.987,-53.993,-15.809,85.781,-4.639]).tolist()
    good=np.deg2rad([16.,1.49,-68.74,-110.64,-59.79,-1.31]).tolist()
    down=np.deg2rad([15.912,-121.822,121.227,2.708,59.876,-1.310]).tolist()
    p.posture_policy=preference;p.wrist_limits=wrist_branch_limits(c);p.wrist_reference=q0
    candidate=contact_candidates(box,p.source['tool'],p.site['topology'])[0]
    world=[dict(id=box['id'],pose=box['pose'],size=box['size_m'],category='carton')]
    replies=[]
    class Client:
        count=0
        def request(self,request,**kwargs):
            if request['op']=='ik':
                self.count+=1
                q=good if request['goal_pose']==candidate['tcp_pose'] or self.count%2==0 else down
                result=dict(status='SUCCESS',q=q,position_error_m=0.,orientation_error_rad=0.)
            else:result=dict(native_valid=True,legacy_intersection_valid=True,failure=None)
            replies.append(deepcopy(result));return result
    client=Client();p.client=client
    request=NativeFlow(p).screen_request(q0,world,box,[candidate],[0],client,folder)
    expected=p.screen_candidate(q0,world,box,candidate,folder)
    assert expected['screen']['approach_q']==good
    native=composite(request,replies,folder)
    replay=ReplayClient(client,None);replay.add(native);p.client=replay
    assert p.screen_candidate(q0,world,box,candidate,folder)==expected
    assert replay.unused()==0 and native['script_consumed']==len(replies)
