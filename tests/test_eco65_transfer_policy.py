"""Directed edge, endpoint, finite-route and segmented native-chain contracts."""
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.eco65.transfer_policy import (j1_valid,j1_direction,stage_chain_valid,
    request_budgets,clear_stack_extractions,endpoint_fk)
from unloading_sim.eco65.native_task import validate_plan_contract
from test_eco65_evaluation import chain_fixture,resign,basic_result


def test_direction_rejects_interior_reversal_despite_legal_endpoints():
    q=np.zeros((4,6));q[:,0]=[2,1,1.2,0]
    assert not j1_valid(q,-1)
    assert j1_valid(q[[0,-1]],-1)
    assert j1_valid(q[::-1][[0,-1]],1)
    assert not j1_valid(q[::-1],1)
    assert j1_direction('residence')==1 and j1_direction('transit')==-1


@pytest.mark.parametrize('count',[1,2,3,4])
def test_only_transit_can_repeat(count):
    names=['pregrasp','contact','extraction']+['transit']*count+['place','withdrawal','residence']
    assert stage_chain_valid([dict(stage=n) for n in names])
    for wrong in [names+['residence'],names[:3]+names[3+count:],names[:2]+['contact']+names[2:]]:
        assert not stage_chain_valid([dict(stage=n) for n in wrong])


def test_budget_reserves_output_and_return_before_rpc_watchdog():
    for remaining in (11,60,600):
        b=request_budgets(remaining,120,300)
        assert b['allowed_planning_time_s']>0
        assert b['allowed_planning_time_s']+b['output_check_timeout_s']+b['return_reserve_s']<=b['request_budget_s']
        assert b['request_budget_s']<=remaining
    with pytest.raises(ValueError):request_budgets(5,120,300)


def test_scene_derived_full_carton_front_clearance_literal_dimensions():
    scene=dict(target_box_id='target',boxes=[dict(id='neighbor',pose=[.205,.102,.53],size_m=[.15,.1,.1])])
    T=np.eye(4);T[:3,3]=[.205,0,.53];tcp=np.eye(4);tcp[:3,3]=[.0742,.0159,.58]
    options=clear_stack_extractions(scene,T,[.15,.1,.1],tcp,'top')
    assert len(options)==3
    np.testing.assert_allclose(options[0][0][:3,3],[-.0892,.0159,.59],atol=1e-12)
    assert options[0][1]['target_final_rear_x_m']==pytest.approx(.1166)
    assert options[2][1]['target_final_rear_x_m']==pytest.approx(.0666)
    assert clear_stack_extractions(scene,T,[.15,.1,.1],tcp,'front')==[]


def test_plan_recomputes_fk_instead_of_trusting_saved_zero_errors(monkeypatch):
    snapshot,plan=chain_fixture(monkeypatch)
    plan['stages'][1]['native_request']['goal_pose'][0][3]+=.005
    assert plan['stages'][1]['native_result']['endpoint_fk']['position_error_m']==0
    resign(plan)
    with pytest.raises(ValueError,match='PLAN_ACTUAL_ENDPOINT_FK_FAILED'):validate_plan_contract(plan,snapshot)


def test_real_math_endpoint_fk_detects_rotation_and_translation():
    class Robot:
        def fk(self,q):return np.eye(4)
    goal=np.eye(4);goal[:3,3]=[.003,.004,0]
    assert endpoint_fk(Robot(),np.zeros(6),goal)['position_error_m']==pytest.approx(.005)


def test_segmented_synthetic_contract_preserves_each_native_parent_and_attachment(monkeypatch):
    """Synthetic protocol fixture only; no claim of real motion generation."""
    snapshot,plan=chain_fixture(monkeypatch);original=plan['stages'][3]
    start,end=np.asarray(original['q']);middle=(start+end)/2
    segments=[];parent=original['parent_stage_id']
    for name,points in [('transit-a',[start,middle]),('transit-b',[middle,end])]:
        s=deepcopy(original);s.update(stage_id=name,parent_stage_id=parent,q=np.asarray(points).tolist(),times=[0.,1.])
        req=s['native_request'];req.pop('goal_pose');req.update(stage_id=name,parent_stage_id=parent,
            q_start=s['q'][0],q_goal=s['q'][-1],planner_id='PTP')
        raw=s['native_result'];raw.update(stage_id=name,parent_stage_id=parent,
            points=[dict(q=q,t=t,v=[0.]*6,a=[0.]*6) for q,t in zip(s['q'],s['times'])])
        basic_result(s);raw['joint_goal_state_status']='PASS';segments.append(s);parent=name
    plan['stages'][3:4]=segments
    place=plan['stages'][5];place['parent_stage_id']=parent
    place['native_request']['parent_stage_id']=place['native_result']['parent_stage_id']=parent
    resign(plan);assert validate_plan_contract(plan,snapshot)
    for key,value in [('parent_stage_id','unrelated'),('attachment',None)]:
        changed=deepcopy(plan);changed['stages'][4][key]=value;resign(changed)
        with pytest.raises(ValueError):validate_plan_contract(changed,snapshot)
