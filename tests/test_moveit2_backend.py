"""Protocol/continuation regressions. Fake transport here is NOT native evidence."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.moveit2_backend import (MoveItLayoutConnector, ResidentMoveItClient,
    MoveItUnavailable, validate_native_result, JOINT_NAMES)
from unloading_sim.moveit2_timing import native_timing_floor, preserve_native_durations
from unloading_sim.timing import JointMotionLimits, time_parameterize_joint_path


def candidate(midpoint=None):
    path=[[0.]*6,[.1]*6] if midpoint is None else [[0.]*6,midpoint,[.1]*6]
    return dict(status='SUCCESS',joint_names=JOINT_NAMES.copy(),points=[dict(q=q,v=[0.]*6,a=[0.]*6,t=float(i)) for i,q in enumerate(path)])


def test_optional_dependency_and_explicit_failure():
    with pytest.raises(MoveItUnavailable,match='MOVEIT2_UNAVAILABLE'):
        ResidentMoveItClient(None)
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    assert LayoutTrajectoryConnector is not None


@pytest.mark.parametrize('change,error',[
    (lambda r:r['joint_names'].reverse(),'JOINT_ORDER'),
    (lambda r:r['points'][0]['q'].__setitem__(0,.01),'START_CHANGED'),
    (lambda r:r['points'][1].__setitem__('t',0.),'TIMES'),
    (lambda r:r['points'][-1]['v'].__setitem__(0,.01),'BOUNDARY_VELOCITY'),
])
def test_native_output_cannot_repair_start_or_change_contract(change,error):
    result=candidate();change(result)
    with pytest.raises(ValueError,match=error):
        validate_native_result(result,np.zeros(6),np.full(6,.1),JOINT_NAMES)


def test_authority_rejection_continues_ompl_and_deduplicates():
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector)
    calls=[]; checks=[]
    responses=iter([candidate(),candidate(),candidate([.04,.05,.05,.05,.05,.05])])
    def request(r,**kwargs):
        assert r['candidate_id']=='fixed-existing-candidate'
        calls.append((r['pipeline_id'],r['planner_id']));return next(responses)
    c.native=SimpleNamespace(request=request);c.native_identity={};c.native_joint_names=JOINT_NAMES
    c._candidate_identity="fixed-existing-candidate"
    c.native_evidence=[];c.native_verified=[];c.native_compliant=set();c.stack_carton_names=set()
    c.robot_state_validator=SimpleNamespace(base_support_obstacle_name='chassis',contact_target_name=None)
    c.native_tools=[]
    c.collision_policy=SimpleNamespace(poc_pair_clearance=True,allows_stack_planning_contact=lambda _:False,compliant_cup_neighbor_contact_mode='ignore',to_mapping=lambda:{})
    c.budget=SimpleNamespace(stage_connection_attempts=2,receiver_runtime_clearance_reserve_m=.003,edge_resolution_rad=.04);c.flange_from_virtual_task_tcp=np.eye(4)
    c._deadline_reached=lambda:False;c._context_identity=lambda *a,**k:'fixed'
    def check(path,*a,**k):
        checks.append(path);return dict(reason='PAIR_GAP',objects=['tool','wall']) if len(checks)==1 else None
    c._path_failure=check;c._state_failure=lambda *a,**k:None
    path,failure,evidence=c._native_plan(np.zeros(6),np.full(6,.1),[],seed=1,stage='pregrasp')
    assert failure is None and len(path)==3 and len(checks)==2
    assert calls==[('pilz_industrial_motion_planner','PTP'),('ompl','RRTConnectkConfigDefault'),('ompl','RRTConnectkConfigDefault')]
    assert evidence['attempts'][1]['authoritative_status']=='DUPLICATE_REJECTED_PATH'
    assert evidence['attempts'][0]['failure']['objects']==['tool','wall']


def test_native_duration_floor_preserved_on_actual_executor_edges():
    result=candidate([.05]*6);result['points'][1]['t']=7.;result['points'][2]['t']=15.
    path=[p['q'] for p in result['points']]
    floors,used=native_timing_floor(path,[result])
    assert floors==[7.,8.] and used[0]['path_range']==[0,2]
    limits=JointMotionLimits(velocity=np.ones(6),acceleration=np.ones(6),jerk=np.ones(6))
    timed=time_parameterize_joint_path([path[0],path[2]],limits)
    retained=preserve_native_durations(timed,dict(path=path,native_backend=dict(minimum_edge_seconds=floors)),[0,2])
    assert retained.duration_seconds==15.
    assert retained.audit(limits)['within_limits']
    np.testing.assert_array_equal(retained.positions,timed.positions)
    assert np.max(np.abs(retained.sample(0)[1]))==0
    assert np.max(np.abs(retained.sample(15)[1]))==0


def test_changed_path_cannot_inherit_native_timing():
    original=candidate();changed=deepcopy([p['q'] for p in original['points']]);changed[1][1]+=.001
    assert native_timing_floor(changed,[original])[1]==[]


def test_cancelled_transport_discards_late_response():
    import sys
    script = "import sys,time,json; r=json.loads(sys.stdin.readline()); time.sleep(2); print(json.dumps({'request_id':r['request_id'],'status':'SUCCESS'}),flush=True)"
    client=ResidentMoveItClient([sys.executable,"-u","-c",script])
    with pytest.raises(MoveItUnavailable,match="CANCELLED"):
        client.request(dict(op="test"),cancelled=lambda:True)
    assert client.process.poll() is not None


def test_response_generation_mismatch_is_fatal():
    import sys
    script = "import sys,json; sys.stdin.readline(); print(json.dumps({'request_id':'obsolete','status':'SUCCESS'}),flush=True)"
    client=ResidentMoveItClient([sys.executable,"-u","-c",script])
    with pytest.raises(MoveItUnavailable,match="REQUEST_ID_MISMATCH"):
        client.request(dict(op="test"))
    assert client.process.poll() is not None


def test_actual_nonzero_start_velocity_is_not_replaced_by_zero():
    scene=SimpleNamespace(snapshot={"robot":{"qd_rad_s":[0.,0.,.01,0.,0.,0.]}})
    with pytest.raises(MoveItUnavailable,match="START_VELOCITY"):
        MoveItLayoutConnector.from_existing(SimpleNamespace(),scene)


def test_capability_is_adapter_scope_and_has_no_reachability_claim():
    from unloading_sim.moveit2_backend import linear_capability
    a=np.eye(4);b=a.copy();b[0,3]=2.
    assert linear_capability(a,b,a,stage='transit',location='test')['supported']
    b[:3,:3]=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
    result=linear_capability(a,b,a,stage='transit',location='test')
    assert not result['supported']
    assert result['implementation_scope']=='adapter_constant_orientation_only_not_a_Pilz_limitation'
    assert result['flange_from_task_tcp']==a.tolist()
