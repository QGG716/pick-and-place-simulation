"""Protocol/control-flow tests. Real FCL evidence is produced by the directed tool."""
from copy import deepcopy
import importlib.util
import math
from types import SimpleNamespace
import sys
import numpy as np
import pytest
from unloading_sim.eco65.evaluation import CHAIN, STRICT, INTERPOLATION, evaluation_profile, validate_output_evaluation, validate_export_reference
from unloading_sim.eco65.model import ROOT, transform
from unloading_sim.eco65 import native_task, native_replay
from unloading_sim.moveit2_backend import digest
from test_eco65_native import synthetic_plan_fixture


def basic_result(stage):
    req,raw=stage['native_request'],stage['native_result']
    q=np.asarray(stage['q']);resolution=req['clearance_policy']['edge_resolution_rad']
    count=1+sum(max(1,math.ceil(float(np.max(np.abs(b-a)))/resolution)) for a,b in zip(q[:-1],q[1:]))
    req['evaluation_profile']=CHAIN
    raw.update(evaluation_profile=CHAIN,native_output_status='BASIC_SAMPLED_PASS',
        task_tcp_identity='synthetic_tcp',interpolated_link='synthetic_tcp_link',
        start_state_status='PASS',joint_goal_state_status='NOT_APPLICABLE_POSE_GOAL',
        dense_output_review='NOT_RUN_PROFILE',planner_internal_process_checks=True,
        clearance={'dense_path_calls':0},process_policy={'dense_path_calls':0,'planner_state_checks':0},
        endpoint_fk={'position_error_m':0.,'orientation_error_rad':0.},
        basic_path_check=dict(status='BASIC_SAMPLED_PASS',completed=True,failure=None,raw_nodes=len(q),
            sampling_rule='MAX_ABS_JOINT_DELTA_CEIL_V1',geometric_interpolation=INTERPOLATION,
            resolution_rad=resolution,checked_states=count,clearance_checks=count,process_checks=count,
            ordinary_collision_checks=count,endpoint_process_checks=1,actual_endpoint_status='PASS',wall_s=.01))
    stage.update(evaluation_profile=CHAIN,python_linear_audit='NOT_RUN_PROFILE')


def chain_fixture(monkeypatch):
    snapshot,plan=synthetic_plan_fixture()
    plan['identity'].update(task_tcp_fingerprint='synthetic_tcp',task_tcp_link='synthetic_tcp_link')
    plan.update(evaluation_profile=CHAIN,replay_interpolation=INTERPOLATION,independent_review='NOT_RUN_PROFILE')
    for stage in plan['stages']:basic_result(stage)
    def frames(q):
        return {'link_6':transform(snapshot['scene']['box']['pose']) if q==plan['events'][0]['q'] else np.asarray(plan['events'][1]['pose'])}
    robot=SimpleNamespace(named_link_frames=frames)
    monkeypatch.setattr(native_task,'robot_for',lambda *a:robot)
    monkeypatch.setattr(native_replay,'robot_for',lambda *a:robot)
    monkeypatch.setattr(native_replay,'verify_snapshot',lambda s:True)
    resign(plan)
    return snapshot,plan


def resign(plan):
    plan.pop('fingerprint',None);plan['fingerprint']=digest(plan)


def test_chain_plan_and_replay_without_review(monkeypatch):
    snapshot,plan=chain_fixture(monkeypatch)
    assert native_task.validate_plan_contract(plan,snapshot)
    assert native_replay.replay_inputs(snapshot,plan)


@pytest.mark.parametrize('case',['profile','stage','request','response','dense_status','missing_samples',
    'count','endpoint','fk','incomplete','dense_called','planner_count','interpolation','path','event','tcp'])
def test_chain_evidence_rejects_incomplete_mixed_or_changed_inputs(monkeypatch,case):
    snapshot,plan=chain_fixture(monkeypatch);s=plan['stages'][0];r=s['native_result'];e=r['basic_path_check']
    if case=='profile':plan['evaluation_profile']='NO_COLLISION'
    if case=='stage':s['evaluation_profile']=STRICT
    if case=='request':s['native_request']['evaluation_profile']=STRICT
    if case=='response':r.pop('evaluation_profile')
    if case=='dense_status':r['native_output_status']='PASS'
    if case=='missing_samples':r.pop('basic_path_check')
    if case=='count':e['checked_states']-=1
    if case=='endpoint':e['endpoint_process_checks']=0
    if case=='fk':r['endpoint_fk']['position_error_m']=.01
    if case=='incomplete':e['completed']=False
    if case=='dense_called':r['process_policy']['dense_path_calls']=1
    if case=='planner_count':r['process_policy'].pop('planner_state_checks')
    if case=='interpolation':plan['replay_interpolation']='CUBIC_SPLINE'
    if case=='path':s['q'][1][0]+=.001
    if case=='event':plan['events'][0]['flange_from_target'][0][3]+=.01
    if case=='tcp':r['task_tcp_identity']='wrong_tcp'
    resign(plan)
    with pytest.raises(ValueError):native_task.validate_plan_contract(plan,snapshot)


def test_default_strict_keeps_dense_output_and_review_gate(monkeypatch):
    snapshot,plan=synthetic_plan_fixture()
    assert native_task.validate_plan_contract(plan,snapshot)
    monkeypatch.setattr(native_replay,'verify_snapshot',lambda s:True)
    with pytest.raises(ValueError,match='REQUIRES_SAME_REVIEWED_PLAN'):native_replay.replay_inputs(snapshot,plan)
    plan['stages'][0]['native_result']['native_output_status']='BASIC_SAMPLED_PASS';resign(plan)
    with pytest.raises(ValueError,match='STRICT_NATIVE_OUTPUT_REQUIRED'):native_task.validate_plan_contract(plan,snapshot)
    with pytest.raises(ValueError,match='UNKNOWN_EVALUATION_PROFILE'):evaluation_profile('anything')


def test_chain_export_and_replay_must_reference_same_plan():
    plan=dict(evaluation_profile=CHAIN,fingerprint='new_plan')
    validate_export_reference(plan,{'home_source':dict(plan)})
    for source in ({}, dict(plan,fingerprint='another_plan'), dict(plan,evaluation_profile=STRICT)):
        with pytest.raises(ValueError,match='WORKSTATION_PLAN_OR_EVALUATION_PROFILE_MISMATCH'):
            validate_export_reference(plan,{'home_source':source})


@pytest.mark.parametrize('profile',[STRICT,CHAIN])
def test_cli_only_strict_invokes_independent_review(tmp_path,monkeypatch,profile):
    spec=importlib.util.spec_from_file_location('chain_entry_test',ROOT/'tools/run_eco65_moveit2_single_carton.py')
    entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
    review_calls=[];profiles=[]
    plan=dict(evaluation_profile=profile,stages=[],native_initialization_s=.1,planning_wall_s=.2,simulated_duration_s=1.)
    class Task:
        def __init__(self,*a,**kw):profiles.append(kw['evaluation_profile'])
        def run(self):return plan
        def close(self):pass
    def review(*a,**kw):
        review_calls.append(True)
        if profile==CHAIN:raise AssertionError('Independent review must not run')
        return {'status':'PASS','wall_s':.3}
    monkeypatch.setattr(entry,'ROOT',tmp_path);monkeypatch.setattr(entry,'NativeTask',Task)
    monkeypatch.setattr(entry,'review_plan',review);monkeypatch.setattr(entry,'load_json',lambda p:{})
    monkeypatch.setattr(entry,'verify_snapshot',lambda s:s)
    out=tmp_path/'outputs'/'plan'
    args=['entry','plan','--output',str(out),'--snapshot','snapshot.json','--worker-command','unused','--attempts','1']
    if profile==CHAIN:args+=['--evaluation-profile',CHAIN]
    monkeypatch.setattr(sys,'argv',args)
    assert entry.main()==0 and profiles==[profile]
    assert len(review_calls)==(profile==STRICT)
    assert (out/'plan.json').exists() and (out/'review.json').exists()==(profile==STRICT)


@pytest.mark.parametrize('profile',[STRICT,CHAIN])
def test_motion_only_strict_calls_linear_audit(monkeypatch,profile):
    snapshot,plan=synthetic_plan_fixture();stage=plan['stages'][1]
    if profile==CHAIN:basic_result(stage)
    task=object.__new__(native_task.NativeTask)
    task.evaluation_profile=profile;task.init=snapshot['native_model']['init'];task.model=snapshot['native_model']
    task.counter=0;task.records=[];task.stages=[];task.parent='pregrasp';task.task_id=plan['task_id'];task.attachment=None
    task.target_pose=np.eye(4);task.robot=SimpleNamespace(fk=lambda q:np.eye(4))
    task.request=lambda *a,**k:deepcopy(stage['native_request'])
    def call(req):
        result=deepcopy(stage['native_result']);result['stage_id']=req['stage_id'];return result
    task.call=call;calls=[]
    def audit(*a,**k):
        calls.append(True)
        if profile==CHAIN:raise AssertionError('Dense LIN audit must not run')
        return {'passed':True}
    monkeypatch.setattr(native_task,'audit_linear_tcp',audit)
    task.motion(np.asarray(stage['q'][0]),'contact',pose=np.eye(4))
    assert len(task.stages)==1 and len(calls)==(profile==STRICT)
