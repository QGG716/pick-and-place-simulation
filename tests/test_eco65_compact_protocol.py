"""Public config and protocol tests; no private CAD or official mesh fixture."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
from unloading_sim.eco65.model import ROOT,canonical_fingerprint,transform,require_simulation
from unloading_sim.eco65.unloading_layout import build_scene,audit_layout,support_at
from unloading_sim.eco65.task_checks import audit_task,PHASES,EVENTS
from unloading_sim.eco65 import pipeline as pipe
from unloading_sim.eco65 import planning_contracts as P,execution_contracts as E

def config(name='eco65_desktop_unloading_compact_v1'):
    return yaml.safe_load((ROOT/f'configs/workcells/{name}.yaml').read_text(encoding='utf-8'))

def test_compact_literal_geometry_and_support():
    s=build_scene(config());assert len(s['boxes'])==4
    assert sorted(tuple(b['pose']) for b in s['boxes'])==pytest.approx(sorted([(0.11,-.115,.08),(.11,.115,.08),(.11,-.115,.24),(.11,.115,.24)]))
    assert s['base_position_m']==[-.36,.1,.18];assert s['floor_z_m']==0;assert s['roof_inner_z_m']==.65
    assert s['footprint']['minimum_static_size_m']==pytest.approx([1.4,.9])
    assert sum(not b['supports'] for b in s['boxes'])==2
    assert all(bool(b['supports'])==(b['layer']==0) for b in s['boxes'])
    assert not any(o['id'] in ('outlet_bridge','outlet_catch_belt','transfer_bridge','required_tabletop_envelope') for o in s['obstacles'])
    audit=audit_layout(s,None);assert audit['status']=='PASS';assert audit['transfer_gap_m']==0
    assert all(r['geometric_support_pass'] for r in audit['routes'].values())
    assert support_at(s,[-.78,-.22],[.22,.22,.16],.18)['coverage']==pytest.approx(1)

def test_old_nine_static_and_supported_gap():
    s=build_scene(config('eco65_desktop_unloading_layout_v1'));a=audit_layout(s,None)
    assert len(s['boxes'])==9 and a['status']=='PASS';assert a['transfer_gap_m']==pytest.approx(.01)
    assert all(r['geometric_support_pass'] for r in a['routes'].values())
    broken=copy.deepcopy(s);broken['support_surface_ids'].remove('transfer_bridge');assert 'route_support:A' in audit_layout(broken,None)['errors']

def test_desk_escape_and_overlap_rejected():
    c=config();c['conveyors']['longitudinal']['bounds_xy_m'][0]=-1.1
    assert any(e.startswith('desk_boundary') for e in audit_layout(build_scene(c),None)['errors'])
    c=config();c['conveyors']['longitudinal']['bounds_xy_m'][3]=-.08
    assert 'static_component_penetration' in audit_layout(build_scene(c),None)['errors']

def artifact():
    segments=[];q=np.zeros(6)
    for phase in PHASES:
        goal=q+0.01
        segments.append(dict(phase=phase,q=[q.tolist(),goal.tolist()],t=[0.,3.],duration_s=3.,target_tcp=np.eye(4).tolist()));q=goal
    a=dict(task_schema=2,segments=segments,box_id='box',attachment_tcp_to_box=np.eye(4).tolist(),released_box_pose=np.eye(4).tolist(),required_events=[dict(after_phase=p,event=e,box_id='box') for p,e in EVENTS])
    return rehash(a)

def rehash(a):
    a.pop('trajectory_fingerprint',None);a['trajectory_fingerprint']=canonical_fingerprint(a);return a

def audit(a,declared=None):
    if declared is None:declared=[np.zeros(6).tolist()]+[s['q'][-1] for s in a['segments']]
    return audit_task(a,declared,np.zeros(6),np.full(6,.06),'box',np.array([[-3,3]]*6),{'joint_velocity_rad_s':.55,'joint_acceleration_rad_s2':1.,'joint_jerk_rad_s3':4.},[3.14]*6)

def test_valid_task_and_legacy_six_stage():
    a=artifact();assert audit(a)['valid'];a.pop('task_schema');a.pop('required_events');assert audit(rehash(a))['valid']

@pytest.mark.parametrize('change', ['empty','missing','order','events','identity','q_shape','t_shape','nan','inf','discontinuity','duration','attachment'])
def test_malformed_task_rejected(change):
    a=artifact()
    if change=='empty':a['segments']=[]
    elif change=='missing':a['segments'].pop(2)
    elif change=='order':a['segments'][0],a['segments'][1]=a['segments'][1],a['segments'][0]
    elif change=='events':a['required_events'].pop()
    elif change=='identity':a['box_id']='other'
    elif change=='q_shape':a['segments'][1]['q']=[[0]*5]*2
    elif change=='t_shape':a['segments'][1]['t']=[0]
    elif change=='nan':a['segments'][1]['q'][0][0]=float('nan')
    elif change=='inf':a['segments'][1]['t'][1]=float('inf')
    elif change=='discontinuity':a['segments'][1]['q'][0][0]=.4
    elif change=='duration':a['segments'][1]['duration_s']=1.
    elif change=='attachment':a['attachment_tcp_to_box'][0][0]=2.
    if change not in ('nan','inf'):rehash(a)
    assert not audit(a)['valid']

def test_time_compression_rehashed_still_rejected():
    a=artifact()
    for s in a['segments']:s['t']=[0,.01];s['duration_s']=.01
    rehash(a);r=audit(a)
    assert any(e.startswith('independent_motion_limits') for e in r['errors'])

def test_declared_path_mismatch():
    a=artifact();assert 'declared_trajectory_mismatch' in audit(a,[[0.]*6,[.06]*6])['errors']

@pytest.mark.parametrize('exception,status,outcome',[(TimeoutError(),P.PlanStatus.TIMEOUT,None),(InterruptedError(),P.PlanStatus.NOT_EVALUATED,P.OperationalOutcome.CANCELLED),(pipe.IKSearchExhausted('contact',0,None),P.PlanStatus.NO_IK,None),(pipe.IKSearchExhausted('contact',1,{'pair':['a','b']}),P.PlanStatus.COLLISION,None),(RuntimeError('bounded RRT'),P.PlanStatus.NOT_EVALUATED,None)])
def test_real_planner_failure_returns(monkeypatch,exception,status,outcome):
    def fail(*args,**kwargs):raise exception
    monkeypatch.setattr(pipe,'find_ik',fail)
    scene=dict(seed=1,planning_budget_s=10,box=dict(id='box',pose=[0,0,.1],size_m=[.2,.2,.2]),margin_m=.002,joint_velocity_rad_s=.55,joint_acceleration_rad_s2=1.,joint_jerk_rad_s3=4.)
    planner=object.__new__(pipe.ECO65PlannerBackend);planner.world=SimpleNamespace(scene=scene,last_failure=None);planner.cancelled=False
    result=planner.plan(SimpleNamespace(seed=1,start_state=[0.]*6),P.PlanningCandidate('box','top'),P.PlanningPath.COLD)
    assert result.status==status and not result.success
    if outcome:assert result.operational_outcome==outcome

class StepWorld:
    scene={'enable_hardware':False};box_id='box';phase='initial'
    def box_pose(self,q,phase):return transform([float(q[0]),0,0])
    def valid(self,q,phase):self.q=np.array(q);self.phase=phase;return True

def running_executor():
    ex=pipe.GeometricExecutionBackend(StepWorld());ex.execution_id='test_execution';ex.plan=SimpleNamespace(plan_id='plan')
    ex.artifact=artifact();ex.elapsed=0.;ex.local_time=0.;ex.segment_index=0;ex.duration=18.;ex.next_frame=0.;ex._boundary=P.MotionBoundaryState.stopped([0.]*6);ex._state=E.ExecutionBackendState.RUNNING
    return ex

def test_stop_acceptance_completion_no_advance_and_unique_commands():
    ex=running_executor();assert ex.step(.02);q=ex.current_boundary().q;t=ex.elapsed
    assert any(abs(v)>0 for v in ex.current_boundary().qd)
    command=ex.request_stop('plan','test');assert command.accepted;assert ex.state==E.ExecutionBackendState.STOPPING
    assert ex.step(.02) is False;assert ex.elapsed==t and ex.current_boundary().q==q
    assert ex.state==E.ExecutionBackendState.IDLE;assert not ex.step(.02)
    assert ex.feedback[-1].status==E.ExecutionFeedbackStatus.STOPPED
    assert ex.request_stop('plan','again').command_id!=command.command_id
    assert not any(f.status==E.ExecutionFeedbackStatus.SUCCEEDED for f in ex.feedback)

def test_concurrent_start_and_exception_terminal():
    ex=running_executor();assert ex.start(SimpleNamespace(plan_id='other')).status==E.ExecutionCommandStatus.ALREADY_RUNNING
    def boom(*args):raise ValueError('test geometry exception')
    ex.w.valid=boom
    with pytest.raises(ValueError):ex.step(.02)
    assert ex.state==E.ExecutionBackendState.FAULTED;assert ex.feedback[-1].status==E.ExecutionFeedbackStatus.FAULTED

def test_hardware_and_continuous_handoff_disabled():
    with pytest.raises(PermissionError):require_simulation({'enable_hardware':True})
    with pytest.raises(PermissionError):pipe.GeometricExecutionBackend(StepWorld(),enable_hardware=True)
    assert not pipe.GeometricExecutionBackend(StepWorld()).capabilities.supports_continuous_handoff


@pytest.fixture
def protocol_envelope(monkeypatch):
    # Protocol doubles deliberately contain no physical geometry; these are not asset tests.
    from unloading_sim.eco65 import task_checks as checks
    class Snapshot:
        def __init__(self,revision):self.fingerprint=revision;self.robot_model_fingerprint='model1';self.world_model_fingerprint=revision
        def planning_context_matches(self,other):return self.fingerprint==other.fingerprint
    w=StepWorld();w.scene={'revision':'scene1','margin_m':.002,'mesh_error_reserve_m':.0002,'joint_velocity_rad_s':.55,'joint_acceleration_rad_s2':1.,'joint_jerk_rad_s3':4.}
    w.current_q=np.zeros(6);w.tool={'version':1};w.robot=SimpleNamespace(joint_limits=np.array([[-3.,3.]]*6));w.released_pose=None;w.model_revision='model1'
    monkeypatch.setattr(pipe,'known_world',lambda world,q:Snapshot(world.scene['revision']))
    monkeypatch.setattr(checks,'runtime_fingerprint',lambda world:world.model_revision)
    monkeypatch.setattr(checks,'current_kinematics_match_source',lambda world:True)
    import xml.etree.ElementTree as ET
    root=ET.fromstring('<robot>'+''.join('<joint><limit velocity="3.14"/></joint>' for _ in range(6))+'</robot>')
    monkeypatch.setattr(pipe.ET,'parse',lambda _:SimpleNamespace(getroot=lambda:root))
    a=artifact();a.update(runtime_fingerprint='model1',source_world='scene1',tool_fingerprint=canonical_fingerprint(w.tool));rehash(a)
    start=P.MotionBoundaryState.stopped(np.zeros(6));end=P.MotionBoundaryState.stopped(np.full(6,.06));snap=Snapshot('scene1')
    env=SimpleNamespace(executable=True,artifact_kind=P.PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY,plan_id='protocol',planned_snapshot=snap,expected_start_state=start.q,expected_end_state=end.q,
        expected_start_boundary=start,expected_end_boundary=end,candidate=SimpleNamespace(target_id='box'),
        result=SimpleNamespace(metadata=a,target_id='box',trajectory=[np.zeros(6).tolist()]+[s['q'][-1] for s in a['segments']]))
    return w,env

@pytest.mark.parametrize('change',['empty','missing','scene','model','tool','actual_start','declared'])
def test_validator_rejects_before_geometry(protocol_envelope,change):
    w,env=protocol_envelope;a=env.result.metadata
    if change=='empty':a['segments']=[];rehash(a)
    if change=='missing':a['segments'].pop();rehash(a)
    if change=='scene':w.scene['revision']='changed'
    if change=='model':w.model_revision='changed'
    if change=='tool':w.tool={'version':2}
    if change=='actual_start':w.current_q=np.ones(6)
    if change=='declared':env.result.trajectory=[[0.]*6,[.06]*6]
    result=pipe.ECO65PlanValidator(w).validate(env,env.planned_snapshot,env.expected_start_boundary)
    assert not result.valid

def test_execution_identity_changes_per_start(protocol_envelope):
    w,env=protocol_envelope;ex=pipe.GeometricExecutionBackend(w);first=ex.start(env)
    assert first.accepted;assert ex.request_stop(env.plan_id,'test').accepted;ex.step()
    second=ex.start(env);assert second.accepted and second.execution_id!=first.execution_id


def test_non_target_support_allowance_cannot_follow_selected_payload():
    from unloading_sim.eco65.task_world import UnloadingTaskWorld
    w=object.__new__(UnloadingTaskWorld);w.scene=build_scene(config());w.boxes={b['id']:b for b in w.scene['boxes']};w.object_info={o['id']:o for o in w.scene['obstacles']+w.scene['boxes']}
    right=next(b for b in w.scene['boxes'] if b['layer']==1 and b['column']==0);left=next(b for b in w.scene['boxes'] if b['layer']==1 and b['column']==1)
    w.box_id=right['id'];w.active_pose=transform([.11,.115,.08]);w.phase='loaded_transfer';w.receiving_region='conveyor_longitudinal'
    upper=dict(name=left['id'],kind='payload');moving=dict(name=right['id'],kind='payload')
    assert not w._support_pair(upper,moving)
    legitimate=dict(name=left['supported_by'][0],kind='payload');assert w._support_pair(upper,legitimate)


def test_validator_and_executor_reject_actual_initial_collision(protocol_envelope):
    w,env=protocol_envelope;w.last_failure={'pair':['robot','wall']};w.valid=lambda *args:False
    validator=pipe.ECO65PlanValidator(w);assert not validator.validate(env,env.planned_snapshot,env.expected_start_boundary).valid
    assert any('initial_clearance' in str(e) for e in validator.report['errors'])
    with pytest.raises(ValueError,match='Initial geometry'):pipe.GeometricExecutionBackend(w).start(env)


def test_restore_failure_cannot_leave_executor_running():
    ex=running_executor()
    def fault(*args):raise ValueError('geometry failure')
    def restore_fault(*args):raise RuntimeError('restore failed')
    ex.w.valid=fault;ex.w.set_state=restore_fault
    with pytest.raises(ValueError,match='geometry failure'):ex.step()
    assert ex.state==E.ExecutionBackendState.FAULTED and ex.restore_error=='restore failed'


@pytest.mark.parametrize('change',['none','other_box','other_execution','still_attached','already_outfed','wrong_pose','not_retreated'])
def test_outfeed_requires_same_actual_execution_and_released_carton(change):
    from unloading_sim.eco65.task_checks import outfeed_preconditions
    w=SimpleNamespace(box_id='box',execution_id='run1',attached=False,released=True,box_states={'box':'SUPPORTED_RELEASE'},release_support={'valid':True},current_q=np.zeros(6),released_pose=np.eye(4))
    replay=dict(status='GEOMETRIC_PICK_PLACE_COMPLETE',geometric_tasks_completed=1,execution_id='run1',events=[dict(event=e,box_id='box',q=[0.]*6,pose=np.eye(4).tolist()) for e in ['ATTACHED','SUPPORTED_RELEASE','RETREAT_COMPLETE']])
    if change=='other_box':replay['events'][0]['box_id']='other'
    if change=='other_execution':replay['execution_id']='run2'
    if change=='still_attached':w.attached=True
    if change=='already_outfed':w.box_states['box']='OUTFED_ASSUMED'
    if change=='wrong_pose':w.released_pose[0,3]=.1
    if change=='not_retreated':w.current_q[0]=.1
    assert outfeed_preconditions(w,replay)==(change=='none')
