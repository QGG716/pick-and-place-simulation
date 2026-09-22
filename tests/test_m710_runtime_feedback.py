"""Actual feedback gates, including the real adapter loop and action barriers."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.m710_replay_physics import RuntimeFeedbackMonitor, finite_gravity_compensated_drive_target
from unloading_sim.qualification import ReplayQualificationPolicy


def monitor():
    return RuntimeFeedbackMonitor(['J1','J2'],[-1,-2],[1,2],[.5,1],ReplayQualificationPolicy())


def observe(m, **updates):
    args=dict(q=[0,0],qd=[0,0],reference=[0,0],reference_velocity=[0,0],attached=False)
    args.update(updates)
    return m.observe(**args)


def test_normal_boundary_and_feedforward_not_a_tracking_reference():
    m=monitor()
    assert observe(m,q=[.05,0])
    assert observe(m,q=[1+5e-10,0],reference=[1,0],qd=[.5+5e-6,0],
                   context={'drive_position_target_rad':[1.3,0],'trajectory_time_s':2.})
    assert m.first_failure is None


@pytest.mark.parametrize('updates,reason',[
    ({'q':[1.00001,0],'reference':[1,0]},'ACTUAL_JOINT_POSITION_LIMIT'),
    ({'qd':[.50002,0]},'ACTUAL_JOINT_VELOCITY_LIMIT'),
    ({'q':[.05001,0]},'ACTUAL_JOINT_TRACKING_ERROR'),
])
def test_hard_feedback_failure_without_debounce(updates,reason):
    m=monitor();assert observe(m)
    assert not observe(m,**updates)
    assert m.first_failure['reason']==reason and m.observations==2


@pytest.mark.parametrize('field',['q','qd','reference','reference_velocity'])
@pytest.mark.parametrize('value',[None,[],[0],[0,0,0],[np.nan,0],[0,np.inf],[[0,0]]])
def test_required_joint_state_is_fail_closed(field,value):
    m=monitor();assert not observe(m,**{field:value})
    assert m.first_failure['reason']=='INVALID_ACTUAL_FEEDBACK'


def attached(**updates):
    args=dict(attached=True,body_pose=[0,0,0,1,0,0,0],carton_pose=[.3,0,0,1,0,0,0],
              captured_relative_pose=[.3,0,0,1,0,0,0])
    args.update(updates);return args


@pytest.mark.parametrize('pose,reason',[
    ([.321,0,0,1,0,0,0],'ACTUAL_ATTACHMENT_TRANSLATION'),
    ([.3,0,0,np.cos(.051),0,0,np.sin(.051)],'ACTUAL_ATTACHMENT_ROTATION'),
    (None,'INVALID_ACTUAL_FEEDBACK'),([0]*7,'INVALID_ACTUAL_FEEDBACK'),
    ([.3,0,0,1,np.nan,0,0],'INVALID_ACTUAL_FEEDBACK'),([0]*6,'INVALID_ACTUAL_FEEDBACK')])
def test_attachment_uses_capture_and_checks_first_observed_step(pose,reason):
    m=monitor()
    assert not observe(m,**attached(carton_pose=pose))
    assert m.first_failure['reason']==reason


def test_attachment_lifecycle_and_rotated_tool():
    m=monitor();assert observe(m,carton_pose=None)
    assert observe(m,**attached())
    # 90-degree rotation about Z: same captured relative transform.
    assert observe(m,**attached(body_pose=[1,2,3,2**-.5,0,0,2**-.5],
                                carton_pose=[1,2.3,3,2**-.5,0,0,2**-.5]))
    assert observe(m,attached=False,legally_released=True,carton_pose=[99]*7)
    assert observe(m,attached=False,legally_released=True,carton_pose=None)
    n=monitor();assert observe(n,**attached())
    assert not observe(n,attached=False)
    assert n.first_failure['reason']=='ACTUAL_ATTACHMENT_LOST'
    n=monitor();assert not observe(n,attachment_expected=True)


def test_first_failure_is_stable_atomic_evidence_and_next_task_is_clean(tmp_path):
    m=monitor();assert observe(m)
    assert not observe(m,q=[2,0],qd=[2,0],context={'simulation_time_s':.1})
    assert [v['reason'] for v in m.first_failure['violations']]==[
        'ACTUAL_JOINT_POSITION_LIMIT','ACTUAL_JOINT_VELOCITY_LIMIT','ACTUAL_JOINT_TRACKING_ERROR']
    first=copy.deepcopy(m.first_failure)
    m.latch([{'reason':'LATER_CONTACT'}],{});assert not observe(m,q=[np.nan,0])
    path=tmp_path/'failure.json';m.persist(path)
    assert json.loads(path.read_text())['first_failure']==first
    assert m.last_valid['q_rad']==[0,0]
    n=monitor();assert n.first_failure is None and n.attachment_baseline is None and observe(n)


SOURCE=Path(__file__).resolve().parents[1]/'scripts/isaacsim_fanuc_replay.py'


def production_namespace(tmp_path, q, qd):
    """Fake only device IO; execute unchanged production checkpoint/stop code."""
    tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
    names={'_runtime_feedback_checkpoint','_persist_feedback_stop'}
    funcs=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name in names]
    calls=[]
    def tensor(value):return SimpleNamespace(numpy=lambda:np.asarray([value],float))
    art=SimpleNamespace(get_dof_positions=lambda:tensor(q),get_dof_velocities=lambda:tensor(qd),
        set_dof_position_targets=lambda value:calls.append('hold_position'),
        set_dof_velocity_targets=lambda value:calls.append('hold_velocity'))
    joint=SimpleNamespace(IsValid=lambda:True,GetAttribute=lambda name:SimpleNamespace(Get=lambda:True))
    ns=dict(np=np,feedback_monitor=monitor(),initial=np.zeros(2),articulation=art,
        ideal_independent_mode=True,args=SimpleNamespace(output=tmp_path,gripper_model='surface_gripper'),
        stage=SimpleNamespace(GetPrimAtPath=lambda path:joint,RemovePrim=lambda path:calls.append('release')),
        grasp_joint_path='/joint',grasp_local_position=[.3,0,0],grasp_local_quaternion=[1,0,0,0],
        grasp_body=SimpleNamespace(get_world_poses=lambda:(tensor([0,0,0]),tensor([1,0,0,0]))),
        target_body=SimpleNamespace(get_world_poses=lambda:(tensor([.3,0,0]),tensor([1,0,0,0]))),
        last_drive_feedforward={'robot_gravity_nm':np.zeros(2),'payload_gravity_nm':np.zeros(2)},
        stiffness=np.ones(2)*1000,effort_limits=np.ones(2)*100,
        finite_gravity_compensated_drive_target=finite_gravity_compensated_drive_target,
        feedback_source_identity={'adapter':'test-extracted-production'},run_started_unix_s=123,
        metadata={'target':'carton'},contact_runtime_context={'stage':'place'},
        world=SimpleNamespace(current_time=5.,step=lambda **kwargs:calls.append('physics')),
        last_issued_command={'trajectory_time_s':4.,'drive_position_target_rad':[.2,0]},
        unexpected_robot_contact_events=[],runtime_stop_reason=None,
        grasp_enabled=True,release_commanded=False,release_open_confirmed=False,
        feedback_reference=np.zeros(2),feedback_reference_velocity=np.zeros(2),feedback_reference_time=4.,
        physics_dt=1/240,physics_steps=2,trajectory_time=4.,release_event_time=4.)
    exec(compile(ast.Module(body=funcs,type_ignores=[]),str(SOURCE),'exec'),ns)
    loop=next(n for n in ast.walk(tree) if isinstance(n,ast.For) and isinstance(n.iter,ast.Call)
        and ast.unparse(n.iter)=='range(physics_steps)')
    return ns,calls,loop


@pytest.mark.parametrize('q,qd',[([.06,0],[0,0]),([0,0],[.6,0]),([np.nan,0],[0,0])])
def test_real_loop_blocks_due_release_and_next_trajectory_before_actions(tmp_path,q,qd):
    ns,calls,loop=production_namespace(tmp_path,q,qd)
    # Entire production loop: any accidental action before the guard either
    # reaches the IO spy or fails loudly on a missing dependency.
    exec(compile(ast.Module(body=[loop],type_ignores=[]),str(SOURCE),'exec'),ns)
    assert calls==['hold_position','hold_velocity']
    assert ns['trajectory_time']==4. and not ns['release_commanded'] and not ns['release_open_confirmed']
    assert ns['runtime_stop_reason']
    assert (tmp_path/'runtime_feedback.json').is_file()


def test_real_post_physics_barrier_blocks_handoff_and_preserves_first_failure(tmp_path):
    ns,calls,loop=production_namespace(tmp_path,[0,0],[.6,0])
    # Run the real body from its physics dispatch: that completed step cannot be
    # undone; all later production actions must be skipped at the next statement.
    start=next(i for i,n in enumerate(loop.body) if ast.unparse(n).startswith('world.step('))
    loop.body=loop.body[start:];ns['step']=0
    exec(compile(ast.Module(body=[loop],type_ignores=[]),str(SOURCE),'exec'),ns)
    assert calls==['physics','hold_position','hold_velocity']
    assert ns['trajectory_time']==4. and not ns['release_open_confirmed']
    failure=ns['feedback_monitor'].first_failure
    assert failure['observation']['physical_step']==1
    assert failure['observation']['observation_point']=='after_physics'
    assert failure['observation']['trajectory_time_s']==4.
    ns['feedback_monitor'].latch([{'reason':'CLEANUP_ERROR'}],{})
    ns['_persist_feedback_stop']()
    assert json.loads((tmp_path/'runtime_feedback.json').read_text())['first_failure']==failure


def test_production_checkpoint_ignores_drive_bias_and_preserves_hold_clock(tmp_path):
    ns,calls,_=production_namespace(tmp_path,[0,0],[0,0])
    assert ns['_runtime_feedback_checkpoint']('before_actions',1,5.,[0,0],[0,0],4.)
    assert not calls
    assert ns['feedback_monitor'].last_valid['last_issued_command']['drive_position_target_rad']==[.2,0]


def test_hold_failure_cannot_erase_first_failure_or_release(tmp_path):
    ns,calls,_=production_namespace(tmp_path,[.06,0],[0,0])
    def broken_hold(value):
        # The original failure must already be durable before drive cleanup.
        assert json.loads((tmp_path/'runtime_feedback.json').read_text())['first_failure']
        raise RuntimeError('device unavailable during stop')
    ns['articulation'].set_dof_position_targets=broken_hold
    assert not ns['_runtime_feedback_checkpoint']('before_actions',1,5.,[0,0],[0,0],4.)
    evidence=json.loads((tmp_path/'runtime_feedback.json').read_text())
    assert evidence['first_failure']['reason']=='ACTUAL_JOINT_TRACKING_ERROR'
    assert evidence['stop_handling']['status']=='HOLD_COMMAND_FAILED_NO_FURTHER_PHYSICS'
    assert not calls and not ns['release_commanded']


def test_failed_production_loop_cannot_count_workflow_success(tmp_path):
    ns,calls,loop=production_namespace(tmp_path,[.06,0],[0,0])
    exec(compile(ast.Module(body=[loop],type_ignores=[]),str(SOURCE),'exec'),ns)
    tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
    completion=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='workflow_cycle_completed' for t in n.targets))
    # Even if every earlier completion condition was true, the actual stop gate
    # in the production assignment must short-circuit before acceptance.
    ns.update(full_schedule_replayed=True,release_open_confirmed=True,
              target_cup_release_gate=SimpleNamespace(pending=False),
              payload_motion_verified=True,unexpected_contacts=[])
    exec(compile(ast.Module(body=[completion],type_ignores=[]),str(SOURCE),'exec'),ns)
    assert ns['workflow_cycle_completed'] is False


def test_production_task_entry_constructs_fresh_monitor(tmp_path):
    ns,_,_=production_namespace(tmp_path,[.06,0],[0,0])
    assert not observe(ns['feedback_monitor'],q=[.06,0])
    tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
    task_loop=next(n for n in ast.walk(tree) if isinstance(n,ast.While)
        and any(isinstance(s,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='actual_frame_states'
            for t in s.targets) for s in n.body))
    reset=next(n for n in task_loop.body if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='feedback_monitor' for t in n.targets))
    ns['_new_feedback_monitor']=monitor
    exec(compile(ast.Module(body=[reset],type_ignores=[]),str(SOURCE),'exec'),ns)
    assert ns['feedback_monitor'].first_failure is None
    assert ns['feedback_monitor'].attachment_baseline is None
