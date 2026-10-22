from contextlib import contextmanager
import numpy as np
import pytest
from unloading_sim.eco65.coupled_task import CoupledTask


def test_relaxed_rigid_tool_trial_keeps_finite_real_lip_approach_standoffs():
    task=CoupledTask.__new__(CoupledTask);task.face='front'
    task.scene={'rigid_tool_carton_contact_mode':'enumerated_rigid_tool_original_cartons_v1'}
    task.target={'size_m':[.15,.1,.1]}
    assert task.approach_offsets()==[(.15,-.1),(.15,.1),(.15,0.),(.025,0.),(.01,0.),(.005,0.)]
    task.face='top';assert task.approach_offsets()==[(.025,0.),(.01,0.),(.005,0.)]
    task.face='front';task.scene={};assert task.approach_offsets()==[(.025,0.),(.01,0.),(.005,0.)]


@pytest.mark.parametrize('endpoint_allowed',[True,False])
def test_wrist_recovery_uses_current_state_and_keeps_same_connection_deadline(endpoint_allowed):
    task=CoupledTask.__new__(CoupledTask)
    task.current_q=np.array([-.8,0.,0.,0.,0.,0.]);initial=task.current_q.copy()
    contact_q=np.array([.2,.1,-1.,-.6,-1.5,-3.])
    task.target_pose=np.eye(4);task.face='front';task.target={'id':'target'}
    task.candidate_id='native-contact';task.cycle_failures=[];task.cycle_records=[]
    task.business_interval='FIRST_APPROACH';task.attachment=None
    task.select_grasp=lambda g:None;task.ik=lambda *a:iter([contact_q])
    task.direction_goal_valid=lambda *a:True;task.goal_cost=lambda *a:0
    task.checkpoint=lambda:task.current_q.copy()
    task.restore=lambda q:setattr(task,'current_q',q.copy())
    task.seal=lambda:{'status':'PASS'}
    @contextmanager
    def connection():
        task.connection_deadline=80.
        yield
        task.connection_deadline=None
    task.connection=connection
    task.request=lambda q,s:dict(q_start=q.copy(),stage=s)
    checks=[];motions=[]
    def call(req):
        assert task.connection_deadline==80.
        checks.append(req['q_start'])
        return dict(status='SUCCESS' if endpoint_allowed else 'REJECTED')
    task.call=call
    def move(stage,**kw):
        assert task.connection_deadline==80.
        motions.append((stage,task.current_q.copy(),kw))
        if 'goal' in kw:
            assert endpoint_allowed
            task.current_q=kw['goal'].copy();task.cycle_records.append({})
        elif stage=='pregrasp' and np.allclose(task.current_q,initial):
            raise ValueError('NATIVE_DIRECT_CONNECTION_FAILED')
        else:task.current_q=contact_q.copy()
    task.move=move
    if endpoint_allowed:
        task.approach(dict(tcp_pose_D=np.eye(4)))
        assert task.cycle_records[0]['motion_purpose']=='WRIST_PREPARATION_BEFORE_APPROACH'
        assert motions[2][1]==pytest.approx([-.8,0.,0.,-.6,-1.5,-3.])
        assert motions[-1][0]=='contact'
    else:
        with pytest.raises(ValueError,match='APPROACH_CONTACT_EXHAUSTED'):
            task.approach(dict(tcp_pose_D=np.eye(4)))
        assert task.current_q==pytest.approx(initial)
        assert not task.cycle_records
    assert len(checks)==1
    assert checks[0]==pytest.approx([-.8,0.,0.,-.6,-1.5,-3.])


@pytest.mark.parametrize('mode',['inherited_state','new_simulation'])
def test_grasp_recovery_reuses_only_identical_inherited_native_root(mode):
    task=CoupledTask.__new__(CoupledTask)
    task.initial_mode=mode;task.max_grasps=9;task.task_id='cold-root'
    task.progress_path=None;task.cycle_failures=[]
    task.geometry_candidates=lambda:dict(candidates=[dict(candidate_id=str(i),tcp_pose_D=np.eye(4)) for i in range(9)])
    task.select_grasp=lambda g:None;task.select_home=lambda pose:(np.zeros(6),{})
    task.checkpoint=lambda:task.current_q.copy()
    task.restore=lambda q:setattr(task,'current_q',q.copy())
    task.initial_candidates=lambda g,q:iter([(q.copy(),dict(mode=mode))])
    task.state=lambda:dict(q=task.current_q.tolist())
    seen=[]
    def fail(grasp):
        seen.append(task.task_id)
        raise ValueError('CONNECTION_NOT_FOUND')
    task.approach=fail
    with pytest.raises(ValueError,match='BOUNDED_CANDIDATES_EXHAUSTED'):
        task.run()
    assert len(seen)==9
    assert len(set(seen))==(1 if mode=='inherited_state' else 9)
    if mode=='inherited_state':assert set(seen)=={'cold-root'}
