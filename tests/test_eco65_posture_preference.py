"""Directed FK posture/branch ordering checks; no native feasibility claim."""
import numpy as np
import pytest
from unloading_sim.eco65 import posture_preference as preference
from unloading_sim.eco65.model import URDF
from unloading_sim.eco65.native_task import NativeTask
from unloading_sim.robot import URDFRobot

STACK=np.array([.205,0.,.38])
# Current-run regression evidence: rejected transfer candidate and successful
# L03_C02 contact. These q's are tests only, never production planning seeds.
BAD=np.array([.24709302092498475,-1.2930418662601777,.9920654144328909,1.871765429860449,-1.570792655257472,-6.033973472075785])
CONTACT=np.array([.06046834454753859,-.16456964419936337,-.7626450009788632,-.6435816816303063,1.5707926536027614,1.633390832247429])


@pytest.fixture
def robot():
    if not URDF.exists():pytest.skip('Official ECO65 URDF assets not installed')
    robot=URDFRobot.from_urdf(URDF,active_joint_names=[f'joint_{i}' for i in range(1,7)],
        base_link='base_link',tip_link='link_6',base_position=[-.34,0.,.24],base_rpy=[0.,0.,np.pi],tool_length=0.)
    # Actual frozen suction-tool TCP, not a flange-origin surrogate.
    robot.tip_from_tcp=np.array([[.9999977397207271,.002126159315938315,0.,0.],
        [-.002126159315938315,.9999977397207271,0.,0.],
        [0.,0.,1.,.14275043380451888],[0.,0.,0.,1.]])
    return robot


def test_official_j2_zero_is_upright_and_recorded_foldback_is_detected(robot):
    zero=preference.posture_metrics(robot,np.zeros(6),STACK)
    assert zero['upper_arm_tilt_deg']==pytest.approx(0.,abs=.001)
    bad=preference.posture_metrics(robot,BAD,STACK)
    good=preference.posture_metrics(robot,CONTACT,STACK)
    assert bad['upper_arm_tilt_deg']==pytest.approx(74.086,abs=.01)
    assert good['upper_arm_tilt_deg']==pytest.approx(9.429,abs=.01)
    assert bad['wrist_forward_of_elbow_m']<-.04
    assert good['wrist_forward_of_elbow_m']>.29
    assert not bad['preferred'] and good['preferred']
    # Even when the folded endpoint is nearer in joint space, prefer the open
    # band. No angle rewriting or goal deletion occurs.
    candidates=[BAD.copy(),CONTACT.copy()]
    candidates.sort(key=lambda q:preference.goal_cost(robot,BAD,q,STACK))
    assert np.array_equal(candidates[0],CONTACT) and len(candidates)==2


def test_preference_is_not_a_vertical_lock_and_keeps_raw_wrist_continuity(robot):
    tilt=CONTACT.copy();tilt[1]=.8
    metric=preference.posture_metrics(robot,tilt,STACK)
    assert metric['upper_arm_tilt_deg']==pytest.approx(45.837,abs=.01)
    assert metric['preferred']
    other=CONTACT.copy();other[5]-=2*np.pi
    assert preference.goal_cost(robot,CONTACT,CONTACT,STACK)<preference.goal_cost(robot,CONTACT,other,STACK)
    assert other[5]==CONTACT[5]-2*np.pi  # No modulo in ranking.


def test_seed_budget_finite_deterministic_and_does_not_move_actual_q(robot):
    q=CONTACT.copy();first=preference.ik_seeds(robot,q,8,6518,STACK)
    second=preference.ik_seeds(robot,q,8,6518,STACK)
    assert len(first)==8 and np.array_equal(first,second)
    assert np.array_equal(first[0],q) and np.array_equal(q,CONTACT)
    assert np.all(np.array(first)>=robot.joint_limits[:,0]) and np.all(np.array(first)<=robot.joint_limits[:,1])
    assert len(preference.ik_seeds(robot,q,1,6518,STACK))==1
    transit=preference.transit_seeds(robot,q,STACK)
    assert len(transit)==4 and all(s[0]==q[0] for s in transit)
    assert all(preference.posture_metrics(robot,s,STACK)['upper_arm_tilt_deg']<30 for s in transit)


def test_seed_priority_still_requires_native_ik_and_state_acceptance(monkeypatch,robot):
    task=NativeTask.__new__(NativeTask);task.counter=0;task.identity={};task.ik_records=[];task.robot=robot
    task.ik_seed_candidates=lambda seed,count:preference.ik_seeds(robot,seed,count,6518,STACK)
    seen=[];returned=iter([BAD,CONTACT])
    def call(req):
        if req['op']=='ik':return dict(status='SUCCESS',native_ik_calls=1,q=next(returned).tolist())
        seen.append(req['q']);return dict(status='STATE_REJECTED' if len(seen)==1 else 'SUCCESS')
    task.call=call;task.request=lambda q,stage:dict(op='check_state_offline',q=np.asarray(q).tolist())
    result=list(task.ik(np.eye(4),CONTACT,'transit',count=2))
    assert len(seen)==2 and len(result)==1 and np.array_equal(result[0],CONTACT)
