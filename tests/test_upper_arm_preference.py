from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.upper_arm_preference import upper_arm, soft_cost, numerical_seed


@pytest.fixture
def robot():
    return load_workcell_layout(Path(__file__).resolve().parents[1]/
        "configs/workcells/m710id70_unloading_layout_v1_transverse_scissor_026.yaml").robot()


def test_official_structural_origins_not_q2_or_mesh_center(robot):
    q=np.zeros(6)
    a=upper_arm(robot,q)
    assert a["length_m"] == pytest.approx(.895)
    assert a["elevation_deg"] == pytest.approx(90.)
    assert a["q2_rad"] == 0.
    q[1]=np.pi/3
    a=upper_arm(robot,q)
    assert a["elevation_deg"] == pytest.approx(30.)
    assert a["q2_rad"] == pytest.approx(np.pi/3)
    frames=robot.joint_axis_frames(q)
    np.testing.assert_allclose(a["shoulder_world_m"],frames["J2"][0])
    np.testing.assert_allclose(a["elbow_world_m"],frames["J3"][0])
    # Wrist turns are not a new upper-arm configuration.
    q[5]=2*np.pi
    assert upper_arm(robot,q)["elevation_deg"] == pytest.approx(a["elevation_deg"])


def test_soft_bounded_cost_and_general_seeds_preserve_start_limits(robot):
    start=np.zeros(6); before=start.copy();limits=robot.joint_limits.copy()
    values=[numerical_seed(robot,start,71070,i) for i in range(30)]
    assert all(robot.within_limits(q) for q in values)
    assert all(0 <= soft_cost(robot,q,start)["cost"] <= 1 for q in values)
    assert min(upper_arm(robot,q)["elevation_deg"] for q in values) < 0.
    np.testing.assert_array_equal(values[0],start)
    np.testing.assert_array_equal(start,before)
    np.testing.assert_array_equal(robot.joint_limits,limits)
    for i,q in enumerate(values):
        np.testing.assert_array_equal(q,numerical_seed(robot,start,71070,i))


def test_low_valid_result_returns_without_post_success_optimization(monkeypatch,robot):
    from test_single_target_recovery import fixture
    from unloading_sim.single_target_recovery import connect_fixed_target
    c,clock,_,iks,motions=fixture(monkeypatch)
    c.robot=robot;c.upper_arm_preference_enabled=True;c.empty_approach_direct_first=False
    low=np.zeros(6);low[1]=2.
    c.ik_pose_once=lambda *a,**k:(low.copy(),None)
    path,q,f,cost=connect_fixed_target(c,robot.fk(low),np.zeros(6),[],71070,
        stage="pregrasp",purpose="FREE_APPROACH",residual_reason="FK")
    assert path and f is None and len(motions)==1
    assert upper_arm(robot,q)["elevation_deg"] < 0.
    np.testing.assert_array_equal(q,low)


def test_j2_edit_alone_does_not_preserve_target(robot):
    q=np.zeros(6);pose=robot.fk(q)
    changed=q.copy();changed[1]=.2
    assert np.linalg.norm(robot.fk(changed)[:3,3]-pose[:3,3]) > .01


def test_failed_optional_preference_keeps_qualified_low_goal(monkeypatch,robot):
    from test_single_target_recovery import fixture
    from unloading_sim.single_target_recovery import connect_fixed_target
    c,clock,_,iks,motions=fixture(monkeypatch)
    c.robot=robot;c.upper_arm_preference_enabled=True;c.empty_approach_direct_first=True
    low=np.zeros(6);low[1]=2.;calls=[]
    def solve(*a,**k):
        calls.append(1);clock[0]+=.01
        return (low.copy(),None) if len(calls)==1 else (None,dict(reason="NATIVE_IK_FAILED"))
    c.ik_pose_once=solve
    path,q,f,_=connect_fixed_target(c,robot.fk(low),np.zeros(6),[],71070,
        stage="pregrasp",purpose="FREE_APPROACH",residual_reason="FK")
    assert path and f is None and len(motions)==1 and len(calls)==2
    np.testing.assert_array_equal(q,low)
