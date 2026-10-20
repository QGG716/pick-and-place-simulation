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


def test_production_connector_backend_matches_official_structural_fk(robot):
    from unloading_sim.layout_single_carton import (load_layout_motion_policy,
        build_verified_motion_input, _build_automatic_trajectory_connector)
    root=Path(__file__).resolve().parents[1]
    policy=load_layout_motion_policy(root/"configs/validation/m710id70_proof_of_concept.yaml")
    scene=build_verified_motion_input(policy,root)
    built=_build_automatic_trajectory_connector(scene,robot)
    assert built.connector is not None
    actual=built.connector.robot
    assert type(actual).__name__=="PinocchioHppFclBackend"
    limits=actual.joint_limits.copy()
    for index in range(8):
        q=numerical_seed(actual,np.zeros(6),71070,index)
        expected=upper_arm(robot,q);observed=upper_arm(actual,q)
        np.testing.assert_allclose(observed["shoulder_world_m"],expected["shoulder_world_m"],atol=1e-12)
        np.testing.assert_allclose(observed["elbow_world_m"],expected["elbow_world_m"],atol=1e-12)
        assert observed["elevation_rad"]==pytest.approx(expected["elevation_rad"],abs=1e-12)
        assert 0 <= soft_cost(actual,q,np.zeros(6))["cost"] <= 1
    np.testing.assert_array_equal(actual.joint_limits,limits)
    from unloading_sim.geometry import OBB
    q=np.zeros(6)
    far=OBB([20.,20.,20.],[.1,.1,.1],np.eye(3),"lift_transverse_test_far","environment")
    measured=actual.upper_arm_mechanism_gap(q,[far])
    assert measured["surface_gap_m"] > .05
    assert measured["pair"]==["J2_link",far.name]
    assert actual.upper_arm_mechanism_gap(q,[])["surface_gap_m"] is None
    assert soft_cost(actual,q,q,.05)["cost"] < soft_cost(actual,q,q,.005)["cost"]
    # Failure of optional distance scoring is neutral, never a validity verdict.
    original=actual._distance
    def unavailable(*a):raise RuntimeError("TEST_DISTANCE_UNAVAILABLE")
    actual._distance=unavailable
    assert actual.upper_arm_mechanism_gap(q,[far])["surface_gap_m"] is None
    actual._distance=original
