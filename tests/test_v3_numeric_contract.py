from pathlib import Path

import numpy as np
import pytest

from unloading_sim.geometry import OBB, make_transform, rotation_matrix_from_rpy, rotation_vector_from_matrix
from unloading_sim.ik import solve_ik
from unloading_sim.fanuc_m710id70 import target_pose
from unloading_sim.timing import JointMotionLimits, time_parameterize_joint_path, scale_timed_trajectory_window
from unloading_sim.validation_config import load_validation_config, DEFAULT
from unloading_sim.validation_physics import RigidAttachment, suction_coverage, support_audit, external_load, aggregate_status


def exp(axis, angle):
    axis = np.asarray(axis,float); axis /= np.linalg.norm(axis)
    x,y,z = axis
    K = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K)


@pytest.mark.parametrize("axis", [[1,-1,0],[-1,2,-3],[0,-1,1],[1,0,0]])
@pytest.mark.parametrize("angle", [np.pi,np.pi-1e-7,np.pi-1e-5,1e-9])
def test_so3_log_reconstructs_mixed_sign_half_turn(axis,angle):
    R=exp(axis,angle); w=rotation_vector_from_matrix(R)
    assert np.allclose(exp(w,np.linalg.norm(w)),R,atol=2e-7)


def test_real_fk_ik_residual_not_joint_centering_equilibrium():
    r=load_validation_config().robot()
    q=np.array([.4,.3,.8,.7,.5,-.4])
    result=solve_ik(r,r.fk(q),q+.08,position_tolerance=1e-7,orientation_tolerance=1e-7,max_iterations=400,damping=.01)
    assert result.success
    assert np.linalg.norm(r.fk(result.q)[:3,3]-r.fk(q)[:3,3])<1e-7


def test_full_jacobian_independent_central_difference_rotated_base_and_tcp():
    r=load_validation_config().robot()
    r.base_transform=make_transform(rotation_matrix_from_rpy(.2,-.4,.7),[-.5,.1,.8])
    r.tip_from_tcp=make_transform(rotation_matrix_from_rpy(.3,.2,-.6),[.12,-.08,.29])
    rng=np.random.default_rng(71070)
    for _ in range(8):
        q=rng.uniform(r.joint_limits[:,0]+.1,r.joint_limits[:,1]-.1)
        jac=r.geometric_jacobian(q); numeric=np.zeros_like(jac); h=1e-6
        current=r.fk(q)
        for j in range(6):
            dq=np.eye(6)[j]*h
            plus,minus=r.fk(q+dq),r.fk(q-dq)
            numeric[:3,j]=(plus[:3,3]-minus[:3,3])/(2*h)
            skew=((plus[:3,:3]-minus[:3,:3])/(2*h))@current[:3,:3].T
            numeric[3:,j]=[skew[2,1],skew[0,2],skew[1,0]]
        assert np.allclose(jac,numeric,atol=2e-8)
        # FK must also equal independent composition of named physical flange.
        frames=r.named_link_frames(q)
        assert np.allclose(current,frames['tool0']@r.tip_from_tcp)


@pytest.mark.parametrize("face", ["front","left","right","top"])
@pytest.mark.parametrize("roll", [0,90,180,270])
def test_attachment_roll_never_rotates_initial_box(face,roll):
    box=OBB(np.array([1.3,0,.9]),np.array([.3,.2,.15]),np.eye(3),'box')
    tcp,_=target_pose(1,0,.9,[.6,.4,.3],face,roll)
    attachment=RigidAttachment.capture(tcp,box)
    assert np.allclose(attachment.box_at(tcp).world_from_local,box.world_from_local)
    moved=make_transform(exp([1,-2,3],.4),[.2,-.1,.3])@tcp
    assert np.allclose(attachment.box_at(moved).world_from_local,moved@attachment.tcp_from_box)


def test_actual_suction_coverage_needs_roll_on_side_not_area():
    cfg=load_validation_config(); box=OBB([1.3,0,.9],[.3,.2,.15],np.eye(3))
    counts=[]
    for roll in [0,90]:
        tcp,_=target_pose(1,0,.9,[.6,.4,.3],'left',roll)
        counts.append(suction_coverage(tcp,box,'left',cfg.data['tool'],.0002)['sealed_cups'])
    assert counts==[36,72]


def test_support_uses_actual_box_and_rejects_overhang_gap_and_penetration():
    deck=OBB([0,0,.1],[.5,.4,.1],np.eye(3))
    assert support_audit(OBB([0,0,.35],[.3,.2,.15],np.eye(3)),deck,.0002,.01)['supported']
    for center in [[0,0,.36],[0,0,.34],[.3,0,.35]]:
        assert not support_audit(OBB(center,[.3,.2,.15],np.eye(3)),deck,.0002,.01)['supported']


def test_single_axis_acceleration_lower_bound_and_polynomial_derivatives():
    limits=JointMotionLimits(np.array([100.]),np.array([1.]),np.array([1e6]))
    trajectory=time_parameterize_joint_path([[0.],[1.]],limits)
    # Any unit-distance rest-to-rest motion with |a|<=1 needs at least 2s.
    assert trajectory.duration_seconds>=2.0
    samples=[trajectory.sample(t) for t in np.linspace(0,trajectory.duration_seconds,1001)]
    assert max(abs(s[2][0]) for s in samples)<=1+1e-9
    for t in [.2,.6,.8]:
        t*=trajectory.duration_seconds;h=1e-5
        q,v,a,j=trajectory.sample(t)
        plus=trajectory.sample(t+h);minus=trajectory.sample(t-h)
        assert np.allclose((plus[0]-minus[0])/(2*h),v,atol=1e-7)
        assert np.allclose((plus[1]-minus[1])/(2*h),a,atol=1e-7)
    scaled=scale_timed_trajectory_window(trajectory,0,1,2)
    assert np.allclose(scaled.peak_acceleration,trajectory.peak_acceleration/4)


def test_config_rejects_silent_unknown_override_and_preserves_source(tmp_path):
    bad=tmp_path/'bad.yaml'
    bad.write_text(f'extends: {DEFAULT.as_posix()}\nplanning:\n  collison_margin_m: 0.0\n',encoding='utf-8')
    with pytest.raises(ValueError,match='unknown parameters'):
        load_validation_config(bad)
    good=tmp_path/'good.yaml'
    good.write_text(f'extends: {DEFAULT.as_posix()}\nplanning:\n  collision_margin_m: 0.023\n',encoding='utf-8')
    cfg=load_validation_config(good)
    assert cfg.data['planning']['collision_margin_m']==.023
    assert Path(cfg.sources['planning.collision_margin_m'])==good


def test_rigid_load_com_preserves_original_front_fail_and_collision_pose():
    cfg=load_validation_config();r=cfg.robot();q=np.asarray(cfg.data['robot']['home_joints'])
    tcp=r.fk(q)
    # World-aligned virtual front grasp converted through the actual TCP.
    relative=make_transform(np.array([[0,1,0],[0,0,1],[1,0,0]]),[0,0,.3])
    attachment=RigidAttachment(relative,np.array([.3,.2,.15]),'box')
    load=external_load(r,cfg.tool,q,attachment,42.5,[0,0,0],cfg.model)
    assert np.isclose(load['combined_com_flange_m'][0],.42193923136)
    assert load['statuses']['payload_cg']=='FAIL'
    assert load['qualification']=='FAIL'
    assert load['statuses']['full_robot_dynamics']=='NOT_EVALUATED'
    assert np.allclose(load['box_pose_world'],attachment.box_at(tcp).world_from_local)
    assert aggregate_status(['PASS','NOT_EVALUATED'])=='NOT_EVALUATED'
    assert aggregate_status(['NOT_EVALUATED','FAIL'])=='FAIL'
