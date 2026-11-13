"""Focused regressions for the directed continuous kinematic revision."""
import numpy as np
from unloading_sim.s50_kinematic_motion import posture,posture_cost,receiving_height,continuous_samples,_hermite


def test_receiver_tracks_carton_bottom_with_literal_stroke_limits():
    pose=np.eye(4);pose[2,3]=2.25
    high=receiving_height(pose,[.3,.2,.15])
    assert high['surface_height_m']==1.5 and high['clamped']
    np.testing.assert_allclose(high['actual_gap_m'],.6)
    pose[2,3]=1.4
    middle=receiving_height(pose,[.3,.2,.15])
    np.testing.assert_allclose(middle['surface_height_m'],1.15)
    np.testing.assert_allclose(middle['actual_gap_m'],.1)
    assert not middle['clamped']
    pose[2,3]=.65
    assert receiving_height(pose,[.3,.2,.15])['surface_height_m']==.55


def test_forearm_preference_uses_world_geometry_and_never_j3_sign():
    class Frames:
        def named_link_frames(self,q):
            frames={name:np.eye(4) for name in ['s50_link_2','s50_link_3','s50_link_4','s50_link_5']}
            frames['s50_link_3'][:3,3]=[1,2,3]
            frames['s50_link_4'][:3,3]=[1+np.sqrt(3),2,4]
            frames['s50_link_5'][:3,3]=[1+np.sqrt(3),2,3.8]
            return frames
    robot=Frames();a=np.deg2rad([0,20,-120,0,0,0]);b=a.copy();b[2]=np.deg2rad(40)
    preference=dict(j2_preferred_deg=[-100,40],forearm_horizontal_max_deg=30)
    metric=posture(robot,a)
    np.testing.assert_allclose(metric['forearm_elevation_deg'],30)
    np.testing.assert_allclose(metric['wrist_direction_world'],[0,0,-1])
    assert posture_cost(robot,a,preference)==posture_cost(robot,b,preference)
    c=a.copy();c[1]=np.deg2rad(60)
    assert np.isfinite(posture_cost(robot,c,preference)) and posture_cost(robot,c,preference)>0


def test_small_forward_shoulder_tilt_is_free_and_large_tilt_is_only_a_cost():
    class Frames:
        def named_link_frames(self,q):
            f={f's50_link_{i}':np.eye(4) for i in range(2,6)}
            f['s50_link_3'][:3,3]=[np.sin(q[1]),0,np.cos(q[1])]
            f['s50_link_4'][:3,3]=f['s50_link_3'][:3,3]+[1,0,0]
            f['s50_link_5'][:3,3]=f['s50_link_4'][:3,3]+[0,0,-.1]
            return f
    p=dict(j2_preferred_deg=[-190,190],forearm_horizontal_max_deg=30,
        upper_arm_forward_free_deg=30,upper_arm_large_forward_weight=32)
    costs=[posture_cost(Frames(),np.deg2rad([0,a,0,0,0,0]),p) for a in [-60,20,30,60]]
    np.testing.assert_allclose(costs,[0,0,0,32],atol=1e-12)


def test_continuous_ik_knots_keep_shared_velocity_and_conservative_rates():
    points=np.array([[0,0],[.2,.1],[.4,.3],[.7,.5]])
    stations=np.array([0,.2,.5,1.])
    rows,law=continuous_samples(points,stations=stations)
    np.testing.assert_allclose(rows[0]['q'],points[0]);np.testing.assert_allclose(rows[-1]['q'],points[-1])
    np.testing.assert_allclose(rows[0]['velocity'],[0,0]);np.testing.assert_allclose(rows[-1]['velocity'],[0,0])
    assert law['maximum_sampled_velocity_rad_s']<=.35
    assert law['maximum_sampled_acceleration_rad_s2']<=.5
    for station in stations[1:-1]:
        q,v,_,_=_hermite(points,stations,np.array([station-1e-9,station,station+1e-9]))
        np.testing.assert_allclose(q[1],points[np.where(stations==station)[0][0]])
        np.testing.assert_allclose(v[0],v[-1],atol=1e-7)
        assert np.linalg.norm(v[1])>.1
    assert not law['interior_stop_required']
