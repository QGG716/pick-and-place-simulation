"""A diagram may show nearest points only when the original geometry proves them."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from visualize_current_carton_blockers import normal_gap_witness
from unloading_sim.workbench.single_carton.geometry import corners,transform


def fixture():
    T=np.eye(4);size=[.1,.1,.11]
    b=dict(pose=T.tolist(),size_m=size,corners_m=corners(T,size).tolist())
    face=dict(center_m=[0.,0.,.05],normal=[0.,0.,1.])
    vertices=np.array([[0.,0.,.057],[.01,.01,.06]])
    return b,face,vertices


def test_verified_gap_has_three_distinct_source_positions():
    b,f,v=fixture();w=normal_gap_witness(v,b,f,.002)
    assert w is not None
    np.testing.assert_allclose(w['rigid_point_m'],[0,0,.057])
    np.testing.assert_allclose(w['envelope_point_m'],[0,0,.055])
    np.testing.assert_allclose(w['observed_plane_point_m'],[0,0,.05])
    assert w['envelope_front_from_observed_plane_m']==pytest.approx(.005)


@pytest.mark.parametrize('case',['outside_face','distance_mismatch','penetration','opposite_normal'])
def test_scalar_does_not_fabricate_a_nearest_pair(case):
    b,f,v=fixture();gap=.002
    if case=='outside_face':v[:,0]+=.2
    elif case=='distance_mismatch':gap=.003
    elif case=='penetration':v[:,2]-=.01
    else:f['normal']=[0,0,-1]
    assert normal_gap_witness(v,b,f,gap) is None


def test_witness_is_invariant_under_rigid_scene_transform():
    b,f,v=fixture();T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[24,38,62],degrees=True).as_matrix();T[:3,3]=[-.6,.2,.37]
    moved=dict(b,pose=T.tolist(),corners_m=transform(T,np.array(b['corners_m'])).tolist())
    face=dict(center_m=transform(T,[f['center_m']])[0].tolist(),normal=(T[:3,:3]@f['normal']).tolist())
    w=normal_gap_witness(transform(T,v),moved,face,.002)
    assert w['independent_gap_m']==pytest.approx(.002,abs=1e-12)
    np.testing.assert_allclose(w['envelope_point_m'],transform(T,[[0,0,.055]])[0],atol=1e-12)
