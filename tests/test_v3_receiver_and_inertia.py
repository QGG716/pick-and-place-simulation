import numpy as np

from unloading_sim.geometry import OBB,rotation_matrix_from_rpy
from unloading_sim.robot_load.model import cuboid_inertia_at_com
from unloading_sim.robot_load.spatial import spatial_inertia_at_point
from unloading_sim.validation_receiver import supported_by_deck_union


def test_rotated_inertia_and_parallel_axis_match_independent_mass_quadrature():
    mass=42.5;size=np.array([.6,.4,.3]);R=rotation_matrix_from_rpy(.3,-.5,.7);com=np.array([.55,.04,-.02])
    analytic=spatial_inertia_at_point(R@cuboid_inertia_at_com(mass,size)@R.T,mass,com,[0,0,0])
    # Tensor-product two-point Gauss quadrature is exact for this quadratic
    # integrand over a homogeneous cuboid; no parallel-axis helper is reused.
    integral=np.zeros((3,3))
    for x in [-1,1]:
        for y in [-1,1]:
            for z in [-1,1]:
                point=R@(np.array([x,y,z])*size/(2*np.sqrt(3)))+com
                integral+=mass/8*(np.dot(point,point)*np.eye(3)-np.outer(point,point))
    assert np.allclose(analytic,integral,atol=1e-12)


def test_receiver_support_union_detects_corner_gap_not_just_center_support():
    long=OBB([.7,0,.1],[.4,.35,.1],np.eye(3))
    cross=OBB([.075,.3,.1],[.225,1,.1],np.eye(3))
    assert supported_by_deck_union(OBB([.35,0,.35],[.3,.2,.15],np.eye(3)),[long,cross],.0002)
    # Center lies on cross leg but a carton corner hangs beyond both decks.
    assert not supported_by_deck_union(OBB([.075,0,.35],[.3,.2,.15],np.eye(3)),[long,cross],.0002)
