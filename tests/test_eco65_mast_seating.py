import importlib.util
from pathlib import Path
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('seating',Path(__file__).parents[1]/'tools/eco65_mast_seating.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_contact_uses_only_overlapping_surface():
    lower=np.array([[[0,0,1],[1,0,1],[0,1,1]],[[5,5,8],[6,5,8],[5,6,8]]])
    upper=np.array([[[0,0,.2],[1,0,.2],[0,1,.2]]])
    result=module.vertical_seating(lower,upper)
    assert result['upper_z_translation_m']==pytest.approx(.8)
    assert result['witness']['lower_triangle']==0


def test_contact_can_be_at_edge_crossing_not_source_vertex():
    # Edges cross at (.5, 1.5), height 3; neither triangle has this vertex.
    lower=np.array([[[0,0,0],[2,0,0],[0,2,4]]])
    upper=np.array([[[.5,-1,0],[.5,3,0],[1.5,-1,0]]])
    result=module.vertical_seating(lower,upper)
    assert result['upper_z_translation_m']==pytest.approx(3.)


def test_no_overlap_is_rejected():
    with pytest.raises(ValueError,match='NO_PROJECTED'):
        module.vertical_seating([[[0,0,0],[1,0,0],[0,1,0]]],[[[2,2,0],[3,2,0],[2,3,0]]])


def test_conformal_seat_follows_both_planes():
    v,f,r=module.conformal_seat([[[0,0,0],[1,0,0],[0,1,0]]],[[[0,0,1],[0,1,2],[1,0,1]]])
    t=v[f];volume=np.sum(np.einsum('ij,ij->i',t[:,0],np.cross(t[:,1],t[:,2])))/6
    assert volume==pytest.approx(2/3)
    assert r['max_surface_error_m']==pytest.approx(0)
    assert r['contact_area_m2']==pytest.approx(.5)


def test_seat_thickness_limit_does_not_bridge_distant_surfaces():
    with pytest.raises(ValueError,match='NO_VERIFIED_SEAT_PATCHES'):
        module.conformal_seat([[[0,0,0],[1,0,0],[0,1,0]]],
                              [[[0,0,1],[0,1,2],[1,0,1]]],max_thickness_m=.015)
