"""Missing stereo returns do not create fictitious zero-depth obstacles."""
import json
from pathlib import Path
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.geometry import contact_candidates,seal_check,transform
from unloading_sim.workbench.single_carton.observed_surface import continuous_depth,observed_surface
from test_single_carton_geometry import plane


def test_missing_depth_keeps_neighbour_returns_but_never_fills_holes():
    y,x=np.indices((9,9));p=np.stack([x*.001,y*.001,np.full(x.shape,.5)],-1)
    p[4,4]=np.nan;mask=np.ones(x.shape,bool)
    accepted=continuous_depth(p,mask)
    assert not accepted[4,4] and accepted.sum()==80


def test_real_jump_outside_mask_still_removes_the_measured_edge():
    y,x=np.indices((9,9));p=np.stack([x*.001,y*.001,np.where(x<5,.5,.52)],-1)
    accepted=continuous_depth(p,x<5)
    assert accepted[:,:4].all() and not accepted[:,4:].any()


def test_sparse_holes_do_not_erode_an_entire_reliable_plane():
    p,m=plane(.24,.18);original=p.copy();p[::3,::3]=np.nan
    observed=observed_surface(p,m,np.eye(4))
    assert observed['valid_depth_points']>np.count_nonzero(m)*.75
    face=observed['observed_face']
    # The existing support radius and normal residual are retained. There is
    # no interpolated-depth output, and no fabricated full carton dimension.
    assert face['support_radius_m']==.0025 and observed['plane_rms_m']<.002
    assert not observed['full_extent_bounded'] and observed['full_dimensions_m'] is None
    assert np.isnan(p[::3,::3]).all() and np.isfinite(original).all()


def test_missing_patch_under_actual_cad_ring_still_rejects_the_seal():
    p,m=plane(.24,.18)
    tool=json.loads(Path('configs/local/eco65_desktop/tool.json').read_text())
    config=load(Path('configs/workbench/single_carton.example.json'), code_defaults=False)
    face=observed_surface(p,m,np.eye(4))['observed_face']
    candidate=contact_candidates(dict(faces=[face]),tool,config['topology'])[0]
    ring=next(iter(candidate['rings_tcp'].values()))
    point=transform(candidate['tcp_pose'],[ring[0]])[0]
    hole=m&(np.linalg.norm(p-point,axis=-1)<.009)
    assert hole.sum()>20
    p[hole]=np.nan
    damaged=observed_surface(p,m,np.eye(4))['observed_face']
    result=seal_check(candidate['tcp_pose'],damaged,candidate['rings_tcp'],candidate['suction_normal_tcp'])
    assert not result['valid'] and not result['observed_surface_covered']
