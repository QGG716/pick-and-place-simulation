"""Site STEP static layer built once, CAD design transform and STEP-replaced depth (user decisions 2026-11-27)."""
from pathlib import Path
from types import SimpleNamespace
import json
import os
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import site_layout
from unloading_sim.workbench.single_carton.site_layout import (load_layout,depth_policy,replaced_volumes,transform_source,
    merge_registered_layout)
from unloading_sim.workbench.single_carton.site_scene import build_scene

ROOT=Path(__file__).resolve().parents[1]
MANIFEST='assets/layout/derived/a4b6ed1298f7491b/site-layout.json'
ROLES='configs/workcells/eco65_site_layout_roles_v1.yaml'
REPLACE=dict(mode='STEP_PARTS_REPLACE_RGBD',margin_m=.02,basis='SYNTHETIC',margin_basis='SYNTHETIC')


def field_layout(**extra):
    if not (ROOT/MANIFEST).is_file():pytest.skip('private site STEP derivatives not present (ignored assets)')
    c=dict(manifest=MANIFEST,roles=ROLES,transform_source='CAD_DESIGN_USER_CONFIRMED',transform_basis='TEST',
        excluded_part_ids=[],exclusion_basis='TEST',lift=dict(h_m=-.046,h_basis='TEST 300 mm'),rgbd_depth_policy=REPLACE)
    c.update(extra);return dict(site_layout=c)


def test_policies_validate_and_default_to_previous_behaviour():
    assert transform_source({})=='RGBD_REGISTRATION' and depth_policy({})=={'mode':'KEEP_ALL_RGBD'}
    with pytest.raises(ValueError,match='TRANSFORM_SOURCE'):transform_source(dict(transform_source='GUESS'))
    for bad in (dict(REPLACE,margin_m=.06),dict(REPLACE,margin_m=-.01),dict(REPLACE,basis=''),dict(REPLACE,margin_basis=None)):
        with pytest.raises(ValueError):depth_policy(dict(rgbd_depth_policy=bad))
    assert replaced_volumes(None,dict(rgbd_depth_policy=REPLACE))==[]


def test_cad_design_transform_is_the_roles_fit_and_cache_reuses_until_a_file_changes():
    c=field_layout()
    first=load_layout(c,ROOT);second=load_layout(c,ROOT)
    assert second['cache']['status']=='REUSED' and second['world']==first['world']
    assert first['identity']['transform_source']=='CAD_DESIGN_USER_CONFIRMED' and first['association_distance_m'] is None
    from unloading_sim.eco65.site_mechanism import load_roles,cad_robot_base
    np.testing.assert_allclose(first['identity']['T_base_layout'],cad_robot_base(load_roles(ROOT/ROLES)))
    second['world'][0]['size']=[9,9,9]
    assert load_layout(c,ROOT)['world'][0]['size']!=[9,9,9]    # callers get copies
    with pytest.raises(ValueError,match='DIFFERS_FROM_ROLES'):load_layout(field_layout(T_base_layout=np.eye(4).tolist()),ROOT)
    # A changed input (here only its ctime/mtime) forces a rebuild.
    roles=ROOT/ROLES;st=os.stat(roles);os.utime(roles,ns=(st.st_atime_ns,st.st_mtime_ns+1))
    try:assert load_layout(c,ROOT)['cache']['status']=='BUILT'
    finally:os.utime(roles,ns=(st.st_atime_ns,st.st_mtime_ns))


def test_cad_mode_merge_needs_no_rgbd_registration_and_records_the_confirmation():
    c=field_layout();layout=load_layout(c,ROOT);records=[];tracked=[]
    scene=dict(world=[dict(id='rgbd_kept',category='observed',pose=np.eye(4).tolist(),size=[.01]*3)],layout_explained_depth_removed=7)
    out=merge_registered_layout(scene,None,None,c,SimpleNamespace(track=tracked.append),None,lambda n,v:records.append((n,v)),ROOT)
    reg=out['layout_registration']
    assert reg['valid'] and reg['status']=='USER_CONFIRMED_CAD_DESIGN_TRANSFORM' and reg['cad_is_measured'] is False
    assert out['world'][0]['id']=='rgbd_kept' and len(out['world'])==1+len(layout['world'])
    assert out['layout_explained_depth_removed']==7 and records[0][0]=='layout-registration.json' and tracked
    assert len(replaced_volumes(layout,c['site_layout']))==len(layout['world'])


def flat_frame():
    shape=(60,80);frame=SimpleNamespace(rgb=np.zeros((*shape,3),'uint8'),depth=np.full(shape,1.),depth_scale_m=1.,
        observation_id='flat-wall',intrinsics=dict(width=80,height=60,fx=60,fy=60,ppx=40,ppy=30,model='SYNTHETIC_PINHOLE',coeffs=[0]*5))
    obs=SimpleNamespace(frame=frame,world_camera=np.eye(4),q_capture=[0.]*6)
    site=dict(workspace_bounds_m=[[-.6,-.5,.5],[.6,.5,1.2]],voxel_m=.04,unknown_policy='USER_CONFIRMED_CLEAR',
        unknown_clearance_basis='SYNTHETIC',fixed_obstacles=[],measured_clear_regions=[])
    return obs,SimpleNamespace(observation_id='flat-wall',masks=[]),site


def test_returns_inside_step_parts_build_no_voxel_and_others_stay():
    obs,result,site=flat_frame()
    plain=build_scene(obs,result,[],site)
    observed=[o for o in plain['world'] if o['category']=='observed']
    assert observed and plain['layout_explained_depth_removed']==0
    # One STEP part covering the left half of the wall (x < 0).
    pose=np.eye(4);pose[:3,3]=[-.4,0,1.]
    volume=dict(id='site_layout_part',pose=pose.tolist(),half_m=[.4,.6,.05])
    out=build_scene(obs,result,[],dict(site,layout_replaced_volumes=[volume]))
    kept=[o for o in out['world'] if o['category']=='observed']
    assert out['layout_explained_depth_removed']>0 and out['layout_replaced_volumes']==1
    assert kept and len(kept)<len(observed)
    assert all(np.asarray(o['pose'])[0,3]>-.02 for o in kept)    # nothing left inside the part (cell centres)


def test_parts_wholly_outside_the_workspace_walls_are_not_imported():
    c=field_layout(fixed_mount_interfaces=[dict(link='base_link',part_id='layout_part_350',basis='TEST mount')])
    every=load_layout(c,ROOT)
    c['site']=dict(workspace_bounds_m=[[-1.0,-.8,-.15],[.8,.8,1.3]])
    inside=load_layout(c,ROOT);lo,hi=np.asarray(c['site']['workspace_bounds_m'])
    pruning=inside['identity']['workspace_pruning']
    assert 0<len(inside['world'])<len(every['world']) and pruning['kept']+pruning['pruned']==len(every['world'])
    for o in every['world']:
        P=np.asarray(o['pose']);half=abs(P[:3,:3])@(np.asarray(o['size'])/2)
        overlaps=np.all(P[:3,3]+half>=lo) and np.all(P[:3,3]-half<=hi)
        assert overlaps==(o['id'] in {x['id'] for x in inside['world']})
    assert inside['fixed_mount_pairs']==[['base_link','site_layout_layout_part_350']]
    with pytest.raises(ValueError,match='FIXED_MOUNT_INVALID'):
        load_layout(field_layout(fixed_mount_interfaces=[dict(link='link_2',part_id='layout_part_350',basis='x')]),ROOT)
