"""Literal partial observations and field scene tests; physical result NOT_RUN."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.partial_geometry import recover_partial
from unloading_sim.workbench.single_carton.geometry import contact_candidates,transform
from unloading_sim.workbench.single_carton.site_scene import build_scene,load_site
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.contracts import DetectionResult
from test_single_carton_geometry import plane
from test_single_carton_observation import observation,frame
from test_single_carton_variable_geometry import measured_box

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('case',['cropped','occluded','depth_hole','single_patch'])
def test_partial_surface_produces_supported_grasp_and_full_payload_envelope(case):
    p,m=plane(width=.24,height=.18)
    if case=='cropped':p=p[:,45:];m=m[:,45:]
    elif case=='occluded':m[:,:45]=False;p[:,:45,2]=.5
    elif case=='depth_hole':p[20:28,20:28]=np.nan
    elif case=='single_patch':m[:,:30]=False;m[-25:]=False
    box=recover_partial(p,m,np.eye(4),prior=[.24,.18,.12])
    assert box['uncertainty']['unique_pose'] is False
    assert not box['quality']['hidden_extent_measured']
    assert box['nominal_size_m']==[.24,.18,.12] and box['size_m'][2]>=.12
    config=load(ROOT/'configs/workbench/single_carton.example.json')
    tool=json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())
    candidates=contact_candidates(box,tool,config['topology'])
    assert candidates and candidates[0]['seal_check']['observed_surface_covered']
    assert candidates[0]['face']['observed'] and len(candidates[0]['commanded_cup_ids'])==12


def test_true_hidden_corners_are_inside_continuous_prior_envelope():
    from unloading_sim.workbench.single_carton.geometry import corners
    for angles in [(0,0,0),(10,30,17),(0,0,42)]:
        p,m,c,R=measured_box((.24,.18,.12),angles)
        m[:,:180]=False
        box=recover_partial(p,m,np.eye(4),prior=[.24,.18,.12])
        actual=np.eye(4);actual[:3,:3]=R;actual[:3,3]=c
        local=transform(np.linalg.inv(box['pose']),corners(actual,[.24,.18,.12]))
        assert np.all(abs(local)<=np.asarray(box['size_m'])/2+.003)


def test_prior_conflict_and_insufficient_patch_have_specific_reasons():
    p,m=plane(width=.4,height=.3)
    with pytest.raises(ValueError,match='DIMENSION_PRIOR_CONFLICT'):recover_partial(p,m,np.eye(4),prior=[.15,.1,.1])
    p[:]=np.nan
    with pytest.raises(ValueError,match='INSUFFICIENT_INTERIOR_DEPTH'):recover_partial(p,m,np.eye(4),prior=[.15,.1,.1])


def test_default_real_site_has_no_simulation_furniture_or_free_unknown_space():
    c=load(ROOT/'configs/workbench/single_carton.example.json')
    assert c['camera']['width']==640 and c['camera']['height']==480 and c['camera']['fps']==30
    assert c['geometry']['prior_size_m']==[.15,.1,.1]
    assert c['release_policy']=='MEASURED_SITE_RECEIVER_REQUIRED'
    p=SinglePlanner(c,None,None,None)
    assert p.build_world({'scene':{'obstacles':['old-trailer','old-conveyor']}})==[]
    assert c['site']['unknown_policy']=='OCCUPIED' and c['site']['measured_clear_regions']==[]
    with pytest.raises(ValueError,match='SITE_RECEIVER_NOT_CONFIGURED'):p.receiver()


def small_site():
    return dict(workspace_bounds_m=[[-.04,-.04,.4],[.04,.04,.6]],voxel_m=.02,
        unknown_policy='OCCUPIED',fixed_obstacles=[],measured_clear_regions=[])


def test_rgbd_scene_retains_unknown_and_unlabelled_obstacles_same_capture():
    obs=observation();result=DetectionResult(obs.frame.observation_id,obs.frame.stamp.source,1,())
    scene=build_scene(obs,result,[],small_site())
    assert scene['unknown_voxels']>0 and scene['observed_voxels']>0
    assert scene['imported_simulation_objects']==0 and scene['q_capture']==[0]*6
    assert {o['category'] for o in scene['world']} >= {'observed','unknown','workspace_boundary'}
    with pytest.raises(ValueError,match='SCENE_FRAME_MISMATCH'):
        build_scene(obs,replace(result,observation_id='old'),[],small_site())


def test_measured_blind_clearance_never_erases_observed_obstacle():
    obs=observation();result=DetectionResult(obs.frame.observation_id,obs.frame.stamp.source,1,())
    c=small_site();c['measured_clear_regions']=[dict(bounds_m=[[-1,-1,-1],[1,1,1]],measurement_basis='SYNTHETIC_TEST_ONLY')]
    scene=build_scene(obs,result,[],c)
    assert scene['unknown_voxels']==0 and scene['observed_voxels']>0


def test_unreconstructed_neighbor_mask_stays_in_environment():
    obs=observation();m=np.zeros(obs.frame.depth.shape,bool);m[:,15:]=True
    items=({'label':'box','score':.8,'frame_id':'camera_optical'},)
    result=DetectionResult(obs.frame.observation_id,obs.frame.stamp.source,1,items,(m,))
    scene=build_scene(obs,result,[],small_site())
    assert scene['observed_voxels']>0


def test_field_scene_requires_measurement_for_blind_clear_and_receiving(tmp_path):
    c=json.loads((ROOT/'configs/workbench/real_site.example.json').read_text())
    c['measured_clear_regions']=[dict(bounds_m=[[-1,-1,-1],[1,1,1]])]
    p=tmp_path/'site.json';p.write_text(json.dumps(c))
    with pytest.raises(ValueError,match='FIELD_MEASUREMENT'):load_site(p)
    c['measured_clear_regions']=[];c['receiver']=dict(support_ids=['nonexistent'])
    p.write_text(json.dumps(c))
    with pytest.raises(ValueError,match='RECEIVER_MUST_BIND'):load_site(p)


def test_self_filter_uses_capture_transform_and_does_not_clear_a_gap_between_solids():
    from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
    urdf='<robot name="test"><link name="link_1">'+''.join(
        f'<collision><origin xyz="{x} 0 0"/><geometry><box size="0.04 0.04 0.04"/></geometry></collision>' for x in [-.04,.04])+'</link></robot>'
    def frames(q):
        T=np.eye(4);T[0,3]=q[0];return {'link_1':T}
    f=RobotSelfFilter(urdf,SimpleNamespace(world_from_base=np.eye(4),frames=frames))
    assert f.contains([[-.04,0,0],[0,0,0],[.04,0,0]],np.zeros(6)).tolist()==[True,False,True]
    # Corners may each lie in a different solid, with an unknown gap inside.
    vertices=np.array([[x,y,z] for x in [-.04,.04] for y in [-.01,.01] for z in [-.01,.01]])
    assert not f.contains(vertices,np.zeros(6),group_size=8)[0]
    assert not f.contains([[-.04,0,0]],np.ones(6))[0]


def test_execute_recheck_rejects_target_depth_holes_and_replacement_at_same_depth():
    from unloading_sim.workbench.single_carton.perception import target_unchanged
    a=observation();mask=np.ones(a.frame.depth.shape,bool)
    depth=a.frame.depth.copy();depth[:4]=0
    b=replace(a,frame=replace(a.frame,depth=depth))
    with pytest.raises(ValueError,match='DEPTH_COVERAGE_LOST'):target_unchanged(a,b,mask)
    b=replace(a,frame=replace(a.frame,rgb=np.full_like(a.frame.rgb,200)))
    with pytest.raises(ValueError,match='TARGET_CHANGED'):target_unchanged(a,b,mask)


def test_execute_recheck_does_not_dilute_small_obstacle_in_unchanged_background():
    from unloading_sim.workbench.single_carton.perception import unchanged
    a=observation();a=replace(a,frame=replace(a.frame,rgb=np.zeros((100,100,3),dtype=np.uint8),
        depth=np.full((100,100),500,dtype=np.uint16),intrinsics=dict(a.frame.intrinsics,width=100,height=100)))
    depth=a.frame.depth.copy();depth[40:43,40:43]-=20
    b=replace(a,frame=replace(a.frame,depth=depth))
    with pytest.raises(ValueError,match='LOCAL_SCENE_CHANGED'):unchanged(a,b)
    depth=a.frame.depth.copy();depth[40,40]-=20
    b=replace(a,frame=replace(a.frame,depth=depth))
    assert unchanged(a,b)['max_local_changed_pixels']==1


def test_release_requires_real_support_and_all_geometry_hypotheses():
    c=load(ROOT/'configs/workbench/single_carton.example.json')
    c['site']['receiver']=dict(support_ids=['actual_table'],bounds_m=[[-.15,-.15,0.],[.15,.15,.049]],
                               margin_m=.003,measurement_basis='SYNTHETIC_TEST_ONLY')
    model=SimpleNamespace(world_from_base=np.eye(4),frames=lambda q:{'link_6':np.eye(4)})
    p=SinglePlanner(c,None,None,model);T=np.eye(4);T[2,3]=.10;reference=np.eye(4);reference[2,3]=.151
    box=dict(pose=T.tolist(),size_m=[.1,.08,.102],uncertainty=dict(reference_pose=reference.tolist(),
        dimension_tolerance_m=.001,hypotheses=[dict(dimensions_m=[.1,.08,.1],yaw_center_rad=0.,yaw_half_width_rad=.001,
                                              center_interval_uv_m=[[0.,0.],[0.,0.]])]))
    table=np.eye(4);table[2,3]=.0245
    world=[dict(id='actual_table',size=[.4,.4,.049],pose=table.tolist())]
    result=p.release_check(np.zeros(6),T,box,world)
    assert result['status']=='PASS' and result['support_confirmed'] is False
    with pytest.raises(ValueError,match='SUPPORT_NOT_IN_COLLISION_SCENE'):p.release_check(np.zeros(6),T,box,[])
    uncertain=deepcopy(box);uncertain['uncertainty']['hypotheses'][0]['dimensions_m'][2]=.08
    with pytest.raises(ValueError,match='GEOMETRY_UNCERTAINTY_AFFECTS_RECEIVER'):p.release_check(np.zeros(6),T,uncertain,world)
    c['site']['receiver']['bounds_m'][0][0]=-.5
    with pytest.raises(ValueError,match='REGION_NOT_COVERED'):p.release_check(np.zeros(6),T,box,world)
