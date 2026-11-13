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


def test_prior_fit_rejects_mask_bleed_beyond_tolerance_on_either_face_axis():
    # Within the 8 mm tolerance (+3 mm edge noise) on either axis: accepted.
    for width,height in [(.15,.1),(.158,.1),(.15,.108)]:
        p,m=plane(width=width,height=height);recover_partial(p,m,np.eye(4),prior=[.15,.1,.1])
    # Yaw-cell growth of the other side must not widen the face-size test.
    for width,height in [(.15,.118),(.166,.1)]:
        p,m=plane(width=width,height=height)
        with pytest.raises(ValueError,match='DIMENSION_PRIOR_CONFLICT'):recover_partial(p,m,np.eye(4),prior=[.15,.1,.1])


def test_oblique_opencv_rectangle_does_not_scale_recovered_native_frames():
    from scipy.spatial.transform import Rotation
    p,m=plane(width=.24,height=.18)
    rotation=Rotation.from_euler('xyz',[14,-21,31],degrees=True).as_matrix()
    p=p@rotation.T
    box=recover_partial(p,m,np.eye(4),prior=[.24,.18,.12])
    config=load(ROOT/'configs/workbench/single_carton.example.json')
    tool=json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())
    candidates=contact_candidates(box,tool,config['topology'])
    matrices=[box['pose'],box['uncertainty']['reference_pose']]
    for c in candidates:
        matrices.extend([c['tcp_pose'],c['precontact_pose'],np.linalg.inv(c['tcp_pose'])@box['pose']])
    for T in matrices:
        R=np.asarray(T)[:3,:3]
        assert np.linalg.norm(R.T@R-np.eye(3))<1e-12
        assert abs(np.linalg.det(R)-1)<1e-12
    invalid=np.eye(4);invalid[0,0]=1.01
    with pytest.raises(ValueError,match='INVALID_RIGID_TRANSFORM'):
        recover_partial(p,m,invalid,prior=[.24,.18,.12])


def test_default_real_site_uses_explicit_user_clearance_without_simulation_furniture():
    c=load(ROOT/'configs/workbench/single_carton.example.json')
    assert c['camera']['width']==640 and c['camera']['height']==480 and c['camera']['fps']==30
    # Field-measured carton, never the simulation's 150 x 100 x 100 mm.
    assert c['geometry']['prior_size_m']==[.17,.128,.101]
    assert '7ccdb15e' in c['geometry']['prior_measurement_basis']
    from unloading_sim.workbench.single_carton.config import issues
    # Remote GPU connection fields come from an ignored local file; geometry
    # and every other field-safety parameter must be complete in the example.
    assert [e for e in issues(c) if not e.startswith('detector.remote')]==[]
    for basis in (None,'','  '):
        broken=dict(c,geometry=dict(c['geometry'],prior_measurement_basis=basis))
        assert any(e.startswith('geometry.prior_measurement_basis') for e in issues(broken))
    assert c['release_policy']=='USER_DEFERRED_RECEIVER_QUALIFICATION'
    assert c['receiver_qualification_basis']
    p=SinglePlanner(c,None,None,None)
    assert p.build_world({'scene':{'obstacles':['old-trailer','old-conveyor']}})==[]
    assert c['site']['unknown_policy']=='USER_CONFIRMED_CLEAR' and c['site']['unknown_clearance_basis']
    assert c['site']['measured_clear_regions']==[]
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


def test_user_clearance_omits_only_unknown_and_preserves_observed_fixed_and_boundary_objects():
    obs=observation();result=DetectionResult(obs.frame.observation_id,obs.frame.stamp.source,1,())
    c=small_site();c['fixed_obstacles']=[dict(id='measured-bracket',pose=np.eye(4).tolist(),size_m=[.01,.02,.03])]
    occupied=build_scene(obs,result,[],c)
    c.update(unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC_TEST_USER_CONFIRMATION')
    cleared=build_scene(obs,result,[],c)
    kept=[o for o in occupied['world'] if o['category']!='unknown']
    assert [o['id'] for o in cleared['world']]==[o['id'] for o in kept]
    assert [o for o in cleared['world'] if o['category']!='observed']==[o for o in kept if o['category']!='observed']
    # Observed cells shrink only on their camera-facing side to the measured
    # extent (+3 mm); they stay inside the original cell and cover every point.
    def bounds(o):
        c=np.asarray(o['pose'])[:3,3];h=np.asarray(o['size'])/2;return c-h,c+h
    for a,b in zip(cleared['world'],kept):
        if a['category']!='observed':continue
        (alo,ahi),(blo,bhi)=bounds(a),bounds(b)
        assert np.all(alo>=blo-1e-12) and np.all(ahi<=bhi+1e-12)
    from unloading_sim.workbench.detection import metric_points
    points=metric_points(obs.frame);depth=obs.frame.depth*obs.frame.depth_scale_m
    world=transform(np.asarray(obs.world_camera),points[(depth>.1)&(depth<3.)].reshape(-1,3))
    lo,hi=np.asarray(c['workspace_bounds_m'])
    world=world[np.all((world>=lo)&(world<hi),axis=1)]
    boxes=[bounds(o) for o in cleared['world'] if o['category']=='observed']
    assert len(world) and all(any(np.all(p>=a-1e-9) and np.all(p<=b+1e-9) for a,b in boxes) for p in world)
    assert cleared['camera_facing_measured_extent_runs']>0 and occupied['camera_facing_measured_extent_runs']==0
    assert cleared['observed_voxels']==occupied['observed_voxels']>0
    assert cleared['unknown_voxels']==occupied['unknown_voxels']>0
    assert cleared['unknown_collision_voxels']==0
    assert cleared['user_confirmed_clear_unknown_voxels']==occupied['unknown_voxels']
    assert cleared['unknown_clearance_evidence']=='USER_FIELD_CONFIRMATION_NOT_SENSOR_OBSERVATION'
    assert {o['category'] for o in cleared['world']}>={'observed','measured_fixed','workspace_boundary'}


@pytest.mark.parametrize('policy,basis,reason',[
    ('USER_CONFIRMED_CLEAR',None,'FIELD_BASIS_REQUIRED'),('USER_CONFIRMED_CLEAR','','FIELD_BASIS_REQUIRED'),
    ('FREE',None,'POLICY_INVALID')])
def test_unknown_clearance_needs_explicit_field_basis(policy,basis,reason,tmp_path):
    c=json.loads((ROOT/'configs/workbench/real_site.example.json').read_text())
    c.update(unknown_policy=policy,unknown_clearance_basis=basis)
    path=tmp_path/'site.json';path.write_text(json.dumps(c))
    with pytest.raises(ValueError,match=reason):load_site(path)
    obs=observation();result=DetectionResult(obs.frame.observation_id,obs.frame.stamp.source,1,())
    with pytest.raises(ValueError,match=reason):build_scene(obs,result,[],c)


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
    c['release_policy']='MEASURED_SITE_RECEIVER_REQUIRED'
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


def envelope_scene_case():
    """Flat depth at 0.50 m; a target envelope that the unmasked depth enters.

    World = camera (T = I). Envelope x 0..60 mm, y -40..40 mm, z 490..550 mm;
    SAM missed every pixel, so with the default policy the points inside the
    envelope are world obstacles. Six columns just beyond +x lie within 8 mm.
    """
    f=frame();shape=(40,60);depth=np.full(shape,500,np.uint16)
    f=replace(f,intrinsics=dict(width=60,height=40,fx=200,fy=200,ppx=30,ppy=20,model='SYNTHETIC_PINHOLE',coeffs=[0]*5),
              rgb=np.zeros((*shape,3),np.uint8),depth=depth)
    obs=observation(f);mask=np.zeros(shape,bool)
    T=np.eye(4);T[:3,3]=[.03,0.,.52];size=[.06,.08,.06]
    from unloading_sim.workbench.single_carton.geometry import corners
    face=dict(observed=True,center_m=[.03,0.,.49],normal=[0.,0.,-1.],u=[1.,0.,0.],v=[0.,-1.,0.],size_m=[.04,.06])
    box=dict(id='target',detection_index=0,pose=T.tolist(),size_m=size,corners_m=corners(T,size).tolist(),faces=[face],quality={})
    result=DetectionResult(f.observation_id,f.stamp.source,1,({'label':'carton','score':.9,'frame_id':'camera_optical'},),(mask,))
    site=dict(workspace_bounds_m=[[-.08,-.08,.45],[.12,.08,.6]],voxel_m=.04,unknown_policy='USER_CONFIRMED_CLEAR',
              unknown_clearance_basis='SYNTHETIC_TEST',fixed_obstacles=[],measured_clear_regions=[])
    return obs,result,box,site


def covered(world,points):
    boxes=[(np.asarray(o['pose'])[:3,3],np.asarray(o['size'])/2) for o in world if o['category']=='observed']
    return all(any(np.all(abs(p-c)<=h+1e-9) for c,h in boxes) for p in points)


def test_points_inside_target_envelope_stay_obstacles_by_default():
    obs,result,box,site=envelope_scene_case()
    scene=build_scene(obs,result,[box],site)
    assert scene['target_envelope_points']=='OBSTACLE' and scene['target_envelope_points_basis'] is None
    assert 'envelope_points_assigned_user_confirmed' not in scene['target_local_scene'][0]
    world=scene['world']+[dict(id='target',pose=box['pose'],size=box['size_m'],category='carton')]
    rest=SinglePlanner(load(ROOT/'configs/workbench/single_carton.example.json'),None,None,None).payload_rest_check(world,box)
    assert rest['intersections'] and all(h['category']=='observed' for h in rest['intersections'])
    # Near-target refinement never drops a measured point.
    assert scene['near_target_refined_runs']>0
    from unloading_sim.workbench.detection import metric_points
    assert covered(scene['world'],metric_points(obs.frame).reshape(-1,3))


@pytest.mark.parametrize('margin',[0.,.008])
def test_user_confirmed_policy_assigns_only_points_in_envelope_plus_margin(margin):
    obs,result,box,site=envelope_scene_case()
    site.update(target_envelope_points='TARGET_USER_CONFIRMED',target_envelope_points_basis='SYNTHETIC_USER_CONFIRMATION',
                target_envelope_points_margin_m=margin)
    scene=build_scene(obs,result,[box],site)
    from unloading_sim.workbench.detection import metric_points
    p=metric_points(obs.frame).reshape(-1,3);half=np.asarray(box['size_m'])/2+margin
    expected=np.all(abs(p-np.asarray(box['pose'])[:3,3])<=half,axis=1)&(p[:,2]>=.49)
    assert scene['target_local_scene'][0]['envelope_points_assigned_user_confirmed']==int(expected.sum())>0
    assert scene['target_envelope_points_evidence']=='USER_FIELD_CONFIRMATION_FALSE_OBSTACLES_NOT_SENSOR_OBSERVATION'
    assert scene['target_envelope_points_margin_m']==margin
    # Every point outside the assigned region is still covered by an obstacle box.
    assert covered(scene['world'],p[~expected])
    world=scene['world']+[dict(id='target',pose=box['pose'],size=box['size_m'],category='carton')]
    rest=SinglePlanner(load(ROOT/'configs/workbench/single_carton.example.json'),None,None,None).payload_rest_check(world,box)
    assert rest['intersections']==[]
    if margin:assert int(expected.sum())>int((np.all(abs(p-np.asarray(box['pose'])[:3,3])<=half-margin,axis=1)&(p[:,2]>=.49)).sum())


@pytest.mark.parametrize('update,reason',[
    (dict(target_envelope_points='TARGET_USER_CONFIRMED'),'FIELD_BASIS_REQUIRED'),
    (dict(target_envelope_points='TARGET_USER_CONFIRMED',target_envelope_points_basis=' '),'FIELD_BASIS_REQUIRED'),
    (dict(target_envelope_points='TARGET_USER_CONFIRMED',target_envelope_points_basis='x',target_envelope_points_margin_m=.05),'MARGIN_INVALID'),
    (dict(target_envelope_points='IGNORE'),'POLICY_INVALID')])
def test_target_envelope_policy_needs_explicit_basis_and_bounded_margin(update,reason,tmp_path):
    c=json.loads((ROOT/'configs/workbench/real_site.example.json').read_text())
    for k in ('target_envelope_points','target_envelope_points_basis','target_envelope_points_margin_m'):c.pop(k,None)
    c.update(update);path=tmp_path/'site.json';path.write_text(json.dumps(c))
    with pytest.raises(ValueError,match=reason):load_site(path)


def test_default_site_records_the_user_confirmed_false_obstacle_basis():
    c=load_site(ROOT/'configs/workbench/real_site.example.json')
    assert c['target_envelope_points']=='TARGET_USER_CONFIRMED' and c['target_envelope_points_margin_m']==.008
    assert '1980356201804419a9e66e4063b9d732' in c['target_envelope_points_basis']
