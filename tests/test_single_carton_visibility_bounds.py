"""Size-free contact/envelope and unknown-neighbor tests; fake IO only."""
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import pytest
from unloading_sim.workbench.detection import metric_points
from unloading_sim.workbench.single_carton.geometry import corners,transform,contact_candidates
from unloading_sim.workbench.single_carton.observed_surface import observed_surface
from unloading_sim.workbench.single_carton.visibility_geometry import bound_visible_carton
from unloading_sim.workbench.single_carton.partial_geometry import possible_bottom_interval
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.site_scene import build_scene
from unloading_sim.workbench.single_carton.size_policy import mark_incomplete_scene,unmodeled_cartons
from test_single_carton_variable_geometry import measured_box
from test_single_carton_planner import case,wall_near

POLICY=dict(size_policy='MEASURED_PER_INSTANCE',size_policy_basis='SYNTHETIC_VISIBILITY_TEST')


def fixture(angles=(0,30,0)):
    points,mask,center,R=measured_box(angles=angles)
    front=mask&(abs(((points-center)@R)[...,2]-.06)<1e-5)
    frame=SimpleNamespace(rgb=np.zeros(points.shape,'uint8'),depth=points[...,2].astype('float32'),
        depth_scale_m=1.,observation_id='synthetic-size-free-rgbd',
        intrinsics=dict(width=400,height=350,fx=450,fy=450,ppx=200,ppy=175,
                       model='SYNTHETIC_PINHOLE',coeffs=[0.]*5))
    actual=np.eye(4);actual[:3,:3]=R;actual[:3,3]=center
    return frame,front,actual


@pytest.mark.parametrize('condition',['front_only','partial_mask','occlusion','depth_hole'])
def test_one_observed_face_can_bound_payload_and_seal_without_exact_dimensions(condition):
    frame,mask,actual=fixture()
    if condition=='partial_mask':mask[:,:170]=False
    elif condition=='occlusion':
        frame.depth[130:180,155:180]=.45;mask[130:180,155:180]=False
    elif condition=='depth_hole':frame.depth[150:158,180:188]=0
    evidence=observed_surface(metric_points(frame),mask,np.eye(4))
    box,report=bound_visible_carton(frame,np.eye(4),evidence)
    assert box and report['front']['bounded'] and report['depth']['bounded']
    assert box['nominal_size_m'] is None and box['full_dimensions_m'] is None
    assert not box['quality']['hidden_extent_measured'] and not box['prior_applied']
    local=transform(np.linalg.inv(box['pose']),corners(actual,[.24,.18,.12]))
    assert np.all(abs(local)<=np.array(box['size_m'])/2+1e-6)
    c=json.loads(Path('configs/workbench/single_carton.example.json').read_text())
    tool=json.loads(Path('configs/local/eco65_desktop/tool.json').read_text())
    candidates=contact_candidates(box,tool,c['topology'])
    assert candidates and all(x['seal_check']['observed_surface_covered'] for x in candidates)
    low,high=possible_bottom_interval(box,box['pose'])
    assert low<=corners(actual,[.24,.18,.12])[:,2].min()<=high


def test_live_recovery_uses_visibility_fallback_without_old_prior():
    frame,mask,actual=fixture();results=[]
    for old in ([.17,.128,.101],[1.7,1.2,.9]):
        boxes,rejected=recover_boxes(frame,np.eye(4),[dict(label='box')],[mask],dict(POLICY,prior_size_m=old))
        assert not rejected and len(boxes)==1
        assert boxes[0]['dimension_source']=='RGBD_VISIBILITY_COLLISION_ENVELOPE_NOT_MEASURED_DIMENSIONS'
        results.append(boxes)
    assert results[0]==results[1]


def test_head_on_plane_keeps_unbounded_thickness_not_search_limit_as_size():
    frame,mask,_=fixture((0,0,0));e=observed_surface(metric_points(frame),mask,np.eye(4))
    box,report=bound_visible_carton(frame,np.eye(4),e)
    assert box is None and report['front']['bounded']
    assert not report['depth']['bounded'] and report['depth']['upper_m'] is None
    assert not report['depth']['search_limit_is_size_bound']


@pytest.mark.parametrize('case',['image_edge','occluder_to_edge','missing_depth_to_edge'])
def test_unknown_pixels_and_occluders_never_close_front_extent(case):
    frame,mask,_=fixture()
    if case=='image_edge':
        frame.rgb=frame.rgb[:,175:];frame.depth=frame.depth[:,175:];mask=mask[:,175:]
        frame.intrinsics.update(width=225,ppx=25)
    else:
        frame.depth[160:190,:190]=0 if case=='missing_depth_to_edge' else .4
        mask[160:190,:190]=False
    e=observed_surface(metric_points(frame),mask,np.eye(4))
    box,report=bound_visible_carton(frame,np.eye(4),e)
    assert box is None and not report['front']['bounded']
    assert not report['front']['image_or_occlusion_boundary_is_box_edge']


def test_unknown_neighbor_is_occupied_even_with_user_clearance_and_measured_clear_box():
    frame,mask,_=fixture((0,0,0));e=observed_surface(metric_points(frame),mask,np.eye(4))
    obs=SimpleNamespace(frame=frame,world_camera=np.eye(4),q_capture=[0.]*6)
    result=SimpleNamespace(observation_id=frame.observation_id,masks=[mask])
    config=dict(workspace_bounds_m=[[-.18,-.15,.4],[.18,.15,1.1]],voxel_m=.04,
        unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC_CLEAR',
        fixed_obstacles=[],measured_clear_regions=[dict(bounds_m=[[-1,-1,0],[1,1,2]],measurement_basis='SYNTHETIC')])
    rejected=[dict(detection_index=0,evidence=e)]
    scene=build_scene(obs,result,[],config,unresolved_cartons=rejected)
    mark_incomplete_scene(scene,POLICY,rejected)
    assert not unmodeled_cartons(scene) and scene['unbounded_carton_indices']==[0]
    bodies=[o for o in scene['world'] if o['category']=='unresolved_carton_volume']
    assert bodies and all(not b['support_contact_qualified'] for b in bodies)
    # A real point behind this head-on face must still be covered, well past
    # the observed thin slice. No guessed depth limit or field clearance.
    point=np.array([[0.,0.,1.05]])
    assert any(np.all(abs(transform(np.linalg.inv(b['pose']),point))<=np.asarray(b['size'])/2) for b in bodies)
    assert scene['unresolved_carton_obstacles'][0]['carton_end_at_workspace_boundary_assumed'] is False
    json.dumps(scene,allow_nan=False)


def test_possible_target_envelope_never_claims_occluder_points_or_invents_support():
    frame,mask,_=fixture();e=observed_surface(metric_points(frame),mask,np.eye(4))
    box,_=bound_visible_carton(frame,np.eye(4),e);box.update(id='partial-target',detection_index=0)
    # Same plane but a distinct instance: it is not owned by the envelope,
    # even if masks overlap or the old envelope-point field basis is enabled.
    other=np.zeros_like(mask);other[160:180,190:210]=True
    obs=SimpleNamespace(frame=frame,world_camera=np.eye(4),q_capture=[0.]*6)
    result=SimpleNamespace(observation_id=frame.observation_id,masks=[mask,other])
    config=dict(workspace_bounds_m=[[-.2,-.2,.4],[.2,.2,1.]],voxel_m=.02,
        unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC',
        target_envelope_points='TARGET_USER_CONFIRMED',target_envelope_points_basis='SYNTHETIC_OLD_EDGE_PIXELS',
        fixed_obstacles=[],measured_clear_regions=[])
    scene=build_scene(obs,result,[box],config)
    local=scene['target_local_scene'][0]
    assert local['envelope_points_assigned_user_confirmed']==0 and not scene['observed_supports']
    witnesses=metric_points(frame)[other][::20]
    objects=[o for o in scene['world'] if o['category']=='observed']
    assert all(any(np.all(abs(transform(np.linalg.inv(o['pose']),[p]))<=np.asarray(o['size'])/2+1e-9)
                   for o in objects) for p in witnesses)


def representation(index):
    return dict(detection_index=index,
        scope='STATIC_OBSTACLE_INTERSECTION_WITH_ENFORCED_WORKSPACE_NOT_PAYLOAD',
        sensor_free_cells_excluded=True,user_clearance_applied=False)


def test_measured_target_plans_all_four_stages_with_unknown_neighbor_in_world(case):
    p,obs,box,folder,calls,_=case;obs.frame.observation_id='current-test-frame'
    obstacle=dict(id='rgbd_carton_possible_1_0_0_0',category='unresolved_carton_volume',
                  pose=np.eye(4).tolist(),size=[.02,.02,.02],support_contact_qualified=False)
    scene=dict(observation_id=obs.frame.observation_id,world=[obstacle],
               unresolved_carton_obstacles=[representation(1)])
    mark_incomplete_scene(scene,POLICY,[dict(detection_index=1)])
    plan=p.plan_cycle(obs,[box],folder,scene=scene)
    motions=[r for r in calls if r.get('op')=='plan']
    assert plan['full_path_validated'] and len(motions)==4
    assert all(obstacle['id'] in {o['id'] for o in r['world']} for r in motions)
    assert all(not any(obstacle['id'] in pair for pair in r['allowed_pairs']) for r in motions)
    assert motions[-1]['attachment'] and len(p.authority._plans)==4


def test_possible_neighbor_overlap_is_not_claimed_as_measured_penetration(case):
    p,obs,box,folder,calls,_=case;obj=wall_near(box,.005)
    obj.update(category='unresolved_carton_volume',support_contact_qualified=False)
    p.world=[obj]
    with pytest.raises(ValueError,match='GEOMETRY_UNCERTAINTY_OVERLAP_AT_REST'):
        p.plan_cycle(obs,[box],folder)
    rest=json.loads((folder/(box['id']+'-payload-rest-check.json')).read_text())
    assert not rest['measured_volume_overlap_proven'] and not rest['all_hypotheses_infeasible_proven']
    assert not p.authority._plans and not [r for r in calls if r.get('op')=='plan']


def test_possible_neighbor_voxel_top_never_acquires_support_permission(case):
    p,_,box,_,_,_=case;obj=wall_near(box,.0)
    bottom=min(np.asarray(box['corners_m'])[:,2])
    obj.update(category='unresolved_carton_volume',support_contact_qualified=False)
    obj['pose'][2][3]=bottom-obj['size'][2]/2
    assert p.detach_supports([obj],box)==[]
