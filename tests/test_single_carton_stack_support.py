"""Confirmed stack topology with no lower size prior; fake data/IO only."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.stack_support import policy,infer_supports
from unloading_sim.workbench.single_carton.geometry import corners,transform
from unloading_sim.workbench.single_carton.size_policy import mark_incomplete_scene,unmodeled_cartons
from test_single_carton_planner import case

GEOMETRY=dict(size_policy='MEASURED_PER_INSTANCE',size_policy_basis='SYNTHETIC',
    confirmed_stack_support=dict(mode='USER_CONFIRMED_TWO_CARTON_STACK',basis='SYNTHETIC_STACK_TOPOLOGY_NOT_FIELD'))


def fixture():
    a=np.zeros((60,80),bool);a[:30,20:60]=True
    b=np.zeros_like(a);b[30:,22:58]=True
    y,x=np.indices(a.shape);pts=np.stack([np.full(a.shape,.38),(x-40)*.003,.294-(y-30)*.003],axis=-1)
    uv=np.stack([pts[...,1],pts[...,2]-.25],axis=-1)[b]
    face=dict(center_m=[.38,0,.25],normal=[1.,0,0],u=[0,1.,0],v=[0,0,1.],support_points_uv_m=uv.tolist())
    T=np.eye(4);T[:3,3]=[.35,0,.35]
    box=dict(id='upper',detection_index=0,pose=T.tolist(),size_m=[.15,.1,.1],corners_m=corners(T,[.15,.1,.1]).tolist(),
        quality=dict(all_dimensions_source='RGBD_VISIBLE_BOUNDARIES'))
    obs=SimpleNamespace(frame=SimpleNamespace(observation_id='synthetic-stack-frame'))
    result=SimpleNamespace(masks=[a,b]);rejected=[dict(detection_index=1,evidence=dict(observed_face=face))]
    site=dict(workspace_bounds_m=[[-.8,-.8,-.15],[.8,.8,1.3]])
    return obs,result,[box],rejected,site,deepcopy(GEOMETRY),pts,b


def test_stack_relation_retains_full_hidden_static_volume_and_exact_support_plane():
    args=fixture();bodies,records=infer_supports(*args)
    body,record=bodies[1];plane=body['support_plane'];n=np.asarray(plane['normal_world']);d=plane['offset_m']
    assert record['interface_points']>=30 and not record['hidden_depth_measured'] and record['full_dimensions_m'] is None
    assert not record['carton_end_at_workspace_boundary_assumed']
    rng=np.random.default_rng(6518);p=rng.uniform([-.8,-.8,-.15],[.8,.8,1.3],(50000,3))
    # All admissible positions in the entire workspace, not only the camera's
    # observed thin face, must be inside the represented supporting body.
    possible=p[(p[:,0]<=.384)&(p@n<=d)]
    assert len(possible)>1000
    assert np.all(abs(transform(np.linalg.inv(body['pose']),possible))<=np.asarray(body['size'])/2+1e-9)
    assert np.max(corners(body['pose'],body['size'])@n)==pytest.approx(d,abs=1e-9)
    assert body['supports_target_id']=='upper' and record['support_penetration_limit_m']==.0004


@pytest.mark.parametrize('change,reason',[
    ('extra_instance','TWO_CARTON'),('uncertain_upper','MEASURED_UPPER'),('prior_upper','MEASURED_UPPER'),
    ('contradicting_lower','CONFLICTS'),('no_interface','NO_SAME_FRAME')])
def test_stack_confirmation_never_binds_unrelated_or_conflicting_observations(change,reason):
    args=list(fixture())
    if change=='extra_instance':args[1].masks.append(np.zeros_like(args[1].masks[0]))
    if change=='uncertain_upper':args[2][0]['uncertainty']={'kind':'BOUNDED_CUBOID_UNION'}
    if change=='prior_upper':args[2][0]['quality']['all_dimensions_source']='SIZE_PRIOR'
    if change=='contradicting_lower':args[3][0]['evidence']['observed_face']['center_m'][2]+=.2
    if change=='no_interface':args[6][...,2]-=.1
    bodies,records=infer_supports(*args)
    assert not bodies and records[0]['status']=='NOT_APPLIED' and reason in records[0]['reason']


def test_support_relation_is_explicit_and_requires_field_basis():
    assert policy(None) is None and policy(dict(confirmed_stack_support=dict(mode='DISABLED'))) is None
    for val in ({'mode':'GUESS'},{'mode':'USER_CONFIRMED_TWO_CARTON_STACK'},{'mode':'USER_CONFIRMED_TWO_CARTON_STACK','basis':''}):
        with pytest.raises(ValueError):policy(dict(confirmed_stack_support=val))
    args=list(fixture());args[5]={};assert infer_supports(*args)==({},[])


def test_scene_associates_only_named_lower_returns_with_complete_support_body(monkeypatch):
    from unloading_sim.workbench.single_carton import site_scene
    obs,result,boxes,rejected,site,geometry,points,valid=fixture()
    shape=valid.shape
    obs.frame.rgb=np.zeros((*shape,3),np.uint8);obs.frame.depth=np.full(shape,500,np.uint16)
    obs.frame.depth_scale_m=.001;obs.frame.intrinsics=dict(width=shape[1],height=shape[0],
        fx=200,fy=200,ppx=40,ppy=30,coeffs=[0]*5,model='SYNTHETIC_PINHOLE')
    obs.world_camera=np.eye(4);obs.q_capture=np.zeros(6)
    result.observation_id=obs.frame.observation_id
    boxes[0]['faces']=[]
    site.update(voxel_m=.08,unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC',fixed_obstacles=[])
    # A small unlabelled object within the same domain must keep its own world
    # collider, even though the full possible support already encloses it.
    result.masks[1][45:48,35:38]=False
    monkeypatch.setattr(site_scene,'metric_points',lambda frame:points)
    scene=site_scene.build_scene(obs,result,boxes,site,unresolved_cartons=rejected,geometry=geometry)
    support=next(o for o in scene['world'] if o['category']=='confirmed_stack_support')
    # Existing observed-support handling is retained for independent remaining
    # returns near the bottom; it does not absorb the unlabelled object below.
    record=scene['confirmed_stack_support'][0]
    assert record['observed_instance_points_represented']==int(result.masks[1].sum())
    p=points[46,36]
    assert any(o['category']=='observed' and np.all(abs(transform(np.linalg.inv(o['pose']),p[None])[0])<=np.asarray(o['size'])/2)
               for o in scene['world'])
    assert support['supports_target_id']=='upper' and scene['observed_voxels']>0


def test_planner_binds_support_permission_only_to_loaded_detach_and_named_target(case):
    p,obs,box,folder,calls,opts=case
    p.site['geometry']=deepcopy(GEOMETRY);opts['capabilities']={'inclined_carton_support_v1':True}
    T=np.eye(4);T[:3,3]=[.4,0,.15]
    body=dict(id='actual-lower-support',category='confirmed_stack_support',supports_target_id=box['id'],
        support_plane=dict(normal_world=[0.,0.,1.],offset_m=.35),support_basis='SYNTHETIC',
        size=[.2,.2,.4],pose=T.tolist())
    obs.frame.observation_id='synthetic-stack-frame'
    scene=dict(world=[body],observation_id=obs.frame.observation_id,unresolved_carton_obstacles=[
        dict(detection_index=1,representation_complete=True,user_clearance_applied=False,
            scope='STATIC_OBSTACLE_INTERSECTION_WITH_ENFORCED_WORKSPACE_NOT_PAYLOAD')])
    mark_incomplete_scene(scene,GEOMETRY,[dict(detection_index=1)])
    assert not unmodeled_cartons(scene)
    plan=p.plan_cycle(obs,[box],folder,scene=scene)
    assert plan['full_path_validated']
    for r in (r for r in calls if r.get('op')=='plan'):
        pair=[box['id'],body['id']]
        assert (pair in r['allowed_pairs'])==(r['stage']=='detach')
        assert body in r['world']
        assert not any(body['id'] in pair and box['id'] not in pair for pair in r['allowed_pairs'])
    other=deepcopy(box);other['id']='other-target';assert not p.detach_supports([body],other)


def test_old_worker_cannot_silently_drop_inclined_support_checks(case):
    p,obs,box,folder,calls,opts=case;p.site['geometry']=deepcopy(GEOMETRY)
    with pytest.raises(ValueError,match='UPDATE_REQUIRED_FOR_INCLINED_CARTON_SUPPORT'):
        p.plan_cycle(obs,[box],folder)
    assert not p.authority._plans and not [r for r in calls if r.get('op')=='plan']


def test_old_worker_cannot_silently_force_measured_side_into_front_rectangle(case):
    p,obs,box,folder,calls,opts=case
    box.setdefault('quality',{})['observed_side_envelope_source']='ALL_ACCEPTED_SIDE_PLANE_SAMPLES_NO_SIZE_PRIOR'
    with pytest.raises(ValueError,match='UPDATE_REQUIRED_FOR_OBSERVED_SIDE_ENVELOPE'):
        p.plan_cycle(obs,[box],folder)
    assert not p.authority._plans and not [r for r in calls if r.get('op')=='plan']
