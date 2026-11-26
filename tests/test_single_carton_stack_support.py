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
    ('second_lower_with_interface','AMBIGUOUS_STACK_LOWER'),('uncertain_upper','MEASURED_UPPER'),('prior_upper','MEASURED_UPPER'),
    ('contradicting_lower','CONFLICTS'),('no_interface','NO_SAME_FRAME|HEIGHT_ESTIMATES_DISAGREE')])
def test_stack_confirmation_never_binds_unrelated_or_conflicting_observations(change,reason):
    args=list(fixture())
    if change=='second_lower_with_interface':
        args[1].masks.append(np.roll(args[1].masks[1],2,axis=1))
        args[3].append(dict(detection_index=2,evidence=deepcopy(args[3][0]['evidence'])))
    if change=='uncertain_upper':args[2][0]['uncertainty']={'kind':'BOUNDED_CUBOID_UNION'}
    if change=='prior_upper':args[2][0]['quality']['all_dimensions_source']='SIZE_PRIOR'
    if change=='contradicting_lower':args[3][0]['evidence']['observed_face']['center_m'][2]+=.2
    if change=='no_interface':args[6][...,2]-=.1
    bodies,records=infer_supports(*args)
    # Shifting all depth also moves the lower visible top, which may now be
    # rejected as disagreeing height estimates before the interface check.
    assert not bodies and records[0]['status']=='NOT_APPLIED' and any(r in records[0]['reason'] for r in reason.split('|'))


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


def test_instance_outside_the_column_neither_blocks_nor_joins_the_stack():
    args=list(fixture());far=np.zeros_like(args[1].masks[0]);far[50:,:6]=True   # no interface with the upper
    args[1].masks.append(far);args[3].append(dict(detection_index=2,evidence=deepcopy(args[3][0]['evidence'])))
    bodies,records=infer_supports(*args)
    assert records[0]['status']=='APPLIED' and records[0]['column']['other_instances_not_in_column']==[2]


def test_interface_band_returns_under_the_upper_face_are_represented_by_the_support(monkeypatch):
    # User decision 2026-11-20: under the upper visible face is the lower. An
    # unlabelled return just below the plane, within the upper mask band, is
    # represented by the support body; a deeper unlabelled object is not.
    from unloading_sim.workbench.single_carton import site_scene
    obs,result,boxes,rejected,site,geometry,points,valid=fixture()
    shape=valid.shape
    obs.frame.rgb=np.zeros((*shape,3),np.uint8);obs.frame.depth=np.full(shape,500,np.uint16)
    obs.frame.depth_scale_m=.001;obs.frame.intrinsics=dict(width=shape[1],height=shape[0],
        fx=200,fy=200,ppx=40,ppy=30,coeffs=[0]*5,model='SYNTHETIC_PINHOLE')
    obs.world_camera=np.eye(4);obs.q_capture=np.zeros(6);result.observation_id=obs.frame.observation_id
    boxes[0]['faces']=[]
    site.update(voxel_m=.08,unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC',fixed_obstacles=[])
    result.masks[1][30:32,25:30]=False   # just under the plane, next to the upper mask
    result.masks[1][45:48,35:38]=False   # deep below: keeps its own collider
    monkeypatch.setattr(site_scene,'metric_points',lambda frame:points)
    scene=site_scene.build_scene(obs,result,boxes,site,unresolved_cartons=rejected,geometry=geometry)
    record=scene['confirmed_stack_support'][0]
    assert record['interface_band_points_represented']['count']>=10
    def emitted(p):
        return any(o['category']=='observed' and np.all(abs(transform(np.linalg.inv(o['pose']),p[None])[0])<=np.asarray(o['size'])/2)
                   for o in scene['world'])
    assert not emitted(points[30,27]) and emitted(points[46,36])


def test_lower_returns_under_the_upper_outside_the_body_become_support_pieces(monkeypatch):
    # The lower front is one fitted plane; its own returns just under the
    # plane but in front of it (directly under the upper) become support pieces
    # whose top lies exactly on the plane, not obstacles touching the payload.
    from unloading_sim.workbench.single_carton import site_scene
    obs,result,boxes,rejected,site,geometry,points,valid=fixture()
    shape=valid.shape;points=points.copy();points[30:33,30:50,0]=.392   # 12 mm in front of the lower plane
    obs.frame.rgb=np.zeros((*shape,3),np.uint8);obs.frame.depth=np.full(shape,500,np.uint16)
    obs.frame.depth_scale_m=.001;obs.frame.intrinsics=dict(width=shape[1],height=shape[0],
        fx=200,fy=200,ppx=40,ppy=30,coeffs=[0]*5,model='SYNTHETIC_PINHOLE')
    obs.world_camera=np.eye(4);obs.q_capture=np.zeros(6);result.observation_id=obs.frame.observation_id
    boxes[0]['faces']=[]
    site.update(voxel_m=.08,unknown_policy='USER_CONFIRMED_CLEAR',unknown_clearance_basis='SYNTHETIC',fixed_obstacles=[])
    monkeypatch.setattr(site_scene,'metric_points',lambda frame:points)
    scene=site_scene.build_scene(obs,result,boxes,site,unresolved_cartons=rejected,geometry=geometry)
    record=scene['confirmed_stack_support'][0]
    pieces=[o for o in scene['world'] if o.get('interface_piece_of')]
    assert record['interface_support_pieces']['count']==len(pieces)>0
    n=np.asarray(record['plane']['normal_world']);d=record['plane']['offset_m']
    for o in pieces:
        assert o['category']=='confirmed_stack_support' and o['supports_target_id']=='upper'
        assert np.max(corners(o['pose'],o['size'])@n)==pytest.approx(d,abs=1e-9)
        assert o['support_plane']==record['plane']
    for p in points[30:33,30:50].reshape(-1,3):
        if p@n<=d:
            assert any(np.all(abs(transform(np.linalg.inv(o['pose']),p[None])[0])<=np.asarray(o['size'])/2+1e-9) for o in pieces)


def test_object_beside_the_upper_footprint_is_not_a_second_lower():
    # Field 9b51a37e: a black object beside the upper's corner, at the height
    # of its bottom, made the lower "ambiguous" under a 60 mm footprint.
    obs,result,boxes,rejected,site,geometry,points,valid=fixture()
    a,b=result.masks;beside=np.zeros_like(a);beside[26:34,62:72]=True
    a=a&~beside;b=b&~beside
    result.masks=[a,b,beside]
    face=rejected[0]['evidence']['observed_face']
    rejected=rejected+[dict(detection_index=2,evidence=dict(observed_face=face))]
    bodies,records=infer_supports(obs,result,boxes,rejected,site,geometry,points,valid|beside)
    assert records[0]['status']=='APPLIED' and records[0]['lower_index']==1
    assert records[0]['column']['interface_points_by_instance'][2]==0


def test_height_spread_limit_is_default_15_mm_and_enlargeable_only_with_basis():
    """User decision 2026-11-23 ("放宽判据"): the lower visible top seen under
    an overhanging upper may lie up to max_height_spread_m (<= 30 mm) below."""
    def overhang():
        args=list(fixture());pts=args[6].copy()
        pts[30:35,:,0]=.39   # top rows of the lower: interface returns off its face plane
        args[6]=pts;return args
    bodies,records=infer_supports(*overhang())
    assert not bodies and records[0]['reason']=='STACK_SUPPORT_HEIGHT_ESTIMATES_DISAGREE'
    assert records[0]['spread_m']>.015 and records[0]['spread_limit_m']==.015
    args=overhang();args[5]['confirmed_stack_support'].update(max_height_spread_m=.025,max_height_spread_basis='SYNTHETIC_TEST')
    bodies,records=infer_supports(*args)
    assert bodies and records[0]['status']=='APPLIED'
    assert records[0]['spread_limit_m']==.025 and records[0]['spread_limit_basis']=='SYNTHETIC_TEST'
    base=dict(mode='USER_CONFIRMED_TWO_CARTON_STACK',basis='SYNTHETIC')
    for extra in ({'max_height_spread_m':.025},{'max_height_spread_m':.025,'max_height_spread_basis':' '},
                  {'max_height_spread_m':.01,'max_height_spread_basis':'X'},{'max_height_spread_m':.031,'max_height_spread_basis':'X'},
                  {'max_height_spread_m':True,'max_height_spread_basis':'X'}):
        with pytest.raises(ValueError):policy(dict(confirmed_stack_support=dict(base,**extra)))
    assert policy(dict(confirmed_stack_support=dict(base,max_height_spread_m=.015,max_height_spread_basis='X')))['max_height_spread_m']==.015


def test_shared_lower_body_of_another_upper_stays_on_that_uppers_side():
    """User decision 2026-11-26: one lower under two measured uppers."""
    from unloading_sim.workbench.single_carton.stack_support import shared_lower_world
    from unloading_sim.workbench.single_carton.geometry import corners
    import numpy as np

    def upper(ident, y):
        # Measured frame as in the field: axis 2 = front normal (world +x), axis 1 = vertical, axis 0 = world y.
        T = np.eye(4); T[:3, :3] = [[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]; T[:3, 3] = [-.5, y, .35]
        size = [.13, .1, .2]
        return dict(id=ident, detection_index=0 if ident == 'a' else 1, pose=T.tolist(), size_m=size, corners_m=corners(T, size).tolist())

    def body(for_id, top):
        T = np.eye(4); T[:3, 3] = [-.5, 0., top-.15]
        return dict(id=f'support_2_for_{for_id}', category='confirmed_stack_support', pose=T.tolist(), size=[.6, 1., .3],
                    supports_target_id=for_id, detection_index=2,
                    support_plane=dict(normal_world=[0., 0., 1.], offset_m=top))
    a, b = upper('a', .07), upper('b', -.07)
    world = [body('a', .30), body('b', .31), dict(id='other', category='observed', pose=np.eye(4).tolist(), size=[.01]*3)]
    # Touching: A's contact plane facing B is A's trimmed envelope side, y = 0.005 (normal +y into A).
    planes = [dict(target_id='a', axis=0, side=-1, normal_world=[0., 1., 0.], offset_m=.005)]
    out, records = shared_lower_world(world, a, [a, b], planes)
    assert world[1]['id'] == 'support_2_for_b' and 'size' in world[1]                       # input untouched
    assert [o['id'] for o in out][:1] == ['support_2_for_a'] and out[0] is world[0]          # own body unchanged
    clipped = next(o for o in out if o.get('shared_lower_clip'))
    v = np.asarray(clipped['convex_mesh']['vertices'])
    assert v[:, 1].max() == pytest.approx(.005, abs=1e-9) and v[:, 2].max() == pytest.approx(.31, abs=1e-9)
    assert records[0]['boundary'] == 'ADJACENT_CONTACT_PLANE' and records[0]['result'] == 'ADJACENT_CONTACT_ON_OTHER_UPPER_SIDE'
    assert clipped['category'] == 'adjacent_contact' and clipped['supports_target_id'] == 'a' and clipped['original_id'] == 'support_2_for_b'
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from types import SimpleNamespace
    a_envelope = dict(a, corners_m=corners(a['pose'], a['size_m']).tolist())
    kinds = {s['kind'] for s in SinglePlanner.detach_supports(SimpleNamespace(object_top=SinglePlanner.object_top), out, a_envelope)
             if 'kind' in s}
    assert kinds == {'ADJACENT_CONTACT'}
    assert out[-1]['id'] == 'other'
    # Not touching: B's body is cut at A's envelope side facing B (y = 0.07 - 0.065 = 0.005).
    out, records = shared_lower_world(world, a, [a, b], [])
    v = np.asarray(next(o for o in out if o.get('shared_lower_clip'))['convex_mesh']['vertices'])
    assert v[:, 1].max() == pytest.approx(.005, abs=1e-9) and records[0]['boundary'] == 'TARGET_ENVELOPE_LATERAL_SIDE'
    # Symmetric for B; and an upper with no body of its own on that lower changes nothing.
    out, records = shared_lower_world(world, b, [a, b], [])
    assert records[0]['bound_to'] == 'a' and np.asarray(next(o for o in out if o.get('shared_lower_clip'))['convex_mesh']['vertices'])[:, 1].min() == pytest.approx(-.005, abs=1e-9)
    assert shared_lower_world(world[1:], a, [a, b], planes) == (world[1:], [])
