"""Named field artefacts never erase neighbours, supports or fixed geometry."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.geometry import corners
from unloading_sim.workbench.single_carton.pointcloud_artifacts import policy,load_manifest,exclude_upper_returns
from unloading_sim.workbench.single_carton.site_scene import build_scene,load_site,SUPPORT_ABOVE_M
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from test_single_carton_partial_site import envelope_scene_case,covered,ROOT


def carton(index,height):
    T=np.eye(4);T[2,3]=height;size=[.1]*3
    face=dict(observed=True,center_m=[0.,0.,height],normal=[1.,0.,0.],u=[0.,1.,0.],v=[0.,0.,1.],size_m=[.1,.1])
    measured=dict(pose=T.tolist(),size_m=size,corners_m=corners(T,size).tolist(),faces=[face])
    return dict(id=f'box-{index}',detection_index=index,**measured,measured_box=deepcopy(measured))


def obj(index,center,size):
    T=np.eye(4);T[:3,3]=center
    return dict(id=f'rgbd_observed_test_{index}',category='observed',pose=T.tolist(),size_m=size)


def save_manifest(path,data):
    path.write_text(json.dumps(data))
    return dict(upper_carton_pointcloud_artifacts=dict(mode='USER_CONFIRMED_NAMED_LIST',
        basis=data['basis'],manifest=str(path),manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))


def manifest(tmp_path,box=None,objects=None):
    b=box or carton(0,.4);m=b['measured_box']
    data=dict(schema='eco65_named_pointcloud_artifacts_v1',frame='base_link',basis='SYNTHETIC_FIELD_CONFIRMATION',
        evidence='USER_FIELD_CONFIRMATION_NOT_SENSOR_FREE_SPACE',reference_task_id='synthetic-task',
        reference_observation_id='synthetic-observation',reference_target=dict(id=b['id'],
            measured_pose=m['pose'],measured_size_m=m['size_m']),
        objects=objects or [obj(0,[0,0,.4],[.02,.02,.14]),obj(1,[.1,0,.4],[.02]*3),obj(2,[0,.1,.4],[.02]*3)])
    return save_manifest(tmp_path/'manifest.json',data)


def fixture():
    points=np.array([[[0.,0.,.4],[.10,0.,.4],[0.,.1,.4],[0.,0.,.352],[.112,0.,.4],[0.,0.,.2],[-.005,0,.4],[0,0,.402]]])
    eligible=np.ones(points.shape[:2],bool);eligible[0,7]=False
    masks=[np.zeros_like(eligible),np.zeros_like(eligible)];masks[1][0,2]=True
    support=np.zeros_like(eligible);support[0,6]=True
    return points,eligible,masks,[carton(0,.4),carton(1,.2)],support


def exclude(p,e,m,b,c,u=(),support=None):
    return exclude_upper_returns(p,e,m,b,u,c,support_above_m=SUPPORT_ABOVE_M,protected_support=support)


def test_only_original_listed_volumes_and_unclaimed_points_are_removed(tmp_path):
    p,e,m,b,s=fixture();before=deepcopy(b);config=manifest(tmp_path)
    removed,r=exclude(p,e,m,b,config,support=s)
    assert removed.tolist()==[[True,True,False,False,False,False,False,False]]
    assert r['ignored_points']==2 and r['detection_index']==0 and r['added_margin_m']==0.
    assert r['evidence']=='USER_FIELD_CONFIRMATION_NOT_SENSOR_FREE_SPACE' and len(r['ignored_mask_sha256'])==64
    assert sum(i['newly_ignored_points'] for i in r['items'])==2
    assert b==before and m[1][0,2] and not e[0,7]


def test_uncertainty_envelope_does_not_expand_ignored_region(tmp_path):
    p,e,m,b,s=fixture();config=manifest(tmp_path);b[0]['size_m']=[.8]*3
    removed,_=exclude(p,e,m,b,config,support=s)
    assert not removed[0,4] and not removed[0,5]


def test_list_stays_in_base_frame_and_does_not_follow_target(tmp_path):
    p,e,m,b,s=fixture();config=manifest(tmp_path)
    # Still associated with the same upper region, but the list never follows
    # this 30 mm target shift. An unlisted point remains, even near the target.
    b[0]['measured_box']['pose'][0][3]=.03
    p[0,4]=[.13,0.,.4]
    removed,r=exclude(p,e,m,b,config,support=s)
    assert r['status']=='APPLIED' and removed[0,1] and not removed[0,4]


def test_rotated_list_volume_uses_obb_not_its_larger_aabb(tmp_path):
    item=obj(0,[0,0,.4],[.08,.01,.02]);theta=np.pi/4
    R=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1.]])
    T=np.asarray(item['pose']);T[:3,:3]=R;item['pose']=T.tolist()
    config=manifest(tmp_path,objects=[item]);p=np.array([[[.02,.02,.4],[.02,-.02,.4]]]);e=np.ones((1,2),bool)
    removed,_=exclude(p,e,[np.zeros_like(e)],[carton(0,.4)],config)
    assert removed.tolist()==[[True,False]]


@pytest.mark.parametrize('kind',['higher_unresolved','no_measured_box','tie','no_faces','disabled','upper_moved','upper_removed'])
def test_missing_or_changed_upper_cannot_redirect_exception_to_lower(tmp_path,kind):
    p,e,m,b,s=fixture();unresolved=[];config=manifest(tmp_path)
    if kind=='higher_unresolved':unresolved=[dict(detection_index=2,evidence=dict(observed_face=dict(center_m=[0,0,.6])))]
    if kind=='no_measured_box':b[0].pop('measured_box')
    if kind=='tie':b[1]=carton(1,.4)
    if kind=='no_faces':b=[]
    if kind=='disabled':config={}
    if kind=='upper_moved':b[0]=carton(0,.8)
    if kind=='upper_removed':b=b[1:]
    removed,r=exclude(p,e,m,b,config,unresolved,s)
    assert not removed.any() and r['status']!='APPLIED'


@pytest.mark.parametrize('value',[None,'IGNORE',dict(mode='USER_CONFIRMED_IGNORE'),dict(mode='UNKNOWN'),
    dict(mode='USER_CONFIRMED_NAMED_LIST'),dict(mode='USER_CONFIRMED_NAMED_LIST',basis=' '),
    dict(mode='USER_CONFIRMED_NAMED_LIST',basis='present')])
def test_no_broad_ignore_or_missing_field_basis_allowed(value):
    with pytest.raises(ValueError):policy(dict(upper_carton_pointcloud_artifacts=value))


@pytest.mark.parametrize('change',['duplicate','support','fixed','shadow','bad_id','bad_pose','bad_size','empty','wrong_frame','wrong_basis'])
def test_manifest_rejects_non_pointcloud_and_invalid_list_geometry(tmp_path,change):
    config=manifest(tmp_path);path=Path(policy(config)['manifest']);d=json.loads(path.read_text())
    if change=='duplicate':d['objects'].append(d['objects'][0])
    elif change in ('support','fixed','shadow'):d['objects'][0]['category']=change
    elif change=='bad_id':d['objects'][0]['id']='neighbor_carton'
    elif change=='bad_pose':d['objects'][0]['pose'][0][0]=1.1
    elif change=='bad_size':d['objects'][0]['size_m'][0]=-1
    elif change=='empty':d['objects']=[]
    elif change=='wrong_frame':d['frame']='world'
    elif change=='wrong_basis':d['basis']='changed'
    modified=save_manifest(path,d)
    if change=='wrong_basis':modified['upper_carton_pointcloud_artifacts']['basis']='SYNTHETIC_FIELD_CONFIRMATION'
    with pytest.raises(ValueError):load_manifest(modified)


def test_modified_manifest_cannot_be_loaded_or_claim_an_old_plan(tmp_path):
    config=manifest(tmp_path);loaded=load_manifest(config)
    p=SinglePlanner.__new__(SinglePlanner);p.site={};p.files={}
    p.track(loaded['path'],loaded['sha256'])
    path=Path(loaded['path']);path.write_text(path.read_text()+'\n')
    with pytest.raises(ValueError,match='MANIFEST_HASH_CHANGED'):load_manifest(config)
    with pytest.raises(ValueError,match='INPUT_CHANGED_REBUILD_PLAN'):p.binding()


def test_config_tracks_manifest_before_planning_and_rejects_late_change(tmp_path):
    artifacts=manifest(tmp_path)
    site=json.loads((ROOT/'configs/workbench/real_site.example.json').read_text());site.update(artifacts)
    site_path=tmp_path/'site.json';site_path.write_text(json.dumps(site))
    c=json.loads((ROOT/'configs/workbench/single_carton.example.json').read_text());c['site_scene']=str(site_path)
    path=tmp_path/'config.json';path.write_text(json.dumps(c));loaded=load(path)
    item=load_manifest(site)
    assert loaded['loaded_file_sha256'][item['path']]==item['sha256']
    manifest_path=Path(item['path']);manifest_path.write_text(manifest_path.read_text()+'\n')
    planner=SinglePlanner(loaded,None,None,None)
    # Raises while checking loaded inputs, before building a model or starting IO.
    with pytest.raises(ValueError,match='ASSET_HASH_MISMATCH'):planner.build()


def test_scene_keeps_neighbor_fixed_bounds_supports_and_raw_frame(tmp_path):
    obs,result,b,site=envelope_scene_case();before=obs.frame.depth.copy()
    b['measured_box']={k:deepcopy(b[k]) for k in ('pose','size_m','corners_m','faces')}
    neighbor=np.zeros_like(result.masks[0]);neighbor[:,:10]=True
    result=replace(result,detections=(*result.detections,dict(label='box',score=.8,frame_id='camera_optical')),
        masks=(*result.masks,neighbor))
    unresolved=[dict(detection_index=1,evidence=dict(observed_face=dict(center_m=[0,0,.45])))]
    fixed=dict(id='measured_fixture',pose=np.eye(4).tolist(),size_m=[.01]*3,measurement_basis='SYNTHETIC')
    site['fixed_obstacles']=[fixed]
    kept=build_scene(obs,result,[b],site,unresolved_cartons=unresolved,neighbor_depth_m=.1)
    site.update(manifest(tmp_path,b,[obj(0,[0,0,.5],[.2,.2,.03])]))
    scene=build_scene(obs,result,[b],site,unresolved_cartons=unresolved,neighbor_depth_m=.1)
    report=scene['upper_carton_pointcloud_artifacts'];assert report['ignored_points']>0
    assert report['observation_id']==obs.frame.observation_id
    assert [o for o in scene['world'] if o['category']!='observed']==[o for o in kept['world'] if o['category']!='observed']
    assert any(x['id']=='measured_fixture' for x in scene['world'])
    assert len([x for x in scene['world'] if x['category']=='workspace_boundary'])==6
    assert scene['unresolved_neighbor_extrusion'][0]['user_clearance_applied'] is False
    from unloading_sim.workbench.detection import metric_points
    assert covered(scene['world'],metric_points(obs.frame)[neighbor])
    assert np.array_equal(obs.frame.depth,before)


def test_current_field_list_has_exact_named_extent_and_source_binding():
    c=load_site(ROOT/'configs/workbench/real_site.example.json');item=load_manifest(c)
    assert policy(c)['mode']=='USER_CONFIRMED_NAMED_LIST'
    assert item['data']['reference_task_id']=='d6d985f17e6f4b6b84eaffb71c5da66b'
    assert len(item['data']['objects'])==53 and '2026_11_19' in policy(c)['basis']
    assert set(item['data']['source_records'])=={'geometry.json','site-scene.json','planning-failures.json'}
    c['upper_carton_pointcloud_artifacts']={'mode':'KEEP'}
    assert policy(c)=={'mode':'KEEP'} and load_manifest(c) is None


def test_scene_preserves_existing_support_plane_noise_band(tmp_path):
    obs,result,b,site=envelope_scene_case()
    b['measured_box']={k:deepcopy(b[k]) for k in ('pose','size_m','corners_m','faces')}
    # These returns are above the nominal bottom +4 mm, but the existing
    # observed support plane's noise window still owns them. The artefact
    # exception must not erase that measured support or change its extents.
    b['quality']['observed_support_plane']=dict(top_world_m=.5,noise_m=.002)
    before=build_scene(obs,result,[b],site)
    site.update(manifest(tmp_path,b,[obj(0,[.03,0,.5],[.04,.06,.03])]))
    after=build_scene(obs,result,[b],site)
    supports=lambda s:[o for o in s['world'] if o['category']=='observed_support']
    assert supports(before) and supports(after)==supports(before)
    assert after['upper_carton_pointcloud_artifacts']['ignored_points']==0
