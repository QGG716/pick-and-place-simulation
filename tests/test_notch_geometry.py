"""CPU evidence gates. Synthetic depths test math, not detector/model accuracy."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from unloading_perception import metric_faces as mf

ROOT=Path(__file__).resolve().parents[1]


def ray_box(faces=3,partial=False):
    h,w=240,320;K=np.array([[280.,0,159.5],[0,280.,119.5],[0,0,1.]])
    yy,xx=np.indices((h,w));rays=np.stack(((xx-159.5)/280,(yy-119.5)/280,np.ones((h,w))),-1)
    ax,ay=np.radians([40 if faces==3 else 0,30]);cx,sx=np.cos(ax),np.sin(ax);cy,sy=np.cos(ay),np.sin(ay)
    axes=(np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])).T
    center=np.array([0.,0.,2.]);dimensions=np.array([.6,.5,.4])
    origin=-center@axes.T;direction=rays@axes.T
    with np.errstate(divide='ignore'):
        t1=(-dimensions/2-origin)/direction;t2=(dimensions/2-origin)/direction
    enter=np.minimum(t1,t2);leave=np.maximum(t1,t2)
    z=enter.max(-1);mask=(z<leave.min(-1))&(z>0)
    depth=np.where(mask,z,4.);labels=np.full((h,w),-1,np.int16);seeds=[]
    hit=np.argmax(enter,-1)
    for axis in np.unique(hit[mask]):
        region=mask&(hit==axis);label=len(seeds);labels[region]=label
        n=axes[axis];point=rays[region][0]*depth[region][0];offset=-n@point
        seeds.append(dict(normal=n.tolist(),offset_m=float(offset),initial_plane_index=int(axis),component=1))
    if partial:
        # Remove silhouette evidence with a nearer opaque object. Interior planes
        # remain, but their complete dimensions are not constrained.
        outside=~mask;depth[outside]=1.
    return depth,mask,K,labels,seeds


@pytest.mark.parametrize('faces',[2,3])
def test_measured_complete_two_and_three_faces(faces):
    pytest.importorskip('cv2')
    d,m,k,l,s=ray_box(faces)
    out=mf.fit_metric_faces(d,m,k,l,s,mask_id=17)
    assert out['accepted'],out.get('completion_diagnostics')
    assert out['reliable_plane_count']==faces
    assert out['cuboid_status']=='ACCEPTED_INDEPENDENTLY_VALIDATED'


@pytest.mark.parametrize('faces',[2,3])
def test_occluded_boundaries_do_not_invent_complete_volume(faces):
    pytest.importorskip('cv2')
    d,m,k,l,s=ray_box(faces,partial=True)
    out=mf.fit_metric_faces(d,m,k,l,s,mask_id=17)
    assert not out['accepted'] and out['camera_facing_faces']
    assert not any(key in out for key in mf._COMPLETE_FIELDS)


def test_single_plane_and_coplanar_components_never_call_complete(monkeypatch):
    pytest.importorskip('cv2')
    def forbidden(*a,**k):raise AssertionError('single plane entered cuboid solver')
    monkeypatch.setattr(mf,'_complete',forbidden)
    d=np.full((140,180),2.);m=np.ones_like(d,bool);k=np.array([[180.,0,89.5],[0,180.,69.5],[0,0,1]])
    labels=np.zeros_like(d,np.int16);labels[:,90:]=1
    seeds=[dict(normal=[0,0,-1],offset_m=2.,initial_plane_index=0,component=i) for i in (1,2)]
    out=mf.fit_metric_faces(d,m,k,labels,seeds,mask_id=3)
    assert len(out['camera_facing_faces'])==2
    assert out['reliable_plane_count']==1
    assert out['cuboid_status']=='NOT_ATTEMPTED_SINGLE_FACE' and not out['cuboid_solver_attempted']
    assert not any(k in out for k in mf._COMPLETE_FIELDS)
    from unloading_perception.observed_faces import observed_faces_from_geometry_record
    result=observed_faces_from_geometry_record(out,source_instance_id='prediction',module_id='m',capture_id='c',capture_time=1.,frame_id='optical')
    assert len(result.faces)==2 and result.complete_cuboid_status=='INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR'


def test_source_ambiguity_blocks_completion(monkeypatch):
    pytest.importorskip('cv2')
    monkeypatch.setattr(mf,'_complete',lambda *a,**k:pytest.fail('ambiguous source entered solver'))
    d,m,k,l,s=ray_box()
    out=mf.fit_metric_faces(d,m,k,l,s,mask_id=7,source_ambiguous=True)
    assert out['source_ambiguous'] and out['cuboid_status']=='BLOCKED_SOURCE_AMBIGUITY'
    assert not out['accepted'] and out['camera_facing_faces']


def test_shared_intersection_requires_actual_two_sided_support():
    d,m,k,l,s=ray_box(3)
    # Use the actual ray-box intersection of two frozen fitted planes.
    first,second=0,2
    n1,n2=np.array(s[first]['normal']),np.array(s[second]['normal'])
    direction=np.cross(n1,n2);direction/=np.linalg.norm(direction)
    origin=np.linalg.lstsq(np.array([n1,n2]),-np.array([s[first]['offset_m'],s[second]['offset_m']]),rcond=None)[0]
    p=mf._points(d,l==first,k)[0];interval=np.quantile(p@direction,[.2,.8])
    edge=origin+interval[:,None]*direction
    groups=[dict(label=i,region=l==i,normal=np.array(s[i]['normal']),offset=s[i]['offset_m']) for i in (first,second)]
    actual=mf._shared_edge_support(edge,*groups,d,k)
    assert actual['supported']
    groups[1]['region']=np.zeros_like(m)
    absent=mf._shared_edge_support(edge,*groups,d,k)
    assert not absent['supported'] and absent['supported_samples']==0


def test_continuous_measured_common_edge_needs_no_depth_jump():
    k=np.array([[4000.,0,100.],[0,4000.,100.],[0,0,1.]])
    yy,xx=np.indices((201,201));depth=2/(1-abs((xx-100)/4000))
    n1=np.array([-1.,0,-1])/np.sqrt(2);n2=np.array([1.,0,-1])/np.sqrt(2)
    polygon=np.array([[15.,10.],[100.,10.],[100.,190.],[15.,190.]])
    old=mf._boundary_kinds(polygon,n1,np.sqrt(2),depth,k)[1]
    assert old['kind']=='UNCLASSIFIED_BOUNDARY'
    edge=np.array([[0,-.045,2.],[0,.045,2.]])
    groups=[dict(label=0,region=xx<100,normal=n1,offset=np.sqrt(2)),
            dict(label=1,region=xx>100,normal=n2,offset=np.sqrt(2))]
    evidence=mf._shared_edge_support(edge,*groups,depth,k)
    assert evidence['supported'] and evidence['supported_samples']==15


def test_final_downgrade_removes_stale_complete_fields():
    from test_metric_patch_adapter import fixture
    record,d,m,k=fixture();bad=deepcopy(record['camera_facing_faces'][0]);bad['support_label']=1
    bad['corners_3d_m']=(np.array(bad['corners_3d_m'])+[0,0,.03]).tolist()
    record['camera_facing_faces'].append(bad);record['frozen_support_regions']['1']=record['frozen_support_regions']['0']
    record.update(accepted=True,metric_solver={'success':True},corners_3d=[[1,2,3]]*8,
        corners_2d=[[1,2]]*8,center_3d=[1,2,3],shape_dimensions=[1,1,1],orthogonal_axes_3d=np.eye(3).tolist())
    out=mf.validate_metric_record(record,d,m,k)
    assert out['cuboid_status']=='REVOKED_SINGLE_FACE' and not out['accepted']
    assert out['rejected_cuboid_candidate']['status']=='REJECTED_DIAGNOSTIC_ONLY'
    assert not any(key in out for key in mf._COMPLETE_FIELDS)


def test_exclusion_binds_recording_not_bare_frame_number():
    from unloading_perception.review_selection import validate_review_capture
    policy=json.loads((ROOT/'configs/perception/notch_geometry_review.json').read_text())
    bad=dict(sensor_epoch='carton-assets-recording-01',frame_sequence=602,capture_id='carton-assets-recording-01:module_0_upper:602')
    with pytest.raises(ValueError,match='EXCLUDED_CAPTURE'):validate_review_capture(bad,'module_0_upper',policy)
    assert validate_review_capture({**bad,'sensor_epoch':'new-recording'},'module_0_upper',policy)
    assert [s['name'] for s in policy['samples']]==['frame32','stack_notch']


def test_notch_removes_render_and_collision_entities_and_preserves_survivors():
    from unloading_perception.effective_scene import derive_effective_scene,derive_notch_scene
    from unloading_perception.isaac_validation import load_validation_config
    config=load_validation_config(ROOT/'configs/isaac/perception_validation.yaml')
    read=lambda name:json.loads((ROOT/name).read_text(encoding='utf-8'))
    s,c=derive_effective_scene(read(config['layout_bundle']['snapshot']),read(config['layout_bundle']['isaac_contract']),config,ROOT)
    frozen=deepcopy((s,c));out,contract=derive_notch_scene(s,c,read('configs/isaac/stack_notch_multiface.json'))
    assert (s,c)==frozen and len(out['cartons'])==29
    removed={r['name'] for r in out['derivation']['stack_notch']['removed_objects']}
    assert len(removed)==11 and not removed&{p['name'] for p in contract['primitives']}
    assert out['cartons']==[r for r in s['cartons'] if r['name'] not in removed]


def test_geometry_stop_does_not_publish_contacts_or_planning(tmp_path,monkeypatch):
    from test_workcell_perception_once import prepare,report
    capture,modules,calls,main,argv=prepare(tmp_path,monkeypatch)
    assert main(argv+['--stop-after','geometry'])==0
    summary=report(capture)
    assert 'algorithm_artifact' not in summary
    assert summary['handoff_not_run_reason']=='STOP_AFTER_GEOMETRY'
    assert set(summary['stages'].values())=={'NOT_RUN'}
    assert all(r['planning_admissible'] is False for r in summary['runs'])


def test_review_is_metric_surface_only_without_completed_box():
    page=(ROOT/'tools/notch_geometry_review.html').read_text(encoding='utf-8')
    assert 'const extent=Math.max(' in page and 'scale=Math.min(c.width,c.height)' in page
    assert "r.record.accepted&&r.record.cuboid_status==='ACCEPTED_INDEPENDENTLY_VALIDATED'" in page
    assert '<input type="checkbox" id="gt">' in page
    assert 'frame602' not in page
