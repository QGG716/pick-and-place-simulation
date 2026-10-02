"""CPU conditioning/mapping tests; fake model below is not a real SAM validation."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from unloading_perception.working_resolution import WorkingGrid
from unloading_perception.prompt_audit import validate_annotation_masks
from unloading_perception.face_quality import support_quality,patch_minimum_width
from unloading_perception.metric_support import observation_support,backproject_pixels
from sam_image_session import sam_image_session


def test_decimation_ray_and_bbox_mapping_without_invented_edge_depth():
    grid=WorkingGrid(8,10,2);K=np.array([[100.,0,4.5],[0,100.,3.5],[0,0,1]])
    depth=np.ones((8,10));depth[:,5:]=3
    work=grid.sample(depth)
    assert set(np.unique(work))=={1,3}
    np.testing.assert_array_equal(grid.boxes([[0,0,10,8]]),[[0,0,5,4]])
    uv=np.array([[1,1],[3,2]])
    np.testing.assert_allclose(backproject_pixels(uv,[1,3],grid.intrinsics(K)),backproject_pixels(uv*2,[1,3],K),atol=0,rtol=0)
    assert grid.restore(work).shape==depth.shape
    assert grid.boxes([]).shape==(0,4)
    for bad in (0,3):
        with pytest.raises(ValueError):WorkingGrid(8,10,bad)
    with pytest.raises(ValueError):grid.boxes([[2,2,1,3]])


def test_oracle_box_uses_bound_rendered_mask_not_sam_box():
    mask=np.zeros((8,10),bool);mask[1:5,2:8]=1
    item=dict(simulation_object_id='object-a',mask_key='a',visible=True,bbox_xyxy=[2,1,8,5])
    assert validate_annotation_masks({'objects':[item]},{'a':mask},mask.shape)[0]['pixels']==24
    with pytest.raises(ValueError,match='BBOX'):validate_annotation_masks({'objects':[{**item,'bbox_xyxy':[2,1,9,5]}]},{'a':mask},mask.shape)
    with pytest.raises(ValueError,match='DUPLICATE'):validate_annotation_masks({'objects':[item,item]},{'a':mask},mask.shape)
    with pytest.raises(ValueError,match='SHAPE'):validate_annotation_masks({'objects':[item]},{'a':mask},(9,10))


def test_parent_support_excludes_invalid_jump_and_does_not_double_erode():
    depth=np.ones((30,40));mask=np.ones_like(depth,bool);depth[15,20]=np.nan;depth[:,30:]=2
    raw,parent,a=observation_support(depth,mask,mask)
    _,face,_=observation_support(depth,mask,parent,parent_support=parent)
    assert np.array_equal(parent,face)
    assert not parent[15,20] and not parent[:,29:31].any()
    assert face.sum()==a['retained_pixels']
    split=parent.copy();split[:,15:]=False
    _,left,_=observation_support(depth,mask,split,parent_support=parent)
    assert np.all(~left|parent) and left.sum()<split.sum()


def plane(w,h):
    x,y=np.meshgrid(np.linspace(0,w,30),np.linspace(0,h,20));return np.c_[x.ravel(),y.ravel(),np.ones(x.size)]


def test_real_small_plane_vs_edge_strip_and_degenerate_support():
    assert support_quality(plane(.04,.05))['geometry_eligible']
    narrow=support_quality(plane(.009,.35))
    assert not narrow['geometry_eligible'] and narrow['role']=='OBSERVED_EDGE_EVIDENCE'
    assert not support_quality(plane(0,.35))['geometry_eligible']
    p=np.array([[0,0,1],[.03,0,1],[.03,.2,1],[0,.2,1]])
    assert patch_minimum_width(p)==pytest.approx(.03)


def test_encoding_reused_only_within_same_image_request():
    calls={'preprocess':0,'encode':0,'decode':0}
    class Images:
        def __call__(self,image,**kw):calls['preprocess']+=1;return {'pixel_values':np.ones((2,2))}
    class Model:
        def get_image_embeddings(self,pixels):calls['encode']+=1;return pixels*2
        def __call__(self,**kw):calls['decode']+=1;return kw['image_embeddings'].sum()
    entry=SimpleNamespace(perturb_boxes=lambda b,w,h:[list(b)]*5);original=entry.perturb_boxes
    processor=SimpleNamespace(image_processor=Images());torch=SimpleNamespace(cuda=SimpleNamespace(synchronize=lambda:None))
    for frame in (1,2):
        with sam_image_session(entry,processor,Model(),torch,identity={'frame':frame}) as (p,m,report):
            image=object()
            for _ in range(3):assert m(**p.image_processor(image))==8
            assert report['image_encoder_calls']==1 and report['prompt_decoder_calls']==3
            with pytest.raises(ValueError,match='IMAGE_CHANGED'):p.image_processor(object())
        assert entry.perturb_boxes is original
    assert calls=={'preprocess':2,'encode':2,'decode':6}


def test_labels_are_subset_of_measurement_parent_and_narrow_face_keeps_evidence():
    pytest.importorskip('cv2')
    from unloading_perception.metric_faces import extract_observation_labels,fit_metric_faces,MetricFitConfig
    depth=np.ones((100,120));mask=np.ones_like(depth,bool);depth[20:30,40:50]=np.nan
    K=[1000,0,60,0,1000,50,0,0,1]
    extractor=lambda xyz,threshold:[dict(points=xyz,normal=[0,0,-1],equation=[0,0,-1,1])]
    labels,seeds,a=extract_observation_labels(depth,mask,K,extractor)
    parent=observation_support(depth,mask,mask)[1]
    assert not np.any((labels>=0)&~parent)
    record=fit_metric_faces(depth,mask,K,labels,seeds,mask_id=1)
    assert record['camera_facing_faces'] and record['support_policy']=='MEASUREMENT_PARENT_V2'
    labels[:]=-1;labels[30:70,50:58]=0
    record=fit_metric_faces(depth,mask,K,labels,seeds[:1],mask_id=1)
    assert not record['camera_facing_faces'] and record['face_quality']
    assert 'PHYSICAL_SHORT_SPAN_BELOW_GEOMETRY_MINIMUM' in record['face_quality'][0]['reasons']


def test_virtual_rig_translation_keeps_body_camera_relation_and_input_immutable():
    import json
    from unloading_perception.capture_rig_motion import translated_virtual_rig
    root=Path(__file__).resolve().parents[1]
    original=json.loads((root/'docs/validation/evidence/roof-mast-lit-20260916/manifest.json').read_text())
    frozen=json.dumps(original,sort_keys=True);moved=translated_virtual_rig(original,.18)
    a,b=[np.asarray(p['mechanisms']['vision_rig']['T_W_vision_flange']) for p in (original,moved)]
    for x,y in zip(original['cameras'],moved['cameras']):
        np.testing.assert_allclose(np.linalg.inv(a)@x['T_W_C'],np.linalg.inv(b)@y['T_W_C'],atol=1e-14)
        assert y['position_world_m'][1]==pytest.approx(x['T_W_C'][1][3]+.18)
    assert original['robot']==moved['robot'] and original['objects']==moved['objects']
    assert original['mechanisms']['vision_rig']['T_W_J1']==moved['mechanisms']['vision_rig']['T_W_J1']
    assert moved['mechanisms']['vision_rig']['parent_frame']=='world_virtual_capture_rig'
    assert frozen==json.dumps(original,sort_keys=True)
    with pytest.raises(ValueError):translated_virtual_rig(original,float('nan'))


def test_wrong_image_proposal_rejected_before_model_call(tmp_path):
    import json
    from PIL import Image
    from vision_resident_worker import ResidentRuntime
    image=tmp_path/'rgb.png';Image.fromarray(np.zeros((8,10,3),np.uint8)).save(image)
    proposal=tmp_path/'proposals.json';proposal.write_text(json.dumps(dict(rgb_sha256='0'*64,instances=[])))
    runtime=object.__new__(ResidentRuntime);runtime.args=SimpleNamespace(processing_divisor=1)
    with pytest.raises(ValueError,match='BINDING_MISMATCH'):
        runtime._sam(image,proposal,tmp_path/'overlay.png',tmp_path/'mask.npz',tmp_path/'instances.json',tmp_path/'log',{},[])


def test_new_canvas_scales_geometry_equally_and_separates_fusion_modes():
    # Structural guard, not a claim that a browser interaction test ran.
    html=(Path(__file__).resolve().parents[1]/'tools/perception_stage_viewer.html').read_text(encoding='utf-8')
    assert 'new ResizeObserver(draw)' in html and 'ctx.setTransform(dpr,0,0,dpr,0,0)' in html
    assert "if(fusionMode==='post')for(let obj of g.objects)" in html
    assert "if(fusionMode==='object')" in html and '选中对象无认证面' in html
    assert 'q[0]*span' not in html and 'H/2-w/size*H*.78*zoom' in html


def test_png_comparison_uses_equal_metric_axes():
    from render_perception_stage_video import metric_limits
    class Axes:
        def set(self,**kw):self.limits=kw
        def set_box_aspect(self,ratio):self.ratio=ratio
    ax=Axes();metric_limits(ax,[[0,0,0],[4,.1,.2]])
    spans=[hi-lo for lo,hi in ax.limits.values()]
    assert spans==pytest.approx([5.2]*3) and ax.ratio==(1,1,1)


def test_contact_gate_reports_valid_but_too_narrow_patch_without_shrinking_tool():
    from test_surface_contacts import fixture,run
    from unloading_perception.surface_contacts import ContactSupportInsufficient
    values=fixture(width=.015,height=.3)
    values[1]['support_policy']='MEASUREMENT_PARENT_V2'
    with pytest.raises(ContactSupportInsufficient,match='COMPLETE_CUP_DISK'):run(values)
    assert values[-1]['layout']['minimum_sealed_cups']==60


def test_two_real_depth_planes_remain_and_final_validation_checks_native_inputs():
    pytest.importorskip('cv2')
    from copy import deepcopy
    from unloading_perception.metric_faces import fit_metric_faces,validate_metric_record
    depth=np.full((100,180),2.,np.float32);mask=np.zeros(depth.shape,bool);mask[20:80,40:160]=True
    depth[:,100:]=200./np.arange(100,180)[None,:]
    labels=np.full(depth.shape,-1,np.int16);labels[20:80,40:100]=0;labels[20:80,100:160]=1
    seeds=[dict(normal=[0,0,-1],offset_m=2),dict(normal=[-1,0,0],offset_m=.2)]
    K=np.array([[1000.,0,0],[0,1000.,50],[0,0,1]])
    record=fit_metric_faces(depth,mask,K,labels,seeds,mask_id=1)
    assert len(record['camera_facing_faces'])==2
    assert all(r['geometry_eligible'] for r in record['face_quality'])
    for kind in ('depth','mask','K'):
        d,m,k=depth.copy(),mask.copy(),K.copy()
        if kind=='depth':d[45,75]+=.01
        if kind=='mask':m[45,75]=False
        if kind=='K':k[0,0]+=1
        with pytest.raises(ValueError,match='BINDING'):validate_metric_record(record,d,m,k)
    wrong=deepcopy(record)
    wrong['camera_facing_faces'][0]['corners_3d_m']=(np.array(wrong['camera_facing_faces'][0]['corners_3d_m'])+[0,0,.02]).tolist()
    assert len(validate_metric_record(wrong,depth,mask,K)['camera_facing_faces'])<2
