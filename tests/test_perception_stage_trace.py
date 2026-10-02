"""Synthetic CPU upstream stub; production extraction/fit with and without observation."""
import json
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))


def test_real_metric_runner_trace_does_not_change_records(tmp_path, monkeypatch):
    cv2=pytest.importorskip('cv2', reason='real OpenCV required; executed in existing GPU vision environment')
    import metric_depth_runner as runner
    # A prior optional-dependency test may have first imported this module with
    # a display substitute. This test must exercise actual OpenCV regardless of order.
    monkeypatch.setattr(runner,'cv2',cv2)
    import diagnose_metric_calibration
    from test_rgbd_pipeline import _metadata
    from unloading_perception.rgbd import register_rgbd, masked_metric_pointmap, PointCloudFilterConfig
    from unloading_contracts import canonical_fingerprint
    K=(100.,0,39.5,0,100.,39.5,0,0,1.)
    depth=np.full((80,80),2.,np.float32); mask=np.ones(depth.shape,bool)
    rgb=np.zeros((*depth.shape,3),np.uint8)
    metadata=_metadata()
    frame=register_rgbd(metadata=metadata,rgb=rgb,depth_optical_z_m=depth,rgb_K=K,depth_K=K,
        T_rgb_depth=np.eye(4),rgb_calibration_identity='calib-a',depth_calibration_identity='calib-a',
        registration_mode='SIMULATION_IDEAL_REGISTERED_DEPTH')
    pointmap=tmp_path/'pointmap.npz'
    masked_metric_pointmap(frame,mask,depth_identity='synthetic',config=PointCloudFilterConfig()).write_npz(pointmap)
    masks=tmp_path/'cargo_masks.npz';np.savez(masks,masks=[mask],mask_ids=[1])
    (tmp_path/'cargo_instances.json').write_text('{"instances":[{"instance_id":1}]}')
    source=tmp_path/'source.png';runner.cv2.imwrite(str(source),rgb)
    monkeypatch.setattr(diagnose_metric_calibration,'load_extractor',lambda _: lambda points,threshold:
        [dict(points=points,normal=np.array([0.,0.,-1.]),equation=np.array([0.,0.,-1.,2.]))])
    kwargs=dict(source=source,masks=masks,pointmap=pointmap,depth=depth,K=K,metadata=metadata,vision_root=tmp_path,rgb=rgb)
    a=runner.run_metric_depth(**kwargs,output=tmp_path/'off')
    b=runner.run_metric_depth(**kwargs,output=tmp_path/'on',trace=True)
    assert canonical_fingerprint(a)==canonical_fingerprint(b)
    assert b['instances'][0]['camera_facing_faces']
    assert not (tmp_path/'off/stage_trace').exists()
    trace=tmp_path/'on/stage_trace'
    index=json.loads((trace/'index.json').read_text())
    assert index['status']=='COMPLETED' and index['mask_ids']==[1]
    assert [e['stage'] for e in index['events']]==['initial_planes','fitted_and_independently_validated']
    initial=json.loads((trace/'mask-1-initial.json').read_text())
    assert initial['planes']==b['instances'][0]['observation_segmentation']['initial_planes']
    assert (np.load(trace/'mask-1-initial.npz')['labels']>=0).any()
    with pytest.raises(FileExistsError):runner.run_metric_depth(**kwargs,output=tmp_path/'on',trace=True)


def test_one_shot_trace_is_explicit_and_forwarded(tmp_path, monkeypatch):
    from test_workcell_perception_once import prepare
    capture,modules,_,entry,argv=prepare(tmp_path,monkeypatch)
    module=sys.modules['run_isaac_rgbd_geometry'];original=module._run_secondary_module;seen=[]
    def checked(**kwargs):
        seen.append(kwargs.pop('stage_trace',False))
        return original(**kwargs)
    monkeypatch.setattr(module,'_run_secondary_module',checked)
    assert entry(argv+['--stage-trace'])==0
    assert seen==[True,True]
