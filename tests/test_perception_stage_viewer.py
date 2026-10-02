import hashlib
import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from build_perception_stage_viewer import validate_trace


def make_trace(tmp_path):
    from types import SimpleNamespace
    import numpy as np
    from perception_stage_trace import StageTrace
    from unloading_perception.metric_faces import MetricFitConfig
    source=tmp_path/'rgb';source.write_bytes(b'synthetic')
    masks=tmp_path/'masks';masks.write_bytes(b'synthetic')
    trace=StageTrace(tmp_path/'trace',source=source,masks=masks,
        metadata=SimpleNamespace(capture_id='c',rgb_frame_id='camera',calibration_identity='K'),config=MetricFitConfig())
    trace.extracted(1,np.zeros((4,4),np.int16),[],{},.1)
    trace.fitted(1,{'mask_id':1,'camera_facing_faces':[],'accepted':False},.2)
    trace.complete([1])
    return trace.directory


def test_valid_empty_algorithm_result_is_complete_not_crash(tmp_path):
    root=make_trace(tmp_path)
    assert validate_trace(root,[1],'c',True)['status']=='COMPLETED'
    assert validate_trace(tmp_path/'legal-empty',[],'c',True)['status']=='NOT_RUN_EMPTY_SEGMENTATION'


@pytest.mark.parametrize('fault',['missing','wrong_capture','wrong_instance','missing_event','hash','labels_hash','late_duplicate'])
def test_missing_or_mixed_new_stage_data_fails_closed(tmp_path,fault):
    root=make_trace(tmp_path);index=root/'index.json';doc=json.loads(index.read_text())
    if fault=='missing':index.unlink()
    elif fault=='wrong_capture':doc['capture_id']='other'
    elif fault=='wrong_instance':doc['mask_ids']=[99]
    elif fault=='missing_event':doc['events'].pop()
    elif fault=='hash':(root/'mask-1-fitted.json').write_text('{}')
    elif fault=='labels_hash':(root/'mask-1-initial.npz').write_bytes(b'changed')
    else:doc['events'].append(doc['events'][0])
    if fault!='missing':index.write_text(json.dumps(doc))
    with pytest.raises(ValueError):validate_trace(root,[1],'c',True)


def test_historical_missing_initial_is_explicit(tmp_path):
    assert validate_trace(tmp_path,[1],'capture',False) is None


@pytest.mark.parametrize('keyword',['rgb_sha256','masks_sha256','calibration'])
def test_trace_input_hashes_and_calibration_bound_to_verified_artifact(tmp_path,keyword):
    root=make_trace(tmp_path)
    with pytest.raises(ValueError,match='INPUT_BINDING'):
        validate_trace(root,[1],'c',True,**{keyword:'wrong'})


def test_viewer_has_no_network_dependency_or_inference_action():
    page=(Path(__file__).resolve().parents[1]/'tools/perception_stage_viewer.html').read_text(encoding='utf-8')
    assert '<script src="data.js">' in page
    assert 'http://' not in page and 'https://' not in page and 'fetch(' not in page
    assert 'NOT_EVALUATED' in page and 'raw_image_automatic=false' not in page  # policy comes from bound data
    assert 'function projection' in page and 'CUP_LIP_OUTSIDE_OBSERVED_PATCH' in page
