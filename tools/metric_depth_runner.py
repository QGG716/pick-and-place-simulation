"""Registered-depth strategy behind the existing RGB-D adapter boundary."""
import json
from pathlib import Path
import cv2
import numpy as np

from unloading_perception.metric_faces import extract_observation_labels, fit_metric_faces, MetricFitConfig
from unloading_perception.final_geometry import project


def run_metric_depth(*, raw, source, masks, pointmap, depth, K, metadata, output, vision_root, python=None, timeout=None):
    from diagnose_metric_calibration import load_extractor
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    with np.load(pointmap,allow_pickle=False) as archive:
        provenance=json.loads(str(archive['metadata_json']))
        if provenance.get('source') not in {'ISAAC_IDEAL_REGISTERED_DEPTH','REGISTERED_RGBD'}:
            raise ValueError('UNVERIFIED_METRIC_SOURCE')
        if (provenance['capture_id']!=metadata.capture_id or provenance['camera_frame']!=metadata.rgb_frame_id
            or provenance['calibration_identity']!=metadata.calibration_identity
            or not np.allclose(archive['K'],np.asarray(K).reshape(3,3),atol=1e-9,rtol=0)):
            raise ValueError('POINTMAP_CAPTURE_CALIBRATION_MISMATCH')
    extractor=load_extractor(vision_root)
    config=MetricFitConfig(positive_infinity_is_no_hit=provenance['source']=='ISAAC_IDEAL_REGISTERED_DEPTH')
    sam=json.loads((Path(masks).parent/'cargo_instances.json').read_text())
    entries=sam.get('instances',[])
    entries={int(v['id']):v for v in entries}
    records=[]
    with np.load(masks,allow_pickle=False) as archive:
        for identity,mask in zip(archive['mask_ids'],archive['masks']):
            labels,seeds,audit=extract_observation_labels(depth,mask,K,extractor,config)
            entry=entries.get(int(identity),{})
            ambiguous=bool(entry.get('supporting_proposal_ids',[]))
            record=fit_metric_faces(depth,mask,K,labels,seeds,mask_id=int(identity),config=config,source_ambiguous=ambiguous)
            record['observation_segmentation']=audit
            records.append(record)
    result={'instances':records,'pointmap_source':'REGISTERED_METRIC_DEPTH','strategy':'DEPTH_CONSTRAINED_METRIC_FACES_V1',
            'legacy_comparison':'tools/metric_v4_runner.py','source':str(source)}
    path=output/'validated_geometry.json'; path.write_text(json.dumps(result,indent=2))
    image=cv2.imread(str(source))
    # Render only the final independent result persisted above.
    for record in json.loads(path.read_text())['instances']:
        for face in record['camera_facing_faces']:
            polygon=project(face['corners_3d_m'],K).round().astype(np.int32)
            cv2.polylines(image,[polygon],True,(0,220,0),2)
    cv2.imwrite(str(output/'final_metric_faces_overlay.png'),image)
    return result
