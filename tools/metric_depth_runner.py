"""Registered-depth strategy behind the existing RGB-D adapter boundary."""
import json
from pathlib import Path
from time import perf_counter
import cv2
import numpy as np

from unloading_perception.metric_faces import extract_observation_labels, fit_metric_faces, MetricFitConfig
from unloading_perception.final_geometry import project


def run_metric_depth(*, source, masks, pointmap, depth, K, metadata, output, vision_root, raw=None, python=None, timeout=None, rgb=None, trace=False, processing_divisor=1, quality_config=None):
    """Independent metric faces; raw is ignored for historical caller compatibility.

    The pinned extractor is imported and SHA-checked on every invocation. No
    legacy cuboid output, image or subprocess initialization is required.
    """
    from diagnose_metric_calibration import load_extractor
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    from contextlib import nullcontext
    context=(nullcontext(json.loads(Path(pointmap).read_text(encoding='utf-8'))) if Path(pointmap).suffix=='.json'
             else np.load(pointmap,allow_pickle=False))
    with context as archive:
        provenance=json.loads(str(archive['metadata_json']))
        if provenance.get('source') not in {'ISAAC_IDEAL_REGISTERED_DEPTH','REGISTERED_RGBD'}:
            raise ValueError('UNVERIFIED_METRIC_SOURCE')
        if (provenance['capture_id']!=metadata.capture_id or provenance['camera_frame']!=metadata.rgb_frame_id
            or provenance['calibration_identity']!=metadata.calibration_identity
            or not np.allclose(archive['K'],np.asarray(K).reshape(3,3),atol=1e-9,rtol=0)):
            raise ValueError('POINTMAP_CAPTURE_CALIBRATION_MISMATCH')
    extractor=load_extractor(vision_root)
    config=MetricFitConfig(positive_infinity_is_no_hit=provenance['source']=='ISAAC_IDEAL_REGISTERED_DEPTH',**(quality_config or {}))
    from unloading_perception.metric_support import observation_support
    from unloading_perception.working_resolution import WorkingGrid
    from dataclasses import replace
    grid=WorkingGrid(*depth.shape,processing_divisor)
    primary=Path(pointmap).suffix=='.json'
    support_audits=[];phase_rows=[]
    sam=json.loads((Path(masks).parent/'cargo_instances.json').read_text(encoding='utf-8'))
    entries=sam.get('instances',[])
    entries={int(v['instance_id']):v for v in entries}
    records=[]
    observer = None
    if trace:
        from perception_stage_trace import StageTrace
        observer = StageTrace(output/'stage_trace', source=source, masks=masks, metadata=metadata, config=config)
    with np.load(masks,allow_pickle=False) as archive:
        for identity,mask in zip(archive['mask_ids'],archive['masks']):
            started = perf_counter()
            phases={'mask_id':int(identity)};phase_rows.append(phases)
            if primary:
                support_start=perf_counter()
                measurement=observation_support(depth,mask,mask,erosion_px=config.erosion_px)
                phases['measurement_parent_seconds']=perf_counter()-support_start
                raw_mask,parent,parent_audit=measurement
                if processing_divisor==1:
                    labels,seeds,audit=extract_observation_labels(depth,mask,K,extractor,config,measurement=measurement)
                else:
                    working_config=replace(config,erosion_px=max(1,config.erosion_px//processing_divisor),
                        local_normal_radius_px=int(np.ceil(config.local_normal_radius_px/processing_divisor)),
                        minimum_points=max(3,int(np.ceil(config.minimum_points/processing_divisor**2))))
                    # Exact sampled native parent: no global percentile or depth interpolation.
                    working_depth,working_mask=grid.sample(depth),grid.sample(mask)
                    working_parent=grid.sample(parent)
                    labels,seeds,audit=extract_observation_labels(working_depth,working_mask,grid.intrinsics(K),extractor,working_config,
                        measurement=(grid.sample(raw_mask),working_parent,dict(parent_audit)))
                    labels=grid.restore(labels);labels[~parent]=-1
                    audit.update(working_grid=grid.to_dict(),working_config=__import__('dataclasses').asdict(working_config),
                        labelled_pixels=int((labels>=0).sum()),unassigned_pixels=int((parent&(labels<0)).sum()),
                        final_fit_and_validation='NATIVE_DEPTH_K_AND_FROZEN_RESTORED_LABELS')
                phases['parent_and_initial_extraction_seconds']=perf_counter()-started
                audit_start=perf_counter()
                reasons=np.ones(depth.shape,np.uint8);reasons[mask]=2;reasons[raw_mask]=3
                # Boundary and jumps are disjoint counts from the same measurement function.
                from unloading_perception.rgbd import _erode_mask
                interior=_erode_mask(mask,config.erosion_px)
                reasons[raw_mask&interior]=4;reasons[parent]=0
                audit_path=output.parent/'pointcloud_filter'/f'mask_{int(identity):04d}.npz'
                audit_path.parent.mkdir(exist_ok=True)
                np.savez_compressed(audit_path,raw_valid_mask=raw_mask,filtered_mask=parent,rejected_reason=reasons)
                phases['support_audit_write_seconds']=perf_counter()-audit_start
                support_audits.append(dict(mask_id=int(identity),status='MEASUREMENT_PARENT',audit_npz=str(audit_path),
                    support_policy='MEASUREMENT_PARENT_V2',dense_xyz='NOT_CONSTRUCTED_NO_PRIMARY_CONSUMER',
                    **parent_audit,seconds_including_extract_and_write=perf_counter()-support_start))
            else:
                labels,seeds,audit=extract_observation_labels(depth,mask,K,extractor,config)
            extracted_seconds = perf_counter()-started
            if observer:
                observer.extracted(int(identity), labels, seeds, audit, extracted_seconds)
            entry=entries.get(int(identity),{})
            ambiguous=bool(entry.get('supporting_proposal_ids',[]))
            started = perf_counter()
            record=fit_metric_faces(depth,mask,K,labels,seeds,mask_id=int(identity),config=config,source_ambiguous=ambiguous,phase_seconds=phases)
            phases['fit_including_independent_validation_seconds']=perf_counter()-started
            if observer:
                observer.fitted(int(identity), record, perf_counter()-started)
            record['observation_segmentation']=audit
            records.append(record)
    if observer:
        observer.complete([int(r['mask_id']) for r in records])
    if primary:
        (output.parent/'metric_pointmap_filter_audit.json').write_text(json.dumps(support_audits,indent=2),encoding='utf-8')
        (output.parent/'metric_pipeline_timing.json').write_text(json.dumps(dict(instances=phase_rows,
            scope='parent included in extraction; fit includes independent validation; observer I/O in stage_trace/index.json, do not add nested times'),indent=2),encoding='utf-8')
    result={'instances':records,'pointmap_source':'REGISTERED_METRIC_DEPTH','strategy':'DEPTH_CONSTRAINED_METRIC_FACES_V1',
            'legacy_comparison':'tools/metric_v4_runner.py','source':str(source)}
    path=output/'validated_geometry.json'; path.write_text(json.dumps(result,indent=2), encoding='utf-8')
    image=cv2.imread(str(source)) if rgb is None else rgb[..., ::-1].copy()
    # Render only the final independent result persisted above.
    for record in result['instances']:
        for face in record['camera_facing_faces']:
            polygon=project(face['corners_3d_m'],K).round().astype(np.int32)
            cv2.polylines(image,[polygon],True,(0,220,0),2)
    cv2.imwrite(str(output/'final_metric_faces_overlay.png'),image)
    return result
