"""Reassemble immutable captures without SAM, MoGe or geometry inference."""
from __future__ import annotations
import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np

from run_isaac_rgbd_geometry import (
    _observation, _worker_artifacts, sha256, CaptureMetadata, IsaacSceneManifest,
    ground_truth_observation, evaluate_observations, dumps, write_json,
)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-directory', type=Path, required=True)
    parser.add_argument('--output-directory', type=Path, required=True)
    args=parser.parse_args()
    root=args.capture_directory.resolve(); out=args.output_directory.resolve()
    if out==root or root in out.parents:
        raise ValueError('output must be separate from frozen evidence')
    out.mkdir(parents=True,exist_ok=False)
    index=json.loads((root/'scene_bundle/index.json').read_text())
    manifest_record=next(i for i in index['scenes'] if i['scene']=='FULL_STACK_NOMINAL')
    manifest=IsaacSceneManifest.from_dict(json.loads((root/'scene_bundle'/manifest_record['path']).read_text()))
    rows=[]; reports={}; file_manifest={}
    for directory in [root/'FULL_STACK_NOMINAL', root/'FULL_STACK_NOMINAL/modules/module_1_lower']:
        artifacts=_worker_artifacts(directory)
        masks=Path(artifacts['cargo_masks.npz']['path'])
        geometry=json.loads((directory/'rgbd_cuboids.json').read_text())
        proposals=json.loads((directory/'oracle_proposals.json').read_text()); proposals['_scene_dir']=str(directory)
        metadata=CaptureMetadata.from_dict(json.loads((directory/'capture_metadata.json').read_text()))
        camera=next(c for c in manifest.cameras if c['frame_id']==metadata.rgb_frame_id)
        module=camera['module_id']; dest=out/module; dest.mkdir()
        old=json.loads((directory/'mode_b_rgbd_observation.json').read_text())['payload']
        elapsed=old['processed_time']-old['capture_time']
        observation,predicted,hypotheses,faces=_observation('FULL_STACK_NOMINAL',manifest,geometry,masks,proposals,elapsed,validate_geometry=False)
        truth=ground_truth_observation(manifest,json.loads((directory/'gt_annotations.json').read_text())['objects'],camera_frame_id=metadata.rgb_frame_id)
        with np.load(directory/'gt_instance_masks.npz',allow_pickle=False) as archive:
            gt_masks={k:archive[k] for k in archive.files}
        report=evaluate_observations(truth,observation,ground_truth_masks=gt_masks,prediction_masks=predicted,T_W_C=metadata.T_W_C_at_capture)
        old_report=json.loads((directory/'mode_b_rgbd_evaluation.json').read_text())
        reports[module]={'before':old_report,'after':report}
        write_json(dest/'identity_corrected_evaluation.json',report)
        (dest/'identity_corrected_observation.json').write_text(dumps(observation))
        write_json(dest/'observed_faces.json',[asdict(f) for f in faces])
        new_metrics={i['prediction_source_instance_id']:i for i in report['per_object']}
        old_metrics={i['prediction_source_instance_id']:i for i in old_report['per_object']}
        source=cv2.imread(str(directory/'sensor_rgb.png')); before=source.copy(); after=source.copy()
        original_by_id={i['id']:i for i in proposals['instances']}
        for cargo in observation.cargo:
            audit=dict(cargo.raw_result['instance_lineage']); mask_id=audit['mask_id']; mask=predicted[cargo.source_instance_id]
            old_proposal=original_by_id.get(mask_id)
            audit.update(old_wrong_proposal_id=None if old_proposal is None else old_proposal['id'],old_wrong_entity=None if old_proposal is None else old_proposal.get('simulation_object_id'),geometry_center_world=None if cargo.pose is None else cargo.pose.position_m,before_error=old_metrics.get(str(mask_id)),after_error=new_metrics.get(cargo.source_instance_id))
            rows.append(audit)
            x,y,_,_=audit['mask_bbox']; contours,_=cv2.findContours(mask.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for canvas,label in [(before,f"SAM {mask_id} -> P{mask_id}"),(after,f"SAM {mask_id} -> P{audit['proposal_id']} +{audit['supporting_proposal_ids']}")]:
                cv2.drawContours(canvas,contours,-1,(0,255,255),2)
                cv2.putText(canvas,label,(x,max(y,20)),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),4,cv2.LINE_AA)
                cv2.putText(canvas,label,(x,max(y,20)),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        overlay=np.hstack((before,after)); cv2.imwrite(str(dest/'identity_corrected_overlay.png'),overlay)
        bounds=np.asarray([r['mask_bbox'] for r in rows if r['module_id']==module])
        x0,y0=np.maximum(bounds[:,:2].min(axis=0)-30,0); x1,y1=np.minimum(bounds[:,2:].max(axis=0)+30,[source.shape[1],source.shape[0]])
        zoom=np.hstack((before[y0:y1,x0:x1],after[y0:y1,x0:x1]))
        cv2.imwrite(str(dest/'identity_corrected_zoom.png'),zoom)
        for p in [directory/'rgbd_cuboids.json',directory/'oracle_proposals.json',directory/'capture_metadata.json',directory/'mode_b_rgbd_evaluation.json',masks,masks.parent/'cargo_instances.json']:
            file_manifest[str(p)]=sha256(p)
    write_json(out/'instance_lineage_audit.json',rows)
    write_json(out/'identity_corrected_evaluation.json',reports)
    with (out/'identity_mapping_before_after.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0])); writer.writeheader()
        writer.writerows({k:json.dumps(v) if isinstance(v,(dict,list,tuple)) else v for k,v in r.items()} for r in rows)
    write_json(out/'manifest.json',{'code_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'inputs':file_manifest,'inference_rerun':False,'geometry_recomputed':False,'historical_metrics_affected':['mask_iou','center_translation_error_m','orientation','dimensions','object_correspondence'],'conflicting_merges_excluded_from_unique_gt_metrics':True})
    print(json.dumps({m:{side:{k:r[k] for k in ('mean_mask_iou','mean_center_translation_error_m','processed_object_count')} for side,r in v.items()} for m,v in reports.items()},indent=2))


if __name__=='__main__': main()
