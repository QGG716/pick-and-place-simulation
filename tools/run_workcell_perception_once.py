"""One existing oracle-proposal SAM/RGB-D run per new module, without tuning."""
import argparse
import json
from pathlib import Path
import sys
import traceback

import cv2
import numpy as np
import yaml

from run_metric_small_matrix import oracle_proposals, infer
from run_isaac_rgbd_geometry import _worker_artifacts, _run_secondary_module, IsaacSceneManifest, sha256


def evaluate_masks(folder, artifacts, output):
    """Post-hoc rendered-mask evaluation; nominal box planes are not scan truth."""
    from evaluate_carton_appearance_ab import contour, boundary_metrics
    gt = np.load(folder / 'gt_instance_masks.npz')
    truth = {k: gt[k].astype(bool) for k in gt.files if gt[k].any()}
    prediction = np.load(artifacts['cargo_masks.npz']['path'])
    masks, ids = prediction['masks'].astype(bool), prediction['mask_ids']
    gt_names = list(truth)
    areas = np.array([truth[k].sum() for k in gt_names])
    pred_areas = masks.sum(axis=(1,2))
    intersections = np.array([[np.count_nonzero(mask & truth[k]) for k in gt_names] for mask in masks])
    intersections = intersections.reshape((len(masks),len(truth)))
    iou = intersections / np.maximum(1, pred_areas[:,None]+areas[None,:]-intersections)
    relation = (intersections>=50)&(intersections>=.1*areas[None,:])&(intersections>=.1*pred_areas[:,None])
    rgb = cv2.imread(str(folder/'sensor_rgb.png')); overlay=rgb.copy();gt_overlay=rgb.copy()
    pred_edge=np.zeros(rgb.shape[:2],bool);gt_edge=pred_edge.copy()
    for identity,mask in zip(ids,masks):
        edge=contour(mask);pred_edge|=edge
        color=tuple(int(v) for v in np.random.default_rng(int(identity)+17).integers(60,250,3))
        overlay[edge]=color
    for mask in truth.values(): gt_edge|=contour(mask)
    gt_overlay[gt_edge]=(255,255,0)
    cv2.imwrite(str(output/'sam_boundaries.png'),overlay)
    cv2.imwrite(str(output/'rendered_gt_boundaries_evaluation_only.png'),gt_overlay)
    summary={'visible_gt_objects':len(truth),'sam_count':len(masks),
        'merged_prediction_count':int((relation.sum(1)>1).sum()),
        'split_or_duplicate_gt_count':int((relation.sum(0)>1).sum()),
        'gt_below_best_iou_0_5':int((iou.max(0)<.5).sum()) if len(masks) else len(truth),
        'gt_without_significant_prediction':int((relation.sum(0)==0).sum()),
        'rendered_mask_boundary':boundary_metrics(pred_edge,gt_edge),
        'boundary_reference':'ACTUAL_RENDERED_OBJECT_MASK_CONTOURS_INCLUDES_OCCLUSION_AND_CLIP_EDGES',
        'exact_mesh_planes_corners':'NOT_EVALUATED',
        'policy':{'significant_pixels':50,'minimum_fraction_each_region':.1,'best_iou_threshold':.5}}
    (output/'mask_evaluation.json').write_text(json.dumps({'summary':summary,'gt_objects':[
        {'id':k,'pixels':int(areas[j]),'best_iou':float(iou[:,j].max()) if len(masks) else 0.,
         'significant_sam_ids':[int(ids[i]) for i in np.flatnonzero(relation[:,j])]} for j,k in enumerate(gt_names)],
        'predictions':[{'sam_id':int(identity),'significant_gt_objects':[gt_names[j] for j in np.flatnonzero(relation[i])]} for i,identity in enumerate(ids)]},indent=2))
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--vision',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True);a=p.parse_args()
    output=a.capture/'perception-once';output.mkdir(exist_ok=False)
    manifest=IsaacSceneManifest.from_dict(json.loads((a.capture/'manifest.json').read_text()))
    from vision_resident_worker import ResidentRuntime, parser as worker_parser
    models=json.loads(a.models.read_text())
    runtime=ResidentRuntime(worker_parser().parse_args(['--upstream-root',str(a.vision),
        '--output-root',str(output/'sam-runs'),'--input-root',str(a.capture),'--sam-model',models['sam']['snapshot_path']]))
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/isaac/perception_validation.yaml').read_text())
    rows=[]
    for camera in manifest.cameras:
        module=camera['module_id'];folder=a.capture/'FULL_STACK_NOMINAL/modules'/module
        delivery=output/module;delivery.mkdir()
        row={'module':module,'status':'NOT_RUN','sam_attempts':0,'metric_attempts':0,
             'proposal_source':'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL','raw_image_automatic':False,
             'planning_admissible':False,'rgb_sha256':sha256(folder/'sensor_rgb.png')}
        rows.append(row)
        try:
            binding,annotations,proposals=oracle_proposals(folder,camera)
            # Same historical prompt policy: GT projected visible boxes only.
            # GT masks/depth/poses never enter SAM. Evaluation reads masks after.
            row['proposal_count']=len(proposals['instances']);row['sam_attempts']=1
            infer(runtime,folder,binding,camera,'roof-mast-'+module)
            artifacts=_worker_artifacts(folder)
            row['mask_evaluation']=evaluate_masks(folder,artifacts,delivery)
            row['metric_attempts']=1
            result=_run_secondary_module(scene='FULL_STACK_NOMINAL',module_dir=folder,manifest=manifest,
                artifacts=artifacts,config=config,vision_root=a.vision,upstream_python=Path(sys.executable),timeout=1200)
            geometry=json.loads((folder/'rgbd_cuboids.json').read_text())
            row.update(status='COMPLETED_WITH_ALGORITHM_RESULTS',observed_faces=sum(len(f.faces) for f in result['observed_face_sets']),
                instances_without_accepted_face=sum(not r['camera_facing_faces'] for r in geometry['instances']),
                complete_cuboids_accepted=sum(bool(r.get('accepted')) for r in geometry['instances']))
        except Exception as exc:
            row.update(status='TECHNICAL_FAILURE',error=str(exc));(delivery/'failure.txt').write_text(traceback.format_exc())
        (output/'summary.json').write_text(json.dumps({'model_manifest':models,'runs':rows},indent=2))
        print(json.dumps(row),flush=True)
    assert runtime.moge_model is None
    return 0


if __name__=='__main__':raise SystemExit(main())
