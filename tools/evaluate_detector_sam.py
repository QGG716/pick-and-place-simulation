"""Post-freeze evaluation. GT correspondence never returns to inference."""
import hashlib
import json
from pathlib import Path
import numpy as np
from unloading_perception.proposals import box_iou, proposal_path
from unloading_perception.prompt_audit import validate_annotation_masks


def match_boxes(predictions, truth, threshold=.5):
    """Score-descending greedy one-to-one, best unused IoU, stable ID tie breaks."""
    used, matches = set(), []
    for p in sorted(predictions, key=lambda p: (-p.get('detector_score', 1.), p['id'])):
        eligible = [(box_iou(p['bbox'], g['bbox_xyxy']), j) for j, g in enumerate(truth) if j not in used]
        eligible.sort(key=lambda pair: (-pair[0], pair[1]))
        if eligible and eligible[0][0] >= threshold:
            iou, j = eligible[0]; used.add(j)
            matches.append(dict(proposal_id=p['id'], gt_id=truth[j]['simulation_object_id'], bbox_iou=iou))
    return matches


def evaluate(source, folder, artifacts):
    from build_sam_review import contours
    from evaluate_carton_appearance_ab import contour, boundary_metrics
    source, folder = Path(source), Path(folder)
    proposals = json.loads(proposal_path(folder).read_text())
    binding = json.loads((source/'capture_binding.json').read_text())
    if binding['rgb_sha256'] != proposals['rgb_sha256']:
        raise ValueError('EVALUATION_RGB_MISMATCH')
    def checked(name, key):
        raw=(source/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=binding[key]:raise ValueError('EVALUATION_GT_HASH_MISMATCH')
        return raw
    from io import BytesIO
    annotations=json.loads(checked('gt_annotations.json', 'gt_snapshot_sha256'))
    with np.load(BytesIO(checked('gt_instance_masks.npz', 'instance_masks_sha256')), allow_pickle=False) as a:
        truth_masks={k:a[k].astype(bool) for k in a.files}
    if (annotations['simulation_epoch']!=binding['simulation_epoch'] or annotations['simulation_frame']!=binding['frame_sequence']
            or annotations.get('module_id',binding['module_id'])!=binding['module_id']):
        raise ValueError('EVALUATION_GT_FRAME_MISMATCH')
    validate_annotation_masks(annotations,truth_masks,proposals['source_size'][::-1])
    truth=[g for g in annotations['objects'] if g['visible']]
    predictions=proposals['instances']; matches=match_boxes(predictions,truth)
    instances=json.loads(Path(artifacts['cargo_instances.json']['path']).read_text())
    with np.load(artifacts['cargo_masks.npz']['path'],allow_pickle=False) as a:
        masks={int(i):mask.astype(bool) for i,mask in zip(a['mask_ids'],a['masks'])}
    owner={p:r for r in instances['instances'] for p in [r['proposal_id'],*r.get('supporting_proposal_ids',[])]}
    matched_masks=set()
    for m in matches:
        r=owner.get(m['proposal_id'])
        if r is None:
            m.update(mask_status='NO_SAM_MASK',mask_iou=0.,mask_id=None);continue
        # A merged SAM mask may not be counted twice in mask-quality aggregates.
        if r['instance_id'] in matched_masks:
            m.update(mask_status='MERGED_MASK_ALREADY_ASSIGNED',mask_iou=0.,mask_id=r['instance_id']);continue
        matched_masks.add(r['instance_id'])
        mask=masks[r['instance_id']];gt=truth_masks[m['gt_id']]
        m.update(mask_id=r['instance_id'],mask_status='MATCHED',mask_iou=float(np.count_nonzero(mask&gt)/max(1,np.count_nonzero(mask|gt))),
                 boundary=boundary_metrics(contour(mask),contour(gt)))
    relations=[]
    for mid,mask in masks.items():
        hits=[]
        for g in truth:
            gt=truth_masks[g['simulation_object_id']];inter=int((mask&gt).sum())
            if inter>=50 and inter>=.1*mask.sum() and inter>=.1*gt.sum():hits.append(g['simulation_object_id'])
        relations.append(dict(mask_id=mid,significant_gt=hits))
    multi=[]
    for p in predictions:
        x0,y0,x1,y1=p['bbox']; hits=[]
        for g in truth:
            y,x=np.nonzero(truth_masks[g['simulation_object_id']])
            if len(x) and np.mean((x+.5>=x0)&(x+.5<x1)&(y+.5>=y0)&(y+.5<y1))>=.5:hits.append(g['simulation_object_id'])
        if len(hits)>1:multi.append(dict(proposal_id=p['id'],gt_ids=hits))
    unmatched_p=[p['id'] for p in predictions if p['id'] not in {m['proposal_id'] for m in matches}]
    unmatched_gt=[g['simulation_object_id'] for g in truth if g['simulation_object_id'] not in {m['gt_id'] for m in matches}]
    summary=dict(gt_count=len(truth),detector_count=len(predictions),matched_count=len(matches),
        precision_iou_0_5=len(matches)/len(predictions) if predictions else None,
        recall_iou_0_5=len(matches)/len(truth) if truth else None,
        unmatched_proposal_ids=unmatched_p,unmatched_gt_ids=unmatched_gt,
        mask_count=len(masks),matched_mask_mean_iou=float(np.mean([m['mask_iou'] for m in matches])) if matches else None,
        mask_iou_sum_over_all_gt=sum(m['mask_iou'] for m in matches)/len(truth) if truth else None,
        merged_mask_count=sum(len(r['significant_gt'])>1 for r in relations),
        split_or_duplicate_gt_ids=[g['simulation_object_id'] for g in truth if sum(g['simulation_object_id'] in r['significant_gt'] for r in relations)>1],
        masks_without_significant_gt=[r['mask_id'] for r in relations if not r['significant_gt']],
        gt_without_significant_mask=[g['simulation_object_id'] for g in truth if not any(g['simulation_object_id'] in r['significant_gt'] for r in relations)],
        multi_box_proposals=multi,
        suppressed_near_identical=sum(c.get('reason')=='NEAR_IDENTICAL_BOX' for c in proposals.get('candidates',[])))
    result=dict(summary=summary,matches=matches,mask_relations=relations,
        policy='bbox IoU >= 0.5; score-descending greedy one-to-one, best unused IoU; stable IDs; no reused GT or mask in means',
        boundary_reference='rendered visible mask contours, including occlusion and clipping; pixels',
        significant_mask_relation=dict(min_pixels=50,min_fraction_each=.1),
        multi_box_policy='>=50% of visible pixels of each of at least two GT objects inside detector box',
        prediction_frozen=True,gt_used_by_inference=False,
        gt_objects=[dict(id=g['simulation_object_id'],bbox=g['bbox_xyxy'],contours=contours(truth_masks[g['simulation_object_id']])) for g in truth])
    (folder/'detector_sam_evaluation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return summary
