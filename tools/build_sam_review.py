"""Small portable stage-02 review, with all proposals including SAM failures."""
import json
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from unloading_perception.proposals import proposal_path
from unloading_perception.lineage import load_instance_lineage
from unloading_perception.prompt_audit import mask_bbox
from unloading_perception.rgb_input import load_rgb_input


def contours(mask):
    import cv2
    curves, _ = cv2.findContours(np.asarray(mask, np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return [c[:, 0, :].tolist() for c in curves if len(c) > 1]


def build_review(folder, artifacts, *, stages=None):
    folder = Path(folder)
    path = proposal_path(folder)
    proposals = json.loads(path.read_text(encoding='utf-8'))
    rgb = load_rgb_input(folder)
    instances_path = Path(artifacts['cargo_instances.json']['path'])
    masks_path = Path(artifacts['cargo_masks.npz']['path'])
    instances = json.loads(instances_path.read_text(encoding='utf-8'))
    lineage = load_instance_lineage(masks_path, instances_path, proposals, {'instances': []},
        sensor_epoch=rgb.metadata.sensor_epoch, module_id=rgb.camera['module_id'], capture_id=rgb.metadata.capture_id,
        source_path=folder/'sensor_rgb.png', proposal_path=path)
    session_path = artifacts.get('sam_image_session.json', {}).get('path')
    session = json.loads(Path(session_path).read_text()) if session_path else {}
    prompts = session.get('prompts_source_coordinates', [])
    if prompts and len(prompts) != len(proposals['instances']):
        raise ValueError('SAM_PROMPT_ROSTER_MISMATCH')
    rows = []
    for index, proposal in enumerate(proposals['instances']):
        identity = proposal['id']
        owner = next((item for item in lineage.values() if identity in [item.sam['proposal_id'], *item.sam.get('supporting_proposal_ids', [])]), None)
        audit = [a for a in instances.get('proposal_audit', []) if a['proposal_id'] == identity]
        rejected = [a for a in instances.get('rejected', []) if a['proposal_id'] == identity]
        prompt = prompts[index] if prompts else None
        rows.append(dict(proposal_id=identity, label=proposal['label'], raw_text_label=proposal.get('raw_text_label'),
            detector_score=proposal.get('detector_score'), bbox=proposal['bbox'],
            actual_prompt=prompt, status=audit[0]['status'] if audit else 'NO_MASK_OR_AUDIT', audit=audit,
            rejected=rejected, sam_instance_id=None if owner is None else owner.mask_id,
            mask_id=None if owner is None else owner.mask_id,
            mask_bbox=None if owner is None else mask_bbox(owner.mask),
            contours=[] if owner is None else contours(owner.mask),
            score_scope='retained mask; original merged/rejected candidate metrics are separate',
            original_candidate_metrics={k:rejected[0].get(k) for k in ('sam_iou_score','sam_prompt_stability','validation_score')} if rejected else None,
            sam_iou_score=None if owner is None else owner.sam['sam_iou_score'],
            sam_prompt_stability=None if owner is None else owner.sam['sam_prompt_stability']))
    evaluation = json.loads((folder/'detector_sam_evaluation.json').read_text()) if (folder/'detector_sam_evaluation.json').exists() else None
    data = dict(source=proposals['source'], rgb='sensor_rgb.png', source_size=proposals['source_size'],
        rgb_sha256=rgb.rgb_sha256, capture_id=rgb.metadata.capture_id, module_id=rgb.camera['module_id'],
        proposals=rows, detector_candidates=proposals.get('candidates', []), detector=proposals.get('detector'),
        truncated=proposals.get('truncated', False), mask_count=len(lineage), evaluation=evaluation,
        stages=stages or {s:'NOT_RUN' for s in ('geometry', 'contacts', 'fusion', 'ros', 'planning', 'execution')},
        prompt_note='Yellow = actual base box given to pinned SAM; detector original/clipping in candidate audit. White = separate mask bbox. Five actual perturbations are optional. Green = final retained mask; upstream does not persist the discarded mask pixels of merged/rejected candidates; their records and scores are retained.')
    (folder/'sam_review.json').write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    (folder/'sam_review_data.js').write_text('window.REVIEW='+json.dumps(data, allow_nan=False).replace('</','<\\/')+';', encoding='utf-8')
    shutil.copyfile(Path(__file__).with_name('sam_review.html'), folder/'sam_review.html')
    # Full resolution renders and per-proposal crops use exactly the viewer data.
    from PIL import ImageDraw
    raw = Image.fromarray(rgb.rgb)
    yellow, green = raw.copy(), raw.copy()
    yd, gd = ImageDraw.Draw(yellow), ImageDraw.Draw(green)
    for row in rows:
        box = row['actual_prompt']['base_bbox'] if row['actual_prompt'] else row['bbox']
        yd.rectangle(box, outline='yellow', width=3)
        yd.text((box[0], box[1]), str(row['proposal_id']), fill='yellow')
        gd.rectangle(box, outline='yellow', width=2)
        for curve in row['contours']:
            gd.line([tuple(p) for p in curve]+[tuple(curve[0])], fill=(30,255,80), width=2)
    yellow.save(folder/'detector_boxes.png'); green.save(folder/'sam_boundaries.png')
    crop_dir = folder/'instances'; crop_dir.mkdir(exist_ok=True)
    for row in rows:
        b = row['bbox']; w,h=raw.size
        bounds=(max(0,int(b[0])-30),max(0,int(b[1])-30),min(w,int(np.ceil(b[2]))+30),min(h,int(np.ceil(b[3]))+30))
        one_box,one_mask=raw.copy(),raw.copy()
        b=row['actual_prompt']['base_bbox'] if row['actual_prompt'] else row['bbox']
        ImageDraw.Draw(one_box).rectangle(b,outline='yellow',width=3)
        draw=ImageDraw.Draw(one_mask);draw.rectangle(b,outline='yellow',width=3)
        for curve in row['contours']:
            draw.line([tuple(p) for p in curve]+[tuple(curve[0])],fill=(30,255,80),width=2)
        pieces=[im.crop(bounds) for im in (raw,one_box,one_mask)]
        crop=Image.new('RGB',(pieces[0].width*3,pieces[0].height))
        for i,piece in enumerate(pieces):crop.paste(piece,(i*piece.width,0))
        crop.save(crop_dir/f"proposal_{row['proposal_id']:04d}.png")
    return dict(path=str(folder/'sam_review.html'), mask_count=len(lineage), proposal_count=len(rows))
