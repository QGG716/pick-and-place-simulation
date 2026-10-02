"""Validate oracle pixel annotations, never infer input prompts from SAM output."""
import numpy as np


def mask_bbox(mask):
    y,x=np.nonzero(mask)
    return [float(x.min()),float(y.min()),float(x.max()+1),float(y.max()+1)] if len(x) else None


def validate_annotation_masks(annotations,masks,shape):
    seen=set();rows=[]
    for item in annotations['objects']:
        identity=item['simulation_object_id']
        if identity in seen:raise ValueError('DUPLICATE_ORACLE_OBJECT_ID')
        seen.add(identity)
        if not item['visible']:continue
        if item.get('mask_key') not in masks:raise ValueError('ORACLE_MASK_MISSING')
        mask=np.asarray(masks[item['mask_key']],bool)
        if mask.shape!=tuple(shape):raise ValueError('ORACLE_MASK_IMAGE_SHAPE_MISMATCH')
        box=mask_bbox(mask)
        if box is None or box!=list(item['bbox_xyxy']):raise ValueError('ORACLE_BBOX_DIFFERS_FROM_BOUND_RENDERED_MASK')
        rows.append(dict(object_id=identity,bbox=box,pixels=int(mask.sum())))
    return rows
