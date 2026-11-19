"""Separate a nested detection for fitting, without deleting scene evidence.

Some SAM instances cover a stack while another mask identifies its lower box.
Only a failed mixed-surface fit may retry with those shared pixels excluded.
The full original mask still supplies conservative envelope boundary evidence;
both detections and every raw RGB-D point remain available to the scene builder.
"""
import numpy as np
from .geometry import recover,GeometryRejected


def measured_instance(points,masks,index,world_camera,*,rgb,maximum):
    mask=np.asarray(masks[index],bool)
    options=dict(rgb=rgb,prior=None,conservative_measured=True,max_carton_extent_m=maximum)
    try:
        return recover(points,mask,world_camera,**options)
    except ValueError as original:
        if str(original)!='MASK_MIXED_NEIGHBOR_OR_NONCUBOID':raise
        fit=mask.copy();excluded=[];area=int(mask.sum())
        for j,other in enumerate(masks):
            if j==index:continue
            other=np.asarray(other,bool);other_area=int(other.sum())
            if other.shape!=mask.shape or not 0<other_area<area:continue
            overlap=int(np.count_nonzero(mask&other))
            # Same 0.95 mask overlap criterion used for saved-instance
            # deduplication. This selects fit evidence, never a free-space rule.
            if overlap/other_area<.95:continue
            fit&=~other
            excluded.append(dict(detection_index=j,pixels=overlap,mask_pixels=other_area))
        if not excluded:raise
        evidence=dict(method='NESTED_INSTANCE_EXCLUSION_FOR_SURFACE_FIT_ONLY',
            original_failure=str(original),original_mask_pixels=area,
            fit_mask_pixels=int(fit.sum()),excluded=excluded,
            original_mask_retained_for_envelope=True,original_scene_evidence_retained=True,
            excluded_pixels_are_free_space=False)
        try:
            box=recover(points,fit,world_camera,envelope_mask=mask,**options)
        except ValueError as failure:
            details=dict(getattr(failure,'evidence',None) or {},instance_fit=evidence)
            raise GeometryRejected(str(failure),details) from failure
        box['instance_fit']=evidence
        return box
