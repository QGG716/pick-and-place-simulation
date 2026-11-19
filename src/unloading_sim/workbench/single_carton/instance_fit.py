"""Separate a nested detection for fitting, without deleting scene evidence.

Some SAM instances cover a stack while another mask identifies one box inside
it. Whenever a smaller detection lies >= 95 % inside this mask, the surface fit
first uses the difference, whatever the full mask would have reported. The full
original mask still supplies conservative envelope boundary evidence; both
detections and every raw RGB-D point remain available to the scene builder.
If the difference cannot be fitted (e.g. a partial duplicate of the same face
cut through a continuing plane) the original mask is tried and both outcomes
are recorded.
"""
import numpy as np
from .geometry import recover, GeometryRejected

NESTED_CONTAINMENT = .95  # same overlap criterion as saved-instance deduplication


def nested_children(masks, index):
    mask = np.asarray(masks[index], bool); area = int(mask.sum()); children = []
    for j, other in enumerate(masks):
        if j == index: continue
        other = np.asarray(other, bool); other_area = int(other.sum())
        if other.shape != mask.shape or not 0 < other_area < area: continue
        overlap = int(np.count_nonzero(mask & other))
        if overlap/other_area >= NESTED_CONTAINMENT:
            children.append(dict(detection_index=j, pixels=overlap, mask_pixels=other_area))
    return children


def measured_instance(points, masks, index, world_camera, *, rgb, maximum, seam_policy=None, bbox=None):
    mask = np.asarray(masks[index], bool)
    options = dict(rgb=rgb, prior=None, conservative_measured=True, max_carton_extent_m=maximum,
                   seam_policy=seam_policy, bbox=bbox)
    children = nested_children(masks, index)
    if not children:
        return recover(points, mask, world_camera, **options)
    fit = mask.copy()
    for child in children: fit &= ~np.asarray(masks[child['detection_index']], bool)
    evidence = dict(method='NESTED_INSTANCE_EXCLUSION_FOR_SURFACE_FIT_ONLY', order='DIFFERENCE_FIRST',
        original_mask_pixels=int(mask.sum()), fit_mask_pixels=int(fit.sum()), excluded=children,
        original_mask_retained_for_envelope=True, original_scene_evidence_retained=True,
        excluded_pixels_are_free_space=False)
    try:
        box = recover(points, fit, world_camera, envelope_mask=mask, **options)
        box['instance_fit'] = dict(evidence, used='DIFFERENCE_MASK')
        return box
    except ValueError as difference_failure:
        evidence['difference_failure'] = str(difference_failure)
        try:
            box = recover(points, mask, world_camera, **options)
        except ValueError as original_failure:
            details = dict(getattr(difference_failure, 'evidence', None) or {},
                           instance_fit=dict(evidence, original_failure=str(original_failure)))
            raise GeometryRejected(str(difference_failure), details) from difference_failure
        box['instance_fit'] = dict(evidence, used='ORIGINAL_MASK_AFTER_DIFFERENCE_FAILED')
        return box
