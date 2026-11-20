"""User-selected stereo-shadow boundary policy (2026-11-20).

The D435 depth image (aligned to colour) has no return in a band on the LEFT
of a near foreground object: those rays reach the background (in the field,
a computer screen behind the cartons) but the second imager cannot see it.
Without depth there, neither the direct boundary (valid, farther return next
to the edge) nor a free ray can bound the carton's left edge, so the envelope
used to grow by ~45 mm to the first free ray beyond the band.

The user decided (2026-11-20) that this band is background and that the
depth difference between the carton face and the return beyond the band
distinguishes them. Under the explicit policy, a left in-plane edge of the
measured front face is a depth boundary when, along the image row, the
carton face (plus at most MAX_FATTENING_PX of coplanar stereo fattening) is
followed by a no-return band and then by a valid return farther than the face
by the existing direct-boundary step. The band itself is never made free
space; it only stops being counted as a possible carton continuation for
this edge. Coverage, strip and pad rules of the direct boundary apply. This is
a field premise, not sensor evidence of free space; it is recorded as
USER_POLICY_INVALID_BAND_IS_BACKGROUND_BEYOND_DEPTH_STEP.
"""
import cv2
import numpy as np

MODES = ('DISABLED', 'INVALID_BAND_BEFORE_FARTHER_RETURN_IS_BACKGROUND')
# Sensor property, not a tuning value: depth is referenced to the left imager
# and aligned to colour, so stereo occlusion lies on the -u side of a
# foreground silhouette. Field frames 31e46172/67dbcaac/b8992c09: left band
# median 61-90 px, right band median 0 px (docs/evidence/20261120_*).
SHADOW_IMAGE_DIRECTION = -1
# Same limits as the coplanar seam's depth-fattening rule (1-6 px band).
MAX_FATTENING_PX = 6
# Same background step as the direct depth boundary in conservative_envelope.
DEPTH_STEP_M = .012
# Coplanarity of fattened samples: the seam rule's tolerance.
FATTENING_TOLERANCE_M = .003
# The edge must run across rows: its outward image direction is horizontal.
HORIZONTAL_RATIO = 2.


def policy(geometry):
    """None when DISABLED (default); otherwise the validated policy."""
    value = (geometry or {}).get('stereo_shadow_boundary_policy', {'mode': 'DISABLED'})
    if not isinstance(value, dict) or value.get('mode') not in MODES:
        raise ValueError('STEREO_SHADOW_POLICY_MODE_INVALID')
    if value['mode'] == 'DISABLED':
        if set(value)-{'mode', 'basis'}:raise ValueError('STEREO_SHADOW_POLICY_FIELDS_INVALID')
        return None
    if set(value) != {'mode', 'basis'}:raise ValueError('STEREO_SHADOW_POLICY_FIELDS_INVALID')
    if not isinstance(value['basis'], str) or not value['basis'].strip():
        raise ValueError('STEREO_SHADOW_BASIS_REQUIRED')
    return dict(mode=value['mode'], basis=value['basis'])


def outward_image_direction(plane, axis, sign, low, high):
    """Image displacement of moving this edge outward on the front plane."""
    centre = (np.asarray(low, float)+np.asarray(high, float))/2; centre[2] = plane.front_z
    edge = centre.copy(); edge[axis] = low[axis] if sign < 0 else high[axis]
    out = edge.copy(); out[axis] += sign*.02
    a, b = plane.project(np.vstack((edge, out)))
    return b-a


def shadow_edge(points, mask, use, local, plane, axis, sign, low, high, pad, rms):
    """Witness pixels and outer bound of one left edge across a no-return band.

    Returns a record with ``witnesses`` (bool image of mask-boundary pixels
    whose row shows face -> no return -> farther return) and ``outer_m``
    (outermost face sample of those rows, plus pad), or a reason.
    """
    record = dict(axis=int(axis), side=int(sign), image_direction=SHADOW_IMAGE_DIRECTION,
                  max_fattening_px=MAX_FATTENING_PX, depth_step_m=DEPTH_STEP_M)
    if axis == 2:return dict(record, accepted=False, reason='THICKNESS_AXIS_NOT_ELIGIBLE')
    if isinstance(plane, Exception):return dict(record, accepted=False, reason=str(plane))
    d = outward_image_direction(plane, axis, sign, low, high)
    record['outward_image_direction_px'] = d.tolist()
    if not (d[0]*SHADOW_IMAGE_DIRECTION > 0 and abs(d[0]) >= HORIZONTAL_RATIO*abs(d[1])):
        return dict(record, accepted=False, reason='EDGE_NOT_ON_STEREO_SHADOW_SIDE')
    h, w = mask.shape; edge = float(low[axis] if sign < 0 else high[axis])
    boundary = mask & ~cv2.erode(mask.astype('uint8'), np.ones((3, 3), 'uint8')).astype(bool)
    centre = (low[axis]+high[axis])/2
    candidates = use & boundary & (abs(local[..., axis]-edge) <= pad[axis]) & ((local[..., axis]-centre)*sign > 0)
    valid = np.isfinite(points).all(-1) & (points[..., 2] > .1) & (points[..., 2] < 3.)
    tolerance = max(FATTENING_TOLERANCE_M, 4.*float(rms))
    witnesses = np.zeros(mask.shape, bool); outer = []; gaps = []; reasons = {}
    step = SHADOW_IMAGE_DIRECTION
    for y, x in zip(*np.nonzero(candidates)):
        z0 = points[y, x, 2]; last = x; k = x+step; fattening = 0
        while 0 <= k < w and valid[y, k] and abs(points[y, k, 2]-z0) <= tolerance and not mask[y, k]:
            last = k; fattening += 1; k += step
            if fattening > MAX_FATTENING_PX:break
        if fattening > MAX_FATTENING_PX:reasons['FACE_CONTINUES_BEYOND_FATTENING'] = reasons.get('FACE_CONTINUES_BEYOND_FATTENING', 0)+1;continue
        if 0 <= k < w and mask[y, k]:reasons['OWN_MASK_AGAIN'] = reasons.get('OWN_MASK_AGAIN', 0)+1;continue
        gap = 0
        while 0 <= k < w and not valid[y, k]:gap += 1; k += step
        if not 0 <= k < w:reasons['IMAGE_BORDER_BEFORE_RETURN'] = reasons.get('IMAGE_BORDER_BEFORE_RETURN', 0)+1;continue
        if points[y, k, 2] <= z0+DEPTH_STEP_M:reasons['NO_FARTHER_RETURN'] = reasons.get('NO_FARTHER_RETURN', 0)+1;continue
        witnesses[y, x] = True; outer.append(float(local[y, last, axis])); gaps.append(gap)
    record.update(candidate_pixels=int(candidates.sum()), witness_pixels=int(witnesses.sum()),
                  rejected_rows=reasons, fattening_tolerance_m=tolerance)
    if not outer:return dict(record, accepted=False, reason='NO_SHADOW_WITNESS', witnesses=witnesses)
    pick = min if sign < 0 else max
    record.update(witnesses=witnesses, no_return_band_px=dict(median=float(np.median(gaps)), max=int(max(gaps))),
                  outermost_face_sample_m=pick(outer), outer_m=float(pick(pick(outer), edge)+sign*pad[axis]))
    return record
