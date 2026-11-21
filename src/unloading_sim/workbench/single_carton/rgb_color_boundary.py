"""User-selected RGB colour boundary policy (2026-11-20).

Where depth cannot bound an in-plane edge of the measured front face (no-return
band to the image border, stereo fattening in front of a wall), the user
decided that the RGB colour difference between the carton and its surroundings
distinguishes them. Under the explicit policy a still-MISSING edge is bounded at
the mask boundary when, along that edge segment:

- the carton's own face chroma (Lab a*, b*; brightness is ignored) holds just
  inside the boundary and is absent 3-14 px outside it;
- nothing in that outside band is in front of the face (an occluder may hide
  a continuation) and the face does not continue coplanar 7-14 px out (tape,
  print or a flush object of another colour);
- the segment does not touch the image border.

The bound is the mask boundary on the front plane plus 3 px of footprint and
the configured uncertainty (0.005-0.02 m, field values may only enlarge it).
It is a field premise, not depth evidence: recorded as
USER_POLICY_RGB_COLOUR_DIFFERS_FROM_SURROUNDINGS_NOT_DEPTH_BOUNDARY.
"""
import cv2
import numpy as np
from .coplanar_seam import (SIDE_WINDOW_M, MIN_SEGMENT_PIXELS, IMAGE_BORDER_PX, FOREGROUND_M, COPLANAR_MIN_M,
                            COPLANAR_RMS_FACTOR, COPLANAR_MAJORITY, MIN_OUTWARD_VALID, FAR_BAND_PX, _distance_to)
from .rgb_edges import mask_boundary

MODES = ('DISABLED', 'CARTON_COLOUR_DIFFERS_FROM_SURROUNDINGS')
MIN_UNCERTAINTY_M = .005   # same user bound as the coplanar seam; field may only enlarge it
MAX_UNCERTAINTY_M = .02
FACE_ERODE_PX = 15          # face colour model away from every boundary
INSIDE_PX = (1, 6)
OUTSIDE_PX = (3, 14)        # 2 px allowed for colour blur at the silhouette
# Field frames d6d985f1/bb81d99e/6e13d059 (docs/evidence/20261120_*): face
# chroma spread p90 3.6-7.1, outside band p10 >= 19.4, carton-like fraction
# outside 0.00-0.02, inside 0.83-0.94 (at 2 x p90).
CHROMA_RADIUS_FACTOR = 2.
CHROMA_RADIUS_MIN = 3.
OUTSIDE_CARTON_MAX = .10
INSIDE_CARTON_MIN = .70


def policy(geometry):
    value = (geometry or {}).get('rgb_color_boundary_policy', {'mode': 'DISABLED'})
    if not isinstance(value, dict) or value.get('mode') not in MODES:
        raise ValueError('RGB_COLOR_BOUNDARY_POLICY_MODE_INVALID')
    if value['mode'] == 'DISABLED':
        if set(value)-{'mode', 'uncertainty_m', 'basis'}:raise ValueError('RGB_COLOR_BOUNDARY_POLICY_FIELDS_INVALID')
        return None
    if set(value) != {'mode', 'uncertainty_m', 'basis'}:raise ValueError('RGB_COLOR_BOUNDARY_POLICY_FIELDS_INVALID')
    u = value['uncertainty_m']
    if type(u) not in (int, float) or not np.isfinite(u) or not MIN_UNCERTAINTY_M <= u <= MAX_UNCERTAINTY_M:
        raise ValueError('RGB_COLOR_BOUNDARY_UNCERTAINTY_OUT_OF_RANGE_0.005_TO_0.02_M')
    if not isinstance(value['basis'], str) or not value['basis'].strip():
        raise ValueError('RGB_COLOR_BOUNDARY_BASIS_REQUIRED')
    return dict(mode=value['mode'], uncertainty_m=float(u), basis=value['basis'])


def colour_edge(points, mask, rgb, plane, axis, sign, low, high, pad, rms, setting, footprint):
    """Record with ``applied`` and, when applied, ``outer_m`` (measured frame)."""
    record = dict(axis=int(axis), side=int(sign), outside_px=list(OUTSIDE_PX), inside_px=list(INSIDE_PX))
    if setting is None:return dict(record, applied=False, reason='POLICY_DISABLED')
    if axis == 2:return dict(record, applied=False, reason='THICKNESS_AXIS_NOT_ELIGIBLE')
    if rgb is None:return dict(record, applied=False, reason='NO_RGB')
    if isinstance(plane, Exception):return dict(record, applied=False, reason=str(plane))
    h, w = mask.shape; other = 1-axis; edge = float(low[axis] if sign < 0 else high[axis])
    span = (float(low[other]+pad[other]), float(high[other]-pad[other]))
    if span[1] <= span[0]:return dict(record, applied=False, reason='EDGE_SHORTER_THAN_PAD')
    L = plane.local
    with np.errstate(invalid='ignore'):
        interior = (L[..., other] > span[0]) & (L[..., other] < span[1])
        segment = (mask_boundary(mask) & interior & (abs(L[..., axis]-edge) <= SIDE_WINDOW_M) &
                   ((L[..., axis]-(low[axis]+high[axis])/2)*sign > 0))
    record['segment_pixels'] = int(segment.sum())
    if segment.sum() < MIN_SEGMENT_PIXELS:return dict(record, applied=False, reason='NO_MASK_BOUNDARY_SEGMENT_ON_THIS_SIDE')
    sy, sx = np.nonzero(segment)
    if ((sx < IMAGE_BORDER_PX) | (sy < IMAGE_BORDER_PX) | (sx >= w-IMAGE_BORDER_PX) | (sy >= h-IMAGE_BORDER_PX)).any():
        return dict(record, applied=False, reason='IMAGE_BORDER')
    # Face chroma model, away from every boundary.
    ab = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2LAB)[..., 1:].astype(float)
    face = cv2.erode(mask.astype(np.uint8), np.ones((FACE_ERODE_PX,)*2, np.uint8)).astype(bool)
    if face.sum() < 100:return dict(record, applied=False, reason='FACE_TOO_SMALL_FOR_COLOUR_MODEL')
    centre = np.median(ab[face], 0); distance = np.linalg.norm(ab-centre, axis=-1)
    radius = max(CHROMA_RADIUS_MIN, CHROMA_RADIUS_FACTOR*float(np.percentile(distance[face], 90)))
    near = _distance_to(segment)
    with np.errstate(invalid='ignore'):
        beyond = (L[..., axis]-edge)*sign > 0
    outside = ~mask & (near >= OUTSIDE_PX[0]) & (near <= OUTSIDE_PX[1]) & interior & beyond
    inside = mask & (near >= INSIDE_PX[0]) & (near <= INSIDE_PX[1]) & interior
    record.update(face_chroma_median=centre.tolist(), chroma_radius=radius, outside_pixels=int(outside.sum()),
                  inside_pixels=int(inside.sum()))
    if outside.sum() < MIN_SEGMENT_PIXELS or inside.sum() < MIN_SEGMENT_PIXELS:
        return dict(record, applied=False, reason='COLOUR_BANDS_TOO_SMALL')
    out_like = float((distance[outside] <= radius).mean()); in_like = float((distance[inside] <= radius).mean())
    record.update(outside_carton_like_fraction=out_like, inside_carton_like_fraction=in_like,
                  thresholds=dict(outside_max=OUTSIDE_CARTON_MAX, inside_min=INSIDE_CARTON_MIN))
    if in_like < INSIDE_CARTON_MIN:return dict(record, applied=False, reason='INSIDE_NOT_CARTON_COLOUR')
    if out_like > OUTSIDE_CARTON_MAX:return dict(record, applied=False, reason='OUTSIDE_HAS_CARTON_COLOUR')
    # Depth: no occluder in front, no coplanar continuation past fattening.
    valid = np.isfinite(points).all(-1) & (points[..., 2] > .1) & (points[..., 2] < 3.)
    signed = (points[outside & valid]-plane.F)@plane.N
    record['outside_foreground'] = int((signed > FOREGROUND_M).sum())
    if record['outside_foreground']:return dict(record, applied=False, reason='FOREGROUND_OCCLUDER')
    far = ~mask & (near >= FAR_BAND_PX[0]) & (near <= FAR_BAND_PX[1]) & interior & beyond & valid
    tolerance = max(COPLANAR_MIN_M, COPLANAR_RMS_FACTOR*float(rms))
    far_signed = (points[far]-plane.F)@plane.N
    coplanar = int((abs(far_signed) <= tolerance).sum()); record.update(far_valid=int(far.sum()), far_coplanar=coplanar)
    if far.sum() >= MIN_OUTWARD_VALID and coplanar/float(far.sum()) >= COPLANAR_MAJORITY:
        return dict(record, applied=False, reason='COPLANAR_CONTINUATION_OF_ANOTHER_COLOUR')
    pick = np.max if sign > 0 else np.min
    mask_outer = float(pick(L[segment][:, axis]))
    outer = (max if sign > 0 else min)(mask_outer, edge)+sign*(OUTSIDE_PX[0]*float(footprint)+setting['uncertainty_m'])
    return dict(record, applied=True, mask_outer_m=mask_outer, outer_m=float(outer))


def carton_coloured(rgb, mask):
    """Pixels whose chroma matches the instance's own face (same model and
    radius as colour_edge). None when no usable face colour model exists."""
    if rgb is None:return None
    mask = np.asarray(mask, bool)
    face = cv2.erode(mask.astype(np.uint8), np.ones((FACE_ERODE_PX,)*2, np.uint8)).astype(bool)
    if face.sum() < 100:return None
    ab = cv2.cvtColor(np.asarray(rgb, np.uint8), cv2.COLOR_RGB2LAB)[..., 1:].astype(float)
    centre = np.median(ab[face], 0); distance = np.linalg.norm(ab-centre, axis=-1)
    radius = max(CHROMA_RADIUS_MIN, CHROMA_RADIUS_FACTOR*float(np.percentile(distance[face], 90)))
    # A neutral (white/grey/black) face cannot be told from neutral
    # surroundings by colour: no colour ownership then.
    if np.linalg.norm(centre-128.) <= radius:return None
    return distance <= radius
