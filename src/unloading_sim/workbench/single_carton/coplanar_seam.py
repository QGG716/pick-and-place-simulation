"""User-selected coplanar seam policy and missing-edge diagnostics (2026-11-19).

A carton face that continues in the same plane into a touching neighbour has
no depth step at its edge, so neither a direct depth boundary nor a free ray
can bound it. The user decided (2026-11-19 20:41) that such an edge may be
taken from the segmentation boundary when it coincides with a strong straight
RGB edge, with an explicit outward error bound (default 5 mm). This is a field
policy, not depth evidence: the measured box is unchanged, the interval is
recorded as USER_POLICY_RGB_SEAM_NOT_DEPTH_BOUNDARY, and only the four in-plane
edges of the measured front face qualify, after direct and free-ray evidence
were both missing. Every remaining missing edge is classified for display.
"""
import cv2
import numpy as np
from .rgb_edges import edge_map, mask_boundary, BAND_PX

MODES = ('DISABLED', 'MASK_RGB_EDGE_COINCIDENT')
MIN_UNCERTAINTY_M = .005   # user decision: +/-5 mm; the field may only enlarge it
MAX_UNCERTAINTY_M = .02

# Edge selection: boundary pixels of this side, away from both corners.
SIDE_WINDOW_M = .015
MIN_SEGMENT_PIXELS = 15
# Condition 1, coplanar continuation just outside the boundary.
OUTWARD_PX = (1, 6)
COPLANAR_MIN_M = .003
COPLANAR_RMS_FACTOR = 4.
FOREGROUND_M = .008
MIN_OUTWARD_VALID = 20
COPLANAR_MAJORITY = .5
BEYOND_LINE_PX = 2.   # stricter than the user's 1-6 px rule: beyond the RGB line
FAR_BAND_PX = (7, 14)  # ... and the continuation must persist past depth fattening
# Condition 2, strong straight RGB edge in +/-BAND_PX of the boundary.
LINE_INLIER_PX = 1.5
EDGE_COVERAGE = .70
EDGE_ANGLE_DEG = 3.
# Calibrated on the field-like synthetic scenes and the saved field frames;
# values and basis in docs/evidence/20261119_geometry_robustness/report.json.
GRADIENT_TEXTURE_RATIO = 2.
GRADIENT_MIN = 40.
# Condition 3, mask boundary coincident with that line.
LINE_MEDIAN_PX = 2.
LINE_WITHIN_PX = 3.
LINE_WITHIN_FRACTION = .80
# Condition 4.
IMAGE_BORDER_PX = 2
BBOX_EDGE_PX = 2
BBOX_TOUCH_FRACTION = .2

CLASSES = ('COPLANAR_CONTINUATION', 'INVALID_DEPTH_BAND', 'FOREGROUND_OCCLUDER', 'IMAGE_BORDER',
           'DETECTION_BOX_EDGE', 'NO_RGB_EDGE', 'OTHER')


def policy(geometry):
    """None when DISABLED (default); otherwise the validated policy."""
    value = (geometry or {}).get('coplanar_seam_policy', {'mode': 'DISABLED'})
    if not isinstance(value, dict) or value.get('mode') not in MODES:
        raise ValueError('COPLANAR_SEAM_POLICY_MODE_INVALID')
    if value['mode'] == 'DISABLED':
        if set(value)-{'mode', 'uncertainty_m', 'basis'}:raise ValueError('COPLANAR_SEAM_POLICY_FIELDS_INVALID')
        return None
    if set(value) != {'mode', 'uncertainty_m', 'basis'}:raise ValueError('COPLANAR_SEAM_POLICY_FIELDS_INVALID')
    u = value['uncertainty_m']
    if type(u) not in (int, float) or not np.isfinite(u) or not MIN_UNCERTAINTY_M <= u <= MAX_UNCERTAINTY_M:
        raise ValueError('COPLANAR_SEAM_UNCERTAINTY_OUT_OF_RANGE_0.005_TO_0.02_M')
    if not isinstance(value['basis'], str) or not value['basis'].strip():
        raise ValueError('COPLANAR_SEAM_BASIS_REQUIRED')
    return dict(mode=value['mode'], uncertainty_m=float(u), basis=value['basis'])


def pinhole(points):
    """Intrinsics (fx, cx, fy, cy) recovered from the organised metric points."""
    valid = np.isfinite(points).all(-1) & (points[..., 2] > .1)
    v, u = np.nonzero(valid)
    if len(u) < 100:raise ValueError('PINHOLE_UNRESOLVED')
    step = max(1, len(u)//20000); v, u = v[::step], u[::step]; P = points[v, u]
    fx, cx = np.polyfit(P[:, 0]/P[:, 2], u, 1); fy, cy = np.polyfit(P[:, 1]/P[:, 2], v, 1)
    return float(fx), float(cx), float(fy), float(cy)


class FrontPlane:
    """Ray/plane geometry of the measured front face in the measured box frame."""

    def __init__(self, points, camera_pose, front_z):
        self.K = pinhole(points); fx, cx, fy, cy = self.K
        h, w = points.shape[:2]; v, u = np.indices((h, w))
        self.rays = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones((h, w))), -1)
        self.R = camera_pose[:3, :3]; self.t = camera_pose[:3, 3]; self.N = self.R[:, 2]
        self.front_z = float(front_z); self.F = self.t+self.N*front_z
        with np.errstate(divide='ignore', invalid='ignore'):
            scale = (self.F@self.N)/(self.rays@self.N)
        q = self.rays*scale[..., None]
        self.local = np.where((scale > 0)[..., None], (q-self.t)@self.R, np.nan)

    def project(self, local):
        c = np.asarray(local)@self.R.T+self.t; fx, cx, fy, cy = self.K
        return np.c_[fx*c[:, 0]/c[:, 2]+cx, fy*c[:, 1]/c[:, 2]+cy]


def _distance_to(pixels_mask):
    return cv2.distanceTransform((~pixels_mask).astype(np.uint8), cv2.DIST_L2, 5)


def evaluate_edge(points, mask, rgb, plane, axis, sign, low, high, pad, rms, bbox=None):
    """Classify one missing in-plane edge and evaluate the seam conditions.

    Returns a record with ``classification`` (one of CLASSES) and, when every
    seam condition holds, ``seam`` with the outer positions of the RGB line and
    the mask boundary on the front plane (measured box frame, metres).
    """
    h, w = mask.shape; other = 1-axis; edge = float(low[axis] if sign < 0 else high[axis])
    span = (float(low[other]+pad[other]), float(high[other]-pad[other]))
    L = plane.local; record = dict(axis=axis, side=sign, edge_m=edge, interior_other_axis_m=list(span))
    if span[1] <= span[0]:return dict(record, classification='OTHER', reason='EDGE_SHORTER_THAN_PAD')
    with np.errstate(invalid='ignore'):
        interior = (L[..., other] > span[0]) & (L[..., other] < span[1])
        boundary = mask_boundary(mask)
        segment = boundary & interior & (abs(L[..., axis]-edge) <= SIDE_WINDOW_M) & ((L[..., axis]-(low[axis]+high[axis])/2)*sign > 0)
    count = int(segment.sum()); record['segment_pixels'] = count
    if count < MIN_SEGMENT_PIXELS:return dict(record, classification='OTHER', reason='NO_MASK_BOUNDARY_SEGMENT_ON_THIS_SIDE')
    sy, sx = np.nonzero(segment)
    record['segment_image_bbox_xyxy'] = [int(sx.min()), int(sy.min()), int(sx.max()), int(sy.max())]
    border = (sx < IMAGE_BORDER_PX) | (sy < IMAGE_BORDER_PX) | (sx >= w-IMAGE_BORDER_PX) | (sy >= h-IMAGE_BORDER_PX)
    record['image_border_pixels'] = int(border.sum())
    if border.any():return dict(record, classification='IMAGE_BORDER')
    if bbox is not None:
        x0, y0, x1, y1 = (float(v) for v in bbox)
        near = ((abs(sx-x0) <= BBOX_EDGE_PX) | (abs(sx-x1) <= BBOX_EDGE_PX) |
                (abs(sy-y0) <= BBOX_EDGE_PX) | (abs(sy-y1) <= BBOX_EDGE_PX))
        record['detection_box_edge_fraction'] = float(near.mean())
        if near.mean() >= BBOX_TOUCH_FRACTION:return dict(record, classification='DETECTION_BOX_EDGE')
    else:record['detection_box_edge_fraction'] = None
    # Condition 1: depth just outside the boundary, beyond this edge. A mask
    # one or two pixels inside the true silhouette would otherwise count the
    # carton's own face as "continuation"; where a straight RGB edge exists,
    # only samples at least BEYOND_LINE_PX outside that line count.
    rgb_record = rgb_line(rgb, mask, segment, plane, axis, sign, edge, span, other) if rgb is not None else None
    if rgb_record is not None:record['rgb'] = rgb_record
    distance = _distance_to(segment)
    with np.errstate(invalid='ignore'):
        outward = (~mask & (distance >= OUTWARD_PX[0]) & (distance <= OUTWARD_PX[1]) & interior &
                   ((L[..., axis]-edge)*sign > 0))
    if rgb_record is not None and rgb_record.get('line_point_px') is not None:
        origin = np.asarray(rgb_record['line_point_px']); d = np.asarray(rgb_record['line_direction_px'])
        normal = np.array([-d[1], d[0]]); v, u = np.indices(mask.shape)
        offset = (u-origin[0])*normal[0]+(v-origin[1])*normal[1]
        inside = np.sign(np.mean(offset[mask & (distance <= OUTWARD_PX[1])])) or 1.
        outward &= -inside*offset >= BEYOND_LINE_PX
        record['outward_beyond_rgb_line_px'] = BEYOND_LINE_PX
    valid = outward & np.isfinite(points).all(-1) & (points[..., 2] > .1) & (points[..., 2] < 3.)
    signed = (points[valid]-plane.F)@plane.N
    tolerance = max(COPLANAR_MIN_M, COPLANAR_RMS_FACTOR*float(rms))
    coplanar = int((abs(signed) <= tolerance).sum()); foreground = int((signed > FOREGROUND_M).sum())
    record.update(outward_pixels=int(outward.sum()), outward_valid_depth=int(valid.sum()),
                  outward_coplanar=coplanar, outward_foreground=foreground, coplanar_tolerance_m=tolerance,
                  foreground_threshold_m=FOREGROUND_M)
    if foreground:return dict(record, classification='FOREGROUND_OCCLUDER')
    if valid.sum() < MIN_OUTWARD_VALID:return dict(record, classification='INVALID_DEPTH_BAND')
    record['outward_coplanar_fraction'] = coplanar/float(valid.sum())
    if coplanar/float(valid.sum()) < COPLANAR_MAJORITY:return dict(record, classification='OTHER', reason='OUTWARD_DEPTH_NOT_COPLANAR')
    # Stereo matching fattens foreground depth a few pixels past a silhouette
    # (seen on field frame d6d985f1: wall in RGB, carton depth in the 1-6 px
    # band). A coplanar neighbour continues; fattening does not. Stricter
    # than the user's 1-6 px rule; it never applies the policy more often.
    with np.errstate(invalid='ignore'):
        far = (~mask & (distance >= FAR_BAND_PX[0]) & (distance <= FAR_BAND_PX[1]) & interior &
               ((L[..., axis]-edge)*sign > 0) & np.isfinite(points).all(-1) & (points[..., 2] > .1) & (points[..., 2] < 3.))
    if rgb_record is not None and rgb_record.get('line_point_px') is not None:far &= -inside*offset >= BEYOND_LINE_PX
    far_signed = (points[far]-plane.F)@plane.N
    record.update(far_band_px=list(FAR_BAND_PX), far_band_valid_depth=int(far.sum()),
                  far_band_coplanar=int((abs(far_signed) <= tolerance).sum()), far_band_foreground=int((far_signed > FOREGROUND_M).sum()))
    if record['far_band_foreground']:return dict(record, classification='FOREGROUND_OCCLUDER')
    if far.sum() < MIN_OUTWARD_VALID:return dict(record, classification='INVALID_DEPTH_BAND', reason='CONTINUATION_NOT_PRESENT_BEYOND_6_PX')
    if record['far_band_coplanar']/float(far.sum()) < COPLANAR_MAJORITY:
        return dict(record, classification='OTHER', reason='CONTINUATION_ENDS_WITHIN_6_PX_POSSIBLE_DEPTH_FATTENING')
    record['classification'] = 'COPLANAR_CONTINUATION'
    if rgb_record is None:return dict(record, rgb=dict(passed=False, reason='NO_RGB'), classification='NO_RGB_EDGE')
    record['rgb'] = rgb_record
    if not rgb_record['passed']:record['classification'] = 'NO_RGB_EDGE'
    return record


def rgb_line(rgb, mask, segment, plane, axis, sign, edge, span, other):
    edges, gradient, _, _ = edge_map(rgb)
    band = _distance_to(segment) <= BAND_PX
    ey, ex = np.nonzero(edges & band)
    out = dict(passed=False, band_px=BAND_PX, band_edge_pixels=int(len(ex)))
    if len(ex) < MIN_SEGMENT_PIXELS:return dict(out, reason='NO_EDGE_PIXELS_IN_BAND')
    vx, vy, x0, y0 = cv2.fitLine(np.c_[ex, ey].astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).ravel()
    direction = np.array([vx, vy], float); origin = np.array([x0, y0], float); normal = np.array([-vy, vx], float)
    residual = abs((np.c_[ex, ey]-origin)@normal); inl = residual <= LINE_INLIER_PX
    ix, iy = ex[inl], ey[inl]
    # Expected image direction of this box edge on the front plane.
    ends = np.zeros((2, 3)); ends[:, axis] = edge; ends[:, other] = span; ends[:, 2] = plane.front_z
    ends = plane.project(ends); expected = ends[1]-ends[0]; length = float(np.linalg.norm(expected))
    angle = float(np.degrees(np.arccos(min(1., abs(direction@expected)/max(length, 1e-9)))))
    t0, t1 = sorted(((ends-origin)@direction).tolist())
    params = (np.c_[ix, iy]-origin)@direction
    bins = np.arange(np.floor(t0), np.ceil(t1)+1)
    occupied = np.histogram(params, bins)[0] > 0 if len(bins) > 1 else np.zeros(0, bool)
    coverage = float(occupied.mean()) if len(occupied) else 0.
    face = cv2.erode(mask.astype(np.uint8), np.ones((15, 15), np.uint8)).astype(bool)
    texture = float(np.percentile(gradient[face], 90)) if face.any() else 0.
    strength = float(np.median(gradient[iy, ix])) if len(ix) else 0.
    sy, sx = np.nonzero(segment); mask_distance = abs((np.c_[sx, sy]-origin)@normal)
    out.update(line_point_px=origin.tolist(), line_direction_px=direction.tolist(), inlier_edge_pixels=int(inl.sum()),
               angle_to_projected_edge_deg=angle, coverage=coverage, edge_gradient_median=strength,
               face_texture_gradient_p90=texture, gradient_ratio=strength/max(texture, 1e-9),
               mask_to_line_median_px=float(np.median(mask_distance)),
               mask_within_3px_fraction=float((mask_distance <= LINE_WITHIN_PX).mean()),
               thresholds=dict(coverage=EDGE_COVERAGE, angle_deg=EDGE_ANGLE_DEG, gradient_ratio=GRADIENT_TEXTURE_RATIO,
                               gradient_min=GRADIENT_MIN, median_px=LINE_MEDIAN_PX, within_px=LINE_WITHIN_PX,
                               within_fraction=LINE_WITHIN_FRACTION))
    failures = []
    if angle > EDGE_ANGLE_DEG:failures.append('LINE_ANGLE')
    if coverage < EDGE_COVERAGE:failures.append('LINE_COVERAGE')
    if strength < GRADIENT_MIN or strength < GRADIENT_TEXTURE_RATIO*texture:failures.append('EDGE_NOT_STRONGER_THAN_TEXTURE')
    if out['mask_to_line_median_px'] > LINE_MEDIAN_PX or out['mask_within_3px_fraction'] < LINE_WITHIN_FRACTION:
        failures.append('MASK_NOT_COINCIDENT_WITH_LINE')
    if failures:return dict(out, reason=failures)
    # Outer positions on the front plane: the RGB line and the mask boundary.
    samples = origin+np.outer(np.linspace(t0, t1, 50), direction)
    rays = np.c_[(samples[:, 0]-plane.K[1])/plane.K[0], (samples[:, 1]-plane.K[3])/plane.K[2], np.ones(len(samples))]
    with np.errstate(divide='ignore', invalid='ignore'):
        q = rays*((plane.F@plane.N)/(rays@plane.N))[:, None]
    line_local = (q-plane.t)@plane.R; boundary_local = plane.local[segment]
    pick = np.max if sign > 0 else np.min
    out.update(passed=True, line_outer_m=float(pick(line_local[:, axis])), mask_outer_m=float(pick(boundary_local[:, axis])),
               line_local_m=line_local[[0, -1]].tolist())
    return out


def missing_edge(points, mask, rgb, plane, axis, sign, low, high, pad, rms, setting, bbox=None):
    """Classification plus, when the policy applies, the seam outer bound.

    ``plane`` is a FrontPlane (or the ValueError raised building it).
    """
    if axis == 2:return dict(classification='OTHER', reason='THICKNESS_AXIS_NOT_SEAM_ELIGIBLE',
                             seam_policy='NOT_APPLICABLE' if setting else 'DISABLED')
    if isinstance(plane, Exception):return dict(classification='OTHER', reason=str(plane),
                                                seam_policy='NOT_APPLICABLE' if setting else 'DISABLED')
    record = evaluate_edge(points, mask, rgb, plane, axis, sign, low, high, pad, rms, bbox)
    rgb_record = record.get('rgb') or {}
    if setting is None:
        record['seam_policy'] = 'DISABLED'; return record
    if record['classification'] != 'COPLANAR_CONTINUATION' or not rgb_record.get('passed'):
        record['seam_policy'] = 'NOT_APPLICABLE'; return record
    edge = record['edge_m']; pick = max if sign > 0 else min
    outer = pick(rgb_record['line_outer_m'], rgb_record['mask_outer_m'], edge)+sign*setting['uncertainty_m']
    record.update(seam_policy='APPLIED', outer_m=float(outer))
    return record
