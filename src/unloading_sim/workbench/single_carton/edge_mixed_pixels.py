"""Silhouette mixed pixels (user decision 2026-11-24, field 9707b9cc).

At a depth edge the D435 can blend foreground and background into a ramp of
returns that float between the two surfaces; each step stays under the 12 mm
3x3 flying-pixel jump, so they survive as obstacles beside a carton. A return
is such an artefact, and removed, only when ALL hold:

1. outside the target mask and within SILHOUETTE_ARTIFACT_PX (14 px, the
   fattening band confirmed 2026-11-20) of it;
2. behind the target's measured front-face plane by more than the 4 mm face
   band (never on the tool side);
3. on no fitted plane: farther than the 4 mm face band from every observed
   plane of the target and from the observed face plane of the instance that
   labels the pixel (if any);
4. its depth lies strictly between the two sides of the edge: more than
   MIX_MARGIN_M behind the nearest target return in its window and more than
   MIX_MARGIN_M in front of the farthest non-target return in that window;
5. not above the target's measured top (something may rest there);
6. within MIX_MAX_DISTANCE_M (30 mm) of the target's measured box in space:
   a blended return lies in the transition next to the carton, never metres
   behind it (a distant desk seen beside the carton is a real surface).

Counts are recorded per target. Real objects lie on a surface or at one of
the two depths and are kept.
"""
import cv2
import numpy as np

MIX_WINDOW_PX = 7          # half window: 15 x 15 pixels
MIX_MARGIN_M = .004
MIX_MAX_DISTANCE_M = .03


def policy(geometry):
    value = (geometry or {}).get('edge_mixed_pixel_policy')
    if value is None or (isinstance(value, dict) and value.get('mode') == 'DISABLED'):
        return None
    if (not isinstance(value, dict) or value.get('mode') != 'REMOVE_INTERPOLATED_SILHOUETTE_RETURNS' or
            not isinstance(value.get('basis'), str) or not value['basis'].strip()):
        raise ValueError('EDGE_MIXED_PIXEL_POLICY_INVALID')
    return dict(value)


def mixed_pixels(box, mask, points, world_points, valid, available, label_faces):
    """Boolean H x W of artefact returns for one target.

    ``points``: camera-frame metric points (depth = z); ``available``: returns
    not yet claimed or removed; ``label_faces``: H x W list-free lookup, a dict
    {detection_index: (mask, face)} of other instances' observed faces."""
    from .geometry import rigid
    from .site_scene import SILHOUETTE_ARTIFACT_PX, TARGET_PLANE_BAND_M
    mask = np.asarray(mask, bool)
    M = box.get('measured_box') or box
    T = rigid(M['pose']); h = np.asarray(M['size_m'], float)/2
    local = (world_points-T[:3, 3])@T[:3, :3]
    distance = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 5)
    with np.errstate(invalid='ignore'):
        band = valid & available & ~mask & (distance <= SILHOUETTE_ARTIFACT_PX)
        band &= local[..., 2] < h[2]-TARGET_PLANE_BAND_M
        vertical = int(np.argmax(abs(T[2, :3])))
        up = np.sign(T[2, vertical])
        band &= up*local[..., vertical] <= h[vertical]
        band &= np.linalg.norm(np.maximum(abs(local)-h, 0.), axis=-1) <= MIX_MAX_DISTANCE_M
        for plane in box.get('observed_plane_models', []):
            band &= abs((world_points-np.asarray(plane['center_m']))@np.asarray(plane['normal'])) > TARGET_PLANE_BAND_M
        for index, (other, face) in label_faces.items():
            on = abs((world_points-np.asarray(face['center_m']))@np.asarray(face['normal'])) <= TARGET_PLANE_BAND_M
            band &= ~(np.asarray(other, bool) & on)
    z = points[..., 2]; out = np.zeros(mask.shape, bool); H, W = mask.shape; r = MIX_WINDOW_PX
    target_z = np.where(mask & valid, z, np.inf); other_z = np.where(~mask & valid, z, -np.inf)
    for y, x in zip(*np.nonzero(band)):
        y0, y1, x0, x1 = max(0, y-r), min(H, y+r+1), max(0, x-r), min(W, x+r+1)
        near = target_z[y0:y1, x0:x1].min(); far = other_z[y0:y1, x0:x1].max()
        if np.isfinite(near) and np.isfinite(far) and near+MIX_MARGIN_M < z[y, x] < far-MIX_MARGIN_M:
            out[y, x] = True
    return out
