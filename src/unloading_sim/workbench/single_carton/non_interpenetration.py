"""Two rigid bodies cannot interpenetrate (user decision 2026-11-23, item 3).

A measured carton's conservative envelope extends past its measured box where
a boundary is missing (maximum extent, coplanar continuation). Returns of
OTHER detected instances are measured surfaces of other bodies: the true
carton cannot contain them. A return that lies beyond the measured box (plus
its pad) on exactly one axis/side, and inside the measured box (plus pad) on
the other two axes, can only be excluded by that one side stopping before it,
so that side is cut there. A return beyond the measured box on two or more
axes could be avoided either way and cuts nothing (the union of the
alternatives is kept). Only other instances' pixels count: the carton's own
mask, its stereo-fattening band and flying pixels never cut it; unlabelled
returns may be the carton itself and never cut. A side is cut at the
NI_ROBUST_RANK-th nearest such return (a few mislabelled pixels only keep the
envelope larger), minus the side's pad. The measured box is never cut.
"""
import cv2
import numpy as np

NI_ROBUST_RANK = 10


def policy(geometry):
    value = (geometry or {}).get('non_interpenetration_policy')
    if value is None or (isinstance(value, dict) and value.get('mode') == 'DISABLED'):
        return None
    if (not isinstance(value, dict) or value.get('mode') != 'OTHER_INSTANCE_SURFACES_CUT_ENVELOPE' or
            not isinstance(value.get('basis'), str) or not value['basis'].strip()):
        raise ValueError('NON_INTERPENETRATION_POLICY_INVALID')
    return dict(value)


def cut_by_other_instances(boxes, points, masks, world_camera, setting, face_band_px=None):
    """Cut each MEASURED_BOUNDARY_INTERVALS envelope in place; returns records."""
    from .geometry import corners, rigid, transform
    from .measured_envelope import BOUNDARY_PIXEL_FOOTPRINTS, DEPTH_NOISE_ALLOWANCE_M, depth_jump
    from .stereo_shadow import MAX_FATTENING_PX
    p = np.asarray(points, float); W = rigid(world_camera)
    valid = np.isfinite(p).all(-1) & (p[..., 2] > .1) & (p[..., 2] < 3.)
    valid &= ~depth_jump(p[..., 2], valid)
    world = transform(W, p.reshape(-1, 3)).reshape(p.shape)
    masks = [np.asarray(m, bool) for m in masks]; records = []
    for box in boxes:
        u = box.get('uncertainty') or {}
        if u.get('kind') != 'MEASURED_BOUNDARY_INTERVALS' or not box.get('measured_box'):
            continue
        own = masks[box['detection_index']]
        others = np.zeros(own.shape, bool)
        for j, m in enumerate(masks):
            if j != box['detection_index']:
                others |= m
        distance = cv2.distanceTransform((~own).astype(np.uint8), cv2.DIST_L2, 5)
        near = distance <= MAX_FATTENING_PX
        T = rigid(box['measured_box']['pose']); h = np.asarray(box['measured_box']['size_m'], float)/2
        if face_band_px is not None:
            # Same silhouette-fattening band the envelope contained (own mask
            # claim policy): unresolved/unlabelled returns on the front-face
            # plane band there never cut; another measured carton's pixels
            # beyond the original 6 px fattening band still do.
            from .site_scene import TARGET_PLANE_BAND_M
            resolved = np.zeros(own.shape, bool)
            for other in boxes:
                if other is not box:
                    resolved |= masks[other['detection_index']]
            depth = ((world-T[:3, 3])@T[:3, :3])[..., 2]
            with np.errstate(invalid='ignore'):
                on_front = abs(depth-h[2]) <= TARGET_PLANE_BAND_M
            near = near | ((distance <= face_band_px) & on_front & ~resolved)
        pick = others & ~own & ~near & valid
        local = (world[pick]-T[:3, 3])@T[:3, :3]
        lo = np.asarray(u['lower_m'], float); hi = np.asarray(u['upper_m'], float)
        record = dict(target_id=box['id'], other_instance_points=int(pick.sum()), cuts=[])
        inside = np.all((local >= lo) & (local <= hi), axis=1)
        for interval in u['intervals']:
            a, s = interval['axis'], interval['side']
            pad = BOUNDARY_PIXEL_FOOTPRINTS*float(interval.get('pixel_footprint_m', 0.))+DEPTH_NOISE_ALLOWANCE_M
            if s*interval['outer_m'] <= h[a]+pad:
                continue
            rest = [k for k in range(3) if k != a]
            beyond = inside & (s*local[:, a] > h[a]+pad)
            within = np.all(abs(local[:, rest]) <= h[rest]+pad, axis=1)
            values = np.sort(s*local[beyond & within, a])
            if len(values) < NI_ROBUST_RANK:
                continue
            cut = max(float(values[NI_ROBUST_RANK-1])-pad, h[a]+pad)
            if cut >= s*interval['outer_m']:
                continue
            previous = float(interval['outer_m']); interval['outer_m'] = s*cut
            interval['interval_m'] = sorted([interval['measured_m'], interval['outer_m']])
            interval['non_interpenetration_cut'] = dict(previous_outer_m=previous, previous_method=interval['method'],
                points_beyond=int(len(values)), robust_rank=NI_ROBUST_RANK, pad_m=pad, basis=setting['basis'],
                evidence_kind='OTHER_INSTANCE_MEASURED_SURFACE_SINGLE_AXIS_EXCLUSION')
            interval['method'] = 'NON_INTERPENETRATION_OTHER_INSTANCE_SURFACE'
            if s < 0:
                lo[a] = interval['outer_m']
            else:
                hi[a] = interval['outer_m']
            record['cuts'].append(dict(axis=a, side=s, previous_outer_m=previous, outer_m=interval['outer_m']))
        if record['cuts']:
            pose = T.copy(); pose[:3, 3] += T[:3, :3]@((lo+hi)/2); size = hi-lo
            box.update(pose=pose.tolist(), size_m=size.tolist(), corners_m=corners(pose, size).tolist())
            box['collision_envelope'] = {k: box[k] for k in ('pose', 'size_m', 'corners_m')}
            u.update(lower_m=lo.tolist(), upper_m=hi.tolist())
        box.setdefault('quality', {})['non_interpenetration'] = record
        records.append(record)
    return records
