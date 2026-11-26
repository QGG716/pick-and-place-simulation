"""Adjacent contact (user decision 2026-11-24): a touching neighbour is a contact plane.

Stacked cartons stand 1-2 mm apart; the 2.4 mm pair clearance, the envelope
margin and the observed-point pad can never all hold beside a touching
neighbour. Like the support under a carton, a neighbour beside or behind it
is a contact plane:

* For each lateral side of the target's measured box (both ends of the face's
  horizontal axis; no back plane), other detected instances' returns lying
  beyond that measured side by at most ``max_gap_m`` and inside the measured
  box itself on the other two axes are the neighbour's measured surface. With
  at least ADJ_MIN_POINTS of them, the plane Q is vertical (horizontal normal
  pointing into the target) at the ADJ_ROBUST_RANK-th nearest one.
* The target envelope's side is set to Q (two solids cannot interpenetrate;
  the neighbour's measured surface is there), never inside the measured box.
* Every world obstacle within ADJ_BAND_M of Q on the neighbour side and over
  the target envelope's range on the other two axes is clipped to the
  neighbour side of Q, or extended to touch Q, and becomes an
  ``adjacent_contact`` object bound to this target and plane: only the
  attached payload, only at detach, may touch it, with no payload corner more
  than 0.4 mm past Q (native ADJACENT_CONTACT kind, same guard as a support).
  Robot and tool keep the full pair clearance against it.
"""
import numpy as np

ADJ_MIN_POINTS = 10
ADJ_ROBUST_RANK = 10
ADJ_BAND_M = .0034          # 2.4 mm pair clearance + 1 mm: obstacles closer than this to Q are contacts
MAX_GAP_LIMITS_M = (.002, .02)


def policy(geometry):
    value = (geometry or {}).get('adjacent_contact_policy')
    if value is None or (isinstance(value, dict) and value.get('mode') == 'DISABLED'):
        return None
    if (not isinstance(value, dict) or value.get('mode') != 'LATERAL_CONTACT_PLANES' or
            not isinstance(value.get('basis'), str) or not value['basis'].strip()):
        raise ValueError('ADJACENT_CONTACT_POLICY_INVALID')
    gap = value.get('max_gap_m', .01)
    if isinstance(gap, bool) or not isinstance(gap, (int, float)) or not MAX_GAP_LIMITS_M[0] <= gap <= MAX_GAP_LIMITS_M[1]:
        raise ValueError('ADJACENT_CONTACT_MAX_GAP_OUT_OF_RANGE')
    return dict(value, max_gap_m=float(gap))


def _sides(T):
    """(axis, sign) of the measured frame: both ends of the face's horizontal
    axis. No back plane: the measured thickness is a lower bound (field
    2026-11-24: 96-113 mm measured, 133 mm true), so nothing is "just behind"
    the measured back with certainty."""
    vertical = int(np.argmax(abs(T[2, :2])))
    horizontal = 1-vertical
    return vertical, [(horizontal, -1), (horizontal, 1)]


def neighbour_returns(box, points, valid, masks, boxes, face_band_px=None):
    """H x W mask of other detected instances' returns that may be a
    neighbour's surface: never this target's mask, its 6 px fattening band or
    (own-mask claim policy) the 14 px front-face band of unresolved returns;
    unlabelled returns may be the target itself and never count."""
    import cv2
    from .stereo_shadow import MAX_FATTENING_PX
    from .geometry import rigid
    own = np.asarray(masks[box['detection_index']], bool)
    others = np.zeros(own.shape, bool)
    for j, m in enumerate(masks):
        if j != box['detection_index']:
            others |= np.asarray(m, bool)
    distance = cv2.distanceTransform((~own).astype(np.uint8), cv2.DIST_L2, 5)
    near = distance <= MAX_FATTENING_PX
    if face_band_px is not None:
        from .site_scene import TARGET_PLANE_BAND_M
        resolved = np.zeros(own.shape, bool)
        for other in boxes:
            if other is not box:
                resolved |= np.asarray(masks[other['detection_index']], bool)
        T = rigid(box['measured_box']['pose']); h = np.asarray(box['measured_box']['size_m'], float)/2
        with np.errstate(invalid='ignore'):
            on_front = abs(((points-T[:3, 3])@T[:3, :3])[..., 2]-h[2]) <= TARGET_PLANE_BAND_M
        near |= (distance <= face_band_px) & on_front & ~resolved
    return others & ~own & ~near & valid


def contact_planes(box, world_points, candidates, setting):
    """Lateral contact planes for one target; trims its envelope in place."""
    from .geometry import corners, rigid
    from .measured_envelope import BOUNDARY_PIXEL_FOOTPRINTS, DEPTH_NOISE_ALLOWANCE_M
    u = box.get('uncertainty') or {}
    if u.get('kind') != 'MEASURED_BOUNDARY_INTERVALS' or not box.get('measured_box'):
        return [], dict(status='NOT_APPLICABLE')
    T = rigid(box['measured_box']['pose']); h = np.asarray(box['measured_box']['size_m'], float)/2
    local = (world_points[candidates]-T[:3, 3])@T[:3, :3]
    lo = np.asarray(u['lower_m'], float); hi = np.asarray(u['upper_m'], float)
    vertical, sides = _sides(T)
    planes = []; record = dict(status='EVALUATED', max_gap_m=setting['max_gap_m'], sides=[])
    for a, s in sides:
        interval = next(i for i in u['intervals'] if (i['axis'], i['side']) == (a, s))
        beyond = s*local[:, a]-h[a]
        # Inside the measured box itself (no pad) on the other two axes: only there
        # does the target certainly exist, so only there does a neighbour
        # return bound its side (single-axis rule, as non_interpenetration).
        within = (abs(local[:, vertical]) <= h[vertical]) & (abs(local[:, 2]) <= h[2])
        near = np.sort(beyond[(beyond > 0) & (beyond <= setting['max_gap_m']) & within])
        side = dict(axis=a, side=s, points=int(len(near)))
        if len(near) < ADJ_MIN_POINTS:
            record['sides'].append(dict(side, applied=False)); continue
        c = h[a]+float(near[ADJ_ROBUST_RANK-1])
        previous = float(interval['outer_m'])
        if c < s*previous:
            interval.update(outer_m=s*c, interval_m=sorted([interval['measured_m'], s*c]), method='ADJACENT_CONTACT_PLANE',
                            adjacent_contact=dict(previous_outer_m=previous, previous_method=interval['method'],
                                                  gap_to_measured_m=c-h[a], points=int(len(near)), basis=setting['basis']))
            if s < 0:
                lo[a] = s*c
            else:
                hi[a] = s*c
        else:
            c = s*previous        # the envelope already ends before the neighbour: the plane is its side
        normal = -s*T[:3, a]; point = T[:3, 3]+T[:3, a]*(s*c)
        planes.append(dict(target_id=box['id'], axis=a, side=s, normal_world=normal.tolist(), offset_m=float(normal@point),
                           gap_to_measured_m=c-h[a]))
        record['sides'].append(dict(side, applied=True, gap_to_measured_m=c-h[a], previous_outer_m=previous))
    if planes:
        pose = T.copy(); pose[:3, 3] += T[:3, :3]@((lo+hi)/2); size = hi-lo
        box.update(pose=pose.tolist(), size_m=size.tolist(), corners_m=corners(pose, size).tolist())
        box['collision_envelope'] = {k: box[k] for k in ('pose', 'size_m', 'corners_m')}
        u.update(lower_m=lo.tolist(), upper_m=hi.tolist())
    box.setdefault('quality', {})['adjacent_contact'] = record
    return planes, record


def _vertices(o):
    from .geometry import corners, rigid
    if 'convex_mesh' in o:
        T = rigid(o.get('pose', np.eye(4).tolist()))
        v = np.asarray(o['convex_mesh']['vertices'], float)
        return v@T[:3, :3].T+T[:3, 3]
    return corners(o['pose'], o['size'])


def _clip_to_plane(o, plane, box, ident, setting):
    """One obstacle against one target contact plane (shared by both callers).

    None when the obstacle does not reach the band of ``plane`` over the
    target envelope; otherwise the clipped (or extended) ``adjacent_contact``
    object bound to the target, and its record.
    """
    from .convex_clip import clip, mesh
    from .geometry import rigid
    v = _vertices(o)
    n = np.asarray(plane['normal_world'], float); d = plane['offset_m']
    E = rigid(box['pose']); eh = np.asarray(box['size_m'], float)/2
    lv = (v-E[:3, 3])@E[:3, :3]
    rest = [i for i in range(3) if i != plane['axis']]
    overlap = np.all((lv[:, rest].max(0) >= -eh[rest]-ADJ_BAND_M) & (lv[:, rest].min(0) <= eh[rest]+ADJ_BAND_M))
    top = float((v@n).max())
    if not overlap or top <= d-ADJ_BAND_M or float((v@n).min()) >= d:
        return None
    part = clip(v, np.r_[n, -d])
    if part is None:
        return None
    part = np.asarray(part, float)
    if float((part@n).max()) < d-1e-9:
        lift = part[part@n >= float((part@n).max())-1e-9]
        part = np.unique(np.round(np.r_[part, lift+np.outer(d-lift@n, n)], 12), axis=0)
    try:
        from scipy.spatial import ConvexHull
        part = part[ConvexHull(part).vertices]     # hull vertices only (native rejects unused vertices)
        shape = mesh(part)
    except Exception:
        return None
    obj = dict(id=ident, pose=np.eye(4).tolist(), convex_mesh=shape, category='adjacent_contact',
               supports_target_id=plane['target_id'], support_basis=setting['basis'],
               support_plane=dict(normal_world=plane['normal_world'], offset_m=d),
               original_id=o['id'], original_category=o.get('category'), geometry_uncertain=True,
               support_contact_qualified=False)
    return obj, dict(original_id=o['id'], target_id=plane['target_id'], clipped_m=max(0., top-d), extended_m=max(0., d-top))


def tag_neighbours(world, boxes, planes, setting):
    """Clip / extend neighbour obstacles to each plane and bind them (in place)."""
    out = []; records = []
    box_by_id = {b['id']: b for b in boxes}
    target_ids = set(box_by_id)
    for o in world:
        if o['id'] in target_ids or o.get('category') in ('confirmed_stack_support', 'observed_support', 'adjacent_contact',
                                                         'workspace_boundary', 'measured_fixed') or \
                ('size' not in o and 'convex_mesh' not in o):
            out.append(o); continue
        done = False
        for k, plane in enumerate(planes):
            clipped = _clip_to_plane(o, plane, box_by_id[plane['target_id']], o['id']+f'_adj{k}', setting)
            if clipped is None:
                continue
            out.append(clipped[0]); records.append(dict(clipped[1], plane=k))
            done = True; break
        if not done:
            out.append(o)
    world[:] = out
    return records


def measured_neighbour_world(world, target, boxes, planes, setting):
    """The planning world of one target whose touching neighbour is itself measured.

    User decision 2026-11-26 ("按此扩展，现在实现"), extending the 2026-11-24
    rule to a neighbour carton that is itself a measured box: while ``target``
    is searched, every other measured carton reaching one of the target's
    contact planes (same band test as any other neighbour obstacle) is clipped
    to the neighbour side of that plane - its own nearest measured returns -
    and becomes an ``adjacent_contact`` object bound to this target: only the
    attached payload, only at detach, may touch it, no payload corner more
    than 0.4 mm past the plane; robot and tool keep the full pair clearance.
    The target's own envelope is unchanged (it reaches the neighbour's
    surface, so the true target stays inside it). When the neighbour is the
    target, the same is applied the other way round. Returns (world, records);
    the input world is returned unchanged (same object) when nothing applies.
    """
    mine = [p for p in planes or () if p['target_id'] == target['id']]
    measured = {b['id'] for b in boxes if b['id'] != target['id']}
    if not mine or not measured:
        return world, []
    out = []; records = []
    for o in world:
        if o.get('category') != 'carton' or o['id'] not in measured:
            out.append(o); continue
        done = False
        for k, plane in enumerate(mine):
            clipped = _clip_to_plane(o, plane, target, f"{o['id']}_adj_for_{target['id']}_{k}", setting)
            if clipped is None:
                continue
            out.append(clipped[0]); records.append(dict(clipped[1], plane_side=[plane['axis'], plane['side']],
                                                        rule='MEASURED_NEIGHBOUR_ADJACENT_CONTACT_2026_11_26'))
            done = True; break
        if not done:
            out.append(o)
    if not records:
        return world, []
    return out, records
