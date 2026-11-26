"""Unresolved volumes at a target's rest pose (user decision 2026-11-26).

"不能治标不治本 ... 只要是某个箱子能看到完整的一个面，即便没恢复出此箱的3维
几何，即便没算出该箱是否有支撑，都让该箱能进入运动规划流程", with the option
chosen for the overlap: "重叠部分都忽略".

Before this rule a target whose conservative envelope overlapped (or came
closer than the pair clearance to) an unresolved volume at rest - a lower
carton's possible body when its support was not recognised, a neighbour
shadow, an unresolved carton volume - was refused before any candidate or
path search (GEOMETRY_UNCERTAINTY_OVERLAP/BLOCKED_AT_REST), and without that
gate the detach start would still collide.

With mode IGNORE_OVERLAP_PAYLOAD_DETACH, while one target is searched:

* every unresolved volume within ADJ_BAND_M of the target envelope loses the
  part inside the envelope (ignored: no collision with anything);
* the rest is split by the envelope's six face planes; each piece lies beyond
  one face. Below the bottom face it becomes a support of the target
  (``confirmed_stack_support`` bound to the bottom plane), beside a side face
  an ``adjacent_contact`` bound to that face - the existing rules: only the
  attached payload, only at detach, may touch it, no payload corner more than
  0.4 mm past the plane. Pieces above the top face stay ordinary obstacles
  (detach moves up through them);
* robot and tool keep the full pair clearance against every piece; measured
  cartons, observed returns, fixed geometry and workspace limits are not
  touched; the target envelope is unchanged.

This is a field premise chosen by the user, not sensor evidence that the
ignored volume is free: a real object there would be touched by the payload.
"""
import numpy as np

from .convex_clip import box_solid, clip, mesh
from .adjacent_contact import ADJ_BAND_M

MODE = 'IGNORE_OVERLAP_PAYLOAD_DETACH'
RULE = 'UNRESOLVED_REST_OVERLAP_IGNORED_2026_11_26'
# Everything at the rest pose except measured cartons (their own adjacent
# rule, 2026-11-26), fixed geometry and the workspace limits: possible
# occupancy (shadows, unresolved volumes, other targets' supports) and
# observed returns / observed supports alike - "即便没算出该箱是否有支撑".
UNRESOLVED_CATEGORIES = ('neighbor_shadow', 'unresolved_carton_volume', 'unknown', 'observed', 'observed_support')
KEEP_CATEGORIES = ('workspace_boundary', 'carton', 'measured_fixed')


def policy(geometry):
    value = (geometry or {}).get('unresolved_rest_overlap_policy')
    if value is None or (isinstance(value, dict) and value.get('mode') == 'DISABLED'):
        return None
    if (not isinstance(value, dict) or value.get('mode') != MODE or set(value) - {'mode', 'basis'}
            or not isinstance(value.get('basis'), str) or not value['basis'].strip()):
        raise ValueError('UNRESOLVED_REST_OVERLAP_POLICY_INVALID')
    return dict(value)


def unresolved(o, target_id):
    """Possible occupancy that is not this target's own support / contact."""
    if o.get('supports_target_id') == target_id:
        return False
    category = o.get('category')
    if category in KEEP_CATEGORIES:
        return False
    if category in UNRESOLVED_CATEGORIES:
        return True
    original = o.get('original_category')
    if category == 'adjacent_contact':      # bound to another target
        return original in UNRESOLVED_CATEGORIES or bool(o.get('geometry_uncertain'))
    if category == 'confirmed_stack_support' and o.get('rule') == RULE:
        return True
    return category == 'confirmed_stack_support' and bool(o.get('geometry_uncertain'))


def _vertices(o):
    T = np.asarray(o['pose'], float)
    if 'convex_mesh' in o:
        v = np.asarray(o['convex_mesh']['vertices'], float)
        return v@T[:3, :3].T+T[:3, 3]
    half = np.asarray(o['size'], float)/2
    corners = np.array([[x, y, z] for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])*half
    return corners@T[:3, :3].T+T[:3, 3]


def target_world(world, target, setting, band_m=ADJ_BAND_M, keep_ids=()):
    """(world, records) for ``target``; the same world object when nothing applies.

    ``keep_ids``: objects the existing rules already accept as this target's
    supports / contacts (planner.detach_supports); they are left unchanged
    (field 9707b9cc: re-binding the target's own observed support split it into
    side contacts and blocked the outward detach)."""
    if setting is None:
        return world, []
    solid = box_solid(target['pose'], target['size_m'])
    eq = solid['equations']                  # rows (m, e): m.x + e <= 0 inside the envelope
    # Bottom first, then the sides, the top last: a piece beyond a side and
    # the top (an edge) is a side contact; only what lies over the footprint
    # above the top face stays an obstacle.
    eq = eq[np.argsort(eq[:, 2])]
    out = []; records = []
    for o in world:
        if (o['id'] == target['id'] or o['id'] in keep_ids or ('size' not in o and 'convex_mesh' not in o)
                or not unresolved(o, target['id'])):
            out.append(o); continue
        v = _vertices(o)
        values = v@eq[:, :3].T+eq[:, 3]
        if np.any(np.all(values > band_m, axis=0)):   # separated from the envelope by more than the band
            out.append(o); continue
        overlapped = not np.any(np.all(values >= 0, axis=0))
        remaining = v; pieces = []
        for j, plane in enumerate(eq):
            part = clip(remaining, -plane) if remaining is not None else None   # beyond face j
            if part is not None:
                pieces.append((j, part))
            remaining = clip(remaining, plane) if remaining is not None else None
            if remaining is None:
                break
        kept = []; bound = []
        for k, (j, part) in enumerate(pieces):
            m, e = eq[j, :3], float(eq[j, 3])
            n = -m                              # into the target; the piece lies at n.x <= e
            # Native binding (eco65_single_carton.h): a SUPPORT plane faces up
            # (n.z >= cos 10 deg, category confirmed_stack_support), an
            # ADJACENT_CONTACT plane is vertical (|n.z| <= sin 10 deg); the
            # object's highest point along n lies exactly on the plane. A
            # piece short of the plane is extended to it (never shrunk), as
            # in adjacent_contact._clip_to_plane.
            kind = 'SUPPORT' if n[2] >= np.cos(np.deg2rad(10)) else (
                'ADJACENT_CONTACT' if abs(n[2]) <= np.sin(np.deg2rad(10)) else None)
            if kind is not None and float((part@n).max()) < e-1e-12:
                lift = part[part@n >= float((part@n).max())-1e-9]
                part = np.unique(np.round(np.r_[part, lift+np.outer(e-lift@n, n)], 12), axis=0)
            try:
                from scipy.spatial import ConvexHull
                part = part[ConvexHull(part).vertices]
                shape = mesh(part)
            except Exception:
                continue                      # zero-volume sliver: not an obstacle solid
            ident = f"{o['id']}_rest_{target['id']}_{k}"
            if kind is None:
                # Above the target (or a steep face): it stays an obstacle for
                # every body, the payload included (detach moves through it).
                kept.append(dict(id=ident, pose=np.eye(4).tolist(), convex_mesh=shape,
                                 category=o.get('original_category') or o.get('category'),
                                 geometry_uncertain=True, original_id=o['id'], support_contact_qualified=False))
                continue
            plane = dict(normal_world=n.tolist(), offset_m=float(np.max(part@n)))
            kept.append(dict(id=ident, pose=np.eye(4).tolist(), convex_mesh=shape,
                             category='confirmed_stack_support' if kind == 'SUPPORT' else 'adjacent_contact',
                             supports_target_id=target['id'], support_basis=setting['basis'], support_plane=plane,
                             original_id=o['id'], original_category=o.get('category'), geometry_uncertain=True,
                             support_contact_qualified=False, rule=RULE))
            bound.append(dict(id=ident, kind=kind, face=int(j)))
        out.extend(kept)
        records.append(dict(original_id=o['id'], original_category=o.get('category'), overlapped=bool(overlapped),
                            ignored_inside_envelope=remaining is not None, pieces=[p['id'] for p in kept],
                            bound=bound, faces=[int(j) for j, _ in pieces]))
    if not records:
        return world, []
    return out, records
