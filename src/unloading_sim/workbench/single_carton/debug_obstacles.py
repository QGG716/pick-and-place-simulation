"""Debug mode: RGB-D obstacles next to the target are left out of planning.

User decision 2026-11-24 ("做软件调试模式"): field runs were repeatedly stopped
before any candidate search because the target's conservative envelope
overlapped (or came within the 2.4 mm pair clearance of) point-cloud derived
objects next to it: observed return clusters, neighbour shadows, adjacent
contact planes and unresolved carton volumes. To run one complete cycle, the
optional policy ``debug_near_target_obstacles`` removes, before planning, every
such object whose distance to a searched target's envelope may be at most
``near_m``, and (optional ``front_corridor_m``) every such object inside the
approach corridor: the target's front face (+Z of the envelope) padded by
``near_m``, extruded outward along its normal by ``front_corridor_m`` (flying
points and returns in front of the grasp face block the tool's approach).
Default absent/DISABLED: unchanged.

Not removed: other detected carton envelopes, observed/stack supports and the
target's own detach supports, workspace boundaries, layout CAD, fixed
geometry, robot and tool solids. Every pair rule and threshold is unchanged
for the objects that remain. This is a debug premise chosen by the user, not
evidence that the removed space is free; every removed object is recorded.
"""
from __future__ import annotations

import numpy as np

from .geometry import obb_overlap_depth

MODE = 'REMOVE_RGBD_OBSTACLES_NEAR_TARGET_DEBUG'
CATEGORIES = frozenset({'observed', 'neighbor_shadow', 'adjacent_contact', 'unresolved_carton_volume'})
NEAR_MAX_M = 0.10
CORRIDOR_MAX_M = 0.30


def policy(site):
    """The validated policy, or None when absent/DISABLED."""
    value = (site or {}).get('debug_near_target_obstacles')
    if value is None:
        return None
    if not isinstance(value, dict) or value.get('mode') not in ('DISABLED', MODE):
        raise ValueError('DEBUG_NEAR_TARGET_OBSTACLES_POLICY_INVALID')
    if value['mode'] == 'DISABLED':
        return None
    near = value.get('near_m')
    if not isinstance(near, (int, float)) or isinstance(near, bool) or not 0 <= near <= NEAR_MAX_M:
        raise ValueError('DEBUG_NEAR_TARGET_OBSTACLES_NEAR_M_INVALID')
    corridor = value.get('front_corridor_m', 0.)
    if not isinstance(corridor, (int, float)) or isinstance(corridor, bool) or not 0 <= corridor <= CORRIDOR_MAX_M:
        raise ValueError('DEBUG_NEAR_TARGET_OBSTACLES_FRONT_CORRIDOR_M_INVALID')
    if not str(value.get('basis', '')).strip():
        raise ValueError('DEBUG_NEAR_TARGET_OBSTACLES_BASIS_REQUIRED')
    return dict(mode=MODE, near_m=float(near), front_corridor_m=float(corridor), basis=str(value['basis']))


def distance_lower_bound(box, obj):
    """A lower bound of the distance between the target envelope and ``obj``.

    Separating-axis projections never exceed the true distance, so an object
    whose bound is above ``near_m`` is certainly farther; one at or below may
    be nearer (it is removed, conservatively for this debug purpose only).
    """
    T = np.asarray(box['pose'], float)
    half = np.asarray(box['size_m'], float) / 2
    if 'size' in obj:
        return -obb_overlap_depth(T, half, np.asarray(obj['pose'], float), np.asarray(obj['size'], float) / 2)
    mesh = obj.get('convex_mesh')
    if not mesh:
        return None
    P = np.asarray(obj.get('pose', np.eye(4).tolist()), float)
    v = np.asarray(mesh['vertices'], float) @ P[:3, :3].T + P[:3, 3]
    local = (v - T[:3, 3]) @ T[:3, :3]
    gaps = np.maximum(local.min(0) - half, -half - local.max(0))
    return float(gaps.max())


def front_corridor(box, near, depth):
    """(pose, half extents) of the prism in front of the target's +Z face."""
    T = np.asarray(box['pose'], float)
    half = np.asarray(box['size_m'], float) / 2
    C = T.copy()
    C[:3, 3] = T[:3, 3] + T[:3, 2] * (half[2] + depth / 2)
    return C, np.array([half[0] + near, half[1] + near, depth / 2])


def apply(site, world, targets, supports_of):
    """Return (world, evidence); world is unchanged without the policy.

    ``supports_of(world, box)`` gives the target's detach supports, which keep
    their contact semantics and are never removed.
    """
    p = policy(site)
    if p is None:
        return world, None
    keep_ids = set()
    for box in targets:
        keep_ids |= {s['id'] for s in supports_of(world, box)}
    removed = {}
    for box in targets:
        for o in world:
            if o.get('category') not in CATEGORIES or o['id'] in keep_ids or o['id'] == box['id']:
                continue
            bound = distance_lower_bound(box, o)
            if bound is None:
                continue
            rule = 'NEAR_ENVELOPE' if bound <= p['near_m'] else None
            if rule is None and p['front_corridor_m'] > 0:
                corridor = dict(pose=(C := front_corridor(box, p['near_m'], p['front_corridor_m']))[0], size_m=2 * C[1])
                if distance_lower_bound(corridor, o) <= 0:
                    rule = 'FRONT_CORRIDOR'
            if rule is None:
                continue
            entry = removed.setdefault(o['id'], dict(id=o['id'], category=o.get('category'), near_targets=[]))
            entry['near_targets'].append(dict(target=box['id'], rule=rule, distance_lower_bound_m=bound))
    kept = [o for o in world if o['id'] not in removed]
    evidence = dict(policy=p, status='USER_DEBUG_PREMISE_NOT_SENSOR_FREE_SPACE',
                    targets=[b['id'] for b in targets], removed=list(removed.values()),
                    removed_count=len(removed), kept_count=len(kept))
    return kept, evidence
