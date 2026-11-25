"""Whole flexible cups and passive cups facing neighbouring objects.

User decisions of 2026-11-25:

* "吸盘从唇边到最后到吸具的连接器，都是柔性对，刚性芯是不对的": with
  ``neighbor_lip_contact.cup_body_basis`` the cup body behind each connected
  lip (the collision of the same CAD cup part other than its terminal band) may
  touch the same named neighbouring objects as the lips. ``cup_bodies`` binds
  that map from the tool CAD; connectors, plates, the rest of the tool and the
  robot keep every pair rule.
* "现在各个吸盘只有统一控制来一起吸，没有单独控制。所以在规划时，应考虑要吸盘尽量
  不附着到邻箱上": with ``passive_cup_neighbour`` every candidate records which
  of its passive (non-sealing) connected cups face a neighbouring object at the
  contact pose, and candidates whose passive cups face none rank first. It is a
  preference: no candidate is removed and no check changes.
"""
from __future__ import annotations

import numpy as np

MODE = 'PREFER_PASSIVE_CUPS_OFF_NEIGHBOURS'
# Ring offsets along the suction axis (+ into the target face, - toward the
# tool). A passive lip meets a neighbour that lies at, or up to the lip band
# and cup body in front of, the target face plane; a surface a little behind
# the plane is reached by the extended lip.
DEFAULT_TOWARD_TOOL_M = .02
DEFAULT_INTO_FACE_M = .005


def cup_bodies(tool, connected, terminal_band_by_contour, tool_links):
    """{seal id: cup body collision id} for every connected cup."""
    seals = {s['id']: s for s in tool['seals']}
    out = {}
    for cid in connected:
        seal = seals.get(cid)
        if seal is None:
            raise ValueError('CUP_BODY_UNKNOWN_SEAL:' + str(cid))
        lip = terminal_band_by_contour[cid]
        if lip != seal.get('contact_band'):
            raise ValueError('CUP_BODY_LIP_BINDING_CHANGED:' + str(cid))
        rows = [c for c in tool['collisions'] if c['source_part'] == seal['component'] and c['id'] != lip]
        if len(rows) != 1:
            raise ValueError('CUP_BODY_NOT_UNIQUE:' + str(cid))
        body = rows[0]['id']
        if body not in tool_links or body in out.values():
            raise ValueError('CUP_BODY_NOT_A_TOOL_LINK:' + str(body))
        out[cid] = body
    return out


def policy(config):
    """The validated passive_cup_neighbour policy, or None when absent/DISABLED."""
    value = (config or {}).get('passive_cup_neighbour')
    if value is None:
        return None
    if not isinstance(value, dict) or value.get('mode') not in ('DISABLED', MODE):
        raise ValueError('PASSIVE_CUP_NEIGHBOUR_POLICY_INVALID')
    if value['mode'] == 'DISABLED':
        return None
    if not str(value.get('basis', '')).strip():
        raise ValueError('PASSIVE_CUP_NEIGHBOUR_BASIS_REQUIRED')
    if not set(value) <= {'mode', 'basis', 'toward_tool_m', 'into_face_m'}:
        raise ValueError('PASSIVE_CUP_NEIGHBOUR_UNKNOWN_FIELD')
    out = dict(mode=MODE, basis=str(value['basis']))
    for key, default, limit in (('toward_tool_m', DEFAULT_TOWARD_TOOL_M, .06), ('into_face_m', DEFAULT_INTO_FACE_M, .02)):
        d = value.get(key, default)
        if type(d) not in (int, float) or not np.isfinite(d) or not 0 <= d <= limit:
            raise ValueError('PASSIVE_CUP_NEIGHBOUR_DEPTH_INVALID:' + key)
        out[key] = float(d)
    if (config or {}).get('planning_module') == 'NATIVE':
        raise ValueError('PASSIVE_CUP_NEIGHBOUR_PYTHON_FLOW_ONLY')
    return out


def _inside(points, obj):
    """Boolean mask: points inside the object's box or convex mesh."""
    T = np.asarray(obj.get('pose', np.eye(4).tolist()), float)
    if 'size' in obj:
        local = (points - T[:3, 3]) @ T[:3, :3]
        return np.all(np.abs(local) <= np.asarray(obj['size'], float) / 2, axis=1)
    mesh = obj.get('convex_mesh')
    if not mesh:
        return np.zeros(len(points), bool)
    from scipy.spatial import Delaunay
    v = np.asarray(mesh['vertices'], float) @ T[:3, :3].T + T[:3, 3]
    try:
        return Delaunay(v).find_simplex(points) >= 0
    except Exception:  # degenerate (flat) hull: never claims a point
        return np.zeros(len(points), bool)


def facing(candidate, world, box, p):
    """Passive connected cups whose lip ring faces a neighbouring object.

    The ring of each passive cup at the candidate's contact pose, and its
    centre, are swept along the suction axis from ``into_face_m`` into the face
    to ``toward_tool_m`` toward the tool; any sample inside an object other than
    the target counts. Returns the evidence dict.
    """
    T = np.asarray(candidate['tcp_pose'], float)
    axis = T[:3, 2] / np.linalg.norm(T[:3, 2])
    rings = candidate['target_lip_rings_tcp']
    steps = np.linspace(-p['toward_tool_m'], p['into_face_m'], 6)
    others = [o for o in world if o['id'] != box['id']]
    cups = []
    for cid in candidate['expected_passively_closed_ids']:
        ring = np.asarray(rings[cid], float)
        local = np.vstack([ring, ring.mean(0)])
        base = local @ T[:3, :3].T + T[:3, 3]
        samples = (base[None, :, :] + steps[:, None, None] * axis).reshape(-1, 3)
        hits = []
        for o in others:
            lo = samples.min(0); hi = samples.max(0)
            c = np.asarray(o.get('pose', np.eye(4).tolist()), float)[:3, 3]
            # Cheap reject: an object whose centre is far beyond any extent.
            reach = np.linalg.norm(np.asarray(o.get('size', [1., 1., 1.]), float)) if 'size' in o else np.inf
            if np.isfinite(reach) and np.any((c < lo - reach) | (c > hi + reach)):
                continue
            if _inside(samples, o).any():
                hits.append(o['id'])
        if hits:
            cups.append(dict(cup=cid, objects=hits))
    return dict(policy=p, passive_cups=list(candidate['expected_passively_closed_ids']),
                facing_neighbour=cups, count=len(cups),
                scope='PREFERENCE_ONLY_NO_CANDIDATE_REMOVED_NO_CHECK_CHANGED')
