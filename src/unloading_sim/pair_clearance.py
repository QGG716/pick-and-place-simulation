"""Uninflated surface queries and shared POC acceptance (no simulator import)."""
from itertools import product
import numpy as np


def obb_surface_distance(first, second):
    """Euclidean distance of disjoint boxes: vertex/face and edge/edge features.

    SAT is used only for intersection, never as Euclidean separation. No
    penetration depth is inferred from overlapping proxy boxes.
    """
    if first.signed_distance_obb(second) <= 0.:
        return 0.
    vertices = [b.corners() for b in (first, second)]
    distances = []
    for points, box in ((vertices[0], second), (vertices[1], first)):
        local = (points-box.center) @ box.rotation
        distances.append(np.min(np.sum(np.maximum(np.abs(local)-box.half_extents, 0.)**2, axis=1)))
    edges = []
    for box in (first, second):
        starts, ends = [], []
        for axis in range(3):
            others = [i for i in range(3) if i != axis]
            for signs in product((-1., 1.), repeat=2):
                a = np.zeros(3); a[others] = np.asarray(signs)*box.half_extents[others]
                b = a.copy(); a[axis] = -box.half_extents[axis]; b[axis] = box.half_extents[axis]
                starts.append(box.center+box.rotation@a); ends.append(box.center+box.rotation@b)
        edges.append((np.asarray(starts), np.asarray(ends)))
    p, pe = edges[0]; q, qe = edges[1]
    u = (pe-p)[:, None, :]; v = (qe-q)[None, :, :]; w = p[:, None, :]-q[None, :, :]
    aa = np.sum(u*u, axis=2); bb = np.sum(u*v, axis=2); cc = np.sum(v*v, axis=2)
    dd = np.sum(u*w, axis=2); ee = np.sum(v*w, axis=2)
    denom = aa*cc-bb*bb
    valid = denom > np.finfo(float).eps*aa*cc
    ss = np.divide(bb*ee-cc*dd, denom, out=np.zeros_like(denom), where=valid)
    tt = np.divide(aa*ee-bb*dd, denom, out=np.zeros_like(denom), where=valid)
    valid &= (ss >= 0) & (ss <= 1) & (tt >= 0) & (tt <= 1)
    interior = np.sum((w+ss[..., None]*u-tt[..., None]*v)**2, axis=2)
    distances.append(np.min(np.where(valid, interior, np.inf)))
    # Segment boundary minima (one parameter is 0 or 1).
    for s in (0., 1.):
        t = np.clip((ee+s*bb)/cc, 0., 1.)
        distances.append(np.min(np.sum((w+s*u-t[..., None]*v)**2, axis=2)))
    for t in (0., 1.):
        s = np.clip((t*bb-dd)/aa, 0., 1.)
        distances.append(np.min(np.sum((w+s[..., None]*u-t*v)**2, axis=2)))
    return float(np.sqrt(max(0., min(distances))))


def classify_pair_distance(distance, required, *, intersection=None, proxy=False,
                           stage, pair, policy, query_method, geometry_scope,
                           permission_source=None):
    """Contact and missing data fail closed; 1 nm only resolves gap comparisons."""
    finite = distance is not None and np.isfinite(distance)
    if permission_source is not None:
        classification = "ALLOWED_STAGE_CONTACT"
    elif not finite or intersection is None:
        classification = "UNKNOWN"
    elif intersection or distance <= 0.:
        classification = "PROXY_GEOMETRY_INTERSECTION" if proxy else "GEOMETRY_CONTACT_OR_INTERSECTION"
    elif distance + 1e-9 < required:
        classification = "CLEARANCE_INSUFFICIENT"
    else:
        classification = "CLEAR"
    return {"classification": classification, "accepted": classification in {"CLEAR", "ALLOWED_STAGE_CONTACT"},
            "stage": stage, "pair": list(pair), "geometry_scope": geometry_scope,
            "query_method": query_method, "geometry_intersection_confirmed": intersection,
            "original_cad_intersection_confirmed": False if proxy else intersection,
            "surface_distance_m": float(distance) if finite and distance >= 0 else None,
            "required_pair_clearance_m": required, "numerical_gap_tolerance_m": 1e-9,
            "contact_permission_source": permission_source, "policy_fingerprint": policy.fingerprint}


def obb_pair_evidence(first, second, policy, margin=0., *, stage="unspecified", proxy=False):
    distance = obb_surface_distance(first, second)
    return classify_pair_distance(distance, policy.pair_clearance("external", margin),
        intersection=distance == 0., proxy=proxy, stage=stage, pair=(first.name, second.name), policy=policy,
        query_method="UNINFLATED_OBB_VERTEX_FACE_EDGE_EDGE_EUCLIDEAN",
        geometry_scope="CURRENT_PROXY_OBB_NOT_ORIGINAL_CAD" if proxy else "FROZEN_SCENE_BOX_SOLIDS")


def obb_pair_failure(first, second, policy, margin, *, stage="unspecified", proxy=False, reason="PAIR_CLEARANCE"):
    if not policy.poc_pair_clearance:
        return {"reason": reason, "pair": [first.name, second.name]} if first.intersects_obb(second, margin=margin) else None
    required = policy.pair_clearance("external", margin)
    # AABB axis separation is a safe lower bound, only used to skip distant pairs.
    a = np.abs(first.rotation)@first.half_extents; b = np.abs(second.rotation)@second.half_extents
    if np.any(np.abs(first.center-second.center)-a-b > required+1e-9):
        return None
    evidence = obb_pair_evidence(first, second, policy, margin, stage=stage, proxy=proxy)
    return None if evidence["accepted"] else {"reason": reason, **evidence}
