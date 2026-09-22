"""Observation-defined support and fixed-calibration metric checks.

Face labels must be computed before fitting the candidate under test. No
candidate-plane distance is used to select or trim validation samples.
"""
from __future__ import annotations

import numpy as np


def backproject_pixels(pixels, depth, K, *, semantics="optical_z_m"):
    K = np.asarray(K, float).reshape(3, 3)
    pixels = np.asarray(pixels, float)
    values = np.asarray(depth, float)
    rays = np.column_stack(((pixels[:, 0]-K[0, 2])/K[0, 0],
                            (pixels[:, 1]-K[1, 2])/K[1, 1], np.ones(len(pixels))))
    if semantics == "euclidean_range_m":
        rays /= np.linalg.norm(rays, axis=1)[:, None]
    elif semantics != "optical_z_m":
        raise ValueError("unsupported depth semantics or units")
    return rays * values[:, None]


def native_pixel_map(shape, native_K, final_K):
    """Integer pixel centres; resample rays without changing either K."""
    y, x = np.indices(shape, dtype=float)
    a, b = np.asarray(native_K).reshape(3, 3), np.asarray(final_K).reshape(3, 3)
    return ((x-b[0, 2])*a[0, 0]/b[0, 0]+a[0, 2],
            (y-b[1, 2])*a[1, 1]/b[1, 1]+a[1, 2])


def observation_support(depth, instance_mask, face_mask, *, erosion_px=2,
                        discontinuity_m=.02):
    """Freeze face membership, then remove image boundaries/invalid depth.

    The 20 mm neighbour jump is an acquisition discontinuity test, independent
    of the candidate. It must not replace face membership on a multiface box.
    """
    from .rgbd import _erode_mask
    depth = np.asarray(depth)
    instance, face = np.asarray(instance_mask, bool), np.asarray(face_mask, bool)
    if depth.ndim != 2 or instance.shape != depth.shape or face.shape != depth.shape:
        raise ValueError("support shape mismatch")
    selected = instance & face
    valid = np.isfinite(depth) & (depth > 0)
    raw = selected & valid
    retained = np.zeros(depth.shape, bool)
    # Outside the selected bounding box the erosion input is identically false.
    # One extra pixel includes every depth neighbour of every selected pixel;
    # neighbours need not belong to this instance/face. No resampling or cache:
    # all values are read afresh, including on independent final validation.
    rows = np.flatnonzero(selected.any(axis=1))
    columns = np.flatnonzero(selected.any(axis=0))
    window = (slice(max(0, rows[0]-1), min(depth.shape[0], rows[-1]+2)),
              slice(max(0, columns[0]-1), min(depth.shape[1], columns[-1]+2))) if len(rows) else (slice(0, 0), slice(0, 0))
    local_selected, local_valid, local_depth = selected[window], valid[window], depth[window]
    interior = _erode_mask(local_selected, erosion_px)
    jumps = np.zeros(local_depth.shape, bool)
    for axis in (0, 1):
        a, b = [slice(None)]*2, [slice(None)]*2
        a[axis], b[axis] = slice(1, None), slice(None, -1)
        a, b = tuple(a), tuple(b)
        difference = np.zeros(local_depth[a].shape, dtype=float)
        np.subtract(local_depth[a], local_depth[b], out=difference, where=local_valid[a] & local_valid[b])
        bad = (~local_valid[a] | ~local_valid[b] | (np.abs(difference) > discontinuity_m))
        jumps[a] |= bad
        jumps[b] |= bad
    local_retained = local_selected & local_valid & interior & ~jumps
    retained[window] = local_retained
    return raw, retained, {
        "selection": "FROZEN_OBSERVATION_FACE_LABEL_INTERSECT_INSTANCE",
        "candidate_plane_used_for_selection": False,
        "selected_pixels": int(local_selected.sum()),
        "excluded_invalid": int((local_selected & ~local_valid).sum()),
        "excluded_boundary": int((local_selected & local_valid & ~interior).sum()),
        "excluded_discontinuity": int((local_selected & local_valid & interior & jumps).sum()),
        "retained_pixels": int(local_retained.sum()),
    }


def check_metric_plane(normal, offset, depth, instance_mask, face_mask, K,
                       *, maximum_mean_m=.003, minimum_points=50, erosion_px=2):
    normal = np.asarray(normal, float)
    if normal.shape != (3,) or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal), 1, atol=1e-6):
        raise ValueError("plane normal must be unit length")
    if not np.isfinite(offset):
        raise ValueError("nonfinite plane offset")
    raw, retained, audit = observation_support(depth, instance_mask, face_mask, erosion_px=erosion_px)
    for name, mask in (("raw", raw), ("interior", retained), ("excluded", raw & ~retained)):
        y, x = np.nonzero(mask)
        points = backproject_pixels(np.column_stack((x, y)), np.asarray(depth)[mask], K)
        errors = np.abs(points @ normal + offset)
        audit[name] = {"count": len(errors),
                       "mean_m": float(errors.mean()) if len(errors) else None,
                       "p95_m": float(np.percentile(errors, 95)) if len(errors) else None}
    n = audit["interior"]["count"]
    mean = audit["interior"]["mean_m"]
    audit.update(status="PASS" if n >= minimum_points and mean <= maximum_mean_m else "REJECTED",
                 point_support_count=n, point_support_ratio=n/max(1, audit["selected_pixels"]),
                 plane_residual_m=mean, plane_residual_p95_m=audit["interior"]["p95_m"],
                 maximum_mean_m=maximum_mean_m)
    return audit
