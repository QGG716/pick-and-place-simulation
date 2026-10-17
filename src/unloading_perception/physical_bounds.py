"""Finite RGB-D boundary constraints, separate from frozen plane fitting.

Axes are ROW vectors: q = camera_xyz @ axes.T; camera_xyz = q @ axes.
Raw boundary pixels never enter plane fitting or its independent validation.
No scene dimensions, object IDs, GT, or learned completion are accepted here.
"""
from __future__ import annotations

import numpy as np

from .final_geometry import SIGNS, project
from .metric_support import backproject_pixels


FACE_IDS = (([0, 3, 7, 4], [1, 2, 6, 5]),
            ([0, 1, 5, 4], [3, 2, 6, 7]),
            ([0, 1, 2, 3], [4, 5, 6, 7]))


def raw_boundary_regions(groups, depth, mask, K, *, search_px=6, residual_m=.006):
    """Connected measured surfaces near this SAM mask, for boundary use ONLY.

    Closest-plane assignment disambiguates adjacent surfaces. A component must
    intersect that plane's frozen support; an unrelated plane cannot seed it.
    The six-pixel halo is the existing boundary search footprint, not a mask
    extension used by fitting/validation. Its end is never a physical edge.
    """
    import cv2
    domain = cv2.dilate(mask.astype(np.uint8), np.ones((2*search_px+1,)*2, np.uint8)).astype(bool)
    y, x = np.nonzero(domain & np.isfinite(depth) & (depth > 0))
    points = backproject_pixels(np.c_[x, y], depth[y, x], K)
    errors = np.array([abs(points @ g['normal']+g['offset']) for g in groups])
    owner = errors.argmin(0)
    result = []
    for i, group in enumerate(groups):
        candidate = np.zeros(mask.shape, np.uint8)
        good = (owner == i) & (errors[i] <= residual_m)
        candidate[y[good], x[good]] = 1
        _, components = cv2.connectedComponents(candidate, connectivity=8)
        touching = np.unique(components[group['region'] & (candidate > 0)])
        touching = touching[touching != 0]
        region = np.isin(components, touching) if len(touching) else np.zeros(mask.shape, bool)
        result.append({**group, 'region': region})
    return result, domain


def _scan_outline(point, outward, group, depth, domain, K, *, search_px, no_hit):
    """Locate a near/far transition, rather than certify a nearby guessed edge."""
    h, w = depth.shape
    n, d = group['normal'], group['offset']

    def pixel(t):
        return np.rint(point+t*outward).astype(int)

    def inside(xy):
        return 0 <= xy[0] < w and 0 <= xy[1] < h

    def expected(xy):
        ray = backproject_pixels(np.array([xy]), [1.], K)[0]
        den = n @ ray
        return -d/den if abs(den) > 1e-9 else np.nan

    # Start on the connected measured face near the proposed line. No jump
    # over a hole or an unrelated surface is allowed after that start.
    starts = [t for t in range(-search_px, search_px+1)
              if inside(pixel(t)) and group['region'][pixel(t)[1], pixel(t)[0]]]
    base = dict(candidate_pixel=point.tolist(), search_start_px=-search_px,
                search_halo_px=search_px, support_label=group['label'])
    if not starts:
        return dict(base, kind='UNCLASSIFIED_BOUNDARY', reason='NO_CONNECTED_FACE_NEAR_CANDIDATE')
    t = min(starts, key=abs)
    last = pixel(t)
    # The finite image and mask halo bound the walk; no box-size prior is used.
    while t < h+w:
        t += 1
        xy = pixel(t)
        if np.array_equal(xy, last):
            continue
        if not inside(xy) or xy[0] in (0, w-1) or xy[1] in (0, h-1):
            return dict(base, kind='IMAGE_CROP_BOUNDARY', reason='MEASURED_FACE_REACHES_IMAGE_CROP',
                        inner_pixel=last.tolist(), search_end_pixel=xy.tolist())
        if group['region'][xy[1], xy[0]]:
            last = xy
            continue
        break
    first_out = xy.copy()
    # A bevel can separate stable planar support and the actual silhouette.
    # Follow continuous raw depth only within the bounded SAM halo. A gradual
    # departure from an infinite plane is NOT an external silhouette: it may
    # be a rounded edge or the seam to a neighbouring carton.
    for delta in range(h+w):
        xy = pixel(t+delta)
        if not inside(xy):
            return dict(base, kind='IMAGE_CROP_BOUNDARY', reason='OUTLINE_BRACKET_LEAVES_IMAGE', inner_pixel=last.tolist())
        z = depth[xy[1], xy[0]]
        prediction = expected(xy)
        row = dict(base, inner_pixel=last.tolist(), outer_pixel=xy.tolist(),
                   search_end_pixel=xy.tolist(), first_departure_pixel=first_out.tolist())
        if not np.isfinite(prediction) or prediction <= 0:
            return dict(row, kind='UNCLASSIFIED_BOUNDARY', reason='INVALID_PLANE_RAY')
        if not np.isfinite(z) or z <= 0:
            if not (no_hit and np.isposinf(z)):
                return dict(row, kind='DEPTH_HOLE_BOUNDARY', reason='INVALID_DEPTH_IS_NOT_FREE_SPACE')
        elif z < prediction-.01:
            return dict(row, kind='OCCLUSION_BOUNDARY', reason='NEARER_SURFACE_BLOCKS_DIMENSION')
        else:
            previous_error = depth[last[1], last[0]]-expected(last)
            jump = float(z-prediction-previous_error)
            row['gradient_corrected_depth_jump_m'] = jump
            # Same acquisition discontinuity threshold as observation_support.
            if jump <= .02:
                if not domain[xy[1], xy[0]]:
                    return dict(row, kind='MASK_OR_LABEL_LIMIT', reason='NO_PHYSICAL_JUMP_BEFORE_SAM_HALO_LIMIT')
                last = xy
                continue
        # Keep the actual bracket, including its uncertainty. Merely finding
        # background at +6 px does not make the original line the boundary.
        location = (last+xy)/2
        ray = backproject_pixels(location[None], [1.], K)[0]
        xyz = ray*(-d/(n @ ray))
        rays = backproject_pixels(np.array([last, xy]), [1., 1.], K)
        bracket = rays*(-d/(rays @ n))[:, None]
        return dict(row, kind='PHYSICAL_OUTLINE', reason='LOCATED_CONNECTED_SURFACE_TO_FARTHER_DEPTH',
                    pixel=location.tolist(), point_m=xyz.tolist(), bracket_m=bracket.tolist())
    kind = 'MASK_OR_LABEL_LIMIT' if not domain[first_out[1], first_out[0]] else 'UNCLASSIFIED_BOUNDARY'
    return dict(base, kind=kind, reason='NO_LOCAL_PHYSICAL_TRANSITION', inner_pixel=last.tolist(),
                first_departure_pixel=first_out.tolist(), search_end_pixel=xy.tolist())


def locate_edge(polygon, edge_index, group, depth, domain, K, *, search_px=6, no_hit=False):
    a, b = polygon[edge_index], polygon[(edge_index+1) % 4]
    outward = np.array([(b-a)[1], -(b-a)[0]])
    if outward @ ((a+b)/2-polygon.mean(0)) < 0:
        outward *= -1
    outward /= max(np.linalg.norm(outward), 1e-12)
    samples = [_scan_outline(a+t*(b-a), outward, group, depth, domain, K,
                             search_px=search_px, no_hit=no_hit) for t in np.linspace(.15, .85, 15)]
    return dict(samples=samples, support_label=group['label'], edge_index=edge_index,
                search_px=search_px, outward_pixel_direction=outward.tolist(),
                candidate_segment_pixels=[a.tolist(), b.tolist()])


def resolve_bounds(groups, axes, low, high, observed, depth, mask, K, config):
    """Constrain each of six bounds by a measured plane or localized outline.

    A free bound needs 10/15 coherent finite outline samples from at least one
    observed face. Other faces may be occluded. All supporting constraints must
    agree within their measured bracket plus the unchanged 6 mm plane tolerance.
    Unobserved ranges are containment inequalities, never finite dimensions.
    """
    raw_groups, domain = raw_boundary_regions(groups, depth, mask, K,
        residual_m=config.maximum_p95_m)
    by_label = {g['label']: g for g in raw_groups}
    corners = (low+(SIGNS+1)/2*(high-low)) @ axes
    edges, constraints = [], []
    for (axis, side), group in observed.items():
        ids = FACE_IDS[axis][side == 'high']
        polygon = project(corners[ids], K)
        for edge_index in range(4):
            end = [ids[edge_index], ids[(edge_index+1) % 4]]
            other = next(i for i in range(3) if i != axis and SIGNS[end[0], i] == SIGNS[end[1], i])
            bound_side = 'low' if SIGNS[end[0], other] < 0 else 'high'
            if (other, bound_side) in observed:
                continue  # measured-plane intersections are checked separately
            edge = locate_edge(polygon, edge_index, by_label[group['label']], depth, domain, K,
                               no_hit=config.positive_infinity_is_no_hit)
            edge.update(axis=axis, side=side, bound_axis=other, bound_side=bound_side)
            physical = [s for s in edge['samples'] if s['kind'] == 'PHYSICAL_OUTLINE']
            if len(physical) >= 10:
                values = np.array([s['point_m'] for s in physical]) @ axes[other]
                uncertainty = np.array([np.ptp(np.array(s['bracket_m']) @ axes[other])/2 for s in physical])
                median = float(np.median(values))
                good = abs(values-median) <= config.maximum_p95_m+uncertainty
                if good.sum() >= 10:
                    interval = [float(np.median(values[good]-uncertainty[good])),
                                float(np.median(values[good]+uncertainty[good]))]
                    constraint = dict(axis=other, side=bound_side, value_m=float(np.median(values[good])),
                        interval_m=interval, source='LOCATED_PHYSICAL_OUTLINE', support_label=group['label'],
                        edge_index=edge_index, coherent_samples=int(good.sum()), total_samples=15)
                    constraints.append(constraint)
                    edge['constraint'] = constraint
            edge['status'] = 'CONSTRAINED' if 'constraint' in edge else 'UNRESOLVED'
            edges.append(edge)
    bounds = []
    for axis in range(3):
        for side in ('low', 'high'):
            item = dict(axis=axis, side=side, value_m=None, status='UNRESOLVED', sources=[])
            if (axis, side) in observed:
                g = observed[axis, side]
                value = -g['offset']/g['sign']
                item.update(value_m=float(value), interval_m=[float(value)]*2, status='CONSTRAINED',
                    sources=[dict(source='MEASURED_PLANE', support_label=g['label'])])
            else:
                evidence = [c for c in constraints if c['axis'] == axis and c['side'] == side]
                item['sources'] = evidence
                if evidence:
                    lower = max(c['interval_m'][0] for c in evidence)
                    upper = min(c['interval_m'][1] for c in evidence)
                    if lower <= upper+config.maximum_p95_m:
                        item.update(value_m=float(np.median([c['value_m'] for c in evidence])),
                                    interval_m=[min(lower, upper), max(lower, upper)], status='CONSTRAINED')
                    else:
                        item['status'] = 'CONFLICTING_PHYSICAL_CONSTRAINTS'
            bounds.append(item)
    return dict(axis_convention='row axes; q = camera_xyz @ axes.T', axes_camera=axes.tolist(),
        bounds=bounds, edges=edges, complete=all(b['status'] == 'CONSTRAINED' for b in bounds),
        boundary_support_policy='RAW_DEPTH_CONNECTED_TO_FROZEN_FACE_IN_SAM_PLUS_6PX_HALO',
        plane_fit_support_changed=False, boundary_regions=raw_groups)


def validate_complete_constraints(record, depth, mask, K):
    """Re-read raw depth and localize all free bounds independently of the box.

    The seed is the original frozen-label candidate, not the final cuboid being
    checked. Thus changing tangential dimensions cannot move the measurement.
    """
    from .metric_faces import MetricFitConfig, _support_mask, _shared_edge_support
    evidence = record['boundary_constraints']
    axes = np.asarray(evidence['axes_camera'])
    low, high = np.asarray(evidence['silhouette_optimized_bounds_m'])
    config = MetricFitConfig(**record['config'])
    groups, observed = [], {}
    for face in record['camera_facing_faces']:
        n = np.asarray(face['plane_normal'])
        axis = int(face['axis_index'])
        sign = float(np.sign(n @ axes[axis]))
        group = dict(label=face['support_label'], normal=n, offset=face['plane_offset_m'], axis=axis, sign=sign,
            region=_support_mask(record['frozen_support_regions'][str(face['support_label'])], depth.shape))
        value = -group['offset']/sign
        side = 'low' if abs(value-low[axis]) < abs(value-high[axis]) else 'high'
        groups.append(group)
        observed[axis, side] = group
    fresh = resolve_bounds(groups, axes, low, high, observed, depth, mask, K, config)
    if not fresh['complete']:
        raise ValueError('FINAL_UNRESOLVED_PHYSICAL_BOUNDS')
    coordinates = np.asarray(record['corners_3d']) @ axes.T
    residuals = []
    for bound in fresh['bounds']:
        i, side = bound['axis'], bound['side']
        value = float(coordinates[:, i].min() if side == 'low' else coordinates[:, i].max())
        lo, hi = bound['interval_m']
        if value < lo-config.maximum_p95_m or value > hi+config.maximum_p95_m:
            raise ValueError('FINAL_TANGENTIAL_BOUND_OUTSIDE_MEASURED_CONSTRAINT')
        residuals.append(dict(axis=i, side=side, signed_constraint_residual_m=value-bound['value_m']))
    raw = {g['label']:g for g in fresh['boundary_regions']}
    keys = list(observed)
    connected = {observed[keys[0]]['label']}
    joints = []
    for i, (axis, side) in enumerate(keys):
        for other, other_side in keys[i+1:]:
            ids = sorted(set(FACE_IDS[axis][side == 'high']) & set(FACE_IDS[other][other_side == 'high']))
            if len(ids) != 2:
                continue
            first, second = observed[axis, side]['label'], observed[other, other_side]['label']
            check = _shared_edge_support(np.asarray(record['corners_3d'])[ids], raw[first], raw[second],
                depth, K, maximum_residual_m=config.maximum_p95_m, maximum_mean_m=config.maximum_mean_m)
            joints.append((first, second, check['supported']))
    for _ in keys:
        for first, second, supported in joints:
            if supported and connected.intersection((first, second)):
                connected.update((first, second))
    if len(connected) != len(keys):
        raise ValueError('FINAL_SHARED_EDGE_LOST_ACTUAL_SUPPORT')
    return dict(status='PASS', checks=residuals, measurement='RAW_DEPTH_RELOCALIZED_FROM_FROZEN_LABEL_SEED',
                validation_bounds_independent_of_final_candidate=True)
