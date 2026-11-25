"""Seeded visible-plane reconstruction and CAD seal coverage, in metres.

Visible face and side boundaries determine variable dimensions. Hidden surfaces
remain inferred; a single face never supplies a fabricated thickness.
No image colour, column count, world alignment or point-median grasp assumption.
"""
import json
from itertools import permutations, product
import cv2
import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from ..coordinates import rigid


def transform(T, points):
    T = rigid(T)
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


def corners(pose, size):
    return transform(pose, np.asarray(list(product((-1., 1.), repeat=3))) * np.asarray(size)/2)


class GeometryRejected(ValueError):
    def __init__(self, reason, evidence):
        super().__init__(reason)
        self.evidence = evidence


# A non-dominant plane is accepted on absolute evidence, not a share of the
# mask: a real side seen obliquely can be a few percent of the pixels.
# Values calibrated on the synthetic field-like/raycast sweeps; see
# docs/evidence/<date>_geometry_robustness/report.json.
SECOND_PLANE_MIN_SUPPORT_PIXELS = 300
ADJACENT_EDGE_BAND_M = .004
ADJACENT_EDGE_BIN_M = .002
ADJACENT_EDGE_MIN_RUN_M = .02
ADJACENT_EDGE_MIN_POINTS = 20
# Same 70 % coverage as before; the band (+/-3 px) now straddles the mask boundary.
COPLANAR_EDGE_MIN_FRACTION = .70


def adjacent_face_evidence(main, plane):
    """Is ``plane`` a face meeting ``main`` along a supported common edge?

    Both planes need support within ADJACENT_EDGE_BAND_M of their intersection
    line, overlapping along it in a continuous run (2 mm bins, one-bin gaps).
    A lower carton's recessed top or a background wall never reaches it.
    """
    n1, n2 = main['normal'], plane['normal']; d = np.cross(n1, n2); length = np.linalg.norm(d)
    if length < np.sin(np.deg2rad(45)): return dict(adjacent=False, reason='NOT_TRANSVERSE')
    d /= length
    near_plane = plane['support'][abs((plane['support']-main['center'])@n1) <= ADJACENT_EDGE_BAND_M]
    near_main = main['support'][abs((main['support']-plane['center'])@n2) <= ADJACENT_EDGE_BAND_M]
    record = dict(edge_points_on_plane=len(near_plane), edge_points_on_main=len(near_main))
    if min(len(near_plane), len(near_main)) < ADJACENT_EDGE_MIN_POINTS:
        return dict(record, adjacent=False, reason='NO_SUPPORTED_COMMON_EDGE', run_m=0.)
    a = near_plane@d; b = near_main@d; lo = max(a.min(), b.min()); hi = min(a.max(), b.max())
    if hi <= lo: return dict(record, adjacent=False, reason='COMMON_EDGE_DOES_NOT_OVERLAP', run_m=0.)
    edges = np.arange(lo, hi+ADJACENT_EDGE_BIN_M, ADJACENT_EDGE_BIN_M)
    occupied = (np.histogram(a, edges)[0] > 0) & (np.histogram(b, edges)[0] > 0)
    run = best = gap = 0
    for o in occupied:
        if o: run += 1+gap; gap = 0; best = max(best, run)
        elif run and gap == 0: gap = 1
        else: run = gap = 0
    run_m = best*ADJACENT_EDGE_BIN_M
    return dict(record, adjacent=run_m >= ADJACENT_EDGE_MIN_RUN_M, run_m=float(run_m),
                reason=None if run_m >= ADJACENT_EDGE_MIN_RUN_M else 'COMMON_EDGE_RUN_TOO_SHORT')


def _visible_planes(points, mask, seed, min_points, residual_m, diagnostics=None):
    valid = mask & np.isfinite(points).all(-1) & (points[..., 2] > .1) & (points[..., 2] < 3.)
    interior = cv2.erode(mask.astype('uint8'), np.ones((5, 5), 'uint8')).astype(bool) & valid
    rng = np.random.default_rng(seed)
    fit = points[interior]
    if len(fit) < min_points:
        raise ValueError('INSUFFICIENT_INTERIOR_DEPTH')
    fit = fit[rng.choice(len(fit), min(len(fit), 6000), replace=False)]
    original = len(fit); planes = []
    for _ in range(5):
        if len(fit) < min_points: break
        best = None
        for _ in range(180):
            a, b, c = fit[rng.choice(len(fit), 3, replace=False)]
            n = np.cross(b-a, c-a); length = np.linalg.norm(n)
            if length < 1e-9: continue
            n /= length; inliers = abs((fit-a)@n) <= residual_m
            if best is None or inliers.sum() > best.sum(): best = inliers
        if best is None or best.sum() < min_points: break
        sample = fit[best]; center = sample.mean(0)
        _, _, vh = np.linalg.svd(sample-center, full_matrices=False)
        n = vh[-1]
        # A RANSAC tolerance also admits points just around a perpendicular
        # edge. Robust refitting removes that edge bias without loosening the
        # plane residual used for actual surface evidence.
        for _ in range(3):
            distances=abs((sample-center)@n)
            cutoff=min(residual_m,max(.0001,6*float(np.median(distances))))
            refined=sample[distances<=cutoff]
            if len(refined)<min_points:break
            center=refined.mean(0);_,_,vh=np.linalg.svd(refined-center,full_matrices=False);n=vh[-1]
        if n@center > 0: n = -n
        # Merge parallel fragments from cardboard waviness; they do not provide
        # independent thickness evidence. Do not count them as a second face.
        if not any(abs(n@p['normal']) > np.cos(np.deg2rad(12)) for p in planes):
            support_mask = valid & (abs((points-center)@n) <= residual_m)
            support = points[support_mask]
            plane = dict(center=center, normal=n, support=support, support_mask=support_mask,
                         fraction=float(best.sum()/original), count=int(best.sum()),
                         rms=float(np.sqrt(np.mean(((sample-center)@n)**2))))
            if not planes:
                planes.append(plane)
            else:
                # Absolute evidence and adjacency to the dominant face; a
                # failed candidate is recorded and the search continues.
                transverse = abs(n@planes[0]['normal']) <= np.sin(np.deg2rad(10))
                check = dict(subsample_inliers=plane['count'], support_pixels=len(support),
                             rms_m=plane['rms'], transverse=bool(transverse), reason=None)
                if len(support) < SECOND_PLANE_MIN_SUPPORT_PIXELS: check['reason'] = 'INSUFFICIENT_SUPPORT_PIXELS'
                elif plane['rms'] > residual_m: check['reason'] = 'PLANE_RMS_EXCEEDS_RESIDUAL'
                elif transverse:
                    adjacency = adjacent_face_evidence(planes[0], plane)
                    check.update(adjacency, reason=None if adjacency['adjacent'] else 'NOT_ADJACENT_FACE:'+str(adjacency['reason']))
                check['accepted'] = check['reason'] is None
                plane['acceptance'] = check
                if diagnostics is not None: diagnostics.append(dict(check, normal_camera=n.tolist()))
                if check['accepted']: planes.append(plane)
        fit = fit[~best]
    if not planes or planes[0]['fraction'] < .25:
        raise ValueError('NO_TRUSTWORTHY_DOMINANT_VISIBLE_PLANE')
    return valid, planes


THICKNESS_BIN_M = .005
THICKNESS_BIN_MIN_POINTS = 3
# Calibrated 2026-11-19 (docs/evidence/20261119_geometry_robustness): 0.8
# under-reads obliquely seen field sides by up to ~14 mm (few far rows); 1.0
# lets mask bleed onto a lower top add 15-24 mm; 0.95 keeps the synthetic
# field-like errors within +/-1 mm and the saved field sides at 104-112 mm.
THICKNESS_FAR_QUANTILE = .95


def connected_side_thickness(points, main, side, face_center):
    """Thickness from the side patch connected to the front/side common edge.

    Only side-plane pixels in an 8-connected image component that reaches the
    common edge (within ADJACENT_EDGE_BAND_M of the front plane) are used. Mask
    bleed onto a lower carton's top crossing the side plane's extension is
    either disconnected or confined to a few rows along the edge direction;
    per-row far ends (5 mm bins along the common edge) and their 95th
    percentile remove it. Excluded pixels are counted, never made free space.
    """
    n = main['normal']; sm = side['support_mask'].astype('uint8')
    seed = side['support_mask'] & (abs((points-main['center'])@n) <= ADJACENT_EDGE_BAND_M)
    _, labels = cv2.connectedComponents(sm, connectivity=8)
    ids = np.unique(labels[seed]); ids = ids[ids != 0]
    keep = np.isin(labels, ids) & side['support_mask']
    pts = points[keep]
    record = dict(side_support_pixels=int(side['support_mask'].sum()), connected_pixels=int(keep.sum()),
                  excluded_disconnected_pixels=int(side['support_mask'].sum()-keep.sum()),
                  bin_m=THICKNESS_BIN_M, far_quantile=THICKNESS_FAR_QUANTILE,
                  method='SIDE_PATCH_CONNECTED_TO_COMMON_EDGE_ROBUST_FAR_END')
    if not len(pts): return 0., pts, dict(record, rows=0)
    d = np.cross(n, side['normal']); d /= np.linalg.norm(d)
    depth = (face_center-pts)@n; along = pts@d
    rows = np.floor((along-along.min())/THICKNESS_BIN_M).astype(int); far = []
    for r in np.unique(rows):
        values = depth[rows == r]
        if len(values) >= THICKNESS_BIN_MIN_POINTS: far.append(values.max())
    if not far: return 0., pts, dict(record, rows=0)
    thickness = float(np.quantile(far, THICKNESS_FAR_QUANTILE))
    return thickness, pts, dict(record, rows=len(far), thickness_m=thickness,
        raw_quantile_998_m=float(np.quantile(depth, .998)),
        points_beyond_robust_far_end=int((depth > thickness).sum()))


def _measured_face(points, plane, x, world_camera):
    n = plane['normal']; y = np.cross(n, x); center = plane['center']
    xy = np.c_[(plane['support']-center)@x, (plane['support']-center)@y]
    lo, hi = np.quantile(xy, [.002, .998], axis=0)
    size = hi-lo
    if np.min(size) <= .005: raise ValueError('DEGENERATE_VISIBLE_FACE')
    area = ConvexHull(xy).volume
    coverage = min(1., area/float(np.prod(size)))
    if coverage < .80: raise ValueError('VISIBLE_FACE_INSUFFICIENT_BOUNDARY_COVERAGE')
    face_center = center+x*(lo[0]+hi[0])/2+y*(lo[1]+hi[1])/2
    f = dict(center_m=transform(world_camera, [face_center])[0].tolist(),
             normal=(world_camera[:3,:3]@n).tolist(),
             u=(world_camera[:3,:3]@x).tolist(), v=(world_camera[:3,:3]@y).tolist(),
             size_m=size.tolist(), observed=True, face='visible_positive_z')
    # Seal coverage uses observed surface samples as well as rectangle edges.
    uv = np.c_[(plane['support']-face_center)@x, (plane['support']-face_center)@y]
    _, idx = np.unique(np.floor(uv/.002).astype(int), axis=0, return_index=True)
    f['support_points_uv_m'] = uv[idx].tolist()
    f['support_radius_m'] = .004
    return f, face_center, size, coverage


def _recover_measured(points, mask, world_camera, *, rgb, seed, min_points, residual_m, bounded=False, single_face=None,
                      continuation_policy=None):
    points = np.asarray(points, float); mask = np.asarray(mask, bool); W = rigid(world_camera)
    if points.shape != (*mask.shape, 3): raise ValueError('MASK_DEPTH_SHAPE_MISMATCH')
    candidates = []
    valid, planes = _visible_planes(points, mask, seed, min_points, residual_m, candidates)
    main = planes[0]; n = main['normal']
    side = next((p for p in planes[1:] if abs(p['normal']@n) <= np.sin(np.deg2rad(10))), None)
    if side is None:
        _, _, vh = np.linalg.svd(main['support']-main['center'], full_matrices=False)
        u = vh[0]; v = np.cross(n,u)
        xy = np.c_[(main['support']-main['center'])@u,(main['support']-main['center'])@v]
        rect = cv2.boxPoints(cv2.minAreaRect(xy.astype('float32'))).astype(float); edge = rect[1]-rect[0]
        x = (u*edge[0]+v*edge[1])/np.linalg.norm(edge)
        # The principal direction of the support samples is not exactly in the
        # fitted plane; the measured frame must be exactly orthonormal.
        x = x-n*(x@n); x /= np.linalg.norm(x)
    else:
        x = side['normal']-n*(side['normal']@n); x /= np.linalg.norm(x)
    face, face_center, size2, coverage = _measured_face(points, main, x, W)
    evidence = dict(observed_face=face, measured_face_size_m=size2.tolist(),
                    plane_rms_m=main['rms'], plane_count=len(planes), hidden_extent_measured=False,
                    second_plane_candidates=candidates)
    # No fixed carton dimensions and no guessed extrusion behind a lone face,
    # unless the single-face policy (user decision 2026-11-23) bounds the
    # unobserved thickness by the configured maximum carton extent.
    single = side is None
    if single and single_face is None: raise GeometryRejected('BOX_THICKNESS_UNOBSERVED_SHOW_SIDE', evidence)
    if mask[:2].any() or mask[-2:].any() or mask[:,:2].any() or mask[:,-2:].any():
        raise GeometryRejected('BOX_ENVELOPE_CLIPPED_BY_IMAGE', evidence)
    # A foreground occluder at the silhouette can hide the far edge. Merely
    # having a few side points is insufficient to call that edge measured.
    outside = cv2.dilate(mask.astype('uint8'),np.ones((5,5),'uint8')).astype(bool)&~mask
    outside &= np.isfinite(points).all(-1)&(points[...,2]>.1)
    occluded = np.zeros(mask.shape,bool)
    for p in ((main,) if single else (main, side)):
        denom = points@p['normal']; offset = p['center']@p['normal']
        ratio = np.divide(offset,denom,out=np.zeros_like(denom),where=abs(denom)>1e-9)
        predicted_z = points[...,2]*ratio
        # Ignore pixels whose plane intersection is beyond the observed face
        # envelope: those may belong to an unrelated plane elsewhere in view.
        near_plane = abs(denom-offset) < .025
        occluded |= outside & near_plane & (predicted_z-points[...,2]>.008)
    if not bounded and occluded.sum()>max(12, outside.sum()*.15):
        raise GeometryRejected('BOX_FAR_BOUNDARY_OCCLUDED_SHOW_SIDE_EDGE',dict(evidence,occluded_boundary_pixels=int(occluded.sum())))
    # The side face must span a finite thickness and have a trustworthy far
    # boundary; the single dominant plane cannot provide this dimension.
    if single:
        # Depth not observed: the measured box keeps a placeholder thickness
        # (positive, required by the native face binding); the envelope's back
        # side is a missing boundary, never a direct one.
        thickness = SINGLE_FACE_PLACEHOLDER_THICKNESS_M; side_points = np.zeros((0, 3))
        thickness_evidence = dict(method='NOT_OBSERVED_SINGLE_VISIBLE_FACE', policy=dict(single_face),
                                  placeholder_thickness_m=thickness)
    else:
        thickness, side_points, thickness_evidence = connected_side_thickness(points, main, side, face_center)
    evidence['thickness_evidence'] = thickness_evidence
    if not single and thickness <= max(.008,4*residual_m):
        if not (single_face and single_face.get('thin_second_plane')=='AS_SINGLE_FACE'):
            raise GeometryRejected('BOX_THICKNESS_UNRESOLVED',evidence)
        # thin_second_plane=AS_SINGLE_FACE (user decision 2026-11-25): the thin
        # plane is not a measured side; the carton is a lone face whose depth
        # the single-face policy bounds. Its samples stay accepted returns.
        thin = dict(thickness_evidence, second_plane_rms_m=side['rms'], resolvable_bound_m=max(.008,4*residual_m))
        single = True; side = None
        thickness = SINGLE_FACE_PLACEHOLDER_THICKNESS_M; side_points = np.zeros((0, 3))
        thickness_evidence = dict(method='NOT_OBSERVED_SINGLE_VISIBLE_FACE', policy=dict(single_face),
                                  placeholder_thickness_m=thickness, thin_second_plane=thin,
                                  rule='SECOND_PLANE_BELOW_RESOLVABLE_THICKNESS_HANDLED_AS_SINGLE_FACE')
        evidence['thickness_evidence'] = thickness_evidence
    y = np.cross(n,x); R = np.column_stack((x,y,n)); size = np.r_[size2,thickness]
    pose = np.eye(4); pose[:3,:3] = R; pose[:3,3] = face_center-n*thickness/2
    all_points = points[valid]; local = (all_points-pose[:3,3])@R
    surface_error = np.min(abs(abs(local)-size/2),axis=1)
    exterior = np.max(abs(local)-size/2,axis=1)
    if single:
        # Returns behind the lone face (its own top/side seen obliquely) are
        # not inconsistent: the depth is unknown. In front of the face, or
        # outside its in-plane rectangle, the same tolerance applies.
        exterior = np.maximum(np.max(abs(local[:, :2])-size[:2]/2,axis=1), local[:, 2]-size[2]/2)
        surface_error = np.zeros(len(local))
    consistent = (surface_error<=3*residual_m)&(exterior<=3*residual_m)
    if consistent.mean()<.90:
        raise GeometryRejected('MASK_MIXED_NEIGHBOR_OR_NONCUBOID',dict(evidence,surface_consistency=float(consistent.mean())))
    # An unobserved break in the same plane is not a measured carton boundary.
    coplanar_continuation = None
    continuation = outside & (abs((points-main['center'])@n)<=residual_m)
    if continuation.sum()>max(10,outside.sum()*.25):
        if rgb is None: raise GeometryRejected('MASK_BOUNDARY_CUTS_CONTINUING_PLANE',evidence)
        from .rgb_edges import boundary_edge_coverage
        seam=boundary_edge_coverage(rgb,mask,continuation);evidence['coplanar_boundary_rgb_edge']=seam
        if seam['fraction']<COPLANAR_EDGE_MIN_FRACTION:
            if continuation_policy is None:
                raise GeometryRejected('COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED',evidence)
            # User decision 2026-11-23: the face may continue past the mask;
            # its edges there stay missing boundaries (the envelope extends to
            # the next real boundary: another instance's surface, a depth step
            # or the maximum extent), never a measured edge.
            coplanar_continuation = dict(policy=dict(continuation_policy), continuation_pixels=int(continuation.sum()),
                rgb_edge=seam, rule='MISSING_EDGES_EXTEND_TO_NEXT_BOUNDARY_NOT_REJECTED')
    # The fitted side is not exactly perpendicular to the front (cardboard
    # bowing and measurement noise). Its normal alone must not determine the
    # box: a front-edge rectangle can be narrower than the actual side farther
    # back. Keep that rectangle as the observed seal patch, and retain a
    # separate observed ownership body containing EVERY accepted
    # side-plane sample. Never shrink the old body, move a contact plane, or
    # infer the hidden opposite side from a lone plane. The full-boundary and
    # mixed-instance checks above still apply before this conservative union.
    if single:
        box = _single_face_box(W, pose, size, main, face, size2, coverage, all_points, residual_m,
                               consistent, occluded, thickness_evidence, single_face)
        if coplanar_continuation:box['quality']['coplanar_continuation'] = coplanar_continuation
        return box
    front_dimensions=size.copy()
    side_x=(side_points-pose[:3,3])@x
    side_lo=min(-size[0]/2,float(side_x.min()))
    side_hi=max(size[0]/2,float(side_x.max()))
    expansion=[float(-size[0]/2-side_lo),float(side_hi-size[0]/2)]
    side_pose=pose.copy();side_size=size.copy()
    side_pose[:3,3]+=x*((side_lo+side_hi)/2);side_size[0]=side_hi-side_lo
    world_pose=W@pose
    return dict(pose=world_pose.tolist(),size_m=size.tolist(),corners_m=corners(world_pose,size).tolist(),
        observed_plane_models=[dict(center_m=transform(W,[plane['center']])[0].tolist(),
            normal=(W[:3,:3]@plane['normal']).tolist(),residual_m=residual_m,
            source='ACCEPTED_SAME_FRAME_VISIBLE_PLANE') for plane in (main,side)],
        observed_side_geometry=dict(pose=(W@side_pose).tolist(),size_m=side_size.tolist(),
            corners_m=corners(W@side_pose,side_size).tolist(),faces=[face],
            source='ALL_ACCEPTED_SIDE_PLANE_SAMPLES_NO_SIZE_PRIOR'),
        faces=[face],observed_faces=['visible_positive_z','supporting_side'],
        inferred_faces=['opposing_faces_from_measured_visible_boundaries'],
        dimension_source='RGBD_VISIBLE_FACE_AND_SIDE_BOUNDARIES_NO_SIZE_PRIOR',
        quality=dict(valid_depth_points=len(all_points),plane_inliers=main['count'],plane_rms_m=main['rms'],
            face_coverage=coverage,measured_face_size_m=size2.tolist(),measured_thickness_m=thickness,
            surface_consistency=float(consistent.mean()),hidden_extent_measured=True,
            unseen_surfaces_observed=False,side_plane_rms_m=side['rms'],
            occluded_boundary_pixels=int(occluded.sum()),
            orthogonality_error_rad=float(np.arcsin(abs(n@side['normal']))),
            front_face_and_depth_dimensions_m=front_dimensions.tolist(),
            observed_side_envelope_expansion_m=expansion,
            observed_side_envelope_source='ALL_ACCEPTED_SIDE_PLANE_SAMPLES_NO_SIZE_PRIOR',
            observed_side_envelope_points=len(side_points),thickness_evidence=thickness_evidence,
            ambiguity='180_DEGREE_FACE_SYMMETRY',all_dimensions_source='RGBD_VISIBLE_BOUNDARIES',
            **(dict(coplanar_continuation=coplanar_continuation) if coplanar_continuation else {})))


SINGLE_FACE_PLACEHOLDER_THICKNESS_M = .001


def _single_face_box(W, pose, size, main, face, size2, coverage, all_points, residual_m, consistent, occluded,
                     thickness_evidence, policy):
    world_pose = W@pose
    return dict(pose=world_pose.tolist(), size_m=size.tolist(), corners_m=corners(world_pose, size).tolist(),
        observed_plane_models=[dict(center_m=transform(W,[main['center']])[0].tolist(),
            normal=(W[:3,:3]@main['normal']).tolist(), residual_m=residual_m,
            source='ACCEPTED_SAME_FRAME_VISIBLE_PLANE')],
        faces=[face], observed_faces=['visible_positive_z'],
        inferred_faces=['opposing_faces_from_measured_visible_boundaries', 'back_face_bounded_by_max_carton_extent'],
        dimension_source='RGBD_VISIBLE_FACE_BOUNDARIES_DEPTH_UNOBSERVED_MAX_EXTENT_BOUND',
        quality=dict(valid_depth_points=len(all_points), plane_inliers=main['count'], plane_rms_m=main['rms'],
            face_coverage=coverage, measured_face_size_m=size2.tolist(), measured_thickness_m=None,
            depth_measured=False, placeholder_thickness_m=float(size[2]),
            surface_consistency=float(consistent.mean()), hidden_extent_measured=False,
            unseen_surfaces_observed=False, occluded_boundary_pixels=int(occluded.sum()),
            thickness_evidence=thickness_evidence, single_face_policy=dict(policy),
            ambiguity='180_DEGREE_FACE_SYMMETRY', all_dimensions_source='RGBD_VISIBLE_FACE_BOUNDARIES_DEPTH_MAX_EXTENT'))


def recover(points, mask, world_camera, *, rgb=None, prior=None, seed=6518,
            min_points=120, residual_m=.002, dimension_tolerance_m=.008,
            conservative_measured=False,max_carton_extent_m=.40,envelope_mask=None,
            seam_policy=None,bbox=None,shadow_policy=None,neighbors=None,colour_policy=None,single_face=None,
            continuation_policy=None,own_mask_first=False):
    if envelope_mask is not None:
        outer=np.asarray(envelope_mask,bool);inner=np.asarray(mask,bool)
        if (prior is not None or not conservative_measured or outer.shape!=inner.shape or
                np.any(inner&~outer)):
            raise ValueError('INVALID_ENVELOPE_EVIDENCE_MASK')
    if prior is None:
        box=_recover_measured(points,mask,world_camera,rgb=rgb,seed=seed,min_points=min_points,
            residual_m=residual_m,bounded=conservative_measured,single_face=single_face if conservative_measured else None,
            continuation_policy=continuation_policy if conservative_measured else None)
        if conservative_measured:
            from .measured_envelope import conservative_envelope
            return conservative_envelope(box,points,mask if envelope_mask is None else envelope_mask,
                world_camera,max_carton_extent_m,rgb=rgb,seam_policy=seam_policy,bbox=bbox,
                shadow_policy=shadow_policy,neighbors=neighbors,colour_policy=colour_policy,own_mask_first=own_mask_first)
        return box
    # Explicit known-size callers remain compatible; the physical entry never supplies a prior.
    points = np.asarray(points, float)
    mask = np.asarray(mask, bool)
    if points.shape != (*mask.shape, 3): raise ValueError('MASK_DEPTH_SHAPE_MISMATCH')
    if mask[0].any() or mask[-1].any() or mask[:,0].any() or mask[:,-1].any():
        raise ValueError('VISIBLE_BOUNDARY_CLIPPED_BY_IMAGE')
    valid = mask & np.isfinite(points).all(-1) & (points[..., 2] > .1) & (points[..., 2] < 3.)
    interior = cv2.erode(mask.astype('uint8'), np.ones((5, 5), 'uint8')).astype(bool) & valid
    fit = points[interior]
    all_points = points[valid]
    if len(fit) < min_points: raise ValueError('INSUFFICIENT_INTERIOR_DEPTH')
    # Avoid background mixed along occlusion boundaries and bound RANSAC cost.
    rng = np.random.default_rng(seed)
    fit = fit[rng.choice(len(fit), min(len(fit), 5000), replace=False)]
    best = None
    for _ in range(160):
        a,b,c = fit[rng.choice(len(fit), 3, replace=False)]
        n = np.cross(b-a, c-a); length = np.linalg.norm(n)
        if length < 1e-9: continue
        n /= length; inliers = abs((fit-a)@n) <= residual_m
        if best is None or inliers.sum() > best.sum(): best = inliers
    if best is None or best.sum() < min_points or best.mean() < .55:
        raise ValueError('NO_TRUSTWORTHY_DOMINANT_VISIBLE_PLANE')
    plane_points = fit[best]; center = plane_points.mean(0)
    _,_,vh = np.linalg.svd(plane_points-center, full_matrices=False)
    n = vh[-1]
    if n@center > 0: n = -n  # outward points toward this optical centre
    outside=cv2.dilate(mask.astype('uint8'),np.ones((5,5),'uint8')).astype(bool)&~mask
    outside &= np.isfinite(points).all(-1)&(points[...,2]>.1)
    continuation=outside&(abs((points-center)@n)<=residual_m)
    if continuation.sum()>max(10,outside.sum()*.25):
        if rgb is None:raise ValueError('MASK_BOUNDARY_CUTS_CONTINUING_PLANE')
        from .rgb_edges import boundary_edge_coverage
        if boundary_edge_coverage(rgb,mask,continuation)['fraction']<COPLANAR_EDGE_MIN_FRACTION:
            raise ValueError('COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED')
    u = vh[0]; v = np.cross(n,u)
    plane = all_points[abs((all_points-center)@n) <= residual_m]
    xy = np.c_[(plane-center)@u, (plane-center)@v]
    # Interior outliers are filtered again; rectangle uses original non-eroded
    # plane samples so erosion does not silently shrink the measured boundary.
    lo,hi = np.quantile(xy,[.005,.995],axis=0)
    xy = xy[np.all((xy>=lo)&(xy<=hi),axis=1)]
    rect = cv2.boxPoints(cv2.minAreaRect(xy.astype(np.float32))).astype(float)
    edges = np.roll(rect,-1,axis=0)-rect
    a = edges[0]; b = edges[1]; measured = np.array([np.linalg.norm(a),np.linalg.norm(b)])
    x = (u*a[0]+v*a[1])/measured[0]; y = np.cross(n,x)
    face_center = center + u*rect[:,0].mean() + v*rect[:,1].mean()
    dimensions = sorted(set(permutations(prior)),key=lambda z: np.linalg.norm(np.array(z[:2])-measured))
    size = np.asarray(dimensions[0])
    if np.max(abs(size[:2]-measured)) > dimension_tolerance_m:
        raise ValueError('VISIBLE_BOUNDARY_INCONSISTENT_WITH_EXPLICIT_DIMENSIONS')
    pose = np.eye(4); pose[:3,:3] = np.column_stack((x,y,n))
    pose[:3,3] = face_center - n*size[2]/2
    local = (all_points-pose[:3,3])@pose[:3,:3]
    exterior = np.max(abs(local)-size/2,axis=1)
    surface = np.min(abs(abs(local)-size/2),axis=1)
    consistent = (exterior <= dimension_tolerance_m) & (surface <= 2*residual_m)
    if consistent.mean() < .90: raise ValueError('MASK_MIXED_NEIGHBOR_OR_CUBOID_AMBIGUOUS')
    coverage = ConvexHull(xy).volume / float(np.prod(size[:2]))
    if coverage < .80: raise ValueError('VISIBLE_FACE_INSUFFICIENT_BOUNDARY_COVERAGE')
    rms = float(np.sqrt(np.mean(((plane_points-center)@n)**2)))
    if rms > residual_m: raise ValueError('PLANE_RESIDUAL_TOO_LARGE')
    world_camera = rigid(world_camera); world_pose = world_camera@pose
    f = dict(center_m=transform(world_camera,[face_center])[0].tolist(),
             normal=(world_camera[:3,:3]@n).tolist(),
             u=(world_camera[:3,:3]@x).tolist(),v=(world_camera[:3,:3]@y).tolist(),
             size_m=size[:2].tolist(), observed=True, face='visible_positive_z')
    return dict(pose=world_pose.tolist(), size_m=size.tolist(), corners_m=corners(world_pose,size).tolist(),
                faces=[f],observed_faces=['visible_positive_z'],
                inferred_faces=['negative_z','positive_x','negative_x','positive_y','negative_y'],
                dimension_source='EXPLICIT_DIMENSIONS_WITH_VISIBLE_BOUNDARY_CONSISTENCY',
                quality=dict(valid_depth_points=len(all_points),fit_points=len(fit),plane_inliers=int(best.sum()),
                             plane_rms_m=rms,face_coverage=float(coverage),measured_face_size_m=measured.tolist(),
                             dimension_error_m=(measured-size[:2]).tolist(),surface_consistency=float(consistent.mean()),
                             ambiguity='180_DEGREE_RECTANGLE_SYMMETRY; SQUARE_FACE_90_DEGREE_SYMMETRY',
                             hidden_extent_measured=False))


def contact_search_settings(value):
    if value is None:return None
    keys={'reference_face_roll_deg','j6_rotation_deg','lift_offsets_m','max_candidates','basis'}
    # Optional precontact standoff (2026-11-21, field cda4e336: at 30 mm the
    # joint-space approach arc swept overhanging cup lips past the envelope).
    optional={'precontact_m','single_contact'}
    if (not isinstance(value,dict) or not keys<=set(value)<=keys|optional or value['reference_face_roll_deg'] not in (0,90,180,270)
            or value['j6_rotation_deg']!=180 or type(value['max_candidates']) is not int or not 1<=value['max_candidates']<=128
            or not isinstance(value['basis'],str) or not value['basis'].strip()):
        raise ValueError('CONTACT_SEARCH_INVALID')
    lifts=value['lift_offsets_m']
    if (not isinstance(lifts,list) or not 1<=len(lifts)<=16 or lifts[0]!=0
            or any(type(x) not in (float,int) or not np.isfinite(x) or not 0<=x<=.15 for x in lifts)
            or lifts!=sorted(set(lifts))):raise ValueError('CONTACT_LIFT_OFFSETS_INVALID')
    if 'precontact_m' in value:
        d=value['precontact_m']
        if type(d) not in (float,int) or not np.isfinite(d) or not .02<=d<=.15:raise ValueError('CONTACT_PRECONTACT_INVALID')
    if 'single_contact' in value:
        # User decision 2026-11-25: one contact only (face centre, cup-array long
        # side along base Y, short side along base Z); no lift or offset variants.
        single=value['single_contact'];tilt=single.get('max_normal_from_x_deg',45.) if isinstance(single,dict) else None
        if (not isinstance(single,dict) or single.get('mode')!='FACE_CENTER_LONG_SIDE_ALONG_BASE_Y'
                or single.get('long_side_toward') not in ('+Y','-Y')
                or not str(single.get('basis','')).strip()
                or not set(single)<={'mode','basis','long_side_toward','max_normal_from_x_deg','vertical_placement','vertical_placement_basis'}
                or single.get('vertical_placement','FACE_CENTER') not in ('FACE_CENTER','LOWER_TWO_ROWS_ON_FACE')
                or ('vertical_placement' in single and single['vertical_placement']!='FACE_CENTER'
                    and not str(single.get('vertical_placement_basis','')).strip())
                or type(tilt) not in (float,int) or not np.isfinite(tilt) or not 5<=tilt<=60):
            raise ValueError('CONTACT_SINGLE_CONTACT_INVALID')
    return dict(value)


def contact_candidates(box, tool, topology, search=None, *, margin_m=.003, precontact_m=.03):
    search=contact_search_settings(search)
    if search and 'precontact_m' in search:precontact_m=float(search['precontact_m'])
    ids = topology.get('connected_cup_ids')
    all_ids = [s['id'] for s in tool['seals']]
    capped = topology.get('physically_capped_cup_ids', [])
    if (topology.get('mode') != 'COMMON_GRIPPER_VALVE' or not ids or len(set(ids))!=len(ids)
            or set(ids)&set(capped) or set(ids)|set(capped)!=set(all_ids)):
        raise ValueError('REAL_CUP_TOPOLOGY_REQUIRED_ALL_12_ACCOUNTED_FOR')
    tcp_cad = np.linalg.inv(rigid(tool['T_flange_tcp']))@rigid(tool['T_flange_tool_cad'])
    seals = [s for s in tool['seals'] if s['id'] in ids]
    rings = {s['id']:transform(tcp_cad,s['ring_cad_m']) for s in seals}
    normals = np.array([tcp_cad[:3,:3]@s['normal_cad'] for s in seals])
    normal = normals.mean(0); normal /= np.linalg.norm(normal)
    if np.max(np.linalg.norm(normals-normal,axis=1)) > .01: raise ValueError('NONCOPLANAR_TOOL_NORMALS')
    # Actual CAD x direction and seal normal define the physical suction frame.
    x = tcp_cad[:3,0]; x -= normal*(x@normal); x /= np.linalg.norm(x)
    physical = np.column_stack((x,np.cross(normal,x),normal))
    center = np.mean(np.concatenate(list(rings.values())),axis=0)
    passive = topology.get('uncovered_cups_self_close') is True
    # Computed once per call (2-second planning step 5, 2026-11-23): the TCP
    # rings as lists (every candidate stores the same values; the lists are
    # shared, never mutated), all rings stacked for valid_cups, and all lip
    # rings stacked for candidate_contact_check. Every value is unchanged.
    ring_lists = {k:p.tolist() for k,p in rings.items()}
    batch = ring_batch(rings)
    lip_points = np.concatenate(list(rings.values()))
    centers = np.array([p.mean(0) for p in rings.values()])
    projections = (centers-center)@physical
    offsets = [np.zeros(3)]
    if passive:
        # Align centres of contiguous cup groups, not independent valve commands.
        xx = np.unique(np.round(projections[:,0],7)); yy = np.unique(np.round(projections[:,1],7))
        xs = sorted(set([0., *xx, *((xx[:-1]+xx[1:])/2)]))
        ys = sorted(set([0., *yy, *((yy[:-1]+yy[1:])/2)]))
        offsets = [physical@np.array([a,b,0.]) for a in xs for b in ys]
    candidates = []
    # One k-d tree per face (step 5): the search branch below reuses it.
    trees = {}
    def face_tree(face):
        if id(face) not in trees:
            trees[id(face)] = (face, cKDTree(face['support_points_uv_m']) if face.get('support_points_uv_m') else None)
        return trees[id(face)][1]
    if search and search.get('single_contact'):
        return [single_face_center_candidate(box,tool,search,ids,rings,ring_lists,batch,lip_points,center,normal,
                                             physical,projections,passive,margin_m,precontact_m,face_tree)]
    for face in box['faces']:
        if not face['observed']: continue
        u,v,n = (np.asarray(face[k]) for k in ('u','v','normal'))
        tree=face_tree(face)
        for turn in (0,1,2,3):
            if search and turn*90!=search['reference_face_roll_deg']:continue
            angle=turn*np.pi/2; ux=u*np.cos(angle)+v*np.sin(angle)
            R=np.column_stack((ux,np.cross(-n,ux),-n))@physical.T
            for offset in offsets:
                T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(face['center_m'])-R@(center+offset)
                ok=valid_cups(T,face,rings,normal,margin_m,support_tree=tree,batch=batch)
                sealing={k:p for k,p in rings.items() if ok[k]}
                if not sealing or (not passive and len(sealing)!=len(rings)):continue
                evidence=seal_check(T,face,sealing,normal,margin_m,support_tree=tree)
                pre=T.copy();pre[:3,3]+=n*precontact_m
                candidates.append(dict(tcp_pose=T.tolist(),precontact_pose=pre.tolist(),face=face,
                    active_ids=list(sealing),seal_check=evidence,rings_tcp={k:ring_lists[k] for k in sealing},
                    suction_normal_tcp=normal.tolist(),pneumatic_mode='COMMON_VALVE_PASSIVE_SELF_CLOSING' if passive else 'COMMON_VALVE_ALL_SEAL',
                    # All connected terminal lips can meet the same target plane.
                    # Only complete supported rings count as anticipated seals;
                    # passive edge contact is never an independent cup command.
                    target_lip_contact_ids=ids,
                    target_lip_rings_tcp=dict(ring_lists),
                    passive_lip_contact_ids=[i for i in ids if i not in sealing],
                    commanded_cup_ids=ids,expected_passively_closed_ids=[i for i in ids if i not in sealing]))
    if search and candidates:
        # Rotate about the actual flange/J6 axis; leave the saved TCP and CAD
        # installation unchanged. Each shifted contact is checked anew below.
        flange_tcp=np.asarray(tool['T_flange_tcp']);turn=np.diag([-1.,-1.,1.,1.])
        spin=np.linalg.inv(flange_tcp)@turn@flange_tcp
        groups={float(h):[] for h in search['lift_offsets_m']}
        for base in sorted(candidates,key=lambda c:-len(c['active_ids'])):
            face=base['face'];n=np.asarray(face['normal']);up=np.array([0.,0.,1.])-n*n[2]
            length=float(np.linalg.norm(up));up=up/length if length>1e-9 else up
            tree=face_tree(face)
            for height,group in groups.items():
                if height and length<1e-9:continue
                T=np.asarray(base['tcp_pose'])@spin;T[:3,3]+=up*height
                ok=valid_cups(T,face,rings,normal,margin_m,support_tree=tree,batch=batch)
                sealing={k:p for k,p in rings.items() if ok[k]}
                if not sealing or (not passive and len(sealing)!=len(rings)):continue
                pre=T.copy();pre[:3,3]+=n*precontact_m
                evidence=seal_check(T,face,sealing,normal,margin_m,support_tree=tree)
                candidate=dict(base,tcp_pose=T.tolist(),precontact_pose=pre.tolist(),active_ids=list(sealing),
                    seal_check=evidence,
                    rings_tcp={k:ring_lists[k] for k in sealing},
                    passive_lip_contact_ids=[i for i in ids if i not in sealing],
                    expected_passively_closed_ids=[i for i in ids if i not in sealing],
                    contact_search=dict(search,reference_tcp_pose=base['tcp_pose'],lift_along_face_m=height,
                                        base_z_lift_m=float(up[2]*height)))
                # The seal part of candidate_contact_check is this same seal_check
                # (same pose, rings, normal and default margin/gap; the face's own
                # support points): reuse it instead of computing it again.
                if candidate_contact_check(T,candidate,seal=evidence if margin_m==.003 else None,
                                           lip_points=lip_points)['valid']:group.append(candidate)
        # Round-robin height bands prevents the maximum-seal-count truncation
        # from removing every raised, partial-cup contact before collision tests.
        from itertools import zip_longest
        candidates=[];seen=set()
        for row in zip_longest(*(groups[h] for h in groups)):
            for candidate in row:
                if candidate is None:continue
                identity=tuple(np.round(np.asarray(candidate['tcp_pose']).ravel(),8))
                if identity in seen:continue
                seen.add(identity);candidates.append(candidate)
                if len(candidates)>=search['max_candidates']:return candidates
        if candidates:return candidates
    if not candidates: raise ValueError('CONNECTED_CUP_SEALS_DO_NOT_FIT_VISIBLE_FACE')
    return sorted(candidates,key=lambda c:-len(c["active_ids"]))[:32]


def single_face_center_candidate(box,tool,search,ids,rings,ring_lists,batch,lip_points,center,normal,
                                 physical,projections,passive,margin_m,precontact_m,face_tree):
    """contact_search.single_contact (user decision 2026-11-25): the only contact.

    Face: the observed face whose normal is closest to base +/-X (within
    max_normal_from_x_deg), i.e. the face spanned by base Y and Z. The centre
    of the connected cup array goes to the face centre; the array's long side
    lies along base Y projected into the face, the short side along base Z.
    Of the two such orientations (180 deg about the suction normal) the
    configured ``long_side_toward`` (+Y or -Y) says where the positive
    physical long axis of the array points. Every seal and contact check is
    unchanged; no lift or offset variants are generated.
    """
    single=search['single_contact'];limit=np.cos(np.deg2rad(single.get('max_normal_from_x_deg',45.)))
    faces=[f for f in box['faces'] if f['observed']]
    if not faces:raise ValueError('SINGLE_CONTACT_NO_OBSERVED_FACE')
    face=max(faces,key=lambda f:abs(np.asarray(f['normal'],float)[0]))
    n=np.asarray(face['normal'],float);n=n/np.linalg.norm(n)
    if abs(n[0])<limit:
        raise ValueError(f'SINGLE_CONTACT_NO_FACE_SPANNING_BASE_Y_Z: 可见面法向与基座 X 轴夹角 {np.rad2deg(np.arccos(min(1.,abs(n[0])))):.1f}°，'
                         f'超过 {single.get("max_normal_from_x_deg",45.)}°')
    extent=np.ptp(projections[:,:2],axis=0)
    if abs(extent[0]-extent[1])<1e-6:raise ValueError('SINGLE_CONTACT_CUP_ARRAY_HAS_NO_LONG_SIDE')
    long_physical=0 if extent[0]>extent[1] else 1
    a=-n;Y=np.array([0.,1.,0.]);Yp=Y-n*(n@Y);Yp/=np.linalg.norm(Yp)
    long_world=(1. if single['long_side_toward']=='+Y' else -1.)*Yp
    ux=long_world if long_physical==0 else np.cross(long_world,a)
    R=np.column_stack((ux,np.cross(a,ux),a))@physical.T
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(face['center_m'],float)-R@center
    placement=single.get('vertical_placement','FACE_CENTER');shift=0.
    if placement=='LOWER_TWO_ROWS_ON_FACE':
        # User decision 2026-11-25 ("将吸具沿着+Z上移，用下两排吸盘来吸，最上面一排吸盘
        # 空着"): the array moves up in the face plane until the centre of its
        # two lowest cup rows is at the face centre; the top row is left free.
        up=np.array([0.,0.,1.])-n*n[2];length=float(np.linalg.norm(up))
        if length<1e-6:raise ValueError('SINGLE_CONTACT_FACE_HAS_NO_UP_DIRECTION')
        up/=length
        heights=np.array([(R@p.mean(0)+T[:3,3])@up for p in rings.values()])
        rows=np.unique(np.round(heights,3))
        if len(rows)<3:raise ValueError('SINGLE_CONTACT_CUP_ARRAY_HAS_FEWER_THAN_THREE_ROWS')
        lower=heights[np.round(heights,3)<=rows[1]]
        shift=float(np.asarray(face['center_m'],float)@up-lower.mean())
        T[:3,3]+=up*shift
    tree=face_tree(face)
    ok=valid_cups(T,face,rings,normal,margin_m,support_tree=tree,batch=batch)
    sealing={k:p for k,p in rings.items() if ok[k]}
    if not sealing or (not passive and len(sealing)!=len(rings)):
        raise ValueError('CONNECTED_CUP_SEALS_DO_NOT_FIT_VISIBLE_FACE: 唯一吸取位姿（面中心、长边沿 Y）没有吸盘完整落在可见面上')
    evidence=seal_check(T,face,sealing,normal,margin_m,support_tree=tree)
    pre=T.copy();pre[:3,3]+=n*precontact_m
    short_world=R@physical[:,1-long_physical]
    candidate=dict(tcp_pose=T.tolist(),precontact_pose=pre.tolist(),face=face,active_ids=list(sealing),seal_check=evidence,
        rings_tcp={k:ring_lists[k] for k in sealing},suction_normal_tcp=normal.tolist(),
        pneumatic_mode='COMMON_VALVE_PASSIVE_SELF_CLOSING' if passive else 'COMMON_VALVE_ALL_SEAL',
        target_lip_contact_ids=ids,target_lip_rings_tcp=dict(ring_lists),passive_lip_contact_ids=[i for i in ids if i not in sealing],
        commanded_cup_ids=ids,expected_passively_closed_ids=[i for i in ids if i not in sealing],
        contact_search=dict(search,reference_tcp_pose=T.tolist(),lift_along_face_m=0.,base_z_lift_m=0.,
                            single_contact=dict(single,long_side_world=long_world.tolist(),short_side_world=short_world.tolist(),
                                                long_from_base_y_deg=float(np.rad2deg(np.arccos(min(1.,abs(long_world[1]))))),
                                                short_from_base_z_deg=float(np.rad2deg(np.arccos(min(1.,abs(short_world[2]))))),
                                                face_normal=n.tolist(),array_centre_at_face_centre=placement=='FACE_CENTER',
                                                vertical_placement=placement,up_shift_in_face_m=shift)))
    check=candidate_contact_check(T,candidate,seal=evidence if margin_m==.003 else None,lip_points=lip_points)
    if not check['valid']:raise ValueError('SINGLE_CONTACT_CONTACT_CHECK_FAILED: '+json.dumps(check.get('reason') or check.get('failure'),ensure_ascii=False,default=str)[:300])
    return candidate


def ring_batch(rings):
    """(keys, stacked ring points, start offsets, sizes) of ``rings`` for valid_cups."""
    keys=list(rings);arrays=[np.asarray(rings[k]) for k in keys];sizes=[len(a) for a in arrays]
    if not keys or min(sizes)<1:raise ValueError('EMPTY_SEAL_RING')
    return keys,np.concatenate(arrays),np.r_[0,np.cumsum(sizes)[:-1]].astype(int),sizes


def valid_cups(tcp, face, rings, suction_normal_tcp, margin=.003, max_gap=.0004, support_tree=None, batch=None):
    """Per cup, exactly ``seal_check(tcp, face, {k: ring}, ...)['valid']`` for every k of ``rings``.

    Candidate generation asks this for each cup at each offset (thousands of
    calls). The arithmetic of every cup is unchanged (same transform, the
    same products with the face axes, the same gap, margin and angle tests);
    the rigid-transform validation is done once per pose and the k-d tree
    distance queries of all cups go in one call (each point's nearest
    distance is independent of the batch). Step 5 (2026-11-23): the rings are
    transformed and projected as one stacked array (``batch``, from
    ``ring_batch(rings)``; built here when absent). Every row of a stacked
    matrix product is the same sum as the per-ring product (checked bitwise
    on 3000 random poses and on every saved record's candidates); the per-cup
    tests are exact maxima / comparisons over each ring's rows.
    """
    T=rigid(tcp);n,u,v=(np.asarray(face[k]) for k in ('normal','u','v'));center=np.asarray(face['center_m'])
    half=np.asarray(face['size_m'])/2-margin
    angle=float(np.arccos(np.clip(-n@(np.asarray(tcp)[:3,:3]@suction_normal_tcp),-1,1)))
    if support_tree is None and face.get('support_points_uv_m'): support_tree=cKDTree(face['support_points_uv_m'])
    keys,stacked,offsets,sizes=ring_batch(rings) if batch is None else batch
    if keys!=list(rings):raise ValueError('RING_BATCH_MISMATCH')
    points=stacked@T[:3,:3].T+T[:3,3]-center
    pu=points@u;pv=points@v;gaps=points@n
    inside=(abs(pu)<=half[0]+1e-9)&(abs(pv)<=half[1]+1e-9)
    inside=np.logical_and.reduceat(inside,offsets);largest_gap=np.maximum.reduceat(abs(gaps),offsets)
    # A cup is valid only if every test passes, so the k-d tree is asked only
    # about cups that pass the exact rectangle, gap and angle tests, and the
    # margin band only about cups whose ring is covered; queries are bounded
    # just above the support radius (a farther point answers inf, a point
    # within it its exact nearest distance), so each answer to "distance <=
    # support radius" is unchanged. Step 5, 2026-11-23.
    ok=inside&(largest_gap<=max_gap)&(angle<=.01)
    if support_tree is not None and ok.any():
        radius=face['support_radius_m'];sizes=np.asarray(sizes)
        def covered(rows_per_point,points_of):
            # Max nearest distance per still-valid cup, over that cup's rows only.
            chosen=np.flatnonzero(ok);start=offsets[chosen]*rows_per_point;count=sizes[chosen]*rows_per_point
            rows=np.concatenate([np.arange(a,a+c) for a,c in zip(start,count)])
            d=support_tree.query(points_of(rows),distance_upper_bound=support_bound(radius))[0]
            ok[chosen]=np.maximum.reduceat(d,np.r_[0,np.cumsum(count)[:-1]])<=radius
        uv=np.c_[pu,pv]
        covered(1,lambda rows:uv[rows])
        if face.get('check_observed_margin') and ok.any():
            angles=np.arange(16)*2*np.pi/16;ring=margin*np.c_[np.cos(angles),np.sin(angles)][None,:,:]
            # Band row r is (uv[:,None,:]+ring).reshape(-1,2)[r] = uv[r//16]+ring[0,r%16].
            covered(16,lambda rows:uv[rows//16]+ring[0,rows%16])
    return {k:bool(o) for k,o in zip(keys,ok)}


def support_bound(radius):
    """k-d query bound just above the support radius: every point at most ``radius`` away is found."""
    return float(radius)*(1+1e-6)+1e-9


def seal_check(tcp, face, rings_tcp, suction_normal_tcp, margin=.003, max_gap=.0004, support_tree=None):
    n,u,v=(np.asarray(face[k]) for k in ('normal','u','v'));center=np.asarray(face['center_m'])
    p=transform(tcp,np.concatenate(list(rings_tcp.values())))-center
    gaps=p@n;uv=np.c_[p@u,p@v];half=np.asarray(face['size_m'])/2-margin
    angle=float(np.arccos(np.clip(-n@(np.asarray(tcp)[:3,:3]@suction_normal_tcp),-1,1)))
    if support_tree is None and face.get('support_points_uv_m'): support_tree=cKDTree(face['support_points_uv_m'])
    # Bounded queries (support_bound): the same answer to "distance <= support radius".
    covered=support_tree is None or bool(np.max(support_tree.query(uv,distance_upper_bound=support_bound(face['support_radius_m']))[0])<=face['support_radius_m'])
    if support_tree is not None and face.get('check_observed_margin'):
        # Test a band around the actual CAD contour, including concave holes
        # and occlusion boundaries. Rectangle area is not seal evidence.
        angles=np.arange(16)*2*np.pi/16
        band=uv[:,None,:]+margin*np.c_[np.cos(angles),np.sin(angles)][None,:,:]
        covered=covered and bool(np.max(support_tree.query(band.reshape(-1,2),distance_upper_bound=support_bound(face['support_radius_m']))[0])<=face['support_radius_m'])
    return dict(valid=bool(covered and np.all(abs(uv)<=half+1e-9) and np.max(abs(gaps))<=max_gap and angle<=.01),
                observed_surface_covered=covered,minimum_edge_margin_m=float(np.min(np.asarray(face['size_m'])/2-abs(uv))),
                maximum_gap_m=float(np.max(abs(gaps))),normal_error_rad=angle,
                connected_cup_ids=list(rings_tcp))


def candidate_contact_check(tcp,candidate,seal=None,lip_points=None):
    """Actual FK: seal coverage plus nonpenetration of every permitted lip.

    A small tilt can leave central sealing cups valid but push a passive edge
    lip through the plane. The native path guard and actual-feedback guard must
    both include those nonsealing lips, without counting them as suction seals.
    ``seal``: the caller's result of exactly ``seal_check(tcp, face,
    candidate['rings_tcp'], candidate['suction_normal_tcp'])`` (candidate
    generation computed it for this pose already); a copy is extended.
    ``lip_points``: candidate generation's stacked float64 array of exactly
    ``target_lip_rings_tcp`` in its key order (the same values the lists
    hold), instead of converting the lists again for every pose.
    """
    face=candidate['face']
    check=dict(seal) if seal is not None else seal_check(tcp,face,candidate['rings_tcp'],candidate['suction_normal_tcp'])
    rings=candidate['target_lip_rings_tcp']
    if set(rings)!=set(candidate['target_lip_contact_ids']):raise ValueError('CONTACT_LIP_GEOMETRY_MISMATCH')
    points=np.concatenate(list(rings.values())) if lip_points is None else lip_points
    p=transform(tcp,points)-np.asarray(face['center_m'])
    gaps=p@np.asarray(face['normal'])
    valid=bool(np.isfinite(gaps).all() and np.min(gaps)>=-.0004)
    check['terminal_lip_contact']=dict(valid=valid,minimum_plane_gap_m=float(np.min(gaps)),ids=list(rings))
    check['valid']=check['valid'] and valid
    return check


def obb_overlap_depth(Ta,half_a,Tb,half_b):
    """Separating-axis test for two oriented boxes.

    Returns the smallest overlap along all 15 candidate axes: positive is a
    certain penetration of at least that depth, zero or negative is touching
    or separated.
    """
    Ta=np.asarray(Ta,float);Tb=np.asarray(Tb,float);Ra=Ta[:3,:3];Rb=Tb[:3,:3]
    half_a=np.asarray(half_a,float);half_b=np.asarray(half_b,float);t=Tb[:3,3]-Ta[:3,3]
    axes=[Ra[:,i] for i in range(3)]+[Rb[:,i] for i in range(3)]+[np.cross(Ra[:,i],Rb[:,j]) for i in range(3) for j in range(3)]
    depth=np.inf
    for axis in axes:
        length=np.linalg.norm(axis)
        if length<1e-9:continue
        axis=axis/length
        depth=min(depth,np.abs(Ra.T@axis)@half_a+np.abs(Rb.T@axis)@half_b-abs(axis@t))
    return float(depth)
