"""Seeded visible-plane reconstruction and CAD seal coverage, in metres.

Visible face and side boundaries determine variable dimensions. Hidden surfaces
remain inferred; a single face never supplies a fabricated thickness.
No image colour, column count, world alignment or point-median grasp assumption.
"""
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


def _visible_planes(points, mask, seed, min_points, residual_m):
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
        if best is None or best.sum() < min_points or best.sum()/original < .04: break
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
            support = points[valid & (abs((points-center)@n) <= residual_m)]
            planes.append(dict(center=center, normal=n, support=support,
                               fraction=float(best.sum()/original), count=int(best.sum()),
                               rms=float(np.sqrt(np.mean(((sample-center)@n)**2)))))
        fit = fit[~best]
    if not planes or planes[0]['fraction'] < .25:
        raise ValueError('NO_TRUSTWORTHY_DOMINANT_VISIBLE_PLANE')
    return valid, planes


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


def _recover_measured(points, mask, world_camera, *, rgb, seed, min_points, residual_m):
    points = np.asarray(points, float); mask = np.asarray(mask, bool); W = rigid(world_camera)
    if points.shape != (*mask.shape, 3): raise ValueError('MASK_DEPTH_SHAPE_MISMATCH')
    valid, planes = _visible_planes(points, mask, seed, min_points, residual_m)
    main = planes[0]; n = main['normal']
    side = next((p for p in planes[1:] if abs(p['normal']@n) <= np.sin(np.deg2rad(10))), None)
    if side is None:
        _, _, vh = np.linalg.svd(main['support']-main['center'], full_matrices=False)
        u = vh[0]; v = np.cross(n,u)
        xy = np.c_[(main['support']-main['center'])@u,(main['support']-main['center'])@v]
        rect = cv2.boxPoints(cv2.minAreaRect(xy.astype('float32'))); edge = rect[1]-rect[0]
        x = (u*edge[0]+v*edge[1])/np.linalg.norm(edge)
    else:
        x = side['normal']-n*(side['normal']@n); x /= np.linalg.norm(x)
    face, face_center, size2, coverage = _measured_face(points, main, x, W)
    evidence = dict(observed_face=face, measured_face_size_m=size2.tolist(),
                    plane_rms_m=main['rms'], plane_count=len(planes), hidden_extent_measured=False)
    # No fixed carton dimensions and no guessed extrusion behind a lone face.
    if side is None: raise GeometryRejected('BOX_THICKNESS_UNOBSERVED_SHOW_SIDE', evidence)
    if mask[:2].any() or mask[-2:].any() or mask[:,:2].any() or mask[:,-2:].any():
        raise GeometryRejected('BOX_ENVELOPE_CLIPPED_BY_IMAGE', evidence)
    # A foreground occluder at the silhouette can hide the far edge. Merely
    # having a few side points is insufficient to call that edge measured.
    outside = cv2.dilate(mask.astype('uint8'),np.ones((5,5),'uint8')).astype(bool)&~mask
    outside &= np.isfinite(points).all(-1)&(points[...,2]>.1)
    occluded = np.zeros(mask.shape,bool)
    for p in (main, side):
        denom = points@p['normal']; offset = p['center']@p['normal']
        ratio = np.divide(offset,denom,out=np.zeros_like(denom),where=abs(denom)>1e-9)
        predicted_z = points[...,2]*ratio
        # Ignore pixels whose plane intersection is beyond the observed face
        # envelope: those may belong to an unrelated plane elsewhere in view.
        near_plane = abs(denom-offset) < .025
        occluded |= outside & near_plane & (predicted_z-points[...,2]>.008)
    if occluded.sum()>max(12, outside.sum()*.15):
        raise GeometryRejected('BOX_FAR_BOUNDARY_OCCLUDED_SHOW_SIDE_EDGE',dict(evidence,occluded_boundary_pixels=int(occluded.sum())))
    depth = (face_center-side['support'])@n
    # The side face must span a finite thickness and have a trustworthy far
    # boundary; the single dominant plane cannot provide this dimension.
    thickness = float(np.quantile(depth,.998))
    if thickness <= max(.008,4*residual_m):
        raise GeometryRejected('BOX_THICKNESS_UNRESOLVED',evidence)
    y = np.cross(n,x); R = np.column_stack((x,y,n)); size = np.r_[size2,thickness]
    pose = np.eye(4); pose[:3,:3] = R; pose[:3,3] = face_center-n*thickness/2
    all_points = points[valid]; local = (all_points-pose[:3,3])@R
    surface_error = np.min(abs(abs(local)-size/2),axis=1)
    exterior = np.max(abs(local)-size/2,axis=1)
    consistent = (surface_error<=3*residual_m)&(exterior<=3*residual_m)
    if consistent.mean()<.90:
        raise GeometryRejected('MASK_MIXED_NEIGHBOR_OR_NONCUBOID',dict(evidence,surface_consistency=float(consistent.mean())))
    # An unobserved break in the same plane is not a measured carton boundary.
    continuation = outside & (abs((points-main['center'])@n)<=residual_m)
    if continuation.sum()>max(10,outside.sum()*.25):
        if rgb is None: raise GeometryRejected('MASK_BOUNDARY_CUTS_CONTINUING_PLANE',evidence)
        edge_pixels=cv2.dilate(cv2.Canny(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),30,80),np.ones((5,5),'uint8'))>0
        if np.mean(edge_pixels[continuation])<.70: raise GeometryRejected('COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED',evidence)
    world_pose=W@pose
    return dict(pose=world_pose.tolist(),size_m=size.tolist(),corners_m=corners(world_pose,size).tolist(),
        faces=[face],observed_faces=['visible_positive_z','supporting_side'],
        inferred_faces=['opposing_faces_from_measured_visible_boundaries'],
        dimension_source='RGBD_VISIBLE_FACE_AND_SIDE_BOUNDARIES_NO_SIZE_PRIOR',
        quality=dict(valid_depth_points=len(all_points),plane_inliers=main['count'],plane_rms_m=main['rms'],
            face_coverage=coverage,measured_face_size_m=size2.tolist(),measured_thickness_m=thickness,
            surface_consistency=float(consistent.mean()),hidden_extent_measured=True,
            unseen_surfaces_observed=False,side_plane_rms_m=side['rms'],
            orthogonality_error_rad=float(np.arcsin(abs(n@side['normal']))),
            ambiguity='180_DEGREE_FACE_SYMMETRY',all_dimensions_source='RGBD_VISIBLE_BOUNDARIES'))


def recover(points, mask, world_camera, *, rgb=None, prior=None, seed=6518,
            min_points=120, residual_m=.002, dimension_tolerance_m=.008):
    if prior is None:
        return _recover_measured(points,mask,world_camera,rgb=rgb,seed=seed,min_points=min_points,residual_m=residual_m)
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
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
        edges=cv2.dilate(cv2.Canny(gray,30,80),np.ones((5,5),'uint8'))>0
        if np.mean(edges[continuation])<.70:raise ValueError('COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED')
    u = vh[0]; v = np.cross(n,u)
    plane = all_points[abs((all_points-center)@n) <= residual_m]
    xy = np.c_[(plane-center)@u, (plane-center)@v]
    # Interior outliers are filtered again; rectangle uses original non-eroded
    # plane samples so erosion does not silently shrink the measured boundary.
    lo,hi = np.quantile(xy,[.005,.995],axis=0)
    xy = xy[np.all((xy>=lo)&(xy<=hi),axis=1)]
    rect = cv2.boxPoints(cv2.minAreaRect(xy.astype(np.float32)))
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


def contact_candidates(box, tool, topology, *, margin_m=.003, precontact_m=.03):
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
    for face in box['faces']:
        if not face['observed']: continue
        u,v,n = (np.asarray(face[k]) for k in ('u','v','normal'))
        tree=cKDTree(face['support_points_uv_m']) if face.get('support_points_uv_m') else None
        for turn in (0,1,2,3):
            angle=turn*np.pi/2; ux=u*np.cos(angle)+v*np.sin(angle)
            R=np.column_stack((ux,np.cross(-n,ux),-n))@physical.T
            for offset in offsets:
                T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(face['center_m'])-R@(center+offset)
                sealing={k:p for k,p in rings.items() if seal_check(T,face,{k:p},normal,margin_m,support_tree=tree)['valid']}
                if not sealing or (not passive and len(sealing)!=len(rings)):continue
                evidence=seal_check(T,face,sealing,normal,margin_m,support_tree=tree)
                pre=T.copy();pre[:3,3]+=n*precontact_m
                candidates.append(dict(tcp_pose=T.tolist(),precontact_pose=pre.tolist(),face=face,
                    active_ids=list(sealing),seal_check=evidence,rings_tcp={k:p.tolist() for k,p in sealing.items()},
                    suction_normal_tcp=normal.tolist(),pneumatic_mode='COMMON_VALVE_PASSIVE_SELF_CLOSING' if passive else 'COMMON_VALVE_ALL_SEAL',
                    # All connected terminal lips can meet the same target plane.
                    # Only complete supported rings count as anticipated seals;
                    # passive edge contact is never an independent cup command.
                    target_lip_contact_ids=ids,
                    target_lip_rings_tcp={k:p.tolist() for k,p in rings.items()},
                    passive_lip_contact_ids=[i for i in ids if i not in sealing],
                    commanded_cup_ids=ids,expected_passively_closed_ids=[i for i in ids if i not in sealing]))
    if not candidates: raise ValueError('CONNECTED_CUP_SEALS_DO_NOT_FIT_VISIBLE_FACE')
    return sorted(candidates,key=lambda c:-len(c["active_ids"]))[:32]


def seal_check(tcp, face, rings_tcp, suction_normal_tcp, margin=.003, max_gap=.0004, support_tree=None):
    n,u,v=(np.asarray(face[k]) for k in ('normal','u','v'));center=np.asarray(face['center_m'])
    p=transform(tcp,np.concatenate(list(rings_tcp.values())))-center
    gaps=p@n;uv=np.c_[p@u,p@v];half=np.asarray(face['size_m'])/2-margin
    angle=float(np.arccos(np.clip(-n@(np.asarray(tcp)[:3,:3]@suction_normal_tcp),-1,1)))
    if support_tree is None and face.get('support_points_uv_m'): support_tree=cKDTree(face['support_points_uv_m'])
    covered=support_tree is None or bool(np.max(support_tree.query(uv)[0])<=face['support_radius_m'])
    return dict(valid=bool(covered and np.all(abs(uv)<=half+1e-9) and np.max(abs(gaps))<=max_gap and angle<=.01),
                observed_surface_covered=covered,minimum_edge_margin_m=float(np.min(np.asarray(face['size_m'])/2-abs(uv))),
                maximum_gap_m=float(np.max(abs(gaps))),normal_error_rad=angle,
                connected_cup_ids=list(rings_tcp))


def candidate_contact_check(tcp,candidate):
    """Actual FK: seal coverage plus nonpenetration of every permitted lip.

    A small tilt can leave central sealing cups valid but push a passive edge
    lip through the plane. The native path guard and actual-feedback guard must
    both include those nonsealing lips, without counting them as suction seals.
    """
    face=candidate['face']
    check=seal_check(tcp,face,candidate['rings_tcp'],candidate['suction_normal_tcp'])
    rings=candidate['target_lip_rings_tcp']
    if set(rings)!=set(candidate['target_lip_contact_ids']):raise ValueError('CONTACT_LIP_GEOMETRY_MISMATCH')
    p=transform(tcp,np.concatenate(list(rings.values())))-np.asarray(face['center_m'])
    gaps=p@np.asarray(face['normal'])
    valid=bool(np.isfinite(gaps).all() and np.min(gaps)>=-.0004)
    check['terminal_lip_contact']=dict(valid=valid,minimum_plane_gap_m=float(np.min(gaps)),ids=list(rings))
    check['valid']=check['valid'] and valid
    return check
