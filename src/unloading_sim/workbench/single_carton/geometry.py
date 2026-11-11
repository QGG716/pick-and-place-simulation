"""Seeded visible-plane reconstruction and CAD seal coverage, in metres.

A hidden face is explicitly a size-prior inference, never a depth measurement.
No image colour, column count, world alignment or point-median grasp assumption.
"""
from itertools import permutations, product
import cv2
import numpy as np
from scipy.spatial import ConvexHull
from ..coordinates import rigid


def transform(T, points):
    T = rigid(T)
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


def corners(pose, size):
    return transform(pose, np.asarray(list(product((-1., 1.), repeat=3))) * np.asarray(size)/2)


def recover(points, mask, world_camera, *, rgb=None, prior=(.15, .10, .10), seed=6518,
            min_points=120, residual_m=.002, dimension_tolerance_m=.008):
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
        raise ValueError('VISIBLE_BOUNDARY_INCONSISTENT_WITH_150_100_100_PRIOR')
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
                dimension_source='150x100x100_MM_PRIOR_WITH_VISIBLE_BOUNDARY_CONSISTENCY',
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
        for turn in (0,1,2,3):
            angle=turn*np.pi/2; ux=u*np.cos(angle)+v*np.sin(angle)
            R=np.column_stack((ux,np.cross(-n,ux),-n))@physical.T
            for offset in offsets:
                T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(face['center_m'])-R@(center+offset)
                sealing={k:p for k,p in rings.items() if seal_check(T,face,{k:p},normal,margin_m)['valid']}
                if not sealing or (not passive and len(sealing)!=len(rings)):continue
                evidence=seal_check(T,face,sealing,normal,margin_m)
                pre=T.copy();pre[:3,3]+=n*precontact_m
                candidates.append(dict(tcp_pose=T.tolist(),precontact_pose=pre.tolist(),face=face,
                    active_ids=list(sealing),seal_check=evidence,rings_tcp={k:p.tolist() for k,p in sealing.items()},
                    suction_normal_tcp=normal.tolist(),pneumatic_mode='COMMON_VALVE_PASSIVE_SELF_CLOSING' if passive else 'COMMON_VALVE_ALL_SEAL',
                    commanded_cup_ids=ids,expected_passively_closed_ids=[i for i in ids if i not in sealing]))
    if not candidates: raise ValueError('CONNECTED_CUP_SEALS_DO_NOT_FIT_VISIBLE_FACE')
    return sorted(candidates,key=lambda c:-len(c["active_ids"]))[:32]


def seal_check(tcp, face, rings_tcp, suction_normal_tcp, margin=.003, max_gap=.0004):
    n,u,v=(np.asarray(face[k]) for k in ('normal','u','v'));center=np.asarray(face['center_m'])
    p=transform(tcp,np.concatenate(list(rings_tcp.values())))-center
    gaps=p@n;uv=np.c_[p@u,p@v];half=np.asarray(face['size_m'])/2-margin
    angle=float(np.arccos(np.clip(-n@(np.asarray(tcp)[:3,:3]@suction_normal_tcp),-1,1)))
    return dict(valid=bool(np.all(abs(uv)<=half+1e-9) and np.max(abs(gaps))<=max_gap and angle<=.01),
                minimum_edge_margin_m=float(np.min(np.asarray(face['size_m'])/2-abs(uv))),
                maximum_gap_m=float(np.max(abs(gaps))),normal_error_rad=angle,
                connected_cup_ids=list(rings_tcp))
