"""Depth-led observed patches and optional shared cuboid completion.

This optional perception strategy uses NumPy/SciPy/OpenCV, fixed capture K and
SI optical depth. The pinned upstream plane extractor is supplied by the
caller. It never receives GT, calibration variables or synthetic dimensions.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import random
import hashlib

import numpy as np

from .final_geometry import SIGNS, project
from .metric_support import backproject_pixels, observation_support, check_metric_plane


@dataclass(frozen=True)
class MetricFitConfig:
    seed: int = 17
    plane_noise_scale_m: float = .001
    extraction_threshold_m: float = .003
    maximum_mean_m: float = .003
    maximum_p95_m: float = .006
    erosion_px: int = 2
    minimum_points: int = 50
    maximum_fit_points: int = 12000
    boundary_noise_scale_px: float = 2.
    boundary_auxiliary_weight: float = .05
    normal_pair_tolerance_degrees: float = 10.
    positive_infinity_is_no_hit: bool = False


def _tls(points):
    center = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points-center, full_matrices=False)
    normal = vh[-1]
    if normal @ center > 0:
        normal = -normal
    return normal, -float(normal @ center)


def _points(depth, mask, K):
    y, x = np.nonzero(mask)
    return backproject_pixels(np.column_stack((x, y)), depth[mask], K), np.column_stack((x, y))


def _support_runs(mask):
    padded = np.pad(np.asarray(mask, np.int8), ((0,0),(1,1)))
    changes = np.diff(padded, axis=1)
    ys, starts = np.nonzero(changes == 1); ye, ends = np.nonzero(changes == -1)
    if not np.array_equal(ys, ye):
        raise ValueError('invalid observation support runs')
    return np.column_stack((ys, starts, ends)).tolist()


def _support_mask(runs, shape):
    result = np.zeros(shape, bool)
    for y, start, end in runs:
        if not (0 <= y < shape[0] and 0 <= start < end <= shape[1]):
            raise ValueError('invalid frozen support extent')
        result[y, start:end] = True
    return result


def _binding(depth, mask, K):
    return {name: hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()
            for name, value in [('depth_float32',np.asarray(depth,np.float32)),
                                ('instance_mask',np.asarray(mask,bool)),('K_float64',np.asarray(K,np.float64).reshape(3,3))]}


def maximum_observation_rectangle(mask):
    """Largest all-observed pixel rectangle, including concave/holey regions."""
    y,x=np.nonzero(mask)
    if not len(x): return None
    x0,x1=int(x.min()),int(x.max())+1; y0,y1=int(y.min()),int(y.max())+1
    heights=np.zeros(x1-x0,dtype=int); best=(0,None)
    for row in range(y0,y1):
        heights=np.where(mask[row,x0:x1],heights+1,0)
        stack=[]
        for column,height in enumerate(np.r_[heights,0]):
            start=column
            while stack and stack[-1][1]>height:
                left,previous=stack.pop(); area=int(previous*(column-left)); start=left
                if area>best[0]: best=(area,(left+x0,row-int(previous)+1,column-1+x0,row))
            if not stack or stack[-1][1]<height: stack.append((start,int(height)))
    if best[1] is None: return None
    left,top,right,bottom=best[1]
    return np.array([[left,top],[right,top],[right,bottom],[left,bottom]],float)


def extract_observation_labels(depth, mask, K, extractor, config=MetricFitConfig()):
    """Initial robust plane segmentation, frozen before constrained fitting.

    All valid instance pixels receive their nearest *initial* plane label;
    there is no final-plane residual trimming. Disconnected islands are kept
    as separate patches. Held-out spatial blocks never enter the final fit.
    """
    import cv2
    raw, interior, audit = observation_support(depth, mask, mask, erosion_px=config.erosion_px)
    xyz, _ = _points(depth, interior, K)
    if len(xyz) < config.minimum_points:
        return np.full(depth.shape, -1, np.int16), [], audit
    rng = np.random.default_rng(config.seed)
    sample = xyz[rng.choice(len(xyz), min(len(xyz), config.maximum_fit_points), replace=False)]
    random.seed(config.seed); np.random.seed(config.seed)
    extracted = extractor(sample, config.extraction_threshold_m)
    # The upstream extractor stops below 7% of the original cloud. A narrow
    # visible face can still contain hundreds of independent metric samples.
    # Reuse its extractor on the residual population, before freezing labels.
    if extracted:
        residual = np.min(np.stack([np.abs(sample @ np.asarray(v['equation'][:3])+v['equation'][3]) for v in extracted]),axis=0)
        remaining = sample[residual > config.extraction_threshold_m]
        if len(remaining) >= max(config.minimum_points*2,100) and len(remaining) < .5*len(sample):
            extra = extractor(remaining, config.extraction_threshold_m)
            for v in extra:
                duplicate=False
                for old in extracted:
                    cosine=float(np.asarray(v['normal'])@np.asarray(old['normal']))
                    if abs(cosine)>.94 and abs(v['equation'][3]-np.sign(cosine)*old['equation'][3])<.02:
                        duplicate=True
                if not duplicate: extracted.append(v)
    planes = []
    for item in extracted:
        points = np.asarray(item['points'])
        for _ in range(3):
            normal, offset = _tls(points)
            errors = np.abs(sample @ normal + offset)
            points = sample[errors < config.extraction_threshold_m]
            if len(points) < config.minimum_points:
                break
        if len(points) >= config.minimum_points:
            normal, offset = _tls(points)
            planes.append((normal, offset))
    labels = np.full(depth.shape, -1, np.int16)
    if not planes:
        return labels, [], audit
    all_xyz, pixels = _points(depth, raw, K)
    errors = np.stack([np.abs(all_xyz @ n+d) for n, d in planes], axis=1)
    assignment = np.argmin(errors, axis=1)
    # Candidate-independent organised-depth normals disambiguate a thin face
    # even when RANSAC could not recover its plane. In particular, its pixels
    # must not be lifted onto a different face of the same instance.
    neighbours=[]; values=[]; neighbour_membership=[]
    for delta in ((-3,0),(3,0),(0,-3),(0,3)):
        xy=pixels+delta
        xy[:,0]=np.clip(xy[:,0],0,depth.shape[1]-1); xy[:,1]=np.clip(xy[:,1],0,depth.shape[0]-1)
        z=depth[xy[:,1],xy[:,0]]; values.append(z)
        neighbour_membership.append(mask[xy[:,1],xy[:,0]])
        neighbours.append(backproject_pixels(xy,np.where(np.isfinite(z)&(z>0),z,0),K))
    local=np.cross(neighbours[1]-neighbours[0],neighbours[3]-neighbours[2])
    lengths=np.linalg.norm(local,axis=1)
    zvalues=np.asarray(values)
    # A grazing plane has a large legitimate depth gradient. A fixed jump bound
    # here would disable normals exactly where thin side faces need them most.
    reliable=(np.isfinite(zvalues).all(axis=0)&(zvalues.min(axis=0)>0)&
              np.asarray(neighbour_membership).all(axis=0)&(lengths>1e-9))
    local/=np.maximum(lengths[:,None],1e-12)
    compatible=np.abs(local@np.asarray([n for n,_ in planes]).T)>np.cos(np.radians(15))
    unknown=reliable & ~compatible.any(axis=1)
    assignment[unknown]=-1
    audit['unclassified_local_normal_pixels']=int(unknown.sum())
    audit['local_normal_selection']='OBSERVED_DEPTH_CENTRAL_DIFFERENCES_NOT_FINAL_PLANE_RESIDUAL'
    seeds = []
    for i, (normal, offset) in enumerate(planes):
        region = np.zeros(depth.shape, np.uint8)
        selected = pixels[assignment == i]
        region[selected[:, 1], selected[:, 0]] = 1
        count, components, stats, _ = cv2.connectedComponentsWithStats(region, 8)
        for component in range(1, count):
            if stats[component, cv2.CC_STAT_AREA] < config.minimum_points:
                continue
            label = len(seeds)
            labels[components == component] = label
            seeds.append({'normal': normal.tolist(), 'offset_m': float(offset),
                          'initial_plane_index': i, 'component': component})
    audit['initial_planes'] = seeds
    audit['unassigned_pixels'] = int((raw & (labels < 0)).sum())
    return labels, seeds, audit


def _orthogonal_fit(groups, config):
    """SO(3) shared normals and observed offsets; robust metric residual only.

    Boundary optimisation is later and cannot move these planes. Per-face
    sqrt(N) normalisation prevents the largest face dominating orientation.
    """
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    normals = np.array([g['normal'] for g in groups])
    first = normals[0]
    other = next((n for n in normals[1:] if abs(n @ first) < .2), None)
    if other is None:
        helper = np.eye(3)[np.argmin(np.abs(first))]
        other = np.cross(first, helper)
    second = other - (other @ first)*first; second /= np.linalg.norm(second)
    axes = np.stack((first, second, np.cross(first, second)))
    association = np.argmax(np.abs(normals @ axes.T), axis=1)
    signs = np.sign(np.sum(normals*axes[association], axis=1))
    initial = np.r_[Rotation.from_matrix(axes.T).as_rotvec(), [g['offset'] for g in groups]]

    def robust_residual(parameters):
        basis = Rotation.from_rotvec(parameters[:3]).as_matrix().T
        residuals = []
        for g, a, s, d in zip(groups, association, signs, parameters[3:]):
            values = (g['fit_points'] @ (basis[a]*s)+d)/config.plane_noise_scale_m
            residuals.append(np.sign(values)*np.sqrt(2*(np.sqrt(1+values*values)-1))/np.sqrt(len(values)))
        return np.concatenate(residuals)

    solved = least_squares(robust_residual, initial, max_nfev=100, ftol=1e-11, xtol=1e-11, gtol=1e-11)
    axes = Rotation.from_rotvec(solved.x[:3]).as_matrix().T
    for g, axis, sign, offset in zip(groups, association, signs, solved.x[3:]):
        g.update(axis=int(axis), normal=axes[axis]*sign, offset=float(offset), sign=float(sign))
    return axes, {'success': bool(solved.success), 'cost': float(solved.cost),
                  'rotation_determinant': float(np.linalg.det(axes)),
                  'objective': 'ROBUST_METRIC_POINT_TO_SHARED_SO3_PLANES',
                  'calibration_optimised': False, 'scale_optimised': False}


def _boundary_kinds(polygon, normal, offset, depth, K, *, search_px=6, positive_infinity_is_no_hit=False):
    """Classify by outside depth: nearer occluder, farther silhouette or crop.

    A mask/proposal edge alone is insufficient physical-edge evidence. Sampling
    excludes edge endpoints and preserves unknown when observations disagree.
    """
    h, w = depth.shape
    center = np.mean(polygon, axis=0)
    result = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        samples = a[None]+np.linspace(.15, .85, 15)[:, None]*(b-a)
        edge = b-a
        outward = np.array([edge[1],-edge[0]])
        if outward @ ((a+b)/2-center) < 0:
            outward *= -1
        outward /= max(np.linalg.norm(outward), 1e-9)
        evidence = []
        for p in samples:
            classifications = []
            for distance in (2, 4, search_px):
                xy = p + outward*distance
                x, y = np.rint(xy).astype(int)
                if x < 0 or x >= w or y < 0 or y >= h:
                    classifications.append('IMAGE_CROP_BOUNDARY'); continue
                z = depth[y, x]
                ray = backproject_pixels(np.array([[x, y]]), [1.], K)[0]
                denominator = normal @ ray
                predicted = -offset/denominator if abs(denominator) > 1e-9 else np.nan
                if positive_infinity_is_no_hit and np.isposinf(z) and np.isfinite(predicted):
                    classifications.append('PHYSICAL_EDGE_SUPPORTED')
                elif not np.isfinite(z) or z <= 0 or not np.isfinite(predicted):
                    classifications.append('UNCLASSIFIED_BOUNDARY')
                elif z < predicted-.01:
                    classifications.append('OCCLUSION_BOUNDARY')
                elif z > predicted+.01:
                    classifications.append('PHYSICAL_EDGE_SUPPORTED')
                else:
                    classifications.append('UNCLASSIFIED_BOUNDARY')
            known = [c for c in classifications if c != 'UNCLASSIFIED_BOUNDARY']
            evidence.append(known[-1] if known else 'UNCLASSIFIED_BOUNDARY')
        kind = max(sorted(set(evidence)), key=evidence.count)
        if evidence.count(kind) < 10:
            kind = 'UNCLASSIFIED_BOUNDARY'
        result.append({'kind': kind, 'support_fraction': evidence.count(kind)/len(evidence)})
    return result


def fit_metric_faces(depth, mask, K, labels, seeds, *, mask_id,
                     config=MetricFitConfig(), source_ambiguous=False):
    """Produce observed patches first; complete only supported physical bounds."""
    import cv2
    depth, mask, K = np.asarray(depth), np.asarray(mask, bool), np.asarray(K).reshape(3, 3)
    groups = []
    y, x = np.indices(depth.shape)
    holdout = ((x//8+y//8) % 4 == 0)
    rng = np.random.default_rng(config.seed)
    for i, seed in enumerate(seeds):
        region = labels == i
        _, retained, _ = observation_support(depth, mask, region, erosion_px=config.erosion_px)
        points, _ = _points(depth, retained & ~holdout, K)
        if len(points) < config.minimum_points:
            continue
        points = points[rng.choice(len(points), min(len(points), 4000), replace=False)]
        normal, offset = _tls(points)
        groups.append({'label': i, 'region': region, 'normal': normal, 'offset': offset,
                       'fit_points': points, 'initial': seed})
    output = {'mask_id': int(mask_id), 'accepted': False, 'method': 'DEPTH_CONSTRAINED_METRIC_FACES_V1',
              'geometry_version': 'DEPTH_METRIC_PATCHES_V1', 'camera_facing_faces': [],
              'final_face_validation': [], 'config': asdict(config),
              'complete_observability': 'UNRESOLVED_PHYSICAL_BOUNDARIES',
              'source_ambiguous': bool(source_ambiguous)}
    output['support_capture_binding'] = _binding(depth, mask, K)
    output['frozen_support_regions'] = {str(g['label']): _support_runs(g['region']) for g in groups}
    if not groups:
        return output
    # Do not force unrelated planes into one frame (merged masks remain explicit).
    compatible = all(abs(float(a['normal'] @ b['normal'])) < np.sin(np.radians(config.normal_pair_tolerance_degrees))
                     or abs(float(a['normal'] @ b['normal'])) > np.cos(np.radians(config.normal_pair_tolerance_degrees))
                     for i, a in enumerate(groups) for b in groups[i+1:])
    if compatible:
        axes, solver = _orthogonal_fit(groups, config)
    else:
        axes, solver = None, {'success': False, 'reason': 'INCOMPATIBLE_INSTANCE_PLANE_NORMALS'}
    output['metric_solver'] = solver
    all_boundary_points = []
    for g in groups:
        region = g['region']
        audit = check_metric_plane(g['normal'], g['offset'], depth, mask, region, K,
                                   maximum_mean_m=config.maximum_mean_m, minimum_points=config.minimum_points,
                                   erosion_px=config.erosion_px)
        held = check_metric_plane(g['normal'], g['offset'], depth, mask, region & holdout, K,
                                  maximum_mean_m=config.maximum_mean_m, minimum_points=config.minimum_points,
                                  erosion_px=0)
        audit['spatial_holdout'] = held['interior']
        audit['label'] = g['label']
        if audit['status'] == 'PASS' and (audit['plane_residual_p95_m'] > config.maximum_p95_m or
                held['status'] != 'PASS'):
            audit['status'] = 'REJECTED'; audit['reason'] = 'TAIL_OR_SPATIAL_HOLDOUT_RESIDUAL'
        output['final_face_validation'].append(audit)
        if audit['status'] != 'PASS':
            continue
        # Lift actual observed pixels onto their measured plane, not a cuboid.
        _, boundary_support, _ = observation_support(depth, mask, region, erosion_px=0)
        _, pixels = _points(depth, boundary_support, K)
        rays = backproject_pixels(pixels, np.ones(len(pixels)), K)
        lifted = rays*(-g['offset']/(rays @ g['normal']))[:, None]
        all_boundary_points.append(lifted)
        if axes is not None:
            tangents = axes[[i for i in range(3) if i != g['axis']]]
        else:
            helper = np.eye(3)[np.argmin(np.abs(g['normal']))]
            u = np.cross(g['normal'], helper); u /= np.linalg.norm(u)
            tangents = np.array([u, np.cross(g['normal'], u)])
        coordinates = lifted @ tangents.T
        low, high = np.quantile(coordinates, [.002, .998], axis=0)
        origin = -g['offset']*g['normal']
        p = origin + np.array([[low[0],low[1]], [high[0],low[1]],
                               [high[0],high[1]], [low[0],high[1]]]) @ tangents
        polygon = project(p, K)
        center_pixel = np.rint(project(p.mean(axis=0)[None],K)[0]).astype(int)
        distance=cv2.distanceTransform(region.astype(np.uint8),cv2.DIST_L2,5)
        if not (0<=center_pixel[0]<depth.shape[1] and 0<=center_pixel[1]<depth.shape[0]
                and distance[center_pixel[1],center_pixel[0]] >= .5*distance.max()):
            cy,cx=np.unravel_index(np.argmax(distance),distance.shape)
            ray=backproject_pixels(np.array([[cx,cy]]),[1.],K)[0]
            center=ray*(-g['offset']/(ray@g['normal']))
            p += center-p.mean(axis=0); polygon=project(p,K)
        # A partial patch must not bridge a nonrectangular occlusion. Shrink
        # around its supported centre until every interior lattice probe belongs
        # to this measured component. This changes patch extent, not the plane.
        for _ in range(60):
            grid = np.array([a*(1-u)*(1-v)+b*u*(1-v)+c*u*v+d*(1-u)*v
                             for u in np.linspace(0.,1.,17) for v in np.linspace(0.,1.,17)
                             for a,b,c,d in [polygon]])
            ij = np.rint(grid).astype(int); inside = ((ij[:,0]>=0)&(ij[:,0]<depth.shape[1])&(ij[:,1]>=0)&(ij[:,1]<depth.shape[0]))
            supported = np.zeros(len(ij), bool); supported[inside] = region[ij[inside,1],ij[inside,0]]
            if supported.all():
                break
            p = p.mean(axis=0)+.96*(p-p.mean(axis=0)); polygon = project(p,K)
        patch_method='INSCRIBED_METRIC_AXIS_QUAD'
        pixel_rectangle=maximum_observation_rectangle(region)
        if pixel_rectangle is not None and abs(cv2.contourArea(pixel_rectangle.astype(np.float32))) > (abs(cv2.contourArea(polygon.astype(np.float32))) if supported.all() else 0):
            rays=backproject_pixels(pixel_rectangle,np.ones(4),K)
            p=rays*(-g['offset']/(rays@g['normal']))[:,None]; polygon=project(p,K)
            patch_method='MAXIMUM_OBSERVATION_RECTANGLE_ON_MEASURED_PLANE'
        elif not supported.all():
            continue
        boundaries = _boundary_kinds(polygon, g['normal'], g['offset'], depth, K,
                                     positive_infinity_is_no_hit=config.positive_infinity_is_no_hit)
        face = {'corner_indices': [], 'corners_3d_m': p.tolist(), 'corners_2d': polygon.tolist(),
                'plane_normal': g['normal'].tolist(), 'plane_offset_m': float(g['offset']),
                'evidence': 'registered_metric_depth_plane', 'final_support': audit,
                'support_label': g['label'], 'boundary_kind': 'OBSERVED_PATCH_CLASSIFIED',
                'patch_boundary_method':patch_method,
                'boundary_evidence': boundaries, 'physical_corners_certified': False,
                'axis_index': g.get('axis'), 'side': 'observed',
                'initial_plane': g['initial']}
        output['camera_facing_faces'].append(face)
    output['projected_camera_facing_face_count'] = len(output['camera_facing_faces'])
    output['depth_supported_face_count'] = len(output['camera_facing_faces'])
    if axes is not None and len(output['camera_facing_faces']) >= 2 and not source_ambiguous:
        _complete(output, groups, axes, all_boundary_points, depth, K, config, mask)
    return validate_metric_record(output, depth, mask, K, maximum_mean_m=config.maximum_mean_m,
                                  minimum_points=config.minimum_points)


def validate_metric_record(record, depth, mask, K, *, maximum_mean_m=.003, minimum_points=50):
    """Independently recheck final patch planes on frozen observation labels."""
    from copy import deepcopy
    from .final_geometry import coherent_cuboid, polygon_pixels
    output = deepcopy(record)
    if output.get('support_capture_binding') != _binding(depth, mask, K):
        raise ValueError('METRIC_SUPPORT_CAPTURE_BINDING_MISMATCH')
    accepted, audits = [], []
    yy, xx = np.indices(np.asarray(depth).shape)
    holdout = (xx//8+yy//8)%4 == 0
    for face in output.get('camera_facing_faces',[]):
        audit = {'status':'REJECTED','label':face['support_label']}
        try:
            p = np.asarray(face['corners_3d_m'],float)
            if p.shape != (4,3) or not np.isfinite(p).all():
                raise ValueError('INVALID_PATCH_POINTS')
            n = np.cross(p[1]-p[0],p[3]-p[0]); n /= np.linalg.norm(n)
            d = -float(n @ p.mean(axis=0))
            if np.max(np.abs(p@n+d)) > 1e-5:
                raise ValueError('NONPLANAR_PATCH')
            region = _support_mask(output['frozen_support_regions'][str(face['support_label'])], np.asarray(depth).shape)
            audit = check_metric_plane(n,d,depth,mask,region,K,maximum_mean_m=maximum_mean_m,minimum_points=minimum_points)
            held = check_metric_plane(n,d,depth,mask,region & holdout,K,maximum_mean_m=maximum_mean_m,minimum_points=minimum_points,erosion_px=0)
            audit['spatial_holdout'] = held['interior']; audit['label']=face['support_label']
            if audit['status'] != 'PASS' or held['status'] != 'PASS' or audit['plane_residual_p95_m'] > .006:
                raise ValueError('FINAL_METRIC_PATCH_RESIDUAL')
            polygon = project(p,K); pixels = polygon_pixels(polygon,np.asarray(depth).shape)
            precision = np.count_nonzero(pixels & region & mask)/max(1,np.count_nonzero(pixels))
            audit['observation_region_precision'] = precision
            audit['patch_observation_coverage'] = np.count_nonzero(pixels & region)/max(1,np.count_nonzero(region))
            audit['patch_pixel_support_count'] = int(np.count_nonzero(pixels & region & mask))
            if audit['patch_pixel_support_count'] < minimum_points:
                raise ValueError('INSUFFICIENT_PATCH_PIXEL_SUPPORT')
            if precision < .97:
                raise ValueError('FINAL_PATCH_CROSSES_OBSERVED_BOUNDARY')
            face['final_support']=audit; accepted.append(face)
        except (ValueError, KeyError, IndexError) as exc:
            audit.update(status='REJECTED',reason=str(exc))
        audits.append(audit)
    output['camera_facing_faces']=accepted; output['final_face_validation']=audits
    output['projected_camera_facing_face_count']=len(accepted)
    if output.get('accepted'):
        try:
            center,axes,dimensions,_=coherent_cuboid(output['corners_3d'])
            if len(accepted) != len(record['camera_facing_faces']):
                raise ValueError('COMPLETE_CUBOID_LOST_OBSERVED_SUPPORT')
            patches={f['support_label']:f for f in accepted}
            for candidate in output['completion_diagnostics']['candidate_faces']:
                patch=patches[candidate['support_label']]
                p=np.asarray(patch['corners_3d_m'])
                n=np.cross(p[1]-p[0],p[3]-p[0]); n/=np.linalg.norm(n)
                d=-n@p.mean(axis=0)
                full=np.asarray(output['corners_3d'])[candidate['corner_indices']]
                if np.max(np.abs(full@n+d))>maximum_mean_m:
                    raise ValueError('COMPLETE_CUBOID_MOVED_FROM_OBSERVED_PLANES')
            output.update(center_3d=center.tolist(),orthogonal_axes_3d=axes.tolist(),shape_dimensions=dimensions.tolist())
            output['complete_cuboid_consistency']={'status':'PASS'}
        except ValueError as exc:
            output['accepted']=False
            output['complete_observability']='UNRESOLVED_PHYSICAL_BOUNDARIES'
            output['complete_cuboid_consistency']={'status':'REJECTED','reason':str(exc)}
    return output


def _complete(output, groups, axes, all_points, depth, K, config, mask):
    """Shared bounds from measured planes and visible silhouette endpoints."""
    coordinates = np.concatenate(all_points) @ axes.T
    low, high = np.quantile(coordinates, [.001, .999], axis=0)
    accepted_labels = {f['support_label'] for f in output['camera_facing_faces']}
    observed = {}
    for g in groups:
        if g['label'] not in accepted_labels:
            continue
        axis = g['axis']; value = -g['offset']/g['sign']
        side = 'low' if abs(value-low[axis]) < abs(value-high[axis]) else 'high'
        (low if side == 'low' else high)[axis] = value
        observed[(axis, side)] = g
    if len({i for i, _ in observed}) < 2 or np.min(high-low) <= .01:
        return
    # Auxiliary silhouette optimisation has authority over unmeasured bounds
    # only. SO(3) and all depth-observed plane bounds remain exactly frozen.
    free = [(i,s) for i in range(3) for s in ('low','high') if (i,s) not in observed]
    pairs = [([0,3,7,4],[1,2,6,5]), ([0,1,5,4],[3,2,6,7]), ([0,1,2,3],[4,5,6,7])]
    import cv2
    from scipy.optimize import least_squares
    face_boundaries=[]
    for (axis,side),g in observed.items():
        region=g['region'] & mask
        signed=cv2.distanceTransform((~region).astype(np.uint8),cv2.DIST_L2,5)-cv2.distanceTransform(region.astype(np.uint8),cv2.DIST_L2,5)
        face_boundaries.append((pairs[axis][side=='high'],signed))
    initial = np.array([(low if s=='low' else high)[i] for i,s in free])
    def boundary_residual(values):
        lo,hi=low.copy(),high.copy()
        for (i,s),v in zip(free,values): (lo if s=='low' else hi)[i]=v
        p=(lo+(SIGNS+1)/2*(hi-lo))@axes
        q=project(p,K)
        residual=[]
        for ids,signed in face_boundaries:
            samples=np.vstack([q[a]+np.linspace(.1,.9,16)[:,None]*(q[b]-q[a]) for a,b in zip(ids,ids[1:]+ids[:1])])
            distance=cv2.remap(signed,samples[:,0,None].astype(np.float32),samples[:,1,None].astype(np.float32),
                               cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=100).ravel()
            residual.extend(distance/config.boundary_noise_scale_px*np.sqrt(config.boundary_auxiliary_weight))
        return np.asarray(residual)
    pixel_m=float(np.median(np.concatenate(all_points)[:,2]))/min(K[0,0],K[1,1])
    if len(free) and face_boundaries:
        fit=least_squares(boundary_residual,initial,bounds=(initial-6*pixel_m,initial+6*pixel_m),
                          loss='soft_l1',diff_step=1e-3,max_nfev=60)
        for (i,s),v in zip(free,fit.x): (low if s=='low' else high)[i]=v
        output['metric_solver']['boundary_fit']={'objective':'SILHOUETTE_SIGNED_PIXEL_DISTANCE_ON_FREE_BOUNDS',
            'initial_free_bounds_m':initial.tolist(),'final_free_bounds_m':fit.x.tolist(),
            'normalised_cost':float(fit.cost),'observed_plane_motion_m':0.,'rotation_motion_rad':0.}
    if np.min(high-low)<=.01:
        output['completion_diagnostics']={'rejection':'NONPOSITIVE_OR_UNRESOLVED_BOUND_INTERVAL'}
        return
    corners = (low+(SIGNS+1)/2*(high-low)) @ axes
    candidate_faces = []
    for (axis, side), g in observed.items():
        ids = pairs[axis][side=='high']; p = corners[ids]; polygon = project(p,K)
        boundaries = _boundary_kinds(polygon, g['normal'],g['offset'],depth,K,
                                     positive_infinity_is_no_hit=config.positive_infinity_is_no_hit)
        # Shared intersection lines are physical edges only when both patches
        # actually reach them; outside depth change already supplies evidence.
        candidate_faces.append({'axis':axis,'side':side,'corner_indices':ids,'support_label':g['label'],
                                'corners_2d':polygon.tolist(),'boundary_evidence':boundaries})
    physical = all(b['kind']=='PHYSICAL_EDGE_SUPPORTED' for f in candidate_faces for b in f['boundary_evidence'])
    output['completion_diagnostics'] = {'bounds_low':low.tolist(),'bounds_high':high.tolist(),
                                         'candidate_faces':candidate_faces,'all_visible_edges_physical':physical,
                                         'silhouette_bound_parameters':free,
                                         'observed_plane_bounds':len(observed),'hidden_bound_constraints':'OBSERVED_SILHOUETTE_ENDPOINTS'}
    if not physical:
        return
    output.update(accepted=True, corners_3d=corners.tolist(), corners_2d=project(corners,K).tolist(),
                  orthogonal_axes_3d=axes.tolist(), shape_dimensions=(high-low).tolist(),
                  center_3d=((low+high)/2 @ axes).tolist(), complete_observability='OBSERVABLE_METRIC_MULTIFACE',
                  complete_cuboid_consistency={'status':'PASS'}, completion_mode='SHARED_METRIC_PLANES_AND_PHYSICAL_BOUNDARIES')
