"""Observation-bound approximate boxes, separate from certified sharp solids.

No object identity, GT geometry or nominal dimensions enter this module. Axes
are rows. Measured planes remain immutable; virtual corners confer no motion
or conservative-envelope qualification.
"""
from __future__ import annotations

import numpy as np

from .final_geometry import SIGNS, project
from .metric_support import backproject_pixels


def stats(values):
    values=np.asarray(values,float)
    return dict(count=int(values.size),mean_m=float(np.mean(values)) if values.size else None,
        p95_m=float(np.percentile(values,95)) if values.size else None,
        max_m=float(np.max(values)) if values.size else None)


def context(record,depth,mask,K):
    from .metric_faces import MetricFitConfig,_support_mask
    from .physical_bounds import resolve_bounds
    config=MetricFitConfig(**record['config'])
    previous=record['boundary_constraints'];axes=np.asarray(previous['axes_camera'])
    low,high=np.asarray(previous['silhouette_optimized_bounds_m'])
    groups=[];observed={}
    for face in record['camera_facing_faces']:
        axis=face['axis_index'];n=np.asarray(face['plane_normal']);sign=float(np.sign(n@axes[axis]))
        g=dict(label=face['support_label'],normal=n,offset=face['plane_offset_m'],axis=axis,sign=sign,
            region=_support_mask(record['frozen_support_regions'][str(face['support_label'])],depth.shape))
        value=-g['offset']/sign;side='low' if abs(value-low[axis])<abs(value-high[axis]) else 'high'
        groups.append(g);observed[axis,side]=g
    finite=resolve_bounds(groups,axes,low,high,observed,depth,mask,K,config)
    return config,axes,observed,finite


def transition_consistency(shared,depth,mask,K,groups,maximum_jump_m=.02):
    """Require connected observed surfaces through a finite transition band.

    Neighbourhood endpoints are measured points on each distinct face. Their
    connecting pixels must remain in the same SAM instance, have finite depth,
    and have no abrupt *departure from the endpoint depth gradient*. The depth
    at the mathematical intersection is NOT tested against both planes.
    All departures and failed probes are retained, not residual-trimmed.
    """
    by_label={g['label']:g for g in groups};rows=[]
    for joint in shared:
        probes=[]
        for sample in joint.get('samples',[]):
            surfaces=sample['surfaces']
            if len(surfaces)!=2 or not all(s.get('point_pixels') for s in surfaces):
                probes.append(dict(valid=False,reason='MISSING_TWO_SIDED_NEARBY_SUPPORT'));continue
            a,b=[np.asarray(s['point_pixels'][0]) for s in surfaces]
            count=int(np.max(abs(b-a)))+1
            xy=np.unique(np.rint(a+np.linspace(0,1,count)[:,None]*(b-a)).astype(int),axis=0)
            z=depth[xy[:,1],xy[:,0]]
            inside=mask[xy[:,1],xy[:,0]]
            row=dict(pixels=xy.tolist(),valid=False)
            if not inside.all() or not np.isfinite(z).all() or (z<=0).any():
                row['reason']='OCCLUSION_MASK_GAP_OR_INVALID_DEPTH';probes.append(row);continue
            # Reorder lexicographic unique pixels along their original segment.
            t=((xy-a)@(b-a))/max(1,float((b-a)@(b-a)));order=np.argsort(t);xy=xy[order];z=z[order];t=t[order]
            expected=z[0]+t*(z[-1]-z[0]);departure=z-expected
            jump=float(np.max(abs(np.diff(departure)))) if len(z)>1 else 0.
            points=backproject_pixels(xy,z,K)
            nearest=np.min([abs(points@by_label[s['support_label']]['normal']+by_label[s['support_label']]['offset']) for s in surfaces],axis=0)
            row.update(valid=jump<=maximum_jump_m,reason='CONTINUOUS_OBSERVED_TRANSITION' if jump<=maximum_jump_m else 'DEPTH_DISCONTINUITY_BETWEEN_FACES',
                maximum_gradient_corrected_jump_m=jump,nearest_plane_shape_deviation=stats(nearest),
                depth_departure_from_endpoint_chord=stats(abs(departure)))
            probes.append(row)
        count=sum(p['valid'] for p in probes)
        rows.append(dict(support_labels=joint['support_labels'],consistent=count>=10,
            consistent_probes=count,total_probes=len(probes),probes=probes))
    connected={groups[0]['label']}
    for _ in groups:
        for row in rows:
            if row['consistent'] and connected.intersection(row['support_labels']):connected.update(row['support_labels'])
    return dict(status='PASS' if len(connected)==len(groups) else 'INCONSISTENT_OR_DISCONNECTED_INSTANCE',
        policy='finite same-mask depth continuity between measured face supports; no sharp-crease equality',
        maximum_gradient_corrected_jump_m=maximum_jump_m,pairs=rows)


def rounded_surface_residual(q,low,high,radii):
    """Metre first-order distance to inner-box + ellipsoid transition surface.

    Radii are nuisance shape parameters inferred from the visible transitions,
    not a fixed expansion. The emitted cuboid is this model's virtual bounding
    box; that is NOT a certified conservative envelope of the physical object.
    """
    radii=np.maximum(radii,1e-5)
    center=(low+high)/2;inner=(high-low)/2-radii
    v=np.maximum(abs(q-center)-inner,0)/radii
    length=np.linalg.norm(v,axis=1)
    gradient=np.linalg.norm(v/radii,axis=1)/np.maximum(length,1e-9)
    residual=(length-1)/np.maximum(gradient,1e-9)
    # Inside the rectangular core, the nearest surface is a main plane. Using
    # minus the smallest radius here would let tiny radii hide interior error.
    return np.where(length>1e-9,residual,-np.min((high-low)/2-abs(q-center),axis=1))


def refine_raw_bounds(low,high,axes,observed,finite,depth,mask,K,config):
    """Six parameters: three free extents and three transition radii.

    Main-face planes and common directions are fixed. Surface residuals are in
    metres; silhouette support-plane residuals are metres at the sensor. Equal
    aggregate weights keep sampling density from changing their balance. Every
    valid instance pixel contributes to reported residuals (no fit-based trim).
    """
    from scipy.optimize import least_squares
    free=[(axis,side) for axis in range(3) for side in ('low','high') if (axis,side) not in observed]
    yy,xx=np.nonzero(mask & np.isfinite(depth) & (depth>0))
    pixels=np.c_[xx,yy];points=backproject_pixels(pixels,depth[yy,xx],K);q=points@axes.T
    indices=np.linspace(0,len(q)-1,min(len(q),12000),dtype=int)
    sampled=q[indices]
    normals=[];scales=[];sources=[]
    center_camera=(low+high)/2@axes
    for edge in finite['edges']:
        tangent=np.diff(np.asarray(edge['candidate_segment_pixels']),axis=0)[0]
        tangent/=max(np.linalg.norm(tangent),1e-9)
        for sample in edge['samples']:
            if sample['kind']!='PHYSICAL_OUTLINE':continue
            pixel=np.asarray(sample['pixel']);rays=backproject_pixels(np.array([pixel,pixel+tangent]),[1.,1.],K)
            n=np.cross(*rays);n/=max(np.linalg.norm(n),1e-9)
            if n@center_camera>0:n=-n
            normals.append(axes@n)
            x,y=sample['inner_pixel'];z=float(depth[y,x])
            scales.append(config.boundary_noise_scale_px*z/min(K[0,0],K[1,1]))
            sources.append(dict(support_label=edge['support_label'],edge_index=edge['edge_index'],pixel=pixel.tolist()))
    normals=np.asarray(normals);scales=np.asarray(scales)
    initial=np.array([(low if side=='low' else high)[axis] for axis,side in free])
    extent=high-low;minimum=float(np.min(extent));radius_limit=minimum*.24
    lo=np.r_[initial-.25*extent,[1e-5]*3];hi=np.r_[initial+.25*extent,[radius_limit]*3]
    surface_scale=config.approximate_surface_scale_m

    def unpack(x):
        a,b=low.copy(),high.copy()
        for (axis,side),v in zip(free,x[:3]):(a if side=='low' else b)[axis]=v
        return a,b,x[3:]

    def evaluate(x,cloud,normal_set,scale_set):
        a,b,r=unpack(x);center=(a+b)/2;inner=(b-a)/2-r
        surfaces=rounded_surface_residual(cloud,a,b,r)
        # Support function of a rectangular core Minkowski-summed with an
        # ellipsoid. A silhouette ray-plane is tangent to that model: h(n)=0.
        outlines=normal_set@center+abs(normal_set)@inner+np.linalg.norm(normal_set*r,axis=1)
        return surfaces,outlines

    runs=[]
    # Fixed initialisations and two deterministic boundary/sample perturbations.
    for fraction,phase in ((.01,None),(.035,None),(.07,None),(.035,0),(.035,1)):
        cloud=sampled if phase is None else sampled[phase::2]
        ns=normals if phase is None else normals[phase::2];ss=scales if phase is None else scales[phase::2]
        def residual(x):
            sr,br=evaluate(x,cloud,ns,ss)
            def robust(v):return np.sign(v)*np.sqrt(2*(np.sqrt(1+v*v)-1))
            return np.r_[robust(sr/surface_scale),robust(br/ss)*np.sqrt(len(cloud)/max(1,len(ns)))]
        solution=least_squares(residual,np.r_[initial,[minimum*fraction]*3],bounds=(lo,hi),
            loss='linear',x_scale='jac',max_nfev=160,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        sr,br=evaluate(solution.x,sampled,normals,scales)
        score=float(np.mean(np.sqrt(1+(sr/surface_scale)**2)-1)+np.mean(np.sqrt(1+(br/scales)**2)-1))
        runs.append(dict(initial_radius_fraction=fraction,sampling_phase=phase,parameters=solution.x.tolist(),
            score=score,success=bool(solution.success),at_parameter_limit=bool(np.any(solution.active_mask)),
            nfev=solution.nfev,surface=stats(abs(sr)),silhouette=stats(abs(br))))
    best=min(runs[:3],key=lambda row:row['score']);a,b,r=unpack(np.array(best['parameters']))
    equivalent=[row for row in runs if row['score']<=best['score']*1.25+1e-6]
    values=np.array([row['parameters'][:3] for row in equivalent])
    spread=np.ptp(values,axis=0)
    stable=all(row['success'] and not row['at_parameter_limit'] for row in equivalent) and np.max(spread)<=config.approximate_stability_m
    all_residual=abs(rounded_surface_residual(q,a,b,r))
    planar=np.min([abs(points@g['normal']+g['offset']) for g in observed.values()],axis=0)
    transition=planar>config.maximum_p95_m
    outlier=all_residual>config.approximate_outlier_m
    surface_ok=float(np.mean(outlier))<=config.approximate_outlier_fraction
    return dict(status='STABLE' if stable and surface_ok else 'UNSTABLE_OR_MODEL_MISMATCH',
        low=a.tolist(),high=b.tolist(),radii_m=r.tolist(),runs=runs,
        stability=dict(status='PASS' if stable else 'AMBIGUOUS',free_bounds_spread_m=spread.tolist(),
            equivalent_solutions=len(equivalent),parameter_ranges_m=np.c_[values.min(0),values.max(0)].tolist(),
            equivalence_rule='same full-data robust objective within 25 percent plus 1e-6 numerical slack',
            maximum_allowed_spread_m=config.approximate_stability_m,
            uncertainty_scope='initialisation and deterministic sampling only; model bias is not a confidence interval'),
        residuals=dict(all_instance_pixels=stats(all_residual),stable_plane_region=stats(all_residual[~transition]),
            transition_region=stats(all_residual[transition]),transition_pixel_count=int(transition.sum()),
            transition_region_definition='farther than 6 mm from every measured main plane; may also contain occlusion or segmentation error, not an automatic rounded-shape label',
            nearest_measured_plane_deviation=stats(planar),outlier_count=int(outlier.sum()),
            outlier_fraction=float(np.mean(outlier)),outlier_threshold_m=config.approximate_outlier_m,
            reported_population='all finite positive raw-depth pixels inside the original SAM mask; no residual trimming'),
        objective=dict(surface_residual='first-order signed distance to inner-box plus ellipsoid, metres',
            surface_scale_m=surface_scale,silhouette_residual='support function on observed tangent ray-plane, metres',
            silhouette_scale='boundary_noise_scale_px * measured_depth / min(fx,fy)',
            aggregate_weight_surface=1.,aggregate_weight_silhouette=1.,loss='soft_l1',
            fixed_measured_planes=True,fit_pixels=len(sampled),reported_pixels=len(q)),
        free_bounds=[dict(axis=axis,side=side) for axis,side in free],silhouette_sources=sources,
        assumption='one convex orthogonal main-body box with shared per-axis ellipsoidal transition radii; estimated from raw depth, not GT')


def estimate(record,depth,mask,K):
    """Build a model, never promote record.accepted or its execution eligibility."""
    depth,mask,K=np.asarray(depth),np.asarray(mask,bool),np.asarray(K,float).reshape(3,3)
    mode=record['config'].get('approximate_cuboid_mode','off')
    result=dict(representation='APPROXIMATE_CUBOID',status='UNCONSTRAINED',mode=mode,
        physical_corners_certified=False,conservative_collision_envelope=False,
        planning_eligible=False,execution_eligible=False,gt_accuracy='NOT_EVALUATED_AT_RUNTIME',
        measured_planes_modified=False,virtual_corners=True)
    if mode=='off':return {**result,'status':'NOT_RUN_DISABLED'}
    if record.get('source_ambiguous') or record.get('reliable_plane_count')!=3 or not record.get('metric_solver',{}).get('success'):
        return {**result,'status':'NOT_ATTEMPTED','reason':'REQUIRES_THREE_DISTINCT_UNAMBIGUOUS_RELIABLE_PLANES'}
    config,axes,observed,finite=context(record,depth,mask,K)
    if len(observed)!=3 or len({a for a,s in observed})!=3:
        return {**result,'status':'UNCONSTRAINED','reason':'REQUIRES_THREE_ORTHOGONAL_FACE_DIRECTIONS'}
    normals=[g['normal'] for g in observed.values()]
    if any(abs(a@b)>np.sin(np.radians(config.normal_pair_tolerance_degrees)) for i,a in enumerate(normals) for b in normals[i+1:]):
        return {**result,'status':'UNCONSTRAINED','reason':'CONTRADICTORY_MAIN_PLANES'}
    raw_groups=finite.pop('boundary_regions')
    consistency=transition_consistency(record['boundary_constraints']['shared_edges'],depth,mask,K,raw_groups)
    result.update(instance_consistency=consistency,sharp_corner_diagnostic=record['boundary_constraints']['shared_edges'],
        sharp_corner_usage='DIAGNOSTIC_ONLY; failure does not identify its physical cause',
        axes_camera=axes.tolist(),bounds=finite['bounds'],edges=finite['edges'],
        assumptions=['one approximately convex carton with three observed orthogonal main faces',
                     'each opposite extent requires a connected finite silhouette; plane patches alone do not determine size'])
    for bound in result['bounds']:
        measured=(bound['axis'],bound['side']) in observed
        bound['interpretation']='MEASURED_MAIN_PLANE' if measured else 'MODEL_CONDITIONAL_OUTLINE_PROJECTION'
        bound['physical_boundary_certified']=False
        bound['sampling_interval_m']=bound.pop('interval_m',None)
    if not finite['complete']:
        result['reason']='MISSING_FINITE_EXTENT_CONSTRAINTS';return result
    if consistency['status']!='PASS':
        result['reason']='INCONSISTENT_INSTANCE_TRANSITION';return result
    low=np.array([b['value_m'] for b in result['bounds'][::2]])
    high=np.array([b['value_m'] for b in result['bounds'][1::2]])
    if np.min(high-low)<=.01:return {**result,'reason':'DEGENERATE_MODEL_EXTENT'}
    result['assumptions'].append('outline ray / main plane intersection is an approximate initial extent, not an exact hidden plane')
    if mode=='raw_boundary':
        refinement=refine_raw_bounds(low,high,axes,observed,finite,depth,mask,K,config)
        result.update(raw_boundary_fit=refinement,stability=refinement['stability'])
        if refinement['status']!='STABLE':
            result.update(status='UNSTABLE',reason='MULTIPLE_SOLUTIONS_OR_OBSERVATION_MODEL_MISMATCH');return result
        low,high=np.asarray(refinement['low']),np.asarray(refinement['high'])
        result['assumptions'].append(refinement['assumption'])
        for bound in result['bounds']:
            axis,side=bound['axis'],bound['side']
            bound['outline_projection_initial_m']=bound['value_m']
            bound['value_m']=float((low if side=='low' else high)[axis])
            if (axis,side) not in observed:
                bound['interpretation']='MODEL_ESTIMATED_RAW_SURFACE_AND_SILHOUETTE'
                bound['model_parameters']='raw_boundary_fit: joint extents and observed transition radii'
                index=refinement['free_bounds'].index(dict(axis=axis,side=side))
                bound['parameter_stability_interval_m']=refinement['stability']['parameter_ranges_m'][index]
                bound['uncertainty_meaning']='fixed initialisation/sampling sensitivity, not physical coverage guarantee'
    else:
        result['stability']=dict(status='SAMPLING_ONLY',model_bias='UNQUANTIFIED_OUTLINE_PROJECTION')
    result.update(status='GENERATED',bounds_low_m=low.tolist(),bounds_high_m=high.tolist(),
        center_3d=((low+high)/2@axes).tolist(),orthogonal_axes_3d=axes.tolist(),shape_dimensions=(high-low).tolist(),
        corners_3d=((low+(SIGNS+1)/2*(high-low))@axes).tolist())
    result['corners_2d']=project(result['corners_3d'],K).tolist()
    return result


def validate(record,depth,mask,K):
    """Re-read the same independent observation policy, without a crease gate."""
    from .final_geometry import coherent_cuboid
    existing=record['approximate_cuboid'];fresh=estimate(record,depth,mask,K)
    if existing.get('status') not in ('GENERATED','VALIDATED_OBSERVATION_MODEL'):
        return fresh
    try:
        if fresh['status']!='GENERATED':raise ValueError('APPROXIMATE_MODEL_LOST_OBSERVATION_CONSTRAINTS')
        center,_,_,_=coherent_cuboid(existing['corners_3d'])
        axes=np.asarray(fresh['axes_camera']);q=np.asarray(existing['corners_3d'])@axes.T
        residual=np.array([q.min(0)-fresh['bounds_low_m'],q.max(0)-fresh['bounds_high_m']])
        if np.max(abs(residual))>.006:raise ValueError('APPROXIMATE_TANGENTIAL_BOUND_CONSTRAINT_MISMATCH')
        if np.linalg.norm(center-np.asarray(existing['center_3d']))>1e-5:raise ValueError('APPROXIMATE_CENTER_MISMATCH')
        if np.max(abs(np.ptp(q,axis=0)-existing['shape_dimensions']))>1e-5:raise ValueError('APPROXIMATE_DIMENSION_MISMATCH')
        if not np.allclose(existing['orthogonal_axes_3d'],axes,atol=1e-6,rtol=0):raise ValueError('APPROXIMATE_AXIS_MISMATCH')
        fresh['status']='VALIDATED_OBSERVATION_MODEL'
        fresh['final_validation']=dict(status='PASS',strategy=fresh['mode'],
            sharp_corner_gate='NOT_APPLIED',bounds_recomputed_from='original depth, SAM and frozen plane labels',
            maximum_bound_difference_m=float(np.max(abs(residual))))
        return fresh
    except ValueError as exc:
        return dict(representation='APPROXIMATE_CUBOID',status='REVOKED_FINAL_VALIDATION',reason=str(exc),
            physical_corners_certified=False,conservative_collision_envelope=False,
            planning_eligible=False,execution_eligible=False,rejected_model=existing)
