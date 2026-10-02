"""Measurement conditioning, not box identity, grasp success or confidence."""
import numpy as np


def support_quality(points, *, minimum_short_m=.015, minimum_area_m2=.0003,
                    maximum_normal_ratio=.20):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('INVALID_QUALITY_POINTS')
    result=dict(points=len(points),geometry_eligible=False,role='OBSERVED_EDGE_EVIDENCE',reasons=[])
    if len(points)<3:
        result['reasons']=['INSUFFICIENT_SPATIAL_SUPPORT'];return result
    center=points.mean(0);_,singular,axes=np.linalg.svd(points-center,full_matrices=False)
    uv=(points-center)@axes[:2].T
    # Actual occupied support spans: no GT sizes, confidence or extrapolation.
    low,high=uv.min(0),uv.max(0);spans=high-low
    short,long=sorted(spans);ratio=float(singular[2]/max(singular[1],1e-15))
    bins=np.minimum(3,((uv-low)/np.maximum(spans,1e-15)*4).astype(int))
    occupied=len(np.unique(bins,axis=0))
    result.update(short_m=float(short),long_m=float(long),area_m2=float(short*long),
        normal_to_tangent_singular_ratio=ratio,occupied_4x4_cells=occupied,
        minimum_short_m=minimum_short_m,minimum_area_m2=minimum_area_m2,
        maximum_normal_ratio=maximum_normal_ratio,
        conditioning_semantics='measured spatial conditioning, not probability or pose covariance')
    if short<minimum_short_m:result['reasons'].append('PHYSICAL_SHORT_SPAN_BELOW_GEOMETRY_MINIMUM')
    if short*long<minimum_area_m2:result['reasons'].append('PHYSICAL_SUPPORT_AREA_BELOW_GEOMETRY_MINIMUM')
    if ratio>maximum_normal_ratio:result['reasons'].append('ILL_CONDITIONED_PLANE_SUPPORT')
    if occupied<6:result['reasons'].append('INSUFFICIENT_TWO_DIMENSIONAL_DISTRIBUTION')
    result['geometry_eligible']=not result['reasons']
    if result['geometry_eligible']:result['role']='GEOMETRY_ESTIMATION'
    return result


def patch_minimum_width(points):
    """Minimum width of a convex coplanar quad; necessary to fit one cup disk."""
    p=np.asarray(points,float);edges=np.roll(p,-1,axis=0)-p
    normal=np.cross(edges[0],edges[1]);normal/=np.linalg.norm(normal)
    inward=np.cross(normal,edges);inward/=np.linalg.norm(inward,axis=1)[:,None]
    return float(min(np.ptp(p@n) for n in inward))
