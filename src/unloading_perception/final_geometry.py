"""Independent final geometry checks against this instance's metric depth.

No GT, camera calibration optimization, or inherited RANSAC residuals.
"""
from __future__ import annotations
from copy import deepcopy
import numpy as np


SIGNS = np.asarray([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])


def coherent_cuboid(corners):
    points=np.asarray(corners,dtype=float)
    if points.shape!=(8,3) or not np.isfinite(points).all():
        raise ValueError('NONFINITE_OR_MISSING_CUBOID')
    vectors=points[[1,3,4]]-points[0]
    dimensions=np.linalg.norm(vectors,axis=1)
    if np.min(dimensions)<=1e-6: raise ValueError('NONPOSITIVE_DIMENSIONS')
    axes=vectors/dimensions[:,None]
    if not np.allclose(axes@axes.T,np.eye(3),atol=1e-4,rtol=0):
        raise ValueError('NONORTHOGONAL_CUBOID')
    if np.linalg.det(axes)<0: axes[2]*=-1
    center=points.mean(axis=0)
    rebuilt=center+(SIGNS*dimensions/2)@axes
    distances=np.linalg.norm(points[:,None]-rebuilt[None,:],axis=2)
    error=max(distances.min(axis=0).max(),distances.min(axis=1).max())
    if error>max(1e-5,1e-4*dimensions.max()): raise ValueError('CUBOID_CORNER_MISMATCH')
    return center,axes,dimensions,float(error)


def project(points,K):
    points=np.asarray(points,dtype=float); K=np.asarray(K).reshape(3,3)
    if np.any(points[:,2]<=0): raise ValueError('FACE_BEHIND_CAMERA')
    xy=points@K.T
    return xy[:,:2]/xy[:,2:]


def polygon_pixels(polygon,shape):
    """Convex quad raster at integer pixel centers (same capture convention)."""
    polygon=np.asarray(polygon,dtype=float)
    if polygon.shape!=(4,2) or not np.isfinite(polygon).all(): raise ValueError('INVALID_FACE_POLYGON')
    edges=np.roll(polygon,-1,axis=0)-polygon
    turns=edges[:,0]*np.roll(edges[:,1],-1)-edges[:,1]*np.roll(edges[:,0],-1)
    if not (np.all(turns>1e-8) or np.all(turns<-1e-8)):
        raise ValueError('NONCONVEX_OR_CROSSED_FACE_BOUNDARY')
    h,w=shape
    x0,y0=np.maximum(np.floor(polygon.min(axis=0)).astype(int),[0,0])
    x1,y1=np.minimum(np.ceil(polygon.max(axis=0)).astype(int)+1,[w,h])
    result=np.zeros(shape,bool)
    if x1<=x0 or y1<=y0: return result
    y,x=np.mgrid[y0:y1,x0:x1]; values=[]
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        values.append((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]))
    values=np.stack(values); result[y0:y1,x0:x1]=np.all(values>=-1e-6,axis=0)|np.all(values<=1e-6,axis=0)
    return result


def validate_final_record(record,depth,mask,K,*,maximum_residual_m=.003,minimum_points=50):
    if record.get('geometry_version') == 'DEPTH_METRIC_PATCHES_V1':
        from .metric_faces import validate_metric_record
        return validate_metric_record(record, depth, mask, K, maximum_mean_m=maximum_residual_m,
                                      minimum_points=minimum_points)
    result=deepcopy(record); points=np.asarray(record.get('corners_3d',[]),float)
    try:
        center,axes,dims,error=coherent_cuboid(points)
        result.update(orthogonal_axes_3d=axes.tolist(),shape_dimensions=dims.tolist(),center_3d=center.tolist(),complete_cuboid_consistency={'status':'PASS','corner_error_m':error})
    except ValueError as exc:
        result['complete_cuboid_consistency']={'status':'REJECTED','reason':str(exc)}
    audits=[]; accepted=[]
    K=np.asarray(K).reshape(3,3); depth=np.asarray(depth); mask=np.asarray(mask,bool)
    if depth.shape!=mask.shape or depth.ndim!=2: raise ValueError('DEPTH_MASK_SHAPE_MISMATCH')
    for ordinal,face in enumerate(record.get('camera_facing_faces',[])):
        audit={'ordinal':ordinal,'status':'REJECTED','residual_source':'RECOMPUTED_FINAL_PLANE_AND_BOUNDARY'}
        try:
            ids=face['corner_indices']
            if len(ids)!=4 or len(set(ids))!=4 or any(type(i)!=int or i<0 or i>=8 for i in ids): raise ValueError('INVALID_FACE_TOPOLOGY')
            if tuple(sorted(ids)) not in ((0,1,2,3),(4,5,6,7),(0,1,4,5),(2,3,6,7),(0,3,4,7),(1,2,5,6)):
                raise ValueError('INVALID_CUBOID_FACE_TOPOLOGY')
            if points.shape!=(8,3) or not np.isfinite(points).all(): raise ValueError('INVALID_FACE_POINTS')
            p=points[ids]; normal=np.cross(p[1]-p[0],p[3]-p[0]); length=np.linalg.norm(normal)
            if length<1e-9: raise ValueError('DEGENERATE_FACE')
            normal/=length; offset=-normal@p.mean(axis=0)
            if np.max(np.abs(p@normal+offset))>1e-4: raise ValueError('NONPLANAR_FACE')
            polygon=project(p,K); pixels=polygon_pixels(polygon,mask.shape)
            valid=pixels & mask & np.isfinite(depth) & (depth>0)
            y,x=np.nonzero(valid); z=depth[valid]
            xyz=np.column_stack(((x-K[0,2])*z/K[0,0],(y-K[1,2])*z/K[1,1],z))
            residual=np.abs(xyz@normal+offset)
            count=len(z); intersection=int((pixels&mask).sum()); area=int(pixels.sum())
            audit.update(point_support_count=count,point_support_ratio=count/max(1,intersection),plane_residual_m=None if not count else float(residual.mean()),plane_residual_p95_m=None if not count else float(np.percentile(residual,95)),mask_precision=intersection/max(1,area),mask_coverage=intersection/max(1,int(mask.sum())),mask_iou=intersection/max(1,int((pixels|mask).sum())),plane_normal=normal.tolist(),plane_offset_m=float(offset))
            if count<minimum_points: raise ValueError('INSUFFICIENT_FINAL_DEPTH_SUPPORT')
            if audit['plane_residual_m']>maximum_residual_m: raise ValueError('FINAL_METRIC_PLANE_RESIDUAL')
            if audit['mask_precision']<.72 or audit['mask_coverage']<.04: raise ValueError('FINAL_FACE_MASK_SUPPORT')
            audit['status']='PASS'
            accepted.append({**face,'corners_2d':polygon.tolist(),'final_support':deepcopy(audit),'boundary_kind':'OBSERVED_PATCH_UNCLASSIFIED','physical_corners_certified':False})
        except (ValueError,KeyError,IndexError) as exc: audit['reason']=str(exc)
        audits.append(audit)
    result['camera_facing_faces']=accepted
    result['final_face_validation']=audits
    result['projected_camera_facing_face_count']=len(accepted)
    result['geometry_version']='FINAL_METRIC_VALIDATED_V1'
    # Face count alone cannot establish hidden dimensions or physical corners.
    result['complete_observability']='UNRESOLVED_PHYSICAL_BOUNDARIES'
    return result
