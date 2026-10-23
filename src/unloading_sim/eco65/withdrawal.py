"""Finite empty-tool retreats after reception; these are native goal candidates."""
import numpy as np


def retreat_candidates(tool_points, received_box_points, face_normal):
    """Keep normal withdrawal first; also permit upward empty-tool separation.

    Projection of every real tool solid guarantees a 12 mm separating slab at
    the candidate endpoint. This does not certify the path or robot clearance:
    each candidate still needs a native LIN and the native drain clearance gate.
    No attached-carton motion is generated here.
    """
    tool=np.asarray(tool_points,float);box=np.asarray(received_box_points,float)
    normal=np.asarray(face_normal,float)
    if (tool.ndim!=2 or box.ndim!=2 or tool.shape[1:]!=(3,) or box.shape[1:]!=(3,)
            or not len(tool) or not len(box) or normal.shape!=(3,)
            or not np.isfinite(tool).all() or not np.isfinite(box).all()
            or not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1)>1e-8):
        raise ValueError('WITHDRAWAL_GEOMETRY_INVALID')
    axes=[('FACE_NORMAL',normal,.002)]
    up=np.array([0.,0.,1.])
    if np.linalg.norm(normal-up)>1e-8:
        axes.append(('EMPTY_TOOL_UP',up,.010))
    result=[]
    for label,axis,minimum in axes:
        tool_min=float(np.min(tool@axis));box_max=float(np.max(box@axis))
        distance=max(minimum,.012+box_max-tool_min)
        for extra in (0.,.010):
            result.append(dict(strategy=label,offset_m=(axis*(distance+extra)).tolist(),
                separating_axis=axis.tolist(),projected_tool_min_m=tool_min,
                projected_box_max_m=box_max,required_separation_m=.010,
                endpoint_projected_gap_m=tool_min+distance+extra-box_max))
    return result
