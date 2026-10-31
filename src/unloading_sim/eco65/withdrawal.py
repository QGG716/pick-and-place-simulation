"""Finite empty-tool retreats after reception; these are native goal candidates."""
import numpy as np


def retreat_candidates(tool_points, received_box_points, face_normal, *, outfeed_points=None):
    """Prefer separation from a supplied outfeed envelope, otherwise the face.

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
    up=np.array([0.,0.,1.])
    axes=[]
    if outfeed_points is not None:
        future=np.asarray(outfeed_points,float)
        if (future.ndim!=2 or future.shape[1:]!=(3,) or not len(future)
                or not np.isfinite(future).all()):
            raise ValueError('WITHDRAWAL_OUTFEED_GEOMETRY_INVALID')
        # The longitudinal belt can rise away from a 100 mm receiver to its
        # 150 mm far end. Separate from the whole prescribed carton route,
        # not just the landed carton. This is a goal candidate, not collision
        # acceptance: the real LIN, robot and outfeed still need native checks.
        minimum=.002 if np.linalg.norm(normal-up)<1e-8 else .010
        axes.append(('IDEAL_OUTFEED_ENVELOPE_UP',up,minimum,np.vstack([box,future])))
    axes.append(('FACE_NORMAL',normal,.002,box))
    if np.linalg.norm(normal-up)>1e-8:
        axes.append(('EMPTY_TOOL_UP',up,.010,box))
    result=[]
    for label,axis,minimum,envelope in axes:
        tool_min=float(np.min(tool@axis));box_max=float(np.max(envelope@axis))
        distance=max(minimum,.012+box_max-tool_min)
        for extra in (0.,.010):
            result.append(dict(strategy=label,offset_m=(axis*(distance+extra)).tolist(),
                separating_axis=axis.tolist(),projected_tool_min_m=tool_min,
                projected_box_max_m=box_max,required_separation_m=.010,
                endpoint_projected_gap_m=tool_min+distance+extra-box_max,
                box_envelope_scope=('COMPLETE_IDEAL_OUTFEED_ROUTE' if label=='IDEAL_OUTFEED_ENVELOPE_UP'
                                    else 'RECEIVED_CARTON')))
    return result
