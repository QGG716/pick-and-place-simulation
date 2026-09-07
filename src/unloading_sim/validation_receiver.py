"""Supported belt transport with a retained receiver occupant."""
from __future__ import annotations

import numpy as np
from .geometry import OBB
from .validation_physics import world_link_boxes


def supported_by_deck_union(carton: OBB, decks, tolerance: float) -> bool:
    """Exact rectangle-union coverage for upright axis-aligned test cartons."""
    if not np.allclose(np.abs(carton.rotation),np.eye(3),atol=1e-8):
        return False
    lower=carton.center-carton.half_extents;upper=carton.center+carton.half_extents
    supports=[d for d in decks if abs(d.center[2]+d.half_extents[2]-lower[2])<=tolerance]
    xs=sorted({lower[0],upper[0],*[float(np.clip(d.center[0]+sign*d.half_extents[0],lower[0],upper[0])) for d in supports for sign in (-1,1)]})
    ys=sorted({lower[1],upper[1],*[float(np.clip(d.center[1]+sign*d.half_extents[1],lower[1],upper[1])) for d in supports for sign in (-1,1)]})
    return bool(supports) and all(any(abs((x0+x1)/2-d.center[0])<=d.half_extents[0]+1e-12 and abs((y0+y1)/2-d.center[1])<=d.half_extents[1]+1e-12 for d in supports)
        for x0,x1 in zip(xs[:-1],xs[1:]) for y0,y1 in zip(ys[:-1],ys[1:]))


def clear_receiving_area(cell,placed,belt,q,remaining,received):
    """Try the longitudinal belt to the cross-leg center without teleporting.

    A carton that cannot be supported at the L corner remains at its release
    pose. This function certifies geometry only, not belt traction/dynamics.
    """
    decks=cell.decks(belt);destination=placed.center.copy();destination[0]=decks[0].center[0]
    delta=destination-placed.center;n=max(1,int(np.ceil(np.linalg.norm(delta)/cell.d['conveyor']['movement_step_m'])))
    paths=[];arm=world_link_boxes(cell.robot,q,cell.shapes);tool=cell.robot.tool_collision_obb(q)
    obstacles=[*cell.fixtures(),*remaining,*received,*arm,*([tool] if tool is not None else [])]
    for u in np.linspace(0,1,2*n+1):
        moved=OBB(placed.center+u*delta,placed.half_extents,placed.rotation,placed.name,placed.category)
        paths.append(moved.world_from_local.tolist())
        if not supported_by_deck_union(moved,decks,cell.p['support_tolerance_m']):
            return None,{'status':'FAIL','reason':'L_CORNER_SUPPORT_GAP','fraction':float(u),'candidate_box_path':paths,'retained_at_release':True}
        for obstacle in obstacles:
            if moved.intersects_obb(obstacle,margin=cell.p['collision_margin_m']):
                return None,{'status':'FAIL','reason':'BELT_TRANSPORT_COLLISION','pair':[moved.name,obstacle.name],'fraction':float(u),'candidate_box_path':paths,'retained_at_release':True}
    return moved,{'status':'PASS_GEOMETRIC_ONLY','reason':'RECEIVING_AREA_CLEARED','box_path':paths,
                  'transport_seconds':float(np.linalg.norm(delta)/cell.d['conveyor']['belt_speed_m_s']),
                  'belt_drive_and_friction_status':'NOT_EVALUATED'}
