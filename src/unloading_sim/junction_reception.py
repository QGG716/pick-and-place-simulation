"""Seam-defined reception rules and finite-resolution height exploration.

No robot paths, archived solutions, physical transport or hardware limits are
invented here. The inlet influence corridor extends from the seam across the
transverse surface in the longitudinal belt's width direction.
"""
from collections import deque
import numpy as np
from .lifted_reception import bounds, StartHeight


def reception_classification(box, transverse, longitudinal):
    if not all(np.isfinite(b.world_from_local).all() and np.isfinite(b.half_extents).all()
               and np.all(b.half_extents>0) for b in (box,transverse,longitudinal)):
        raise ValueError("JUNCTION_NONFINITE_OR_INVALID_GEOMETRY")
    if not np.allclose(transverse.rotation, np.eye(3), atol=1e-10, rtol=0):
        raise ValueError("JUNCTION_REQUIRES_WORLD_ALIGNED_TRANSVERSE")
    # Longitudinal local +X is its upper end, local +Z its surface normal.
    upper = (longitudinal.center + longitudinal.rotation[:, 0]*longitudinal.half_extents[0]
             + longitudinal.rotation[:, 2]*longitudinal.half_extents[2])
    inlet = np.array([upper-longitudinal.rotation[:, 1]*longitudinal.half_extents[1],
                      upper+longitudinal.rotation[:, 1]*longitudinal.half_extents[1]])
    seam_x = transverse.center[0]-transverse.half_extents[0]
    if (np.max(np.abs(inlet[:, 0]-seam_x)) > 1e-8 or
            np.max(np.abs(inlet[:, 2]-bounds(transverse)[1][2])) > 1e-8):
        raise ValueError("JUNCTION_SEAM_IDENTITY_MISMATCH")
    y0, y1 = float(inlet[:, 1].min()), float(inlet[:, 1].max())
    x1 = float(transverse.center[0]+transverse.half_extents[0])
    rectangle = np.array([[seam_x,y0],[x1,y0],[x1,y1],[seam_x,y1]])
    projected = box.corners()[:, :2]
    # SAT of a projected OBB (a zonotope) and the inlet corridor. Use all
    # projected corners and all generator normals, not only the carton centre.
    axes = [np.array([1.,0.]), np.array([0.,1.])]
    for v in box.rotation[:2].T:
        if np.linalg.norm(v) > 1e-12:
            axes.append(np.array([-v[1],v[0]])/np.linalg.norm(v))
    intersects = all(min(np.max(projected@a),np.max(rectangle@a)) >=
                     max(np.min(projected@a),np.min(rectangle@a))-1e-9 for a in axes)
    return dict(region_class="LONGITUDINAL_INLET_JUNCTION" if intersects else "TRANSVERSE",
        junction=bool(intersects), outfeed_direction_world=[-1.,0.,0.] if intersects else [0.,-1.,0.],
        inlet_edge_world=inlet.tolist(), influence_polygon_world_xy_m=rectangle.tolist(),
        box_projection_world_xy_m=projected.tolist(),
        definition="FULL_PROJECTION_INTERSECTS_SEAM_INLET_WIDTH_CORRIDOR",
        physical_transport="NOT_RUN")


def reject_late_sync(option):
    if (option.get("late_receiver_sync") or option.get("synchronization") ==
            "CHECKED_LOW_APPROACH_EXTRACTION_THEN_ATTACHED_LIFT"):
        raise ValueError("EXTRACTION_THEN_LIFT_DISABLED")


def require_receive_height(world, expected, *, tolerance=1e-8):
    belts = [b for b in world if b.name == "conveyor_transverse"]
    if len(belts) != 1:
        raise ValueError("RECEIVE_HEIGHT_WORLD_IDENTITY")
    actual = float(bounds(belts[0])[1][2])
    if not np.isfinite([actual,expected]).all() or abs(actual-expected) > tolerance:
        raise ValueError("EXTRACTION_BEFORE_REQUIRED_RECEIVE_HEIGHT")
    return actual


def validate_junction_release(box, original, world, working_axis, receive_height):
    axis=np.asarray(working_axis,dtype=float)
    if axis.shape!=(3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1)>1e-8:
        raise ValueError("JUNCTION_INVALID_WORKING_AXIS")
    by_name = {b.name:b for b in world}
    region = reception_classification(box, by_name["conveyor_transverse"], by_name["conveyor_longitudinal"])
    facing_outfeed = float(axis@np.asarray(region["outfeed_direction_world"])) < -1+1e-8
    capture = StartHeight.capture(original)
    actual = require_receive_height(world, receive_height)
    if region["junction"] and facing_outfeed and abs(actual-capture.hmin) > 1e-8:
        raise ValueError("JUNCTION_FRONT_REQUIRES_HMIN")
    if abs(box.center[1]-original.center[1]) > 1e-5 or np.max(np.abs(box.rotation-original.rotation)) > 2e-4:
        raise ValueError("JUNCTION_FRONT_Y_OR_ORIENTATION_CHANGED")
    return dict(**region, actual_receive_height_m=actual, frozen_height=capture.evidence(),
                working_axis_world=list(working_axis), passed=True)


def temporary_heights(receive, minimum, *, resolution=.01):
    """Hmin first, nearby lowering, then fair interval refinement to 10 mm.

    Coarse spacing only prioritizes samples. It never rules out untested
    intervals or assumes monotonic clearance. All intervals are then bisected.
    """
    if not np.isfinite([receive,minimum,resolution]).all() or resolution<=0 or receive<minimum:
        raise ValueError("TEMPORARY_HEIGHT_DOMAIN")
    yield float(receive)
    points=[float(receive)]
    height=receive
    while height-minimum > 1e-10:
        height=max(minimum,height-5*resolution)
        points.append(float(height))
        yield float(height)
    pending=deque(zip(points[1:],points[:-1]))
    while pending:
        low,high=pending.popleft()
        if high-low <= resolution+1e-10:
            continue
        middle=(low+high)/2
        yield middle
        pending.extend(((middle,high),(low,middle)))


def synchronization_options(option, current, minimum):
    reject_late_sync(option)
    receive=float(option["height_m"])
    if (option.get("reception") or {}).get("junction"):
        for height in temporary_heights(receive, minimum):
            for where in (("ALREADY_AT_RECEIVE",) if abs(height-receive)<1e-9 else ("PREGRASP","CONTACT")):
                yield dict(option, H_current_m=float(current), H_receive_m=receive, H_temporary_m=height,
                    pre_approach_height_m=height, synchronization=where,
                    height_resolution_m=.01, third_sync_disabled=True)
    else:
        yield option
        if current > minimum+.005:
            yield dict(option, synchronization="PREGRASP_OR_CONTACT",pre_approach_height_m=minimum)
