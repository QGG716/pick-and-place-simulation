"""Descriptive path costs. Bounded revolute angles are never wrapped."""
import numpy as np

# Equal dimensionless weights, fixed before all comparisons; radians and metres.
JOINT_WEIGHTS = np.ones(6)


def path_metrics(robot, points, flange_from_contact=None, planning_wall_s=None):
    q = np.asarray([p["q"] for p in points], dtype=float)
    if q.ndim != 2 or q.shape[1] != 6 or not len(q) or not np.isfinite(q).all():
        raise ValueError("INVALID_QUALITY_PATH")
    delta = np.diff(q, axis=0)
    tcp = np.asarray([robot.fk(x)[:3, 3] for x in q])
    result = dict(joint_weights=JOINT_WEIGHTS.tolist(), joint_units="rad",
        distance_rule="bounded_revolute_unwrapped_weighted_L1",
        q_start_rad=q[0].tolist(), q_goal_rad=q[-1].tolist(),
        endpoint_delta_rad=(q[-1]-q[0]).tolist(),
        cumulative_abs_joint_rad=np.abs(delta).sum(axis=0).tolist(),
        weighted_joint_length_rad=float((np.abs(delta)*JOINT_WEIGHTS).sum()),
        endpoint_joint_distance_rad=float((np.abs(q[-1]-q[0])*JOINT_WEIGHTS).sum()),
        motion_duration_s=float(points[-1]["t"]-points[0]["t"]),
        planning_wall_s=planning_wall_s)
    for name, xyz in [("tcp", tcp)] + ([] if flange_from_contact is None else [
        ("physical_contact", np.asarray([(robot.named_link_frames(x)["flange"] @
            flange_from_contact)[:3, 3] for x in q]))]):
        length = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
        chord = float(np.linalg.norm(xyz[-1]-xyz[0]))
        result[name+"_length_m"] = length
        result[name+"_chord_m"] = chord
        result[name+"_length_over_chord"] = length/chord if chord > 1e-12 else None
    return result
