"""Explicit FRONT posture family measured from official structural joint FK.

This optional user-selected process gate supplements all collision/FK checks.
It does not infer elbow height from joint angle signs or alter the business pose.
"""
import numpy as np
from .upper_arm_preference import upper_arm

POLICY = 'UPRIGHT_UPPER_FORWARD_FOREARM_V1'


def front_arm_posture(robot, q, *, initial=False):
    upper = upper_arm(robot, q)
    frames = robot.named_link_frames(q)
    root, wrist = frames['J4_link'][:3, 3], frames['J5_link'][:3, 3]
    direction = wrist - root
    length = float(np.linalg.norm(direction))
    if not np.isfinite(length) or length < 1e-9:
        raise ValueError('FOREARM_STRUCTURAL_JOINTS_UNDEFINED')
    direction = direction / length
    elevation = float(np.degrees(np.arctan2(direction[2], np.linalg.norm(direction[:2]))))
    # Home is nearly vertical/horizontal. During FRONT contact the upper arm
    # stays vertically dominant; the forearm remains within 30 deg of horizontal.
    minimum_upper, maximum_forearm = (75., 20.) if initial else (45., 30.)
    passed = (upper['elevation_deg'] >= minimum_upper - 1e-8
              and abs(elevation) <= maximum_forearm + 1e-8 and direction[0] > 0.)
    return dict(policy=POLICY, passed=bool(passed), initial=initial,
        upper_arm=upper, forearm_root_world_m=root.tolist(), wrist_world_m=wrist.tolist(),
        forearm_direction_world=direction.tolist(), forearm_length_m=length,
        forearm_elevation_deg=elevation, minimum_upper_elevation_deg=minimum_upper,
        maximum_forearm_absolute_elevation_deg=maximum_forearm,
        forearm_forward_world_plus_x_required=True, collision_checks_unchanged=True)
