"""Initial-state selection only: face the stack without fixing any joint angle.

These directional checks never apply to grasp approach/contact endpoints.
Joint limits, tool orientation and collision validation are separate gates.
"""
import numpy as np


def front_working_initial_evidence(robot, q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('INVALID_INITIAL_CONFIGURATION')
    frames = robot.named_link_frames(q)
    upper = frames['J3_link'][:3, 3] - frames['J2_link'][:3, 3]
    forearm = frames['J5_link'][:3, 3] - frames['J4_link'][:3, 3]
    for direction in (upper, forearm):
        length = np.linalg.norm(direction)
        if not np.isfinite(length) or length < 1e-9:
            raise ValueError('INITIAL_ARM_STRUCTURAL_DIRECTION_UNDEFINED')
        direction /= length
    body_forward = float(frames['J1_link'][0, 0])
    passed = bool(body_forward > 0 and upper[0] >= -1e-9 and forearm[0] > 0)
    # Continuous preference, never an equality constraint or a near-zero window.
    j1_cost = float(abs(q[0]) / np.pi)
    # Prefer less forearm roll (J4); the tool's prescribed grid orientation may
    # legitimately require a larger J6 angle. Neither joint is locked.
    wrist_cost = float((.8 * abs(q[3]) + .2 * abs(q[5])) / (2 * np.pi))
    return dict(policy='STACK_FORWARD_INITIAL_ONLY_V1', passed=passed,
        scope='STATIC_INITIAL_STATE_ONLY', affects_grasp_endpoints=False,
        upper_direction_world=upper.tolist(), forearm_direction_world=forearm.tolist(),
        body_forward_world_x=body_forward, j1_rad=float(q[0]),
        j1_deg=float(np.degrees(q[0])), j1_fixed=False,
        j1_near_zero_hard_window=False, exact_upright_or_horizontal_required=False,
        selection_cost=.8 * j1_cost + .2 * wrist_cost,
        preference_components=dict(j1_near_zero=j1_cost, bounded_wrist_rotation=wrist_cost))
