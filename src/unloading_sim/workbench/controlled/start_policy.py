"""Pure, shared interpretation of digest-bound first-start limits."""
import math


def start_limits(policy):
    if policy is not None and not isinstance(policy,dict):raise ValueError('START_CHANGED_REPLAN_REQUIRED: INVALID_POLICY')
    policy = policy or {}
    tolerance = policy.get('tolerance_deg', .01)
    cap = policy.get('hard_cap_deg', .5)
    for value, maximum in ((tolerance, .05), (cap, .5)):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError('INVALID_START_JOINT_LIMIT')
    return math.radians(tolerance), math.radians(cap)
