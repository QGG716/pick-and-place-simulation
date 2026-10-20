"""Time allocations and legal bounded-angle variants for local recovery.

These are solver time slices, not request-count quotas or business candidates.
The caller owns the shared absolute task/stage deadline.
"""
import numpy as np


def native_schedule(initial, *, budget_driven, legacy_attempts, deadline, now,
                    allow_ompl, constrained):
    for method in initial:
        if deadline is not None and now()>=deadline:return
        yield method
    if allow_ompl and not constrained:
        if budget_driven:
            if deadline is None or not np.isfinite(deadline):
                raise ValueError("BUDGET_SCHEDULE_REQUIRES_FINITE_DEADLINE")
            while now()<deadline:
                yield ("ompl","RRTConnectkConfigDefault")
        else:
            for _ in range(legacy_attempts):
                yield ("ompl","RRTConnectkConfigDefault")


def equivalent_bounded_configurations(robot, q):
    """Optional 2pi representations; every link frame, limit and q are checked.

    Original representation remains present. No trajectory is edited and no
    intermediate path is inferred to be equivalent.
    """
    q=np.asarray(q,float)
    result=[q.copy()]
    if not hasattr(robot,"joint_limits") or not hasattr(robot,"named_link_frames"):
        return result
    original=robot.named_link_frames(q)
    # A Cartesian product of finite within-limit angle representations only;
    # the robot model remains bounded revolute. Not an IK/business-pose scan.
    for joint in range(len(q)):
        candidates=[]
        for base in result:
            lo,hi=robot.joint_limits[joint]
            for turns in range(int(np.ceil((lo-base[joint])/(2*np.pi))),
                               int(np.floor((hi-base[joint])/(2*np.pi)))+1):
                if turns==0:continue
                variant=base.copy();variant[joint]+=turns*2*np.pi
                if not robot.within_limits(variant):continue
                frames=robot.named_link_frames(variant)
                if set(frames)==set(original) and all(np.allclose(frames[k],original[k],atol=1e-9,rtol=0) for k in original):
                    candidates.append(variant)
        result.extend(candidates)
    return result
