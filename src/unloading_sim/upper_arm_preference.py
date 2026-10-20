"""Bounded ranking and reproducible numerical seeds, never an acceptance gate."""
import numpy as np


def upper_arm(robot, q):
    # Both production Pinocchio and lightweight URDF backends expose link FK.
    # Official J2/J3 child frames are at the structural joint origins; joint
    # rotation changes their orientation, not their origin position.
    frames = robot.named_link_frames(q)
    shoulder, elbow = frames["J2_link"][:3,3], frames["J3_link"][:3,3]
    u = elbow-shoulder
    length = float(np.linalg.norm(u))
    if not np.isfinite(length) or length < 1e-9:
        raise ValueError("UPPER_ARM_STRUCTURAL_JOINTS_UNDEFINED")
    alpha = float(np.arctan2(u[2],np.linalg.norm(u[:2])))
    return dict(shoulder_world_m=shoulder.tolist(), elbow_world_m=elbow.tolist(),
        direction_world=(u/length).tolist(), length_m=length,
        elevation_rad=alpha, elevation_deg=float(np.degrees(alpha)),
        q2_rad=float(q[1]), definition="OFFICIAL_J2_J3_JOINT_ORIGINS_WORLD_FK")


def soft_cost(robot, q, start, clearance_m=None):
    arm = upper_arm(robot, q)
    raised = (.5-arm["elevation_rad"]/np.pi)  # [0,1], no threshold
    travel = float(np.mean(np.minimum(1.,np.abs(np.asarray(q)-start)/np.ptp(robot.joint_limits,axis=1))))
    clearance = .5 if clearance_m is None else float(1-np.clip(clearance_m/.05,0,1))
    return dict(cost=.35*raised+.45*travel+.20*clearance,
        raised_cost=raised, motion_continuity_cost=travel, clearance_cost=clearance,
        clearance_known=clearance_m is not None, weights=[.35,.45,.20], **arm)


def numerical_seed(robot, start, seed, index):
    """Alternate original, general and softly ranked full-joint initial guesses."""
    if index == 0: return np.asarray(start).copy()
    rng = np.random.default_rng(np.random.SeedSequence([int(seed),int(index)]))
    guesses = rng.uniform(robot.joint_limits[:,0],robot.joint_limits[:,1],(8,6))
    if index % 2: return guesses[0]  # retain general seeds throughout
    return min(guesses,key=lambda q:soft_cost(robot,q,start)["cost"]).copy()
