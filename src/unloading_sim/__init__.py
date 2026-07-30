"""Fast geometric simulation for industrial-arm trailer unloading."""

from .geometry import Capsule, OBB
from .grasp import PickPlan, SuctionGraspCandidate, plan_pick
from .ik import IKResult, solve_ik, solve_ik_multistart
from .planner import PlanResult, RRTConnectPlanner
from .robot import DHRobot6, RobotKinematics6, URDFRobot6
from .scene import TrailerScene, load_scene_config

__all__ = [
    "Capsule",
    "OBB",
    "PickPlan",
    "SuctionGraspCandidate",
    "plan_pick",
    "IKResult",
    "solve_ik",
    "solve_ik_multistart",
    "PlanResult",
    "RRTConnectPlanner",
    "DHRobot6",
    "RobotKinematics6",
    "URDFRobot6",
    "TrailerScene",
    "load_scene_config",
]
