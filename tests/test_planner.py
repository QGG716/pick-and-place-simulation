import numpy as np

from unloading_sim.grasp import _conveyor_preplace_poses, _plan_wrist_first_carry
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.robot import URDFRobot6


def test_rrt_connect_around_joint_space_obstacle():
    def valid(q):
        # Circular forbidden region in a two-dimensional toy joint space.
        return np.linalg.norm(q) > 0.35

    planner = RRTConnectPlanner(
        lower_limits=np.array([-1.0, -1.0]),
        upper_limits=np.array([1.0, 1.0]),
        is_state_valid=valid,
        step_size=0.12,
        edge_resolution=0.03,
        max_iterations=3000,
        rng=np.random.default_rng(3),
    )
    result = planner.plan(np.array([-0.8, 0.0]), np.array([0.8, 0.0]))
    assert result.success
    assert all(valid(q) for q in planner.densify(result.path, 0.02))


def test_rrt_connect_with_kuka_urdf_limits():
    robot = URDFRobot6.kuka_kr50_r2500()
    start = np.array([0.0, -1.0, 0.8, 0.0, -0.4, 0.0])
    goal = np.array([0.35, -0.75, 0.55, 0.45, -0.15, 0.25])
    planner = RRTConnectPlanner(
        lower_limits=robot.joint_limits[:, 0],
        upper_limits=robot.joint_limits[:, 1],
        is_state_valid=robot.within_limits,
        step_size=0.25,
        edge_resolution=0.05,
        max_iterations=500,
        rng=np.random.default_rng(5),
    )
    result = planner.plan(start, goal)
    assert result.success
    assert all(robot.within_limits(q) for q in planner.densify(result.path, 0.05))


def test_wrist_first_carry_presets_wrist_before_turning_j1():
    start = np.array([0.1, -1.2, 0.8, -0.2, 0.3, -0.4])
    goal = np.array([1.0, -1.0, 0.85, 0.6, -0.2, 0.5])
    result = _plan_wrist_first_carry(start, goal, lambda q: True, resolution=0.05)

    assert result.success
    j1_turn_start = next(index for index, q in enumerate(result.path) if q[0] > start[0] + 1e-8)
    assert np.allclose(result.path[j1_turn_start - 1][3:], goal[3:])
    assert np.allclose(result.path[j1_turn_start - 1][1:3], start[1:3])


def test_conveyor_preplace_poses_cover_each_requested_approach_direction():
    release = np.eye(4)
    release[:3, 3] = [0.4, -0.2, 0.8]

    poses = dict(_conveyor_preplace_poses(release, 0.30))

    assert set(poses) == {"front", "top", "left", "right"}
    assert all(np.allclose(pose[:3, :3], release[:3, :3]) for pose in poses.values())
    assert np.allclose(poses["front"][:3, 3], [0.7, -0.2, 0.8])
    assert np.allclose(poses["top"][:3, 3], [0.4, -0.2, 1.1])
    assert np.allclose(poses["left"][:3, 3], [0.4, 0.1, 0.8])
    assert np.allclose(poses["right"][:3, 3], [0.4, -0.5, 0.8])


def test_conveyor_preplace_directions_follow_conveyor_frame():
    release = np.eye(4)
    quarter_turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

    poses = dict(_conveyor_preplace_poses(release, 0.30, ("front", "left"), quarter_turn))

    assert np.allclose(poses["front"][:3, 3], [0.0, 0.3, 0.0])
    assert np.allclose(poses["left"][:3, 3], [-0.3, 0.0, 0.0])
