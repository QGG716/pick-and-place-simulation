import numpy as np

from unloading_sim.geometry import make_tool_rotation, make_transform
from unloading_sim.ik import solve_ik_multistart
from unloading_sim.robot import DHRobot6, URDFRobot6


def test_fk_jacobian_shape_and_ik():
    robot = DHRobot6.ur5e_like(base_rpy=[0.0, 0.0, np.pi])
    home = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    assert robot.fk(home).shape == (4, 4)
    assert robot.geometric_jacobian(home).shape == (6, 6)

    target = make_transform(make_tool_rotation(np.array([1.0, 0.0, 0.0])), [0.10, 0.0, 1.00])
    result = solve_ik_multistart(
        robot,
        target,
        seeds=[home],
        random_restarts=20,
        rng=np.random.default_rng(2),
        position_tolerance=0.012,
        orientation_tolerance=0.10,
    )
    assert result.success


def test_kuka_urdf_fk_jacobian_and_ik():
    robot = URDFRobot6.kuka_kr50_r2500(tool_length=0.0)
    assert (2, 4) in robot.self_collision_exclusions
    q = np.array([0.15, -0.75, 0.55, 0.25, -0.45, 0.2])
    assert robot.fk(q).shape == (4, 4)
    assert robot.geometric_jacobian(q).shape == (6, 6)
    assert robot.within_limits(q)

    result = solve_ik_multistart(
        robot,
        robot.fk(q),
        seeds=[np.zeros(6)],
        random_restarts=8,
        rng=np.random.default_rng(4),
    )
    assert result.success
    assert result.position_error < 0.012
    assert result.orientation_error < 0.10


def test_fanuc_m20id35_urdf_fk_jacobian_and_ik():
    robot = URDFRobot6.fanuc_m20id35(tool_length=0.0)
    q = np.array([-0.25, -0.65, -0.75, 0.45, -1.10, 0.20])
    assert robot.name == "fanuc_m20id35"
    assert robot.fk(q).shape == (4, 4)
    assert robot.geometric_jacobian(q).shape == (6, 6)
    assert robot.within_limits(q)

    result = solve_ik_multistart(
        robot,
        robot.fk(q),
        seeds=[q + 0.05],
        random_restarts=4,
        rng=np.random.default_rng(14),
    )
    assert result.success


def test_link_elevation_uses_physical_link_direction():
    robot = URDFRobot6.kuka_kr50_r2500()
    home = np.array([0.6964, -0.3128, 1.8886, -0.4056, -0.5357, -0.2895])

    assert robot.link_elevation_degrees(home, 3) < 30.0
