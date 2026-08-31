import numpy as np

from unloading_sim.geometry import OBB, make_tool_rotation, make_transform
from unloading_sim.ik import solve_ik, solve_ik_multistart
from unloading_sim.robot import DHRobot6, URDFRobot, URDFRobot6


def test_ik_accepts_solution_reached_on_final_update():
    class OneAxisRobot:
        joint_limits = np.array([[-2.0, 2.0]])

        def clamp(self, q):
            return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

        def fk(self, q):
            pose = np.eye(4)
            pose[0, 3] = q[0]
            return pose

        def geometric_jacobian(self, _q):
            jacobian = np.zeros((6, 1))
            jacobian[0, 0] = 1.0
            return jacobian

        def is_collision_free(self, _q, _obstacles, ignored_obstacle_names=None):
            return True

    target = np.eye(4)
    target[0, 3] = 0.5

    result = solve_ik(
        OneAxisRobot(),
        target,
        np.zeros(1),
        max_iterations=1,
        damping=1e-6,
        max_step=1.0,
        position_tolerance=1e-6,
    )

    assert result.success
    assert result.message == "converged on final update"


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


def test_fanuc_tool_envelope_participates_in_core_collision_checks():
    robot = URDFRobot6.fanuc_m20id35(
        tool_length=0.2275,
        tool_collision_size=[0.288, 0.576, 0.024],
        tool_collision_center_offset=0.0655,
    )
    q = np.array([-0.25, -0.65, -0.75, 0.45, -1.10, 0.20])
    envelope = robot.tool_collision_obb(q)
    assert envelope is not None
    assert np.allclose(envelope.half_extents, [0.144, 0.288, 0.012])
    working_plane = robot.fk(q)
    expected_center = working_plane[:3, :3] @ [0.0, 0.0, -0.0655] + working_plane[:3, 3]
    assert np.allclose(envelope.center, expected_center)
    obstacle = OBB(
        center=envelope.center,
        half_extents=np.full(3, 0.01),
        rotation=np.eye(3),
        name="tool_probe",
    )
    collision = robot.collision_result(q, [obstacle], margin=0.0, check_self=False)
    assert collision.in_collision
    assert collision.first_link == "tool_envelope"


def test_link_elevation_uses_physical_link_direction():
    robot = URDFRobot6.kuka_kr50_r2500()
    home = np.array([0.6964, -0.3128, 1.8886, -0.4056, -0.5357, -0.2895])

    assert robot.link_elevation_degrees(home, 3) < 30.0


def test_generic_urdf_backend_supports_non_six_dof(tmp_path):
    urdf = tmp_path / "two_joint.urdf"
    urdf.write_text(
        """<robot name="two_joint">
  <link name="base_link"/><link name="link1"/><link name="tool0"/>
  <joint name="slide" type="prismatic">
    <parent link="base_link"/><child link="link1"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="turn" type="revolute">
    <parent link="link1"/><child link="tool0"/><origin xyz="0 0 0.5"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>""",
        encoding="utf-8",
    )
    robot = URDFRobot.from_urdf(
        urdf,
        active_joint_names=["slide", "turn"],
        base_link="base_link",
        tip_link="tool0",
        tool_length=0.1,
        link_radii=[0.05, 0.04],
    )
    q = np.array([0.2, 0.3])
    assert robot.dof == 2
    assert robot.joint_limits.shape == (2, 2)
    assert robot.geometric_jacobian(q).shape == (6, 2)
    assert len(robot.link_capsules(q)) == 3
