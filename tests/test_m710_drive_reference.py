"""Control math is independent of Isaac; finite drive limits remain authoritative."""
from pathlib import Path
import ast

import numpy as np
import pytest

from unloading_sim.m710_replay_physics import (
    BoundedFreeTransitGate, BoundedTargetCupReleaseClearance, finite_gravity_compensated_drive_target,
    payload_gravity_compensation, sample_joint_reference, resolve_actual_task_stage,
)


def test_reference_positions_exactly_preserve_existing_interpolation_and_derivative():
    rng = np.random.default_rng(9281)
    times = np.r_[0., np.cumsum(rng.uniform(.01, .2, 100))]
    values = rng.normal(size=(len(times), 6))
    for time in np.r_[-1., times, rng.uniform(0., times[-1], 200), times[-1] + 1.]:
        q, qd = sample_joint_reference(times, values, time)
        previous = np.array([np.interp(np.clip(time, 0., times[-1]), times, values[:, j]) for j in range(6)])
        np.testing.assert_array_equal(q.astype(np.float32), previous.astype(np.float32))
        if 0 < time < times[-1]:
            index = np.searchsorted(times, time, side="right") - 1
            np.testing.assert_array_equal(qd, (values[index + 1] - values[index]) / (times[index + 1] - times[index]))
        else:
            np.testing.assert_array_equal(qd, np.zeros(6))
        held_q, held_qd = sample_joint_reference(times, values, time, held=True)
        np.testing.assert_array_equal(held_q, q)
        np.testing.assert_array_equal(held_qd, np.zeros(6))


def test_internal_knot_uses_right_segment_not_previous_velocity():
    q, qd = sample_joint_reference([0., 1., 3.], np.array([[0.], [1.], [0.]]), 1.)
    assert q == [1.]
    assert qd == [-.5]


def test_reference_does_not_clip_a_bad_schedule_to_make_it_appear_valid():
    _, qd = sample_joint_reference([0., .01], np.array([[0.], [1.]]), .005)
    assert qd == [100.]  # Runtime rejects against the bound official limit.


def test_actual_contact_lifecycle_spans_controller_dwell_after_cpu_contact_window():
    windows = [{"stage": "pregrasp", "start_time_s": 0., "end_time_s": 2.110486},
               {"stage": "contact", "start_time_s": 2.110486, "end_time_s": 6.4063225},
               {"stage": "extraction", "start_time_s": 6.4063225, "end_time_s": 33.232902}]
    def stage(t, commanded=False, wait=None):
        return resolve_actual_task_stage(windows, t, grasp_commanded=commanded,
                                          grasp_event_time_s=6.6063225, contact_wait_started_s=wait)
    assert stage(1.) == "pregrasp"
    assert stage(5.55) == "contact"
    assert stage(6.4125) == "contact"  # Real previous run's finite-separation rejection.
    assert stage(6.6063225) == "contact"
    assert stage(6.7, wait=6.6083) == "contact"  # Bounded actual wait is runtime-owned.
    assert stage(6.7, commanded=True, wait=6.6083) == "extraction"
    assert stage(7.) == "extraction"  # Cannot authorize arbitrary unattached motion.


def test_actual_place_lifecycle_spans_release_dwell_until_constraint_removal():
    windows = [
        {"stage": "transit", "start_time_s": 33.23, "end_time_s": 100.3400965},
        {"stage": "place", "start_time_s": 100.3400965, "end_time_s": 105.0922653},
        {"stage": "withdrawal", "start_time_s": 105.0922653, "end_time_s": 109.53},
    ]

    def stage(t, *, attached=True, released=False, wait=None):
        return resolve_actual_task_stage(
            windows,
            t,
            grasp_commanded=True,
            grasp_event_time_s=6.606,
            contact_wait_started_s=None,
            attached=attached,
            release_commanded=released,
            release_event_time_s=105.1922653,
            support_wait_started_s=wait,
        )

    assert stage(105.095) == "place"  # Actual round timing: CPU PLACE has ended.
    assert stage(105.1922653) == "place"
    assert stage(105.3, wait=105.1923) == "place"  # Runtime-owned bounded wait.
    assert stage(105.3, released=True, attached=False) == "withdrawal"
    assert stage(105.3) == "withdrawal"  # No arbitrary late attached permission.


def test_external_gravity_compensation_matches_potential_derivative_and_sign():
    # Revolute Y joint: world x lever arm positive => negative compensation.
    # FK TCP at [cos(q),0,-sin(q)], payload COM further 0.3m along rotated X.
    for angle in [-.9, 0., .7]:
        c, s = np.cos(angle), np.sin(angle)
        tcp = np.array([c, 0., -s])
        com = 1.3 * tcp
        jac = np.array([[-s], [0.], [-c], [0.], [1.], [0.]])
        tau = payload_gravity_compensation(jac, tcp, com, 42.5, [0., 0., -9.81])
        eps = 1.e-6
        potential = lambda q: 42.5 * 9.81 * (-1.3 * np.sin(q))
        derivative = (potential(angle + eps) - potential(angle - eps)) / (2 * eps)
        assert tau[0] == pytest.approx(derivative, abs=1.e-7)
        assert tau[0] < 0.


def test_official_robot_external_gravity_matches_fixed_attachment_potential_gradient():
    from unloading_sim.robot import URDFRobot
    path = Path(__file__).resolve().parents[1] / "assets/robots/fanuc_m710id_70/official/fanuc_m710_description/urdf/m710id_70_official.urdf"
    robot = URDFRobot.from_urdf(path, active_joint_names=[f"J{i}" for i in range(1, 7)],
                               tip_link="J6_link", tool_length=0.)
    robot.tip_from_tcp = np.eye(4)
    robot.tip_from_tcp[0, 3] = .25
    q = np.array([-.8406665921, -.7738604546, -.4879170954, 1.837798357, .8868675828, -5.1062703133])
    local_com = np.array([.267641663671, -.0000308125133, .0238805172635])
    gravity = np.array([0., 0., -9.81])
    def com_at(q):
        transform = robot.fk(q)
        return transform[:3, 3] + transform[:3, :3] @ local_com
    for offset in (np.zeros(6), [.07, -.04, .03, .05, -.02, .04], [-.04, .02, -.06, -.03, .05, -.02]):
        state = q + offset
        tau = payload_gravity_compensation(robot.geometric_jacobian(state), robot.fk(state)[:3, 3], com_at(state), 42.5, gravity)
        eps = 1e-6
        numerical = np.array([-42.5 * gravity @ (com_at(state + e * eps) - com_at(state - e * eps)) / (2 * eps)
                              for e in np.eye(6)])
        np.testing.assert_allclose(tau, numerical, atol=2e-5, rtol=0.)


def test_gravity_wrench_is_frame_invariant_and_tcp_origin_invariant():
    rng = np.random.default_rng(1938)
    jac = rng.normal(size=(6, 6))
    tcp, com, shift = rng.normal(size=(3, 3))
    g = np.array([0., 0., -9.81])
    expected = payload_gravity_compensation(jac, tcp, com, 42.5, g)
    rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    rotated = np.vstack((rotation @ jac[:3], rotation @ jac[3:]))
    np.testing.assert_allclose(payload_gravity_compensation(
        rotated, rotation @ tcp + shift, rotation @ com + shift, 42.5, rotation @ g), expected, atol=1e-12)
    shifted = jac.copy()
    shifted[:3] += np.cross(jac[3:].T, shift).T
    np.testing.assert_allclose(payload_gravity_compensation(
        shifted, tcp + shift, com, 42.5, g), expected, atol=1e-12)


@pytest.mark.parametrize("mass", [0., -1., np.nan, np.inf])
def test_gravity_rejects_unknown_payload_mass(mass):
    with pytest.raises(ValueError):
        payload_gravity_compensation(np.zeros((6, 6)), [0]*3, [0]*3, mass, [0, 0, -9.81])


def test_combined_robot_payload_ff_stays_inside_same_finite_drive():
    target = finite_gravity_compensated_drive_target([.2, -.1], np.array([200., -30.]) + [300., -300.], [1000., 500.], [250., 200.])
    np.testing.assert_allclose((target - [.2, -.1]) * [1000., 500.], [250., -200.])


def test_gate_clamps_exact_boundary_pauses_clock_and_uses_actual_state():
    gate = BoundedFreeTransitGate(1.03, 1.)
    assert gate.advance(1., .1, 4.) == 1.03
    assert not gate.evaluate(1., 1., False)["hold"]
    assert gate.evaluate(1.03, 1.1, False) == {"hold": True, "reason": None}
    assert gate.evaluate(1.03, 1.9, False) == {"hold": True, "reason": None}
    assert gate.evaluate(1.03, 2., True) == {"hold": False, "reason": None}
    assert gate.events[-1]["wait_elapsed_s"] == pytest.approx(.9)
    assert gate.advance(1.03, .1, 4.) == pytest.approx(1.13)


def test_gate_timeout_retains_original_failure_and_no_clearance_override():
    gate = BoundedFreeTransitGate(3., 1.)
    gate.evaluate(3., 5., False)
    assert gate.evaluate(3., 5.999, False)["reason"] is None
    assert gate.evaluate(3., 6., False)["reason"] == "ACTUAL_PAYLOAD_NOT_CLEAR_FOR_FREE_TRANSIT"
    assert not gate.passed


def test_release_clearance_waits_for_last_exact_target_cup_lost_not_constraint_independence():
    gate = BoundedTargetCupReleaseClearance(.75)
    target = "/target"
    cups = {"/robot/cup0", "/robot/cup1"}
    first = ("/robot", target, "/robot/cup0", target)
    second = ("/robot", target, "/robot/cup1", target)
    neighbor = ("/neighbor", "/robot", "/neighbor", "/robot/cup0")
    rigid = ("/robot", target, "/robot/insert", target)
    active = {first, second, neighbor, rigid}
    gate.begin(4.)
    assert gate.observe(4.01, active, target_path=target, compliant_paths=cups) is None
    assert gate.pending
    active.remove(first)
    assert gate.observe(4.1, active, target_path=target, compliant_paths=cups) is None
    assert gate.pending  # Even after independent relative motion, proximity remains.
    active.remove(second)
    assert gate.observe(4.2, active, target_path=target, compliant_paths=cups) is None
    assert not gate.pending  # Neighbor/rigid never acquire target-cup permission.
    active.add(first)
    gate.observe(4.3, active, target_path=target, compliant_paths=cups)
    assert not gate.pending  # Recontact cannot reopen the exemption.
    with pytest.raises(ValueError, match="restarted"):
        gate.begin(4.4)


def test_release_clearance_has_independent_timeout_without_holding_trajectory():
    gate = BoundedTargetCupReleaseClearance(.75)
    gate.begin(2.)
    active = {("/robot", "/target", "/robot/cup", "/target")}
    assert gate.observe(2.749, active, target_path="/target", compliant_paths={"/robot/cup"}) is None
    assert gate.observe(2.75, active, target_path="/target", compliant_paths={"/robot/cup"}) == "TARGET_CUP_RELEASE_CLEARANCE_TIMEOUT"
    assert gate.events[-1]["active_target_compliant_pairs"] == 1


@pytest.mark.parametrize("field", ["joint_velocity_feedforward_enabled", "attached_payload_gravity_feedforward_enabled"])
def test_same_world_feedforward_policy_cannot_change_between_segments(field):
    from unloading_sim.m710_replay_physics import validate_same_world_continuation
    before = {field: True}
    bundle = {"format": "isaacsim_fanuc_replay_v1", "metadata": {field: False}}
    with pytest.raises(ValueError, match=f"fixed physical input: {field}"):
        validate_same_world_continuation(before, bundle, {"attached": False})


def test_runtime_consumes_velocity_and_payload_ff_without_uncapped_effort_source():
    source = (Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    drive = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "_drive_target")
    text = ast.unparse(drive)
    assert "grasp_joint is not None" in text
    assert "payload_gravity_compensation" in text
    assert "GetCenterOfMassAttr" in text and "get_world_poses" in text
    assert "finite_gravity_compensated_drive_target" in text
    assert "set_dof_efforts" not in text
    assert "set_dof_velocity_targets(command_velocity.astype(np.float32)[None, :])" in source
    assert "free_transit_gate.advance(trajectory_time, physics_dt, requested_duration)" in source


def test_actual_runtime_classifier_does_not_hide_rigid_shape_in_an_allowed_actor_pair():
    from unloading_sim.collision_policy import SimulationCollisionPolicy
    from unloading_sim.isaac_collision_policy import classify_compliant_cup_contact, ZeroPointContactResolver
    import math
    source = (Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")
    node = next(node for node in ast.walk(ast.parse(source))
                if isinstance(node, ast.FunctionDef) and node.name == "_classify_runtime_contact")
    namespace = {"math": math, "classify_compliant_cup_contact": classify_compliant_cup_contact,
                 "zero_point_contact_resolver": ZeroPointContactResolver(),
                 "_contact_scope_token": lambda: ("contact", "/Validation/Scene/target", False, False, False),
                 "compliant_cup_index_by_path": {"/robot/J6/cup": 0}, "commanded_cup_mask": [False],
                 "target_carton_path": "/Validation/Scene/target", "metadata": {"stack_carton_names": ["neighbor"]},
                 "_safe_prim_name": lambda name: name, "effective_collision_policy": SimulationCollisionPolicy(),
                 "gripper_cfg": {"physical_cup_compression_m": .01}, "contact_clock_s": [5.7],
                 "contact_runtime_context": {"stage": "contact", "attached": False, "actual_free_space": False,
                                             "release_validation_pending": False},
                 "unexpected_robot_contact_events": []}
    exec(compile(ast.Module(body=[node], type_ignores=[]), "runtime_classifier", "exec"), namespace)
    record = {}
    callback = namespace["_classify_runtime_contact"]
    callback(record, "/robot/J6", "/Validation/Scene/neighbor", "/robot/J6/cup", "/Validation/Scene/neighbor", [.001], lost=False)
    assert not namespace["unexpected_robot_contact_events"]
    callback(record, "/robot/J6", "/Validation/Scene/neighbor", "/robot/J6/rigid_insert", "/Validation/Scene/neighbor", [.019], lost=False)
    assert record["unexpected_runtime_event_count"] == 1
    assert len(record["runtime_collider_classifications"]) == 2
    assert namespace["unexpected_robot_contact_events"][0]["minimum_separation_m"] == .019
