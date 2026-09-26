"""Named raw-angle diagnostics and bounded production wiring; no physics claims."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import unloading_sim.wrist_transfer as wrist
import unloading_sim.layout_trajectory as lt
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.motion_quality import path_quality, quality_improves
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.layout_single_carton import load_layout_motion_policy
from test_feasible_result_budget import clock, connector
from test_layout_trajectory import _segment
from test_lookahead_contact import official_scene, case, START_Q, CONTACT_Q, context
from test_default_history_adaptation import actual_case, source_fixture


@pytest.mark.parametrize("values,net,total", [([0., 6.], 6., 6.), ([0., 3., 0.], 0., 6.),
    ([3.13, -3.13], 6.26, 6.26), ([0., 2*np.pi], 2*np.pi, 2*np.pi)])
def test_named_raw_metrics_and_so3_include_entire_turn(values, net, total):
    q = np.zeros((len(values), 6)); q[:, 1] = values
    def fk(point):
        pose = np.eye(4); pose[:3, :3] = rotation_matrix_from_rpy(0, 0, point[1]); return pose
    result = path_quality(q, fk=fk, joint_names=["J1", "J6", "J2", "J3", "J4", "J5"])
    assert result["j6_index"] == 1
    assert result["j6_net_rad"] == pytest.approx(net)
    assert result["j6_total_rad"] == pytest.approx(total)
    assert result["j6_extra_rad"] == pytest.approx(total-net)
    assert result["orientation_travel_so3_rad"] == pytest.approx(total)
    assert result["start_q_rad"][1] == values[0] and result["end_q_rad"][1] == values[-1]


def test_lower_score_cannot_hide_increased_j6_or_wrist():
    before = dict(soft_score=10., total_joint_travel_rad=5., wrist_total_rad=3., j6_total_rad=1.)
    assert quality_improves(before, {**before, "soft_score": 9.})
    for name in ("total_joint_travel_rad", "wrist_total_rad", "j6_total_rad"):
        assert not quality_improves(before, {**before, "soft_score": 8., name: before[name]+.001})
    assert not quality_improves(before, None)


def test_common_free_optimizer_uses_copy_and_fixed_domain(clock):
    c = connector(clock)
    path = [np.zeros(6), np.full(6, .2), np.full(6, .1)]
    saved = deepcopy(path)
    assert c._path_failure(path, [], stage="pregrasp") is None
    changed, report = c._improve_free_path(path, [])
    assert report["adopted"] and len(changed) == 2
    np.testing.assert_array_equal(path, saved)
    np.testing.assert_array_equal(changed[0], path[0]); np.testing.assert_array_equal(changed[-1], path[-1])


class PeriodicRobot:
    dof = 6
    base_transform = np.eye(4)
    active_joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    joint_limits = np.tile([-7., 7.], (6, 1))
    model = SimpleNamespace(velocityLimit=np.ones(6), joints=[None]+[
        SimpleNamespace(nq=1, shortname=lambda: "JointModelRZ") for _ in range(6)])
    def fk(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q)[:3]
        pose[:3, :3] = rotation_matrix_from_rpy(*q[3:]) @ np.diag([1., -1., -1.])
        return pose
    def geometric_jacobian(self, q):
        jac = np.eye(6)
        jac[3:, 3] = rotation_matrix_from_rpy(0, q[4], q[5])[:, 0]
        jac[3:, 4] = rotation_matrix_from_rpy(0, 0, q[5])[:, 1]
        return jac
    def clamp(self, q):
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])
    def within_limits(self, q):
        return bool(np.all(np.asarray(q) >= self.joint_limits[:, 0]) and np.all(np.asarray(q) <= self.joint_limits[:, 1]))
    def named_link_frames(self, q):
        return {"flange": self.fk(q)}


@pytest.mark.parametrize("mode", ["better", "collision", "coverage", "suffix", "late", "no_time"])
def test_endpoint_candidate_really_connects_and_completes_or_retains(clock, monkeypatch, mode):
    monkeypatch.setattr(wrist, "perf_counter", clock)
    c = connector(clock)
    c.robot = PeriodicRobot()
    c.ik.update(max_iterations=100, damping=.005, max_step_rad=.18, orientation_weight=.45)
    c.tool_collision_obbs_provider = lambda q: []
    target = OBB([0, 0, 0], [1., 1., .3], np.eye(3), "carton_l07_c02", "carton")
    suction = load_layout_motion_policy("configs/validation/m710id70_layout_v1_single_carton.yaml").data["suction"]
    start = np.array([0., 0., .3, 0., 0., 0.])
    grasp = start.copy(); grasp[5] = 2*np.pi+.2
    scene = SimpleNamespace(all_obstacles=[target], receiver=target,
        support_graph=SimpleNamespace(supported_by={target.name: ()}), policy=SimpleNamespace(data={"suction": suction}))
    calls = []
    active_mode = ["baseline"]
    def validator(q, obstacles, **kwargs):
        # Synthetic obstacle in configuration space, also exercising strict edges.
        if active_mode[0] == "collision" and q[5] < 1:
            return {"reason": "SYNTHETIC_RIGID_OBSTACLE"}
    c.robot_state_validator = validator
    real_selection = c._contact_selection
    def selection(q, *args):
        selected = real_selection(q, *args)
        if active_mode[0] == "coverage" and q[5] < 1:
            raise ValueError("injected current ring coverage boundary")
        return selected
    monkeypatch.setattr(c, "_contact_selection", selection)
    def branch(**args):
        q = np.asarray(args["grasp_q"])
        cups = c._contact_selection(q, target, "top", suction)
        path, failure, evidence = c._transit(start, q, [target], seed=args["seed"],
            iteration_budget=10, stage="pregrasp", purpose="FREE_APPROACH")
        calls.append((q.copy(), failure))
        if failure:
            return None, failure, evidence
        # Controlled simple fixture has seven real checked edges; the production
        # connector still owns IK, selection, limits, edge sampling and D binding.
        segment = _segment()
        points = [start, (start+q)/2, q]
        for index in range(1, 6):
            point = q.copy(); point[0] += .01*index; points.append(point)
        physical = c.robot.fk(q)
        rigid = RigidAttachment.capture(physical, target)
        attachment = lt.PhysicalContactAttachment(c.robot, rigid, np.eye(4), np.eye(4))
        for stage, (first, last) in segment["stage_ranges"].items():
            failure = c._path_failure(points[first:last+1], [], stage=stage,
                attachment=attachment if stage in {"support-release", "extraction", "transit", "place"} else None)
            if active_mode[0] == "suffix" and stage == "transit":
                failure = {"reason": "SYNTHETIC_LOADED_SUFFIX_BLOCKED", "stage": stage}
            if failure:
                return None, failure, {"checked_through": stage}
        segment["path"] = [p.tolist() for p in points]
        segment["contact"].update(actual_q_rad=q.tolist(), cup_selection=cups,
            actual_virtual_task_tcp_pose_world=physical.tolist(), actual_physical_contact_pose_world=physical.tolist(),
            requested_virtual_task_tcp_pose_world=physical.tolist(), requested_physical_contact_pose_world=physical.tolist(),
            physical_contact_from_box=rigid.tcp_from_box.tolist())
        if active_mode[0] == "late":
            clock.now = c._branch_final_deadline
        failure = c._finalize_task(segment, [target], target)
        return None if failure else segment, failure, {"all_synthetic_stage_edges_checked": True}
    monkeypatch.setattr(c, "_plan_branch", branch)
    original = c.plan(target=target, face="top", requested_virtual_contact=c.robot.fk(grasp),
        grasp_candidates=[{"q_rad": grasp}], home_q=start, all_obstacles=[target], receiver=target,
        support_names=(), suction=suction, seed=1)
    assert original.success
    baseline = deepcopy(original.segment)
    active_mode[0] = mode
    if mode == "no_time":
        c._deadline_monotonic = clock()+.5
    result, attempts, report = wrist.improve_complete_task(c, scene, target, baseline, (),
        deadline=c._deadline_monotonic, attempt_limit=2)
    assert baseline == original.segment
    assert report["attempts_count"] <= 2
    if mode == "better":
        assert report["status"] == "ADOPTED", report
        assert len(calls) >= 2 and abs(calls[-1][0][5]-.2) < 1e-6
        assert result["validation"]["completion"]["validation_level"] == "D_COMPLETE_TASK"
        assert np.asarray(result["path"])[result["grasp_index"], 5] == pytest.approx(.2)
        assert report["after"]["j6_total_rad"] < report["before"]["j6_total_rad"]-6
    else:
        assert result == baseline and report["selected"] == "BASELINE"


def test_official_periodic_fk_and_limits(official_scene):
    _, c = official_scene
    candidates = wrist.periodic_j6_candidates(c.robot, CONTACT_Q)
    assert candidates
    index = list(c.robot.active_joint_names).index("J6")
    for q in candidates:
        assert c.robot.joint_limits[index, 0] <= q[index] <= c.robot.joint_limits[index, 1]
        assert abs(q[index]-CONTACT_Q[index]) == pytest.approx(2*np.pi)
        np.testing.assert_allclose(c.robot.fk(q), c.robot.fk(CONTACT_Q), atol=1e-9, rtol=0)
    restricted = PeriodicRobot(); restricted.joint_limits = np.tile([-1., 1.], (6, 1))
    assert wrist.periodic_j6_candidates(restricted, np.zeros(6)) == []


def test_verified_departure_baseline_uses_real_next_contact(case, monkeypatch):
    c = case.c
    saved = context(case)
    # Isolate next-contact comparison from unrelated release construction.
    # Both history flight validation and official near-contact gates remain real.
    from unloading_sim.history_adaptation import checked_departure
    import unloading_sim.history_adaptation as adaptation
    monkeypatch.setattr(adaptation, "current_release_sweep", lambda *a: [case.placed])
    hint = {"segment": {"path": [START_Q.tolist(), START_Q.tolist()],
                         "stage_ranges": {"withdrawal": [0, 1]}},
            "attempt_provenance": {"candidate_id": "a"*64}}
    prediction = {"flight_time_s": 0., "landing_support": {"receiver_names": ["fixture"]}}
    result, failure, evidence = checked_departure(c, hint, START_Q, case.placed,
        case.scene.all_obstacles, [1., 0., 0.], prediction)
    assert failure is None
    assert evidence["next_contact"]["status"] == "CHECKED_NEXT_CONTACT_CONNECTION", evidence
    assert evidence["two_task_cost_rad"] > 0
    assert evidence["compared_safe_departures"][0]["next_contact_target"] == case.target.name
    assert c._lookahead_remaining_s <= 12. and context(case) == saved


@pytest.mark.parametrize("mode", ["changed_mask", "blocked_join"])
def test_history_calls_common_optimizer_only_after_original_prefix_passes(actual_case, monkeypatch, mode):
    from test_default_history_adaptation import test_adaptation_rechecks_current_context_before_completion
    c = actual_case.c
    original = c._improve_free_path
    calls = []
    def improve(path, obstacles, **kwargs):
        before = deepcopy(path)
        result, evidence = original(path, obstacles, **kwargs)
        np.testing.assert_array_equal(path, before)
        np.testing.assert_array_equal(result[0], before[0])
        np.testing.assert_array_equal(result[-1], before[-1])
        calls.append(evidence)
        return result, evidence
    monkeypatch.setattr(c, "_improve_free_path", improve)
    test_adaptation_rechecks_current_context_before_completion(actual_case, monkeypatch, mode)
    assert len(calls) == (1 if mode == "changed_mask" else 0)


@pytest.mark.parametrize("unknown", [False, True])
def test_departure_comparison_uses_baseline_next_cost_first(clock, monkeypatch, unknown):
    c = connector(clock)
    placed = OBB([.03, 0, 1], [.02]*3, np.eye(3), "placed", "carton")
    tool = OBB([0, 0, 1], [.005]*3, np.eye(3), "tool", "tool")
    target = OBB([0, 1, 1], [.1]*3, np.eye(3), "next", "carton")
    c.tool_collision_obbs_provider = lambda _: [tool]
    c.next_contact_provider = lambda _: [dict(target=target, requested_virtual_contact=np.eye(4))]
    baseline = [np.zeros(6), np.full(6, .2)]
    assert c._path_failure(baseline, [], stage="withdrawal") is None
    observed = []
    def preview(start, *args, **kwargs):
        observed.append(start.copy())
        cost = None if unknown else (1. if start[0] > .1 else .1)
        return dict(status="UNKNOWN" if unknown else "CHECKED_NEXT_CONTACT_CONNECTION",
            target="next", row_id="same", face="top", joint_path_length_rad=cost)
    monkeypatch.setattr(c, "_next_contact_cost", preview)  # Cost boundary only; ranking and paths are real.
    monkeypatch.setattr(c, "_cartesian", lambda start, *a, **k: ([start.copy(), np.full(6, .05)], None, {}))
    path, failure, evidence = c._departure(np.zeros(6), placed, [], [1., 0., 0.], seed=4,
        working_normal=np.array([1., 0, 0]),
        release_prediction={"flight_time_s": 0., "landing_support": {"receiver_names": ["fixture"]}},
        verified_baseline=(baseline, {"next_approach_start_q_rad": baseline[-1].tolist()}))
    assert failure is None
    np.testing.assert_array_equal(observed[0], baseline[-1])
    if unknown:
        np.testing.assert_array_equal(path, baseline)
        assert len(observed) == 1 and evidence["two_task_cost_rad"] is None
    else:
        assert len(observed) <= 3 and path[-1][0] == .05
        assert evidence["two_task_cost_rad"] < evidence["compared_safe_departures"][0]["two_task_cost_rad"]
