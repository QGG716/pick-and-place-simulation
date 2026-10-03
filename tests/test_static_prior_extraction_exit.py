"""Fast-only exit ordering: real Cartesian geometry plus focused lineage probes."""
from types import SimpleNamespace
import numpy as np
import pytest

from test_extraction_boundary_guard import fixture
from unloading_sim.geometry import OBB
from unloading_sim.stage_motion_policy import MotionPurpose


def fast_fixture():
    connector, start, attachment, tracker, neighbor = fixture()
    connector.planning_only = True
    connector.planning_mode = "static_prior_fast"
    connector.static_prior_loaded = True
    virtual = np.eye(4)
    virtual[2, 3] = .25
    connector.native_scene = SimpleNamespace(policy=SimpleNamespace(
        layout_validation=SimpleNamespace(layout=SimpleNamespace(data={"carton_stack": {
            "front_face_x_m": 0., "carton_size_xyz_m": [.6, .4, .3]}})),
        data={"tool_frame_contract": {"T_flange_virtual_task_tcp": virtual.tolist()}}))
    return connector, start, attachment, tracker, neighbor


@pytest.mark.parametrize("attribute,value", [
    ("planning_only", False), ("planning_mode", "cold_from_scratch"),
    ("static_prior_loaded", False)])
def test_exit_heuristic_requires_explicit_loaded_fast_mode(attribute, value):
    c, start, *_ = fast_fixture()
    setattr(c, attribute, value)
    assert c._static_prior_front_channel_goal(start, np.array([-.6, 0, 0, 0, 0, 0]), [-1., 0., 0.]) is None


def test_exit_goal_keeps_pose_and_existing_total_distance_bound():
    c, start, *_ = fast_fixture()
    end = np.array([-.61, .02, .03, .1, -.2, .3])
    goal, evidence = c._static_prior_front_channel_goal(start, end, [-1., 0., 0.])
    actual = c.robot.fk(end)
    np.testing.assert_array_equal(goal[:3, :3], actual[:3, :3])
    np.testing.assert_array_equal(goal[1:3, 3], actual[1:3, 3])
    assert evidence["front_plane_x_m"] == pytest.approx(-.9)
    assert evidence["extraction_distance_m"] + evidence["connection_distance_m"] == pytest.approx(.8)
    assert not evidence["requested_front_plane_reached"]
    assert evidence["compatibility_status"] == "HEURISTIC_PENDING_CURRENT_PRIOR_CONNECTION"
    assert c._static_prior_front_channel_goal(start, end, [0., 0., 1.]) is None
    assert c._static_prior_front_channel_goal(start, np.array([-.81, 0, 0, 0, 0, 0]), [-1., 0., 0.]) is None


def test_real_cartesian_exit_restores_normal_rules_and_reuses_original_prefix(monkeypatch):
    c, start, attachment, tracker, neighbor = fast_fixture()
    calls = []
    original = c._cartesian
    def observe(*args, **kwargs):
        result = original(*args, **kwargs)
        calls.append((args, kwargs, result))
        return result
    monkeypatch.setattr(c, "_cartesian", observe)
    options = c._extraction_with_prior_exit_options(start, attachment, [neighbor], tracker,
        np.array([-1., 0., 0.]), seed=71070)
    preferred = next(options)
    extraction, released, failure, _, prefix, evidence = preferred
    assert failure is None and released.fully_released and evidence["selected"]
    assert len(prefix) > 1 and calls[-1][1]["stage"] == "transit"
    assert calls[-1][1]["purpose"] == MotionPurpose.FREE_LOADED_TRANSFER
    assert "initial_proximity" not in calls[-1][1] and "support_names" not in calls[-1][1]
    assert calls[-1][0][0] is extraction[-1]
    assert evidence["actual_end_tcp_world_m"][0] == pytest.approx(-.8, abs=1e-4)
    fallback = next(options)
    assert fallback[0] is extraction and fallback[0][-1] is extraction[-1]
    assert fallback[4] == [] and not fallback[5]["selected"]
    assert len(calls) == 2  # One extraction and one extension; no repeated prefix.
    assert not tracker.fully_released


def test_blocked_extension_reuses_release_without_contact_permissions(monkeypatch):
    c, start, attachment, tracker, neighbor = fast_fixture()
    # Obstacle is clear of the original release endpoint but obstructs the
    # farther free-space extension. The real state/path validator rejects it.
    blocker = OBB([-.75, 0., 0.], [.025, .025, .025], np.eye(3), "fixed_block", "trailer")
    result = next(c._extraction_with_prior_exit_options(start, attachment, [neighbor, blocker],
        tracker, np.array([-1., 0., 0.]), seed=71070))
    extraction, released, failure, _, prefix, evidence = result
    assert failure is None and released.fully_released and prefix == []
    assert evidence["status"] == "REJECTED" and evidence["failure"] is not None
    assert extraction[-1][0] > -.7


@pytest.mark.parametrize("use_prefix", [False, True])
def test_downstream_connection_uses_selected_endpoint_object(monkeypatch, use_prefix):
    c, start, attachment, tracker, neighbor = fast_fixture()
    options = c._extraction_with_prior_exit_options(start, attachment, [neighbor], tracker,
        np.array([-1., 0., 0.]), seed=71070)
    extraction, released, _, _, prefix, _ = next(options)
    prefix = prefix if use_prefix else []
    expected = prefix[-1] if prefix else extraction[-1]
    captured = []
    def stop_at_connection(pose, seeds, actual_start, obstacles, **kwargs):
        captured.append((actual_start, seeds[0]))
        return None, [], {"reason": "TEST_STOP_AFTER_PARENT_SELECTION"}, {}
    monkeypatch.setattr(c, "_connect_pose", stop_at_connection)
    placement = SimpleNamespace(payload=attachment.box_at(expected), receiver_names=("receiver",))
    _, failure, _ = c._finish_place_branch(target=attachment.box_at(start), face="front",
        requested_virtual_contact=np.eye(4), home_q=start, contact_q=start,
        physical_contact=np.eye(4), rigid=attachment.rigid, attachment=attachment,
        selection={}, pregrasp=[start], contact=[start], support_release=[start],
        extraction=extraction, released_tracker=released, payload_obstacles=[neighbor],
        placement=placement, selected_supports=[], trace={"stages": {}}, seed=71070,
        transit_prefix=prefix)
    assert failure["reason"] == "TEST_STOP_AFTER_PARENT_SELECTION"
    assert captured[0][0] is expected and captured[0][1] is expected


def test_cold_generator_does_not_add_or_inspect_new_exit_metadata(monkeypatch):
    c, start, attachment, tracker, neighbor = fast_fixture()
    c.planning_mode = "cold_from_scratch"
    original_path, original_evidence = [start], {"existing": "untouched"}
    monkeypatch.setattr(c, "_extraction_options", lambda *a, **k: iter([
        (original_path, tracker, None, original_evidence)]))
    def forbidden(*a, **k):
        raise AssertionError("cold must not request an additional LIN")
    monkeypatch.setattr(c, "_cartesian", forbidden)
    result = list(c._extraction_with_prior_exit_options(start, attachment, [neighbor], tracker,
        np.array([-1., 0., 0.]), seed=71070))
    assert len(result) == 1 and result[0][0] is original_path
    assert result[0][3] is original_evidence and result[0][4:] == ([], None)
