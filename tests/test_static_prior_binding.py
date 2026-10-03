"""Pure binding/format regressions; no fabricated successful planner evidence."""
from copy import deepcopy
import json
import math

import pytest

from unloading_sim.static_prior import (applicability, candidate_mode, fingerprint,
    load_prior, make_database, nominal_front_attachment, request_context, static_world,
    validate_mode)


def request(loaded=False):
    pose = [[1., 0., 0., .3], [0., 1., 0., 0.], [0., 0., 1., .25], [0., 0., 0., 1.]]
    return dict(identity=dict(model_tool_fingerprint="full-robot-tool", task_tcp_fingerprint="tcp",
            scene_version="layout", policy_fingerprint="policy"),
        flange_from_task_tcp=pose,
        world=[dict(id="wall", category="trailer", size=[.05, 2.3, 2.7], pose=pose),
               dict(id="carton-1", category="carton", size=[.6, .4, .3], pose=pose),
               dict(id="conveyor", category="conveyor", size=[2.8, .7, .12], pose=pose)],
        allowed_pairs=[["base_link", "chassis"], ["cup", "carton-1"]],
        attachment=dict(id="payload", size=[.6, .4, .3], pose=pose,
            touch_links=["cup"]) if loaded else None,
        clearance_policy=dict(tool_links=["rigid", "cup"], compliant_tool_links=["cup"],
            source_policy=dict(free_space_clearance_m=.005),
            edge_resolution_rad=.035, receiver_reserve_m=0.))


def context(loaded=False):
    return request_context(request(loaded), placement_policy_sha256="placement",
        joint_limits_rad=[[-3., 3.]]*6)


def graph(loaded=False):
    return dict(status="SUCCESS", context=context(loaded),
        nodes=[[0.]*6, [.1]*6], edges=[[0, 1]])


def test_static_world_preserves_all_unknown_and_fixed_bodies():
    world = request()["world"]+[dict(id="unknown", category="new_obstacle")]
    assert [b["id"] for b in static_world(world)] == ["conveyor", "unknown", "wall"]
    assert "carton-1" not in json.dumps(context()["allowed_pairs"])
    changed = request()
    changed["world"] = [b for b in changed["world"] if b["id"] != "carton-1"]
    changed["allowed_pairs"] = [["base_link", "chassis"]]
    assert request_context(changed, placement_policy_sha256="placement",
        joint_limits_rad=[[-3., 3.]]*6) == context()


@pytest.mark.parametrize("field,value", [
    ("identity", {"model_tool_fingerprint": "different"}),
    ("flange_from_task_tcp", [[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]),
    ("static_world", []), ("tool_links", ["small-tool"]),
    ("collision_policy", {"free_space_clearance_m": 0.}),
    ("joint_limits_rad", [[-4., 4.]]*6), ("placement_policy_sha256", "different")])
def test_incompatible_binding_never_reuses_static_conclusion(field, value):
    current = context()
    current[field] = value
    result = applicability(context(), current)
    assert not result["covered"]
    assert result["static_clearance_inherited"] is False


def test_empty_route_is_not_a_loaded_route():
    outcome = applicability(context(), context(True))
    assert not outcome["covered"]
    assert outcome["reason"] == "STATIC_PRIOR_LOAD_CLASS_MISMATCH"


def test_attachment_perturbation_only_permits_current_checked_candidate():
    changed = context(True)
    changed["attachment"]["pose"][1][3] += .01
    result = applicability(context(True), changed)
    assert result["covered"]
    assert result["current_geometry_check_required"] is True
    assert result["static_clearance_inherited"] is False
    changed["attachment"]["pose"][1][3] += .02
    assert not applicability(context(True), changed)["covered"]


def test_attachment_orientation_outside_scope_is_uncovered():
    changed = context(True)
    angle = .06
    changed["attachment"]["pose"] = [
        [math.cos(angle), -math.sin(angle), 0., .3],
        [math.sin(angle), math.cos(angle), 0., 0.],
        [0., 0., 1., .25], [0., 0., 0., 1.]]
    assert not applicability(context(True), changed)["covered"]


def test_different_box_size_is_uncovered():
    changed = context(True)
    changed["attachment"]["size"][0] = .61
    assert applicability(context(True), changed)["reason"] == "STATIC_PRIOR_PAYLOAD_MISMATCH"


def test_nominal_attachment_comes_from_front_geometry_not_previous_q():
    transform = nominal_front_attachment([.6, .4, .3])
    assert transform == [[0.,0.,1.,0.],[0.,-1.,0.,0.],[1.,0.,0.,.3],[0.,0.,0.,1.]]
    assert nominal_front_attachment([.8, .4, .3])[2][3] == .4


@pytest.mark.parametrize("corrupt", [
    lambda data: data["nodes"][0].__setitem__(0, float("nan")),
    lambda data: data["nodes"][0].__setitem__(0, 3.01),
    lambda data: data.__setitem__("edges", [[0, 2]]),
    lambda data: data.__setitem__("edges", [[0, True]]),
    lambda data: data.__setitem__("edges", [[0, 0]]),
    lambda data: data.__setitem__("status", "TIMEOUT")])
def test_malformed_or_incomplete_graph_is_rejected(corrupt):
    data = graph()
    corrupt(data)
    with pytest.raises(ValueError):
        validate_mode(data)


def test_database_load_reports_cost_and_detects_tampering(tmp_path):
    data = make_database({"empty": graph()}, {"seed": 42})
    path = tmp_path/"prior.json"
    path.write_text(json.dumps(data))
    loaded, stats = load_prior(path)
    assert loaded == data and stats["bytes"] == path.stat().st_size
    assert stats["load_s"] >= 0.
    data["modes"]["empty"]["nodes"][0][0] = .5
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="HASH_MISMATCH"):
        load_prior(path)


def test_missing_load_class_returns_uncovered():
    data = make_database({"empty": graph()}, {})
    mode, evidence = candidate_mode(data, request(True),
        placement_policy_sha256="placement", joint_limits_rad=[[-3., 3.]]*6)
    assert mode is None and evidence["reason"] == "STATIC_PRIOR_LOAD_CLASS_UNCOVERED"


def test_context_hash_is_stable_over_mapping_order():
    value = context()
    assert fingerprint(value) == fingerprint(dict(reversed(list(value.items()))))


def test_stale_unknown_permission_does_not_disappear_from_context():
    changed = request()
    changed["world"] = [b for b in changed["world"] if b["id"] != "carton-1"]
    current = request_context(changed, placement_policy_sha256="placement",
        joint_limits_rad=[[-3., 3.]]*6)
    assert not applicability(context(), current)["covered"]


@pytest.mark.parametrize("mode", ["empty", "loaded"])
def test_builder_free_request_uses_configuration_geometry_without_task_or_paths(mode):
    # Exercise the real adapter's request constructor without starting a worker.
    import runpy
    from pathlib import Path
    from types import SimpleNamespace, MethodType
    import numpy as np
    from unloading_sim.geometry import OBB
    from unloading_sim.moveit2_backend import MoveItLayoutConnector

    build = runpy.run_path(str(Path(__file__).parents[1] / "tools/build_m710_static_prior.py"))["build_request"]
    robot = SimpleNamespace(joint_limits=np.array([[-3., 3.]]*6),
        fk=lambda q: np.eye(4), named_link_frames=lambda q: {"flange": np.eye(4)})
    collision = SimpleNamespace(compliant_cup_neighbor_contact_mode="ignore",
        to_mapping=lambda: {"required_pair_clearance_m": .005})
    c = SimpleNamespace(robot=robot, robot_state_validator=SimpleNamespace(
            base_support_obstacle_name="chassis", contact_target_name=None),
        collision_policy=collision, native_tools=["tool"], native_compliant=["cup"],
        native_cold=True, require_native_motion=True, native_task_id="offline-fixture",
        native_identity={"model_tool_fingerprint": "fixture"}, planning_only=True,
        planning_mode="static_prior_fast", contact_tolerance_m=.0002,
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
        budget=SimpleNamespace(receiver_runtime_clearance_reserve_m=.003,
            edge_resolution_rad=.035))
    c._build_native_request = MethodType(MoveItLayoutConnector._build_native_request, c)
    c._native_process_policy = MethodType(MoveItLayoutConnector._native_process_policy, c)
    layout = SimpleNamespace(data={"carton_stack": {"carton_size_xyz_m": [.6, .4, .3]}})
    scene = SimpleNamespace(policy=SimpleNamespace(layout_validation=SimpleNamespace(
        initial_q=np.zeros(6), layout=layout)), all_obstacles=(
            OBB(np.array([2., 0., 0.]), np.array([.1, 1., 1.]), np.eye(3), "wall", "trailer"),
            OBB(np.zeros(3), np.array([.3, .2, .15]), np.eye(3), "any-carton", "carton")))
    result = build(c, scene, mode=mode, seed=71071, node_limit=160,
        attempt_limit=1600, neighbors=10, placement_policy_sha256="placement")
    assert result["planning_mode"] == "static_prior_fast"
    assert result["op"] == "build_static_prior"
    assert [body["id"] for body in result["world"]] == ["wall"]
    assert not any(key in result for key in ("process_policy", "task_id", "require_native_motion", "path"))
    assert result["prior_context"]["attachment"]["mode"] == mode
    if mode == "loaded":
        assert result["attachment"]["size"] == [.6, .4, .3]
        assert result["attachment"]["pose"] == nominal_front_attachment([.6, .4, .3])
    else:
        assert result["attachment"] is None and result["clearance_policy"]["target_id"] == ""


def test_request_context_is_the_exact_json_transport_value():
    req = request()
    req["clearance_policy"]["source_policy"].update(
        wrist_tool_exempt_links=("J5_link", "J6_link"),
        stack_contact_stages=("support-release", "extraction"))
    binding = request_context(req, placement_policy_sha256="placement",
        joint_limits_rad=[[-3., 3.]]*6)
    assert binding == json.loads(json.dumps(binding))
    assert binding["collision_policy"]["wrist_tool_exempt_links"] == ["J5_link", "J6_link"]
    native = dict(status="SUCCESS", context=json.loads(json.dumps(binding)), nodes=[], edges=[])
    assert validate_mode(native, expected_context=binding) is native


def test_native_context_mismatch_is_not_hidden_by_json_normalization():
    expected = context()
    expected["collision_policy"]["stages"] = ("extraction", "transit")
    returned = graph()
    returned["context"] = json.loads(json.dumps(expected))
    validate_mode(returned, expected_context=expected)
    returned["context"]["collision_policy"]["stages"][0] = "contact"
    with pytest.raises(ValueError, match="CONTEXT_CHANGED"):
        validate_mode(returned, expected_context=expected)


def portal_scene():
    from types import SimpleNamespace
    import numpy as np
    from unloading_sim.geometry import OBB
    data = dict(layout_id="fixture-layout", world={"floor_z_m": 0.},
        trailer={"height_m": 2.7, "left_wall_y_m": 1.15, "right_wall_y_m": -1.15},
        carton_stack=dict(carton_size_xyz_m=[.6, .4, .3], front_face_x_m=0.,
            height_layers=8, layer_gap_m=0., center_y_m=[-.84, -.42, 0., .42, .84]))
    base = np.eye(4)
    base[:3, 3] = [-1.325, .35, .6]
    layout = SimpleNamespace(data=data, robot_base_transform=lambda: base)
    virtual, physical = np.eye(4), np.eye(4)
    virtual[:3, :3] = physical[:3, :3] = [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]]
    virtual[0, 3], physical[0, 3] = .25, .2175
    policy = SimpleNamespace(layout_validation=SimpleNamespace(layout=layout, initial_q=np.zeros(6)),
        data={"tool_frame_contract": {"T_flange_virtual_task_tcp": virtual.tolist(),
            "T_flange_nominal_compressed_contact": physical.tolist()}})
    chassis = OBB([-1.95, .35, .3], [1.05, .75, .3], np.eye(3), "chassis", "chassis")
    receiver = OBB([-1.6, -.75, .54], [1.4, .35, .06], np.eye(3), "receiver", "conveyor")
    return SimpleNamespace(policy=policy, fixed_components=(chassis, receiver), receiver=receiver)


def test_portal_poses_follow_layout_geometry_and_existing_frame_constructors():
    import numpy as np
    from unloading_sim.static_prior import layout_portal_poses
    from unloading_sim.geometry import make_tool_rotation
    from unloading_sim.conveyor_placement import placement_working_normal, TOP_DOWN
    scene = portal_scene()
    poses, sources = layout_portal_poses(scene)
    assert len(poses) == 30 and len({p["portal_id"] for p in poses}) == 30
    assert sources["sampling_offsets_are_clearance_claims"] is False
    assert sources["no_query_endpoints_used"] is True
    assert sources["front_sampling_x_m"] == pytest.approx([-.9, -1.1125])
    assert sources["front_sampling_y_m"] == pytest.approx([-.74, 0., .74])
    assert sources["receiver_sampling_z_m"] == pytest.approx([1.2675, 1.6675])
    for p in poses:
        expected = make_tool_rotation(np.array([1.,0.,0.])) if p["family"].endswith("front") else make_tool_rotation(placement_working_normal(TOP_DOWN))
        assert np.allclose(np.asarray(p["pose"])[:3, :3], expected)
    scene.policy.layout_validation.layout.data["carton_stack"]["front_face_x_m"] += .02
    changed, changed_sources = layout_portal_poses(scene)
    assert changed[0]["pose"][0][3] == pytest.approx(poses[0]["pose"][0][3]+.02)
    assert fingerprint(changed_sources) != fingerprint(sources)


def test_portal_halton_seeds_are_bounded_and_have_no_graph_dependency():
    from unloading_sim.static_prior import portal_ik_seeds
    limits = [[-3.+i*.1, 3.+i*.1] for i in range(6)]
    initial = [.1]*6
    seeds = portal_ik_seeds(limits, initial, seed=71071)
    assert seeds[0] == initial and len(seeds) == 17
    assert seeds == portal_ik_seeds(limits, initial, seed=71071)
    assert seeds != portal_ik_seeds(limits, initial, seed=71072)
    assert all(lo <= q <= hi for vector in seeds for q, (lo, hi) in zip(vector, limits))
    with pytest.raises(ValueError):
        portal_ik_seeds(limits, initial, seed=71071, count=25)


def test_portal_branches_preserve_actual_q_and_bound_diverse_selection():
    from unloading_sim.static_prior import diverse_portal_branches
    solutions = [[0.]*6, [1e-5]*6, [.1]*6, [-2.]*6, [2.]*6]
    selected = diverse_portal_branches(solutions, [0.]*6)
    assert len(selected) == 3 and selected[0] == [0.]*6
    assert [-2.]*6 in selected and [2.]*6 in selected
    assert all(q in solutions for q in selected)


def test_portal_metadata_is_hashed_without_breaking_old_graph_id():
    from unloading_sim.static_prior import graph_id
    old = graph()
    old_id = fingerprint({k: old[k] for k in ("context", "nodes", "edges")})
    assert graph_id(old) == old_id
    new = deepcopy(old)
    new["accepted_portals"] = [dict(portal_id="fixture/branch-0", node_index=0,
        pose=request()["flange_from_task_tcp"], deduplicated=False)]
    validate_mode(new)
    assert graph_id(new) != old_id
    changed = deepcopy(new)
    changed["accepted_portals"][0]["node_index"] = 1
    assert graph_id(changed) != graph_id(new)
    changed["accepted_portals"][0]["node_index"] = 2
    with pytest.raises(ValueError, match="PORTAL_METADATA"):
        validate_mode(changed)


def test_builder_portal_ik_batches_are_bounded_and_not_claimed_collision_checked(monkeypatch):
    import runpy
    from pathlib import Path
    from types import SimpleNamespace
    import numpy as np
    function = runpy.run_path(str(Path(__file__).parents[1]/"tools/build_m710_static_prior.py"))["generate_layout_portals"]
    monkeypatch.setitem(function.__globals__, "build_request", lambda *a, **kw:
        dict(op="build_static_prior", q_goal=[0.]*6, prior_build={}, prior_context={}))
    calls = []
    robot = SimpleNamespace(joint_limits=np.array([[-3.,3.]]*6),
        within_limits=lambda q: bool(np.all(np.abs(q)<=3.)),
        fk=lambda q: np.asarray(calls[-1]["goal_pose"]))
    def request_batch(req, timeout):
        calls.append(req)
        rows = [dict(seed_index=i, native_ik_calls=1, status="SUCCESS", q=q)
            for i, q in enumerate(req["ik_seeds"])]
        return dict(status="SUCCESS", results=rows, consume_count=len(rows),
            native_ik_calls=len(rows), ik_s=0.)
    connector = SimpleNamespace(robot=robot, native=SimpleNamespace(request=request_batch),
        ik={"position_tolerance_m": 1e-4, "orientation_tolerance_rad": 2e-4},
        native_ik_seconds=.25)
    candidates, report = function(connector, portal_scene(), seed=71071,
        ipc_timeout_s=360., build_budget_s=300., placement_policy_sha256="fixture")
    assert len(candidates) == 90
    assert report["native_ik_calls"] == 510 and report["native_batch_calls"] == 90
    assert report["collision_screening"] == "DEFERRED_TO_EACH_MODE_NATIVE_BUILD"
    assert all(len(c["ik_seeds"]) <= 8 and c["stop_on_first_success"] is False for c in calls)
    assert all("q_goal" not in c and "prior_build" not in c and "prior_context" not in c for c in calls)
    assert all(c["collision_check_in_native_ik"] is False for c in calls)
