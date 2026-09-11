from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.geometry import OBB
from unloading_sim.isaac_collision_policy import expand_owned_tool_wrist_pairs
from unloading_sim.m710_replay_physics import (
    ActualStackContactMonitor, audit_payload_support_contact,
    finite_gravity_compensated_drive_target,
    replay_command_arrays,
    validate_continuation_request,
    validate_same_world_continuation,
)


def test_wrist_tool_pair_exemption_uses_exact_owned_shapes():
    links = {"J5_link": ["/robot/J5/collision"], "J6_link": ["/robot/J6/collision"],
             "J4_link": ["/robot/J4/collision"]}
    ownership = {"/robot/J6/box": "wantai", "/robot/J6/cup": "wantai",
                 "/scene/tool_looking_trailer": "trailer"}
    pairs = expand_owned_tool_wrist_pairs(links, ownership, tool_owner="wantai")
    assert len(pairs) == 4
    assert all("J4" not in first and "trailer" not in second for first, second in pairs)
    assert {second for first, second in pairs} == {"/robot/J6/box", "/robot/J6/cup"}
    with pytest.raises(ValueError, match="only official"):
        expand_owned_tool_wrist_pairs(links, ownership, tool_owner="wantai", exempt_links=["J4_link"])


def test_actual_stack_contact_allows_sliding_then_requires_geometric_free_space():
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")
    target = OBB([0, 0, 0.45], [0.3, 0.2, 0.15], np.eye(3), name="selected")
    neighbor = OBB([0, 0, 0.15], [0.3, 0.2, 0.15], np.eye(3), name="support")
    monitor = ActualStackContactMonitor(target, [neighbor], policy)
    sliding = OBB([-0.05, 0, 0.4495], target.half_extents, np.eye(3), name=target.name)
    result = monitor.observe(0.1, sliding, [neighbor], commanded_motion=True)
    assert result["accepted"] and not result["free_space_reached"]
    assert result["actual_penetration_m"] == pytest.approx(0.0005)
    escaped = OBB([-0.63, 0, 0.45], target.half_extents, np.eye(3), name=target.name)
    result = monitor.observe(1.0, escaped, [neighbor], commanded_motion=True)
    assert result["accepted"] and result["free_space_reached"]
    assert result["first_free_space_time_s"] == 1.0
    severe = OBB([0, 0, 0.40], target.half_extents, np.eye(3), name=target.name)
    assert monitor.observe(1.1, severe, [neighbor], commanded_motion=True)["reason"] == "SEVERE_ACTUAL_STACK_PENETRATION"


def test_actual_motion_stall_and_neighbor_disturbance_remain_failures():
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")
    target = OBB([0, 0, 0.45], [0.3, 0.2, 0.15], np.eye(3), name="selected")
    neighbor = OBB([0, 0, 0.15], [0.3, 0.2, 0.15], np.eye(3), name="support")
    monitor = ActualStackContactMonitor(target, [neighbor], policy)
    monitor.observe(0, target, [neighbor], commanded_motion=True)
    assert monitor.observe(3.1, target, [neighbor], commanded_motion=True)["reason"] == "ACTUAL_STACK_EXTRACTION_STALLED"
    moved = OBB([0.07, 0, 0.15], neighbor.half_extents, np.eye(3), name=neighbor.name)
    assert monitor.observe(3.2, target, [moved], commanded_motion=False)["reason"] == "EXCESSIVE_ACTUAL_NEIGHBOR_DISPLACEMENT"


def test_actual_support_gate_consumes_complete_two_belt_union_and_rejects_gap():
    first = {"name": "left", "center_m": [0, 0, 0.5], "size_m": [0.6, 0.6, 0.2],
             "rotation_matrix": np.eye(3).tolist()}
    second = {**first, "name": "right", "center_m": [0.6, 0, 0.5]}
    kwargs = dict(payload_center_m=[0.3, 0, 0.75], payload_rotation=np.eye(3),
                  payload_size_m=[0.6, 0.4, 0.3], payload_linear_velocity_m_s=[0, 0, 0],
                  payload_angular_velocity_rad_s=[0, 0, 0], support=first)
    result = audit_payload_support_contact(**kwargs, supports=[first, second])
    assert result.accepted and result.footprint_overlap_ratio == pytest.approx(1.0)
    shifted = {**second, "center_m": [0.62, 0, 0.5]}
    assert not audit_payload_support_contact(**kwargs, supports=[first, shifted]).accepted
    penetrating = {**kwargs, "payload_center_m": [0.3, 0, 0.745]}
    assert audit_payload_support_contact(**penetrating, supports=[first, second]).reason == "PAYLOAD_PENETRATES_SUPPORT"


def test_gravity_compensation_stays_inside_single_finite_force_drive():
    target = finite_gravity_compensated_drive_target([0, 1], [50, 200], [1000, 500], [100, 150])
    assert target == pytest.approx([0.05, 1.3])
    assert (target - [0, 1]) * [1000, 500] == pytest.approx([50, 150])
    with pytest.raises(ValueError):
        finite_gravity_compensated_drive_target([0], [1], [0], [10])


def test_same_world_continuation_rejects_stale_poses_and_retains_occupied_cartons():
    import copy
    carton = {"name": "remaining", "dynamic": True, "center_m": [0, 0, 0.15],
              "rotation_matrix": np.eye(3).tolist(), "size_m": [0.6, 0.4, 0.3]}
    occupied = {**carton, "name": "completed_on_belt", "center_m": [-1, 0, 0.75]}
    metadata = {"joint_names": ["J1"], "simulation_execution_ready": True,
                "completed_carton_ids": ["completed_on_belt"],
                "execution_blockers": [], "target": "remaining", "scene_primitives": [carton, occupied]}
    bundle = {"format": "isaacsim_fanuc_replay_v1", "metadata": copy.deepcopy(metadata),
              "timestamps_seconds": [0, 1], "positions_rad": [[0.1], [0.2]]}
    actual = {"attached": False, "q_rad": [0.1], "joint_names": ["J1"],
              "completed_carton_ids": ["completed_on_belt"], "world_session_id": "same_world",
              "cartons": [{"name": item["name"], "position_m": item["center_m"],
                           "orientation_wxyz": [1, 0, 0, 0]} for item in [carton, occupied]]}
    result = validate_same_world_continuation(metadata, bundle, actual)
    assert result["accepted"] and result["retained_carton_count"] == 2
    assert not result["scene_restored_or_teleported"]
    stale = copy.deepcopy(bundle)
    stale["metadata"]["scene_primitives"][0]["center_m"][0] += 0.02
    with pytest.raises(ValueError, match="stale actual carton position"):
        validate_same_world_continuation(metadata, stale, actual)
    stale = copy.deepcopy(bundle)
    stale["metadata"]["scene_primitives"].pop()
    with pytest.raises(ValueError, match="every live carton"):
        validate_same_world_continuation(metadata, stale, actual)
    with pytest.raises(ValueError, match="confirmed release"):
        validate_same_world_continuation(metadata, bundle, {**actual, "attached": True})
    missing_events = copy.deepcopy(bundle)
    missing_events["metadata"]["completed_carton_ids"] = []
    with pytest.raises(ValueError, match="cumulative execution events"):
        validate_same_world_continuation(metadata, missing_events, actual)
    changed_urdf = copy.deepcopy(bundle)
    changed_urdf["metadata"]["urdf_path"] = "wrong.urdf"
    with pytest.raises(ValueError, match="fixed physical input: urdf_path"):
        validate_same_world_continuation(metadata, changed_urdf, actual)


def test_initial_and_continuation_consume_real_exported_command_schema():
    from pathlib import Path
    bundle = {"timestamps_seconds": [0, 0.02, 0.04], "positions_rad": [[0], [0.1], [0.2]]}
    times, commands = replay_command_arrays(bundle, ["J1"])
    assert times.tolist() == [0, 0.02, 0.04]
    assert commands.shape == (3, 1)
    with pytest.raises(ValueError, match="timestamps_seconds"):
        replay_command_arrays({"timestamps_s": [0, 1], "positions_rad": [[0], [1]]}, ["J1"])
    with pytest.raises(ValueError, match="dimensions or times"):
        replay_command_arrays({**bundle, "timestamps_seconds": [0, 0.02, 0.01]}, ["J1"])
    source = (Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")
    assert source.count("timestamps, positions = replay_command_arrays(bundle, expected_joint_names)") == 2
    assert 'bundle["timestamps_s"]' not in source


def test_full_asset_audit_uses_package_import_after_app_before_world():
    from pathlib import Path
    source = (Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")
    # Run the real package audit (including its CAD-relative import); source
    # ordering additionally proves the heavyweight adapter consumes it safely.
    from unloading_sim import asset_audit
    audit = asset_audit.audit_m710id70_asset_set(Path(__file__).resolve().parents[1])
    assert audit["tool"]["manifest_sha256"]
    assert source.index("simulation_app = SimulationApp(") < source.index("from unloading_sim.m710_execution import audit_m710_replay_assets")
    assert source.index("current_asset_audit = audit_m710_replay_assets") < source.index("world = World(")
    assert "_m710_asset_audit_pre_simulation_gate" not in source


def test_continuation_requests_reject_other_worlds_and_stale_actual_state():
    expected = {"world_session_id": "retained_world", "actual_state_sha256": "a" * 64}
    assert validate_continuation_request({"bundle_path": "next.json", **expected}, **expected) == expected
    assert validate_continuation_request({"stop": True}, **expected) == expected
    for field, wrong in (("world_session_id", "old_world"), ("actual_state_sha256", "b" * 64)):
        with pytest.raises(ValueError, match=f"stale {field}"):
            validate_continuation_request({"stop": True, **expected, field: wrong}, **expected)
