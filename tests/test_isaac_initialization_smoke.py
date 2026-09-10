"""Diagnostic adapter contracts, separate from execution-preflight tests."""
from __future__ import annotations

import ast
import copy
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.isaac_collision_policy import apply_robot_only_srdf_filters, expand_robot_only_srdf_pairs
from unloading_sim.m710_initialization_diagnostic import (
    SCOPE, build_initialization_diagnostic_contract, combined_j6_inertial,
    generated_usd_tree_identity,
    summarize_initialization_motion, trailer_side_wall_transform,
    verify_initialization_diagnostic_contract,
)
from unloading_sim.workcell_layout import canonical_digest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def contract():
    return build_initialization_diagnostic_contract(
        ROOT / "configs/validation/m710id70_layout_v1.yaml",
        ROOT / "configs/simulation/m710id70_official_dynamics_v2.yaml", ROOT)


def test_diagnostic_preserves_clearance_failure_and_full_mass_inventory(contract):
    assert contract["scope"] == SCOPE
    assert contract["initial_state_audit"]["status"] == "FAIL"
    assert contract["initial_state_audit"]["failures"]
    assert not contract["motion_execution_permitted"]
    assert not contract["attachment_permitted"]
    assert sum(p["role"] == "dynamic_carton" for p in contract["primitives"]) == 40
    assert sum(p["role"] == "rigid_tool" for p in contract["primitives"]) == 58
    assert contract["dynamics"]["cartons"]["mass_kg_each"] == 42.5
    mass, _, tensor = combined_j6_inertial(contract)
    assert mass == pytest.approx(20.447)
    assert np.all(np.linalg.eigvalsh(tensor) > 0)
    assert not np.allclose(tensor, np.diag(np.diag(tensor)))


@pytest.mark.parametrize("field", ["motion_execution_permitted", "attachment_permitted"])
def test_rehashed_diagnostic_cannot_authorize_execution(contract, field):
    value = copy.deepcopy(contract)
    value[field] = True
    value.pop("contract_fingerprint")
    value["contract_fingerprint"] = canonical_digest(value)
    with pytest.raises(ValueError, match="never authorize"):
        verify_initialization_diagnostic_contract(value)


def test_srdf_excludes_only_robot_meshes_never_tool_or_whole_j6():
    links = {"J5_link": "/Robot/J5_link", "J6_link": "/Robot/J6_link"}
    shapes = {name: [path + "/collisions/mesh"] for name, path in links.items()}
    assert expand_robot_only_srdf_pairs(links, shapes, [("J5_link", "J6_link")]) == [
        ("/Robot/J5_link/collisions/mesh", "/Robot/J6_link/collisions/mesh")]
    shapes["J6_link"].append("/Robot/J6_link/SuctionToolRigidCollision_000")
    with pytest.raises(ValueError, match="tool collision"):
        expand_robot_only_srdf_pairs(links, shapes, [("J5_link", "J6_link")])
    shapes["J6_link"] = ["/Robot/J6_link"]
    with pytest.raises(ValueError, match="whole bodies"):
        expand_robot_only_srdf_pairs(links, shapes, [("J5_link", "J6_link")])


def test_rehashed_diagnostic_cannot_raise_official_effort_limits(contract):
    value = copy.deepcopy(contract)
    value["dynamics"]["joint_drives"]["J5"]["effort_limit_nm"] = 1001.0
    value.pop("contract_fingerprint")
    value["contract_fingerprint"] = canonical_digest(value)
    with pytest.raises(ValueError, match="finite official joint limits"):
        verify_initialization_diagnostic_contract(value)


def test_smoke_uses_finite_runtime_drives_and_no_per_frame_teleport():
    source = (ROOT / "scripts/isaacsim_m710_initialization_smoke.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert not any(n.func.attr == "set_dof_positions" for n in calls)
    assert not any(n.func.attr in {"set_world_poses", "set_local_poses"} for n in calls)
    assert "FixedJoint.Define" not in source
    for method in ("set_dof_max_efforts", "set_dof_max_velocities", "set_dof_gains"):
        assert any(n.func.attr == method for n in calls)
    assert source.index("CreatePositionAttr(q_deg)") < source.index("world.reset()")
    assert source.index("CreateTargetPositionAttr(q_deg)") < source.index("world.reset()")
    assert "np.degrees(q_by_name[prim.GetName()])" in source
    assert "reset_q_error > 1.0e-5" in source
    assert "physics_dt=0.0" in source
    assert source.index("reset_q_error > 1.0e-5") < source.index("world.set_simulation_dt(")
    assert "pause_timeline=False, delta_time=0.0, wait_for_render=True" in source
    assert "world.step(render=render)" not in source
    physics_step = source.index("world.step(render=False, update_fabric=True)")
    formal_capture = source.index("rt_subframes=1, pause_timeline=False")
    formal_rgb_read = source.index("rgba = np.asarray(rgb.get_data())")
    assert physics_step < formal_capture < formal_rgb_read
    assert source.index("after_capture_q =") > formal_capture
    assert '"max_capture_q_delta_rad"' in source
    assert '"second.png"' in source
    assert '"width": 1920, "height": 1080' in source
    assert "apply_robot_only_srdf_filters" in source
    replay = (ROOT / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")
    assert replay.index("apply_robot_only_srdf_filters(") < replay.index("suction_tool_path =")


def test_generated_usd_tree_identity_is_scoped_and_content_addressed(tmp_path):
    first = tmp_path / "robot_3"
    (first / "payloads").mkdir(parents=True)
    entry = first / "robot.usda"
    entry.write_bytes(b"entry")
    (first / "payloads" / "physics.usda").write_bytes(b"physics")
    sibling = tmp_path / "robot_2"
    sibling.mkdir()
    (sibling / "old.usda").write_bytes(b"not-this-run")
    identity = generated_usd_tree_identity(entry, tmp_path)
    assert identity["entrypoint"] == "robot_3/robot.usda"
    assert [item["path"] for item in identity["files"]] == [
        "robot_3/payloads/physics.usda", "robot_3/robot.usda"]
    assert [item["size_bytes"] for item in identity["files"]] == [7, 5]
    assert len(identity["aggregate_sha256"]) == 64
    changed = identity["aggregate_sha256"]
    entry.write_bytes(b"entry-changed")
    assert generated_usd_tree_identity(entry, tmp_path)["aggregate_sha256"] != changed


def test_motion_summary_does_not_call_rest_after_dispersion_stable():
    criteria = {
        "required_stable_duration_s": 0.5,
        "max_linear_speed_m_s": 0.01,
        "max_angular_speed_rad_s": 0.02,
        "max_position_drift_m": 0.005,
    }
    states = []
    for index in range(20):
        position = [0.0, 0.0, 0.0] if index == 0 else [2.0, 0.0, 0.0]
        velocity = [4.0, 0.0, 0.0] if index == 1 else [0.0, 0.0, 0.0]
        states.append({
            "time_s": (index + 1) / 10.0,
            "carton_names": ["carton_a"],
            "carton_positions_m": [position],
            "carton_linear_velocities_m_s": [velocity],
            "carton_angular_velocities_rad_s": [[0.0, 0.0, 0.0]],
            "q_rad": [0.0],
        })
    result = summarize_initialization_motion(
        states, np.zeros((1, 3)), np.zeros(1), criteria
    )
    assert result["final_window_motion_stable"] is True
    assert result["initial_configuration_retained_throughout"] is False
    assert result["classification"] == "INITIAL_CONFIGURATION_DISPERSED"
    assert result["full_run_peak_linear_speed_m_s"] == 4.0
    assert result["max_authored_position_displacement_m"] == 2.0
    assert result["worst_displaced_carton"] == "carton_a"


def test_trailer_side_wall_normals_both_point_into_known_width():
    right = trailer_side_wall_transform(-1.15, 1.0)
    left = trailer_side_wall_transform(1.15, -1.0)
    np.testing.assert_allclose(right[:3, 1], [0.0, 1.0, 0.0])
    np.testing.assert_allclose(left[:3, 1], [0.0, -1.0, 0.0])
    assert right[1, 3] == -1.15
    assert left[1, 3] == 1.15


def test_real_usd_instance_colliders_are_materialized_without_filtering_tool(tmp_path):
    Usd = pytest.importorskip("pxr.Usd")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    UsdPhysics = pytest.importorskip("pxr.UsdPhysics")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Source")
    source = UsdGeom.Mesh.Define(stage, "/Source/mesh").GetPrim()
    UsdPhysics.CollisionAPI.Apply(source)
    UsdGeom.Xform.Define(stage, "/Robot")
    names = ["base_link", *(f"J{i}_link" for i in range(1, 7))]
    parent = "/Robot"
    links = {}
    for name in names:
        path = f"{parent}/{name}"
        body = UsdGeom.Xform.Define(stage, path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        instance = UsdGeom.Xform.Define(stage, path + "/collision_instance").GetPrim()
        instance.GetReferences().AddInternalReference("/Source")
        instance.SetInstanceable(True)
        links[name] = path
        parent = path
    srdf = tmp_path / "robot.srdf"
    srdf.write_text('<robot>' + ''.join(
        f'<disable_collisions link1="{a}" link2="{b}" reason="Adjacent"/>'
        for a, b in zip(names, names[1:])) + '</robot>', encoding="utf-8")
    assert not any(str(p.GetPath()).startswith("/Robot/") and p.HasAPI(UsdPhysics.CollisionAPI)
                   for p in stage.Traverse())
    records = apply_robot_only_srdf_filters(stage, "/Robot", srdf)
    assert len(records) == 6
    assert len(records[0]["materialized_collision_instances"]) == 7
    tool = UsdGeom.Cube.Define(stage, links["J6_link"] + "/SuctionToolRigidCollision_000").GetPrim()
    UsdPhysics.CollisionAPI.Apply(tool)
    assert not tool.HasAPI(UsdPhysics.FilteredPairsAPI)
    for path in links.values():
        body = stage.GetPrimAtPath(path)
        relation = body.GetRelationship("physics:filteredPairs")
        assert not relation.IsValid() or not relation.GetTargets()
    for record in records:
        for first, second in record["collider_pairs"]:
            assert first.endswith("/collision_instance/mesh")
            assert second.endswith("/collision_instance/mesh")
            assert not stage.GetPrimAtPath(first).IsInstanceProxy()
