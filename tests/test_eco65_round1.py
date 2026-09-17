"""Focused desktop tests; uses local private assets without copying them into fixtures."""
from pathlib import Path
import copy,json
import numpy as np
import pytest
from unloading_sim.eco65.model import *
from unloading_sim.eco65.pipeline import numeric_checks,GeometricExecutionBackend,known_world
from unloading_sim.eco65.observation_contracts import Pose3D,EvidenceKind

@pytest.fixture(scope="module")
def world():
    if not (LOCAL/"tool.json").exists():pytest.skip("private STEP-derived tool not available")
    w=World(load_json(LOCAL/"tool.json"),load_json(LOCAL/"scene.json"))
    yield w
    w.close()

def test_hardware_is_unconditionally_disabled():
    with pytest.raises(PermissionError):require_simulation({"enable_hardware":True})
    with pytest.raises(PermissionError):require_simulation({"execution_mode":"hardware"})
    with pytest.raises(PermissionError):GeometricExecutionBackend(None,enable_hardware=True)

def test_perception_pose_is_explicit_xyzw_si():
    p=Pose3D((1,2,3),(0,0,0,1),"world","+X into trailer,+Y left,+Z up",EvidenceKind.SYNTHETIC)
    assert p.evidence is EvidenceKind.SYNTHETIC
    with pytest.raises(ValueError):Pose3D((1,2,3),(1,1,1,1),"world","SI",EvidenceKind.SYNTHETIC)

def test_se3_rejects_scaling_and_reflection():
    t=np.eye(4);t[0,0]=.001
    with pytest.raises(ValueError):check_transform(t)
    t[0,0]=-1
    with pytest.raises(ValueError):check_transform(t)

def test_independent_fk_and_jacobian(world):
    assert numeric_checks(world)["status"]=="PASS"

def test_actual_step_and_transforms_preserved():
    d=asset_check()
    assert d["file_length_units"]=="millimetres"
    assert d["normalization_scale_to_m"]==.001
    assert not d["external_references"] and not d["unprocessed_parts"]
    assert all(p["topology_valid"] and p["solids"]>0 for p in d["parts"])
    for n in d["assembly_nodes"]:
        check_transform(n["local_transform_m"]);check_transform(n["global_transform_m"])
    assert max(p["max_vertex_outside_hull_m"] for p in d["parts"])<1e-8

def test_tcp_chain_and_contact_pair_policy(world):
    tool=world.tool
    assert np.allclose(np.array(tool["T_flange_tool_cad"])@tool["T_tool_cad_tcp"],tool["T_flange_tcp"])
    assert world.robot.self_collision_exclusions==set()
    assert len(tool["seals"])==len([p for p in tool["visual_parts"] if p["size_m"][1]>.05 and .03<p["size_m"][0]<.05])
    assert tool["contact"]["independent_control_verified"] is False
    assert not GeometricExecutionBackend(world).capabilities.supports_continuous_handoff

def test_far_attachment_and_small_box_rejected(world):
    home=np.array(load_json(OUTPUT/"reports/home.json")["q"])
    with pytest.raises(ValueError):world.attach(home)
    plan=load_json(OUTPUT/"trajectory/plan.json")
    contact=np.array(next(s for s in plan["segments"] if s["phase"]=="contact")["q"][-1])
    size=world.scene["box"]["size_m"]
    try:
        world.scene["box"]["size_m"]=[.05,.05,.10]
        assert not world.seal_fit(contact,transform(world.scene["box"]["pose"]))[0]
    finally:world.scene["box"]["size_m"]=size

def test_rigid_penetration_and_wrong_release_rejected(world):
    from unloading_sim.ik import solve_ik
    plan=load_json(OUTPUT/"trajectory/plan.json")
    contact=np.array(next(s for s in plan["segments"] if s["phase"]=="contact")["q"][-1])
    target=world.robot.fk(contact);target[2,3]-=.02
    result=solve_ik(world.robot,target,contact,position_tolerance=.00005,orientation_tolerance=.0003,max_iterations=500)
    assert result.success
    assert not world.valid(result.q,"contact")
    assert world.last_failure["reason"]=="collision"
    world.attach(contact)
    with pytest.raises(ValueError):world.release(contact)

def test_configuration_and_robot_start_change_world_identity(world):
    q=np.array(load_json(OUTPUT/"reports/home.json")["q"])
    a=known_world(world,q);b=known_world(world,q+.001)
    assert a.fingerprint!=b.fingerprint
    assert a.robot_state_revision.fingerprint!=b.robot_state_revision.fingerprint

def test_final_task_evidence_if_finished():
    p=OUTPUT/"replay/states.json"
    if not p.exists():pytest.skip("full path validation is still running")
    d=load_json(p)
    assert d["status"]=="GEOMETRIC_REPLAY_ONLY" and d["geometric_tasks_completed"]==1
    assert d["dynamics"]=="NOT_EVALUATED" and not d["hardware_connected"]
    assert [e["event"] for e in d["events"]]==["ATTACHED","SUPPORTED_RELEASE","RETREAT_COMPLETE"]
    assert len({e["box_id"] for e in d["events"]})==1
    assert d["trajectory_fingerprint"]==load_json(OUTPUT/"trajectory/plan.json")["trajectory_fingerprint"]
    assert d["trajectory_fingerprint"]==load_json(OUTPUT/"reports/validation.json")["trajectory_fingerprint"]

def test_split_collision_hulls_cover_actual_cad_vertices(world):
    import trimesh
    from scipy.spatial import ConvexHull
    for part in world.tool['visual_parts']:
        vertices=np.asarray(trimesh.load(ROOT/part['visual'],process=False).vertices)
        covered=np.zeros(len(vertices),dtype=bool)
        for collision in world.tool['collisions']:
            if collision['source_part']!=part['id']:continue
            points=trimesh.load(ROOT/collision['mesh'],process=False).vertices
            equations=ConvexHull(points).equations
            for start in range(0,len(vertices),1000):
                v=vertices[start:start+1000]
                covered[start:start+1000] |= np.max(v@equations[:,:3].T+equations[:,3],axis=1)<1e-8
        assert covered.all(),part['id']