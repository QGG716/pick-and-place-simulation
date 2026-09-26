"""Real C++ Tesseract/FCL/OMPL checks; explicitly separate from transport fakes."""
import json
import os
from pathlib import Path

import numpy as np
import pytest

from tools.run_tesseract_ompl_comparison import fixture_context
from unloading_sim.geometry import OBB
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.tesseract_ompl_backend import NativeWorker


@pytest.fixture(scope="module")
def native():
    executable = os.environ.get("UNLOADING_TESSERACT_WORKER")
    if not executable:
        pytest.skip("native executable not configured; not evidence of native integration")
    fixtures = Path(__file__).parent / "fixtures/tesseract_ompl"
    world, segment, c, attachment = fixture_context(fixtures / "historical_state.json", fixtures / "historical_segment.json")
    q = np.asarray(segment["path"][segment["stage_ranges"]["transit"][0]])
    scene = export_scene(c, world.all_obstacles, stage="pregrasp")
    worker = NativeWorker(executable)
    data = dict(scene=scene, q_start=q.tolist(), q_goal=q.tolist(), seed=71070, max_state_checks=2000)
    evidence = []
    def call(**changes):
        raw = worker.call({**data, **changes})
        evidence.append(raw)
        return raw
    yield world, segment, c, attachment, q, scene, call
    destination = os.environ.get("UNLOADING_NATIVE_TEST_EVIDENCE")
    if destination:
        Path(destination).write_text(json.dumps(evidence, indent=2))
    worker.close()


def test_native_fk_joint_order_frames_and_no_convex_conversion(native):
    world, segment, c, a, q, scene, call = native
    probes = [q, np.zeros(6), np.array([.2, -.3, .4, .5, -.6, .7])]
    result = call(fk_probes=[x.tolist() for x in probes])
    assert result["status"] == "CANDIDATE"
    assert scene["joint_names"] == [f"J{i}" for i in range(1, 7)]
    assert 'tesseract:make_convex="false"' in scene["urdf"]
    errors = []
    for state, observed in zip(probes, result["fk_probes"]):
        expected = c.robot.named_link_frames(state)
        for link in ["base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link", "flange", "fanuc_flange", "tool0"]:
            err = float(np.max(np.abs(expected[link]-np.asarray(observed[link]))))
            errors.append(dict(link=link, max_abs_error=err))
            assert err < 1e-10
        assert np.max(np.abs(c.robot.fk(state)-observed["backend_tcp"])) < 1e-10
    result["fk_agreement"] = dict(tolerance=1e-10, errors=errors)


def test_native_limits_collision_cancel_budget(native, tmp_path):
    world, segment, c, a, q, scene, call = native
    bad = q.copy(); bad[0] = 100
    assert call(q_start=bad.tolist())["status"] == "INVALID_START"
    assert call(q_goal=bad.tolist())["status"] == "INVALID_GOAL"
    assert call(max_state_checks=1)["status"] == "BUDGET_EXHAUSTED"
    marker = tmp_path / "cancel"; marker.touch()
    # NativeWorker owns its cancel marker; exercise in-flight polling predicate.
    w = NativeWorker(os.environ["UNLOADING_TESSERACT_WORKER"])
    try:
        result = w.call(dict(scene=scene, q_start=q.tolist(), q_goal=q.tolist(), seed=1, max_state_checks=100), lambda: True)
        assert result["status"] == "CANCELLED"
    finally:
        w.close()


def test_native_attachment_changes_real_collision_queries(native):
    world, segment, c, a, q, scene, call = native
    lo, hi = segment["stage_ranges"]["transit"]
    points = np.asarray(segment["path"])
    # This is the real fixed straight-line witness, just after departure from
    # the stack: unloaded tool clears, the attached carton hits its neighbor.
    witness = points[lo] + .003*(points[hi]-points[lo])
    assert c._state_failure(witness, world.all_obstacles, stage="pregrasp") is None
    assert call(q_start=witness.tolist(), q_goal=witness.tolist())["status"] == "CANDIDATE"
    obs = [b for b in world.all_obstacles if b.name != segment["target"]]
    loaded = export_scene(c, obs, stage="transit", attachment=a)
    result = call(scene=loaded, q_start=witness.tolist(), q_goal=witness.tolist())
    assert result["status"] == "INVALID_START"
    assert segment["target"] in result["failure"]["pair"]
    assert call(scene=loaded, q_goal=witness.tolist())["status"] == "INVALID_GOAL"
    assert sum(b["name"] == segment["target"] for b in loaded["boxes"]) == 1
    assert loaded["attachment"]["parent"] == "backend_tcp"
    with pytest.raises(ValueError, match="still present"):
        export_scene(c, world.all_obstacles, stage="transit", attachment=a)


def test_native_scene_update_invalidates_geometry_and_permissions_do_not_expand(native):
    world, segment, c, a, q, scene, call = native
    assert call()["status"] == "CANDIDATE"
    assert call()["environment_reused"]
    tool = c.robot_state_validator.tool_transform_robot.tool_collision_obbs(q)[0]
    obstacle = OBB(tool.center, np.array([.008, .008, .008]), np.eye(3), "J5_link_tool_environment_obstacle", "fixture")
    changed = export_scene(c, [*world.all_obstacles, obstacle], stage="pregrasp")
    assert changed["fingerprint"] != scene["fingerprint"]
    result = call(scene=changed)
    assert not result["environment_reused"] and result["status"] == "INVALID_START"
    assert obstacle.name in result["failure"]["pair"]
    result = call()
    assert not result["environment_reused"] and result["status"] == "CANDIDATE"


def test_process_constraints_are_explicitly_unsupported(native):
    world, segment, c, a, q, scene, call = native
    for stage in ["contact", "extraction", "withdrawal", "place", "residence"]:
        with pytest.raises(ValueError, match="UNSUPPORTED_CONSTRAINT"):
            export_scene(c, world.all_obstacles, stage=stage)


def test_native_rrtconnect_synthetic_detour_separate_from_fanuc(native):
    # Deliberately a UNIT scene, not an approximate replacement for FANUC.
    # It makes a real RRTConnect solution cheap enough to regress independently
    # of the engineering sample's finite-budget outcome.
    from unloading_sim.planning_contract import fingerprint
    from unloading_sim.pair_clearance import obb_surface_distance
    *_, call = native
    urdf = '''<robot name="unit_xy" xmlns:tesseract="https://tesseract-robotics.github.io" tesseract:make_convex="false">
      <link name="world"/><link name="base_link"/><link name="x"/>
      <link name="flange"><collision><geometry><box size="0.10 0.10 0.10"/></geometry></collision></link>
      <link name="tool0"/><link name="backend_tcp"/>
      <link name="obstacle"><collision><geometry><box size="0.2 0.6 0.2"/></geometry></collision></link>
      <joint name="base" type="fixed"><parent link="world"/><child link="base_link"/></joint>
      <joint name="Jx" type="prismatic"><parent link="base_link"/><child link="x"/><origin xyz="0 0 0.5"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="Jy" type="prismatic"><parent link="x"/><child link="flange"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="tool" type="fixed"><parent link="flange"/><child link="tool0"/></joint>
      <joint name="tcp" type="fixed"><parent link="tool0"/><child link="backend_tcp"/></joint>
      <joint name="obstacle_mount" type="fixed"><parent link="world"/><child link="obstacle"/><origin xyz="0 0 0.5"/></joint>
      </robot>'''
    scene = dict(urdf=urdf, srdf='<robot name="unit_xy"><group name="manipulator"><chain base_link="base_link" tip_link="tool0"/></group></robot>',
        joint_names=["Jx", "Jy"], joint_limits=[[-1, 1], [-1, 1]],
        joints=[dict(name="Jx", link="x", type="prismatic", axis=[1, 0, 0]),
                dict(name="Jy", link="flange", type="prismatic", axis=[0, 1, 0])],
        active_links=["flange"], collision_objects=["flange", "obstacle"], default_margin=.005, pair_margins=[],
        constraints=dict(joint_margin=0., maximum_jacobian_condition=100., radial_limit=None, edge_resolution_rad=.055, point_motion_bound_m=.00125, lever_arm_m=4.))
    scene["fingerprint"] = fingerprint(scene)
    result = call(scene=scene, q_start=[-.6, 0], q_goal=[.6, 0], seed=71070, max_state_checks=100000)
    result["fixture_kind"] = "SYNTHETIC_XY_UNIT_NOT_FANUC_ENGINEERING"
    assert result["status"] == "CANDIDATE" and result["exact_solution"]
    assert result["direct_valid"] is False and result["ompl_status"] == "Exact solution"
    obstacle = OBB([0, 0, .5], [.1, .3, .1], np.eye(3), "obstacle", "unit")
    samples = 0
    for first, last in zip(result["path"][:-1], result["path"][1:]):
        first, last = np.asarray(first), np.asarray(last)
        n = max(1, int(np.ceil(np.abs(last-first).sum()/.0003125)))
        for u in np.linspace(0, 1, n+1):
            q = first + u*(last-first)
            moving = OBB([q[0], q[1], .5], [.05]*3, np.eye(3), "moving", "unit")
            assert obb_surface_distance(moving, obstacle) + 1e-9 >= .005
            samples += 1
    result["independent_repository_geometry_recheck"] = dict(accepted=True, samples=samples)
    limited = call(scene=scene, q_start=[-.6, 0], q_goal=[.6, 0], seed=71070, max_state_checks=10000)
    limited["fixture_kind"] = "SYNTHETIC_XY_UNIT_APPROXIMATE_REJECTION"
    assert limited["status"] == "BUDGET_EXHAUSTED" and limited["counters"]["state_checks"] == 10000
    assert limited["approximate_solution_found"] and limited["candidate_found"]
    assert not limited["exact_solution"] and not limited["native_validated"] and limited["path"] == []


@pytest.mark.parametrize("key,value", [("lever_arm_m", 2.), ("point_motion_bound_m", .0025),
                                       ("lever_arm_m", 0.), ("edge_resolution_rad", .1)])
def test_real_worker_rejects_unsupported_subdivision(native, key, value):
    from copy import deepcopy
    from unloading_sim.planning_contract import fingerprint
    *_, scene, call = native
    changed=deepcopy(scene)
    changed["constraints"][key]=value
    changed["fingerprint"]=fingerprint({k:v for k,v in changed.items() if k!="fingerprint"})
    result=call(scene=changed)
    assert result["status"] == "UNSUPPORTED_CONSTRAINT"
    assert result.get("counters",{}).get("state_checks",0) == 0


def test_real_audit_grid_coverage_refinement_and_incomplete_edge(native):
    *_, q, scene, call=native
    end=q.copy(); end[0]+=.001
    kwargs=dict(operation="audit",audit_path=[q.tolist(),end.tolist()],trace_samples=True)
    result=call(**kwargs)
    assert result["status"] == "AUDIT_VALID" and result["counters"]["valid_edges"] == 1
    samples=result["checked_samples"]
    n=samples[0]["subdivisions"]
    assert n == 4
    assert [s["index"] for s in samples][:3] == [0,n,n//2]
    assert sorted(s["index"] for s in samples) == list(range(n+1))
    finer=call(**kwargs,refinement=1,l1_resolution_rad=.00015625)
    assert finer["effective_rule"]["l1_resolution_rad"] == .00015625
    assert finer["checked_samples"][0]["subdivisions"] == 7
    assert len(finer["checked_samples"]) == 8
    from copy import deepcopy
    from unloading_sim.planning_contract import fingerprint
    angular=deepcopy(scene);angular["constraints"]["edge_resolution_rad"]=.0001
    angular["fingerprint"]=fingerprint({k:v for k,v in angular.items() if k!="fingerprint"})
    changed=call(**kwargs,scene=angular)
    assert changed["checked_samples"][0]["subdivisions"] == 22
    assert len(changed["checked_samples"]) == 23
    for bad_refinement in [True, -2147483648, 8, 1.5]:
        assert call(**kwargs,refinement=bad_refinement)["status"] == "UNSUPPORTED_CONSTRAINT"
    assert call(**kwargs,l1_resolution_rad=.0002)["status"] == "UNSUPPORTED_CONSTRAINT"
    incomplete=call(**kwargs,max_state_checks=2)
    assert incomplete["status"] == "BUDGET_EXHAUSTED" and not incomplete["audit_complete"]
    assert incomplete["counters"]["valid_edges"] == 0
    assert incomplete["counters"]["incomplete_edges"] == 1
    assert incomplete["counters"]["invalid_edges"] == 0


def test_real_exact_cache_scoped_to_each_request_and_identity(native):
    from copy import deepcopy
    from unloading_sim.planning_contract import fingerprint
    world,segment,c,attachment,q,scene,call=native
    near=q.copy(); near[0]=np.nextafter(near[0],np.inf)
    states=[q.tolist(),q.tolist(),near.tolist(),q.tolist()]
    raw=call(operation="audit",states=states)
    assert raw["state_verdicts"] == [True]*4
    assert raw["counters"]["state_requests"] == 4
    assert raw["counters"]["state_checks"] == 2
    assert raw["counters"]["cache_hits"] == 2
    # Even a reused environment gets a fresh request-local cache.
    assert call(operation="audit",states=[q.tolist()])["counters"]["state_checks"] == 1
    for field in ["policy_fingerprint", "stage", "attachment"]:
        changed=deepcopy(scene)
        changed[field] = "changed" if field!="attachment" else {"identity":"changed"}
        changed["fingerprint"]=fingerprint({k:v for k,v in changed.items() if k!="fingerprint"})
        raw=call(scene=changed,operation="audit",states=[q.tolist()])
        assert raw["counters"]["cache_hits"] == 0 and raw["counters"]["state_checks"] == 1


def test_real_middle_collision_rejected_before_dense_prefix(native):
    world,segment,c,attachment,q,scene,call=native
    # Fixed engineering straight edge: both endpoints pass, interior collides.
    path=np.asarray(segment["path"])
    first,last=path[78],path[208]
    result=call(operation="audit",audit_path=[first.tolist(),last.tolist()],trace_samples=True)
    assert result["status"] == "AUDIT_INVALID"
    samples=result["checked_samples"]
    assert samples[0]["index"] == 0 and samples[1]["index"] == samples[0]["subdivisions"]
    assert len(samples) == 3 and samples[-1]["index"] == samples[0]["subdivisions"]//2
    authority_failure=c._state_failure(np.asarray(samples[-1]["q_rad"]),world.all_obstacles,stage="pregrasp")
    result["authority_first_rejection"]=authority_failure
    assert authority_failure is not None
    assert result["counters"]["invalid_edges"] == 1
    assert result["counters"]["incomplete_edges"] == 0



def test_known_engineering_path_audit_evidence_matches_running_binary():
    import hashlib
    destination=os.environ.get("UNLOADING_KNOWN_PATH_AUDIT")
    if not destination:
        pytest.skip("full engineering audit evidence is supplied separately from small native tests")
    result=json.loads(Path(destination).read_text())
    assert result["worker_sha256"] == hashlib.sha256(Path(os.environ["UNLOADING_TESSERACT_WORKER"]).read_bytes()).hexdigest()
    assert result["native_path"]["status"] == "AUDIT_VALID"
    assert result["native_path"]["audit_complete"]
    assert result["native_path"]["counters"]["valid_edges"] == 28
    assert result["authority_evidence_reuse"]["identical_actual_q_grid"]
    assert result["authority_evidence_reuse"]["same_q_mismatches"] == 0
    assert not result["search_started"]
