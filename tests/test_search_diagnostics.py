import json
import numpy as np
import pytest

from unloading_sim.layout_trajectory import LayoutTrajectoryBudget, LayoutTrajectoryConnector
from unloading_sim.search_diagnostics import SearchDiagnostics
from unloading_sim.layout_single_carton import run_layout_single_carton_audit
from unloading_sim.planning_profile import DEFAULT_MOTION
from test_layout_trajectory import _Robot


def connector(validator, diagnostics=None):
    result = LayoutTrajectoryConnector(_Robot(), validator,
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
        ik_policy={}, collision_margin_m=.005, contact_tolerance_m=.0002,
        joint_margin_rad=.01, maximum_jacobian_condition=1e4,
        validator_identity="test", execution_qualified=True,
        budget=LayoutTrajectoryBudget(planning_wall_time_s=None))
    result.diagnostics = diagnostics
    result.start_planning_request()
    return result


def blocked(q, *args, **kwargs):
    if .15 < q[0] < .25:
        return dict(reason="TOOL_COLLISION", classification="PROXY_GEOMETRY_INTERSECTION",
                    pair=["tool_rigid_0", "wall"], surface_distance_m=0., required_pair_clearance_m=.005)


def test_rrt_diagnostics_observe_real_rejections_without_changing_search(tmp_path):
    d = SearchDiagnostics(tmp_path / "diagnostics.json")
    outputs = []
    for diagnostic in (None, d):
        c = connector(blocked, diagnostic)
        outputs.append(c._transit(np.zeros(6), np.array([.4, 0, 0, 0, 0, 0]), [],
            seed=44, iteration_budget=40, stage="pregrasp"))
    assert outputs[0][1] == outputs[1][1]
    assert outputs[0][2] == outputs[1][2]
    assert not outputs[1][0]
    assert d.groups
    example = next(iter(d.groups.values()))["examples"][0]
    assert example["origin"] == "rrt_internal"
    assert example["edge"]["start_q_rad"] == [0.] * 6
    assert blocked(example["q_rad"])["pair"] == example["failure"]["pair"]
    assert json.loads(d.path.read_text())["contexts"]


def test_successful_rrt_full_edge_recheck_has_separate_rejection():
    # Thin interval between the RRT's coarse samples; production recheck must catch it.
    def thin(q, *args, **kwargs):
        if .016 < q[0] < .017:
            return dict(reason="CLEARANCE", classification="CLEARANCE_INSUFFICIENT", pair=["a", "b"])
    d = SearchDiagnostics()
    c = connector(thin, d)
    path, failure, evidence = c._transit(np.zeros(6), np.array([.1, 0, 0, 0, 0, 0]), [],
        seed=2, iteration_budget=40, stage="transit")
    assert evidence["search_success"] and not evidence["success"] and not path
    assert failure["reason"] == "CLEARANCE"
    assert next(iter(d.groups.values()))["examples"][0]["origin"] == "rrt_success_full_edge_recheck"


def test_counts_include_cached_rejections_and_examples_are_bounded():
    d = SearchDiagnostics(examples_per_group=2)
    c = connector(blocked, d)
    for _ in range(20):
        assert c.validate_unloaded_state([.2, 0, 0, 0, 0, 0], [], stage="contact")
    group = next(iter(d.groups.values()))
    assert group["count"] == 20 and len(group["examples"]) == 2
    assert c._statistics["state_cache_hits"] == 19
    assert len(d.contexts) == 1


@pytest.mark.parametrize("target", ["carton_l00_c00", "unknown"])
def test_explicit_target_cannot_bypass_current_row_legality(target):
    with pytest.raises(ValueError, match="not a legal current row candidate"):
        run_layout_single_carton_audit(DEFAULT_MOTION, target_id=target)


def test_scheduler_identity_overlap_and_error_checkpoint(tmp_path):
    d = SearchDiagnostics(tmp_path / "diagnostics.json")
    d.bind_candidate(dict(face="top", candidate_id="c", retry_index=1, path_seed=42),
                     target="box", face="top", roll=0)
    d.event(dict(event="ERROR", reason="interrupted"))
    assert d.candidate["path_seed"] == 42 and d.candidate["target"] == "box"
    assert json.loads(d.path.read_text())["last_event"]["reason"] == "interrupted"


@pytest.mark.parametrize("quota", [None, 720])
def test_production_connector_wires_existing_local_sample_quota(quota, tmp_path):
    from copy import deepcopy
    from pathlib import Path
    import yaml
    from unloading_sim.layout_single_carton import (
        load_layout_motion_policy, build_verified_motion_input,
        _build_automatic_trajectory_connector,
    )
    policy = load_layout_motion_policy(DEFAULT_MOTION)
    data = deepcopy(policy.data)
    data["layout_validation_config"] = str((Path(DEFAULT_MOTION).parent /
                                           data["layout_validation_config"]).resolve())
    if quota is not None:
        data["search_strategy"]["local_transit_cartesian_sample_budget"] = quota
    config = tmp_path / "policy.yaml"
    config.write_text(yaml.safe_dump(data), encoding="utf-8")
    scene = build_verified_motion_input(load_layout_motion_policy(config), project_root=policy.project_root)
    built = _build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot())
    assert built.status == "AVAILABLE"
    assert built.connector.budget.local_transit_cartesian_sample_budget == (quota or 240)
    assert built.connector.budget.planning_wall_time_s is None


def test_cartesian_rejection_records_actual_sample_seed_and_stage(tmp_path):
    d = SearchDiagnostics(tmp_path / "diagnostics.json")
    def stop(q, *args, **kwargs):
        if q[0] > .01:
            return dict(reason="TOOL_COLLISION", pair=["tool", "wall"])
    c = connector(stop, d)
    class CartesianRobot(_Robot):
        def clamp(self, q):
            return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])
    c.robot = CartesianRobot()
    c.ik = dict(max_iterations=50, damping=.001, max_step_rad=.1,
                position_tolerance_m=1e-5, orientation_tolerance_rad=1e-5, orientation_weight=1.)
    destination = np.eye(4)
    destination[0, 3] = .03
    _, failure, _ = c._cartesian(np.zeros(6), destination, [], seed=101, stage="transit")
    assert failure["reason"] == "TOOL_COLLISION"
    example = next(iter(d.groups.values()))["examples"][0]
    context = d.contexts[example["context"]]
    assert context["ik_seed"] == 101 + context["cartesian_sample_index"]
    assert context["ik_seed_kind"] == "PREVIOUS_Q_NO_RANDOM_RESTARTS"
    d.flush()
    snapshot = json.loads(d.path.read_text())
    assert snapshot["last_validation"]["stage"] == "transit"
    assert sum(snapshot["validation_queries"].values()) == d.query_count


def test_cartesian_numerical_failure_keeps_real_ik_result():
    from unloading_sim.geometry import rotation_matrix_from_rotation_vector
    class TranslationOnlyRobot(_Robot):
        def clamp(self, q):
            return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])
        def geometric_jacobian(self, q):
            return np.diag([1., 1., 1., 0., 0., 0.])
    d = SearchDiagnostics()
    c = connector(blocked, d)
    c.robot = TranslationOnlyRobot()
    c.ik = dict(max_iterations=3, damping=.001, max_step_rad=.1,
                position_tolerance_m=1e-5, orientation_tolerance_rad=1e-5, orientation_weight=1.)
    destination = np.eye(4)
    destination[:3, :3] = rotation_matrix_from_rotation_vector([0., 0., .05])
    path, failure, _ = c._cartesian(np.zeros(6), destination, [], seed=11, stage="transit")
    assert len(path) == 1 and failure["reason"] == "NO_IK"
    example = next(iter(d.outcomes.values()))["examples"][0]
    assert example["q_rad"] == example["seed_q_rad"] == [0.] * 6
    assert example["ik_seed"] == 12 and example["random_restarts"] == 0
    assert example["failure"]["orientation_error_rad"] == pytest.approx(.05)
