import itertools
import json
import os
from pathlib import Path
import shutil

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.validation_config import DEFAULT, _canonical_digest, load_validation_config
from unloading_sim.validation_motion import Cell, escape_path_proposals
from unloading_sim.validation_scenes import grid_tasks
from tools.run_m710id70_v3 import Run, TASK_CACHE_SCHEMA


def _override(tmp_path: Path, body: str, name: str = "override.yaml") -> Path:
    path = tmp_path / name
    path.write_text(f'extends: "{DEFAULT.as_posix()}"\n{body}', encoding="utf-8")
    return path


def _urdf_override(tmp_path: Path) -> tuple[Path, Path]:
    source = DEFAULT.parents[2] / "assets/robots/fanuc_m710id_70/m710id_70.urdf"
    urdf = tmp_path / "robot.urdf"
    shutil.copyfile(source, urdf)
    return _override(tmp_path, "robot:\n  urdf_path: robot.urdf\n"), urdf


def test_model_asset_fingerprint_changes_when_same_urdf_path_content_changes(tmp_path):
    config, urdf = _urdf_override(tmp_path)
    before = load_validation_config(config)
    content = urdf.read_text(encoding="utf-8")
    urdf.write_text(content.replace('box size="0.565 0.56 0.40"',
                                    'box size="0.566 0.56 0.40"'), encoding="utf-8")
    after = load_validation_config(config)

    assert before.urdf_path == after.urdf_path
    assert before.asset_manifest["semantic_fingerprint_sha256"] != after.asset_manifest["semantic_fingerprint_sha256"]
    before_run=Run(before,tmp_path/"before");after_run=Run(after,tmp_path/"after")
    assert before_run.configuration_digest != after_run.configuration_digest
    target=OBB([0,0,0],[.1,.1,.1],np.eye(3),"target")
    before_key,before_digest=before_run.task_identity(Cell(before),target,[target],np.zeros(6),(0,.2),1,"fixed",None,False)
    _,after_digest=after_run.task_identity(Cell(after),target,[target],np.zeros(6),(0,.2),1,"fixed",None,False)
    cache=tmp_path/"old-task.json";old_result={"validation_evidence":{
        "model_asset_fingerprint_sha256":before.asset_manifest["semantic_fingerprint_sha256"]}}
    cache.write_text(json.dumps({"cache_schema_version":TASK_CACHE_SCHEMA,
        "input_digest":before_digest,"inputs":before_key,"result":old_result}),encoding="utf-8")
    assert Run.cached_result(cache,after_digest,after.asset_manifest["semantic_fingerprint_sha256"]) is None


def test_actual_tool_collision_asset_change_updates_fingerprint(tmp_path):
    source = DEFAULT.parents[2] / "configs/tools/unloading_gripper_20kg.yaml"
    tool = tmp_path / "tool.yaml"
    shutil.copyfile(source, tool)
    config = _override(tmp_path, "tool:\n  config: tool.yaml\n")
    before = load_validation_config(config)
    content = tool.read_text(encoding="utf-8")
    tool.write_text(content.replace("outer_size_m: [0.576, 0.288, 0.250]",
                                    "outer_size_m: [0.577, 0.288, 0.250]"), encoding="utf-8")
    after = load_validation_config(config)

    assert before.asset_manifest["semantic_fingerprint_sha256"] != after.asset_manifest["semantic_fingerprint_sha256"]
    assert before.robot().tool_collision_size[1] != after.robot().tool_collision_size[1]


def test_asset_fingerprint_ignores_mtime_and_canonical_digest_ignores_mapping_order(tmp_path):
    config, urdf = _urdf_override(tmp_path)
    before = load_validation_config(config).asset_manifest["semantic_fingerprint_sha256"]
    stat = urdf.stat();os.utime(urdf, (stat.st_atime + 100, stat.st_mtime + 100))
    after = load_validation_config(config).asset_manifest["semantic_fingerprint_sha256"]

    assert before == after
    assert _canonical_digest({"assets": {"b": "2", "a": "1"}, "backend": "x"}) == \
           _canonical_digest({"backend": "x", "assets": {"a": "1", "b": "2"}})


def test_required_model_asset_missing_fails_with_named_field(tmp_path):
    config = _override(tmp_path, "robot:\n  urdf_path: missing.urdf\n")

    with pytest.raises(ValueError, match="required model asset robot_urdf is unreadable"):
        load_validation_config(config)


def test_old_cache_without_new_schema_and_fingerprint_is_not_reused(tmp_path):
    path = tmp_path / "task.json";digest = "same-input"
    path.write_text(json.dumps({"input_digest": digest, "result": {"sentinel": True}}), encoding="utf-8")

    assert Run.cached_result(path, digest, "asset") is None

    path.write_text(json.dumps({"cache_schema_version": TASK_CACHE_SCHEMA,
                                "input_digest": digest, "result": {"sentinel": True}}), encoding="utf-8")
    assert Run.cached_result(path, digest, "asset") is None

    current = {"sentinel": True, "validation_evidence": {"model_asset_fingerprint_sha256": "asset"}}
    path.write_text(json.dumps({"cache_schema_version": TASK_CACHE_SCHEMA,
                                "input_digest": digest, "result": current}), encoding="utf-8")
    assert Run.cached_result(path, digest, "asset") == current


@pytest.mark.parametrize(("body", "field"), [
    ("planning:\n  collision_margin_m: -.001\n", "planning.collision_margin_m"),
    ("planning:\n  cartesian_step_m: 0\n", "planning.cartesian_step_m"),
    ("planning:\n  ik_iterations: true\n", "planning.ik_iterations"),
    ("planning:\n  escape_path_attempt_limit: -1\n", "planning.escape_path_attempt_limit"),
    ("planning:\n  ik_restarts: 1.5\n", "planning.ik_restarts"),
    ("planning:\n  stage_ik_candidate_limit: 0\n", "planning.stage_ik_candidate_limit"),
    ("planning:\n  stage_connection_attempt_limit: true\n", "planning.stage_connection_attempt_limit"),
    ("planning:\n  stage_connection_iteration_budget: 0\n", "planning.stage_connection_iteration_budget"),
    ("planning:\n  ik_candidate_dedup_tolerance_rad: 0\n", "planning.ik_candidate_dedup_tolerance_rad"),
    ("planning:\n  stage_ik_search_mode: unlimited\n", "planning.stage_ik_search_mode"),
    ("planning:\n  contact_tolerance_m: .inf\n", "planning.contact_tolerance_m"),
    ("scene:\n  random_type_a_probability: 1.1\n", "scene.random_type_a_probability"),
    ("scene:\n  neighbor_gap_m: -.01\n", "scene.neighbor_gap_m"),
    ("scene:\n  random_seeds: [true]\n", "scene.random_seeds\\[0\\]"),
    ("robot:\n  home_joints: [0, 1]\n", "robot.home_joints"),
    ("robot:\n  installation_xyz_m: [.nan, 0, 0]\n", "robot.installation_xyz_m"),
    ("conveyor:\n  minimum_receiving_surface_z_m: 2\n  maximum_receiving_surface_z_m: 1\n",
     "conveyor receiving Z bounds"),
])
def test_invalid_numeric_configuration_fails_before_planning(tmp_path, body, field):
    with pytest.raises(ValueError, match=field.replace(".", r"\.")):
        load_validation_config(_override(tmp_path, body))


def test_signed_offsets_and_zero_disabled_escape_budget_remain_valid(tmp_path):
    config = _override(tmp_path, "planning:\n  grasp_face_offset_candidates_m: [0, -.025, .025]\n"
                                "  grasp_tilt_candidates_rad: [0, -.0005, .0005]\n"
                                "  escape_rotation_candidates_rad: [0, -.1, .1]\n"
                                "  escape_path_attempt_limit: 0\n")

    loaded = load_validation_config(config)

    assert loaded.data["planning"]["escape_path_attempt_limit"] == 0


CONTACT_Q = np.array([-0.300479359612743, 1.2937834890279378, 0.4895834197945346,
                      3.666909416623987e-06, -0.7665782546700639, -4.411918905504988])


def _contact_fixture():
    cfg = load_validation_config();cell = Cell(cfg)
    _, target, neighbors, valid = list(grid_tasks(cfg.data["scene"]))[22]
    conveyor = cfg.data["conveyor"]
    obstacles = [*cell.fixtures(), *neighbors,
                 *cell.decks((conveyor["fixed_extension_m"], conveyor["fixed_z_m"])), target]
    assert valid
    return cell, target, obstacles


def test_actual_contact_endpoint_is_closed_before_attachment_and_is_continuous():
    cell, target, obstacles = _contact_fixture()

    state, failure = cell.validate_contact_endpoint(CONTACT_Q, target, "top", obstacles)

    assert failure is None and state is not None
    assert state.coverage["geometric_coverage"]
    assert state.collision_validation["valid"]
    assert state.model_asset_fingerprint_sha256 == \
           cell.config.asset_manifest["semantic_fingerprint_sha256"]
    assert state.evidence()["attachment_pose_continuity_max_abs"] < 1e-12
    assert np.allclose(state.attachment.box_at(cell.robot.fk(CONTACT_Q)).world_from_local,
                       target.world_from_local, atol=1e-12)


def test_actual_contact_endpoint_outside_coverage_is_rejected_without_attachment():
    cell, target, obstacles = _contact_fixture();original = target.world_from_local.copy()
    shifted = OBB(target.center + target.rotation[:, 0] * .2, target.half_extents,
                  target.rotation, target.name, target.category)
    obstacles = [box if box.name != target.name else shifted for box in obstacles]

    state, failure = cell.validate_contact_endpoint(CONTACT_Q, shifted, "top", obstacles)

    assert state is None
    assert failure["reason"] == "FINAL_CONTACT_COVERAGE_FAILED"
    assert not failure["coverage"]["geometric_coverage"]
    assert np.array_equal(target.world_from_local, original)


def _escape_scene():
    target = OBB([1.3, 0, .8], [.3, .2, .15], np.eye(3), "target")
    left = OBB([1.3, .41, .8], [.3, .2, .15], np.eye(3), "left")
    right = OBB([1.3, -.41, .8], [.3, .2, .15], np.eye(3), "right")
    return target, [left, right]


def test_escape_schedule_covers_stations_and_directions_before_rotation_refinement():
    planning = load_validation_config().data["planning"]
    target, constraints = _escape_scene()
    first = list(itertools.islice(
        escape_path_proposals(target, [-1, 0, 0], constraints, .62, planning), 4))
    again = list(itertools.islice(
        escape_path_proposals(target, [-1, 0, 0], constraints, .62, planning), 4))

    assert first == again
    assert len({item["station_index"] for item in first}) >= 3
    assert len({item["escape_direction"] for item in first}) >= 3
    assert {item["rotation_index"] for item in first} == {0}


def test_escape_keeps_longer_detour_and_global_budget_can_continue_after_first_rejection():
    planning = load_validation_config().data["planning"]
    target, constraints = _escape_scene()
    proposals = list(itertools.islice(
        escape_path_proposals(target, [-1, 0, 0], constraints, .62, planning),
        planning["escape_path_attempt_limit"] + 1))
    attempted = []
    selected = None
    for proposal in proposals[:planning["escape_path_attempt_limit"]]:
        attempted.append(proposal)
        if proposal["station_index"] > 0:
            selected = proposal
            break

    assert len(attempted) == 2
    assert selected["station_index"] > 0
    assert any(item["longer_than_pure_straight"] for item in proposals)
    assert len(proposals[:planning["escape_path_attempt_limit"]]) == planning["escape_path_attempt_limit"]


def test_pure_straight_cartesian_candidate_still_uses_real_path_validation():
    cell, _, _ = _contact_fixture();destination = cell.robot.fk(CONTACT_Q).copy()
    destination[:3, 3] += np.array([0, 0, .01])

    path, failure = cell.cartesian(CONTACT_Q, destination, [], 991)

    assert failure is None
    assert len(path) >= 2
    assert cell.path_failure(path, []) is None
