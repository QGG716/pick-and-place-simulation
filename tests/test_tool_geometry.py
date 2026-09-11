from __future__ import annotations

import copy
import json
from pathlib import Path
import shutil

import numpy as np
import pytest

from unloading_sim.tool_geometry import (
    TOOL_DIRECTORY,
    audit_tool_geometry,
    classify_tool_solids,
    verified_tool_bounds_step_mm,
)
from unloading_sim.workcell_layout import load_workcell_layout


ROOT = Path(__file__).resolve().parents[1]


def _analysis():
    return json.loads((ROOT / TOOL_DIRECTORY / "mass_properties.json").read_text(encoding="utf-8"))


def test_source_coverage_restores_every_rigid_insert_and_covers_all_solids():
    audit = audit_tool_geometry(ROOT)
    assert audit["coverage_verified"]
    assert audit["simulation_geometry_qualified"]
    assert audit["solid_occurrence_count"] == 202
    assert audit["historical_rigid_solid_count"] == 58
    assert audit["restored_rigid_insert_count"] == 72
    assert audit["rigid_solid_count"] == 130
    assert audit["compliant_bellows_count"] == 72
    assert set(audit["rigid_solid_indices"]).isdisjoint(audit["bellows_solid_indices"])
    assert set(audit["rigid_solid_indices"] + audit["bellows_solid_indices"]) == set(range(202))
    assert audit["machine_certification"] == "NOT_EVALUATED_SEPARATE_FROM_SIMULATION"
    assert audit["rigid_front_from_flange_m"] == pytest.approx(0.19585, abs=3e-9)
    assert audit["rigid_clearance_at_10mm_compression_m"] == pytest.approx(0.02165, abs=3e-9)
    assert audit["rigid_clearance_at_15mm_compression_m"] == pytest.approx(0.01665, abs=3e-9)
    bounds = verified_tool_bounds_step_mm(ROOT)
    original = np.asarray(_analysis()["solid_bounding_boxes_step_mm"])[audit["rigid_solid_indices"]]
    assert bounds.shape == (130, 6)
    assert np.all(bounds[:, :3] < original[:, :3])
    assert np.all(bounds[:, 3:] > original[:, 3:])


def test_layout_robot_exposes_all_physical_tool_solids_for_swept_path_audit():
    layout = load_workcell_layout(
        ROOT / "configs/workcells/m710id70_unloading_layout_v1.yaml"
    )
    robot = layout.robot()
    q = np.zeros(6)
    assert len(robot.tool_collision_obbs(q)) == 130
    assert len(robot.tool_compliant_collision_obbs(q)) == 72
    all_physical = robot.tool_all_physical_obbs(q)
    assert len(all_physical) == 202
    assert len({box.name for box in all_physical}) == 202


def test_unknown_structure_is_not_removed_as_a_cup():
    analysis = _analysis()
    result = classify_tool_solids(analysis)
    assert 0 in result["rigid_solid_indices"]
    analysis["deformable_cup_solid_indices"].append(0)
    with pytest.raises(ValueError, match="cannot be reconciled"):
        classify_tool_solids(analysis)


def test_cached_evidence_does_not_share_mutable_results():
    audit = audit_tool_geometry(ROOT)
    audit["rigid_solid_indices"].clear()
    audit["rigid_collision_bounding_boxes_step_mm"][0][0] = 1e9
    repeated = audit_tool_geometry(ROOT)
    assert len(repeated["rigid_solid_indices"]) == 130
    assert repeated["rigid_collision_bounding_boxes_step_mm"][0][0] < 0


def test_insert_size_or_frame_drift_cannot_pass_classification():
    analysis = _analysis()
    index = classify_tool_solids(analysis)["rigid_insert_solid_indices"][0]
    changed = copy.deepcopy(analysis)
    changed["solid_bounding_boxes_step_mm"][index][4] += 2.0
    with pytest.raises(ValueError, match="retained insert"):
        classify_tool_solids(changed)
    changed = copy.deepcopy(analysis)
    changed["rotation_step_from_tool"][0][1] = 1000.0
    with pytest.raises(ValueError, match="proper SE"):
        classify_tool_solids(changed)


def test_coverage_rejects_actual_outside_bound_and_missing_product_attribution(tmp_path):
    (tmp_path / TOOL_DIRECTORY).mkdir(parents=True)
    (tmp_path / "res").mkdir()
    shutil.copyfile(ROOT / "res/上海皖泰真空吸盘三分区.STEP", tmp_path / "res/上海皖泰真空吸盘三分区.STEP")
    shutil.copyfile(ROOT / TOOL_DIRECTORY / "mass_properties.json", tmp_path / TOOL_DIRECTORY / "mass_properties.json")
    path = tmp_path / TOOL_DIRECTORY / "geometry_coverage.json"
    source = json.loads((ROOT / TOOL_DIRECTORY / "geometry_coverage.json").read_text())
    changed = copy.deepcopy(source)
    changed["regenerated_solid_bounds_step_mm"][0][3] += 0.1
    path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match="do not contain"):
        audit_tool_geometry(tmp_path)
    changed = copy.deepcopy(source)
    changed["product_attribution_verified"] = False
    path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match="unverified"):
        audit_tool_geometry(tmp_path)
