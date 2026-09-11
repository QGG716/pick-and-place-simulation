"""Audited Wantai engineering collision geometry, independent of a CAD runtime.

The historical conversion removed both members of every FG42 occurrence.
This representation restores every insert and keeps all other solids rigid.
Only the named FG42 bellows are assigned the explicit compliant-cup model.
The source STEP and independently regenerated bounds are hash-bound; runtime
does not import OpenCascade.  This is simulation geometry, not certification.
"""

from __future__ import annotations

import hashlib
import copy
from functools import lru_cache
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np


TOOL_DIRECTORY = Path("assets/grippers/shanghai_wantai_three_zone")
COVERAGE_SCHEMA = "wantai_step_geometry_coverage_v2"
_SOURCE_SHA256 = "d54ec5818a772877162e7d8eded484c67920de148037373d9d7d8e35cfa0694b"
_ANALYSIS_SHA256 = "2596f20270848f8874cf8e766602be7985da8dd047d769889b83f690009f13f0"
# Original bounds were rounded to six decimal places in millimetres.  This
# outward pad exceeds that rounding error without changing engineering margins.
BOUND_PAD_MM = 2e-6


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def classify_tool_solids(analysis: Mapping[str, Any]) -> dict[str, Any]:
    """Reconcile all source solids and retain ambiguous parts as rigid.

    The two repeated shapes share the 72 CAD-derived cup axes.  Their known
    bounding dimensions identify the FG42 bellows and its smaller insert;
    the offline check also binds those shapes to the STEP product.  Counts
    derive from the cup-centre list; they are not task-population constants.
    """
    bounds = np.asarray(analysis["solid_bounding_boxes_step_mm"], dtype=float)
    if bounds.ndim != 2 or bounds.shape[1] != 6 or not np.all(np.isfinite(bounds)):
        raise ValueError("source solid bounds must be finite Nx6")
    if len(bounds) != int(analysis["solid_occurrence_count"]):
        raise ValueError("source solid count mismatch")
    if np.any(bounds[:, 3:] <= bounds[:, :3]):
        raise ValueError("source solid bounds must have positive extents")
    flange = np.asarray(analysis["flange_origin_step_mm"], dtype=float)
    rotation = np.asarray(analysis["rotation_step_from_tool"], dtype=float)
    if flange.shape != (3,) or rotation.shape != (3, 3) or not np.allclose(
        rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0.0
    ) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0.0):
        raise ValueError("STEP-to-tool transform must be proper SE(3)")
    cup_yz = np.asarray(analysis["cup_centers_tool_yz_m"], dtype=float)
    centers_step = np.column_stack((np.zeros(len(cup_yz)), cup_yz)) @ rotation.T * 1000.0 + flange
    cup_axes = centers_step[:, [0, 2]]
    bellows: list[int] = []
    inserts: list[int] = []
    cup_assignments: list[dict[str, int]] = []
    for cup_index, axis in enumerate(cup_axes):
        matches = []
        for index, row in enumerate(bounds):
            center_xz = 0.5 * (row[[0, 2]] + row[[3, 5]])
            if np.linalg.norm(center_xz - axis) <= 0.002 and row[1] > 0.0:
                matches.append(index)
        large = [i for i in matches if np.allclose(bounds[i, 3:] - bounds[i, :3], [43.7, 47.5, 43.7], atol=0.002, rtol=0.0)]
        small = [i for i in matches if np.allclose(bounds[i, 3:] - bounds[i, :3], [18.8, 30.85, 18.8], atol=0.002, rtol=0.0)]
        if len(large) != 1 or len(small) != 1:
            raise ValueError(f"FG42 cup {cup_index} must have one bellows and one retained insert")
        bellows.append(large[0])
        inserts.append(small[0])
        cup_assignments.append({"cup_index": cup_index, "bellows_solid_index": large[0], "rigid_insert_solid_index": small[0]})
    if len(set(bellows)) != len(cup_axes) or len(set(inserts)) != len(cup_axes):
        raise ValueError("cup solids must have unique physical ownership")
    historical_removed = set(int(i) for i in analysis["deformable_cup_solid_indices"])
    if historical_removed != set(bellows + inserts):
        raise ValueError("historical removed-solid list cannot be reconciled")
    original_indices = [i for i in range(len(bounds)) if i not in historical_removed]
    original = np.asarray(analysis["rigid_collision_bounding_boxes_step_mm"], dtype=float)
    if original.shape != (len(original_indices), 6) or not np.array_equal(original, bounds[original_indices]):
        raise ValueError("historical rigid boxes do not cover their source solids")
    # Preserve old box indices for traceability; append restored rigid inserts.
    rigid_indices = original_indices + inserts
    return {
        "all_bounds_step_mm": bounds,
        "rigid_solid_indices": rigid_indices,
        "bellows_solid_indices": bellows,
        "rigid_insert_solid_indices": inserts,
        "historical_rigid_solid_count": len(original_indices),
        "cup_assignments": cup_assignments,
    }


def _audit_tool_geometry_uncached(repository_root: str | Path) -> dict[str, Any]:
    """Verify CAD-derived coverage and return consumable simulation readiness."""
    root = Path(repository_root).resolve()
    analysis_path = root / TOOL_DIRECTORY / "mass_properties.json"
    source_path = root / "res/上海皖泰真空吸盘三分区.STEP"
    certificate_path = root / TOOL_DIRECTORY / "geometry_coverage.json"
    if _digest(source_path) != _SOURCE_SHA256 or _digest(analysis_path) != _ANALYSIS_SHA256:
        raise ValueError("Wantai geometry source/analysis SHA-256 mismatch")
    analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
    classification = classify_tool_solids(analysis)
    certificate = json.loads(certificate_path.read_text(encoding="utf-8"))
    if certificate.get("schema") != COVERAGE_SCHEMA or certificate.get("source_step_sha256") != _SOURCE_SHA256 or certificate.get("analysis_sha256") != _ANALYSIS_SHA256:
        raise ValueError("tool coverage evidence does not bind the active CAD inputs")
    if certificate.get("source_units") != "millimetre" or certificate.get("product_attribution_verified") is not True:
        raise ValueError("tool source units or FG42 product attribution are unverified")
    regenerated = np.asarray(certificate["regenerated_solid_bounds_step_mm"], dtype=float)
    stored = classification["all_bounds_step_mm"]
    if regenerated.shape != stored.shape or not np.all(np.isfinite(regenerated)):
        raise ValueError("regenerated STEP bounds shape mismatch")
    outward = stored.copy()
    outward[:, :3] -= BOUND_PAD_MM
    outward[:, 3:] += BOUND_PAD_MM
    if np.any(outward[:, :3] > regenerated[:, :3]) or np.any(outward[:, 3:] < regenerated[:, 3:]):
        raise ValueError("outward boxes do not contain regenerated STEP geometry")
    if certificate.get("bellows_solid_indices") != classification["bellows_solid_indices"] or certificate.get("rigid_insert_solid_indices") != classification["rigid_insert_solid_indices"]:
        raise ValueError("CAD product ownership differs from runtime classification")
    rigid_indices = classification["rigid_solid_indices"]
    flange = np.asarray(analysis["flange_origin_step_mm"], dtype=float)
    front_x_m = float(np.max(outward[rigid_indices, 4] - flange[1]) * 0.001)
    uncompressed = float(analysis["tool_length_flange_to_uncompressed_cup_extreme_m"])
    return {
        "schema": COVERAGE_SCHEMA,
        "source_integrity": True,
        "coverage_verified": True,
        "execution_qualified": True,
        "simulation_geometry_qualified": True,
        "machine_certification": "NOT_EVALUATED_SEPARATE_FROM_SIMULATION",
        "coverage_evidence_sha256": _digest(certificate_path),
        "source_step_sha256": _SOURCE_SHA256,
        "analysis_sha256": _ANALYSIS_SHA256,
        "solid_occurrence_count": len(stored),
        "historical_rigid_solid_count": classification["historical_rigid_solid_count"],
        "restored_rigid_insert_count": len(classification["rigid_insert_solid_indices"]),
        "rigid_solid_count": len(rigid_indices),
        "compliant_bellows_count": len(classification["bellows_solid_indices"]),
        "rigid_solid_indices": rigid_indices,
        "bellows_solid_indices": classification["bellows_solid_indices"],
        "cup_assignments": classification["cup_assignments"],
        "rigid_collision_bounding_boxes_step_mm": outward[rigid_indices].tolist(),
        "compliant_bellows_bounds_step_mm": outward[classification["bellows_solid_indices"]].tolist(),
        "flange_origin_step_mm": analysis["flange_origin_step_mm"],
        "rotation_step_from_tool": analysis["rotation_step_from_tool"],
        "rigid_front_from_flange_m": front_x_m,
        "uncompressed_plane_from_flange_m": uncompressed,
        "rigid_clearance_at_10mm_compression_m": uncompressed - 0.010 - front_x_m,
        "rigid_clearance_at_15mm_compression_m": uncompressed - 0.015 - front_x_m,
        "compliant_assumption": "FG42 rubber bellows use the bounded 0-15mm axial compression contact model; every insert and all other solids remain rigid; inactive cups remain physical",
        "representation": "SOURCE_VERIFIED_STRUCTURE_AND_INSERT_OBBS_WITH_SEPARATE_COMPLIANT_FG42_BELLOWS",
        "outward_pad_mm": BOUND_PAD_MM,
        "maximum_bound_reconstruction_error_mm": float(np.max(np.abs(stored - regenerated))),
    }


@lru_cache(maxsize=8)
def _cached_tool_geometry(root: str, identities: tuple[tuple[str, int, int], ...]) -> dict[str, Any]:
    # identities is intentionally part of the key even though the audited
    # content is read by the implementation.  Any source or evidence rewrite
    # invalidates the result before another robot snapshot can consume it.
    return _audit_tool_geometry_uncached(root)


def audit_tool_geometry(repository_root: str | Path) -> dict[str, Any]:
    """Return isolated audit evidence, cached only while input file stats match."""
    root = Path(repository_root).resolve()
    paths = (
        root / "res/上海皖泰真空吸盘三分区.STEP",
        root / TOOL_DIRECTORY / "mass_properties.json",
        root / TOOL_DIRECTORY / "geometry_coverage.json",
    )
    identities = tuple((str(path.resolve()), path.stat().st_size, path.stat().st_mtime_ns) for path in paths)
    return copy.deepcopy(_cached_tool_geometry(str(root), identities))


def verified_tool_bounds_step_mm(repository_root: str | Path) -> np.ndarray:
    """Return every audited rigid structure and insert box in source mm."""
    return np.asarray(audit_tool_geometry(repository_root)["rigid_collision_bounding_boxes_step_mm"], dtype=float)
