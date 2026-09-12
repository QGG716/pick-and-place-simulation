"""Thin, auditable adapter for the pinned upstream observed-face V4 worker.

The upstream implementation deliberately remains in the separately pinned
vision checkout.  This module only translates evidence names, constructs the
exact subprocess command, and validates/annotates the result.  It does not
copy the OpenCV/SciPy algorithm into the lightweight core package.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


UPSTREAM_V4_SHA = "1d208f2ed380a207e6e46b4a62d2ac640edfe477"
UPSTREAM_V4_SCRIPT = "pipeline/geometry/refine_multiplane_cuboids.py"

REGISTERED_DEPTH_FACE = "registered_metric_depth_plane"
UPSTREAM_SELECTOR_FACE = "monocular_depth_plane"
REGISTERED_JOINT_FACE = "joint_sam_registered_depth_multiplane_pnp"
UPSTREAM_JOINT_FACE = "joint_sam_depth_multiplane_pnp"


def prepare_registered_depth_baseline(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Translate direct metric-depth faces for the pinned worker selector.

    V4 at the pinned SHA selects faces by the historical literal
    ``monocular_depth_plane``.  The compatibility translation is explicit on
    every face and is reversed after the worker returns, so metric RGB-D is
    never published with monocular provenance.
    """

    prepared = deepcopy(dict(payload))
    for record in prepared.get("instances", ()):  # type: ignore[union-attr]
        for face in record.get("camera_facing_faces", ()):
            evidence = face.get("evidence")
            if evidence in {REGISTERED_DEPTH_FACE, UPSTREAM_SELECTOR_FACE}:
                face["adapter_source_evidence"] = REGISTERED_DEPTH_FACE
                face["evidence"] = UPSTREAM_SELECTOR_FACE
    prepared["v4_adapter_input"] = {
        "schema_version": "registered_rgbd_upstream_v4_adapter_v1",
        "upstream_sha": UPSTREAM_V4_SHA,
        "selector_compatibility_label": UPSTREAM_SELECTOR_FACE,
        "physical_source_evidence": REGISTERED_DEPTH_FACE,
    }
    return prepared


def finalize_registered_depth_result(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Restore truthful RGB-D evidence names and attach algorithm provenance."""

    result = deepcopy(dict(payload))
    observed_count = 0
    refined_count = 0
    suppressed_count = 0
    for record in result.get("instances", ()):  # type: ignore[union-attr]
        corner_key = "corners_3d"
        plane_pair = record.get("orthogonal_plane_pair_diagnostics", {})
        if (
            int(record.get("depth_supported_face_count", record.get("visible_plane_count", 0))) >= 2
            and plane_pair.get("reliable") is True
            and len(record.get("unanchored_corners_3d", ())) == 8
        ):
            # Keep this selection identical to hypotheses_from_geometry_record.
            # V4's 2D silhouette anchor may publish a projective eight-corner
            # fit that is not an orthogonal cuboid.  Once at least two
            # orthogonal depth planes are reliable, the unanchored metric
            # result is the coherent cuboid datum.
            corner_key = "unanchored_corners_3d"
        corners = np.asarray(record.get(corner_key, ()), dtype=float)
        if corners.shape == (8, 3):
            vectors = np.asarray((corners[1] - corners[0], corners[3] - corners[0], corners[4] - corners[0]))
            dimensions = np.linalg.norm(vectors, axis=1)
            if np.all(np.isfinite(dimensions)) and float(np.min(dimensions)) > 1e-9:
                axes = vectors / dimensions[:, None]
                if float(np.linalg.det(axes)) < 0.0:
                    axes[2] *= -1.0
                record["orthogonal_axes_3d_before_adapter_coherence"] = record.get("orthogonal_axes_3d")
                record["shape_dimensions_before_adapter_coherence"] = record.get("shape_dimensions")
                record["orthogonal_axes_3d"] = np.round(axes, 8).tolist()
                record["shape_dimensions"] = np.round(dimensions, 8).tolist()
                record["geometry_coherence"] = f"DERIVED_FROM_{corner_key.upper()}_0_1_3_4"
                record["geometry_coherence_corner_source"] = corner_key
        if record.get("multiplane_refinement_status") == (
            "joint_sam_silhouette+depth_faces+fixed_intrinsics_pnp"
        ):
            refined_count += 1
        published_faces = []
        for face in record.get("camera_facing_faces", ()):
            evidence = face.get("evidence")
            if evidence == UPSTREAM_JOINT_FACE:
                face["evidence"] = REGISTERED_JOINT_FACE
                face["depth_source"] = "REGISTERED_METRIC_DEPTH"
            elif evidence == UPSTREAM_SELECTOR_FACE or face.get("adapter_source_evidence") == REGISTERED_DEPTH_FACE:
                face["evidence"] = REGISTERED_DEPTH_FACE
                face["depth_source"] = "REGISTERED_METRIC_DEPTH"
            if face.get("evidence") in {REGISTERED_DEPTH_FACE, REGISTERED_JOINT_FACE, "registered_sam_visible_face"}:
                observed_count += 1
                published_faces.append(face)
            else:
                suppressed_count += 1
        if "camera_facing_faces" in record:
            record["camera_facing_faces"] = published_faces
            record["projected_camera_facing_face_count"] = len(published_faces)
    result["pointmap_source"] = "REGISTERED_METRIC_DEPTH"
    result["v4_adapter"] = {
        "schema_version": "registered_rgbd_upstream_v4_adapter_v1",
        "upstream_sha": UPSTREAM_V4_SHA,
        "upstream_script": UPSTREAM_V4_SCRIPT,
        "algorithm_reused": True,
        "intrinsics_policy": "FIXED_CAPTURE_K",
        "metric_depth_policy": "DO_NOT_SCALE_OR_MOVE_OBSERVED_PLANES_TO_IMPROVE_2D_IOU",
        "hidden_faces": "NOT_OBSERVED",
        "refined_instance_count": refined_count,
        "published_observed_face_count": observed_count,
        "suppressed_completion_face_count": suppressed_count,
    }
    return result


def build_upstream_v4_command(
    python: str | Path,
    vision_root: str | Path,
    *,
    source: str | Path,
    masks: str | Path,
    pointmap: str | Path,
    baseline: str | Path,
    fallback: str | Path,
    json_output: str | Path,
    image_output: str | Path,
    min_face_precision: float = 0.72,
    min_face_coverage: float = 0.04,
    max_pnp_rmse: float = 6.0,
) -> tuple[str, ...]:
    """Return the deterministic command for the pinned V4 implementation."""

    return (
        str(python), str(Path(vision_root) / UPSTREAM_V4_SCRIPT),
        str(source), str(masks), str(pointmap), str(baseline),
        "--fallback", str(fallback), "--suppress-hidden-fallback",
        "--json-output", str(json_output), "--output", str(image_output),
        "--min-face-precision", f"{float(min_face_precision):g}",
        "--min-face-coverage", f"{float(min_face_coverage):g}",
        "--max-pnp-rmse", f"{float(max_pnp_rmse):g}",
    )


def validate_upstream_v4_command(command: Sequence[str], vision_root: str | Path) -> None:
    """Fail closed if the requested worker is not the pinned V4 entry point."""

    if len(command) < 2 or Path(command[1]).resolve() != (Path(vision_root) / UPSTREAM_V4_SCRIPT).resolve():
        raise ValueError("command does not invoke the pinned upstream V4 entry point")
    required = {"--fallback", "--suppress-hidden-fallback", "--json-output", "--output"}
    if not required.issubset(command):
        raise ValueError("upstream V4 command omits required observed-face safeguards")
