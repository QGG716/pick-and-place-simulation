"""Run Mode B registered-RGB-D geometry and compare it with Mode C MoGe.

SAM masks and 2D faces come from the pinned upstream worker.  This adapter
replaces only the point-map source, reusing the pinned plane/orthogonal-cuboid
recovery with explicit registered-depth provenance.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from time import perf_counter

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import (  # noqa: E402
    SCHEMA_VERSION, CargoObservation, EvidenceKind, ObservationStatus,
    PerceptionObservation, ResourceReference, UnknownRegion, Validity, dumps,
)
from unloading_perception.isaac_evaluation import evaluate_observations, write_evaluation  # noqa: E402
from unloading_perception.isaac_validation import IsaacSceneManifest, ground_truth_observation, write_json  # noqa: E402
from unloading_perception.observed_faces import (  # noqa: E402
    observed_faces_from_geometry_record, transform_observed_face_set,
)
from unloading_perception.fusion import ModuleFaceBatch, fuse_module_face_batches  # noqa: E402
from unloading_perception.lineage import load_instance_lineage  # noqa: E402
from unloading_perception.isaac_payload import CapturePayloadError, require_capture_payload, write_capture_snapshot
from unloading_perception.final_geometry import validate_final_record  # noqa: E402
from unloading_perception.rgbd import (  # noqa: E402
    CaptureMetadata, MetricPointMap, MetricPointMapSource, PointCloudFilterConfig,
    hypotheses_from_geometry_record, masked_metric_pointmap_with_filter, register_rgbd,
    transform_hypothesis_to_world,
)
from unloading_perception.upstream_v4 import (  # noqa: E402
    build_upstream_v4_command, finalize_registered_depth_result,
    prepare_registered_depth_baseline, validate_upstream_v4_command,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _scalar_mean(value) -> float | None:
    if value is None:
        return None
    array = np.asarray(value, dtype=float)
    return None if array.size == 0 else float(np.mean(array))


def _worker_artifacts(scene_dir: Path, *, payload=None) -> dict:
    if payload is not None:
        payload = require_capture_payload(scene_dir, payload.manifest, payload=payload)
    response = json.loads((scene_dir / "mode_b1_worker_response.json").read_text(encoding="utf-8").strip().splitlines()[-1])
    if response.get("status") != "COMPLETE":
        raise RuntimeError(f"SAM/2D worker response is incomplete for {scene_dir.name}")
    metrics_path = Path(response["metrics_reference"]["path"])
    if sha256(metrics_path) != response["metrics_reference"]["sha256"]:
        raise ValueError("worker metrics digest mismatch")
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    if response["input_sha256"] != sha256(scene_dir / "sensor_rgb.png"):
        raise ValueError("cached SAM image hash mismatch")
    if payload is not None and response['input_sha256'] != payload.binding.rgb_sha256:
        raise ValueError('cached SAM image differs from original capture binding')
    if metrics["config"]["proposal_sha256"] != sha256(scene_dir / "oracle_proposals.json"):
        raise ValueError("cached SAM proposal hash mismatch")
    from unloading_perception.upstream_v4 import UPSTREAM_V4_SHA
    config = metrics["config"]
    if (metrics.get("upstream_commit", config.get("upstream_sha")) != UPSTREAM_V4_SHA
            or config.get("sam_revision") != "70c1a07f894ebb5b307fd9eaaee97b9dfc16068f"):
        raise ValueError("cached SAM model/upstream identity mismatch")
    artifacts = metrics["artifacts"]
    for name in ("cargo_masks.npz", "cargo_instances.json", "box_geometry_2d.json"):
        item = artifacts[name]
        if sha256(Path(item["path"])) != item["sha256"]:
            raise ValueError(f"worker artifact digest mismatch: {name}")
    metadata = payload.metadata if payload is not None else CaptureMetadata.from_dict(
        json.loads((scene_dir / 'capture_metadata.json').read_text(encoding='utf-8')))
    proposals = json.loads((scene_dir / 'oracle_proposals.json').read_text(encoding='utf-8'))
    if payload is not None:
        from run_metric_small_matrix import oracle_proposal_document
        if proposals != oracle_proposal_document(payload):
            raise ValueError('oracle proposals differ from verified capture annotations')
    load_instance_lineage(
        Path(artifacts["cargo_masks.npz"]["path"]), Path(artifacts["cargo_instances.json"]["path"]),
        proposals, {"instances": []},
        sensor_epoch=metadata.sensor_epoch, module_id=metadata.rgb_frame_id.removesuffix("_rgb_optical"),
        capture_id=metadata.capture_id, source_path=scene_dir / "sensor_rgb.png", proposal_path=scene_dir / "oracle_proposals.json",
    )
    return artifacts


def _camera_for_frame(manifest: IsaacSceneManifest, frame_id: str) -> dict:
    matches = [camera for camera in manifest.cameras if str(camera["frame_id"]) == frame_id]
    if len(matches) != 1:
        raise ValueError(f"capture frame is not bound to exactly one manifest camera: {frame_id}")
    return matches[0]


def _build_pointmap(scene_dir: Path, manifest: IsaacSceneManifest, masks_path: Path, config: dict, *, payload=None) -> tuple[MetricPointMap, list[dict]]:
    payload = require_capture_payload(scene_dir, manifest, payload=payload)
    metadata, camera, rgb, depth = payload.metadata, payload.camera, payload.rgb, payload.depth
    depth_identity = payload.binding.extensions['metric_depth_sha256']
    frame = register_rgbd(
        metadata=metadata, rgb=rgb, depth_optical_z_m=depth,
        rgb_K=camera["K"], depth_K=camera["K"], T_rgb_depth=camera["T_rgb_depth"],
        rgb_calibration_identity=metadata.calibration_identity,
        depth_calibration_identity=metadata.calibration_identity,
        registration_mode=camera["registration_mode"],
    )
    with np.load(masks_path, allow_pickle=False) as archive:
        masks = archive["masks"].astype(bool)
        labels = archive["labels"].astype(str)
        ids = archive["mask_ids"].astype(int)
    filter_config = PointCloudFilterConfig(
        boundary_erosion_px=int(config["boundary_erosion_px"]),
        depth_percentile_low=float(config["depth_percentile_low"]),
        depth_percentile_high=float(config["depth_percentile_high"]),
        discontinuity_mad_scale=float(config["discontinuity_mad_scale"]),
        discontinuity_floor_m=float(config["discontinuity_floor_m"]),
        local_depth_component_selection=bool(config["local_depth_component_selection"]),
        retain_multiple_depth_components=bool(config["retain_multiple_depth_components"]),
        minimum_component_points=int(config["minimum_component_points"]),
        minimum_points=int(config["minimum_points"]),
    )
    combined_points = np.full((*depth.shape, 3), np.nan, dtype=np.float32)
    combined_valid = np.zeros(depth.shape, dtype=bool)
    audits = []
    for mask_id, label, mask in zip(ids, labels, masks):
        if label not in {"box", "cardboard_box"}:
            continue
        try:
            item, filter_result = masked_metric_pointmap_with_filter(
                frame, mask, depth_identity=depth_identity,
                source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH, config=filter_config,
            )
            combined_points[item.valid_mask] = item.points_camera_xyz_m[item.valid_mask]
            combined_valid |= item.valid_mask
            audit_path = scene_dir / "pointcloud_filter" / f"mask_{int(mask_id):04d}.npz"
            audit_path.parent.mkdir(parents=True, exist_ok=True)
            rows, columns = np.indices(depth.shape, dtype=np.float32)
            raw_points = np.full((*depth.shape, 3), np.nan, dtype=np.float32)
            raw = filter_result.raw_valid_mask
            raw_points[raw, 0] = (columns[raw] - camera["K"][2]) * depth[raw] / camera["K"][0]
            raw_points[raw, 1] = (rows[raw] - camera["K"][5]) * depth[raw] / camera["K"][4]
            raw_points[raw, 2] = depth[raw]
            np.savez_compressed(
                audit_path, raw_valid_mask=filter_result.raw_valid_mask,
                filtered_mask=filter_result.retained_mask,
                rejected_reason=filter_result.rejected_reason,
                raw_points_camera_xyz_m=raw_points,
                filtered_points_camera_xyz_m=item.points_camera_xyz_m,
            )
            audits.append({
                "mask_id": int(mask_id), "status": "PASS",
                "audit_npz": str(audit_path), **dict(item.filter_evidence),
            })
        except ValueError as exc:
            audits.append({"mask_id": int(mask_id), "status": "REJECTED", "reason": str(exc)})
    if not np.any(combined_valid):
        raise RuntimeError(f"no SAM instance retained registered depth for {scene_dir.name}")
    pointmap = MetricPointMap(
        "metric_point_map_v1", MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,
        combined_points, depth, combined_valid, tuple(camera["K"]), str(camera["frame_id"]),
        (depth.shape[1], depth.shape[0]), metadata.capture_id, metadata.capture_center_time,
        metadata.calibration_identity, depth_identity,
        {
            "per_instance_filtering": audits,
            "metric_scale_validity": "VALID",
            "depth_semantics": "optical_z_m",
            "gt_geometry_used_for_filtering": False,
        },
    )
    return pointmap, audits


def _hypothesis_dict(hypothesis) -> dict:
    value = asdict(hypothesis)
    for pose_name in ("pose_camera", "pose_world"):
        pose = value.get(pose_name)
        if pose is not None:
            pose["evidence"] = pose["evidence"].value
    return value


def _observation(
    scene: str,
    manifest: IsaacSceneManifest,
    geometry_payload: dict,
    masks_path: Path,
    proposals: dict,
    elapsed: float,
    *, validate_geometry: bool = True, payload=None, lineage_directory=None,
) -> tuple[PerceptionObservation, dict[str, np.ndarray], dict[str, tuple], tuple]:
    scene_dir = Path(proposals['_scene_dir'])
    payload = require_capture_payload(scene_dir, manifest, payload=payload)
    metadata, camera = payload.metadata, payload.camera
    module_id = str(camera.get("module_id", metadata.rgb_frame_id.removesuffix("_rgb_optical")))
    if not np.allclose(metadata.T_W_C_at_capture, camera["T_W_C"], atol=1e-9, rtol=0):
        raise ValueError("CAPTURE_TF_DIFFERS_FROM_BOUND_MANIFEST")
    lineage_directory = scene_dir if lineage_directory is None else Path(lineage_directory)
    lineage = load_instance_lineage(
        masks_path, masks_path.parent / "cargo_instances.json", proposals, geometry_payload,
        sensor_epoch=metadata.sensor_epoch, module_id=module_id, capture_id=metadata.capture_id,
        source_path=lineage_directory / "sensor_rgb.png", proposal_path=lineage_directory / "oracle_proposals.json",
    )
    masks_by_id = {item.identity: item.mask for item in lineage.values()}
    records = {int(item["mask_id"]): item for item in geometry_payload["instances"]}
    cargo = []
    unknown = []
    hypothesis_groups = {}
    observed_face_sets = []
    depth = payload.depth if validate_geometry else None
    for mask_id, item in sorted(lineage.items()):
        record = records.get(mask_id, {"mask_id": mask_id, "accepted": False})
        source_id = item.identity
        proposal = item.proposal
        bbox = tuple(float(value) for value in proposal["bbox"])
        if validate_geometry:
            record = validate_final_record(record, depth, item.mask, camera["K"])
        observed_camera = observed_faces_from_geometry_record(
            record, source_instance_id=source_id, module_id=module_id,
            capture_id=metadata.capture_id, capture_time=metadata.capture_center_time,
            frame_id=metadata.rgb_frame_id,
        )
        observed_world = transform_observed_face_set(observed_camera, metadata.T_W_C_at_capture, "world")
        observed_face_sets.append(observed_world)
        adaptation_error = None
        try:
            hypotheses = hypotheses_from_geometry_record(
                record, source_instance_id=source_id,
                pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,
                presence_score=float(item.sam["validation_score"]),
                camera_frame=metadata.rgb_frame_id,
            )
        except ValueError as exc:
            # Keep certified observed faces, but fail this complete-cuboid
            # candidate closed.  A single malformed upstream record must not
            # erase valid evidence from the rest of the batch.
            hypotheses = ()
            adaptation_error = str(exc)
        world_hypotheses = tuple(transform_hypothesis_to_world(item, metadata) for item in hypotheses)
        hypothesis_groups[source_id] = world_hypotheses
        if world_hypotheses:
            selected = world_hypotheses[0]
            reasons = tuple(selected.eligibility_reasons)
            cargo.append(CargoObservation(
                source_instance_id=source_id,
                object_id=None,
                track_id=None,
                category=str(proposal.get("label", "box")),
                bbox_xyxy=bbox,
                mask_reference=ResourceReference(masks_path.resolve().as_uri() + f"#mask_id={mask_id}", sha256(masks_path), "application/x-npz; array=bool"),
                detection_score=None,
                contour_score=None,
                reprojection_score=float(record.get("mask_reprojection_iou", 0.0)),
                geometry_error_m=selected.plane_residual_m,
                pose=selected.pose_world,
                full_dimensions_m=selected.full_dimensions_xyz_m,
                corners_3d_m=None,
                axes_3d_rows=None,
                pose_evidence=EvidenceKind.MODEL_ESTIMATED,
                size_evidence=EvidenceKind.CONSTRAINT_COMPLETED,
                depth_evidence=EvidenceKind.OBSERVED,
                scale_evidence=EvidenceKind.OBSERVED,
                metric_scale_validity=Validity.VALID,
                geometry_validity=Validity.VALID,
                candidate_eligible=selected.candidate_eligible,
                eligibility_reasons=reasons,
                raw_result={
                    "instance_lineage": item.audit,
                    "final_face_validation": record.get("final_face_validation", []),
                    "pointmap_source": "ISAAC_IDEAL_REGISTERED_DEPTH",
                    "hypothesis_count": len(world_hypotheses),
                    "hypotheses": [_hypothesis_dict(item) for item in world_hypotheses],
                    "observed_face_set": observed_world.to_dict(),
                    "algorithm": "PINNED_UPSTREAM_V4_OBSERVED_FACE_RECOVERY",
                },
            ))
        else:
            if adaptation_error is not None:
                reasons = ("INVALID_UPSTREAM_COMPLETE_CUBOID",)
                geometry_validity = Validity.INVALID
            elif observed_world.complete_cuboid_status == "INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR":
                reasons = ("COMPLETE_CUBOID_UNOBSERVABLE_SINGLE_FACE",)
                geometry_validity = Validity.UNKNOWN
            elif observed_world.complete_cuboid_status == "SUFFICIENT_MULTIFACE_EVIDENCE":
                reasons = ("COMPLETE_CUBOID_SELF_CONSISTENCY_REJECTED",)
                geometry_validity = Validity.INVALID
            else:
                reasons = ("NO_CERTIFIED_OBSERVED_FACE",)
                geometry_validity = Validity.INVALID
            cargo.append(CargoObservation(
                source_instance_id=source_id,
                object_id=None,
                track_id=None,
                category=str(proposal.get("label", "box")),
                bbox_xyxy=bbox,
                mask_reference=ResourceReference(masks_path.resolve().as_uri() + f"#mask_id={mask_id}", sha256(masks_path), "application/x-npz; array=bool"),
                detection_score=None,
                contour_score=None,
                reprojection_score=None,
                geometry_error_m=None,
                pose=None,
                full_dimensions_m=None,
                corners_3d_m=None,
                axes_3d_rows=None,
                pose_evidence=None,
                size_evidence=None,
                depth_evidence=EvidenceKind.OBSERVED,
                scale_evidence=EvidenceKind.OBSERVED,
                metric_scale_validity=Validity.VALID,
                geometry_validity=geometry_validity,
                candidate_eligible=False,
                eligibility_reasons=reasons,
                raw_result={
                    "instance_lineage": item.audit,
                    "pointmap_source": "ISAAC_IDEAL_REGISTERED_DEPTH", "record": record,
                    "observed_face_set": observed_world.to_dict(),
                    "complete_cuboid_status": observed_world.complete_cuboid_status,
                    "complete_cuboid_adaptation_error": adaptation_error,
                },
            ))
            unknown.append(UnknownRegion(f"rgbd-{source_id}", metadata.rgb_frame_id, reasons[0], bbox))
        surfaces = tuple({
            **face.to_dict(), "schema_version": "observed_surface_v1",
            "module_id": module_id, "capture_id": metadata.capture_id,
            "source_instance_id": source_id, "sensor_epoch": metadata.sensor_epoch,
            "capture_time": metadata.capture_center_time, "clock_domain": metadata.clock_domain,
            "calibration_identity": metadata.calibration_identity,
            "T_W_C_at_capture": metadata.T_W_C_at_capture,
            "volume_status": "UNKNOWN", "source_kind": "ALGORITHM_FROM_ISAAC_RENDERED_RGBD",
            "boundary_kind": "OBSERVED_PATCH_UNCLASSIFIED",
        } for face in observed_world.faces)
        cargo[-1] = replace(cargo[-1], observed_surfaces=surfaces)
    observation = PerceptionObservation(
        SCHEMA_VERSION, f"rgbd-{scene}-{metadata.capture_id}", metadata.sensor_epoch,
        metadata.frame_sequence, metadata.capture_center_time, metadata.capture_center_time + elapsed,
        metadata.clock_domain, "staged-registered-rgbd-geometry", "1d208f2ed380a207e6e46b4a62d2ac640edfe477",
        "sam+pinned-plane-cuboid+registered-depth", metadata.calibration_identity,
        ObservationStatus.COMPLETE if not unknown else ObservationStatus.PARTIAL,
        None, None, tuple(cargo), tuple(unknown),
        {
            "mode": "STAGED_RGBD", "role": "PRIMARY", "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
            "raw_image_automatic": False,
            "detector_metrics": "NOT_EVALUATED", "pointmap_source": "ISAAC_IDEAL_REGISTERED_DEPTH",
            "capture_id": metadata.capture_id, "absence_means_free_space": False,
            "input_provenance": payload.input_provenance(),
            "module_binding": {"module_id": module_id, "capture_id": metadata.capture_id,
                               "calibration_identity": metadata.calibration_identity,
                               "T_W_C_at_capture": metadata.T_W_C_at_capture},
        },
        False,
    )
    return observation, masks_by_id, hypothesis_groups, tuple(observed_face_sets)


def _top_view(path: Path, observation: PerceptionObservation) -> None:
    import cv2
    image = np.full((900, 1100, 3), 245, dtype=np.uint8)
    x_min, x_max, y_min, y_max = -2.5, 1.0, -1.5, 1.5
    def pixel(x, y):
        return int((x - x_min) / (x_max - x_min) * 1000 + 50), int((y_max - y) / (y_max - y_min) * 800 + 50)
    for item in observation.cargo:
        if item.pose is None or item.full_dimensions_m is None:
            continue
        corners = np.asarray(item.raw_result.get("hypotheses", [{}])[0].get("evidence", {}).get("declared_corners_world_m", ()), dtype=float)
        if corners.shape != (8, 3):
            continue
        hull = cv2.convexHull(np.asarray([pixel(float(point[0]), float(point[1])) for point in corners], dtype=np.int32))
        cv2.polylines(image, [hull], True, (20, 130, 230), 2, cv2.LINE_AA)
    cv2.putText(image, "MODE B REGISTERED RGB-D CUBOIDS | WORLD TOP VIEW", (36, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.arrowedLine(image, pixel(-2.2, -1.2), pixel(-1.4, -1.2), (0, 0, 220), 3)
    cv2.putText(image, "+X trailer", pixel(-1.9, -1.27), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 220), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), image)


def _run_secondary_module(
    *, scene: str, module_dir: Path, manifest: IsaacSceneManifest, artifacts: dict,
    config: dict, vision_root: Path, upstream_python: Path, timeout: float,
    payload=None, output_directory=None,
) -> dict:
    module_started = perf_counter()
    expected_module = module_dir.name if module_dir.name in {c['module_id'] for c in manifest.cameras} else manifest.cameras[0]['module_id']
    payload = require_capture_payload(module_dir, manifest, payload=payload,
        expected_module_id=expected_module, with_instance_masks=True)
    lineage_directory = module_dir
    proposals = json.loads((module_dir / 'oracle_proposals.json').read_text(encoding='utf-8'))
    from run_metric_small_matrix import oracle_proposal_document
    if proposals != oracle_proposal_document(payload):
        raise ValueError('oracle proposals differ from verified capture annotations')
    if payload.directory == payload.source_directory:
        payload = write_capture_snapshot(payload, output_directory or module_dir/'geometry-run')
        module_dir = payload.directory
        (module_dir/'oracle_proposals.json').write_text(json.dumps(proposals, indent=2), encoding='utf-8')
    masks_path = Path(artifacts["cargo_masks.npz"]["path"])
    pointmap_started = perf_counter()
    pointmap, audits = _build_pointmap(module_dir, manifest, masks_path, config["vision"]["pointcloud_filter"], payload=payload)
    pointmap_build_seconds = perf_counter() - pointmap_started
    pointmap_path = module_dir / "registered_metric_pointmap.npz"
    pointmap_write_started = perf_counter()
    pointmap.write_npz(pointmap_path)
    pointmap_write_seconds = perf_counter() - pointmap_write_started
    raw_json = module_dir / "rgbd_cuboids_baseline_raw.json"
    base_image = module_dir / "rgbd_cuboids_baseline.png"
    base_command = [
        str(upstream_python), str(vision_root / "pipeline/geometry/recover_box_cuboids_3d.py"),
        str(module_dir / "sensor_rgb.png"), str(masks_path), str(pointmap_path),
        "--faces-json", str(Path(artifacts["box_geometry_2d.json"]["path"])),
        "--relative-threshold", "0.003", "--seed", "17",
        "--json-output", str(raw_json), "--output", str(base_image),
    ]
    write_json(module_dir/'geometry_input_provenance.json', {
        'capture': payload.input_provenance(), 'external_geometry_command': base_command,
        'derived_pointmap': {'path': str(pointmap_path), 'sha256': sha256(pointmap_path),
            'parent_depth_sha256': payload.binding.extensions['metric_depth_sha256'],
            'parent_rgb_sha256': payload.binding.rgb_sha256,
            'gt_geometry_used_for_filtering': False}})
    started = perf_counter()
    completed = subprocess.run(base_command, cwd=vision_root, text=True, capture_output=True, timeout=timeout)
    if completed.returncode != 0:
        raise RuntimeError(f"registered RGB-D baseline failed for {scene}/{module_dir.name}: {completed.stderr[-1000:]}")
    from metric_depth_runner import run_metric_depth as run_metric_v4
    metadata, camera = payload.metadata, payload.camera
    geometry_json = module_dir / "rgbd_cuboids.json"
    geometry_image = module_dir / "v4_validation/final_metric_faces_overlay.png"
    geometry = run_metric_v4(
        raw=json.loads(raw_json.read_text(encoding='utf-8')), source=module_dir / "sensor_rgb.png",
        masks=masks_path, pointmap=pointmap_path,
        depth=payload.depth, rgb=payload.rgb,
        K=camera["K"], metadata=metadata, output=module_dir / "v4_validation",
        vision_root=vision_root, python=upstream_python, timeout=timeout,
    )
    elapsed = perf_counter() - started
    geometry["method"] = "registered_metric_depth+pinned_plane_extraction+depth_constrained_metric_faces"
    write_json(geometry_json, geometry)
    proposals["_scene_dir"] = str(module_dir)
    observation, predicted_masks, hypotheses, observed_sets = _observation(
        scene, manifest, geometry, masks_path, proposals, elapsed, payload=payload, lineage_directory=lineage_directory,
    )
    truth = ground_truth_observation(manifest, payload.annotations["objects"], camera_frame_id=metadata.rgb_frame_id)
    report = evaluate_observations(
        truth, observation, ground_truth_masks=payload.instance_masks,
        prediction_masks=predicted_masks, T_W_C=camera["T_W_C"],
    )
    report["mode"] = "STAGED_RGBD"
    report["module_id"] = camera['module_id']
    report["world_transform_evaluation"]["metric_scale_valid"] = True
    report["world_transform_evaluation"]["prediction_camera_frame"] = metadata.rgb_frame_id
    write_evaluation(module_dir / "mode_b_rgbd_evaluation.json", report)
    (module_dir / "mode_b_rgbd_observation.json").write_text(dumps(observation), encoding="utf-8")
    write_json(module_dir / "metric_pointmap_filter_audit.json", audits)
    # Keep elapsed_seconds as the historical geometry-only interval above.
    # The additive total includes input validation, point maps, audits and evaluation,
    # ending immediately before this small timing record is written.
    timing = {
        "pointmap_build_including_filter_audits": pointmap_build_seconds,
        "pointmap_write": pointmap_write_seconds,
        "legacy_geometry": elapsed,
        "module_total_before_timing_record": perf_counter() - module_started,
    }
    write_json(module_dir / "rgbd_stage_timing.json", timing)
    return {
        "module_id": camera['module_id'], "elapsed_seconds": elapsed, "observation": observation,
        "timing_seconds": timing,
        "observed_face_sets": observed_sets, "hypotheses": hypotheses, "report": report,
        "geometry_image": geometry_image, "predicted_masks": predicted_masks,
        "metadata": metadata, "payload": payload, "module_directory": module_dir,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-directory", required=True, type=Path)
    parser.add_argument("--bundle-directory", required=True, type=Path)
    parser.add_argument("--vision-root", required=True, type=Path)
    parser.add_argument("--upstream-python", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=1200.0)
    parser.add_argument("--comparison", action="store_true")
    parser.add_argument("--output-directory", type=Path, help="new isolated run directory; never overwrite captures")
    args = parser.parse_args()
    source_capture_root = args.capture_directory.resolve()
    capture_root = (args.output_directory or source_capture_root / "rgbd-geometry-run").resolve()
    capture_root.mkdir(parents=True, exist_ok=False)
    bundle_root = args.bundle_directory.resolve()
    vision_root = args.vision_root.resolve()
    index = json.loads((bundle_root / "index.json").read_text(encoding="utf-8"))
    config = json.loads(json.dumps(__import__("yaml").safe_load((ROOT / "configs/isaac/perception_validation.yaml").read_text(encoding="utf-8"))))
    mode_c = json.loads((source_capture_root / "mode_c_moge_summary.json").read_text(encoding="utf-8")) if args.comparison else {"scenes": []}
    mode_c_by_scene = {item["scene"]: item for item in mode_c["scenes"]}
    results = []
    ordered_records = sorted(
        index["scenes"],
        key=lambda item: (item["scene"] != "RGBD_CALIBRATION_BOX", index["scenes"].index(item)),
    )
    calibration_passed = False
    for record in ordered_records:
        scene = record["scene"]
        source_dir = source_capture_root / scene
        manifest = IsaacSceneManifest.from_dict(json.loads((bundle_root / record["path"]).read_text(encoding="utf-8")))
        primary_payload = require_capture_payload(source_dir, manifest,
            expected_module_id=manifest.cameras[0]['module_id'], with_instance_masks=True)
        artifacts = _worker_artifacts(source_dir, payload=primary_payload)
        primary = _run_secondary_module(scene=scene, module_dir=source_dir, manifest=manifest,
            artifacts=artifacts, config=config, vision_root=vision_root, upstream_python=args.upstream_python,
            timeout=args.timeout, payload=primary_payload, output_directory=capture_root/scene)
        scene_dir, primary_payload = primary['module_directory'], primary['payload']
        primary_metadata, primary_camera = primary_payload.metadata, primary_payload.camera
        report, observation = primary['report'], primary['observation']
        predicted_masks, hypothesis_groups = primary['predicted_masks'], primary['hypotheses']
        upper_observed_sets, elapsed = primary['observed_face_sets'], primary['elapsed_seconds']
        geometry_image = primary['geometry_image']
        report['world_transform_evaluation']['claim_boundary'] = (
            'registered metric depth is evaluated in world using capture-time T_W_C; '
            'this does not establish execution qualification')
        write_evaluation(scene_dir / 'mode_b_rgbd_evaluation.json', report)
        secondary_results = []
        for module_camera in (
            camera for camera in manifest.cameras
            if str(camera["frame_id"]) != str(primary_camera["frame_id"])
        ):
            module_dir = source_dir / "modules" / str(module_camera["module_id"])
            module_payload = require_capture_payload(module_dir, manifest,
                expected_module_id=module_camera['module_id'], with_instance_masks=True)
            secondary_results.append(_run_secondary_module(
                scene=scene, module_dir=module_dir, manifest=manifest,
                artifacts=_worker_artifacts(module_dir, payload=module_payload), config=config, vision_root=vision_root,
                upstream_python=args.upstream_python, timeout=args.timeout, payload=module_payload,
                output_directory=scene_dir/"modules"/module_camera["module_id"],
            ))
        upper_metadata = primary_metadata
        batches = [ModuleFaceBatch(
            str(primary_camera["module_id"]), upper_metadata.sensor_epoch,
            upper_metadata.frame_sequence, upper_metadata.capture_center_time,
            tuple(item for item in upper_observed_sets if item.faces),
        )]
        for item in secondary_results:
            metadata = item["metadata"]
            batches.append(ModuleFaceBatch(
                item["module_id"], metadata.sensor_epoch, metadata.frame_sequence,
                metadata.capture_center_time, tuple(face_set for face_set in item["observed_face_sets"] if face_set.faces),
            ))
        fusion = fuse_module_face_batches(
            batches, expected_modules=tuple(str(camera["module_id"]) for camera in manifest.cameras),
        )
        write_json(scene_dir / "fused_observed_faces.json", asdict(fusion))
        fusion_metrics = {
            "fused_object_count": len(fusion.objects),
            "overlap_group_count": sum(len(item.source_members) > 1 for item in fusion.objects),
            "conflict_group_count": sum(item.association_status == "CONFLICT_RETAINED_NO_AVERAGE" for item in fusion.objects),
            "published_observed_face_count": sum(len(item.observed_faces) for item in fusion.objects),
            "coverage_status": fusion.coverage_status,
            "oracle_identity_used": False,
        }
        write_json(scene_dir / "multimodule_fusion_metrics.json", fusion_metrics)
        mode_c_scene = mode_c_by_scene.get(scene, {})
        comparison = {
            "scene": scene,
            "rgbd": {
                "center_error_m": report["mean_center_translation_error_m"],
                "orientation_error_deg": report["mean_orientation_angular_error_deg"],
                "dimension_abs_error_m": _scalar_mean(report["mean_full_dimension_abs_error_m"]),
                "dimension_relative_error": report["mean_per_axis_dimension_relative_error"],
            },
            "moge": {
                "status": "EVALUATED" if args.comparison else "NOT_REQUESTED",
                "center_error_m": mode_c_scene.get("mean_center_translation_error_m"),
                "orientation_error_deg": mode_c_scene.get("mean_orientation_angular_error_deg"),
                "dimension_abs_error_m": _scalar_mean(mode_c_scene.get("mean_full_dimension_abs_error_m")),
                "dimension_relative_error": mode_c_scene.get("mean_per_axis_dimension_relative_error"),
            },
            "hypothesis_count": sum(len(items) for items in hypothesis_groups.values()),
            "ambiguous_instance_count": sum(len(items) > 1 for items in hypothesis_groups.values()),
            "elapsed_seconds": elapsed,
            "modules": {
                batches[0].module_id: {"elapsed_seconds": elapsed, "timing_seconds": primary['timing_seconds'],
                    "observed_face_sets": len(batches[0].face_sets)},
                **{item["module_id"]: {
                    "elapsed_seconds": item["elapsed_seconds"],
                    "timing_seconds": item["timing_seconds"],
                    "observed_face_sets": len(item["observed_face_sets"]),
                } for item in secondary_results},
            },
            "fusion": fusion_metrics,
        }
        for key in ("center_error_m", "orientation_error_deg", "dimension_abs_error_m"):
            moge = comparison["moge"][key]
            rgbd = comparison["rgbd"][key]
            comparison.setdefault("relative_improvement", {})[key] = None if moge in (None, 0.0) or rgbd is None else (moge - rgbd) / moge
        results.append(comparison)
        if scene == "RGBD_CALIBRATION_BOX":
            limits = config["vision"]["calibration_gate"]
            checks = {
                "center_translation_error_m": {
                    "value": report["mean_center_translation_error_m"],
                    "maximum": float(limits["max_center_translation_error_m"]),
                },
                "orientation_error_deg": {
                    "value": report["mean_cuboid_symmetry_orientation_error_deg"],
                    "maximum": float(limits["max_orientation_error_deg"]),
                    "metric": "CUBOID_D2_SYMMETRY_EQUIVALENT",
                    "raw_quaternion_error_deg": report["mean_orientation_angular_error_deg"],
                },
                "full_dimension_abs_error_m": {
                    "value": _scalar_mean(report["mean_full_dimension_abs_error_m"]),
                    "maximum": float(limits["max_full_dimension_abs_error_m"]),
                },
            }
            calibration_passed = all(
                item["value"] is not None and float(item["value"]) <= item["maximum"]
                for item in checks.values()
            )
            gate = {
                "schema_version": "rgbd_calibration_gate_v1",
                "status": "PASS" if calibration_passed else "FAIL",
                "checks": checks,
                "failure_action": "STOP_BEFORE_FULL_STACK_RGBD_EVALUATION",
            }
            write_json(capture_root / "rgbd_calibration_gate.json", gate)
            if not calibration_passed:
                raise RuntimeError(f"registered RGB-D calibration gate failed: {checks}")
        elif not calibration_passed:
            raise RuntimeError("RGB-D full-stack evaluation cannot precede the calibration-box gate")
        if scene == "FULL_STACK_NOMINAL":
            import cv2
            segmentation = primary_payload.rgb[..., ::-1].copy()
            layer = segmentation.copy()
            for mask_id, mask in predicted_masks.items():
                color = tuple(int(value) for value in hashlib.sha256(mask_id.encode()).digest()[:3])
                layer[mask] = color
            segmentation = cv2.addWeighted(segmentation, 0.60, layer, 0.40, 0.0)
            cv2.imwrite(str(capture_root / "10_segmentation.png"), segmentation)
            cv2.imwrite(str(capture_root / "11_rgbd_cuboids.png"), cv2.imread(str(geometry_image)))
            if not args.comparison:
                _top_view(capture_root / "14_world_topview_cuboids.png", observation)
                continue
            moge_image = cv2.imread(str(Path(artifacts["final_instance_aware.jpg"]["path"])))
            cv2.imwrite(str(capture_root / "12_moge_cuboids.png"), moge_image)
            size = (1280, 720)
            comparison_image = np.hstack((
                cv2.resize(cv2.imread(str(geometry_image)), size, interpolation=cv2.INTER_AREA),
                cv2.resize(moge_image, size, interpolation=cv2.INTER_AREA),
            ))
            cv2.putText(comparison_image, "REGISTERED RGB-D | MONOCULAR MOGE", (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(str(capture_root / "13_rgbd_vs_moge.png"), comparison_image)
            _top_view(capture_root / "14_world_topview_cuboids.png", observation)
    summary = {
        "schema_version": "rgbd_vs_moge_summary_v1",
        "status": "PASS",
        "primary_mode": "STAGED_RGBD",
        "comparison_mode": "STAGED_MONOCULAR_MOGE" if args.comparison else "DISABLED",
        "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
        "raw_image_automatic": False,
        "detector_metrics": "NOT_EVALUATED",
        "calibration_gate": "PASS",
        "scenes": results,
        "claim_boundary": "PERCEPTION_GEOMETRY_VALID is distinct from FEASIBILITY_EXECUTION_VALID",
    }
    write_json(capture_root / "rgbd_vs_moge_summary.json", summary)
    domain_path = capture_root / "domain_summary.json"
    domain = json.loads((source_capture_root / "domain_summary.json").read_text(encoding="utf-8"))
    domain["mode_b_staged_rgbd"] = "PASS"
    domain["mode_b_summary"] = str((capture_root / "rgbd_vs_moge_summary.json").resolve())
    write_json(domain_path, domain)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except CapturePayloadError as exc:
        print(f'payload_validation: {exc}', file=sys.stderr, flush=True)
        raise SystemExit(1)
