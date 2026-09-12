"""Run Mode B registered-RGB-D geometry and compare it with Mode C MoGe.

SAM masks and 2D faces come from the pinned upstream worker.  This adapter
replaces only the point-map source, reusing the pinned plane/orthogonal-cuboid
recovery with explicit registered-depth provenance.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from time import perf_counter

import cv2
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
from unloading_perception.rgbd import (  # noqa: E402
    CaptureMetadata, MetricPointMap, MetricPointMapSource, PointCloudFilterConfig,
    filter_registered_instance_depth, hypotheses_from_geometry_record, masked_metric_pointmap, register_rgbd,
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


def _worker_artifacts(scene_dir: Path) -> dict:
    response = json.loads((scene_dir / "mode_b1_worker_response.json").read_text(encoding="utf-8").strip().splitlines()[-1])
    if response.get("status") != "COMPLETE":
        raise RuntimeError(f"Mode C worker response is incomplete for {scene_dir.name}")
    metrics = json.loads(Path(response["metrics_reference"]["path"]).read_text(encoding="utf-8"))
    return metrics["artifacts"]


def _build_pointmap(scene_dir: Path, manifest: IsaacSceneManifest, masks_path: Path, config: dict) -> tuple[MetricPointMap, list[dict]]:
    metadata = CaptureMetadata.from_dict(json.loads((scene_dir / "capture_metadata.json").read_text(encoding="utf-8")))
    camera = next(item for item in manifest.cameras if str(item["frame_id"]) == metadata.rgb_frame_id)
    rgb = np.load(scene_dir / "sensor_rgb.npy", allow_pickle=False)
    depth_path = scene_dir / "metric_depth_m.npy"
    depth = np.load(depth_path, allow_pickle=False)
    frame = register_rgbd(
        metadata=metadata, rgb=rgb, depth_optical_z_m=depth,
        rgb_K=camera["K"], depth_K=camera["K"], T_rgb_depth=camera["T_rgb_depth"],
        rgb_calibration_identity=metadata.calibration_identity,
        depth_calibration_identity=metadata.calibration_identity,
        registration_mode=camera["registration_mode"],
    )
    archive = np.load(masks_path, allow_pickle=False)
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
            filter_result = filter_registered_instance_depth(frame, mask, filter_config)
            item = masked_metric_pointmap(
                frame, mask, depth_identity=sha256(depth_path),
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
        metadata.calibration_identity, sha256(depth_path),
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
) -> tuple[PerceptionObservation, dict[str, np.ndarray], dict[str, tuple], tuple]:
    proposal_by_id = {int(item["id"]): item for item in proposals["instances"]}
    mask_archive = np.load(masks_path, allow_pickle=False)
    masks_by_id = {
        str(int(mask_id)): mask_archive["masks"][index].astype(bool)
        for index, mask_id in enumerate(mask_archive["mask_ids"])
    }
    metadata = CaptureMetadata.from_dict(json.loads((Path(proposals["_scene_dir"]) / "capture_metadata.json").read_text(encoding="utf-8")))
    camera = next(item for item in manifest.cameras if str(item["frame_id"]) == metadata.rgb_frame_id)
    module_id = str(camera.get("module_id", metadata.rgb_frame_id.removesuffix("_rgb_optical")))
    cargo = []
    unknown = []
    hypothesis_groups = {}
    observed_face_sets = []
    for record in geometry_payload["instances"]:
        mask_id = int(record["mask_id"])
        source_id = str(mask_id)
        proposal = proposal_by_id.get(mask_id, {})
        bbox = tuple(float(value) for value in proposal.get("bbox", (0.0, 0.0, 1.0, 1.0)))
        observed_camera = observed_faces_from_geometry_record(
            record, source_instance_id=source_id, module_id=module_id,
            capture_id=metadata.capture_id, capture_time=metadata.capture_center_time,
            frame_id=metadata.rgb_frame_id,
        )
        observed_world = transform_observed_face_set(observed_camera, metadata.T_W_C_at_capture, "world")
        observed_face_sets.append(observed_world)
        hypotheses = hypotheses_from_geometry_record(
            record, source_instance_id=source_id,
            pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,
            presence_score=float(proposal.get("score", 1.0)),
            camera_frame=metadata.rgb_frame_id,
        )
        world_hypotheses = tuple(transform_hypothesis_to_world(item, metadata) for item in hypotheses)
        hypothesis_groups[source_id] = world_hypotheses
        if world_hypotheses:
            selected = world_hypotheses[0]
            reasons = tuple(selected.eligibility_reasons)
            cargo.append(CargoObservation(
                source_instance_id=source_id,
                object_id=str(proposal.get("simulation_object_id")) if proposal.get("simulation_object_id") else None,
                track_id=None,
                category=str(proposal.get("label", "box")),
                bbox_xyxy=bbox,
                mask_reference=ResourceReference(masks_path.resolve().as_uri() + f"#{source_id}", sha256(masks_path), "application/x-npz; array=bool"),
                detection_score=1.0,
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
                    "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
                    "oracle_proposal_source_id": proposal.get("simulation_object_id"),
                    "simulation_object_id": proposal.get("simulation_object_id"),
                    "pointmap_source": "ISAAC_IDEAL_REGISTERED_DEPTH",
                    "hypothesis_count": len(world_hypotheses),
                    "hypotheses": [_hypothesis_dict(item) for item in world_hypotheses],
                    "observed_face_set": observed_world.to_dict(),
                    "algorithm": "PINNED_UPSTREAM_V4_OBSERVED_FACE_RECOVERY",
                },
            ))
        else:
            if observed_world.complete_cuboid_status == "INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR":
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
                object_id=str(proposal.get("simulation_object_id")) if proposal.get("simulation_object_id") else None,
                track_id=None,
                category=str(proposal.get("label", "box")),
                bbox_xyxy=bbox,
                mask_reference=ResourceReference(masks_path.resolve().as_uri() + f"#{source_id}", sha256(masks_path), "application/x-npz; array=bool"),
                detection_score=1.0,
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
                    "pointmap_source": "ISAAC_IDEAL_REGISTERED_DEPTH", "record": record,
                    "observed_face_set": observed_world.to_dict(),
                    "complete_cuboid_status": observed_world.complete_cuboid_status,
                },
            ))
            unknown.append(UnknownRegion(f"rgbd-{source_id}", metadata.rgb_frame_id, reasons[0], bbox))
    observation = PerceptionObservation(
        SCHEMA_VERSION, f"rgbd-{scene}-{metadata.capture_id}", metadata.sensor_epoch,
        metadata.frame_sequence, metadata.capture_center_time, metadata.capture_center_time + elapsed,
        metadata.clock_domain, "staged-registered-rgbd-geometry", "1d208f2ed380a207e6e46b4a62d2ac640edfe477",
        "sam+pinned-plane-cuboid+registered-depth", metadata.calibration_identity,
        ObservationStatus.COMPLETE if not unknown else ObservationStatus.PARTIAL,
        None, None, tuple(cargo), tuple(unknown),
        {
            "mode": "STAGED_RGBD", "role": "PRIMARY", "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
            "detector_metrics": "NOT_EVALUATED", "pointmap_source": "ISAAC_IDEAL_REGISTERED_DEPTH",
            "capture_id": metadata.capture_id, "absence_means_free_space": False,
        },
        False,
    )
    return observation, masks_by_id, hypothesis_groups, tuple(observed_face_sets)


def _top_view(path: Path, observation: PerceptionObservation) -> None:
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
) -> dict:
    masks_path = Path(artifacts["cargo_masks.npz"]["path"])
    pointmap, audits = _build_pointmap(module_dir, manifest, masks_path, config["vision"]["pointcloud_filter"])
    pointmap_path = module_dir / "registered_metric_pointmap.npz"
    pointmap.write_npz(pointmap_path)
    raw_json = module_dir / "rgbd_cuboids_baseline_raw.json"
    base_image = module_dir / "rgbd_cuboids_baseline.png"
    base_command = [
        str(upstream_python), str(vision_root / "pipeline/geometry/recover_box_cuboids_3d.py"),
        str(module_dir / "sensor_rgb.png"), str(masks_path), str(pointmap_path),
        "--faces-json", str(Path(artifacts["box_geometry_2d.json"]["path"])),
        "--relative-threshold", "0.003", "--seed", "17",
        "--json-output", str(raw_json), "--output", str(base_image),
    ]
    started = perf_counter()
    completed = subprocess.run(base_command, cwd=vision_root, text=True, capture_output=True, timeout=timeout)
    if completed.returncode != 0:
        raise RuntimeError(f"registered RGB-D baseline failed for {scene}/{module_dir.name}: {completed.stderr[-1000:]}")
    baseline_json = module_dir / "rgbd_cuboids_v4_input.json"
    write_json(baseline_json, prepare_registered_depth_baseline(json.loads(raw_json.read_text(encoding="utf-8"))))
    worker_json = module_dir / "rgbd_cuboids_v4_worker.json"
    geometry_json = module_dir / "rgbd_cuboids.json"
    geometry_image = module_dir / "rgbd_cuboids.png"
    command = build_upstream_v4_command(
        upstream_python, vision_root, source=module_dir / "sensor_rgb.png", masks=masks_path,
        pointmap=pointmap_path, baseline=baseline_json, fallback=baseline_json,
        json_output=worker_json, image_output=geometry_image,
    )
    validate_upstream_v4_command(command, vision_root)
    completed = subprocess.run(command, cwd=vision_root, text=True, capture_output=True, timeout=timeout)
    elapsed = perf_counter() - started
    if completed.returncode != 0:
        raise RuntimeError(f"observed-face V4 failed for {scene}/{module_dir.name}: {completed.stderr[-1000:]}")
    geometry = finalize_registered_depth_result(json.loads(worker_json.read_text(encoding="utf-8")))
    geometry["method"] = "registered_metric_depth+pinned_planes+upstream_v4_joint_observed_faces"
    write_json(geometry_json, geometry)
    proposals = json.loads((module_dir / "oracle_proposals.json").read_text(encoding="utf-8"))
    proposals["_scene_dir"] = str(module_dir)
    observation, predicted_masks, hypotheses, observed_sets = _observation(
        scene, manifest, geometry, masks_path, proposals, elapsed,
    )
    truth_payload = json.loads((module_dir / "gt_annotations.json").read_text(encoding="utf-8"))
    truth = ground_truth_observation(manifest, truth_payload["objects"])
    gt_archive = np.load(module_dir / "gt_instance_masks.npz", allow_pickle=False)
    metadata = CaptureMetadata.from_dict(json.loads((module_dir / "capture_metadata.json").read_text(encoding="utf-8")))
    camera = next(item for item in manifest.cameras if item["frame_id"] == metadata.rgb_frame_id)
    report = evaluate_observations(
        truth, observation, ground_truth_masks={name: gt_archive[name] for name in gt_archive.files},
        prediction_masks=predicted_masks, T_W_C=camera["T_W_C"],
    )
    report["mode"] = "STAGED_RGBD"
    report["module_id"] = module_dir.name
    report["world_transform_evaluation"]["metric_scale_valid"] = True
    report["world_transform_evaluation"]["prediction_camera_frame"] = metadata.rgb_frame_id
    write_evaluation(module_dir / "mode_b_rgbd_evaluation.json", report)
    (module_dir / "mode_b_rgbd_observation.json").write_text(dumps(observation), encoding="utf-8")
    write_json(module_dir / "metric_pointmap_filter_audit.json", audits)
    return {
        "module_id": module_dir.name, "elapsed_seconds": elapsed, "observation": observation,
        "observed_face_sets": observed_sets, "hypotheses": hypotheses, "report": report,
        "geometry_image": geometry_image, "predicted_masks": predicted_masks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-directory", required=True, type=Path)
    parser.add_argument("--bundle-directory", required=True, type=Path)
    parser.add_argument("--vision-root", required=True, type=Path)
    parser.add_argument("--upstream-python", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=1200.0)
    args = parser.parse_args()
    capture_root = args.capture_directory.resolve()
    bundle_root = args.bundle_directory.resolve()
    vision_root = args.vision_root.resolve()
    index = json.loads((bundle_root / "index.json").read_text(encoding="utf-8"))
    config = json.loads(json.dumps(__import__("yaml").safe_load((ROOT / "configs/isaac/perception_validation.yaml").read_text(encoding="utf-8"))))
    mode_c = json.loads((capture_root / "mode_c_moge_summary.json").read_text(encoding="utf-8"))
    mode_c_by_scene = {item["scene"]: item for item in mode_c["scenes"]}
    results = []
    ordered_records = sorted(
        index["scenes"],
        key=lambda item: (item["scene"] != "RGBD_CALIBRATION_BOX", index["scenes"].index(item)),
    )
    calibration_passed = False
    for record in ordered_records:
        scene = record["scene"]
        scene_dir = capture_root / scene
        manifest = IsaacSceneManifest.from_dict(json.loads((bundle_root / record["path"]).read_text(encoding="utf-8")))
        artifacts = _worker_artifacts(scene_dir)
        masks_path = Path(artifacts["cargo_masks.npz"]["path"])
        pointmap, filter_audits = _build_pointmap(scene_dir, manifest, masks_path, config["vision"]["pointcloud_filter"])
        pointmap_path = scene_dir / "registered_metric_pointmap.npz"
        pointmap.write_npz(pointmap_path)
        baseline_raw_json = scene_dir / "rgbd_cuboids_baseline_raw.json"
        baseline_image = scene_dir / "rgbd_cuboids_baseline.png"
        base_command = [
            str(args.upstream_python), str(vision_root / "pipeline/geometry/recover_box_cuboids_3d.py"),
            str(scene_dir / "sensor_rgb.png"), str(masks_path), str(pointmap_path),
            "--faces-json", str(Path(artifacts["box_geometry_2d.json"]["path"])),
            "--relative-threshold", "0.003", "--seed", "17",
            "--json-output", str(baseline_raw_json), "--output", str(baseline_image),
        ]
        started = perf_counter()
        completed = subprocess.run(base_command, cwd=vision_root, text=True, capture_output=True, timeout=args.timeout)
        if completed.returncode != 0:
            raise RuntimeError(f"registered RGB-D baseline recovery failed for {scene}: {completed.stderr[-1000:]}")
        baseline = json.loads(baseline_raw_json.read_text(encoding="utf-8"))
        prepared_baseline = prepare_registered_depth_baseline(baseline)
        baseline_json = scene_dir / "rgbd_cuboids_v4_input.json"
        write_json(baseline_json, prepared_baseline)
        geometry_worker_json = scene_dir / "rgbd_cuboids_v4_worker.json"
        geometry_json = scene_dir / "rgbd_cuboids.json"
        geometry_image = scene_dir / "rgbd_cuboids.png"
        command = build_upstream_v4_command(
            args.upstream_python, vision_root,
            source=scene_dir / "sensor_rgb.png", masks=masks_path, pointmap=pointmap_path,
            baseline=baseline_json, fallback=baseline_json,
            json_output=geometry_worker_json, image_output=geometry_image,
        )
        validate_upstream_v4_command(command, vision_root)
        completed = subprocess.run(command, cwd=vision_root, text=True, capture_output=True, timeout=args.timeout)
        elapsed = perf_counter() - started
        if completed.returncode != 0:
            raise RuntimeError(f"registered RGB-D observed-face V4 recovery failed for {scene}: {completed.stderr[-1000:]}")
        geometry = finalize_registered_depth_result(json.loads(geometry_worker_json.read_text(encoding="utf-8")))
        geometry["method"] = "registered_metric_depth+pinned_planes+upstream_v4_joint_observed_faces"
        write_json(geometry_json, geometry)
        proposals = json.loads((scene_dir / "oracle_proposals.json").read_text(encoding="utf-8"))
        proposals["_scene_dir"] = str(scene_dir)
        observation, predicted_masks, hypothesis_groups, upper_observed_sets = _observation(
            scene, manifest, geometry, masks_path, proposals, elapsed,
        )
        truth_payload = json.loads((scene_dir / "gt_annotations.json").read_text(encoding="utf-8"))
        truth = ground_truth_observation(manifest, truth_payload["objects"])
        gt_archive = np.load(scene_dir / "gt_instance_masks.npz", allow_pickle=False)
        report = evaluate_observations(
            truth, observation,
            ground_truth_masks={name: gt_archive[name] for name in gt_archive.files},
            prediction_masks=predicted_masks, T_W_C=manifest.cameras[0]["T_W_C"],
        )
        report["mode"] = "STAGED_RGBD"
        report["world_transform_evaluation"]["metric_scale_valid"] = True
        report["world_transform_evaluation"]["prediction_camera_frame"] = "module_0_main_rgb_optical"
        report["world_transform_evaluation"]["claim_boundary"] = (
            "registered metric depth is evaluated in world using capture-time T_W_C; "
            "this does not establish execution qualification"
        )
        write_evaluation(scene_dir / "mode_b_rgbd_evaluation.json", report)
        (scene_dir / "mode_b_rgbd_observation.json").write_text(dumps(observation), encoding="utf-8")
        write_json(scene_dir / "metric_pointmap_filter_audit.json", filter_audits)
        secondary_results = []
        for module_camera in manifest.cameras[1:]:
            module_dir = scene_dir / "modules" / str(module_camera["module_id"])
            secondary_results.append(_run_secondary_module(
                scene=scene, module_dir=module_dir, manifest=manifest,
                artifacts=_worker_artifacts(module_dir), config=config, vision_root=vision_root,
                upstream_python=args.upstream_python, timeout=args.timeout,
            ))
        upper_metadata = CaptureMetadata.from_dict(json.loads((scene_dir / "capture_metadata.json").read_text(encoding="utf-8")))
        batches = [ModuleFaceBatch(
            str(manifest.cameras[0].get("module_id", "module_0_upper")), upper_metadata.sensor_epoch,
            upper_metadata.frame_sequence, upper_metadata.capture_center_time,
            tuple(item for item in upper_observed_sets if item.faces),
        )]
        for item in secondary_results:
            module_dir = scene_dir / "modules" / item["module_id"]
            metadata = CaptureMetadata.from_dict(json.loads((module_dir / "capture_metadata.json").read_text(encoding="utf-8")))
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
        mode_c_scene = mode_c_by_scene[scene]
        comparison = {
            "scene": scene,
            "rgbd": {
                "center_error_m": report["mean_center_translation_error_m"],
                "orientation_error_deg": report["mean_orientation_angular_error_deg"],
                "dimension_abs_error_m": _scalar_mean(report["mean_full_dimension_abs_error_m"]),
                "dimension_relative_error": report["mean_per_axis_dimension_relative_error"],
            },
            "moge": {
                "center_error_m": mode_c_scene["mean_center_translation_error_m"],
                "orientation_error_deg": mode_c_scene["mean_orientation_angular_error_deg"],
                "dimension_abs_error_m": _scalar_mean(mode_c_scene["mean_full_dimension_abs_error_m"]),
                "dimension_relative_error": mode_c_scene["mean_per_axis_dimension_relative_error"],
            },
            "hypothesis_count": sum(len(items) for items in hypothesis_groups.values()),
            "ambiguous_instance_count": sum(len(items) > 1 for items in hypothesis_groups.values()),
            "elapsed_seconds": elapsed,
            "modules": {
                batches[0].module_id: {"elapsed_seconds": elapsed, "observed_face_sets": len(batches[0].face_sets)},
                **{item["module_id"]: {
                    "elapsed_seconds": item["elapsed_seconds"],
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
                    "value": report["mean_orientation_angular_error_deg"],
                    "maximum": float(limits["max_orientation_error_deg"]),
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
            segmentation = cv2.imread(str(scene_dir / "sensor_rgb.png"))
            layer = segmentation.copy()
            for mask_id, mask in predicted_masks.items():
                color = tuple(int(value) for value in hashlib.sha256(mask_id.encode()).digest()[:3])
                layer[mask] = color
            segmentation = cv2.addWeighted(segmentation, 0.60, layer, 0.40, 0.0)
            cv2.imwrite(str(capture_root / "10_segmentation.png"), segmentation)
            cv2.imwrite(str(capture_root / "11_rgbd_cuboids.png"), cv2.imread(str(geometry_image)))
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
        "comparison_mode": "STAGED_MONOCULAR_MOGE",
        "proposal_source": "ISAAC_GT_ORACLE_PROPOSAL",
        "detector_metrics": "NOT_EVALUATED",
        "calibration_gate": "PASS",
        "scenes": results,
        "claim_boundary": "PERCEPTION_GEOMETRY_VALID is distinct from FEASIBILITY_EXECUTION_VALID",
    }
    write_json(capture_root / "rgbd_vs_moge_summary.json", summary)
    domain_path = capture_root / "domain_summary.json"
    domain = json.loads(domain_path.read_text(encoding="utf-8"))
    domain["mode_b_staged_rgbd"] = "PASS"
    domain["mode_b_summary"] = str((capture_root / "rgbd_vs_moge_summary.json").resolve())
    write_json(domain_path, domain)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
