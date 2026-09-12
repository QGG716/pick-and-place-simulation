"""Run the pinned vision pipeline on Isaac RGB with oracle proposals only."""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from queue import Empty, Queue
import subprocess
import sys
from threading import Thread
from time import perf_counter

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import SCHEMA_VERSION, dumps, loads  # noqa: E402
from unloading_perception.isaac_evaluation import evaluate_depth, evaluate_observations, write_evaluation  # noqa: E402
from unloading_perception.isaac_validation import IsaacSceneManifest, ground_truth_observation, write_json  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _stable_color(identity: str) -> tuple[int, int, int]:
    digest = hashlib.sha256(identity.encode("utf-8")).digest()
    return tuple(80 + int(value) % 176 for value in digest[:3])


def _label_panel(image: np.ndarray, label: str, color: tuple[int, int, int]) -> np.ndarray:
    result = image.copy()
    cv2.rectangle(result, (0, 0), (result.shape[1], 30), (15, 15, 15), -1)
    cv2.putText(result, label, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2, cv2.LINE_AA)
    return result


def _camera_for_frame(manifest: IsaacSceneManifest, frame_id: str) -> dict:
    matches = [camera for camera in manifest.cameras if str(camera["frame_id"]) == frame_id]
    if len(matches) != 1:
        raise ValueError(f"capture frame is not bound to exactly one manifest camera: {frame_id}")
    return matches[0]


class _ResidentWorkerClient:
    def __init__(self, command: list[str], *, cwd: Path, worker_epoch: str, timeout: float, log_path: Path) -> None:
        self.command = command
        self.cwd = cwd
        self.worker_epoch = worker_epoch
        self.timeout = timeout
        self.log_stream = log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.log_stream, text=True, bufsize=1,
        )
        self.lines: Queue[str] = Queue(maxsize=2)
        assert self.process.stdout is not None and self.process.stdin is not None

        def read_stdout() -> None:
            assert self.process.stdout is not None
            for line in self.process.stdout:
                self.lines.put(line)
            self.lines.put("")

        Thread(target=read_stdout, name="isaac-b1-resident-stdout", daemon=True).start()
        self.process.stdin.write(json.dumps({
            "schema_version": SCHEMA_VERSION, "op": "hello", "worker_epoch": worker_epoch,
        }, sort_keys=True) + "\n")
        self.process.stdin.flush()
        ready = self._read()
        if ready.get("op") != "ready" or ready.get("worker_epoch") != worker_epoch:
            self.close(force=True)
            raise RuntimeError(f"resident worker handshake failed: {ready}")
        self.ready = ready

    def _read(self) -> dict:
        try:
            line = self.lines.get(timeout=self.timeout)
        except Empty as exc:
            raise TimeoutError("resident worker response timed out") from exc
        if not line:
            raise RuntimeError(f"resident worker exited with code {self.process.poll()}")
        return json.loads(line)

    def infer(self, request: dict) -> dict:
        if self.process.poll() is not None or self.process.stdin is None:
            raise RuntimeError("resident worker is not running")
        self.process.stdin.write(json.dumps(request, sort_keys=True) + "\n")
        self.process.stdin.flush()
        return self._read()

    def close(self, *, force: bool = False) -> None:
        if self.process.poll() is None and not force and self.process.stdin is not None:
            self.process.stdin.write(json.dumps({
                "schema_version": SCHEMA_VERSION, "op": "shutdown", "worker_epoch": self.worker_epoch,
            }, sort_keys=True) + "\n")
            self.process.stdin.flush()
        if self.process.poll() is None:
            try:
                self.process.wait(timeout=10.0 if not force else 1.0)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
        self.log_stream.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-directory", required=True, type=Path)
    parser.add_argument("--bundle-directory", required=True, type=Path)
    parser.add_argument("--vision-root", required=True, type=Path)
    parser.add_argument("--gpu-python", required=True, type=Path)
    parser.add_argument("--model-manifest", required=True, type=Path)
    parser.add_argument("--worker-timeout", type=float, default=1200.0)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--reuse-existing", action="store_true", help="Re-evaluate validated worker responses without GPU inference")
    args = parser.parse_args()
    if args.worker_timeout <= 0.0 or args.fps <= 0 or args.seconds <= 0.0:
        raise ValueError("positive timeout, FPS and duration are required")

    capture_root = args.capture_directory.resolve()
    bundle_root = args.bundle_directory.resolve()
    vision_root = args.vision_root.resolve()
    model_manifest = json.loads(args.model_manifest.read_text(encoding="utf-8"))
    index = json.loads((bundle_root / "index.json").read_text(encoding="utf-8"))
    worker_root = capture_root / "mode_b1_worker_runs"
    worker_root.mkdir(parents=True, exist_ok=True)
    results = []
    video_frames = []
    previous_elapsed = {}
    previous_summary = capture_root / "mode_b1_summary.json"
    previous_data = None
    if args.reuse_existing and previous_summary.is_file():
        previous_data = json.loads(previous_summary.read_text(encoding="utf-8"))
        previous_elapsed = {
            item["scene"]: float(item["elapsed_seconds"])
            for item in previous_data["scenes"]
        }

    worker_epoch = f"isaac-b1-resident-{sha256(bundle_root / 'index.json')[:16]}"
    worker = None
    resident_ready = None if previous_data is None else previous_data.get("resident_worker_ready")
    if not args.reuse_existing:
        command = [
            str(args.gpu_python), str(ROOT / "tools/vision_resident_worker.py"),
            "--upstream-root", str(vision_root), "--output-root", str(worker_root),
            "--input-root", str(capture_root), "--input-root", str(vision_root),
            "--sam-model", str(model_manifest["sam"]["snapshot_path"]),
            "--sam-model-id", str(model_manifest["sam"]["repository"]),
            "--sam-revision", str(model_manifest["sam"]["revision"]),
            "--moge-model", str(model_manifest["moge"]["model_path"]),
            "--moge-model-id", str(model_manifest["moge"]["repository"]),
            "--moge-revision", str(model_manifest["moge"]["revision"]),
            "--stage-timeout", str(args.worker_timeout),
        ]
        worker = _ResidentWorkerClient(
            command, cwd=ROOT, worker_epoch=worker_epoch,
            timeout=args.worker_timeout + 60.0,
            log_path=capture_root / "mode_b1_resident_worker.log",
        )
        resident_ready = worker.ready

    try:
        for record in index["scenes"]:
            scene = record["scene"]
            scene_dir = capture_root / scene
            manifest = IsaacSceneManifest.from_dict(json.loads((bundle_root / record["path"]).read_text(encoding="utf-8")))
            annotations = json.loads((scene_dir / "gt_annotations.json").read_text(encoding="utf-8"))
            binding = json.loads((scene_dir / "capture_binding.json").read_text(encoding="utf-8"))
            capture_metadata = json.loads((scene_dir / "capture_metadata.json").read_text(encoding="utf-8"))
            primary_camera = _camera_for_frame(manifest, str(capture_metadata["rgb_frame_id"]))
            source = scene_dir / "sensor_rgb.png"
            proposals = scene_dir / "oracle_proposals.json"
            request = {
                "schema_version": SCHEMA_VERSION,
                "op": "infer",
                "request_id": f"isaac-b1-resident-{scene}-{binding['frame_sequence']}",
                "worker_epoch": worker_epoch,
                "proposal_reference": {"uri": proposals.resolve().as_uri(), "sha256": sha256(proposals)},
                "frame": {
                    "source": "isaac-sim-6.0.1", "stream": "perception_validation",
                    "epoch": binding["simulation_epoch"], "sequence": binding["frame_sequence"],
                    "capture_time": binding["simulation_time"], "receive_time": binding["simulation_time"],
                    "clock_domain": "ros_sim_time", "frame_id": primary_camera["frame_id"],
                    "width": primary_camera["resolution"][0], "height": primary_camera["resolution"][1],
                    "encoding": "rgb8", "rgb_uri": source.resolve().as_uri(), "rgb_sha256": sha256(source),
                },
            }
            response_path = scene_dir / "mode_b1_worker_response.json"
            if args.reuse_existing:
                if not response_path.is_file() or scene not in previous_elapsed:
                    raise FileNotFoundError(f"validated Mode B1 response is unavailable for {scene}")
                response = json.loads(response_path.read_text(encoding="utf-8").strip().splitlines()[-1])
                elapsed = previous_elapsed[scene]
            else:
                assert worker is not None
                started = perf_counter()
                response = worker.infer(request)
                elapsed = perf_counter() - started
                response_path.write_text(json.dumps(response, sort_keys=True) + "\n", encoding="utf-8")
            if response.get("status") != "COMPLETE":
                raise RuntimeError(f"Mode B1 worker failed for {scene}: {response.get('error_code')} {response.get('error_message')}")
            prediction = loads(response["observation"])
            prediction = replace(
                prediction, capture_time=float(binding["simulation_time"]),
                processed_time=float(binding["simulation_time"]) + elapsed,
                clock_domain="ros_sim_time",
                coverage={**dict(prediction.coverage), "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL", "raw_image_automatic": False},
            )
            if prediction.synthetic:
                raise RuntimeError("Mode B1 prediction cannot be synthetic")
            truth = ground_truth_observation(
                manifest, annotations["objects"], camera_frame_id=str(primary_camera["frame_id"]),
            )
            metrics_path = Path(response["metrics_reference"]["path"])
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            artifacts = metrics["artifacts"]
            pointmap = np.load(artifacts["moge2_pointmap.npz"]["path"])
            depth_report = evaluate_depth(
                np.load(scene_dir / "metric_depth_m.npy"), pointmap["depth"], valid_mask=pointmap["valid_mask"]
            )
            gt_archive = np.load(scene_dir / "gt_instance_masks.npz")
            gt_masks = {name: gt_archive[name] for name in gt_archive.files}
            predicted_archive = np.load(artifacts["cargo_masks.npz"]["path"])
            predicted_masks = {
                str(mask_id): predicted_archive["masks"][index].astype(bool)
                for index, mask_id in enumerate(predicted_archive["mask_ids"])
            }
            report = evaluate_observations(
                truth, prediction, ground_truth_masks=gt_masks,
                prediction_masks=predicted_masks, depth_evaluation=depth_report,
                T_W_C=primary_camera["T_W_C"],
            )
            write_evaluation(scene_dir / "mode_b1_evaluation.json", report)
            (scene_dir / "mode_b1_perception_observation.json").write_text(dumps(prediction), encoding="utf-8")

            matched_ids = {item["prediction_source_instance_id"]: item["simulation_object_id"] for item in report["per_object"]}
            image = cv2.imread(str(source))
            mask_layer = image.copy()
            for item in prediction.cargo:
                color = _stable_color(matched_ids.get(item.source_instance_id, item.source_instance_id))
                mask = predicted_masks.get(item.source_instance_id)
                if mask is not None and mask.shape == image.shape[:2]:
                    mask_layer[mask] = color
            image = cv2.addWeighted(image, 0.65, mask_layer, 0.35, 0.0)
            for item in prediction.cargo:
                identity = matched_ids.get(item.source_instance_id, item.source_instance_id)
                color = _stable_color(identity)
                x1, y1, x2, y2 = (int(round(value)) for value in item.bbox_xyxy)
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                status = "ELIGIBLE" if item.candidate_eligible else "REJECTED"
                cv2.putText(image, f"PRED {identity[-8:]} {status}", (x1, max(42, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1, cv2.LINE_AA)
            cv2.rectangle(image, (0, 0), (image.shape[1], 54), (15, 15, 15), -1)
            cv2.putText(image, f"MODE C MOGE | {scene} | n={len(prediction.cargo)}", (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (80, 255, 80), 2, cv2.LINE_AA)
            cv2.putText(image, f"ORACLE ROI; RAW=false; UNKNOWN={len(prediction.unknown_regions)}", (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 180, 255), 1, cv2.LINE_AA)
            prediction_overlay = scene_dir / "mode_b1_prediction_overlay.png"
            cv2.imwrite(str(prediction_overlay), image)

            gt = cv2.imread(str(source))
            gt_layer = gt.copy()
            for annotation in annotations["objects"]:
                identity = str(annotation["simulation_object_id"])
                mask = gt_masks.get(identity)
                color = _stable_color(identity)
                if mask is not None and mask.shape == gt.shape[:2]:
                    gt_layer[mask] = color
                if annotation.get("visible", False):
                    x1, y1, x2, y2 = (int(round(value)) for value in annotation["bbox_xyxy"])
                    cv2.rectangle(gt, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(gt, f"GT {identity[-8:]}", (x1, max(35, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1, cv2.LINE_AA)
            gt = cv2.addWeighted(gt, 0.65, gt_layer, 0.35, 0.0)
            gt = _label_panel(gt, f"ISAAC GT | n={len(truth.cargo)}", (80, 220, 255))
            cv2.imwrite(str(scene_dir / "mode_b1_gt_overlay.png"), gt)

            rgb_panel = _label_panel(cv2.imread(str(source)), "ISAAC RGB INPUT", (255, 255, 255))
            overview = cv2.resize(cv2.imread(str(scene_dir / "isaac_overview.png")), (image.shape[1], image.shape[0]))
            overview = _label_panel(overview, "WORLD / CAMERA VIEW", (255, 220, 80))
            upstream_3d = cv2.imread(str(artifacts["final_instance_aware.jpg"]["path"]))
            upstream_3d = cv2.resize(upstream_3d, (image.shape[1], image.shape[0]))
            upstream_3d = _label_panel(upstream_3d, "PREDICTED 3D GEOMETRY (MONOCULAR SCALE)", (80, 255, 80))
            tile_size = (640, 360)
            comparison = np.vstack((
                np.hstack((cv2.resize(rgb_panel, tile_size, interpolation=cv2.INTER_AREA), cv2.resize(gt, tile_size, interpolation=cv2.INTER_AREA))),
                np.hstack((cv2.resize(image, tile_size, interpolation=cv2.INTER_AREA), cv2.resize(upstream_3d, tile_size, interpolation=cv2.INTER_AREA))),
            ))
            cv2.imwrite(str(scene_dir / "mode_b1_gt_prediction_comparison.png"), comparison)
            video_frames.append(comparison)
            results.append({
                "scene": scene, "status": "PASS", "elapsed_seconds": elapsed,
                "proposal_count": len(json.loads(proposals.read_text(encoding="utf-8"))["instances"]),
                "prediction_count": len(prediction.cargo), "processed_object_count": report["processed_object_count"],
                "mean_proposal_bbox_iou": report["mean_proposal_bbox_iou"],
                "mean_estimated_mask_bbox_iou": report["mean_estimated_mask_bbox_iou"],
                "mean_mask_iou": report["mean_mask_iou"],
                "mean_center_translation_error_m": report["mean_center_translation_error_m"],
                "mean_orientation_angular_error_deg": report["mean_orientation_angular_error_deg"],
                "mean_full_dimension_abs_error_m": report["mean_full_dimension_abs_error_m"],
                "mean_per_axis_dimension_relative_error": report["mean_per_axis_dimension_relative_error"],
                "unknown_region_count": report["unknown_region_count"],
                "depth": depth_report, "evaluation_fingerprint": report["evaluation_fingerprint"],
                "worker_response_sha256": sha256(response_path),
            })
            module_runs = {
                str(primary_camera["module_id"]): {
                    "response": str(response_path), "elapsed_seconds": elapsed,
                    "status": "PASS",
                }
            }
            for module_camera in (
                camera for camera in manifest.cameras
                if str(camera["frame_id"]) != str(primary_camera["frame_id"])
            ):
                module_id = str(module_camera["module_id"])
                module_dir = scene_dir / "modules" / module_id
                module_binding = json.loads((module_dir / "capture_binding.json").read_text(encoding="utf-8"))
                module_source = module_dir / "sensor_rgb.png"
                module_proposals = module_dir / "oracle_proposals.json"
                module_request = {
                    "schema_version": SCHEMA_VERSION, "op": "infer",
                    "request_id": f"isaac-b1-resident-{scene}-{module_id}-{module_binding['frame_sequence']}",
                    "worker_epoch": worker_epoch,
                    "proposal_reference": {"uri": module_proposals.resolve().as_uri(), "sha256": sha256(module_proposals)},
                    "frame": {
                        "source": "isaac-sim-6.0.1", "stream": f"perception_validation/{module_id}",
                        "epoch": module_binding["simulation_epoch"], "sequence": module_binding["frame_sequence"],
                        "capture_time": module_binding["simulation_time"], "receive_time": module_binding["simulation_time"],
                        "clock_domain": "ros_sim_time", "frame_id": module_camera["frame_id"],
                        "width": module_camera["resolution"][0], "height": module_camera["resolution"][1],
                        "encoding": "rgb8", "rgb_uri": module_source.resolve().as_uri(),
                        "rgb_sha256": sha256(module_source),
                    },
                }
                module_response_path = module_dir / "mode_b1_worker_response.json"
                if args.reuse_existing:
                    if not module_response_path.is_file():
                        raise FileNotFoundError(f"validated Mode B1 response is unavailable for {scene}/{module_id}")
                    module_response = json.loads(module_response_path.read_text(encoding="utf-8").strip().splitlines()[-1])
                    module_elapsed = float(module_response.get("elapsed_seconds", 0.0))
                else:
                    assert worker is not None
                    module_started = perf_counter()
                    module_response = worker.infer(module_request)
                    module_elapsed = perf_counter() - module_started
                    module_response["elapsed_seconds"] = module_elapsed
                    module_response_path.write_text(json.dumps(module_response, sort_keys=True) + "\n", encoding="utf-8")
                if module_response.get("status") != "COMPLETE":
                    raise RuntimeError(
                        f"Mode B1 worker failed for {scene}/{module_id}: "
                        f"{module_response.get('error_code')} {module_response.get('error_message')}"
                    )
                module_metrics = json.loads(Path(module_response["metrics_reference"]["path"]).read_text(encoding="utf-8"))
                module_artifacts = module_metrics["artifacts"]
                module_preview = cv2.imread(str(module_artifacts["sam_crossvalidated.jpg"]["path"]))
                if module_preview is not None:
                    cv2.imwrite(str(module_dir / "mode_b1_sam_segmentation.png"), module_preview)
                module_runs[module_id] = {
                    "response": str(module_response_path), "elapsed_seconds": module_elapsed,
                    "status": "PASS", "artifacts": module_artifacts,
                }
            results[-1]["modules"] = module_runs
    finally:
        if worker is not None:
            worker.close()

    video_path = capture_root / "isaac_perception_mode_b1_validation.mp4"
    height, width = video_frames[0].shape[:2]
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), float(args.fps), (width, height))
    if not writer.isOpened():
        raise RuntimeError("could not create Mode B1 MP4")
    repeats = max(1, int(round(args.fps * args.seconds / len(video_frames))))
    for frame in video_frames:
        for _ in range(repeats):
            writer.write(frame)
    writer.release()
    summary = {
        "schema_version": "isaac_perception_mode_c_summary_v2", "status": "PASS",
        "mode": "STAGED_MONOCULAR_MOGE", "role": "COMPARISON", "raw_image_automatic": False,
        "execution_model": "one resident GPU worker with SAM and MoGe loaded once",
        "resident_worker_ready": resident_ready,
        "vision_commit": "1d208f2ed380a207e6e46b4a62d2ac640edfe477",
        "scene_count": len(results), "scenes": results,
        "video": str(video_path), "video_sha256": sha256(video_path),
        "claim_boundary": "synthetic rendered-image results do not establish real-camera accuracy",
    }
    write_json(capture_root / "mode_b1_summary.json", summary)
    write_json(capture_root / "mode_c_moge_summary.json", summary)
    domain_path = capture_root / "domain_summary.json"
    domain = json.loads(domain_path.read_text(encoding="utf-8"))
    domain["mode_c_moge"] = "PASS"
    domain["mode_c_moge_summary"] = str((capture_root / "mode_c_moge_summary.json").resolve())
    write_json(domain_path, domain)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
