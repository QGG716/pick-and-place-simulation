"""Run the pinned vision pipeline on Isaac RGB with oracle proposals only."""

from __future__ import annotations

import argparse
from dataclasses import replace
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

from unloading_contracts import SCHEMA_VERSION, dumps, loads  # noqa: E402
from unloading_perception.isaac_evaluation import evaluate_depth, evaluate_observations, write_evaluation  # noqa: E402
from unloading_perception.isaac_validation import IsaacSceneManifest, ground_truth_observation, write_json  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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

    for record in index["scenes"]:
        scene = record["scene"]
        scene_dir = capture_root / scene
        manifest = IsaacSceneManifest.from_dict(json.loads((bundle_root / record["path"]).read_text(encoding="utf-8")))
        annotations = json.loads((scene_dir / "gt_annotations.json").read_text(encoding="utf-8"))
        binding = json.loads((scene_dir / "capture_binding.json").read_text(encoding="utf-8"))
        source = scene_dir / "sensor_rgb.png"
        proposals = scene_dir / "oracle_proposals.json"
        request = {
            "schema_version": SCHEMA_VERSION,
            "op": "infer",
            "request_id": f"isaac-b1-{scene}-{binding['frame_sequence']}",
            "worker_epoch": f"isaac-b1-{binding['simulation_epoch']}",
            "frame": {
                "source": "isaac-sim-6.0.1", "stream": "perception_validation",
                "epoch": binding["simulation_epoch"], "sequence": binding["frame_sequence"],
                "capture_time": binding["simulation_time"], "receive_time": binding["simulation_time"],
                "clock_domain": "ros_sim_time", "frame_id": manifest.cameras[0]["frame_id"],
                "width": manifest.cameras[0]["resolution"][0], "height": manifest.cameras[0]["resolution"][1],
                "encoding": "rgb8", "rgb_uri": source.resolve().as_uri(), "rgb_sha256": sha256(source),
            },
        }
        command = [
            str(args.gpu_python), str(ROOT / "tools/vision_worker_entry.py"),
            "--upstream-root", str(vision_root), "--proposal-json", str(proposals),
            "--output-root", str(worker_root), "--input-root", str(capture_root),
            "--input-root", str(vision_root), "--sam-model", str(model_manifest["sam"]["snapshot_path"]),
            "--sam-model-id", str(model_manifest["sam"]["repository"]),
            "--sam-revision", str(model_manifest["sam"]["revision"]),
            "--moge-model", str(model_manifest["moge"]["model_path"]),
            "--moge-model-id", str(model_manifest["moge"]["repository"]),
            "--moge-revision", str(model_manifest["moge"]["revision"]),
            "--stage-timeout", str(args.worker_timeout),
        ]
        started = perf_counter()
        completed = subprocess.run(
            command, input=json.dumps(request) + "\n", text=True, capture_output=True,
            timeout=args.worker_timeout + 60.0, cwd=ROOT,
        )
        elapsed = perf_counter() - started
        response_path = scene_dir / "mode_b1_worker_response.json"
        response_path.write_text(completed.stdout, encoding="utf-8")
        (scene_dir / "mode_b1_worker_stderr.log").write_text(completed.stderr, encoding="utf-8")
        try:
            response = json.loads(completed.stdout.strip().splitlines()[-1])
        except (IndexError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Mode B1 worker emitted invalid JSON for {scene}") from exc
        if completed.returncode or response.get("status") != "COMPLETE":
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
        truth = ground_truth_observation(manifest, annotations["objects"])
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
        )
        write_evaluation(scene_dir / "mode_b1_evaluation.json", report)
        (scene_dir / "mode_b1_perception_observation.json").write_text(dumps(prediction), encoding="utf-8")

        image = cv2.imread(str(source))
        for item in prediction.cargo:
            x1, y1, x2, y2 = (int(round(value)) for value in item.bbox_xyxy)
            cv2.rectangle(image, (x1, y1), (x2, y2), (80, 255, 80), 2)
        cv2.rectangle(image, (0, 0), (image.shape[1], 52), (15, 15, 15), -1)
        cv2.putText(image, f"MODE B1 PRED | {scene} | n={len(prediction.cargo)}", (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (80, 255, 80), 2, cv2.LINE_AA)
        cv2.putText(image, "oracle proposals; SAM+MoGe estimate; RAW_IMAGE_AUTOMATIC=false", (10, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 255, 80), 1, cv2.LINE_AA)
        prediction_overlay = scene_dir / "mode_b1_prediction_overlay.png"
        cv2.imwrite(str(prediction_overlay), image)
        gt = cv2.imread(str(scene_dir / "gt_overlay.png"))
        comparison = np.hstack((gt, image))
        cv2.imwrite(str(scene_dir / "mode_b1_gt_prediction_comparison.png"), comparison)
        video_frames.append(comparison)
        results.append({
            "scene": scene, "status": "PASS", "elapsed_seconds": elapsed,
            "proposal_count": len(json.loads(proposals.read_text(encoding="utf-8"))["instances"]),
            "prediction_count": len(prediction.cargo), "processed_object_count": report["processed_object_count"],
            "mean_bbox_iou": report["mean_bbox_iou"], "mean_mask_iou": report["mean_mask_iou"],
            "depth": depth_report, "evaluation_fingerprint": report["evaluation_fingerprint"],
            "worker_response_sha256": sha256(response_path),
        })

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
        "schema_version": "isaac_perception_mode_b1_summary_v1", "status": "PASS",
        "mode": "ISAAC_SENSOR_WITH_ORACLE_PROPOSALS", "raw_image_automatic": False,
        "vision_commit": "1d208f2ed380a207e6e46b4a62d2ac640edfe477",
        "scene_count": len(results), "scenes": results,
        "video": str(video_path), "video_sha256": sha256(video_path),
        "claim_boundary": "synthetic rendered-image results do not establish real-camera accuracy",
    }
    write_json(capture_root / "mode_b1_summary.json", summary)
    domain_path = capture_root / "domain_summary.json"
    domain = json.loads(domain_path.read_text(encoding="utf-8"))
    domain["mode_b1"] = "PASS"
    domain["mode_b1_summary"] = str((capture_root / "mode_b1_summary.json").resolve())
    write_json(domain_path, domain)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
