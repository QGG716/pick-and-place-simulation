"""One-request, fail-closed adapter for the pinned upstream GPU pipeline.

Stdout contains exactly one JSON response. Stage diagnostics go to files and
stderr so they cannot corrupt the protocol.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from time import monotonic, perf_counter, time
from urllib.parse import unquote, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "unloading_contracts" / "src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import ImageMapping, ObservationStatus, ResourceReference, SCHEMA_VERSION, SensorFrame, dumps
from unloading_perception.backends import CargoJsonReplayBackend, UPSTREAM_COMMIT


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inside(root: Path, value: Path, *, must_exist: bool = True) -> Path:
    root, value = root.resolve(), value.resolve()
    if value != root and root not in value.parents:
        raise ValueError(f"controlled path escapes {root}")
    if must_exist and not value.is_file():
        raise FileNotFoundError(value)
    return value


def emit(request_id: str, worker_epoch: str, input_hash: str | None, status: str, **extra) -> int:
    print(json.dumps({"schema_version": SCHEMA_VERSION, "request_id": request_id, "worker_epoch": worker_epoch, "input_sha256": input_hash, "status": status, **extra}, sort_keys=True))
    return 0 if status == "COMPLETE" else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream-root", type=Path, required=True)
    parser.add_argument("--proposal-json", type=Path, required=True)
    parser.add_argument("--person-masks", type=Path)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, action="append", required=True)
    parser.add_argument("--sam-model", default="facebook/sam-vit-base")
    parser.add_argument("--sam-model-id", default="facebook/sam-vit-base")
    parser.add_argument("--sam-revision", default="70c1a07f894ebb5b307fd9eaaee97b9dfc16068f")
    parser.add_argument("--moge-model", default="Ruicheng/moge-2-vits-normal")
    parser.add_argument("--moge-model-id", default="Ruicheng/moge-2-vits-normal")
    parser.add_argument("--moge-revision", default="26b477f41595707c5db6770294c0d1721e8ed4ed")
    parser.add_argument("--num-tokens", type=int, default=1800)
    parser.add_argument("--stage-timeout", type=float, default=900.0)
    parser.add_argument("--allow-model-download", action="store_true")
    args = parser.parse_args()

    request_id, worker_epoch, input_hash = "unknown", "unknown", None
    try:
        request = json.loads(sys.stdin.readline())
        request_id = str(request["request_id"])
        worker_epoch = str(request["worker_epoch"])
        if request.get("schema_version") != SCHEMA_VERSION or request.get("op") != "infer":
            raise ValueError("unsupported request schema/op")
        frame_data = request["frame"]
        input_hash = str(frame_data["rgb_sha256"])
        parsed = urlparse(str(frame_data["rgb_uri"]))
        if parsed.scheme != "file" or parsed.netloc not in ("", "localhost"):
            raise ValueError("worker accepts local file URIs only")
        source = Path(unquote(parsed.path)).resolve()
        if not any(source == root.resolve() or root.resolve() in source.parents for root in args.input_root):
            raise ValueError("input escapes configured worker roots")
        if not source.is_file() or sha256(source) != input_hash:
            raise ValueError("input file missing or SHA-256 mismatch")
        upstream = args.upstream_root.resolve()
        actual = subprocess.run(["git", "rev-parse", "HEAD"], cwd=upstream, text=True, capture_output=True, timeout=5, check=True).stdout.strip()
        if actual != UPSTREAM_COMMIT:
            return emit(request_id, worker_epoch, input_hash, "FAILED", error_code="UPSTREAM_SHA_MISMATCH", error_message=f"expected {UPSTREAM_COMMIT}, got {actual}")
        if not args.allow_model_download:
            if not Path(args.sam_model).is_dir() or not Path(args.moge_model).is_file():
                return emit(
                    request_id, worker_epoch, input_hash, "FAILED",
                    error_code="MODEL_MISSING",
                    error_message="offline worker requires a local SAM snapshot directory and MoGe model.pt",
                )
        proposal = inside(upstream, args.proposal_json)
        person_masks = None if args.person_masks is None else inside(upstream, args.person_masks)
        output_root = args.output_root.resolve()
        output_root.mkdir(parents=True, exist_ok=True)
        safe_id = "".join(character for character in request_id if character.isalnum() or character in "-_")
        run_dir = inside(output_root, output_root / f"{frame_data['epoch']}-{int(frame_data['sequence']):06d}-{safe_id}", must_exist=False)
        run_dir.mkdir(parents=False, exist_ok=False)

        import torch
        if not torch.cuda.is_available():
            return emit(request_id, worker_epoch, input_hash, "FAILED", error_code="GPU_UNAVAILABLE", error_message="torch.cuda.is_available() is false")
        torch.cuda.reset_peak_memory_stats()
        probe_start = perf_counter()
        probe = torch.arange(1024, device="cuda", dtype=torch.float32).square().sum()
        torch.cuda.synchronize()
        cuda_probe_seconds = perf_counter() - probe_start
        if float(probe.cpu()) <= 0.0:
            raise RuntimeError("CUDA probe returned an invalid result")

        timings = {}
        logs = {}
        gpu_samples = []
        env = os.environ.copy()
        if not args.allow_model_download:
            env["HF_HUB_OFFLINE"] = "1"
        python = sys.executable

        def sample_gpu(name: str, pid: int) -> None:
            try:
                sample = subprocess.run(
                    [
                        "nvidia-smi", "--query-compute-apps=pid,used_gpu_memory",
                        "--format=csv,noheader,nounits",
                    ],
                    text=True, capture_output=True, timeout=2.0, check=True,
                )
                for line in sample.stdout.splitlines():
                    fields = [item.strip() for item in line.split(",")]
                    if len(fields) == 2 and int(fields[0]) == pid:
                        gpu_samples.append({
                            "stage": name,
                            "elapsed_seconds": perf_counter() - probe_start,
                            "pid": pid,
                            "used_gpu_memory_mib": int(fields[1]),
                        })
            except (FileNotFoundError, ValueError, subprocess.SubprocessError):
                return

        def stage(name: str, command: list[str]) -> None:
            started = perf_counter()
            process = subprocess.Popen(
                command, cwd=upstream, env=env, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                start_new_session=os.name == "posix",
            )
            deadline = monotonic() + args.stage_timeout
            while True:
                remaining = deadline - monotonic()
                if remaining <= 0.0:
                    if os.name == "posix":
                        os.killpg(process.pid, signal.SIGKILL)
                    else:
                        process.kill()
                    process.communicate()
                    raise subprocess.TimeoutExpired(command, args.stage_timeout)
                try:
                    stdout, stderr = process.communicate(timeout=min(0.25, remaining))
                    break
                except subprocess.TimeoutExpired:
                    sample_gpu(name, process.pid)
            timings[name] = perf_counter() - started
            log_path = run_dir / f"{name}.log"
            log_path.write_text(stdout + "\n--- stderr ---\n" + stderr, encoding="utf-8")
            stage_samples = [item["used_gpu_memory_mib"] for item in gpu_samples if item["stage"] == name]
            logs[name] = {
                "path": str(log_path), "sha256": sha256(log_path),
                "returncode": process.returncode,
                "peak_process_gpu_memory_mib_observed": max(stage_samples, default=None),
                "gpu_memory_samples": len(stage_samples),
            }
            if process.returncode:
                tail = (stderr or stdout)[-2000:]
                code = "WORKER_OOM" if "out of memory" in tail.lower() else "STAGE_FAILED"
                raise RuntimeError(f"{code}:{name}:{tail}")

        sam_image = run_dir / "sam_crossvalidated.jpg"
        masks = run_dir / "cargo_masks.npz"
        instances = run_dir / "cargo_instances.json"
        sam_command = [python, "pipeline/segmentation/sam_from_generated_boxes.py", str(source), str(proposal), "--output", str(sam_image), "--masks-output", str(masks), "--json-output", str(instances), "--sam-model", args.sam_model, "--device", "cuda"]
        if not args.allow_model_download:
            sam_command.append("--local-files-only")
        if person_masks is not None:
            sam_command.extend(["--person-masks", str(person_masks)])
        stage("sam", sam_command)

        faces_image = run_dir / "box_geometry_2d.jpg"
        faces_json = run_dir / "box_geometry_2d.json"
        faces_dir = run_dir / "face_crops"
        stage("geometry_2d", [python, "pipeline/geometry/recover_box_faces.py", str(source), str(masks), "--axis-mode", "none", "--refinement-fallback", "minrect", "--max-face-area-ratio", "0.15", "--min-mask-area", "50", "--min-iou", "0.45", "--min-line-support", "0.15", "--output", str(faces_image), "--json-output", str(faces_json), "--faces-dir", str(faces_dir)])

        pointmap = run_dir / "moge2_pointmap.npz"
        stage("moge", [python, "pipeline/geometry/infer_moge_pointmap.py", str(source), "--model", args.moge_model, "--device", "cuda", "--num-tokens", str(args.num_tokens), "--output", str(pointmap)])

        cuboids_json = run_dir / "box_cuboids_instance_aware.json"
        cuboids_image = run_dir / "box_cuboids_instance_aware.jpg"
        stage("geometry_3d", [python, "pipeline/geometry/recover_box_cuboids_3d.py", str(source), str(masks), str(pointmap), "--faces-json", str(faces_json), "--relative-threshold", "0.003", "--seed", "17", "--json-output", str(cuboids_json), "--output", str(cuboids_image)])

        final_json = run_dir / "final_instance_aware.json"
        final_image = run_dir / "final_instance_aware.jpg"
        stage("assembly", [python, "pipeline/results/assemble_cargo_results.py", str(source), str(instances), str(masks), str(faces_json), "--cuboids-json", str(cuboids_json), "--output", str(final_image), "--json-output", str(final_json)])

        torch.cuda.synchronize()
        finished = time()
        frame = SensorFrame(
            frame_data["source"], frame_data["stream"], frame_data["epoch"], int(frame_data["sequence"]),
            float(frame_data["capture_time"]), max(float(frame_data["receive_time"]), finished),
            frame_data["clock_domain"], frame_data["frame_id"], int(frame_data["width"]),
            int(frame_data["height"]), frame_data["encoding"], ResourceReference(source.as_uri(), input_hash),
            image_mapping=ImageMapping(int(frame_data["width"]), int(frame_data["height"])),
        )
        observation = CargoJsonReplayBackend(UPSTREAM_COMMIT).read(final_json, frame=frame)
        sam_weight = next(
            (path for name in ("model.safetensors", "pytorch_model.bin") if (path := Path(args.sam_model) / name).is_file()),
            None,
        )
        config = {
            "mode": "REAL_IMAGE_WITH_FROZEN_PROPOSALS",
            "sam_model_id": args.sam_model_id,
            "sam_revision": args.sam_revision,
            "sam_weight_sha256": None if sam_weight is None else sha256(sam_weight),
            "moge_model_id": args.moge_model_id,
            "moge_revision": args.moge_revision,
            "moge_weight_sha256": sha256(Path(args.moge_model)) if Path(args.moge_model).is_file() else None,
            "num_tokens": args.num_tokens,
            "proposal_sha256": sha256(proposal),
            "person_masks_sha256": None if person_masks is None else sha256(person_masks),
        }
        artifacts = {path.name: {"path": str(path), "sha256": sha256(path)} for path in (sam_image, masks, instances, faces_json, pointmap, cuboids_json, final_json, final_image)}
        observation = replace(
            observation, observation_id=f"gpu-{request_id}", processed_time=max(observation.capture_time, finished),
            provider="cargo-real-image-gpu-worker",
            model_identity=f"{args.sam_model_id}@{args.sam_revision}+{args.moge_model_id}@{args.moge_revision}",
            config_identity=hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
            status=ObservationStatus.PARTIAL,
            coverage={**dict(observation.coverage), "mode": config["mode"], "run_id": run_dir.name, "input_sha256": input_hash, "artifacts": artifacts, "stage_timings_seconds": timings, "absence_means_free_space": False},
        )
        metrics = {
            "run_id": run_dir.name, "upstream_commit": actual, "input_sha256": input_hash,
            "config": config, "timings_seconds": {"cuda_probe": cuda_probe_seconds, **timings},
            "torch_version": torch.__version__, "torch_cuda": torch.version.cuda,
            "gpu_name": torch.cuda.get_device_name(0),
            "adapter_peak_allocated_bytes": torch.cuda.max_memory_allocated(),
            "adapter_peak_reserved_bytes": torch.cuda.max_memory_reserved(),
            "peak_process_gpu_memory_mib_observed": max(
                (item["used_gpu_memory_mib"] for item in gpu_samples), default=None,
            ),
            "gpu_memory_samples": gpu_samples,
            "artifacts": artifacts, "logs": logs,
        }
        metrics_path = run_dir / "metrics.json"
        metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
        (run_dir / ".complete").write_text(sha256(final_json), encoding="ascii")
        return emit(request_id, worker_epoch, input_hash, "COMPLETE", observation=dumps(observation), metrics_reference={"path": str(metrics_path), "sha256": sha256(metrics_path)}, output_reference={"path": str(final_json), "sha256": sha256(final_json)}, stage_timings_seconds=timings)
    except subprocess.TimeoutExpired as exc:
        return emit(request_id, worker_epoch, input_hash, "FAILED", error_code="WORKER_TIMEOUT", error_message=f"stage timed out after {exc.timeout}s")
    except Exception as exc:
        message = str(exc)
        code = message.split(":", 1)[0] if message.startswith(("WORKER_OOM:", "STAGE_FAILED:")) else "WORKER_ERROR"
        print(f"vision worker failed: {message}", file=sys.stderr)
        return emit(request_id, worker_epoch, input_hash, "FAILED", error_code=code, error_message=message[-2000:])


if __name__ == "__main__":
    raise SystemExit(main())
